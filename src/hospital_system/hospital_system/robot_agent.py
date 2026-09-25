"""로봇 한 대의 작업 순서를 조율한다 — 팔(Isaac, OmniGraph 토픽) + 주행(hospital_mission).

    ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot1 -p run_cycles:=1

한 사이클 (채취실 East 에 도킹한 상태에서 시작한다):
    PICKING     arm/command load     -> arm/status result=done (적재 + 랙 ArUco 긴급도)
    DELIVERING  hospital_mission collection -> analysis (출발, 차선 주행, 책상 옆 도킹)
    PLACING     arm/command unload   -> arm/status result=done
    RETURNING   hospital_mission analysis -> collection
    IDLE

단계 이름은 DB robot_state_history.status 와 같다 (P5 에서 그대로 기록한다).
주행은 기존 hospital_mission 을 하위 프로세스로 돌린다 — 성공 0 / 실패 1 / 잘못된 route 2 로 끝나게
만들어 둔 것이라 그대로 이어 붙일 수 있고, 검증된 주행 코드를 건드리지 않는다.
토픽은 상대 이름이다: arm/command, arm/status, agent_status (네임스페이스 /robotN).
"""
import json
import shlex
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from hospital_system.stations import ANALYSIS, COLLECTION, mission_route

FINAL_ARM_RESULTS = ("done", "failed", "stopped")


class RobotAgent(Node):
    def __init__(self):
        super().__init__("robot_agent")
        self.declare_parameter("run_cycles", 1)
        self.declare_parameter("arm_timeout_s", 600.0)
        self.declare_parameter("mission_timeout_s", 1500.0)
        self.declare_parameter("mission_command", "ros2 run nav_to_goal hospital_mission")
        # 주행이 도중에 실패하면(대개 사람 앞 양보 예산 초과) 이만큼 쉬고 현재 위치에서 이어 간다
        self.declare_parameter("resume_attempts", 3)
        self.declare_parameter("resume_wait_s", 5.0)

        self.arm_status = None
        self.stage = "IDLE"
        self.cycle = 0
        self.detail = ""
        self.history = []            # 사이클 결과 요약 (agent_status 로 같이 보낸다)
        self._sent = None

        self.arm_pub = self.create_publisher(
            String, "arm/command", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.status_pub = self.create_publisher(String, "agent_status", 10)
        self.create_subscription(String, "arm/status", self._on_arm_status, 10)
        self.create_timer(1.0, lambda: self.publish(force=True))

    # ── 상태 ─────────────────────────────────────────────────────
    def _on_arm_status(self, msg):
        try:
            self.arm_status = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f"arm/status is not JSON: {msg.data[:80]}")

    def set_stage(self, stage, detail=""):
        self.stage, self.detail = stage, detail
        self.get_logger().info(f"[{stage}] {detail}")
        self.publish(force=True)

    def publish(self, force=False):
        status = {"robot": self.get_namespace().strip("/"), "stage": self.stage, "cycle": self.cycle,
                  "detail": self.detail, "arm": self.arm_status, "history": self.history}
        if force or status != self._sent:
            self.status_pub.publish(String(data=json.dumps(status)))
            self._sent = status

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, end - time.monotonic())))

    # ── 팔 ───────────────────────────────────────────────────────
    def arm(self, cmd):
        """arm/command 를 보내고 결과(status dict)를 기다린다. 시간 초과면 None."""
        text = f"{cmd}:{int(time.time() * 1000)}"
        # Isaac 의 OmniGraph 구독이 이 발행자를 찾을 때까지 기다린다. Nav2 가 같이 뜨는 동안은 DDS 발견이
        # 20 s 넘게 걸린 적이 있다 (2026-09-26 p4f: 상태는 받는데 명령이 한 번도 전달되지 않았다)
        end = time.monotonic() + 60.0
        while self.arm_pub.get_subscription_count() == 0 and time.monotonic() < end:
            self.spin_for(0.5)
        started = False
        # 구독이 붙은 직후 보낸 한 번이 사라질 수 있어 받았다는 상태가 보일 때까지 몇 번 보낸다
        for i in range(60):
            self.arm_pub.publish(String(data=text))
            self.spin_for(1.0)
            s = self.arm_status or {}
            if s.get("cmd_text") == text:
                started = True
                break
            if i and i % 10 == 0:
                self.get_logger().warn(f"arm has not taken {text} yet ({i} s, "
                                       f"subscribers {self.arm_pub.get_subscription_count()})")
        if not started:
            self.get_logger().error(f"arm did not accept {text} (last status {self.arm_status})")
            return None
        end = time.monotonic() + self.get_parameter("arm_timeout_s").value
        while rclpy.ok() and time.monotonic() < end:
            self.spin_for(0.5)
            s = self.arm_status or {}
            if s.get("cmd_text") == text and s.get("result") in FINAL_ARM_RESULTS:
                return s
        self.get_logger().error(f"arm {cmd} timed out")
        return None

    # ── 주행 ─────────────────────────────────────────────────────
    def drive(self, origin, destination):
        """hospital_mission 을 돌린다. 도중 실패(종료 1)면 resume 으로 현재 위치에서 이어 간다.

        hospital_mission 은 사람 앞에서 15 s 넘게 양보하면 '차선이 막혔다는 증거는 아님' 으로 실패한다.
        처음부터 다시 할 수는 없으므로(도킹 자세 출발 검사) 같은 경로를 현재 위치부터 이어 간다.
        종료 코드: 0 성공, 1 실패, 2 잘못된 route, -1 시간 초과 (2/-1 은 재시도하지 않는다)."""
        code = self._run_mission(origin, destination, resume=False)
        for attempt in range(1, int(self.get_parameter("resume_attempts").value) + 1):
            if code != 1:
                break
            wait = float(self.get_parameter("resume_wait_s").value)
            self.set_stage(self.stage, f"{origin} -> {destination}: resume {attempt} after {wait:.0f} s")
            self.spin_for(wait)
            code = self._run_mission(origin, destination, resume=True)
        return code

    def _run_mission(self, origin, destination, resume):
        route = mission_route(origin, destination)
        cmd = shlex.split(self.get_parameter("mission_command").value) + [
            "--ros-args", "-p", f"route_id:={route}", "-p", f"resume:={str(resume).lower()}"]
        self.get_logger().info(f"mission: {' '.join(cmd)}")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

        def forward():
            for line in proc.stdout:
                print(f"[mission] {line.rstrip()}", flush=True)

        reader = threading.Thread(target=forward, daemon=True)
        reader.start()
        end = time.monotonic() + self.get_parameter("mission_timeout_s").value
        while proc.poll() is None:
            if not rclpy.ok() or time.monotonic() > end:
                self.get_logger().error(f"mission {route} timed out — stopping it")
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                reader.join(timeout=2)
                return -1
            self.spin_for(0.5)
        reader.join(timeout=2)
        return proc.returncode

    # ── 사이클 ───────────────────────────────────────────────────
    def run_cycle(self):
        self.cycle += 1
        t0 = time.monotonic()
        record = {"cycle": self.cycle}

        def fail(why):
            record.update(result="failed", reason=why, seconds=round(time.monotonic() - t0))
            self.history.append(record)
            self.set_stage("ERROR", why)
            return False

        self.set_stage("PICKING", "arm load at collection")
        s = self.arm("load")
        if s is None or s.get("result") != "done":
            return fail(f"load: {s and s.get('result')} {s and s.get('detail')}")
        record.update(loaded=s["loaded"], urgency=s["urgency"], load_warnings=s.get("warnings", []))
        if s.get("warnings"):
            # 트레이가 랙 제자리에 없다 — 테두리에 걸렸거나 그리퍼에 매달린 채일 수 있어 주행하면 안 된다
            return fail(f"load: tray not in place, not driving: {s['warnings']}")

        self.set_stage("DELIVERING", f"{COLLECTION} -> {ANALYSIS}")
        code = self.drive(COLLECTION, ANALYSIS)
        if code != 0:
            return fail(f"drive {COLLECTION}->{ANALYSIS} exit {code}")

        self.set_stage("PLACING", "arm unload at analysis")
        s = self.arm("unload")
        if s is None:
            return fail("unload: no result")
        record.update(unloaded=s["unloaded"], unload_warnings=s.get("warnings", []))
        if s.get("result") != "done":
            # 트레이가 책상 제자리에 안 놓였다 — 관제가 알아야 하지만 로봇은 돌아간다
            self.get_logger().warn(f"unload {s.get('result')}: {s.get('warnings')}")

        self.set_stage("RETURNING", f"{ANALYSIS} -> {COLLECTION}")
        code = self.drive(ANALYSIS, COLLECTION)
        if code != 0:
            return fail(f"drive {ANALYSIS}->{COLLECTION} exit {code}")

        record.update(result="done" if s.get("result") == "done" else "done_with_warnings",
                      seconds=round(time.monotonic() - t0))
        self.history.append(record)
        self.set_stage("IDLE", f"cycle {self.cycle} {record['result']} in {record['seconds']} s")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RobotAgent()
    ok = True
    try:
        node.spin_for(2.0)
        for _ in range(int(node.get_parameter("run_cycles").value)):
            if not node.run_cycle():
                ok = False
                break
        node.get_logger().info(f"history: {json.dumps(node.history, ensure_ascii=False)}")
        node.spin_for(1.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        ok = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
