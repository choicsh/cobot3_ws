"""로봇 한 대의 작업 순서를 조율한다 — 팔(Isaac, OmniGraph 토픽) + 주행(hospital_mission).

    ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot1 -p run_cycles:=1

한 사이클 (채취실 East 에 도킹한 상태에서 시작한다):
    PICKING     arm/command load     -> arm/status result=done (적재 + 랙 ArUco 긴급도)
    DELIVERING  hospital_mission collection -> analysis (출발, 차선 주행, 책상 옆 도킹)
    PLACING     arm/command unload   -> arm/status result=done
    RETURNING   hospital_mission analysis -> collection
    IDLE

단계 이름은 DB robot_state_history.status 와 같다. 주행 끝의 책상 도킹 동안은 PLACE_DOCKING(분석실) /
PICK_DOCKING(채취실) 이다.

DB (use_db, 기본 켬 — records.py): 단계·위치·heartbeat 는 Redis, 적재 뒤 트레이 + 작업을 PostgreSQL 에 만들고
IN_TRANSIT -> ARRIVED -> COMPLETED(하역이 제자리에 안 되면 FAILED) 로 갱신한다. 이벤트는 Redis 스트림에 올리고
db_worker 가 robot_event_log 로 옮긴다. DB 없이 돌리려면 -p use_db:=false.

주행은 기존 hospital_mission 을 하위 프로세스로 돌린다 — 성공 0 / 실패 1 / 잘못된 route 2 로 끝나게
만들어 둔 것이라 그대로 이어 붙일 수 있고, 검증된 주행 코드를 건드리지 않는다.
토픽은 상대 이름이다: arm/command, arm/status, agent_status (네임스페이스 /robotN).

관제 모드(-p fleet:=true, P6): 사이클을 스스로 시작하지 않고 task 토픽({"seq": n, "cmd": "cycle"})을 기다린다.
주행은 관제 구역 예약을 따른다 — hospital_mission 에 require_zone_hold 와 구간 키(zone_leg)를 넘기고
zone_hold 토픽을 이 네임스페이스의 것으로 잇는다. 위치(TF map->base_link)를 fleet_pose 로 5 Hz 보낸다.
"""
import json
import math
import os
import shlex
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from hospital_system.records import Recorder, trays_from_load
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
        self.declare_parameter("use_db", True)
        self.declare_parameter("robot_name", "")          # 비우면 네임스페이스 (robot1)
        self.declare_parameter("robot_model", "nova_carter")
        self.declare_parameter("test_type", "GENERAL")    # tray.test_type — 시뮬레이션에는 검사 종류가 없다
        self.declare_parameter("fleet", False)            # True: 관제 지시(task)로만 사이클, 구역 예약을 따른다
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")  # Nav2 네임스페이스(tf 접두)는 P7

        self.arm_status = None
        self.stage = "IDLE"
        self.cycle = 0
        self.detail = ""
        self.history = []            # 사이클 결과 요약 (agent_status 로 같이 보낸다)
        self._sent = None
        self._docking = False        # 주행 하위 프로세스가 도킹을 시작했다 (출력 읽는 스레드가 켠다)
        self.fleet = bool(self.get_parameter("fleet").value)
        self.leg = ""                # 지금(또는 마지막) 주행 구간 키 "<cycle>:<route_id>" — 관제 zone_hold 와 맞춘다
        self.cargo = None            # 실은 트레이 {"loaded", "urgency"} — 관제 우선순위
        self.task_seq = 0            # 끝낸 관제 작업 번호
        self._task = None
        self.pose = None             # (x, y, yaw) map 기준

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, "fleet_pose", 10)
        self.create_timer(0.2, self._update_pose)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "task", self._on_task, latched)

        self.rec = None
        if self.get_parameter("use_db").value:
            name = self.get_parameter("robot_name").value or self.get_namespace().strip("/") or "robot1"
            self.rec = Recorder(name, self.get_parameter("robot_model").value, self.get_logger())
            self.create_timer(1.0, self._db_tick)

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

    def _update_pose(self):
        """TF map->base_link -> fleet_pose (5 Hz). /amcl_pose 는 움직일 때만 갱신돼 도킹 뒤 0.5 m 전 값이 남았다(P5)."""
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter("map_frame").value,
                                                 self.get_parameter("base_frame").value, Time())
        except Exception:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        self.pose = (t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        msg = PoseStamped()
        msg.header = tf.header
        msg.header.frame_id = self.get_parameter("map_frame").value
        msg.pose.position.x, msg.pose.position.y = t.x, t.y
        msg.pose.orientation = q
        self.pose_pub.publish(msg)

    def _db_tick(self):
        self.rec.heartbeat()
        if self.pose:
            self.rec.pose(*self.pose)

    def _on_task(self, msg):
        try:
            task = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f"task is not JSON: {msg.data[:80]}")
            return
        if int(task.get("seq", 0)) > self.task_seq:
            self._task = task

    def set_stage(self, stage, detail=""):
        self.stage, self.detail = stage, detail
        self.get_logger().info(f"[{stage}] {detail}")
        self.publish(force=True)
        if self.rec:
            self.rec.stage(stage, detail)

    def event(self, event_type, **payload):
        if self.rec:
            self.rec.event(event_type, **payload)

    def publish(self, force=False):
        status = {"robot": self.get_namespace().strip("/"), "stage": self.stage, "cycle": self.cycle,
                  "detail": self.detail, "leg": self.leg, "cargo": self.cargo, "fleet": self.fleet,
                  "task_seq": self.task_seq, "arm": self.arm_status, "history": self.history}
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
    def start_leg(self, stage, origin, destination):
        """주행 구간 시작 — 구간 키를 먼저 바꾸고(관제가 새 경로로 예약을 시작한다) 단계를 알린다."""
        self.leg = f"{self.cycle}:{mission_route(origin, destination)}"
        self.set_stage(stage, f"{origin} -> {destination}")

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
            self.event("MISSION_RESUME", attempt=attempt, origin=origin, destination=destination)
            self.spin_for(wait)
            code = self._run_mission(origin, destination, resume=True)
        return code

    def _run_mission(self, origin, destination, resume):
        route = mission_route(origin, destination)
        cmd = shlex.split(self.get_parameter("mission_command").value) + [
            "--ros-args", "-p", f"route_id:={route}", "-p", f"resume:={str(resume).lower()}"]
        if self.fleet:
            # 관제 구역 예약: 이 로봇의 zone_hold 를 따르고, 이 구간(leg) 메시지만 쓴다
            ns = self.get_namespace().rstrip("/")
            cmd += ["-p", "require_zone_hold:=true", "-p", f"zone_leg:={self.leg}",
                    "-r", f"zone_hold:={ns}/zone_hold"]
        self.get_logger().info(f"mission: {' '.join(cmd)}")
        # 파이프로 읽으면 하위 파이썬이 출력을 모아 두므로 버퍼를 끈다 (도킹 시작 줄을 바로 봐야 한다)
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                env={**os.environ, "PYTHONUNBUFFERED": "1"})
        self._docking = False

        def forward():
            for line in proc.stdout:
                print(f"[mission] {line.rstrip()}", flush=True)
                if line.startswith("[MISSION] DOCKING"):
                    self._docking = True

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
            if self._docking:
                self._docking = False
                self.set_stage("PLACE_DOCKING" if destination == ANALYSIS else "PICK_DOCKING",
                               f"docking at {destination}")
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
            if self.rec:
                self.rec.task("FAILED")
            self.set_stage("ERROR", why)
            if self.rec:
                self.rec.finish_task()
            return False

        self.set_stage("PICKING", "arm load at collection")
        s = self.arm("load")
        if s is None or s.get("result") != "done":
            return fail(f"load: {s and s.get('result')} {s and s.get('detail')}")
        record.update(loaded=s["loaded"], urgency=s["urgency"], load_warnings=s.get("warnings", []))
        if s.get("warnings"):
            # 트레이가 랙 제자리에 없다 — 테두리에 걸렸거나 그리퍼에 매달린 채일 수 있어 주행하면 안 된다
            return fail(f"load: tray not in place, not driving: {s['warnings']}")
        self.cargo = {"loaded": s["loaded"], "urgency": s["urgency"]}
        if self.rec:
            trays = trays_from_load(s["loaded"], s["urgency"])
            record["task_id"] = self.rec.create_task(trays, COLLECTION, ANALYSIS,
                                                     self.get_parameter("test_type").value)

        if self.rec:
            self.rec.task("IN_TRANSIT", departed_at=True)
        self.start_leg("DELIVERING", COLLECTION, ANALYSIS)
        code = self.drive(COLLECTION, ANALYSIS)
        if code != 0:
            return fail(f"drive {COLLECTION}->{ANALYSIS} exit {code}")
        if self.rec:
            self.rec.task("ARRIVED", arrived_at=True)

        self.set_stage("PLACING", "arm unload at analysis")
        s = self.arm("unload")
        if s is None:
            return fail("unload: no result")
        record.update(unloaded=s["unloaded"], unload_warnings=s.get("warnings", []))
        self.cargo = None
        if s.get("result") != "done":
            # 트레이가 책상 제자리에 안 놓였다 — 관제가 알아야 하지만 로봇은 돌아간다
            self.get_logger().warn(f"unload {s.get('result')}: {s.get('warnings')}")
        if self.rec:
            # 인계완료는 책상 제자리에 다 놓였을 때만 — 아니면 실패로 남겨 관제가 확인하게 한다
            self.rec.task("COMPLETED" if s.get("result") == "done" else "FAILED")
            self.event("UNLOADED", unloaded=s["unloaded"], warnings=s.get("warnings", []))
            self.rec.finish_task()

        self.start_leg("RETURNING", ANALYSIS, COLLECTION)
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
    try:
        node = RobotAgent()
    except RuntimeError as e:   # DB 가 꺼져 있다 — 기록 없이 로봇을 움직이지 않는다
        print(f"[robot_agent] {e}", file=sys.stderr, flush=True)
        rclpy.shutdown()
        sys.exit(1)
    ok = True
    try:
        node.spin_for(2.0)
        for _ in range(int(node.get_parameter("run_cycles").value)):
            if node.fleet:
                # 관제 지시를 기다린다 (그동안 위치·상태는 타이머가 계속 보낸다)
                while rclpy.ok() and node._task is None:
                    node.spin_for(0.5)
                task, node._task = node._task, None
                node.get_logger().info(f"task {task}")
            if not node.run_cycle():
                ok = False
                break
            if node.fleet:
                node.task_seq = int(task.get("seq", 0))
                node.publish(force=True)
        node.get_logger().info(f"history: {json.dumps(node.history, ensure_ascii=False)}")
        node.spin_for(1.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        ok = False
    finally:
        if node.rec:
            node.rec.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
