#!/usr/bin/env python3
"""관제 웹 재생용 기록기 — rosbag 의 agent_status / fleet_pose / arm/status 를 받아 robot_agent 가 하던
DB 기록을 같은 Recorder 함수, 같은 순서로 다시 한다 (scripts/replay_dashboard.sh 가 띄운다).

    python3 scripts/replay_recorder.py robot1 robot2 robot3
    python3 scripts/replay_recorder.py --bag-start <bag 폴더>   # 첫 agent_status 의 UTC 시각, bag 시작 뒤 초

robot_agent.run_cycle 의 기록 순서 ([] 는 단계 알림 전에 하는 작업 기록):
    PICKING -> [트레이+작업 생성, IN_TRANSIT] DELIVERING -> PLACE_DOCKING -> [ARRIVED] PLACING
    -> [COMPLETED|FAILED, UNLOADED, 작업 끝] RETURNING -> PICK_DOCKING -> IDLE
    실패: [FAILED] ERROR [작업 끝].  같은 단계에서 detail 만 바뀌면 resume / reroute 이벤트.
위치와 heartbeat 는 에이전트처럼 1 s 마다. 종료(SIGINT/SIGTERM)하면 에이전트처럼 열린 작업을 취소한다.
"""
import json
import math
import re
import sys
import time


def bag_start(uri):
    import datetime
    import rosbag2_py

    info = rosbag2_py.Info().read_metadata(uri, "mcap")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    topics = [t.name for t in reader.get_all_topics_and_types() if t.name.endswith("/agent_status")]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    first = reader.read_next()[2] / 1e9
    utc = datetime.datetime.fromtimestamp(first, datetime.timezone.utc).isoformat()
    print(utc, f"{first - info.starting_time.nanoseconds / 1e9:.1f}")


def main(names):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    from hospital_system.records import Recorder, trays_from_load
    from hospital_system.stations import ANALYSIS, COLLECTION

    class Robot:
        def __init__(self, name):
            self.name, self.rec, self.last, self.arm, self.pose, self.seen = name, None, None, None, None, 0.0

    class ReplayRecorder(Node):
        def __init__(self):
            super().__init__("replay_recorder")
            self.robots = {n: Robot(n) for n in names}
            for r in self.robots.values():
                self.create_subscription(String, f"/{r.name}/agent_status", lambda m, r=r: self.on_status(r, m), 10)
                self.create_subscription(String, f"/{r.name}/arm/status", lambda m, r=r: self.on_arm(r, m), 10)
                self.create_subscription(PoseStamped, f"/{r.name}/fleet_pose", lambda m, r=r: self.on_pose(r, m), 10)
            self.create_timer(1.0, self.tick)

        def on_arm(self, r, msg):
            try:
                r.arm = json.loads(msg.data)
            except ValueError:
                pass

        def on_pose(self, r, msg):
            q = msg.pose.orientation
            r.pose = (msg.pose.position.x, msg.pose.position.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            r.seen = time.monotonic()

        def tick(self):
            for r in self.robots.values():
                if r.rec and time.monotonic() - r.seen < 2.0:
                    r.rec.heartbeat()
                    if r.pose:
                        r.rec.pose(*r.pose)

        def on_status(self, r, msg):
            s = json.loads(msg.data)
            r.seen = time.monotonic()
            key = (s["stage"], s["detail"])
            if r.rec is None:
                # 에이전트 시작 = Recorder 생성 (robot_info is_active). 첫 메시지의 IDLE 은 set_stage 가 아니다
                r.rec, r.last = Recorder(r.name, "nova_carter", self.get_logger()), key
                return
            if key == r.last:
                return
            (stage, detail), prev = key, r.last[0]
            r.last = key
            rec = r.rec
            if stage == "DELIVERING" and prev != "DELIVERING":
                cargo = s.get("cargo") or {}
                rec.create_task(trays_from_load(cargo.get("loaded", []), cargo.get("urgency", [])),
                                COLLECTION, ANALYSIS, "GENERAL")
                rec.task("IN_TRANSIT", departed_at=True)
            elif stage == "PLACING" and prev != "PLACING":
                rec.task("ARRIVED", arrived_at=True)
            elif stage == "RETURNING" and prev == "PLACING":
                arm = r.arm or {}
                rec.task("COMPLETED" if arm.get("result") == "done" else "FAILED")
                rec.event("UNLOADED", unloaded=arm.get("unloaded"), warnings=arm.get("warnings", []))
                rec.finish_task()
            elif stage == "ERROR":
                rec.task("FAILED")
            rec.stage(stage, detail)
            if stage == "ERROR":
                rec.finish_task()
            m = re.match(r"(\S+) -> (\S+): resume (\d+) after", detail)
            if m:
                rec.event("MISSION_RESUME", attempt=int(m[3]), origin=m[1], destination=m[2])
            m = re.search(r": rerouted to (\S+) v(\d+)$", detail)
            if m:
                rec.event("REROUTED", track=m[1], version=int(m[2]))

        def close(self):
            for r in self.robots.values():
                if r.rec:
                    r.rec.cancel_task(f"robot_agent stopped at {r.last[0]}")
                    r.rec.close()

    rclpy.init()
    node = ReplayRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if sys.argv[1:2] == ["--bag-start"]:
        bag_start(sys.argv[2])
    else:
        main(sys.argv[1:] or ["robot1", "robot2", "robot3"])
