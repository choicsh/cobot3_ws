"""관제 — 로봇마다 사이클 지시, 차선 구역 예약, 정지점 전달 (docs/SYSTEM_INTEGRATION_PLAN.md §3.2, §3.3).

    ros2 run hospital_system fleet_manager --ros-args -p robots:="['robot1']" -p cycles:=2

로봇마다 (네임스페이스 /robotN, robot_agent -p fleet:=true)
    받음  agent_status  단계, 구간 키(leg), 실은 트레이(cargo), 끝낸 작업 번호(task_seq)
          fleet_pose    map 기준 위치 (5 Hz)
    보냄  task          {"seq": n, "cmd": "cycle"}           — 채취실에서 IDLE 이면 다음 사이클
          zone_hold     {"leg", "stop": [x,y,yaw]|null, "dock", "seq", "zones"}  — hospital_mission 이 따른다
구간 키가 바뀌면(DELIVERING/RETURNING 시작) 그 구간 경로(lane_graph.route)로 예약을 새로 시작한다.
우선순위는 실은 트레이 긴급도(priority.loaded_score), 빈 로봇은 (0,0,0).
Redis(use_db): robot:{id}:route = 지금 구간 경로(1 m 간격), fleet:zones = 구역 -> 로봇 이름.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from hospital_system.lane_graph import LaneGraph, Reservations, RobotPlan, hospital_edges
from hospital_system.priority import loaded_score
from hospital_system.stations import ROUTE_NODES

POSE_TIMEOUT_S = 3.0     # 위치가 이만큼 끊기면 경고 (예약은 풀지 않는다 — 어디 있는지 모른다)


class Robot:
    def __init__(self, name):
        self.name = name
        self.status = {}
        self.pose = None
        self.pose_time = 0.0
        self.leg = None           # 예약을 잡고 있는 구간 키
        self.sent_seq = 0         # 보낸 작업 번호
        self.hold_sent = None
        self.hold_time = 0.0
        self.warned = False


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        self.declare_parameter("robots", ["robot1"])
        self.declare_parameter("cycles", 0)            # 로봇마다 지시할 사이클 수 (0 = 계속)
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("use_db", True)
        self.graph = LaneGraph(hospital_edges())
        self.res = Reservations(self.graph)
        self.robots = {}
        self.pubs = {}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in self.get_parameter("robots").value:
            r = self.robots[name] = Robot(name)
            self.create_subscription(String, f"/{name}/agent_status", lambda m, r=r: self._on_status(r, m), 10)
            self.create_subscription(PoseStamped, f"/{name}/fleet_pose", lambda m, r=r: self._on_pose(r, m), 10)
            self.pubs[name] = (self.create_publisher(String, f"/{name}/task", latched),
                               self.create_publisher(String, f"/{name}/zone_hold", latched))
        self.db = None
        if self.get_parameter("use_db").value:
            from hospital_system import db
            self.db = db
            self._robot_ids = {}
        self.seq = 0
        self.done = False
        self.create_timer(1.0 / float(self.get_parameter("tick_hz").value), self.tick)
        shared = sorted(z for z, c in self.graph.conflicts.items() if c)
        self.get_logger().info(f"[FLEET] {len(self.graph.zones)} zones, conflict zones {shared}, "
                               f"robots {list(self.robots)}")

    # ── 입력 ─────────────────────────────────────────────────────
    def _on_status(self, r, msg):
        try:
            r.status = json.loads(msg.data)
        except ValueError:
            pass

    def _on_pose(self, r, msg):
        q = msg.pose.orientation
        r.pose = (msg.pose.position.x, msg.pose.position.y,
                  math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        r.pose_time = time.monotonic()

    # ── 계획 ─────────────────────────────────────────────────────
    def _leg_zones(self, leg):
        route = leg.split(":", 1)[1]
        return self.graph.route_zones(self.graph.route(*ROUTE_NODES[route]))

    def _initial_plan(self, r):
        """처음 본 로봇: 두 구간 경로 중 위치가 올라가는 쪽 (보통 채취실 도킹 = 복귀 구간 끝)."""
        best = None
        for route in ROUTE_NODES:
            zones = self._leg_zones(f"0:{route}")
            s, d = self.graph.project(zones, *r.pose)
            if s is not None and (best is None or d < best[0]):
                best = (d, route, zones, s)
        if best is None or best[0] > 1.5:
            self.get_logger().warn(f"[FLEET] {r.name} at {r.pose} is not on a lane")
            return False
        _, route, zones, s = best
        first = self.graph.zones[zones[0]].edge
        if s < 2.0:
            # 구간 시작(도킹 자세)에 서 있다 — 뒤 차체가 걸친 앞 간선 마지막 구역도 잡아야 한다
            prev = [e for e in self.graph.edges.values() if e.end == self.graph.edges[first].start]
            if prev:
                z = self.graph.edge_zones[prev[0].name][-1]
                zones, s = [z] + zones, s + self.graph.zones[z].length
        r.leg = f"0:{route}"
        self.res.set_plan(RobotPlan(r.name, zones, s))
        self.get_logger().info(f"[FLEET] {r.name} starts on {route} s={s:.1f}/{sum(self.graph.zones[z].length for z in zones):.1f}")
        return True

    def _update_plan(self, r):
        leg = r.status.get("leg") or ""
        if leg and leg != r.leg:
            cargo = r.status.get("cargo") or {}
            score = loaded_score(cargo.get("loaded", []), cargo.get("urgency", [])) if cargo else (0, 0, 0)
            r.leg = leg
            self.res.set_plan(RobotPlan(r.name, self._leg_zones(leg), 0.0, score))
            plan = self.res.robots[r.name]
            self.get_logger().info(f"[FLEET] {r.name} leg {leg} score {score} "
                                   f"({len(plan.zones)} zones, kept {plan.zones[:1]})")
            self._record_route(r, plan)
        plan = self.res.robots[r.name]
        s, d = self.graph.project(plan.zones, *r.pose, near=plan.s)
        if s is None:                          # 추적을 놓쳤다 (재시작 등) — 경로 전체에서 방향이 맞는 점
            s, d = self.graph.project(plan.zones, *r.pose)
            d = d if d is not None and d < 1.0 else None
        if s is not None and d is not None and d < 3.0:   # 사람 피해 옆으로 2.8 m 까지 비켜 간다
            plan.s = max(plan.s - 0.5, s)      # 투영 잡음으로 뒤로 튀지 않게

    # ── 지시 ─────────────────────────────────────────────────────
    def _dispatch(self, r):
        st = r.status
        cycles = int(self.get_parameter("cycles").value)
        if not st.get("fleet") or st.get("stage") != "IDLE" or int(st.get("task_seq", 0)) < r.sent_seq:
            return
        if cycles and r.sent_seq >= cycles:
            return
        self.seq += 1
        r.sent_seq = int(st.get("task_seq", 0)) + 1
        self.pubs[r.name][0].publish(String(data=json.dumps({"seq": r.sent_seq, "cmd": "cycle", "id": self.seq})))
        self.get_logger().info(f"[FLEET] {r.name} task {r.sent_seq}: cycle")

    def tick(self):
        now = time.monotonic()
        for r in self.robots.values():
            if r.pose is None:
                continue
            if now - r.pose_time > POSE_TIMEOUT_S and not r.warned:
                r.warned = True
                self.get_logger().warn(f"[FLEET] {r.name} pose lost — keeping its zones")
            elif now - r.pose_time <= POSE_TIMEOUT_S:
                r.warned = False
            if r.name not in self.res.robots and not self._initial_plan(r):
                continue
            self._update_plan(r)
        result = self.res.tick(now)
        for name, other, zid in [(a, c, z) for a, z, c in self.res.intrusions]:
            self.get_logger().error(f"[FLEET] {name} body in {zid} held by {other}")
        for name, (stop, zones) in result.items():
            r = self.robots[name]
            plan = self.res.robots[name]
            point = None if stop is None else [round(v, 3) for v in self.graph.point_at(plan.zones, max(stop, 0.0))]
            hold = {"leg": r.leg, "stop": point, "dock": stop is None, "zones": zones[-3:]}
            if hold != r.hold_sent or now - r.hold_time > 1.0:
                if hold != r.hold_sent:
                    self.get_logger().info(
                        f"[FLEET] {name} s={plan.s:.1f} grant {zones[-1] if zones else '-'} "
                        f"stop {'none (dock)' if point is None else f'{stop:.1f} {point[:2]}'}")
                r.hold_sent, r.hold_time = hold, now
                self.pubs[name][1].publish(String(data=json.dumps({**hold, "seq": int(now * 1000)})))
            self._dispatch(r)
        self._record_zones()
        cycles = int(self.get_parameter("cycles").value)
        if cycles and not self.done and all(
                int(r.status.get("task_seq", 0)) >= cycles for r in self.robots.values()):
            self.done = True
            self.get_logger().info(f"[FLEET] all robots finished {cycles} cycles")

    # ── Redis ────────────────────────────────────────────────────
    def _robot_id(self, name):
        if name not in self._robot_ids:
            row = self.db.robot_info_get_by_name(name)
            if row is None:
                return None
            self._robot_ids[name] = row["robot_id"]
        return self._robot_ids[name]

    def _record_route(self, r, plan):
        if not self.db:
            return
        try:
            rid = self._robot_id(r.name)
            if rid is not None:
                total = sum(self.graph.zones[z].length for z in plan.zones)
                pts = [self.graph.point_at(plan.zones, s)[:2] for s in [i * 1.0 for i in range(int(total) + 1)]]
                self.db.robot_route(rid, [(round(x, 2), round(y, 2)) for x, y in pts])
        except Exception as e:
            self.get_logger().warn(f"[DB] route failed: {e}")

    def clear_zones(self):
        """관제가 끝나면 예약도 끝이다 — 웹이 남은 예약을 그리지 않게 지운다."""
        if self.db:
            try:
                self.db.get_redis().delete("fleet:zones")
            except Exception as e:
                self.get_logger().warn(f"[DB] zones clear failed: {e}")

    def _record_zones(self):
        if not self.db:
            return
        owners = dict(self.res.owner)
        if owners == getattr(self, "_zones_sent", None):
            return
        try:
            r = self.db.get_redis()
            pipe = r.pipeline(transaction=True)
            pipe.delete("fleet:zones")
            if owners:
                pipe.hset("fleet:zones", mapping=owners)
            pipe.execute()
            self._zones_sent = owners
        except Exception as e:
            self.get_logger().warn(f"[DB] zones failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info("[FLEET] done")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.clear_zones()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
