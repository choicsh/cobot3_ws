"""관제 — 로봇마다 사이클 지시, 차선 구역 예약, 정지점 전달 (docs/SYSTEM_INTEGRATION_PLAN.md §3.2, §3.3).

    ros2 run hospital_system fleet_manager --ros-args -p robots:="['robot1']" -p cycles:=2

로봇마다 (네임스페이스 /robotN, robot_agent -p fleet:=true)
    받음  agent_status  단계, 구간 키(leg), 실은 트레이(cargo), 끝낸 작업 번호(task_seq)
          fleet_pose    map 기준 위치 (5 Hz)
    보냄  task          {"seq": n, "cmd": "cycle"}           — 채취실에서 IDLE 이면 다음 사이클
          route         {"leg", "track", "version"}           — 이 구간에 쓸 가운데 복도 (latched)
          zone_hold     {"leg": "<leg>#<version>", "stop": [x,y,yaw]|null, "dock", "seq", "zones"}
구간 키가 바뀌면(DELIVERING/RETURNING 시작) lane_graph.choose_route 로 복도를 고른다: 짧은 순으로, 같은 복도에
반대 방향 로봇이 없는 것. 반대 방향 로봇이 모두 긴급도가 낮고 아직 복도에 안 들어갔으면 그 복도를 가져가고
그 로봇들에게 다음 경로를 준다(route version 을 올린다 — 에이전트가 주행을 현재 위치에서 새 경로로 다시 시작).
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

from hospital_system.lane_graph import (LEG_NODES, LaneGraph, Reservations, RobotPlan, assign_route,
                                        hospital_edges)
from hospital_system.priority import loaded_score
from hospital_system.stations import ROUTE_LEG

POSE_TIMEOUT_S = 3.0     # 위치가 이만큼 끊기면 경고 (예약은 풀지 않는다 — 어디 있는지 모른다)


class Robot:
    def __init__(self, name):
        self.name = name
        self.status = {}
        self.pose = None
        self.pose_time = 0.0
        self.leg = None           # 예약을 잡고 있는 구간 키 (에이전트의 "<cycle>:<route_id>")
        self.version = 0          # 경로 번호 — 바꿀 때마다 올린다
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
                               self.create_publisher(String, f"/{name}/zone_hold", latched),
                               self.create_publisher(String, f"/{name}/route", latched))
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
    def _publish_route(self, r):
        track, _ = self.graph.track_of(self.res.robots[r.name].edges)
        self.pubs[r.name][2].publish(String(data=json.dumps({"leg": r.leg, "track": track, "version": r.version})))
        r.hold_sent = None                     # 새 구간 키로 zone_hold 를 바로 보낸다

    def _initial_plan(self, r):
        """처음 본 로봇: 가능한 모든 경로 중 위치가 올라가는 것 (보통 채취실 도킹 = 운송 경로 시작)."""
        best = None
        for route_id, leg in ROUTE_LEG.items():
            for _, edges in self.graph.routes(*LEG_NODES[leg]):
                zones = self.graph.route_zones(edges)
                s, d = self.graph.project(zones, *r.pose)
                if s is not None and (best is None or d < best[0]):
                    best = (d, route_id, edges, zones, s)
        if best is None or best[0] > 1.5:
            self.get_logger().warn(f"[FLEET] {r.name} at {r.pose} is not on a lane")
            return False
        _, route_id, edges, zones, s = best
        first = self.graph.zones[zones[0]].edge
        prefix = 0
        if s < 2.0:
            # 구간 시작(도킹 자세)에 서 있다 — 뒤 차체가 걸친 앞 간선 마지막 구역도 잡아야 한다
            prev = [e for e in self.graph.edges.values() if e.end == self.graph.edges[first].start and not e.track]
            if prev:
                z = self.graph.edge_zones[prev[0].name][-1]
                zones, s, prefix = [z] + zones, s + self.graph.zones[z].length, 1
        r.leg, r.version = f"0:{route_id}", 1
        self.res.set_plan(RobotPlan(r.name, zones, s, leg=ROUTE_LEG[route_id], edges=edges, prefix=prefix))
        self._publish_route(r)
        self.get_logger().info(f"[FLEET] {r.name} starts on {route_id} via {self.graph.track_of(edges)[0]} "
                               f"s={s:.1f}/{sum(self.graph.zones[z].length for z in zones):.1f}")
        return True

    def _score(self, r):
        cargo = r.status.get("cargo") or {}
        return loaded_score(cargo.get("loaded", []), cargo.get("urgency", [])) if cargo else (0, 0, 0)

    def _new_leg(self, r, leg):
        score = self._score(r)
        r.leg = leg
        changed = assign_route(self.graph, self.res, r.name, ROUTE_LEG[leg.split(":", 1)[1]], score)
        for name, edges in changed.items():
            other = self.robots[name]
            plan = self.res.robots[name]
            other.version += 1
            track = self.graph.track_of(edges)[0]
            if name == r.name:
                self.get_logger().info(f"[FLEET] {name} leg {leg} score {score} via {track} v{other.version} "
                                       f"({len(plan.zones)} zones, kept {plan.zones[:plan.prefix]})")
            else:
                self.get_logger().info(f"[FLEET] {name} (score {plan.score}) yields its corridor to {r.name} "
                                       f"(score {score}) -> {track} v{other.version}")
            self._publish_route(other)
            self._record_route(other, plan)

    def _update_plan(self, r):
        leg = r.status.get("leg") or ""
        if leg and leg != r.leg:
            self._new_leg(r, leg)
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
            hold = {"leg": f"{r.leg}#{r.version}", "stop": point, "dock": stop is None, "zones": zones[-3:]}
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
