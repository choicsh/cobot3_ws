"""구역 예약 CPU 시험 — 가상 로봇 3대가 순환 루프를 돈다 (P6 완료 기준: 충돌·교착 0, 우선순위 순서).

지도 차선은 nav_to_goal.hospital_mission 에서 가져온다 (ROS 환경 필요).
"""
import math
import random

import pytest

from hospital_system.lane_graph import FRONT_M, REAR_M, LaneGraph, Reservations, RobotPlan, hospital_edges
from hospital_system.priority import score

DT = 0.5
SPEED = 0.6          # hospital_avoidance max_speed


@pytest.fixture(scope="module")
def graph():
    return LaneGraph(hospital_edges())


def plan_len(g, plan):
    return sum(g.zones[z].length for z in plan.zones)


def body_points(g, plan, step=0.25):
    s, pts = plan.s - REAR_M, []
    while s <= plan.s + FRONT_M:
        pts.append(g.point_at(plan.zones, max(0.0, min(s, plan_len(g, plan))))[:2])
        s += step
    return pts


def min_body_gap(g, a, b):
    """두 차체 중심선 사이 최소 거리 — 차폭 1.0 m 이므로 1.0 보다 작으면 겹친다"""
    return min(math.dist(p, q) for p in body_points(g, a) for q in body_points(g, b))


class Sim:
    """책상에서 머무르고(적재/하역), 사람 때문에 가끔 서고, 정지점 앞에서 서는 로봇들"""

    def __init__(self, g, seed=0):
        self.g, self.rng = g, random.Random(seed)
        self.res = Reservations(g)
        self.legs = {"deliver": g.route("collection_dock", "analysis_dock"),
                     "return": g.route("analysis_dock", "collection_dock")}
        self.state = {}
        self.t = 0.0
        self.min_gap = math.inf
        self.log = []

    def add(self, name, leg, s, docked_before=None):
        zones = self.g.route_zones(self.legs[leg])
        if docked_before:                      # 도킹 자세에서 시작: 뒤 차체가 걸친 도킹 구역을 앞에 붙인다
            zones = [docked_before] + zones
            s += self.g.zones[docked_before].length
        self.res.set_plan(RobotPlan(name, zones, s))
        self.state[name] = {"leg": leg, "dwell": 0.0, "pause": 0.0, "cycles": 0, "moved": 0.0}

    def step(self):
        result = self.res.tick(self.t)
        assert not self.res.intrusions, f"t={self.t} 남의 구역 침범 {self.res.intrusions}"
        for name, plan in self.res.robots.items():
            st = self.state[name]
            stop, _ = result[name]
            assert stop is None or stop >= plan.s - 0.05, f"{name} 정지점이 뒤에 있다 {stop} < {plan.s}"
            end = plan_len(self.g, plan)
            if st["dwell"] > 0:
                st["dwell"] -= DT
                if st["dwell"] <= 0:
                    self._next_leg(name)
                continue
            if st["pause"] > 0:
                st["pause"] -= DT
                continue
            if self.rng.random() < 0.02:           # 사람에게 양보
                st["pause"] = self.rng.uniform(2, 10)
            target = end if stop is None else min(stop, end)
            if plan.s < target:
                plan.s = min(target, plan.s + SPEED * DT)
                st["moved"] = self.t
            elif stop is None and plan.s >= end - 1e-6:
                st["dwell"] = self.rng.uniform(40, 90)   # 적재 ~75 s, 하역 ~50 s
        names = list(self.res.robots)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                gap = min_body_gap(self.g, self.res.robots[a], self.res.robots[b])
                self.min_gap = min(self.min_gap, gap)
                assert gap >= 1.0, f"t={self.t} {a}-{b} 차체 겹침 {gap:.2f} m"
        self.t += DT

    def _next_leg(self, name):
        st = self.state[name]
        st["leg"] = "return" if st["leg"] == "deliver" else "deliver"
        if st["leg"] == "deliver":
            st["cycles"] += 1
            urg = [self.rng.choice([1, 2, 3]) for _ in range(3)]
            sc = score(urg)
        else:
            sc = (0, 0, 0)
        self.res.set_plan(RobotPlan(name, self.g.route_zones(self.legs[st["leg"]]), 0.0, sc))


def test_conflict_zones_are_the_two_doors(graph):
    shared = {z for z, c in graph.conflicts.items() if c}
    xs = sorted({round(graph.zones[z].points[len(graph.zones[z].points) // 2][0]) for z in shared})
    # 서쪽은 입구 한 줄(y=12.5) + 방 안 분기(x=-39.5) + 복도 쪽 갈림(x≈-31.5, 2026-09-26 경로 변경)
    assert shared and all(8 <= x <= 15 or -41 <= x <= -31 for x in xs), xs


def test_three_robots_cycle_without_collision_or_deadlock(graph):
    sim = Sim(graph, seed=1)
    sim.add("robot1", "deliver", 0.0, docked_before="collection_dock_in:0")
    sim.add("robot2", "deliver", 45.0)
    sim.add("robot3", "return", 30.0)
    for _ in range(4000):                      # 2000 s — 로봇마다 3바퀴쯤
        sim.step()
        for name, st in sim.state.items():
            idle = sim.t - st["moved"]
            assert st["dwell"] > 0 or idle < 300, f"{name} {idle:.0f} s 동안 못 움직임 (교착?)"
    assert all(st["cycles"] >= 2 for st in sim.state.values()), sim.state
    assert sim.min_gap >= 1.0


@pytest.mark.parametrize("seed", range(5))
def test_random_seeds(graph, seed):
    sim = Sim(graph, seed=seed + 10)
    sim.add("robot1", "deliver", 0.0, docked_before="collection_dock_in:0")
    sim.add("robot2", "deliver", 20.0 + seed * 7)
    sim.add("robot3", "return", 10.0 + seed * 9)
    for _ in range(3000):
        sim.step()
    assert all(st["cycles"] >= 1 for st in sim.state.values()), sim.state


def test_loaded_robot_takes_the_door_first(graph):
    """동쪽 문에 운송(적재) 로봇과 복귀(빈) 로봇이 같이 오면 실은 로봇이 먼저 지난다."""
    res = Reservations(graph)
    deliver = graph.route_zones(graph.route("collection_dock", "analysis_dock"))
    ret = graph.route_zones(graph.route("analysis_dock", "collection_dock"))
    run_d = deliver.index("deliver:4")
    run_r = ret.index("return:21")
    s_d = sum(graph.zones[z].length for z in deliver[:run_d]) - 3.0
    s_r = sum(graph.zones[z].length for z in ret[:run_r]) - 3.0
    res.set_plan(RobotPlan("empty", ret, s_r, (0, 0, 0)))
    res.set_plan(RobotPlan("loaded", deliver, s_d, score([1, 1, 1])))
    res.tick(1.0)
    assert res.owner.get("deliver:5") == "loaded"
    assert res.owner.get("return:22") is None
    # 빈 로봇은 문 앞에 선다
    stop, _ = res.tick(2.0)["empty"]
    assert stop is not None and stop < s_r + 3.0


def test_higher_urgency_wins_between_loaded_robots(graph):
    res = Reservations(graph)
    deliver = graph.route_zones(graph.route("collection_dock", "analysis_dock"))
    ret = graph.route_zones(graph.route("analysis_dock", "collection_dock"))
    s_d = sum(graph.zones[z].length for z in deliver[:deliver.index("deliver:4")]) - 3.0
    s_r = sum(graph.zones[z].length for z in ret[:ret.index("return:21")]) - 3.0
    res.set_plan(RobotPlan("low", deliver, s_d, score([2, 2, 2])))
    res.set_plan(RobotPlan("high", ret, s_r, score([3, 1, 1])))   # 가정: 실은 채 복귀 차선
    res.tick(1.0)
    assert res.owner.get("return:22") == "high"
    assert res.owner.get("deliver:5") is None


def test_queue_behind_docked_robot_keeps_a_zone_gap(graph):
    res = Reservations(graph)
    ret = graph.route_zones(graph.route("analysis_dock", "collection_dock"))
    end = sum(graph.zones[z].length for z in ret)
    res.set_plan(RobotPlan("docked", ret, end))          # 채취실 책상에 도킹해 있다
    res.set_plan(RobotPlan("next", ret, end - 12.0))
    stop, _ = res.tick(1.0)["next"]
    assert stop is not None
    gap = (end - REAR_M) - (stop + FRONT_M)
    assert gap >= 2.0, gap


def test_score_orders_by_highest_then_count():
    assert score([3, 1, 1]) > score([2, 2, 2])
    assert score([3, 3, 1]) > score([3, 2, 2])
    assert score([]) < score([1])
