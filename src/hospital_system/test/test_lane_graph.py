"""구역 예약 CPU 시험 — 가상 로봇 3대가 순환 루프를 돈다 (P6 완료 기준: 충돌·교착 0, 우선순위 순서).

지도 차선은 nav_to_goal.hospital_mission 에서 가져온다 (ROS 환경 필요).
"""
import math
import random

import pytest

from hospital_system.lane_graph import (FRONT_M, REAR_M, LaneGraph, Reservations, RobotPlan, assign_route,
                                        choose_route, hospital_edges)
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
        self.state = {}
        self.tracks = []             # (로봇, 구간, 복도) — 배정 기록
        self.reroutes = 0
        self.t = 0.0
        self.min_gap = math.inf
        self.log = []

    def add(self, name, leg, s, docked_before=None):
        edges, _ = choose_route(self.g, self.res, name, leg, (0, 0, 0))
        zones = self.g.route_zones(edges)
        if docked_before:                      # 도킹 자세에서 시작: 뒤 차체가 걸친 도킹 구역을 앞에 붙인다
            zones = [docked_before] + zones
            s += self.g.zones[docked_before].length
        self.res.set_plan(RobotPlan(name, zones, s, leg=leg, edges=edges, prefix=1 if docked_before else 0))
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
        st["leg"] = "to_collection" if st["leg"] == "to_analysis" else "to_analysis"
        if st["leg"] == "to_analysis":
            st["cycles"] += 1
            urg = [self.rng.choice([1, 2, 3]) for _ in range(3)]
            sc = score(urg)
        else:
            sc = (0, 0, 0)
        changed = assign_route(self.g, self.res, name, st["leg"], sc)
        self.reroutes += len(changed) - 1
        for n, edges in changed.items():
            self.tracks.append((n, self.res.robots[n].leg, self.g.track_of(edges)[0]))


def test_conflict_zones_are_the_two_doors(graph):
    shared = {z for z, c in graph.conflicts.items() if c}
    xs = sorted({round(graph.phys_zones[z].points[len(graph.phys_zones[z].points) // 2][0]) for z in shared})
    # 두 문: 입구 한 줄(y=12.5) + 방 안 분기 + 문 바깥 분기점에서 갈라지는 복도 4개의 첫 구역
    assert shared and all(7 <= x <= 15 or -41 <= x <= -31 for x in xs), xs


def test_three_robots_cycle_without_collision_or_deadlock(graph):
    sim = Sim(graph, seed=1)
    sim.add("robot1", "to_analysis", 0.0, docked_before="collection_dock_in:0")
    sim.add("robot2", "to_analysis", 45.0)
    sim.add("robot3", "to_collection", 30.0)
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
    sim.add("robot1", "to_analysis", 0.0, docked_before="collection_dock_in:0")
    sim.add("robot2", "to_analysis", 20.0 + seed * 7)
    sim.add("robot3", "to_collection", 10.0 + seed * 9)
    for _ in range(3000):
        sim.step()
    assert all(st["cycles"] >= 1 for st in sim.state.values()), sim.state


def _plan(graph, name, edges, before, score_=(0, 0, 0), leg=""):
    """edges 경로에서 첫 충돌 구역 before m 앞에 선 로봇"""
    zones = graph.route_zones(edges)
    first = next(i for i, z in enumerate(zones) if graph.zones[z].shared and i > 0)
    s = sum(graph.zones[z].length for z in zones[:first]) - before
    return RobotPlan(name, zones, s, score_, leg=leg, edges=edges), zones[first]


def test_loaded_robot_takes_the_east_junction_first(graph):
    """동쪽 분기점에 운송(적재, 채취실에서 나옴) 로봇과 복귀(빈, 복도에서 옴) 로봇이 같이 오면 실은 로봇이 먼저."""
    res = Reservations(graph)
    empty, e_zone = _plan(graph, "empty", ["analysis_out", "lower>E", "collection_in", "collection_dock_in"], 60.0)
    # 빈 로봇은 아래 복도 끝(동쪽 분기점 앞)까지 와 있다
    empty.s = sum(graph.zones[z].length for z in empty.zones[:empty.zones.index("lower>E:21")]) - 2.0
    loaded, l_zone = _plan(graph, "loaded", ["collection_out", "upper>W", "analysis_in", "analysis_dock_in"], 2.0,
                           score([1, 1, 1]))
    res.set_plan(empty)
    res.set_plan(loaded)
    res.tick(1.0)
    assert res.owner.get(graph.phys(l_zone)) == "loaded"
    assert res.owner.get("collection_in:0") is None and res.owner.get("lower:21") is None
    stop, _ = res.tick(2.0)["empty"]                      # 빈 로봇은 분기점 앞에 선다
    assert stop is not None and stop < empty.s + 3.0


def test_opposite_robot_gets_the_reserve_track_and_same_direction_follows(graph):
    res = Reservations(graph)
    a = assign_route(graph, res, "a", "to_analysis", score([1, 1, 1]))["a"]
    assert graph.track_of(a) == ("upper", "W")
    b = assign_route(graph, res, "b", "to_collection", (0, 0, 0))["b"]
    assert graph.track_of(b) == ("upper_reserve", "E")           # 반대 방향 -> 예비선
    c = assign_route(graph, res, "c", "to_analysis", (0, 0, 0))["c"]
    assert graph.track_of(c) == ("upper", "W")                   # 같은 방향 -> 본선 뒤따르기


def test_urgent_robot_takes_the_short_track_from_a_waiting_low_robot(graph):
    """빈 로봇이 위 본선을 받았지만 아직 복도에 안 들어갔다 -> 실은 로봇이 가져가고 빈 로봇은 예비선."""
    res = Reservations(graph)
    assign_route(graph, res, "empty", "to_collection", (0, 0, 0))
    assert graph.track_of(res.robots["empty"].edges)[0] == "upper"
    res.tick(0.0)
    changed = assign_route(graph, res, "loaded", "to_analysis", score([3, 1, 1]))
    assert graph.track_of(changed["loaded"]) == ("upper", "W")
    assert graph.track_of(changed["empty"]) == ("upper_reserve", "E")
    assert graph.track_of(res.robots["empty"].edges)[0] == "upper_reserve"


def test_robot_already_in_the_corridor_keeps_it(graph):
    res = Reservations(graph)
    assign_route(graph, res, "empty", "to_collection", (0, 0, 0))
    plan = res.robots["empty"]
    k = next(i for i, z in enumerate(plan.zones) if graph.zones[z].track)   # 복도 첫 구역
    plan.s = sum(graph.zones[z].length for z in plan.zones[:k + 3])
    res.tick(0.0)
    assert res.entered("empty", "upper")
    changed = assign_route(graph, res, "loaded", "to_analysis", score([3, 3, 3]))
    assert list(changed) == ["loaded"] and graph.track_of(changed["loaded"])[0] == "upper_reserve"


def test_direction_lock_never_grants_a_track_to_opposite_robots(graph):
    """배정을 거치지 않고 억지로 반대 방향 두 로봇을 같은 복도에 넣어도 둘째는 복도 구역을 못 받는다."""
    res = Reservations(graph)
    w, _ = _plan(graph, "w", ["collection_out", "upper>W", "analysis_in", "analysis_dock_in"], 1.0)
    e, _ = _plan(graph, "e", ["analysis_out", "upper>E", "collection_in", "collection_dock_in"], 1.0)
    res.set_plan(w)
    res.tick(0.0)
    res.set_plan(e)
    for t in range(5):
        res.tick(t)
    held = {r: {z for z, o in res.owner.items() if o == r and z.startswith("upper:")} for r in ("w", "e")}
    assert held["w"] and not held["e"]


def test_queue_behind_docked_robot_keeps_a_zone_gap(graph):
    res = Reservations(graph)
    ret = graph.route_zones(graph.routes("analysis_dock", "collection_dock")[0][1])
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
