import math

from nav_to_goal.hospital_mission import (
    LANES, ROUTES, resume_stages, route_length, sample_route, split_route)


def _stages(route_id):
    lane = ROUTES[route_id]
    departure, transit, arrival = split_route(lane, LANES[lane])
    return [("station_departure", departure, "FollowPath", "t"),
            (lane, transit, "FollowPathMPPI", "t"),
            ("station_arrival", arrival, "FollowPathDock", "g", 1.57)]


def test_resume_on_transit_line_keeps_remaining_route():
    stages = _stages("specimen_to_lab")
    transit = stages[1][1][0]                       # lane_upper 긴 직선 (-31.5, 14.9) -> (7.9, 14.9)
    assert transit[0] == "line"
    pose = (-10.0, 15.2, 0.0)                       # 콘 옆, 차선에서 0.3 m 비켜 있음, 동쪽을 봄
    out = resume_stages(stages, pose)
    assert [s[0] for s in out] == ["lane_upper", "station_arrival"]
    first = out[0][1][0]
    assert first[0] == "line" and abs(first[1][0] - (-10.0)) < 1e-6 and abs(first[1][1] - 14.9) < 1e-6
    assert out[1] == stages[2]                      # 도착 구간은 그대로


def test_resume_mid_arc_cuts_the_arc():
    stages = _stages("specimen_to_lab")
    seg = stages[0][1][1]                           # 출발 구간 두 번째 원소 = 원호
    assert seg[0] == "arc"
    (cx, cy), r, a0, a1 = seg[1], seg[2], seg[3], seg[4]
    mid = math.radians((a0 + a1) / 2)
    yaw = mid + (math.pi / 2 if a1 > a0 else -math.pi / 2)
    out = resume_stages(stages, (cx + r * math.cos(mid), cy + r * math.sin(mid), yaw))
    cut = out[0][1][0]
    assert out[0][0] == "station_departure" and cut[0] == "arc"
    assert abs(cut[3] - (a0 + a1) / 2) < 1e-6 and cut[4] == a1
    total = sum(route_length(s[1]) for s in stages)
    assert sum(route_length(s[1]) for s in out) < total


def test_resume_rejects_off_route_or_reversed():
    stages = _stages("specimen_to_lab")
    assert resume_stages(stages, (-10.0, 17.5, 0.0)) is None          # 2.6 m 밖
    assert resume_stages(stages, (-10.0, 14.9, math.pi)) is None      # 반대 방향


def test_resume_points_follow_route():
    stages = _stages("lab_to_specimen")
    pts = sample_route([seg for s in stages for seg in s[1]])
    p = pts[len(pts) // 2]
    out = resume_stages(stages, p)
    rest = sample_route([seg for s in out for seg in s[1]])
    assert math.dist(rest[0][:2], p[:2]) < 0.05 and math.dist(rest[-1][:2], pts[-1][:2]) < 1e-6
