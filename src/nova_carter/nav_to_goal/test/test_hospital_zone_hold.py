import math

from nav_to_goal.hospital_zone_hold import HOLD_DISTANCE_M, ZoneHold


def hold(msg, required=False):
    z = ZoneHold.__new__(ZoneHold)      # 구독 없이 판단 로직만
    z.required, z.msg = required, msg
    return z


# 북쪽으로 가는 직선 기준 경로 (0.05 m 간격, 10 m)
POINTS = [(0.0, 0.05 * i, math.pi / 2) for i in range(201)]


def test_no_fleet_never_holds():
    assert hold(None).distance(POINTS, 0) is None
    assert hold(None).dock_allowed()


def test_fleet_required_holds_until_first_message():
    assert hold(None, required=True).held(POINTS, 0)
    assert not hold(None, required=True).dock_allowed()


def test_stop_ahead_distance_along_path():
    z = hold({"stop": [0.0, 5.0, math.pi / 2], "dock": False})
    assert abs(z.distance(POINTS, 20) - 4.0) < 1e-6
    assert not z.held(POINTS, 20)
    assert z.held(POINTS, int((5.0 - HOLD_DISTANCE_M + 0.05) / 0.05))


def test_overshot_stop_is_negative_and_holds():
    z = hold({"stop": [0.0, 5.0, math.pi / 2]})
    assert z.distance(POINTS, 110) < 0
    assert z.held(POINTS, 110)


def test_stop_on_opposite_lane_is_ignored():
    # 동쪽 문처럼 반대 방향 차선이 같은 선일 때: 방향이 반대인 정지점은 이 경로의 것이 아니다
    z = hold({"stop": [0.0, 5.0, -math.pi / 2]})
    assert z.distance(POINTS, 0) is None


def test_null_stop_means_whole_leg_granted():
    z = hold({"stop": None, "dock": True})
    assert not z.held(POINTS, 0) and z.dock_allowed()
