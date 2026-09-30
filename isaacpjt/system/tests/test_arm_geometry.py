"""pick_and_place_detection.check_math() 를 옮긴 것 + 긴급도 변환. 시스템 python3 로 돈다."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm import geometry as g  # noqa: E402

TARGETS = ([-0.15, 0.80, 0.28], [0.80, 0.03, 0.11], [0.10, -0.75, 0.20])


@pytest.mark.parametrize("target", TARGETS)
def test_grasp_tool_z_faces_approach_direction(target):
    target = np.array(target)
    for w in g.GRASP_YAW_WEIGHTS:
        d, q = g.grasp_frame(target, w)
        assert np.dot(g.quat_to_matrix(q)[:, 2][:2], d[:2]) > 0.99


@pytest.mark.parametrize("target", TARGETS)
def test_grasp_frame_weights(target):
    target = np.array(target)
    d1, _ = g.grasp_frame(target, 1.0)
    assert np.allclose(d1, g.approach_direction(target)[0], atol=1e-9)
    d0, q0 = g.grasp_frame(target, 0.0)
    assert np.allclose(d0, [0.0, 1.0, 0.0], atol=1e-9)
    assert np.allclose(q0, g.make_target_quat(*g.POINT2_RPY), atol=1e-9)


@pytest.mark.parametrize("p", TARGETS)
def test_rack_yaw_delta(p):
    p = np.array(p)
    assert abs(g.rack_yaw_delta_deg(p, p)) < 1e-6
    assert abs(abs(g.rack_yaw_delta_deg(p, -p)) - 180.0) < 1e-6


def test_marker_slots_and_votes():
    assert g.assign_slots([(2, 0), (1, 2), (0, 2), (2, 7)]) == [2, -1, 1]
    assert g.merge_urgencies([[2, -1, -1], [0, -1, 1], [2, -1, -1]]) == [2, -1, 1]


def test_aruco_to_urgency():
    assert g.aruco_to_urgency([2, 1, 0]) == [3, 2, 1]
    assert g.aruco_to_urgency([-1, 2, -1]) == [1, 3, 1]


def test_base_world_round_trip():
    robot_pos = np.array([20.27, 13.7, 0.45])
    robot_quat = g.quat_from_axis([0, 0, 1], 90.0)
    world, _ = g.base_to_world(g.POINT4_TCP, g.POINT4_RPY, robot_pos, robot_quat)
    assert np.allclose(g.world_to_base_pos(world, robot_pos, robot_quat), g.POINT4_TCP)


def test_grasp_frame_is_yaw_frame_of_bearing():
    for target in TARGETS:
        target = np.array(target)
        d, q = g.grasp_frame(target, 1.0)
        d2, q2 = g.grasp_frame_yaw(g.approach_direction(target)[1])
        assert np.allclose(d, d2) and np.allclose(q, q2)


@pytest.mark.parametrize("bearing", [-103.1, -101.0, -90.0, -83.3, -70.7, -46.0])
def test_square_yaw_snaps_to_tray_axis(bearing):
    # 책상 트레이는 base 기준 yaw +90 (2026-09-25 실측). 그 변에 수직인 접근 = -90 (base +x)
    yaw = g.square_grasp_yaw(bearing, 90.0)
    assert yaw == pytest.approx(-90.0)
    d, q = g.grasp_frame_yaw(yaw)
    assert np.allclose(d, [1.0, 0.0, 0.0], atol=1e-9)
    assert np.dot(g.quat_to_matrix(q)[:, 2], [1.0, 0.0, 0.0]) > 0.99   # 툴 +Z 가 트레이를 향한다


def test_square_yaw_other_quadrants():
    assert g.square_grasp_yaw(10.0, 90.0) == pytest.approx(0.0)
    assert g.square_grasp_yaw(170.0, 90.0) == pytest.approx(180.0) or g.square_grasp_yaw(170.0, 90.0) == pytest.approx(-180.0)
    assert g.square_grasp_yaw(-50.0, 30.0) == pytest.approx(-60.0)


def test_grasp_point_pushes_along_grasp_yaw():
    base_pos, base_quat = np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    surface = np.array([0.80, 0.25, 0.08])                    # 가로로 흩어진 트레이 앞면
    grasp = g.grasp_point_base(surface, base_pos, base_quat, yaw_deg=-90.0)
    assert np.allclose(grasp[:2], [0.80 + g.TRAY_HALF_DEPTH_M, 0.25])   # 옆으로 안 밀린다
    old = g.grasp_point_base(surface, base_pos, base_quat)             # 원본: 방위각 방향
    assert abs(old[1] - 0.25) > 0.01

