"""CPU-only geometry/policy tests; no claim about Isaac/MPPI runtime behavior."""
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from nav_to_goal.hospital_avoidance import (
    LaneFrame, MovingBody, SafetySettings, body_gap, choose_candidate,
    command_clearance, forward_path_valid, limited_command, offset_candidates,
    path_clearance,
    tail_clear_for_rejoin,
)
from nav_to_goal.hospital_costmap import Grid
from nav_to_goal.obstacle_tracking import Tracker


def test_rear_corner_not_center_circle():
    assert body_gap((0, 0, 0), -1.30, .50, .1) < 0
    assert math.hypot(-1.30, .50) > 1.3
    assert body_gap((0, 0, 0), 2, 0, .4) == pytest.approx(1.12)


def test_rotated_rectangle_is_frame_invariant():
    gap = body_gap((0, 0, 0), -1.4, .6, .4)
    assert body_gap((5, 6, math.pi/2), 4.4, 4.6, .4) == pytest.approx(gap)


def test_rotation_sweeps_rear_toward_person():
    tracks = [MovingBody(1, -.9, -1.6, 0, 0, .35)]
    assert command_clearance((0, 0, 0), (0, 0), (0, .9), tracks) < .4
    assert command_clearance((0, 0, 0), (0, 0), (0, 0), tracks) > .4


def test_same_place_different_time_is_not_same_collision():
    crossing = MovingBody(1, 1.8, -1.5, 0, 1, .4)
    passed = MovingBody(1, 1.8, 1.5, 0, 1, .4)
    assert command_clearance((0, 0, 0), (.6, 0), (.6, 0), [crossing]) < .4
    assert command_clearance((0, 0, 0), (.6, 0), (.6, 0), [passed]) > .4


def test_stop_does_not_instantly_remove_momentum():
    settings = replace(SafetySettings(), horizon=1.5)
    obstacle = [MovingBody(1, 1.3, 0, 0, 0, .4)]
    moving = command_clearance((0, 0, 0), (.6, 0), (0, 0), obstacle, settings)
    stopped = command_clearance((0, 0, 0), (0, 0), (0, 0), obstacle, settings)
    assert moving < stopped-.25


def test_guard_preserves_clear_command_and_reports_unavoidable_case():
    assert limited_command((0, 0, 0), (.6, 0), (.6, .2), [])[0] == (.6, .2)
    cmd, state, gap = limited_command((0, 0, 0), (0, 0), (.6, 0),
                                    [MovingBody(1, 0, 0, 0, 0, .4)])
    assert cmd == (0, 0) and state == 'NO_SAFE_COMMAND' and gap < 0


@pytest.mark.parametrize('yaw', [0, math.pi])
def test_forward_candidates_for_both_lanes(yaw):
    lane = LaneFrame(10, 2, yaw, 40)
    pose = lane.world(3, .2, .1)
    candidates = offset_candidates(lane, pose)
    assert candidates
    for side, path in candidates:
        assert forward_path_valid(path, lane, 3)
        assert math.dist(path[0][:2], pose[:2]) < 1e-8
        assert path[0][2] == pytest.approx(pose[2])
        assert lane.local(path[-1])[0] > 3
        assert abs(lane.local(path[-1])[1]) < 1e-8
        assert abs(lane.local(path[-1])[2]) < 1e-8


def test_no_u_turn_reverse_or_last_second_rejoin():
    lane = LaneFrame(0, 0, 0, 40)
    path = offset_candidates(lane, (0, 0, 0))[0][1]
    assert not forward_path_valid(list(reversed(path)), lane, 0)
    assert not forward_path_valid([(0, 0, 0), (.05, 0, math.pi)], lane, 0)
    assert offset_candidates(lane, (38, 0, 0)) == []


def test_early_head_on_can_select_visible_offset_but_late_is_not_forced():
    pose = (0, 0, 0)
    lane = LaneFrame(0, 0, 0, 40)
    candidates = offset_candidates(lane, pose)
    reference = [(i*.05, 0, 0) for i in range(801)]
    early = [MovingBody(1, 6.8, 0, -1.1, 0, .4)]
    assert path_clearance(reference, pose, (.6, 0), early) < .6
    selected = choose_candidate(candidates, pose, (.6, 0), early, lambda _: True)
    assert selected is not None and selected[2] >= .6
    assert max(abs(p[1]) for p in selected[1]) >= 1.5
    late = [MovingBody(1, 5, 0, -1.1, 0, .4)]
    assert choose_candidate(candidates, pose, (.6, 0), late, lambda _: True) is None


def test_rejected_static_paths_cannot_be_selected():
    candidates = offset_candidates(LaneFrame(0, 0, 0, 40), (0, 0, 0))
    assert choose_candidate(candidates, (0, 0, 0), (.6, 0), [], lambda _: False) is None


def test_hold_extends_until_static_obstacle_is_behind_tail():
    lane = LaneFrame(0, 0, 0, 40)
    candidates = offset_candidates(lane, (0, 0, 0), clear_after_s=9.)
    assert candidates
    for side, path in candidates:
        # Four metre return must not start before s=9.
        assert lane.local(path[-1])[0]-4 >= 9.-1e-6


def test_rejoin_checks_tail_not_only_front_passing():
    lane = LaneFrame(0, 0, 0, 40)
    person = [MovingBody(1, 5, 0, 0, 0, .4)]
    assert not tail_clear_for_rejoin(lane, (6, 1.5, 0), person)
    assert tail_clear_for_rejoin(lane, (8, 1.5, 0), person)


def test_side_latch_and_alternative_if_side_blocked():
    candidates = offset_candidates(LaneFrame(0, 0, 0, 40), (0, 0, 0), preferred_side=-1)
    choice = choose_candidate(candidates, (0, 0, 0), (.6, 0), [], lambda _: True, preferred_side=-1)
    assert choice[0] == -1
    choice = choose_candidate(candidates, (0, 0, 0), (.6, 0), [],
                              lambda p: max(v[1] for v in p) > .5, preferred_side=-1)
    assert choice[0] == 1


def test_stopped_previously_moving_track_remains_protected():
    tracker = Tracker()
    for i in range(8):
        tracker.update([(i*.125, 0)], i*.125)
    identity = tracker.snapshots(.875)[0][0]
    for i in range(8, 48):
        tracker.update([(.875, 0)], i*.125)
    states = tracker.snapshots(47*.125)
    assert len(states) == 1 and states[0][0] == identity
    assert abs(states[0][3]) < .01
    assert tracker.predictions(47*.125)
    assert tracker.snapshots(7.2) == []  # missing observations still expire (1.2 s)


def test_static_cluster_never_masquerades_as_person():
    tracker = Tracker()
    for i in range(10):
        tracker.update([(2, 2)], i*.125)
    assert tracker.snapshots(1.125) == []


def make_grid(raw=False, yaw=0):
    origin = NS(position=NS(x=-5., y=-5.),
                orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))
    metadata = NS(width=200, height=200, size_x=200, size_y=200,
                  resolution=.05, origin=origin)
    message = NS(metadata=metadata, info=metadata, data=[0]*40000,
                 header=NS(frame_id='map', stamp=NS(sec=1, nanosec=0)))
    return message, Grid(message, raw)


def test_grid_checks_polygon_interior_not_only_center_or_edges():
    message, grid = make_grid()
    assert grid.body_clear((0, 0, 0))
    # Small isolated object inside rear part of rectangle.
    message.data[100*200+80] = 100
    grid = Grid(message)
    assert not grid.body_clear((0, 0, 0))
    assert grid.body_clear((2, 0, 0))


def test_unknown_and_outside_map_not_certified_free():
    message, grid = make_grid()
    message.data[100*200+100] = -1
    assert not Grid(message).body_clear((0, 0, 0))
    assert not grid.body_clear((4.9, 0, 0))
    assert grid.body_clear((20, 0, 0), require_inside=False)


def test_rotated_grid_and_raw_unknown():
    message, _ = make_grid(raw=True)
    message.data[100*200+100] = 255
    assert not Grid(message, raw=True).body_clear((0, 0, 0))
    message, grid = make_grid(yaw=math.pi/2)
    assert grid.body_clear((-10, 0, math.pi/2))


def test_arrival_and_collision_monitor_configuration_contract():
    root = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((root/'carter_navigation/params/hospital_navigation_params.yaml').read_text())
    server = params['controller_server']['ros__parameters']
    assert server['FollowPathDock']['max_vel_x'] == .5
    assert server['FollowPathDock']['max_speed_xy'] == .5
    assert server['general_goal_checker']['plugin'].endswith('StoppedGoalChecker')
    for name in ('FollowPath', 'FollowPathDock'):
        assert 'ObstacleFootprint' in server[name]['critics']
        assert 'BaseObstacle' not in server[name]['critics']
    assert server['FollowPathMPPI']['wz_max'] == params['velocity_smoother']['ros__parameters']['max_velocity'][2]
    # velocity_smoother -> hospital_velocity_guard -> collision_monitor.
    assert params['collision_monitor']['ros__parameters']['cmd_vel_in_topic'] == 'cmd_vel_human_checked'
    launch = (root/'carter_navigation/launch/hospital_navigation.launch.py').read_text()
    assert 'executable="hospital_velocity_guard"' in launch
