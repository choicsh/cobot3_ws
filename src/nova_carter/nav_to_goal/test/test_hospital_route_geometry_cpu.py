"""Source-derived map checks without importing ROS or launching a robot."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
from PIL import Image
import pytest
import yaml

from nav_to_goal.hospital_avoidance import LaneFrame, SafetySettings, offset_candidates
from nav_to_goal.hospital_costmap import Grid


ROOT = Path(__file__).resolve().parents[2]


def source_routes():
    """차선 기하(hospital_lanes, ROS 없음) + 경로 샘플링(hospital_mission)"""
    from nav_to_goal import hospital_lanes as lanes
    from nav_to_goal.hospital_mission import (
        ARRIVAL_YAWS, LANE_LOWER, LANE_UPPER, LANES, ROUTES, sample_route, split_route)
    scope = {name: getattr(lanes, name) for name in dir(lanes) if not name.startswith('_')}
    scope.update(ARRIVAL_YAWS=ARRIVAL_YAWS, LANES=LANES, ROUTES=ROUTES, LANE_UPPER=LANE_UPPER,
                 LANE_LOWER=LANE_LOWER, sample_route=sample_route, split_route=split_route)
    return scope


ALL_ROUTES = [(d, t) for d in ('to_analysis', 'to_collection')
              for t in ('upper', 'upper_reserve', 'lower', 'lower_reserve')]


def map_grid():
    directory = ROOT/'carter_navigation/maps'
    config = yaml.safe_load((directory/'hospital_integration_human.yaml').read_text())
    pixels = np.asarray(Image.open(directory/config['image']).convert('L'))
    occupancy = 1-pixels/255.
    data = np.where(occupancy > config['occupied_thresh'], 100,
                    np.where(occupancy < config['free_thresh'], 0, -1))
    ox, oy, yaw = config['origin']
    message = NS(info=NS(width=pixels.shape[1], height=pixels.shape[0], resolution=config['resolution'],
        origin=NS(position=NS(x=ox, y=oy), orientation=NS(x=0, y=0, z=math.sin(yaw/2), w=math.cos(yaw/2)))),
        data=np.flipud(data).ravel(), header=NS(frame_id='map', stamp=NS(sec=0, nanosec=0)))
    return Grid(message)


@pytest.mark.parametrize('direction,track', ALL_ROUTES)
def test_actual_reference_footprint_and_joins(direction, track):
    """방향 2 x 복도 4 = 8 경로: 지도상 차체 여유, 원소 이음(위치·방향) 연속, 분할(출발/MPPI 직선/도착)"""
    scope, grid = source_routes(), map_grid()
    route, split = scope['compose'](direction, track)
    assert all(grid.body_clear(p) for p in scope['sample_route'](route)), (direction, track)
    for a, b in zip(route, route[1:]):
        end, start = scope['sample_route']([a])[-1], scope['sample_route']([b])[0]
        assert math.dist(end[:2], start[:2]) < 1e-8, (a, b)
        assert abs(math.atan2(math.sin(end[2]-start[2]), math.cos(end[2]-start[2]))) < 1e-8, (a, b)
        assert not (a[0] == b[0] == 'arc'), (a, b)
    assert all(abs(s[4] - s[3]) <= 90 for s in route if s[0] == 'arc')
    departure, transit, arrival = scope['split_route'](track, route, split)
    assert len(transit) == 1 and transit[0][0] == 'line'
    assert departure[-1][0] == 'line' and math.dist(departure[-1][1], departure[-1][2]) >= 1.5
    assert arrival[0][0] == 'arc'
    assert route[-1][0] == 'line' and math.dist(route[-1][1], route[-1][2]) >= 1.5
    assert departure+transit+arrival == route


def test_default_lanes_keep_the_previous_routes():
    """관제 없이 도는 기본 경로 = 운송 아래 복도, 복귀 위 복도 (예전 lane_lower / lane_upper)"""
    scope = source_routes()
    assert scope['LANES']['lane_lower'] == scope['compose']('to_analysis', 'lower')[0]
    assert scope['LANES']['lane_upper'] == scope['compose']('to_collection', 'upper')[0]


def test_reserve_tracks_are_two_metres_off_their_main_track():
    scope = source_routes()
    for main, reserve in (('upper', 'upper_reserve'), ('lower', 'lower_reserve')):
        a, b = scope['TRACKS'][main]['straight'], scope['TRACKS'][reserve]['straight']
        assert all(abs(abs(p[1] - q[1]) - 2.0) < 1e-9 for p, q in zip(a, b))     # 긴 직선이 2 m 나란히


@pytest.mark.parametrize('direction,track', ALL_ROUTES)
def test_visible_forward_offsets_fit_actual_map_in_middle_of_lane(direction, track):
    scope, grid = source_routes(), map_grid()
    route, split = scope['compose'](direction, track)
    _, transit, _ = scope['split_route'](track, route, split)
    a, b = transit[0][1:]
    lane = LaneFrame(a[0], a[1], math.atan2(b[1]-a[1], b[0]-a[0]), math.dist(a, b))
    candidates = offset_candidates(lane, lane.world(5, 0))
    assert any(all(grid.body_clear(p) for p in path) for _, path in candidates)


def test_shared_footprint_matches_nav2_configuration():
    params = yaml.safe_load((ROOT/'carter_navigation/params/hospital_navigation_params.yaml').read_text())
    polygon = ast.literal_eval(params['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    settings = SafetySettings()
    assert min(p[0] for p in polygon) == -settings.rear
    assert max(p[0] for p in polygon) == settings.front
    assert max(abs(p[1]) for p in polygon) == settings.half_width


def test_usd_spawn_is_within_lab_dock_departure_tolerance():
    scope = source_routes()
    start = scope['sample_route'](scope['LANE_LOWER'])[0]
    assert math.dist(start[:2], (20.27, 13.74534)) < .5
    assert abs(start[2]-math.pi/2) < 1e-8


def test_staging_to_dock_straight_clears_map_beside_long_desk_side():
    scope, grid = source_routes(), map_grid()
    settings = SafetySettings()
    # Desk long sides x=20.835 (lab, west side) and x=-45.391 (specimen, east side).
    for dock, staging, yaw, desk_x in [
            (scope['LAB_DOCK'], scope['LAB_STATION'], math.pi/2, 20.835000023841857),
            (scope['SPECIMEN_DOCK'], scope['SPECIMEN_STATION'], -math.pi/2, -45.391000023841855)]:
        assert math.isclose(abs(desk_x-dock[0])-settings.half_width, .15, abs_tol=1e-6)
        straight = scope['sample_route']([('line', staging, dock)])
        assert all(abs(p[2]-yaw) < 1e-8 for p in straight)
        assert all(grid.body_clear(p) for p in straight)
