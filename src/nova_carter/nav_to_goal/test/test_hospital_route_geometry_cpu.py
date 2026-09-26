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
    source = ROOT/'nav_to_goal/nav_to_goal/hospital_mission.py'
    tree = ast.parse(source.read_text())
    names = {'PATH_STEP', 'LAB_DOCK', 'LAB_STATION', 'SPECIMEN_DOCK', 'SPECIMEN_STATION',
             'ARRIVAL_YAWS', 'ROUTES', 'WEST_DOOR_Y', 'WEST_LOOP_X', 'LANE_UPPER', 'LANE_LOWER', 'LANES'}
    nodes = [n for n in tree.body if
             (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in n.targets))
             or (isinstance(n, ast.FunctionDef) and n.name in ('sample_route', 'route_length', 'split_route'))]
    scope = {'math': math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), scope)
    return scope


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


@pytest.mark.parametrize('name', ['lane_upper', 'lane_lower'])
def test_actual_reference_footprint_and_joins(name):
    scope, grid = source_routes(), map_grid()
    route = scope['LANES'][name]
    assert all(grid.body_clear(p) for p in scope['sample_route'](route))
    for a, b in zip(route, route[1:]):
        end, start = scope['sample_route']([a])[-1], scope['sample_route']([b])[0]
        assert math.dist(end[:2], start[:2]) < 1e-8
        assert abs(math.atan2(math.sin(end[2]-start[2]), math.cos(end[2]-start[2]))) < 1e-8
    departure, transit, arrival = scope['split_route'](name, route)
    assert len(transit) == 1 and transit[0][0] == 'line'
    assert departure+transit+arrival == route


@pytest.mark.parametrize('name', ['lane_upper', 'lane_lower'])
def test_visible_forward_offsets_fit_actual_map_in_middle_of_lane(name):
    scope, grid = source_routes(), map_grid()
    _, transit, _ = scope['split_route'](name, scope['LANES'][name])
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
