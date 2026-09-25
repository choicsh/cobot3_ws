import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path

from nav_to_goal.hospital_mission import (
    ARRIVAL_YAWS, LAB_DOCK, LAB_STATION, LANES, ROUTES, SPECIMEN_DOCK,
    SPECIMEN_STATION, first_blocked_path_point, sample_route, split_route,
)
from nav_to_goal.hospital_docking import TABLES


def yaw_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_mppi_only_straight_and_all_turns_dwb():
    for lane, route in LANES.items():
        departure, transit, arrival = split_route(lane, route)
        assert len(transit) == 1 and transit[0][0] == "line"
        assert departure + transit + arrival == route
        assert departure[-1][0] == arrival[0][0] == "arc"
        for before, after in [(departure, transit), (transit, arrival)]:
            a, b = sample_route(before)[-1], sample_route(after)[0]
            assert math.dist(a[:2], b[:2]) < 1e-8
            assert yaw_error(a[2], b[2]) < 1e-8


def test_routes_run_from_origin_dock_to_destination_staging():
    ends = {"lab_to_specimen": ("lab", "specimen"), "specimen_to_lab": ("specimen", "lab")}
    for route_id, lane in ROUTES.items():
        origin, destination = ends[route_id]
        points = sample_route(LANES[lane])
        dock = TABLES[origin]["dock"]
        # Depart straight along the origin desk, continuing the dock heading.
        assert math.dist(points[0][:2], dock[:2]) < 1e-6
        assert yaw_error(points[0][2], dock[2]) < 1e-8
        staging, final_dock = TABLES[destination]["staging"], TABLES[destination]["dock"]
        assert math.dist(points[-1][:2], staging) < 1e-6
        assert yaw_error(points[-1][2], ARRIVAL_YAWS[route_id]) < 1e-8
        assert yaw_error(final_dock[2], ARRIVAL_YAWS[route_id]) < 1e-8


def test_mission_station_constants_match_docking_tables():
    for name, dock, staging in [("lab", LAB_DOCK, LAB_STATION),
                                ("specimen", SPECIMEN_DOCK, SPECIMEN_STATION)]:
        assert math.dist(dock, TABLES[name]["dock"][:2]) < 1e-6
        assert math.dist(staging, TABLES[name]["staging"]) < 1e-6


def test_static_blockage_scan_finds_lethal_but_ignores_unknown():
    costmap = Costmap()
    costmap.metadata.resolution = 0.1
    costmap.metadata.size_x = 100
    costmap.metadata.size_y = 100
    costmap.metadata.origin.orientation.w = 1.0
    costmap.data = [0] * 10000
    # Path y=5.0, x=1.0..8.0. x=5.0 lethal, x=3.0 unknown.
    costmap.data[50 * 100 + 30] = 255
    costmap.data[50 * 100 + 50] = 254

    path = Path()
    path.header.frame_id = "map"
    for index in range(71):
        pose = PoseStamped()
        pose.pose.position.x = 1.0 + index * 0.1
        pose.pose.position.y = 5.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0

    blocked = first_blocked_path_point(
        costmap, path, 0, transform, min_ahead=0.5, lookahead=5.0
    )
    assert blocked is not None
    assert math.isclose(blocked[0], 5.0, abs_tol=0.11)
    assert blocked[-1] == 254
