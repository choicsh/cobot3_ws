import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path

from nav_to_goal.hospital_mission import (
    ARRIVAL_YAWS, LANES, ROUTES, first_blocked_path_point,
    sample_route, split_route,
)


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


def test_arrival_yaw_matches_tangent_and_staging_position():
    for route_id, lane in ROUTES.items():
        final = sample_route(LANES[lane])[-1]
        other = "lane_lower" if lane == "lane_upper" else "lane_upper"
        next_start = sample_route(LANES[other])[0]
        assert math.dist(final[:2], next_start[:2]) < 1e-8
        assert yaw_error(final[2], ARRIVAL_YAWS[route_id]) < 1e-8
        # Specimen leaves its table by backing out, then spinning north in
        # open space. Lab staging can directly depart west after undocking.
        expected = math.pi/2 if lane == 'lane_lower' else math.pi
        assert yaw_error(next_start[2], expected) < 1e-8


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
