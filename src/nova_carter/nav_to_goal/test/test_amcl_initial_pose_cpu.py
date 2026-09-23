"""Check the Isaac-start/odom composition without requiring a ROS runtime."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace


SOURCE = Path(__file__).resolve().parents[1] / 'nav_to_goal' / 'amcl_initial_pose.py'
MODULE = ast.parse(SOURCE.read_text(encoding='utf-8'))
FUNCTION = next(
    node for node in MODULE.body
    if isinstance(node, ast.FunctionDef) and node.name == 'map_pose_from_start_and_odom'
)
namespace = {'math': math}
exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), str(SOURCE), 'exec'), namespace)
map_pose_from_start_and_odom = namespace[FUNCTION.name]


def odom_pose(x, y, yaw):
    return SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y),
        rotation=SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)))


def test_initial_odom_origin_keeps_isaac_start_pose():
    x, y, yaw = map_pose_from_start_and_odom(
        12.0, 8.0, -math.pi / 2.0, odom_pose(0.0, 0.0, 0.0))
    assert math.isclose(x, 12.0)
    assert math.isclose(y, 8.0)
    assert math.isclose(yaw, -math.pi / 2.0)


def test_navigation_started_after_robot_moved_uses_current_pose():
    x, y, yaw = map_pose_from_start_and_odom(
        12.0, 8.0, -math.pi / 2.0, odom_pose(2.0, 0.0, math.pi / 2.0))
    assert math.isclose(x, 12.0)
    assert math.isclose(y, 6.0)
    assert math.isclose(yaw, 0.0, abs_tol=1e-12)
