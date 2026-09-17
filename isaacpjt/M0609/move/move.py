"""First test: move the M0609 transport asset only, without Pick & Place or ROS 2.

Before executing, change only these values:
  1. USD_PATH: the full path of the environment USD saved in Isaac Sim.
  2. TRANSPORT_ROOT_PATH: the cube/Xform that parents M0609, tray, and wheels.
  3. ROUTE_XY_TO_ANALYSIS: later replace with the team's actual D-shaped path.

Place this one file here on the GPU PC:
    /home/hj/cobot3_ws/isaacpjt/M0609/lula_ik/1_transport_movement_test.py

Run with Isaac Sim's Python, not Ubuntu's normal python3:
    ./python.sh 1_transport_movement_test.py
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import math

import numpy as np
import omni.usd

from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import is_stage_loading, open_stage

# ---------------------------------------------------------------------------
# Team setup: edit these three values only.
# ---------------------------------------------------------------------------
USD_PATH = Path("/CHANGE/THIS/TO/your_hospital_environment.usd")
TRANSPORT_ROOT_PATH = "/World/TransportRoot"

# World XY coordinates in metres. The script retains the current root Z height.
# The example makes a D-shaped route: straight -> right -> down.
ROUTE_XY_TO_ANALYSIS = [
    np.array([0.0, 1.20]),
    np.array([1.80, 1.20]),
    np.array([1.80, -0.80]),
]

SPEED_MPS = 0.25
TURN_SPEED_DEG_S = 120.0
POSITION_TOLERANCE_M = 0.01


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(quat_wxyz: np.ndarray) -> float:
    """Extract Z yaw from Isaac Sim quaternion order: (w, x, y, z)."""
    w, x, y, z = quat_wxyz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> np.ndarray:
    """Create an Isaac Sim quaternion in (w, x, y, z) order."""
    half_yaw = yaw / 2.0
    return np.array([math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)])


def set_transport_pose(root: XFormPrim, position: np.ndarray, orientation: np.ndarray) -> None:
    """Set the root pose; also neutralize old velocity if the root has physics."""
    root.set_world_pose(position=position, orientation=orientation)
    if hasattr(root, "set_linear_velocity"):
        root.set_linear_velocity(np.zeros(3))
    if hasattr(root, "set_angular_velocity"):
        root.set_angular_velocity(np.zeros(3))


def main() -> None:
    if not USD_PATH.is_file():
        raise FileNotFoundError(
            f"USD file was not found: {USD_PATH}\n"
            "Set USD_PATH to the full path of the saved Isaac Sim environment."
        )

    if not open_stage(str(USD_PATH)):
        raise RuntimeError(f"Could not open USD: {USD_PATH}")
    while is_stage_loading():
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    transport_root = XFormPrim(
        prim_path=TRANSPORT_ROOT_PATH,
        name="transport_root",
    )
    world.scene.add(transport_root)
    world.reset()

    # Fail early if the prim path was mistyped or the asset was not saved.
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(TRANSPORT_ROOT_PATH)
    if not prim.IsValid():
        raise RuntimeError(
            f"Transport root does not exist: {TRANSPORT_ROOT_PATH}\n"
            "In Isaac Sim's Stage panel, copy the cube/Xform path that parents "
            "the M0609, tray, and wheels."
        )

    start_position, _ = transport_root.get_world_pose()
    route_to_analysis = [
        np.array([point_xy[0], point_xy[1], start_position[2]])
        for point_xy in ROUTE_XY_TO_ANALYSIS
    ]
    print(f"[START] root={TRANSPORT_ROOT_PATH}, position={start_position}")
    print(f"[ROUTE] {route_to_analysis}")

    waypoint_index = 0
    turn_speed_rad_s = math.radians(TURN_SPEED_DEG_S)

    reached_goal_printed = False
    while simulation_app.is_running():
        world.step(render=True)

        if reached_goal_printed:
            continue

        dt = world.get_physics_dt()
        position, orientation = transport_root.get_world_pose()
        position = np.asarray(position, dtype=float)
        orientation = np.asarray(orientation, dtype=float)
        target = route_to_analysis[waypoint_index]
        offset = target - position
        planar_distance = float(np.linalg.norm(offset[:2]))

        # First turn toward the next segment, then move at a constant speed.
        if planar_distance > POSITION_TOLERANCE_M:
            target_yaw = math.atan2(offset[1], offset[0])
            yaw_error = wrap_angle(target_yaw - yaw_from_quat(orientation))
            if abs(yaw_error) > math.radians(2.0):
                yaw_step = math.copysign(
                    min(abs(yaw_error), turn_speed_rad_s * dt), yaw_error
                )
                set_transport_pose(
                    transport_root,
                    position,
                    quat_from_yaw(yaw_from_quat(orientation) + yaw_step),
                )
                continue

        distance = float(np.linalg.norm(offset))
        if distance <= POSITION_TOLERANCE_M:
            set_transport_pose(transport_root, target, orientation)
            waypoint_index += 1
            if waypoint_index == len(route_to_analysis):
                print("[DONE] Final waypoint reached. The movement-only test is complete.")
                reached_goal_printed = True
            continue

        travel = min(SPEED_MPS * dt, distance)
        new_position = position + offset / distance * travel
        set_transport_pose(transport_root, new_position, orientation)

        # Keep the final pose visible. Close the Isaac Sim window to finish.


try:
    main()
finally:
    simulation_app.close()
