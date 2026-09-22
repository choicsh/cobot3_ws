"""제작 로봇이 포함된 병원 씬을 실행하고 ROS 2 브리지를 활성화한다.

    isaac_python run_hospital_sim.py
    isaac_python run_hospital_sim.py headless
"""

import json
import math
import sys

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": "headless" in sys.argv})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import is_stage_loading, open_stage


USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/hospital_integration.usd"
BASE_LINK_PRIM = "/World/robot_nova/nova_carter/chassis_link/base_link"
START_POSE_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/start_pose.json"


open_stage(USD_PATH)
while is_stage_loading():
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.reset()

# 기존 ground_truth_localization 흐름을 그대로 사용한다.
positions, quaternions = XFormPrim(BASE_LINK_PRIM).get_world_poses()
x, y = float(positions[0][0]), float(positions[0][1])
w, qx, qy, qz = (float(value) for value in quaternions[0])
yaw_deg = math.degrees(
    math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
)

with open(START_POSE_PATH, "w", encoding="utf-8") as pose_file:
    json.dump(
        {"frame": "map", "x": x, "y": y, "yaw_deg": yaw_deg},
        pose_file,
        indent=2,
    )

print(
    f"[HOSPITAL] start pose -> {START_POSE_PATH}: "
    f"x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f} deg",
    flush=True,
)

omni.timeline.get_timeline_interface().play()
print(f"[HOSPITAL] {USD_PATH} 재생 시작 (Ctrl+C로 종료)", flush=True)

try:
    while simulation_app.is_running():
        world.step(render=True)
finally:
    simulation_app.close()
