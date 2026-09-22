"""ROS2 토픽 확인용 실행기 — 씬 열고 ROS2 브리지 켜고 Play 까지

    isaac_python run_nova_sim.py           # GUI
    isaac_python run_nova_sim.py headless  # 창 없이

Ctrl+C 로 종료할 때까지 계속 돈다.
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

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/intergration_nova.usd"
# AMCL 의 base_frame_id(base_link) 와 같은 프림. Nav2 쪽에서 이 파일을 읽어 초기 위치로 쓴다.
BASE_LINK_PRIM = "/World/robot_nova/nova_carter/chassis_link/base_link"
START_POSE_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/start_pose.json"

open_stage(USD_PATH)
while is_stage_loading():
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.reset()

pos, quat = XFormPrim(BASE_LINK_PRIM).get_world_poses()
x, y = float(pos[0][0]), float(pos[0][1])
w, qx, qy, qz = (float(v) for v in quat[0])
yaw_deg = math.degrees(math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
with open(START_POSE_PATH, "w") as f:
    json.dump({"frame": "map", "x": x, "y": y, "yaw_deg": yaw_deg}, f, indent=2)
print(f"[RUN] start pose -> {START_POSE_PATH}: x={x:.3f} y={y:.3f} yaw={yaw_deg:.1f}deg", flush=True)

omni.timeline.get_timeline_interface().play()
print(f"\n[RUN] {USD_PATH} 재생 시작 — ROS2 토픽 발행 중 (Ctrl+C 로 종료)\n", flush=True)

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
