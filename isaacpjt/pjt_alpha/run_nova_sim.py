"""ROS2 토픽 확인용 실행기 — 씬 열고 ROS2 브리지 켜고 Play 까지

    isaac_python run_nova_sim.py           # GUI
    isaac_python run_nova_sim.py headless  # 창 없이

Ctrl+C 로 종료할 때까지 계속 돈다.
"""
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": "headless" in sys.argv})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.stage import is_stage_loading, open_stage

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/intergration_nova.usd"

open_stage(USD_PATH)
while is_stage_loading():
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.reset()
omni.timeline.get_timeline_interface().play()
print(f"\n[RUN] {USD_PATH} 재생 시작 — ROS2 토픽 발행 중 (Ctrl+C 로 종료)\n", flush=True)

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
