from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
import omni.timeline

timeline = omni.timeline.get_timeline_interface()

world = World(stage_units_in_meters=1.0)               
stage = omni.usd.get_context().get_stage()            

cube_prim = DynamicCuboid(                             
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([1.0, 0.0, 0.0]),
)

world.scene.add_default_ground_plane()                 
world.scene.add(cube_prim)

world.reset()

step_count = 0

while simulation_app.is_running():                     
    world.step(render=True)
    time.sleep(0.01)
    step_count += 1
    if step_count % 100 ==0:
        print(step_count)

    if step_count == 500:
        print("시뮬레이션 종료")
        break

simulation_app.close()