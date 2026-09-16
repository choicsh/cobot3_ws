from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)               
stage = omni.usd.get_context().get_stage()            

cube_prim_big = DynamicCuboid(                             
    prim_path="/World/BigCube",
    name="big_cube",
    position=np.array([0.0, 0.0, 0.15]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)
cube_prim_small = DynamicCuboid(                             
    prim_path="/World/SmallCube",
    name="small_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.1, 0.1, 0.1]),
    color=np.array([0.0, 1.0, 0.0]),
)

world.scene.add_default_ground_plane()                 
world.scene.add(cube_prim_big)
world.scene.add(cube_prim_small)

world.reset()

while simulation_app.is_running():                     
    world.step(render=True)

simulation_app.close()