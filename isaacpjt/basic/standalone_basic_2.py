from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)               
stage = omni.usd.get_context().get_stage()            

cube_prim_r = DynamicCuboid(                             
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([1.0, 0.0, 0.0]),
)
cube_prim_g = DynamicCuboid(                             
    prim_path="/World/GreenCube",
    name="green_cube",
    position=np.array([1.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 1.0, 0.0]),
)
cube_prim_b = DynamicCuboid(                             
    prim_path="/World/BlueCube",
    name="blue_cube",
    position=np.array([-1.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 0.0, 1.0]),
)

world.scene.add_default_ground_plane()                 
world.scene.add(cube_prim_r)
world.scene.add(cube_prim_g)
world.scene.add(cube_prim_b)

world.reset()

while simulation_app.is_running():                     
    world.step(render=True)

simulation_app.close()