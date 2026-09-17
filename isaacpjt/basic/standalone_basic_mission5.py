from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import time
import omni.usd
import omni.timeline

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid


world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

timeline = omni.timeline.get_timeline_interface()


initial_position = np.array([0.0, 0.0, 0.3])


cube_prim_r = DynamicCuboid(
    prim_path="/World/RedCube",
    name="red_cube",
    position=initial_position,
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)


world.scene.add_default_ground_plane()
world.scene.add(cube_prim_r)

world.reset()


step_count = 0
was_playing = False
moved = False


while simulation_app.is_running():

    world.step(render=True)

    is_playing = timeline.is_playing()


    # Stop -> Play로 다시 시작한 순간
    if is_playing and not was_playing:

        step_count = 0
        moved = False

        cube_prim_r.set_world_pose(
            position=initial_position
        )

        cube_prim_r.set_linear_velocity(
            np.array([0.0, 0.0, 0.0])
        )

        cube_prim_r.set_angular_velocity(
            np.array([0.0, 0.0, 0.0])
        )

        print("[리셋] Play 시작 -> step_count = 0")


    # Play 중일 때만 카운트 증가
    if is_playing:

        step_count += 1

        if step_count % 100 == 0:
            print("step:", step_count)


        # 300 step에서 순간이동
        if step_count == 300 and not moved:

            cube_prim_r.set_world_pose(
                position=np.array([0.0, 0.0, 1.0])
            )

            moved = True

            print("[이동] RedCube -> z = 1.0 m")


    was_playing = is_playing

    time.sleep(0.01)


simulation_app.close()