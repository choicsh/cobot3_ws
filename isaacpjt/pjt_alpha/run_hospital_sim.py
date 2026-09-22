"""제작 로봇과 보행자 6명이 포함된 병원 씬을 실행한다.

    isaac_python run_hospital_sim.py
    isaac_python run_hospital_sim.py headless
"""

import json
import math
import sys
import time
import os
from pathlib import Path

from isaacsim import SimulationApp


simulation_app = SimulationApp({
    "headless": "headless" in sys.argv,
    "width": 1280,
    "height": 720,
    "anti_aliasing": 2,  # FXAA: avoid temporal upscaling overhead in the operator view.
})

from isaacsim.core.utils.extensions import enable_extension

PEOPLE_EXTENSIONS = [
    "omni.anim.people",
    "omni.anim.navigation.bundle",
    "omni.anim.timeline",
    "omni.anim.graph.bundle",
    "omni.anim.graph.core",
    "omni.anim.retarget.bundle",
    "omni.anim.retarget.core",
    "omni.kit.scripting",
]
for extension_name in PEOPLE_EXTENSIONS:
    enable_extension(extension_name)
    simulation_app.update()

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import carb

PEOPLE_COMMAND_FILE = (
    "/home/rokey/cobot3_ws/isaacpjt/assets/people/command.txt"
)
PEOPLE_WALK_SPEED_SCALE = float(os.environ.get("HOSPITAL_PEOPLE_WALK_BLEND", "0.75"))
if not 0.1 <= PEOPLE_WALK_SPEED_SCALE <= 1.0:
    raise ValueError("HOSPITAL_PEOPLE_WALK_BLEND must be between 0.1 and 1.0")

settings = carb.settings.get_settings()
# Avoid running faster than real time when the GPU has spare capacity.
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
settings.set_int("/app/runLoops/main/rateLimitFrequency", 30)
settings.set(
    "/exts/omni.anim.people/command_settings/command_file_path",
    PEOPLE_COMMAND_FILE,
)
settings.set(
    "/exts/omni.anim.people/command_settings/number_of_loop", "inf"
)
settings.set(
    # 자유 보행(무작위 GoTo) 모드: 가구/건물을 돌아가야 하므로 NavMesh 경로계획을 켠다.
    # (직선 왕복 시험 때는 NavMesh 투영이 y=14.9 선에서 벗어나게 해서 False 로 뒀었다.)
    # NavMeshVolume 은 hospital_people.usd 에 x[-36,12] y[-8.45,22.65] 로 추가됨.
    "/exts/omni.anim.people/navigation_settings/navmesh_enabled", True
)
settings.set(
    "/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled",
    False,
)
simulation_app.update()

# GoTo 명령에는 속도 인자가 없으므로 캐릭터별 Walk blend 값을 제한한다.
# Walk blend 상한이다. 실제 m/s 및 기존 대비 속도 비율은 audit으로 측정한다.
from omni.anim.people.scripts.commands.base_command import Command

_original_people_walk = Command.walk


def _slow_people_walk(command, delta_time):
    result = _original_people_walk(command, delta_time)
    if command.desired_walk_speed > 0.0:
        command.character.set_variable(
            "Walk",
            min(command.actual_walk_speed, PEOPLE_WALK_SPEED_SCALE),
        )
    return result


Command.walk = _slow_people_walk

import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import is_stage_loading, open_stage


USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/hospital_people.usd"
BASE_LINK_PRIM = "/World/robot_nova/nova_carter/chassis_link/base_link"
START_POSE_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/start_pose.json"

# This launcher is for our six known NVIDIA character behaviors. Validate every
# attached script before allowing this stage to execute them without a GUI prompt.
from pxr import Usd
import omni.anim.people.scripts.character_behavior as people_behavior

preflight = Usd.Stage.Open(USD_PATH)
expected_script = Path(people_behavior.__file__).resolve()
script_count = 0
for prim in preflight.Traverse():
    scripts = prim.GetAttribute("omni:scripting:scripts")
    if not scripts or not scripts.Get():
        continue
    for script in scripts.Get():
        if (not str(prim.GetPath()).startswith("/World/Characters/")
                or Path(script.path).resolve() != expected_script):
            raise RuntimeError(f"Unexpected USD behavior: {prim.GetPath()} {script}")
        script_count += 1
if script_count != 6:
    raise RuntimeError(f"Expected six pedestrian behaviors, found {script_count}")
preflight = None
previous_script_prompt = settings.get_as_bool("/app/scripting/ignoreWarningDialog")
settings.set_bool("/app/scripting/ignoreWarningDialog", True)

open_stage(USD_PATH)
while is_stage_loading():
    simulation_app.update()

# Keep 60 Hz physics, but refresh the operator view at 30 Hz. The application
# still updates on every render tick; do not replace this with render=False,
# which would also stop refreshing the UI and render-based sensors.
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0,
              rendering_dt=1.0 / 30.0)

# Infinite People loops append a return to their initial pose. Match that pose
# to the final TXT waypoint so an old spawn inside the widened path cannot
# introduce an unintended intermediate stop. Session-only: neither USD is saved.
from pxr import Gf
from hospital_people_commands import loop_origins

with Usd.EditContext(world.stage, world.stage.GetSessionLayer()):
    for name, position in loop_origins(PEOPLE_COMMAND_FILE).items():
        prim = world.stage.GetPrimAtPath(f"/World/Characters/{name}")
        translate = prim.GetAttribute("xformOp:translate")
        if not translate:
            raise RuntimeError(f"Missing pedestrian translate op: {name}")
        translate.Set(Gf.Vec3d(*position))
        print(f"[HOSPITAL] {name} loop origin={position}", flush=True)


def bake_navmesh(timeout_seconds=30.0):
    """USD에 구성된 병원 NavMesh 볼륨을 standalone 실행 시 굽는다."""
    import omni.anim.navigation.core as navigation

    interface = navigation.acquire_interface()
    interface.start_navmesh_baking()
    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        simulation_app.update()
        if not interface.is_navmesh_baking():
            break
    else:
        raise RuntimeError(
            f"NavMesh bake timed out after {timeout_seconds:.0f}s"
        )

    if interface.get_navmesh() is None:
        raise RuntimeError("NavMesh bake failed")
    print("[HOSPITAL] NavMesh bake complete", flush=True)


bake_navmesh()
world.reset()
settings.set_bool("/app/scripting/ignoreWarningDialog", previous_script_prompt)

# 기존 ground_truth_localization 흐름을 그대로 사용한다.
robot_base = XFormPrim(BASE_LINK_PRIM)
positions, quaternions = robot_base.get_world_poses()
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
print(
    f"[HOSPITAL] {USD_PATH} 재생 시작 — 사람 6명, "
    f"Walk blend 상한 {PEOPLE_WALK_SPEED_SCALE:.2f} "
    f"({PEOPLE_COMMAND_FILE}) (Ctrl+C로 종료)",
    flush=True,
)

try:
    while simulation_app.is_running():
        world.step(render=True)
finally:
    simulation_app.close()
