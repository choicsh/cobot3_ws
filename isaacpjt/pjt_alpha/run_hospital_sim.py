"""제작 로봇과 보행자 3명이 포함된 병원 씬을 실행한다.

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

SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR.parent / "assets"

PEOPLE_COMMAND_FILE = str(
    ASSETS_DIR / "people" / "hospital_integration_human_command.txt"
)
PEOPLE_NAMES = {"Character", "Character_01", "Character_02"}
PEOPLE_ENABLED = os.environ.get("HOSPITAL_PEOPLE_ENABLED", "1") != "0"
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
    # NavMeshVolume 은 병원 USD에 포함된 설정을 사용한다.
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


USD_PATH = str(ASSETS_DIR / "hospital_integration_human.usd")
BASE_LINK_PRIM = "/World/robot_nova/nova_carter/chassis_link/base_link"
START_POSE_PATH = str(ASSETS_DIR / "start_pose.json")

# This launcher is for the new scene's three NVIDIA character behaviors. Validate every
# attached script before allowing this stage to execute them without a GUI prompt.
from pxr import Sdf, Usd
import omni.anim.people.scripts.character_behavior as people_behavior

preflight = Usd.Stage.Open(USD_PATH)
expected_script = Path(people_behavior.__file__).resolve()
# The scene stores the authoring machine's absolute path. Accept only this
# extension's character_behavior.py wherever Isaac Sim is installed; the
# session layer below remaps the value to the local copy.
EXPECTED_SCRIPT_TAIL = Path(*expected_script.parts[-5:])
behavior_prim_paths = []
for prim in preflight.Traverse():
    scripts = prim.GetAttribute("omni:scripting:scripts")
    if not scripts or not scripts.Get():
        continue
    for script in scripts.Get():
        candidate = Path(script.path)
        if (not str(prim.GetPath()).startswith("/World/Characters/")
                or len(candidate.parts) < 5
                or Path(*candidate.parts[-5:]) != EXPECTED_SCRIPT_TAIL):
            raise RuntimeError(f"Unexpected USD behavior: {prim.GetPath()} {script}")
    behavior_prim_paths.append(prim.GetPath())
if len(behavior_prim_paths) != len(PEOPLE_NAMES):
    raise RuntimeError(
        f"Expected {len(PEOPLE_NAMES)} pedestrian behaviors, "
        f"found {len(behavior_prim_paths)}")
preflight = None
previous_script_prompt = settings.get_as_bool("/app/scripting/ignoreWarningDialog")
settings.set_bool("/app/scripting/ignoreWarningDialog", True)

open_stage(USD_PATH)
while is_stage_loading():
    simulation_app.update()

# 뷰포트 라이팅을 Stage Lights 대신 Default 로 (뷰포트 메뉴와 같은 액션). 헤드리스면 액션이 없다.
import omni.kit.actions.core

_lighting = omni.kit.actions.core.get_action_registry().get_action(
    "omni.kit.viewport.menubar.lighting", "set_lighting_mode_rig")
if _lighting is not None:
    _lighting.execute("Default")

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
    # Point the validated behaviors at this machine's extension copy.
    for behavior_path in behavior_prim_paths:
        world.stage.GetPrimAtPath(behavior_path).GetAttribute(
            "omni:scripting:scripts"
        ).Set(Sdf.AssetPathArray([str(expected_script)]))
    for name, position in loop_origins(PEOPLE_COMMAND_FILE, PEOPLE_NAMES).items():
        prim = world.stage.GetPrimAtPath(f"/World/Characters/{name}")
        translate = prim.GetAttribute("xformOp:translate")
        if not translate:
            raise RuntimeError(f"Missing pedestrian translate op: {name}")
        translate.Set(Gf.Vec3d(*position))
        print(f"[HOSPITAL] {name} loop origin={position}", flush=True)
    if not PEOPLE_ENABLED:
        world.stage.GetPrimAtPath('/World/Characters').SetActive(False)


FRONT_LIDAR_PRIM = "/World/robot_nova/nova_carter/chassis_link/sensors/XT_32/front_3d_lidar"


def lighten_front_lidar(stage):
    """Isaac 라이다 발행이 약 5 MB/s에서 막혀 770 KB 스캔이 10 Hz 중 6.4 Hz만 나갔다.

    USD 라이다는 발광기 128개 = 고도 32개 x 방위 오프셋 4개(-3~+3 deg)라 수평 간격이
    0.1 deg로 Nav2 스캔(0.5 deg)보다 훨씬 촘촘하다. 고도마다 방위 0에 가장 가까운
    발광기 하나만 남기고(실제 XT-32의 32채널) 발사 빈도를 18 kHz(0.2 deg)로 두면
    306 KB, 8.4 Hz, 최대 간격 0.3 s (2026-09-25 실측). 세션 레이어만 바꾼다.
    """
    lidar = stage.GetPrimAtPath(FRONT_LIDAR_PRIM)
    prefix = "omni:sensor:Core:emitterState:s001:"
    azimuth = list(lidar.GetAttribute(prefix + "azimuthDeg").Get())
    elevation = list(lidar.GetAttribute(prefix + "elevationDeg").Get())
    keep = {}
    for index, (az, el) in enumerate(zip(azimuth, elevation)):
        key = round(el, 2)
        if key not in keep or abs(az) < abs(azimuth[keep[key]]):
            keep[key] = index
    indices = sorted(keep.values(), key=lambda i: elevation[i])
    for name in ("azimuthDeg", "elevationDeg", "fireTimeNs"):
        attribute = lidar.GetAttribute(prefix + name)
        values = list(attribute.Get())
        attribute.Set(type(attribute.Get())([values[i] for i in indices]))
    channel = lidar.GetAttribute(prefix + "channelId")
    channel.Set(type(channel.Get())(list(range(1, len(indices) + 1))))
    lidar.GetAttribute("omni:sensor:Core:numberOfEmitters").Set(len(indices))
    lidar.GetAttribute("omni:sensor:Core:numberOfChannels").Set(len(indices))
    lidar.GetAttribute("omni:sensor:Core:reportRateBaseHz").Set(18000)
    print(f"[HOSPITAL] front lidar: {len(indices)} channels, 18 kHz", flush=True)


with Usd.EditContext(world.stage, world.stage.GetSessionLayer()):
    lighten_front_lidar(world.stage)

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

# 실제 시작 자세를 AMCL 초기 위치 입력으로 저장한다.
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
    f"[HOSPITAL] {USD_PATH} 재생 시작 — 사람 {len(PEOPLE_NAMES) if PEOPLE_ENABLED else 0}명, "
    f"Walk blend 상한 {PEOPLE_WALK_SPEED_SCALE:.2f} "
    f"({PEOPLE_COMMAND_FILE}) (Ctrl+C로 종료)",
    flush=True,
)

try:
    while simulation_app.is_running():
        world.step(render=True)
finally:
    simulation_app.close()
