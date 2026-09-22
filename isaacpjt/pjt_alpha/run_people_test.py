"""MPPI 복도 회피 테스트용 실행기 — 사람 1명이 있는 씬(integration_human.usd)을 열고
ROS2 브리지 + 사람 애니메이션 확장을 켠 뒤 Play 까지 자동으로 한다.

    isaac_python run_people_test.py           # GUI
    isaac_python run_people_test.py headless  # 창 없이

run_nova_sim.py 와 구조는 같고, 차이는:
  - 씬을 integration_human.usd 로 (로봇 + 사람 1명 + NavMeshVolume x[3.5,10.5] y[1.5,18.5])
  - omni.anim.people 계열 확장을 씬을 열기 전에 켠다 (안 켜면 캐릭터가 안 움직인다)
  - 사람 경로는 people/command.txt 그대로 (여기서 좌표를 정하지 않는다). 무한 반복.
    주의: command.txt 는 씬을 열 때 한 번만 읽으므로, 파일을 고치면 이 스크립트를 다시 실행해야 반영된다.
    GoTo 좌표는 NavMeshVolume 안에 있어야 한다 (밖이면 명령이 거부되어 제자리에 서 있는다).

Ctrl+C 로 종료할 때까지 계속 돈다.
"""
import json
import math
import sys
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": "headless" in sys.argv})

from isaacsim.core.utils.extensions import enable_extension

# omni.anim.people 의 캐릭터 behavior 스크립트가 omni.anim.graph.core 등을 import 하므로
# 스테이지를 열기 전에 켜야 한다 (pickup_place_go_nova.py 와 동일한 목록).
PEOPLE_EXTENSIONS = [
    "omni.anim.people",
    "omni.anim.navigation.bundle",
    "omni.anim.timeline",
    "omni.anim.graph.bundle",
    "omni.anim.graph.core",
    "omni.anim.graph.ui",
    "omni.anim.retarget.bundle",
    "omni.anim.retarget.core",
    "omni.anim.retarget.ui",
    "omni.kit.scripting",
]
for _ext in PEOPLE_EXTENSIONS:
    enable_extension(_ext)
    simulation_app.update()

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# command.txt 는 초기화 시점에 한 번만 읽으므로 반드시 open_stage() 앞에서 설정해야 한다.
import carb

PEOPLE_COMMAND_FILE = "/home/rokey/cobot3_ws/isaacpjt/assets/people/command.txt"
_settings = carb.settings.get_settings()
_settings.set("/exts/omni.anim.people/command_settings/command_file_path", PEOPLE_COMMAND_FILE)
_settings.set("/exts/omni.anim.people/command_settings/number_of_loop", "inf")  # 기본 "0" 은 1회 재생 후 정지
_settings.set("/exts/omni.anim.people/navigation_settings/navmesh_enabled", True)
_settings.set("/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled", False)  # 사람이 로봇을 스스로 피하면 "로봇이 피하는지" 테스트가 안 된다
simulation_app.update()

import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import is_stage_loading, open_stage

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/integration_human.usd"
BASE_LINK_PRIM = "/World/robot_nova/nova_carter/chassis_link/base_link"
START_POSE_PATH = "/home/rokey/cobot3_ws/isaacpjt/assets/start_pose.json"

open_stage(USD_PATH)
while is_stage_loading():
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.reset()


def bake_navmesh(timeout_s=30.0):
    """NavMesh 를 베이크하고 완료를 기다린다.

    베이크 결과는 USD 에 저장되지 않는다(스키마에 담을 프림 타입 자체가 없다).
    GUI 는 autoRebakeOnChanges 로 알아서 다시 굽지만 standalone 은 직접 호출해야 한다.
    안 구우면 캐릭터의 GoTo 가 전부 'invalid command' 로 거부되어 사람이 안 움직인다
    (pickup_place_go_nova.py 의 bake_navmesh() 와 동일).
    """
    try:
        import omni.anim.navigation.core as nav
    except ImportError as e:
        print(f"[RUN] navmesh 건너뜀 (omni.anim.navigation.core 없음: {e})", flush=True)
        return False

    inav = nav.acquire_interface()
    inav.start_navmesh_baking()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        simulation_app.update()
        if not inav.is_navmesh_baking():
            break
    else:
        print(f"[RUN] navmesh 베이크 시간 초과 ({timeout_s:.0f}s)", flush=True)
        return False

    if inav.get_navmesh() is None:
        print("[RUN] navmesh 베이크 실패 (get_navmesh() 가 None). NavMeshVolume 범위를 확인할 것", flush=True)
        return False
    print("[RUN] navmesh 베이크 완료", flush=True)
    return True


bake_navmesh()

pos, quat = XFormPrim(BASE_LINK_PRIM).get_world_poses()
x, y = float(pos[0][0]), float(pos[0][1])
w, qx, qy, qz = (float(v) for v in quat[0])
yaw_deg = math.degrees(math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
with open(START_POSE_PATH, "w") as f:
    json.dump({"frame": "map", "x": x, "y": y, "yaw_deg": yaw_deg}, f, indent=2)
print(f"[RUN] start pose -> {START_POSE_PATH}: x={x:.3f} y={y:.3f} yaw={yaw_deg:.1f}deg", flush=True)

omni.timeline.get_timeline_interface().play()
print(f"\n[RUN] {USD_PATH} 재생 시작 — 사람 경로: {PEOPLE_COMMAND_FILE}, ROS2 토픽 발행 중 (Ctrl+C 로 종료)\n", flush=True)

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
