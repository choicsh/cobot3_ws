"""병원 씬 + 로봇 N대(최대 3) + 보행자 3명.

    export ROS_DOMAIN_ID=136
    ~/isaacsim/python.sh isaacpjt/system/run_fleet_sim.py --robots 2
    ~/isaacsim/python.sh isaacpjt/system/run_fleet_sim.py --robots 3 --no-wrist --headless

로봇 i 의 ROS 토픽은 /robot{i}/ 아래다 (scene.py 참고). 로봇 1 은 씬 원래 위치(East 책상 옆),
2·3 은 운송 차선(lane_lower) 남쪽 직선 위에 서쪽을 보고 놓는다. 시작 자세는
assets/fleet_start_poses.json 에 기록한다 (AMCL 초기 위치 입력용).
"""

import argparse
import json
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--robots", type=int, default=2, choices=(1, 2, 3))
parser.add_argument("--headless", action="store_true")
parser.add_argument("--no-people", action="store_true")
parser.add_argument("--no-wrist", action="store_true", help="손목 카메라 발행 안 함")
parser.add_argument("--front-cam", action="store_true",
                    help="전방 스테레오 카메라 발행 (병원 주행은 안 씀. 켜면 3대에서 실시간 비율 0.67->0.60)")
parser.add_argument("--walk-blend", type=float, default=0.75)
parser.add_argument("--pose", action="append", default=[], metavar="I:X,Y,YAW",
                    help="로봇 I(2 이상)의 시작 자세 덮어쓰기, 예: --pose 2:20.2,17.5,180")
args = parser.parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "width": 1280,
    "height": 720,
    "anti_aliasing": 2,
})

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scene  # noqa: E402  (SimulationApp 이후)

scene.enable_people_extensions(simulation_app)
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import carb  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402

# run_hospital_sim.py 와 같게: GPU 가 남아도 실시간보다 빨리 돌지 않게 30 Hz 로 묶는다
settings = carb.settings.get_settings()
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
settings.set_int("/app/runLoops/main/rateLimitFrequency", 30)
scene.configure_people(args.walk_blend)
simulation_app.update()

# 차선 위 빈 자리. 로봇 1 값은 쓰지 않는다(씬에 있는 자세 그대로).
ROBOT_POSES = [
    (20.27, 13.745, 90.0),     # robot1: East 책상(채취실) 도킹 자세
    (-5.0, -1.5, 180.0),       # robot2: lane_lower 남쪽 직선
    (-20.0, -1.5, 180.0),      # robot3: lane_lower 남쪽 직선
]

for item in args.pose:
    index, values = item.split(":")
    ROBOT_POSES[int(index) - 1] = tuple(float(v) for v in values.split(","))

behavior_paths, behavior_script = scene.validated_behavior_prims()
previous_prompt = settings.get_as_bool("/app/scripting/ignoreWarningDialog")
settings.set_bool("/app/scripting/ignoreWarningDialog", True)

open_stage(scene.SCENE_USD)
while is_stage_loading():
    simulation_app.update()
if scene.set_viewport_lighting("Default"):
    print("[FLEET] viewport lighting: Default", flush=True)

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0)
stage = world.stage
scene.setup_people(stage, behavior_paths, behavior_script, enabled=not args.no_people)
scene.add_robots(stage, ROBOT_POSES[:args.robots], front_camera=args.front_cam)
for _ in range(5):
    simulation_app.update()
if not args.no_wrist:
    for index in range(1, args.robots + 1):
        scene.build_wrist_camera_graph(stage, index)

scene.bake_navmesh(simulation_app)
world.reset()
settings.set_bool("/app/scripting/ignoreWarningDialog", previous_prompt)

poses = {}
for index in range(1, args.robots + 1):
    x, y, yaw = scene.base_link_pose(index)
    poses[scene.namespace(index)] = {"frame": "map", "x": x, "y": y, "yaw_deg": yaw}
    print(f"[FLEET] /{scene.namespace(index)} base_link x={x:.3f} y={y:.3f} yaw={yaw:.1f}", flush=True)
with open(scene.ASSETS_DIR / "fleet_start_poses.json", "w", encoding="utf-8") as f:
    json.dump(poses, f, indent=2)

omni.timeline.get_timeline_interface().play()
print(f"[FLEET] 재생 시작 — 로봇 {args.robots}대, 사람 {'없음' if args.no_people else '3명'}, "
      f"손목 카메라 {'끔' if args.no_wrist else '켬'}, 전방 카메라 {'켬' if args.front_cam else '끔'}",
      flush=True)

try:
    while simulation_app.is_running():
        world.step(render=True)
finally:
    simulation_app.close()
