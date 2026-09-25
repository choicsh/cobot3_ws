"""병원 씬 + 로봇 N대(최대 3) + 보행자 3명.

    export ROS_DOMAIN_ID=136
    ~/isaacsim/python.sh isaacpjt/system/run_fleet_sim.py --robots 2
    ~/isaacsim/python.sh isaacpjt/system/run_fleet_sim.py --robots 3 --no-arm --headless

로봇 i 의 ROS 토픽은 /robot{i}/ 아래다 (scene.py 참고). 로봇 1 은 씬 원래 위치(East 책상 옆),
2·3 은 운송 차선(lane_lower) 남쪽 직선 위에 서쪽을 보고 놓는다. 시작 자세는
assets/fleet_start_poses.json 에 기록한다 (AMCL 초기 위치 입력용).

팔 (arm/controller.py) — 로봇마다 ArmTaskController. 명령/상태는 OmniGraph 로 주고받는다:
    ros2 topic pub --once /robot1/arm/command std_msgs/String "{data: 'load:1'}"
    ros2 topic echo /robot1/arm/status
    검출은 로봇마다 tray_detector 를 네임스페이스로 띄운다 (admin_ws/README.md).
    GUI 에서는 뷰포트를 클릭한 뒤 L(적재) / U(하역) 키로 robot1 에 명령할 수도 있다.
책상 트레이(/World/tray)는 시작 시 2개를 더 복제해 가로 한 줄로 놓는다 (원본 P&P 와 같다).
"""

import argparse
import json
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--robots", type=int, default=2, choices=(1, 2, 3))
parser.add_argument("--headless", action="store_true")
parser.add_argument("--no-people", action="store_true")
parser.add_argument("--no-arm", action="store_true", help="팔 컨트롤러/트레이 복제 없이 주행 로봇만")
parser.add_argument("--no-wrist", action="store_true", help="손목 카메라 그래프를 만들지 않음 (--no-arm 일 때만)")
parser.add_argument("--front-cam", action="store_true",
                    help="전방 스테레오 카메라 발행 (병원 주행은 안 씀. 켜면 3대에서 실시간 비율 0.67->0.60)")
parser.add_argument("--walk-blend", type=float, default=0.75)
parser.add_argument("--pose", action="append", default=[], metavar="I:X,Y,YAW",
                    help="로봇 I 의 시작 자세, 예: --pose 2:20.2,17.5,180 또는 도킹 자세 --pose 1:collection")
parser.add_argument("--preload-rack", action="append", type=int, default=[], metavar="I",
                    help="로봇 I 의 랙 3칸에 트레이를 미리 싣는다 (하역 단독 시험용)")
args = parser.parse_args()
if not args.no_arm and args.no_wrist:
    parser.error("팔은 손목 카메라 검출이 필요하다 — --no-wrist 는 --no-arm 과 같이 쓸 것")

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
import carb.input  # noqa: E402
import omni.appwindow  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402

# run_hospital_sim.py 와 같게: GPU 가 남아도 실시간보다 빨리 돌지 않게 30 Hz 로 묶는다
settings = carb.settings.get_settings()
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
settings.set_int("/app/runLoops/main/rateLimitFrequency", 30)
scene.configure_people(args.walk_blend)
simulation_app.update()

# 로봇 1 은 None = 씬 USD 자세 (20.27, 13.746, 90) — P&P 를 맞춘 자세다. 실제 도킹 자세와 다르다(아래).
# 2·3 은 운송 차선(lane_lower) 남쪽 직선 위 빈 자리.
ROBOT_POSES = [
    None,
    (-5.0, -1.5, 180.0),
    (-20.0, -1.5, 180.0),
]
# 책상 도킹이 끝났을 때의 base_link 자세 — nav_to_goal/hospital_docking.TABLES 의 dock 과 같은 값
# (책상 긴 변이 로봇 우측, 간격 0.15 m). 로봇 프림 원점 = base_link 이다.
# 씬 USD 자세보다 책상에서 8.5 cm 멀고 6.7 cm 북쪽이다.
NAMED_POSES = {
    "collection": (20.185, 13.8125, 90.0),     # East_DockDesk, 채취실 (적재)
    "analysis": (-44.741, 12.9125, -90.0),     # West_DockDesk, 분석실 (하역)
}

for item in args.pose:
    index, values = item.split(":")
    ROBOT_POSES[int(index) - 1] = (NAMED_POSES[values] if values in NAMED_POSES
                                   else tuple(float(v) for v in values.split(",")))

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

arms = []
if not args.no_arm:
    from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
    from arm.config import ARM_BASE_REL  # noqa: E402
    from arm.controller import ArmTaskController  # noqa: E402
    from arm.rosio import ArmRosIO  # noqa: E402
    from arm.trays import TrayRegistry  # noqa: E402

    trays = TrayRegistry()
    # 트레이 책상(East, 채취실)에 도킹해 있는 로봇 1 의 팔 base 기준으로 복제한다
    trays.spawn_copies(*SingleXFormPrim(f"{scene.robot_prim(1)}/{ARM_BASE_REL}").get_world_pose(),
                       simulation_app.update)
    for index in args.preload_rack:
        trays.preload_rack(index, *SingleXFormPrim(f"{scene.robot_prim(index)}/{ARM_BASE_REL}").get_world_pose(),
                           simulation_app.update)
    for index in range(1, args.robots + 1):
        io = ArmRosIO(index, simulation_app.update)
        io.build()
        arm = ArmTaskController(index, scene.robot_prim(index), io, trays)
        arm.setup_drives(stage)
        arm.register(world)
        arms.append(arm)

scene.bake_navmesh(simulation_app)
world.reset()
settings.set_bool("/app/scripting/ignoreWarningDialog", previous_prompt)
for arm in arms:
    arm.post_reset(world)
    # 물리 스텝(60 Hz)마다 1틱 — 원본 P&P 와 같은 시뮬 시간 기준
    world.add_physics_callback(f"arm_robot{arm.index}", lambda _dt, a=arm: a.tick())

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


class KeyTap:
    """뷰포트에 포커스가 있을 때의 키 PRESS 를 모아 둔다 (GUI 수동 시험용)."""

    def __init__(self):
        self._taps = []
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._input = carb.input.acquire_input_interface()
        self._sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_event)

    def _on_event(self, event, *_):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._taps.append(event.input.name)
        return True

    def take(self):
        taps, self._taps = self._taps, []
        return taps


keys = KeyTap() if arms and not args.headless else None
was_playing = False
try:
    while simulation_app.is_running():
        world.step(render=True)
        playing = world.is_playing()
        if playing and not was_playing:
            for arm in arms:
                arm.on_play(world)
        elif was_playing and not playing:
            for arm in arms:
                arm.on_stop()
        was_playing = playing
        if keys is not None and playing:
            for key in keys.take():
                if key in ("L", "U"):
                    arms[0].command("load" if key == "L" else "unload", f"key {key}")
finally:
    simulation_app.close()
