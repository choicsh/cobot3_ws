"""
통합 시나리오 — 잡고 / 이동하고 / 놓는다

    isaac_python pickup_place_go.py

    1 PICK   pick_and_place.py 의 POINT 순서를 '역순'으로 실행해 물건을 잡는다
             (마지막 그리퍼 열기는 빼서 물건을 든 채로 끝난다)
    2 MOVE   노바카터 바퀴로 W1..W3 waypoint 를 따라 실제 주행한다
             (순간이동 없음. 트레이는 랙 위에 마찰로 실려 간다)
    3 PLACE  POINT 순서를 그대로 실행해 물건을 놓는다
             (물건을 들고 왔으므로 첫 스텝에서 그리퍼를 열지 않는다)

동작 코드는 새로 쓰지 않고 기존 두 파일을 그대로 import 해서 쓴다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys
from pathlib import Path

import numpy as np
import isaacsim

# pick_and_place / move 모듈도 자기 SimulationApp 을 만들려 하므로 위에서 만든 것을 돌려준다
isaacsim.SimulationApp = lambda *args, **kwargs: simulation_app

M0609_DIR = Path(__file__).resolve().parent.parent / "M0609"
sys.path[:0] = [str(M0609_DIR / "move"), str(M0609_DIR / "teleop")]

# 씬의 ActionGraph 가 /clock, TF, /chassis/odom, 라이다를 내보내려면 브리지가 켜져 있어야 한다.
# 켜지 않으면 Nav2 가 센서 데이터를 전혀 못 받는다. 스테이지를 열기 전에 활성화할 것.
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.usd

from isaacsim.core.api import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from pick_and_place import (
    ARM_JOINTS,
    JOINT1_ROTATE_DEG,
    POINT1_TCP, POINT1_RPY,
    POINT2_TCP, POINT2_RPY,
    POINT3_TCP, POINT3_RPY,
    POINT4_TCP, POINT4_RPY,
    POINT5_TCP, POINT5_RPY,
    POINT6_TCP, POINT6_RPY,
    READY_JOINTS_RAD,
    PickPlaceSequence,
    base_to_world,
    build_sequence,
    create_ik_solver,
    init_robot,
    register_robot,
    section,
    setup_arm_drives,
    setup_gripper_drive,
    sync_base_pose,
    vec,
)


USD_PATH = Path("/home/rokey/cobot3_ws/isaacpjt/assets/intergration_nova.usd")
MOVE_ROOT_PATH = "/World/robot_nova"

# 카터와 팔이 하나의 아티큘레이션이다.
#   - 드라이브 설정 / EE 검색은 팔이 들어 있는 nova_carter 서브트리 기준
#   - Articulation 등록은 실제 루트인 chassis_link 기준 (바퀴로 주행하려면 루트가 리지드 바디여야 한다)
import pick_and_place as _pnp
_pnp.ROBOT_PRIM_PATH = MOVE_ROOT_PATH + "/nova_carter"
ART_ROOT_PATH = _pnp.ROBOT_PRIM_PATH + "/chassis_link"
ARM_BASE_PATH = _pnp.ROBOT_PRIM_PATH + "/Robot/m0609_camera/m0609/base_link"

# 책상을 카트에서 월드 -x 로 10cm 물렸으므로(간격 -1cm -> +9cm) 책상 쪽 파지점도 그만큼 옮긴다.
# 팔 base_link 는 월드 대비 z 180deg 회전이라 '월드 -x' = 'base +x' 이다.
# P4/P5 만 책상 쪽 점이고 P1/P3/P6 은 카트 랙 쪽이라 건드리지 않는다.
DESK_BACKOFF_M = 0.10
POINT4_TCP = _pnp.POINT4_TCP = POINT4_TCP + np.array([DESK_BACKOFF_M, 0.0, 0.0])
POINT5_TCP = _pnp.POINT5_TCP = POINT5_TCP + np.array([DESK_BACKOFF_M, 0.0, 0.0])


def register_robot(world):
    """pick_and_place.register_robot 과 같지만 prim_path 만 아티큘레이션 루트로 바꾼다"""
    from isaacsim.robot.manipulators import SingleManipulator
    from isaacsim.robot.manipulators.grippers import ParallelGripper

    ee_path = _pnp.find_all_named(_pnp.EE_LINK_NAME, _pnp.ROBOT_PRIM_PATH)[0]
    gripper = ParallelGripper(
        end_effector_prim_path=ee_path,
        joint_prim_names=_pnp.GRIPPER_JOINTS,
        joint_opened_positions=np.array([_pnp.GRIPPER_OPEN_POS] * 2),
        joint_closed_positions=np.array([_pnp.GRIPPER_CLOSE_POS] * 2),
        action_deltas=None,
    )
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ART_ROOT_PATH,
            name="m0609_robot",
            end_effector_prim_path=ee_path,
            gripper=gripper,
        )
    )
    print(f"   EE frame     {ee_path}")
    return robot


def init_robot(robot, world):
    """카터 관절까지 한 아티큘레이션에 섞여 있어 홈 자세는 dof 이름으로 찾아 넣는다"""
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    q = robot.get_joint_positions()
    for name, angle in zip(ARM_JOINTS, READY_JOINTS_RAD):
        q[robot.get_dof_index(name)] = angle
    robot.set_joint_positions(q)


def sync_base_pose(lula, robot):
    """IK 기준은 아티큘레이션 루트(카터)가 아니라 팔의 base_link 다"""
    base_pos, base_quat = SingleXFormPrim(ARM_BASE_PATH).get_world_pose()
    lula.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
    return base_pos, base_quat


# ---------------------------------------------------------------- 미션 연계
# 주행은 nav_mission.py(프로세스 B)가 담당한다.
# Isaac Python 은 3.11 인데 Jazzy rclpy 는 3.12 전용이라 한 프로세스에 못 들어간다.
# 그래서 이 파일은 팔(PICK/PLACE)만 맡고, 주행 구간은 파일 신호로 주고받는다.
#
#   pick_done  이 파일이 생성 : 트레이를 랙에 실었다. 주행 시작해도 된다
#   nav_done   B가 생성       : 마지막 지점 도착. 내려놓기 시작해도 된다
HANDSHAKE_DIR = Path("/tmp/cobot3_mission")
PICK_DONE = HANDSHAKE_DIR / "pick_done"
NAV_DONE = HANDSHAKE_DIR / "nav_done"

APPROACH_BACKOFF_M = 0.082   # P1->P2 와 같은 수평 접근 거리
RACK_SETTLE_S = 0.5          # 랙 위(P3)에서 내려놓기(P6) 전 제자리 대기
PICK_PAUSE_S = 1.0           # 홈 복귀 후 신호를 내보내기까지 대기 시간


def build_pick_sequence(base_pos, base_quat, settle_steps):
    """놓는 순서(build_sequence)의 역순. 놓고 -> 그리퍼 열고 -> 홈 복귀로 끝난다"""
    p1, p2, p3, p4, p5, p6 = (
        base_to_world(tcp, rpy, base_pos, base_quat)
        for tcp, rpy in (
            (POINT1_TCP, POINT1_RPY),
            (POINT2_TCP, POINT2_RPY),
            (POINT3_TCP, POINT3_RPY),
            (POINT4_TCP, POINT4_RPY),
            (POINT5_TCP, POINT5_RPY),
            (POINT6_TCP, POINT6_RPY),
        )
    )
    # joint_1 을 돌리기 전에 P3 와 같은 높이까지 올려야 주변에 부딪히지 않는다
    lift_tcp = np.array([POINT4_TCP[0], POINT4_TCP[1], POINT3_TCP[2]])
    lift = base_to_world(lift_tcp, POINT4_RPY, base_pos, base_quat)

    joint1_delta = np.zeros(6)
    joint1_delta[0] = np.radians(JOINT1_ROTATE_DEG)

    return [
        {"type": "joint", "label": "joint_1 선회전", "target": lambda start: start + joint1_delta, "gripper": "open"},
        {"type": "pose",  "label": "접근 안전 위치", "target": p5, "gripper": None},
        {"type": "pose",  "label": "파지 위치",      "target": p4, "gripper": None},
        {"type": "hold",  "label": "그리퍼 닫기",    "gripper": "close"},
        {"type": "pose",  "label": "들어올리기",     "target": lift, "gripper": None},
        {"type": "joint", "label": "joint_1 역회전", "target": lambda start: start - joint1_delta, "gripper": None},
        {"type": "pose",  "label": "P3 정렬",        "target": p3, "gripper": None},
        # 랙 위에 도착한 자세가 흔들린 채로 곧장 내려가면 트레이가 랙 턱에 걸린다
        {"type": "hold",  "label": f"대기 {RACK_SETTLE_S}s", "gripper": None, "steps": settle_steps},
        {"type": "pose",  "label": "놓는 위치",      "target": p6, "gripper": None},
        {"type": "hold",  "label": "그리퍼 열기",    "gripper": "open"},
        {"type": "pose",  "label": "후퇴 안전 위치", "target": p1, "gripper": None},
        {"type": "joint", "label": "홈 복귀",        "target": np.array(READY_JOINTS_RAD, dtype=float), "gripper": None},
    ]


def build_place_sequence(base_pos, base_quat):
    """놓는 순서 그대로. 단 접근/파지 지점은 1단계에서 실제로 내려놓은 P6 로 맞춘다"""
    steps = build_sequence(base_pos, base_quat)
    # P1->P2, P5->P4 와 마찬가지로 같은 높이에서 수평으로 들어가 잡는다.
    # 위에서 수직으로 내려오면 손가락이 트레이 윗면을 건드린다.
    approach_tcp = POINT6_TCP - np.array([0.0, APPROACH_BACKOFF_M, 0.0])
    p6_approach = base_to_world(approach_tcp, POINT6_RPY, base_pos, base_quat)
    p6 = base_to_world(POINT6_TCP, POINT6_RPY, base_pos, base_quat)
    steps[0] = {"type": "pose", "label": "접근 위치(P6 앞)", "target": p6_approach, "gripper": "open"}
    steps[1] = {"type": "pose", "label": "파지 위치(P6)", "target": p6, "gripper": None}
    # 드라이브가 한 스텝 안에 목표를 못 따라와 몇 cm 못 미친 채로 그리퍼가 닫힌다.
    # 같은 목표를 한 번 더 줘서 남은 오차를 마저 밀어넣는다
    steps.insert(2, {"type": "pose", "label": "파지 위치(P6) 정착", "target": p6, "gripper": None})
    return steps


def main():
    if not USD_PATH.is_file():
        raise FileNotFoundError(f"USD file was not found: {USD_PATH}")
    if not open_stage(str(USD_PATH)):
        raise RuntimeError(f"Could not open USD: {USD_PATH}")
    while is_stage_loading():
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()

    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    print(f"   scene        {USD_PATH.name}")
    setup_arm_drives()
    setup_gripper_drive()
    robot = register_robot(world)

    world.reset()
    init_robot(robot, world)

    for _ in range(30):
        world.step(render=True)

    section("SOLVER")
    lula, ik_solver = create_ik_solver(robot)
    arm_indices = [robot.get_dof_index(j) for j in ARM_JOINTS]

    section("RUN")
    print("   뷰포트를 클릭해 포커스를 준 뒤 Play 를 누른다\n")

    phase = None
    seq = None
    wait_ticks = 0
    was_playing = False

    while simulation_app.is_running():
        world.step(render=True)

        is_playing = world.is_playing()
        if is_playing and not was_playing:
            init_robot(robot, world)
            HANDSHAKE_DIR.mkdir(parents=True, exist_ok=True)
            for f in (PICK_DONE, NAV_DONE):
                if f.exists():
                    f.unlink()
            base_pos, base_quat = sync_base_pose(lula, robot)
            print(f"   base pos     {vec(base_pos)}")
            settle_steps = max(1, int(RACK_SETTLE_S / world.get_physics_dt()))
            seq = PickPlaceSequence(robot, ik_solver, arm_indices,
                                    build_pick_sequence(base_pos, base_quat, settle_steps))
            phase = "PICK"
            print("[PHASE] PICK")
        was_playing = is_playing

        if not is_playing or phase is None:
            continue

        if phase == "PICK":
            seq.tick()
            if seq.done:
                wait_ticks = int(PICK_PAUSE_S / world.get_physics_dt())
                phase = "WAIT"
                print(f"[PHASE] WAIT {PICK_PAUSE_S}s")

        elif phase == "WAIT":
            wait_ticks -= 1
            if wait_ticks <= 0:
                PICK_DONE.write_text("ok\n")
                phase = "MOVE"
                print(f"[PHASE] MOVE  (신호 {PICK_DONE} 생성, nav_mission.py 대기)")

        elif phase == "MOVE":
            # 주행은 nav_mission.py 가 한다. 팔은 홈 자세 그대로 두고
            # 트레이는 랙 위에 마찰로 실려 간다. 여기서는 도착 신호만 기다린다
            if NAV_DONE.exists():
                base_pos, base_quat = sync_base_pose(lula, robot)
                print(f"   base pos     {vec(base_pos)}")
                seq = PickPlaceSequence(robot, ik_solver, arm_indices,
                                        build_place_sequence(base_pos, base_quat))
                phase = "PLACE"
                print("[PHASE] PLACE")

        elif phase == "PLACE":
            seq.tick()
            if seq.done:
                phase = "DONE"
                print("[PHASE] DONE")


try:
    main()
finally:
    simulation_app.close()
