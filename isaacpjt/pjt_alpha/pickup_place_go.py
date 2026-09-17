"""
통합 시나리오 — 잡고 / 이동하고 / 놓는다

    isaac_python pickup_place_go.py

    1 PICK   pick_and_place.py 의 POINT 순서를 '역순'으로 실행해 물건을 잡는다
             (마지막 그리퍼 열기는 빼서 물건을 든 채로 끝난다)
    2 MOVE   move_integration.py 처럼 W0..W3 waypoint 를 따라 이동한다
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

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import is_stage_loading, open_stage

from waypoint_mover import WaypointMover
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


USD_PATH = Path("/home/rokey/cobot3_ws/isaacpjt/assets/integration.usd")
MOVE_ROOT_PATH = "/World/robot"

WAYPOINTS = [
    np.array([0.0, 0.0, 0.0]),        # W0
    np.array([6.72354, 0.0, 0.0]),    # W1
    np.array([6.72354, 15.5, 0.0]),   # W2
    np.array([0.0, 15.5, 0.0]),       # W3
]

SPEED_MPS = 1.0
ACCEL_MPS2 = 1.0        # 주행 가감속
PICK_PAUSE_S = 1.0      # 홈 복귀 후 주행을 시작하기까지 대기 시간


def build_pick_sequence(base_pos, base_quat):
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
        {"type": "pose",  "label": "놓는 위치",      "target": p6, "gripper": None},
        {"type": "hold",  "label": "그리퍼 열기",    "gripper": "open"},
        {"type": "pose",  "label": "후퇴 안전 위치", "target": p1, "gripper": None},
        {"type": "joint", "label": "홈 복귀",        "target": np.array(READY_JOINTS_RAD, dtype=float), "gripper": None},
    ]


def main():
    if not USD_PATH.is_file():
        raise FileNotFoundError(f"USD file was not found: {USD_PATH}")
    if not open_stage(str(USD_PATH)):
        raise RuntimeError(f"Could not open USD: {USD_PATH}")
    while is_stage_loading():
        simulation_app.update()

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

    mover = WaypointMover(
        stage=omni.usd.get_context().get_stage(),
        prim_path=MOVE_ROOT_PATH,
        waypoints=WAYPOINTS,
        speed_mps=SPEED_MPS,
        accel_mps2=ACCEL_MPS2,
    )

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
            mover.reset_to_start()
            base_pos, base_quat = sync_base_pose(lula, robot)
            print(f"   base pos     {vec(base_pos)}")
            seq = PickPlaceSequence(robot, ik_solver, arm_indices,
                                    build_pick_sequence(base_pos, base_quat))
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
                phase = "MOVE"
                print("[PHASE] MOVE")

        elif phase == "MOVE":
            # 홈 자세로 물건 없이 주행하므로 팔은 마지막 drive target 그대로 둔다
            if mover.step(world.get_physics_dt()):
                base_pos, base_quat = sync_base_pose(lula, robot)
                print(f"   base pos     {vec(base_pos)}")
                seq = PickPlaceSequence(robot, ik_solver, arm_indices,
                                        build_sequence(base_pos, base_quat))
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
