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
from pxr import Gf, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim, SingleXFormPrim
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


USD_PATH = Path("/home/rokey/cobot3_ws/isaacpjt/assets/intergration_nova.usd")
MOVE_ROOT_PATH = "/World/robot_nova"

# nova 씬은 카터와 팔이 하나의 아티큘레이션(root = nova_carter)이라 그 경로를 쓴다
import pick_and_place as _pnp
_pnp.ROBOT_PRIM_PATH = MOVE_ROOT_PATH + "/nova_carter"
ARM_BASE_PATH = _pnp.ROBOT_PRIM_PATH + "/Robot/m0609_camera/m0609/base_link"


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

# 주행 중 짐을 물리적으로 끌고 가기 위한 프림들
PAYLOAD_PRIM_PATH = "/World/tray/tray"          # 옮기는 물건 (유일한 dynamic rigid body)
CARRY_ANCHOR_PATH = "/World/carry_anchor"
CARRY_JOINT_PATH = "/World/carry_joint"

WAYPOINTS = [
    np.array([1.08010, 0.55106, 0.0]),        # W0  (씬에 배치된 카트 위치 = 팔이 트레이를 잡는 자리)
    np.array([7.7, 0.0, 0.0]),    # W1
    np.array([7.7, 15.5, 0.0]),   # W2
    np.array([1.08010, 15.5, 0.0]),       # W3
]

SPEED_MPS = 1.0
ACCEL_MPS2 = 1.0        # 주행 가감속
PICK_PAUSE_S = 1.0      # 홈 복귀 후 주행을 시작하기까지 대기 시간


def clear_carry(stage):
    for path in (CARRY_JOINT_PATH, CARRY_ANCHOR_PATH):
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)


class PayloadCarry:
    """주행 구간 동안만 짐을 kinematic anchor 에 fixed joint 로 묶어 물리적으로 끌고 간다.

    anchor 를 짐과 똑같은 월드 포즈의 스케일 없는 Xform 으로 만들기 때문에
    joint local frame 이 양쪽 다 0 이고, 그래서 'disjointed body transforms' 경고가 나지 않는다."""

    def __init__(self, stage, payload, mover):
        clear_carry(stage)
        self._stage = stage
        self._mover = mover
        self._payload_start, payload_quat = payload.get_world_pose()
        self._robot_start = mover.get_world_position()

        anchor = UsdGeom.Xform.Define(stage, CARRY_ANCHOR_PATH)
        anchor.ClearXformOpOrder()
        self._translate_op = anchor.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        self._translate_op.Set(Gf.Vec3d(*[float(v) for v in self._payload_start]))
        anchor.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(*[float(v) for v in payload_quat])
        )
        UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim()).CreateKinematicEnabledAttr(True)

        joint = UsdPhysics.FixedJoint.Define(stage, CARRY_JOINT_PATH)
        joint.CreateBody0Rel().SetTargets([CARRY_ANCHOR_PATH])
        joint.CreateBody1Rel().SetTargets([PAYLOAD_PRIM_PATH])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    def update(self):
        # ponytail: waypoint 가 평행이동뿐이라 translate 만 따라간다. 주행 중 회전이 생기면 orient 도 갱신해야 한다
        delta = self._mover.get_world_position() - self._robot_start
        self._translate_op.Set(Gf.Vec3d(*[float(v) for v in (self._payload_start + delta)]))

    def release(self):
        clear_carry(self._stage)


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


def build_place_sequence(base_pos, base_quat):
    """놓는 순서 그대로. 단 접근/파지 지점은 1단계에서 실제로 내려놓은 P6 로 맞춘다"""
    steps = build_sequence(base_pos, base_quat)
    # PICK 때와 똑같이 P3(트레이 바로 위)에서 수직으로 내려와야 손가락이 트레이를 빗맞지 않는다
    steps[0] = {
        "type": "pose",
        "label": "P3 정렬(접근)",
        "target": base_to_world(POINT3_TCP, POINT3_RPY, base_pos, base_quat),
        "gripper": "open",
    }
    p6 = base_to_world(POINT6_TCP, POINT6_RPY, base_pos, base_quat)
    steps[1] = {"type": "pose", "label": "파지 위치(P6)", "target": p6, "gripper": None}
    # 드라이브가 목표를 따라오는 데 한 스텝으로는 모자라 6cm 쯤 뜬 채로 그리퍼가 닫힌다.
    # 같은 목표를 한 번 더 주면 남은 오차만큼만 더 내려가 트레이를 제대로 문다
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
    clear_carry(stage)

    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    print(f"   scene        {USD_PATH.name}")
    setup_arm_drives()
    setup_gripper_drive()
    robot = register_robot(world)

    world.reset()
    init_robot(robot, world)

    payload = SingleRigidPrim(PAYLOAD_PRIM_PATH, name="payload")
    payload.initialize()

    for _ in range(30):
        world.step(render=True)

    section("SOLVER")
    lula, ik_solver = create_ik_solver(robot)
    arm_indices = [robot.get_dof_index(j) for j in ARM_JOINTS]

    mover = WaypointMover(
        stage=stage,
        prim_path=MOVE_ROOT_PATH,
        waypoints=WAYPOINTS,
        speed_mps=SPEED_MPS,
        accel_mps2=ACCEL_MPS2,
    )

    section("RUN")
    print("   뷰포트를 클릭해 포커스를 준 뒤 Play 를 누른다\n")

    phase = None
    seq = None
    carry = None
    wait_ticks = 0
    was_playing = False

    while simulation_app.is_running():
        world.step(render=True)

        is_playing = world.is_playing()
        if is_playing and not was_playing:
            init_robot(robot, world)
            clear_carry(stage)
            carry = None
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
                carry = PayloadCarry(stage, payload, mover)
                phase = "MOVE"
                print("[PHASE] MOVE")

        elif phase == "MOVE":
            # 홈 자세로 주행하므로 팔은 마지막 drive target 그대로 두고, 짐만 anchor 로 끌고 간다
            arrived = mover.step(world.get_physics_dt())
            carry.update()
            if arrived:
                carry.release()
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
