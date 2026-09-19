"""
Pick & Place — 정해진 좌표를 순서대로 실행하는 자동 시퀀스

    isaac_python pick_and_place.py

pnp_teleop.py 로 손으로 찾은 좌표(POINT1~5, base 기준)를 그대로 실행한다.
Play 를 누르면 키 입력 없이 아래 순서를 자동으로 수행한다.

    1 안전 위치(파지 전)로 이동
    2 파지 위치로 이동
    3 그리퍼 닫기
    4 들어올리는 위치로 이동
    5 joint_1 을 -90도 회전
    6 놓는 위치로 이동
    7 그리퍼 열기
    8 후퇴 안전 위치로 이동
    9 홈 관절 각도로 복귀
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import carb
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)


# ══════════════════════════════════════════════════════════════
#  경로 / 로봇 설정 — pnp_teleop.py 와 동일
# ══════════════════════════════════════════════════════════════
THIS_DIR   = Path(__file__).resolve().parent
M0609_DIR  = THIS_DIR.parent

SCENE_USD        = str(M0609_DIR.parent / "assets/PnP_test.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

ROBOT_PRIM_PATH = "/World/robot/Robot/m0609_camera/m0609"
EE_LINK_NAME    = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 3e5
DRIVE_DAMPING   = 3e4
DRIVE_MAX_FORCE = 300.0

GRIPPER_JOINTS    = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 1.09

GRIPPER_DRIVE_STIFFNESS = 1e6
GRIPPER_DRIVE_DAMPING   = 1e3
GRIPPER_DRIVE_MAX_FORCE = 15.0

# link_6 로컬 +Z 기준 손가락 패드 끝까지의 거리 (실측)
TCP_OFFSET = np.array([0.0, 0.0, 0.21671])

READY_JOINTS_RAD = [1.57, 0.0, 2.157, 0.0, -0.6, 1.57]

IK_POSITION_TOLERANCE    = 0.003    # m
IK_ORIENTATION_TOLERANCE = 0.02     # rad


# ══════════════════════════════════════════════════════════════
#  Pick & Place 목표 좌표 — pnp_teleop.py 로 손으로 찾은 값 (base 기준)
# ══════════════════════════════════════════════════════════════
POINT1_TCP = np.array([-0.0093, 0.7190, 0.2776])   # 잡기 전 안전 위치
POINT1_RPY = (-89.3, 0.1, 180.0)

POINT2_TCP = np.array([-0.0091, 0.8007, 0.2776])   # 파지 위치
POINT2_RPY = (-89.3, 0.1, 180.0)

POINT3_TCP = np.array([-0.0088, 0.7989, 0.4496])   # 들어올린 위치
POINT3_RPY = (-89.3, 0.1, 180.0)

POINT4_TCP = np.array([0.7734, 0.0513, 0.1102])    # 놓는 위치
POINT4_RPY = (-89.3, 89.1, 180.0)

POINT5_TCP = np.array([0.6514, 0.0496, 0.1102])    # 놓고 후퇴하는 안전 위치
POINT5_RPY = (-89.3, 89.1, 180.0)

JOINT1_ROTATE_DEG = -90.0   # 들어올린 뒤 joint_1 을 이만큼 상대 회전한다

POINT6_TCP = np.array([-0.0093, 0.7690, 0.24])   # 처음에 내려놓는 위치 (base +y 로 50mm 더 안쪽)
POINT6_RPY = (-89.3, 0.1, 180.0)

# 보간 속도 — 스텝당 이동량을 고정하고 구간 길이로 스텝 수를 정한다
TCP_SPEED_M        = 0.004   # m / step
JOINT_SPEED_DEG     = 0.5    # deg / step
MIN_STEPS           = 60
MAX_STEPS           = 600
GRIPPER_WAIT_STEPS  = 120    # 그리퍼가 실제로 여닫힐 때까지 제자리에서 기다리는 스텝 수
LOG_INTERVAL        = 60


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — pnp_teleop.py 와 동일 (base<->world 변환용으로 필요한 것만)
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    """쿼터니언 곱. 순서는 (w, x, y, z)"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, deg):
    """회전축과 각도(도)로 쿼터니언을 만든다"""
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    """roll, pitch 로 접근 방향을 정한 뒤 yaw 를 마지막에 곱해 툴축을 돌린다"""
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """쿼터니언 -> 회전행렬. 1열이 로컬 +X, 3열이 로컬 +Z"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def matrix_to_rpy(m):
    """회전행렬 -> (roll, pitch, yaw) 도. make_target_quat 의 합성 순서에 맞춘 역변환"""
    pitch = np.degrees(np.arcsin(np.clip(m[0, 2], -1.0, 1.0)))
    roll  = np.degrees(np.arctan2(-m[1, 2], m[2, 2]))
    yaw   = np.degrees(np.arctan2(-m[0, 1], m[0, 0]))
    return roll, pitch, yaw


def quat_slerp(q0, q1, t):
    """단위 쿼터니언 구면 선형보간"""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    dot = np.clip(dot, -1.0, 1.0)
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta = np.arccos(dot) * t
    ortho = q1 - q0 * dot
    ortho = ortho / np.linalg.norm(ortho)
    return q0 * np.cos(theta) + ortho * np.sin(theta)


def tcp_to_flange(tcp_pos, quat):
    """손가락 끝 목표를 플랜지(link_6) 목표로 바꾼다"""
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def base_to_world(base_pos, base_rpy, robot_pos, robot_quat):
    """로봇 base 기준 좌표(POINT1~5)를 world 좌표로 바꾼다.
    pnp_teleop.py 의 world_to_base 의 역변환이다."""
    robot_rot = quat_to_matrix(robot_quat)
    world_pos = robot_pos + robot_rot @ np.array(base_pos, dtype=float)
    local_quat = make_target_quat(*base_rpy)
    world_quat = quat_mul(robot_quat, local_quat)
    return world_pos, world_quat / np.linalg.norm(world_quat)


def lerp(start, goal, alpha):
    return start + alpha * (goal - start)


# ══════════════════════════════════════════════════════════════
#  씬 / 로봇 구성 — pnp_teleop.py 와 동일
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}\n {title}\n{'─' * 66}")


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def load_scene():
    """PnP 씬을 /World 아래 참조로 올린다"""
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

    world_prim.GetReferences().AddReference(SCENE_USD, "/World")
    for _ in range(30):
        simulation_app.update()

    print(f"   scene        {Path(SCENE_USD).name}")


def find_all_named(name, root_path=None):
    """이름이 일치하는 프림의 전체 경로를 모은다. root_path 를 주면 그 아래만 본다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path) if root_path else stage.GetPseudoRoot()
    if not root.IsValid():
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(root) if p.GetName() == name]


def setup_arm_drives():
    """IK 결과를 로봇이 따라가도록 팔 관절의 Drive 를 강화한다"""
    stage = omni.usd.get_context().get_stage()
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
        if prim.GetName() not in ARM_JOINTS:
            continue
        for drive_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                count += 1
    print(f"   arm drives   {count}")


def setup_gripper_drive():
    """그리퍼가 무는 힘(최대 힘)을 제한한다"""
    stage = omni.usd.get_context().get_stage()
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
        if prim.GetName() not in GRIPPER_JOINTS:
            continue
        for drive_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive:
                drive.GetStiffnessAttr().Set(GRIPPER_DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(GRIPPER_DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(GRIPPER_DRIVE_MAX_FORCE)
                count += 1
    print(f"   gripper drive {count}")


def register_robot(world):
    """로봇과 그리퍼를 Articulation 으로 등록한다"""
    under = find_all_named(EE_LINK_NAME, ROBOT_PRIM_PATH)
    if not under:
        stage = omni.usd.get_context().get_stage()
        exists = stage.GetPrimAtPath(ROBOT_PRIM_PATH).IsValid()
        anywhere = find_all_named(EE_LINK_NAME)
        roots = [
            str(p.GetPath())
            for p in Usd.PrimRange(stage.GetPseudoRoot())
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        raise RuntimeError(
            f"'{EE_LINK_NAME}' 을 찾지 못했다.\n"
            f"   ROBOT_PRIM_PATH   {ROBOT_PRIM_PATH}  (존재: {exists})\n"
            f"   스테이지 전체의 '{EE_LINK_NAME}'  {anywhere or '없음'}\n"
            f"   ArticulationRoot  {roots or '없음'}\n"
            f"   위 '{EE_LINK_NAME}' 경로의 조상 중 ArticulationRoot 를 "
            f"ROBOT_PRIM_PATH 로 쓴다."
        )
    ee_path = under[0]

    gripper = ParallelGripper(
        end_effector_prim_path=ee_path,
        joint_prim_names=GRIPPER_JOINTS,
        joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
        joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
        action_deltas=None,
    )
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PRIM_PATH,
            name="m0609_robot",
            end_effector_prim_path=ee_path,
            gripper=gripper,
        )
    )
    print(f"   EE frame     {ee_path}")
    return robot


def init_robot(robot, world):
    """Articulation 과 그리퍼를 초기화하고 홈 관절 각도로 보낸다"""
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    q = np.zeros(robot.num_dof)
    q[:6] = READY_JOINTS_RAD
    robot.set_joint_positions(q)


def create_ik_solver(robot):
    """Lula 계산기를 만들고 로봇과 연결한다. base pose 는 로봇의 실제 월드 pose 를 읽어서 넣는다"""
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )
    lula.set_default_position_tolerance(IK_POSITION_TOLERANCE)
    lula.set_default_orientation_tolerance(IK_ORIENTATION_TOLERANCE)

    missing = [j for j in lula.get_joint_names() if j not in robot.dof_names]
    if missing:
        raise RuntimeError(
            f"descriptor 의 cspace 관절이 USD 에 없다: {missing}\n"
            f"   USD dof: {list(robot.dof_names)}"
        )

    ik = ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )
    return lula, ik


def sync_base_pose(lula, robot):
    """로봇 베이스의 현재 월드 pose 를 솔버에 넘긴다"""
    base_pos, base_quat = robot.get_world_pose()
    lula.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
    return base_pos, base_quat


# ══════════════════════════════════════════════════════════════
#  Pick & Place 시퀀스
# ══════════════════════════════════════════════════════════════
def steps_for_pose(start_pos, target_pos, start_quat, target_quat):
    """구간 길이(위치/자세)를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(target_pos - start_pos))
    dot = float(np.clip(abs(np.dot(start_quat, target_quat)), -1.0, 1.0))
    angle_deg = np.degrees(2.0 * np.arccos(dot))
    n = max(dist / TCP_SPEED_M, angle_deg / JOINT_SPEED_DEG)
    return int(np.clip(n, MIN_STEPS, MAX_STEPS))


def steps_for_joint(start_joints, target_joints):
    delta_deg = np.degrees(np.abs(np.array(target_joints) - np.array(start_joints)))
    n = float(np.max(delta_deg)) / JOINT_SPEED_DEG
    return int(np.clip(n, MIN_STEPS, MAX_STEPS))


def build_sequence(base_pos, base_quat):
    """POINT1~5(base 기준)를 world 좌표로 바꿔 순서대로 실행할 스텝을 만든다"""
    def to_world(tcp, rpy):
        return base_to_world(tcp, rpy, base_pos, base_quat)

    p1 = to_world(POINT1_TCP, POINT1_RPY)
    p2 = to_world(POINT2_TCP, POINT2_RPY)
    p3 = to_world(POINT3_TCP, POINT3_RPY)
    p4 = to_world(POINT4_TCP, POINT4_RPY)
    p5 = to_world(POINT5_TCP, POINT5_RPY)

    joint1_delta = np.zeros(6)
    joint1_delta[0] = np.radians(JOINT1_ROTATE_DEG)

    return [
        {"type": "pose",  "label": "안전 위치(파지 전)", "target": p1, "gripper": "open"},
        {"type": "pose",  "label": "파지 위치",          "target": p2, "gripper": None},
        {"type": "hold",  "label": "그리퍼 닫기",        "gripper": "close"},
        {"type": "pose",  "label": "들어올리기",         "target": p3, "gripper": None},
        {"type": "joint", "label": "joint_1 회전",       "target": lambda start: start + joint1_delta, "gripper": None},
        {"type": "pose",  "label": "놓는 위치",          "target": p4, "gripper": None},
        {"type": "hold",  "label": "그리퍼 열기",        "gripper": "open"},
        {"type": "pose",  "label": "후퇴 안전 위치",     "target": p5, "gripper": None},
        {"type": "joint", "label": "홈 복귀",            "target": np.array(READY_JOINTS_RAD, dtype=float), "gripper": None},
    ]


class PickPlaceSequence:
    """steps 를 처음부터 끝까지 하나씩 실행하는 상태 기계.

    pose/hold 스텝은 TCP 를 IK 로, joint 스텝은 관절각을 직접 보간해 움직인다."""

    def __init__(self, robot, ik_solver, arm_indices, steps):
        self._robot = robot
        self._ik = ik_solver
        self._arm_indices = arm_indices
        self.steps = steps
        self.reset()

    def reset(self):
        self.index = 0
        self.step_tick = 0
        self.n_steps = MIN_STEPS
        self.start_pos = self.start_quat = None
        self.start_joints = None
        self.gripper = "open"
        self.done = False

    @property
    def current(self):
        return self.steps[self.index]

    def _current_flange_pose(self):
        flange_pos, flange_rot = self._ik.compute_end_effector_pose()
        pos = flange_pos + flange_rot @ TCP_OFFSET
        quat = make_target_quat(*matrix_to_rpy(flange_rot))
        return pos, quat

    def _enter_step(self):
        step = self.current
        if step["gripper"] is not None:
            self.gripper = step["gripper"]

        if step["type"] in ("pose", "hold"):
            self.start_pos, self.start_quat = self._current_flange_pose()
            if step["type"] == "hold":
                self.target_pos, self.target_quat = self.start_pos, self.start_quat
                self.n_steps = GRIPPER_WAIT_STEPS
            else:
                self.target_pos, self.target_quat = step["target"]
                self.n_steps = steps_for_pose(
                    self.start_pos, self.target_pos, self.start_quat, self.target_quat
                )
        else:  # joint
            q = self._robot.get_joint_positions()
            self.start_joints = np.array([q[i] for i in self._arm_indices])
            target = step["target"]
            self.target_joints = target(self.start_joints) if callable(target) else np.array(target)
            self.n_steps = steps_for_joint(self.start_joints, self.target_joints)

        print(f"   [{self.index}] {step['label']:16s} {self.n_steps:4d} steps   gripper {self.gripper}")

    def tick(self):
        """한 스텝 진행한다. 시퀀스가 끝났으면 아무 것도 하지 않는다"""
        if self.done:
            return True

        if self.start_pos is None and self.start_joints is None:
            self._enter_step()

        step = self.current
        alpha = min(1.0, self.step_tick / float(self.n_steps))
        solved = True

        if step["type"] in ("pose", "hold"):
            pos = lerp(self.start_pos, self.target_pos, alpha)
            quat = quat_slerp(self.start_quat, self.target_quat, alpha)
            action, solved = self._ik.compute_inverse_kinematics(
                target_position=tcp_to_flange(pos, quat),
                target_orientation=quat,
            )
            if solved:
                self._robot.apply_action(action)
        else:  # joint
            joints = lerp(self.start_joints, self.target_joints, alpha)
            self._robot.apply_action(
                ArticulationAction(joint_positions=joints, joint_indices=self._arm_indices)
            )

        self._robot.apply_action(self._robot.gripper.forward(action=self.gripper))

        self.step_tick += 1
        if self.step_tick >= self.n_steps:
            self.index += 1
            self.step_tick = 0
            self.start_pos = self.start_quat = self.start_joints = None
            if self.index >= len(self.steps):
                self.done = True
                print("   [DONE] pick & place 완료")

        return solved


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    load_scene()
    setup_arm_drives()
    setup_gripper_drive()
    robot = register_robot(world)

    world.reset()
    init_robot(robot, world)
    for _ in range(30):
        world.step(render=True)

    section("SOLVER")
    lula, ik_solver = create_ik_solver(robot)
    base_pos, base_quat = sync_base_pose(lula, robot)
    print(f"   base pos     {vec(base_pos)}")

    arm_indices = [robot.get_dof_index(j) for j in ARM_JOINTS]
    steps = build_sequence(base_pos, base_quat)
    seq = PickPlaceSequence(robot, ik_solver, arm_indices, steps)

    section("RUN")
    print("   뷰포트를 클릭해 포커스를 준 뒤 Play 를 누른다\n")

    was_playing = False
    fail_streak = 0
    tick_count = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()
        if is_playing and not was_playing:
            init_robot(robot, world)
            seq.reset()
            fail_streak = 0
            tick_count = 0
        was_playing = is_playing

        if not is_playing:
            continue

        solved = seq.tick()
        if solved:
            fail_streak = 0
        else:
            fail_streak += 1
            if fail_streak == 1 or fail_streak % LOG_INTERVAL == 0:
                carb.log_warn(f"IK 미수렴 ({fail_streak})  step {seq.index}")

        if tick_count % LOG_INTERVAL == 0 and not seq.done:
            print(f"   step {seq.index} '{seq.current['label']}'   {seq.step_tick}/{seq.n_steps}")
        tick_count += 1

    simulation_app.close()


if __name__ == "__main__":
    main()
