"""
Task space 텔레오퍼레이션 — 키보드/마우스로 집어서 옮기기

    isaac_python pnp_teleop.py

목적은 pick & place 좌표를 손으로 찾아내는 것이다.
TCP 를 직교 좌표로 직접 몰고 다니면서 집어 보고, 마음에 드는 지점에서
키를 눌러 좌표를 기록한다. 종료 시 붙여넣기 가능한 상수로 출력된다.

조작
  이동    W/S  +X/-X      A/D  +Y/-Y      Q/E  +Z/-Z
  자세    I/K  pitch      J/L  roll       U/O  yaw
  그리퍼  SPACE 열기/닫기 토글
  기록    1 = PICK 으로 기록    2 = PLACE 로 기록    P = 기록 출력
  기타    M = 키보드/마커 모드 전환    R = 시작 자세 복귀
          [ / ] = 이동 스텝 축소/확대    H = 도움말

마커 모드에서는 뷰포트에서 초록 마커를 마우스 기즈모로 끌면 TCP 가 따라온다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import carb
import carb.input
import numpy as np
import omni.appwindow
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR   = Path(__file__).resolve().parent
M0609_DIR  = THIS_DIR.parent
ASSETS_DIR = M0609_DIR.parent / "assets"

SCENE_USD        = str(ASSETS_DIR / "PnP_test.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

# 씬 안에 로봇이 없을 때만 따로 올린다
ROBOT_USD_FALLBACK = str(M0609_DIR / "Collected_m0609_gripper/m0609_gripper.usd")


# ══════════════════════════════════════════════════════════════
#  로봇 설정
# ══════════════════════════════════════════════════════════════
EE_LINK_NAME = "link_6"
ARM_JOINTS   = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

GRIPPER_JOINTS    = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

# link_6 로컬 +Z 기준 손가락 패드 끝까지의 거리
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET       = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 시작 시 TCP 를 놓을 위치 (로봇 base 기준 상대 오프셋)
START_TCP_OFFSET = np.array([0.35, 0.0, 0.30])


# ══════════════════════════════════════════════════════════════
#  IK 솔버 파라미터
# ══════════════════════════════════════════════════════════════
# 텔레오퍼레이션은 매 프레임 조금씩만 움직이므로 웜스타트가 잘 듣는다.
# 허용오차를 명시해 두면 "수렴했다"는 말의 의미가 분명해진다.
IK_POSITION_TOLERANCE    = 0.003    # m
IK_ORIENTATION_TOLERANCE = 0.02     # rad
CCD_MAX_ITERATIONS       = 300
BFGS_MAX_ITERATIONS      = 150


# ══════════════════════════════════════════════════════════════
#  조작 감도
# ══════════════════════════════════════════════════════════════
STEP_CHOICES = [0.001, 0.002, 0.005, 0.010, 0.020]
STEP_INDEX   = 2          # 기본 5mm
ROT_STEP_DEG = 2.0

# 시작 접근 자세 — 툴이 바닥을 향한다
START_ROLL_DEG  = 180.0
START_PITCH_DEG = 0.0
START_YAW_DEG   = 0.0

# 누르고 있는 방식으로 처리하는 키. tap 처리에서 걸러내는 용도로만 쓴다
JOG_KEYS = {"W", "S", "A", "D", "Q", "E", "I", "K", "J", "L", "U", "O"}


# ══════════════════════════════════════════════════════════════
#  회전 유틸
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


def tcp_to_flange(tcp_pos, quat):
    """손가락 끝 목표를 플랜지(link_6) 목표로 바꾼다"""
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  출력 유틸
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


# ══════════════════════════════════════════════════════════════
#  씬 구성
# ══════════════════════════════════════════════════════════════
def load_scene():
    """PnP 씬을 /World 아래 참조로 올린다"""
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

    world_prim.GetReferences().AddReference(SCENE_USD)
    for _ in range(30):
        simulation_app.update()

    print(f"   scene        {Path(SCENE_USD).name}")


def print_hierarchy(max_depth=4):
    """
    스테이지 구조를 찍는다.

    PnP_test.usd 의 내부 프림 경로를 모르는 상태이므로 첫 실행에서 이걸 보고
    ROBOT_PRIM_PATH 를 확정한다. 경로가 확정되면 max_depth 를 줄여도 된다.
    """
    stage = omni.usd.get_context().get_stage()

    section("STAGE")
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        path = str(prim.GetPath())
        depth = path.count("/") - 1
        if depth < 1 or depth > max_depth:
            continue
        type_name = prim.GetTypeName() or "-"
        marks = []
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            marks.append("ARTICULATION_ROOT")
        if UsdPhysics.Joint(prim):
            marks.append("joint")
        tag = ("   << " + " ".join(marks)) if marks else ""
        print(f"   {'  ' * (depth - 1)}{prim.GetName()}  [{type_name}]{tag}")


def find_articulation_root():
    """
    ArticulationRootAPI 가 붙은 프림을 찾는다.

    씬 안에 로봇이 이미 들어 있으면 그 경로를 쓰고, 없으면 None 을 돌려준다.
    팔 관절 이름이 실제로 존재하는지까지 확인해 카메라 등 다른 articulation 을
    잘못 집지 않도록 한다.
    """
    stage = omni.usd.get_context().get_stage()

    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        joint_names = {
            p.GetName() for p in Usd.PrimRange(prim) if UsdPhysics.Joint(p)
        }
        if set(ARM_JOINTS).issubset(joint_names):
            return str(prim.GetPath())

    return None


def load_robot_fallback():
    """씬에 로봇이 없을 때만 로봇 USD 를 따로 올린다"""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World/m0609")
    stage.GetPrimAtPath("/World/m0609").GetReferences().AddReference(ROBOT_USD_FALLBACK)
    for _ in range(30):
        simulation_app.update()

    print(f"   robot        {Path(ROBOT_USD_FALLBACK).name} (씬에 없어 따로 로드)")


def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def setup_arm_drives(robot_prim_path):
    """IK 결과를 로봇이 따라가도록 팔 관절의 Drive 를 강화한다"""
    stage = omni.usd.get_context().get_stage()
    count = 0

    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
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


def register_robot(world, robot_prim_path):
    """로봇과 그리퍼를 Articulation 으로 등록한다"""
    ee_path = find_prim_path(robot_prim_path, EE_LINK_NAME)
    if ee_path is None:
        raise RuntimeError(f"'{EE_LINK_NAME}' not found under {robot_prim_path}")

    gripper = ParallelGripper(
        end_effector_prim_path=ee_path,
        joint_prim_names=GRIPPER_JOINTS,
        joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
        joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
        action_deltas=None,
    )

    robot = world.scene.add(
        SingleManipulator(
            prim_path=robot_prim_path,
            name="m0609_robot",
            end_effector_prim_path=ee_path,
            gripper=gripper,
        )
    )
    print(f"   EE frame     {ee_path}")
    return robot


def init_gripper(robot, world):
    """그리퍼는 Articulation 초기화 이후에 따로 초기화한다"""
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def set_ready_pose(robot):
    """시작 자세로 보낸다"""
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


# ══════════════════════════════════════════════════════════════
#  IK 솔버
# ══════════════════════════════════════════════════════════════
def create_ik_solver(robot):
    """
    Lula 계산기를 만들고 로봇과 연결한다.

    base pose 는 상수로 박지 않고 로봇의 실제 월드 pose 를 읽어서 넣는다.
    PnP 씬처럼 로봇이 책상 위에 올라가 있으면 원점 가정이 그대로 오차가 된다.
    """
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )

    lula.set_default_position_tolerance(IK_POSITION_TOLERANCE)
    lula.set_default_orientation_tolerance(IK_ORIENTATION_TOLERANCE)
    lula.ccd_max_iterations = CCD_MAX_ITERATIONS
    lula.bfgs_max_iterations = BFGS_MAX_ITERATIONS

    ik = ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )
    return lula, ik


def sync_base_pose(lula, robot):
    """로봇 베이스의 현재 월드 pose 를 솔버에 넘긴다. 매 프레임 호출한다"""
    base_pos, base_quat = robot.get_world_pose()
    lula.set_robot_base_pose(
        robot_position=base_pos,
        robot_orientation=base_quat,
    )
    return base_pos, base_quat


def self_check(lula, robot):
    """
    설정 파일 3종(URDF / descriptor / USD)이 서로 맞는지 기동 시 확인한다.

    여기서 걸리는 문제는 런타임 깊은 곳에서 엉뚱한 증상으로 나타나므로
    미리 걸러내는 편이 싸다.
    """
    section("SELF CHECK")

    solver_joints = lula.get_joint_names()
    stage_dofs = list(robot.dof_names)
    print(f"   solver joints  {', '.join(solver_joints)}")
    print(f"   stage dofs     {', '.join(stage_dofs)}")

    missing = [j for j in solver_joints if j not in stage_dofs]
    if missing:
        raise RuntimeError(
            f"descriptor 의 cspace 관절이 USD 에 없다: {missing}\n"
            f"   USD dof: {stage_dofs}"
        )
    print("   joint match    OK")

    frames = lula.get_all_frame_names()
    if EE_LINK_NAME not in frames:
        raise RuntimeError(
            f"'{EE_LINK_NAME}' 는 URDF 프레임이 아니다. 사용 가능: {frames}"
        )
    print(f"   frames         {', '.join(frames)}")

    lower, upper = lula.get_cspace_position_limits()
    print(f"   joint limits   lower {vec(lower, 2)}")
    print(f"                  upper {vec(upper, 2)}")


# ══════════════════════════════════════════════════════════════
#  키보드 입력
# ══════════════════════════════════════════════════════════════
class Keyboard:
    """
    눌려 있는 키를 집합으로 들고 있는다.

    KEY_PRESS 만 보면 한 번 누를 때 한 칸씩만 움직이므로,
    누르고 있는 동안 계속 움직이도록 PRESS/RELEASE 로 상태를 관리한다.
    """

    def __init__(self):
        self._held = set()
        self._tapped = []

        app_window = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._sub = self._input.subscribe_to_keyboard_events(
            app_window.get_keyboard(), self._on_event
        )

    def _on_event(self, event, *args):
        name = event.input.name
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._held.add(name)
            self._tapped.append(name)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._held.discard(name)
        return True

    def held(self, name):
        """누르고 있는 동안 계속 True"""
        return name in self._held

    def take_taps(self):
        """이번 프레임에 새로 눌린 키들을 꺼내 간다"""
        taps = self._tapped
        self._tapped = []
        return taps

    def close(self):
        self._input.unsubscribe_to_keyboard_events(
            omni.appwindow.get_default_app_window().get_keyboard(), self._sub
        )


# ══════════════════════════════════════════════════════════════
#  텔레오퍼레이션 상태
# ══════════════════════════════════════════════════════════════
class Teleop:
    """TCP 목표를 들고 있으면서 키 입력만큼 조금씩 옮긴다"""

    def __init__(self, start_tcp):
        self.tcp = np.array(start_tcp, dtype=float)
        self.roll = START_ROLL_DEG
        self.pitch = START_PITCH_DEG
        self.yaw = START_YAW_DEG

        self.step_index = STEP_INDEX
        self.gripper_closed = False
        self.marker_mode = False

        self.picks = []
        self.places = []

    @property
    def step(self):
        return STEP_CHOICES[self.step_index]

    def quat(self):
        return make_target_quat(self.roll, self.pitch, self.yaw)

    def jog(self, kb):
        """눌려 있는 키를 읽어 목표를 옮긴다. 옮기기 전 값을 돌려준다"""
        before = (self.tcp.copy(), self.roll, self.pitch, self.yaw)

        d = self.step
        if kb.held("W"): self.tcp[0] += d
        if kb.held("S"): self.tcp[0] -= d
        if kb.held("A"): self.tcp[1] += d
        if kb.held("D"): self.tcp[1] -= d
        if kb.held("Q"): self.tcp[2] += d
        if kb.held("E"): self.tcp[2] -= d

        r = ROT_STEP_DEG
        if kb.held("I"): self.pitch += r
        if kb.held("K"): self.pitch -= r
        if kb.held("J"): self.roll += r
        if kb.held("L"): self.roll -= r
        if kb.held("U"): self.yaw += r
        if kb.held("O"): self.yaw -= r

        return before

    def revert(self, before):
        """IK 가 못 풀면 방금 준 입력을 무른다"""
        self.tcp, self.roll, self.pitch, self.yaw = (
            before[0], before[1], before[2], before[3]
        )


def print_help():
    section("KEYS")
    print("   이동    W/S  +X/-X      A/D  +Y/-Y      Q/E  +Z/-Z")
    print("   자세    I/K  pitch      J/L  roll       U/O  yaw")
    print("   그리퍼  SPACE 열기/닫기 토글")
    print("   기록    1 = PICK 기록   2 = PLACE 기록   P = 기록 출력")
    print("   기타    M = 키보드/마커 모드   R = 시작 자세 복귀")
    print("           [ / ] = 스텝 축소/확대   H = 도움말")


def print_records(teleop):
    """지금까지 기록한 좌표를 붙여넣기 가능한 형태로 찍는다"""
    section("RECORDED")

    if not teleop.picks and not teleop.places:
        print("   기록된 좌표가 없다. 1 또는 2 로 현재 TCP 를 기록한다.")
        return

    for label, items in (("PICK", teleop.picks), ("PLACE", teleop.places)):
        for i, (pos, rpy) in enumerate(items, 1):
            name = f"{label}_{i}"
            print(f"   {name}_TCP   = np.array([{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}])")
            print(f"   {name}_RPY   = ({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})")
    print()


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    load_scene()

    robot_prim_path = find_articulation_root()
    if robot_prim_path is None:
        load_robot_fallback()
        robot_prim_path = find_articulation_root()
    if robot_prim_path is None:
        print_hierarchy()
        raise RuntimeError(
            "팔 관절 6개를 가진 articulation 을 찾지 못했다. "
            "위 STAGE 출력을 보고 ARM_JOINTS 이름을 확인한다."
        )
    print(f"   robot prim   {robot_prim_path}")

    setup_arm_drives(robot_prim_path)
    robot = register_robot(world, robot_prim_path)

    print_hierarchy()

    world.reset()
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)

    section("SOLVER")
    lula, ik_solver = create_ik_solver(robot)
    base_pos, base_quat = sync_base_pose(lula, robot)
    print(f"   base pos     {vec(base_pos)}")
    print(f"   base quat    {vec(base_quat, 4)}")
    if np.linalg.norm(base_pos) > 1e-6:
        print("   NOTE         로봇이 원점에 있지 않다. base pose 동기화가 필수다")

    self_check(lula, robot)

    # 시작 TCP 는 베이스 기준 상대 위치로 잡는다
    teleop = Teleop(base_pos + START_TCP_OFFSET)

    marker = world.scene.add(
        VisualCuboid(
            prim_path="/World/teleop_target",
            name="teleop_target",
            position=teleop.tcp,
            size=0.02,
            color=np.array([0.1, 0.9, 0.2]),
        )
    )

    print_help()

    section("RUN")
    print("   뷰포트에서 Play 를 누른 뒤 조작한다\n")

    kb = Keyboard()
    was_playing = False
    step = 0
    fail_streak = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        if is_playing and not was_playing:
            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)
            step = 0

        for key in kb.take_taps():
            if key == "SPACE":
                teleop.gripper_closed = not teleop.gripper_closed
                state = "CLOSE" if teleop.gripper_closed else "OPEN"
                print(f"   gripper      {state}")
            elif key == "KEY_1":
                teleop.picks.append((teleop.tcp.copy(),
                                     (teleop.roll, teleop.pitch, teleop.yaw)))
                print(f"   PICK  기록   {vec(teleop.tcp, 4)}")
            elif key == "KEY_2":
                teleop.places.append((teleop.tcp.copy(),
                                      (teleop.roll, teleop.pitch, teleop.yaw)))
                print(f"   PLACE 기록   {vec(teleop.tcp, 4)}")
            elif key == "P":
                print_records(teleop)
            elif key == "H":
                print_help()
            elif key == "M":
                teleop.marker_mode = not teleop.marker_mode
                mode = "마커(마우스)" if teleop.marker_mode else "키보드"
                print(f"   mode         {mode}")
            elif key == "R":
                set_ready_pose(robot)
                teleop.tcp = base_pos + START_TCP_OFFSET
                teleop.roll, teleop.pitch, teleop.yaw = (
                    START_ROLL_DEG, START_PITCH_DEG, START_YAW_DEG
                )
                print("   reset        시작 자세로 복귀")
            elif key == "LEFT_BRACKET":
                teleop.step_index = max(0, teleop.step_index - 1)
                print(f"   step         {teleop.step * 1000:.0f} mm")
            elif key == "RIGHT_BRACKET":
                teleop.step_index = min(len(STEP_CHOICES) - 1, teleop.step_index + 1)
                print(f"   step         {teleop.step * 1000:.0f} mm")
            elif key not in JOG_KEYS:
                # carb 의 키 이름을 확신할 수 없으므로 모르는 키는 이름을 찍어 둔다.
                # 바인딩이 안 먹으면 여기 출력된 이름으로 위 조건을 고치면 된다.
                print(f"   (unbound key '{key}')")

        if not is_playing:
            was_playing = is_playing
            continue

        # 베이스가 움직일 수 있으므로 매 프레임 동기화한다
        sync_base_pose(lula, robot)

        if teleop.marker_mode:
            # 마우스 기즈모로 끌어 둔 마커를 그대로 목표로 삼는다
            marker_pos, _ = marker.get_world_pose()
            teleop.tcp = np.array(marker_pos, dtype=float)
            before = None
        else:
            before = teleop.jog(kb)

        target_quat = teleop.quat()
        flange_target = tcp_to_flange(teleop.tcp, target_quat)

        action, solved = ik_solver.compute_inverse_kinematics(
            target_position=flange_target,
            target_orientation=target_quat,
        )

        if solved:
            robot.apply_action(action)
            fail_streak = 0
            if not teleop.marker_mode:
                # 키보드 모드에서는 마커가 목표를 따라다니게 한다
                marker.set_world_pose(position=teleop.tcp)
        else:
            fail_streak += 1
            # 목표가 작업 공간 밖으로 달아나면 되돌아올 수 없으므로 입력을 무른다
            if before is not None:
                teleop.revert(before)
            if fail_streak == 1 or fail_streak % LOG_INTERVAL == 0:
                carb.log_warn(
                    f"IK 미수렴 ({fail_streak}) target TCP {vec(teleop.tcp, 4)}"
                )

        # 그리퍼 명령은 손가락 관절 인덱스만 담고 있어서 팔 IK 지령과 겹치지 않는다
        command = "close" if teleop.gripper_closed else "open"
        robot.apply_action(robot.gripper.forward(action=command))

        if step % LOG_INTERVAL == 0:
            flange, _ = robot.end_effector.get_world_pose()
            actual_tcp = flange + quat_to_matrix(target_quat) @ TCP_OFFSET
            error = float(np.linalg.norm(actual_tcp - teleop.tcp))
            mode = "marker" if teleop.marker_mode else "keys"
            print(
                f"   [{mode:6s}] tcp {vec(teleop.tcp)}  "
                f"rpy ({teleop.roll:+6.1f} {teleop.pitch:+6.1f} {teleop.yaw:+6.1f})  "
                f"err {error:.4f}  step {teleop.step * 1000:.0f}mm"
            )
        step += 1

        was_playing = is_playing

    print_records(teleop)
    kb.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
