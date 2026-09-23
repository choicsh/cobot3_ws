"""
Task space 텔레오퍼레이션 — 키보드/마우스로 집어서 옮기기

    isaac_python pnp_teleop.py

pick & place 좌표를 손으로 찾기 위한 도구. TCP 를 직교 좌표로 몰고 다니면서
집어 보고, 마음에 드는 지점에서 키를 눌러 좌표를 기록한다.

  이동    W/S  카메라 기준 위/아래   A/D  카메라 기준 좌/우   Q/E  카메라 기준 전/후
  자세    J/L  pitch      I/K  roll       U/O  yaw
  그리퍼  NUMPAD 0 열기/닫기 토글
  기록    1 = 좌표 추가 기록    2 = 기록 출력
  기타    M = 키보드/마커 모드 전환    R = 시작 자세 복귀
          [ / ] = 스텝 축소/확대    H = 도움말

마커 모드에서는 뷰포트의 초록 마커를 마우스 기즈모로 끌면 TCP 가 따라온다.
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
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)


# ══════════════════════════════════════════════════════════════
#  경로 / 로봇 설정
# ══════════════════════════════════════════════════════════════
THIS_DIR   = Path(__file__).resolve().parent
M0609_DIR  = THIS_DIR.parent

# SCENE_USD        = str(M0609_DIR.parent / "assets/PnP_test.usd")
SCENE_USD        = str(M0609_DIR.parent / "assets/intergration_nova.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

# intergration_nova.usd: 카터와 팔이 하나의 아티큘레이션으로 병합돼 있다.
#   - 프림 검색(드라이브/EE/카메라)은 팔을 포함하는 nova_carter 기준
#   - Articulation 등록은 실제 루트인 chassis_link 기준
#   - IK 기준 프레임은 팔의 base_link (섀시와 전방 -0.85m / 상방 +0.82m / yaw +90도 차이)
ROBOT_PRIM_PATH = "/World/robot_nova/nova_carter"
ART_ROOT_PATH   = ROBOT_PRIM_PATH + "/chassis_link"
ARM_BASE_PATH   = ROBOT_PRIM_PATH + "/Robot/m0609_camera/m0609/base_link"
EE_LINK_NAME    = "link_6"
D455_CAMERA_NAME = "RSD455"    # 그리퍼에 달린 손목 카메라. 조그 방향 기준으로 쓴다

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# 예전엔 사실상 무한대라 목표 각도로 매 스텝 즉시 스냅했다.
# 유한한 값으로 낮춰서 서서히 가속/감속하며 부드럽게 따라가게 한다.
# 실측 토크 스펙이 아니라 임의 시작값 — 너무 처지면 올리고, 여전히 빠르면 더 낮춘다.
DRIVE_STIFFNESS = 3e5
DRIVE_DAMPING   = 3e4
DRIVE_MAX_FORCE = 300.0

# finger_joint 가 구동 관절이고 나머지는 Mimic 으로 따라온다
# 두 번째 이름은 ParallelGripper 가 요구하는 형식상 필요하다
GRIPPER_JOINTS    = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0     #   0.0 deg
GRIPPER_CLOSE_POS = 1.09    #  45.8 deg — 잡는 폭. 키우면 더 좁게(꽉) 닫힌다


GRIPPER_DRIVE_STIFFNESS = 1e6
GRIPPER_DRIVE_DAMPING   = 1e3
GRIPPER_DRIVE_MAX_FORCE = 15.0   # N — 잡는 힘 상한. 낮추면 살살, 높이면 세게 잡는다

# link_6 로컬 +Z 기준 손가락 패드 끝까지의 거리 (실측)
TCP_OFFSET = np.array([0.0, 0.0, 0.21671])

READY_JOINTS_RAD = [1.57, 0.0, 2.157, 0.0, -0.6, 1.57]

# check_math() 자체 검증용 — "툴이 바닥을 향한다" 케이스 하나로 부호만 확인한다.
# 실제 시작 TCP/자세는 READY_JOINTS_RAD 의 FK 로 계산하지, 이 값을 쓰지 않는다.
START_RPY_DEG = (180.0, 0.0, 0.0)

# 텔레오퍼레이션은 매 프레임 조금씩만 움직이므로 웜스타트가 잘 듣는다
IK_POSITION_TOLERANCE    = 0.003    # m
IK_ORIENTATION_TOLERANCE = 0.02     # rad

STEP_CHOICES = [0.001, 0.002, 0.005, 0.010, 0.020]
ROT_STEP_DEG = 1.0
JOG_KEYS     = {"W", "S", "A", "D", "Q", "E", "I", "K", "J", "L", "U", "O"}
LOG_INTERVAL = 60


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 7_pick_place_color.py 에서 검증된 것을 그대로 쓴다
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


def quat_conjugate(q):
    """단위 쿼터니언의 역회전"""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def matrix_to_rpy(m):
    """회전행렬 -> (roll, pitch, yaw) 도.  make_target_quat 의 합성 순서
    (Rx · Ry · Rz) 에 맞춘 역변환이다 — 순서가 다르면 값이 안 맞는다."""
    pitch = np.degrees(np.arcsin(np.clip(m[0, 2], -1.0, 1.0)))
    roll  = np.degrees(np.arctan2(-m[1, 2], m[2, 2]))
    yaw   = np.degrees(np.arctan2(-m[0, 1], m[0, 0]))
    return roll, pitch, yaw


def quat_to_rpy(q):
    """쿼터니언 -> (roll, pitch, yaw) 도. matrix_to_rpy 참고"""
    return matrix_to_rpy(quat_to_matrix(q))


def world_to_base(world_pos, world_rpy, base_pos, base_quat):
    """world 기준 TCP 좌표를 로봇 base 기준 상대좌표로 바꾼다"""
    base_rot = quat_to_matrix(base_quat)
    local_pos = base_rot.T @ (np.array(world_pos) - np.array(base_pos))

    world_quat = make_target_quat(*world_rpy)
    local_quat = quat_mul(quat_conjugate(base_quat), world_quat)
    local_quat = local_quat / np.linalg.norm(local_quat)

    return local_pos, quat_to_rpy(local_quat)


def tcp_to_flange(tcp_pos, quat):
    """손가락 끝 목표를 플랜지(link_6) 목표로 바꾼다"""
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def flange_to_tcp(flange_pos, quat):
    """플랜지 위치에서 손가락 끝 위치를 구한다"""
    return np.array(flange_pos) + quat_to_matrix(quat) @ TCP_OFFSET


def check_math():
    """TCP 변환의 부호가 맞는지 확인한다. 여기가 틀리면 전부 틀린다"""
    q = make_target_quat(*START_RPY_DEG)
    tcp = np.array([0.4, 0.1, 0.2])
    assert np.allclose(flange_to_tcp(tcp_to_flange(tcp, q), q), tcp), "TCP 변환 왕복 실패"
    # 툴이 바닥을 향하면 플랜지는 TCP 보다 위에 있어야 한다
    assert tcp_to_flange(tcp, q)[2] > tcp[2], "툴 방향 부호가 뒤집혔다"


# ══════════════════════════════════════════════════════════════
#  씬 구성
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

    # PnP_test.usd 에 defaultPrim 이 없어서 prim 경로를 명시해야 참조가 걸린다
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
        # 경로가 틀린 건지, 경로는 맞는데 이름이 다른 건지 구분해서 알려준다
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
            prim_path=ART_ROOT_PATH,
            name="m0609_robot",
            end_effector_prim_path=ee_path,
            gripper=gripper,
        )
    )
    print(f"   EE frame     {ee_path}")
    return robot


def init_robot(robot, world):
    """Articulation 과 그리퍼를 초기화하고 시작 자세로 보낸다"""
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    # 카터 관절(바퀴/캐스터)까지 한 아티큘레이션에 섞여 dof 가 19개다.
    # 앞 6개가 팔이 아니고, zeros 로 덮으면 바퀴까지 리셋돼 로봇이 튄다.
    q = robot.get_joint_positions()
    for name, angle in zip(ARM_JOINTS, READY_JOINTS_RAD):
        q[robot.get_dof_index(name)] = angle
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

    # descriptor 의 cspace 관절이 이 USD 에 실제로 있는지 본다.
    # 안 맞으면 ArticulationSubset 안쪽에서 알아보기 힘든 형태로 터진다.
    missing = [j for j in lula.get_joint_names() if j not in robot.dof_names]
    if missing:
        raise RuntimeError(
            f"descriptor 의 cspace 관절이 USD 에 없다: {missing}\n"
            f"   USD dof: {list(robot.dof_names)}"
        )

    # 엔드 이펙터 프레임이 URDF 에 없으면 생성자가 유효 목록을 담아 ValueError 를 낸다
    ik = ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )
    return lula, ik


def arm_base_pose():
    """IK 기준은 아티큘레이션 루트(카터 섀시)가 아니라 팔의 base_link 다"""
    return SingleXFormPrim(ARM_BASE_PATH).get_world_pose()


def sync_base_pose(lula, robot):
    """팔 base_link 의 현재 월드 pose 를 솔버에 넘긴다"""
    base_pos, base_quat = arm_base_pose()
    lula.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
    return base_pos


# ══════════════════════════════════════════════════════════════
#  키보드
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
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._input = carb.input.acquire_input_interface()
        self._sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_event)

    def _on_event(self, event, *args):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._held.add(event.input.name)
            self._tapped.append(event.input.name)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._held.discard(event.input.name)
        return True

    def held(self, name):
        return name in self._held

    def take_taps(self):
        taps, self._tapped = self._tapped, []
        return taps

    def close(self):
        self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub)


def get_camera_axes(cam_prim_path):
    """그리퍼에 달린 D455 의 world 기준 right/up/forward 단위벡터를 구한다.
    이 마운트는 표준 카메라 축(±Z 전방/Y 상단)이 아니라 로컬 +X 가 화면 전방,
    +Y 가 좌우, +Z 가 상하로 나온다 (실측으로 확인, 3_10 조그 테스트 기준).
    로봇을 따라 움직이는 카메라라서 조그할 때마다 매 프레임 새로 조회한다."""
    fallback = (np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))

    stage = omni.usd.get_context().get_stage()
    cam_prim = stage.GetPrimAtPath(cam_prim_path)
    if not cam_prim.IsValid():
        return fallback

    mat = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
    right   = np.array(mat.TransformDir((0.0, 1.0, 0.0)))
    up      = np.array(mat.TransformDir((0.0, 0.0, 1.0)))
    forward = np.array(mat.TransformDir((1.0, 0.0, 0.0)))
    return (right / np.linalg.norm(right),
            up / np.linalg.norm(up),
            forward / np.linalg.norm(forward))


# ══════════════════════════════════════════════════════════════
#  텔레오퍼레이션 상태
# ══════════════════════════════════════════════════════════════
class Teleop:
    """TCP 목표를 들고 있으면서 키 입력만큼 조금씩 옮긴다"""

    def __init__(self, start_tcp, start_rpy):
        self.home = np.array(start_tcp, dtype=float)
        self.tcp = self.home.copy()
        self.home_rpy = tuple(start_rpy)
        self.roll, self.pitch, self.yaw = self.home_rpy
        self.step_index = 1          # 기본 2mm
        self.gripper_closed = False
        self.marker_mode = False
        self.points = []

    @property
    def step(self):
        return STEP_CHOICES[self.step_index]

    @property
    def rpy(self):
        return (self.roll, self.pitch, self.yaw)

    def quat(self):
        return make_target_quat(self.roll, self.pitch, self.yaw)

    def state(self):
        return (self.tcp.copy(), self.roll, self.pitch, self.yaw)

    def restore(self, state):
        """IK 가 못 풀면 방금 준 입력을 무른다"""
        self.tcp, self.roll, self.pitch, self.yaw = state[0], state[1], state[2], state[3]

    def home_pose(self):
        self.tcp = self.home.copy()
        self.roll, self.pitch, self.yaw = self.home_rpy

    def jog(self, kb, cam_right, cam_up, cam_forward):
        """눌려 있는 키를 읽어 카메라 시점 기준으로 목표를 옮긴다
        W/S 카메라 상하, A/D 카메라 좌우, Q/E 카메라 전후"""
        d, r = self.step, ROT_STEP_DEG
        for key, axis, sign in (("W", cam_up, +1), ("S", cam_up, -1),
                                ("D", cam_right, +1), ("A", cam_right, -1),
                                ("Q", cam_forward, -1), ("E", cam_forward, +1)):
            if kb.held(key):
                self.tcp += sign * d * axis

        if kb.held("J"): self.pitch += r
        if kb.held("L"): self.pitch -= r
        if kb.held("K"): self.roll += r
        if kb.held("I"): self.roll -= r
        if kb.held("O"): self.yaw += r
        if kb.held("U"): self.yaw -= r


def print_records(teleop, robot):
    """기록한 좌표를 붙여넣기 가능한 형태로 찍는다.
    저장은 world 기준으로 해 두고, 출력할 때 로봇 base 기준 상대좌표로 바꾼다."""
    section("RECORDED")
    if not teleop.points:
        print("   기록 없음. 1 로 현재 TCP 를 기록한다.")
        return
    base_pos, base_quat = arm_base_pose()
    for i, (world_pos, world_rpy) in enumerate(teleop.points, start=1):
        pos, rpy = world_to_base(world_pos, world_rpy, base_pos, base_quat)
        print(f"   POINT{i}_TCP = np.array([{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}])   # base 기준")
        print(f"   POINT{i}_RPY = ({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def handle_taps(kb, teleop, robot):
    """한 번 누르는 키들을 처리한다"""
    for key in kb.take_taps():
        if key == "NUMPAD_0":
            teleop.gripper_closed = not teleop.gripper_closed
            print(f"   gripper      {'CLOSE' if teleop.gripper_closed else 'OPEN'}")
        elif key == "KEY_1":
            teleop.points.append((teleop.tcp.copy(), teleop.rpy))
            print(f"   POINT{len(teleop.points)} 기록   {vec(teleop.tcp, 4)}")
        elif key == "KEY_2":
            print_records(teleop, robot)
        elif key == "H":
            print(__doc__)
        elif key == "M":
            teleop.marker_mode = not teleop.marker_mode
            print(f"   mode         {'마커(마우스)' if teleop.marker_mode else '키보드'}")
        elif key == "R":
            teleop.home_pose()
            print("   reset        시작 자세로 복귀")
        elif key in ("LEFT_BRACKET", "RIGHT_BRACKET"):
            delta = -1 if key == "LEFT_BRACKET" else 1
            teleop.step_index = min(max(teleop.step_index + delta, 0), len(STEP_CHOICES) - 1)
            print(f"   step         {teleop.step * 1000:.0f} mm")
        elif key not in JOG_KEYS:
            # carb 의 키 이름을 확신할 수 없으므로 모르는 키는 이름을 찍어 둔다
            print(f"   (unbound key '{key}')")


def main():
    check_math()

    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    load_scene()
    setup_arm_drives()
    setup_gripper_drive()
    robot = register_robot(world)

    camera_matches = find_all_named(D455_CAMERA_NAME, ROBOT_PRIM_PATH)
    if not camera_matches:
        raise RuntimeError(f"'{D455_CAMERA_NAME}' 카메라를 로봇 아래에서 찾지 못했다.")
    camera_prim_path = camera_matches[0]
    print(f"   camera       {camera_prim_path}")

    world.reset()
    init_robot(robot, world)
    for _ in range(30):
        world.step(render=True)

    section("SOLVER")
    lula, ik_solver = create_ik_solver(robot)
    base_pos = sync_base_pose(lula, robot)
    print(f"   base pos     {vec(base_pos)}")
    print(f"   joints       {', '.join(lula.get_joint_names())}")
    if np.linalg.norm(base_pos) > 1e-6:
        print("   NOTE         로봇이 원점에 있지 않다. base pose 동기화가 필수다")

    # 시작 TCP/자세는 READY_JOINTS_RAD 로 실제로 선 FK 로 구한다 —
    # 따로 값을 지어내면 이 목표가 실제 관절 자세와 어긋나서, 다음 프레임에 IK 가
    # 그 어긋난 목표로 끌고가버려 방금 세팅한 홈 자세가 무의미해진다.
    flange_pos, flange_rot = ik_solver.compute_end_effector_pose()
    home_tcp = flange_pos + flange_rot @ TCP_OFFSET
    home_rpy = matrix_to_rpy(flange_rot)
    print(f"   home TCP     {vec(home_tcp)}")
    print(f"   home RPY     ({home_rpy[0]:+.1f} {home_rpy[1]:+.1f} {home_rpy[2]:+.1f})")

    teleop = Teleop(home_tcp, home_rpy)
    marker = world.scene.add(
        VisualCuboid(
            prim_path="/World/teleop_target",
            name="teleop_target",
            position=teleop.tcp,
            size=0.02,
            color=np.array([0.1, 0.9, 0.2]),
        )
    )

    print(__doc__)
    section("RUN")
    print("   뷰포트를 클릭해 포커스를 준 뒤 Play 를 누른다\n")

    kb = Keyboard()
    was_playing = False
    step = fail_streak = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()
        if is_playing and not was_playing:
            init_robot(robot, world)
            teleop.home_pose()
            step = 0
        was_playing = is_playing

        handle_taps(kb, teleop, robot)
        if not is_playing:
            continue

        # 베이스가 움직일 수 있으므로 매 프레임 동기화한다
        sync_base_pose(lula, robot)

        if teleop.marker_mode:
            # 마우스 기즈모로 끌어 둔 마커를 그대로 목표로 삼는다
            teleop.tcp = np.array(marker.get_world_pose()[0], dtype=float)
            before = None
        else:
            before = teleop.state()
            cam_right, cam_up, cam_forward = get_camera_axes(camera_prim_path)
            teleop.jog(kb, cam_right, cam_up, cam_forward)

        quat = teleop.quat()
        action, solved = ik_solver.compute_inverse_kinematics(
            target_position=tcp_to_flange(teleop.tcp, quat),
            target_orientation=quat,
        )

        if solved:
            robot.apply_action(action)
            fail_streak = 0
            if not teleop.marker_mode:
                marker.set_world_pose(position=teleop.tcp)
        else:
            fail_streak += 1
            # 목표가 작업 공간 밖으로 달아나면 되돌아올 수 없으므로 입력을 무른다
            if before is not None:
                teleop.restore(before)
            if fail_streak == 1 or fail_streak % LOG_INTERVAL == 0:
                carb.log_warn(f"IK 미수렴 ({fail_streak}) TCP {vec(teleop.tcp, 4)}")

        # 그리퍼 명령은 손가락 관절 인덱스만 담고 있어 팔 IK 지령과 겹치지 않는다
        robot.apply_action(
            robot.gripper.forward(action="close" if teleop.gripper_closed else "open")
        )

        if step % LOG_INTERVAL == 0:
            actual = flange_to_tcp(robot.end_effector.get_world_pose()[0], quat)
            print(
                f"   [{'marker' if teleop.marker_mode else 'keys':6s}] "
                f"tcp {vec(teleop.tcp)}  rpy ({teleop.roll:+6.1f} {teleop.pitch:+6.1f} "
                f"{teleop.yaw:+6.1f})  err {np.linalg.norm(actual - teleop.tcp):.4f}  "
                f"step {teleop.step * 1000:.0f}mm"
            )
        step += 1

    print_records(teleop, robot)
    kb.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
