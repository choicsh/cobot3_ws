"""
Task space 텔레오퍼레이션 — 키보드/마우스로 집어서 옮기기

    isaac_python pnp_teleop.py

pick & place 좌표를 손으로 찾기 위한 도구. TCP 를 직교 좌표로 몰고 다니면서
집어 보고, 마음에 드는 지점에서 키를 눌러 좌표를 기록한다.

  이동    W/S  +X/-X      A/D  +Y/-Y      Q/E  +Z/-Z
  자세    I/K  pitch      J/L  roll       U/O  yaw
  그리퍼  NUMPAD 0 열기/닫기 토글
  기록    1 = PICK    2 = PLACE    P = 기록 출력
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

SCENE_USD        = str(M0609_DIR.parent / "assets/move_test.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

# PnP_test.usd 안에서 로봇이 놓인 위치
ROBOT_PRIM_PATH = "/World/robot_and_amr/robot/Robot/m0609_camera/m0609"
EE_LINK_NAME    = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

# finger_joint 가 구동 관절이고 나머지는 Mimic 으로 따라온다
# 두 번째 이름은 ParallelGripper 가 요구하는 형식상 필요하다
GRIPPER_JOINTS    = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0     #   0.0 deg
GRIPPER_CLOSE_POS = 1.3     #  45.8 deg — 잡는 폭. 키우면 더 좁게(꽉) 닫힌다

GRIPPER_DRIVE_STIFFNESS = 1e6
GRIPPER_DRIVE_DAMPING   = 1e3
GRIPPER_DRIVE_MAX_FORCE = 30.0   # N — 잡는 힘 상한. 낮추면 살살, 높이면 세게 잡는다

# link_6 로컬 +Z 기준 손가락 패드 끝까지의 거리 (실측)
TCP_OFFSET = np.array([0.0, 0.0, 0.19671])

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 시작 TCP — 로봇 base 기준 상대 위치
START_TCP_OFFSET = np.array([0.35, 0.0, 0.30])
START_RPY_DEG    = (180.0, 0.0, 0.0)    # 툴이 바닥을 향한다

# 텔레오퍼레이션은 매 프레임 조금씩만 움직이므로 웜스타트가 잘 듣는다
IK_POSITION_TOLERANCE    = 0.003    # m
IK_ORIENTATION_TOLERANCE = 0.02     # rad

STEP_CHOICES = [0.001, 0.002, 0.005, 0.010, 0.020]
ROT_STEP_DEG = 2.0
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
            prim_path=ROBOT_PRIM_PATH,
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


def sync_base_pose(lula, robot):
    """로봇 베이스의 현재 월드 pose 를 솔버에 넘긴다"""
    base_pos, base_quat = robot.get_world_pose()
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


# ══════════════════════════════════════════════════════════════
#  텔레오퍼레이션 상태
# ══════════════════════════════════════════════════════════════
class Teleop:
    """TCP 목표를 들고 있으면서 키 입력만큼 조금씩 옮긴다"""

    def __init__(self, start_tcp):
        self.home = np.array(start_tcp, dtype=float)
        self.tcp = self.home.copy()
        self.roll, self.pitch, self.yaw = START_RPY_DEG
        self.step_index = 2          # 기본 5mm
        self.gripper_closed = False
        self.marker_mode = False
        self.pick = None
        self.place = None

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
        self.roll, self.pitch, self.yaw = START_RPY_DEG

    def jog(self, kb):
        """눌려 있는 키를 읽어 목표를 옮긴다"""
        d, r = self.step, ROT_STEP_DEG
        for key, axis, sign in (("W", 0, +1), ("S", 0, -1),
                                ("A", 1, +1), ("D", 1, -1),
                                ("Q", 2, +1), ("E", 2, -1)):
            if kb.held(key):
                self.tcp[axis] += sign * d

        if kb.held("I"): self.pitch += r
        if kb.held("K"): self.pitch -= r
        if kb.held("J"): self.roll += r
        if kb.held("L"): self.roll -= r
        if kb.held("U"): self.yaw += r
        if kb.held("O"): self.yaw -= r


def print_records(teleop):
    """기록한 좌표를 붙여넣기 가능한 형태로 찍는다"""
    section("RECORDED")
    if teleop.pick is None and teleop.place is None:
        print("   기록 없음. 1 또는 2 로 현재 TCP 를 기록한다.")
        return
    for label, rec in (("PICK", teleop.pick), ("PLACE", teleop.place)):
        if rec is None:
            continue
        pos, rpy = rec
        print(f"   {label}_TCP = np.array([{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}])")
        print(f"   {label}_RPY = ({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})")


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
            teleop.pick = (teleop.tcp.copy(), teleop.rpy)
            print(f"   PICK  기록   {vec(teleop.tcp, 4)}")
        elif key == "KEY_2":
            teleop.place = (teleop.tcp.copy(), teleop.rpy)
            print(f"   PLACE 기록   {vec(teleop.tcp, 4)}")
        elif key == "P":
            print_records(teleop)
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
            teleop.jog(kb)

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

    print_records(teleop)
    kb.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
