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

import omni.usd

from isaacsim.core.api import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.core.utils.types import ArticulationAction
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


# 맵 제작 때 씬 전체를 x축으로 -1.84091 옮겼으므로 경로 좌표도 그만큼 옮겼다.
#
# 책상 옆(x = -0.761)은 로봇 옆면과 책상 사이가 1cm 라 '직진만' 가능한 구간이다.
# 그래서 경로를 이렇게 잡는다:
#   1) 파지 자세에서 그대로 남쪽으로 직진해 책상에서 벗어난다
#   2) 넓은 곳에서 회전해 복도를 돌아 북쪽 책상 위(y=19.5)까지 간다
#   3) 거기서 -90deg 로 돌아 남쪽으로 직진해 내려놓는 자세로 들어간다
#      (도착 지점에서는 제자리 회전이 안 되므로 미리 방향을 맞춰서 들어간다)
PICK_POSE_XY = np.array([1.08010 - 1.84091, 0.55106])    # = (-0.761, 0.551)
PLACE_POSE_XY = np.array([1.08010 - 1.84091, 15.5])      # = (-0.761, 15.5)
WAYPOINTS = [
    PICK_POSE_XY,                      # W0  출발 = 파지 자세
    np.array([-0.761,  -1.9]),         # W1  책상에서 직진으로 빠져나온다.
                                       #     회전 가능 구간은 y = -1.8 ~ -2.0 로 좁다.
                                       #     이보다 북쪽이면 꽁무니가 책상(y>=-0.49)에,
                                       #     남쪽이면 남쪽 벽(y=-2.85)에 닿는다
    np.array([ 5.859,  -1.0]),         # W2  복도. 여기서 북쪽으로 돌 때 꽁무니가 남쪽 벽을
                                       #     쓸지 않도록 벽에서 충분히 떨어뜨렸다
    np.array([ 5.859,  19.5]),         # W3  북쪽 끝까지
    np.array([-0.761,  19.5]),         # W4  내려놓는 자세의 바로 위
    PLACE_POSE_XY,                     # W5  남쪽으로 직진해 진입
]

APPROACH_BACKOFF_M = 0.082   # P1->P2 와 같은 수평 접근 거리

# 바퀴 주행 (길이 2배 스케일 실측값)
WHEEL_JOINTS = ("joint_wheel_left", "joint_wheel_right")
WHEEL_RADIUS_M = 0.28
WHEEL_DISTANCE_M = 0.8264
SPEED_MPS = 0.8          # 직진 속도
TURN_RPS = 0.6           # 제자리 회전 속도
ACCEL_MPS2 = 0.5         # 가감속 (급출발하면 트레이가 랙에서 미끄러진다)
POS_TOL_M = 0.15         # waypoint 도달 판정
YAW_TOL_RAD = np.radians(3.0)
STRAIGHT_DEADBAND_RAD = np.radians(5.0)   # 이 안쪽이면 조향 없이 직진
TURN_FIRST_RAD = np.radians(20.0)   # 이보다 많이 틀어져 있으면 먼저 제자리 회전
FINAL_YAW_DEG = -90.0    # 도착 후 정렬할 방향 (파지할 때와 같은 자세)
PICK_PAUSE_S = 1.0       # 홈 복귀 후 주행을 시작하기까지 대기 시간


def yaw_of(quat):
    """(w, x, y, z) 쿼터니언에서 z축 회전각"""
    w, x, y, z = [float(v) for v in quat]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class WheelDriver:
    """노바카터 바퀴로 실제 주행한다.

    waypoint 마다 '많이 틀어져 있으면 제자리 회전 -> 아니면 직진' 을 반복하고,
    마지막 waypoint 에 닿으면 FINAL_YAW 로 자세를 맞춘다.
    트레이는 랙 위에 마찰로 실려 가므로 따로 붙잡지 않는다."""

    def __init__(self, robot, waypoints, final_yaw_rad):
        self._robot = robot
        self._wheel_idx = np.array([robot.get_dof_index(j) for j in WHEEL_JOINTS])
        self._targets = [np.asarray(w[:2], dtype=float) for w in waypoints[1:]]
        self._final_yaw = final_yaw_rad
        self._i = 0
        self._v = 0.0
        self.done = False

    def _apply(self, v, w):
        half = 0.5 * WHEEL_DISTANCE_M * w
        self._robot.apply_action(ArticulationAction(
            joint_velocities=np.array([(v - half) / WHEEL_RADIUS_M,
                                       (v + half) / WHEEL_RADIUS_M]),
            joint_indices=self._wheel_idx,
        ))

    def stop(self):
        self._v = 0.0
        self._apply(0.0, 0.0)

    def step(self, dt):
        if self.done:
            self.stop()
            return True

        pos, quat = self._robot.get_world_pose()
        yaw = yaw_of(quat)

        if self._i >= len(self._targets):          # 마지막 자세 정렬
            err = wrap_pi(self._final_yaw - yaw)
            if abs(err) < YAW_TOL_RAD:
                self.stop()
                self.done = True
                return True
            self._apply(0.0, float(np.clip(2.0 * err, -TURN_RPS, TURN_RPS)))
            return False

        delta = self._targets[self._i] - np.asarray(pos[:2], dtype=float)
        dist = float(np.linalg.norm(delta))
        if dist < POS_TOL_M:
            self._i += 1
            print(f"   waypoint {self._i}/{len(self._targets)} 도착")
            return False

        err = wrap_pi(np.arctan2(delta[1], delta[0]) - yaw)
        # 책상 옆 1cm 틈에서는 조금만 조향해도 긁혀서 갇힌다 -> 거의 정렬돼 있으면 완전 직진
        w = 0.0 if abs(err) < STRAIGHT_DEADBAND_RAD else float(np.clip(2.0 * err, -TURN_RPS, TURN_RPS))
        v_target = 0.0 if abs(err) > TURN_FIRST_RAD else min(SPEED_MPS, dist)
        step_v = ACCEL_MPS2 * dt
        self._v = float(np.clip(v_target, self._v - step_v, self._v + step_v))
        self._apply(self._v, w)
        return False


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
    driver = None
    wait_ticks = 0
    was_playing = False

    while simulation_app.is_running():
        world.step(render=True)

        is_playing = world.is_playing()
        if is_playing and not was_playing:
            init_robot(robot, world)
            driver = WheelDriver(robot, WAYPOINTS, np.radians(FINAL_YAW_DEG))
            driver.stop()
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
            # 팔은 홈 자세 그대로 두고 바퀴로 주행한다. 트레이는 랙 위에 마찰로 실려 간다
            if driver.step(world.get_physics_dt()):
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
