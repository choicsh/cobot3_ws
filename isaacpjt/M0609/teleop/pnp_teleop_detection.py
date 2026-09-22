"""
Task space 텔레오퍼레이션 + 비전 pick & place — pnp_teleop.py 에 검출 연동을 붙인 판

    source /opt/ros/jazzy/setup.bash
    ~/isaacsim/python.sh pnp_teleop_detection.py

ROS2 를 먼저 source 하지 않으면 토픽이 하나도 안 나간다 — isaac-sim.sh 와 달리
python.sh 는 setup_ros_env.sh 를 source 하지 않아서 브리지가 ROS2 라이브러리를
찾지 못한다. 에러 없이 조용히 실패하므로 증상만 보면 원인을 알기 어렵다.

손목 D455 의 rgb / depth / camera_info 를 ROS2 로 관제 PC 에 보내고, 관제 PC 가
돌려주는 /tray_detection 을 구독한다. C 를 누르면 그 시점의 최신 검출값으로
**화면 왼쪽 트레이부터** 집어 랙 1,2,3 번 자리에 차례로 넣는다.

검출을 관제 PC 가 맡는 건 성능 분산 때문만이 아니다 — ultralytics 는 python3.12 이고
Isaac Sim 은 자체 python3.11 이라 애초에 한 프로세스에 합칠 수 없다.
관제 PC 쪽 코드는 같은 저장소의 admin_ws/ 에 있다.

  이동    W/S  카메라 기준 위/아래   A/D  카메라 기준 좌/우   Q/E  카메라 기준 전/후
  자세    J/L  yaw (화면 좌우로 돌아봄)   U/O  roll (화면이 기움)   I/K  pitch (위아래 젖힘)
  그리퍼  NUMPAD 0 열기/닫기 토글
  기록    1 = 좌표 추가 기록    2 = 기록 출력
  검출    V = 검출 좌표만 확인 (빨간 마커 표시, 팔은 안 움직인다)
          C = 최신 검출 결과로 pick & place 실행 (왼쪽 트레이부터 랙 1,2,3 번)
  기타    M = 키보드/마커 모드 전환    R = 시작 자세 복귀
          [ / ] = 스텝 축소/확대    H = 도움말

마커 모드에서는 뷰포트의 초록 마커를 마우스 기즈모로 끌면 TCP 가 따라온다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension

# 그래프 노드를 만들기 전에 켜야 한다
enable_extension("isaacsim.ros2.bridge")

from pathlib import Path
import time

import carb
import carb.input
import numpy as np
import omni.appwindow
import omni.graph.core as og
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.prims import set_targets
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)

# 파지 시퀀스 실행기와 base<->world 변환을 재사용한다. 이 모듈은 import 만 해도
# 자기 SimulationApp 을 만들려 하므로 위에서 만든 것을 돌려준다
# (pickup_place_go_nova.py 가 쓰는 것과 같은 방법).
import isaacsim
isaacsim.SimulationApp = lambda *args, **kwargs: simulation_app
import pick_and_place as _pnp
from pick_and_place import PickPlaceSequence


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

# RSD455 는 Xform 이고 실제 렌더링되는 컬러 카메라는 그 아래 프림이다.
# (Left/Right/Pseudo_Depth 도 같이 있으므로 이름을 정확히 짚어야 한다)
D455_COLOR_CAMERA_NAME = "Camera_OmniVision_OV9782_Color"

# ROS2 — rgb/depth/camera_info 를 관제 PC 로 보내고, 검출 결과를 돌려받는다.
# depth 와 camera_info 는 컬러와 '같은' render product 에 붙여야 픽셀이 정렬된다.
DETECT_GRAPH_PATH = "/World/DetectGraph"
COLOR_TOPIC       = "wrist_camera/color/image_raw"
DEPTH_TOPIC       = "wrist_camera/depth/image_raw"
INFO_TOPIC        = "wrist_camera/color/camera_info"
RESULT_TOPIC      = "tray_detection"
IMAGE_RESOLUTION  = (640, 480)
# 매 렌더 프레임마다 발행하면 텔레오퍼레이션도 네트워크도 버겁다. N 프레임 건너뛴다
IMAGE_FRAME_SKIP  = 4

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

    # 파지 yaw 보정 — 보정 후 툴 +Z(손가락 방향)가 트레이 쪽을 향해야 한다.
    # 부호가 틀리면 그리퍼가 트레이 옆구리를 보고 들어간다.
    for target in ([-0.0091, 0.8007, 0.2776], [0.8289, 0.0786, 0.1061]):
        d, yaw = approach_direction(np.array(target))
        rot = quat_to_matrix(quat_mul(quat_from_axis([0, 0, 1], yaw),
                                      make_target_quat(*_pnp.POINT2_RPY)))
        assert np.dot(rot[:2, 2], d[:2]) > 0.99, f"파지 yaw 보정이 틀렸다: {target}"

    # joint_1 회전량 — 기존 하드코딩 상황(트레이 +y, 랙 +x)을 재현하면 -90 도 근처가 나와야 한다
    old = rack_yaw_delta_deg(np.array([-0.0091, 0.8007, 0.2776]), np.array([0.7734, 0.0513, 0.1102]))
    assert abs(old - _pnp.JOINT1_ROTATE_DEG) < 5.0, f"joint_1 회전 계산이 기존 값과 다르다: {old:+.1f}"


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
#  ROS2 — 이미지 발행 + 스냅 트리거
# ══════════════════════════════════════════════════════════════
def find_color_camera():
    """손목 D455 의 컬러 카메라 프림 경로를 찾는다.

    nova_carter 에도 스테레오 카메라가 여럿 달려 있으므로, 팔의 RSD455 아래로
    범위를 좁혀서 찾는다."""
    rsd455 = find_all_named(D455_CAMERA_NAME, ROBOT_PRIM_PATH)
    if not rsd455:
        raise RuntimeError(f"'{D455_CAMERA_NAME}' 를 로봇 아래에서 찾지 못했다.")
    color = find_all_named(D455_COLOR_CAMERA_NAME, rsd455[0])
    if not color:
        stage = omni.usd.get_context().get_stage()
        under = [p.GetName() for p in Usd.PrimRange(stage.GetPrimAtPath(rsd455[0]))
                 if p.GetTypeName() == "Camera"]
        raise RuntimeError(
            f"'{D455_COLOR_CAMERA_NAME}' 를 {rsd455[0]} 아래에서 찾지 못했다.\n"
            f"   그 아래 Camera 프림: {under or '없음'}"
        )
    return color[0]


def build_detection_graph(color_camera_path):
    """손목 카메라를 ROS2 이미지 토픽에 연결한다. 실제 발행 여부는 게이트로 막는다."""
    og.Controller.edit(
        {"graph_path": DETECT_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("rp", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("pub_color", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("pub_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("pub_info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("sub_detect", "isaacsim.ros2.bridge.ROS2Subscriber"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "rp.inputs:execIn"),
                ("tick.outputs:tick", "sub_detect.inputs:execIn"),
                ("rp.outputs:execOut", "pub_color.inputs:execIn"),
                ("rp.outputs:execOut", "pub_depth.inputs:execIn"),
                ("rp.outputs:execOut", "pub_info.inputs:execIn"),
                # 셋 다 같은 render product 를 봐야 컬러/깊이 픽셀이 정렬된다
                ("rp.outputs:renderProductPath", "pub_color.inputs:renderProductPath"),
                ("rp.outputs:renderProductPath", "pub_depth.inputs:renderProductPath"),
                ("rp.outputs:renderProductPath", "pub_info.inputs:renderProductPath"),
                ("context.outputs:context", "pub_color.inputs:context"),
                ("context.outputs:context", "pub_depth.inputs:context"),
                ("context.outputs:context", "pub_info.inputs:context"),
                ("context.outputs:context", "sub_detect.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("rp.inputs:width", IMAGE_RESOLUTION[0]),
                ("rp.inputs:height", IMAGE_RESOLUTION[1]),
                ("pub_color.inputs:type", "rgb"),
                ("pub_color.inputs:topicName", COLOR_TOPIC),
                ("pub_color.inputs:frameId", "d455_color_optical_frame"),
                ("pub_color.inputs:frameSkipCount", IMAGE_FRAME_SKIP),
                # depth = DistanceToImagePlane (광축 방향 Z). 핀홀 역투영 식이 그대로 맞다
                ("pub_depth.inputs:type", "depth"),
                ("pub_depth.inputs:topicName", DEPTH_TOPIC),
                ("pub_depth.inputs:frameId", "d455_color_optical_frame"),
                ("pub_depth.inputs:frameSkipCount", IMAGE_FRAME_SKIP),
                ("pub_info.inputs:topicName", INFO_TOPIC),
                ("pub_info.inputs:frameId", "d455_color_optical_frame"),
                ("pub_info.inputs:frameSkipCount", IMAGE_FRAME_SKIP),
                ("sub_detect.inputs:topicName", RESULT_TOPIC),
            ],
        },
    )
    # cameraPrim 은 relationship 이라 SET_VALUES 로는 못 넣는다
    set_targets(
        prim=omni.usd.get_context().get_stage().GetPrimAtPath(f"{DETECT_GRAPH_PATH}/rp"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[color_camera_path],
    )
    set_generic_message_type(f"{DETECT_GRAPH_PATH}/sub_detect", "std_msgs", "msg", "Float32MultiArray")

    for name, topic in (("color", COLOR_TOPIC), ("depth", DEPTH_TOPIC), ("info", INFO_TOPIC)):
        print(f"   pub {name:<6s}   /{topic}")
    print(f"   sub detect   /{RESULT_TOPIC}  (std_msgs/Float32MultiArray)")


def set_generic_message_type(node_path, package, subfolder, name):
    """제네릭 ROS2 Publisher/Subscriber 의 메시지 타입을 설정한다.

    messageName 이 바뀌면 노드가 메시지 필드에 해당하는 동적 어트리뷰트를 다시
    만든다. 세 값을 og.Controller.edit 의 SET_VALUES 로 한꺼번에 넣으면 타입이
    덜 채워진 상태로 재생성돼 구독/발행이 조용히 안 올라온다.
    공식 테스트(isaacsim.ros2.bridge/tests/test_subscriber.py)와 같은 순서로,
    사이에 app update 를 끼워 넣어야 한다."""
    def attr(field):
        return og.Controller.attribute(f"{node_path}.inputs:{field}")

    attr("messageName").set("")          # 먼저 비워서 재생성을 유도한다
    simulation_app.update()
    attr("messagePackage").set(package)
    attr("messageSubfolder").set(subfolder)
    attr("messageName").set(name)        # name 을 마지막에 — 이때 재생성된다
    simulation_app.update()


def read_detections(verbose=False):
    """관제 PC 가 보낸 최신 검출 결과를 읽는다.

    반환: (seq, [(np.array([x, y, z]), conf), ...])  카메라 광학 프레임, **화면 왼쪽부터**.
    seq 는 관제 PC 의 발행 카운터. 값이 올라갔다는 건 새로 찍은 프레임이라는 뜻이다.

    제네릭 ROS2Subscriber 는 메시지 필드를 동적 output 어트리뷰트로 만든다.
    경로 문자열로 읽는 법과 노드 객체로 읽는 법 두 가지가 있는데, 어느 쪽이 되는지
    버전에 따라 다르므로 둘 다 시도한다. verbose 면 실패 이유를 찍는다."""
    node_path = f"{DETECT_GRAPH_PATH}/sub_detect"
    data, why = None, []
    try:
        data = og.Controller.attribute(f"{node_path}.outputs:data").get()
    except Exception as exc:
        why.append(f"경로 문자열: {exc}")
    if data is None:
        try:
            data = og.Controller.attribute("outputs:data", og.Controller.node(node_path)).get()
        except Exception as exc:
            why.append(f"노드 객체: {exc}")

    if data is None:
        if verbose:
            print("   구독자에서 outputs:data 를 못 읽었다:")
            for line in why:
                print(f"      {line}")
            prim = omni.usd.get_context().get_stage().GetPrimAtPath(node_path)
            outs = [a.GetName() for a in prim.GetAttributes() if a.GetName().startswith("outputs:")] \
                if prim.IsValid() else None
            print(f"      실제 output 어트리뷰트: {outs if outs else '없음 / 프림 없음'}")
        return -1, []

    if len(data) < 2:
        if verbose:
            print(f"   구독자 data 길이가 {len(data)} 다 — 아직 수신 전이거나 형식이 다르다: {list(data)}")
        return -1, []

    seq, count = int(data[0]), int(data[1])
    out = []
    for i in range(count):
        x, y, z, conf = data[2 + i * 4 : 6 + i * 4]
        out.append((np.array([x, y, z], dtype=float), float(conf)))
    if verbose:
        print(f"   구독자 수신: seq {seq}, {count}개, raw {list(data)}")
    return seq, out


# ══════════════════════════════════════════════════════════════
#  검출 좌표 -> 파지 / 랙 적재
# ══════════════════════════════════════════════════════════════
# 랙 자리 (팔 base 기준).
# 2번 = pick_and_place 의 POINT6("놓는 위치") 그대로. 이미 검증된 좌표다.
# 1/3번은 거기서 슬롯 간격만큼 좌우로 옮긴 것. 간격 0.1563 은 실측 3점의 전체 폭 절반이다
# (개별 간격이 0.1659 / 0.1468 로 들쭉날쭉해서 등간격으로 폈다).
# 축별로 따로 잡아둔다 — 실물 보고 손볼 때 이 세 값만 만지면 된다
RACK_CENTER_X = -0.0093   # 2번 랙(가운데). POINT6 과 같은 값
RACK_ROW_Y    =  0.8190   # 트레이가 랙에 앉는 깊이.
                          #   POINT6 은 0.7690 인데 후퇴량(50mm)만큼 얕아서 그만큼 더 뻗었다.
                          #   참고: POINT2(랙에서 트레이를 집는 자리) 는 0.8007
RACK_ROW_Z    =  0.2500
RACK_PITCH_M  =  0.1563   # 슬롯 간격. 실측 3점 전체 폭의 절반
RACK_SLOTS = [
    np.array([RACK_CENTER_X + i * RACK_PITCH_M, RACK_ROW_Y, RACK_ROW_Z]) for i in (-1, 0, 1)
]
RACK_RPY = (-89.3, 0.1, 180.0)     # = POINT6_RPY. 실측한 (-95.1,-1.4,-179.1) 과 6도 이내다
# 놓은 뒤 후퇴 오프셋. 기존 POINT6->POINT1 은 [0, -0.05, +0.0276] 이었는데
# 5cm 로는 손가락이 트레이에서 충분히 빠지지 않아 15cm 수평 후퇴로 늘렸다.
RACK_RETREAT_OFFSET = np.array([0.0, -0.15, 0.0])

# 검출점은 트레이의 '보이는 앞면' 위의 점이지 기하 중심이 아니다. 접근축(base +y)으로
# 이만큼 밀어 중심을 추정한다. tray.usd 실측이 0.091 x 0.139 x 0.158 m 라 어느 면을
# 보느냐에 따라 0.045~0.070 사이다. 실물을 보고 맞추는 튜닝 값이다.
# ponytail: 단일 상수 추정, 방향까지 알아야 하면 점군 PCA 로 올릴 것
# 실물 보고 맞춘 값. 0.057 -> 0.037 -> 0.017 -> -0.003 으로 매번 2cm 씩 얕게 당겼다.
# 음수 = 검출점(트레이 앞면)보다 살짝 더 로봇 쪽을 잡는다.
TRAY_HALF_DEPTH_M    = -0.003
GRASP_ABOVE_CENTER_M = 0.04    # 검출점(트레이 앞면) 기준. 최초 사양의 '센터 +2cm' 에 실측 보정 2cm 를 더한 값
APPROACH_BACKOFF_M   = 0.082   # pick_and_place 의 POINT1->POINT2 와 같은 수평 접근 거리
LIFT_Z_M             = 0.4496  # pick_and_place 의 POINT3 높이 (base 기준 절대값)
# 관제 PC 쪽에도 같은 게이트가 있지만, 받은 쪽에서도 한 번 더 본다.
# 여기를 통과한 값으로 팔이 실제로 움직이므로 신뢰 경계다.
DETECT_DEPTH_MIN_M   = 0.15
DETECT_DEPTH_MAX_M   = 1.5
# 1단계에서 트레이 정면 이 거리까지 다가가 멈춘 뒤 다시 관측한다 (TCP 기준).
# 파지 접근 위치와 같은 지점이다 — 실제로 잡으러 갈 자리에서 재서야 오차가 작다.
# 카메라는 TCP 보다 TCP_OFFSET(0.217m) 만큼 뒤라 실제 관측 거리는 약 0.30m 다.
OBSERVE_DISTANCE_M   = 0.082
# 도착 직후에는 팔이 아직 흔들린다. 이만큼 정지한 뒤에 재관측을 받는다
OBSERVE_SETTLE_FRAMES = 90
# 팔이 닿지 않는 거리의 검출은 벽/배경 오검출이다. base 원점 기준 파지점 거리로 거른다
# (검증된 파지점들이 0.71~0.87m 대역이다)
GRASP_REACH_MIN_M    = 0.25
GRASP_REACH_MAX_M    = 0.95
# 관측 자세 도착 후 '새로 찍은' 검출을 기다린다. seq 가 이만큼 올라가야 인정한다
FRESH_SEQ_ADVANCE    = 2
# 그 안에 새 검출이 안 오면 포기한다 (관제 노드가 죽었을 수 있다)
FRESH_TIMEOUT_FRAMES = 300


def optical_to_world(point_optical, color_camera_path):
    """관제 PC 가 준 카메라 광학 프레임 좌표를 월드로 옮긴다.

    ROS 광학 규약은 (x 우, y 하, z 전방)이고 USD 카메라는 (x 우, y 상, -z 전방)이라
    y, z 의 부호가 뒤집힌다. 여기서 부호 하나 틀리면 로봇이 엉뚱한 곳으로 간다."""
    x, y, z = point_optical
    usd_point = Gf.Vec3d(float(x), float(-y), float(-z))
    cam_prim = omni.usd.get_context().get_stage().GetPrimAtPath(color_camera_path)
    mat = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
    world = np.array(mat.Transform(usd_point), dtype=float)

    # 진단용 — 카메라 원점과 축이 어디를 향하는지 같이 찍는다.
    # world 가 cam_origin 에서 'fwd 방향으로 cam 의 z 만큼' 떨어져 있어야 정상이다.
    cam_origin = np.array(mat.Transform(Gf.Vec3d(0, 0, 0)), dtype=float)
    fwd = np.array(mat.TransformDir(Gf.Vec3d(0, 0, -1)), dtype=float)   # USD 카메라 전방 = -Z
    scale = np.linalg.norm(fwd)
    print(f"      cam_origin {vec(cam_origin)}  fwd {vec(fwd)}  |fwd| {scale:.4f}"
          f"  (|fwd| 이 1.0 이 아니면 조상에 스케일이 걸려 있다)")
    print(f"      기대 위치   {vec(cam_origin + fwd / max(scale, 1e-9) * float(z))}  (원점에서 정면으로 z 만큼)")
    return world


def grasp_point_base(point_world, base_pos, base_quat):
    """검출점(월드) -> 파지 목표(팔 base 기준).

    접근은 base +y 방향이므로(POINT1->POINT2 와 같다) 트레이 중심 보정도 +y 로 준다."""
    surface, _ = world_to_base(point_world, RACK_RPY, base_pos, base_quat)
    # 검출점은 트레이 앞면이므로 '베이스에서 트레이를 향하는 방향'으로 더 밀어 중심을 잡는다
    direction, _ = approach_direction(surface)
    return surface + direction * TRAY_HALF_DEPTH_M + np.array([0.0, 0.0, GRASP_ABOVE_CENTER_M])


def approach_direction(grasp_base):
    """base 원점에서 트레이를 향하는 수평 단위벡터와, 기준자세에서 돌려야 할 각도.

    POINT2_RPY 는 트레이가 base +y 정면에 있을 때 맞춘 자세다. 트레이가 다른
    방향에 있으면 그만큼 base z 축으로 돌려야 그리퍼가 트레이를 마주본다.
    Rz(t)*[0,1,0] = [-sin t, cos t, 0] 이므로 t = atan2(-dx, dy).

    ponytail: 트레이의 실제 yaw 는 2D 박스로 알 수 없어서 '베이스에서 트레이를
    향하는 방향'을 정면으로 본다. 트레이가 비스듬히 놓이면 어긋난다 —
    그때는 깊이 점군 PCA 로 주축을 추정해야 한다."""
    d = np.array([grasp_base[0], grasp_base[1], 0.0], dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-6:
        return np.array([0.0, 1.0, 0.0]), 0.0
    d /= n
    return d, float(np.degrees(np.arctan2(-d[0], d[1])))


def rack_yaw_delta_deg(grasp_base, place_base):
    """트레이에서 랙으로 갈 때 joint_1 을 얼마나 돌려야 하는가 (도).

    기존 코드는 JOINT1_ROTATE_DEG = -90 고정이었다. 그건 트레이가 base +y 정면에
    있을 때의 값이라(그 경우 실제 delta 가 -86.9 도), 검출 위치가 달라지면 엉뚱한
    쪽으로 돌아버린다. 두 지점의 방위각 차이로 계산한다.
    +-180 도로 접어서 먼 쪽으로 도는 것을 막는다."""
    _, tray_yaw = approach_direction(grasp_base)
    _, rack_yaw = approach_direction(place_base)
    return (rack_yaw - tray_yaw + 180.0) % 360.0 - 180.0


def base_pose_to_world(tcp_base, quat_base, robot_pos, robot_quat):
    """_pnp.base_to_world 와 같지만 RPY 대신 쿼터니언을 받는다.

    툴을 base z 축으로 추가 회전시키려면 Rx*Ry*Rz 오일러에 각을 더하는 게 아니라
    바깥에서 곱해야 해서, RPY 를 거치지 않는 경로가 필요하다."""
    world_pos = np.array(robot_pos) + quat_to_matrix(robot_quat) @ np.array(tcp_base, dtype=float)
    world_quat = quat_mul(robot_quat, quat_base)
    return world_pos, world_quat / np.linalg.norm(world_quat)


def grasp_frame(grasp_base):
    """파지 자세(쿼터니언)와 접근 방향을 함께 돌려준다"""
    direction, yaw_deg = approach_direction(grasp_base)
    q = quat_mul(quat_from_axis([0, 0, 1], yaw_deg), make_target_quat(*_pnp.POINT2_RPY))
    return direction, q / np.linalg.norm(q)


def build_observe_steps(grasp_base, base_pos, base_quat):
    """1단계 — 트레이 정면 관측 자세로 이동만 한다.

    파지 방향선상에서 OBSERVE_DISTANCE_M 만큼 물러난 곳으로 TCP 를 보낸다.
    카메라는 TCP 보다 TCP_OFFSET 만큼 뒤에 있으므로 실제 관측 거리는 그만큼 더 멀다.
    여기 도착한 뒤 새 검출을 받아 좌표를 다시 잡는다 — 비스듬히 본 값으로 곧장
    파지하면 박스 중심이 트레이 중심과 어긋난다."""
    direction, grasp_quat = grasp_frame(grasp_base)
    observe = grasp_base - direction * OBSERVE_DISTANCE_M
    return [{
        "type": "pose",
        "label": f"정면 관측({OBSERVE_DISTANCE_M * 100:.0f}cm)",
        "target": base_pose_to_world(observe, grasp_quat, base_pos, base_quat),
        "gripper": "open",
    }]


def build_vision_steps(grasp_base, slot_index, base_pos, base_quat):
    """2단계 — 정면에서 다시 잰 좌표로 집어 랙 slot_index 자리에 넣는다.

    pick_and_place.build_sequence 와 같은 형태지만 POINT1/POINT2(파지 지점)만
    검출값으로 갈아끼운다. 나머지 기하는 이미 검증된 값을 그대로 쓴다."""
    def to_world(tcp, rpy):
        return _pnp.base_to_world(tcp, rpy, base_pos, base_quat)

    # 트레이 방향을 마주보도록 기준 파지자세를 base z 축으로 돌린다
    direction, grasp_quat = grasp_frame(grasp_base)

    def to_world_q(tcp):
        return base_pose_to_world(tcp, grasp_quat, base_pos, base_quat)

    # 접근도 같은 방향으로 물러난다 (기존 POINT1->POINT2 는 +y 였다)
    approach = grasp_base - direction * APPROACH_BACKOFF_M
    lift     = np.array([grasp_base[0], grasp_base[1], LIFT_Z_M])
    place = RACK_SLOTS[slot_index]
    retreat = place + RACK_RETREAT_OFFSET

    joint1_delta = np.zeros(6)
    joint1_delta[0] = np.radians(rack_yaw_delta_deg(grasp_base, place))

    return [
        {"type": "pose",  "label": "접근 위치", "target": to_world_q(approach), "gripper": "open"},
        {"type": "pose",  "label": "파지 위치", "target": to_world_q(grasp_base), "gripper": None},
        # 드라이브가 한 스텝에 목표를 못 따라와 몇 cm 못 미친 채로 그리퍼가 닫힌다.
        # 같은 목표를 한 번 더 줘서 남은 오차를 마저 밀어넣는다 (기존 코드에서 배운 것)
        {"type": "pose",  "label": "파지 정착", "target": to_world_q(grasp_base), "gripper": None},
        {"type": "hold",  "label": "그리퍼 닫기", "gripper": "close"},
        {"type": "pose",  "label": "들어올리기", "target": to_world_q(lift), "gripper": None},
        {"type": "joint", "label": f"joint_1 {np.degrees(joint1_delta[0]):+.1f}deg",
         "target": lambda start: start + joint1_delta, "gripper": None},
        {"type": "pose",  "label": f"랙 {slot_index + 1}번 적재", "target": to_world(place, RACK_RPY), "gripper": None},
        {"type": "hold",  "label": "그리퍼 열기", "gripper": "open"},
        {"type": "pose",  "label": "후퇴", "target": to_world(retreat, RACK_RPY), "gripper": None},
        {"type": "joint", "label": "홈 복귀", "target": np.array(READY_JOINTS_RAD, dtype=float), "gripper": None},
    ]


def first_tray_grasp(color_camera_path, base_pos, base_quat):
    """가장 왼쪽 트레이의 파지점(base)을 구한다. 없으면 None.

    깊이 범위를 여기서 한 번 더 본다 — 팔이 실제로 움직이는 건 이쪽이라
    받은 값을 그대로 믿으면 안 된다."""
    _seq, detections = read_detections(verbose=True)
    for i, (point_optical, conf) in enumerate(detections):
        depth = float(point_optical[2])
        if not (DETECT_DEPTH_MIN_M <= depth <= DETECT_DEPTH_MAX_M):
            print(f"   트레이{i + 1} 무시 — 깊이 {depth:.3f}m 가 "
                  f"{DETECT_DEPTH_MIN_M}~{DETECT_DEPTH_MAX_M}m 범위 밖이다")
            continue
        tray_world = optical_to_world(point_optical, color_camera_path)
        grasp = grasp_point_base(tray_world, base_pos, base_quat)
        reach = float(np.linalg.norm(grasp))
        if not (GRASP_REACH_MIN_M <= reach <= GRASP_REACH_MAX_M):
            print(f"   트레이{i + 1} 무시 — 파지점이 base 에서 {reach:.3f}m "
                  f"({GRASP_REACH_MIN_M}~{GRASP_REACH_MAX_M}m 밖). 벽/배경 오검출로 본다")
            continue
        print(f"   트레이 conf {conf:.2f}  cam {vec(point_optical, 3)}  "
              f"world {vec(tray_world)}  파지(base) {vec(grasp, 4)} "
              f"reach {reach:.3f}m  yaw {approach_direction(grasp)[1]:+.1f}deg")
        return grasp, tray_world
    return None


def all_tray_worlds(color_camera_path, base_pos, base_quat):
    """V(확인 전용)용 — 검출된 트레이 전부의 월드 좌표"""
    _seq, detections = read_detections()
    if not detections:
        print("   검출 없음 — 관제 PC 가 /tray_detection 을 보내고 있는지 확인할 것")
    out = []
    for i, (point_optical, conf) in enumerate(detections):
        depth = float(point_optical[2])
        if not (DETECT_DEPTH_MIN_M <= depth <= DETECT_DEPTH_MAX_M):
            print(f"   트레이{i + 1} 무시 — 깊이 {depth:.3f}m 범위 밖")
            continue
        tray_world = optical_to_world(point_optical, color_camera_path)
        grasp = grasp_point_base(tray_world, base_pos, base_quat)
        print(f"   트레이{i + 1} conf {conf:.2f}  cam {vec(point_optical, 3)}  "
              f"world {vec(tray_world)}  파지(base) {vec(grasp, 4)} "
              f"yaw {approach_direction(grasp)[1]:+.1f}deg")
        out.append(tray_world)
    return out


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


# 손목 특이점 대응. joint 5 가 0 에 가까우면 joint 4 와 6 의 축이 일직선이 되어
# 자코비안이 퇴화하고, 현재 자세를 warm start 로 줘도 IK 가 다른 해로 넘어간다.
MAX_IK_JOINT_JUMP_DEG = 20.0   # 한 프레임에 이보다 크게 튀는 해는 버린다
WRIST_NEAR_SINGULAR_DEG = 8.0  # |joint 5| 가 이보다 작으면 경고만 찍는다


class SingularityGuardedIK:
    """IK 해가 한 프레임에 크게 튀면 미해결로 돌려보내는 래퍼.

    실행기와 텔레오퍼레이션 모두 compute_inverse_kinematics 만 호출하므로 그대로
    끼워 넣을 수 있다. 버려진 프레임은 지령이 나가지 않아 팔이 뒤틀리지 않는다.
    대신 그 스텝은 목표에 못 미친 채 끝나므로, 경고 횟수를 같이 찍는다."""

    def __init__(self, ik_solver, robot, arm_indices):
        self._ik = ik_solver
        self._robot = robot
        self._arm_indices = arm_indices
        self._max_jump = np.radians(MAX_IK_JOINT_JUMP_DEG)
        self.rejected = 0

    def __getattr__(self, name):
        return getattr(self._ik, name)   # compute_end_effector_pose 등은 그대로 통과

    def compute_inverse_kinematics(self, **kwargs):
        action, solved = self._ik.compute_inverse_kinematics(**kwargs)
        if not solved or action.joint_positions is None:
            return action, solved

        q = self._robot.get_joint_positions()
        current = np.array([q[i] for i in self._arm_indices], dtype=float)
        target = np.asarray(action.joint_positions, dtype=float)
        if target.shape != current.shape:
            return action, solved        # 형태를 모르면 검사하지 않는다

        jump = np.abs(target - current)
        worst = int(np.argmax(jump))
        if jump[worst] > self._max_jump:
            self.rejected += 1
            if self.rejected == 1 or self.rejected % LOG_INTERVAL == 0:
                carb.log_warn(
                    f"IK 해가 튐 ({self.rejected}) — joint_{worst + 1} 가 한 프레임에 "
                    f"{np.degrees(jump[worst]):.0f}도. joint_5 = {np.degrees(current[4]):+.1f}도 "
                    f"(0 도 부근이면 손목 특이점)"
                )
            return action, False
        if abs(current[4]) < np.radians(WRIST_NEAR_SINGULAR_DEG):
            carb.log_warn(f"손목 특이점 근처 — joint_5 = {np.degrees(current[4]):+.1f}도")
        return action, solved


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
        # 자세를 RPY 세 숫자가 아니라 쿼터니언으로 들고 있는다.
        # 회전을 카메라 축 기준으로 주려면 월드 오일러각에 더하는 방식으로는 안 된다.
        self.home_quat = make_target_quat(*self.home_rpy)
        self.quat_target = self.home_quat.copy()
        self.step_index = 1          # 기본 2mm
        self.gripper_closed = False
        self.marker_mode = False
        self.points = []
        self.pick_requested = False    # C 를 누르면 True (실제로 움직인다)
        self.verify_requested = False  # V 를 누르면 True (좌표만 확인, 안 움직인다)

    @property
    def step(self):
        return STEP_CHOICES[self.step_index]

    @property
    def rpy(self):
        """기록/로그용. 쿼터니언에서 역산한다"""
        return quat_to_rpy(self.quat_target)

    @property
    def roll(self):
        return self.rpy[0]

    @property
    def pitch(self):
        return self.rpy[1]

    @property
    def yaw(self):
        return self.rpy[2]

    def quat(self):
        return self.quat_target

    def state(self):
        return (self.tcp.copy(), self.quat_target.copy())

    def restore(self, state):
        """IK 가 못 풀면 방금 준 입력을 무른다"""
        self.tcp, self.quat_target = state[0], state[1]

    def home_pose(self):
        self.tcp = self.home.copy()
        self.quat_target = self.home_quat.copy()

    def jog(self, kb, cam_right, cam_up, cam_forward):
        """눌려 있는 키를 읽어 카메라 시점 기준으로 목표를 옮긴다
        W/S 카메라 상하, A/D 카메라 좌우, Q/E 카메라 전후"""
        d, r = self.step, ROT_STEP_DEG
        for key, axis, sign in (("W", cam_up, +1), ("S", cam_up, -1),
                                ("D", cam_right, +1), ("A", cam_right, -1),
                                ("Q", cam_forward, -1), ("E", cam_forward, +1)):
            if kb.held(key):
                self.tcp += sign * d * axis

        # 회전도 카메라 축 기준으로 준다. 월드 오일러각을 증감하면 팔이 돌아간 만큼
        # 체감 방향이 어긋나서, 초기 자세에서 J/L 과 I/K 가 뒤바뀐 것처럼 느껴진다.
        # cam_forward 는 RSD455 로컬 +X 라 실제 시선의 반대다 (USD 에서 내적 -1 확인)
        cam_view = -cam_forward
        for key, axis, sign in (("J", cam_up,    +1), ("L", cam_up,    -1),   # yaw
                                ("U", cam_view,  +1), ("O", cam_view,  -1),   # roll
                                ("I", cam_right, +1), ("K", cam_right, -1)):  # pitch
            if kb.held(key):
                q = quat_mul(quat_from_axis(axis, sign * r), self.quat_target)
                self.quat_target = q / np.linalg.norm(q)


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
        elif key == "V":
            teleop.verify_requested = True
        elif key == "C":
            teleop.pick_requested = True
            print("   pick         최신 검출로 pick & place 시작")
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

    section("ROS2")
    color_camera_path = find_color_camera()
    print(f"   color cam    {color_camera_path}")
    build_detection_graph(color_camera_path)

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
    # 검출 좌표 확인용. teleop 마커(초록)와 달리 IK 목표가 아니라 표시 전용이다
    detect_marker = world.scene.add(
        VisualCuboid(
            prim_path="/World/detect_marker",
            name="detect_marker",
            position=np.array([0.0, 0.0, -1.0]),   # 처음엔 바닥 아래로 숨겨둔다
            size=0.03,
            color=np.array([0.9, 0.1, 0.1]),
        )
    )
    # 랙 자리 확인용. base 기준 좌표가 월드 어디에 떨어지는지 눈으로 본다
    def marker_row(prefix, color, size):
        return [
            world.scene.add(
                VisualCuboid(
                    prim_path=f"/World/{prefix}_{i}",
                    name=f"{prefix}_{i}",
                    position=np.array([0.0, 0.0, -1.0]),   # 처음엔 바닥 아래로 숨겨둔다
                    size=size,
                    color=np.array(color),
                )
            )
            for i in range(len(RACK_SLOTS))
        ]

    rack_markers    = marker_row("rack_marker",    [0.1, 0.3, 0.9], 0.03)   # 파랑 = 놓는 위치
    retreat_markers = marker_row("retreat_marker", [0.9, 0.8, 0.1], 0.02)   # 노랑 = 후퇴 위치

    print(__doc__)
    section("RUN")
    print("   뷰포트를 클릭해 포커스를 준 뒤 Play 를 누른다\n")

    kb = Keyboard()
    arm_indices = [robot.get_dof_index(j) for j in ARM_JOINTS]
    # 이후의 텔레오퍼레이션과 시퀀스 실행이 모두 이 가드를 거친다
    ik_solver = SingularityGuardedIK(ik_solver, robot, arm_indices)
    # pick & place 상태 기계. None 이면 수동 텔레오퍼레이션.
    #   scan   -> 새 검출을 기다렸다가 관측 자세 계획
    #   observe-> 관측 자세로 이동 중
    #   settle -> 정면 도착. 그 자리에서 새로 찍은 검출을 기다린다
    #   grasp  -> 파지 + 랙 적재 실행 중
    sequence = None
    pick_state = None
    pick_slot = 0
    seq_mark = -1
    wait_frames = 0
    was_playing = False
    step = fail_streak = 0

    # Ctrl+C 로 끄면 simulation_app.close() 를 못 거치고 atexit 로 직행해서, render product 와
    # replicator writer 가 붙은 채로 omni.graph / syntheticdata 가 해제되며 segfault 가 난다.
    # 실제 오류 트레이스백까지 같이 묻히므로 받아서 정리한다.
    try:
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

            if teleop.verify_requested:
                teleop.verify_requested = False
                section("VERIFY (팔은 움직이지 않는다)")
                base_pos, base_quat = arm_base_pose()
                found = all_tray_worlds(color_camera_path, base_pos, base_quat)
                for i, place in enumerate(RACK_SLOTS):
                    retreat = place + RACK_RETREAT_OFFSET
                    world_place, _ = _pnp.base_to_world(place, RACK_RPY, base_pos, base_quat)
                    world_retreat, _ = _pnp.base_to_world(retreat, RACK_RPY, base_pos, base_quat)
                    rack_markers[i].set_world_pose(position=world_place)
                    retreat_markers[i].set_world_pose(position=world_retreat)
                    print(f"   랙 {i + 1}번 놓기  base {vec(place, 4)} -> world {vec(world_place)}")
                    print(f"          후퇴  base {vec(retreat, 4)} -> world {vec(world_retreat)}")
                print("   파랑(큰 것) = 놓는 위치, 노랑(작은 것) = 후퇴 위치. 실제 랙과 겹치는지 볼 것.")
                print("   어긋나면 TCP 를 랙 자리로 몰고 가서 1(기록) -> 2(출력) 로 실제 좌표를 확인한다")
                if found:
                    detect_marker.set_world_pose(position=found[0])
                    print("   빨간 마커를 트레이1 위치에 놓았다. 실제 트레이와 겹치는지 눈으로 볼 것")

            if teleop.pick_requested:
                teleop.pick_requested = False
                if pick_state is not None:
                    print("   pick         이미 실행 중이다")
                else:
                    section("PICK & PLACE")
                    pick_state, pick_slot = "scan", 0
                    seq_mark, wait_frames = read_detections()[0], 0

            # 시퀀스가 도는 동안에는 키 조그와 IK 를 건너뛴다. 둘 다 같은 팔에
            # 지령을 내리므로 겹치면 서로를 덮어쓴다
            if pick_state is not None:
                if pick_state in ("scan", "settle"):
                    seq_now = read_detections()[0]
                    wait_frames += 1
                    if seq_now - seq_mark < FRESH_SEQ_ADVANCE:
                        if wait_frames > FRESH_TIMEOUT_FRAMES:
                            print(f"   pick         새 검출을 {FRESH_TIMEOUT_FRAMES} 프레임 동안 못 받았다 — 중단")
                            pick_state = None
                        continue

                    # 계획 단계에서 터져도 시뮬 전체를 죽이지 않는다. 오류를 찍고 수동으로 돌아간다
                    try:
                        base_pos, base_quat = arm_base_pose()
                        found = first_tray_grasp(color_camera_path, base_pos, base_quat)
                        if found is None:
                            print("   pick         남은 트레이 없음 — 종료")
                            pick_state = None
                            continue
                        # 주의: main() 의 'world' 는 시뮬레이션 World 객체다. 덮어쓰면 다음 루프에서
                        # world.step() 이 numpy 배열의 메서드를 찾다가 죽는다.
                        grasp, tray_world = found
                        detect_marker.set_world_pose(position=tray_world)

                        if pick_state == "scan":
                            print(f"   [1단계] 트레이 정면 {OBSERVE_DISTANCE_M * 100:.0f}cm 지점으로 이동")
                            steps = build_observe_steps(grasp, base_pos, base_quat)
                            next_state = "observe"
                        else:
                            print(f"   [2단계] 정면에서 재측정한 좌표로 파지 -> 랙 {pick_slot + 1}번")
                            steps = build_vision_steps(grasp, pick_slot, base_pos, base_quat)
                            next_state = "grasp"
                        sequence = PickPlaceSequence(robot, ik_solver, arm_indices, steps)
                        sequence.reset()
                        pick_state = next_state
                    except Exception:
                        import traceback
                        print("   pick         계획 실패 — 아래 오류를 보고할 것")
                        traceback.print_exc()
                        sequence, pick_state = None, None
                    continue

                sequence.tick()
                if sequence.done:
                    sequence = None
                    seq_mark, wait_frames = read_detections()[0], 0
                    if pick_state == "observe":
                        pick_state = "settle"          # 도착. 이 자리에서 다시 잰다
                    else:
                        pick_slot += 1
                        if pick_slot >= len(RACK_SLOTS):
                            print("   pick         랙이 다 찼다 — 종료")
                            pick_state = None
                            teleop.home_pose()
                        else:
                            pick_state = "scan"        # 다음 트레이를 다시 찾는다
                continue

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

    except KeyboardInterrupt:
        print("\n   중단됨 (Ctrl+C)")

    print_records(teleop, robot)
    kb.close()
    # 그래프를 먼저 지워 render product / writer 를 떼고 나서 앱을 닫는다
    try:
        world.stop()
        omni.usd.get_context().get_stage().RemovePrim(DETECT_GRAPH_PATH)
        for _ in range(5):
            simulation_app.update()
    except Exception as exc:
        print(f"   정리 중 오류 (무시): {exc}")
    simulation_app.close()


if __name__ == "__main__":
    main()
