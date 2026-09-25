"""
Pick & Place + 손목 카메라 검출 — pick_and_place.py 에 검출 연동을 붙인 판.

    source /opt/ros/jazzy/setup.bash
    isaac_python pick_and_place_detection.py

pick_and_place.py 와 달리 파지 좌표(POINT1/POINT2)를 손으로 찾은 값 대신 검출로
구한다. 대신 place 좌표(POINT4/POINT5)는 그대로 고정값을 쓴다 — 이번 변경은
"pick 쪽만" 검출을 쓰라는 요청에 맞춘 것이다.

Play 를 누르면 아래 순서를 자동으로 수행한다.

    0 joint_1 을 SCAN_ROTATE_DEG 만큼 회전하고, z 를 SCAN_DESCEND_Z_M 까지 낮춰
      손목 카메라로 트레이를 본다 (도착 후 정지, 새로 찍힌 검출 프레임을 기다린다)
    1 검출점의 카메라 광학 x 만큼 카메라 기준 수평으로 이동해 ROI 를 화면
      중앙으로 맞춘다 (도착 후 정지, 다시 새 검출 프레임을 기다린다)
    2 중앙 정렬 후 다시 검출한 좌표(depth 포함)로 파지 위치/깊이를 확정하고,
      트레이 앞에서 접근 위치로 이동 (그리퍼 열기)
    3 파지 위치로 이동
    4 그리퍼 닫기
    5 들어올리는 위치로 이동
    6 joint_1 을 파지 방향 -> place 방향 차이만큼 회전 (고정각 아님, rack_yaw_delta_deg 로 계산)
    7 놓는 위치(POINT4, 고정값)로 이동
    8 그리퍼 열기
    9 후퇴 안전 위치(POINT5, 고정값)로 이동
    10 홈 관절 각도로 복귀

검출은 관제 PC 의 admin_ws/src/tray_detector 가 맡는다. ultralytics 는 python3.12,
Isaac Sim 은 자체 python3.11 이라 한 프로세스에 합칠 수 없다.

주행(Nav2)까지 묶은 전체 시나리오 — 프로세스 3개다 (rclpy 를 번들 파이썬에서 못 쓴다).

    1) 관제 PC 검출 노드     detect_node.py            (~/yolo-venv)
    2) 이 파일               isaac_python              씬 + 사람 + 팔 + 후방 라이다
    3) Nav2 스택 + 주행 미션 ros2 launch carter_navigation nav2_human_test.launch.py
                             ros2 run nav_to_goal through_pose_human_test

    적재(트레이 3개) -> 랙 ArUco 관측 -> 팔 홈 복귀 -> /mission_state 1 발행
      -> (주행 프로세스가 undock -> goThroughPoses -> dock) -> /nav_done 1 수신 -> 하역

이 파일이 run_human_scene.py 의 일(사람 확장/navmesh/후방 라이다)까지 겸한다.
주행만 단독으로 시험할 때는 그쪽을 그대로 쓰면 된다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension

# 검출 그래프 노드를 만들기 전에 켜야 한다
enable_extension("isaacsim.ros2.bridge")

from pathlib import Path
import random
import sys
import time

import carb
import carb.input
import numpy as np
import omni.appwindow
import omni.usd
import omni.graph.core as og
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdSkel
from isaacsim.core.utils.prims import set_targets

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim, SingleXFormPrim
from isaacsim.core.utils.stage import open_stage, is_stage_loading
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

# 이력: integration_human.usd -> hospital_integration_human.usd (2026-09-23, 맵 교체).
# hospital_integration.usd 사본에 integration_human.usd 의 액션 그래프 4개
# (/World/ActionGraph, /World/nova_carter_ros/{differential_drive,transform_tree_odometry,ros_lidars})
# 를 이식한 것. 트레이/책상/사람은 아직 안 옮겼으므로 --drive-only 로만 실행할 것.
SCENE_USD        = str(M0609_DIR.parent / "assets/hospital_integration_human.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

# 씬 USD(integration_human / hospital_integration_human 공통): 카터와 팔이 하나의 아티큘레이션이다.
#   - 드라이브 설정 / EE / 카메라 검색은 팔이 들어 있는 nova_carter 서브트리 기준
#   - Articulation 등록은 실제 루트인 chassis_link 기준 (바퀴로 주행 가능하려면 필요)
#   - IK 기준 프레임은 팔의 base_link 다 (chassis_link 가 아니다)
ROBOT_PRIM_PATH = "/World/robot_nova/nova_carter"
ART_ROOT_PATH   = ROBOT_PRIM_PATH + "/chassis_link"
ARM_BASE_PATH   = ROBOT_PRIM_PATH + "/Robot/m0609_camera/m0609/base_link"
EE_LINK_NAME    = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# ── 주행(Nav2)용 씬 설정 — isaacpjt/pjt_alpha/run_human_scene.py 에서 가져왔다.
# 이 파일 하나로 적재/주행/하역을 다 돌리므로 run_human_scene.py 를 따로 띄우지 않는다
# (그쪽은 주행만 단독 시험할 때 계속 쓴다). 값이 바뀌면 두 파일 다 고칠 것.
PEOPLE_EXTENSIONS = [
    "omni.anim.people",
    "omni.anim.navigation.bundle",
    "omni.anim.timeline",
    "omni.anim.graph.bundle",
    "omni.anim.graph.core",
    "omni.anim.graph.ui",
    "omni.anim.retarget.bundle",
    "omni.anim.retarget.core",
    "omni.anim.retarget.ui",
    "omni.kit.scripting",
]
PEOPLE_COMMAND_FILE = str(M0609_DIR.parent / "assets/people/command.txt")
# 캐릭터에 애니메이션 그래프 + behavior 스크립트를 붙일 위치. 씬 USD 에는 이게 안 들어 있다.
CHARACTERS_ROOT     = "/World/Characters"
BIPED_SETUP_PATH    = CHARACTERS_ROOT + "/Biped_Setup"

# 후방 2D 라이다(SLAMTEC RPLIDAR S2E). Nav2 params 의 local/global costmap 이 둘 다
# /scan_rear 를 본다 — 이게 없으면 뒤쪽 장애물이 costmap 에 안 찍힌다.
REAR_RPLIDAR_PATH = ROBOT_PRIM_PATH + "/chassis_link/sensors/rear_RPLidar"
REAR_TOPIC        = "scan_rear"
REAR_FRAME        = "rear_rplidar"   # 발행 frameId 와 TF 프레임 이름이 같아야 한다
TF_SENSORS_NODE   = "/World/nova_carter_ros/transform_tree_odometry/tf_sensors"

# RSD455 는 Xform 이고 실제 렌더링되는 컬러 카메라는 그 아래 프림이다
D455_CAMERA_NAME       = "RSD455"
D455_COLOR_CAMERA_NAME = "Camera_OmniVision_OV9782_Color"

# ROS2 — rgb/depth/camera_info 를 관제 PC 로 보내고, 검출 결과를 돌려받는다
DETECT_GRAPH_PATH = "/World/DetectGraph"
COLOR_TOPIC       = "wrist_camera/color/image_raw"
DEPTH_TOPIC       = "wrist_camera/depth/image_raw"
INFO_TOPIC        = "wrist_camera/color/camera_info"
RESULT_TOPIC      = "tray_detection"
MARKER_TOPIC      = "aruco_markers"
# 주행 프로세스(nav_to_goal/through_pose_human_test.py)와의 손잡기.
#   MISSION_TOPIC  Isaac -> 주행 :  1 = 적재+관측 끝났다, 출발해도 된다
#   NAV_DONE_TOPIC 주행 -> Isaac :  1 = 도킹까지 끝났다, 하역해도 된다
# rclpy 를 못 쓰니(번들 파이썬 3.11) 검출과 같은 제네릭 ROS2 노드로 주고받는다.
MISSION_TOPIC     = "mission_state"
NAV_DONE_TOPIC    = "nav_done"
IMAGE_RESOLUTION  = (640, 480)
IMAGE_FRAME_SKIP  = 4   # 매 프레임 발행하면 네트워크/렌더 둘 다 버겁다

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
# Lula 는 축별 구속을 못 한다 — 이 값은 x/y/z 축에 똑같이 걸린다. 키우면 손목 특이점 근처에서
# 해를 찾을 여지가 생기는 대신 트레이가 그만큼 기운 채로도 성공 판정이 난다.
# 이력: 0.02 -> 0.1 -> 0.1745 (10도) -> 0.1222 (7도). STEP_ROT_TOL_DEG 를 이보다 높게 유지할 것
IK_ORIENTATION_TOLERANCE = 0.1222   # rad = 7도


# ══════════════════════════════════════════════════════════════
#  Pick & Place 목표 좌표 — pnp_teleop.py 로 손으로 찾은 값 (base 기준)
# ══════════════════════════════════════════════════════════════
POINT1_TCP = np.array([-0.0093, 0.7190, 0.2776])   # 잡기 전 안전 위치
POINT1_RPY = (-89.3, 0.1, 180.0)

POINT2_TCP = np.array([-0.0091, 0.8007, 0.2776])   # 파지 위치
POINT2_RPY = (-89.3, 0.1, 180.0)

POINT3_TCP = np.array([-0.0088, 0.7989, 0.4496])   # 들어올린 위치
POINT3_RPY = (-89.3, 0.1, 180.0)

# 이 값들은 원래 PnP_test.usd(고정 로봇, 카트 없음)에서 손으로 찾은 좌표라
# integration_human.usd(nova_carter) 로 바꾸면 물리적으로 의미가 없어진다(로봇
# 팔 자체가 다른 데 달려 있다). 대신 같은 nova_carter 마운트에서 실측된
# pnp_teleop_detection.py 의 랙 가운데 슬롯 좌표를 그대로 가져왔다 —
# RACK_CENTER_X/RACK_ROW_Y/RACK_ROW_Z/RACK_RETREAT_OFFSET, base 기준.
# 정확한 자리가 아니면 pnp_teleop_detection.py 의 V 키로 확인하거나 1/2 키로
# 다시 실측해서 이 두 값만 바꿀 것.
# y 이력: 0.7690 -> 0.8190(+5cm) -> 0.7590(-6cm, 테두리 충돌) -> 0.7890(+3cm).
# 놓기 위 안전 위치는 POINT4_TCP 에서 z 만 올려 만들므로 자동으로 같이 움직인다.
# 후퇴점(POINT5)도 같은 양만큼 옮겨 수평 후퇴 15cm 를 유지한다.
# z 이력: 0.2500 -> 0.2600(+1cm, 그리퍼 열 때 트레이 내부 형상과 충돌해서) -> 0.2700(+1cm)
POINT4_TCP = np.array([-0.0093, 0.7890, 0.2700])    # 놓는 위치 (랙 가운데 슬롯)
POINT4_RPY = (-89.3, 0.1, 180.0)

POINT5_TCP = np.array([-0.0093, 0.6390, 0.2700])    # 놓고 후퇴하는 안전 위치 (수평 15cm 후퇴, z 는 POINT4 와 같게)
POINT5_RPY = (-89.3, 0.1, 180.0)

POINT6_TCP = np.array([-0.0093, 0.7690, 0.25])   # 처음에 내려놓는 위치 (base +y 로 50mm 더 안쪽)
POINT6_RPY = (-89.3, 0.1, 180.0)

# 랙 3칸 (base 기준). POINT4 가 가운데(2번)이고, 칸은 base x 축으로 늘어선다
# (씬 실측: 칸막이가 x = -0.233 / -0.069 / +0.094 / +0.258, 간격 ~0.163).
# 1번 = x -0.165, 3번 = x +0.165. y 는 접근(깊이) 축이라 칸별 보정으로만 건드린다
RACK_PITCH_M = 0.165
# 칸별 x 보정 (칸이 늘어선 방향). USD 칸막이로 계산한 기하 중심이 세 칸 모두 +x 로 ~2cm 라,
# 3번(테두리에 닿았다)에 이어 1/2번도 같은 값으로 맞췄다. 세 칸이 같은 값이지만 칸마다
# 따로 재서 넣는 자리이므로 튜플로 둔다
RACK_SLOT_X_TRIM = (0.02, 0.02, 0.02)
RACK_SLOTS   = [POINT4_TCP + np.array([i * RACK_PITCH_M + t, 0.0, 0.0])
                for i, t in zip((-1, 0, 1), RACK_SLOT_X_TRIM)]
RACK_RETREAT = POINT5_TCP - POINT4_TCP          # 놓은 뒤 수평 15cm 후퇴, 슬롯마다 같은 양

# ── 랙에 놓인 트레이 정렬
# 설정: 랙 칸에 정렬 가이드(자석)가 있어, 트레이가 칸에 들어가면 칸 중앙·칸 방향으로 붙는다.
# 그 결과를 트레이 프림 pose 에 직접 반영한다 — 팔 동작은 건드리지 않는다.
#
# IK_ORIENTATION_TOLERANCE 를 7도까지 푼 뒤로 트레이가 기운 채 "성공" 판정이 나서
# 랙에 비스듬히 올라간다. 팔로 고치려면 그 원인인 IK 를 다시 믿어야 해서 수렴하지 않는다.
#
# 목표는 **절대 기준**이다 — 그리퍼가 트레이를 어떻게 물고 있었는지는 보지 않는다.
#   자세  스폰 자세(씬이 만든 반듯한 자세)를 base z 로 90도의 정수배만큼 돌린 것 중
#         지금 자세에 가장 가까운 하나. 정수배는 지금 자세에서 읽으므로 상수가 필요 없다
#   위치  슬롯 중심(RACK_SLOTS[slot]) 의 xy. z 는 물리가 잡아 준 값을 그대로 둔다
#
# 이력(2026-09-24): 처음엔 '팔이 슬롯에 정확히 도달했다면 트레이가 있었을 자리'로 잡아서
# 그리퍼에 물린 상대 자세를 그대로 얹었다. 놓을 때의 IK 오차는 지워졌지만 **트레이가
# 그리퍼에 삐뚤게 물린 것이 그대로 남아** 랙에서 여전히 기울어 보였다. 물린 각도를 아예
# 안 보는 지금 방식으로 바꿨다 — 놓은 뒤에 절대 기준으로 세우는 게 요구사항이었다.
RACK_TRAY_ALIGN     = True
# 그리퍼(TCP) xy 에서 이 반경 밖이면 물고 있는 게 아니라고 보고 정렬을 건너뛴다. 놓기
# 스텝이 실패해 건너뛰어졌거나 트레이를 놓친 경우를 거른다 — 엉뚱한 트레이를 옮기지 않는다.
# 칸 간격(0.165)의 절반보다 작게 둬서 옆 칸 트레이가 후보로 들어올 여지도 없앤다
RACK_TRAY_MATCH_R_M = 0.08

# 시작 시 원본 트레이(/World/tray)를 이만큼 더 복제해 흩뿌린다. 자세는 원본 그대로,
# 위치는 원본을 중심으로 **가로 방향 한 줄**(base->트레이 방향에 수직)로 ±TRAY_SPREAD_R_M.
# 이력: +y 쪽 반원 -> 가로 한 줄. 반원은 파지점 도달거리를 0.69~1.09m 로 퍼뜨려서
# 먼 쪽이 손목 특이점(joint_5 ~ 0)에 걸렸다. 가로로만 벌리면 0.91~0.94m 로 모인다.
TRAY_PRIM_PATH   = "/World/tray"
TRAY_COPIES      = 2
# 이력: 0.20 -> 0.25. 한 줄 배치는 2차원보다 자리가 빠듯해서, 0.20 이면 TRAY_MIN_GAP_M
# 을 만족하는 자리를 20회 안에 못 찾는 실행이 7% 였다 (0.25 면 1%)
TRAY_SPREAD_R_M  = 0.25
TRAY_MIN_GAP_M   = 0.15     # 트레이끼리 이보다 가까우면 다시 뽑는다 (겹치면 물리로 튄다)

# 긴급도 ArUco 마커 — 트레이마다 id 0/1/2(하/중/상)를 무작위로 골라 손잡이 윗판에 붙인다.
# 적재/하역과 무관. 랙 적재가 끝난 뒤 랙을 대각선 위에서 관측해 칸별 긴급도만 출력한다.
# 윗판 = handle/Cube_01: handle 기준 x ±0.025, y ±0.05, 윗면 z = 0.1449 + 0.0025 (실측, tray.usd)
ARUCO_DIR        = str(M0609_DIR.parent / "assets/markers")   # make_markers.py 출력
ARUCO_IDS        = (0, 1, 2)
ARUCO_SIDE_M     = 0.049    # 흰 여백 포함 한 변. 검은 마커 = *6/7 = 0.042 (solvePnP markerLength)
ARUCO_HANDLE_REL = "tray/test_tube_rack/handle"
ARUCO_TOP_Z_M    = 0.1474 + 0.0005   # 윗판 윗면 + z-fighting 방지 여유
URGENCY_NAMES    = {0: "하", 1: "중", 2: "상"}

# 적재 완료 후 랙 관측 자세. **관절 공간 고정값** — 마커가 실제로 읽힌 자세에서 그대로 떠 왔다
# (2026-09-23 실측). 예전엔 base 기준 TCP 좌표 + 50도 숙임을 IK 로 풀었는데, 그 목표에 도달을
# 못 해 실제 자세가 계획보다 10cm 낮은 곳에서 끝났다 — IK 가 어디까지 가느냐에 따라 관측 자세가
# 매번 달라졌다는 뜻이다. 관절값으로 박으면 IK 를 안 타니 실패할 수도, 달라질 수도 없다.
# FK 역검산: TCP(base) = [-0.0138, +0.4991, +0.4680], 툴 하향각 50.4도.
# **랙이나 로봇 위치가 바뀌면 이 값은 의미가 없다** (base 기준 좌표가 아니라 관절각이다).
# 실측값은 j4=-175.5 / j5=-80.7 / j6=+265.5 였는데, 손목 플립 등가해
# (j4+180, j5 부호 반전, j6-180) 로 바꿔 적었다 — 말단 자세가 보존되는 변환이라
# TCP 오차 0.01cm / 자세 오차 0.03도로 사실상 같은 자세이고, 마커 판독 조건도 그대로다.
# 홈에서 오는 최대 관절 변화가 175.6 -> 115.0도로 줄어 이동이 5.8초 -> 3.8초가 된다
# (손목을 180도씩 두 번 돌리던 동작이 사라진다).
OBSERVE_JOINTS_DEG = [+87.895, -4.758, +64.720, +4.469, +80.661, +85.544]
# 마커가 5px/셀 근처라 프레임마다 0~2개로 깜빡인다. 한 장만 믿지 말고 첫 신선 프레임부터
# 이만큼(시뮬 프레임, 60Hz 가정 ≈1.5초) 누적해서 칸별로 가장 많이 나온 id 를 쓴다
OBSERVE_COLLECT_FRAMES = 90

# 보간 속도 — 스텝당 이동량을 고정하고 구간 길이로 스텝 수를 정한다.
# 이력: 2026-09-23 전 구간 2.5배 (0.004/0.5/60 -> 0.010/1.25/24).
# **MIN_STEPS 를 같이 내리는 게 핵심이다** — 이동 0.24m 미만 / 회전 30도 미만 구간은
# 전부 이 바닥에 걸려 있어서, 속도만 올리면 짧은 구간은 하나도 안 빨라진다.
TCP_SPEED_M        = 0.010   # m / step
JOINT_SPEED_DEG     = 1.25   # deg / step
MIN_STEPS           = 24
MAX_STEPS           = 600

# 스텝 dict 의 "speed" 키로 등급을 고른다. 키가 없으면 기본(빈손).
# 이력: 예전엔 "slow": True 불리언 하나였는데, 트레이를 들고 있는 구간이 따로 필요해져
# 이름 있는 3단으로 바꿨다 (불리언을 하나 더 붙이면 조합이 금방 엉킨다).
#   None    빈손. 2.5배. 흔들릴 게 없으니 가장 빠르게
#   carry   트레이를 들고 있는 구간. 1.5배 + "ease": 2 로 감가속을 부드럽게.
#           2.5배로 joint_1 을 최대 86도 돌리면 들고 있는 트레이가 흔들린다 (실측 증상)
#   slow    그리퍼 여닫기 직전/직후. 원래 속도. 접촉 순간은 빠르면 트레이를 치거나 놓친다
SPEEDS = {
    None:    (TCP_SPEED_M, JOINT_SPEED_DEG, MIN_STEPS),   # 2.5배
    "carry": (0.006, 0.75, 40),                           # 1.5배
    "slow":  (0.004, 0.50, 60),                           # 1.0배
}
GRIPPER_WAIT_STEPS  = 120    # 그리퍼가 실제로 여닫힐 때까지 제자리에서 기다리는 스텝 수
# 60Hz 가정(GRIPPER_WAIT_STEPS=120 이 약 2초인 것과 같은 기준) — 랙에 내려놓기
# 전 흔들림이 가라앉을 시간을 준다. 1초 = 60 스텝
# 이력: 60(1초) -> 120(2초). 보간 속도를 2.5배로 올린 뒤 아직 흔들리는 중에 하강했다.
# 이름 이력: RACK_PLACE_WAIT_STEPS -> PLACE_WAIT_STEPS. 적재(랙)와 하역(책상) 양쪽의
# 수직 하강 직전에 같이 쓴다 — 두 자리 모두 같은 원인(도착 감속 충격)으로 흔들린다.
PLACE_WAIT_STEPS = 120
# 놓는 위치 바로 위 안전 지점의 높이(= 수직으로 내려가는 거리). 대각선으로 진입하면
# 트레이가 랙 테두리에 걸려서, 슬롯 위로 먼저 간 뒤 수직으로만 내려가게 한다.
# 이력: 0.10 -> 0.11 (안전 위치가 너무 낮아서, POINT4 z +1cm 와 합쳐 총 +2cm)
RACK_ABOVE_Z_M = 0.11
# 하역에서 꺼낼 때 들어올리는 높이. 적재와 따로 둔다 — 넣을 때와 뺄 때 걸리는 조건이 달라서
# 한쪽을 맞추면 다른 쪽이 틀어진다. 이력: RACK_ABOVE_Z_M 공용 0.11 -> 전용 0.12 (+1cm)
UNLOAD_ABOVE_Z_M = 0.12
LOG_INTERVAL        = 60


# ══════════════════════════════════════════════════════════════
#  검출 연동 튜닝값
# ══════════════════════════════════════════════════════════════
# 이전 씬(PnP_test.usd, 고정 로봇) 기준으로는 +90(반시계) 이 이론상 맞았으나,
# integration_human.usd(nova_carter) 로 바꾸면서 실측 결과 -90 이 픽업 방향과
# 맞는다고 확인됐다. 고정 로봇과 카터 마운트는 서로 다른 물리 배치라 부호가
# 달라지는 게 이상한 일은 아니다 — 씬을 또 바꾸면 이 부호도 다시 확인해야 한다.
SCAN_ROTATE_DEG = -90.0
# joint_1 회전만으로는 카메라 높이/각도가 안 맞다. 회전이 끝난 뒤 TCP 를 이
# 높이까지 낮춘다 (base 기준 절대 z). 0.11 은 다른(고정 로봇) 씬의 place 높이를
# 빌려온 추측값이었는데, 그 값으로도 카메라는 여전히 책상보다 위(벽)를 보고
# 있었다 — 저장된 검출 이미지로 확인함. integration_human.usd 의 트레이 실측
# base 기준 z(약 0.11, tray world pos (2.033,4.194,0.820)를 base_link pose 로
# 변환해서 구함)로 맞춘다. 그래도 시선 각도 자체는 안 바뀌므로 PITCH_DOWN_DEG
# 로 따로 아래를 보게 기울인다 — 벽을 본 건 위치보다도 각도 문제였다.
SCAN_DESCEND_Z_M = 0.11
# 카메라를 아래로 기울이는 각도. 부호는 수치 검산(카메라 up~world+Z 가정)을
# 거쳤지만 실측은 아니다 — 반대로 기울면 이 부호부터 뒤집을 것.
PITCH_DOWN_DEG = -30.0

# 스캔 자세 도착 후 팔이 흔들리는 시간을 준 뒤에야 새 검출을 받는다
DETECT_SETTLE_FRAMES = 90
# 그 자리에서 '새로 찍힌' 프레임인지는 관제 PC 의 seq 카운터로 판단한다
FRESH_SEQ_ADVANCE    = 2
# 그 안에 새 검출이 안 오면 포기한다 (관제 노드가 죽었을 수 있다)
FRESH_TIMEOUT_FRAMES = 300
# depth 노이즈(배경 오염 등)로 게이트에 걸려 트레이를 못 찾은 경우, 바로 포기하지
# 않고 새 프레임을 다시 기다려서 이만큼 더 시도해본다
MAX_DETECT_RETRIES = 3

# 관제 PC 도 같은 게이트를 두지만, 팔이 실제로 움직이는 건 이쪽이라 한 번 더 본다
DETECT_DEPTH_MIN_M = 0.15
# 정렬 직후 파지 단계에서, 고른 후보가 화면 중앙에서 이만큼 넘게 벗어나 있으면 집지 않는다.
# 근거: 트레이는 가로 한 줄로 ±TRAY_SPREAD_R_M(0.25m), 서로 최소 TRAY_MIN_GAP_M(0.15m)
# 떨어져 있다. 그 절반(7.5cm)을 넘으면 '정렬한 그것'이 아니라 이웃 트레이일 수 있다.
# 실측된 오선택은 23cm 였다. 단위는 픽셀이 아니라 그 깊이에서의 실제 좌우 거리(m)다.
CENTER_TOL_M = 0.08
DETECT_DEPTH_MAX_M = 1.0    # 1.5 -> 1.0: 1.1m 대 벽/연기 오검출 차단. 진짜 트레이가 걸리면 로그 값 보고 올릴 것
# 파지점이 base 에서 이 범위 밖이면 벽/배경 오검출로 보고 버린다.
# 이 값은 3D norm (z≈0.45 포함) 이다. 수평 거리 = sqrt(gate^2 - 0.45^2).
# 상한 근거: 플랜지 reach 0.9 (URDF 0.411+0.368+0.121) 를 파지 높이(어깨보다 ~0.315 위)에서
# 수평으로 풀면 0.843, TCP 오프셋 0.217 을 더한 수평 이론 최대 1.06 = 3D 1.15.
# 1.12 = 수평 1.03 (뻗음 96%) — 팔꿈치가 거의 직선이라 IK 미수렴/해 튐이 잦을 수 있다.
# 이력: 0.95 -> 1.05 -> 1.12. 자꾸 걸리면 게이트보다 스폰 위치를 로봇 쪽으로 옮길 것
GRASP_REACH_MIN_M  = 0.25
GRASP_REACH_MAX_M  = 1.12

# 검출점(트레이 앞면) 기준 보정 — 실물 보고 맞춘 값. 음수 = 검출점보다 로봇 쪽을 잡는다.
# -0.003 -> 0.017(+2cm) -> 0.047(+3cm) 로 실측 피드백 따라 트레이 쪽으로 계속 당김
TRAY_HALF_DEPTH_M    = 0.057
GRASP_ABOVE_CENTER_M = 0.04     # "센터보다 2cm 위" 를 실측 보정한 값
APPROACH_BACKOFF_M   = 0.082    # 원래 POINT1->POINT2 거리(0.0817m)와 같다
LIFT_Z_M             = POINT3_TCP[2]   # 들어올리는 높이는 원래 값을 그대로 쓴다
# 들어올린 뒤 base 쪽으로 수평 후퇴하는 거리. 잡은 자리(0.9~1.0m)에서 바로 joint_1 을
# 돌리면 회전 반경이 커서 옆 트레이를 쓸고 지나간다. 부딪히면 0.30 으로 올릴 것
LIFT_BACK_M          = 0.25

# 하역: 랙에서 꺼내 책상 위에 가로 일렬로 놓는 자리 (base 기준 고정 좌표).
# 중심 xy 는 원본 트레이 자리 — integration_human.usd 에서 계산한 base 기준 (0.831, 0.050).
# (스캔 회전 -90도가 +x 를 보는 것과 일치. 인수인계 문서 §9 의 (0.369, -0.833) 은 갱신 안 된 값이라 쓰지 말 것)
# 시작 로그 'tray origin' 의 base 값과 맞는지 매 실행 확인할 것.
# 가로 = base->중심 방향의 수직. DESK_LATERAL 은 로봇이 중심을 볼 때 '왼쪽'이다.
# 하역은 3번 슬롯부터인데, 랙(+y)에서 시계방향으로 돌면 왼쪽 자리를 먼저 지나 오른쪽에 닿는다.
# 왼쪽부터 채우면 나중 트레이가 먼저 놓은 것 위를 쓸고 지나가므로 3번(첫 하역) -> 오른쪽,
# 2번 -> 가운데, 1번 -> 왼쪽 순으로 채운다 (k = +1 왼쪽, -1 오른쪽)
DESK_ROW_CENTER = np.array([0.831, 0.050, 0.0])   # z 는 DESK_Z_M 이 대신한다
DESK_PITCH_M    = 0.20
# 내려놓는 TCP 높이(base). None 이면 적재 때 기록한 파지점 z 평균을 쓰고 로그로 찍는다 (측정용).
# 2026-09-22 측정: 파지점 z 0.1266 / 0.1121 / 0.1245, 평균 0.1211 -> 0.121 로 고정
DESK_Z_M        = 0.121
_c = DESK_ROW_CENTER[:2] / np.linalg.norm(DESK_ROW_CENTER[:2])
DESK_LATERAL    = np.array([-_c[1], _c[0], 0.0])
DESK_SLOTS      = [DESK_ROW_CENTER + DESK_LATERAL * DESK_PITCH_M * k for k in (1, 0, -1)]   # [1번=왼쪽, 2번=가운데, 3번=오른쪽]
UNLOAD_KEY      = "U"      # 적재 완료 후 뷰포트에서 이 키를 누르면 3번부터 하역

# --drive-only: 팔을 전혀 안 쓰고 주행만 시험한다. Play 를 누르면 적재/관측을 건너뛰고
# 바로 /mission_state 1 을 발행해 주행 프로세스를 출발시키고, /nav_done 을 받으면 끝낸다.
# 씬·사람·navmesh·후방 라이다·ROS2 그래프는 그대로 필요해서 이 파일을 그대로 쓴다.
# 이 모드에서는 관제 노드(detect_node)를 띄울 필요가 없다.
DRIVE_ONLY = "--drive-only" in sys.argv

# 회전 전 후퇴를 직교(IK) 대신 관절로 한다. 자세를 고정한 채 TCP 를 당기면
# joint_5 가 0 을 지나 손목 특이점을 관통한다 (실측: IK 거부 161틱, 25cm 중 7cm 만 가고 중단).
# 어깨(joint_2)/팔꿈치(joint_3)를 접고 joint_5 로 그만큼 되돌려 트레이 기울기를 유지한다.
# 크기는 실측으로 맞출 값 — 실행 로그 'retreat' 줄의 수평 도달거리 감소량을 보고 조정할 것.
# (부호는 코드가 FK 로 두 방향을 미리 재서 고른다. 반대로 접으면 랙 쪽으로 뻗는다)
RETREAT_SHOULDER_DEG    = 10.0    # joint_2
RETREAT_ELBOW_DEG       = 14.0    # joint_3
# 접으면 TCP 가 따라 내려간다. 하역은 슬롯 위 RACK_ABOVE_Z_M(11cm) 에서 후퇴하므로 낙폭이 크면
# 방금 들어올린 트레이가 랙으로 도로 내려간다 — 하역에서만 이 상한을 걸어 부호를 고른다.
# 적재는 LIFT_Z_M(0.45) 에서 후퇴해 여유가 있어 안 건다(= 기존 동작 유지)
RETREAT_MAX_DROP_M      = 0.03

# 스텝 도달 실패 복구. 흐름을 끊지 않는 게 목표다 — 실패한 스텝은 손목을 풀고 한 번 더,
# 그래도 안 되면 그 단계만 포기하고 다음으로 넘어간다 (미션은 계속된다)
MAX_STEP_RETRIES        = 1
WRIST_UNLOCK_DEG        = 15.0    # joint_5 를 0 에서 띄우는 양. joint_3 로 같은 양을 되돌려 기울기는 유지
MAX_STAGE_FAILS         = 2       # 적재에서 이만큼 실패하면 남은 칸을 포기하고 관측/주행으로 넘어간다

# 손목 특이점(joint_5 부근) 대응. 자코비안이 퇴화하면 IK 가 한 프레임에 크게 튀는
# 해로 넘어가 팔이 뒤틀린다 — 그런 해는 걸러서 그 프레임만 미해결 처리한다
MAX_IK_JOINT_JUMP_DEG   = 20.0
# pose 스텝 도달 확인. 틱 수만 세고 넘기면 IK 가 거부된 스텝(특이점/도달 불가)에서 팔이 제자리에
# 있는데도 다음 스텝(예: joint_1 회전)이 시작된다 — "올린 뒤 후퇴 없이 회전" 증상. 오차가 크면
# 목표를 계속 주며 STEP_EXTRA_TICKS 더 기다리고, 그래도 못 가면 시퀀스를 중단한다(계속 가면 충돌).
STEP_POS_TOL_M          = 0.01
# IK_ORIENTATION_TOLERANCE(0.1222rad = 7도) 보다 높아야 한다. 낮으면 IK 는 풀렸는데
# 도달 검사에서만 걸리는 실패가 생긴다. 이력: 5.0 -> 7.0 -> 12.0
STEP_ROT_TOL_DEG        = 12.0
STEP_EXTRA_TICKS        = 120
# 바닥에 닿는 스텝(랙/책상 수직 하강)은 트레이가 먼저 닿아 목표보다 위에서 멈추는 게 정상이라 느슨하게
STEP_CONTACT_TOL_M      = 0.03
WRIST_NEAR_SINGULAR_DEG = 8.0
# 파지 yaw 를 정면(0.0)에서 검출 방위각(1.0) 쪽으로 양보해 가는 순서.
# 앞쪽부터 IK 를 물어보고 처음 풀리는 값을 쓴다 (pick_grasp_yaw_weight).
#
# 되돌림(2026-09-23): (0.0, 0.3, 0.6, 1.0) 으로 넣었다가 (1.0,) 으로 복구했다.
# 트레이는 팔 기준 방위각이 90도 가까운 자리에 있어서(스캔이 joint_1 을 -90도 돌려야 보인다)
# w=0.0 이 채택되면 파지 위치는 트레이 위인데 자세만 로봇 정면이 된다 — 그리퍼가 트레이를
# 90도 옆에서 가로질러 들어간다. 실측: 스캔 자세에서 '안전 위치(파지 전)' 로 넘어가며 손목이 뒤틀렸다.
# 다시 쓰려면 (1-w)*|방위각| 상한을 먼저 넣을 것. 상한 없이는 방위각이 큰 자리에서 항상 이 꼴이 된다.
GRASP_YAW_WEIGHTS = (1.0,)


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


def quat_conj(q):
    """단위 쿼터니언의 역 (w, -x, -y, -z)"""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


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


def tray_yaw_deg(quat_base):
    """트레이 자세(base 기준)의 수평 방위각. 로컬 +X 를 base 수평면에 투영해 읽는다.

    오일러 분해보다 안전하다 — roll 이 -89도쯤인 자세에서 yaw/roll 이 서로 넘나든다."""
    rot = quat_to_matrix(quat_base)
    v = rot @ np.array([1.0, 0.0, 0.0])
    if np.hypot(v[0], v[1]) < 1e-6:          # 로컬 +X 가 수직이면 +Y 로 읽는다
        v = rot @ np.array([0.0, 1.0, 0.0])
    return float(np.degrees(np.arctan2(v[1], v[0])))


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


def check_math():
    """파지 회전 보정의 부호를 확인한다. 여기가 틀리면 그리퍼가 트레이 반대쪽을 본다.

    POINT1~5 는 이제 이 파일에서 직접 쓰이지 않는다(파지는 검출, place 는
    RACK_* 상수) — 그래서 그 좌표에 기대는 회귀 검증 대신, 씬/좌표에 의존하지
    않는 구조적 성질만 확인한다."""
    # 임의의 방향에서 보정 후 툴 +Z(손가락 방향)가 그 방향을 향해야 한다
    for target in ([-0.15, 0.80, 0.28], [0.80, 0.03, 0.11], [0.10, -0.75, 0.20]):
        target = np.array(target)
        # weight 가 몇이든 툴 +Z 는 자기가 돌려준 direction 을 향해야 한다
        for w in GRASP_YAW_WEIGHTS:
            d, q = grasp_frame(target, w)
            tool_z = quat_to_matrix(q)[:, 2]
            assert np.dot(tool_z[:2], d[:2]) > 0.99, f"파지 yaw 보정이 틀렸다: {target} w={w}"
        # weight 1.0 은 기존 동작 — direction 이 base 에서 파지점을 향하는 방위각과 같아야 한다
        d1, _ = grasp_frame(target, 1.0)
        assert np.allclose(d1, approach_direction(target)[0], atol=1e-9), f"w=1.0 이 기존과 다르다: {target}"
        # weight 0.0 은 로봇 정면 — POINT2_RPY 그대로이므로 direction 은 base +Y
        d0, q0_ = grasp_frame(target, 0.0)
        assert np.allclose(d0, [0.0, 1.0, 0.0], atol=1e-9), f"w=0.0 이 정면이 아니다: {target} -> {d0}"
        assert np.allclose(q0_, make_target_quat(*POINT2_RPY), atol=1e-9), f"w=0.0 자세가 POINT2_RPY 가 아니다: {target}"

    # rack_yaw_delta_deg 구조 검증 — 씬 좌표와 무관하게 항상 성립해야 하는 성질들
    for p in ([-0.15, 0.80, 0.28], [0.80, 0.03, 0.11], [0.10, -0.75, 0.20]):
        p = np.array(p)
        same = rack_yaw_delta_deg(p, p)
        assert abs(same) < 1e-6, f"같은 방향인데 회전량이 0 이 아니다: {p} -> {same:+.4f}"
        opposite = rack_yaw_delta_deg(p, -p)
        assert abs(abs(opposite) - 180.0) < 1e-6, f"반대 방향인데 180 도가 아니다: {p} -> {opposite:+.4f}"

    # 랙 관측: (id, 칸) -> 칸별 id. 같은 칸 중복은 먼저 온 것, 범위 밖 칸은 무시 -> [2, -1, 1]
    got = assign_slots([(2, 0), (1, 2), (0, 2), (2, 7)])
    assert got == [2, -1, 1], got
    # 프레임 누적: 1번 칸은 2 가 두 번/0 이 한 번 -> 2, 2번 칸은 한 번도 없음 -> -1, 3번 칸은 한 번만 -> 1
    assert merge_urgencies([[2, -1, -1], [0, -1, 1], [2, -1, -1]]) == [2, -1, 1]


def world_to_base_pos(world_pos, base_pos, base_quat):
    """world 좌표를 로봇 base 기준 위치로 바꾼다 (자세는 다루지 않는다).
    base_to_world 의 위치 성분만의 역변환이다."""
    base_rot = quat_to_matrix(base_quat)
    return base_rot.T @ (np.array(world_pos, dtype=float) - np.array(base_pos, dtype=float))


def approach_direction(grasp_base):
    """base 원점에서 grasp_base 를 향하는 수평 단위벡터와, POINT2_RPY 기준
    자세에서 돌려야 할 base z 축 회전각(도).

    POINT2_RPY 는 트레이가 base +y 정면에 있을 때 맞춘 자세다. 트레이가 다른
    방향에 있으면 그만큼 base z 축으로 돌려야 그리퍼가 트레이를 마주본다.
    Rz(t)*[0,1,0] = [-sin t, cos t, 0] 이므로 t = atan2(-dx, dy).

    ponytail: 트레이의 실제 yaw 는 2D 박스로 알 수 없어서 'base 에서 트레이를
    향하는 방향'을 정면으로 본다. 트레이가 비스듬히 놓이면 어긋난다 — 그때는
    깊이 점군 PCA 로 주축을 추정해야 한다."""
    d = np.array([grasp_base[0], grasp_base[1], 0.0], dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-6:
        return np.array([0.0, 1.0, 0.0]), 0.0
    d /= n
    return d, float(np.degrees(np.arctan2(-d[0], d[1])))


def grasp_frame(grasp_base, weight=1.0):
    """파지 자세(쿼터니언)와 접근 방향을 함께 돌려준다.
    base z 축 회전을 POINT2_RPY 기준 자세 바깥에서 곱한다 — 오일러각에 더하면
    툴 자기 축 회전이 되어 버리므로 이 순서가 맞다.

    weight 는 방위각을 얼마나 따를지다. 1.0 = 검출 방위각 그대로(기존 동작,
    Rz(yaw)*[0,1,0] 이 곧 방위각 방향이라 direction 도 예전 값과 같다),
    0.0 = 로봇 정면(POINT2_RPY) 자세. 중간값은 그 사이를 선형으로 간다.
    direction 도 같이 돌려야 한다 — 접근 후퇴(APPROACH_BACKOFF_M)가 그리퍼가
    보는 방향과 어긋나면 옆에서 들이민다. check_math 의 tool_z 불변식이 이걸 잡는다."""
    _, yaw_deg = approach_direction(grasp_base)
    rz = quat_from_axis([0, 0, 1], yaw_deg * weight)
    q = quat_mul(rz, make_target_quat(*POINT2_RPY))
    direction = quat_to_matrix(rz) @ np.array([0.0, 1.0, 0.0])
    return direction, q / np.linalg.norm(q)


def pick_grasp_yaw_weight(lula, grasp_base, base_pos, base_quat):
    """정면(0.0)부터 방위각(1.0)까지 순서대로 IK 를 물어보고 처음 풀리는 각도를 쓴다.

    팔은 정면에서 가장 여유가 크고, 방위각이 커질수록 손목이 특이점 쪽으로 끌려간다.
    그래서 정면을 먼저 시도하고 안 되면 방위각 쪽으로 한 칸씩 양보한다. 전부 실패하면
    1.0(방위각) 으로 떨어진다 — 기존 동작이라 최소한 지금보다 나빠지지 않는다.

    팔을 움직이지 않고 미리 물어본다. warm_start 를 주지 않아 solver 기본 시드를 쓰므로
    같은 입력이면 항상 같은 답이 나온다 (build_return_steps 가 따로 불러도 일치한다).

    ponytail: 파지점 하나만 본다. 접근/들어올리기는 같은 yaw 에 더 쉬운 자세라 보통 같이 풀린다.
    ponytail: (1-w)*|yaw_deg| 가 크면 접근선이 옆 트레이를 가로지를 수 있다. 실제로 부딪히면
              그때 상한을 넣을 것 — 지금은 IK 가 풀리는지만 본다."""
    j5 = list(lula.get_joint_names()).index("joint_5")
    for w in GRASP_YAW_WEIGHTS:
        _, q = grasp_frame(grasp_base, w)
        pos, quat = base_pose_to_world(grasp_base, q, base_pos, base_quat)
        sol, ok = lula.compute_inverse_kinematics(EE_LINK_NAME, tcp_to_flange(pos, quat), quat)
        if ok and abs(np.degrees(sol[j5])) >= WRIST_NEAR_SINGULAR_DEG:
            print(f"   파지 yaw weight {w:.2f} 채택 "
                  f"(방위각 {approach_direction(grasp_base)[1]:+.1f}deg 중 "
                  f"{approach_direction(grasp_base)[1] * w:+.1f}deg 사용, "
                  f"joint_5 {np.degrees(sol[j5]):+.1f}deg)")
            return w
    print("   파지 yaw — 어느 weight 로도 IK 가 안 풀렸다. 방위각(1.0) 그대로 간다")
    return 1.0


def rack_yaw_delta_deg(grasp_base, place_base):
    """파지 위치에서 place 위치로 갈 때 joint_1 을 얼마나 돌려야 하는가 (도).

    grasp_base 는 검출값이라 매번 방향이 달라진다. JOINT1_ROTATE_DEG 처럼 고정값을
    relative 로 더하면, 파지 방향이 기존과 다를 때 place 근처에도 못 미치거나
    지나쳐 버린다. 두 지점의 방위각(approach_direction 의 yaw) 차이로 계산하면
    파지 위치가 어디든 정확히 place 방향까지 돌아간다.
    +-180 도로 접어서 먼 쪽으로 도는 것을 막는다."""
    _, grasp_yaw = approach_direction(grasp_base)
    _, place_yaw = approach_direction(place_base)
    return (place_yaw - grasp_yaw + 180.0) % 360.0 - 180.0


def base_pose_to_world(tcp_base, quat_base, robot_pos, robot_quat):
    """base_to_world 와 같지만 RPY 대신 쿼터니언을 받는다 (grasp_frame 의 결과용)"""
    world_pos = np.array(robot_pos) + quat_to_matrix(robot_quat) @ np.array(tcp_base, dtype=float)
    world_quat = quat_mul(robot_quat, quat_base)
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


def setup_people():
    """omni.anim.people 캐릭터가 걸어다니게 한다 (run_human_scene.py 에서 가져옴).

    확장도 carb 설정도 **씬 로드보다 먼저** 해야 한다. 캐릭터 behavior 스크립트는
    초기화 시점에 명령 파일 경로를 한 번만 읽고, standalone 은 확장을 최소만 로드한다.
    확장이 빠지면 'No module named omni.anim.graph.core' 로 사람이 제자리에 선다."""
    for ext in PEOPLE_EXTENSIONS:
        enable_extension(ext)
        simulation_app.update()
    st = carb.settings.get_settings()
    st.set("/exts/omni.anim.people/command_settings/command_file_path", PEOPLE_COMMAND_FILE)
    st.set("/exts/omni.anim.people/command_settings/number_of_loop", "inf")   # 기본 "0" 은 1회 재생
    st.set("/exts/omni.anim.people/navigation_settings/navmesh_enabled", True)
    st.set("/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled", True)
    st.set("/app/scripting/ignoreWarningDialog", True)
    simulation_app.update()
    print(f"   people       확장 {len(PEOPLE_EXTENSIONS)}개 + {Path(PEOPLE_COMMAND_FILE).name}")


def setup_characters():
    """캐릭터마다 애니메이션 그래프와 behavior 스크립트를 붙인다. **씬 로드 뒤에** 호출한다.

    이게 사람이 안 움직이던 진짜 이유다. 확장을 켜고 command_file_path 를 지정해도,
    명령을 읽어 실행하는 주체는 캐릭터 SkelRoot 에 붙은 `character_behavior.py`
    (omni.kit.scripting BehaviorScript) 다. GUI 의 People 확장에서 'Setup characters' 를
    눌러야 USD 에 저장되는데 integration_human.usd 에는 없다 — 실측으로 확인:
    /World/Characters/Character{,_01} 에 omni:scripting:scripts 도, AnimationGraphAPI 도 없다.

    isaacsim.replicator.agent.core 의 stage_util.setup_animation_graph_to_character /
    setup_python_scripts_to_character 와 같은 명령을 그대로 쓴다."""
    import omni.kit.app
    import omni.kit.commands

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(CHARACTERS_ROOT)
    if not root.IsValid():
        print(f"   people       {CHARACTERS_ROOT} 가 없다 — 사람 없는 씬")
        return 0

    # Biped_Setup 은 스켈레톤/애니메이션 원본이라 제외한다 (캐릭터가 아니다)
    skelroots = [prim for prim in Usd.PrimRange(root)
                 if prim.IsA(UsdSkel.Root) and not str(prim.GetPath()).startswith(BIPED_SETUP_PATH)]
    if not skelroots:
        print("   people       캐릭터 SkelRoot 를 못 찾았다 — 캐릭터 에셋이 아직 로드 안 됐다")
        return 0

    biped = stage.GetPrimAtPath(BIPED_SETUP_PATH)
    graph = next((prim for prim in Usd.PrimRange(biped) if prim.GetTypeName() == "AnimationGraph"),
                 None) if biped.IsValid() else None

    paths = [prim.GetPath() for prim in skelroots]
    if graph is None:
        print(f"   people       {BIPED_SETUP_PATH} 아래 AnimationGraph 가 없다 — 걷는 모션이 안 붙는다")
    else:
        omni.kit.commands.execute("RemoveAnimationGraphAPICommand", paths=paths)
        omni.kit.commands.execute("ApplyAnimationGraphAPICommand", paths=paths,
                                  animation_graph_path=graph.GetPath())

    script = (omni.kit.app.get_app().get_extension_manager()
              .get_extension_path_by_module("omni.anim.people")
              + "/omni/anim/people/scripts/character_behavior.py")
    omni.kit.commands.execute("RemoveScriptingAPICommand", paths=paths)
    omni.kit.commands.execute("ApplyScriptingAPICommand", paths=paths)
    for prim in skelroots:
        prim.GetAttribute("omni:scripting:scripts").Set([script])
    for _ in range(10):
        simulation_app.update()

    names = ", ".join(prim.GetName() for prim in skelroots)
    print(f"   characters   {len(skelroots)}명 연결 ({names}) — 그래프 {'O' if graph else 'X'}, behavior 스크립트 O")
    print(f"                명령은 {Path(PEOPLE_COMMAND_FILE).name} 의 이름과 캐릭터 프림 이름이 같아야 실행된다")
    return len(skelroots)


def bake_navmesh(timeout_s=30.0):
    """NavMesh 를 굽는다. 베이크 결과는 USD 에 저장되지 않아 매 실행 다시 구워야 한다.
    안 구우면 캐릭터의 GoTo 가 전부 'invalid command' 로 거부돼 사람이 안 움직인다."""
    try:
        import omni.anim.navigation.core as nav
    except ImportError as exc:
        print(f"   navmesh      건너뜀 ({exc})")
        return False
    inav = nav.acquire_interface()
    inav.start_navmesh_baking()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        simulation_app.update()
        if not inav.is_navmesh_baking():
            break
    else:
        print(f"   navmesh      베이크 시간 초과 ({timeout_s:.0f}s)")
        return False
    if inav.get_navmesh() is None:
        print("   navmesh      실패 — NavMeshVolume 범위와 agentMinIslandRadius 를 확인할 것")
        return False
    print("   navmesh      베이크 완료")
    return True


def find_rtx_lidar(root_path):
    """root_path 하위의 RTX 라이다 프림. 스톡 nova_carter.usd 가 온라인 에셋이라
    하위 프림 이름을 확인할 수 없어서 이름 대신 타입으로 찾는다."""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "OmniLidar":
            return prim
    for prim in Usd.PrimRange(root):
        if any(a.GetName().startswith("omni:sensor:") for a in prim.GetAttributes()):
            return prim
    return None


def load_scene():
    """PnP 씬을 연다"""
    if not open_stage(SCENE_USD):
        raise RuntimeError(f"Could not open USD: {SCENE_USD}")
    while is_stage_loading():
        simulation_app.update()

    print(f"   scene        {Path(SCENE_USD).name}")


def set_viewport_lighting(rig="Default"):
    """뷰포트 라이팅을 Stage Lights 대신 rig(Default) 로 바꾼다. 씬 로드 후에 부른다.
    뷰포트 메뉴의 'Default' 와 같은 액션이다. 손목 카메라 렌더에도 적용된다."""
    import omni.kit.actions.core
    action = omni.kit.actions.core.get_action_registry().get_action(
        "omni.kit.viewport.menubar.lighting", "set_lighting_mode_rig")
    if action is None:
        carb.log_warn("라이팅 액션을 못 찾았다 — 뷰포트 메뉴에서 Default 를 직접 고를 것")
        return
    action.execute(rig)
    print(f"   lighting     {rig}")


class KeyTap:
    """뷰포트에 포커스가 있을 때의 키 PRESS 를 모아 둔다 (pnp_teleop_detection 의 축소판)"""

    def __init__(self):
        self._taps = []
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._input = carb.input.acquire_input_interface()
        self._sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_event)

    def _on_event(self, event, *args):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._taps.append(event.input.name)
        return True

    def take(self):
        taps, self._taps = self._taps, []
        return taps

    def close(self):
        self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub)


def attach_aruco(tray_path, marker_id):
    """트레이 손잡이 윗판 위에 ArUco 텍스처 사각형(시각 전용, 콜리전 없음)을 붙인다.

    handle Xform 의 자식이라 트레이가 움직이면 같이 따라간다. 위에서 볼 때 반시계 순서로
    점을 두고 st 를 같은 순서로 주면 이미지가 거울상 없이 보인다(거울상이면 id 가 안 읽힌다)."""
    stage = omni.usd.get_context().get_stage()
    root = f"{tray_path}/{ARUCO_HANDLE_REL}/aruco"
    h, z = ARUCO_SIDE_M / 2, ARUCO_TOP_Z_M
    mesh = UsdGeom.Mesh.Define(stage, root)
    mesh.CreatePointsAttr([(-h, -h, z), (h, -h, z), (h, h, z), (-h, h, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0, 0, 1)] * 4)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                            UsdGeom.Tokens.varying).Set([(0, 0), (1, 0), (1, 1), (0, 1)])

    mat = UsdShade.Material.Define(stage, f"{root}/mat")
    pbr = UsdShade.Shader.Define(stage, f"{root}/mat/pbr")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)   # 반사광으로 셀이 날아가지 않게
    tex = UsdShade.Shader.Define(stage, f"{root}/mat/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"{ARUCO_DIR}/aruco_{marker_id}.png")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    st = UsdShade.Shader.Define(stage, f"{root}/mat/st")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.CreateOutput("result", Sdf.ValueTypeNames.Float2))
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
    mat.CreateSurfaceOutput().ConnectToSource(pbr.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    print(f"   aruco        {tray_path}  id {marker_id} (긴급도 {'하중상'[marker_id]})")


def prim_world_quat(path):
    """프림의 월드 자세(쿼터니언 w,x,y,z). xformOp 구성이 무엇이든 합성 행렬에서 뽑는다"""
    stage = omni.usd.get_context().get_stage()
    mat = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    q = mat.RemoveScaleShear().ExtractRotationQuat().GetNormalized()
    return np.array([q.GetReal(), *q.GetImaginary()], dtype=float)


# ── 트레이 정렬용 — spawn_tray_copies 가 채우고, 정렬 쪽에서만 쓴다
TRAY_BODY_REL  = "tray"   # 실제 rigid body 는 한 단계 아래다 (/World/tray/tray, pickup_place_go.py 와 같은 구조)
_tray_bodies   = []       # 트레이 rigid body 프림 경로
_tray_rigid    = {}       # 경로 -> SingleRigidPrim (처음 쓸 때 만들어 재사용)
_held_tray     = None     # 그리퍼에 물린 트레이와 body 원점 어긋남. capture_tray_in_gripper 가 채운다
_aligned_trays = set()    # 이미 정렬한 트레이. 트레이당 정확히 한 번만 손댄다
# 스폰 자세(base 기준). 랙에서 '반듯한' 자세의 기준이다 — 책상 위에 반듯하게 놓여 있고
# IK 를 안 거친 자세다. 랙 목표 자세는 이걸 base z 로 90도의 정수배만큼 돌린 것들 중 하나다
_tray_spawn_quat_base = None


def spawn_tray_copies():
    """원본 트레이를 TRAY_COPIES 개 복제해 한 줄로 흩뿌리고, 전부에 긴급도 마커를 붙인다.
    world.reset() 전에 부른다.

    벌리는 방향은 **base 에서 트레이를 향하는 방향에 수직인 수평축**이다. 반지름 방향으로
    벌리면 파지점 도달거리가 0.69~1.09m 로 퍼지는데(파지점은 검출면보다 TRAY_HALF_DEPTH_M
    만큼 더 깊다), 1.10m 부근은 joint_5 가 0 으로 밀려 손목 특이점이다 — IK 가 거부되며
    들어올리기가 실패하던 원인이었다. 가로로만 벌리면 0.90~0.93m 에 모여 전부 안전하다.

    duplicate_prim 은 참조까지 합쳐 복사하므로 rigid body / 콜리전이 그대로 따라온다.
    마커는 복제 뒤에 붙여야 트레이마다 다른 id 를 가진다."""
    stage = omni.usd.get_context().get_stage()
    origin = np.array(stage.GetPrimAtPath(TRAY_PRIM_PATH).GetAttribute("xformOp:translate").Get(), dtype=float)

    base_pos, base_quat = arm_base_pose()
    radial = world_to_base_pos(origin, base_pos, base_quat)[:2]
    radial = radial / np.linalg.norm(radial)
    lateral = quat_to_matrix(base_quat) @ np.array([-radial[1], radial[0], 0.0])
    print(f"   tray spread  가로축(월드) {vec(lateral)}  범위 +-{TRAY_SPREAD_R_M * 100:.0f}cm")

    _tray_bodies.clear()
    _tray_bodies.append(f"{TRAY_PRIM_PATH}/{TRAY_BODY_REL}")
    # 랙 정렬의 기준 자세. 복제본은 duplicate_prim 이 xform 을 통째로 복사하므로 전부 같다
    global _tray_spawn_quat_base
    _tray_spawn_quat_base = quat_mul(
        quat_conj(base_quat), prim_world_quat(f"{TRAY_PRIM_PATH}/{TRAY_BODY_REL}"))
    print(f"   tray spawn   기준 자세(base) rpy "
          f"{vec(matrix_to_rpy(quat_to_matrix(_tray_spawn_quat_base)), 1)}  "
          f"방위각 {tray_yaw_deg(_tray_spawn_quat_base):+.1f}deg")

    placed = [origin]
    attach_aruco(TRAY_PRIM_PATH, random.choice(ARUCO_IDS))
    for i in range(1, TRAY_COPIES + 1):
        for _ in range(20):
            pos = origin + lateral * np.random.uniform(-TRAY_SPREAD_R_M, TRAY_SPREAD_R_M)
            if all(np.linalg.norm(pos[:2] - q[:2]) >= TRAY_MIN_GAP_M for q in placed):
                break
        placed.append(pos)
        path = f"{TRAY_PRIM_PATH}_{i}"
        omni.usd.duplicate_prim(stage, TRAY_PRIM_PATH, path)
        stage.GetPrimAtPath(path).GetAttribute("xformOp:translate").Set(Gf.Vec3d(*pos))
        print(f"   tray copy    {path}  world {vec(pos)}")
        attach_aruco(path, random.choice(ARUCO_IDS))
        _tray_bodies.append(f"{path}/{TRAY_BODY_REL}")
    simulation_app.update()
    return origin


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
            prim_path=ART_ROOT_PATH,   # 아티큘레이션 루트 = chassis_link
            name="m0609_robot",
            end_effector_prim_path=ee_path,
            gripper=gripper,
        )
    )
    print(f"   EE frame     {ee_path}")
    return robot


def init_robot(robot, world):
    """Articulation 과 그리퍼를 초기화하고 홈 관절 각도로 보낸다.

    카터 관절(바퀴/캐스터)까지 한 아티큘레이션에 섞여 dof 가 19개다.
    앞 6개가 팔이 아니고, zeros 로 덮으면 바퀴까지 리셋돼 로봇이 튄다."""
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


def arm_base_pose():
    """IK 기준은 아티큘레이션 루트(카터 섀시)가 아니라 팔의 base_link 다"""
    return SingleXFormPrim(ARM_BASE_PATH).get_world_pose()


def sync_base_pose(lula, robot):
    """팔 base_link 의 현재 월드 pose 를 솔버에 넘긴다.
    robot.get_world_pose() 는 아티큘레이션 루트(chassis_link) 를 주므로 여기 쓰면 안 된다."""
    base_pos, base_quat = arm_base_pose()
    lula.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
    return base_pos, base_quat


# ══════════════════════════════════════════════════════════════
#  ROS2 — 손목 카메라 rgb/depth 발행, 검출 결과 구독
# ══════════════════════════════════════════════════════════════
def find_color_camera():
    """손목 D455 의 컬러 카메라 프림 경로를 찾는다"""
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


def build_detection_graph(color_camera_path):
    """손목 카메라를 ROS2 rgb/depth/camera_info 토픽에 연결하고, 검출 결과를 구독한다.

    depth 와 camera_info 는 컬러와 '같은' render product 에 붙여야 픽셀이 정렬된다."""
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
                ("sub_aruco", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("pub_mission", "isaacsim.ros2.bridge.ROS2Publisher"),
                ("sub_nav", "isaacsim.ros2.bridge.ROS2Subscriber"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "rp.inputs:execIn"),
                ("tick.outputs:tick", "sub_detect.inputs:execIn"),
                ("tick.outputs:tick", "sub_aruco.inputs:execIn"),
                ("tick.outputs:tick", "pub_mission.inputs:execIn"),
                ("tick.outputs:tick", "sub_nav.inputs:execIn"),
                ("rp.outputs:execOut", "pub_color.inputs:execIn"),
                ("rp.outputs:execOut", "pub_depth.inputs:execIn"),
                ("rp.outputs:execOut", "pub_info.inputs:execIn"),
                ("rp.outputs:renderProductPath", "pub_color.inputs:renderProductPath"),
                ("rp.outputs:renderProductPath", "pub_depth.inputs:renderProductPath"),
                ("rp.outputs:renderProductPath", "pub_info.inputs:renderProductPath"),
                ("context.outputs:context", "pub_color.inputs:context"),
                ("context.outputs:context", "pub_depth.inputs:context"),
                ("context.outputs:context", "pub_info.inputs:context"),
                ("context.outputs:context", "sub_detect.inputs:context"),
                ("context.outputs:context", "sub_aruco.inputs:context"),
                ("context.outputs:context", "pub_mission.inputs:context"),
                ("context.outputs:context", "sub_nav.inputs:context"),
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
                ("sub_aruco.inputs:topicName", MARKER_TOPIC),
                ("pub_mission.inputs:topicName", MISSION_TOPIC),
                ("sub_nav.inputs:topicName", NAV_DONE_TOPIC),
            ],
        },
    )
    set_generic_message_type(f"{DETECT_GRAPH_PATH}/sub_detect", "std_msgs", "msg", "Float32MultiArray")
    set_generic_message_type(f"{DETECT_GRAPH_PATH}/sub_aruco", "std_msgs", "msg", "Float32MultiArray")
    set_generic_message_type(f"{DETECT_GRAPH_PATH}/pub_mission", "std_msgs", "msg", "Float32MultiArray")
    set_generic_message_type(f"{DETECT_GRAPH_PATH}/sub_nav", "std_msgs", "msg", "Float32MultiArray")
    # cameraPrim 은 relationship 이라 SET_VALUES 로는 못 넣는다
    set_targets(
        prim=omni.usd.get_context().get_stage().GetPrimAtPath(f"{DETECT_GRAPH_PATH}/rp"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[color_camera_path],
    )
    for name, topic in (("color", COLOR_TOPIC), ("depth", DEPTH_TOPIC), ("info", INFO_TOPIC)):
        print(f"   pub {name:<6s}   /{topic}")
    print(f"   sub detect   /{RESULT_TOPIC}  (std_msgs/Float32MultiArray)")
    print(f"   sub aruco    /{MARKER_TOPIC}  (std_msgs/Float32MultiArray)")
    print(f"   pub mission  /{MISSION_TOPIC}  (1 = 적재 완료, 주행 시작해도 됨)")
    print(f"   sub nav      /{NAV_DONE_TOPIC}  (1 = 도킹 완료, 하역 시작)")


def build_rear_lidar_graph():
    """후방 2D 라이다를 /scan_rear 로 발행하고 TF 에 얹는다 (run_human_scene.py 와 같은 일).

    씬의 액션그래프를 건드리지 않고 **DetectGraph 에 노드 2개만 더한다** — tick/context 를
    그대로 재사용하면 되고, 참조로 올라온 그래프 안에 프림을 만들지 않아도 된다.
    TF 는 씬 그래프의 tf_sensors 에 대상만 추가한다(참조 위에 over 로 얹힌다).
    이게 빠지면 토픽은 나오는데 costmap 이 전부 버린다 — 조용한 실패다."""
    lidar = find_rtx_lidar(REAR_RPLIDAR_PATH)
    if lidar is None:
        print(f"   rear lidar   못 찾음: {REAR_RPLIDAR_PATH} — 후방 장애물이 costmap 에 안 찍힌다")
        return False
    lidar_path = str(lidar.GetPath())

    # ROS2PublishTransformTree 는 프림 이름(또는 nameOverride)을 프레임 이름으로 쓴다
    ov = lidar.GetAttribute("isaac:nameOverride")
    if not ov:
        ov = lidar.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
    if ov.Get() != REAR_FRAME:
        ov.Set(REAR_FRAME)

    g = DETECT_GRAPH_PATH
    og.Controller.edit(g, {
        og.Controller.Keys.CREATE_NODES: [
            ("rp_rear", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("pub_rear", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("rp_rear.inputs:width", 1),
            ("rp_rear.inputs:height", 1),
            ("pub_rear.inputs:topicName", REAR_TOPIC),
            ("pub_rear.inputs:frameId", REAR_FRAME),
            ("pub_rear.inputs:type", "laser_scan"),   # 2D 라 point_cloud 가 아니다
            ("pub_rear.inputs:fullScan", True),       # 360도 한 바퀴를 한 메시지로
            ("pub_rear.inputs:enabled", True),
            ("pub_rear.inputs:resetSimulationTimeOnStop", False),
        ],
        og.Controller.Keys.CONNECT: [
            (g + "/tick.outputs:tick", "rp_rear.inputs:execIn"),
            ("rp_rear.outputs:execOut", "pub_rear.inputs:execIn"),
            ("rp_rear.outputs:renderProductPath", "pub_rear.inputs:renderProductPath"),
            (g + "/context.outputs:context", "pub_rear.inputs:context"),
        ],
    })
    set_targets(prim=omni.usd.get_context().get_stage().GetPrimAtPath(f"{g}/rp_rear"),
                attribute="inputs:cameraPrim", target_prim_paths=[lidar_path])

    tf = omni.usd.get_context().get_stage().GetPrimAtPath(TF_SENSORS_NODE)
    if not tf.IsValid():
        print(f"   tf_sensors   노드 없음: {TF_SENSORS_NODE} — /{REAR_TOPIC} 는 나가지만 costmap 이 버린다")
        return False
    rel = tf.GetRelationship("inputs:targetPrims")
    targets = list(rel.GetTargets())
    if Sdf.Path(lidar_path) not in targets:
        rel.SetTargets(targets + [Sdf.Path(lidar_path)])
    print(f"   pub rear     /{REAR_TOPIC}  (laser_scan, frame={REAR_FRAME})")
    return True


def read_sub_data(node_name, verbose=False):
    """DetectGraph 의 제네릭 ROS2Subscriber 가 받은 최신 Float32MultiArray data. 못 읽으면 None.

    제네릭 ROS2Subscriber 는 메시지 필드를 동적 output 어트리뷰트로 만든다.
    경로 문자열로 읽는 법과 노드 객체로 읽는 법 두 가지가 있어 둘 다 시도한다."""
    node_path = f"{DETECT_GRAPH_PATH}/{node_name}"
    data, why = None, []
    try:
        data = og.Controller.attribute(f"{node_path}.outputs:data").get()
    except Exception as exc:
        why.append(f"경로로 읽기 실패: {exc}")
    if data is None:
        try:
            node = og.Controller.node(node_path)
            data = node.get_attribute("outputs:data").get()
        except Exception as exc:
            why.append(f"노드로 읽기 실패: {exc}")
    if data is None:
        if verbose:
            print(f"   {node_name} 에서 outputs:data 를 못 읽었다:")
            for line in why:
                print(f"      {line}")
        return None
    if len(data) < 2:
        if verbose:
            print(f"   {node_name} data 길이가 {len(data)} 다 — 아직 수신 전이거나 형식이 다르다")
        return None
    return data


def read_detections(verbose=False):
    """관제 PC 가 보낸 최신 검출 결과를 읽는다.

    반환: (seq, [(np.array([x, y, z]), conf), ...])  카메라 광학 프레임, **가까운 것부터**.
    seq 는 관제 PC 의 발행 카운터. 값이 올라갔다는 건 새로 찍은 프레임이라는 뜻이다."""
    data = read_sub_data("sub_detect", verbose)
    if data is None:
        return -1, []
    seq, count = int(data[0]), int(data[1])
    out = []
    for i in range(count):
        x, y, z, conf = data[2 + i * 4 : 6 + i * 4]
        out.append((np.array([x, y, z], dtype=float), float(conf)))
    if verbose:
        print(f"   구독자 수신: seq {seq}, {count}개")
    return seq, out


def read_markers(verbose=False):
    """관제 PC 가 보낸 최신 ArUco 마커. 반환: (seq, [(id, slot), ...]).
    slot 은 관제 PC 가 랙 ROI 를 가로 3등분해 매긴 칸 0/1/2 (화면 왼쪽부터 = 랙 1/2/3번)."""
    data = read_sub_data("sub_aruco", verbose)
    if data is None:
        return -1, []
    seq, count = int(data[0]), int(data[1])
    return seq, [(int(data[2 + i * 2]), int(data[3 + i * 2])) for i in range(count)]


def publish_mission_state(value):
    """주행 프로세스에 적재 완료 여부를 알린다. 노드는 매 틱 이 값을 발행한다."""
    og.Controller.attribute(f"{DETECT_GRAPH_PATH}/pub_mission.inputs:data").set([float(value)])


def clear_nav_done():
    """구독 노드가 들고 있는 /nav_done 값을 0 으로 지운다 (정지할 때 호출).

    제네릭 ROS2Subscriber 는 마지막으로 받은 값을 계속 들고 있다. 주행을 한 번 끝낸
    세션에서 Play 를 다시 누르면 이전 주행의 1 이 그대로 남아 있어, 적재+관측이 끝나자마자
    wait_unload 가 통과돼 **주행 없이 하역이 시작된다.**"""
    try:
        og.Controller.attribute(f"{DETECT_GRAPH_PATH}/sub_nav.outputs:data").set([0.0])
    except Exception as exc:
        print(f"   nav_done 초기화 실패 (무시): {exc}")


def nav_done():
    """주행 프로세스가 도킹까지 끝냈으면 True. 한 번이라도 1 을 받으면 구독 노드가 값을 들고 있다."""
    try:
        data = og.Controller.attribute(f"{DETECT_GRAPH_PATH}/sub_nav.outputs:data").get()
    except Exception:
        return False
    return data is not None and len(data) >= 1 and float(data[0]) >= 1.0


def optical_to_world(point_optical, color_camera_path):
    """관제 PC 가 준 카메라 광학 프레임 좌표를 월드로 옮긴다.

    ROS 광학 규약은 (x 우, y 하, z 전방)이고 USD 카메라는 (x 우, y 상, -z 전방)이라
    y, z 의 부호가 뒤집힌다."""
    x, y, z = point_optical
    usd_point = Gf.Vec3d(float(x), float(-y), float(-z))
    cam_prim = omni.usd.get_context().get_stage().GetPrimAtPath(color_camera_path)
    mat = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
    return np.array(mat.Transform(usd_point), dtype=float)


def grasp_point_base(tray_world, base_pos, base_quat):
    """검출점(월드) -> 파지 목표(base 기준).

    접근 방향(base 에서 트레이를 향하는 방향)으로 트레이 중심 보정과 높이 보정을 준다."""
    surface = world_to_base_pos(tray_world, base_pos, base_quat)
    direction, _ = approach_direction(surface)
    return surface + direction * TRAY_HALF_DEPTH_M + np.array([0.0, 0.0, GRASP_ABOVE_CENTER_M])


def iter_tray_optical(verbose=False):
    """깊이 게이트를 통과한 검출을 **가까운 순으로 하나씩** 내준다.

    관제 PC 가 카메라에서 가까운 순으로 보내주므로 앞에서부터 쓰면 된다. 앞엣것을 먼저
    집어야 뒤/옆 트레이를 건드리지 않는다.

    깊이 범위만 거른다 — base 도달거리 게이트는 파지점을 계산한 뒤에만 의미가
    있어서(원본 점 자체는 base 기준이 아니다) 여기서는 보지 않는다."""
    _seq, detections = read_detections(verbose=verbose)
    for i, (point_optical, conf) in enumerate(detections):
        depth = float(point_optical[2])
        if not (DETECT_DEPTH_MIN_M <= depth <= DETECT_DEPTH_MAX_M):
            print(f"   트레이{i + 1} 무시 — 깊이 {depth:.3f}m 가 "
                  f"{DETECT_DEPTH_MIN_M}~{DETECT_DEPTH_MAX_M}m 범위 밖이다")
            continue
        yield point_optical, conf


def find_first_tray_optical(verbose=False):
    """가장 가까운 트레이의 원본 검출점(카메라 광학 프레임)과 conf. 없으면 None."""
    return next(iter_tray_optical(verbose=verbose), None)


def first_tray_grasp(color_camera_path, base_pos, base_quat, verbose=False,
                     centered_first=False):
    """집을 트레이의 파지점(base)과 월드 좌표를 구한다. 없으면 None.

    base 에서의 도달 거리를 여기서 본다 — 팔이 실제로 움직이는 건 이쪽이라
    받은 값을 그대로 믿으면 안 된다 (벽 오검출을 여기서 거른다).

    centered_first=True 면 **화면 중앙에 가장 가까운 것**(|x_optical| 최소)부터 본다.
    정렬 단계(find_first_tray_optical)가 3D 최근접 트레이를 화면 중앙에 맞춰 놓은 뒤라,
    여기서 3D 최근접을 다시 고르면 **정렬한 그 트레이가 아닌 다른 트레이**를 집으러 간다 —
    카메라가 옆으로 옮겨져 화면 구성이 바뀌었고, 트레이가 가로 한 줄로 3개 놓여 있어
    순위가 쉽게 뒤집히기 때문이다 (실측 증상). 정렬 결과를 이어받으려면 중앙 기준이어야 한다.

    이력(2026-09-23): 예전엔 가장 가까운 후보 **하나만** 보고 그게 도달거리에서 걸리면
    바로 None 을 냈다. 그러면 뒤에 있던 진짜 트레이를 한 번도 안 보고 '못 찾음'이 된다.
    이제 걸린 후보는 건너뛰고 다음 후보로 넘어간다."""
    cands = list(iter_tray_optical(verbose=verbose))
    if centered_first:
        cands.sort(key=lambda c: abs(float(c[0][0])))
    for i, (point_optical, conf) in enumerate(cands):
        off = abs(float(point_optical[0]))
        if centered_first and off > CENTER_TOL_M:
            print(f"   후보{i + 1} 무시 — 화면 중앙에서 {off * 100:.0f}cm 벗어났다 "
                  f"(허용 {CENTER_TOL_M * 100:.0f}cm). 정렬한 그 트레이가 아니다")
            continue
        tray_world = optical_to_world(point_optical, color_camera_path)
        grasp = grasp_point_base(tray_world, base_pos, base_quat)
        reach = float(np.linalg.norm(grasp))
        if not (GRASP_REACH_MIN_M <= reach <= GRASP_REACH_MAX_M):
            print(f"   후보{i + 1} 무시 — 파지점이 base 에서 {reach:.3f}m "
                  f"({GRASP_REACH_MIN_M}~{GRASP_REACH_MAX_M}m 밖). 다음 후보로 넘어간다")
            continue
        print(f"   트레이 conf {conf:.2f}  cam {vec(point_optical, 3)}  "
              f"world {vec(tray_world)}  파지(base) {vec(grasp, 4)}  "
              f"reach {reach:.3f}m  yaw {approach_direction(grasp)[1]:+.1f}deg")
        return grasp, tray_world
    return None


# ══════════════════════════════════════════════════════════════
#  손목 특이점 가드
# ══════════════════════════════════════════════════════════════
class SingularityGuardedIK:
    """IK 해가 한 프레임에 크게 튀면 **경고만 찍는** 래퍼. 지금은 막지 않는다.

    joint 5 가 0 에 가까우면 joint 4/6 축이 일직선이 되어 자코비안이 퇴화하고,
    현재 자세를 warm start 로 줘도 IK 가 다른 해로 넘어간다.
    예전엔 그런 프레임을 미해결로 돌려보냈는데, 지령이 안 나가 팔이 제자리에 서면서
    스텝이 통째로 실패했다. 막는 대신 통과시키고 진단 로그만 남긴다 —
    다시 막으려면 아래 `return action, solved` 를 `return action, False` 로 되돌릴 것."""

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
            return action, solved

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
            return action, solved
        if abs(current[4]) < np.radians(WRIST_NEAR_SINGULAR_DEG):
            carb.log_warn(f"손목 특이점 근처 — joint_5 = {np.degrees(current[4]):+.1f}도")
        return action, solved


# ══════════════════════════════════════════════════════════════
#  Pick & Place 시퀀스
# ══════════════════════════════════════════════════════════════
def ease_alpha(u, times=1):
    """코사인 S-커브를 times 번 겹친다. u in [0,1] -> [0,1].

    1번(기본): f(u) = 0.5 - 0.5cos(pi u). 양끝 **속도**는 0 이지만 **가속도**는 양끝에서
      최대(|f''(0)| = |f''(1)| = pi^2/2)다. 즉 도착하는 순간이 감속 충격의 정점이라,
      들고 있는 트레이가 그때 흔들린다.
    2번: g = f(f(u)). g'(1) = f'(f(1)) * f'(1) = 0 이고 g''(1) = f'(1) * f''(1) = 0 이라
      **도착 가속도까지 0** 이 된다. 점차 느려지며 멈춘다.
      대신 중간 최고 속도가 선형의 pi/2 -> pi^2/4 배로 1.57배 올라간다
      (같은 거리를 같은 스텝 수에 가므로). 중간이 너무 빠르면 그 스텝만 "speed": "slow" 를 줄 것.
    """
    for _ in range(times):
        u = 0.5 - 0.5 * np.cos(np.pi * u)
    return u


def steps_for_pose(start_pos, target_pos, start_quat, target_quat, speed=None):
    """구간 길이(위치/자세)를 속도로 나눠 스텝 수를 정한다. speed 는 SPEEDS 의 등급"""
    tcp_v, joint_v, lo = SPEEDS[speed]
    dist = float(np.linalg.norm(target_pos - start_pos))
    dot = float(np.clip(abs(np.dot(start_quat, target_quat)), -1.0, 1.0))
    angle_deg = np.degrees(2.0 * np.arccos(dot))
    n = max(dist / tcp_v, angle_deg / joint_v)
    return int(np.clip(n, lo, MAX_STEPS))


def steps_for_joint(start_joints, target_joints, speed=None):
    _, joint_v, lo = SPEEDS[speed]
    delta_deg = np.degrees(np.abs(np.array(target_joints) - np.array(start_joints)))
    n = float(np.max(delta_deg)) / joint_v
    return int(np.clip(n, lo, MAX_STEPS))


def build_scan_steps():
    """joint_1 을 SCAN_ROTATE_DEG 만큼 상대 회전해 손목 카메라로 트레이 쪽을 본다.
    도착 후 정지 대기 + 새 검출 확인은 main() 의 상태 기계가 맡는다."""
    scan_delta = np.zeros(6)
    scan_delta[0] = np.radians(SCAN_ROTATE_DEG)
    return [{
        "type": "joint",
        "label": f"스캔 회전({SCAN_ROTATE_DEG:+.0f}deg)",
        "target": lambda start: start + scan_delta,
        "gripper": "open",
    }]


def current_tcp_pose(ik_solver):
    """현재 팔의 실제 TCP pose(월드 pos, quat).

    scan 회전은 상대 조인트 이동이라 도착 위치를 미리 계산할 수 없다 — 회전이
    끝난 뒤 실제로 어디에 있는지 읽어서 다음 목표(하강)를 만드는 데 쓴다.
    PickPlaceSequence._current_flange_pose 와 같은 계산이다."""
    flange_pos, flange_rot = ik_solver.compute_end_effector_pose()
    pos = flange_pos + flange_rot @ TCP_OFFSET
    quat = make_target_quat(*matrix_to_rpy(flange_rot))
    return pos, quat


def camera_world_right(color_camera_path):
    """컬러 카메라의 현재 world 기준 right(local +X) 단위벡터"""
    cam_prim = omni.usd.get_context().get_stage().GetPrimAtPath(color_camera_path)
    mat = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
    right = np.array(mat.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)), dtype=float)
    return right / np.linalg.norm(right)


def build_descend_step(ik_solver, base_pos, base_quat, target_z_base, color_camera_path, pitch_deg=PITCH_DOWN_DEG):
    """스캔 회전 직후 현재 위치를 유지한 채, **base 기준** z 만 target_z_base 로 낮추고
    카메라를 pitch_deg(기본 PITCH_DOWN_DEG) 만큼 아래로 기울인다. 랙 관측 숙임에도 재사용한다.

    z 만 바꾸고 자세(quat)를 그대로 두면 보는 '방향'은 안 바뀐다 — 저장된 검출
    이미지로 확인해보니 카메라가 책상보다 위(벽)를 보고 있었다. 카메라의 현재
    world right 축 기준으로 회전을 얹어 아래로 기울인다.

    current_tcp_pose() 가 주는 건 월드 좌표다. base_pos/base_quat 없이 z 를 바로
    덮어쓰면 base_link 의 월드 높이(약 0.4m)를 무시한 채 '월드 z=target_z_base'
    로 가버린다 — base 가 nova_carter 위에 얹혀 있어 world != base 이므로 이 변환이
    반드시 필요하다."""
    pos_world, quat = current_tcp_pose(ik_solver)
    pos_base = world_to_base_pos(pos_world, base_pos, base_quat)
    pos_base[2] = target_z_base
    target_world = np.array(base_pos, dtype=float) + quat_to_matrix(base_quat) @ pos_base

    right = camera_world_right(color_camera_path)
    # 카메라 up 이 대략 world +Z 라는 가정 하에 +각도가 아래로 기운다(수치로 검산함).
    # 반대로 기울면 PITCH_DOWN_DEG 부호부터 뒤집을 것 — SCAN_ROTATE_DEG 와 같은 처지다.
    tilt_q = quat_from_axis(right, pitch_deg)
    target_quat = quat_mul(tilt_q, quat)
    target_quat /= np.linalg.norm(target_quat)

    return [{
        "type": "pose",
        "label": f"디텍션 하강+하향({pitch_deg:+.0f}deg, base z={target_z_base:.2f})",
        "target": (target_world, target_quat),
        "gripper": "open",
    }]


def build_observe_step():
    """관측 자세로 이동. 관절 보간이라 IK 를 안 타고, 이동+숙임이 한 스텝으로 끝난다."""
    return [{"type": "joint", "label": "랙 관측 자세(고정 관절값)",
             "target": np.radians(OBSERVE_JOINTS_DEG), "gripper": None}]


def assign_slots(markers):
    """[(id, slot)] -> 칸별 id [s0, s1, s2] (없으면 -1). 같은 칸에 둘이면 먼저 온 것.

    이전엔 마커를 3D 로 옮겨 RACK_SLOTS 와 x 거리를 쟀는데, 깊이 오차가 양옆 칸 x 로 번져
    반 피치 밖으로 밀려나 버려지는 일이 있었다(실측 [0, 11, 2]). 칸은 관제 PC 의 픽셀 3등분."""
    out = [-1] * len(RACK_SLOTS)
    for mid, slot in markers:
        if 0 <= slot < len(out) and out[slot] < 0:
            out[slot] = mid
    return out


def merge_urgencies(votes):
    """프레임별 칸 배정 결과 [[s0,s1,s2], ...] -> 칸별 최빈 id (-1 은 표로 안 친다). 한 번도 없으면 -1."""
    out = []
    for k in range(len(RACK_SLOTS)):
        seen = [v[k] for v in votes if v[k] >= 0]
        out.append(max(set(seen), key=seen.count) if seen else -1)
    return out


def format_urgencies(urg):
    return "   ".join(f"{k + 1}번: " + (f"긴급도 {URGENCY_NAMES.get(u, '?')}({u})" if u >= 0 else "미검출(-1)")
                      for k, u in enumerate(urg))


def build_center_step(ik_solver, color_camera_path, point_optical):
    """검출점의 카메라 광학 x(오른쪽 +)만큼 카메라 기준 수평으로 옮겨 ROI 를
    화면 중앙으로 맞춘다. 수직(y)은 그대로 둔다 — "수평 이동" 지시대로.

    역투영된 x_optical = (u-cx)*z/fx 은 이미 '그 깊이에서 광축으로부터 실제로
    얼마나 떨어져 있는지'(미터)다. 그래서 픽셀/ROI 를 따로 다룰 필요 없이 이
    값만큼 카메라를 그 방향으로 옮기면 중앙에 온다(카메라가 x_optical 만큼
    오른쪽으로 이동하면, 안 움직인 물체의 상대 오프셋이 그만큼 줄어든다).
    카메라는 TCP 에 강체로 붙어 있으니 TCP 를 같은 양만큼 옮기면 카메라도 같이
    옮겨진다."""
    cam_right = camera_world_right(color_camera_path)
    x_offset = float(point_optical[0])
    pos_world, quat = current_tcp_pose(ik_solver)
    target_world = pos_world + cam_right * x_offset
    # slow — 보간 속도를 2.5배로 올린 뒤 이 스텝이 60 -> 24 스텝이 됐다. 23cm 를 0.4초
    # (약 0.6 m/s)에 옆으로 빼면, 이미 도달거리 가장자리인 자세에서 IK 가 따라오지 못한다.
    # 특이점 가드가 지금 거부를 안 하고 경고만 찍으므로 엉뚱한 해가 그대로 팔에 나간다.
    return [{
        "type": "pose",
        "label": f"ROI 중앙 정렬(수평 {x_offset * 100:+.1f}cm)",
        "target": (target_world, quat),
        "gripper": "open",
        "speed": "slow",
    }]


def tcp_pose_base(lula, joints, base_pos, base_quat):
    """관절각이 joints 일 때 TCP 가 어디인지 (base 기준). 팔을 안 움직이고 미리 본다."""
    pos, rot = lula.compute_forward_kinematics(EE_LINK_NAME, np.asarray(joints, dtype=float))
    return world_to_base_pos(pos + rot @ TCP_OFFSET, base_pos, base_quat)


def joint_retreat_step(lula, base_pos, base_quat, max_drop_m=None):
    """회전 전 후퇴를 관절 공간으로 한다. IK 를 안 거치니 손목 특이점에 안 걸린다.

    어깨/팔꿈치를 접고 joint_5 로 되돌려 도구 기울기(= 트레이 기울기)를 유지한다.
    접는 부호는 FK 로 양쪽을 재서 수평 도달거리가 줄어드는 쪽을 고른다.
    max_drop_m 을 주면 z 가 그보다 많이 떨어지는 부호를 먼저 걸러낸다 (RETREAT_MAX_DROP_M 참고)."""
    def target(start):
        delta = np.radians([0.0, RETREAT_SHOULDER_DEG, RETREAT_ELBOW_DEG, 0.0,
                            -(RETREAT_SHOULDER_DEG + RETREAT_ELBOW_DEG), 0.0])
        here = tcp_pose_base(lula, start, base_pos, base_quat)
        here_reach = float(np.linalg.norm(here[:2]))

        cands = []
        for sign in (1.0, -1.0):
            cand = start + sign * delta
            there = tcp_pose_base(lula, cand, base_pos, base_quat)
            cands.append((sign, cand, float(np.linalg.norm(there[:2])), float(here[2] - there[2]), there))

        ok = cands if max_drop_m is None else [c for c in cands if c[3] <= max_drop_m]
        if ok:
            best = min(ok, key=lambda c: c[2])      # 수평 도달거리가 가장 많이 줄어드는 쪽
        else:
            best = min(cands, key=lambda c: c[3])   # 둘 다 상한 초과 — 낙폭이라도 작은 쪽
            print(f"   retreat      경고: 두 부호 모두 낙폭이 {max_drop_m * 100:.0f}cm 를 넘는다 "
                  f"— 접는 각도를 줄이거나 RACK_ABOVE_Z_M 을 키울 것")
        for sign, _cand, reach, drop, there in cands:
            print(f"   retreat      {sign:+.0f}: 수평 도달 {here_reach:.3f} -> {reach:.3f} m, "
                  f"z {here[2]:.3f} -> {there[2]:.3f} (낙폭 {drop:+.3f})"
                  + ("  <- 채택" if sign == best[0] else ""))
        return best[1]

    return {"type": "joint", "label": "로봇 쪽 후퇴(관절)", "target": target, "gripper": None}


def home_step():
    return {"type": "joint", "label": "홈 복귀",
            "target": np.array(READY_JOINTS_RAD, dtype=float), "gripper": None}


def wrist_unlock_step():
    """joint_5 를 0(손목 특이점) 에서 WRIST_UNLOCK_DEG 만큼 띄운다. 실패한 스텝 재시도 전에 쓴다.

    joint_3 으로 같은 양을 되돌리므로 도구 기울기는 그대로다 — 들고 있는 트레이가 안 기운다."""
    def target(start):
        unlock = np.radians(WRIST_UNLOCK_DEG) * (-1.0 if start[4] <= 0 else 1.0)
        delta = np.zeros(6)
        delta[2], delta[4] = -unlock, unlock
        return start + delta

    return {"type": "joint", "label": "손목 풀기(재시도)", "target": target, "gripper": None}


def build_return_steps(lula, grasp_base, base_pos, base_quat):
    """들고 있던 트레이를 원래 집은 자리에 되돌려 놓고 홈으로. 적재 실패 복구용.

    그 자리는 방금 집어 온 곳이라 비어 있다 — 아무 데나 떨어뜨리는 것보다 안전하다."""
    direction, grasp_quat = grasp_frame(
        grasp_base, pick_grasp_yaw_weight(lula, grasp_base, base_pos, base_quat))

    def to_world_q(tcp):
        return base_pose_to_world(tcp, grasp_quat, base_pos, base_quat)

    above = np.array([grasp_base[0], grasp_base[1], LIFT_Z_M])
    approach = grasp_base - direction * APPROACH_BACKOFF_M
    return [
        {"type": "pose", "label": "되돌려놓기 위", "target": to_world_q(above), "gripper": None},
        {"type": "pose", "label": "원래 자리",     "target": to_world_q(grasp_base), "gripper": None,
         "tol": STEP_CONTACT_TOL_M},
        {"type": "hold", "label": "그리퍼 열기",   "gripper": "open"},
        {"type": "pose", "label": "후퇴",          "target": to_world_q(approach), "gripper": None},
        home_step(),
    ]


def reset_tray_align():
    """Play 를 다시 누를 때. 트레이가 원래 자리로 돌아가므로 정렬 기록도 지운다.

    rigid body 핸들도 버린다 — stop/play 로 물리 뷰가 새로 만들어지면 예전 핸들은 죽는다."""
    global _held_tray
    _held_tray = None
    _aligned_trays.clear()
    _tray_rigid.clear()


def tray_body(path):
    """트레이 rigid body 핸들. 처음 쓸 때 만들어 두고 재사용한다"""
    body = _tray_rigid.get(path)
    if body is None:
        body = SingleRigidPrim(path, name="tray_body_" + path.strip("/").replace("/", "_"))
        body.initialize()
        _tray_rigid[path] = body
    return body


def nearest_tray_body(world_xy):
    """world_xy 에 가장 가까운, 아직 정렬하지 않은 트레이 rigid body 의 (경로, 거리).

    이미 정렬한 것을 후보에서 빼는 게 핵심이다 — 2·3번 칸을 다룰 때 옆 칸에 이미 세워 둔
    트레이가 후보로 들어오지 않는다. 트레이 하나당 정확히 한 번만 손댄다."""
    best, best_d = None, None
    for path in _tray_bodies:
        if path in _aligned_trays:
            continue
        pos, _ = tray_body(path).get_world_pose()
        dist = float(np.linalg.norm(np.asarray(pos, dtype=float)[:2] - np.asarray(world_xy)[:2]))
        if best_d is None or dist < best_d:
            best, best_d = path, dist
    return best, best_d


def rack_square_quat(quat_now_world, base_quat):
    """지금 자세를 '랙과 나란한' 가장 가까운 자세로 스냅한 월드 쿼터니언.

    랙은 base 축에 정렬돼 있다 (칸이 base x 축으로 늘어선다). 그리고 스폰 자세는 씬이
    만든 '반듯한' 기준이다 — 책상 위에 반듯하게 놓여 있고 IK 를 안 거쳤다. 그래서 랙에서
    반듯한 자세는 **스폰 자세를 base z 축으로 90도의 정수배만큼 돌린 것들** 중 하나다.

    정수배 k 는 **지금 자세에서 읽는다.** 그래서 90도인지 -90도인지 같은 사분면을 상수로
    정할 필요가 없다 — 놓인 트레이는 정답에서 10도 안쪽이라 반올림이 항상 맞는 칸을 고른다.

    roll/pitch 는 스폰 값을 그대로 쓴다 (책상에서도 랙에서도 평평하다). 그리퍼에 물린
    각도는 **보지 않는다** — 삐뚤게 물렸으면 그게 그대로 남는다는 게 이전 판의 문제였다."""
    spawn_base = _tray_spawn_quat_base
    now_base = quat_mul(quat_conj(base_quat), np.asarray(quat_now_world, dtype=float))
    k = round((tray_yaw_deg(now_base) - tray_yaw_deg(spawn_base)) / 90.0)
    target_base = quat_mul(quat_from_axis([0, 0, 1], k * 90.0), spawn_base)
    target = quat_mul(base_quat, target_base)
    return target / np.linalg.norm(target)


def capture_tray_in_gripper(seq, slot):
    """그리퍼를 열기 **직전에** 트레이 원점이 TCP 기준 어디에 있는지 잰다 (p_rel).

    이 스텝의 on_enter 는 손가락이 아직 닫혀 있고 팔이 슬롯에 멈춰 있는 순간에 불린다.

    p_rel 이 필요한 이유: set_world_pose 는 **body 원점**을 옮기는데 그게 트레이 형상
    중심이 아닐 수 있다. 파지점(grasp_point_base)은 검출면에서 TRAY_HALF_DEPTH_M 만큼
    민 값이라 **TCP 가 트레이 수평 중심**에 온다. 그래서 p_rel 이 곧 '형상 중심 대비
    body 원점의 어긋남'이고, 이걸 슬롯 좌표에 더해야 트레이가 칸 **중앙**에 선다.

    자세는 여기서 안 쓴다 — 물린 각도를 물려받으면 삐뚤게 물린 게 그대로 남는다."""
    global _held_tray
    _held_tray = None
    if not RACK_TRAY_ALIGN:
        return
    try:
        tcp_pos, tcp_quat = seq._current_flange_pose()
        path, dist = nearest_tray_body(tcp_pos)
        if path is None or dist > RACK_TRAY_MATCH_R_M:
            near = "없다" if dist is None else f"최근접 {dist * 100:.1f}cm"
            print(f"   rack align   랙 {slot + 1}번 — 그리퍼 근처에 트레이가 {near}. 정렬을 건너뛴다")
            return
        pos, _quat = tray_body(path).get_world_pose()
        _held_tray = {
            "path": path,
            "slot": slot,
            "p_rel": quat_to_matrix(tcp_quat).T @ (np.asarray(pos, dtype=float) - tcp_pos),
        }
    except Exception as exc:
        print(f"   rack align   랙 {slot + 1}번 파지 상태 측정 실패 (정렬 생략): {exc}")


def align_tray_on_rack(slot, base_pos, base_quat):
    """방금 놓은 트레이를 칸 중앙·칸 방향으로 한 번 맞춰 놓는다. 후퇴 스텝 진입 때 한 번.

    설정: 랙 칸에 정렬 가이드(자석)가 있어 트레이가 칸에 들어가면 칸 중앙·칸 방향으로
    붙는다. 그 결과를 트레이 프림 pose 에 직접 반영한다 — **팔 동작은 건드리지 않는다.**
    기운 원인이 IK_ORIENTATION_TOLERANCE 를 7도까지 푼 것이라, 같은 IK 로 다시 세우려
    하면 또 7도 어긋난다 (원인을 도구로 쓰는 셈이라 수렴하지 않는다).

    자세: rack_square_quat — 스폰 자세가 정의하는 90도 격자에 **절대 기준으로** 스냅한다.
    위치: 슬롯 중심(RACK_SLOTS[slot]) 의 xy + p_rel 로 body 원점 어긋남 보정.

    **z 는 현재값을 그대로 둔다** — 수직 위치는 트레이가 칸 바닥에 앉으면서 물리가 이미
    정확히 잡아 준 값이다. 계산값을 넣으면 박히거나 떠서 튄다.

    rigid body 라 한 번만 맞춰 두면 다른 것이 건드리지 않는 한 그대로 실려 간다.
    단 **선/각속도를 0 으로 눌러야** 한다 — 안 그러면 남아 있던 각속도로 PhysX 가
    방금 맞춘 것을 곧바로 다시 틀어 놓는다. '1회만'이 성립하는 건 이 두 줄 덕분이다.

    이력: 처음엔 '팔이 슬롯에 정확히 도달했다면 트레이가 있었을 자리'로 잡아서, 그리퍼에
    물린 상대 자세(q_rel)를 그대로 얹었다. 놓을 때의 IK 오차는 지워졌지만 **트레이가
    그리퍼에 삐뚤게 물린 것은 그대로 남아** 랙에서 여전히 기울어 보였다 (2026-09-24 실측).
    물린 각도를 아예 안 보고 절대 기준으로 스냅하는 지금 방식으로 바꿨다.

    보정하지 못해도 미션을 멈추지 않는다. 로그만 남기고 후퇴는 그대로 진행한다."""
    global _held_tray
    held, _held_tray = _held_tray, None
    if not RACK_TRAY_ALIGN or held is None or held["slot"] != slot:
        return
    if held["path"] in _aligned_trays:   # 손목 풀기 재시도로 스텝이 다시 실행된 경우
        return
    try:
        body = tray_body(held["path"])
        pos, quat = body.get_world_pose()
        pos = np.asarray(pos, dtype=float)

        target_quat = rack_square_quat(quat, base_quat)
        # 슬롯 중심에 트레이 수평 중심이 오도록. p_rel 은 이상적인 TCP 자세로 돌려서 얹는다
        ideal_pos, ideal_quat = base_to_world(RACK_SLOTS[slot], POINT4_RPY, base_pos, base_quat)
        target_pos = ideal_pos + quat_to_matrix(ideal_quat) @ held["p_rel"]

        base_rot_t = quat_to_matrix(base_quat).T
        yaw_before = tray_yaw_deg(quat_mul(quat_conj(base_quat), np.asarray(quat, dtype=float)))
        yaw_after = tray_yaw_deg(quat_mul(quat_conj(base_quat), target_quat))
        before_rpy = matrix_to_rpy(base_rot_t @ quat_to_matrix(np.asarray(quat, dtype=float)))
        after_rpy = matrix_to_rpy(base_rot_t @ quat_to_matrix(target_quat))
        moved = float(np.linalg.norm(target_pos[:2] - pos[:2]))

        body.set_world_pose(position=np.array([target_pos[0], target_pos[1], pos[2]]),
                            orientation=target_quat)
        body.set_linear_velocity(np.zeros(3))
        body.set_angular_velocity(np.zeros(3))
        _aligned_trays.add(held["path"])

        print(f"   rack align   랙 {slot + 1}번  {held['path']}")
        print(f"                방위각(base) {yaw_before:+.1f} -> {yaw_after:+.1f}deg "
              f"({yaw_after - yaw_before:+.1f})   xy {moved * 100:.1f}cm 이동")
        print(f"                rpy(base) {vec(before_rpy, 1)} -> {vec(after_rpy, 1)}")
    except Exception as exc:
        print(f"   rack align   랙 {slot + 1}번 정렬 실패 (무시하고 계속): {exc}")


def build_pick_steps(lula, grasp_base, base_pos, base_quat, slot):
    """검출로 구한 grasp_base(파지점, base 기준)로 집어서 RACK_SLOTS[slot] 에 놓는다.

    build_sequence 와 같은 형태지만 POINT1/POINT2(파지 지점)만 검출값으로
    갈아끼운다. place 쪽(POINT4/POINT5)은 고정값 그대로지만, 그리로 도는 joint_1
    회전량은 고정값이 아니라 파지 방향 -> place 방향의 실제 차이로 계산한다 —
    파지 위치가 검출값이라 매번 달라지므로 고정 각도로는 정확히 못 돌아온다.
    "pick 쪽만" 검출을 쓰라는 요청에 맞춘 것이다."""
    def to_world(tcp, rpy):
        return base_to_world(tcp, rpy, base_pos, base_quat)

    direction, grasp_quat = grasp_frame(
        grasp_base, pick_grasp_yaw_weight(lula, grasp_base, base_pos, base_quat))

    def to_world_q(tcp):
        return base_pose_to_world(tcp, grasp_quat, base_pos, base_quat)

    approach = grasp_base - direction * APPROACH_BACKOFF_M
    lift = np.array([grasp_base[0], grasp_base[1], LIFT_Z_M])
    place = RACK_SLOTS[slot]
    p4 = to_world(place, POINT4_RPY)
    # 슬롯 바로 위 — x, y 는 놓는 위치와 같고 z 만 높다. 여기서 수직으로 내려간다
    p4_above = to_world(place + np.array([0.0, 0.0, RACK_ABOVE_Z_M]), POINT4_RPY)
    p5 = to_world(place + RACK_RETREAT, POINT5_RPY)

    joint1_delta = np.zeros(6)
    joint1_delta[0] = np.radians(rack_yaw_delta_deg(grasp_base, place))  # 고정 JOINT1_ROTATE_DEG 대신 계산값

    return [
        {"type": "pose",  "label": "안전 위치(파지 전)", "target": to_world_q(approach), "gripper": "open"},
        {"type": "pose",  "label": "파지 위치",          "target": to_world_q(grasp_base), "gripper": None, "speed": "slow"},
        {"type": "hold",  "label": "그리퍼 닫기",        "gripper": "close"},
        {"type": "pose",  "label": "들어올리기",         "target": to_world_q(lift), "gripper": None, "speed": "slow"},
        # carry — 여기서부터 랙에 놓을 때까지는 트레이를 들고 있다. 최대 86도 회전을
        # 2.5배로 돌리면 트레이가 흔들린다. 1.5배 + ease 2 로 감가속을 부드럽게 한다
        {"type": "joint", "label": f"joint_1 {np.degrees(joint1_delta[0]):+.1f}deg",
         "target": lambda start: start + joint1_delta, "gripper": None,
         "speed": "carry", "ease": 2},
        # 놓는 위치로 대각선으로 내려가면 트레이가 랙 테두리에 걸린다.
        # 슬롯 바로 위로 먼저 가서, 거기서 수직으로만 내려간다.
        # ease 2 — 여기 도착할 때의 감속 충격이 트레이를 흔들어, 바로 다음 하강이
        # 흔들리는 중에 시작됐다. 도착 가속도를 0 으로 만들어 점차 느려지며 멈추게 한다.
        {"type": "pose",  "label": f"놓기 위 안전 위치(+{RACK_ABOVE_Z_M * 100:.0f}cm)",
         "target": p4_above, "gripper": None, "speed": "carry", "ease": 2},
        {"type": "hold",  "label": f"하강 전 대기({PLACE_WAIT_STEPS / 60:.0f}s)", "gripper": None, "steps": PLACE_WAIT_STEPS},
        {"type": "pose",  "label": f"랙 {slot + 1}번 놓기(수직 하강)", "target": p4, "gripper": None, "tol": STEP_CONTACT_TOL_M, "speed": "slow"},
        # 랙 정렬 2단계. 여기 진입 시점은 손가락이 **아직 닫혀 있고** 팔이 슬롯에 멈춰 있는
        # 순간이라, 트레이가 그리퍼에 물린 상대 pose 를 재기에 맞다
        {"type": "hold",  "label": "그리퍼 열기",        "gripper": "open",
         "on_enter": lambda seq: capture_tray_in_gripper(seq, slot)},
        # 앞 스텝이 GRIPPER_WAIT_STEPS(2초)를 다 쓴 뒤라 손가락은 이미 벌어져 있다 — 물린 채로
        # 돌리는 일이 없다. 하역(책상)에는 안 건다: 랙에만 정렬 가이드가 있다는 설정이다
        {"type": "pose",  "label": "후퇴 안전 위치",     "target": p5, "gripper": None, "speed": "slow",
         "on_enter": lambda seq: align_tray_on_rack(slot, base_pos, base_quat)},
        {"type": "joint", "label": "홈 복귀",            "target": np.array(READY_JOINTS_RAD, dtype=float), "gripper": None},
    ]


def build_unload_steps(lula, slot, desk_z, base_pos, base_quat):
    """RACK_SLOTS[slot] 의 트레이를 꺼내 DESK_SLOTS[slot] 에 놓는다.

    적재의 역순이 아니라 하역 전용 시퀀스다 — 넣을 때와 뺄 때 걸리는 조건이 달라서
    들어올리는 높이를 UNLOAD_ABOVE_Z_M 으로 따로 둔다.
    랙 쪽은 적재와 같은 고정 자세(POINT4/5_RPY), 책상 쪽은 세 자리 모두
    DESK_ROW_CENTER 기준 자세 하나로 놓아 트레이가 서로 평행하게 정렬된다."""
    def to_world(tcp, rpy):
        return base_to_world(tcp, rpy, base_pos, base_quat)

    direction, desk_quat = grasp_frame(DESK_ROW_CENTER)

    def to_world_q(tcp):
        return base_pose_to_world(tcp, desk_quat, base_pos, base_quat)

    place = RACK_SLOTS[slot]
    p_above = place + np.array([0.0, 0.0, UNLOAD_ABOVE_Z_M])
    desk = np.array([DESK_SLOTS[slot][0], DESK_SLOTS[slot][1], desk_z])
    lift = np.array([desk[0], desk[1], LIFT_Z_M])
    lift_back = lift - direction * LIFT_BACK_M
    approach = desk - direction * APPROACH_BACKOFF_M

    joint1_delta = np.zeros(6)
    joint1_delta[0] = np.radians(rack_yaw_delta_deg(place, desk))

    return [
        {"type": "pose",  "label": f"랙 {slot + 1}번 앞(후퇴점)", "target": to_world(place + RACK_RETREAT, POINT5_RPY), "gripper": "open"},
        {"type": "pose",  "label": f"랙 {slot + 1}번 진입",      "target": to_world(place, POINT4_RPY), "gripper": None, "speed": "slow"},
        {"type": "hold",  "label": "그리퍼 닫기",              "gripper": "close"},
        {"type": "pose",  "label": f"수직 들어올리기(+{UNLOAD_ABOVE_Z_M * 100:.0f}cm)", "target": to_world(p_above, POINT4_RPY), "gripper": None, "speed": "slow"},
        # carry — 트레이를 들고 도는 구간. 적재 쪽과 같은 이유로 1.5배 + ease 2
        {"type": "joint", "label": f"joint_1 {np.degrees(joint1_delta[0]):+.1f}deg",
         "target": lambda start: start + joint1_delta, "gripper": None,
         "speed": "carry", "ease": 2},
        # 적재 쪽과 대칭. ease 2 로 도착 가속도를 0 으로 만들고, 그래도 남는 흔들림은
        # 대기로 가라앉힌 뒤에 수직 하강한다 (적재의 '놓기 위 안전 위치' + '하강 전 대기' 와 같은 구성)
        {"type": "pose",  "label": "책상 위",                  "target": to_world_q(lift), "gripper": None, "speed": "carry", "ease": 2},
        {"type": "hold",  "label": f"하강 전 대기({PLACE_WAIT_STEPS / 60:.0f}s)", "gripper": None, "steps": PLACE_WAIT_STEPS},
        {"type": "pose",  "label": f"책상 {slot + 1}자리 내려놓기(수직 하강)", "target": to_world_q(desk), "gripper": None, "tol": STEP_CONTACT_TOL_M, "speed": "slow"},
        {"type": "hold",  "label": "그리퍼 열기",              "gripper": "open"},
        {"type": "pose",  "label": "후퇴",                     "target": to_world_q(approach), "gripper": None, "speed": "slow"},
        # 홈 복귀는 관절 보간이라 데카르트 경로를 보장하지 않는다. 트레이 높이에서 바로 돌리면
        # 그리퍼가 책상 위를 휩쓸며 방금 놓은 트레이를 친다 — 높고 뒤로 물러난 곳에서 출발한다
        {"type": "pose",  "label": "책상 위(후퇴 반경)로 올리기", "target": to_world_q(lift_back), "gripper": None},
        {"type": "joint", "label": "홈 복귀",                  "target": np.array(READY_JOINTS_RAD, dtype=float), "gripper": None},
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
        self.failed = None       # 도달 실패로 중단됐으면 사유 문자열
        self.step_rejects = 0    # 현재 스텝에서 IK 가 거부된 틱 수

    @property
    def current(self):
        return self.steps[self.index]

    def skip(self):
        """실패한 스텝을 버리고 다음 스텝부터 계속한다. 뒤 스텝은 절대 목표라 그대로 이어진다."""
        self.index += 1
        self.step_tick = 0
        self.step_rejects = 0
        self.start_pos = self.start_quat = self.start_joints = None
        self.failed = None
        self.done = self.index >= len(self.steps)   # tick() 이 실패 때 세운 done 을 되돌린다

    def _current_flange_pose(self):
        flange_pos, flange_rot = self._ik.compute_end_effector_pose()
        pos = flange_pos + flange_rot @ TCP_OFFSET
        quat = make_target_quat(*matrix_to_rpy(flange_rot))
        return pos, quat

    def _enter_step(self):
        step = self.current
        if step["gripper"] is not None:
            self.gripper = step["gripper"]

        # 스텝에 진입할 때 한 번 부르는 선택 훅. 씬 쪽 부수 효과용이고 팔은 건드리지 않는다
        # (지금 쓰는 곳: build_pick_steps 의 그리퍼 열기 -> 파지 상태 측정, 후퇴 -> 트레이 정렬)
        if step.get("on_enter") is not None:
            step["on_enter"](self)

        if step["type"] in ("pose", "hold"):
            self.start_pos, self.start_quat = self._current_flange_pose()
            if step["type"] == "hold":
                self.target_pos, self.target_quat = self.start_pos, self.start_quat
                # steps 키가 있으면 그 길이만큼 제자리 대기한다 (없으면 그리퍼 여닫이 기준값)
                self.n_steps = int(step.get("steps", GRIPPER_WAIT_STEPS))
            else:
                self.target_pos, self.target_quat = step["target"]
                self.n_steps = steps_for_pose(
                    self.start_pos, self.target_pos, self.start_quat, self.target_quat,
                    speed=step.get("speed")
                )
        else:  # joint
            q = self._robot.get_joint_positions()
            self.start_joints = np.array([q[i] for i in self._arm_indices])
            target = step["target"]
            self.target_joints = target(self.start_joints) if callable(target) else np.array(target)
            self.n_steps = steps_for_joint(self.start_joints, self.target_joints,
                                           speed=step.get("speed"))

        print(f"   [{self.index}] {step['label']:16s} {self.n_steps:4d} steps   gripper {self.gripper}")

    def tick(self):
        """한 스텝 진행한다. 시퀀스가 끝났으면 아무 것도 하지 않는다"""
        if self.done:
            return True

        if self.start_pos is None and self.start_joints is None:
            self._enter_step()

        step = self.current
        alpha = min(1.0, self.step_tick / float(self.n_steps))
        # 코사인 S-커브로 가감속을 준다. 선형이면 정지 순간의 충격으로 들고 있는 트레이가
        # 틀어진다. 스텝이 "ease" 를 주면 그만큼 겹쳐 도착 감속을 더 부드럽게 한다 — ease_alpha 참고
        alpha = ease_alpha(alpha, step.get("ease", 1))
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
        self.step_rejects += 0 if solved else 1

        self.step_tick += 1
        if self.step_tick >= self.n_steps:
            if step["type"] == "pose":
                pos_err, rot_err = self._pose_error()
                if pos_err > step.get("tol", STEP_POS_TOL_M) or rot_err > STEP_ROT_TOL_DEG:
                    if self.step_tick < self.n_steps + STEP_EXTRA_TICKS:
                        return solved            # 목표(alpha=1)를 계속 주며 더 기다린다
                    self.failed = (f"step {self.index} '{step['label']}' 도달 실패 — 위치 오차 "
                                   f"{pos_err * 100:.1f}cm, 자세 오차 {rot_err:.1f}deg, IK 거부 {self.step_rejects}틱")
                    self.done = True
                    return False
            self.index += 1
            self.step_tick = 0
            self.step_rejects = 0
            self.start_pos = self.start_quat = self.start_joints = None
            if self.index >= len(self.steps):
                self.done = True
                print("   [DONE] pick & place 완료")

        return solved

    def _pose_error(self):
        """현재 TCP 와 목표의 위치(m)/자세(deg) 오차"""
        pos, quat = self._current_flange_pose()
        dot = float(np.clip(abs(np.dot(quat, self.target_quat)), -1.0, 1.0))
        return float(np.linalg.norm(pos - self.target_pos)), float(np.degrees(2.0 * np.arccos(dot)))


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    check_math()

    section("SCENE")
    setup_people()          # 반드시 load_scene() 앞 (캐릭터 스크립트가 초기화 때 한 번만 읽는다)
    load_scene()
    try:
        setup_characters()  # 반드시 load_scene() 뒤 (SkelRoot 가 올라와 있어야 한다)
    except Exception as exc:
        # 사람이 안 걸을 뿐 적재/주행은 할 수 있다 — 여기서 실행을 죽이지 않는다
        print(f"   characters   연결 실패 (무시): {exc}")
    bake_navmesh()          # 매 실행 다시 구워야 한다 (USD 에 저장 안 됨)
    set_viewport_lighting()
    tray_origin = spawn_tray_copies()
    setup_arm_drives()
    setup_gripper_drive()

    world = World(stage_units_in_meters=1.0)
    robot = register_robot(world)

    world.reset()
    init_robot(robot, world)
    for _ in range(30):
        world.step(render=True)

    section("ROS2")
    color_camera_path = find_color_camera()
    print(f"   color cam    {color_camera_path}")
    build_detection_graph(color_camera_path)
    build_rear_lidar_graph()

    section("SOLVER")
    lula, ik_solver = create_ik_solver(robot)
    base_pos, base_quat = sync_base_pose(lula, robot)
    print(f"   base pos     {vec(base_pos)}")
    print(f"   tray origin  world {vec(tray_origin)}  base {vec(world_to_base_pos(tray_origin, base_pos, base_quat))}"
          f"   (DESK_ROW_CENTER xy {vec(DESK_ROW_CENTER[:2])})")

    arm_indices = [robot.get_dof_index(j) for j in ARM_JOINTS]
    ik_solver = SingularityGuardedIK(ik_solver, robot, arm_indices)

    section("RUN")
    print("   뷰포트를 클릭해 포커스를 준 뒤 Play 를 누른다\n")
    keys = KeyTap()

    # pick 상태 기계. None 이면 대기(정지했거나 아직 시작 전이거나 끝났다).
    #   scan    -> joint_1 을 돌려 손목 카메라가 트레이 쪽을 보게 한다
    #   descend -> 회전 직후 실제 도착 위치에서 z 만 낮춘다
    #   settle  -> 도착 후 정지, 새 검출 프레임을 기다린다
    #              (settle_next 로 끝나면 어디로 갈지 갈린다: "center" 또는 "pick")
    #   center  -> ROI(검출된 트레이)가 화면 중앙에 오도록 카메라 기준 수평 이동
    #   pick    -> 중앙 정렬 후 다시 검출한 좌표로 접근/파지/들어올리기/회전/
    #              놓기/후퇴/홈복귀 실행
    #   observe_go/observe_settle/observe_home
    #           -> 랙이 다 찬 뒤 랙을 대각선 위에서 보고 ArUco 로 칸별 긴급도를 출력한 뒤 홈으로.
    #              적재/하역 로직과는 무관하고, 못 읽어도 미검출(-1)로 찍고 그냥 진행한다
    #   recover -> 검출 실패 복구 (스텝 실패는 그 스텝만 건너뛰고 이어간다).
    #              끝나면 남은 칸이 있으면 scan, 아니면 관측으로 — 어떤 경우에도 미션을 멈추지 않는다
    #   wait_unload -> 주행 프로세스의 도킹 완료(/nav_done 1)를 기다린다. UNLOAD_KEY 로 수동 진행도 된다
    #   unload  -> 랙 3번부터 꺼내 책상 위 DESK_SLOTS 에 가로로 놓는다
    pick_state = None
    settle_next = None      # settle 완료 후 갈 곳: "center" | "pick"
    sequence = None
    seq_mark = -1
    wait_frames = 0
    retry_count = 0          # 트레이를 못 찾았을 때 재시도한 횟수
    slot_index = 0           # 다음에 채울 랙 칸. 1번부터 차례로 채운다
    unload_slot = -1         # 하역 중인 랙 칸 (3번 -> 1번 순)
    desk_z = 0.0             # 하역 시작 시 확정 (DESK_Z_M 또는 측정값)
    grasp_log = []           # 적재 때 쓴 파지점(base). 하역 z 상수 검증용
    votes, last_marker_seq, collect_start = [], -1, -1   # 랙 관측 누적 (observe_settle)
    step_retries = 0         # 현재 시퀀스에서 손목을 풀고 재시도한 횟수
    stage_fails = 0          # 적재 단계를 통째로 실패한 횟수

    was_playing = False
    fail_streak = 0
    tick_count = 0

    def begin_recover(reason, grasp=None):
        """적재 쪽 실패를 복구한다. 들고 있으면 원래 자리에 되돌려 놓고, 아니면 홈으로만 빠진다.

        여기서 미션을 멈추지 않는다 — 복구가 끝나면 남은 칸에 따라 다음 트레이 스캔 또는
        관측 단계로 넘어간다 (아래 pick_state == "recover" 처리)."""
        nonlocal pick_state, sequence, stage_fails, step_retries, tick_count
        held = sequence.gripper      # 새 시퀀스는 reset 에서 open 으로 시작한다 — 들고 있으면 놓아버린다
        stage_fails += 1
        step_retries = 0
        where = "트레이를 원래 자리에 되돌려 놓고" if grasp is not None else "홈으로 빠져"
        print(f"   복구         {reason} — {where} 계속한다 (적재 실패 {stage_fails}/{MAX_STAGE_FAILS})")
        sequence = PickPlaceSequence(
            robot, ik_solver, arm_indices,
            build_return_steps(lula, grasp, base_pos, base_quat) if grasp is not None else [home_step()])
        sequence.reset()
        sequence.gripper = held
        pick_state = "recover"
        tick_count = 0

    # Ctrl+C 로 끄면 simulation_app.close() 를 못 거치고 atexit 로 직행해서, render
    # product 와 replicator writer 가 붙은 채로 omni.graph / syntheticdata 가 해제되며
    # segfault 가 난다. 실제 오류 트레이스백까지 같이 묻히므로 받아서 정리한다.
    try:
        while simulation_app.is_running():
            world.step(render=True)
            time.sleep(0.005)

            is_playing = world.is_playing()
            if was_playing and not is_playing:
                # 정지하면 주행 프로세스와의 핸드셰이크를 양쪽 다 지운다.
                # 안 지우면 다음 Play 때 이전 주행의 /nav_done 1 이 남아 주행을 건너뛴다.
                publish_mission_state(0)
                clear_nav_done()
            if is_playing and not was_playing:
                init_robot(robot, world)
                seq_mark, wait_frames, retry_count = -1, 0, 0
                slot_index, unload_slot, grasp_log = 0, -1, []
                reset_tray_align()
                step_retries, stage_fails = 0, 0
                fail_streak = 0
                tick_count = 0
                if DRIVE_ONLY:
                    pick_state, sequence = "drive_only", None
                    publish_mission_state(1)
                    print(f"   drive-only   적재/관측을 건너뛰고 /{MISSION_TOPIC} 1 발행 — 주행만 시험한다")
                else:
                    pick_state = "scan"
                    sequence = PickPlaceSequence(robot, ik_solver, arm_indices, build_scan_steps())
                    sequence.reset()
                    publish_mission_state(0)
            was_playing = is_playing

            if not is_playing or pick_state is None:
                continue

            # 카터가 모바일 베이스라 이론상 움직일 수 있다 — 매 프레임 동기화한다
            base_pos, base_quat = sync_base_pose(lula, robot)

            if pick_state == "drive_only":
                if nav_done():
                    publish_mission_state(0)   # 주행 프로세스를 다시 띄워도 바로 출발하지 않게
                    print("   drive-only   주행 완료(/nav_done 1) — 하역은 건너뛴다. "
                          "다시 시험하려면 Stop 후 Play")
                    pick_state = None
                continue

            if pick_state == "wait_unload":
                # 주행 프로세스가 도킹을 끝내면 자동으로, 안 되면 UNLOAD_KEY 로 수동 진행한다
                if UNLOAD_KEY in keys.take() or nav_done():
                    publish_mission_state(0)      # 주행 프로세스를 다시 띄워도 바로 출발하지 않게
                    unload_slot = len(RACK_SLOTS) - 1
                    measured_z = float(np.mean([g[2] for g in grasp_log]))
                    desk_z = measured_z if DESK_Z_M is None else DESK_Z_M
                    print(f"   unload       적재 파지점 z 평균 {measured_z:.4f} -> 내려놓는 z {desk_z:.4f}"
                          + ("  (DESK_Z_M 이 None 이라 측정값 사용. 이 값을 DESK_Z_M 에 고정할 것)" if DESK_Z_M is None else ""))
                    print(f"   unload       랙 {unload_slot + 1}번 -> 책상 {unload_slot + 1}자리")
                    sequence = PickPlaceSequence(robot, ik_solver, arm_indices,
                                                 build_unload_steps(lula, unload_slot, desk_z, base_pos, base_quat))
                    sequence.reset()
                    pick_state = "unload"
                    tick_count = 0
                continue

            if pick_state == "observe_settle":
                wait_frames += 1
                seq_now, markers = read_markers()
                fresh = seq_now - seq_mark >= FRESH_SEQ_ADVANCE
                if wait_frames < DETECT_SETTLE_FRAMES or not (fresh or wait_frames > FRESH_TIMEOUT_FRAMES):
                    continue
                if fresh and seq_now != last_marker_seq:      # 새 메시지마다 한 표
                    last_marker_seq = seq_now
                    votes.append(assign_slots(markers))
                    if collect_start < 0:
                        collect_start = wait_frames
                if fresh and wait_frames - collect_start < OBSERVE_COLLECT_FRAMES and wait_frames <= FRESH_TIMEOUT_FRAMES:
                    continue
                if not votes:
                    print(f"   observe      마커 토픽을 {FRESH_TIMEOUT_FRAMES} 프레임 동안 못 받았다 — 미검출로 진행")
                urg = merge_urgencies(votes)
                print(f"   observe      {len(votes)}프레임 누적 (칸별 판독 횟수 "
                      f"{[sum(v[k] >= 0 for v in votes) for k in range(len(RACK_SLOTS))]}) -> 랙 {format_urgencies(urg)}")
                pick_state = "observe_home"
                sequence = PickPlaceSequence(robot, ik_solver, arm_indices, [home_step()])
                sequence.reset()
                tick_count = 0
                continue

            if pick_state == "settle":
                wait_frames += 1
                seq_now, _ = read_detections()
                if wait_frames < DETECT_SETTLE_FRAMES or seq_now - seq_mark < FRESH_SEQ_ADVANCE:
                    if wait_frames > FRESH_TIMEOUT_FRAMES:
                        begin_recover(f"새 검출을 {FRESH_TIMEOUT_FRAMES} 프레임 동안 못 받았다")
                    continue

                if settle_next == "center":
                    found = find_first_tray_optical(verbose=True)
                    if found is None:
                        retry_count += 1
                        if retry_count > MAX_DETECT_RETRIES:
                            begin_recover(f"{MAX_DETECT_RETRIES}번 재시도해도 트레이를 못 찾았다")
                        else:
                            print(f"   pick         트레이를 못 찾음 — 재시도 ({retry_count}/{MAX_DETECT_RETRIES})")
                            seq_mark, wait_frames = read_detections()[0], 0
                        continue
                    retry_count = 0
                    point_optical, _conf = found
                    steps = build_center_step(ik_solver, color_camera_path, point_optical)
                    sequence = PickPlaceSequence(robot, ik_solver, arm_indices, steps)
                    sequence.reset()
                    pick_state = "center"
                    tick_count = 0
                    continue

                # settle_next == "pick" — 중앙 정렬 후 다시 검출해서 최종 파지점을 구한다
                try:
                    # centered_first — 바로 앞 정렬 단계가 중앙에 맞춰 놓은 그 트레이를 집는다
                    found = first_tray_grasp(color_camera_path, base_pos, base_quat,
                                             verbose=True, centered_first=True)
                except Exception:
                    import traceback
                    print("   pick         계획 실패 — 아래 오류를 보고할 것")
                    traceback.print_exc()
                    found = None

                if found is None:
                    retry_count += 1
                    if retry_count > MAX_DETECT_RETRIES:
                        begin_recover(f"{MAX_DETECT_RETRIES}번 재시도해도 트레이를 못 찾았다")
                    else:
                        print(f"   pick         트레이를 못 찾음 — 재시도 ({retry_count}/{MAX_DETECT_RETRIES})")
                        seq_mark, wait_frames = read_detections()[0], 0
                    continue
                retry_count = 0

                grasp, _tray_world = found
                grasp_log.append(grasp)
                steps = build_pick_steps(lula, grasp, base_pos, base_quat, slot_index)
                sequence = PickPlaceSequence(robot, ik_solver, arm_indices, steps)
                sequence.reset()
                pick_state = "pick"
                tick_count = 0
                continue

            # scan / descend / center / pick — 다 PickPlaceSequence 를 그대로 돌린다
            solved = sequence.tick()
            if solved:
                fail_streak = 0
            else:
                fail_streak += 1
                if fail_streak == 1 or fail_streak % LOG_INTERVAL == 0:
                    carb.log_warn(f"IK 미수렴 ({fail_streak})  [{pick_state}] step {sequence.index}")

            if sequence.failed:
                print(f"   [{pick_state}] {sequence.failed}")
                if step_retries < MAX_STEP_RETRIES:
                    # 손목을 특이점에서 띄우고 실패한 스텝부터 다시 — 뒤 스텝은 절대 목표라 그대로 이어진다
                    held = sequence.gripper   # 새 시퀀스는 reset 에서 open 으로 시작한다
                    step_retries += 1
                    print(f"   복구         손목을 풀고 실패 스텝부터 재시도 ({step_retries}/{MAX_STEP_RETRIES})")
                    sequence = PickPlaceSequence(robot, ik_solver, arm_indices,
                                                 [wrist_unlock_step()] + sequence.steps[sequence.index:])
                    sequence.reset()
                    sequence.gripper = held
                    tick_count = 0
                    continue
                step_retries = 0
                # 홈으로 빠지지 않고 다음 스텝으로 넘어간다. 같은 시퀀스를 계속 쓰므로
                # 그리퍼 상태가 그대로 유지된다 — 들고 있던 트레이를 놓지 않는다
                print(f"   건너뜀       step {sequence.index} '{sequence.current['label']}' — 다음 스텝으로 넘어간다")
                sequence.skip()
                tick_count = 0
                continue

            if tick_count % LOG_INTERVAL == 0 and not sequence.done:
                print(f"   [{pick_state}] step {sequence.index} '{sequence.current['label']}'   "
                      f"{sequence.step_tick}/{sequence.n_steps}")
            tick_count += 1

            if sequence.done:
                if pick_state == "scan":
                    # joint 회전만으로는 원하는 높이가 안 나온다 — 회전 직후 실제
                    # 도착 위치를 읽어서 z 만 낮추는 스텝을 별도로 만든다
                    pick_state = "descend"
                    sequence = PickPlaceSequence(
                        robot, ik_solver, arm_indices,
                        build_descend_step(ik_solver, base_pos, base_quat, SCAN_DESCEND_Z_M, color_camera_path),
                    )
                    sequence.reset()
                elif pick_state == "descend":
                    pick_state, settle_next = "settle", "center"
                    seq_mark, wait_frames = read_detections()[0], 0
                elif pick_state == "center":
                    pick_state, settle_next = "settle", "pick"
                    seq_mark, wait_frames = read_detections()[0], 0
                elif pick_state == "recover":
                    if stage_fails >= MAX_STAGE_FAILS or slot_index >= len(RACK_SLOTS):
                        print(f"   복구         적재 실패 {stage_fails}회 — 남은 칸을 포기하고 관측으로 넘어간다")
                        pick_state = "observe_go"
                        sequence = PickPlaceSequence(robot, ik_solver, arm_indices,
                                                     build_observe_step())
                    else:
                        print(f"   복구         복구 완료 — 다음 트레이 스캔 (랙 {slot_index + 1}번부터 다시)")
                        pick_state = "scan"
                        sequence = PickPlaceSequence(robot, ik_solver, arm_indices, build_scan_steps())
                        seq_mark, wait_frames, retry_count = -1, 0, 0
                    sequence.reset()
                    tick_count = 0
                elif pick_state == "observe_go":
                    pick_state = "observe_settle"
                    seq_mark, wait_frames = read_markers()[0], 0
                    votes, last_marker_seq, collect_start = [], -1, -1
                elif pick_state == "observe_home":
                    publish_mission_state(1)
                    print(f"   observe      홈 복귀 완료 — /{MISSION_TOPIC} 1 발행. 주행 프로세스가 "
                          f"도킹을 끝내(/{NAV_DONE_TOPIC} 1) 알려오면 하역한다 "
                          f"(수동 진행: 뷰포트 클릭 후 '{UNLOAD_KEY}' 키)")
                    keys.take()      # 관측 중 눌린 키는 버린다
                    pick_state = "wait_unload"
                elif pick_state == "unload":
                    unload_slot -= 1
                    if unload_slot < 0:
                        print("   unload       하역 완료 — Play 를 다시 누르면 처음부터 재시도한다")
                        pick_state = None
                    else:
                        print(f"   unload       랙 {unload_slot + 1}번 -> 책상 {unload_slot + 1}자리")
                        sequence = PickPlaceSequence(robot, ik_solver, arm_indices,
                                                     build_unload_steps(lula, unload_slot, desk_z, base_pos, base_quat))
                        sequence.reset()
                        tick_count = 0
                else:  # pick 완료 — 다음 칸이 남았으면 홈에서 다시 스캔한다
                    slot_index += 1
                    step_retries = 0
                    if slot_index >= len(RACK_SLOTS):
                        print("   pick         랙이 다 찼다 — 랙을 위에서 관측해 긴급도를 읽는다")
                        pick_state = "observe_go"
                        sequence = PickPlaceSequence(robot, ik_solver, arm_indices, build_observe_step())
                        sequence.reset()
                        tick_count = 0
                    else:
                        print(f"   pick         랙 {slot_index}번 완료 — 다음 트레이 스캔")
                        pick_state = "scan"
                        sequence = PickPlaceSequence(robot, ik_solver, arm_indices, build_scan_steps())
                        sequence.reset()
                        seq_mark, wait_frames, retry_count = -1, 0, 0
    except KeyboardInterrupt:
        print("\n   중단됨 (Ctrl+C)")

    # 그래프를 먼저 지워 render product / writer 를 떼고 나서 앱을 닫는다
    try:
        keys.close()
        world.stop()
        omni.usd.get_context().get_stage().RemovePrim(DETECT_GRAPH_PATH)
        for _ in range(5):
            simulation_app.update()
    except Exception as exc:
        print(f"   정리 중 오류 (무시): {exc}")
    simulation_app.close()


if __name__ == "__main__":
    main()
