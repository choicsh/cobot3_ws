"""팔 동작 — 스텝 빌더, 손목 특이점 가드, PickPlaceSequence. pick_and_place_detection.py 에서 옮겼다.

원본과 다른 점 (로봇 여러 대 대응):
  - build_pick_steps 는 트레이 정렬 훅(TrayHooks)을 인자로 받는다 (원본은 모듈 전역 함수).
  - PickPlaceSequence 로그에 로봇 태그를 붙인다.
"""
import carb
import numpy as np
import omni.usd
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, UsdGeom

from .geometry import *  # noqa: F401,F403  원본과 같은 이름으로 쓴다


def plan_grasp(lula, tray_world, base_pos, base_quat, tray_yaw_base_deg):
    """검출점(월드) -> (파지점 base, 파지 yaw). 트레이 축에 맞춘 yaw 를 먼저, 안 풀리면 원본 방위각.

    원본(pick_grasp_yaw_weight)은 yaw = base->트레이 방위각이었다. 트레이가 가로로 흩어져 있어
    방위각이 트레이 축과 최대 19deg 어긋나고, 그만큼 비스듬히 물어 랙 칸막이에 걸렸다
    (geometry.square_grasp_yaw 참고). 파지점도 같은 yaw 방향으로 반폭만큼 민다.

    IK 는 원본과 같은 조건으로 미리 물어본다 — 풀리고 joint_5 가 손목 특이점(WRIST_NEAR_SINGULAR_DEG)
    밖이어야 채택. 둘 다 안 되면 방위각으로 간다(원본 동작, 최소한 지금보다 나빠지지 않는다)."""
    surface = world_to_base_pos(tray_world, base_pos, base_quat)
    bearing = approach_direction(surface)[1]
    square = square_grasp_yaw(bearing, tray_yaw_base_deg)
    j5 = list(lula.get_joint_names()).index("joint_5")
    for name, yaw in (("트레이 축", square), ("방위각", bearing)):
        grasp = grasp_point_base(tray_world, base_pos, base_quat, yaw)
        _, q = grasp_frame_yaw(yaw)
        pos, quat = base_pose_to_world(grasp, q, base_pos, base_quat)
        sol, ok = lula.compute_inverse_kinematics(EE_LINK_NAME, tcp_to_flange(pos, quat), quat)
        if ok and abs(np.degrees(sol[j5])) >= WRIST_NEAR_SINGULAR_DEG:
            print(f"   파지 yaw     {name} {yaw:+.1f}deg 채택 (방위각 {bearing:+.1f}, 트레이 축 {square:+.1f}, "
                  f"joint_5 {np.degrees(sol[j5]):+.1f}deg)")
            return grasp, yaw
    print(f"   파지 yaw     트레이 축 {square:+.1f} / 방위각 {bearing:+.1f} 둘 다 IK 가 안 풀렸다 — 방위각으로 간다")
    return grasp_point_base(tray_world, base_pos, base_quat, bearing), bearing


def optical_to_world(point_optical, color_camera_path):
    """관제 PC 가 준 카메라 광학 프레임 좌표를 월드로 옮긴다.

    ROS 광학 규약은 (x 우, y 하, z 전방)이고 USD 카메라는 (x 우, y 상, -z 전방)이라
    y, z 의 부호가 뒤집힌다."""
    x, y, z = point_optical
    usd_point = Gf.Vec3d(float(x), float(-y), float(-z))
    cam_prim = omni.usd.get_context().get_stage().GetPrimAtPath(color_camera_path)
    mat = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
    return np.array(mat.Transform(usd_point), dtype=float)


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


def build_return_steps(lula, grasp_base, grasp_yaw_deg, base_pos, base_quat):
    """들고 있던 트레이를 원래 집은 자리에 되돌려 놓고 홈으로. 적재 실패 복구용.

    그 자리는 방금 집어 온 곳이라 비어 있다 — 아무 데나 떨어뜨리는 것보다 안전하다."""
    direction, grasp_quat = grasp_frame_yaw(grasp_yaw_deg)

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


def build_pick_steps(lula, grasp_base, grasp_yaw_deg, base_pos, base_quat, slot, hooks):
    """검출로 구한 grasp_base(파지점, base 기준)로 집어서 RACK_SLOTS[slot] 에 놓는다.

    build_sequence 와 같은 형태지만 POINT1/POINT2(파지 지점)만 검출값으로
    갈아끼운다. place 쪽(POINT4/POINT5)은 고정값 그대로지만, 그리로 도는 joint_1
    회전량은 고정값이 아니라 파지 방향 -> place 방향의 실제 차이로 계산한다 —
    파지 위치가 검출값이라 매번 달라지므로 고정 각도로는 정확히 못 돌아온다.
    "pick 쪽만" 검출을 쓰라는 요청에 맞춘 것이다."""
    def to_world(tcp, rpy):
        return base_to_world(tcp, rpy, base_pos, base_quat)

    direction, grasp_quat = grasp_frame_yaw(grasp_yaw_deg)   # plan_grasp 가 고른 yaw

    def to_world_q(tcp):
        return base_pose_to_world(tcp, grasp_quat, base_pos, base_quat)

    approach = grasp_base - direction * APPROACH_BACKOFF_M
    lift = np.array([grasp_base[0], grasp_base[1], LIFT_Z_M])
    place = RACK_SLOTS[slot]
    p4 = to_world(place, POINT4_RPY)
    # 슬롯 바로 위 — x, y 는 놓는 위치와 같고 z 만 높다. 여기서 수직으로 내려간다
    p4_above = to_world(place + np.array([0.0, 0.0, RACK_ABOVE_Z_M]), POINT4_RPY)
    p4_waypoint = to_world(place + np.array([0.0, 0.0, RACK_ABOVE_Z_M + CARRY_WAYPOINT_Z_M]), POINT4_RPY)
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
        # joint_1 회전 뒤 자세가 놓는 자세와 최대 30deg 어긋난다(파지 yaw 가 트레이 축). 원본처럼 IK 로 보간하며
        # 돌리면 손목이 특이점 근처(joint_5 -12 ~ -30)라 joint_4 가 한 틱에 20deg 넘게 튀어 물린 트레이가 흔들렸다
        # (2026-09-25 p4b 3번 칸 걸림). 경유점까지는 관절 보간(IK 1회), 거기서 같은 자세로 수직 하강한다.
        # 경유점 +7cm: 관절 보간은 도중에 최대 6cm 처져서, 경유점이 없으면 칸막이에 닿는 경우가 있었다
        # (오프라인 IK 재생 33경로: 틱당 최악 11.6 -> 1.9deg, 실린 트레이·칸막이 충돌 후보 0).
        {"type": "joint_ik", "label": f"놓기 위 경유점(+{(RACK_ABOVE_Z_M + CARRY_WAYPOINT_Z_M) * 100:.0f}cm, 관절)",
         "target": p4_waypoint, "gripper": None, "speed": "carry", "ease": 2},
        {"type": "pose",  "label": f"놓기 위 안전 위치(+{RACK_ABOVE_Z_M * 100:.0f}cm, 수직)",
         "target": p4_above, "gripper": None, "speed": "carry", "ease": 2},
        {"type": "hold",  "label": "하강 전 대기(트레이 안정)", "gripper": None, "steps": PLACE_WAIT_MAX_STEPS,
         "min_steps": PLACE_WAIT_STEPS, "until": hooks.settled},
        {"type": "pose",  "label": f"랙 {slot + 1}번 놓기(수직 하강)", "target": p4, "gripper": None, "tol": STEP_CONTACT_TOL_M, "speed": "slow"},
        # 랙 정렬 2단계. 여기 진입 시점은 손가락이 **아직 닫혀 있고** 팔이 슬롯에 멈춰 있는
        # 순간이라, 트레이가 그리퍼에 물린 상대 pose 를 재기에 맞다
        {"type": "hold",  "label": "그리퍼 열기",        "gripper": "open",
         "on_enter": lambda seq: hooks.capture(seq, slot)},
        # 앞 스텝이 GRIPPER_WAIT_STEPS(2초)를 다 쓴 뒤라 손가락은 이미 벌어져 있다 — 물린 채로
        # 돌리는 일이 없다. 하역(책상)에는 안 건다: 랙에만 정렬 가이드가 있다는 설정이다
        {"type": "pose",  "label": "후퇴 안전 위치",     "target": p5, "gripper": None, "speed": "slow",
         "on_enter": lambda seq: hooks.align(slot, base_pos, base_quat)},
        {"type": "joint", "label": "홈 복귀",            "target": np.array(READY_JOINTS_RAD, dtype=float), "gripper": None},
    ]


def build_unload_steps(lula, slot, desk_z, base_pos, base_quat, tray_yaw_base_deg, hooks):
    """RACK_SLOTS[slot] 의 트레이를 꺼내 DESK_SLOTS[slot] 에 놓는다.

    적재의 역순이 아니라 하역 전용 시퀀스다 — 넣을 때와 뺄 때 걸리는 조건이 달라서
    들어올리는 높이를 UNLOAD_ABOVE_Z_M 으로 따로 둔다.
    랙 쪽은 적재와 같은 고정 자세(POINT4/5_RPY), 책상 쪽은 세 자리 모두
    DESK_ROW_CENTER 기준 자세 하나로 놓아 트레이가 서로 평행하게 정렬된다."""
    def to_world(tcp, rpy):
        return base_to_world(tcp, rpy, base_pos, base_quat)

    # 원본은 DESK_ROW_CENTER 방위각(-86.6deg)으로 놓아 책상 위 트레이가 3.4deg 비스듬했다. 트레이 축에 맞춘다
    desk_yaw = square_grasp_yaw(approach_direction(DESK_ROW_CENTER)[1], tray_yaw_base_deg)
    direction, desk_quat = grasp_frame_yaw(desk_yaw)

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
        # 적재와 같은 이유로 관절 보간 (랙 yaw -> 책상 yaw 재정렬에서 손목이 튄다). 책상 위 높이(TCP LIFT_Z_M)는
        # 이미 책상 트레이 윗면보다 충분히 높아 경유점이 필요 없다
        {"type": "joint_ik", "label": "책상 위(관절)",          "target": to_world_q(lift), "gripper": None, "speed": "carry", "ease": 2},
        {"type": "hold",  "label": "하강 전 대기(트레이 안정)", "gripper": None, "steps": PLACE_WAIT_MAX_STEPS,
         "min_steps": PLACE_WAIT_STEPS, "until": hooks.settled},
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

    def __init__(self, robot, ik_solver, arm_indices, steps, tag=""):
        self.tag = tag
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

        self.mode = step["type"]
        if self.mode == "joint_ik":
            # 목표 자세의 IK 를 지금 관절에서 이어 한 번만 풀고 관절 보간한다. 안 풀리면 IK 보간(pose)으로
            pos, quat = step["target"]
            # 가드(SingularityGuardedIK)를 거치지 않는다 — 목표까지의 전체 관절 변화를 '한 틱 튐' 으로 경고하게 된다
            solver = getattr(self._ik, "_ik", self._ik)
            action, ok = solver.compute_inverse_kinematics(target_position=tcp_to_flange(pos, quat),
                                                           target_orientation=quat)
            if ok and action.joint_positions is not None:
                indices = action.joint_indices if action.joint_indices is not None else self._arm_indices
                by_index = dict(zip(list(indices), list(action.joint_positions)))
                q = self._robot.get_joint_positions()
                self.start_joints = np.array([q[i] for i in self._arm_indices])
                self.target_joints = np.array([by_index.get(i, q[i]) for i in self._arm_indices])
                self.n_steps = steps_for_joint(self.start_joints, self.target_joints, speed=step.get("speed"))
                self.mode = "joint"
            else:
                print(f"{self.tag}   [{self.index}] {step['label']} — IK 안 풀림, IK 보간으로 간다")
                self.mode = "pose"
        if self.mode == "joint" and self.start_joints is not None:
            pass
        elif self.mode in ("pose", "hold"):
            self.start_pos, self.start_quat = self._current_flange_pose()
            if self.mode == "hold":
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

        print(f"{self.tag}   [{self.index}] {step['label']:16s} {self.n_steps:4d} steps   gripper {self.gripper}")

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

        if self.mode in ("pose", "hold"):
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
        if self.mode == "hold" and step.get("until") is not None:
            # 최소 min_steps 는 기다리고, 그 뒤 조건이 서면 일찍 끝낸다 (최대 n_steps)
            if self.step_tick >= step.get("min_steps", self.n_steps) and self.step_tick < self.n_steps:
                if step["until"](self, final=False):
                    self.step_tick = self.n_steps
            elif self.step_tick == self.n_steps:
                step["until"](self, final=True)
        if self.step_tick >= self.n_steps:
            if self.mode == "pose":
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
                print(f"{self.tag}   [DONE] 시퀀스 완료")

        return solved

    def _pose_error(self):
        """현재 TCP 와 목표의 위치(m)/자세(deg) 오차"""
        pos, quat = self._current_flange_pose()
        dot = float(np.clip(abs(np.dot(quat, self.target_quat)), -1.0, 1.0))
        return float(np.linalg.norm(pos - self.target_pos)), float(np.degrees(2.0 * np.arccos(dot)))

