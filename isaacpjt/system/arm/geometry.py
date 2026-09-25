"""팔 작업의 순수 계산 (numpy 만). pick_and_place_detection.py 에서 그대로 옮겼다.

Isaac 없이 시스템 python3 로 테스트한다: python3 -m pytest isaacpjt/system/tests
"""
import numpy as np

from .config import *  # noqa: F401,F403  원본과 같은 이름으로 상수를 쓴다


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
    return grasp_frame_yaw(yaw_deg * weight)


def grasp_frame_yaw(yaw_deg):
    """base z 축으로 yaw_deg 돌린 파지 자세와 접근 방향. yaw 는 approach_direction 과 같은 정의
    (Rz(yaw)*[0,1,0] = 접근 방향, 0 = base +y, -90 = base +x)."""
    rz = quat_from_axis([0, 0, 1], yaw_deg)
    q = quat_mul(rz, make_target_quat(*POINT2_RPY))
    direction = quat_to_matrix(rz) @ np.array([0.0, 1.0, 0.0])
    return direction, q / np.linalg.norm(q)



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


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def grasp_point_base(tray_world, base_pos, base_quat, yaw_deg=None):
    """검출점(월드) -> 파지 목표(base 기준).

    접근 방향으로 트레이 중심 보정(TRAY_HALF_DEPTH_M)과 높이 보정을 준다. yaw_deg 를 주면 그 파지
    방향으로 민다(square_grasp_yaw). 없으면 원본처럼 base->트레이 방위각 방향으로 민다."""
    surface = world_to_base_pos(tray_world, base_pos, base_quat)
    if yaw_deg is None:
        direction, _ = approach_direction(surface)
    else:
        direction, _ = grasp_frame_yaw(yaw_deg)
    return surface + direction * TRAY_HALF_DEPTH_M + np.array([0.0, 0.0, GRASP_ABOVE_CENTER_M])


def square_grasp_yaw(bearing_deg, tray_yaw_base_deg):
    """방위각을 트레이 축(tray_yaw_base_deg + k*90)에 스냅한 파지 yaw.

    원본은 파지 yaw = base->트레이 방위각이었다. 트레이는 책상에 반듯하게 놓여 있는데(base 기준
    yaw 가 90 의 배수) 가로 한 줄로 +-25cm 흩어져 있어 방위각은 -71 ~ -103deg 로 퍼진다. 그 차이
    (최대 19deg)만큼 비스듬히 물고, 그대로 랙 칸에 넣는다 — 칸 폭 15.4cm 에 트레이 9.1x14cm 가
    19deg 돌면 폭 13.2cm 가 되어 여유가 1cm 남고, 파지점 오차가 그걸 넘으면 칸막이 위에 얹혀
    40deg 기운다 (2026-09-25 실측, docs/SYSTEM_INTEGRATION_PLAN.md P2). 트레이 축에 맞춰 물면
    어긋남이 0 이라 여유가 3cm 로 유지된다.

    yaw 정의는 approach_direction 과 같다. 트레이 로컬 +X 의 방위(tray_yaw_deg)가 a 이면 트레이 변에
    수직인 접근 방향들의 yaw 도 a + k*90 이다 (Rz(t)*[0,1,0] 의 방위 = t + 90)."""
    k = round((bearing_deg - tray_yaw_base_deg) / 90.0)
    return (tray_yaw_base_deg + 90.0 * k + 180.0) % 360.0 - 180.0


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



def aruco_to_urgency(ids):
    """칸별 ArUco id(0 하 / 1 중 / 2 상 / -1 미검출) -> 긴급도 1~3 (3 이 가장 높다, 미검출은 1).

    DB tray.priority 에 그대로 넣는 값이다 (docs/SYSTEM_INTEGRATION_PLAN.md D2)."""
    return [i + 1 if i >= 0 else 1 for i in ids]
