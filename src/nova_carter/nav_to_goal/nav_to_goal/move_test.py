import math
import os
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# ==========================================================
# 🧮 [연산 함수 1] 오일러 각도(Rad) -> 쿼터니언 변환 함수
# ==========================================================
def get_quaternion_from_euler(roll, pitch, yaw):
    """math 모듈만 사용하여 오일러 각도를 쿼터니언으로 변환"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# ==========================================================
# 🧮 [연산 함수 2] 쿼터니언 추출 -> 오일러 각도(Rad) 변환 함수
# ==========================================================
def get_euler_from_quaternion(x, y, z, w):
    """수신된 쿼터니언 값을 추출하여 오일러 각도(라디안)로 연산"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

# ==========================================================
# 🖨️ [출력 함수] 최종 도착 위치 및 방향 출력기
# ==========================================================
def print_final_pose(pose_msg):
    """PoseStamped 메시지를 받아 위치와 오일러 각도를 예쁘게 출력"""
    if not pose_msg:
        return

    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    
    # 쿼터니언 -> 오일러 각도(라디안) 추출
    roll_rad, pitch_rad, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    
    # 라디안 -> 디그리(도) 변환
    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)
    yaw_deg = math.degrees(yaw_rad)
    
    # 결과 출력
    print("-" * 50)
    print(f"📍 최종 위치: X = {pos.x:.3f} m, Y = {pos.y:.3f} m")
    print(f"🧭 최종 방향 (Radian): Roll = {roll_rad:.3f}, Pitch = {pitch_rad:.3f}, Yaw = {yaw_rad:.3f}")
    print(f"🧭 최종 방향 (Degree): {yaw_deg:.1f}°")
    print("-" * 50)


# ==========================================================
# 🚀 메인 주행 로직
# ==========================================================
def create_pose(navigator, x, y, yaw_deg):
    """x, y, yaw(도 단위) → PoseStamped 생성"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)

    yaw_rad = math.radians(yaw_deg)
    q = get_quaternion_from_euler(0, 0, yaw_rad)
    
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

PATH_STEP = 0.05   # 경로 포즈 간격 [m]


def sample_route(route):
    """route 의 line/arc 조각을 PATH_STEP 간격의 (x, y, yaw_rad) 로 펼친다.

    ('line', (x0, y0), (x1, y1))              : 직선
    ('arc',  (cx, cy), r, a0_deg, a1_deg)     : 중심 (cx, cy), 반지름 r, 각도 a0 -> a1 (증가 = 반시계 = 좌회전)
    """
    pts = []
    for seg in route:
        if seg[0] == 'line':
            (x0, y0), (x1, y1) = seg[1], seg[2]
            length = math.hypot(x1 - x0, y1 - y0)
            yaw = math.atan2(y1 - y0, x1 - x0)
            n = max(1, int(length / PATH_STEP))
            for i in range(n + 1):
                t = i / n
                pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw))
        else:
            (cx, cy), r, a0, a1 = seg[1], seg[2], math.radians(seg[3]), math.radians(seg[4])
            ccw = a1 > a0
            n = max(1, int(abs(a1 - a0) * r / PATH_STEP))
            for i in range(n + 1):
                a = a0 + (a1 - a0) * i / n
                yaw = a + (math.pi / 2 if ccw else -math.pi / 2)
                pts.append((cx + r * math.cos(a), cy + r * math.sin(a), yaw))
    return pts


def path_length(route):
    total = 0.0
    for seg in route:
        if seg[0] == 'line':
            total += math.hypot(seg[2][0] - seg[1][0], seg[2][1] - seg[1][1])
        else:
            total += abs(math.radians(seg[4]) - math.radians(seg[3])) * seg[2]
    return total


def build_path(nav, route, final_yaw_deg):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = nav.get_clock().now().to_msg()
    pts = sample_route(route)
    for x, y, yaw in pts:
        path.poses.append(create_pose(nav, x, y, math.degrees(yaw)))
    path.poses[-1] = create_pose(nav, pts[-1][0], pts[-1][1], final_yaw_deg)
    return path


def wait_until_nav2_active(nav):
    """controller_server 활성화 + map->base_link TF(ground_truth_localization + Isaac odom)까지 기다린다.

    nav.waitUntilNav2Active() 는 AMCL 전제라 쓰지 않는다 (amcl 노드를 기다리고, 기본값 (0,0,0)을
    /initialpose 로 쏜다).
    """
    from tf2_ros import Buffer, TransformListener
    nav._waitForNodeToActivate('controller_server')
    tf_buffer = Buffer()
    nav._tf_listener = TransformListener(tf_buffer, nav)
    nav.get_logger().info('map->base_link TF 대기 중...')
    while rclpy.ok() and not tf_buffer.can_transform('map', 'base_link', Time()):
        rclpy.spin_once(nav, timeout_sec=0.5)
    nav.get_logger().info('Nav2 준비 완료')
    return tf_buffer


def remaining_path(path, tf_buffer, start_index):
    """로봇에서 가장 가까운 포즈(start_index 이후만 탐색)부터 끝까지 잘라낸 Path 와 그 인덱스."""
    try:
        t = tf_buffer.lookup_transform('map', 'base_link', Time()).transform.translation
    except Exception:
        return None, start_index
    best_i, best_d = start_index, float('inf')
    for i in range(start_index, len(path.poses)):
        p = path.poses[i].pose.position
        d = (p.x - t.x) ** 2 + (p.y - t.y) ** 2
        if d < best_d:
            best_i, best_d = i, d
    rest = Path()
    rest.header = path.header
    rest.poses = path.poses[best_i:]
    return rest, best_i


def main():
    rclpy.init()
    nav = BasicNavigator()
    # launch 쪽 Nav2 가 Isaac Sim 클록(use_sim_time)으로 돌므로 goal stamp 도 같은 클록이어야 한다.
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    # 1. 출발점/현재 위치는 여기서 정하지 않는다. Isaac Sim 이 기록한 start_pose.json 으로
    # launch 의 ground_truth_localization 이 map->odom 을 잡으므로, 씬에서 카트를 옮겨도 손댈 곳이 없다.
    tf_buffer = wait_until_nav2_active(nav)

    # 2. 주행은 세 단계다 (실제 AMR 의 undock -> navigate -> dock 과 같은 구조):
    #   [A] 책상 옆 탈출  : FollowPath 고정 직선. 꼬리(1.38 m)가 책상을 벗어날 때까지는 조향 없이 직진해야 해서
    #                       planner 에 맡기지 않는다.
    #   [B] 자율주행      : NavigateToPose 로 도킹 라인 앞 staging 지점까지. planner(Smac Hybrid-A*, 회전 반경 0.6)
    #                       가 1 Hz 로 재계획하므로 사람이 막으면 우회하고, 차선 마스크 덕에 평소엔 ㄷ자 동선 가운데로 간다.
    #   [C] 도킹          : FollowPath 고정 직선 4.3 m. 도착 시 회전이 없어야 12 cm 옆 간격에서 책상을 안 친다.
    #
    # 씬 기준값 (intergration_nova.usd / intergration_nova.png):
    #   시작 base_link = (1.233, 3.363, 180deg), 아래 책상 x[-0.02, 2.92] y >= 3.986 -> 옆면~책상 12cm
    #   위 책상은 아래 책상보다 정확히 +15.8m -> 시작과 같은 간격/방향의 도킹 pose = (1.233, 19.163, 180deg)
    #   아래 방 x[-3.9, 4.0] y[-2.8, 5.2] / 복도 x[4.0, 10.0] (중앙 7.0) / 위 방 y[12.1, 21.1]
    # footprint 앞 0.48 / 뒤 1.38 / 폭 1.0.
    # 서쪽 포켓(책상 끝 -0.02 ~ 벽 -3.875)은 꼬리 1.38 m 때문에 U턴 여유가 거의 없어 planner 에 맡기지 않고
    # 계산으로 검증한 R=0.6 고정 호로 돈다 (footprint 모서리 기준 벽 0.51 m, 책상 0.12 m 이상 유지).
    undock_route = [
        ('line', (1.233, 3.363), (-1.6, 3.363)),   # 서쪽 직진. 끝날 때 꼬리 x=-0.22 (책상 끝 -0.02 밖)
        ('arc',  (-1.6, 2.763), 0.6, 90.0, 180.0), # 좌회전 -> 남향
        ('line', (-2.2, 2.763), (-2.2, 1.2)),
        ('arc',  (-1.6, 1.2), 0.6, 180.0, 270.0),  # 좌회전 -> 동향, 아래 방 가운데(y=0.6) 에서 자율주행 시작
    ]
    staging_pose = (5.5, 19.163, 180.0)            # 도킹 라인 위, 책상 구간(x<=3.4) 에서 2.1 m 앞
    dock_route = [
        ('line', (5.5, 19.163), (1.233, 19.163)),  # 직진 진입 -> 시작과 같은 책상 간격/방향
    ]
    dock_yaw_deg = 180.0

    bt_xml = os.path.join(get_package_share_directory('carter_navigation'),
                          'behavior_trees', 'navigate_to_pose_carter.xml')   # Spin/BackUp 복구 제거판

    # RViz 의 Global Planner > Path 디스플레이(/plan). 고정 구간은 우리가, 자율 구간은 planner 가 발행한다.
    plan_pub = nav.create_publisher(Path, 'plan', QoSProfile(
        depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def run_follow_path(name, route, final_yaw_deg):
        path = build_path(nav, route, final_yaw_deg)
        print(f"[{name}] FollowPath {path_length(route):.1f} m -> "
              f"({path.poses[-1].pose.position.x:.3f}, {path.poses[-1].pose.position.y:.3f}, {final_yaw_deg:.0f}deg)")
        plan_pub.publish(path)
        nav.followPath(path)
        passed_index = 0
        while not nav.isTaskComplete():
            rest, passed_index = remaining_path(path, tf_buffer, passed_index)
            if rest is not None:
                plan_pub.publish(rest)
            time.sleep(0.5)
        return nav.getResult()

    def run_navigate(name, pose):
        print(f"[{name}] NavigateToPose -> {pose}")
        nav.goToPose(create_pose(nav, *pose), behavior_tree=bt_xml)
        last_print = 0.0
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            now = time.time()
            if feedback and now - last_print > 3.0:
                last_print = now
                print(f"-> 남은 거리 {feedback.distance_remaining:.1f} m, 복구 {feedback.number_of_recoveries}회")
            time.sleep(0.5)
        return nav.getResult()

    stages = [
        ('A 탈출',  lambda: run_follow_path('A 탈출', undock_route, 0.0)),
        ('B 자율주행', lambda: run_navigate('B 자율주행', staging_pose)),
        ('C 도킹',  lambda: run_follow_path('C 도킹', dock_route, dock_yaw_deg)),
    ]
    result = TaskResult.UNKNOWN
    for name, run in stages:
        result = run()
        if result != TaskResult.SUCCEEDED:
            print(f"[{name}] 실패: {result.name}")
            break
        print(f"[{name}] 완료")

    # 4. 결과 처리
    if result == TaskResult.SUCCEEDED:
        print('\n🎉 도킹 지점 도착 완료!')
    elif result == TaskResult.CANCELED:
        print('주행 취소됨')
    elif result == TaskResult.FAILED:
        print('주행 실패')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
