import math
import time
import rclpy
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

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. 출발점 설정: intergration_nova.usd 의 /World/robot_nova 배치값 그대로.
    # Isaac 월드 좌표 == 맵 좌표 이므로 변환 없이 넣는다 (yaw -90deg).
    # 씬에서 카트를 옮기면 이 값도 같이 고쳐야 AMCL 이 처음부터 제대로 붙는다.
    init_pose = create_pose(nav, -0.761, 0.551, -90.0)
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()

    # 2. 순차 방문할 좌표를 (x[m], y[m], yaw[deg]) 형식으로 입력한다.
    # 예: (2.0, 1.0, 90.0)
    # 맵 제작 시 free space 기준점이 책상과 겹쳐 씬 전체를 x축으로 -1.84091 옮겼으므로
    # 좌표를 그만큼 옮겼다.
    #
    # 책상 옆(x = -0.761)은 로봇 옆면과 책상 사이가 1cm 뿐이라 '직진만' 되는 구간이다.
    # 그래서 출발 직후엔 남쪽으로 빠져나오고, 도착할 때도 위(y=19.5)에서 -90deg 로
    # 방향을 맞춘 뒤 남쪽으로 진입한다. 도착 지점에서 제자리 회전을 시키면 책상에 부딪힌다.
    waypoint_specs = [
        (-0.761, -1.9,  -90),    # 책상에서 직진으로 빠져나온다 (회전 가능 구간 y=-1.8~-2.0)
        ( 5.859, -1.0,    0),    # 복도 (여기서부터 회전 가능)
        ( 5.859, 19.5,   90),    # 북쪽 끝
        (-0.761, 19.5,  180),    # 내려놓는 자세 바로 위
        (-0.761, 15.5,  -90),    # 남쪽으로 진입 = 팔이 트레이를 내려놓는 자세
    ]

    if not waypoint_specs:
        nav.get_logger().error(
            'waypoint_specs가 비어 있습니다. (x, y, yaw_deg) 좌표를 입력하세요.'
        )
        rclpy.shutdown()
        return

    waypoints = [
        create_pose(nav, x, y, yaw_deg)
        for x, y, yaw_deg in waypoint_specs
    ]

    # 3. 입력된 waypoint를 순서대로 방문한다.
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            current_index = feedback.current_waypoint
            print(
                f"현재 waypoint: {current_index + 1}/{len(waypoints)}"
            )

        time.sleep(0.5)

    # 4. 결과 처리
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('\n🎉 모든 waypoint 도착 완료!')
    elif result == TaskResult.CANCELED:
        print('주행 취소됨')
    elif result == TaskResult.FAILED:
        print('주행 실패')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
