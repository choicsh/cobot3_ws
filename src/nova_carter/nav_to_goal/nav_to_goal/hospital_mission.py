"""병원 검체실/분석실 사이의 고정 정류장 경로를 FollowPath로 실행한다.

두 방향은 서로 다른 물리 경로를 사용한다.

* specimen_to_lab: lane_upper (중앙 구조물 위쪽)
* lab_to_specimen: lane_lower (중앙 구조물 아래쪽)

양쪽 경로의 끝점은 동일한 정류장이다. 현재 단계에는 도킹 동선이나
Pick & Place가 없으며, 선택 차선이 막혀도 다른 차선으로 전환하지 않는다.
"""

from enum import Enum
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time


PATH_STEP = 0.05

# hospital_integration.usd에 사용자가 배치한 로봇의 base_link가 lab 정류장이다.
# specimen 정류장은 사용자가 지정한 (-36, 8)이며 순환 경로의 접선 자세를 쓴다.
# dock/pre-dock 구분 및 테이블 접근 경로는 이 미션에서 다루지 않는다.
LAB_STATION = (12.00, 8.00)
SPECIMEN_STATION = (-36.00, 8.00)

ARRIVAL_YAWS = {
    "specimen_to_lab": math.radians(-90.0),
    "lab_to_specimen": math.radians(90.0),
}

ROUTES = {
    "specimen_to_lab": "lane_upper",
    "lab_to_specimen": "lane_lower",
}

# line: (시작점, 끝점)
# arc: (중심, 반지름, 시작각 deg, 끝각 deg)
# 각도 증가가 반시계 방향이다. 두 lane은 서로를 뒤집어 만들지 않은 독립 경로다.
LANE_UPPER = [
    ("line", SPECIMEN_STATION, (-36.00, 10.90)),
    ("arc", (-32.00, 10.90), 4.0, 180.0, 90.0),
    ("line", (-32.00, 14.90), (8.00, 14.90)),
    ("arc", (8.00, 10.90), 4.0, 90.0, 0.0),
    ("line", (12.00, 10.90), LAB_STATION),
]

LANE_LOWER = [
    # upper의 남향 도착 자세를 이어받아 회전 없이 아래 차선으로 출발한다.
    ("line", LAB_STATION, (12.00, 7.00)),
    ("arc", (11.00, 7.00), 1.0, 0.0, -90.0),
    ("line", (11.00, 6.00), (9.00, 6.00)),
    ("arc", (9.00, 4.00), 2.0, 90.0, 180.0),
    ("line", (7.00, 4.00), (7.00, 1.50)),
    ("arc", (4.00, 1.50), 3.0, 0.0, -90.0),
    ("line", (4.00, -1.50), (-28.55, -1.50)),
    ("arc", (-28.55, 1.50), 3.0, 270.0, 180.0),
    ("line", (-31.55, 1.50), (-31.55, 4.00)),
    ("arc", (-33.55, 4.00), 2.0, 0.0, 90.0),
    ("line", (-33.55, 6.00), (-35.00, 6.00)),
    ("arc", (-35.00, 7.00), 1.0, 270.0, 180.0),
    ("line", (-36.00, 7.00), SPECIMEN_STATION),
]

LANES = {
    "lane_upper": LANE_UPPER,
    "lane_lower": LANE_LOWER,
}


class MissionStatus(Enum):
    SUCCEEDED = "SUCCEEDED"
    BLOCKED = "BLOCKED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"


def create_pose(navigator, x, y, yaw_rad):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def sample_route(route):
    """직선/원호 목록을 PATH_STEP 간격의 (x, y, yaw)로 변환한다."""
    points = []
    for segment in route:
        if segment[0] == "line":
            (x0, y0), (x1, y1) = segment[1], segment[2]
            length = math.hypot(x1 - x0, y1 - y0)
            count = max(1, math.ceil(length / PATH_STEP))
            yaw = math.atan2(y1 - y0, x1 - x0)
            for index in range(count + 1):
                ratio = index / count
                points.append(
                    (
                        x0 + (x1 - x0) * ratio,
                        y0 + (y1 - y0) * ratio,
                        yaw,
                    )
                )
        else:
            (cx, cy), radius = segment[1], segment[2]
            start = math.radians(segment[3])
            finish = math.radians(segment[4])
            count = max(
                1, math.ceil(abs(finish - start) * radius / PATH_STEP)
            )
            counter_clockwise = finish > start
            for index in range(count + 1):
                angle = start + (finish - start) * index / count
                yaw = angle + (
                    math.pi / 2.0
                    if counter_clockwise
                    else -math.pi / 2.0
                )
                points.append(
                    (
                        cx + radius * math.cos(angle),
                        cy + radius * math.sin(angle),
                        yaw,
                    )
                )
    return points


def route_length(route):
    length = 0.0
    for segment in route:
        if segment[0] == "line":
            length += math.dist(segment[1], segment[2])
        else:
            length += segment[2] * abs(
                math.radians(segment[4]) - math.radians(segment[3])
            )
    return length


def build_path(navigator, route):
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = navigator.get_clock().now().to_msg()
    path.poses = [
        create_pose(navigator, x, y, yaw)
        for x, y, yaw in sample_route(route)
    ]
    return path


def wait_until_nav2_active(navigator):
    """controller_server와 map -> base_link TF를 기다린다."""
    from tf2_ros import Buffer, TransformListener

    navigator._waitForNodeToActivate("controller_server")
    tf_buffer = Buffer()
    navigator._hospital_tf_listener = TransformListener(tf_buffer, navigator)
    navigator.get_logger().info("map -> base_link TF 대기 중")
    while rclpy.ok() and not tf_buffer.can_transform(
        "map", "base_link", Time()
    ):
        rclpy.spin_once(navigator, timeout_sec=0.5)
    navigator.get_logger().info("병원 경로 실행 준비 완료")
    return tf_buffer


def remaining_path(path, tf_buffer, start_index):
    """현재 위치에 가장 가까운 포즈부터 남은 RViz 표시 경로를 만든다."""
    try:
        translation = tf_buffer.lookup_transform(
            "map", "base_link", Time()
        ).transform.translation
    except Exception:
        return None, start_index

    if start_index >= len(path.poses):
        return None, start_index

    closest = min(
        range(start_index, len(path.poses)),
        key=lambda index: (
            path.poses[index].pose.position.x - translation.x
        )
        ** 2
        + (path.poses[index].pose.position.y - translation.y) ** 2,
    )
    remaining = Path()
    remaining.header = path.header
    remaining.poses = path.poses[closest:]
    return remaining, closest


def task_result_to_status(result):
    if result == TaskResult.SUCCEEDED:
        return MissionStatus.SUCCEEDED
    if result == TaskResult.CANCELED:
        return MissionStatus.CANCELED
    # 선택한 고정 차선에서 FollowPath가 실패하면 자동 우회하지 않고 관제에
    # BLOCKED를 반환한다. 코드 예외 등 주행 외 실패는 main에서 FAILED 처리한다.
    if result == TaskResult.FAILED:
        return MissionStatus.BLOCKED
    return MissionStatus.FAILED


def run_path_stage(
    navigator,
    tf_buffer,
    plan_publisher,
    stage_name,
    route,
    controller_id,
    goal_checker_id,
    final_yaw=None,
):
    path = build_path(navigator, route)
    if final_yaw is not None:
        path.poses[-1].pose.orientation.z = math.sin(final_yaw / 2.0)
        path.poses[-1].pose.orientation.w = math.cos(final_yaw / 2.0)
    destination = path.poses[-1].pose.position
    print(
        f"[{stage_name}] {controller_id}/{goal_checker_id}, "
        f"{route_length(route):.1f} m -> "
        f"({destination.x:.3f}, {destination.y:.3f})"
    )
    plan_publisher.publish(path)
    navigator.followPath(
        path,
        controller_id=controller_id,
        goal_checker_id=goal_checker_id,
    )

    last_report = 0.0
    passed_index = 0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        now = time.monotonic()
        if feedback and now - last_report >= 3.0:
            distance = getattr(feedback, "distance_to_goal", float("nan"))
            speed = getattr(feedback, "speed", float("nan"))
            print(f"  남은 거리={distance:.1f} m, 속도={speed:.2f} m/s")
            last_report = now

        remaining, passed_index = remaining_path(
            path, tf_buffer, passed_index
        )
        if remaining is not None:
            plan_publisher.publish(remaining)
        time.sleep(0.5)

    return task_result_to_status(navigator.getResult())


def split_route(lane_id, route):
    """정류장 진출/중앙 이동/정류장 진입으로 나눈다.

    정류장 주변과 방/벽 사이의 작은 호는 DWB로 정확히 유지하고,
    중앙의 열린 공간에는 MPPI의 제한적인 측면 회피를 허용한다.
    """
    if lane_id == "lane_lower":
        # 방/벽 사이의 작은 호는 DWB로 고정하고, 중앙 하단 열린 구간만
        # MPPI에 맡긴다.
        return route[:5], route[5:8], route[8:]
    return route[:1], route[1:-1], route[-1:]


def run_mission(navigator, route_id):
    lane_id = ROUTES[route_id]
    route = LANES[lane_id]
    departure, transit, arrival = split_route(lane_id, route)

    print(
        f"[MISSION] {route_id} -> {lane_id}, 총 {route_length(route):.1f} m"
    )
    print(
        f"[STATIONS] specimen={SPECIMEN_STATION}, lab={LAB_STATION}"
    )

    tf_buffer = wait_until_nav2_active(navigator)
    plan_publisher = navigator.create_publisher(
        Path,
        "/plan",
        QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )

    stages = [
        (
            "station_departure",
            departure,
            "FollowPath",
            "transit_goal_checker",
        ),
        (
            lane_id,
            transit,
            "FollowPathMPPI",
            "transit_goal_checker",
        ),
        (
            "station_arrival",
            arrival,
            "FollowPath",
            "general_goal_checker",
            ARRIVAL_YAWS[route_id],
        ),
    ]

    for stage in stages:
        status = run_path_stage(
            navigator, tf_buffer, plan_publisher, *stage
        )
        if status != MissionStatus.SUCCEEDED:
            print(f"[MISSION] {status.value}: {stage[0]}")
            return status

    print(f"[MISSION] {MissionStatus.SUCCEEDED.value}: station reached")
    return MissionStatus.SUCCEEDED


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters(
        [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )
    navigator.declare_parameter("route_id", "specimen_to_lab")
    route_id = navigator.get_parameter("route_id").value

    if route_id not in ROUTES:
        navigator.get_logger().error(
            f"잘못된 route_id={route_id!r}; 사용 가능: {list(ROUTES)}"
        )
        navigator.destroy_node()
        rclpy.shutdown()
        return

    try:
        run_mission(navigator, route_id)
    except KeyboardInterrupt:
        navigator.cancelTask()
        print(f"[MISSION] {MissionStatus.CANCELED.value}")
    except Exception as error:
        navigator.get_logger().error(f"mission failed: {error}")
        print(f"[MISSION] {MissionStatus.FAILED.value}")
        raise
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
