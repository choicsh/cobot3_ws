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
from nav2_msgs.msg import Costmap
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud


PATH_STEP = 0.05
MPPI_MAX_ATTEMPTS = 3
MPPI_RETRY_WAIT_SECONDS = 2.0

# 정적 장애물(콘 등) 우회: MPPI 구간에서 FollowPath 가 실패하면 planner 를 한 번만 불러
# "현재 위치 -> 이 구간의 끝점" 우회 경로를 받아 같은 FollowPath 로 실행한다.
# NavigateToPose 를 정상 주행 방식으로 쓰는 것이 아니라, 실패했을 때의 복구 수단이다.
# 우회 경로 실행이 끝나면 원래 레퍼런스의 남은 구간으로 복귀한다.
DETOUR_ENABLED = True
DETOUR_PLANNER_ID = "GridBased"
# 정체 감지: MPPI 는 앞이 막히면 "정지" 를 유효한 제어로 계속 반환하므로 FollowPath 가 실패하지 않는다.
# progress checker(45 s) 를 기다리지 않고, 레퍼런스 진행이 멈춘 채 STUCK_SECONDS 가 지나면 바로 우회한다.
STUCK_SECONDS = 6.0
STUCK_PROGRESS_M = 0.15

# 정지 전에 정적 차단을 판별하는 보강. local costmap의 남은 레퍼런스를
# 미리 검사하되 움직이는 track의 예측점과 겹치면 사람으로 보고 planner를
# 호출하지 않는다. 6초 정체 판정은 센서/분류 실패 시의 백업으로 유지한다.
STATIC_BLOCK_LOOKAHEAD_M = 5.0
STATIC_BLOCK_MIN_AHEAD_M = 0.8
STATIC_BLOCK_PERSISTENCE_S = 1.5
STATIC_BLOCK_POSITION_TOLERANCE_M = 0.5
STATIC_BLOCK_COST = 253
MOVING_PREDICTION_CLEARANCE_M = 0.8
PREDICTION_MAX_AGE_S = 0.8

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


def _yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _transform_xy(x, y, transform):
    translation = transform.transform.translation
    yaw = _yaw_from_quaternion(transform.transform.rotation)
    c, s = math.cos(yaw), math.sin(yaw)
    return (
        translation.x + c * x - s * y,
        translation.y + s * x + c * y,
    )


def _costmap_cell(costmap, x, y):
    """Costmap frame의 (x, y)에 해당하는 uint8 cost를 반환한다."""
    origin = costmap.metadata.origin
    yaw = _yaw_from_quaternion(origin.orientation)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - origin.position.x, y - origin.position.y
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    mx = math.floor(local_x / costmap.metadata.resolution)
    my = math.floor(local_y / costmap.metadata.resolution)
    if not (0 <= mx < costmap.metadata.size_x
            and 0 <= my < costmap.metadata.size_y):
        return None
    return costmap.data[my * costmap.metadata.size_x + mx]


def first_blocked_path_point(
    costmap,
    path,
    start_index,
    path_to_costmap,
    min_ahead=STATIC_BLOCK_MIN_AHEAD_M,
    lookahead=STATIC_BLOCK_LOOKAHEAD_M,
):
    """남은 레퍼런스 앞쪽의 첫 lethal/inscribed 지점을 찾는다.

    반환값은 (path frame x/y, costmap frame x/y, 진행거리, cost)다.
    inflation의 낮은 비용은 정적 차단으로 보지 않는다.
    """
    if not path.poses or start_index >= len(path.poses):
        return None
    distance = 0.0
    previous = path.poses[start_index].pose.position
    for pose in path.poses[start_index:]:
        point = pose.pose.position
        distance += math.hypot(point.x - previous.x, point.y - previous.y)
        previous = point
        if distance < min_ahead:
            continue
        if distance > lookahead:
            break
        cost_x, cost_y = _transform_xy(point.x, point.y, path_to_costmap)
        cost = _costmap_cell(costmap, cost_x, cost_y)
        if (cost is not None and cost != 255
                and cost >= STATIC_BLOCK_COST):
            return point.x, point.y, cost_x, cost_y, distance, cost
    return None


class AheadBlockageMonitor:
    """정적 전방 차단과 움직이는 예측 track을 구분한다."""

    def __init__(self, navigator, tf_buffer):
        self.navigator = navigator
        self.tf_buffer = tf_buffer
        self.costmap = None
        self.prediction = None
        self.subscriptions = [
            navigator.create_subscription(
                Costmap,
                "/local_costmap/costmap_raw",
                self._costmap_callback,
                qos_profile_sensor_data,
            ),
            navigator.create_subscription(
                PointCloud,
                "/hospital/predicted_obstacles",
                self._prediction_callback,
                qos_profile_sensor_data,
            ),
        ]

    def _costmap_callback(self, message):
        self.costmap = message

    def _prediction_callback(self, message):
        self.prediction = message

    @staticmethod
    def _stamp_seconds(header):
        return header.stamp.sec + header.stamp.nanosec / 1e9

    def observe(self, path, start_index):
        """차단이 없으면 None, 있으면 위치/거리/dynamic 분류를 반환한다."""
        if self.costmap is None:
            return None
        costmap_frame = self.costmap.header.frame_id
        try:
            path_to_costmap = self.tf_buffer.lookup_transform(
                costmap_frame, path.header.frame_id, Time()
            )
        except Exception:
            return None
        blocked = first_blocked_path_point(
            self.costmap, path, start_index, path_to_costmap
        )
        if blocked is None:
            return None
        map_x, map_y, cost_x, cost_y, distance, cost = blocked
        dynamic = False
        prediction = self.prediction
        now = self.navigator.get_clock().now().nanoseconds / 1e9
        prediction_age = (
            now - self._stamp_seconds(prediction.header)
            if prediction is not None else math.inf
        )
        if prediction is not None and 0.0 <= prediction_age <= PREDICTION_MAX_AGE_S:
            try:
                prediction_to_costmap = self.tf_buffer.lookup_transform(
                    costmap_frame, prediction.header.frame_id, Time()
                )
            except Exception:
                prediction_to_costmap = None
            radii = prediction.channels[0].values if prediction.channels else []
            if prediction_to_costmap is not None:
                for index, point in enumerate(prediction.points):
                    px, py = _transform_xy(
                        point.x, point.y, prediction_to_costmap
                    )
                    radius = radii[index] if index < len(radii) else 0.0
                    if math.hypot(px - cost_x, py - cost_y) <= (
                        MOVING_PREDICTION_CLEARANCE_M + radius
                    ):
                        dynamic = True
                        break
        return {
            "map_xy": (map_x, map_y),
            "costmap_xy": (cost_x, cost_y),
            "distance": distance,
            "cost": cost,
            "dynamic": dynamic,
        }


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
    # FollowPath 실패만으로 장애물에 의한 BLOCKED라고 단정할 수 없다.
    # 동적 장애물은 MPPI가 정지/회피/경로 복귀로 처리하며, 재시도 뒤에도
    # 실패한 경우 원인 미확정 FAILED를 반환한다. BLOCKED는 추후 costmap에서
    # 선택 차선의 완전 차단을 명시적으로 판정할 때만 사용한다.
    if result == TaskResult.FAILED:
        return MissionStatus.FAILED
    return MissionStatus.FAILED


def request_detour(navigator, tf_buffer, goal_pose):
    """planner 에게 현재 위치 -> goal_pose 우회 경로를 한 번 요청한다.

    성공하면 nav_msgs/Path, 실패하면 None. 전역 재계획을 상시 도는 것이 아니라
    FollowPath 가 실패한 시점에만 호출한다.
    """
    try:
        translation = tf_buffer.lookup_transform(
            "map", "base_link", Time()
        ).transform.translation
    except Exception:
        return None
    try:
        rotation = tf_buffer.lookup_transform(
            "map", "base_link", Time()
        ).transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        start = create_pose(navigator, translation.x, translation.y, yaw)
        detour = navigator.getPath(
            start, goal_pose, planner_id=DETOUR_PLANNER_ID, use_start=True
        )
    except Exception as error:
        print(f"  [DETOUR] planner 호출 실패: {error}")
        return None
    if detour is None or len(detour.poses) < 2:
        return None
    return detour


def path_is_blocked(path, start_index, costmap_checker):
    """향후 BLOCKED 판정용 자리. 지금은 판정하지 않는다."""
    return False


def run_path_stage(
    navigator,
    tf_buffer,
    plan_publisher,
    stage_name,
    route,
    controller_id,
    goal_checker_id,
    final_yaw=None,
    blockage_monitor=None,
):
    path = build_path(navigator, route)
    if final_yaw is not None:
        path.poses[-1].pose.orientation.z = math.sin(final_yaw / 2.0)
        path.poses[-1].pose.orientation.w = math.cos(final_yaw / 2.0)
    max_attempts = (
        MPPI_MAX_ATTEMPTS if controller_id == "FollowPathMPPI" else 1
    )

    for attempt in range(1, max_attempts + 1):
        event = (f"[AUDIT] stage={stage_name} attempt={attempt} controller={controller_id} "
                 f"start_sim_s={navigator.get_clock().now().nanoseconds / 1e9:.3f}")
        print(event, flush=True)
        navigator.get_logger().info(event)
        destination = path.poses[-1].pose.position
        print(
            f"[{stage_name}] {controller_id}/{goal_checker_id}, "
            f"{route_length(route):.1f} m -> "
            f"({destination.x:.3f}, {destination.y:.3f}), "
            f"시도 {attempt}/{max_attempts}"
        )
        plan_publisher.publish(path)
        navigator.followPath(
            path,
            controller_id=controller_id,
            goal_checker_id=goal_checker_id,
        )

        last_report = 0.0
        passed_index = 0
        latest_remaining = None
        distances = [0.0] * len(path.poses)
        for index in range(len(path.poses) - 2, -1, -1):
            a, b = path.poses[index].pose.position, path.poses[index + 1].pose.position
            distances[index] = distances[index + 1] + math.hypot(a.x - b.x, a.y - b.y)
        stuck_since = time.monotonic()
        stuck_index = 0
        static_block_since = None
        static_block_xy = None
        detour_done = attempt > 1 or controller_id != "FollowPathMPPI"
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            latest_remaining, passed_index = remaining_path(
                path, tf_buffer, passed_index
            )
            now = time.monotonic()

            # 콘처럼 같은 위치에 남아 있는 차단은 로봇이 완전히 멈추기 전에
            # planner에 넘긴다. 움직이는 사람 예측과 겹치면 MPPI에 맡긴다.
            early_detour = False
            blockage = None
            if (DETOUR_ENABLED and not detour_done
                    and blockage_monitor is not None):
                blockage = blockage_monitor.observe(path, passed_index)
                if blockage is None or blockage["dynamic"]:
                    static_block_since = None
                    static_block_xy = None
                else:
                    sim_now = navigator.get_clock().now().nanoseconds / 1e9
                    if (static_block_xy is None or math.dist(
                        static_block_xy, blockage["map_xy"]
                    ) > STATIC_BLOCK_POSITION_TOLERANCE_M):
                        static_block_xy = blockage["map_xy"]
                        static_block_since = sim_now
                    elif (static_block_since is not None and
                          sim_now - static_block_since >= STATIC_BLOCK_PERSISTENCE_S):
                        early_detour = True

            # 레퍼런스를 따라 얼마나 진행했는지로 정체를 판정한다 (제자리 회전은 진행으로 안 친다).
            if distances[stuck_index] - distances[passed_index] >= STUCK_PROGRESS_M:
                stuck_index = passed_index
                stuck_since = now
            if early_detour or (DETOUR_ENABLED and not detour_done
                                and now - stuck_since >= STUCK_SECONDS):
                detour_done = True
                if early_detour:
                    reason = (
                        f"전방 {blockage['distance']:.1f}m 정적 차단 "
                        f"{STATIC_BLOCK_PERSISTENCE_S:.1f}s 지속"
                    )
                else:
                    reason = f"{STUCK_SECONDS:.0f}s 동안 전진 없음"
                print(f"  [DETOUR] {reason} -> planner 우회 요청")
                navigator.get_logger().info(
                    f"[AUDIT] stage={stage_name} detour_trigger={reason}"
                )
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    time.sleep(0.2)
                detour = request_detour(navigator, tf_buffer, path.poses[-1])
                if detour is None:
                    print("  [DETOUR] 우회 경로 없음 -> 레퍼런스 재개")
                    navigator.followPath(path, controller_id=controller_id,
                                         goal_checker_id=goal_checker_id)
                else:
                    print(f"  [DETOUR] 우회 경로 {len(detour.poses)} 포즈로 진행")
                    navigator.get_logger().info(
                        f"[AUDIT] stage={stage_name} detour poses={len(detour.poses)}")
                    plan_publisher.publish(detour)
                    navigator.followPath(detour, controller_id=controller_id,
                                         goal_checker_id=goal_checker_id)
                stuck_since = now
                continue
            if feedback and now - last_report >= 3.0:
                distance = distances[passed_index] if latest_remaining is not None else float("nan")
                speed = getattr(feedback, "speed", float("nan"))
                print(
                    f"  레퍼런스 남은 거리={distance:.1f} m, "
                    f"속도={speed:.2f} m/s"
                )
                last_report = now

            if latest_remaining is not None:
                plan_publisher.publish(latest_remaining)
            time.sleep(0.5)

        status = task_result_to_status(navigator.getResult())
        print(f"[AUDIT] stage={stage_name} attempt={attempt} status={status.value} "
              f"end_sim_s={navigator.get_clock().now().nanoseconds / 1e9:.3f}",
              flush=True)
        if status != MissionStatus.FAILED or attempt == max_attempts:
            return status

        # 같은 레퍼런스를 다시 시도하기 전에 planner 우회를 한 번 시도한다.
        # (사람처럼 지나가는 장애물은 재시도로 풀리지만, 콘처럼 남아 있는 정적
        #  장애물은 레퍼런스를 몇 번 재시도해도 계속 막히기 때문이다.)
        if DETOUR_ENABLED and controller_id == "FollowPathMPPI" and attempt == 1:
            detour = request_detour(navigator, tf_buffer, path.poses[-1])
            if detour is not None:
                print(f"  [DETOUR] planner 우회 경로 {len(detour.poses)} 포즈로 진행")
                navigator.get_logger().info(
                    f"[AUDIT] stage={stage_name} detour poses={len(detour.poses)}"
                )
                plan_publisher.publish(detour)
                navigator.followPath(
                    detour,
                    controller_id=controller_id,
                    goal_checker_id=goal_checker_id,
                )
                while not navigator.isTaskComplete():
                    time.sleep(0.5)
                detour_status = task_result_to_status(navigator.getResult())
                print(f"[AUDIT] stage={stage_name} detour status={detour_status.value}")
                if detour_status == MissionStatus.SUCCEEDED:
                    return detour_status
            else:
                print("  [DETOUR] 우회 경로를 얻지 못했습니다. 레퍼런스를 재시도합니다")

        if latest_remaining is not None and len(latest_remaining.poses) >= 2:
            path = latest_remaining
            stamp = navigator.get_clock().now().to_msg()
            path.header.stamp = stamp
            for pose in path.poses:
                pose.header.stamp = stamp

        print(
            f"[{stage_name}] FollowPath 실패 원인은 아직 미확정입니다. 잠시 대기 후 "
            f"남은 경로를 재시도합니다 ({MPPI_RETRY_WAIT_SECONDS:.0f}s)"
        )
        time.sleep(MPPI_RETRY_WAIT_SECONDS)

    return MissionStatus.FAILED


def split_route(lane_id, route):
    """정류장 진출/중앙 이동/정류장 진입으로 나눈다.

    모든 원호는 DWB로 실행한다. MPPI의 기준 경로는 복도 직선뿐이며,
    사람을 피할 때의 국소적인 측면 이탈은 허용한다.
    """
    if lane_id == "lane_lower":
        # 방/벽 사이의 작은 호는 DWB로 고정하고, 중앙 하단 열린 구간만
        # MPPI에 맡긴다.
        return route[:6], route[6:7], route[7:]
    if lane_id == "lane_upper":
        return route[:2], route[2:3], route[3:]
    raise ValueError(f"Unknown lane: {lane_id}")


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
    blockage_monitor = AheadBlockageMonitor(navigator, tf_buffer)
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
            # 도착은 0.55 m/s / 각속도 0.6 의 전용 DWB. 0.8 로 들어오면 마지막 정렬에서 과회전한다.
            "FollowPathDock",
            "general_goal_checker",
            ARRIVAL_YAWS[route_id],
        ),
    ]

    for stage in stages:
        status = run_path_stage(
            navigator, tf_buffer, plan_publisher, *stage,
            blockage_monitor=blockage_monitor,
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
