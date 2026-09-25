"""병원 검체실/분석실 사이의 고정 정류장 경로를 FollowPath로 실행한다.

두 방향은 서로 다른 물리 경로를 사용한다.

* specimen_to_lab: lane_upper (중앙 구조물 위쪽)
* lab_to_specimen: lane_lower (중앙 구조물 아래쪽)

양쪽 방의 Desks 테이블 긴 변을 로봇 우측에 두고 15cm 옆면 간격으로 도킹한다.
Pick & Place는 포함하지 않으며 선택 차선이 막혀도 다른 차선으로 전환하지 않는다.
"""

from enum import Enum
import math
import os
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
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud


PATH_STEP = 0.05
# hospital_stage_runner에서 선택 lane 안의 전방 합류를 검증한 뒤만 사용.
DETOUR_PLANNER_ID = "GridBased"

# 정지 전에 정적 차단을 판별하는 보강. local costmap의 남은 레퍼런스를
# 미리 검사하되 움직이는 track의 예측점과 겹치면 사람으로 보고 planner를
# 호출하지 않는다. 단순 6초 정체만으로 planner를 호출하지 않는다.
STATIC_BLOCK_LOOKAHEAD_M = 5.0
STATIC_BLOCK_MIN_AHEAD_M = 0.8
STATIC_BLOCK_PERSISTENCE_S = 1.5
STATIC_BLOCK_POSITION_TOLERANCE_M = 0.5
STATIC_BLOCK_COST = 253
MOVING_PREDICTION_CLEARANCE_M = 0.8
PREDICTION_MAX_AGE_S = 1.2
LATEST_SENSOR_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE)

# Existing NewRooms desks, unchanged (1.8 m x 2.4 m, long sides north-south).
# The robot docks beside a long side with the desk on its right, like the
# USD spawn at the lab desk. *_DOCK must match hospital_docking.TABLES; each
# *_STATION is the staging pose 2.5 m before the dock on the same straight.
# The lab is entered heading north and left northward; the specimen desk is
# entered heading south and left southward, so neither needs an undock.
ARRIVAL_YAWS = {"specimen_to_lab": math.pi / 2, "lab_to_specimen": -math.pi / 2}
ROUTES = {"specimen_to_lab": "lane_upper", "lab_to_specimen": "lane_lower"}
LOWER_WEST_Y = 11.612499952316282
LAB_DOCK = (20.185, 13.8125)
LAB_STATION = (20.185, 11.3125)
SPECIMEN_DOCK = (-44.741, 12.9125)
SPECIMEN_STATION = (-44.741, 15.4125)
LANE_UPPER = [('line', SPECIMEN_DOCK, (-44.741, 10.4)),
 ('arc', (-43.241, 10.4), 1.5, 180, 270),
 ('line', (-43.241, 8.9), (-41.0, 8.9)),
 ('arc', (-41.0, 10.4), 1.5, 270, 360),
 ('line', (-39.5, 10.4), (-39.5, 11.0)),
 ('arc', (-38.0, 11.0), 1.5, 180, 90),
 ('line', (-38.0, 12.5), (-34.5, 12.5)),
 ('arc', (-34.5, 13.7), 1.2, 270, 360),
 ('arc', (-32.1, 13.7), 1.2, 180, 90),
 ('line', (-32.1, 14.9), (7.9, 14.9)),
 ('arc', (7.9, 13.7), 1.2, 90, 0),
 ('arc', (10.3, 13.7), 1.2, 180, 270),
 ('line', (10.3, 12.5), (14, 12.5)),
 ('arc', (14, 11.5), 1, 90, 0),
 ('arc', (16, 11.5), 1, 180, 270),
 ('line', (16, 10.5), (19.3725, 10.5)),
 ('arc', (19.3725, 11.3125), 0.8125, 270, 360)]
LANE_LOWER = [('line', LAB_DOCK, (20.185, 16.5)),
 ('arc', (18.685, 16.5), 1.5, 0, 90),
 ('line', (18.685, 18), (16, 18)),
 ('arc', (16, 16.55), 1.45, 90, 180),
 ('line', (14.55, 16.55), (14.55, 13.95)),
 ('arc', (13.1, 13.95), 1.45, 0, -90),
 ('line', (13.1, 12.5), (10, 12.5)),
 ('arc', (10, 9.5), 3, 90, 180),
 ('line', (7, 9.5), (7, 1.5)),
 ('arc', (4, 1.5), 3, 0, -90),
 ('line', (4, -1.5), (-28.55, -1.5)),
 ('arc', (-28.55, 1.5), 3, 270, 180),
 ('line', (-31.55, 1.5), (-31.55, 8.612499952316282)),
 ('arc', (-34.55, 8.612499952316282), 3, 0, 90),
 ('line', (-34.55, LOWER_WEST_Y), (-40.241, LOWER_WEST_Y)),
 ('arc', (-40.241, LOWER_WEST_Y+1.5), 1.5, 270, 180),
 ('line', (-41.741, LOWER_WEST_Y+1.5), (-41.741, 16.2)),
 ('arc', (-43.241, 16.2), 1.5, 0, 180),
 ('line', (-44.741, 16.2), SPECIMEN_STATION)]

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
                LATEST_SENSOR_QOS,
            ),
            navigator.create_subscription(
                PointCloud,
                "/hospital/predicted_obstacles",
                self._prediction_callback,
                LATEST_SENSOR_QOS,
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
        now = self.navigator.get_clock().now().nanoseconds / 1e9
        if not 0 <= now-self._stamp_seconds(self.costmap.header) <= 1.0:
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


def _wait_until_active(navigator, node_name):
    """BasicNavigator._waitForNodeToActivate without its unbounded wait.

    Isaac 실행 중 get_state 응답이 한 번 유실되자 미션이 goal 없이 계속 멈춰 있었다.
    """
    from lifecycle_msgs.srv import GetState

    client = navigator.create_client(GetState, f"{node_name}/get_state")
    try:
        while rclpy.ok():
            if not client.wait_for_service(timeout_sec=1.0):
                navigator.get_logger().info(f"{node_name}/get_state 대기 중")
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(navigator, future, timeout_sec=2.0)
            if not future.done():
                navigator.get_logger().warn(f"{node_name}/get_state 응답 없음, 재요청")
                continue
            if future.result().current_state.label == "active":
                return
            time.sleep(1.0)
    finally:
        navigator.destroy_client(client)


def wait_until_nav2_active(navigator):
    """controller_server와 map -> base_link TF를 기다린다."""
    from tf2_ros import Buffer, TransformListener

    _wait_until_active(navigator, "controller_server")
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
    지속 차단에서 측방 후보가 없을 때만 호출한다.
    """
    try:
        translation = tf_buffer.lookup_transform(
            "map", "base_link", Time()
        ).transform.translation
    except Exception:
        return None
    try:
        if not navigator.compute_path_to_pose_client.wait_for_server(timeout_sec=0.0):
            print("  [DETOUR] planner server not ready")
            return None
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
    # NavFn's grid path can contain sharp corners. The configured smoother
    # gets one attempt; geometry and costmap validation still decide
    # whether the resulting path is safe to follow.
    try:
        if navigator.smoother_client.wait_for_server(timeout_sec=0.0):
            smoothed = navigator.smoothPath(
                detour, smoother_id="simple_smoother", max_duration=1.0,
                check_for_collision=True,
            )
            if smoothed is not None and len(smoothed.poses) >= 2:
                detour = smoothed
    except Exception as error:
        print(f"  [DETOUR] smoother 사용 불가; 원본 경로 검증: {error}")
    return detour


def path_is_blocked(path, start_index, costmap_checker):
    """향후 BLOCKED 판정용 자리. 지금은 판정하지 않는다."""
    return False


def run_path_stage(
    navigator, tf_buffer, plan_publisher, stage_name, route, controller_id,
    goal_checker_id, final_yaw=None, blockage_monitor=None,
):
    from nav_to_goal.hospital_stage_runner import follow_stage
    return follow_stage(
        navigator, tf_buffer, plan_publisher, stage_name, route, controller_id,
        goal_checker_id, final_yaw, blockage_monitor,
    )


def split_route(lane_id, route):
    """정류장 진출/중앙 이동/정류장 진입으로 나눈다.

    모든 원호는 DWB로 실행한다. MPPI의 기준 경로는 복도 직선뿐이며,
    사람을 피할 때의 국소적인 측면 이탈은 허용한다.
    """
    if lane_id == "lane_lower":
        # 방/벽 사이의 작은 호는 DWB로 고정하고, 중앙 하단 열린 구간만
        # MPPI에 맡긴다.
        return route[:10], route[10:11], route[11:]
    if lane_id == "lane_upper":
        return route[:9], route[9:10], route[10:]
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
    from nav_to_goal.hospital_docking import TABLES, TableDocking

    origin = 'lab' if route_id == 'lab_to_specimen' else 'specimen'
    destination = 'specimen' if route_id == 'lab_to_specimen' else 'lab'
    current_tf = tf_buffer.lookup_transform('map', 'base_link', Time()).transform
    current = (current_tf.translation.x, current_tf.translation.y,
               _yaw_from_quaternion(current_tf.rotation))
    # 책상 옆에서 AMCL이 수십 cm 밀리면 costmap이 차체를 책상 안에 둔다.
    # 출발 전에 라이다로 잰 책상 기준 자세로 AMCL을 다시 맞춘다.
    if math.dist(current[:2], TABLES[origin]['dock'][:2]) < 1.0:
        docking = TableDocking(navigator, tf_buffer)
        try:
            docking.relocalize(origin)
        finally:
            docking.close()
        current_tf = tf_buffer.lookup_transform('map', 'base_link', Time()).transform
        current = (current_tf.translation.x, current_tf.translation.y,
                   _yaw_from_quaternion(current_tf.rotation))
    # 출발 경로는 출발지 책상 도킹 자세(USD spawn 포함)에서 책상을 따라 직진한다.
    # Isaac에서 정지 중인 로봇이 분당 ~6 cm 밀리므로 첫 직선 위 가장 가까운 점과 비교한다.
    expected = min(sample_route(departure[:1]),
                   key=lambda p: math.dist(p[:2], current[:2]))
    if (math.dist(current[:2], expected[:2]) > .5 or
            abs(math.atan2(math.sin(current[2]-expected[2]), math.cos(current[2]-expected[2]))) > .35):
        navigator.get_logger().error(
            f'Unexpected start pose {current}; expected dock {expected}. No goal sent.')
        return MissionStatus.FAILED
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
            "FollowPathDock",
            # transit(0.8 rad)은 FollowPathDock의 회전 창(0.05 m)에 들기 전에
            # 도착 처리해서 책상 옆 직진 방향이 맞지 않는다. general은 그 창과 같다.
            "general_goal_checker",
            ARRIVAL_YAWS[route_id],  # 책상 긴 변이 로봇 우측에 오는 직진 방향
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

    docking = TableDocking(navigator, tf_buffer)
    try:
        if not docking.move(destination):
            print('[MISSION] FAILED: table_docking')
            return MissionStatus.FAILED
        docking.relocalize(destination)
    finally:
        docking.close()
    print(f"[MISSION] {MissionStatus.SUCCEEDED.value}: table docked (desk long side on the right)")
    return MissionStatus.SUCCEEDED


def _use_large_udp_buffers():
    """TableDocking이 받는 800 KB 라이다가 기본 UDP 버퍼에서 유실되지 않게 한다."""
    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        return
    from ament_index_python.packages import (
        PackageNotFoundError, get_package_share_directory)
    try:
        share = get_package_share_directory("carter_navigation")
    except PackageNotFoundError:
        return
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = os.path.join(
        share, "params", "fastdds_udp_4mb.xml")


def main():
    _use_large_udp_buffers()
    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters(
        [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )
    navigator.declare_parameter("route_id", "lab_to_specimen")
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
