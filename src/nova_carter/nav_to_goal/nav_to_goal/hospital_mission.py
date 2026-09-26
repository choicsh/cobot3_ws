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
import sys
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

from nav_to_goal.hospital_lanes import (  # noqa: E402 — 차선 기하는 한 곳(hospital_lanes)에
    DEFAULT_TRACK, LAB_DOCK, LAB_STATION, SPECIMEN_DOCK, SPECIMEN_STATION, TO_ANALYSIS,
    TO_COLLECTION, TRACKS, WEST_DOOR_Y, WEST_LOOP_X, compose)

ARRIVAL_YAWS = {"specimen_to_lab": math.pi / 2, "lab_to_specimen": -math.pi / 2}
ROUTES = {"specimen_to_lab": "lane_upper", "lab_to_specimen": "lane_lower"}
# route_id -> 방향 (복도를 서->동 / 동->서)
DIRECTIONS = {"specimen_to_lab": TO_COLLECTION, "lab_to_specimen": TO_ANALYSIS}
# 관제 없이 돌 때의 두 경로 (예전 lane_upper = 복귀 + 위 복도, lane_lower = 운송 + 아래 복도)
_DEFAULT = {lane: compose(DIRECTIONS[route_id], DEFAULT_TRACK[DIRECTIONS[route_id]])
            for route_id, lane in ROUTES.items()}
LANE_UPPER = _DEFAULT["lane_upper"][0]
LANE_LOWER = _DEFAULT["lane_lower"][0]

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
                "local_costmap/costmap_raw",
                self._costmap_callback,
                LATEST_SENSOR_QOS,
            ),
            navigator.create_subscription(
                PointCloud,
                "hospital/predicted_obstacles",
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
    goal_checker_id, final_yaw=None, blockage_monitor=None, zone_hold=None,
):
    from nav_to_goal.hospital_stage_runner import follow_stage
    return follow_stage(
        navigator, tf_buffer, plan_publisher, stage_name, route, controller_id,
        goal_checker_id, final_yaw, blockage_monitor, zone_hold,
    )


def split_route(lane_id, route, split=None):
    """정류장 진출/중앙 이동/정류장 진입으로 나눈다.

    모든 원호는 DWB로 실행한다. MPPI의 기준 경로는 복도 직선뿐이며,
    사람을 피할 때의 국소적인 측면 이탈은 허용한다. 출발 구간은 1.5 m 직선으로
    끝나 DWB가 방향을 맞춘 뒤 MPPI에 넘긴다(아크 끝에서 넘기면 22도 제자리 회전).
    split = hospital_lanes.compose 의 (MPPI 시작, 끝) — 없으면 기본 두 경로의 것.
    """
    if split is None:
        if lane_id not in _DEFAULT:
            raise ValueError(f"Unknown lane: {lane_id}")
        split = _DEFAULT[lane_id][1]
    i, j = split
    return route[:i], route[i:j], route[j:]


def _project_segment(segment, x, y):
    """(거리, 남은 부분 segment 또는 None) — 점 (x, y) 를 segment 에 투영해 그 뒤쪽만 남긴다."""
    if segment[0] == "line":
        (x0, y0), (x1, y1) = segment[1], segment[2]
        dx, dy = x1 - x0, y1 - y0
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / length2))
        px, py = x0 + t * dx, y0 + t * dy
        rest = None if (1.0 - t) * math.sqrt(length2) < PATH_STEP else ("line", (px, py), (x1, y1))
        return math.hypot(x - px, y - py), rest, math.atan2(dy, dx)
    (cx, cy), radius, start, finish = segment[1], segment[2], segment[3], segment[4]
    angle = math.degrees(math.atan2(y - cy, x - cx))
    lo, hi = min(start, finish), max(start, finish)
    # 호의 각도 범위([lo, hi], 360 넘어갈 수 있음)에 맞게 360 배수를 옮긴 뒤 자른다
    angle = min((angle + 360.0 * k for k in range(-2, 3)),
                key=lambda a: 0.0 if lo <= a <= hi else min(abs(a - lo), abs(a - hi)))
    angle = max(lo, min(hi, angle))
    px, py = cx + radius * math.cos(math.radians(angle)), cy + radius * math.sin(math.radians(angle))
    remaining = abs(finish - angle)
    rest = None if math.radians(remaining) * radius < PATH_STEP else ("arc", (cx, cy), radius, angle, finish)
    yaw = math.radians(angle) + (math.pi / 2 if finish > start else -math.pi / 2)
    return math.hypot(x - px, y - py), rest, yaw


def resume_stages(stages, pose, max_distance=1.0, max_heading=math.radians(60.0)):
    """차선 중간에서 멈춘 로봇이 같은 경로를 이어 가도록 stages 를 현재 위치부터 자른다.

    stages = [(이름, route segments, ...)], pose = (x, y, yaw). 가장 가까운 segment(진행 방향이
    max_heading 안인 것)를 찾아 그 stage 의 그 segment 를 투영점부터 남기고, 앞쪽은 버린다.
    경로에서 max_distance 보다 멀면 None (이어갈 수 없다 — 경로 밖이다).

    양보 예산 초과(yield_budget_exceeded_not_proof_of_lane_blockage)로 끝난 미션을 처음 도킹 자세부터
    다시 할 수는 없으므로(출발 검사) 에이전트가 이 모드로 재시도한다 (2026-09-25 P4)."""
    best = None
    for si, stage in enumerate(stages):
        for gi, segment in enumerate(stage[1]):
            distance, rest, yaw = _project_segment(segment, pose[0], pose[1])
            heading = abs(math.atan2(math.sin(pose[2] - yaw), math.cos(pose[2] - yaw)))
            if heading <= max_heading and (best is None or distance < best[0]):
                best = (distance, si, gi, rest)
    if best is None or best[0] > max_distance:
        return None
    _, si, gi, rest = best
    first = list(stages[si])
    first[1] = ([rest] if rest is not None else []) + list(stages[si][1][gi + 1:])
    out = ([tuple(first)] if first[1] else []) + [tuple(s) for s in stages[si + 1:]]
    return out


def run_mission(navigator, route_id, resume=False, zone_hold=None, track=""):
    """track = 가운데 복도 (hospital_lanes.TRACKS, 관제가 준다). 비우면 예전 고정 경로(lane_upper/lane_lower)."""
    if track:
        lane_id = track
        route, split = compose(DIRECTIONS[route_id], track)
    else:
        lane_id = ROUTES[route_id]
        route, split = LANES[lane_id], None
    departure, transit, arrival = split_route(lane_id, route, split)

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
    stages = [
        ("station_departure", departure, "FollowPath", "transit_goal_checker"),
        (lane_id, transit, "FollowPathMPPI", "transit_goal_checker"),
        # transit(0.8 rad)은 FollowPathDock의 회전 창(0.05 m)에 들기 전에 도착 처리해서 책상 옆 직진 방향이
        # 맞지 않는다. general은 그 창과 같다. 마지막 값 = 책상 긴 변이 로봇 우측에 오는 직진 방향
        ("station_arrival", arrival, "FollowPathDock", "general_goal_checker", ARRIVAL_YAWS[route_id]),
    ]
    if resume:
        stages = resume_stages(stages, current)
        if stages is None:
            navigator.get_logger().error(f'Resume: pose {current} is not on {lane_id}. No goal sent.')
            return MissionStatus.FAILED
        print(f"[MISSION] resume from {current[0]:.2f}, {current[1]:.2f}: "
              f"{', '.join(s[0] for s in stages)}", flush=True)
    # 책상 옆에서 AMCL이 수십 cm 밀리면 costmap이 차체를 책상 안에 둔다.
    # 출발 전에 라이다로 잰 책상 기준 자세로 AMCL을 다시 맞춘다.
    if not resume and math.dist(current[:2], TABLES[origin]['dock'][:2]) < 1.0:
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
    if not resume and (math.dist(current[:2], expected[:2]) > .5 or
            abs(math.atan2(math.sin(current[2]-expected[2]), math.cos(current[2]-expected[2]))) > .35):
        navigator.get_logger().error(
            f'Unexpected start pose {current}; expected dock {expected}. No goal sent.')
        return MissionStatus.FAILED
    blockage_monitor = AheadBlockageMonitor(navigator, tf_buffer)
    plan_publisher = navigator.create_publisher(
        Path,
        "plan",
        QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )


    for stage in stages:
        status = run_path_stage(
            navigator, tf_buffer, plan_publisher, *stage,
            blockage_monitor=blockage_monitor, zone_hold=zone_hold,
        )
        if status != MissionStatus.SUCCEEDED:
            print(f"[MISSION] {status.value}: {stage[0]}")
            return status

    if zone_hold is not None and not zone_hold.dock_allowed():
        # 앞 로봇이 아직 책상에 있다 — 관제가 도킹 구역을 줄 때까지 정류장에서 기다린다
        print(f"[MISSION] WAIT_DOCK: {destination}", flush=True)
        while rclpy.ok() and not zone_hold.dock_allowed():
            rclpy.spin_once(navigator, timeout_sec=0.1)
    # robot_agent 가 이 줄을 보고 단계를 PLACE_DOCKING / PICK_DOCKING 으로 바꾼다
    print(f"[MISSION] DOCKING: {destination}", flush=True)
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
    # True: 도킹 자세 출발 검사 없이 현재 위치에서 같은 경로를 이어 간다 (resume_stages)
    navigator.declare_parameter("resume", False)
    # True: 관제 구역 예약(zone_hold 토픽)을 따른다 — 첫 메시지 전에는 출발하지 않는다 (hospital_zone_hold)
    navigator.declare_parameter("require_zone_hold", False)
    navigator.declare_parameter("zone_leg", "")   # 이 미션의 구간 키 (robot_agent 가 준다)
    # 가운데 복도 (upper, upper_reserve, lower, lower_reserve — 관제가 고른다). 비우면 예전 고정 경로
    navigator.declare_parameter("route_track", "")
    route_id = navigator.get_parameter("route_id").value
    track = navigator.get_parameter("route_track").value
    resume = bool(navigator.get_parameter("resume").value)
    from nav_to_goal.hospital_zone_hold import ZoneHold
    zone_hold = ZoneHold(navigator, bool(navigator.get_parameter("require_zone_hold").value),
                         navigator.get_parameter("zone_leg").value)

    if route_id not in ROUTES or (track and track not in TRACKS):
        navigator.get_logger().error(
            f"잘못된 route_id={route_id!r} / route_track={track!r}; 사용 가능: {list(ROUTES)} / {list(TRACKS)}"
        )
        navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    status = MissionStatus.FAILED
    try:
        status = run_mission(navigator, route_id, resume, zone_hold, track)
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
    # 실패한 미션 뒤에 다음 미션이 이어서 실행되지 않도록 셸에 결과를 알린다.
    sys.exit(0 if status == MissionStatus.SUCCEEDED else 1)


if __name__ == "__main__":
    main()
