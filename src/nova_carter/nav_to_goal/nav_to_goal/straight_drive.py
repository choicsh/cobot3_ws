"""자세를 바꾸지 않고 직진만 하는 cmd_vel 드라이버.

Nav2 를 쓰지 않는 구간(책상 도킹 / 이탈)에 쓴다. 경로 생성도 회피도 하지 않는다.
isaacpjt/pjt_alpha/nav_mission.py 의 FinalDriver 에서 필요한 부분만 가져왔고,
상수는 거기서 실측으로 잡힌 값을 그대로 쓴다.

직진(run_forward)은 목표를 좌표로 받지 않고 "현재 heading 방향으로 몇 m" 로 받는다.
책상 옆이나 좁은 통로에서는 회전 자체가 불가능하기 때문이다.
(실측: 출발 지점 통과폭 1.20m, 차폭 1.0m, 회전 필요 지름 2.11m)

**회전(turn_to)은 넓은 곳에서만 쓸 것.** 장애물 검사를 하지 않는다 — DWB 를 거치지 않고
cmd_vel 을 직접 내보내므로, 부르는 쪽이 그 자리의 여유를 책임져야 한다.
제자리 회전 필요 반경은 sqrt(1.86^2 + 1.0^2) / 2 = 1.06 m 다.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

# nav_mission.py 에서 실측으로 잡힌 값들
CMD_HZ = 200.0           # /cmd_vel 에 20Hz 로 흐르는 0 을 희석한다. 200Hz 면 0 비중이 약 9%
CRUISE_MPS = 0.35        # 랙 위 트레이가 미끄러지지 않을 속도
APPROACH_MPS = 0.10      # 남은 거리가 SLOW_AT_M 이하일 때
SLOW_AT_M = 0.60
STOP_TOL_M = 0.03        # 남은 전진거리가 이 값 이하면 정지
TIMEOUT_S = 90.0

# 제자리 회전용. 각속도는 Nav2 쪽과 같은 상한을 쓴다 (스크립트가 더 거칠면 안 된다).
TURN_MAX_W = 0.5      # rad/s. FollowPath.max_vel_theta / behavior_server.max_rotational_vel 과 동일
TURN_MIN_W = 0.08     # 이 아래로 떨어지면 정지 마찰을 못 이겨 각도가 안 줄어든다
TURN_KP = 1.0         # 오차 0.5 rad 부터 비례 감속. 그 위는 TURN_MAX_W 로 포화된다
TURN_TOL_RAD = 0.02   # 1.15도. AMCL 각도 추정 오차(~0.03 rad)보다 작게 잡을 이유는 없다


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StraightDriver(Node):
    """현재 자세 그대로 앞뒤로만 움직인다."""

    def __init__(self, name="straight_driver", topic="/cmd_vel"):
        super().__init__(name)
        self.pub = self.create_publisher(Twist, topic, 10)
        self.buf = Buffer()
        TransformListener(self.buf, self)

    def pose(self):
        try:
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def send(self, v, w=0.0):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub.publish(m)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.001)

    def wait_pose(self, timeout=15.0):
        """TF 리스너를 막 만든 직후에는 버퍼가 비어 있다. 채워질 때까지 스핀하며 기다린다."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is not None:
                return p
        return None

    def wait_stopped(self, tol_m=0.01, settle_s=0.2, timeout=20.0):
        """위치 변화가 멎을 때까지 기다린 뒤 최종 pose 를 준다.

        시간이 아니라 **이동량**으로 판정한다. 이 노드들은 use_sim_time 을 안 쓰므로
        time.time() 은 벽시계이고, Isaac 이 실시간보다 느리게 돌면 고정 대기시간은
        감속이 끝나기 전에 풀린다.
        """
        t0 = time.time()
        last = self.wait_pose()
        while time.time() - t0 < timeout:
            self.spin_for(settle_s)
            p = self.pose()
            if p is None:
                continue
            if last is not None and math.dist(p[:2], last[:2]) < tol_m:
                return p
            last = p
        return self.pose()

    def run_forward(self, dist, label=""):
        """현재 자세 그대로 dist 만큼 전진한다. 음수면 후진.

        정지 판정은 "목표까지 남은 거리를 진행방향에 투영한 값"이다.
        주행 거리를 시간으로 재면 가감속 구간에서 오차가 누적되는데, 투영값은 실제 위치 기준이라
        그게 흡수된다. (nav_mission.py FinalDriver.run 과 같은 방식)
        """
        p = self.wait_pose()
        if p is None:
            self.get_logger().error("map->base_link TF 를 15초 동안 못 받았다. 직진 불가")
            return False
        x0, y0, yaw = p
        gx, gy = x0 + dist * math.cos(yaw), y0 + dist * math.sin(yaw)
        sign = 1.0 if dist >= 0 else -1.0
        tag = f"[{label}] " if label else ""
        self.get_logger().info(
            f"{tag}직진 시작 ({x0:.3f}, {y0:.3f}) yaw {math.degrees(yaw):+.1f} deg "
            f"-> {dist:+.2f} m -> ({gx:.3f}, {gy:.3f})")

        dt = 1.0 / CMD_HZ
        t0 = time.time()
        while time.time() - t0 < TIMEOUT_S:
            rclpy.spin_once(self, timeout_sec=0.0)
            p = self.pose()
            if p is None:
                self.send(0.0)
                self.spin_for(dt)
                continue
            x, y, yaw_now = p
            # 진행방향 투영 잔여거리. 후진이면 sign 으로 부호를 뒤집어 같은 식을 쓴다
            remain = sign * ((gx - x) * math.cos(yaw_now) + (gy - y) * math.sin(yaw_now))
            if remain <= STOP_TOL_M:
                self.send(0.0)
                self.spin_for(0.3)
                self.get_logger().info(
                    f"{tag}직진 완료 ({x:.3f}, {y:.3f})  실제 이동 "
                    f"{math.dist((x0, y0), (x, y)):.3f} m  (남은거리 {remain * 100:+.1f} cm)")
                return True
            v = APPROACH_MPS if remain < SLOW_AT_M else CRUISE_MPS
            self.send(sign * v)
            self.spin_for(dt)

        self.send(0.0)
        self.spin_for(0.3)
        self.get_logger().error(f"{tag}직진 시간 초과 ({TIMEOUT_S:.0f}s)")
        return False

    def turn_to(self, yaw_goal, label=""):
        """제자리에서 map 기준 절대 yaw(rad) 로 돌린다. 병진은 0 이다.

        **장애물 검사가 없다.** 부르는 쪽이 그 자리에 회전 여유(반경 1.06 m)가 있는지
        확인한 뒤 부를 것. run_forward 와 달리 목표가 절대각이라 누적 오차가 안 쌓인다.
        """
        p = self.wait_pose()
        if p is None:
            self.get_logger().error("map->base_link TF 를 15초 동안 못 받았다. 회전 불가")
            return False
        tag = f"[{label}] " if label else ""
        err0 = math.atan2(math.sin(yaw_goal - p[2]), math.cos(yaw_goal - p[2]))
        self.get_logger().info(
            f"{tag}회전 시작 yaw {math.degrees(p[2]):+.1f} -> {math.degrees(yaw_goal):+.1f} deg "
            f"({math.degrees(err0):+.1f} deg)")

        dt = 1.0 / CMD_HZ
        t0 = time.time()
        while time.time() - t0 < TIMEOUT_S:
            rclpy.spin_once(self, timeout_sec=0.0)
            p = self.pose()
            if p is None:
                self.send(0.0)
                self.spin_for(dt)
                continue
            err = math.atan2(math.sin(yaw_goal - p[2]), math.cos(yaw_goal - p[2]))
            if abs(err) <= TURN_TOL_RAD:
                self.send(0.0)
                self.spin_for(0.3)
                self.get_logger().info(
                    f"{tag}회전 완료 yaw {math.degrees(p[2]):+.2f} deg "
                    f"(남은 오차 {math.degrees(err):+.2f} deg)")
                return True
            w = min(TURN_MAX_W, max(TURN_MIN_W, TURN_KP * abs(err)))
            self.send(0.0, math.copysign(w, err))
            self.spin_for(dt)

        self.send(0.0)
        self.spin_for(0.3)
        self.get_logger().error(f"{tag}회전 시간 초과 ({TIMEOUT_S:.0f}s)")
        return False
