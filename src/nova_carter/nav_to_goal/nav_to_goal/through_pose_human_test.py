"""goThroughPoses 가 걸어다니는 사람을 실제로 피하는지 확인하는 실험.

    ros2 run nav_to_goal through_pose_human_test

전제: Isaac 에서 run_human_scene.py 로 integration_human.usd 가 떠 있고,
      단일 로봇 Nav2 스택(carter_navigation_params.yaml, 맵 intergration_nova.yaml)이 살아 있을 것.

확인하려는 것 두 가지
  1. 경유지에서 멈추는가          -> 멈추면 안 된다. NavigateThroughPoses 는 중간 pose 를
                                     goal 이 아니라 viapoint 로 다루고(RemovePassedGoals radius 0.7),
                                     goal_checker/RotateToGoal 을 거치지 않는다.
  2. 사람을 피해 경로를 다시 짜는가 -> 전역 costmap 이 /scan 을 받고 BT 가 0.333Hz 로 재계획한다.
                                     /plan 이 바뀌면 우회한 것이다.

정지가 잡히면 그 원인을 가르기 위해 두 토픽을 같이 본다
  /cmd_vel_nav  controller_server(DWB) 가 내보낸 원 명령
  /cmd_vel      velocity_smoother -> collision_monitor 를 거쳐 로봇에 실제로 들어간 명령
둘 다 0 이면 DWB 가 멈춘 것, /cmd_vel_nav 만 살아 있으면 스무더/모니터가 깎은 것이다.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray

from nav_to_goal.straight_drive import StraightDriver

# Isaac(pick_and_place_detection.py) 과의 손잡기. 같은 이름을 그쪽에도 둔다.
#   /mission_state  Isaac -> 여기 : 1 = 적재+관측 끝났다, 출발해도 된다
#   /nav_done       여기 -> Isaac : 1 = 도킹까지 끝났다, 하역해도 된다
# --solo 로 실행하면 기다리지 않고 바로 출발한다 (주행만 단독 시험할 때).
MISSION_TOPIC = "/mission_state"
NAV_DONE_TOPIC = "/nav_done"
DONE_ANNOUNCE_S = 5.0   # Isaac 쪽은 OmniGraph 구독이라 latched 를 못 믿는다 — 잠깐 반복 발행

# nav_mission.py 의 실측 좌표. 주석 처리돼 있던 경유지를 되살렸다.
# (6.641, 0.138) -> (7.398, 17.238) 구간이 people/command.txt 의 사람 배회 구역
# (대략 x 5~10, y 6~10) 을 정면으로 관통한다. 조우가 보장된다.
INIT_POSE = (1.2303, 3.36282, 180.0)

# 책상 이탈 / 도킹은 Nav2 가 아니라 cmd_vel 직진으로 한다.
# 출발 지점 통과폭이 1.20m(차폭 1.0m, 회전 필요 지름 2.11m)라 회전이 불가능하고,
# 경로 생성도 회피도 필요 없는 구간이다.
UNDOCK_M = 3.0    # 출발 자세(yaw 180deg) 그대로 전진. 통과폭 1.20 -> 3.70m 구간을 빠져나온다
DOCK_M = 5.0      # 마지막 경유지 도착 후 같은 자세로 전진해 도킹 지점까지 간다.
                  # 4.0 으로 정상 주행 확인 후 5.0 으로 연장. 통과폭은 4~5m 구간 내내 0.90~0.95m 로 동일
DOCK_GUARD_M = 1.0  # 마지막 경유지에서 이 거리 안에 있을 때만 도킹한다.
                    # Nav2 결과(SUCCEEDED)만 믿으면 안 된다. 실측으로, 로봇이 Nav2 구간을
                    # 한 발짝도 못 갔는데 SUCCEEDED 가 나와 UNDOCK 종료 지점(-1.765, 3.425)에서
                    # 3m 를 돌진했고, 그 방향은 2.25m 앞이 벽이었다.
# 주의: 맵상 WP5 에서 3m 지점은 통과폭 0.90m 로 차폭 1.0m 보다 좁다.
#       맵에 없는 책상 틈으로 들어가는 동작이라 실행 결과를 보고 조정할 것.

# 원래 있던 (-1.937, 3.416) 은 뺐다. UNDOCK_M 3m 지점 (-1.770, 3.363) 과 0.175m 라
# 출발하자마자 도달해 버려 경유지로서 의미가 없다.
WAYPOINTS = [
    (-1.958, -0.352, 270.0),
    ( 6.641,  0.138,  90.0),
    ( 7.398, 17.238, 150.0),
    ( 5.826, 19.343, 180.0),
]

STOP_SPEED = 0.02      # m/s. 이 아래면 정지로 본다
STOP_MIN_SEC = 0.4     # 이 시간 이상 이어져야 "정지 1회" 로 센다
NEAR_WP_M = 1.2        # 정지 지점이 경유지에서 이 거리 안이면 '경유지 정지' 로 분류
DETOUR_M = 0.3         # 전역 경로 길이가 이만큼 "늘어나면" 우회로 센다.
                       # 길이 변화가 아니라 증가만 본다. 전진하면 경로는 자연히 짧아지므로
                       # 변화량으로 세면 단순 전진을 우회로 오판한다 (실측: 3회 전부 오탐).
VIAPOINT_RADIUS = 0.7  # BT XML 의 RemovePassedGoals radius 와 반드시 같게 둘 것.
                       # 경유지 최근접 거리가 이 값보다 크면 그 경유지는 영영 안 지워지고
                       # 로봇이 되돌아간다.


def make_pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    r = math.radians(yaw_deg) / 2.0
    p.pose.orientation.z = math.sin(r)
    p.pose.orientation.w = math.cos(r)
    return p


def path_length(msg):
    pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class Probe:
    """nav 노드에 구독만 얹어 계측한다. isTaskComplete() 가 매번 spin 하므로 콜백이 돈다."""

    def __init__(self, nav, wps):
        self.nav = nav
        self.wps = wps
        # 경유지별 (최근접 거리, 그때 시각). 통과 정확도 판정용.
        self.near = [[float("inf"), -1.0] for _ in wps]
        self.t0 = time.time()
        self.pose = None
        self.v_now = 0.0
        self.v_nav = 0.0
        self.v_min_moving = 9.9        # 움직이는 동안의 최저 속도
        self.samples = 0
        self.stop_since = None
        self.stops = []                # [(시작초, 지속초, (x,y), DWB도_0이었나)]
        self.plan_len = None
        self.detours = []              # [(초, 이전길이, 새길이)] — 길이가 늘어난 경우만
        self.track = []                # [(초, x, y)]

        nav.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        nav.create_subscription(Twist, "/cmd_vel_nav", self._cmd_nav, 10)
        nav.create_subscription(Path, "/plan", self._plan, 10)
        nav.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl, 10)

    # ---- 콜백

    def _cmd_nav(self, msg):
        self.v_nav = math.hypot(msg.linear.x, msg.linear.y)

    def _cmd(self, msg):
        t = time.time() - self.t0
        v = math.hypot(msg.linear.x, msg.linear.y)
        self.v_now = v
        self.samples += 1

        if v < STOP_SPEED:
            if self.stop_since is None:
                self.stop_since = (t, self.pose, self.v_nav < STOP_SPEED)
        else:
            self.v_min_moving = min(self.v_min_moving, v)
            if self.stop_since is not None:
                t0, pose, dwb_zero = self.stop_since
                if t - t0 >= STOP_MIN_SEC:
                    self.stops.append((t0, t - t0, pose, dwb_zero))
                self.stop_since = None

    def _plan(self, msg):
        if not msg.poses:
            return
        L = path_length(msg)
        if self.plan_len is None:
            self.plan_len = L
            return
        if L - self.plan_len >= DETOUR_M:          # 증가만. 감소는 단순 전진이다
            self.detours.append((time.time() - self.t0, self.plan_len, L))
        self.plan_len = L

    def _amcl(self, msg):
        p = msg.pose.pose.position
        t = time.time() - self.t0
        self.pose = (p.x, p.y)
        self.track.append((t, p.x, p.y))
        for i, w in enumerate(self.wps):
            d = math.dist(self.pose, w[:2])
            if d < self.near[i][0]:
                self.near[i] = [d, t]

    # ---- 결과

    def close_open_stop(self):
        if self.stop_since is not None:
            t0, pose, dwb_zero = self.stop_since
            dur = (time.time() - self.t0) - t0
            if dur >= STOP_MIN_SEC:
                self.stops.append((t0, dur, pose, dwb_zero))
            self.stop_since = None

    def travelled(self):
        return sum(math.dist(a[1:], b[1:]) for a, b in zip(self.track, self.track[1:]))

    def report(self, wps, result):
        self.close_open_stop()
        dur = time.time() - self.t0
        print("\n" + "=" * 62)
        print(f"주행 {dur:.1f}s | 이동거리 {self.travelled():.1f}m | "
              f"cmd_vel 샘플 {self.samples}")

        if self.samples == 0:
            print("\n  /cmd_vel 을 한 번도 못 받았다. Nav2 가 안 떴거나 토픽 이름이 다르다.")
            print("  ros2 topic list | grep cmd_vel 로 먼저 확인할 것.")
            return

        print(f"\n[정지] {len(self.stops)}회  (v<{STOP_SPEED} 가 {STOP_MIN_SEC}s 이상)")
        for t0, d, pose, dwb_zero in self.stops:
            where, tag = "위치불명", ""
            if pose:
                near = min(((math.dist(pose, w[:2]), i) for i, w in enumerate(wps)),
                           default=(9e9, -1))
                where = f"({pose[0]:.1f}, {pose[1]:.1f})"
                tag = (f"  <- 경유지 {near[1] + 1} 에서 {near[0]:.2f}m  **경유지 정지**"
                       if near[0] <= NEAR_WP_M else f"  (가장 가까운 경유지까지 {near[0]:.1f}m)")
            cause = "DWB 가 0 을 냄" if dwb_zero else "DWB 는 살아 있음(스무더/모니터가 깎음)"
            print(f"   t={t0:6.1f}s  {d:4.1f}s  {where}  [{cause}]{tag}")

        wp_stops = sum(1 for _, _, p, _ in self.stops
                       if p and min(math.dist(p, w[:2]) for w in wps) <= NEAR_WP_M)

        print(f"\n[경유지 통과 정확도]  radius={VIAPOINT_RADIUS}m")
        missed = 0
        for i, (w, (d, t)) in enumerate(zip(wps, self.near)):
            if d == float("inf"):
                print(f"   WP{i + 1} ({w[0]:6.2f},{w[1]:6.2f})  접근 기록 없음")
                missed += 1
                continue
            last = (i == len(wps) - 1)
            if last:
                verdict = "최종 목표 (goal_checker 담당)"
            elif d <= VIAPOINT_RADIUS:
                verdict = "통과 판정 O"
            else:
                verdict = "**미판정** <- 이 경유지가 안 지워진다"
                missed += 1
            print(f"   WP{i + 1} ({w[0]:6.2f},{w[1]:6.2f})  최근접 {d:5.2f}m  "
                  f"t={t:6.1f}s   {verdict}")

        print(f"\n[우회] {len(self.detours)}회  (경로 길이가 {DETOUR_M}m 이상 늘어남)")
        for t, a, b in self.detours:
            print(f"   t={t:6.1f}s  {a:.1f}m -> {b:.1f}m  (+{b - a:.1f}m)")

        print(f"\n[최저 속도] 움직이는 동안 {self.v_min_moving:.2f} m/s")
        print(f"[결과] {result}")

        print("\n--- 판정 ---")
        if wp_stops == 0:
            print("  경유지 정지 0회 — goThroughPoses 가 의도대로 연속 통과했다.")
        else:
            print(f"  경유지 정지 {wp_stops}회 — viapoint 인데 멈췄다. "
                  "RemovePassedGoals 의 radius(기본 0.7m)와 BT 설정을 확인할 것.")
        if missed:
            print(f"  경유지 {missed}개가 radius 밖으로 지나갔다. 그 경유지는 지워지지 않으므로 "
                  "로봇이 되돌아간다. BT XML 의 radius 를 그 최근접 거리보다 크게 하거나, "
                  "\n  경로가 그 점에 더 가까이 붙도록 해야 한다.")
        else:
            print("  모든 경유지를 radius 안으로 통과했다.")
        if self.detours:
            print(f"  우회 {len(self.detours)}회 — 전역 경로가 길어졌다. 사람을 피해 돌아간 것이다.")
        else:
            print("  우회 0회 — 사람을 만나지 않았거나, /scan 이 전역 costmap 에 안 들어가고 있다.")
        print("=" * 62)


def wait_for_mission(node):
    """Isaac 이 적재+관측을 끝낼 때까지 기다린다."""
    state = [0.0]
    node.create_subscription(
        Float32MultiArray, MISSION_TOPIC,
        lambda m: state.__setitem__(0, m.data[0] if m.data else 0.0), 10)
    print(f"\n[WAIT] Isaac 의 적재 완료 신호({MISSION_TOPIC} = 1) 대기 — Isaac 에서 Play 를 누를 것")
    t0 = time.time()
    last = 0.0
    while state[0] < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.time() - t0
        if now - last >= 30.0:
            last = now
            print(f"   t={now:5.0f}s  아직 대기 중 (현재 값 {state[0]:.0f}). "
                  f"토픽 확인: ros2 topic echo {MISSION_TOPIC} --once")
    print(f"[WAIT] 적재 완료 신호 수신 ({time.time() - t0:.0f}s 대기) — 주행을 시작한다")


def announce_done(node):
    """도킹 완료를 Isaac 에 알린다."""
    pub = node.create_publisher(Float32MultiArray, NAV_DONE_TOPIC, 10)
    msg = Float32MultiArray(data=[1.0])
    end = time.time() + DONE_ANNOUNCE_S
    while time.time() < end:
        pub.publish(msg)
        time.sleep(0.1)
    print(f"[DONE] {NAV_DONE_TOPIC} = 1 을 {DONE_ANNOUNCE_S:.0f}초간 발행했다 — Isaac 이 하역을 시작한다")


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose(nav, *INIT_POSE))
    print("Nav2 기동 대기...")
    nav.waitUntilNav2Active()

    drv = StraightDriver()

    if "--solo" not in sys.argv:
        wait_for_mission(drv)

    print(f"\n[UNDOCK] cmd_vel 로 {UNDOCK_M:.1f} m 전진 (Nav2 미사용)")
    if not drv.run_forward(UNDOCK_M, "UNDOCK"):
        print("  undock 실패. 중단한다.")
        drv.destroy_node()
        nav.destroyNode()
        rclpy.shutdown()
        return

    # Probe 는 undock 이 끝난 뒤에 만든다. undock 중 대기가 "정지"로 잡히면 안 되고,
    # 계측 시각 t0 도 Nav2 구간 시작에 맞춘다.
    poses = [make_pose(nav, *w) for w in WAYPOINTS]
    probe = Probe(nav, WAYPOINTS)

    print(f"\n[NAV] goThroughPoses 경유지 {len(poses)}개 — 한 번만 발행한다")
    nav.goThroughPoses(poses)

    last = 0.0
    while not nav.isTaskComplete():
        # 드라이버 노드도 계속 돌려야 한다. TF 리스너는 노드를 spin 해야 데이터가 들어오는데,
        # 여기서 안 돌리면 Nav2 구간 내내 버퍼가 undock 직후 값에 멈춘다.
        # lookup_transform(..., Time()) 은 "가장 최신"이라 조회는 성공하고 값만 옛날 것이 된다.
        # (실측: 도착 후 도킹 가드가 100초 전 좌표를 읽어 마지막 경유지까지 17.64m 로 판정)
        rclpy.spin_once(drv, timeout_sec=0)
        fb = nav.getFeedback()
        now = time.time() - probe.t0
        if fb and now - last >= 2.0:
            last = now
            print(f"  t={now:5.1f}s  남은 {fb.distance_remaining:5.2f}m  "
                  f"v={probe.v_now:.2f}  정지 {len(probe.stops)}회  "
                  f"우회 {len(probe.detours)}회")

    result = {
        TaskResult.SUCCEEDED: "SUCCEEDED",
        TaskResult.CANCELED: "CANCELED",
        TaskResult.FAILED: "FAILED",
    }.get(nav.getResult(), str(nav.getResult()))

    # 도킹 전에 실제 위치를 확인한다. Nav2 결과만으로는 부족하다 (DOCK_GUARD_M 주석 참고).
    dock_ok = None
    here = drv.wait_pose(timeout=3.0)
    d_last = math.dist(here[:2], WAYPOINTS[-1][:2]) if here else float("inf")
    if result != "SUCCEEDED":
        print(f"\n[DOCK] 건너뜀 — Nav2 결과가 {result}")
    elif d_last > DOCK_GUARD_M:
        print(f"\n[DOCK] 건너뜀 — 마지막 경유지까지 {d_last:.2f} m "
              f"(허용 {DOCK_GUARD_M:.1f} m). 현재 위치 "
              f"{'(%.3f, %.3f)' % here[:2] if here else '알 수 없음'}")
        print("        Nav2 는 SUCCEEDED 를 냈지만 실제로는 도착하지 않았다.")
    else:
        print(f"\n[DOCK] cmd_vel 로 {DOCK_M:.1f} m 전진 "
              f"(마지막 경유지까지 {d_last:.2f} m, Nav2 미사용)")
        dock_ok = drv.run_forward(DOCK_M, "DOCK")
        if dock_ok:
            announce_done(drv)

    probe.report(WAYPOINTS, result)
    if dock_ok is None:
        print(f"[도킹] 실행 안 함  (마지막 경유지까지 {d_last:.2f} m)")
    else:
        print(f"[도킹] {'성공' if dock_ok else '실패'}  ({DOCK_M:.1f} m 전진)")

    drv.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
