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

# 새 East 도킹 자세. 씬(hospital_integration_human.usd) 의 /World/robot_nova 실측값이다.
# 이력: (1.2303, 3.36282, 180.0) -> 현재 (2026-09-23, 병원 맵 교체).
# params yaml 의 amcl.initial_pose 와 **반드시 같은 값**이어야 한다 (두 군데에 있다).
INIT_POSE = (20.270, 13.745, 90.0)

# 책상 이탈 / 도킹은 Nav2 가 아니라 cmd_vel 직진으로 한다.
# 출발 지점 통과폭이 1.20m(차폭 1.0m, 회전 필요 지름 2.11m)라 회전이 불가능하고,
# 경로 생성도 회피도 필요 없는 구간이다.
UNDOCK_M = 3.0    # 출발 자세(yaw 90도, +y 방향) 그대로 전진해 East_DockDesk 앞을 빠져나온다.
                  # 이력: 옛 맵에서는 yaw 180도였고 '통과폭 1.20 -> 3.70m' 구간을 빠져나오는
                  # 동작이었다. 그 통과폭 실측은 옛 맵 값이라 병원 맵에서는 재측정 전이다.
                  # 3.0 이라는 거리 자체는 아직 새 맵에서 검증 안 됐다 — 이탈 방향(+y)이
                  # 책상에서 멀어지는 쪽이 맞는지 육안 확인 필요.
# 도킹 지점. **이 좌표 하나가 단일 기준**이고 마지막 경유지는 여기서 파생된다.
# (옛 구성은 DOCK_M 과 경유지 좌표가 서로 짝이라 한쪽만 바꾸면 어긋났다. 그 제약을 없앴다.)
# West_DockDesk bbox x[-47.191..-45.391] y[12.163..14.563] 상판 z 0.8127 기준,
# East 도킹 배치를 두 책상 중점(x -12.278)으로 180도 회전한 값이다.
DOCK_POSE = (-44.826, 12.980, -90.0)
DOCK_BACK_M = 4.0   # 마지막 경유지를 도킹 축 위 이만큼 뒤에 둔다.
                    # 맵 실측 자유반경 (제자리 회전 필요 1.06 m):
                    #   도킹점 +0.0m 0.55 / +2.0m 0.70 / +3.0m 1.55 / +4.0m 2.50
                    # 막는 건 서쪽(-x) 책상 한쪽뿐이고 동쪽은 뚫려 있다.
                    # 3.0 이었을 때 회전을 책상 바로 옆에서 해야 해 정렬이 안 됐다 -> 4.0.
ARRIVE_M = 0.5      # 마지막 경유지까지 이 거리 안에 들면 Nav2 를 취소하고 스스로 도킹한다.
                    # **Nav2 의 마지막 판정을 기다리지 않는다.** 정지거리(0.348 m)가 성공 창
                    # (xy_goal_tolerance 0.1)보다 커서, 목표를 지나친 자리에서 RotateToGoal 이
                    # 병진을 전부 불법 처리해 고착되는 일이 재현됐다 (실측: 0.33 m 앞에서 정지).
                    # do_dock() 이 정지 위치에서 도킹점까지 다시 재므로 축 방향 오차는
                    # 전부 흡수된다 — Nav2 가 정확히 설 이유 자체가 없다.
                    # 0.5 인 이유: 취소 시점 속도 1.18 m/s, 감속 2.0 -> 타력주행 0.348 m.
                    #   0.5 에서 취소하면 경유지 0.15 m 부근에 선다. 지나치지 않는 값이다.
# 도킹 직진(4m) 중 풋프린트가 통과 가능한 횡오차 한계. 맵 실측값이다.
# 책상이 서쪽(-x)에만 있어 극단적으로 비대칭이다:
#   서쪽(책상 쪽) +0.04 m,  동쪽 -0.80 m
# 축에서 서쪽으로 5cm 만 밀려도 도킹점 근처에서 책상에 닿는다 (실측으로 박았다).
DOCK_LAT_WEST_MAX = 0.04
DOCK_LAT_EAST_MAX = 0.80
SNAP_SKIP_M = 0.10  # 경유지까지 이보다 가까우면 조준 방위가 잡음에 묻히므로 보정을 건너뛴다

DOCK_GUARD_M = 1.0  # 마지막 경유지에서 이 거리 안에 있을 때만 도킹한다.
                    # Nav2 결과(SUCCEEDED)만 믿으면 안 된다. 실측으로, 로봇이 Nav2 구간을
                    # 한 발짝도 못 갔는데 SUCCEEDED 가 나와 UNDOCK 종료 지점(-1.765, 3.425)에서
                    # 3m 를 돌진했고, 그 방향은 2.25m 앞이 벽이었다.
# (삭제됨) 옛 맵 WP5 의 통과폭 0.90m 경고는 병원 맵과 무관해 지웠다.
# 새 맵 도킹 구간: 경유지 (-44.826, 16.980) -> 도킹 (-44.826, 12.980) 의 4m 직진.
# 두 점 모두 West_Room(x -49.20~-35.70, y 3.91~22.63) 안이고, 책상 동쪽 면(x -45.391)
# 에서 0.565m 떨어진 열린 쪽이다. 차체 반폭 0.5m 라 책상 쪽 여유는 6cm 뿐이다.
# 맵 실측 (0.1m 간격, 축 좌우 0.5m): 4m 전 구간 자유. 단 횡방향 오차에는 여유가 없다.

# 2026-09-23 병원 맵 교체. 경로 두 가지를 두고 --lane= 로 고른다.
#
# 공통 끝점. West_DockDesk bbox x[-47.191..-45.391] y[12.163..14.563] 상판 z 0.8127 기준,
# East 도킹 배치를 두 책상 중점(x -12.278)으로 180도 회전한 도킹 포즈가
# (-44.826, 12.980, -90도) 다. yaw -90 은 -y 를 보므로 4m 뒤 = y +4 -> 16.980.
# 이력: 3m 뒤(15.980) -> 4m 뒤. 3m 는 책상이 1.5m 옆이라 최종 정렬 회전이 잘 안 됐다.
# DOCK_POSE / DOCK_BACK_M 에서 계산한다 — 손으로 적지 않는다.
def _axis_point(back_m):
    """도킹 축 위에서 도킹점보다 back_m 만큼 뒤의 (x, y). yaw -90 이면 +y 쪽이다."""
    yaw = math.radians(DOCK_POSE[2])
    return (DOCK_POSE[0] - back_m * math.cos(yaw),
            DOCK_POSE[1] - back_m * math.sin(yaw))


DOCK_APPROACH = (*_axis_point(DOCK_BACK_M), DOCK_POSE[2])

# upper_lane — 경유지가 끝점 하나뿐이다. 맵 실측상 y~16 띠가 East 방부터 West 방까지
#   완전히 뚫려 있어(x -49~25 전 구간 free) 사실상 직선이다. 경유지가 하나면
#   RemovePassedGoals 는 아무것도 안 지우므로(소스가 while (goal_poses.size() > 1))
#   VIAPOINT_RADIUS / BT 의 radius / hz / forward_prune_distance 가 통과 판정에 영향을 안 준다.
#
# lower_lane — 병원 남쪽 (-10.968, -1.543) 을 경유한다 (맵 실측 +-3m 전부 free).
#   이쪽은 **진짜 경유지**라 radius 0.7 안으로 지나가야 하고, 못 지나가면 경유지가 안 지워져
#   경로가 뒤로 재생성되며 멈춘다. 중간 경유지의 yaw 는 다음 점을 향하는 방위각인데,
#   NavFn 은 중간 경유지의 방향을 쓰지 않으므로 표시용이다.
#
#   경유지 2개 -> 3개 (2026-09-23, (-30.906, 12.407) 추가).
#   통과 여유 계산 — e = R * (1/cos(꺾임각/2) - 1), 필요 radius = sqrt(e^2 + (v/hz/2)^2).
#   R = v / max_vel_theta = 1.18 / 0.5 = 2.36 m, v/hz = 1.18 / 2.0 = 0.59 m (BT hz 2.0 기준).
#
#     경유지      진입      진출     꺾임      e     필요radius   여유(0.7 기준)
#     WP1      -149.65  +145.02  -65.33   0.443    0.532      0.168 m
#     WP2(신규) +145.02  +161.81  +16.79   0.026    0.296      0.404 m
#
#   ⚠ WP1 의 꺾임각이 59.0 -> 65.33 도로 커졌다 (진출 방위가 DOCK_APPROACH 대신 신규 점을
#     향하게 됐기 때문). 여유가 0.28 -> 0.168 m 로 줄었으니 WP1 에서 되돌아가는 현상이
#     재발하면 여기부터 의심할 것. 신규 점 자체는 반경 4m 안이 전부 free (맵 실측).
ROUTES = {
    "upper_lane": [DOCK_APPROACH],
    "lower_lane": [
        (-10.968, -1.543, 145.0),
        (-30.906349182128906, 12.407171249389648, 161.8),
        DOCK_APPROACH,
    ],
}
LANE = next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--lane=")), "lower_lane")
assert LANE in ROUTES, f"--lane= 는 {list(ROUTES)} 중 하나여야 한다 (받은 값: {LANE!r})"
WAYPOINTS = ROUTES[LANE]

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


def do_dock(drv, here):
    """Nav2 가 놓고 간 자리에서 도킹점까지 스스로 간다. Nav2 미사용.

    Nav2 는 위치만 맞추고 방향은 안 본다 (params 의 yaw_goal_tolerance 3.15).
    여기서 **도킹 축 방향(DOCK_POSE 의 yaw)으로 고정 회전**한 뒤 축을 따라 직진한다.

    도킹점을 겨냥하는 방식(횡오차를 흡수하는 대신 비스듬히 진입)도 만들어 봤지만,
    책상과 나란하지 않게 들어가 자세가 틀어졌다 (육안 확인, 2026-09-23). 되돌렸다.
    지금 방식은 **자세는 항상 축과 나란하고, 횡방향 오차는 그대로 남는다.**
    책상 쪽 여유가 6cm 뿐이므로 Nav2 정지 위치의 횡오차가 그만큼 중요해진다 —
    도착 후 아래 '횡오차' 출력을 볼 것. 이게 크면 Nav2 정지 정확도부터 봐야 한다.

    전진 거리는 유클리드 거리가 아니라 **축 방향 투영**이다. 횡오차가 있을 때
    유클리드로 가면 그만큼 더 가서 책상 옆을 지나쳐 버린다.

    진입 뒤 한 번 더 정렬한다. run_forward 는 angular.z 를 0 으로 두고 병진만 하므로
    직진 중 yaw 가 밀리는 걸 잡아주지 않는다.

    도킹점에서 크게 돌 수는 없다 — 풋프린트가 자유 공간에 남는 한계가 맵 실측으로
    서쪽 2.0도 / 동쪽 6.0도 뿐이다 (책상이 서쪽 0.565m 에 있다. 축 위 지점별 한계:
    +0.0m 2/6도, +1.0m 4.5/6도, +2.0m 79.5/6도, +3.0m 이상 제한 없음).
    그런데 **목표각 쪽으로 도는 건 항상 안전하다.** 자유 범위가 목표각을 품은 한 구간
    [-6도, +2도] 이라, 지금 자세가 충돌 없이 서 있다면 이미 그 범위 안이고,
    목표각으로 좁히는 회전은 침범에서 멀어지는 방향이기 때문이다.
    (반대로 재정렬 각도가 이 범위를 넘게 나온다면, 그건 이미 진입 중에 책상을
     스쳤다는 뜻이다. 그 경우 회전이 문제가 아니라 진입 자체를 봐야 한다.)
    """
    gx, gy, gyaw_deg = DOCK_POSE
    gyaw = math.radians(gyaw_deg)
    ax, ay = DOCK_APPROACH[0], DOCK_APPROACH[1]

    # --- 1단계: 마지막 경유지로 붙는다 (횡오차 보정) ---
    # ARRIVE_M 으로 Nav2 를 일찍 끊으면 경유지까지 최대 0.5 m 가 남는다. 그 오차의 축 방향
    # 성분은 아래 투영이 흡수하지만 **횡방향은 흡수하지 못한다**. 서쪽 여유가 0.04 m 뿐이라
    # 그대로 직진하면 책상에 닿는다 (실측). 그래서 먼저 경유지를 찍고 간다.
    # 이 자리는 자유반경 1.5 m 이상이라 회전이 자유롭다 (맵 실측).
    x, y, _ = here
    d_ap = math.dist((x, y), (ax, ay))
    print(f"\n[DOCK] Nav2 미사용. 현재 ({x:.3f}, {y:.3f}), 마지막 경유지까지 {d_ap:.3f} m")
    if d_ap > SNAP_SKIP_M:
        print(f"       경유지 ({ax:.3f}, {ay:.3f}) 로 먼저 붙는다")
        if not drv.turn_to(math.atan2(ay - y, ax - x), "DOCK-SNAP-TURN"):
            return False
        if not drv.run_forward(d_ap, "DOCK-SNAP"):
            return False
        here = drv.wait_pose() or here
    else:
        print(f"       {SNAP_SKIP_M:.2f} m 안이라 경유지 보정은 건너뛴다")

    # --- 2단계: 도킹 축으로 정렬 ---
    if not drv.turn_to(gyaw, "DOCK-TURN"):
        return False

    # --- 3단계: 횡오차 확인 후 직진 ---
    here = drv.wait_pose() or here
    x, y, yaw = here
    along = (gx - x) * math.cos(gyaw) + (gy - y) * math.sin(gyaw)    # 축 방향 투영
    lateral = -(gx - x) * math.sin(gyaw) + (gy - y) * math.cos(gyaw)  # + 서쪽(책상), - 동쪽
    print(f"       축 방향 {along:.3f} m 전진,  횡오차 {lateral:+.3f} m "
          f"({'서쪽/책상쪽' if lateral > 0 else '동쪽'})")
    if lateral > DOCK_LAT_WEST_MAX or lateral < -DOCK_LAT_EAST_MAX:
        print(f"       ** 중단 — 횡오차가 통과 범위를 벗어났다 "
              f"(서 {DOCK_LAT_WEST_MAX:.2f} / 동 {DOCK_LAT_EAST_MAX:.2f} m, 맵 실측). "
              f"그대로 가면 책상에 박는다 **")
        return False
    if not drv.run_forward(along, "DOCK-DRIVE"):
        return False

    # --- 4단계: 진입 후 재정렬 ---
    after = drv.wait_pose()
    if after is None:
        print("       재정렬 건너뜀 — TF 를 못 읽었다")
        return True
    drift = math.degrees(math.atan2(math.sin(gyaw - after[2]), math.cos(gyaw - after[2])))
    print(f"       진입 후 yaw {math.degrees(after[2]):+.2f} deg, "
          f"축 대비 {drift:+.2f} deg 밀렸다 -> 재정렬")
    if abs(drift) > 6.0:
        print(f"       ** {abs(drift):.1f} deg 는 도킹점 자유 범위(서 2.0 / 동 6.0 deg)를 넘는다. "
              f"진입 중 책상을 스쳤을 수 있다 — 육안 확인 필요 **")
    return drv.turn_to(gyaw, "DOCK-REALIGN")


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

    print(f"\n[NAV] {LANE} — goThroughPoses 경유지 {len(poses)}개, 한 번만 발행한다")
    nav.goThroughPoses(poses)

    last = 0.0
    arrived = False           # Nav2 를 우리가 취소했는가 (= 도착으로 친다)
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

        # 마지막 경유지에 충분히 붙으면 Nav2 의 도착 판정을 기다리지 않고 끊는다.
        # feedback 의 distance_remaining 이 아니라 TF 실측 거리를 쓴다 — 전자는 경로를 따라
        # 남은 길이라 경유지를 지나친 뒤에도 줄지 않는 경우가 있다.
        p_now = drv.pose()
        if p_now is not None and math.dist(p_now[:2], WAYPOINTS[-1][:2]) <= ARRIVE_M:
            print(f"\n[NAV] 마지막 경유지 {ARRIVE_M:.1f} m 안 진입 — Nav2 취소하고 직접 도킹한다")
            nav.cancelTask()
            arrived = True
            break

    if arrived:
        here = drv.wait_stopped()   # 타력주행이 끝날 때까지 (시간이 아니라 이동량으로 판정)
        print(f"[NAV] 정지 완료 ({here[0]:.3f}, {here[1]:.3f}) "
              f"yaw {math.degrees(here[2]):+.1f} deg" if here else "[NAV] 정지 후 TF 를 못 읽었다")

    result = {
        TaskResult.SUCCEEDED: "SUCCEEDED",
        TaskResult.CANCELED: "CANCELED",
        TaskResult.FAILED: "FAILED",
    }.get(nav.getResult(), str(nav.getResult()))
    # 우리가 끊었으면 Nav2 상태는 의미가 없다. cancelTask 뒤 isTaskComplete 를 다시 안 부르므로
    # BasicNavigator.status 가 갱신되지 않아 getResult() 는 UNKNOWN 을 낸다. 그대로 찍으면 오해를 준다.
    if arrived:
        result = f"CANCELED (마지막 경유지 {ARRIVE_M:.1f} m 진입 — 의도된 취소, 실패 아님)"

    # 도킹 전에 실제 위치를 확인한다. Nav2 결과만으로는 부족하다 (DOCK_GUARD_M 주석 참고).
    dock_ok = None
    if not arrived:
        here = drv.wait_pose(timeout=3.0)
    d_last = math.dist(here[:2], WAYPOINTS[-1][:2]) if here else float("inf")
    if not arrived and result != "SUCCEEDED":
        print(f"\n[DOCK] 건너뜀 — Nav2 결과가 {result}")
    elif d_last > DOCK_GUARD_M:
        print(f"\n[DOCK] 건너뜀 — 마지막 경유지까지 {d_last:.2f} m "
              f"(허용 {DOCK_GUARD_M:.1f} m). 현재 위치 "
              f"{'(%.3f, %.3f)' % here[:2] if here else '알 수 없음'}")
        print("        Nav2 결과만으로는 도착을 믿을 수 없다 (DOCK_GUARD_M 주석 참고).")
    else:
        dock_ok = do_dock(drv, here)
        if dock_ok:
            announce_done(drv)

    probe.report(WAYPOINTS, result)
    if dock_ok is None:
        print(f"[도킹] 실행 안 함  (마지막 경유지까지 {d_last:.2f} m)")
    else:
        print(f"[도킹] {'성공' if dock_ok else '실패'}")

    drv.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
