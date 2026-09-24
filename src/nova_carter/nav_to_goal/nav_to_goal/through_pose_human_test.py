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
DOCK_GUARD_M = 1.0  # 마지막 경유지에서 이 거리 안에 있을 때만 도킹한다.
                    # Nav2 결과(SUCCEEDED)만 믿으면 안 된다. 실측으로, 로봇이 Nav2 구간을
                    # 한 발짝도 못 갔는데 SUCCEEDED 가 나와 UNDOCK 종료 지점(-1.765, 3.425)에서
                    # 3m 를 돌진했고, 그 방향은 2.25m 앞이 벽이었다.
# do_dock() 의 횡오차 경고 임계값. **경고만 하고 주행은 막지 않는다** (판단 재료용).
# 부호 규약은 do_dock() 과 같다: + 가 서쪽(책상 쪽), - 가 동쪽(열린 쪽).
DOCK_LAT_WEST_MAX = 0.04  # 서쪽 = 책상 쪽 여유. do_dock() docstring 의 실측값.
                          # 교차검증: 도킹점 x -44.826 에서 책상 동쪽 면 x -45.391 까지 0.565 m,
                          # 차체 반폭 0.5 + footprint_padding 0.01 을 빼면 0.055 m.
                          # 둘 중 작은 쪽(실측 0.04)을 쓴다.
DOCK_LAT_EAST_MAX = 0.50  # 동쪽은 벽이 아니라 열린 공간이라 물리 한계가 아니다.
                          # 맵 실측으로 "축 좌우 0.5m, 4m 전 구간 자유"가 확인된 범위까지만
                          # 정상으로 본다. 그 밖이면 검증 안 된 자리라는 뜻이다.

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

# 도킹 축 위 정렬점. **마지막 구간을 축을 따라가는 직선으로 만드는 게 목적**이다.
# 없으면 로봇이 DOCK_APPROACH 에 엉뚱한 방위로 도착해 거기서 큰 제자리 회전을 해야 하는데,
# 이 로봇은 그게 불가능하다 (2026-09-23 실측):
#   base_link 회전 반경 0.155 m -> 90도 회전이 위치를 0.219 m 망친다.
#   general_goal_checker 의 xy 허용은 0.1 m 이므로 위치를 지킨 채 돌 수 있는 각도는 37.6도뿐.
#   실제로는 137도를 돌아야 했고, 결과는 87초 넘는 무한 좌우 회전이었다:
#     위치 0.009 m 일 때 yaw 오차 137도 / yaw 오차 2.1도 일 때 위치 0.189 m
#     -> goal checker 두 조건 동시 만족 0회 (2563 샘플)
# 여기서 미리 축 방위로 서고 직선으로 들어가면 최종 회전이 작아져 수렴한다.
#
# 7.0 인 이유 — 맵 실측 자유반경 (제자리 회전 필요 1.06 m):
#   +4m 2.60 / +5m 3.00 / +6m 3.00 / +7m 2.55 / +8m 1.55 / +9m 0.55
# +7m 은 회전 여유가 넉넉하면서 DOCK_APPROACH 까지 3 m 직선이 남는다.
# (GoalCritic.threshold_to_consider 3.3 과도 맞는다 — 여기서부터 목표 수렴이 켜진다)
ALIGN_BACK_M = 7.0
assert ALIGN_BACK_M > DOCK_BACK_M, "정렬점은 DOCK_APPROACH 보다 뒤에 있어야 한다"
DOCK_ALIGN = (*_axis_point(ALIGN_BACK_M), DOCK_POSE[2])

# upper_lane — 맵 실측상 y~16 띠가 East 방부터 West 방까지 완전히 뚫려 있어
#   (x -49~25 전 구간 free) 사실상 직선이다.
#   이력: 경유지가 DOCK_APPROACH 하나뿐이었다. 그때는 RemovePassedGoals 가 아무것도 안 지워서
#   (소스가 while (goal_poses.size() > 1)) radius / hz / prune 이 통과 판정에 무관했다.
#   2026-09-23 에 DOCK_ALIGN 을 앞에 넣으면서 **경유지가 2개가 됐다** — 이제 DOCK_ALIGN 은
#   radius 0.7 안으로 지나가야 지워진다. 축 위 직선이라 여유는 충분하다.
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
    "upper_lane": [DOCK_ALIGN, DOCK_APPROACH],
    "lower_lane": [
        (-10.968, -1.543, 145.0),
        (-30.906349182128906, 12.407171249389648, 161.8),
        DOCK_ALIGN,
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
    """마지막 경유지에서 도킹점까지 **직진만** 한다. Nav2 미사용, 회전 없음.

    최종 방향 정렬은 Nav2 가 끝낸 상태로 넘어온다 (general_goal_checker.yaw_goal_tolerance
    0.05 = 2.9도). 여기서 회전하지 않는 이유는 실측 때문이다 — 105도 제자리 회전 한 번에
    base_link 가 0.22 m 움직였다 (역산 회전 반경 0.138 m. base_link 가 구동륜 축에서
    14 cm 떨어져 있다). 도킹 통로의 서쪽 여유가 0.04 m 뿐이라 감당할 수 없는 양이다.
    **위치를 맞춘 뒤 회전하면 그 회전이 위치를 망친다.**

    전진 거리는 고정값이 아니라 현재 위치에서 도킹점까지의 **축 방향 투영**이다.
    Nav2 가 경유지에 얼마나 못 미치거나 지나쳤든 앞뒤 오차는 여기서 흡수된다.
    유클리드 거리를 쓰면 횡오차가 있을 때 그만큼 더 가서 책상 옆을 지나쳐 버린다.
    """
    gx, gy, gyaw_deg = DOCK_POSE
    gyaw = math.radians(gyaw_deg)
    x, y, yaw = here
    along = (gx - x) * math.cos(gyaw) + (gy - y) * math.sin(gyaw)     # 축 방향 투영
    lateral = -(gx - x) * math.sin(gyaw) + (gy - y) * math.cos(gyaw)  # + 서쪽(책상), - 동쪽
    drift = math.degrees(math.atan2(math.sin(gyaw - yaw), math.cos(gyaw - yaw)))
    print(f"\n[DOCK] Nav2 미사용, 직진만. 현재 ({x:.3f}, {y:.3f}) yaw {math.degrees(yaw):+.2f} deg")
    print(f"       도킹점 ({gx:.3f}, {gy:.3f}) yaw {gyaw_deg:+.1f} deg  "
          f"(축 대비 {drift:+.2f} deg)")
    print(f"       축 방향 {along:.3f} m 전진,  횡오차 {lateral:+.3f} m "
          f"({'서쪽/책상쪽' if lateral > 0 else '동쪽'}, 보정 안 함)")
    # 통과 가능 범위를 벗어나도 멈추지 않고 경고만 한다. 판단 재료를 남기는 게 목적이다.
    if lateral > DOCK_LAT_WEST_MAX or lateral < -DOCK_LAT_EAST_MAX:
        print(f"       ** 경고: 횡오차가 통과 범위(서 {DOCK_LAT_WEST_MAX:.2f} / "
              f"동 {DOCK_LAT_EAST_MAX:.2f} m, 맵 실측)를 벗어났다. 책상에 닿을 수 있다 **")
    return drv.run_forward(along, "DOCK")


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

    # [가드 해제] 미션 신호 대기 없이 바로 출발 허용
    # if "--solo" not in sys.argv:
    #     wait_for_mission(drv)

    print(f"\n[UNDOCK] cmd_vel 로 {UNDOCK_M:.1f} m 전진 (Nav2 미사용)")
    if not drv.run_forward(UNDOCK_M, "UNDOCK"):
        print("  undock 경고: 미도달 (중단하지 않고 Nav2 계속 진행)")

    # Probe 는 undock 이 끝난 뒤에 만든다. undock 중 대기가 "정지"로 잡히면 안 되고,
    # 계측 시각 t0 도 Nav2 구간 시작에 맞춘다.
    poses = [make_pose(nav, *w) for w in WAYPOINTS]
    probe = Probe(nav, WAYPOINTS)

    print(f"\n[NAV] {LANE} — goThroughPoses 경유지 {len(poses)}개, 한 번만 발행한다")
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

    # 도킹 전에 실제 위치와 Nav2 성공 여부를 확인한다 (마지막 경유지 도달 시에만 도킹 수행)
    dock_ok = None
    here = drv.wait_pose(timeout=3.0)
    d_last = math.dist(here[:2], WAYPOINTS[-1][:2]) if here else float("inf")
    if result != "SUCCEEDED":
        print(f"\n[DOCK] 건너뜀 — Nav2 주행 미완료 (결과: {result})")
    elif d_last > DOCK_GUARD_M:
        print(f"\n[DOCK] 건너뜀 — 마지막 경유지까지 {d_last:.2f} m (허용 반경 {DOCK_GUARD_M:.1f} m 이탈). "
              f"현재 위치: {'(%.3f, %.3f)' % here[:2] if here else '알 수 없음'}")
    elif here:
        dock_ok = do_dock(drv, here)
        if dock_ok:
            announce_done(drv)
    else:
        print("\n[DOCK] 건너뜀 — 현재 위치(TF)를 읽을 수 없습니다.")

    probe.report(WAYPOINTS, result)
    if dock_ok is None:
        print(f"[도킹] 실행 안 함  (마지막 경유지까지 {d_last:.2f} m)")
    else:
        print(f"[도킹] {'성공' if dock_ok else '실패'}")

    drv.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
