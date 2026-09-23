#!/usr/bin/env python3
"""WHCA* 다중로봇 경로 조정 시뮬레이터.

설계 문서: "다중로봇 경로 계획 (WHCA) 설계 정리.md"
계획 로직(space_time_astar / plan_step / assign_goal)은 tkinter 무의존 순수 함수다.
추후 ROS 2 coordinator 노드가 이 함수들을 그대로 import 한다. test_script/INTEGRATION.md 참조.

    python3 test_script/lane_sim.py            # GUI
    python3 test_script/lane_sim.py --check    # 헤드리스 검증 (1열/2열/3열 + 실패주입 + K 비교)
"""

import argparse
import heapq
import random
from collections import deque
from dataclasses import dataclass, field

# ---------------------------------------------------------------- 상수

FREE, WALL, OBST, CORR = 0, 1, 2, 3

W_COLS, H_ROWS = 40, 16
SPLIT_A, SPLIT_B = 12, 28                      # 벽 구간 [SPLIT_A, SPLIT_B)
CORRIDOR_ROWS = {1: [3, 4], 2: [10, 11, 12]}   # {corridor_id: [row, ...]}
N_OBSTACLES = 6                                # 넓은 공간 A, B 각각

WINDOW = 8
K_GAP = 1   # 1 = 조건 3(vertex)과 동일 = 사실상 끔. 설계 문서 기본값은 2.
            # --check 가 K=1/K=2 를 비교 출력한다. 근거는 INTEGRATION.md "조건 5" 절.

# 8방향. 인덱스 차이가 그대로 45도 단위 회전량이 되도록 반시계 순서로 둔다.
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
DIR_IDX = {d: i for i, d in enumerate(DIRS)}

# ponytail: 회전 비용은 실제 로봇의 (제자리 회전 속도 / 직진 속도) 비율로 재보정해야 한다.
# 아래 값은 max_vel_theta 0.5 rad/s, max_vel_x 0.5 m/s, 셀 2.2m 기준의 어림값.
BASE_COST = (1.0, 1.4)                    # (직교 이동, 대각 이동)
TURN_COST = (0.0, 0.4, 1.0, 1.5, 2.0)     # 회전량 0 / 45 / 90 / 135 / 180 도
WAIT_COST = 1.0

MAX_WAIT = 30    # 이 스텝 수만큼 연속 대기하면 교착으로 판정


def _prio(r):
    """계획 순서 키. stuck 인 로봇을 맨 앞으로 보낸다 (PIBT 의 우선순위 상속).

    설계 문서 3.1 은 (-urgency, id) 고정이다. 그러면 1열 복도에서 교착한다
    (실측: 시드 10개 중 6개). 원인은 우선순위 자체가 아니라 "미루기"다:

      R1(u2, 목표 오른쪽) - R3(u1, 목표 오른쪽) - R2(u3, 목표 왼쪽) 이 1열 복도에 있을 때
      R2 가 먼저 계획해 t=7 에 R1 의 칸까지 예약한다.
      R1 은 "t=7 까지만 비우면 된다"는 계획을 세우고 t=1..6 은 제자리를 예약한다.
      R3 는 왼쪽(R1 의 칸)이 t=0..7 예약돼 있어 후퇴 불가, 오른쪽은 R2 -> 경로 없음(stuck).
      그런데 실행은 plan[1] 한 칸뿐이라 R1 은 영원히 t=7 에 도달하지 못한다. 영구 교착.

    연속 대기 시간으로 긴급도를 올리는 방식은 듣지 않는다. 끼인 세 대가 같이 대기하므로
    보정이 균등하게 올라가 상대 순서가 그대로다 (실측: 시드 3, t=180~189).

    stuck 인 로봇을 맨 앞으로 보내면 그 로봇이 빈 예약 테이블에서 먼저 방향을 잡고,
    막고 있던 로봇이 그 예약에 밀려 t=1 에 실제로 비켜난다.
    stuck 은 plan_step 이 매번 갱신하므로 풀리는 즉시 원래 우선순위로 돌아온다.
    """
    return (0 if r.stuck else 1, -r.urgency, r.id)


def turn_amount(a, b):
    """방향 인덱스 a -> b 의 45도 단위 회전량 (0~4)."""
    return min((a - b) % 8, (b - a) % 8)


# ---------------------------------------------------------------- 맵

def make_map(seed=0, corridor_rows=None):
    """(grid, corr_of) 반환. corr_of[(x,y)] = (corridor_id, row_index)."""
    rows = corridor_rows if corridor_rows is not None else CORRIDOR_ROWS
    for attempt in range(50):
        rnd = random.Random(seed + attempt)
        grid = [[FREE] * W_COLS for _ in range(H_ROWS)]
        for y in range(H_ROWS):
            for x in range(SPLIT_A, SPLIT_B):
                grid[y][x] = WALL

        corr_of = {}
        for cid, rs in rows.items():
            for i, y in enumerate(rs):
                for x in range(SPLIT_A, SPLIT_B):
                    grid[y][x] = CORR
                    corr_of[(x, y)] = (cid, i)

        # 복도 입구 앞 칸은 장애물 금지. 막으면 복도가 막다른 길이 되고,
        # 그 안에서 마주친 두 대는 알고리즘이 아니라 맵 때문에 진짜 교착한다.
        # (실측: seed 7 에서 row 3 복도의 B 쪽 입구가 막혀 정면 시나리오가 영구 교착)
        mouths = {(x, y) for rs in rows.values() for y in rs
                  for x in (SPLIT_A - 1, SPLIT_A - 2, SPLIT_B, SPLIT_B + 1)}
        for lo, hi in ((0, SPLIT_A), (SPLIT_B, W_COLS)):
            placed = 0
            for _ in range(10000):
                if placed >= N_OBSTACLES:
                    break
                x, y = rnd.randrange(lo, hi), rnd.randrange(H_ROWS)
                if grid[y][x] == FREE and (x, y) not in mouths:
                    grid[y][x] = OBST
                    placed += 1

        # 복도 2 가운데 열에 고정 장애물 1개 (열 변경 유도). 1열 복도면 막히므로 건너뛴다.
        rs2 = rows.get(2, [])
        if len(rs2) >= 2:
            mx, my = (SPLIT_A + SPLIT_B) // 2, rs2[len(rs2) // 2]
            grid[my][mx] = OBST
            corr_of.pop((mx, my), None)

        if _connected(grid):
            return grid, corr_of
    raise RuntimeError("연결된 맵 생성 실패")


def passable(grid, x, y):
    return 0 <= x < W_COLS and 0 <= y < H_ROWS and grid[y][x] in (FREE, CORR)


def _connected(grid):
    """넓은 공간 A -> B 4방향 연결 확인 (보수적: 4연결이면 8연결도 성립)."""
    start = next(((x, y) for y in range(H_ROWS) for x in range(SPLIT_A)
                  if grid[y][x] == FREE), None)
    if start is None:
        return False
    seen, q = {start}, [start]
    while q:
        x, y = q.pop()
        if x >= SPLIT_B:
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (x + dx, y + dy)
            if n not in seen and passable(grid, *n):
                seen.add(n)
                q.append(n)
    return False


# ---------------------------------------------------------------- 로봇 / 예약 테이블

@dataclass
class Robot:
    id: int
    urgency: int
    pos: tuple
    goal: tuple = None
    d: int = 0                                      # 현재 heading (DIRS 인덱스)
    plan: list = field(default_factory=list)        # [(x, y, d), ...] 길이 W+1
    wait_cnt: int = 0
    stuck: bool = False
    backing: bool = False
    displaced: bool = False                         # 다른 로봇과 같은 셀에 겹쳐 있음
    arrivals: int = 0
    t_start: int = 0
    steps_to_goal: list = field(default_factory=list)
    turns180: int = 0
    hfield: dict = None          # dist_field 캐시
    hgoal: tuple = None


class Table:
    """{t: {cell: robot_id}}. 먼저 쓴 쪽(= 우선순위 높은 쪽)이 소유한다."""

    def __init__(self):
        self.t = {}

    def get(self, tt, cell):
        return self.t.get(tt, {}).get(cell)

    def reserve(self, cell, tt, rid):
        self.t.setdefault(tt, {}).setdefault(cell, rid)

    def release(self, rid):
        for d in self.t.values():
            for c in [c for c, v in d.items() if v == rid]:
                del d[c]


# ---------------------------------------------------------------- 시공간 A*

def _ok(x, y, nx, ny, t, r, table, grid, corr_of, K, dirs_by_id):
    """전이 (x,y,t) -> (nx,ny,t+1) 허용 여부. 설계 문서 3.3 금지 조건 1~5."""
    if not passable(grid, nx, ny):                                   # 1
        return False

    owner = table.get(t + 1, (nx, ny))                               # 3. vertex
    if owner is not None and owner != r.id:
        return False

    prev = table.get(t, (nx, ny))                                    # 4. edge/swap
    if prev is not None and prev != r.id and table.get(t + 1, (x, y)) == prev:
        return False

    dx, dy = nx - x, ny - y

    if dx and dy:                                                    # 2. 모서리 끊기 금지
        for cx, cy in ((x + dx, y), (x, y + dy)):
            if not passable(grid, cx, cy):
                return False
            oc = table.get(t + 1, (cx, cy))                          # 타 로봇 점유 칸에도 적용
            if oc is not None and oc != r.id:
                return False

    if K > 1 and dx and (nx, ny) in corr_of:                         # 5. 복도 간격 K
        step = 1 if dx > 0 else -1
        cid, row = corr_of[(nx, ny)]
        for i in range(1, K + 1):
            cell = (nx + step * i, ny)
            if corr_of.get(cell) != (cid, row):
                break
            oid = table.get(t + 1, cell)
            if oid is None or oid == r.id:
                continue
            od = dirs_by_id.get(oid)
            # 같은 방향으로 가는 로봇에만 적용. 마주 오는 로봇에 걸면 3.5 후진 회피를 막는다.
            if od is not None and DIRS[od][0] * step > 0:
                return False
    return True


def dist_field(grid, goal):
    """goal 까지의 8방향 BFS 거리표.

    설계 문서 3.3 은 휴리스틱으로 체비셰프 거리를 쓰라고 했으나 그것으로는 교착한다.
    복도처럼 벽을 우회해야 하는 구간에서 체비셰프는 전진해도 값이 줄지 않아
    "대기"와 "전진"의 f 가 같아지고, 윈도우 절단(t==W) 때문에 대기가 안정 고정점이 된다.
    (실측: 1열 복도에서 로봇이 이유 없이 영구 정지)
    벽을 우회한 실제 거리를 쓰면 전진할 때 f 가 반드시 줄어 이 고정점이 사라진다.
    이동 최소 비용이 1.0 이므로 admissible 하다. HCA* 원논문의 RRA* 와 같은 역할.
    """
    dist, q = {goal: 0}, deque([goal])
    while q:
        x, y = q.popleft()
        for dx, dy in DIRS:
            n = (x + dx, y + dy)
            if n not in dist and passable(grid, n[0], n[1]):
                dist[n] = dist[(x, y)] + 1
                q.append(n)
    return dist


UNREACHABLE = float(W_COLS * H_ROWS)


def _pad(path, W):
    while len(path) < W + 1:
        path.append(path[-1])
    return path


def space_time_astar(r, table, grid, corr_of, W=WINDOW, K=K_GAP, dirs_by_id=None):
    """로봇 1대의 윈도우 내 계획. 경로 없으면 None.

    상태는 (x, y, heading, t). heading 을 상태에 넣어야 회전 비용을 셀 수 있고,
    그래야 실제 로봇에서 비싼 지그재그/180도 기동이 계획 단계에서 억제된다.
    """
    dirs_by_id = dirs_by_id or {}
    gx, gy = r.goal
    if r.hgoal != r.goal:                      # 목표가 바뀌었을 때만 거리표 재계산
        r.hfield, r.hgoal = dist_field(grid, r.goal), r.goal
    hf = r.hfield
    start = (r.pos[0], r.pos[1], r.d, 0)
    g_score = {start: 0.0}
    came = {start: None}
    openq = [(hf.get(r.pos, UNREACHABLE), 0.0, start)]
    best, best_key = None, (float("inf"), float("inf"))

    while openq:
        f, g, s = heapq.heappop(openq)
        if g > g_score.get(s, float("inf")):
            continue
        x, y, d, t = s

        if (x, y) == (gx, gy):
            return _pad(_reconstruct(came, s), W)
        if t == W:
            key = (f, hf.get((x, y), UNREACHABLE))   # 동점이면 목표에 더 가까운 쪽
            if key < best_key:
                best_key, best = key, s
            continue

        for j, (dx, dy) in enumerate(DIRS):
            nx, ny = x + dx, y + dy
            if not _ok(x, y, nx, ny, t, r, table, grid, corr_of, K, dirs_by_id):
                continue
            cost = BASE_COST[1 if dx and dy else 0] + TURN_COST[turn_amount(d, j)]
            _push(openq, g_score, came, s, (nx, ny, j, t + 1), g + cost, hf)

        if _ok(x, y, x, y, t, r, table, grid, corr_of, K, dirs_by_id):   # 제자리 대기
            _push(openq, g_score, came, s, (x, y, d, t + 1), g + WAIT_COST, hf)

    return _pad(_reconstruct(came, best), W) if best else None


def _push(openq, g_score, came, parent, ns, ng, hf):
    if ng < g_score.get(ns, float("inf")):
        g_score[ns] = ng
        came[ns] = parent
        heapq.heappush(openq, (ng + hf.get((ns[0], ns[1]), UNREACHABLE), ng, ns))


def _reconstruct(came, s):
    out = []
    while s is not None:
        out.append((s[0], s[1], s[2]))
        s = came.get(s)
    return out[::-1]


# ---------------------------------------------------------------- 한 스텝

def plan_step(robots, grid, corr_of, W=WINDOW, K=K_GAP):
    """우선순위 순 시공간 A*. robots 의 plan / stuck / displaced 를 채운다.

    robots 의 pos 는 "직전 스텝이 만들어낸 위치"가 아니어도 된다.
    실제 연동에서는 amcl_pose 를 셀로 스냅한 값이 들어오며, 주행 실패 시
    두 로봇이 같은 셀로 스냅될 수 있다. 그 경우 긴급도 높은 쪽이 셀을 소유하고
    낮은 쪽은 t=0 예약 없이 탈출 경로를 찾는다.
    """
    order = sorted(robots, key=_prio)
    dirs_by_id = {r.id: r.d for r in robots}
    table = Table()

    # 설계 문서 3.4 는 모든 로봇의 현재 위치를 t=0 과 t=1 에 예약하라고 했으나, t=1 예약은
    # 라이브락을 만든다: 하위 로봇 L 은 t=1 에 제자리 대기가 항상 합법이므로 후퇴를 t=2 로
    # 미루는 쪽이 f 가 더 낮고, 상위 로봇 H 는 L 의 t=1 예약에 막혀 역시 대기한다.
    # 실행은 plan[1](= t=1) 만 하므로 둘 다 영원히 안 움직인다. (실측: 1열 복도 t=167)
    # t=0 만 예약하면 H 가 t=1 에 L 의 칸을 예약 -> L 이 t=1 에 강제로 비켜난다.
    # barrier 동기에서는 전 로봇이 한 스텝을 동시에 실행하므로 이 "따라 들어가기"는 정상이다.
    for r in order:                       # 우선순위 순 예약 -> 중복 셀은 상위 로봇이 소유
        table.reserve(r.pos, 0, r.id)

    for r in order:
        table.release(r.id)
        table.reserve(r.pos, 0, r.id)
        r.displaced = table.get(0, r.pos) != r.id
        path = space_time_astar(r, table, grid, corr_of, W, K, dirs_by_id)
        r.stuck = path is None
        if path is None:
            path = [(r.pos[0], r.pos[1], r.d)] * (W + 1)
        for t, (x, y, _) in enumerate(path):
            table.reserve((x, y), t, r.id)
        r.plan = path
    return order


def execute(robots, grid, rnd=None, fail_rate=0.0):
    """각 로봇을 plan[1] 로 이동. 반환값은 계획대로 이동했을 때의 위치(검증용)."""
    order = sorted(robots, key=_prio)
    nxt = {r.id: (r.plan[1] if len(r.plan) > 1 else (r.pos[0], r.pos[1], r.d)) for r in robots}

    # 안전망: 실행 직전 vertex/swap 재검사. 정상 동작 시 거의 발동하지 않는다.
    # 단순히 "우선순위 낮은 쪽이 대기"로는 부족하다. 하위 로봇이 stuck 이라 제자리인데
    # 상위 로봇이 그 칸으로 들어오면 둘 다 같은 칸에 놓이고, 다음 스텝에 서로를 통과한다.
    # (실측: 1열 복도 정면, t=9) 그래서 "이미 그 칸에 서 있는 로봇이 우선"으로 판정하고,
    # 되돌림이 뒤 로봇에 연쇄되므로 고정점까지 반복한다.
    pos = {r.id: r.pos for r in robots}
    final = {r.id: nxt[r.id][:2] for r in robots}
    for _ in range(len(robots) + 1):
        conflict = False
        occ = {}
        for r in order:
            occ.setdefault(final[r.id], []).append(r.id)
        for cell, ids in occ.items():
            if len(ids) > 1:
                keep = next((i for i in ids if pos[i] == cell), ids[0])
                for rid in ids:
                    if rid != keep and final[rid] != pos[rid]:
                        final[rid], conflict = pos[rid], True
        for a in order:
            for b in order:
                if a.id != b.id and final[a.id] == pos[b.id] and final[b.id] == pos[a.id] \
                        and final[b.id] != pos[b.id]:
                    final[b.id], conflict = pos[b.id], True
        if not conflict:
            break
    for r in robots:
        if final[r.id] != nxt[r.id][:2]:
            nxt[r.id] = (final[r.id][0], final[r.id][1], r.d)

    planned = {r.id: nxt[r.id][:2] for r in robots}

    for r in robots:
        tx, ty = nxt[r.id][:2]
        if fail_rate and rnd and rnd.random() < fail_rate:
            # Nav2 주행 실패 모사: 대부분 제자리, 일부는 엉뚱한 인접 셀로 재스냅(드리프트).
            if rnd.random() < 0.3:
                cand = [(r.pos[0] + dx, r.pos[1] + dy) for dx, dy in DIRS
                        if passable(grid, r.pos[0] + dx, r.pos[1] + dy)]
                tx, ty = rnd.choice(cand) if cand else r.pos
            else:
                tx, ty = r.pos

        delta = (tx - r.pos[0], ty - r.pos[1])
        if delta == (0, 0):
            r.wait_cnt += 1
            r.backing = False
        else:
            nd = DIR_IDX.get(delta, r.d)
            r.backing = turn_amount(r.d, nd) == 4
            if r.backing:
                r.turns180 += 1
            r.d = nd
            r.wait_cnt = 0
        r.pos = (tx, ty)
    return planned


def assign_goal(r, grid, robots, rnd):
    """반대편 넓은 공간의 랜덤 FREE 셀. 타 로봇의 목표/현재 위치는 제외."""
    far = list(range(SPLIT_B, W_COLS)) if r.pos[0] < (SPLIT_A + SPLIT_B) // 2 \
        else list(range(0, SPLIT_A))
    banned = {o.goal for o in robots if o is not r} | {o.pos for o in robots if o is not r}
    for _ in range(2000):
        c = (rnd.choice(far), rnd.randrange(H_ROWS))
        if grid[c[1]][c[0]] == FREE and c not in banned:
            r.goal = c
            return True
    return False


def make_robots(grid, rnd, specs=((1, 2), (2, 3), (3, 1))):
    """기본 3대. 번호 순과 긴급도 순을 다르게 배치해 규칙이 눈에 보이게 한다."""
    robots, used = [], set()
    for rid, urg in specs:
        while True:
            c = (rnd.randrange(0, SPLIT_A), rnd.randrange(H_ROWS))
            if grid[c[1]][c[0]] == FREE and c not in used:
                used.add(c)
                break
        robots.append(Robot(id=rid, urgency=urg, pos=c))
    for r in robots:
        assign_goal(r, grid, robots, rnd)
    return robots


def step_world(robots, grid, corr_of, t, rnd, W=WINDOW, K=K_GAP, fail_rate=0.0):
    """도착 처리 -> 계획 -> 실행. (planned_positions, arrived_ids) 반환."""
    arrived = []
    for r in robots:
        if r.pos == r.goal:
            r.arrivals += 1
            r.steps_to_goal.append(t - r.t_start)
            r.t_start = t
            arrived.append(r.id)
            assign_goal(r, grid, robots, rnd)
    plan_step(robots, grid, corr_of, W, K)
    planned = execute(robots, grid, rnd, fail_rate)
    return planned, arrived


# ---------------------------------------------------------------- 검증

CASES = {
    "1열":        {1: [3], 2: [11]},
    "기본(2+3열)": CORRIDOR_ROWS,
    "3열":        {1: [3, 4, 5], 2: [10, 11, 12]},
}


def run_check(steps=500, corridor_rows=None, K=K_GAP, fail_rate=0.0, seed=1, label=""):
    grid, corr_of = make_map(seed, corridor_rows)
    rnd = random.Random(seed)
    robots = make_robots(grid, rnd)

    overlap_since, gap_viol = {}, 0
    prev = {r.id: r.pos for r in robots}

    for t in range(steps):
        planned, _ = step_world(robots, grid, corr_of, t, rnd, WINDOW, K, fail_rate)

        # 계획된 이동 자체는 항상 무충돌이어야 한다 (실패 주입과 무관).
        cells = list(planned.values())
        assert len(set(cells)) == len(cells), f"[{label}] t={t} 계획 vertex 충돌: {planned}"
        for a in robots:
            for b in robots:
                if a.id < b.id and planned[a.id] == prev[b.id] and planned[b.id] == prev[a.id]:
                    assert False, f"[{label}] t={t} 계획 swap 충돌: {a.id}<->{b.id}"

        for r in robots:
            assert grid[r.pos[1]][r.pos[0]] in (FREE, CORR), \
                f"[{label}] t={t} 로봇 {r.id} 가 벽/장애물 위: {r.pos}"
            assert r.wait_cnt < MAX_WAIT, (
                f"[{label}] t={t} 로봇 {r.id} 가 {MAX_WAIT}스텝 연속 대기 (교착 의심). "
                f"위치={[(o.id, o.pos, o.urgency, o.stuck) for o in robots]}")

        # 실제 위치 겹침(드리프트 결과)은 허용하되 3스텝 안에 풀려야 한다.
        occ = {}
        for r in robots:
            occ.setdefault(r.pos, []).append(r.id)
        now = {tuple(sorted(ids)): cell for cell, ids in occ.items() if len(ids) > 1}
        overlap_since = {k: v for k, v in overlap_since.items() if k in now}
        for key, cell in now.items():
            overlap_since.setdefault(key, t)
            assert t - overlap_since[key] <= 3, \
                f"[{label}] t={t} 로봇 {list(key)} 가 {cell} 에서 3스텝 넘게 겹침 (복구 실패)"

        gap_viol += _count_gap_violations(robots, corr_of, K)

        prev = {r.id: r.pos for r in robots}

    for r in robots:
        assert r.arrivals >= 1, f"[{label}] 로봇 {r.id} 가 한 번도 목표에 도착하지 못함"

    avg = {r.id: sum(r.steps_to_goal) / len(r.steps_to_goal) for r in robots if r.steps_to_goal}
    hi = max(robots, key=lambda r: r.urgency)
    lo = min(robots, key=lambda r: r.urgency)
    if len(hi.steps_to_goal) >= 2 and len(lo.steps_to_goal) >= 2:
        assert avg[hi.id] <= avg[lo.id] * 1.25, \
            f"[{label}] 긴급도가 도착 시간에 반영되지 않음: u{hi.urgency}={avg[hi.id]:.1f} " \
            f"vs u{lo.urgency}={avg[lo.id]:.1f}"

    turns = sum(r.turns180 for r in robots) / (steps * len(robots))
    assert turns <= 0.1, f"[{label}] 180도 전환이 과다: 스텝·로봇당 {turns:.3f} (회전 비용 확인)"

    return {
        "도착": {r.id: r.arrivals for r in robots},
        "간격K위반": gap_viol,
        "평균도착스텝": {k: round(v, 1) for k, v in avg.items()},
        "180도/스텝": round(turns, 3),
    }


def _count_gap_violations(robots, corr_of, K):
    """복도 안에서 같은 방향 앞 로봇과의 거리가 K 미만인 쌍의 수.

    조건 5 는 전이 제약일 뿐 불변식이 아니다. 앞 로봇이 방향을 틀거나 멈추면
    간격 K 는 깨질 수 있다. 그래서 assert 가 아니라 지표로만 센다.
    """
    n = 0
    if K <= 1:
        return 0
    for a in robots:
        if a.pos not in corr_of:
            continue
        step = DIRS[a.d][0]
        if step == 0:
            continue
        for b in robots:
            if b is a or corr_of.get(b.pos) != corr_of.get(a.pos):
                continue
            gap = (b.pos[0] - a.pos[0]) * step
            if 0 < gap < K and DIRS[b.d][0] * step > 0:
                n += 1
    return n


def headon_steps(K, rows={1: [3], 2: [11]}, limit=200):
    """1열 복도 양끝에서 마주 보는 2대가 서로 반대편에 도달할 때까지의 스텝 수."""
    grid, corr_of = make_map(7, rows)
    y = rows[1][0]

    def free_near(x0, y0):
        for d in range(H_ROWS):
            for yy in (y0 - d, y0 + d):
                if passable(grid, x0, yy) and grid[yy][x0] == FREE:
                    return (x0, yy)
        raise RuntimeError("자유 셀 없음")

    # hi 는 왼쪽에서 오른쪽으로, lo 는 오른쪽에서 왼쪽으로. 복도에서 정면으로 만난다.
    hi = Robot(id=1, urgency=3, pos=(SPLIT_A + 2, y), d=0, goal=free_near(W_COLS - 2, y))
    lo = Robot(id=2, urgency=1, pos=(SPLIT_B - 3, y), d=4, goal=free_near(1, y))
    robots = [hi, lo]
    rnd = random.Random(0)
    for t in range(limit):
        plan_step(robots, grid, corr_of, WINDOW, K)
        execute(robots, grid, rnd, 0.0)
        if all(r.pos == r.goal for r in robots):
            return t + 1
    return None


def main_check(steps, seeds):
    """시드 여러 개를 쓸어야 한다. 단일 시드는 교착을 놓친다.

    실측: 시드 1 만으로는 1열 케이스가 통과하지만, 시드 10개로 넓히면 6개에서
    영구 교착이 나왔다 (우선순위 미루기 문제, _prio 참조).
    """
    def sweep(rows, K, fr, label):
        out = [run_check(steps, rows, K, fr, seed=sd, label=f"{label}/s{sd}")
               for sd in range(1, seeds + 1)]
        return ("시드 %d개 통과 | 총 도착 %3d | 180도/스텝 %.3f | 간격K위반 %d" % (
            seeds,
            sum(sum(r["도착"].values()) for r in out),
            sum(r["180도/스텝"] for r in out) / len(out),
            sum(r["간격K위반"] for r in out)))

    print(f"=== 기본 검증 (K={K_GAP}, fail_rate=0, {steps}스텝, 시드 1~{seeds}) ===")
    for name, rows in CASES.items():
        print(f"  {name:12s} {sweep(rows, K_GAP, 0.0, name)}")

    print("\n=== 실패 주입 (Nav2 주행 실패 모사) — 연동 가능 여부 판정 항목 ===")
    for rate in (0.1, 0.25):
        for name, rows in CASES.items():
            print(f"  fail={rate:<5} {name:12s} {sweep(rows, K_GAP, rate, f'{name}/f{rate}')}")

    print("\n=== 조건 5 (복도 간격 K) 비교 — assert 아님, 기본값 결정 근거 ===")
    for K in (1, 2, 3):
        print(f"  K={K}  1열 정면 해소 {str(headon_steps(K)):>5} 스텝 | "
              f"{sweep(CASES['1열'], K, 0.0, f'K{K}')}")

    print("\n모든 검증 통과")


# ---------------------------------------------------------------- GUI

COLORS = {1: "#d62728", 2: "#1f77b4", 3: "#2ca02c"}
CELL = 26


def run_gui(seed, K, fail_rate):
    import tkinter as tk

    grid, corr_of = make_map(seed)
    rnd = random.Random(seed)
    robots = make_robots(grid, rnd)

    root = tk.Tk()
    root.title("WHCA* lane_sim")
    cv = tk.Canvas(root, width=W_COLS * CELL, height=H_ROWS * CELL, bg="white")
    cv.pack()
    bar = tk.Frame(root)
    bar.pack(fill="x", pady=4)

    state = {"t": 0, "play": False}
    v_speed = tk.IntVar(value=200)
    v_k = tk.IntVar(value=K)
    v_fail = tk.DoubleVar(value=fail_rate)
    v_urg = {r.id: tk.IntVar(value=r.urgency) for r in robots}
    lbl = tk.Label(bar, text="step 0")

    def draw():
        cv.delete("all")
        for y in range(H_ROWS):
            for x in range(W_COLS):
                c = {WALL: "#555555", OBST: "#000000", CORR: "#eef3dd"}.get(grid[y][x])
                if c:
                    cv.create_rectangle(x * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL,
                                        fill=c, outline="#dddddd")
        for r in robots:
            col = COLORS.get(r.id, "#888888")
            if r.plan:
                pts = []
                for x, y, _ in r.plan:
                    pts += [x * CELL + CELL / 2, y * CELL + CELL / 2]
                if len(pts) >= 4:
                    cv.create_line(*pts, fill=col, width=2)
            gx, gy = r.goal
            cv.create_text(gx * CELL + CELL / 2, gy * CELL + CELL / 2,
                           text="×", fill=col, font=("", 16, "bold"))
            x, y = r.pos
            outline = ("#999999" if r.stuck else
                       "#ff0000" if r.backing else
                       "#ffcc00" if r.wait_cnt else col)
            cv.create_oval(x * CELL + 3, y * CELL + 3, (x + 1) * CELL - 3, (y + 1) * CELL - 3,
                           fill=col, outline=outline, width=3)
            dx, dy = DIRS[r.d]
            cx, cy = x * CELL + CELL / 2, y * CELL + CELL / 2
            cv.create_line(cx, cy, cx + dx * CELL * 0.45, cy + dy * CELL * 0.45,
                           fill="white", width=3, arrow="last")
            cv.create_text(cx, cy - CELL, text=f"{r.id} u{r.urgency}", fill=col,
                           font=("", 9, "bold"))
        lbl.config(text=f"step {state['t']}   " +
                        "  ".join(f"{r.id}:{r.arrivals}회" for r in robots))

    def step():
        for r in robots:
            r.urgency = v_urg[r.id].get()
        step_world(robots, grid, corr_of, state["t"], rnd,
                   WINDOW, v_k.get(), v_fail.get())
        state["t"] += 1
        draw()

    def loop():
        if state["play"]:
            step()
            root.after(v_speed.get(), loop)

    def toggle():
        state["play"] = not state["play"]
        btn_play.config(text="Pause" if state["play"] else "Play")
        loop()

    tk.Button(bar, text="Step", command=step).pack(side="left", padx=3)
    btn_play = tk.Button(bar, text="Play", command=toggle)
    btn_play.pack(side="left", padx=3)
    tk.Scale(bar, from_=800, to=30, orient="horizontal", variable=v_speed,
             label="ms/step", length=140).pack(side="left", padx=6)
    for r in robots:
        tk.Label(bar, text=f"  로봇{r.id} 긴급도").pack(side="left")
        tk.Spinbox(bar, from_=1, to=3, width=3, textvariable=v_urg[r.id]).pack(side="left")
    tk.Label(bar, text="  K").pack(side="left")
    tk.Spinbox(bar, from_=1, to=4, width=3, textvariable=v_k).pack(side="left")
    tk.Label(bar, text="  fail").pack(side="left")
    tk.Spinbox(bar, from_=0.0, to=0.5, increment=0.05, width=5,
               textvariable=v_fail).pack(side="left")
    lbl.pack(side="left", padx=10)

    plan_step(robots, grid, corr_of, WINDOW, v_k.get())
    draw()
    root.mainloop()


# ---------------------------------------------------------------- main

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="WHCA* 다중로봇 경로 조정 시뮬레이터")
    ap.add_argument("--check", action="store_true", help="헤드리스 검증")
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--seeds", type=int, default=5, help="--check 가 쓸 시드 개수")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--k", type=int, default=K_GAP, help="복도 간격 K (1 = 사실상 끔)")
    ap.add_argument("--fail-rate", type=float, default=0.0,
                    help="스텝당 주행 실패 확률 (Nav2 실패 모사)")
    a = ap.parse_args()
    if a.check:
        main_check(a.steps, a.seeds)
    else:
        run_gui(a.seed, a.k, a.fail_rate)
