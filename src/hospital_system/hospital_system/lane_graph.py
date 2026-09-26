"""차선 그래프 · 구역(zone) · 구역 예약 — 철도 폐색 방식 (docs/SYSTEM_INTEGRATION_PLAN.md §3.2).

순수 파이썬(기하는 nav_to_goal.hospital_mission 의 차선 정의를 그대로 쓴다)이라 로봇 없이 시험할 수 있다.

그래프 (nav_to_goal.hospital_lanes)
    노드 = 책상 도킹 자세·정류장, 문 바깥 분기점 east_junction / west_junction.
    방 간선(한 방향): collection_out / collection_in / analysis_out / analysis_in / *_dock_in.
    복도 간선(양방향): upper, upper_reserve, lower, lower_reserve — 방향마다 간선 하나("upper>E", "upper>W")지만
    구역은 물리적으로 하나다(phys id "upper:k"). 같은 복도에 반대 방향 로봇이 동시에 있을 수 없다(방향 잠금).
    routes() 가 가능한 경로를 짧은 순으로, choose_route() 가 긴급도 순으로 복도를 배정한다.
구역
    간선을 ZONE_M 이하 길이로 고르게 나눈다 (도킹 직선 2.5 m 는 1구역).
    서로 다른 차선의 구역끼리 중심선이 CONFLICT_M 안으로 가까우면 '충돌' — 동시에 한 로봇만.
    두 문(x≈11, x≈-35.9)은 방 출입 두 방향이 같은 줄(y=12.5)이고 분기점에서 복도 4개가 갈라진다 → 충돌 구간.
예약 (tick 마다)
    1. 로봇 차체(뒤 1.38 m, 앞 0.48 m + 여유)가 걸친 구역은 그 로봇 것, 뒤로 벗어난 구역은 푼다.
    2. 우선순위 순서로 앞 구역을 LOOKAHEAD_M 까지 늘린다. 한 구역은
       - 다른 로봇이 잡고 있지 않고, 충돌 구역도 다른 로봇이 잡고 있지 않고,
       - 그 다음 구역도 다른 로봇 것이 아니어야(차간 1구역) 준다.
       충돌 구역 연속 구간은 빠져나간 다음 구역까지 한꺼번에 주거나 안 준다 — 문 안에서 서지 않는다.
    3. 정지점 = 받은 마지막 구역 끝 - (앞 길이 + 여유). 목적지 도킹 구역까지 받았으면 정지점 없음.
    이미 준 구역은 뺏지 않는다(교착 방지 규칙이 우선순위보다 앞선다).
"""
import heapq
import math
from dataclasses import dataclass, field

ZONE_M = 3.0          # 구역 최대 길이 (footprint 1.86 m + 제동)
CONFLICT_M = 1.6      # 다른 차선 구역 중심선이 이보다 가까우면 동시 점유 불가 (차폭 1.0 + 0.6)
NEIGHBOR_M = 6.0      # 같은 차선에서 이 거리 안의 앞뒤 구역은 충돌로 보지 않는다 (차간은 따로 지킨다)
FRONT_M = 0.48        # base_link 앞 차체 (hospital_avoidance.SafetySettings)
REAR_M = 1.38         # base_link 뒤 차체 (랙·팔 포함)
BODY_MARGIN_M = 0.3
LOOKAHEAD_M = 6.0     # 차체 앞에서 이만큼은 미리 예약한다
SAMPLE_M = 0.25


def _sample(segments, step=SAMPLE_M):
    """hospital_mission 경로 원소(line/arc) -> [(x, y, yaw)] (간격 step 이하, 끝점 포함)"""
    pts = []
    for seg in segments:
        if seg[0] == "line":
            (x0, y0), (x1, y1) = seg[1], seg[2]
            n = max(1, math.ceil(math.dist((x0, y0), (x1, y1)) / step))
            yaw = math.atan2(y1 - y0, x1 - x0)
            new = [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, yaw) for i in range(n + 1)]
        else:
            (cx, cy), r, a0, a1 = seg[1], seg[2], math.radians(seg[3]), math.radians(seg[4])
            n = max(1, math.ceil(abs(a1 - a0) * r / step))
            turn = math.pi / 2 if a1 > a0 else -math.pi / 2
            new = [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n),
                    a0 + (a1 - a0) * i / n + turn) for i in range(n + 1)]
        if pts and math.dist(pts[-1][:2], new[0][:2]) < 1e-6:
            new = new[1:]
        pts.extend(new)
    return pts


def _cumulative(points):
    s = [0.0]
    for a, b in zip(points, points[1:]):
        s.append(s[-1] + math.dist(a[:2], b[:2]))
    return s


@dataclass
class Edge:
    name: str
    start: str
    end: str
    points: list
    s: list = field(default_factory=list)
    track: str = ""       # 양방향 복도 이름 (방 간선은 "")
    direction: str = ""   # 복도 방향 "E" / "W"

    @property
    def length(self):
        return self.s[-1]


@dataclass
class Zone:
    id: str
    edge: str
    s0: float
    s1: float
    points: list          # 중심선 샘플 (x, y, yaw)
    shared: bool = False  # 다른 차선 구역과 충돌이 하나라도 있다
    phys: str = ""        # 물리 구역 id — 양방향 복도는 두 방향 간선이 같은 구역을 쓴다
    track: str = ""
    direction: str = ""

    @property
    def length(self):
        return self.s1 - self.s0


def hospital_edges():
    """지금 지도의 간선 — 주행 코드와 같은 기하(nav_to_goal.hospital_lanes).

    (이름, 시작 노드, 끝 노드, 원소 목록[, 복도 이름]) — 복도는 서->동 "t>E", 동->서 "t>W" 두 간선."""
    from nav_to_goal import hospital_lanes as L
    edges = [
        ("collection_out", "collection_dock", "east_junction", L.COLLECTION_OUT),
        ("collection_in", "east_junction", "collection_station", L.COLLECTION_IN),
        ("collection_dock_in", "collection_station", "collection_dock", [("line", L.LAB_STATION, L.LAB_DOCK)]),
        ("analysis_out", "analysis_dock", "west_junction", L.ANALYSIS_OUT),
        ("analysis_in", "west_junction", "analysis_station", L.ANALYSIS_IN),
        ("analysis_dock_in", "analysis_station", "analysis_dock", [("line", L.SPECIMEN_STATION, L.SPECIMEN_DOCK)]),
    ]
    for track in L.TRACKS:
        edges.append((f"{track}>E", "west_junction", "east_junction", L.corridor(track, L.TO_COLLECTION)[0], track))
        edges.append((f"{track}>W", "east_junction", "west_junction", L.corridor(track, L.TO_ANALYSIS)[0], track))
    return edges


# 구간(leg) 출발/도착 노드 — 도착은 책상 도킹까지 (도킹 구역도 예약한다)
LEG_NODES = {"to_analysis": ("collection_dock", "analysis_dock"),
             "to_collection": ("analysis_dock", "collection_dock")}


class LaneGraph:
    def __init__(self, edges, zone_m=ZONE_M, conflict_m=CONFLICT_M, neighbor_m=NEIGHBOR_M):
        self.edges = {}
        self.zones = {}
        self.edge_zones = {}
        self.segments = {}
        for name, start, end, segments, *rest in edges:
            track = rest[0] if rest else ""
            pts = _sample(segments)
            e = Edge(name, start, end, pts, _cumulative(pts), track, name[-1] if track else "")
            self.edges[name] = e
            self.segments[name] = segments
            n = max(1, math.ceil(e.length / zone_m - 1e-9))
            ids = []
            for k in range(n):
                s0, s1 = e.length * k / n, e.length * (k + 1) / n
                zp = [p for p, s in zip(e.points, e.s) if s0 - 1e-9 <= s <= s1 + 1e-9]
                # 복도는 서->동 순서로 물리 구역 번호를 매긴다 — 동->서 간선의 k 번째 = 서->동의 n-1-k 번째
                phys = f"{name}:{k}" if not track else f"{track}:{k if e.direction == 'E' else n - 1 - k}"
                z = Zone(f"{name}:{k}", name, s0, s1, zp, phys=phys, track=track, direction=e.direction)
                self.zones[z.id] = z
                ids.append(z.id)
            self.edge_zones[name] = ids
        # 물리 구역 -> 대표 구역 (복도는 서->동 간선의 것)
        self.phys_zones = {}
        for z in self.zones.values():
            if z.phys not in self.phys_zones or z.direction == "E":
                self.phys_zones[z.phys] = z
        self.conflicts = {p: set() for p in self.phys_zones}
        self._find_conflicts(conflict_m, neighbor_m)

    def phys(self, zid):
        return self.zones[zid].phys

    # ── 기하 ─────────────────────────────────────────────────────
    def _continues(self, a, b):
        """간선 a 끝에서 b 로 방향이 이어진다 (분기점에서 되돌아가는 연결은 경로가 아니다)"""
        ya, yb = self.edges[a].points[-1][2], self.edges[b].points[0][2]
        return abs(math.atan2(math.sin(ya - yb), math.cos(ya - yb))) < math.radians(30)

    def _path_distance(self, a, b, limit):
        """구역 a 끝에서 b 시작까지 간선을 따라간 거리(<= limit 일 때만, 아니면 inf). 같은 간선이면 s 차이."""
        za, zb = self.zones[a], self.zones[b]
        if za.edge == zb.edge and zb.s0 >= za.s1 - 1e-9:
            return zb.s0 - za.s1
        best = math.inf
        todo = [(self.edges[za.edge].length - za.s1, za.edge)]
        seen = set()
        while todo:
            d, edge = heapq.heappop(todo)
            if d > limit or edge in seen:
                continue
            seen.add(edge)
            for nxt in self.edges.values():
                if nxt.start == self.edges[edge].end and self._continues(edge, nxt.name):
                    if nxt.name == zb.edge:
                        best = min(best, d + zb.s0)
                    heapq.heappush(todo, (d + nxt.length, nxt.name))
        return best if best <= limit else math.inf

    def _find_conflicts(self, conflict_m, neighbor_m):
        """물리 구역끼리. 같은 복도 안(두 방향 포함)은 충돌로 보지 않는다 — 방향 잠금과 차간으로 지킨다."""
        reps = list(self.phys_zones.values())
        for i, za in enumerate(reps):
            for zb in reps[i + 1:]:
                if za.track and za.track == zb.track:
                    continue
                a, b = za.id, zb.id
                if (self._path_distance(a, b, neighbor_m) <= neighbor_m or
                        self._path_distance(b, a, neighbor_m) <= neighbor_m):
                    continue
                pa, pb = za.points, zb.points
                if min(math.dist(p[:2], q[:2]) for p in pa[::2] + pa[-1:] for q in pb[::2] + pb[-1:]) < conflict_m:
                    self.conflicts[za.phys].add(zb.phys)
                    self.conflicts[zb.phys].add(za.phys)
        for z in self.zones.values():
            z.shared = bool(self.conflicts[z.phys])

    # ── 경로 ─────────────────────────────────────────────────────
    def route(self, start_node, end_node):
        """최단 간선 목록 (Dijkstra, 가중치 = 길이)"""
        todo, seen = [(0.0, start_node, [])], set()
        while todo:
            d, node, path = heapq.heappop(todo)
            if node == end_node and path:
                return path
            if node in seen:
                continue
            seen.add(node)
            for e in self.edges.values():
                if e.start == node:
                    heapq.heappush(todo, (d + e.length, e.end, path + [e.name]))
        raise ValueError(f"no route {start_node} -> {end_node}")

    def routes(self, start_node, end_node):
        """가능한 경로 전부 [(길이, [간선])], 짧은 순. 노드를 다시 지나지 않는다 (그래프가 작다)."""
        out = []

        def walk(node, path, seen, length):
            if node == end_node and path:
                out.append((length, path))
                return
            for e in self.edges.values():
                if e.start == node and e.end not in seen and (not path or self._continues(path[-1], e.name)):
                    walk(e.end, path + [e.name], seen | {e.end}, length + e.length)

        walk(start_node, [], {start_node}, 0.0)
        return sorted(out, key=lambda item: item[0])

    def track_of(self, edge_names):
        """경로의 복도 (이름, 방향) — 없으면 ("", "")"""
        for name in edge_names:
            e = self.edges[name]
            if e.track:
                return e.track, e.direction
        return "", ""

    def route_zones(self, edge_names):
        return [z for e in edge_names for z in self.edge_zones[e]]

    def point_at(self, zone_ids, s):
        """구역 목록을 이은 경로 위 거리 s 의 (x, y, yaw)"""
        base = 0.0
        for zid in zone_ids:
            z = self.zones[zid]
            if s <= base + z.length + 1e-9 or zid == zone_ids[-1]:
                e = self.edges[z.edge]
                target = z.s0 + max(0.0, min(z.length, s - base))
                i = min(range(len(e.s)), key=lambda k: abs(e.s[k] - target))
                return e.points[i]
            base += z.length
        raise ValueError("empty route")

    def project(self, zone_ids, x, y, yaw=None, near=None, window=(-1.5, 6.0)):
        """경로(구역 목록) 위로 (x, y) 를 투영한 거리 s. near 가 있으면 그 근처(window)에서만 찾는다 —
        동쪽 문처럼 두 차선이 겹치는 곳에서 반대 차선으로 튀지 않게. 진행 방향과 90도 넘게 다르면 제외."""
        best, base = None, 0.0
        for zid in zone_ids:
            z = self.zones[zid]
            e = self.edges[z.edge]
            for p, s in zip(e.points, e.s):
                if not (z.s0 - 1e-9 <= s <= z.s1 + 1e-9):
                    continue
                rs = base + s - z.s0
                if near is not None and not (near + window[0] <= rs <= near + window[1]):
                    continue
                if yaw is not None and abs(math.atan2(math.sin(yaw - p[2]), math.cos(yaw - p[2]))) > math.pi / 2:
                    continue
                d = math.dist((x, y), p[:2])
                if best is None or d < best[0]:
                    best = (d, rs)
            base += z.length
        return (best[1], best[0]) if best else (None, None)


@dataclass
class RobotPlan:
    """예약 관리가 보는 로봇 한 대: 지금 경로(구역 목록)와 그 위 위치."""
    name: str
    zones: list                 # 경로 구역 (지나온 것 중 아직 차체가 걸친 것 포함)
    s: float = 0.0              # zones 를 이은 경로 위 base_link 거리
    score: tuple = (0, 0, 0)    # priority.score — 클수록 먼저
    since: float = 0.0          # 요청이 막히기 시작한 시각 (같은 점수면 먼저 기다린 쪽)
    frontier: int = -1          # 받은(허가된) 마지막 구역의 zones 인덱스 — 점유와 따로 센다
    leg: str = ""               # 구간 방향 (LEG_NODES) — 경로를 다시 고를 때
    edges: list = field(default_factory=list)   # 이 구간 경로 (간선)
    prefix: int = 0             # zones 앞에 붙은 지난 구간 구역 수 (set_plan 이 채운다)

    def offsets(self, graph):
        out, base = [], 0.0
        for zid in self.zones:
            out.append((zid, base, base + graph.zones[zid].length))
            base += graph.zones[zid].length
        return out


class Reservations:
    def __init__(self, graph, lookahead=LOOKAHEAD_M, front=FRONT_M, rear=REAR_M, margin=BODY_MARGIN_M):
        self.g = graph
        self.lookahead, self.front, self.rear, self.margin = lookahead, front, rear, margin
        self.owner = {}          # zone -> robot name
        self.robots = {}         # name -> RobotPlan

    def held(self, name):
        """이 로봇이 잡은 물리 구역"""
        return [z for z, r in self.owner.items() if r == name]

    def track_claims(self, exclude=None):
        """복도 방향 잠금 {복도: {로봇: 방향}} — 경로에 복도가 있고 아직 그 복도를 다 빠져나가지 않은 로봇"""
        claims = {}
        for r in self.robots.values():
            if r.name == exclude:
                continue
            for zid, a, b in r.offsets(self.g):
                z = self.g.zones[zid]
                if z.track and b > r.s - self.rear - self.margin + 1e-6:
                    claims.setdefault(z.track, {})[r.name] = z.direction
        return claims

    def entered(self, name, track):
        """이 로봇이 복도에 들어갔거나 그 구역을 이미 받았다 — 경로를 바꿀 수 없다"""
        r = self.robots.get(name)
        if r is None:
            return False
        return any(self.g.zones[zid].track == track for zid in r.zones[:r.frontier + 1])

    def reroute(self, name, edges):
        """복도에 안 들어간 로봇의 경로를 바꾼다 — 앞부분(지난 구간 구역 + 출발 방)이 같아 받은 구역은 그대로다."""
        r = self.robots[name]
        zones = r.zones[:r.prefix] + self.g.route_zones(edges)
        common = 0
        while common < min(len(zones), len(r.zones)) and zones[common] == r.zones[common]:
            common += 1
        assert r.frontier < common, f"{name} already holds zones past the common part"
        r.zones, r.edges = zones, list(edges)

    def set_plan(self, plan):
        """경로 교체 (새 구간 시작). 이전 경로에서 아직 잡고 있는 구역(차체가 걸친 도킹 구역 등)은 새 경로 앞에
        붙여 계속 잡고 있다가 차체가 벗어나면 푼다. plan.s 는 새 경로 기준 -> 붙인 길이만큼 민다."""
        old = self.robots.get(plan.name)
        if old is not None:
            keep = [z for z in old.zones if self.owner.get(self.g.phys(z)) == plan.name and z not in plan.zones]
            plan.zones = keep + list(plan.zones)
            plan.s += sum(self.g.zones[z].length for z in keep)
            plan.prefix += len(keep)
        # 지금 차체 앞이 들어가 있는 구역까지는 이미 받은 것으로 친다 (시작 자세, 도킹 자세)
        front = plan.s + self.front + self.margin
        plan.frontier = max((k for k, (_, a, _) in enumerate(plan.offsets(self.g)) if a < front - 1e-6), default=-1)
        self.robots[plan.name] = plan

    def remove(self, name):
        self.robots.pop(name, None)
        for z in self.held(name):
            del self.owner[z]

    def _free_for(self, zid, name, claims):
        z = self.g.zones[zid]
        other = self.owner.get(z.phys)
        if other is not None and other != name:
            return False
        if z.track and any(r != name and d != z.direction for r, d in claims.get(z.track, {}).items()):
            return False          # 같은 복도에 반대 방향 로봇 — 관제 배정이 막지만 여기서도 주지 않는다
        return not any(self.owner.get(c) not in (None, name) for c in self.g.conflicts[z.phys])

    def tick(self, now=0.0):
        """점유 갱신 + 예약 확장. 반환: {name: (정지 거리 s 또는 None, 받은 구역 목록)}

        정지점은 '받은' 구역 기준이다. 로봇이 제동하며 정지점을 넘어 안 받은 구역에 들어가면 그 구역은
        점유로 잡혀(다른 로봇을 막는다) 있지만 앞으로 더 주지 않고, 정지점이 뒤에 있으니 그 자리에 선다.
        남의 구역에 차체가 들어가면 intrusions 에 남긴다 (관제 경고)."""
        self.intrusions = []
        # 1. 차체가 걸친 구역은 그 로봇 것 (위치가 먼저다), 뒤로 벗어난 구역은 푼다
        for r in self.robots.values():
            lo, hi = r.s - self.rear - self.margin, r.s + self.front + self.margin
            for zid, a, b in r.offsets(self.g):
                pid = self.g.phys(zid)
                if b <= lo + 1e-6 and self.owner.get(pid) == r.name:
                    del self.owner[pid]
                elif a < hi - 1e-6 and b > lo + 1e-6:
                    other = self.owner.get(pid)
                    if other not in (None, r.name):
                        self.intrusions.append((r.name, pid, other))
                    else:
                        self.owner[pid] = r.name
        # 2. 우선순위 순서로 앞으로 늘린다
        # 같은 점수면 먼저 막혀 기다린 로봇부터 (since 0 = 아직 막힌 적 없음)
        order = sorted(self.robots.values(),
                       key=lambda r: (tuple(-v for v in r.score), r.since or math.inf, r.name))
        result = {}
        claims = self.track_claims()
        for r in order:
            offs = r.offsets(self.g)
            front_k = r.frontier
            blocked = False
            while front_k + 1 < len(offs) and offs[front_k][2] < r.s + self.front + self.lookahead:
                run = [front_k + 1]
                while self.g.zones[offs[run[-1]][0]].shared and run[-1] + 1 < len(offs):
                    run.append(run[-1] + 1)      # 충돌 구간은 빠져나간 첫 구역까지
                ids = [offs[k][0] for k in run]
                after = offs[run[-1] + 1][0] if run[-1] + 1 < len(offs) else None
                if all(self._free_for(z, r.name, claims) for z in ids) and (
                        after is None or self.owner.get(self.g.phys(after)) in (None, r.name)):
                    for z in ids:
                        self.owner[self.g.phys(z)] = r.name
                    front_k = r.frontier = run[-1]
                else:
                    blocked = True
                    break
            if blocked:
                r.since = r.since or now
            else:
                r.since = 0.0
            if front_k + 1 >= len(offs):
                result[r.name] = (None, [o[0] for o in offs[:front_k + 1]])
            else:
                end = offs[front_k][2] if front_k >= 0 else 0.0
                result[r.name] = (end - self.front - self.margin, [o[0] for o in offs[:front_k + 1]])
        return result


def choose_route(graph, res, name, leg, score):
    """구간(leg: to_analysis / to_collection)의 경로 — 짧은 순으로, 복도가 반대 방향으로 잡혀 있지 않은 것.

    반대 방향으로 잡은 로봇이 모두 이 로봇보다 긴급도가 낮고 아직 그 복도에 안 들어갔으면 그 경로를 가져간다
    (반환의 두 번째 = 경로를 다시 골라 줘야 할 로봇들). 같은 방향 뒤따르기는 된다.
    반환: (간선 목록, 양보할 로봇 목록)"""
    start, end = LEG_NODES[leg]
    options = graph.routes(start, end)
    claims = res.track_claims(exclude=name)
    for _, edges in options:
        track, direction = graph.track_of(edges)
        against = [r for r, d in claims.get(track, {}).items() if d != direction]
        if not against:
            return edges, []
        if all(tuple(res.robots[r].score) < tuple(score) and not res.entered(r, track) for r in against):
            return edges, against
    return options[0][1], []      # 모든 복도가 막혔다 (로봇 3대면 생기지 않는다) — 가장 짧은 길 입구에서 기다린다


def assign_route(graph, res, name, leg, score):
    """구간 시작: 경로를 고르고 예약 계획을 세운다. 복도를 내준 로봇들은 다음 경로로 바꾼다.

    반환: {로봇: 간선 목록} — 경로가 새로 정해지거나 바뀐 로봇 전부 (관제가 route 를 다시 보낸다)."""
    edges, yielders = choose_route(graph, res, name, leg, score)
    res.set_plan(RobotPlan(name, graph.route_zones(edges), 0.0, tuple(score), leg=leg, edges=edges))
    changed = {name: edges}
    for other in yielders:
        # 점수 (-1,) — 다른 로봇의 복도를 다시 뺏지 않는다
        alt, _ = choose_route(graph, res, other, res.robots[other].leg, (-1,))
        res.reroute(other, alt)
        changed[other] = alt
    return changed
