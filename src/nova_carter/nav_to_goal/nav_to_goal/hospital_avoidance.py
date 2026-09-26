"""ROS-free forward-only avoidance geometry and short-horizon human checks.

These are kinematic checks, not a dynamics model or a safety certification.
All poses/tracks passed to a check MUST be in one common metric frame.
"""
from dataclasses import dataclass, replace
import math


@dataclass(frozen=True)
class SafetySettings:
    front: float = 0.48
    rear: float = 1.38
    half_width: float = 0.5
    minimum_gap: float = 0.4
    # 0.6 이면 사람 옆 0.55 m 를 0.6 m/s 로 차선 이탈 없이 지나갔다(Isaac 실측).
    # 예측 간격이 이보다 작으면 MPPI 복도에서 옆으로 비켜 가는 경로를 만든다.
    preferred_gap: float = 1.0
    yield_gap: float = 1.0
    # 비켜 갈 경로가 없거나(방 안 DWB) 아직 비켜 가는 중이면 이 간격 안에서는 천천히 지나간다.
    passing_gap: float = 1.0
    passing_speed: float = 0.3
    horizon: float = 3.0
    guard_horizon: float = 1.5
    dt: float = 0.1
    acceleration: float = 0.5
    deceleration: float = 0.6
    angular_acceleration: float = 1.5
    max_speed: float = 0.6
    max_yaw_rate: float = 0.9
    # Isaac GUI lidar arrives every 0.1-1.2 s (3 Hz mean); shared stale-data limit.
    track_timeout: float = 1.2
    uncertainty_rate: float = 0.10
    # Candidate centerlines stay inside this lane-local strip; actual body too.
    lane_half_width: float = 2.8
    minimum_radius: float = 1.2
    reaction_time: float = 0.2


@dataclass(frozen=True)
class MovingBody:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    age: float = 0.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def body_gap(pose, x, y, radius, settings=SafetySettings()):
    """Signed circle-to-asymmetric-rectangle gap; negative means overlap."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x-px, y-py
    bx, by = c*dx+s*dy, -s*dx+c*dy
    outside_x = max(-settings.rear-bx, 0., bx-settings.front)
    outside_y = max(abs(by)-settings.half_width, 0.)
    if outside_x or outside_y:
        return math.hypot(outside_x, outside_y)-radius
    return -min(bx+settings.rear, settings.front-bx,
                settings.half_width-abs(by))-radius


def predicted_gap(pose, tracks, seconds, settings=SafetySettings()):
    return min((body_gap(
        pose, t.x+t.vx*seconds, t.y+t.vy*seconds,
        t.radius+settings.uncertainty_rate*(t.age+seconds), settings)
        for t in tracks), default=math.inf)


def ramp(value, target, up, down, dt):
    limit = up if abs(target) > abs(value) and value*target >= 0 else down
    return value + max(-limit*dt, min(limit*dt, target-value))


def command_clearance(pose, velocity, command, tracks, settings=SafetySettings(), include_now=True):
    """Roll out measured velocity -> command, including reaction/ramp and tail.

    A stop is simulated with finite deceleration, not an instantaneous freeze.
    Tracks are linear extrapolations with growing uncertainty, rechecked online.
    """
    x, y, yaw = pose
    v, w = velocity
    gap = predicted_gap(pose, tracks, 0., settings) if include_now else math.inf
    for k in range(1, math.ceil(settings.horizon/settings.dt)+1):
        seconds = k*settings.dt
        target_v, target_w = velocity if seconds <= settings.reaction_time else command
        v = ramp(v, target_v, settings.acceleration, settings.deceleration, settings.dt)
        w = ramp(w, target_w, settings.angular_acceleration,
                 settings.angular_acceleration, settings.dt)
        middle = yaw+w*settings.dt/2
        x += v*math.cos(middle)*settings.dt
        y += v*math.sin(middle)*settings.dt
        yaw = wrap(yaw+w*settings.dt)
        gap = min(gap, predicted_gap((x, y, yaw), tracks, seconds, settings))
    return gap


def limited_command(pose, velocity, command, tracks, settings=SafetySettings()):
    """Prefer the full command; reduce only when the human envelope requires it.

    This guard cannot generate an escape direction. The mission selects paths.
    If no tested command is adequate, return stop AND an explicit unsafe status.
    """
    if not tracks:
        return command, "CLEAR", math.inf
    # A current velocity is not a 3-second constant-command plan. Use a shorter
    # braking horizon here, while path selection considers 3 seconds ahead.
    settings = replace(settings, horizon=max(settings.guard_horizon,
        abs(velocity[0])/settings.deceleration+settings.reaction_time))
    for scale in (1., .75, .5, 0.):
        candidate = command[0]*scale, command[1]*scale
        gap = command_clearance(pose, velocity, candidate, tracks, settings)
        threshold = settings.minimum_gap if scale == 1. else settings.yield_gap
        if gap >= threshold:
            if scale == 1. and gap < settings.passing_gap and abs(candidate[0]) > settings.passing_speed:
                # Pass close people slowly; scale w with v to keep the curvature.
                k = settings.passing_speed/abs(candidate[0])
                slow = candidate[0]*k, candidate[1]*k
                slow_gap = command_clearance(pose, velocity, slow, tracks, settings)
                if slow_gap >= settings.minimum_gap:
                    return slow, 'PASSING_SLOW', slow_gap
                continue
            state = ('CLEAR' if gap >= settings.preferred_gap else 'PASS_MARGIN_SHORTFALL') if scale == 1. else 'YIELDING'
            return candidate, state, gap
    # Already inside the minimum gap (a person stopped beside the robot, or one
    # walked through it): every rollout above includes that current gap, so all
    # commands fail and robot and person can wait for each other forever
    # (2026-09-25: a crossing pedestrian stood 0.4 m beside-behind the robot
    # for 2 minutes). Allow a slow command that does not close the distance.
    escape = escape_command(pose, velocity, command, tracks, settings)
    if escape is not None:
        return escape
    state = 'YIELD_MARGIN_SHORTFALL' if gap >= settings.minimum_gap else 'NO_SAFE_COMMAND'
    return (0., 0.), state, gap


def _geometric(settings):
    """Pure geometry: no growth of track uncertainty with look-ahead time.

    'Does this motion close the distance?' must not count the uncertainty that
    grows by itself; beside a stopped person every motion would look closer."""
    return replace(settings, uncertainty_rate=0.)


def escape_command(pose, velocity, command, tracks, settings=SafetySettings(), tolerance=.01):
    """(slow command, 'ESCAPING', gap) if already inside minimum_gap and the
    command, capped at passing_speed, keeps the geometric gap from shrinking."""
    geometric = _geometric(settings)
    now = predicted_gap(pose, tracks, 0., geometric)
    if now >= settings.minimum_gap or abs(command[0]) < .01:
        return None
    k = min(1., settings.passing_speed/abs(command[0]))
    slow = command[0]*k, command[1]*k
    future = command_clearance(pose, velocity, slow, tracks, geometric, include_now=False)
    return (slow, 'ESCAPING', future) if future >= now-tolerance else None


def path_escapes(path, pose, velocity, tracks, threshold, settings=SafetySettings(), tolerance=.01):
    """True if already inside threshold and following path at passing speed
    keeps the geometric gap from shrinking (the stage need not yield)."""
    geometric = _geometric(settings)
    now = predicted_gap(pose, tracks, 0., geometric)
    if now >= threshold:
        return False
    future = path_clearance(path, pose, velocity, tracks, geometric, settings.passing_speed, include_now=False)
    return future >= now-tolerance


@dataclass(frozen=True)
class LaneFrame:
    x: float
    y: float
    yaw: float
    length: float

    def local(self, pose):
        dx, dy = pose[0]-self.x, pose[1]-self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c*dx+s*dy, -s*dx+c*dy, wrap(pose[2]-self.yaw)

    def world(self, s, d, angle=0.):
        c, sn = math.cos(self.yaw), math.sin(self.yaw)
        return self.x+c*s-sn*d, self.y+sn*s+c*d, wrap(self.yaw+angle)


def lane_for_path(path):
    a, b = path[0], path[-1]
    return LaneFrame(a[0], a[1], math.atan2(b[1]-a[1], b[0]-a[0]),
                     math.dist(a[:2], b[:2]))


def densify_planner_path(path, step=.05):
    """Give the planner's sparse XY path continuous forward headings.

    This does not smooth corners; forward_path_valid must still reject turns
    that the long chassis cannot follow.
    """
    xy = []
    for pose in path:
        point = pose[:2]
        if not all(math.isfinite(value) for value in point):
            return []
        if not xy or math.dist(xy[-1], point) > 1e-4:
            xy.append(point)
    if len(xy) < 2:
        return []
    sampled = [xy[0]]
    for a, b in zip(xy, xy[1:]):
        count = max(1, math.ceil(math.dist(a, b)/step))
        sampled.extend((a[0]+(b[0]-a[0])*i/count,
                        a[1]+(b[1]-a[1])*i/count)
                       for i in range(1, count+1))
    return [(p[0], p[1], math.atan2(
        sampled[min(i+1, len(sampled)-1)][1]-sampled[max(i-1, 0)][1],
        sampled[min(i+1, len(sampled)-1)][0]-sampled[max(i-1, 0)][0]))
        for i, p in enumerate(sampled)]


def footprint_points(pose, settings=SafetySettings(), step=.10):
    """Interior AND perimeter samples; grid checker adds cell/sampling padding."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    nx = math.ceil((settings.front+settings.rear)/step)
    ny = math.ceil(2*settings.half_width/step)
    for i in range(nx+1):
        bx = -settings.rear+(settings.front+settings.rear)*i/nx
        for j in range(ny+1):
            by = -settings.half_width+2*settings.half_width*j/ny
            yield pose[0]+c*bx-s*by, pose[1]+s*bx+c*by


def forward_path_valid(path, lane, start_s, settings=SafetySettings()):
    """Reject reverse progress, loops, sharp joins and body leaving lane strip."""
    if len(path) < 2:
        return False
    last_s, last_pose = start_s-.06, None
    for pose in path:
        s, d, angle = lane.local(pose)
        if (not all(math.isfinite(v) for v in pose) or s < last_s-.005 or
                s > lane.length+.05 or abs(angle) > math.radians(65)):
            return False
        # Furthest lateral corner: no circular-body approximation.
        if abs(d)+settings.half_width*abs(math.cos(angle))+max(
                settings.rear, settings.front)*abs(math.sin(angle)) > settings.lane_half_width:
            return False
        if last_pose is not None:
            distance = math.dist(last_pose[:2], pose[:2])
            if distance > .12 or distance < 1e-6:
                return False
            tangent = math.atan2(pose[1]-last_pose[1], pose[0]-last_pose[0])
            if abs(wrap(tangent-pose[2])) > .18:
                return False
            if abs(wrap(pose[2]-last_pose[2])) / distance > 1/settings.minimum_radius:
                return False
        last_s, last_pose = s, pose
    return abs(lane.local(path[-1])[1]) < .02 and abs(lane.local(path[-1])[2]) < .02


def _transition(lane, start_s, length, d0, d1, slope0=0., step=.05):
    """Quintic lateral shift with continuous slope and zero endpoint curvature."""
    n = math.ceil(length/step)
    points = []
    delta, m = d1-d0, slope0*length
    a3, a4, a5 = 10*delta-6*m, -15*delta+8*m, 6*delta-3*m
    for i in range(n+1):
        t = i/n
        d = d0+m*t+a3*t**3+a4*t**4+a5*t**5
        slope = (m+3*a3*t*t+4*a4*t**3+5*a5*t**4)/length
        points.append(lane.world(start_s+t*length, d, math.atan(slope)))
    return points


def detour_clear_after(blocked_s, person_near, settings=SafetySettings()):
    """Lane s the whole body must pass before a detour starts its return to the lane.

    blocked_s = first blocked reference point. A person keeps the person gap; a
    static blockage (cone, or a person who has stood still long enough) is checked
    against the costmap along the candidate instead. With the person gap an
    obstacle within ~7 m of a stage end left no candidate at all: the return
    would run past the stage (2026-09-26, P7 p7e, x=1.5 on lane_upper ending x=7.9).
    """
    return blocked_s+settings.rear+(settings.preferred_gap+.5 if person_near else .5)


def offset_candidates(lane, pose, settings=SafetySettings(), preferred_side=0, clear_after_s=None):
    """Forward S paths; no nearest-XY rejoin and no NavFn U-turn shortcuts."""
    start_s, d, angle = lane.local(pose)
    if start_s < -.3 or abs(angle) > math.radians(60):
        return []
    candidates = []
    sides = (preferred_side, -preferred_side) if preferred_side else (1, -1)
    # A latched side may still be abandoned if every candidate on it is unsafe.
    for side in sides:
        for offset in (1.5, 1.8, 2.0):
            for shift in (3., 4., 5.):
                hold, rejoin = 2.5, 4.
                if clear_after_s is not None:
                    hold = max(hold, clear_after_s-start_s-shift)
                end_s = start_s+shift+hold+rejoin
                if end_s > lane.length-.3:
                    continue
                path = _transition(lane, start_s, shift, d, side*offset, math.tan(angle))
                path += _transition(lane, start_s+shift, hold, side*offset, side*offset)[1:]
                path += _transition(lane, start_s+shift+hold, rejoin, side*offset, 0.)[1:]
                if forward_path_valid(path, lane, start_s, settings):
                    candidates.append((side, path))
    return candidates


def rejoin_clear(path, pose, velocity, tracks, settings=SafetySettings(), tolerance=.01):
    """May the robot turn back into the lane along path (its active remaining path)?

    Follows the actual rejoin path with the full footprint and asks whether the
    gap to any track falls below preferred_gap, or below the current gap if it
    is already smaller. Pure geometry (see _geometric); the output guard still
    checks every real command with the full rules.

    Replaced a zone test that refused to rejoin while any track stood anywhere
    across the corridor (+-2.8 m) from 2.8 m behind to 2 m ahead: a person on
    the far side, or one already passed and standing behind, forced a second
    sideways detour on an open lane (2026-09-26 rosbag p4e).
    """
    if not tracks:
        return True
    geometric = _geometric(settings)
    now = predicted_gap(pose, tracks, 0., geometric)
    future = path_clearance(path, pose, velocity, tracks, geometric, include_now=False)
    return future >= min(settings.preferred_gap, now)-tolerance


def path_clearance(path, pose, velocity, tracks, settings=SafetySettings(), speed=None, include_now=True):
    """Approximate pursuit rollout, with acceleration/yaw-rate limits.

    Tests reference selection, not MPPI's internal sampled trajectories. The
    separate output guard rechecks the command that MPPI actually produces.
    """
    if not tracks:
        return math.inf
    speed = settings.max_speed if speed is None else speed
    x, y, yaw = pose
    v, w = velocity
    cursor = 0
    gap = predicted_gap(pose, tracks, 0., settings) if include_now else math.inf
    for k in range(1, math.ceil(settings.horizon/settings.dt)+1):
        cursor = min(range(cursor, min(len(path), cursor+65)),
                     key=lambda i: math.hypot(path[i][0]-x, path[i][1]-y))
        target = path[min(len(path)-1, cursor+16)]
        distance = max(.2, math.hypot(target[0]-x, target[1]-y))
        alpha = wrap(math.atan2(target[1]-y, target[0]-x)-yaw)
        target_v = speed if math.dist((x, y), path[-1][:2]) > .15 else 0.
        target_w = max(-settings.max_yaw_rate, min(settings.max_yaw_rate,
                       2*max(v, .15)*math.sin(alpha)/distance))
        if k*settings.dt <= settings.reaction_time:
            target_v, target_w = velocity
        v = ramp(v, target_v, settings.acceleration, settings.deceleration, settings.dt)
        w = ramp(w, target_w, settings.angular_acceleration, settings.angular_acceleration, settings.dt)
        middle = yaw+w*settings.dt/2
        x += v*math.cos(middle)*settings.dt
        y += v*math.sin(middle)*settings.dt
        yaw = wrap(yaw+w*settings.dt)
        gap = min(gap, predicted_gap((x, y, yaw), tracks, k*settings.dt, settings))
    return gap


def choose_candidate(candidates, pose, velocity, tracks, static_clear,
                     settings=SafetySettings(), preferred_side=0):
    """Prefer target clearance; minimum clearance is a rejection threshold."""
    ranked = []
    for side, path in candidates:
        # Account for downstream slowdown: it scales both v and w to 40%.
        gap = min(path_clearance(path, pose, velocity, tracks, settings, speed)
                  for speed in (settings.max_speed, settings.max_speed*.4))
        if gap < settings.minimum_gap:
            continue
        penalty = max(0., settings.preferred_gap-gap)*20
        penalty += 2. if preferred_side and side != preferred_side else 0.
        penalty += sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))*.02
        ranked.append((penalty, side, path, gap))
    for _, side, path, gap in sorted(ranked, key=lambda item: item[0]):
        if static_clear(path):
            return side, path, gap
    return None
