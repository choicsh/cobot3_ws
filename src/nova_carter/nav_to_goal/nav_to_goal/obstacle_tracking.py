"""Short-lived scan-cluster tracks. No simulator poses or human identity labels."""
from collections import deque
from dataclasses import dataclass, field
import math


@dataclass
class Track:
    x: float
    y: float
    stamp: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    motion_until: float = -math.inf
    history: deque = field(default_factory=lambda: deque(maxlen=8))
    track_id: int = 0
    was_moving: bool = False


class Tracker:
    def __init__(self, timeout=1.2):
        self.timeout = timeout
        self.tracks = []
        self.last_stamp = None
        self.next_id = 1

    def update(self, detections, stamp):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.tracks = []
        self.last_stamp = stamp
        self.tracks = [t for t in self.tracks if 0 <= stamp - t.stamp <= self.timeout]
        pairs = []
        for i, track in enumerate(self.tracks):
            dt = stamp - track.stamp
            for j, (x, y) in enumerate(detections):
                distance = math.hypot(x - track.x - track.vx*dt, y - track.y - track.vy*dt)
                if distance < max(0.45, 2.0*dt):
                    pairs.append((distance, i, j))
        used_tracks, used_detections = set(), set()
        for _, i, j in sorted(pairs):
            if i in used_tracks or j in used_detections:
                continue
            used_tracks.add(i)
            used_detections.add(j)
            track = self.tracks[i]
            x, y = detections[j]
            track.history.append((stamp, x, y))
            # Use a short history to reduce scan-silhouette jitter, while detecting
            # a new crossing within a few scans rather than seconds.
            history = [p for p in track.history if stamp - p[0] <= 0.45]
            t0, x0, y0 = history[0]
            dt = stamp - t0
            if dt >= 0.1:
                vx, vy = (x - x0) / dt, (y - y0) / dt
                speed = math.hypot(vx, vy)
                if speed <= 2.2:
                    track.vx = 0.65*vx + 0.35*track.vx
                    track.vy = 0.65*vy + 0.35*track.vy
                else:
                    track.vx = track.vy = 0.0
            track.x, track.y, track.stamp = x, y, stamp
            track.hits += 1
            if track.hits >= 3 and math.hypot(track.vx, track.vy) >= 0.25:
                track.was_moving = True
            # A previously moving object remains protected while observed, even
            # after it stops. Expiry still handles missing observations.
            if track.was_moving:
                track.motion_until = stamp + 2.0
        for j, (x, y) in enumerate(detections):
            if j not in used_detections:
                track = Track(x, y, stamp)
                track.track_id = self.next_id
                self.next_id += 1
                track.history.append((stamp, x, y))
                self.tracks.append(track)
        return self.tracks

    def snapshots(self, stamp, radius=0.4):
        """Current positions + velocity + observation age, not collapsed futures."""
        return [(t.track_id, t.x+t.vx*(stamp-t.stamp), t.y+t.vy*(stamp-t.stamp),
                 t.vx, t.vy, radius, stamp-t.stamp)
                for t in self.tracks if t.was_moving and t.hits >= 3 and
                0 <= stamp-t.stamp <= self.timeout]

    def predictions(self, stamp, horizon=1.8, step=0.2, radius=0.4):
        result = []
        for track in self.tracks:
            age = stamp - track.stamp
            if track.hits < 3 or track.motion_until < stamp or not 0 <= age <= self.timeout:
                continue
            moving = track.hits >= 3 and math.hypot(track.vx, track.vy) >= 0.2
            count = int(horizon / step) + 1 if moving else 1
            for i in range(count):
                dt = age + i*step
                result.append((track.x + track.vx*dt, track.y + track.vy*dt,
                               radius + min(0.2, 0.1*dt)))
        return result


def scan_clusters(points, gap=0.3):
    """Consecutive scan hits; None separates invalid, self or distant returns."""
    groups, group = [], []
    for point in points:
        if point is None or (group and math.dist(point, group[-1]) > gap):
            if group:
                groups.append(group)
            group = []
        if point is not None:
            group.append(point)
    if group:
        groups.append(group)
    if len(groups) > 1 and math.dist(groups[-1][-1], groups[0][0]) <= gap:
        groups[0] = groups.pop() + groups[0]
    centers = []
    for group in groups:
        if len(group) < 3:
            continue
        xs, ys = zip(*group)
        if math.hypot(max(xs)-min(xs), max(ys)-min(ys)) > 1.0:
            continue
        centers.append((sum(xs)/len(xs), sum(ys)/len(ys)))
    return centers
