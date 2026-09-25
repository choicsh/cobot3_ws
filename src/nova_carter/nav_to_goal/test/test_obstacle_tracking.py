import math

from nav_to_goal.obstacle_tracking import Tracker, scan_clusters


def test_crossing_velocity_and_expiry():
    tracker = Tracker()
    for i in range(10):
        tracker.update([(2.0, -2.0 + i*0.125)], i*0.125)
    track = tracker.tracks[0]
    assert abs(track.vy - 1.0) < 0.05
    predictions = tracker.predictions(1.125)
    assert predictions[-1][1] > 0.8
    assert tracker.predictions(2.4) == []  # last seen 1.125, timeout 1.2 s
    assert tracker.update([], 2.4) == []


def test_static_tracks_no_swept_corridor_and_clock_reset():
    tracker = Tracker()
    for i in range(8):
        tracker.update([(2.0, 1.0)], i*0.125)
    assert len(tracker.predictions(.875)) == 0
    tracker.update([(20.0, 10.0)], 0.0)
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].hits == 1


def test_two_tracks_never_assigned_same_detection():
    tracker = Tracker()
    tracker.update([(0, 0), (.5, 0)], 0)
    tracker.update([(.1, 0)], .125)
    assert sorted(t.hits for t in tracker.tracks) == [1, 2]


def test_wall_rejected_small_cluster_retained():
    points = [(i*.05, 0.0) for i in range(60)]
    points += [None, (4, 1), (4.05, 1), (4.1, 1), None]
    clusters = scan_clusters(points)
    assert len(clusters) == 1
    assert math.dist(clusters[0], (4.05, 1)) < 1e-9


def test_standing_person_is_tracked_only_when_persistent_and_accepted():
    tracker = Tracker()
    for i in range(4):
        tracker.update([(3.0, 1.0)], i*0.1)
    accept = lambda x, y: True
    assert tracker.snapshots(0.3, standing_ok=accept) == []  # 4 scans: not yet
    tracker.update([(3.0, 1.0)], 0.4)
    snaps = tracker.snapshots(0.4, standing_ok=accept)
    assert len(snaps) == 1 and abs(snaps[0][3]) < .01 and abs(snaps[0][4]) < .01
    # Near mapped structure (e.g. the dock desk) a standing object is ignored,
    # and without a predicate the old moving-only contract is unchanged.
    assert tracker.snapshots(0.4, standing_ok=lambda x, y: False) == []
    assert tracker.snapshots(0.4) == []


def test_never_moved_object_becomes_static_after_standing_static_s():
    from nav_to_goal.obstacle_tracking import STANDING_STATIC_S
    tracker = Tracker()
    accept = lambda x, y: True
    seen = {}
    for i in range(int(STANDING_STATIC_S / 0.1) + 5):
        t = i*0.1
        tracker.update([(3.0, 1.0)], t)                         # a cone: never moves
        seen[round(t, 1)] = len(tracker.snapshots(t, standing_ok=accept))
    assert seen[1.0] == 1                                       # early: may be a person
    assert seen[round(STANDING_STATIC_S - 0.5, 1)] == 1
    assert seen[round(STANDING_STATIC_S + 0.2, 1)] == 0         # still for > 8 s: static


def test_moving_person_stays_tracked_after_standing_static_s():
    tracker = Tracker()
    t = 0.0
    for i in range(8):                                          # walks 1 m/s, then stops
        tracker.update([(3.0, i*0.125)], t)
        t += 0.125
    for _ in range(100):                                        # stands for ~12 s
        tracker.update([(3.0, 7*0.125)], t)
        t += 0.125
    assert len(tracker.snapshots(t - 0.125, standing_ok=lambda x, y: True)) == 1
