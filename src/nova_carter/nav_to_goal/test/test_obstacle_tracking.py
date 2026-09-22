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
    assert tracker.predictions(2.0) == []
    assert tracker.update([], 2.0) == []


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
