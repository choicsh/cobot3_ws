"""FollowPath stage supervision: lateral path choice, yield and forward rejoin.

No robot commands are sent directly here. Path selection uses a short-horizon
kinematic approximation; Collision Monitor checks the smoothed commands.
"""
import math
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String

from nav_to_goal.hospital_avoidance import (
    SafetySettings, choose_candidate, densify_planner_path, forward_path_valid,
    lane_for_path, offset_candidates, path_clearance, path_escapes, tail_clear_for_rejoin, wrap,
)
from nav_to_goal.hospital_safety import SafetyObservations, yaw_of


def path_points(path):
    return [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in path.poses]


def drain_observations(navigator):
    """Service ready clock/TF/sensor callbacks before assessing freshness.

    A single spin per 0.1 s cannot service several 30 Hz TF publishers plus
    clock, odom and scan. Bound the work so path supervision still runs.
    """
    deadline = time.monotonic() + .01
    for _ in range(128):
        rclpy.spin_once(navigator, timeout_sec=0.)
        if time.monotonic() >= deadline:
            break


def follow_stage(navigator, tf_buffer, plan_publisher, stage_name, route,
                 controller_id, goal_checker_id, final_yaw=None, blockage_monitor=None):
    # Deferred import keeps geometry reusable without ROS and avoids import cycles.
    from nav_to_goal.hospital_mission import (
        MissionStatus, STATIC_BLOCK_PERSISTENCE_S,
        STATIC_BLOCK_POSITION_TOLERANCE_M, build_path, create_pose,
        remaining_path, request_detour, task_result_to_status,
    )

    settings = SafetySettings()
    reference = build_path(navigator, route)
    if final_yaw is not None:
        reference.poses[-1].pose.orientation.z = math.sin(final_yaw/2)
        reference.poses[-1].pose.orientation.w = math.cos(final_yaw/2)
    reference_points = path_points(reference)
    mppi = controller_id == 'FollowPathMPPI'
    lane = lane_for_path(reference_points) if mppi else None
    observations = SafetyObservations(navigator, tf_buffer, with_maps=True)
    qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    reference_pub = navigator.create_publisher(Path, '/hospital/reference_plan', qos)
    state_pub = navigator.create_publisher(String, '/hospital/mission_state', qos)
    reference_pub.publish(reference)
    active = reference
    active_index = reference_index = 0
    active_task = False
    state = None
    side = 0
    in_detour = False
    rejoin_s = math.inf
    wait_since = clear_since = stale_since = None
    last_replan = -math.inf
    last_plan_pub = -math.inf
    static_since = None
    static_xy = None
    stopped_tracks = {}
    planner_last_s = -math.inf
    failure_count = 0
    previous_now = None

    def event(new_state, reason):
        nonlocal state
        if state != new_state:
            state = new_state
            message = f'stage={stage_name} state={state} reason={reason}'
            navigator.get_logger().info(message)
            state_pub.publish(String(data=message))

    def as_path(points):
        result = Path()
        result.header = reference.header
        result.header.stamp = navigator.get_clock().now().to_msg()
        result.poses = [create_pose(navigator, *point) for point in points]
        return result

    def dispatch(path):
        nonlocal active, active_index, active_task
        # A new goal to the SAME controller updates the reference without a
        # cancel/stop between every safe avoidance selection.
        active = path
        active_index = 0
        active.header.stamp = navigator.get_clock().now().to_msg()
        plan_publisher.publish(active)
        accepted = navigator.followPath(active, controller_id=controller_id,
                                        goal_checker_id=goal_checker_id)
        active_task = accepted is not False
        return active_task

    def cancel():
        nonlocal active_task
        if active_task:
            navigator.cancelTask()
            deadline = time.monotonic()+2.0
            while not navigator.isTaskComplete():
                if time.monotonic() > deadline:
                    return False
                time.sleep(.02)
            active_task = False
        return True

    try:
        # Do not send a moving goal before the new guard's observations exist.
        deadline = time.monotonic()+5.0
        while rclpy.ok() and (observations.snapshot() is None or observations.static is None
                              or observations.local is None):
            rclpy.spin_once(navigator, timeout_sec=.05)
            if time.monotonic() > deadline:
                event('FAILED', 'initial_observations_unavailable')
                return MissionStatus.FAILED
        if not dispatch(reference):
            event('FAILED', 'follow_path_rejected')
            return MissionStatus.FAILED
        event('TRACKING', 'reference')

        while rclpy.ok():
            # isTaskComplete spins callbacks only with an outstanding action.
            complete = navigator.isTaskComplete() if active_task else False
            if not active_task:
                rclpy.spin_once(navigator, timeout_sec=.05)
            drain_observations(navigator)
            now = observations.now()
            if previous_now is not None and now < previous_now:
                event('FAILED', 'clock_reset_requires_new_mission')
                return MissionStatus.FAILED
            previous_now = now
            snapshot = observations.snapshot()
            if snapshot is None:
                if not cancel():
                    return MissionStatus.FAILED
                event('WAITING_DATA', 'tracks_odom_or_tf_unavailable')
                stale_since = now if stale_since is None else stale_since
                if now-stale_since > 2.0:
                    event('FAILED', 'observation_timeout')
                    return MissionStatus.FAILED
                time.sleep(.05)
                continue
            stale_since = None
            pose, velocity, tracks = snapshot
            remaining_ref, reference_index = remaining_path(reference, tf_buffer, reference_index)
            rest, active_index = remaining_path(active, tf_buffer, active_index)
            if rest is None or remaining_ref is None:
                if not cancel():
                    return MissionStatus.FAILED
                event('FAILED', 'reference_transform_unavailable')
                return MissionStatus.FAILED
            points = path_points(rest)

            if complete:
                result = task_result_to_status(navigator.getResult())
                active_task = False
                if result == MissionStatus.CANCELED:
                    event('CANCELED', 'external_cancel')
                    return result
                if result == MissionStatus.SUCCEEDED:
                    if final_yaw is not None:
                        # Require several fresh measured stopped samples. Keep
                        # 0.5m/s arrival cap; do not confuse cap with goal speed.
                        stable_start = None
                        deadline = time.monotonic()+3.0
                        while time.monotonic() < deadline:
                            rclpy.spin_once(navigator, timeout_sec=.05)
                            drain_observations(navigator)
                            sample = observations.snapshot()
                            if sample is None:
                                stable_start = None
                                continue
                            actual, speed, _ = sample
                            good = (math.dist(actual[:2], reference_points[-1][:2]) <= .05 and
                                    abs(wrap(actual[2]-final_yaw)) <= .10 and
                                    abs(speed[0]) <= .03 and abs(speed[1]) <= .05)
                            if not good:
                                stable_start = None
                            elif stable_start is None:
                                stable_start = observations.now()
                            elif observations.now()-stable_start >= .3:
                                event('SUCCEEDED', 'station_pose_and_stop_confirmed')
                                return result
                        event('FAILED', 'station_pose_or_stop_not_confirmed')
                        return MissionStatus.FAILED
                    event('SUCCEEDED', 'stage_reached')
                    return result
                failure_count += 1
                if not mppi or failure_count >= 3:
                    event('FAILED', 'controller_failure')
                    return MissionStatus.FAILED

            # Re-evaluate actual active path, not the original line during detour.
            # The predictive reference selector belongs to the open MPPI
            # corridor. In room turns DWB's full-footprint obstacle critic and
            # Collision Monitor handle current obstacles; there is no lateral
            # candidate mechanism here to resolve apparent moving wall corners.
            gap = (path_clearance(points, pose, velocity, tracks, settings, .6)
                   if mppi else math.inf)
            threshold = settings.minimum_gap if in_detour else settings.preferred_gap
            risk = mppi and gap < threshold
            if risk and path_escapes(points, pose, velocity, tracks, threshold, settings):
                # Already inside the margin, but the path leads away: yielding would
                # only wait for a person who may be waiting for us. The output
                # guard caps the speed (PASSING_SLOW / ESCAPING).
                risk = False
            if in_detour and lane.local(pose)[0] >= rejoin_s-4.0:
                risk = risk or not tail_clear_for_rejoin(lane, pose, tracks, settings)
            blockage = blockage_monitor.observe(reference, reference_index) if mppi and blockage_monitor else None
            # A person who remains in the route is also a persistent blockage.
            # Keep a recently stopped track in the moving branch for 1.5 s,
            # then allow the same bounded planner fallback as for a cone.
            near = ([t for t in tracks if math.dist((t.x, t.y), blockage['map_xy']) < 2.0]
                    if blockage is not None else [])
            stopped_tracks = {t.track_id: stopped_tracks.get(t.track_id, now)
                              for t in near if math.hypot(t.vx, t.vy) < .2}
            settled = (bool(near) and len(stopped_tracks) == len(near) and
                       all(now-stopped_tracks[t.track_id] >= STATIC_BLOCK_PERSISTENCE_S
                           for t in near))
            static_block = (blockage is not None and
                            (not blockage['dynamic'] or settled) and
                            not any(math.hypot(t.vx, t.vy) >= .2 for t in near))
            if static_block:
                if (static_xy is None or math.dist(static_xy, blockage['map_xy']) >
                        STATIC_BLOCK_POSITION_TOLERANCE_M):
                    static_xy, static_since = blockage['map_xy'], now
            else:
                static_xy = static_since = None
            static_ready = (static_since is not None and
                            now-static_since >= STATIC_BLOCK_PERSISTENCE_S)
            # Try the validated FollowPath offset as soon as a blockage is
            # visible. Persistence gates only the slower NavFn fallback.
            blocked_reference = blockage is not None and not in_detour
            need_path = risk or blocked_reference

            if mppi and need_path and now-last_replan >= 1.0:
                last_replan = now
                clear_after = None
                if blockage is not None:
                    blocked_s = lane.local((*blockage['map_xy'], 0.))[0]
                    clear_after = blocked_s+settings.rear+settings.preferred_gap+.5
                selection_started = time.monotonic()
                selected = choose_candidate(offset_candidates(lane, pose, settings, side, clear_after),
                    pose, velocity, tracks, lambda p: observations.path_clear(p, settings),
                    settings, side)
                selection_seconds = time.monotonic()-selection_started
                if selection_seconds > .25:
                    navigator.get_logger().info(
                        f'[AVOIDANCE] candidate evaluation took {selection_seconds:.2f}s')
                if selected is not None:
                    side, chosen, candidate_gap = selected
                    join_s = lane.local(chosen[-1])[0]
                    # Append forward reference only. Never return to an earlier index.
                    tail = [p for p in reference_points[reference_index:] if lane.local(p)[0] > join_s+.01]
                    combined = chosen+tail
                    if forward_path_valid(combined, lane, lane.local(pose)[0], settings):
                        # Candidate checking may consume a full sensor cycle.
                        # Process pending observations and recheck before sending.
                        rclpy.spin_once(navigator, timeout_sec=0.0)
                        fresh = observations.snapshot()
                        if (fresh is not None and math.dist(fresh[0][:2], pose[:2]) <= .2 and
                                observations.path_clear(combined, settings) and
                                path_clearance(combined, *fresh, settings) >= settings.minimum_gap):
                            if not dispatch(as_path(combined)):
                                return MissionStatus.FAILED
                            in_detour = True
                            rejoin_s = join_s
                            wait_since = clear_since = None
                            event('AVOIDING', f'forward_offset side={side} predicted_gap={candidate_gap:.2f}')
                            continue
                # Bounded NavFn fallback is only for persistent static evidence.
                # A second attempt at the same progress location is suppressed.
                progress = lane.local(pose)[0]
                if static_ready and progress-planner_last_s >= 2.0:
                    planner_last_s = progress
                    join_s = min(lane.length, progress+10.0)
                    if join_s-progress >= 4.0:
                        if not cancel():
                            return MissionStatus.FAILED
                        navigator.get_logger().info(
                            f'[DETOUR] requesting NavFn from s={progress:.2f} to s={join_s:.2f}')
                        planner_started = time.monotonic()
                        detour = request_detour(navigator, tf_buffer,
                            create_pose(navigator, *lane.world(join_s, 0.)))
                        navigator.get_logger().info(
                            f'[DETOUR] planner and smoother took {time.monotonic()-planner_started:.2f}s')
                        if detour is not None:
                            join = lane.world(join_s, 0.)
                            raw = path_points(detour)
                            # NavFn may finish within its goal tolerance, not
                            # exactly on the reference. Connect only a short,
                            # fully validated forward segment to the lane.
                            chosen = (densify_planner_path(raw+[join])
                                      if math.dist(raw[-1][:2], join[:2]) <= .25 else [])
                            fresh = observations.snapshot()
                            if fresh is None or not chosen:
                                navigator.get_logger().info(
                                    '[DETOUR] rejected: stale observations or planner endpoint')
                                continue
                            pose, velocity, tracks = fresh
                            tail = [p for p in reference_points[reference_index:] if lane.local(p)[0] > join_s+.01]
                            combined = chosen+tail
                            geometry_ok = (math.dist(chosen[0][:2], pose[:2]) <= .3 and
                                           forward_path_valid(combined, lane, lane.local(pose)[0], settings))
                            costmap_ok = geometry_ok and observations.path_clear(chosen, settings)
                            clearance_ok = (costmap_ok and
                                            path_clearance(combined, pose, velocity, tracks, settings) >= settings.minimum_gap)
                            if clearance_ok:
                                if not dispatch(as_path(combined)):
                                    return MissionStatus.FAILED
                                in_detour = True
                                rejoin_s = join_s
                                wait_since = clear_since = None
                                event('AVOIDING', 'validated_static_planner')
                                continue
                            navigator.get_logger().info(
                                f'[DETOUR] rejected: geometry={geometry_ok} '
                                f'costmap={costmap_ok} clearance={clearance_ok}')
                        else:
                            navigator.get_logger().info('[DETOUR] planner returned no path')

            if risk or blocked_reference:
                if not cancel():
                    return MissionStatus.FAILED
                event('YIELDING', 'no_admissible_forward_candidate')
                clear_since = None
                wait_since = now if wait_since is None else wait_since
            elif not active_task:
                # Retain the active detour after a yield; do not jump sideways
                # onto a geometrically nearest point on the reference.
                clear_since = now if clear_since is None else clear_since
                if now-clear_since >= .5:
                    if len(rest.poses) < 2 or not dispatch(rest):
                        event('FAILED', 'cannot_resume_forward_path')
                        return MissionStatus.FAILED
                    event('AVOIDING' if in_detour else 'TRACKING', 'clearance_stable_resume')
                    wait_since = clear_since = None
            else:
                if in_detour and lane is not None:
                    lateral = abs(lane.local(pose)[1])
                    # Do not resubmit the old path or reset the progress cursor.
                    progress = lane.local(pose)[0]
                    if progress >= rejoin_s and lateral < .15 and not risk:
                        in_detour, side = False, 0
                        event('TRACKING', 'forward_rejoin_completed')
                    elif progress >= rejoin_s-2.0:
                        event('REJOINING', 'following_forward_detour_to_reference')
                if now-last_plan_pub >= 1.0:
                    plan_publisher.publish(rest)
                    last_plan_pub = now
            if wait_since is not None and now-wait_since >= 15.0:
                event('FAILED', 'yield_budget_exceeded_not_proof_of_lane_blockage')
                return MissionStatus.FAILED
            time.sleep(.10)
    finally:
        cancel()
        for sub in observations.subscriptions:
            navigator.destroy_subscription(sub)
        navigator.destroy_publisher(reference_pub)
        navigator.destroy_publisher(state_pub)
    return MissionStatus.CANCELED
