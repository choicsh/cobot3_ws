"""Straight table-side docking through the normal smoother/Collision Monitor.

The authored NewRooms tables are never moved. The robot drives straight along
a long desk side with the desk on its right. Map pose controls longitudinal
position; a fitted lidar table edge checks side clearance and parallelism.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from nav_to_goal.hospital_safety import yaw_of, stamp_seconds, observation_age

DOCK_GAP = .15
HALF_WIDTH = .5
# base_link lies 0.45 m ahead of the chassis centre (front 0.48 m / rear 1.38 m).
BODY_CENTER_OFFSET = .45
STAGING_DISTANCE = 2.5
X_TOLERANCE = .02
GAP_TOLERANCE = .02
ANGLE_TOLERANCE = math.radians(1.)


def desk_side(start, end):
    """Dock beside the desk edge start->end, driving start->end, desk on the right."""
    length = math.dist(start, end)
    ux, uy = (end[0]-start[0])/length, (end[1]-start[1])/length
    lateral = HALF_WIDTH+DOCK_GAP
    mx, my = (start[0]+end[0])/2, (start[1]+end[1])/2
    dock = (mx-uy*lateral+ux*BODY_CENTER_OFFSET, my+ux*lateral+uy*BODY_CENTER_OFFSET,
            math.atan2(uy, ux))
    # At the dock the far end of the edge is this far ahead of base_link.
    return {'edge': (start, end), 'dock': dock, 'end_ahead': length/2-BODY_CENTER_OFFSET,
            'staging': (dock[0]-ux*STAGING_DISTANCE, dock[1]-uy*STAGING_DISTANCE)}


# NewRooms desks are 1.8 m (x) by 2.4 m (y); both docks use a long side.
TABLES = {
    # West side of East_DockDesk, heading north (same side as the USD spawn).
    'lab': desk_side((20.835000023841857, 12.162499952316283),
                     (20.835000023841857, 14.562499952316283)),
    # East side of West_DockDesk, heading south (the west wall is too close).
    'specimen': desk_side((-45.391000023841855, 14.562499952316283),
                          (-45.391000023841855, 12.162499952316283)),
}


def dock_errors(table, pose):
    """Remaining distance along the dock heading and leftward offset from the dock line."""
    x, y, yaw = table['dock']
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x-pose[0], y-pose[1]
    return dx*c+dy*s, dx*s-dy*c


def map_pose_from_edge(table, slope, offset, end_x):
    """Map pose of base_link from the lidar edge y=slope*x+offset and its far end x.

    The desk is fixed in the map, so this is an absolute fix beside it.
    """
    (x0, y0), (x1, y1) = table['edge']
    length = math.dist((x0, y0), (x1, y1))
    ux, uy = (x1-x0)/length, (y1-y0)/length
    angle = math.atan(slope)  # edge direction seen from the robot
    distance = -offset/math.hypot(1., slope)  # base_link to the edge line
    along = length-(end_x*math.cos(angle)+(slope*end_x+offset)*math.sin(angle))
    return (x0+ux*along-uy*distance, y0+uy*along+ux*distance,
            wrap(math.atan2(uy, ux)-angle))


def along_edge(table, x, y):
    """Coordinates along the desk edge and perpendicular distance from it."""
    (x0, y0), (x1, y1) = table['edge']
    length = math.dist((x0, y0), (x1, y1))
    ux, uy = (x1-x0)/length, (y1-y0)/length
    return (x-x0)*ux+(y-y0)*uy, (x-x0)*uy-(y-y0)*ux, length


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def table_front_envelope(points, bin_width=.025):
    """Keep the nearest table boundary, not dense arcs on its horizontal top.

    The table is to the robot's right (negative Y). Its front edge is the
    largest Y return in each longitudinal bin. RANSAC still rejects isolated
    foreground returns and corners.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(points):
        return points
    bins = np.floor(points[:, 0]/bin_width).astype(int)
    return np.array([group[np.argmax(group[:, 1])]
                     for b in np.unique(bins)
                     for group in [points[bins == b]]])


def fit_table_edge(points):
    """Fit a near-parallel line y=m*x+b, rejecting corners and isolated returns."""
    points = np.asarray(points, dtype=float)
    if len(points) < 6:
        return None
    # Bound pair enumeration, preserving the full spatial extent.
    points = points[np.argsort(points[:, 0])]
    seeds = points[np.linspace(0, len(points)-1, min(20, len(points))).astype(int)]
    best = None
    for i, a in enumerate(seeds):
        for z in seeds[i+1:]:
            if z[0]-a[0] < .5:
                continue
            slope = (z[1]-a[1])/(z[0]-a[0])
            if abs(slope) > .25:
                continue
            offset = a[1]-slope*a[0]
            mask = np.abs(points[:, 1]-slope*points[:, 0]-offset) < .025
            if mask.sum() < 6 or np.ptp(points[mask, 0]) < .5:
                continue
            if best is None or mask.sum() > best.sum():
                best = mask
    if best is None:
        return None
    slope, offset = np.polyfit(points[best, 0], points[best, 1], 1)
    if abs(slope) > .25:
        return None
    return float(slope), float(offset), int(best.sum())


def docking_command(x_error, heading_error, gap_error, direction):
    """Bounded straight-line feedback. Cruise speed elsewhere is unchanged."""
    if abs(x_error) <= .012:
        return (0., 0.) if abs(heading_error) <= math.radians(.5) else (0., math.copysign(.15, heading_error))
    speed = direction*min(.18, max(.025, .65*abs(x_error)))
    turn = 1.8*heading_error-direction*2.*gap_error
    if abs(turn) > .008:
        turn = math.copysign(min(.25, max(.15, abs(turn))), turn)
    else:
        turn = 0.
    return speed, turn


class TableDocking:
    def __init__(self, navigator, buffer):
        self.node, self.buffer = navigator, buffer
        self.scan = self.odom = self.cloud = None
        self.subscriptions = [
            navigator.create_subscription(LaserScan, '/scan_body_filtered', self._scan, qos_profile_sensor_data),
            navigator.create_subscription(PointCloud2, '/front_3d_lidar/lidar_points', self._cloud, qos_profile_sensor_data),
            navigator.create_subscription(Odometry, '/chassis/odom', self._odom, qos_profile_sensor_data),
        ]
        self.publisher = navigator.create_publisher(Twist, '/cmd_vel_nav', 10)

    def _scan(self, msg):
        self.scan = msg

    def _cloud(self, msg):
        self.cloud = msg

    def _odom(self, msg):
        self.odom = msg

    def publish(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.publisher.publish(msg)

    def observe(self, table):
        now = self.node.get_clock().now().nanoseconds/1e9
        if self.scan is None or self.odom is None or self.cloud is None:
            return None
        if (observation_age(now, stamp_seconds(self.scan.header)) is None or
                observation_age(now, stamp_seconds(self.odom.header)) is None or
                observation_age(now, stamp_seconds(self.cloud.header)) is None):
            return None
        try:
            stamped = self.buffer.lookup_transform('map', 'base_link', Time())
            if observation_age(now, stamp_seconds(stamped.header)) is None:
                return None
            t = stamped.transform
            pose = t.translation.x, t.translation.y, yaw_of(t.rotation)
            # A scan describes the robot at acquisition time. Compensate its
            # motion before fitting a centimetre-scale gap while steering.
            t = self.buffer.lookup_transform_full(
                'base_link', Time.from_msg(stamped.header.stamp),
                self.cloud.header.frame_id, Time.from_msg(self.cloud.header.stamp),
                'odom').transform
            q = t.rotation
            rotation = np.array([
                [1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w), 2*(q.x*q.z+q.y*q.w)],
                [2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z-q.x*q.w)],
                [2*(q.x*q.z-q.y*q.w), 2*(q.y*q.z+q.x*q.w), 1-2*(q.x*q.x+q.y*q.y)]])
            raw = read_points_numpy(self.cloud, field_names=('x','y','z'), skip_nans=True)
            xyz = raw.reshape(-1,3) @ rotation.T + [t.translation.x,t.translation.y,t.translation.z]
            x, y, z = xyz.T
            c, sn = math.cos(pose[2]), math.sin(pose[2])
            wx, wy = pose[0]+c*x-sn*y, pose[1]+sn*x+c*y
            # Select the desk in the robot frame (right side). AMCL drifted
            # 0.37 m beside the specimen desk in Isaac, so the map only gates
            # coarsely against other objects.
            along, across, length = along_edge(table, wx, wy)
            mask = ((y < -.3) & (y > -1.2) & (x > -1.6) & (x < 4.) &
                    (np.abs(across) < 1.) & (along > -1.) & (along < length+1.) &
                    (z > .15) & (z < .85))
            points = table_front_envelope(xyz[mask, :2])
            edge = fit_table_edge(points)
            if edge is not None:
                slope, offset, count = edge
                inliers = points[np.abs(points[:, 1]-slope*points[:, 0]-offset) < .025]
                edge = (slope, offset, count, float(inliers[:, 0].max()))
            return pose, edge, self.odom.twist.twist
        except Exception:
            return None

    def move(self, station):
        from nav_to_goal.hospital_stage_runner import drain_observations
        table = TABLES[station]
        target_yaw = table['dock'][2]
        deadline = time.monotonic()+90.
        settled = None
        unavailable = None
        last_log = 0.
        # A single edge fit once jumped ~12 deg (corner gap -0.14 m) at 0.8 m to
        # go; the robot turns at most ~1.4 deg per 0.1 s. Reject such jumps and
        # abort on a negative corner gap only if it persists.
        previous_edge = None
        overlap_since = None
        self.node.get_logger().info(
            f'[DOCK] {station} dock: pose=({table["dock"][0]:.3f}, {table["dock"][1]:.3f}), '
            f'gap={DOCK_GAP:.2f}, yaw={math.degrees(target_yaw):.0f} deg')
        try:
            if not self.align(table):
                return False
            while rclpy.ok() and time.monotonic() < deadline:
                drain_observations(self.node)
                observed = self.observe(table)
                now = time.monotonic()
                overlaps_table = False
                if observed is not None:
                    pose, edge, velocity = observed
                    progress, _, length = along_edge(table, pose[0], pose[1])
                    overlaps_table = (progress-1.38-.12 < length and progress+.48+.12 > 0.)
                    if edge is not None:
                        angle = math.atan(edge[0])
                        # 3 deg plus what a 0.3 rad/s turn could add since the last fit.
                        if (previous_edge is not None and now-previous_edge[0] < .5 and
                                abs(angle-previous_edge[1]) > math.radians(3.)+.3*(now-previous_edge[0])):
                            edge = None
                        else:
                            previous_edge = (now, angle)
                # Beside the desk the side gap must come from the lidar edge.
                if observed is None or (edge is None and overlaps_table):
                    self.publish()
                    unavailable = unavailable or now
                    if now-unavailable > 5.:
                        self.node.get_logger().error('[DOCK] fresh odom/TF/table-edge scan unavailable')
                        return False
                    time.sleep(.03)
                    continue
                unavailable = None
                dx, gap_error = dock_errors(table, pose)
                map_dx = dx
                heading_error = wrap(target_yaw-pose[2])
                gap = None
                if edge is not None:
                    slope, offset, count, end_x = edge
                    # Longitudinal error from the visible far end of the desk edge.
                    dx = end_x-table['end_ahead']
                    gap = -offset-HALF_WIDTH
                    heading_error = math.atan(slope)
                    gap_error = gap-DOCK_GAP
                    # Full chassis corners, not just centerline separation.
                    corner_gap = min(-HALF_WIDTH-slope*x-offset for x in (-1.38, .48))/math.hypot(1., slope)
                    if corner_gap > 0.:
                        overlap_since = None
                    if overlaps_table and corner_gap < .025:
                        if corner_gap <= 0.:
                            overlap_since = overlap_since or now
                            if now-overlap_since >= .5:
                                self.node.get_logger().error(f'[DOCK] nonpositive measured corner gap {corner_gap:.3f} m')
                                return False
                            self.publish()
                            time.sleep(.04)
                            continue
                        # Stop longitudinal motion and reduce the tilt that
                        # makes a corner closer than the centerline. Rotating
                        # toward a parallel pose increases this minimum gap.
                        self.publish(0., math.copysign(.15, heading_error)
                                     if abs(heading_error) > math.radians(.5) else 0.)
                        time.sleep(.04)
                        continue
                if abs(wrap(pose[2]-target_yaw)) > .18:
                    self.node.get_logger().error('[DOCK] staging heading not aligned; refusing a turn beside the table')
                    return False
                good = (abs(dx) <= X_TOLERANCE and abs(heading_error) <= ANGLE_TOLERANCE and
                        abs(velocity.linear.x) <= .01 and abs(velocity.angular.z) <= .01 and
                        gap is not None and abs(gap-DOCK_GAP) <= GAP_TOLERANCE)
                if good:
                    self.publish()
                    settled = settled or now
                    if now-settled >= .5:
                        self.node.get_logger().info(f'[DOCK] verified x_error={dx:.4f}, gap={gap}, parallel_error_deg={math.degrees(heading_error):.3f}')
                        return True
                else:
                    settled = None
                    direction = 1. if dx > 0. else -1.
                    v, w = docking_command(dx, heading_error, gap_error, direction)
                    self.publish(v, w)
                if now-last_log > 2.:
                    self.node.get_logger().info(f'[DOCK] dx={dx:.3f} map_dx={map_dx:.3f} gap={gap} angle_deg={math.degrees(heading_error):.2f}')
                    last_log = now
                time.sleep(.04)
            self.node.get_logger().error('[DOCK] timeout; docking not successful')
            return False
        finally:
            self.publish()

    def relocalize(self, station, timeout=10.):
        """Seed AMCL from the lidar pose beside a fixed desk while stopped.

        AMCL drifted 0.39 m / 8 deg into the specimen desk in Isaac, which made
        the local costmap put the footprint inside the desk.
        """
        from nav_to_goal.hospital_stage_runner import drain_observations
        table = TABLES[station]
        deadline = time.monotonic()+timeout
        publisher = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        target = None
        last_publish = 0.
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                drain_observations(self.node)
                observed = self.observe(table)
                if observed is not None and observed[1] is not None:
                    pose, edge, velocity = observed
                    if abs(velocity.linear.x) <= .01 and abs(velocity.angular.z) <= .01:
                        target = map_pose_from_edge(table, *edge[:2], edge[3])
                        if (math.dist(pose[:2], target[:2]) <= .03 and
                                abs(wrap(pose[2]-target[2])) <= math.radians(1.)):
                            self.node.get_logger().info(
                                f'[DOCK] AMCL at lidar pose ({target[0]:.3f}, {target[1]:.3f}, '
                                f'{math.degrees(target[2]):.1f} deg)')
                            return True
                        if time.monotonic()-last_publish >= 1.:
                            msg = PoseWithCovarianceStamped()
                            msg.header.stamp = self.node.get_clock().now().to_msg()
                            msg.header.frame_id = 'map'
                            msg.pose.pose.position.x, msg.pose.pose.position.y = target[:2]
                            msg.pose.pose.orientation.z = math.sin(target[2]/2)
                            msg.pose.pose.orientation.w = math.cos(target[2]/2)
                            msg.pose.covariance[0] = msg.pose.covariance[7] = .02**2
                            msg.pose.covariance[35] = math.radians(1.)**2
                            publisher.publish(msg)
                            last_publish = time.monotonic()
                            self.node.get_logger().info(
                                f'[DOCK] AMCL reset from lidar: map=({pose[0]:.3f}, {pose[1]:.3f}, '
                                f'{math.degrees(pose[2]):.1f}) -> ({target[0]:.3f}, {target[1]:.3f}, '
                                f'{math.degrees(target[2]):.1f})')
                time.sleep(.05)
            self.node.get_logger().warn(f'[DOCK] AMCL relocalization at {station} not confirmed')
            return False
        finally:
            self.node.destroy_publisher(publisher)

    def align(self, table):
        """Turn only at the open staging point, before approaching the desk."""
        from nav_to_goal.hospital_stage_runner import drain_observations
        deadline = time.monotonic()+35.
        stable = None
        while rclpy.ok() and time.monotonic() < deadline:
            drain_observations(self.node)
            observed = self.observe(table)
            if observed is None:
                self.publish()
                time.sleep(.04)
                continue
            pose, edge, velocity = observed
            # Never run a staging turn after reaching the table itself.
            if math.dist(pose[:2], table['staging']) > .55:
                self.node.get_logger().error('[DOCK] alignment requires the open staging point')
                return False
            # The desk starts ahead of the staging point; use map heading until
            # its edge is in view.
            error = math.atan(edge[0]) if edge is not None else wrap(table['dock'][2]-pose[2])
            if abs(error) <= ANGLE_TOLERANCE:
                self.publish()
                if abs(velocity.angular.z) <= .01:
                    stable = stable or time.monotonic()
                    if time.monotonic()-stable >= .3:
                        return True
            else:
                stable = None
                # The legacy front slowdown scales yaw to 40%. DWB's 0.03
                # rad/s sample then cannot overcome stationary friction.
                # Short bounded turns at the staging point avoid that deadband.
                self.publish(0., math.copysign(min(.3, max(.15, 1.8*abs(error))), error))
            time.sleep(.04)
        self.publish()
        self.node.get_logger().error('[DOCK] staging alignment failed')
        return False

    def close(self):
        self.publish()
        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        self.node.destroy_publisher(self.publisher)
