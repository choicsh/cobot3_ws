"""Straight table-side docking through the normal smoother/Collision Monitor.

The authored NewRooms tables are never moved. Map pose controls longitudinal
position; a fitted lidar table edge checks side clearance and parallelism.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from nav_to_goal.hospital_safety import yaw_of, stamp_seconds, observation_age

TABLES = {
    'lab': {'x_min': 20.835000023841857, 'x_max': 22.63499997615814,
            'south': 12.162499952316283, 'dock_x': 21.285, 'staging_x': 18.7},
    'specimen': {'x_min': -47.19099997615814, 'x_max': -45.391000023841855,
                 'south': 12.162499952316283, 'dock_x': -46.741, 'staging_x': -43.4},
}
DOCK_GAP = .05
HALF_WIDTH = .5


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
            mask = ((wx >= table['x_min']+.06) & (wx <= table['x_max']-.06) &
                    (np.abs(wy-table['south']) < .35) & (y < -.3) &
                    (z > .15) & (z < .85))
            points = table_front_envelope(xyz[mask, :2])
            return pose, fit_table_edge(points), self.odom.twist.twist
        except Exception:
            return None

    def move(self, station, undock=False):
        from nav_to_goal.hospital_stage_runner import drain_observations
        table = TABLES[station]
        target_x = table['staging_x'] if undock else table['dock_x']
        target_y = table['south']-HALF_WIDTH-DOCK_GAP
        deadline = time.monotonic()+90.
        settled = None
        unavailable = None
        last_log = 0.
        self.node.get_logger().info(f'[DOCK] {station} {"undock" if undock else "dock"}: x={target_x:.3f}, gap=0.05, yaw=180 deg')
        try:
            if not undock and not self.align(table):
                return False
            while rclpy.ok() and time.monotonic() < deadline:
                drain_observations(self.node)
                observed = self.observe(table)
                now = time.monotonic()
                if observed is None or (observed[1] is None and not undock):
                    self.publish()
                    unavailable = unavailable or now
                    if now-unavailable > 5.:
                        self.node.get_logger().error('[DOCK] fresh odom/TF/table-edge scan unavailable')
                        return False
                    time.sleep(.03)
                    continue
                unavailable = None
                pose, edge, velocity = observed
                dx = target_x-pose[0]
                heading_error = wrap(math.pi-pose[2])
                gap_error = target_y-pose[1]
                gap = None
                if edge is not None:
                    slope, offset, count = edge
                    gap = -offset-HALF_WIDTH
                    heading_error = math.atan(slope)
                    gap_error = gap-DOCK_GAP
                    # Full chassis corners, not just centerline separation.
                    corner_gap = min(-HALF_WIDTH-slope*x-offset for x in (-1.38, .48))/math.hypot(1., slope)
                    # Before the chassis reaches the table's X interval, use
                    # the open staging space to correct lateral/heading error.
                    overlaps_table = (pose[0]-.6 < table['x_max'] and
                                      pose[0]+1.5 > table['x_min'])
                    if overlaps_table and corner_gap < .025 and not undock:
                        if corner_gap <= 0.:
                            self.node.get_logger().error(f'[DOCK] nonpositive measured corner gap {corner_gap:.3f} m')
                            return False
                        # Stop longitudinal motion and reduce the tilt that
                        # makes a corner closer than the centerline. Rotating
                        # toward a parallel pose increases this minimum gap.
                        self.publish(0., math.copysign(.15, heading_error)
                                     if abs(heading_error) > math.radians(.5) else 0.)
                        time.sleep(.04)
                        continue
                if abs(wrap(pose[2]-math.pi)) > .18:
                    self.node.get_logger().error('[DOCK] staging heading not aligned; refusing a turn beside the table')
                    return False
                good = (abs(dx) <= .015 and abs(heading_error) <= math.radians(.5) and
                        abs(velocity.linear.x) <= .01 and abs(velocity.angular.z) <= .01 and
                        (undock or (gap is not None and abs(gap-DOCK_GAP) <= .01)))
                if good:
                    self.publish()
                    settled = settled or now
                    if now-settled >= .5:
                        self.node.get_logger().info(f'[DOCK] verified x_error={dx:.4f}, gap={gap}, parallel_error_deg={math.degrees(heading_error):.3f}')
                        return True
                else:
                    settled = None
                    # Heading pi: forward moves toward smaller world X.
                    direction = -1. if dx > 0. else 1.
                    v, w = docking_command(dx, heading_error, gap_error, direction)
                    self.publish(v, w)
                if now-last_log > 2.:
                    self.node.get_logger().info(f'[DOCK] dx={dx:.3f} gap={gap} angle_deg={math.degrees(heading_error):.2f}')
                    last_log = now
                time.sleep(.04)
            self.node.get_logger().error('[DOCK] timeout; docking not successful')
            return False
        finally:
            self.publish()

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
            if abs(pose[0]-table['staging_x']) > .55:
                self.node.get_logger().error('[DOCK] alignment requires the open staging point')
                return False
            error = math.atan(edge[0]) if edge is not None else wrap(math.pi-pose[2])
            if edge is not None and abs(error) <= math.radians(.5):
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
