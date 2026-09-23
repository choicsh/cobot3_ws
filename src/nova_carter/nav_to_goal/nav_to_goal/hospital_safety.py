"""Read-only ROS observations shared by mission and velocity guard."""
import math

from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.msg import Costmap
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud

from nav_to_goal.hospital_avoidance import MovingBody, SafetySettings, wrap
from nav_to_goal.hospital_costmap import Grid


LATEST_SENSOR_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp_seconds(header):
    return header.stamp.sec+header.stamp.nanosec/1e9




class SafetyObservations:
    def __init__(self, node, buffer, with_maps=False):
        self.node, self.buffer = node, buffer
        self.tracks = self.odom = self.local = self.static = None
        self.subscriptions = [
            node.create_subscription(PointCloud, '/hospital/tracked_obstacles',
                                     self._tracks, LATEST_SENSOR_QOS),
            node.create_subscription(Odometry, '/chassis/odom', self._odom, LATEST_SENSOR_QOS),
        ]
        if with_maps:
            self.subscriptions += [
                node.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                         self._local, LATEST_SENSOR_QOS),
                node.create_subscription(OccupancyGrid, '/map', self._map,
                    QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)),
            ]

    def _tracks(self, msg):
        self.tracks = msg

    def _odom(self, msg):
        self.odom = msg

    def _local(self, msg):
        self.local = Grid(msg, raw=True)

    def _map(self, msg):
        self.static = Grid(msg)

    def now(self):
        return self.node.get_clock().now().nanoseconds/1e9

    def transform_pose(self, pose, source, target):
        if source == target:
            return pose
        transform = self.buffer.lookup_transform(target, source, Time()).transform
        angle = yaw_of(transform.rotation)
        c, s = math.cos(angle), math.sin(angle)
        return (transform.translation.x+c*pose[0]-s*pose[1],
                transform.translation.y+s*pose[0]+c*pose[1], wrap(pose[2]+angle))

    def snapshot(self, frame='map'):
        """None means unavailable/stale, whereas [] is a fresh empty track scan."""
        now = self.now()
        if self.tracks is None or self.odom is None:
            return None
        track_age, odom_age = now-stamp_seconds(self.tracks.header), now-stamp_seconds(self.odom.header)
        if not (0 <= track_age <= .6 and 0 <= odom_age <= .6):
            return None
        try:
            stamped = self.buffer.lookup_transform(frame, 'base_link', Time())
            if not 0 <= now-stamp_seconds(stamped.header) <= .6:
                return None
            transform = stamped.transform
            pose = (transform.translation.x, transform.translation.y, yaw_of(transform.rotation))
            channels = {c.name: c.values for c in self.tracks.channels}
            required = ('track_id', 'vx', 'vy', 'radius', 'observation_age')
            if any(name not in channels or len(channels[name]) != len(self.tracks.points)
                   for name in required):
                return None
            tracks = []
            # Rotation of velocity is distinct from transforming position.
            zero = self.transform_pose((0., 0., 0.), self.tracks.header.frame_id, frame)
            c, s = math.cos(zero[2]), math.sin(zero[2])
            for i, p in enumerate(self.tracks.points):
                vx, vy = channels['vx'][i], channels['vy'][i]
                radius, observed_age = channels['radius'][i], channels['observation_age'][i]
                vals = (p.x, p.y, vx, vy, radius, observed_age, channels['track_id'][i])
                if not all(math.isfinite(v) for v in vals) or not (0 < radius <= .8 and 0 <= observed_age <= .6):
                    return None
                position = self.transform_pose((p.x+vx*track_age, p.y+vy*track_age, 0.),
                                               self.tracks.header.frame_id, frame)
                tracks.append(MovingBody(int(channels['track_id'][i]), *position[:2],
                                         c*vx-s*vy, s*vx+c*vy, radius, track_age+observed_age))
            twist = self.odom.twist.twist
            # Twist is expressed at child_frame_id, not header.frame_id. Preserve
            # existing robot frame names, applying rigid-link lever-arm correction.
            child = self.odom.child_frame_id.lstrip('/')
            if not child:
                return None
            vx, vy, wz = twist.linear.x, twist.linear.y, twist.angular.z
            if child != 'base_link':
                relative = self.buffer.lookup_transform('base_link', child, Time()).transform
                angle = yaw_of(relative.rotation)
                c, s = math.cos(angle), math.sin(angle)
                vx, vy = (c*vx-s*vy+wz*relative.translation.y,
                          s*vx+c*vy-wz*relative.translation.x)
            velocity = (vx, wz)
            if not all(math.isfinite(v) for v in (*pose, *velocity)):
                return None
            return pose, velocity, tracks
        except Exception:
            return None

    def path_clear(self, path, settings=SafetySettings()):
        if self.static is None or self.local is None or not 0 <= self.now()-self.local.stamp <= 1.0:
            return False
        try:
            # Resolve each frame once for the entire candidate. Thousands of
            # per-pose TF lookups can starve this node's sensor callbacks.
            static_tf = self.transform_pose((0., 0., 0.), 'map', self.static.frame)
            local_tf = self.transform_pose((0., 0., 0.), 'map', self.local.frame)

            def apply(pose, transform):
                c, s = math.cos(transform[2]), math.sin(transform[2])
                return (transform[0]+c*pose[0]-s*pose[1],
                        transform[1]+s*pose[0]+c*pose[1], wrap(pose[2]+transform[2]))

            for pose in path:
                if not self.static.body_clear(apply(pose, static_tf), settings):
                    return False
                if not self.local.body_clear(apply(pose, local_tf), settings,
                                             require_inside=False):
                    return False
        except Exception:
            return False
        return True
