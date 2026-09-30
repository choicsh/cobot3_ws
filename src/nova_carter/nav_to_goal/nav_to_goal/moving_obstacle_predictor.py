"""Convert compensated scan motion into expiring local prediction envelopes."""
import math

import rclpy
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import ChannelFloat32, LaserScan, PointCloud
from tf2_ros import Buffer, TransformException, TransformListener

from nav_to_goal.obstacle_tracking import Tracker, scan_clusters
from nav_to_goal.static_scan_mask import StaticScanMask


class MovingObstaclePredictor(Node):
    def __init__(self):
        super().__init__("hospital_moving_obstacle_predictor")
        self.declare_parameter("prediction_horizon", 1.8)
        self.declare_parameter("body_radius", 0.4)
        self.declare_parameter("tracking_range", 8.0)
        self.horizon = self.get_parameter("prediction_horizon").value
        self.radius = self.get_parameter("body_radius").value
        self.tracking_range = self.get_parameter("tracking_range").value
        if not 0 < self.horizon <= 3 or not 0.1 <= self.radius <= 0.8:
            raise ValueError("Invalid prediction horizon/body radius")
        if not 1.0 <= self.tracking_range <= 12.0:
            raise ValueError("Invalid tracking range")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tracker = Tracker()
        self.static_mask = None
        self.standing_mask = None
        self.create_subscription(OccupancyGrid, 'map', self.receive_map,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.last_tf_warning = -math.inf
        sensor_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(PointCloud, "hospital/predicted_obstacles", sensor_qos)
        # Keep the existing radius-only cloud contract for the C++ soft layer.
        # A separate current-state stream preserves time/identity for supervision.
        self.track_publisher = self.create_publisher(
            PointCloud, "hospital/tracked_obstacles", sensor_qos)
        self.create_subscription(LaserScan, "scan_body_filtered", self.scan, sensor_qos)

    def receive_map(self, msg):
        if msg.header.frame_id == 'map':
            self.static_mask = StaticScanMask(msg)
            # A standing object counts as a person only this far from mapped
            # structure: covers AMCL error beside the dock desk (gap 0.15 m).
            self.standing_mask = StaticScanMask(msg, margin=.45)

    def scan(self, msg):
        try:
            t = self.buffer.lookup_transform("odom", msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException as error:
            now = self.get_clock().now().nanoseconds/1e9
            if now < self.last_tf_warning or now-self.last_tf_warning >= 2.0:
                self.get_logger().warn(f"Scan dropped: odom transform unavailable: {error}")
                self.last_tf_warning = now
            return
        p, q = t.transform.translation, t.transform.rotation
        r00, r01 = 1 - 2*(q.y*q.y + q.z*q.z), 2*(q.x*q.y - q.z*q.w)
        r10, r11 = 2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x*q.x + q.z*q.z)
        map_transform = None
        if self.static_mask is not None:
            try:
                transform = self.buffer.lookup_transform('map', 'odom', Time()).transform
                qmap = transform.rotation
                yaw = math.atan2(2*(qmap.w*qmap.z+qmap.x*qmap.y),
                                 1-2*(qmap.y*qmap.y+qmap.z*qmap.z))
                map_transform = (transform.translation.x, transform.translation.y,
                                 math.cos(yaw), math.sin(yaw))
            except TransformException:
                pass
        points = []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not msg.range_min <= distance <= min(self.tracking_range, msg.range_max):
                points.append(None)
                continue
            angle = msg.angle_min + i*msg.angle_increment
            x, y = distance*math.cos(angle), distance*math.sin(angle)
            ox, oy = p.x+r00*x+r01*y, p.y+r10*x+r11*y
            if map_transform is not None:
                mx, my, c, s = map_transform
                if self.static_mask.contains(mx+c*ox-s*oy, my+s*ox+c*oy):
                    points.append(None)
                    continue
            points.append((ox, oy))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        self.tracker.update(scan_clusters(points), stamp)
        predictions = self.tracker.predictions(stamp, horizon=self.horizon, radius=self.radius)
        cloud = PointCloud()
        cloud.header = msg.header
        cloud.header.frame_id = "odom"
        cloud.points = [Point32(x=x, y=y, z=0.0) for x, y, _ in predictions]
        cloud.channels = [ChannelFloat32(name="radius", values=[r for _, _, r in predictions])]
        self.publisher.publish(cloud)  # Also send empty clouds to clear old envelopes.
        standing_ok = None
        if map_transform is not None and self.standing_mask is not None:
            mx, my, c, s = map_transform
            standing_ok = lambda x, y: not self.standing_mask.contains(mx+c*x-s*y, my+s*x+c*y)
        snapshots = self.tracker.snapshots(stamp, radius=self.radius, standing_ok=standing_ok)
        state = PointCloud()
        state.header = cloud.header
        state.points = [Point32(x=t[1], y=t[2], z=0.0) for t in snapshots]
        state.channels = [ChannelFloat32(name=name, values=[float(t[index]) for t in snapshots])
                          for name, index in (("track_id", 0), ("vx", 3), ("vy", 4),
                                              ("radius", 5), ("observation_age", 6))]
        self.track_publisher.publish(state)


def main():
    rclpy.init()
    node = MovingObstaclePredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
