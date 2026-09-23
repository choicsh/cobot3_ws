"""Convert compensated scan motion into expiring local prediction envelopes."""
import math

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import ChannelFloat32, LaserScan, PointCloud
from tf2_ros import Buffer, TransformException, TransformListener

from nav_to_goal.obstacle_tracking import Tracker, scan_clusters


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
        self.publisher = self.create_publisher(PointCloud, "/hospital/predicted_obstacles", qos_profile_sensor_data)
        # Keep the existing radius-only cloud contract for the C++ soft layer.
        # A separate current-state stream preserves time/identity for supervision.
        self.track_publisher = self.create_publisher(
            PointCloud, "/hospital/tracked_obstacles", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_body_filtered", self.scan, qos_profile_sensor_data)

    def scan(self, msg):
        try:
            t = self.buffer.lookup_transform("odom", msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        p, q = t.transform.translation, t.transform.rotation
        r00, r01 = 1 - 2*(q.y*q.y + q.z*q.z), 2*(q.x*q.y - q.z*q.w)
        r10, r11 = 2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x*q.x + q.z*q.z)
        points = []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not msg.range_min <= distance <= min(self.tracking_range, msg.range_max):
                points.append(None)
                continue
            angle = msg.angle_min + i*msg.angle_increment
            x, y = distance*math.cos(angle), distance*math.sin(angle)
            points.append((p.x + r00*x + r01*y, p.y + r10*x + r11*y))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        self.tracker.update(scan_clusters(points), stamp)
        predictions = self.tracker.predictions(stamp, horizon=self.horizon, radius=self.radius)
        cloud = PointCloud()
        cloud.header = msg.header
        cloud.header.frame_id = "odom"
        cloud.points = [Point32(x=x, y=y, z=0.0) for x, y, _ in predictions]
        cloud.channels = [ChannelFloat32(name="radius", values=[r for _, _, r in predictions])]
        self.publisher.publish(cloud)  # Also send empty clouds to clear old envelopes.
        snapshots = self.tracker.snapshots(stamp, radius=self.radius)
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
