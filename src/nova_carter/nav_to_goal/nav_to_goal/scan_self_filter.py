"""Remove robot-body returns for Collision Monitor, preserving exterior returns.

Costmaps retain their original /scan and footprint clearing. This filter has no
human labels or simulator ground truth. Missing transforms produce no scan, so
Collision Monitor's source timeout stops the robot rather than using bad data.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def inside_body(x, y, bounds):
    return bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__("hospital_scan_self_filter")
        self.declare_parameter("body_bounds", [-1.38, 0.48, -0.5, 0.5])
        self.declare_parameter("base_frame", "base_link")
        self.bounds = self.get_parameter("body_bounds").value
        if len(self.bounds) != 4 or self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3]:
            raise ValueError("body_bounds must be [xmin, xmax, ymin, ymax]")
        self.base_frame = self.get_parameter("base_frame").value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(LaserScan, "scan_body_filtered", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self.scan, qos_profile_sensor_data)
        self.reported = False

    def scan(self, msg):
        try:
            # The lidar is rigidly mounted to base_link. Latest extrinsics avoid
            # waiting for the unrelated map/odom transform in this callback.
            transform = self.buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except TransformException:
            return
        q, p = transform.transform.rotation, transform.transform.translation
        r00, r01 = 1 - 2*(q.y*q.y + q.z*q.z), 2*(q.x*q.y - q.z*q.w)
        r10, r11 = 2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x*q.x + q.z*q.z)
        removed = 0
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not msg.range_min <= distance <= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x, y = distance * math.cos(angle), distance * math.sin(angle)
            if inside_body(p.x + r00*x + r01*y, p.y + r10*x + r11*y, self.bounds):
                msg.ranges[i] = math.nan
                removed += 1
        self.publisher.publish(msg)
        if not self.reported:
            self.get_logger().info(f"Self filter active: {removed} body returns removed; bounds={self.bounds}")
            self.reported = True


def main():
    rclpy.init()
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
