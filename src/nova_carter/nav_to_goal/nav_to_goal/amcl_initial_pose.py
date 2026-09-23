"""Seed hospital AMCL from Isaac's start pose without publishing map -> odom."""

import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


START_POSE_PATH = os.path.expanduser('~/cobot3_ws/isaacpjt/assets/start_pose.json')


def map_pose_from_start_and_odom(start_x, start_y, start_yaw, odom_to_base):
    """Compose Isaac's start map pose with its current odom -> base_link pose."""
    p, q = odom_to_base.translation, odom_to_base.rotation
    odom_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    c, s = math.cos(start_yaw), math.sin(start_yaw)
    return (
        start_x + c * p.x - s * p.y,
        start_y + s * p.x + c * p.y,
        start_yaw + odom_yaw,
    )


class AmclInitialPose(Node):
    def __init__(self):
        super().__init__('hospital_amcl_initial_pose')
        self.declare_parameter('start_pose_path', START_POSE_PATH)
        path = os.path.expanduser(self.get_parameter('start_pose_path').value)
        with open(path, encoding='utf-8') as pose_file:
            start = json.load(pose_file)
        if start.get('frame') != 'map':
            raise ValueError(f'{path}: start pose must be in the map frame')
        self.start_x = float(start['x'])
        self.start_y = float(start['y'])
        self.start_yaw = math.radians(float(start['yaw_deg']))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_received, 10)
        self.timer = self.create_timer(1.0, self._try_initial_pose)
        self.last_published_ns = 0
        self.initialized = False

    def _try_initial_pose(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.initialized or now_ns == 0 or self.publisher.get_subscription_count() == 0:
            return
        if self.last_published_ns and now_ns - self.last_published_ns < 3_000_000_000:
            return
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link', Time()).transform
        except TransformException as error:
            self.get_logger().warn(f'AMCL initial pose waiting for odom -> base_link: {error}')
            return

        # Isaac's odom starts at the Play pose. If navigation starts later, seed
        # AMCL at the robot's *current* pose rather than its recorded start pose.
        x, y, yaw = map_pose_from_start_and_odom(
            self.start_x, self.start_y, self.start_yaw, odom_to_base)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Isaac supplies the actual spawn pose, not a user's coarse RViz click.
        # A 0.5 m seed spread can put particles inside the adjacent dock desk.
        # This describes seed uncertainty, not measured scan-map accuracy.
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02 ** 2
        msg.pose.covariance[35] = math.radians(1.0) ** 2
        self.publisher.publish(msg)
        self.last_published_ns = now_ns
        self.get_logger().info(
            f'AMCL initial pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg')

    def _pose_received(self, msg):
        if self.last_published_ns and not self.initialized:
            self.initialized = True
            self.timer.cancel()
            self.get_logger().info('AMCL pose received; initial-pose publishing stopped')


def main():
    rclpy.init(args=sys.argv)
    node = AmclInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
