"""Isaac Sim(run_nova_sim.py)이 기록한 start_pose.json 을 읽어 AMCL 에 /initialpose 로 한 번 넘기고 끝난다.

씬에서 로봇 시작점을 옮겨도 params.yaml 이나 주행 스크립트의 숫자를 고칠 필요가 없게 하기 위한 노드.
"""
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState

START_POSE_PATH = os.path.expanduser('~/cobot3_ws/isaacpjt/assets/start_pose.json')


class InitialPoseFromSim(Node):
    def __init__(self):
        super().__init__('initial_pose_from_sim')
        self.declare_parameter('start_pose_path', START_POSE_PATH)
        self.declare_parameter('localizer', 'amcl')
        self.path = os.path.expanduser(self.get_parameter('start_pose_path').value)
        self.localizer = self.get_parameter('localizer').value

        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.received = False
        # nav2 AMCL 은 amcl_pose 를 transient_local 로 내보낸다 (BasicNavigator 와 같은 QoS).
        amcl_pose_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose, amcl_pose_qos)

    def _on_amcl_pose(self, _msg):
        self.received = True

    def load_pose(self):
        if not os.path.isfile(self.path):
            self.get_logger().warn(
                f'{self.path} 없음 — Isaac Sim(run_nova_sim.py)을 먼저 띄우면 생깁니다. '
                'RViz 의 2D Pose Estimate 로 직접 찍으세요.'
            )
            return None
        with open(self.path) as f:
            sp = json.load(f)
        return float(sp['x']), float(sp['y']), float(sp['yaw_deg'])

    def wait_for_localizer_active(self):
        client = self.create_client(GetState, f'{self.localizer}/get_state')
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() is not None and future.result().current_state.label == 'active':
                    return
            self.get_logger().info(f'{self.localizer} 활성화 대기 중...', throttle_duration_sec=5.0)

    def wait_for_clock(self):
        # use_sim_time 이면 /clock 이 오기 전까지 now() 가 0 이라 stamp 가 무효가 된다.
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.2)

    def publish_until_accepted(self, x, y, yaw_deg):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        yaw = math.radians(yaw_deg)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = math.radians(10.0) ** 2

        self.get_logger().info(f'초기 위치 전송: x={x:.3f} y={y:.3f} yaw={yaw_deg:.1f}deg')
        while rclpy.ok() and not self.received:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info('AMCL 이 초기 위치를 받았습니다.')


def main():
    rclpy.init(args=sys.argv)
    node = InitialPoseFromSim()
    try:
        pose = node.load_pose()
        if pose is None:
            return
        node.wait_for_localizer_active()
        node.wait_for_clock()
        node.publish_until_accepted(*pose)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
