"""Isaac Sim 전용 ground-truth 위치 추정: map -> odom 을 정적 TF 로 발행한다.

Isaac 의 odom(IsaacComputeOdometry)은 물리 엔진 실제 자세에서 계산되어 drift 가 없고,
Play 시점 자세를 원점으로 삼는다. 그리고 Isaac 월드 좌표 == 맵 좌표이므로
map -> odom 은 run_nova_sim.py 가 기록한 Play 시점 자세(start_pose.json) 그 자체다.

AMCL 대신 이걸 쓰면 위치/각도 오차가 0 이 된다 (AMCL 은 책상 옆에서 30cm / 7도까지 틀어졌다).
실기에서는 쓸 수 없으니 launch 인자 localization:=amcl 로 되돌린다.
"""
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

START_POSE_PATH = os.path.expanduser('~/cobot3_ws/isaacpjt/assets/start_pose.json')


class GroundTruthLocalization(Node):
    def __init__(self):
        super().__init__('ground_truth_localization')
        self.declare_parameter('start_pose_path', START_POSE_PATH)
        path = os.path.expanduser(self.get_parameter('start_pose_path').value)
        with open(path) as f:
            sp = json.load(f)
        x, y, yaw = float(sp['x']), float(sp['y']), math.radians(float(sp['yaw_deg']))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(t)
        self.get_logger().info(
            f'map->odom 정적 TF 발행 (Isaac ground truth): x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg'
        )


def main():
    rclpy.init(args=sys.argv)
    node = GroundTruthLocalization()
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
