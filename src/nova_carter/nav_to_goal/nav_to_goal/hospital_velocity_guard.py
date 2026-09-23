"""Predictive human check between smoother and Collision Monitor (Twist/Jazzy).

Only scales existing commands or stops; never invents an escape motion. The
mission's forward-path selector is responsible for active lateral avoidance.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from nav_to_goal.hospital_avoidance import SafetySettings, limited_command
from nav_to_goal.hospital_safety import SafetyObservations


class HospitalVelocityGuard(Node):
    def __init__(self):
        super().__init__('hospital_velocity_guard')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.observations = SafetyObservations(self, self.buffer)
        self.settings = SafetySettings()
        self.command, self.command_stamp = (0., 0.), -math.inf
        self.previous_state = None
        self.last_time = None
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.receive, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_human_checked', 10)
        self.state_publisher = self.create_publisher(String, '/hospital/human_guard_state', 10)
        self.create_timer(.05, self.tick)

    def receive(self, msg):
        self.command = (msg.linear.x, msg.angular.z)
        self.command_stamp = self.observations.now()

    def tick(self):
        now = self.observations.now()
        if self.last_time is not None and now < self.last_time:
            self.command_stamp = -math.inf
            self.observations.tracks = self.observations.odom = None
        self.last_time = now
        command, state, gap = (0., 0.), 'STALE_COMMAND', float('nan')
        snapshot = self.observations.snapshot(frame='odom')
        if not all(math.isfinite(v) for v in self.command):
            state = 'INVALID_COMMAND'
        elif 0 <= now-self.command_stamp <= .5:
            if snapshot is None:
                state = 'STALE_OBSERVATION'
            else:
                pose, velocity, tracks = snapshot
                command, state, gap = limited_command(pose, velocity, self.command, tracks, self.settings)
        msg = Twist()
        msg.linear.x, msg.angular.z = command
        self.publisher.publish(msg)
        if state != self.previous_state:
            event = f'{state} predicted_gap={gap:.3f}'
            self.state_publisher.publish(String(data=event))
            self.get_logger().info(event)
            self.previous_state = state


def main():
    rclpy.init()
    node = HospitalVelocityGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
