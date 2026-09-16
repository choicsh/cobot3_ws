"""Detect a blue or green cube in the M0609 wrist-camera image."""

import os
import time

import cv2

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image

from std_msgs.msg import Int32


IMAGE_TOPIC = '/rgb'
COLOR_TOPIC = '/color_id'


class M0609ColorDetector(Node):
    """Classify the largest configured HSV color region in each image."""

    def __init__(self):
        """Create ROS interfaces and HSV detector parameters."""
        super().__init__('m0609_color_detector')

        self.declare_parameter('min_area', 500)
        self.declare_parameter('show_preview', True)
        self.declare_parameter('log_interval_sec', 1.0)

        self._min_area = int(self.get_parameter('min_area').value)
        preview_requested = bool(self.get_parameter('show_preview').value)
        has_display = bool(os.environ.get('DISPLAY'))
        self._show_preview = preview_requested and has_display
        log_interval = self.get_parameter('log_interval_sec').value
        self._log_interval = float(log_interval)
        self._last_log_time = 0.0
        self._last_color_id = None
        self._bridge = CvBridge()

        self._publisher = self.create_publisher(Int32, COLOR_TOPIC, 10)
        self._subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self._image_callback,
            qos_profile_sensor_data,
        )

        if preview_requested and not self._show_preview:
            self.get_logger().warning(
                'DISPLAY가 없어 OpenCV 미리보기를 비활성화합니다.'
            )
        self.get_logger().info(
            f'구독 {IMAGE_TOPIC} -> 발행 {COLOR_TOPIC}, '
            f'min_area={self._min_area}'
        )

    @staticmethod
    def _mask_area(hsv_image, lower, upper):
        mask = cv2.inRange(
            hsv_image,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return int(cv2.countNonZero(mask))

    def _image_callback(self, message):
        try:
            bgr_image = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except CvBridgeError as error:
            self.get_logger().error(f'이미지 변환 실패: {error}')
            return

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # OpenCV hue range is 0..179.
        blue_area = self._mask_area(
            hsv_image, (100, 80, 50), (140, 255, 255)
        )
        green_area = self._mask_area(
            hsv_image, (40, 80, 50), (85, 255, 255)
        )

        color_id = 0
        label = 'NONE'
        if blue_area >= self._min_area and blue_area > green_area:
            color_id = 1
            label = 'BLUE'
        elif green_area >= self._min_area and green_area > blue_area:
            color_id = 2
            label = 'GREEN'

        self._publisher.publish(Int32(data=color_id))

        now = time.monotonic()
        color_changed = color_id != self._last_color_id
        if color_changed or now - self._last_log_time >= self._log_interval:
            self.get_logger().info(
                f'{label} color_id={color_id} '
                f'blue_area={blue_area} green_area={green_area}'
            )
            self._last_color_id = color_id
            self._last_log_time = now

        if self._show_preview:
            preview = bgr_image.copy()
            cv2.putText(
                preview,
                f'{label}  color_id={color_id}',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            try:
                cv2.imshow('M0609 Color Detector', preview)
                cv2.waitKey(1)
            except cv2.error as error:
                self._show_preview = False
                self.get_logger().warning(
                    f'OpenCV 미리보기를 비활성화합니다: {error}'
                )

    def destroy_node(self):
        """Close the optional preview before destroying the ROS node."""
        if self._show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    """Run the M0609 color detector until ROS shutdown."""
    rclpy.init(args=args)
    node = M0609ColorDetector()
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
