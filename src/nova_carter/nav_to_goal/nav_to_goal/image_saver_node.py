import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class SingleImageSaver(Node):
    def __init__(
        self,
        topic_name: str = '/front_stereo_camera/left/image_raw',
        save_path: str = '~/saved_images/single_frame.png',
    ):
        super().__init__('single_image_saver')

        # 경로 및 디렉터리 준비
        self.save_path = os.path.expanduser(save_path)
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)

        self.bridge = CvBridge()
        self.is_saved = False

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )

        self.get_logger().info(f"1프레임 수신 대기 중: {topic_name}")

    def image_callback(self, msg: Image):
        # 이미 저장한 경우 추가 저장 방지
        if self.is_saved:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge 변환 에러: {e}")
            return

        # 1프레임 저장
        success = cv2.imwrite(self.save_path, cv_image)
        if success:
            self.is_saved = True
            self.get_logger().info(f"1프레임 저장 성공: {self.save_path}")
        else:
            self.get_logger().warn(f"파일 저장 실패: {self.save_path}")


def capture_single_frame(
    topic_name: str = '/front_stereo_camera/left/image_raw',
    save_path: str = '~/saved_images/single_frame.png',
    timeout_sec: float = 5.0
) -> bool:
    """
    다른 코드에서 1프레임을 저장하기 위해 호출하는 함수.
    
    :param topic_name: 구독할 이미지 토픽
    :param save_path: 저장할 파일 전체 경로 (파일명 포함)
    :param timeout_sec: 토픽 수신 최대 대기 시간(초)
    :return: 저장 성공 여부 (bool)
    """
    should_shutdown = False
    if not rclpy.ok():
        rclpy.init()
        should_shutdown = True

    node = SingleImageSaver(topic_name=topic_name, save_path=save_path)

    start_time = node.get_clock().now()
    saved = False

    try:
        # 프레임이 저장되거나 타임아웃이 날 때까지 1회씩 spin
        while rclpy.ok() and not node.is_saved:
            rclpy.spin_once(node, timeout_sec=0.1)

            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                node.get_logger().warn(f"[{timeout_sec}초] 타임아웃: 토픽 메시지를 받지 못했습니다.")
                break

        saved = node.is_saved

    finally:
        node.destroy_node()
        # 이 함수 내부에서 rclpy를 시작한 경우에만 shutdown
        if should_shutdown and rclpy.ok():
            rclpy.shutdown()

    return saved


if __name__ == '__main__':
    # 단독 실행 테스트
    capture_single_frame()