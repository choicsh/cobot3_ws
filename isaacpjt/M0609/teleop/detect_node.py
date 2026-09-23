#!/usr/bin/env python3
"""손목 D455 이미지를 구독하다가 스냅 트리거가 오면 YOLO 검출 이미지를 만든다.

pnp_teleop_detection.py 와 짝이다. Isaac Sim 은 자체 python3.11 + numpy 1.26 이고
ultralytics 는 ~/yolo-venv 의 python3.12 라 한 프로세스에 합칠 수 없어서 노드를 나눴다.

실행 (ROS2 Jazzy 가 python3.12 라 venv 에서 rclpy 를 그대로 쓸 수 있다):

    source /opt/ros/jazzy/setup.bash
    PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages \
        ~/yolo-venv/bin/python isaacpjt/M0609/teleop/detect_node.py [weights.pt]

Isaac 쪽이 평소엔 발행 게이트를 닫아두고 C 를 누른 프레임에만 1장 내보내므로,
여기 도착하는 이미지는 전부 스냅샷이다. 결과는 _detections/ 에 저장한다.
"""
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

IMAGE_TOPIC = "/wrist_camera/image_raw"
# train_yolo.py 의 project 가 상대경로라 ultralytics 가 runs_dir(runs) + task(detect)
# 아래에 한 번 더 중첩시킨다. 그래서 경로가 이렇게 길다.
DEFAULT_WEIGHTS = "runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt"
CONF_THRESHOLD = 0.85
OUT_DIR = Path(__file__).resolve().parent / "_detections"
# pnp_teleop_detection.py 가 C 키에 이 파일을 만든다. 양쪽 다 이 스크립트 옆 경로라
# 실행 디렉터리와 무관하게 같은 곳을 본다.
TRIGGER = Path(__file__).resolve().parent / "_snap.trigger"


def image_to_bgr(msg):
    """sensor_msgs/Image -> ultralytics 가 기대하는 BGR numpy 배열.

    cv_bridge 를 쓸 만큼 복잡하지 않다. Isaac 의 ROS2CameraHelper 는 rgb8 로 보낸다."""
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        return arr[:, :, ::-1]
    if msg.encoding == "rgba8":
        return arr[:, :, 2::-1]
    if msg.encoding == "bgr8":
        return arr
    raise ValueError(f"예상 못 한 인코딩: {msg.encoding}")


class DetectNode(Node):
    def __init__(self, weights):
        super().__init__("wrist_detect")
        self.model = YOLO(weights)
        self.latest = None
        self.count = 0
        OUT_DIR.mkdir(exist_ok=True)
        TRIGGER.unlink(missing_ok=True)  # 지난 실행이 남긴 트리거를 먹고 시작하지 않도록
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 1)
        self.create_timer(0.05, self._check_trigger)
        self.get_logger().info(f"{weights} 로드 (conf>={CONF_THRESHOLD}). Isaac 에서 C 를 누른다")

    def _on_image(self, msg):
        self.latest = msg  # 최신 프레임만 들고 있다가, 트리거가 올 때만 추론한다

    def _check_trigger(self):
        if not TRIGGER.exists():
            return
        TRIGGER.unlink(missing_ok=True)
        if self.latest is None:
            self.get_logger().warn(f"{IMAGE_TOPIC} 수신 이력이 없다. Isaac 에서 Play 를 눌렀는지 확인할 것")
            return

        result = self.model(image_to_bgr(self.latest), conf=CONF_THRESHOLD, verbose=False)[0]
        self.count += 1
        out = OUT_DIR / f"{datetime.now():%H%M%S}_{self.count:03d}.png"
        result.save(filename=str(out))  # 박스까지 그려서 저장해 준다

        if len(result.boxes) == 0:
            self.get_logger().info(f"conf>={CONF_THRESHOLD} 검출 없음 -> {out}")
            return
        found = ", ".join(
            f"{result.names[int(c)]} {conf:.2f}"
            for c, conf in zip(result.boxes.cls.tolist(), result.boxes.conf.tolist())
        )
        self.get_logger().info(f"{len(result.boxes)}개: {found} -> {out}")


def main():
    weights = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_WEIGHTS
    if not Path(weights).is_file():
        sys.exit(f"가중치가 없다: {weights}\n학습을 먼저 돌리거나 경로를 인자로 넘길 것")
    rclpy.init()
    node = DetectNode(weights)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def demo():
    """인코딩 변환 부호 확인 — 여기가 틀리면 색이 뒤집힌 채로 추론한다"""

    class FakeMsg:
        height, width = 1, 2

    m = FakeMsg()
    m.encoding = "rgb8"
    m.data = bytes([1, 2, 3, 4, 5, 6])  # (R,G,B) = (1,2,3), (4,5,6)
    assert image_to_bgr(m).tolist() == [[[3, 2, 1], [6, 5, 4]]]
    m.encoding = "rgba8"
    m.data = bytes([1, 2, 3, 255, 4, 5, 6, 255])
    assert image_to_bgr(m).tolist() == [[[3, 2, 1], [6, 5, 4]]]
    m.encoding = "bgr8"
    m.data = bytes([1, 2, 3, 4, 5, 6])
    assert image_to_bgr(m).tolist() == [[[1, 2, 3], [4, 5, 6]]]
    print("demo ok")


if __name__ == "__main__":
    demo() if len(sys.argv) == 2 and sys.argv[1] == "demo" else main()
