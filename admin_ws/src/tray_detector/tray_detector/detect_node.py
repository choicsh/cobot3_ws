#!/usr/bin/env python3
"""관제 PC — Isaac 의 손목 D455 스트림을 받아 트레이를 검출하고 3D 좌표를 돌려준다.

구독  /wrist_camera/color/image_raw    (rgb8)
      /wrist_camera/depth/image_raw    (32FC1, 미터, 광축 방향 Z 깊이)
      /wrist_camera/color/camera_info  (fx, fy, cx, cy)
발행  /tray_detection  std_msgs/Float32MultiArray
          data = [seq, n, x1,y1,z1,conf1, x2,y2,z2,conf2, ...]
          seq 는 발행할 때마다 1 씩 오르는 카운터. Isaac 이 '관측 자세로 이동한
          뒤에 새로 찍은 값인지'를 구분하는 데 쓴다 (메시지에 타임스탬프가 없다).
          카메라 광학 프레임(x 우, y 하, z 전방) 기준 미터.
          **화면 왼쪽부터 정렬**해서 보낸다 — 로봇이 왼쪽 트레이부터 집는다.

깊이는 컬러 카메라와 같은 render product 에서 뽑혀 나오므로 픽셀 단위로 정렬돼 있다.
별도 정합이 필요 없다.
"""
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO

COLOR_TOPIC = "/wrist_camera/color/image_raw"
DEPTH_TOPIC = "/wrist_camera/depth/image_raw"
INFO_TOPIC = "/wrist_camera/color/camera_info"
RESULT_TOPIC = "/tray_detection"

CONF_THRESHOLD = 0.85
MAX_TRAYS = 3            # 랙 자리가 3개다
# 손목 카메라 화면 아래쪽은 늘 그리퍼가 차지한다. 거기 뜨는 오검출은 신뢰도로 못 거른다
# (0.85 를 넘겨서 올라온다). 물리적으로 말이 안 되는 거리로 거른다.
MIN_DEPTH_M = 0.15       # 이보다 가까우면 그리퍼다
MAX_DEPTH_M = 1.5        # 이보다 멀면 팔이 닿지 않는다
PATCH_FRAC = 0.2         # 박스 중앙 이 비율만큼만 깊이 표본으로 쓴다
OUT_DIR = Path.home() / "tray_detections"


def image_to_bgr(msg):
    """sensor_msgs/Image -> ultralytics 가 기대하는 BGR numpy 배열"""
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        return arr[:, :, ::-1]
    if msg.encoding == "rgba8":
        return arr[:, :, 2::-1]
    if msg.encoding == "bgr8":
        return arr
    raise ValueError(f"예상 못 한 컬러 인코딩: {msg.encoding}")


def depth_to_meters(msg):
    """sensor_msgs/Image -> float32 깊이 배열 (미터)"""
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    if msg.encoding == "16UC1":  # 실물 RealSense 는 mm 정수로 보낸다
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width) * 0.001
    raise ValueError(f"예상 못 한 깊이 인코딩: {msg.encoding}")


def sample_depth(depth, x0, y0, x1, y1):
    """박스 중앙 패치의 깊이 중앙값. 표본이 없으면 None.

    트레이는 시험관 랙이라 구멍이 많다. 중심 픽셀 하나만 보면 구멍 바닥이나
    뒤 배경을 찍을 수 있어서, 패치를 떠서 유효값의 중앙값을 쓴다."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hw = max(1, int((x1 - x0) * PATCH_FRAC / 2))
    hh = max(1, int((y1 - y0) * PATCH_FRAC / 2))
    patch = depth[
        max(0, int(cy - hh)) : min(depth.shape[0], int(cy + hh) + 1),
        max(0, int(cx - hw)) : min(depth.shape[1], int(cx + hw) + 1),
    ]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    return float(np.median(valid)) if valid.size else None


def deproject(u, v, z, fx, fy, cx, cy):
    """픽셀 + 광축 Z 깊이 -> 카메라 광학 프레임 3D 점.

    Isaac 의 depth 는 DistanceToImagePlane(광축 방향 Z)이라 이 핀홀 식이 그대로 맞다.
    유클리드 거리였다면 다른 식을 써야 한다."""
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


class TrayDetector(Node):
    def __init__(self, weights):
        super().__init__("tray_detector")
        self.model = YOLO(weights)
        self.depth = None
        self.info = None
        self.saved = 0
        self.seq = 0
        OUT_DIR.mkdir(exist_ok=True)

        self.pub = self.create_publisher(Float32MultiArray, RESULT_TOPIC, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._on_info, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, 1)
        self.create_subscription(Image, COLOR_TOPIC, self._on_color, 1)
        self.get_logger().info(f"{weights} 로드 (conf>={CONF_THRESHOLD}). {COLOR_TOPIC} 대기 중")

    def _on_info(self, msg):
        # k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.info = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_depth(self, msg):
        self.depth = msg

    def _on_color(self, msg):
        if self.depth is None or self.info is None:
            return  # 아직 깊이나 내부 파라미터가 안 왔다
        depth = depth_to_meters(self.depth)
        if depth.shape != (msg.height, msg.width):
            self.get_logger().warn(
                f"컬러 {msg.height}x{msg.width} 와 깊이 {depth.shape} 해상도가 다르다. "
                "두 헬퍼가 같은 render product 를 쓰는지 확인할 것"
            )
            return

        result = self.model(image_to_bgr(msg), conf=CONF_THRESHOLD, verbose=False)[0]
        fx, fy, cx, cy = self.info

        found, rejected = [], []
        for box, conf in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist()):
            z = sample_depth(depth, *box)
            if z is None:
                rejected.append("깊이 없음")
                continue
            if not (MIN_DEPTH_M <= z <= MAX_DEPTH_M):
                rejected.append(f"{z:.3f}m (범위 밖)")
                continue
            u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            found.append((u, *deproject(u, v, z, fx, fy, cx, cy), conf))

        if rejected:
            self.get_logger().info(f"버림 {len(rejected)}개: {', '.join(rejected)}")

        found.sort(key=lambda f: f[0])          # 화면 왼쪽부터
        found = found[:MAX_TRAYS]

        self.seq += 1
        out = [float(self.seq), float(len(found))]
        for _, x, y, z, conf in found:
            out += [x, y, z, conf]
        self.pub.publish(Float32MultiArray(data=out))

        if found:
            self._log_and_save(result, found)

    def _log_and_save(self, result, found):
        self.saved += 1
        if self.saved % 10 == 1:  # 매 프레임 저장하면 디스크가 남아나지 않는다
            path = OUT_DIR / f"{datetime.now():%H%M%S}_{self.saved:04d}.png"
            result.save(filename=str(path))
        where = " | ".join(f"({x:+.3f} {y:+.3f} {z:.3f}) {c:.2f}" for _, x, y, z, c in found)
        self.get_logger().info(f"{len(found)}개 (왼쪽부터): {where}")


def main(args=None):
    weights = sys.argv[1] if len(sys.argv) > 1 else str(Path.home() / "tray_best.pt")
    if not Path(weights).is_file():
        sys.exit(f"가중치가 없다: {weights}\n학습된 best.pt 를 관제 PC 로 복사하거나 경로를 인자로 넘길 것")
    rclpy.init(args=args)
    node = TrayDetector(weights)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def demo():
    """역투영과 깊이 표본의 부호/값 확인. 여기가 틀리면 로봇이 엉뚱한 곳으로 간다"""
    # 주점(cx,cy)에 있는 픽셀은 광축 위라 x=y=0
    assert deproject(320, 240, 2.0, 400, 400, 320, 240) == (0.0, 0.0, 2.0)
    # 주점보다 오른쪽/아래 픽셀은 +x, +y (ROS 광학 규약: x 우, y 하)
    x, y, z = deproject(420, 340, 2.0, 400, 400, 320, 240)
    assert (x, y, z) == (0.5, 0.5, 2.0), (x, y, z)

    depth = np.full((100, 100), np.nan, dtype=np.float32)
    depth[45:56, 45:56] = 1.5
    depth[50, 50] = 0.0                      # 구멍 — 유효값이 아니다
    assert sample_depth(depth, 20, 20, 80, 80) == 1.5
    assert sample_depth(np.full((10, 10), np.nan, np.float32), 0, 0, 9, 9) is None

    m = type("M", (), {"height": 1, "width": 2, "encoding": "32FC1"})()
    m.data = np.array([[1.0, 2.0]], dtype=np.float32).tobytes()
    assert depth_to_meters(m).tolist() == [[1.0, 2.0]]
    m.encoding, m.data = "16UC1", np.array([[1500, 2000]], dtype=np.uint16).tobytes()
    assert np.allclose(depth_to_meters(m), [[1.5, 2.0]])

    m2 = type("M", (), {"height": 1, "width": 2, "encoding": "rgb8"})()
    m2.data = bytes([1, 2, 3, 4, 5, 6])
    assert image_to_bgr(m2).tolist() == [[[3, 2, 1], [6, 5, 4]]]
    print("demo ok")


if __name__ == "__main__":
    demo() if len(sys.argv) == 2 and sys.argv[1] == "demo" else main()
