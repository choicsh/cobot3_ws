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
          **카메라에서 가까운 것부터 정렬**해서 보낸다 — 로봇이 앞에 있는 트레이부터 집는다.
          (왼쪽부터 집던 때는, 왼쪽 것을 잡으러 들어가다 앞에 있는 오른쪽 트레이를 건드렸다.)
      /aruco_markers   std_msgs/Float32MultiArray
          data = [seq, n, id1,slot1, id2,slot2, ...]
          트레이 손잡이 윗판의 ArUco(DICT_4X4_50). id = 긴급도 0/1/2(하/중/상),
          slot = RACK_ROI 를 가로 3등분한 칸 번호 0/1/2 (화면 왼쪽부터 = 랙 1/2/3번).
          3D 로 옮겨 칸과 거리를 재던 방식은 깊이 오차로 양옆 칸이 버려져서 픽셀 칸으로 바꿨다.
          적재/하역과 무관 — Isaac 이 적재 완료 후 랙을 위에서 관측할 때만 읽는다.

깊이는 컬러 카메라와 같은 render product 에서 뽑혀 나오므로 픽셀 단위로 정렬돼 있다.
별도 정합이 필요 없다.
"""
import sys
from datetime import datetime
from pathlib import Path

import cv2
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
MARKER_TOPIC = "/aruco_markers"

CONF_THRESHOLD = 0.75    # 이력: 0.85 -> 0.75 (2026-09-23). 검출 자체가 덜 잡히는 편이라 낮춰본다
MAX_TRAYS = 3            # 랙 자리가 3개다
# 손목 카메라 화면 아래쪽은 늘 그리퍼가 차지한다. 거기 뜨는 오검출은 신뢰도로 못 거른다
# (0.85 일 때도 넘겨서 올라왔다 — 문턱을 0.75 로 내렸으니 더 올라온다). 물리적으로
# 말이 안 되는 거리로 거른다.
MIN_DEPTH_M = 0.15       # 이보다 가까우면 그리퍼다
# 그리퍼는 손목에 고정돼 있어 화면에서 **항상 같은 자리**다. 깊이로 거르는 건 새는 구멍이
# 있었다 — sample_depth 가 박스의 15퍼센타일을 쓰는 탓에 그리퍼 박스에서도 먼 배경 깊이가
# 나올 수 있다. 픽셀 위치로 직접 막는다.
# 실측(검출 이미지 60장의 픽셀별 중앙값/잔차): 그리퍼 최상단 y = 345 (x 128~191 손가락 관절).
#   트레이 박스 중심 v ~ 219 / 그리퍼 오검출 박스 중심 v ~ 394 로 175px 벌어져 있다.
# 박스 '중심'으로 판정한다 — 트레이를 집으러 내려가면 박스가 그리퍼에 겹치는 게 정상이고,
# 중심까지 내려가야 그리퍼 자체를 물체로 잡은 것이다.
GRIPPER_TOP_Y = 330      # 실측 345 에서 15px 여유
MAX_DEPTH_M = 1.0        # 이보다 멀면 팔이 닿지 않는다 (1.5 -> 1.0: 1.1m 대 벽/연기 오검출 차단)
# 트레이는 손잡이+랙+시험관이 붙은 비대칭 조립체라, 박스의 '기하학적 중앙'이
# 물체가 아니라 손잡이-랙 사이 빈틈에 떨어지기 쉽다 — 거기를 패치로 찍으면
# 물체를 뚫고 뒤 배경(책상/벽)이 잡힌다. 그래서 중앙 패치 대신 박스 전체에서
# 표본을 모으고, '카메라는 배경보다 물체에 항상 더 가깝다'는 사실을 이용해
# 하위 퍼센타일(가까운 쪽)을 쓴다 — 배경이 박스의 대부분을 차지해도 안전하다.
DEPTH_PERCENTILE = 15    # 유효 표본 중 이 퍼센타일(가까운 쪽)을 물체 깊이로 본다
MIN_DEPTH_SAMPLES = 20   # 유효 표본이 이보다 적으면 신뢰하지 않는다
# 디버그 이미지 저장 간격(프레임). 0개인 프레임은 '왜 못 찾았는지'를 볼 유일한 자료라
# 반드시 남기되, 검출이 있는 프레임보다 드물게 남겨 디스크를 아낀다
SAVE_EVERY_FOUND = 10
SAVE_EVERY_EMPTY = 30
OUT_DIR = Path.home() / "tray_detections"
ARUCO = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
# 랙 관측 자세(Isaac OBSERVE_*)에서 랙이 화면에 잡히는 영역 (x0, y0, x1, y1). 마커가 원본에서
# ~28px(4px/셀)라 그대로는 못 읽는다 — 이 영역만 잘라 ARUCO_ZOOM 배 키워서 읽는다. 관측 자세를
# 바꾸면 저장 이미지의 노란 사각형(ROI)이 랙을 덮는지 확인하고 이 값을 맞출 것.
# 이력: (160,190,480,320) -> (165,175,510,255). 오른쪽 칸 마커가 x=480 경계에 잘리고,
# 아래쪽 94px 은 마커가 없는 트레이 몸통/책상이라 버렸다. 실측 마커 중심 x = 225/338/451
# (간격 113px) 이라, 폭 345 의 3등분 중앙(222/337/452)에 칸마다 하나씩 오도록 맞췄다 —
# slot_of() 가 이 ROI 를 가로 3등분해 칸을 매기므로 폭과 위치가 칸 배정에 직접 영향을 준다.
RACK_ROI = (165, 175, 510, 255)
ARUCO_ZOOM = 3
RACK_SLOTS = 3


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
    """박스 전체에서 하위 DEPTH_PERCENTILE(가까운 쪽)의 깊이. 표본이 부족하면 None.

    박스 안에는 물체(가까움)와 배경(멂)이 섞여 있을 수 있다. 어느 쪽이 몇 %를
    차지하는지 몰라도, '물체가 항상 배경보다 카메라에 가깝다'는 사실 하나로
    가까운 쪽 퍼센타일을 뽑으면 배경이 절반을 넘게 섞여 있어도 물체 깊이를
    골라낼 수 있다."""
    patch = depth[max(0, int(y0)):int(y1), max(0, int(x0)):int(x1)]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if valid.size < MIN_DEPTH_SAMPLES:
        return None
    return float(np.percentile(valid, DEPTH_PERCENTILE))


def read_markers(bgr):
    """RACK_ROI 를 ARUCO_ZOOM 배 키워서 읽은 ArUco 마커 -> [(id, cx, cy), ...] 원본 픽셀 중심. 없으면 []"""
    x0, y0, x1, y1 = RACK_ROI
    roi = cv2.resize(bgr[y0:y1, x0:x1], None, fx=ARUCO_ZOOM, fy=ARUCO_ZOOM, interpolation=cv2.INTER_CUBIC)
    corners, ids, _ = ARUCO.detectMarkers(roi)
    if ids is None:
        return []
    return [(int(i), *(c[0].mean(axis=0) / ARUCO_ZOOM + (x0, y0))) for i, c in zip(ids.ravel(), corners)]


def slot_of(cx):
    """마커 중심 x -> RACK_ROI 를 가로 3등분한 칸 0/1/2 (왼쪽부터)"""
    x0, x1 = RACK_ROI[0], RACK_ROI[2]
    return min(RACK_SLOTS - 1, max(0, int((cx - x0) * RACK_SLOTS / (x1 - x0))))


def cam_dist2(f):
    """검출 튜플 (u, x, y, z, conf) 의 카메라 원점까지 거리 제곱. 정렬 키다.

    광축 z 만 보지 않는 이유: 카메라가 아래로 기울어 있어 양옆으로 벌어진 트레이는
    z 가 비슷해도 실제 거리가 꽤 다르다."""
    return f[1] ** 2 + f[2] ** 2 + f[3] ** 2


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
        self.last_log = ""       # 같은 내용이면 다시 안 찍는다 (터미널이 15Hz 로그에 버거워한다)
        self.seq = 0
        OUT_DIR.mkdir(exist_ok=True)

        self.pub = self.create_publisher(Float32MultiArray, RESULT_TOPIC, 10)
        self.pub_markers = self.create_publisher(Float32MultiArray, MARKER_TOPIC, 10)
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

        bgr = image_to_bgr(msg)
        result = self.model(bgr, conf=CONF_THRESHOLD, verbose=False)[0]
        markers = read_markers(bgr)
        fx, fy, cx, cy = self.info

        found, rejected = [], []
        for box, conf in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist()):
            u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            if v >= GRIPPER_TOP_Y:
                rejected.append(f"v={v:.0f} (그리퍼 영역)")
                continue
            z = sample_depth(depth, *box)
            if z is None:
                rejected.append("깊이 없음")
                continue
            if not (MIN_DEPTH_M <= z <= MAX_DEPTH_M):
                rejected.append(f"{z:.3f}m (범위 밖)")
                continue
            found.append((u, *deproject(u, v, z, fx, fy, cx, cy), conf))

        if rejected:
            self._log_changed(f"버림 {len(rejected)}개: {', '.join(rejected)}")

        found.sort(key=cam_dist2)               # 가까운 것부터 — 앞엣것을 먼저 집어야 뒤엣것을 안 건드린다
        found = found[:MAX_TRAYS]

        self.seq += 1
        out = [float(self.seq), float(len(found))]
        for _, x, y, z, conf in found:
            out += [x, y, z, conf]
        self.pub.publish(Float32MultiArray(data=out))

        # 마커는 검출과 별도 토픽. 3D 없이 (id, 픽셀 칸) 만 보낸다
        mk = [float(self.seq), float(len(markers))]
        for mid, mx, _my in markers:
            mk += [float(mid), float(slot_of(mx))]
        self.pub_markers.publish(Float32MultiArray(data=mk))

        self._log_and_save(result, found, markers)

    def _log_and_save(self, result, found, markers):
        self.saved += 1
        # 매 프레임 저장하면 디스크가 남아나지 않는다. 단 마커 프레임은 관측 자세에서 잠깐만
        # 나오는 데다 판독이 깜빡여서(0~2개) 전부 남긴다 — 어느 칸이 왜 안 읽혔는지 볼 유일한 자료다
        #
        # 이력(2026-09-23): 예전엔 호출부가 `if found or markers:` 로 걸려 있어 **0개인 프레임이
        # 한 장도 안 남았다.** 정렬 이동 뒤 검출이 0개가 된 실패를 분석할 자료가 통째로 비어
        # 있었다 — 정작 봐야 할 순간이 그때다. 이제 0개도 남기되 빈도만 낮춘다.
        every = SAVE_EVERY_FOUND if found else SAVE_EVERY_EMPTY
        if markers or self.saved % every == 1:
            path = OUT_DIR / f"{datetime.now():%H%M%S}_{self.saved:04d}.png"
            img = result.plot()
            cv2.rectangle(img, RACK_ROI[:2], RACK_ROI[2:], (0, 255, 255), 1)   # 랙 ROI — 랙이 이 안에 있어야 한다
            for mid, cx, cy in markers:   # 마커 id 를 마커 '위'에 찍는다 (마커를 덮으면 이미지로 재분석이 안 된다)
                cv2.putText(img, f"aruco {mid}", (int(cx) - 20, int(cy) - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.imwrite(str(path), img)
        # 좌표는 mm 단위로 흔들려 매 프레임 다르니 cm 로 뭉개서 '내용이 바뀔 때만' 찍는다
        where = " | ".join(f"({x:+.2f} {y:+.2f} {z:.2f}) {c:.1f}" for _, x, y, z, c in found)
        self._log_changed(f"{len(found)}개 (가까운 순): {where}"
                          + (f"   aruco {len(markers)}개 {sorted(m[0] for m in markers)}" if markers else ""))

    def _log_changed(self, text):
        if text != self.last_log:
            self.last_log = text
            self.get_logger().info(text)


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

    # 박스 대부분은 물체(0.75m), 딱 중앙(기하학적 중심, 옛 patch 방식이 찍던 자리)만
    # 배경(3.0m)이 뚫려 보이는 상황 — 손잡이-랙 사이 빈틈을 흉내낸 것이다.
    # 중앙만 보던 옛 방식이면 3.0 을 골랐을 것이고, 새 방식은 0.75 를 골라야 맞다.
    # (0.75 는 이진수로 정확히 표현되는 값이라 float32/float64 비교가 안전하다)
    depth = np.full((100, 100), 0.75, dtype=np.float32)
    depth[45:56, 45:56] = 3.0
    assert sample_depth(depth, 0, 0, 100, 100) == 0.75

    depth[50, 50] = 0.0                      # 구멍 — 유효값이 아니다 (여전히 걸러져야 한다)
    assert sample_depth(depth, 0, 0, 100, 100) == 0.75

    assert sample_depth(np.full((10, 10), np.nan, np.float32), 0, 0, 9, 9) is None
    assert sample_depth(np.full((3, 3), 1.0, np.float32), 0, 0, 3, 3) is None  # 표본 부족

    # 마커 id 와 '원본' 픽셀 중심 복원: 흰 바탕의 ROI 안 (300,240)~(324,264) 에 id 2 마커 24px(4px/셀,
    # 관측 자세 실측 크기)를 붙인다. ROI 확대 없이는 이 크기를 못 읽는다
    canvas = np.full((480, 640, 3), 255, np.uint8)
    marker = cv2.aruco.generateImageMarker(ARUCO.getDictionary(), 2, 24)
    canvas[240:264, 300:324] = marker[:, :, None]
    (mid, mx, my), = read_markers(canvas)
    assert mid == 2 and abs(mx - 312) < 2 and abs(my - 252) < 2, (mid, mx, my)
    canvas[240:264, 300:324] = 255
    canvas[20:44, 20:44] = marker[:, :, None]          # ROI 밖은 안 본다
    assert read_markers(canvas) == []
    # 픽셀 칸: ROI(160~480) 3등분 -> 경계 포함 왼쪽부터 0/1/2, 밖은 가장자리 칸으로
    assert [slot_of(x) for x in (160, 266, 267, 373, 374, 479, 0, 639)] == [0, 0, 1, 1, 2, 2, 0, 2]

    # 정렬 키는 화면 위치가 아니라 카메라까지의 거리다. 왼쪽(u 작음)이라도 멀면 뒤로 간다
    far_left, near_right = (40, -0.30, 0.0, 0.80, 0.9), (600, 0.30, 0.0, 0.50, 0.9)
    assert sorted([far_left, near_right], key=cam_dist2) == [near_right, far_left]

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
