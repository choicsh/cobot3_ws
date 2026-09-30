"""로봇 한 대의 팔 ROS 입출력 — OmniGraph 제네릭 ROS2 노드 (Isaac 5.1 = Python 3.11, rclpy 불가).

그래프 /World/robot{i}_arm_io, 네임스페이스 /robot{i}:
    sub  tray_detection  std_msgs/Float32MultiArray  [seq, n, x,y,z,conf, ...]  (tray_detector)
    sub  aruco_markers   std_msgs/Float32MultiArray  [seq, n, id,slot, ...]     (tray_detector)
    sub  arm/command     std_msgs/String   "load:<id>" / "unload:<id>" / "stop:<id>" 또는 JSON {"cmd","id"}
    pub  arm/status      std_msgs/String   JSON (ArmTaskController.status). 바뀔 때 + 주기적으로 발행

제네릭 구독 노드는 마지막으로 받은 값을 계속 들고 있다. 명령은 '문자열이 바뀌었을 때'만 새 명령으로
본다 — 같은 명령을 다시 보내려면 id 를 바꿀 것.
"""
import json

import carb
import numpy as np
import omni.graph.core as og


class ArmRosIO:
    def __init__(self, index, update):
        """update: simulation_app.update — 제네릭 노드 타입 설정 사이에 앱을 한 번씩 돌려야 한다."""
        self.index = index
        self.graph = f"/World/robot{index}_arm_io"
        self.camera_graph = f"/World/robot{index}_wrist"   # system/scene.build_wrist_camera_graph
        self._update = update
        self._camera_enabled = None

    def build(self):
        ns = f"/robot{self.index}"
        og.Controller.edit(
            {"graph_path": self.graph, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("tick", "omni.graph.action.OnPlaybackTick"),
                    ("impulse", "omni.graph.action.OnImpulseEvent"),
                    ("context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("sub_detect", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("sub_aruco", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("sub_cmd", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("pub_status", "isaacsim.ros2.bridge.ROS2Publisher"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("tick.outputs:tick", "sub_detect.inputs:execIn"),
                    ("tick.outputs:tick", "sub_aruco.inputs:execIn"),
                    ("tick.outputs:tick", "sub_cmd.inputs:execIn"),
                    ("impulse.outputs:execOut", "pub_status.inputs:execIn"),
                    ("context.outputs:context", "sub_detect.inputs:context"),
                    ("context.outputs:context", "sub_aruco.inputs:context"),
                    ("context.outputs:context", "sub_cmd.inputs:context"),
                    ("context.outputs:context", "pub_status.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("sub_detect.inputs:nodeNamespace", ns),
                    ("sub_detect.inputs:topicName", "tray_detection"),
                    ("sub_aruco.inputs:nodeNamespace", ns),
                    ("sub_aruco.inputs:topicName", "aruco_markers"),
                    ("sub_cmd.inputs:nodeNamespace", ns),
                    ("sub_cmd.inputs:topicName", "arm/command"),
                    ("pub_status.inputs:nodeNamespace", ns),
                    ("pub_status.inputs:topicName", "arm/status"),
                ],
            },
        )
        for node, name in (("sub_detect", "Float32MultiArray"), ("sub_aruco", "Float32MultiArray"),
                           ("sub_cmd", "String"), ("pub_status", "String")):
            self._set_message_type(f"{self.graph}/{node}", "std_msgs", "msg", name)

    def _set_message_type(self, node_path, package, subfolder, name):
        """제네릭 노드 메시지 타입. 세 값을 한꺼번에 넣으면 동적 어트리뷰트가 덜 만들어져
        조용히 안 올라온다 — name 을 비웠다가 마지막에 넣고, 사이에 앱을 돌린다
        (pick_and_place_detection.set_generic_message_type 과 같은 순서)."""
        def attr(field):
            return og.Controller.attribute(f"{node_path}.inputs:{field}")

        attr("messageName").set("")
        self._update()
        attr("messagePackage").set(package)
        attr("messageSubfolder").set(subfolder)
        attr("messageName").set(name)
        self._update()

    # ── 읽기 ─────────────────────────────────────────────────────
    def _read(self, node_name):
        try:
            return og.Controller.attribute(f"{self.graph}/{node_name}.outputs:data").get()
        except Exception:
            return None

    def _read_array(self, node_name, verbose=False):
        data = self._read(node_name)
        if data is None or len(data) < 2:
            if verbose:
                print(f"   robot{self.index} {node_name} — 아직 수신 전이거나 형식이 다르다")
            return None
        return data

    def read_detections(self, verbose=False):
        """(seq, [(np.array([x, y, z]), conf), ...]) 카메라 광학 프레임, 가까운 것부터. 없으면 (-1, [])."""
        data = self._read_array("sub_detect", verbose)
        if data is None:
            return -1, []
        seq, count = int(data[0]), int(data[1])
        out = [(np.array(data[2 + i * 4: 5 + i * 4], dtype=float), float(data[5 + i * 4]))
               for i in range(count)]
        if verbose:
            print(f"   robot{self.index} 검출 수신: seq {seq}, {count}개")
        return seq, out

    def read_markers(self, verbose=False):
        """(seq, [(id, slot), ...]). slot 은 tray_detector 가 랙 ROI 를 3등분한 칸 0/1/2."""
        data = self._read_array("sub_aruco", verbose)
        if data is None:
            return -1, []
        seq, count = int(data[0]), int(data[1])
        return seq, [(int(data[2 + i * 2]), int(data[3 + i * 2])) for i in range(count)]

    def read_command(self):
        """마지막으로 받은 명령 문자열 (없으면 "")."""
        data = self._read("sub_cmd")
        return data if isinstance(data, str) else ""

    # ── 쓰기 ─────────────────────────────────────────────────────
    def publish_status(self, status):
        og.Controller.attribute(f"{self.graph}/pub_status.inputs:data").set(json.dumps(status))
        og.Controller.attribute(f"{self.graph}/impulse.state:enableImpulse").set(True)

    def set_camera(self, enabled):
        """손목 카메라 render product 를 켜고 끈다 (끄면 렌더도 멈춘다 — 3대 동시 발행 시 약 2 Hz, R7)."""
        if enabled == self._camera_enabled:
            return
        try:
            og.Controller.attribute(f"{self.camera_graph}/rp.inputs:enabled").set(bool(enabled))
            self._camera_enabled = enabled
        except Exception as exc:
            carb.log_warn(f"robot{self.index} wrist camera toggle failed: {exc}")


def parse_command(text):
    """"load:3" / "load" / '{"cmd": "load", "id": 3}' -> "load". 모르는 형식이면 ""."""
    text = (text or "").strip()
    if not text:
        return ""
    if text.startswith("{"):
        try:
            return str(json.loads(text).get("cmd", "")).strip().lower()
        except (ValueError, AttributeError):
            return ""
    return text.split(":", 1)[0].strip().lower()
