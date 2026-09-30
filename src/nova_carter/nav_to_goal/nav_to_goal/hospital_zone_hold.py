"""관제(hospital_system fleet_manager)의 구역 예약 정지점 — 받은 구역 끝을 넘지 않게 구간 실행기가 선다.

토픽 zone_hold (상대 이름, std_msgs/String JSON, reliable + transient local — 새 미션도 마지막 값을 바로 받는다):
    {"leg": "<cycle>:<route_id>", "stop": [x, y, yaw] | null, "dock": true | false, "seq": n}
    leg  = 어느 주행 구간에 대한 허가인지. 미션은 자기 구간(zone_leg 파라미터)의 메시지만 쓴다 — 이전 구간
           끝의 '다 받음(stop null)' 을 새 구간이 쓰면 허가 없이 출발하게 된다.
    stop = base_link 가 이 차선 중심선 점을 넘으면 안 된다. null = 목적지 도킹 구역까지 받았다.
    dock = 목적지 책상 도킹 구역을 받았다 (정류장에서 도킹 시작 허가).
required=False(관제 없이 단독 실행)면 메시지가 없을 때 막지 않는다. required=True 면 첫 메시지 전에는 선다.
사람 양보(YIELDING)와 달리 이 대기는 15 s 양보 예산에 넣지 않는다 — 앞 로봇이 하역하는 동안 1분 넘게 설 수 있다.
"""
import json
import math

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

HOLD_DISTANCE_M = 0.8    # 정지점까지 남은 경로 거리가 이보다 짧으면 선다 (0.6 m/s 제동 0.3 m + 반응)
MATCH_M = 0.35           # 정지점이 이 기준 경로 위에 있다고 볼 거리
BACK_SEARCH = 40         # 기준 경로 점 간격 0.05 m -> 2 m 뒤까지 (정지점을 조금 지나쳤을 때)


class ZoneHold:
    def __init__(self, node, required=False, leg="", topic="zone_hold"):
        self.required, self.leg = required, leg
        self.msg = None
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = node.create_subscription(String, topic, self._on, qos)

    def _on(self, message):
        try:
            msg = json.loads(message.data)
        except ValueError:
            return
        if not self.leg or msg.get("leg") == self.leg:
            self.msg = msg

    def dock_allowed(self):
        if self.msg is None:
            return not self.required
        return bool(self.msg.get("dock"))

    def distance(self, points, index):
        """기준 경로 points[index] 에서 정지점까지 경로 거리. 정지점이 이 경로에 없으면 None,
        지나쳤으면 음수. 관제 필수인데 아직 메시지가 없으면 0 (선다)."""
        if self.msg is None:
            return 0.0 if self.required else None
        stop = self.msg.get("stop")
        if not stop:
            return None
        best = None     # 처음 가까워진 구간에서 가장 가까운 점 (경로가 나중에 다시 지나가도 첫 통과만)
        for k in range(max(0, index - BACK_SEARCH), len(points)):
            p = points[k]
            d = math.dist(p[:2], stop[:2])
            heading = abs(math.atan2(math.sin(p[2] - stop[2]), math.cos(p[2] - stop[2])))
            if d <= MATCH_M and heading < math.pi / 2:
                if best is None or d < best[0]:
                    best = (d, k)
            elif best is not None:
                break
        if best is None:
            return None
        k = best[1]
        lo, hi, sign = (index, k, 1.0) if k >= index else (k, index, -1.0)
        return sign * sum(math.dist(points[i][:2], points[i + 1][:2]) for i in range(lo, hi))

    def held(self, points, index):
        d = self.distance(points, index)
        return d is not None and d <= HOLD_DISTANCE_M
