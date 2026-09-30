"""토픽 수신 주기/최대 간격/대역폭과 시뮬레이션 실시간 비율을 잰다 (시스템 python3, rclpy).

    export ROS_DOMAIN_ID=136
    python3 isaacpjt/system/measure_topics.py --robots 2 --seconds 30 [--wrist] [--json out.json]

직렬화된 바이트만 받아(raw) 역직렬화 비용이 측정에 섞이지 않게 한다. 수신 버퍼는
Nav2 와 같은 4 MB 프로파일(carter_navigation/params/fastdds_udp_4mb.xml)을 쓴다.
"""

import argparse
import json
import os
import time
from pathlib import Path

PROFILE = (Path(__file__).resolve().parents[2]
           / "src/nova_carter/carter_navigation/params/fastdds_udp_4mb.xml")
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", str(PROFILE))

import rclpy  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image, PointCloud2  # noqa: E402

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def summarize(stamps, sizes, seconds):
    if len(stamps) < 2:
        return {"hz": 0.0, "max_gap_s": None, "kb": None, "mb_s": 0.0, "count": len(stamps)}
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    return {
        "hz": round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 2),
        "max_gap_s": round(max(gaps), 3),
        "kb": round(sum(sizes) / len(sizes) / 1024, 1),
        "mb_s": round(sum(sizes) / seconds / 1e6, 2),
        "count": len(stamps),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", type=int, default=2)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--wrist", action="store_true")
    parser.add_argument("--json")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("fleet_topic_meter")
    topics = {}
    for i in range(1, args.robots + 1):
        topics[f"/robot{i}/front_3d_lidar/lidar_points"] = PointCloud2
        if args.wrist:
            topics[f"/robot{i}/wrist_camera/color/image_raw"] = Image
            topics[f"/robot{i}/wrist_camera/depth/image_raw"] = Image

    recording = False
    data = {name: ([], []) for name in topics}
    clock = []

    def on_message(name):
        def callback(raw):
            if recording:
                data[name][0].append(time.monotonic())
                data[name][1].append(len(raw))
        return callback

    for name, msg_type in topics.items():
        node.create_subscription(msg_type, name, on_message(name), SENSOR_QOS, raw=True)

    def on_clock(msg):
        if recording:
            clock.append((time.monotonic(), msg.clock.sec + msg.clock.nanosec * 1e-9))

    node.create_subscription(Clock, "/clock", on_clock, SENSOR_QOS)

    end = time.monotonic() + args.warmup
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    recording = True
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    result = {name: summarize(*data[name], args.seconds) for name in topics}
    rtf = None
    if len(clock) >= 2:
        rtf = round((clock[-1][1] - clock[0][1]) / (clock[-1][0] - clock[0][0]), 3)
    total = round(sum(r["mb_s"] for r in result.values()), 2)

    print(f"{'topic':<45} {'Hz':>6} {'max gap s':>10} {'KB':>8} {'MB/s':>6}")
    for name, r in result.items():
        print(f"{name:<45} {r['hz']:>6} {str(r['max_gap_s']):>10} {str(r['kb']):>8} {r['mb_s']:>6}")
    print(f"total {total} MB/s, sim real-time factor {rtf}")
    if args.json:
        Path(args.json).write_text(json.dumps(
            {"topics": result, "rtf": rtf, "total_mb_s": total}, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
