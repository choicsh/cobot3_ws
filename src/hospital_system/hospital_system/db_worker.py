"""관제 PC 에서 도는 DB 워커 (ROS 없음) — P6 fleet_manager 가 생기면 그쪽으로 옮긴다.

    ros2 run hospital_system db_worker [--history-period 5]

- Redis stream:robot_events -> PostgreSQL robot_event_log (컨슈머 그룹 + ACK, 워커가 죽었다 살아도 빠짐없이)
- history-period 마다 heartbeat 가 살아 있는 로봇의 Redis state -> robot_state_history
"""
import argparse
import time

from hospital_system import db


def alive_robots(r=None):
    r = r or db.get_redis()
    ids = sorted(int(k.split(":")[1]) for k in r.scan_iter("robot:*:state"))
    return [rid for rid in ids if db.robot_is_alive(rid, r=r)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--history-period", type=float, default=5.0, help="위치 이력 주기 s")
    ap.add_argument("--consumer", default="worker-1")
    args, _ = ap.parse_known_args()   # ros2 run 이 붙이는 --ros-args 는 무시

    for name, (on, err) in db.db_check().items():
        if not on:
            raise SystemExit(f"[db_worker] {name} off: {err}")
    print(f"[db_worker] postgres {db.PG_DSN.split('@')[-1]}, redis {db.REDIS_URL.split('@')[-1]}", flush=True)

    next_snapshot = 0.0
    events = rows = 0
    last = None
    try:
        while True:
            try:
                events += db.robot_event_sync(consumer=args.consumer, block_ms=1000)
                if time.monotonic() >= next_snapshot:
                    next_snapshot = time.monotonic() + args.history_period
                    ids = alive_robots()
                    rows += db.robot_state_history_snapshot(ids) if ids else 0
                    if (events, ids) != last:   # 바뀔 때만 (위치 이력은 매번 늘어난다)
                        last = (events, ids)
                        print(f"[db_worker] events {events}  history rows {rows}  alive {ids}", flush=True)
            except Exception as e:   # DB 재시작 등 — 잠시 뒤 다시 (연결은 모듈이 다시 연다)
                print(f"[db_worker] {type(e).__name__}: {e}", flush=True)
                db.close_all()
                time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        db.close_all()


if __name__ == "__main__":
    main()
