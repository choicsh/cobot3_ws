#!/usr/bin/env python3
"""Continuously write realistic demo robot positions to robotdb3_sql.

This is a local development helper. It appends state-history rows so the web
dashboard behaves exactly as it will when live robot telemetry starts writing
to the database.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import time


DB_CONTAINER = os.environ.get("ROBOT_DB_CONTAINER", "robotdb3_sql")
DB_NAME = os.environ.get("ROBOT_DB_NAME", "robotdb3_sql")
DB_USER = os.environ.get("ROBOT_DB_USER", "rokey")

# Waypoints are map-world coordinates in metres. The three loops occupy
# separate clear areas of integration_hospital.png so markers remain readable.
ROBOT_ROUTES = {
    1: [(4.0, 7.0), (10.0, 7.0), (10.0, 12.0), (4.0, 12.0)],
    2: [(4.0, 1.0), (10.0, 1.0), (10.0, 4.0), (4.0, 4.0)],
    3: [(0.0, -4.0), (8.0, -4.0), (8.0, -1.0), (0.0, -1.0)],
}

ROBOT_STATUS = {1: "운송중", 2: "배정", 3: "대기"}
ROBOT_PHASE = {1: 0.00, 2: 0.34, 3: 0.67}


running = True


def stop(_signum: int, _frame: object) -> None:
    global running
    running = False


def position_on_loop(route: list[tuple[float, float]], progress: float) -> tuple[float, float]:
    """Linearly interpolate a position around a closed waypoint loop."""
    wrapped = progress % 1.0
    segment_float = wrapped * len(route)
    segment = int(math.floor(segment_float))
    fraction = segment_float - segment
    start = route[segment]
    end = route[(segment + 1) % len(route)]
    return (
        start[0] + (end[0] - start[0]) * fraction,
        start[1] + (end[1] - start[1]) * fraction,
    )


def write_positions(positions: dict[int, tuple[float, float]], max_history: int) -> None:
    values = ",\n".join(
        f"({robot_id}, clock_timestamp(), {x:.3f}, {y:.3f}, 1, '{ROBOT_STATUS[robot_id]}')"
        for robot_id, (x, y) in positions.items()
    )
    sql = f"""
BEGIN;
INSERT INTO robot_state_history (robot_id, recorded_at, x, y, floor, status)
VALUES {values};

WITH ranked AS (
    SELECT robot_id, recorded_at,
           row_number() OVER (PARTITION BY robot_id ORDER BY recorded_at DESC) AS row_no
    FROM robot_state_history
), old_rows AS (
    SELECT robot_id, recorded_at FROM ranked WHERE row_no > {max_history}
)
DELETE FROM robot_state_history history
USING old_rows
WHERE history.robot_id = old_rows.robot_id
  AND history.recorded_at = old_rows.recorded_at;
COMMIT;
"""
    command = [
        "docker", "exec", DB_CONTAINER,
        "psql", "-X", "-U", DB_USER, "-d", DB_NAME,
        "-q", "-v", "ON_ERROR_STOP=1", "-c", sql,
    ]
    result = subprocess.run(command, capture_output=True, text=True, timeout=8)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "위치 데이터 쓰기 실패")


def main() -> None:
    parser = argparse.ArgumentParser(description="실시간 로봇 위치 더미데이터 생성기")
    parser.add_argument("--interval", type=float, default=1.5, help="DB 갱신 간격(초)")
    parser.add_argument("--lap-seconds", type=float, default=48.0, help="경로 한 바퀴 시간(초)")
    parser.add_argument("--max-history", type=int, default=120, help="로봇별 유지할 최대 이력")
    parser.add_argument("--ticks", type=int, default=0, help="0이면 중단할 때까지 실행")
    args = parser.parse_args()

    if args.interval < 0.2 or args.lap_seconds <= args.interval:
        raise SystemExit("interval은 0.2초 이상, lap-seconds는 interval보다 커야 합니다.")
    if args.max_history < 3:
        raise SystemExit("max-history는 3 이상이어야 합니다.")

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print(
        f"실시간 더미 위치 생성 시작: {DB_CONTAINER}/{DB_NAME} | "
        f"{args.interval:.1f}초 간격 | Ctrl+C로 종료"
    )
    started = time.monotonic()
    tick = 0
    while running and (args.ticks == 0 or tick < args.ticks):
        elapsed = time.monotonic() - started
        base_progress = elapsed / args.lap_seconds
        positions = {
            robot_id: position_on_loop(route, base_progress + ROBOT_PHASE[robot_id])
            for robot_id, route in ROBOT_ROUTES.items()
        }
        try:
            write_positions(positions, args.max_history)
        except (RuntimeError, subprocess.TimeoutExpired) as exc:
            print(f"DB 갱신 오류: {exc}")
        else:
            if tick == 0 or (tick + 1) % 10 == 0:
                summary = " | ".join(
                    f"R{robot_id}=({x:.1f}, {y:.1f})"
                    for robot_id, (x, y) in positions.items()
                )
                print(f"tick {tick + 1}: {summary}", flush=True)
        tick += 1
        if running and (args.ticks == 0 or tick < args.ticks):
            time.sleep(args.interval)

    print(f"실시간 더미 위치 생성 종료 (총 {tick}회 갱신)")


if __name__ == "__main__":
    main()
