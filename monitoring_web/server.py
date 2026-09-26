#!/usr/bin/env python3
"""Lightweight localhost dashboard server for the hospital specimen transport system.

Only Python's standard library is used. PostgreSQL (robotdb3_sql, DB v5 schema) and
Redis (robotdb3_nosql) are queried through psql / redis-cli inside the existing Docker
containers, so no local database driver or package installation is required.

- PostgreSQL: robots, transport tasks + trays, task status log, robot events
- Redis: live robot state (stage, pose, task) and heartbeat, current leg route,
  fleet zone reservations (fleet:zones) written by hospital_system.fleet_manager
- Lane zones for the map come from hospital_system.lane_graph when ROS is sourced
  (the same geometry the robots drive); without it the map shows robots only.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
from pathlib import Path
import struct
import subprocess
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"
# The map the robots localize on (hospital_navigation.launch.py)
DEFAULT_MAP_YAML = ROOT.parent / "src/nova_carter/carter_navigation/maps/hospital_integration_human.yaml"

DB_CONTAINER = os.environ.get("ROBOT_DB_CONTAINER", "robotdb3_sql")
DB_NAME = os.environ.get("ROBOT_DB_NAME", "robotdb3_sql")
DB_USER = os.environ.get("ROBOT_DB_USER", "rokey")
REDIS_CONTAINER = os.environ.get("ROBOT_REDIS_CONTAINER", "robotdb3_nosql")
REDIS_USER = os.environ.get("ROBOT_REDIS_USER", "rokey")
REDIS_PASSWORD = os.environ.get("ROBOT_REDIS_PASSWORD", "rokey")
MAP_YAML = Path(os.environ.get("ROBOT_MAP_YAML", DEFAULT_MAP_YAML)).resolve()

# Task statuses that are finished (task_status ENUM, DB v5)
DONE = "('COMPLETED', 'CANCELLED', 'FAILED')"

DASHBOARD_SQL = rf"""
SELECT json_build_object(
    'serverTime', now(),
    'summary', json_build_object(
        'totalRobots', (SELECT count(*) FROM robot_info),
        'activeRobots', (SELECT count(*) FROM robot_info WHERE is_active),
        'activeTasks', (SELECT count(*) FROM transport_task WHERE status NOT IN {DONE}),
        'urgentTrays', (
            SELECT count(*) FROM transport_task t, unnest(t.tray_ids) AS tid
            JOIN tray ON tray.tray_id = tid
            WHERE t.status NOT IN {DONE} AND tray.priority = 3
        ),
        'attentionTasks', (SELECT count(*) FROM transport_task WHERE status IN ('CANCELLED', 'FAILED'))
    ),
    'robots', COALESCE((
        SELECT json_agg(json_build_object(
            'robotId', r.robot_id,
            'name', r.robot_name,
            'model', r.model,
            'slots', r.container_slots,
            'isActive', r.is_active,
            'x', h.x, 'y', h.y, 'status', h.status, 'recordedAt', h.recorded_at
        ) ORDER BY r.robot_id)
        FROM robot_info r
        LEFT JOIN LATERAL (
            SELECT x, y, status, recorded_at FROM robot_state_history
            WHERE robot_id = r.robot_id ORDER BY recorded_at DESC LIMIT 1
        ) h ON true
    ), '[]'::json),
    'tasks', COALESCE((
        SELECT json_agg(task ORDER BY task.rank, task."taskId" DESC) FROM (
            SELECT t.task_id AS "taskId", t.status::text AS status,
                   CASE WHEN t.status NOT IN {DONE} THEN 0 ELSE 1 END AS rank,
                   t.robot_id AS "robotId", r.robot_name AS "robotName",
                   t.origin, t.destination, t.departed_at AS "departedAt", t.arrived_at AS "arrivedAt",
                   t.cancel_reason AS "cancelReason",
                   (SELECT min(changed_at) FROM task_status_log l WHERE l.task_id = t.task_id) AS "createdAt",
                   (SELECT json_agg(json_build_object('trayId', tr.tray_id, 'slot', s.slot,
                                                      'priority', tr.priority, 'testType', tr.test_type)
                                    ORDER BY s.slot)
                      FROM unnest(t.tray_ids, t.slot_nos) AS s(tray_id, slot)
                      JOIN tray tr ON tr.tray_id = s.tray_id) AS trays
            FROM transport_task t
            LEFT JOIN robot_info r ON r.robot_id = t.robot_id
            ORDER BY t.task_id DESC
            LIMIT 40
        ) task
    ), '[]'::json),
    'trays', COALESCE((
        SELECT json_agg(item ORDER BY item."createdAt" DESC, item."trayId") FROM (
            SELECT tr.tray_id AS "trayId", tr.test_type AS "testType", tr.priority,
                   tr.specimen_count AS "specimenCount", tr.created_at AS "createdAt",
                   t.task_id AS "taskId", t.status::text AS "taskStatus",
                   t.slot_nos[array_position(t.tray_ids, tr.tray_id)] AS slot
            FROM tray tr
            LEFT JOIN transport_task t ON t.tray_ids @> ARRAY[tr.tray_id]::varchar[]
            ORDER BY tr.created_at DESC, tr.tray_id
            LIMIT 24
        ) item
    ), '[]'::json),
    'events', COALESCE((
        SELECT json_agg(json_build_object(
            'eventId', e.event_id, 'robotId', e.robot_id, 'robotName', r.robot_name,
            'eventType', e.event_type, 'detail', e.detail, 'occurredAt', e.occurred_at
        ) ORDER BY e.occurred_at DESC, e.event_id DESC)
        FROM (SELECT * FROM robot_event_log ORDER BY occurred_at DESC, event_id DESC LIMIT 20) e
        JOIN robot_info r ON r.robot_id = e.robot_id
    ), '[]'::json),
    'statusLogs', COALESCE((
        SELECT json_agg(json_build_object(
            'logId', l.log_id, 'taskId', l.task_id,
            'fromStatus', l.from_status, 'toStatus', l.to_status, 'changedAt', l.changed_at
        ) ORDER BY l.changed_at DESC, l.log_id DESC)
        FROM (SELECT * FROM task_status_log ORDER BY changed_at DESC, log_id DESC LIMIT 20) l
    ), '[]'::json)
)::text;
"""

# One round trip: every robot:{id}:state hash + heartbeat + current route, and fleet:zones.
REDIS_LUA = r"""
local out = {robots = {}, zones = {}}
for _, key in ipairs(redis.call('KEYS', 'robot:*:state')) do
  local id = string.match(key, 'robot:(%d+):state')
  local raw = redis.call('HGETALL', key)
  local st = {}
  for i = 1, #raw, 2 do st[raw[i]] = raw[i + 1] end
  st.alive = redis.call('EXISTS', 'robot:' .. id .. ':heartbeat') == 1
  st.route = redis.call('LRANGE', 'robot:' .. id .. ':route', 0, -1)
  out.robots[id] = st
end
local zones = redis.call('HGETALL', 'fleet:zones')
for i = 1, #zones, 2 do out.zones[zones[i]] = zones[i + 1] end
return cjson.encode(out)
"""


def read_map(yaml_path: Path) -> tuple[dict, Path]:
    """Map metadata from a ROS map YAML (no PyYAML: the keys we need are one-liners)."""
    meta = {}
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        if ":" in line and not line.lstrip().startswith("#"):
            key, value = line.split(":", 1)
            meta[key.strip()] = value.strip()
    image = (yaml_path.parent / meta["image"]).resolve()
    origin = [float(v) for v in meta["origin"].strip("[]").split(",")]
    with image.open("rb") as f:           # PNG IHDR: width, height
        header = f.read(24)
    width, height = struct.unpack(">II", header[16:24])
    return {
        "imageWidth": width, "imageHeight": height,
        "resolution": float(meta["resolution"]),
        "originX": origin[0], "originY": origin[1],
    }, image


def load_lanes() -> dict | None:
    """Zone centerlines from hospital_system.lane_graph (needs the ROS workspace sourced)."""
    try:
        from hospital_system.lane_graph import LaneGraph, hospital_edges
    except Exception as exc:  # noqa: BLE001 - any import problem just disables the lane layer
        print(f"차선 구역 표시 끔 (hospital_system 을 import 하지 못함: {exc})")
        return None
    graph = LaneGraph(hospital_edges())
    # 물리 구역 기준 — 양방향 복도는 두 방향 간선이 같은 구역을 쓰고, 관제 예약(fleet:zones)도 이 id 로 적는다
    return {
        "zones": {
            pid: {"edge": z.track or z.edge, "shared": z.shared,
                  "points": [[round(p[0], 2), round(p[1], 2)] for p in z.points]}
            for pid, z in graph.phys_zones.items()
        },
    }


def run(command: list[str], what: str) -> str:
    try:
        return subprocess.run(command, check=True, capture_output=True, text=True, timeout=8).stdout.strip()
    except subprocess.SubprocessError as exc:
        detail = getattr(exc, "stderr", "") or str(exc)
        raise RuntimeError(f"{what} 조회 실패: {detail.strip()}") from exc


def fetch_redis() -> dict:
    out = run(["docker", "exec", REDIS_CONTAINER, "redis-cli", "--user", REDIS_USER, "--pass", REDIS_PASSWORD,
               "--no-auth-warning", "EVAL", REDIS_LUA, "0"], "Redis")
    try:
        data = json.loads(out)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Redis 응답 해석 실패: {out[:200]}") from exc
    # cjson encodes empty tables as {} — normalise to the lists / dicts the page expects
    for key in ("robots", "zones"):
        if not isinstance(data.get(key), dict):
            data[key] = {}
    for st in data["robots"].values():
        if not isinstance(st.get("route"), list):
            st["route"] = []
    return data


def fetch_dashboard() -> dict:
    """One dashboard snapshot: PostgreSQL records merged with Redis live state."""
    out = run(["docker", "exec", DB_CONTAINER, "psql", "-X", "-U", DB_USER, "-d", DB_NAME,
               "-At", "-v", "ON_ERROR_STOP=1", "-c", DASHBOARD_SQL], "DB")
    try:
        data = json.loads(out)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"DB 응답 해석 실패: {out[:200]}") from exc
    data["map"] = MAP
    try:
        live = fetch_redis()
        data["redisOk"] = True
    except RuntimeError as exc:
        live = {"robots": {}, "zones": {}}
        data["redisOk"] = False
        data["redisError"] = str(exc)
    names = {}
    for robot in data["robots"]:
        names[robot["name"]] = robot["robotId"]
        st = live["robots"].get(str(robot["robotId"]))
        robot["alive"] = bool(st and st.get("alive"))
        if st:
            # Redis is the live source (1 s); robot_state_history is a 5 s snapshot kept as fallback
            robot.update(x=float(st.get("x", 0)), y=float(st.get("y", 0)), theta=float(st.get("theta", 0)),
                         status=st.get("status") or robot.get("status"),
                         taskId=int(st["task_id"]) if st.get("task_id") else None,
                         updatedAt=int(st.get("updated_at", 0)) or None,
                         route=[[float(v) for v in wp.split(",")] for wp in st.get("route", [])])
    data["zones"] = {zid: {"robot": name, "robotId": names.get(name)} for zid, name in live["zones"].items()}
    return data


class DashboardHandler(SimpleHTTPRequestHandler):
    server_version = "RobotControl/2.0"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, format: str, *args) -> None:
        if "/api/stream" not in (args[0] if args else ""):
            print(f"[{self.log_date_time_string()}] {format % args}")

    def send_json(self, payload, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def serve_map(self) -> None:
        body = MAP_IMAGE.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mimetypes.guess_type(MAP_IMAGE.name)[0] or "image/png")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "public, max-age=3600")
        self.end_headers()
        self.wfile.write(body)

    def serve_stream(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        last_payload = None
        last_sent = 0.0
        try:
            while True:
                try:
                    data = fetch_dashboard()
                    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
                    now = time.monotonic()
                    if payload != last_payload or now - last_sent >= 12:
                        self.wfile.write(f"event: dashboard\ndata: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        last_payload, last_sent = payload, now
                except RuntimeError as exc:
                    error = json.dumps({"message": str(exc)}, ensure_ascii=False)
                    self.wfile.write(f"event: db-error\ndata: {error}\n\n".encode("utf-8"))
                    self.wfile.flush()
                time.sleep(1.0)
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/dashboard":
            try:
                self.send_json(fetch_dashboard())
            except RuntimeError as exc:
                self.send_json({"error": "database_unavailable", "message": str(exc)},
                               HTTPStatus.SERVICE_UNAVAILABLE)
            return
        if path == "/api/lanes":
            self.send_json(LANES or {"zones": {}})
            return
        if path == "/api/health":
            try:
                snapshot = fetch_dashboard()
                self.send_json({"status": "ok", "database": DB_NAME, "redis": snapshot.get("redisOk"),
                                "serverTime": snapshot.get("serverTime")})
            except RuntimeError as exc:
                self.send_json({"status": "error", "message": str(exc)}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        if path == "/api/stream":
            self.serve_stream()
            return
        if path == "/assets/map.png":
            self.serve_map()
            return
        if path == "/":
            self.path = "/index.html"
        super().do_GET()


MAP, MAP_IMAGE = read_map(MAP_YAML)
LANES = None


def main() -> None:
    global LANES
    parser = argparse.ArgumentParser(description="검체 이송 로봇 관제 웹 서버")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    LANES = load_lanes()
    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    server.daemon_threads = True
    print(f"관제 웹페이지: http://{args.host}:{args.port}")
    print(f"DB: {DB_CONTAINER}/{DB_NAME} | Redis: {REDIS_CONTAINER} | Map: {MAP_IMAGE.name} {MAP}")
    print(f"차선 구역: {len(LANES['zones']) if LANES else 0}개")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n서버를 종료합니다.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
