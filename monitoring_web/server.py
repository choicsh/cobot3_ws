#!/usr/bin/env python3
"""Lightweight localhost dashboard server for robotdb3_sql.

The project intentionally uses only Python's standard library. PostgreSQL is
queried through psql inside the existing Docker container, so no local database
driver or package installation is required.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
from pathlib import Path
import subprocess
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"
DEFAULT_MAP_PATH = (
    ROOT.parent
    / "src/nova_carter/carter_navigation/maps/integration_hospital.png"
)

DB_CONTAINER = os.environ.get("ROBOT_DB_CONTAINER", "robotdb3_sql")
DB_NAME = os.environ.get("ROBOT_DB_NAME", "robotdb3_sql")
DB_USER = os.environ.get("ROBOT_DB_USER", "rokey")
MAP_PATH = Path(os.environ.get("ROBOT_MAP_PATH", DEFAULT_MAP_PATH)).resolve()


DASHBOARD_SQL = r"""
SELECT json_build_object(
    'serverTime', now(),
    'map', json_build_object(
        'imageWidth', 1510,
        'imageHeight', 660,
        'resolution', 0.05,
        'originX', -49.975,
        'originY', -9.475
    ),
    'summary', json_build_object(
        'totalRobots', (SELECT count(*) FROM robot_info),
        'activeRobots', (SELECT count(*) FROM robot_info WHERE is_active),
        'activeTasks', (
            SELECT count(*) FROM transport_task
            WHERE status NOT IN ('인계완료', '취소', '실패')
        ),
        'urgentSpecimens', (
            SELECT count(*) FROM specimen s
            WHERE s.priority = 1
              AND EXISTS (
                  SELECT 1 FROM transport_task t
                  WHERE t.specimen_id = s.specimen_id
                    AND t.status NOT IN ('인계완료', '취소', '실패')
              )
        ),
        'attentionTasks', (
            SELECT count(*) FROM transport_task WHERE status IN ('취소', '실패')
        )
    ),
    'robots', COALESCE((
        SELECT json_agg(json_build_object(
            'robotId', r.robot_id,
            'name', r.robot_name,
            'model', r.model,
            'slots', r.container_slots,
            'isActive', r.is_active,
            'registeredAt', r.registered_at,
            'x', state.x,
            'y', state.y,
            'floor', state.floor,
            'status', state.status,
            'recordedAt', state.recorded_at,
            'taskId', task.task_id,
            'taskStatus', task.status,
            'taskOrigin', task.origin,
            'taskDestination', task.destination,
            'slotNo', task.slot_no
        ) ORDER BY r.robot_id)
        FROM robot_info r
        LEFT JOIN LATERAL (
            SELECT h.x, h.y, h.floor, h.status, h.recorded_at
            FROM robot_state_history h
            WHERE h.robot_id = r.robot_id
            ORDER BY h.recorded_at DESC
            LIMIT 1
        ) state ON true
        LEFT JOIN LATERAL (
            SELECT t.task_id, t.status::text, t.origin, t.destination, t.slot_no
            FROM transport_task t
            WHERE t.robot_id = r.robot_id
              AND t.status NOT IN ('인계완료', '취소', '실패')
            ORDER BY t.created_at DESC
            LIMIT 1
        ) task ON true
    ), '[]'::json),
    'tasks', COALESCE((
        SELECT json_agg(json_build_object(
            'taskId', t.task_id,
            'status', t.status::text,
            'specimenId', t.specimen_id,
            'testType', s.test_type,
            'priority', s.priority,
            'trayId', t.tray_id,
            'robotId', t.robot_id,
            'robotName', r.robot_name,
            'slotNo', t.slot_no,
            'origin', t.origin,
            'destination', t.destination,
            'departedAt', t.departed_at,
            'arrivedAt', t.arrived_at,
            'cancelledAt', t.cancelled_at,
            'cancelReason', t.cancel_reason,
            'createdAt', t.created_at
        ) ORDER BY
            CASE t.status
                WHEN '운송중' THEN 1 WHEN '픽업중' THEN 2 WHEN '배정' THEN 3
                WHEN '대기' THEN 4 WHEN '실패' THEN 5 WHEN '취소' THEN 6
                ELSE 7
            END,
            t.created_at DESC)
        FROM transport_task t
        JOIN specimen s ON s.specimen_id = t.specimen_id
        LEFT JOIN robot_info r ON r.robot_id = t.robot_id
    ), '[]'::json),
    'specimens', COALESCE((
        SELECT json_agg(json_build_object(
            'specimenId', s.specimen_id,
            'patientId', s.patient_id,
            'testType', s.test_type,
            'priority', s.priority,
            'trayId', s.tray_id,
            'collectedAt', s.collected_at,
            'createdAt', s.created_at,
            'taskId', task.task_id,
            'taskStatus', task.status
        ) ORDER BY s.priority, s.collected_at DESC)
        FROM specimen s
        LEFT JOIN LATERAL (
            SELECT t.task_id, t.status::text
            FROM transport_task t
            WHERE t.specimen_id = s.specimen_id
            ORDER BY t.created_at DESC
            LIMIT 1
        ) task ON true
    ), '[]'::json),
    'events', COALESCE((
        SELECT json_agg(json_build_object(
            'eventId', event.event_id,
            'robotId', event.robot_id,
            'robotName', event.robot_name,
            'eventType', event.event_type,
            'detail', event.detail,
            'occurredAt', event.occurred_at
        ) ORDER BY event.occurred_at DESC)
        FROM (
            SELECT e.*, r.robot_name
            FROM robot_event_log e
            JOIN robot_info r ON r.robot_id = e.robot_id
            ORDER BY e.occurred_at DESC
            LIMIT 12
        ) event
    ), '[]'::json),
    'statusLogs', COALESCE((
        SELECT json_agg(json_build_object(
            'logId', log.log_id,
            'taskId', log.task_id,
            'fromStatus', log.from_status,
            'toStatus', log.to_status,
            'changedAt', log.changed_at,
            'changedBy', log.changed_by
        ) ORDER BY log.changed_at DESC)
        FROM (
            SELECT l.*
            FROM task_status_log l
            ORDER BY l.changed_at DESC
            LIMIT 12
        ) log
    ), '[]'::json)
)::text;
"""


def fetch_dashboard() -> dict:
    """Fetch one internally consistent dashboard snapshot from PostgreSQL."""
    command = [
        "docker",
        "exec",
        DB_CONTAINER,
        "psql",
        "-X",
        "-U",
        DB_USER,
        "-d",
        DB_NAME,
        "-At",
        "-v",
        "ON_ERROR_STOP=1",
        "-c",
        DASHBOARD_SQL,
    ]
    try:
        result = subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
            timeout=8,
        )
        return json.loads(result.stdout.strip())
    except (subprocess.SubprocessError, json.JSONDecodeError) as exc:
        detail = getattr(exc, "stderr", "") or str(exc)
        raise RuntimeError(f"DB 조회 실패: {detail.strip()}") from exc


class DashboardHandler(SimpleHTTPRequestHandler):
    server_version = "RobotControl/1.0"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, format: str, *args) -> None:
        print(f"[{self.log_date_time_string()}] {format % args}")

    def send_json(self, payload: dict, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def serve_map(self) -> None:
        if not MAP_PATH.is_file():
            self.send_error(HTTPStatus.NOT_FOUND, "맵 이미지가 없습니다")
            return
        body = MAP_PATH.read_bytes()
        mime = mimetypes.guess_type(MAP_PATH.name)[0] or "image/png"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mime)
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
                        message = f"event: dashboard\ndata: {payload}\n\n"
                        self.wfile.write(message.encode("utf-8"))
                        self.wfile.flush()
                        last_payload = payload
                        last_sent = now
                except RuntimeError as exc:
                    error = json.dumps({"message": str(exc)}, ensure_ascii=False)
                    self.wfile.write(f"event: db-error\ndata: {error}\n\n".encode("utf-8"))
                    self.wfile.flush()
                time.sleep(1.5)
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/dashboard":
            try:
                self.send_json(fetch_dashboard())
            except RuntimeError as exc:
                self.send_json(
                    {"error": "database_unavailable", "message": str(exc)},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
            return
        if path == "/api/health":
            try:
                snapshot = fetch_dashboard()
                self.send_json(
                    {
                        "status": "ok",
                        "database": DB_NAME,
                        "container": DB_CONTAINER,
                        "serverTime": snapshot.get("serverTime"),
                    }
                )
            except RuntimeError as exc:
                self.send_json(
                    {"status": "error", "message": str(exc)},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
            return
        if path == "/api/stream":
            self.serve_stream()
            return
        if path == "/assets/integration_hospital.png":
            self.serve_map()
            return
        if path == "/":
            self.path = "/index.html"
        super().do_GET()


def main() -> None:
    parser = argparse.ArgumentParser(description="검체 이송 로봇 관제 웹 서버")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    if not MAP_PATH.is_file():
        raise SystemExit(f"맵 이미지를 찾을 수 없습니다: {MAP_PATH}")

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    server.daemon_threads = True
    print(f"관제 웹페이지: http://{args.host}:{args.port}")
    print(f"DB: {DB_CONTAINER}/{DB_NAME} | Map: {MAP_PATH}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n서버를 종료합니다.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
