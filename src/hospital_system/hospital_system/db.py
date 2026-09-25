"""
hospital_system.db (feature/note DB_container/hospital_amr_db_v5_module.py 를 옮김)
병원 검체 운송 AMR — DB 구조 v5 단위 작업 함수 모듈 (psycopg2 + redis)

접속 주소는 환경변수 HOSPITAL_PG_DSN / HOSPITAL_REDIS_URL (없으면 localhost). 관제 PC 가 따로면 로봇 PC 에서
    export HOSPITAL_PG_DSN=postgresql://rokey:rokey@<관제PC>:5432/robotdb3_sql
    export HOSPITAL_REDIS_URL=redis://rokey:rokey@<관제PC>:6379/0

수행 로직(설계서) ↔ 함수
 [연결 확인]
  PostgreSQL / Redis on 여부                                  → pg_ping() / redis_ping() / db_check()
 [PostgreSQL]
  robot_info          이름으로 조회                           → robot_info_get_by_name()
  robot_info         is_active 제외 사전 insert            → robot_info_insert()
                      연동/해제 시 is_active 만 update       → robot_info_update()
  tray                모든 요소 함께 insert                   → tray_insert()
  transport_task      배정 시 task_id ~ destination insert    → transport_task_insert()
                      시간·상태·취소 내용은 이벤트 시 update  → transport_task_update()
  task_status_log     status 변경 시 insert                   → task_status_log_insert()
                      (transport_task_update(status=...) 가 같은 트랜잭션에서 자동 기록)
  robot_event_log     이벤트 발생 시 (일괄) insert            → robot_event_log_insert() / _insert_many()
  robot_state_history 주기적으로 일괄 insert                  → robot_state_history_insert() / _insert_many()
                                                                 robot_state_history_snapshot()
 [Redis]
  robot:{id}:state     짧은 주기 업데이트 (없으면 생성)       → robot_state()
  robot:{id}:heartbeat 짧은 주기 업데이트 (TTL 3초)            → robot_heartbeat()
  robot:{id}:route     이벤트 발생 시 업데이트                → robot_route()
  stream:robot_events  이벤트 발생 시 추가 후 SQL 연동        → robot_event_publish() → robot_event_sync()

사용 예)
    from hospital_system import db

    rid = db.robot_info_insert("AMR-01", "MediCart-S3", 3)
    db.robot_info_update(rid, is_active=True)
    db.robot_state(rid, status="IDLE", x=1.0, y=1.0, theta=0.0)
    db.robot_heartbeat(rid)

    db.tray_insert("TR20260923-0001", "CBC", priority=2, specimen_count=5)
    tid = db.transport_task_insert(["TR20260923-0001"], "채혈실", "혈액검사실", robot_id=rid)
    db.transport_task_update(tid, status="IN_TRANSIT", departed_at=db.NOW)

설치:  sudo apt install python3-psycopg2 python3-redis   (또는 pip install psycopg2-binary redis)
데모:  python3 -m hospital_system.db --demo   (DEMO- 접두어 데이터를 실제 DB에 기록)
"""
import json
import os
import time
from datetime import datetime, timezone

import psycopg2
from psycopg2 import sql
from psycopg2.extras import Json, RealDictCursor, execute_values
import redis

PG_DSN = os.environ.get("HOSPITAL_PG_DSN", "postgresql://rokey:rokey@localhost:5432/robotdb3_sql")
REDIS_URL = os.environ.get("HOSPITAL_REDIS_URL", "redis://rokey:rokey@localhost:6379/0")

ROBOT_STATUSES = ("IDLE", "PICK_DOCKING", "PICKING", "DELIVERING",
                  "PLACE_DOCKING", "PLACING", "RETURNING", "ERROR")
HEARTBEAT_TTL = 3                       # 초 (만료 시 통신 두절 판정)
EVENT_STREAM = "stream:robot_events"
EVENT_STREAM_MAXLEN = 100_000           # 스트림 최대 길이 (근사치 트림)


class _Now:
    """시간 컬럼에 NOW 를 넘기면 DB 서버 시각 now() 로 기록"""
    def __repr__(self):
        return "NOW"


NOW = _Now()


# =====================================================================
# 연결 관리
# =====================================================================
_pg_conn = None
_redis = None


def get_pg():
    """공용 PostgreSQL 연결 (끊겼으면 재연결)"""
    global _pg_conn
    if _pg_conn is None or _pg_conn.closed:
        _pg_conn = psycopg2.connect(PG_DSN)
    return _pg_conn


def get_redis():
    """공용 Redis 클라이언트 (문자열 디코딩)"""
    global _redis
    if _redis is None:
        _redis = redis.from_url(REDIS_URL, decode_responses=True)
    return _redis


def close_all():
    global _pg_conn, _redis
    if _pg_conn is not None and not _pg_conn.closed:
        _pg_conn.close()
    if _redis is not None:
        _redis.close()
    _pg_conn = _redis = None


def pg_ping():
    """PostgreSQL 연결 확인. 반환: True / 실패 시 예외"""
    _run("SELECT 1", ())
    return True


def redis_ping(r=None):
    """Redis 연결 확인. 반환: True / 실패 시 예외"""
    r = r or get_redis()
    return bool(r.ping())


def db_check():
    """
    PostgreSQL / Redis 상태 확인 (예외를 던지지 않음).
    반환: {"postgres": (on 여부, 오류메시지|None), "redis": (on 여부, 오류메시지|None)}
    """
    result = {}
    for name, fn in (("postgres", pg_ping), ("redis", redis_ping)):
        try:
            fn()
            result[name] = (True, None)
        except Exception as e:
            result[name] = (False, str(e))
    return result


def _json(obj):
    return Json(obj if obj is not None else {},
                dumps=lambda o: json.dumps(o, ensure_ascii=False, default=str))


def _ts(value):
    """시간 인자 → SQL 조각 + 파라미터 (None 은 DB 기본값 now())"""
    if value is None or value is NOW:
        return sql.SQL("now()"), []
    return sql.Placeholder(), [value]


def _ts_or_null(value):
    """시간 인자 → SQL 조각 + 파라미터 (None 은 NULL, NOW 는 now())"""
    if value is None:
        return sql.SQL("NULL"), []
    return _ts(value)


def _check_robot_status(status):
    if status not in ROBOT_STATUSES:
        raise ValueError(f"로봇 status 는 {ROBOT_STATUSES} 중 하나여야 합니다: {status!r}")


def _run(query, params=(), fetch=None, conn=None):
    """쿼리 1건 실행 + 커밋 (예외 시 롤백)"""
    conn = conn or get_pg()
    with conn:
        with conn.cursor(cursor_factory=RealDictCursor) as cur:
            cur.execute(query, params)
            if fetch == "one":
                return cur.fetchone()
            if fetch == "all":
                return cur.fetchall()
            return cur.rowcount


# =====================================================================
# PostgreSQL — robot_info
# =====================================================================
def robot_info_insert(robot_name, model, container_slots=3, registered_at=None, conn=None):
    """로봇 사전 등록 (is_active 제외 → DB 기본값 FALSE). 반환: robot_id"""
    ts_sql, ts_param = _ts(registered_at)
    query = sql.SQL("""
        INSERT INTO robot_info (robot_name, model, container_slots, registered_at)
        VALUES (%s, %s, %s, {})
        RETURNING robot_id
    """).format(ts_sql)
    return _run(query, [robot_name, model, container_slots] + ts_param, "one", conn)["robot_id"]


def robot_info_update(robot_id, is_active=True, conn=None):
    """로봇 연동(True) / 해제(False) 시 is_active 만 갱신. 반환: 갱신 성공 여부"""
    return _run("UPDATE robot_info SET is_active = %s WHERE robot_id = %s",
                (is_active, robot_id), conn=conn) == 1


def robot_info_get(robot_id=None, active_only=False, conn=None):
    """robot_id 지정 시 1건(dict), 아니면 목록(list[dict])"""
    if robot_id is not None:
        return _run("SELECT * FROM robot_info WHERE robot_id = %s", (robot_id,), "one", conn)
    where = "WHERE is_active" if active_only else ""
    return _run(f"SELECT * FROM robot_info {where} ORDER BY robot_id", (), "all", conn)


def robot_info_get_by_name(robot_name, conn=None):
    """robot_name 으로 로봇 1건 조회(dict). 없으면 None (같은 이름이 여럿이면 가장 먼저 등록된 것)"""
    return _run("SELECT * FROM robot_info WHERE robot_name = %s ORDER BY robot_id LIMIT 1",
                (robot_name,), "one", conn)


# =====================================================================
# PostgreSQL — tray
# =====================================================================
def tray_insert(tray_id, test_type, priority, specimen_count,
                packed_at=None, created_at=None, conn=None):
    """트레이 등록 (모든 요소 함께 insert). 반환: tray_id"""
    p_sql, p_param = _ts_or_null(packed_at)
    c_sql, c_param = _ts(created_at)
    query = sql.SQL("""
        INSERT INTO tray (tray_id, test_type, priority, specimen_count, packed_at, created_at)
        VALUES (%s, %s, %s, %s, {}, {})
        RETURNING tray_id
    """).format(p_sql, c_sql)
    params = [tray_id, test_type, priority, specimen_count] + p_param + c_param
    return _run(query, params, "one", conn)["tray_id"]


# =====================================================================
# PostgreSQL — transport_task / task_status_log
# =====================================================================
def transport_task_insert(tray_ids, origin, destination, robot_id=None, slot_nos=None,
                          departed_at=None, conn=None):
    """
    운송 작업 배정 시 insert (task_id ~ destination).
      tray_ids : 트레이 ID 리스트 (1~3개)
      slot_nos : tray_ids 와 같은 인덱스의 슬롯 번호. 생략 시 [1, 2, ...]
      status   : DB 기본값(첫 ENUM 값, 예: WAITING) — 이후 transport_task_update 로 변경
    반환: task_id
    """
    tray_ids = list(tray_ids)
    slot_nos = list(slot_nos) if slot_nos is not None else list(range(1, len(tray_ids) + 1))
    if not 1 <= len(tray_ids) <= 3:
        raise ValueError("tray_ids 는 1~3개여야 합니다.")
    if len(slot_nos) != len(tray_ids):
        raise ValueError("slot_nos 는 tray_ids 와 개수가 같아야 합니다 (같은 인덱스로 대응).")

    d_sql, d_param = _ts_or_null(departed_at)
    query = sql.SQL("""
        INSERT INTO transport_task (tray_ids, slot_nos, robot_id, origin, departed_at, destination)
        VALUES (%s::varchar[], %s::smallint[], %s, %s, {}, %s)
        RETURNING task_id
    """).format(d_sql)
    params = [tray_ids, slot_nos, robot_id, origin] + d_param + [destination]
    return _run(query, params, "one", conn)["task_id"]


def transport_task_update(task_id, status=None, robot_id=None, departed_at=None,
                          arrived_at=None, cancelled_at=None, cancel_reason=None,
                          log_status=True, conn=None):
    """
    이벤트 발생 시 작업 갱신 — 넘긴 인자만 update (None 은 변경 안 함).
      시간 인자에 NOW 를 넘기면 DB 서버 시각으로 기록.  예) departed_at=NOW
      status 가 바뀌면 같은 트랜잭션에서 task_status_log 에 (이전 → 변경) 이력 insert.
    반환: 갱신된 행(dict), 해당 작업이 없으면 None
    """
    fields = {"status": status, "robot_id": robot_id, "departed_at": departed_at,
              "arrived_at": arrived_at, "cancelled_at": cancelled_at, "cancel_reason": cancel_reason}
    fields = {k: v for k, v in fields.items() if v is not None}
    if not fields:
        raise ValueError("갱신할 인자가 없습니다.")

    sets, params = [], []
    for col, val in fields.items():
        if val is NOW:
            sets.append(sql.SQL("{} = now()").format(sql.Identifier(col)))
        else:
            sets.append(sql.SQL("{} = {}").format(sql.Identifier(col), sql.Placeholder()))
            params.append(val)
    update_q = sql.SQL("UPDATE transport_task SET {} WHERE task_id = %s RETURNING *").format(
        sql.SQL(", ").join(sets))

    conn = conn or get_pg()
    with conn:
        with conn.cursor(cursor_factory=RealDictCursor) as cur:
            cur.execute("SELECT status FROM transport_task WHERE task_id = %s FOR UPDATE", (task_id,))
            old = cur.fetchone()
            if old is None:
                return None
            cur.execute(update_q, params + [task_id])
            row = cur.fetchone()
            if log_status and status is not None and old["status"] != row["status"]:
                cur.execute("""
                    INSERT INTO task_status_log (task_id, from_status, to_status, changed_at)
                    VALUES (%s, %s, %s, now())
                """, (task_id, old["status"], row["status"]))
            return row


def task_status_log_insert(task_id, from_status, to_status, changed_at=None, conn=None):
    """작업 상태 이력 직접 insert (transport_task_update 를 쓰면 자동 기록됨). 반환: log_id"""
    ts_sql, ts_param = _ts(changed_at)
    query = sql.SQL("""
        INSERT INTO task_status_log (task_id, from_status, to_status, changed_at)
        VALUES (%s, %s, %s, {})
        RETURNING log_id
    """).format(ts_sql)
    return _run(query, [task_id, from_status, to_status] + ts_param, "one", conn)["log_id"]


def transport_task_get(task_id, with_log=False, conn=None):
    """작업 1건(dict). with_log=True 면 'status_log' 키에 상태 이력 포함"""
    row = _run("SELECT * FROM transport_task WHERE task_id = %s", (task_id,), "one", conn)
    if row is not None and with_log:
        row["status_log"] = _run("""
            SELECT from_status, to_status, changed_at FROM task_status_log
             WHERE task_id = %s ORDER BY changed_at, log_id
        """, (task_id,), "all", conn)
    return row


def tray_id_for(task_id, slot, day=None):
    """트레이 ID = TR{YYYYMMDD}-{task_id}-S{slot} (ArUco 는 긴급도 3종뿐이라 개체를 구분하지 못한다)"""
    day = day or datetime.now()
    return f"TR{day:%Y%m%d}-{task_id}-S{slot}"


def loaded_task_create(robot_id, trays, origin, destination, test_type="GENERAL", conn=None):
    """
    적재가 끝난 트레이들로 작업을 만들고 로봇에 배정한다 — 한 트랜잭션.
      trays : [(slot, priority), ...]  (slot 1~3, priority 1~3)
    tray_id 에 task_id 가 들어가고 작업 insert 는 트레이가 먼저 있어야 하므로(트리거) 시퀀스에서
    task_id 를 먼저 받는다. 상태 이력: NULL → WAITING → ASSIGNED.
    반환: (task_id, [tray_id, ...])
    """
    trays = list(trays)
    if not 1 <= len(trays) <= 3:
        raise ValueError("트레이는 1~3개여야 합니다.")
    conn = conn or get_pg()
    with conn:
        with conn.cursor(cursor_factory=RealDictCursor) as cur:
            cur.execute("SELECT nextval(pg_get_serial_sequence('transport_task', 'task_id')) AS id")
            task_id = cur.fetchone()["id"]
            tray_ids = [tray_id_for(task_id, slot) for slot, _ in trays]
            execute_values(cur, """
                INSERT INTO tray (tray_id, test_type, priority, specimen_count, packed_at) VALUES %s
            """, [(tid, test_type, int(p), 0) for tid, (_, p) in zip(tray_ids, trays)],
                template="(%s, %s, %s, %s, now())")
            cur.execute("""
                INSERT INTO transport_task (task_id, tray_ids, slot_nos, robot_id, origin, destination)
                VALUES (%s, %s::varchar[], %s::smallint[], %s, %s, %s)
            """, (task_id, tray_ids, [int(s) for s, _ in trays], robot_id, origin, destination))
            cur.execute("UPDATE transport_task SET status = 'ASSIGNED' WHERE task_id = %s", (task_id,))
            execute_values(cur, """
                INSERT INTO task_status_log (task_id, from_status, to_status) VALUES %s
            """, [(task_id, None, "WAITING"), (task_id, "WAITING", "ASSIGNED")],
                template="(%s, %s::task_status, %s::task_status)")
    return task_id, tray_ids


# =====================================================================
# PostgreSQL — robot_event_log
# =====================================================================
def robot_event_log_insert(robot_id, event_type, detail=None, occurred_at=None, conn=None):
    """로봇 이벤트 1건 insert. 반환: event_id"""
    ts_sql, ts_param = _ts(occurred_at)
    query = sql.SQL("""
        INSERT INTO robot_event_log (robot_id, event_type, detail, occurred_at)
        VALUES (%s, %s, %s, {})
        RETURNING event_id
    """).format(ts_sql)
    return _run(query, [robot_id, event_type, _json(detail)] + ts_param, "one", conn)["event_id"]


def robot_event_log_insert_many(events, conn=None):
    """
    로봇 이벤트 일괄 insert.
      events : [(robot_id, event_type, detail(dict|None), occurred_at(datetime|None)), ...]
    반환: insert 건수
    """
    rows = [(rid, et, _json(d), None if ts is NOW else ts) for rid, et, d, ts in events]
    if not rows:
        return 0
    conn = conn or get_pg()
    with conn:
        with conn.cursor() as cur:
            execute_values(cur, """
                INSERT INTO robot_event_log (robot_id, event_type, detail, occurred_at) VALUES %s
            """, rows, template="(%s, %s, %s, COALESCE(%s::timestamptz, now()))")
    return len(rows)


# =====================================================================
# PostgreSQL — robot_state_history
# =====================================================================
def robot_state_history_insert(robot_id, x, y, status, recorded_at=None, conn=None):
    """위치 이력 1건 insert"""
    _check_robot_status(status)
    ts_sql, ts_param = _ts(recorded_at)
    query = sql.SQL("""
        INSERT INTO robot_state_history (robot_id, recorded_at, x, y, status)
        VALUES (%s, {}, %s, %s, %s)
    """).format(ts_sql)
    return _run(query, [robot_id] + ts_param + [x, y, status], conn=conn)


def robot_state_history_insert_many(rows, conn=None):
    """
    위치 이력 일괄 insert (주기 스냅샷).
      rows : [(robot_id, x, y, status, recorded_at(datetime|None)), ...]
    반환: insert 건수
    """
    rows = [(rid, None if ts is NOW else ts, x, y, st) for rid, x, y, st, ts in rows]
    for r in rows:
        _check_robot_status(r[4])
    if not rows:
        return 0
    conn = conn or get_pg()
    with conn:
        with conn.cursor() as cur:
            execute_values(cur, """
                INSERT INTO robot_state_history (robot_id, recorded_at, x, y, status) VALUES %s
            """, rows, template="(%s, COALESCE(%s::timestamptz, now()), %s, %s, %s)")
    return len(rows)


def robot_state_history_snapshot(robot_ids=None, conn=None, r=None):
    """
    Redis robot:{id}:state 를 읽어 robot_state_history 에 일괄 insert (주기 작업용).
      robot_ids 생략 시 Redis 에 state 가 있는 모든 로봇.
    반환: insert 건수
    """
    r = r or get_redis()
    if robot_ids is None:
        robot_ids = sorted(int(k.split(":")[1]) for k in r.scan_iter("robot:*:state"))
    rows = []
    for rid in robot_ids:
        st = robot_state_get(rid, r=r)
        if st:
            rows.append((rid, st["x"], st["y"], st["status"], None))
    return robot_state_history_insert_many(rows, conn=conn)


# =====================================================================
# Redis — robot:{id}:state  (Hash)
# =====================================================================
def _state_key(robot_id):
    return f"robot:{robot_id}:state"


def robot_state(robot_id, status=None, x=None, y=None, theta=None, task_id=None, r=None):
    """
    robot:{id}:state 없으면 생성, 있으면 넘긴 인자만 갱신 (updated_at 은 항상 갱신).
      생성 시 기본값: status=IDLE, x=y=theta=0.0, task_id=""(없음)
      task_id="" 를 넘기면 수행 작업 해제.
    반환: 갱신 후 state(dict)
    """
    r = r or get_redis()
    if status is not None:
        _check_robot_status(status)

    fields = {"status": status, "x": x, "y": y, "theta": theta, "task_id": task_id}
    fields = {k: v for k, v in fields.items() if v is not None}
    fields["updated_at"] = int(time.time())
    defaults = {"status": "IDLE", "x": 0.0, "y": 0.0, "theta": 0.0, "task_id": ""}

    key = _state_key(robot_id)
    pipe = r.pipeline(transaction=True)
    for f, v in defaults.items():          # 키/필드가 없을 때만 기본값 채움
        if f not in fields:
            pipe.hsetnx(key, f, v)
    pipe.hset(key, mapping=fields)
    pipe.execute()
    return robot_state_get(robot_id, r=r)


def robot_state_get(robot_id, r=None):
    """state 조회 (숫자 필드 형변환). 없으면 None"""
    r = r or get_redis()
    raw = r.hgetall(_state_key(robot_id))
    if not raw:
        return None
    return {
        "status": raw.get("status"),
        "x": float(raw.get("x", 0)), "y": float(raw.get("y", 0)),
        "theta": float(raw.get("theta", 0)),
        "task_id": int(raw["task_id"]) if raw.get("task_id") else None,
        "updated_at": int(raw.get("updated_at", 0)),
    }


def robot_keys_delete(robot_id, r=None):
    """로봇 연동 해제 시 state / route / heartbeat 키 삭제. 반환: 삭제된 키 수"""
    r = r or get_redis()
    return r.delete(_state_key(robot_id), f"robot:{robot_id}:route", f"robot:{robot_id}:heartbeat")


# =====================================================================
# Redis — robot:{id}:heartbeat  (String + TTL)
# =====================================================================
def robot_heartbeat(robot_id, ttl=HEARTBEAT_TTL, r=None):
    """통신 점검 타임스탬프 기록 (TTL 초 뒤 만료 → 통신 두절). 반환: last_seen"""
    r = r or get_redis()
    now = int(time.time())
    r.set(f"robot:{robot_id}:heartbeat", now, ex=ttl)
    return now


def robot_is_alive(robot_id, r=None):
    """heartbeat 키가 살아 있으면 True"""
    r = r or get_redis()
    return r.exists(f"robot:{robot_id}:heartbeat") == 1


# =====================================================================
# Redis — robot:{id}:route  (List)
# =====================================================================
def robot_route(robot_id, waypoints, r=None):
    """
    경로 전체 교체 (이벤트 발생 시).
      waypoints : [(x, y), ...] 또는 ["x,y", ...]. 빈 리스트면 경로 삭제.
    반환: 저장된 웨이포인트 수
    """
    r = r or get_redis()
    key = f"robot:{robot_id}:route"
    items = [wp if isinstance(wp, str) else f"{wp[0]},{wp[1]}" for wp in waypoints]
    pipe = r.pipeline(transaction=True)
    pipe.delete(key)
    if items:
        pipe.rpush(key, *items)
    pipe.execute()
    return len(items)


def robot_route_get(robot_id, r=None):
    """경로 조회 → [(x, y), ...]"""
    r = r or get_redis()
    return [tuple(float(v) for v in wp.split(",")) for wp in r.lrange(f"robot:{robot_id}:route", 0, -1)]


# =====================================================================
# Redis — stream:robot_events  (Stream) → PostgreSQL robot_event_log
# =====================================================================
def robot_event_publish(robot_id, event_type, payload=None, r=None):
    """이벤트를 스트림에 추가. 반환: entry_id"""
    r = r or get_redis()
    return r.xadd(EVENT_STREAM, {
        "robot_id": robot_id,
        "event_type": event_type,
        "payload": json.dumps(payload or {}, ensure_ascii=False, default=str),
    }, maxlen=EVENT_STREAM_MAXLEN, approximate=True)


def robot_event_sync(group="sql_worker", consumer="worker-1", count=100, block_ms=None,
                     conn=None, r=None):
    """
    워커: 스트림 이벤트를 robot_event_log 에 적재 (컨슈머 그룹 + ACK).
      - 그룹이 없으면 스트림 처음부터 읽도록 생성
      - 이전에 읽고 ACK 못 한 항목(장애 등)을 먼저 재처리한 뒤 새 항목 처리
      - occurred_at 은 스트림 entry_id 의 밀리초 타임스탬프
      - block_ms 지정 시 새 이벤트를 그 시간만큼 대기 (루프에서 호출용)
    반환: 적재 건수
    """
    r = r or get_redis()
    try:
        r.xgroup_create(EVENT_STREAM, group, id="0", mkstream=True)
    except redis.ResponseError as e:
        if "BUSYGROUP" not in str(e):
            raise

    total = 0
    for start in ("0", ">"):               # 0 = 미처리(pending) 재처리, > = 새 이벤트
        resp = r.xreadgroup(group, consumer, {EVENT_STREAM: start}, count=count,
                            block=block_ms if start == ">" else None)
        entries = [e for _, items in (resp or []) for e in items if e[1]]
        if not entries:
            continue
        rows = []
        for entry_id, f in entries:
            try:
                payload = json.loads(f.get("payload") or "{}")
            except json.JSONDecodeError:
                payload = {"raw": f.get("payload")}
            ts = datetime.fromtimestamp(int(entry_id.split("-")[0]) / 1000, tz=timezone.utc)
            rows.append((int(f["robot_id"]), f["event_type"], payload, ts))
        robot_event_log_insert_many(rows, conn=conn)   # 커밋 성공 후에만 ACK
        r.xack(EVENT_STREAM, group, *[eid for eid, _ in entries])
        total += len(rows)
    return total


# =====================================================================
# 데모 (python hospital_amr_db_v5_module.py --demo)
# =====================================================================
def _demo():
    tag = datetime.now().strftime("%H%M%S")
    print("① robot_info_insert / robot_info_update")
    rid = robot_info_insert(f"DEMO-{tag}", "MediCart-S3", 3)
    print("   robot_id =", rid, "| is_active(등록 직후) =", robot_info_get(rid)["is_active"])
    robot_info_update(rid, is_active=True)
    print("   is_active(연동 후) =", robot_info_get(rid)["is_active"])

    print("② Redis robot_state / heartbeat (없으면 생성)")
    print("  ", robot_state(rid, x=1.0, y=1.0))
    robot_heartbeat(rid)
    print("   alive =", robot_is_alive(rid))

    print("③ tray_insert / transport_task_insert")
    trays = [tray_insert(f"DEMO-{tag}-{i}", "CBC", priority=2, specimen_count=4 + i) for i in (1, 2)]
    tid = transport_task_insert(trays, "채혈실", "혈액검사실", robot_id=rid)
    print("   task_id =", tid, "| trays =", trays)

    print("④ 작업 진행: transport_task_update + robot_state + route + 이벤트 발행")
    labels = [row["enumlabel"] for row in _run("""
        SELECT e.enumlabel FROM pg_enum e
          JOIN pg_type t ON t.oid = e.enumtypid
         WHERE t.oid = (SELECT atttypid FROM pg_attribute
                         WHERE attrelid = 'transport_task'::regclass AND attname = 'status')
         ORDER BY e.enumsortorder""", (), "all")]
    flow = [lb for lb in labels if not any(k in lb.upper() for k in ("CANCEL", "FAIL", "취소", "실패"))]
    robot_steps = ["PICK_DOCKING", "PICKING", "DELIVERING", "PLACE_DOCKING", "PLACING"]
    for i, st in enumerate(flow[1:]):
        extra = {}
        if i == len(flow) - 4:
            extra["departed_at"] = NOW
        if i == len(flow) - 3:
            extra["arrived_at"] = NOW
        transport_task_update(tid, status=st, **extra)
        rs = robot_steps[min(i, len(robot_steps) - 1)]
        robot_state(rid, status=rs, task_id=tid, x=1.0 + i * 5, y=1.0 + i * 3)
        if rs == "DELIVERING":
            robot_route(rid, [(1 + i * 5, 1 + i * 3), (20, 12), (35, 20)])
        robot_event_publish(rid, "STATUS_CHANGED", {"task_id": tid, "task_status": st, "robot_status": rs})
    robot_state(rid, status="RETURNING", task_id="")
    robot_route(rid, [(35, 20), (18, 10), (1, 1)])
    print("   route =", robot_route_get(rid))
    for lg in transport_task_get(tid, with_log=True)["status_log"]:
        print(f"   {lg['from_status']} → {lg['to_status']}")

    print("⑤ robot_event_sync (stream → robot_event_log)")
    print("   적재 건수 =", robot_event_sync())

    print("⑥ robot_state_history_snapshot (Redis state → 위치 이력)")
    print("   insert 건수 =", robot_state_history_snapshot([rid]))

    print("⑦ 연동 해제")
    robot_info_update(rid, is_active=False)
    print("   삭제된 Redis 키 수 =", robot_keys_delete(rid))


if __name__ == "__main__":
    import sys
    if "--demo" in sys.argv:
        try:
            _demo()
        finally:
            close_all()
    else:
        print(__doc__)
