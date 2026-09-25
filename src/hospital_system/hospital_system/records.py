"""로봇 에이전트가 DB 에 남기는 것 — Redis 상태/heartbeat/이벤트 + PostgreSQL 로봇·트레이·작업.

Redis   robot:{id}:state      단계(IDLE..ERROR), 위치, task_id
        robot:{id}:heartbeat  1 s 마다 (TTL 3 s)
        stream:robot_events   STAGE / TASK_CREATED / MISSION_RESUME / ... -> db_worker 가 robot_event_log 로
Postgres robot_info           이름(robot1..)으로 찾고 없으면 등록, 실행 중 is_active
        tray, transport_task  적재 + 긴급도 판독 뒤 1건 생성 (D4), 이후 IN_TRANSIT -> ARRIVED -> COMPLETED
        task_status_log       상태가 바뀔 때마다 (db.transport_task_update 가 같이 기록)

DB 가 도중에 끊겨도 로봇은 멈추지 않는다: 기록 실패는 경고만 남긴다 (시작 시 연결 확인은 실패하면 종료).
"""


def trays_from_load(loaded, urgency):
    """arm/status 의 loaded[k], urgency[k] (k = 랙 칸 0..2) -> [(slot 1..3, priority 1..3)] (놓인 칸만)"""
    return [(k + 1, int(urgency[k]) if k < len(urgency) else 1) for k, ok in enumerate(loaded) if ok]


class Recorder:
    def __init__(self, robot_name, model, logger, container_slots=3):
        try:
            from hospital_system import db   # psycopg2/redis 는 DB 를 쓸 때만 필요하다
        except ImportError as e:
            raise RuntimeError(f"{e} — sudo apt install python3-psycopg2 python3-redis "
                               "(DB 없이 돌리려면 -p use_db:=false)") from None
        self.db, self.log = db, logger
        status = db.db_check()
        for name, (on, err) in status.items():
            if not on:
                raise RuntimeError(f"{name} off: {err} (HOSPITAL_PG_DSN / HOSPITAL_REDIS_URL 확인)")
        row = db.robot_info_get_by_name(robot_name)
        self.robot_id = row["robot_id"] if row else db.robot_info_insert(robot_name, model, container_slots)
        db.robot_info_update(self.robot_id, is_active=True)
        self.task_id = None
        self.log.info(f"[DB] {robot_name} robot_id={self.robot_id} active "
                      f"(postgres {db.PG_DSN.split('@')[-1]}, redis {db.REDIS_URL.split('@')[-1]})")

    def _safe(self, what, fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except Exception as e:   # 기록 실패로 로봇이 멈추면 안 된다
            self.log.warn(f"[DB] {what} failed: {e}")
            return None

    # ── Redis ────────────────────────────────────────────────────
    def heartbeat(self):
        self._safe("heartbeat", self.db.robot_heartbeat, self.robot_id)

    def pose(self, x, y, theta):
        self._safe("state pose", self.db.robot_state, self.robot_id,
                   x=round(x, 3), y=round(y, 3), theta=round(theta, 4))

    def stage(self, stage, detail=""):
        # task_id "" = 수행 작업 없음
        self._safe("state", self.db.robot_state, self.robot_id, status=stage,
                   task_id=self.task_id if self.task_id is not None else "")
        self.event("STAGE", stage=stage, detail=detail)

    def event(self, event_type, **payload):
        if self.task_id is not None:
            payload.setdefault("task_id", self.task_id)
        self._safe(f"event {event_type}", self.db.robot_event_publish, self.robot_id, event_type, payload)

    # ── 작업 ─────────────────────────────────────────────────────
    def create_task(self, trays, origin, destination, test_type):
        made = self._safe("task create", self.db.loaded_task_create, self.robot_id, trays,
                          origin, destination, test_type)
        if made:
            self.task_id, tray_ids = made
            self.event("TASK_CREATED", trays=tray_ids, slots=[s for s, _ in trays],
                       priority=[p for _, p in trays])
        return self.task_id

    def task(self, status, **times):
        """status 와 시간 컬럼 갱신. times 값 True -> DB now() (예: departed_at=True)"""
        if self.task_id is None:
            return
        times = {k: self.db.NOW for k, v in times.items() if v}
        self._safe(f"task {status}", self.db.transport_task_update, self.task_id, status=status, **times)

    def finish_task(self):
        self.task_id = None

    def close(self):
        self._safe("is_active", self.db.robot_info_update, self.robot_id, is_active=False)
        self._safe("close", self.db.close_all)
