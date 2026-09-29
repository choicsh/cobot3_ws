-- =====================================================================
--  병원 검체 운송 AMR — PostgreSQL 스키마 (DB 구조 v5)
--  대상 DB : robotdb3_sql   (접속: postgresql://rokey:rokey@localhost:5432/robotdb3_sql)
--
--  실행 :  psql "postgresql://rokey:rokey@localhost:5432/robotdb3_sql" -f hospital_amr_db_v5.sql
--
--  ※ 주의: 재실행할 수 있도록 맨 앞에서 기존 테이블/타입을 DROP 합니다.
--          기존 데이터가 모두 삭제되니 필요하면 먼저 백업하세요.
--  ※ DB 자체가 없다면 superuser 로 먼저 생성:
--       CREATE DATABASE robotdb3_sql OWNER rokey;
-- =====================================================================

BEGIN;

-- ---------------------------------------------------------------------
-- 0. 초기화
-- ---------------------------------------------------------------------
DROP TABLE IF EXISTS task_status_log     CASCADE;
DROP TABLE IF EXISTS transport_task      CASCADE;
DROP TABLE IF EXISTS tray                CASCADE;
DROP TABLE IF EXISTS robot_event_log     CASCADE;
DROP TABLE IF EXISTS robot_state_history CASCADE;
DROP TABLE IF EXISTS robot_info          CASCADE;
DROP FUNCTION IF EXISTS check_task_trays() CASCADE;
DROP TYPE  IF EXISTS task_status         CASCADE;

-- ---------------------------------------------------------------------
-- 1. 작업 상태 ENUM
--    정상 흐름 : 대기 → 배정 → 픽업중 → 운송중 → 도착 → 인계완료
--    예외      : 취소 / 실패
--    (더미 데이터 스크립트는 이 정의 순서를 작업 흐름 순서로 사용합니다)
-- ---------------------------------------------------------------------
CREATE TYPE task_status AS ENUM (
    'WAITING',      -- 대기 (로봇 미배정)
    'ASSIGNED',     -- 배정
    'PICKING_UP',   -- 픽업중 (채혈실에서 트레이 적재)
    'IN_TRANSIT',   -- 운송중
    'ARRIVED',      -- 도착
    'COMPLETED',    -- 인계완료
    'CANCELLED',    -- 취소
    'FAILED'        -- 실패
);

-- ---------------------------------------------------------------------
-- 2. robot_info (로봇_정보)
--    수행 로직: is_active 제외 사전 insert → 로봇 연동/해제 시 is_active 만 update
-- ---------------------------------------------------------------------
CREATE TABLE robot_info (
    robot_id        SERIAL       PRIMARY KEY,                          -- 로봇 식별자 ID
    robot_name      VARCHAR(50)  NOT NULL UNIQUE,                      -- 표시 이름
    model           VARCHAR(50)  NOT NULL,                             -- 기종
    container_slots SMALLINT     NOT NULL DEFAULT 3
                    CHECK (container_slots BETWEEN 1 AND 3),           -- 적재 슬롯 수
    is_active       BOOLEAN      NOT NULL DEFAULT FALSE,               -- 운용 여부
    registered_at   TIMESTAMPTZ  NOT NULL DEFAULT now()                -- 등록일
);

-- ---------------------------------------------------------------------
-- 3. tray (트레이)
--    수행 로직: 모든 요소 함께 insert
-- ---------------------------------------------------------------------
CREATE TABLE tray (
    tray_id         VARCHAR(30)  PRIMARY KEY,                          -- 트레이 ID (바코드)
    test_type       VARCHAR(50)  NOT NULL,                             -- 검사 종류
    priority        SMALLINT     NOT NULL DEFAULT 1
                    CHECK (priority BETWEEN 1 AND 3),                  -- 긴급도 1~3 (3 이 가장 긴급, 미검출 = 1)
    specimen_count  SMALLINT     NOT NULL DEFAULT 0
                    CHECK (specimen_count >= 0),                       -- 담긴 검체 수
    packed_at       TIMESTAMPTZ,                                       -- 트레이 구성 시각
    created_at      TIMESTAMPTZ  NOT NULL DEFAULT now()                -- 등록 시각
);

-- ---------------------------------------------------------------------
-- 4. transport_task (운송작업)
--    수행 로직: 배정 시 task_id ~ destination 일괄 insert,
--               시간 / 상태 / 취소 내용은 이벤트 발생 시 update
--    tray_ids[i] ↔ slot_nos[i] 같은 인덱스로 대응 (최대 3개)
-- ---------------------------------------------------------------------
CREATE TABLE transport_task (
    task_id         BIGSERIAL    PRIMARY KEY,                          -- 작업 ID
    tray_ids        VARCHAR(30)[] NOT NULL,                            -- 적재 트레이 ID (FK: tray, 트리거로 검사)
    slot_nos        SMALLINT[]   NOT NULL,                             -- 트레이별 슬롯 S1~S3
    robot_id        INT          NULL
                    REFERENCES robot_info(robot_id)
                    ON UPDATE CASCADE ON DELETE SET NULL,              -- 배정 로봇
    origin          VARCHAR(50)  NOT NULL,                             -- 출발지
    departed_at     TIMESTAMPTZ,                                       -- 출발 시간
    destination     VARCHAR(50)  NOT NULL,                             -- 목적지
    arrived_at      TIMESTAMPTZ,                                       -- 도착 시간
    status          task_status  NOT NULL DEFAULT 'WAITING',           -- 현재 작업 상태
    cancelled_at    TIMESTAMPTZ,                                       -- 취소 시각
    cancel_reason   VARCHAR(200),                                      -- 취소 사유

    -- 트레이 1~3개, 두 배열 길이 동일
    CONSTRAINT chk_tray_count  CHECK (cardinality(tray_ids) BETWEEN 1 AND 3),
    CONSTRAINT chk_slot_match  CHECK (cardinality(slot_nos) = cardinality(tray_ids)),
    -- 슬롯 번호는 1~3
    CONSTRAINT chk_slot_range  CHECK (slot_nos <@ ARRAY[1,2,3]::SMALLINT[]),
    -- 배정 대기가 아니면 로봇이 있어야 함
    CONSTRAINT chk_robot_assigned CHECK (status IN ('WAITING', 'CANCELLED') OR robot_id IS NOT NULL),
    -- 도착은 출발 이후
    CONSTRAINT chk_time_order  CHECK (arrived_at IS NULL OR departed_at IS NULL OR arrived_at >= departed_at)
);

-- 배열 컬럼은 FOREIGN KEY 를 걸 수 없으므로 트리거로 참조 무결성 검사
CREATE FUNCTION check_task_trays() RETURNS trigger AS $$
DECLARE
    missing TEXT[];
BEGIN
    SELECT array_agg(t) INTO missing
      FROM unnest(NEW.tray_ids) AS t
     WHERE NOT EXISTS (SELECT 1 FROM tray WHERE tray.tray_id = t);

    IF missing IS NOT NULL THEN
        RAISE EXCEPTION 'transport_task.tray_ids 에 존재하지 않는 트레이: %', missing;
    END IF;
    IF (SELECT count(DISTINCT s) FROM unnest(NEW.slot_nos) AS s) <> cardinality(NEW.slot_nos) THEN
        RAISE EXCEPTION 'transport_task.slot_nos 에 중복 슬롯이 있습니다: %', NEW.slot_nos;
    END IF;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trg_check_task_trays
    BEFORE INSERT OR UPDATE OF tray_ids, slot_nos ON transport_task
    FOR EACH ROW EXECUTE FUNCTION check_task_trays();

-- ---------------------------------------------------------------------
-- 5. task_status_log (작업 상태 이력)
--    수행 로직: status 변경 이벤트 발생 시 insert
-- ---------------------------------------------------------------------
CREATE TABLE task_status_log (
    log_id          BIGSERIAL    PRIMARY KEY,                          -- 로그 ID
    task_id         BIGINT       NOT NULL
                    REFERENCES transport_task(task_id) ON DELETE CASCADE, -- 작업 ID
    from_status     task_status,                                       -- 이전 상태 (최초 생성 시 NULL 가능)
    to_status       task_status  NOT NULL,                             -- 변경된 상태
    changed_at      TIMESTAMPTZ  NOT NULL DEFAULT now()                -- 변경 시각
);

-- ---------------------------------------------------------------------
-- 6. robot_event_log (로봇 이벤트 이력)
--    수행 로직: 이벤트 발생 시 insert (Redis stream:robot_events 를 워커가 적재)
-- ---------------------------------------------------------------------
CREATE TABLE robot_event_log (
    event_id        BIGSERIAL    PRIMARY KEY,                          -- 이벤트 ID
    robot_id        INT          NOT NULL
                    REFERENCES robot_info(robot_id) ON DELETE CASCADE, -- 로봇 ID
    event_type      VARCHAR(50)  NOT NULL,                             -- 이벤트 종류
    detail          JSONB        NOT NULL DEFAULT '{}'::jsonb,         -- 이벤트 상세
    occurred_at     TIMESTAMPTZ  NOT NULL DEFAULT now()                -- 발생 시각
);

-- ---------------------------------------------------------------------
-- 7. robot_state_history (위치 이력)
--    수행 로직: 주기적으로 insert
--    status 값은 Redis robot:{id}:state.status 와 동일한 값 사용
-- ---------------------------------------------------------------------
CREATE TABLE robot_state_history (
    robot_id        INT          NOT NULL
                    REFERENCES robot_info(robot_id) ON DELETE CASCADE, -- 로봇 ID
    recorded_at     TIMESTAMPTZ  NOT NULL DEFAULT now(),               -- 기록 시각
    x               DOUBLE PRECISION NOT NULL,                         -- 좌표 x
    y               DOUBLE PRECISION NOT NULL,                         -- 좌표 y
    status          VARCHAR(20)  NOT NULL
                    CHECK (status IN ('IDLE', 'PICK_DOCKING', 'PICKING', 'DELIVERING',
                                      'PLACE_DOCKING', 'PLACING', 'RETURNING', 'ERROR')) -- 당시 상태
);

-- ---------------------------------------------------------------------
-- 8. 인덱스
-- ---------------------------------------------------------------------
CREATE INDEX idx_task_robot        ON transport_task (robot_id);
CREATE INDEX idx_task_status       ON transport_task (status);
CREATE INDEX idx_task_tray_ids     ON transport_task USING GIN (tray_ids);   -- WHERE tray_ids @> ARRAY['TR...']
CREATE INDEX idx_status_log_task   ON task_status_log (task_id, changed_at);
CREATE INDEX idx_event_robot_time  ON robot_event_log (robot_id, occurred_at DESC);
CREATE INDEX idx_event_type        ON robot_event_log (event_type);
CREATE INDEX idx_state_robot_time  ON robot_state_history (robot_id, recorded_at DESC);

-- ---------------------------------------------------------------------
-- 9. 코멘트
-- ---------------------------------------------------------------------
COMMENT ON TABLE  robot_info          IS '로봇_정보';
COMMENT ON TABLE  tray                IS '트레이 (운송 단위)';
COMMENT ON TABLE  transport_task      IS '운송작업 — 작업 1건에 트레이 1~3개 적재';
COMMENT ON COLUMN transport_task.tray_ids IS '적재 트레이 ID 배열 (slot_nos 와 같은 인덱스로 대응)';
COMMENT ON COLUMN transport_task.slot_nos IS '트레이별 적재 슬롯 번호 1~3';
COMMENT ON TABLE  task_status_log     IS '작업 상태 이력';
COMMENT ON TABLE  robot_event_log     IS '로봇 이벤트 이력 (Redis stream 적재)';
COMMENT ON TABLE  robot_state_history IS '로봇 위치 이력 (주기 스냅샷)';

-- ---------------------------------------------------------------------
-- 10. 권한 (rokey 가 아닌 계정으로 실행한 경우 대비)
-- ---------------------------------------------------------------------
GRANT ALL ON ALL TABLES    IN SCHEMA public TO rokey;
GRANT ALL ON ALL SEQUENCES IN SCHEMA public TO rokey;

COMMIT;
