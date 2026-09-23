BEGIN;

-- This database is used for demos, so replace the existing sample set as one unit.
TRUNCATE TABLE
    task_status_log,
    robot_event_log,
    robot_state_history,
    transport_task,
    specimen,
    robot_info
RESTART IDENTITY CASCADE;

SELECT set_config('amr.actor', 'demo_seed', true);

INSERT INTO robot_info
    (robot_id, robot_name, model, container_slots, is_active, registered_at)
VALUES
    (1, 'AMR-01', 'Nova Carter AMR', 3, true, now() - interval '90 days'),
    (2, 'AMR-02', 'Nova Carter AMR', 3, true, now() - interval '60 days'),
    (3, 'AMR-03', 'Nova Carter AMR', 3, true, now() - interval '30 days');

INSERT INTO specimen
    (specimen_id, patient_id, test_type, priority, tray_id, collected_at, created_at)
VALUES
    ('SP-20260923-001', 'PT-1001', '일반혈액검사(CBC)', 2, 'TRAY-A01', now() - interval '25 minutes', now() - interval '24 minutes'),
    ('SP-20260923-002', 'PT-1002', '응급 생화학검사',   1, 'TRAY-A02', now() - interval '18 minutes', now() - interval '17 minutes'),
    ('SP-20260923-003', 'PT-1003', '혈액응고검사',     2, 'TRAY-B01', now() - interval '12 minutes', now() - interval '11 minutes'),
    ('SP-20260923-004', 'PT-1004', '혈액형검사',       3, 'TRAY-C01', now() - interval '75 minutes', now() - interval '74 minutes'),
    ('SP-20260923-005', 'PT-1005', '간기능검사',       3, 'TRAY-C02', now() - interval '8 minutes',  now() - interval '7 minutes');

-- The INSERT trigger creates one matching task_status_log row per task.
INSERT INTO transport_task
    (task_id, specimen_id, tray_id, robot_id, slot_no, origin, destination,
     status, departed_at, arrived_at, cancelled_at, cancel_reason, created_at)
VALUES
    (1, 'SP-20260923-001', 'TRAY-A01', NULL, NULL, '검체 체취실', '검체 검사실', '대기',
     NULL, NULL, NULL, NULL, now() - interval '20 minutes'),
    (2, 'SP-20260923-002', 'TRAY-A02', 1, 1, '검체 체취실', '검체 검사실', '운송중',
     now() - interval '12 minutes', NULL, NULL, NULL, now() - interval '16 minutes'),
    (3, 'SP-20260923-003', 'TRAY-B01', 2, 2, '검체 체취실', '검체 검사실', '배정',
     NULL, NULL, NULL, NULL, now() - interval '10 minutes'),
    (4, 'SP-20260923-004', 'TRAY-C01', 3, 1, '검체 체취실', '검체 검사실', '인계완료',
     now() - interval '65 minutes', now() - interval '38 minutes', NULL, NULL, now() - interval '72 minutes'),
    (5, 'SP-20260923-005', 'TRAY-C02', NULL, NULL, '검체 체취실', '검체 검사실', '취소',
     NULL, NULL, now() - interval '4 minutes', '검체 라벨 재발행 필요', now() - interval '6 minutes');

-- Give the trigger-generated audit rows realistic demo times.
UPDATE task_status_log AS log
SET changed_at = task.created_at,
    changed_by = CASE
        WHEN task.status = '취소' THEN '검체 체취실 사용자'
        WHEN task.status = '대기' THEN '검체 체취실 단말'
        ELSE '관제 시스템'
    END
FROM transport_task AS task
WHERE task.task_id = log.task_id;

INSERT INTO robot_state_history
    (robot_id, recorded_at, x, y, floor, status)
VALUES
    (1, now() - interval '8 minutes',  4.20,  2.10, 1, '픽업중'),
    (1, now() - interval '1 minute',   8.60,  5.40, 1, '운송중'),
    (2, now() - interval '6 minutes', 12.30,  3.80, 1, '대기'),
    (2, now() - interval '30 seconds',11.90,  4.10, 1, '배정'),
    (3, now() - interval '2 minutes',  1.50, -2.20, 1, '대기');

INSERT INTO robot_event_log
    (event_id, robot_id, event_type, detail, occurred_at)
VALUES
    (1, 1, '작업배정', jsonb_build_object('task_id', 2, 'message', '응급 검체 운송 작업 배정', 'battery_percent', 82), now() - interval '15 minutes'),
    (2, 1, '픽업완료', jsonb_build_object('task_id', 2, 'tray_id', 'TRAY-A02', 'slot_no', 1, 'message', '검체 픽업 완료'), now() - interval '12 minutes'),
    (3, 2, '작업배정', jsonb_build_object('task_id', 3, 'message', '검체 체취실 픽업 대기', 'battery_percent', 67), now() - interval '9 minutes'),
    (4, 3, '인계완료', jsonb_build_object('task_id', 4, 'tray_id', 'TRAY-C01', 'message', '검체 검사실 인계 완료'), now() - interval '36 minutes'),
    (5, 3, '충전완료', jsonb_build_object('message', '충전 완료 후 대기 지점 복귀', 'battery_percent', 100), now() - interval '2 minutes');

SELECT setval(pg_get_serial_sequence('robot_info', 'robot_id'), 3, true);
SELECT setval(pg_get_serial_sequence('transport_task', 'task_id'), 5, true);
SELECT setval(pg_get_serial_sequence('task_status_log', 'log_id'), 5, true);
SELECT setval(pg_get_serial_sequence('robot_event_log', 'event_id'), 5, true);

COMMIT;
