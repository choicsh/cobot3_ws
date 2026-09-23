# 다중 로봇 트레이 운반 시스템 — 목표 정의 및 작업 계획

작성 2026-09-22. 기준 커밋 `2ef23c7`(픽앤플레이스 + 자율주행 통합).
이 문서의 모든 현황 서술은 **문서가 아니라 코드를 읽어 확인한 것**이다. 문서와 코드가
어긋난 곳은 §3 에 따로 적었다.

관련 문서 — 이 계획은 아래를 대체하지 않고 참조한다.
- `ALGORITHM.md` — 단일 로봇 적재/관측/하역 상태기계 구조도
- `HANDOFF_tray_detection.md` — 좌표계·통신 규약·튜닝 상수·삽질 기록
- `test_script/INTEGRATION.md` — WHCA* 다중로봇 조정의 ROS2 연동 설계서

---

## 1. 목표

로봇 **2대**가 도킹 스테이션 A 에서 트레이를 적재하고, 실내를 돌아다니는 사람을 피해
자율주행해 도킹 스테이션 B 에 도달한 뒤 하역한다.

1. 각 로봇은 적재 후 트레이의 **긴급도를 관측해 DB 에 기록**한다.
2. **관제 노드가 긴급도에 따라 경로 계획 우선순위를 부여**하고 경로를 정한다.
3. 하역이 끝나면 로봇이 **DB 에 작업 완료를 기록**하고, 해당 트레이들의 상태가 완료가 된다.

### 확정된 선택

| 항목 | 결정 | 근거 |
|---|---|---|
| 로봇 대수 | **2대**로 시작, 알고리즘은 3대까지 대응 | Isaac 부하가 첫 실패 지점(§7 R1) |
| 긴급도 우선순위 | **stuck-first 채택** — 절대 우선 아님 | 절대 우선은 1열 복도에서 시드 10개 중 6개 영구 교착 (`INTEGRATION.md` §2.5 실측) |
| 데이터베이스 | **Python 표준 `sqlite3`** (관제 노드 통합) | 서버 설치 0, 포트 관리 0, 로컬 파일 기반 무결성 (Ponytail / Stdlib 원칙) |
| 스테이션 점유 | **관제 노드 인메모리 Lock** (`set`/`dict`) | DB 트랜잭션 오버헤드 제거, ROS 2 Service 기반 즉시 처리 (Ponytail 원칙) |
| DB 클라이언트 위치 | **관제 노드(`fleet_manager`, py3.12)** | 로봇 주행 노드는 ROS 2 토픽/서비스로 관제와 통신, Isaac 번들 환경(3.11) 분리 유지 |

---

## 2. 현황 — 코드로 확인한 것

| 영역 | 상태 | 확인한 코드 |
|---|---|---|
| 단일 로봇 적재(랙 3칸) | 동작 | `pick_and_place_detection.py:1765,2035,2037` — `slot_index` 0→2, `RACK_SLOTS` 3칸 |
| 긴급도 ArUco 관측 | 판독됨, **로그 출력만** | `assign_slots`/`merge_urgencies`(:1368,1380) → `print`(:1863) |
| 하역(랙 3→1 → 책상) | 동작 | `build_unload_steps`(:1533), `unload_slot = len(RACK_SLOTS)-1`(:1833) |
| 트레이 검출 | 동작 | `detect_node.py` — YOLO conf≥0.85, 깊이 하위 15퍼센타일 |
| 사람 회피 주행 | 동작 | `through_pose_human_test.py` + Nav2 로컬 costmap + `/scan` |
| 도킹/이탈 | 동작 (Nav2 미사용) | `straight_drive.py` — `cmd_vel` 직진, undock 3m / dock 5m |
| 다중로봇 경로계획 | **시뮬레이터만** 검증 | `lane_sim.py` 720줄. ROS2 연동 코드 0줄 |
| 데이터베이스 | **없음** | 저장소 전체에 sqlite/mysql/psycopg 참조 없음 |
| 로봇 2대 이상 | **없음** | 토픽 전부 글로벌 하드코딩, Isaac 씬에 로봇 1대 |

### 현재 프로세스 구성 (1대 기준, 3개)

```
1) 관제 PC 검출      detect_node.py            (~/yolo-venv, py3.12)
2) Isaac Sim         python.sh                 (번들 py3.11, rclpy 불가)
3) Nav2 + 주행 미션  nav2_human_test.launch.py + through_pose_human_test
```

핸드셰이크는 `/mission_state`(Isaac→주행, 1=출발 가능)와 `/nav_done`(주행→Isaac, 1=하역 가능)
두 개뿐이다. 둘 다 `Float32MultiArray`, `data[0]` 만 쓴다.

---

## 3. 기존 문서 검증 결과 — 코드와 어긋난 곳

### V1. `HANDOFF_tray_detection.md` §8 이 오래되었다
> "트레이 여러 개 처리는 이 파일에 **없다** (한 개만 집는다)."

**틀렸다.** 코드는 랙 3칸을 순회한다 (`slot_index`를 0→2 로 올리며 `build_pick_steps(..., slot_index)`
호출, `slot_index >= len(RACK_SLOTS)` 에서 관측 단계로 전이). `ALGORITHM.md`(같은 날 작성)가 맞다.
→ **조치**: P0 에서 HANDOFF §8 수정.

### V2. 셀 크기 근거식이 틀렸다 — 2.11 m 가 아니라 **2.94 m**
`INTEGRATION.md` §3 은 "셀 한 변 ≥ 로봇 대각 길이 = sqrt(길이²+폭²)" 라고 한다.
`straight_drive.py` 주석도 "회전 필요 지름 2.11m" 로 같은 식을 쓴다.

이 식은 **회전 중심이 footprint 중심일 때만** 맞다. 실제 footprint 는 base_link 기준
`[[0.48,0.5],[0.48,-0.5],[-1.38,-0.5],[-1.38,0.5]]`(`carter_navigation_params_human_test.yaml:294,405`)
로, base_link 가 앞쪽으로 치우쳐 있다.

```
base_link 에서 가장 먼 footprint 꼭짓점 = sqrt(1.38² + 0.5²) = 1.468 m
제자리 회전이 쓸고 가는 원의 지름      = 2.936 m        (2.11 m 가 아니다)
```

→ **조치**: 셀 크기를 **2.94 m 이상**으로 잡는다. `straight_drive.py` 의 "회전 불가" 결론 자체는
그대로 유효하다(오히려 더 강해진다). 숫자만 바로잡는다.

### V3. 실측 — 도킹 지점은 격자에 올릴 수 없다
`intergration_nova.png`(1001×1001, res 0.05, 50.05×50.05 m)를 `INTEGRATION.md` §4 규칙으로
타일링해 직접 셌다.

| 셀 크기 | 격자 | FREE 셀 | 연결성분 |
|---|---|---|---|
| 2.2 m | 22×22 = 484 | **37** | 1개 (전부 연결) |
| 2.94 m | 16×16 = 256 | **17** | 1개 (전부 연결) |

연결성은 문제없다. 문제는 **미션 지점들이 격자 밖이라는 것**이다.

| 지점 | 2.2 m 셀 | 2.94 m 셀 |
|---|---|---|
| INIT/스테이션 A (1.230, 3.363) | FREE | **OBST** |
| UNDOCK 끝 (-1.770, 3.363) | FREE | **OBST** |
| WP1 (-1.958, -0.352) | FREE | FREE |
| WP2 (6.641, 0.138) | FREE | FREE |
| WP3 (7.398, 17.238) | FREE | FREE |
| WP4 (5.826, 19.343) | **OBST** | **OBST** |
| DOCK 끝 (추정) | **UNKNOWN** | **UNKNOWN** |

→ **결론(설계 변경 아님, 기존 구조 유지의 근거)**: WHCA* 격자는 **WP1~WP3 중간 구간만** 덮는다.
스테이션 진입/이탈은 지금처럼 격자 밖 `cmd_vel` 직진으로 남긴다. 격자에 억지로 올리려고
셀을 잘게 쪼개면 §V2 의 회전 조건이 깨진다.

### V4. 맵의 88.5 % 가 unknown — 격자 생성 규칙에 구멍이 있다
free 107,322 px / 점유 8,989 px / **unknown 885,690 px** (전체 1,002,001).
`INTEGRATION.md` §4 는 "셀 안에 점유 픽셀이 하나라도 있으면 OBST, 아니면 FREE" 라고만 한다.
이 규칙대로면 **전부 unknown 인 셀이 FREE 가 된다** (2.2 m 격자에서 394셀, 2.94 m 에서 203셀).
로봇이 매핑되지 않은 영역으로 계획된다.

→ **조치**: 격자 생성 규칙을 3분류로 바꾼다.
```
OBST : 점유 픽셀이 하나라도 있음
FREE : 점유 0 이고 free 픽셀 비율 ≥ FREE_FRAC (초기값 0.9)
UNK  : 그 외 — 격자에서 통행 불가로 취급
```
(실측상 이 맵에서는 FREE_FRAC 0.5 와 1.0 의 결과가 같다. 경계가 뚜렷하다는 뜻이라 0.9 로 둔다.)

### V5~V9. 그 밖에 확인된 사실 (문서와 일치, 계획 입력값)
- **V5** `ARUCO_IDS = (0, 1, 2)` — 긴급도 전용. **트레이 개체 식별 불가** (§4 G1)
- **V6** `RACK_ROI = (160,190,480,320)`, `ARUCO_ZOOM = 3` — 관측 자세에 묶인 고정 픽셀 상수.
  로봇마다 관측 자세가 같아야 하거나 로봇별 ROI 가 필요하다
- **V7** `detect_node.py` 의 토픽 5개 전부 글로벌 하드코딩, 카메라 1대 전제
- **V8** `params/office/multi_robot_*.yaml` 은 footprint 0.747×0.5 / `max_vel_x` 1.8 —
  이 로봇(1.86×1.0 / 0.5)과 다르다. 그대로 쓰면 안 된다는 문서 주장이 코드로 확인됨
- **V9** `lane_sim._prio` 는 이미 stuck-first 로 구현돼 있다. 설계 원안(`(-urgency, id)` 고정)과
  다르고, 그 이유가 함수 docstring 에 실측과 함께 적혀 있다

---

## 4. 비어 있던 곳 — 결정 G1~G12

### G1. 트레이 식별 — **(mission_id, slot) 복합키로 식별한다**
"DB 에 해당 트레이들의 상태를 완료로 표시한다" 를 구현하려면 트레이를 특정해야 하는데,
ArUco id 는 0/1/2(긴급도)뿐이라 개체를 구분하지 못한다(V5).

검토한 대안과 기각 사유:
- *ArUco id 를 `urgency*16 + tray_no` 로 인코딩(0~47)* — 마커 하나로 식별+긴급도를 얻지만,
  DICT_4X4_50 에서 사용 id 를 3개→48개로 늘리면 **마커 간 해밍 여유가 줄어 오독이 정답으로
  둔갑**한다. 현재 판독 마진이 이미 없다(HANDOFF: 확대 전 2번 칸 0/56 프레임). 기각.
- *더 큰 딕셔너리(5X5/6X6)* — 같은 물리 크기에서 셀당 픽셀이 더 줄어 악화. 기각.
- *마커 물리 크기 확대* — 윗판이 5×10 cm 라 4.9 cm 에서 여유가 거의 없다. 기각.

**채택**: 트레이는 `(mission_id, slot)` 으로 식별한다. 로봇이 랙 칸 k 에 올린 트레이가 곧
그 미션의 slot k 다. ArUco 는 지금 그대로 **긴급도만** 읽는다. 마커/씬 변경 0.
전역 트레이 추적이 실제로 필요해지면 그때 id 인코딩을 도입한다 (그 시점에는 관측 자세를
더 가깝게 잡아 판독 마진을 먼저 확보해야 한다).

### G2. 작업 단위
`mission` = 로봇 1대의 1회 왕복(트레이 1~3개). `mission_tray` = 개별 트레이 행.
완료는 **트레이 단위**로 기록한다(하역이 칸 단위로 일어나므로).

### G3. 도킹 스테이션
A(적재) 1개, B(하역) 1개로 시작. 진입/이탈은 격자 밖 직진 유지(V3).
**스테이션 점유권(lock)을 관제 노드가 인메모리(`set`/`dict`)로 발급**한다 — 통과폭 1.20 m 구간에서 두 대가 마주치면 복구 불가.
DB 트랜잭션 대신 ROS 2 Service(`RequestStationLock`)로 즉시 처리하여 레이턴시 및 외부 의존성을 없앤다.

### G4. 긴급도 집계
`우선순위 = (최대 긴급도, 그 긴급도를 가진 트레이 수, 대기시간)` 사전식 비교.
미검출(-1)은 0(하)으로 간주하되 DB 에는 -1 그대로 남긴다.

### G5. 긴급도는 절대 규칙이 아니다 (확정)
계획 순서는 `(0 if stuck else 1, -urgency, id)`. 긴급도는 ① 태스크 배정 ② 계획 순서
③ 스테이션 점유 순서 3곳에 반영되고, 교착 해소가 그보다 앞선다.

### G6. DB 쓰기 주체
**관제 노드(`fleet_manager`)가 SQLite3에 기록**하거나, 주행 프로세스가 관제 노드로 상태 보고(ROS 2 Service/Topic).
복잡한 분산 큐/스풀링을 배제하고, 기록 실패 시 단순 재시도(2~3회) 후 경고 로그(`logging.warning`)를 남기며 주행 미션은 지속한다 (Ponytail 원칙).

### G7. 관제 노드
단일 `fleet_manager` 노드: 태스크 배정 + WHCA* coordinator + 인메모리 스테이션 lock + SQLite3 상태 저장 일원화.
외부 DB 데몬 없이 프로세스 내에서 가볍고 안전하게 처리한다.

### G8. 네임스페이스
`/robot1/*`, `/robot2/*` 전면 적용. 관제만 글로벌.
`detect_node` 는 **로봇당 1개** 띄운다(V7) — 토픽 이름을 파라미터로 뺀다.

### G9. Isaac 성능이 게이트 (Phase 0 조기 검증)
2대 = 아티큘레이션 2 + 손목 카메라 2(RGB+Depth) + YOLO 스트림 2 + 사람 애니메이션.
barrier 동기는 가장 느린 로봇에 전체를 맞추므로 여기서 막히면 뒤가 전부 막힌다.
**모든 개발에 앞서 Phase 0에서 즉시 측정(Fail-Fast)**하고, 미달이면 조기 튜닝하거나 씬 최적화를 진행한다.

### G10. 사람이 복도를 통째로 막는 경우
`INTEGRATION.md` §8 이 지적한 미구현 항목. "barrier 타임아웃 N회 연속인 셀을 일시적으로
OBST 처리" 를 P5 에 포함한다.

### G11. 격자 생성 규칙
V4 의 3분류 + 셀 크기 2.94 m + 격자 적용 구간은 WP1~WP3(V3).

### G12. 성공 기준
① 무충돌 ② 무교착(N스텝 내 전원 도착) ③ 긴급도 역전율 ④ 왕복 처리량 ⑤ DB 정합성
(완료 트레이 수 = 실제 하역 수). 측정 스크립트는 P6 산출물.

---

## 5. 아키텍처

```
                        관제 PC
   ┌──────────────────────────────────────────────┐
   │  fleet_manager (rclpy, py3.12)               │
   │   - 태스크 배정                              │
   │   - WHCA* coordinator                        │
   │   - 인메모리 스테이션 lock (ROS 2 Service)   │
   │   - 경량 상태 DB: fleet.db (SQLite3)         │
   └────────▲──────────────┬──────────────────────┘
            │ 상태 보고/    │ 다음 셀 목표/
            │ Lock 요청(Srv)│ Lock 승인
   ┌────────┴──────────────▼──────────────────────┐
   │  로봇 N — 주행 프로세스 (rclpy, py3.12)       │
   │   Nav2 스택 + 미션 실행                      │
   └────────▲──────────────┬──────────────────────┘
            │ /robotN/      │ /robotN/
            │ mission_state │ nav_done
   ┌────────┴──────────────▼──────────────────────┐
   │  Isaac Sim (번들 py3.11) — 로봇 2대 한 씬     │
   │   팔 상태기계 · 적재/관측/하역 · OmniGraph    │
   └────────┬──────────────────────────────────────┘
            │ /robotN/wrist_camera/*
   ┌────────▼──────────────────────────────────────┐
   │  detect_node × 2 (~/yolo-venv)                │
   └───────────────────────────────────────────────┘
```

DB(SQLite3)는 관제 노드(`fleet_manager`)가 로컬 파일(`fleet.db`)로 단일 관리한다. 로봇 주행 노드는 복잡한 SQL/DB 클라이언트 없이 순수 ROS 2 인터페이스만 사용한다. Isaac 은 DB 를 모른다.

---

## 6. DB 스키마 (SQLite3)

PostgreSQL 대신 Python 표준 `sqlite3` DDL을 적용하여 테이블 구조를 최소화한다.
진단용 `robot_event`는 ROS 2 자체 로깅(`rclpy.logging`)으로 대체하여 제거한다 (YAGNI).

```sql
-- SQLite3 DDL (관제 노드 fleet.db)

CREATE TABLE IF NOT EXISTS mission (
    mission_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id     TEXT NOT NULL,
    src_station  TEXT NOT NULL DEFAULT 'A',
    dst_station  TEXT NOT NULL DEFAULT 'B',
    state        TEXT NOT NULL CHECK (state IN ('created','loading','ready','moving','unloading','done','aborted')) DEFAULT 'created',
    priority     INTEGER NOT NULL DEFAULT 0,   -- G4 로 계산한 집계값
    created_at   DATETIME NOT NULL DEFAULT (datetime('now','localtime')),
    done_at      DATETIME
);

CREATE TABLE IF NOT EXISTS mission_tray (
    mission_id   INTEGER NOT NULL REFERENCES mission(mission_id) ON DELETE CASCADE,
    slot         INTEGER NOT NULL CHECK (slot BETWEEN 0 AND 2),   -- 랙 칸 (0, 1, 2)
    urgency      INTEGER NOT NULL DEFAULT -1,                      -- 0 하 / 1 중 / 2 상 / -1 미검출
    status       TEXT NOT NULL CHECK (status IN ('waiting','loaded','in_transit','unloaded','failed')) DEFAULT 'waiting',
    loaded_at    DATETIME,
    unloaded_at  DATETIME,
    PRIMARY KEY (mission_id, slot)                                 -- G1: 트레이 식별 복합키
);
```

상태 전이 (트레이):
```
waiting --적재+관측--> loaded --출발--> in_transit --하역--> unloaded
                                   \--실패--> failed
```

스테이션 점유는 SQL 트랜잭션 대신 관제 노드 인메모리로 3줄 처리한다:
```python
# fleet_manager 내부 인메모리 관리
held_stations = {}  # {station_id: robot_id}

def try_acquire_station(station_id, robot_id) -> bool:
    if held_stations.get(station_id) in (None, robot_id):
        held_stations[station_id] = robot_id
        return True
    return False

def release_station(station_id, robot_id):
    if held_stations.get(station_id) == robot_id:
        del held_stations[station_id]
```

---

## 7. 단계별 계획 (Ponytail 압축 로드맵)

원칙: **최대 리스크(Isaac 2대 부하 R1)를 Phase 0에서 조기 확인(Fail-Fast)하고, Python 내장 `sqlite3`와 인메모리 Lock으로 오버엔지니어링을 제거하여 최단 기간에 완수한다.**

| # | 단계 | 산출물 | 완료 판정 | 규모 |
|---|---|---|---|---|
| **Phase 0** | **Isaac 2대 성능 스파이크** | 2대 로봇 USD 씬 로드 및 토픽/센서 프레임레이트(FPS) 측정 | **성능 게이트 통과**: 목표 FPS 달성 확인 (조기 실패 차단) | 0.5일 |
| **Phase 1** | **문서 정합 및 격자 맵 전처리** | V1~V4 반영 (`HANDOFF`, `INTEGRATION` 정정, 2.94m 셀, unknown 마스크 3분류) | 문서 정합 완료 및 격자 데이터 생성 코드 확보 | 0.5일 |
| **Phase 2** | **네임스페이스(`/robot1`, `/robot2`) 분리** | Nav2 launch/params 복제, 주행 노드 네임스페이스화, detect_node 파라미터 분리 | 2대 개별 cmd_vel, odom, scan 분리 구동 확인 | 1.5일 |
| **Phase 3** | **관제 노드 & 스테이션 Lock** | `fleet_manager` 노드 생성, 인메모리 Lock (`held_stations`) 및 ROS 2 Service 인터페이스 | 2대 동시 요청 시 순차 진입, 좁은 통로 동시 진입 0회 | 1일 |
| **Phase 4** | **경량 상태 저장소 (`sqlite3`)** | 관제 노드 내 `fleet.db` DDL 적용, 적재/관측/하역 이벤트 기록 및 완료 상태 갱신 | 트레이 관측→DB 기록→하역→완료 기록 전 구간 정합 | 1일 |
| **Phase 5** | **WHCA* 연동 및 다중 주행 검증** | `lane_sim.py` 재사용 로직을 관제 노드로 연동, barrier 실행기, 지표 수집 | 실기 2대 무충돌, 500스텝 내 무교착, 긴급도 우선 처리 | 2~3일 |

**총 6.5~7.5일** (원안 11~15일 대비 약 40~50% 일정 단축).

### 롤백 지점
Phase 0, 1은 기존 코드에 영향을 주지 않는다.
Phase 2에서 처음으로 기존 launch/토픽 이름이 바뀌므로, **Phase 1 완료 커밋이 롤백 지점**이다.

---

## 8. 착수 전 확인할 지뢰

| # | 내용 | 출처 |
|---|---|---|
| R1 | **Isaac 2대 성능** — **Phase 0에서 최우선 즉시 검증 (Fail-Fast)** | G9 |
| R2 | `POINT4`/`POINT5`(랙 좌표)가 다른 씬에서 옮겨온 **미검증 값**. `V` 키로 재확인 | HANDOFF §8 |
| R3 | 셀 크기 **2.94 m** — 2.11 m 로 잡으면 격자상 합법 이동이 실행 불가가 된다 | V2 |
| R4 | 도킹 지점은 격자 밖 — 억지로 올리지 말 것 | V3 |
| R5 | unknown 셀을 FREE 로 두면 미매핑 영역으로 계획된다 | V4 |
| R6 | `RACK_ROI` 가 관측 자세에 묶인 고정 픽셀 상수 — 로봇별 자세가 다르면 재조정 | V6 |
| R7 | `multi_robot_*.yaml` 을 그대로 쓰면 안 됨(footprint/속도 불일치) | V8 |
| R8 | 놓은 뒤 후퇴가 수평 15 cm — 손가락이 테두리를 스칠 수 있음 | HANDOFF §8 |
| R9 | barrier 는 가장 느린 로봇에 전체를 맞춤. 답답하면 ADG 비동기 — 단 §2.1 주의사항 | INTEGRATION §8 |

---

## 9. 성공 기준 (G12)

| 지표 | 정의 | 목표 |
|---|---|---|
| 무충돌 | 두 로봇 footprint 겹침 횟수 | 0 |
| 무교착 | 전원이 N스텝 안에 목표 도달 | 500스텝 내 100% |
| 긴급도 역전율 | 긴급도 '상' 트레이가 '하' 보다 늦게 완료된 비율 | 측정 후 기준 설정 |
| 처리량 | 시간당 완료 트레이 수 | 측정 후 기준 설정 |
| DB 정합성 | `unloaded` 트레이 수 = 실제 하역 횟수 | 100% |
