# 병원 검체 운송 통합 시스템 — 코드 구성 계획

작성 2026-09-25, 브랜치 `feature/system-integration` (기준: `feature/hospital-dispatch` `d3e481d`).
현황은 세 브랜치의 코드를 직접 읽고 확인했다.

## 1. 목표

| 기능 | 내용 | 출처 브랜치 |
|---|---|---|
| 픽 앤 플레이스 | **East Desk = 검체 채취실**: 책상 트레이 → 로봇 랙 적재(YOLO 위치 확인 → 중앙 정렬 → 재검출 파지 → 랙 자동 정렬) → 랙 ArUco로 긴급도 판독. **West Desk = 검체 분석실**: 랙 → 책상 하역 | `feature/hhj-0923` `pick_and_place_detection.py` |
| 자율주행 | Nav2 FollowPath 고정 차선 + MPPI + 책상 옆 라이다 도킹 + 사람 회피 | `feature/hospital-dispatch` |
| 관제 / DB | PostgreSQL(작업·트레이·이력) + Redis(로봇 상태·heartbeat·경로·이벤트 스트림) 기반 배차·경로 결정 | `feature/note` `DB_container/` |

추가 요구사항 (2026-09-25 확정)
- **로봇 최대 3대**, 여러 PC에 나눠 실행할 수 있어야 한다 (§3.4).
- 차선은 **구역(칸) 단위로 나누고, 여러 차선을 두고, 경로를 실시간으로 조합**할 수 있어야 한다 (§3.2).
- 긴급도는 **3 이 가장 높다**. 높은 긴급도 트레이가 **있을수록, 많을수록** 우선한다 (§3.3).

## 2. 현황 요약 (코드로 확인)

### 2.1 픽 앤 플레이스 (`pick_and_place_detection.py`, 2464줄)
- Isaac standalone 한 파일에 **씬 로드·사람·navmesh·라이다 그래프·팔 IK·상태기계·ROS 핸드셰이크**가 모두 들어 있다.
- 상태기계는 `main()` 안의 지역 변수 + `nonlocal` 로 구현 → 로봇 2대 인스턴스화 불가.
- 상태: `scan → descend → settle → center → settle → pick` (×3칸) → `observe_go/settle/home`(ArUco 긴급도) → `wait_unload` → `unload`(×3), 실패 시 `recover`.
- 외부 연동: `/mission_state`(Isaac→주행), `/nav_done`(주행→Isaac), `/tray_detection`, `/aruco_markers` — 전부 전역 토픽, 로봇 1대 전제.
- 검출은 별도 프로세스 `admin_ws/src/tray_detector/detect_node.py`(yolo-venv, py3.12). Isaac 번들 파이썬(3.11)과 한 프로세스에 합칠 수 없다.
- 로봇 기준 좌표 상수(`POINT4_TCP`, `RACK_SLOTS`, `DESK_ROW_CENTER`, `DESK_Z_M`, `SCAN_ROTATE_DEG`)는 **옛 씬 기준**이다. 파일 주석에 "hospital_integration_human.usd 에는 트레이/책상을 아직 안 옮겼으므로 `--drive-only` 로만 실행" 이라고 적혀 있다.

### 2.2 자율주행 (`feature/hospital-dispatch`)
- `hospital_mission.py`: 방향별 고정 차선 — `specimen_to_lab`=`lane_upper`, `lab_to_specimen`=`lane_lower`. 출발→MPPI 이송→도착 3단계 + `TableDocking`.
- 왕복 연속 성공(약 395 s), 도킹 간격 0.150 ± 0.003 m.
- 토픽/프레임이 전부 전역(`/scan`, `/plan`, `/initialpose`, `map`, `base_link`) → 네임스페이스 작업 필요.
- **라이다 처리량 한계**: Isaac 라이다 발행 경로가 약 5 MB/s에서 막혔고, 경량화 후 1대 약 2.6 MB/s(306 KB × 8.4 Hz). 2대면 한계에 근접한다 → §6 R1.

### 2.3 DB (`feature/note`)
- `hospital_amr_db_v5_schema.sql`: `robot_info`, `tray`, `transport_task`(트레이 1~3개/슬롯), `task_status_log`, `robot_event_log`, `robot_state_history`.
- `hospital_amr_db_v5_module.py`: psycopg2 + redis 단위 함수. Redis 키 `robot:{id}:state|heartbeat|route`, 스트림 `stream:robot_events` → SQL 워커.
- `manage.py`: 1단계 관리 노드(로봇 등록, is_active, 위치→Redis). 배차/경로 결정은 아직 없음.
- Docker 컨테이너: `robotdb3_sql`(postgres:16, 5432), `robotdb3_nosql`(redis:8.8, 6379).

### 2.4 명칭 불일치 — 반드시 정리
현재 주행 코드는 **East_DockDesk = `lab`(분석실), West_DockDesk = `specimen`(검체실)** 이다.
요구사항은 **East = 채취실(적재), West = 분석실(하역)** 이므로 의미가 반대다.
물리 경로는 그대로 쓰고 이름만 바꾼다.

| 새 이름 | 책상 | 역할 | 기존 코드 이름 |
|---|---|---|---|
| `collection` | East_DockDesk (x≈20.8) | 적재 + 긴급도 판독 | `lab` |
| `analysis` | West_DockDesk (x≈−45.4) | 하역 | `specimen` |
| `collection_to_analysis` | lane_lower | 운송 | `lab_to_specimen` |
| `analysis_to_collection` | lane_upper | 복귀 | `specimen_to_lab` |

두 차선이 **일방통행 순환 루프**(운송=아래, 복귀=위)를 이룬다. 로봇끼리 정면으로 마주칠 일이 없으므로,
옛 `docs/PLAN.md`(feature/note)의 WHCA* 격자(2.94 m 셀 전체 격자)는 쓰지 않는다. 대신 차선을 따라
나눈 **구역 예약(§3.2)** 으로 책상 점유·대기열·차간 거리를 한 가지 방식으로 처리한다.

## 3. 목표 아키텍처

```
┌──────────────────────────── 관제 PC ─────────────────────────────┐
│  PostgreSQL(robotdb3_sql)   Redis(robotdb3_nosql)   [Docker]    │
│        ▲  작업/트레이/이력        ▲  state·heartbeat·route·events │
│        └──────────┬───────────────┘                              │
│         fleet_manager (rclpy)                                    │
│          - 트레이 긴급도 → transport_task 생성/우선순위           │
│          - 로봇 배정, 책상 점유 lock, 대기열, 차간 거리            │
│          - robot:{id}:route 기록, 로봇에 작업 지시                │
└──────────────┬───────────────────────────▲───────────────────────┘
               │ /robotN/task (지시)        │ /robotN/agent_status
┌──────────────▼───────────────────────────┴───────────────────────┐
│  robot_agent × N (rclpy, ns=/robotN)                             │
│   단계 실행: 적재 → 출발 → 이송 → 도킹 → 하역 → 복귀               │
│   주행 = hospital_mission 라이브러리화 / 팔 = arm 명령 토픽        │
│   Redis state/heartbeat/event 기록 (manage.py 기능 흡수)          │
├──────────────┬───────────────────────────▲───────────────────────┤
│  Nav2 × N (ns=/robotN, hospital 파라미터)                         │
└──────────────┼───────────────────────────┼───────────────────────┘
               │ /robotN/arm/command        │ /robotN/arm/status
┌──────────────▼───────────────────────────┴───────────────────────┐
│  Isaac Sim (번들 py3.11) — 병원 씬 + 로봇 N대 + 사람               │
│   run_system_sim.py : 씬/로봇/ROS 그래프(ns별) 구성, 스텝 루프       │
│   ArmTaskController × N : LOAD / UNLOAD 상태기계 (P&P 분리 결과)    │
└──────────────┬───────────────────────────────────────────────────┘
               │ /robotN/wrist_camera/*  ↔  /robotN/tray_detection, aruco_markers
┌──────────────▼───────────────────────────────────────────────────┐
│  tray_detector × N (yolo-venv, ns=/robotN)                        │
└──────────────────────────────────────────────────────────────────┘
```

### 3.1 인터페이스 (기존 `/mission_state`, `/nav_done` 대체)

| 토픽 | 방향 | 타입 | 내용 |
|---|---|---|---|
| `/robotN/arm/command` | agent → Isaac | `std_msgs/String` | `load`, `unload`, `stop` |
| `/robotN/arm/status` | Isaac → agent | `std_msgs/String`(JSON) | `{"state","command","result":"running/done/failed","urgency":[..3],"loaded":[..3]}` |
| `/robotN/task` | fleet → agent | `std_msgs/String`(JSON) | `{"task_id","origin","destination","route"}` |
| `/robotN/agent_status` | agent → fleet | `std_msgs/String`(JSON) | 단계, 결과 |

**파이썬 제약 (전 단계 공통)**: Isaac Sim **5.1.0** 은 번들 **Python 3.11** 이고, 시스템 ROS 2 Jazzy 의 rclpy 는
Python 3.12 용이라 Isaac 프로세스 안에서 쓸 수 없다. 따라서
- Isaac 쪽 ROS 입출력은 전부 **OmniGraph ROS2 노드**(Publisher/Subscriber/CameraHelper/RtxLidarHelper 등)로 한다.
  팔 명령/상태도 OmniGraph ROS2 Subscriber/Publisher(String)로 구현한다(기존 `/mission_state` 방식과 같다).
- rclpy 가 필요한 코드(robot_agent, fleet_manager, tray_detector, 측정 도구)는 **별도 프로세스**(시스템 python3.12
  또는 yolo-venv)로 둔다. Isaac 스크립트에서 rclpy 를 import 하지 않는다.
- 커스텀 msg 패키지는 만들지 않는다(OmniGraph 제네릭 노드에서 쓰기 번거롭고 JSON String 으로 충분).

### 3.2 차선 그래프와 구역 예약

철도 폐색(block signal) 방식이다. 관제가 차선을 **구역(zone)** 으로 나눠 로봇에게 앞 구역을 예약해 주고,
예약받은 구역까지만 달리게 한다. 로봇끼리 서로 센서로 보지 못해도(다른 PC의 Isaac, §3.4) 충돌하지 않는다.

- **차선 그래프** (`lanes.yaml` + `lane_graph.py`)
  - 노드: 책상 도킹(`collection_dock`, `analysis_dock`), 정류장(staging), 대기 칸(queue), 분기/합류점.
  - 간선: 기존 기하 원소(`line`/`arc`) 목록. 지금의 `LANE_UPPER`/`LANE_LOWER` 를 간선으로 쪼개 옮긴다.
  - 차선을 추가하려면 yaml 에 간선만 추가한다(예: 대기 칸 우회로, 병렬 차선).
- **구역 분할**: 간선을 로봇 길이 + 여유(초기 3.0 m, footprint 1.86 m + 제동 거리) 단위로 자동 분할.
  도킹 구역·정류장·대기 칸은 각각 1구역.
- **실시간 경로 생성**: 배정 시 그래프에서 최단 경로(가중치 = 길이 + 점유/정체 비용)를 구하고,
  간선 기하를 이어 `nav_msgs/Path` 를 그 자리에서 만든다. 막힌 구역(정적 장애물 신고)이 생기면
  그 간선 비용을 올려 다음 경로부터 우회한다. 결과는 Redis `robot:{id}:route` 에 기록.
- **예약 규칙**
  - 로봇은 현재 구역 + 앞 `k` 구역(초기 2)을 예약해야 진입한다. 한 구역 = 한 로봇.
  - 예약 경계 끝에 멈춤 목표를 두므로 앞 로봇과 최소 한 구역 간격이 유지된다.
  - 로봇 N대에 구역이 N+1개 이상인 순환 루프이면 교착이 없다. 루프 구역 수는 수십 개라 3대는 여유.
  - heartbeat(Redis TTL 3 s) 가 끊긴 로봇의 예약은 유지(위치를 모르므로 풀지 않음)하고 경고.
- **책상 대기열**: 각 책상 앞에 대기 칸 2개(로봇 3대 기준 최대 2대 대기). 대기 칸은 가능하면
  **옆으로 나란히**(칸 나누기) 두어, 우선순위가 높은 로봇이 앞 로봇을 추월해 먼저 책상에 들어가게 한다.
  위치는 P6 에서 지도(여유 폭)를 보고 확정. 폭이 안 되면 직렬 대기 + 우선순위는 배차에서만 반영.

### 3.3 우선순위

- ArUco 판독 → 긴급도: 상 = **3**, 중 = 2, 하 = 1, 미검출 = 1 (DB `tray.priority` 에 그대로 저장, 3 이 가장 높음).
  스키마의 `priority DEFAULT 3` 은 "가장 긴급" 이 기본값이 되므로 **DEFAULT 1** 로 바꾼다.
- 작업 점수 = `(3 개수, 2 개수, 1 개수)` 를 사전식으로 내림차순 비교, 같으면 먼저 만든 작업.
  - 예: `[3,1,1]` > `[2,2,2]` (높은 것이 있음), `[3,3,1]` > `[3,2,2]` (높은 것이 많음).
- 반영 위치: ① 구역 예약 경합(합류점·대기 칸 진입) ② 책상 진입 순서 ③ 경로 선택(우선순위 높은 로봇에 짧은 경로).
  교착 방지 규칙(예약 순서)이 우선순위보다 앞선다 — 이미 예약된 구역을 빼앗지 않는다.

### 3.4 다중 PC 배치

| 구성 요소 | 위치 | 비고 |
|---|---|---|
| PostgreSQL, Redis, fleet_manager | 관제 PC 1대 | DB 접속 주소는 환경변수(`HOSPITAL_PG_DSN`, `HOSPITAL_REDIS_URL`)로. `localhost` 하드코딩 제거 |
| 로봇 스택(Nav2, robot_agent, tray_detector) | 로봇마다 아무 PC | `/robotN` 네임스페이스, 같은 `ROS_DOMAIN_ID` |
| Isaac Sim (5.1.0, py3.11) | 로봇 1~3대를 한 PC 에 두거나, PC 마다 따로 | 따로 두면 서로 안 보이므로 로봇 간 안전은 §3.2 예약만으로 보장. 한 PC 3대까지 가능(§7 P1) |

- 로봇 식별자: `robot_name`(`AMR-01..03`) ↔ 네임스페이스(`robot1..3`) ↔ `robot_info.robot_id`.
- PC 사이 대용량 토픽(라이다, 카메라)은 흘리지 않는다 — 각 로봇 스택이 자기 Isaac 과 같은 PC 에 있거나
  `ROS_LOCALHOST_ONLY`/DDS 설정으로 제한. PC 사이에는 작업·상태 토픽과 DB 연결만 오간다.

### 3.5 디렉터리 구성 (계획)

```
isaacpjt/system/                     # Isaac 번들 파이썬
  run_system_sim.py                  # 진입점: 씬 + 로봇 N대 + 사람 + ROS 그래프 + 루프
  scene.py                           # 씬 로드, 사람/navmesh, 라이다 경량화(run_hospital_sim.py 에서)
  robots.py                          # 로봇 N대 등록, 드라이브/그리퍼 설정, ns별 OmniGraph
  arm/
    config.py                        # 상수 (로봇/책상/랙 좌표, 속도, 허용치)
    geometry.py                      # quat/좌표 변환 (순수 numpy, CPU 테스트 가능)
    motion.py                        # PickPlaceSequence, 스텝 빌더, SingularityGuardedIK
    perception.py                    # ns별 검출/마커 구독 읽기
    tray.py                          # 랙 정렬/파지 캡처
    controller.py                    # ArmTaskController: load()/unload()/tick()
src/hospital_system/                 # ament_python, 관제 PC py3.12
  hospital_system/db.py              # hospital_amr_db_v5_module (feature/note)
  hospital_system/fleet_manager.py
  hospital_system/robot_agent.py
  hospital_system/lane_graph.py      # 차선 그래프, 구역 분할, 경로 생성, 예약 (순수 파이썬, CPU 테스트)
  hospital_system/priority.py        # 긴급도 → 작업 점수
  config/lanes.yaml                  # 책상·정류장·대기 칸·차선 간선 기하
  sql/hospital_amr_db_v5_schema.sql
  launch/system.launch.py            # DB 확인 → fleet_manager + 로봇별 (Nav2, agent, detector)
src/nova_carter/nav_to_goal/         # hospital_mission 을 ns 대응 라이브러리로 정리
admin_ws/src/tray_detector/          # 토픽 ns 파라미터화
```

`pick_and_place_detection.py`, `hospital_mission` CLI 는 회귀 비교용으로 남긴다.

## 4. 단계별 계획

각 단계는 **완료 판정을 통과하고 커밋한 뒤** 다음으로 넘어간다.

| # | 단계 | 산출물 | 완료 판정 |
|---|---|---|---|
| **P0** | 브랜치 통합 | `feature/note`(→ hhj-0923 포함) 병합, 충돌 해결, 빌드 | colcon 빌드 성공, 기존 CPU 테스트 통과, 단일 로봇 왕복 회귀 1회 |
| **P1** | Isaac 다중 로봇 성능 스파이크 | 병원 씬에 로봇 2대, 3대 + 사람 + 라이다 + 손목 카메라 | 로봇별 `/scan` ≥ 6 Hz, 최대 간격 < 0.5 s. 한 PC 에서 몇 대까지 되는지 기록 → 다중 PC 배치 기준 |
| **P2** | P&P 기능 분리 | `isaacpjt/system/arm/*`, `ArmTaskController`, 단일 로봇 호스트 | 로봇 1대가 `arm/command load` → 적재 3칸·긴급도 판독, `unload` → 하역. 기존 파일과 동일 동작 |
| **P3** | 병원 씬 작업대 배치 | East 책상에 트레이 스폰, 도킹 자세 기준 책상/랙 좌표 재측정 | 도킹 자세에서 적재 3/3, West 도킹 자세에서 하역 3/3 |
| **P4** | 단일 로봇 E2E (DB 없음) | `robot_agent`, `hospital_mission` 라이브러리화/이름 변경 | 적재→운송→하역→복귀 1사이클 무개입 성공 |
| **P5** | DB 연동 | `hospital_system/db.py`, 컨테이너 확인, agent 상태/이벤트 기록, 트레이·작업 기록 | 사이클 1회 후 `tray`/`transport_task`/`task_status_log` 정합, Redis state 갱신 |
| **P6** | 관제 | `lane_graph`(구역 분할·경로 생성·예약), `priority`, `fleet_manager`(작업 생성·배정·route 기록) | CPU 테스트: 가상 로봇 3대 순환 1000 스텝 충돌/교착 0, 우선순위 순서 검증. 실기: 로봇 1대 관제 지시만으로 2사이클 |
| **P7** | 다중 로봇 | Nav2/agent/detector 네임스페이스, 로봇 2대 → 3대, 필요 시 PC 분산 | 동시 운용, 구역 중복 점유 0, 책상 동시 점유 0, 긴급도 높은 작업 먼저 하역, DB 정합 |

P1 을 앞에 둔 이유: 2대 부하가 안 되면 P7 설계(카메라·라이다 해상도, 헤드리스 등)가 바뀌므로 가장 먼저 확인한다.

## 5. 결정이 필요한 항목

| # | 항목 | 제안 기본값 |
|---|---|---|
모두 확정 (2026-09-25).

| # | 항목 | 결정 |
|---|---|---|
| D1 | 로봇 대수 | 2대로 시작, 설계·테스트는 3대 기준, 다중 PC 가능 |
| D2 | 긴급도 ↔ DB `priority` | ArUco 상(id 2)→**3**, 중(id 1)→2, 하(id 0)→1, 미검출(−1)→1. 3 이 가장 긴급. 점수는 §3.3 |
| D3 | 트레이 ID | ArUco는 긴급도 3종뿐이라 개체 식별 불가 → `TR{YYYYMMDD}-{task_id}-S{slot}` 로 생성 |
| D4 | 작업(transport_task) 생성 시점 | 적재 후 긴급도 판독이 끝났을 때 트레이 3개로 1건 생성 → 즉시 해당 로봇에 배정 |
| D5 | 대기 위치 | 책상마다 대기 칸 2개(§3.2). 위치는 P6 에서 지도로 확정 |
| D6 | 맵 충돌 | `feature/note` 의 `hospital_integration_human` 맵(map_excluded, 원점 변경) 대신 주행을 튜닝한 `hospital-dispatch` 맵 유지 |
| D7 | USD 충돌 (`robot_nova.usd`, `integration_*.usd`) | `hospital-dispatch` 쪽 유지. P&P에 필요한 차이(카메라 등)는 P2에서 확인 후 반영 |

## 6. 위험

| # | 내용 | 대응 |
|---|---|---|
| R1 | ~~Isaac 라이다 발행 약 5 MB/s 한계~~ → P1 에서 해소 확인(3대 합계 21 MB/s). 남은 병목은 CPU(시뮬 실시간 비율 0.67) | Nav2 ×3(MPPI) 를 같은 PC 에 올렸을 때 재측정(P7). 부족하면 Nav2 를 다른 PC 로 |
| R7 | 손목 카메라 3대 동시 발행 시 약 2 Hz, 최대 간격 2.6 s | 팔이 일하는 로봇만 켜기(P2 에서 render product 켜고 끄기 검토). P&P 는 프레임 수 기준 대기라 느려질 뿐 동작은 유지 예상 |
| R8 | Isaac 5.1 = Python 3.11, rclpy 사용 불가 | Isaac 쪽 ROS 는 OmniGraph 만, rclpy 코드는 별도 프로세스(§3.1) |
| R2 | P&P 좌표 상수가 옛 씬 기준 | P3 에서 도킹 자세 기준으로 재측정 |
| R3 | 도킹 시 책상이 로봇 **우측** 0.15 m. 팔 작업 반경(0.25–1.12 m)·스캔 방향(`SCAN_ROTATE_DEG`) 적합성 미확인 | P3 첫 작업으로 확인. 안 맞으면 도킹 간격/정지 위치 조정 |
| R4 | 정지 로봇이 분당 약 6 cm 밀림 — 적재 중(수 분) 도킹 자세 이탈 | 적재 전후 라이다 재측위(기존 `relocalize`), 적재 중 브레이크(바퀴 속도 0 유지) 확인 |
| R5 | Nav2 전역 토픽/TF 하드코딩 | P7 에서 ns + `tf` 리매핑. P4 에서 라이브러리화할 때 이름을 인자로 받도록 미리 정리 |
| R6 | ROS 도메인: 비대화형 셸은 `ROS_DOMAIN_ID` 가 0 | 모든 launch/스크립트에서 `ROS_DOMAIN_ID=136` 명시 |

## 7. 진행 기록

### P0 브랜치 통합 — 완료 (2026-09-25)
- `feature/note` 병합(`f7ca5a8`): 충돌은 바이너리/맵/사람 명령 파일뿐, 전부 dispatch 쪽 유지(D6, D7).
  - `robot_nova.usd` 에서 note 쪽 차이는 `rear_RPLidar` 활성화, 미사용 `XT_33` 페이로드 비활성화,
    그리퍼 재질 바인딩뿐이었다. 병원 주행은 둘 다 쓰지 않는다. P&P 의 후방 라이다(`/scan_rear`)는
    필요하면 P2 에서 세션 레이어로 켠다.
- `feature/hhj-0923` 추가 병합: note 에 없던 4커밋(랙 트레이 정렬 등). 충돌 없음.
- 검증
  - `pick_and_place_detection.py`, `tray_detector` = hhj-0923 과 동일. `DB_container/` = note 와 동일.
  - 병원 주행에 쓰이는 파일 전부(nav_to_goal 병원 모듈, hospital_dynamic_layer, 파라미터, 맵, BT,
    launch, `run_hospital_sim.py`, `robot_nova.usd`, `hospital_integration_human.usd`, `start_pose.json`)가
    dispatch 와 바이트 단위로 같다 → dispatch 에서 확인한 왕복 결과가 그대로 유효. 실기 재주행은 생략.
  - `colcon build` (carter_navigation, nav_to_goal, hospital_dynamic_layer) 성공.
  - `nav_to_goal` pytest: 57 통과, 1 건너뜀. flake8/pep257 2건 실패는 병합 전 dispatch 에도 있던 것(스타일).

### P1 Isaac 다중 로봇 성능 — 완료 (2026-09-25)
산출물: `isaacpjt/system/scene.py`(씬·사람·라이다 경량화·로봇 복제·손목 카메라 그래프),
`isaacpjt/system/run_fleet_sim.py`(`--robots 1..3`), `isaacpjt/system/measure_topics.py`(시스템 python3, rclpy).

**로봇 복제 방식**: 씬의 `/World/robot_nova` 와 ROS 그래프 `/World/nova_carter_ros` 를 세션 레이어에
`Sdf.CopySpec` 으로 복제(`/World/robot_nova_{i}`, `/World/robot{i}_ros`)하고, 경로를 새 로봇으로 바꾸고
모든 ROS 노드 `nodeNamespace` 를 `/robot{i}` 로 둔다. 로봇 1 도 `/robot1` 로 바꾼다. 토픽:
`/robot{i}/cmd_vel`, `chassis/odom`, `tf`, `front_3d_lidar/lidar_points`, `wrist_camera/{color,depth}/...`.
frame id 는 그대로(`base_link`, `odom`) — 로봇마다 tf 토픽이 다르다(Nav2 다중 로봇 표준 방식).

**기능 확인**
- `/robot2/cmd_vel` 0.3 m/s × 3 s → robot2 0.945 m 이동, robot1 0.005 m(정지 중 밀림 수준). 구동 분리 OK.
- 복제 로봇에서 Fabric `cannot find protoPath` 오류 31건이 나오지만, robot1 라이다가 3.5–5 m 앞 robot2 를
  정상 검출(해당 영역 점 0 → 5,032). RTX 라이다에는 영향 없음.
- 스테이지 조명은 실행 시 Default(뷰포트 메뉴와 같은 액션)로 바꾼다. `run_hospital_sim.py` 에도 반영.

**측정** (이 PC: RTX 4060 8 GB, 20코어, GUI, 사람 3명. 30 s, 벽시계 기준 Hz. 각 1회 측정이라 실행마다 ±0.1 정도 흔들림)

| 구성 | 라이다 Hz / 최대 간격 | 손목 카메라 Hz | 실시간 비율 | GPU | Isaac CPU |
|---|---|---|---|---|---|
| 1대, 전방캠 켬 | 9.84 / 0.11 s | 3.5 | 0.98 | – | – |
| 2대, 전방캠 켬 | 8.05 / 0.14 s | 2.0–2.9 | 0.81 | 73 % | – |
| 2대, 전방캠 끔 | 7.19 / 0.33 s | 2.0–2.8 | 0.72 | 54 % | 4.5 코어 |
| 3대, 전방캠 켬 | 5.96 / 0.20 s | 1.5–1.9 | 0.60 | 79 % | 4.7 코어 |
| **3대, 전방캠 끔 (기본값)** | **6.74 / 0.32 s** | 1.8–2.4 | **0.67** | 65 % | 4.0 코어 |
| 3대, 전방캠 끔, 헤드리스 | 6.63 / 0.33 s | 1.4–2.2 | 0.66 | 64 % | 3.9 코어 |
| 3대, 라이다만 | 7.87 / 0.31 s | – | 0.79 | 35 % | 3.6 코어 |

**결론**
- 판정 기준(로봇별 라이다 ≥ 6 Hz, 최대 간격 < 0.5 s)을 **3대까지 통과**. 라이다 발행 5 MB/s 한계는 재현되지 않았다
  (3대 합계 9–21 MB/s). 벽시계 Hz 가 떨어지는 것은 시뮬 자체가 느려져서이고, 시뮬 시간 기준으로는 약 10 Hz 로 스캔을 놓치지 않는다.
- 전방 스테레오 카메라는 병원 주행에 안 쓰므로 **기본값을 끔**(`--front-cam` 으로 켬). 3대에서 실시간 비율 0.60 → 0.67.
- 헤드리스는 이득이 없다(GUI 는 병목 아님). 남은 부하는 CPU(물리·사람·ROS 발행)와 손목 카메라 렌더.
- 아직 **Nav2 ×3 을 같은 PC 에 올리지 않은 수치**다. MPPI 3개가 CPU 를 더 쓰므로 P7 에서 재측정하고,
  부족하면 로봇 스택(Nav2·agent)을 다른 PC 로 분산한다(§3.4).
- 손목 카메라는 3대 동시에 약 2 Hz(R7). P2 에서 팔이 일하는 로봇만 발행하도록 켜고 끄는 방법을 넣는다.
