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
| `/robotN/arm/command` | agent → Isaac | `std_msgs/String` | `load:<id>`, `unload:<id>`, `stop:<id>` (또는 JSON `{"cmd","id"}`). 문자열이 바뀔 때만 새 명령 |
| `/robotN/arm/status` | Isaac → agent | `std_msgs/String`(JSON) | `{"robot","cmd","result":"idle/running/done/failed/stopped","state","slot","loaded":[3],"aruco":[3],"urgency":[3],"unloaded":[3],"warnings":[],"detail"}` — 바뀔 때 + 0.5 s 마다 |
| `/robotN/tray_detection`, `/robotN/aruco_markers` | tray_detector → Isaac | `Float32MultiArray` | 기존 형식 그대로, 네임스페이스만 추가 |
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

### P2 픽 앤 플레이스 기능 분리 — 완료 (2026-09-25)
산출물: `isaacpjt/system/arm/` — `config.py`(상수), `geometry.py`(순수 계산), `motion.py`(스텝·시퀀스),
`trays.py`(트레이 복제·랙 정렬), `rosio.py`(OmniGraph 입출력), `controller.py`(`ArmTaskController`),
`isaacpjt/system/tests/test_arm_geometry.py`(CPU, 21개). `run_fleet_sim.py` 가 로봇마다 컨트롤러를 붙인다.
원본 `pick_and_place_detection.py` 는 그대로 둔다.

- 원본 코드는 줄 단위로 옮기고, 로봇 한 대 전제였던 전역 상태만 인스턴스로 바꿨다
  (물고 있는 트레이·정렬 기록은 로봇별, 트레이 목록·rigid 핸들은 씬 공용).
- 명령/상태: `/robotN/arm/command`, `/robotN/arm/status` (§3.1). `/mission_state`, `/nav_done`, `--drive-only` 제거.
- 틱은 물리 콜백(60 Hz) — 원본과 같은 시뮬 시간 기준. 손목 카메라는 `load` 동안만 켠다
  (IsaacCreateRenderProduct `enabled`, 끄면 렌더도 멈춘다. 2대 idle 시 실시간 비율 0.72 → 0.89).
- `tray_detector` 는 토픽을 상대 이름으로 바꿔 `--ros-args -r __ns:=/robotN` 로 로봇별 실행.
- 놓기 결과 확인을 추가했다: 시퀀스가 끝나도 트레이가 제자리가 아니면 `loaded`/`unloaded` 를 False 로,
  `warnings` 에 사유를 넣는다 (원본은 떨어뜨려도 완료로 넘어갔다).

**알고리즘 문제 발견과 수정 (원본에도 있던 것)**
- 증상: 트레이가 랙 칸막이에 걸려 약 40° 기울고, 정렬(순간이동)이 그걸 세우다 튕겨 내 테두리 위로 올라감.
  하역 때 떨어뜨림 (수평 89–115 cm). 원본 기준선에서도 같은 yaw 어긋남(방위각 −83° → 놓을 때 +7°) 확인.
- 원인 1 — 파지 yaw 가 트레이 방향이 아니라 base→트레이 **방위각**이었다. 트레이는 책상에 반듯이 놓여 있는데
  가로 ±25 cm 로 흩어져 방위각이 −69 ~ −103° 로 퍼지고, 그 차이(최대 19°)만큼 비스듬히 물어 그대로 랙에 넣었다.
  USD 실측: 칸 폭 15.4 cm, 트레이 9.1×14 cm → 19° 돌면 13.2 cm, 여유 1 cm. 파지점도 방위각 방향으로
  반폭만큼 밀어 중심이 약 1.9 cm 어긋나 여유를 넘었다. 로그상 방위각 오차 ≥ 16° 인 칸이 걸렸다.
- 원인 2 — 랙 정렬이 기울어진 트레이도 수평으로 세워 칸막이에 박힌 자세를 만들었다.
- 수정: 파지 yaw 를 트레이 축(base 기준 스폰 방위의 90° 격자)에 스냅(`geometry.square_grasp_yaw`,
  `motion.plan_grasp`, IK/손목 특이점 검사 후 안 되면 원본 방위각), 파지점도 그 방향으로 민다.
  하역 책상 자세도 같은 기준(원본은 3.4° 비스듬). 10° 넘게 기울었거나 바닥에 안 앉은 트레이는 정렬하지 않고 실패로 보고.
- 랙 놓는 점 `POINT4/5` y +3.9 cm(0.789 → 0.828): 반듯이 물게 된 뒤 트레이 중심이 칸 깊이 중심보다
  3.9 cm 앞이라 앞쪽 1.4 cm 가 랙 바닥 밖이었다. 이제 앞뒤 여유 각 2.5 cm.

**검증** (로봇 1대, 병원 씬 East 책상, 매 실행 트레이 위치·긴급도 무작위)

| | 수정 전 (3회, 9칸) | 수정 후 (6회, 18칸) |
|---|---|---|
| 놓을 때 트레이 yaw 어긋남 | 5–19° | 0–4° |
| 칸막이에 걸림(기울기 ~40°) | 3칸 | 0 |
| 랙 제자리 (칸 폭 방향 오차) | 1회 실패 | 18/18 (≤ 0.3 cm) |
| 책상 하역 제자리 | 1회 2칸 떨어뜨림 | 18/18 |
| 적재 / 하역 시간 | 69–93 s / 47–56 s | 69–78 s / 48–54 s |

- 로봇 2대 동시(수정 전 코드): robot1 적재/하역과 robot2(트레이 없음 → 재시도·복구 후 `failed`)가 서로 간섭 없음.
- ArUco 판독은 10프레임 중 칸별 3–10회(다수결 충분). 원본(카메라 15 Hz)보다 누적 프레임이 절반이다.
- 남은 일: `tray_detector` 의 `demo` 자가 시험이 0923 `RACK_ROI` 변경 뒤로 깨져 있다(이번 변경과 무관, 원본에서도 실패).

### P3 병원 씬 작업대 배치 (도킹 자세 기준) — 완료 (2026-09-25)
- 실제 도킹 자세(`hospital_docking.TABLES`, 책상 긴 변이 로봇 우측, 간격 0.15 m):
  collection = East (20.185, 13.8125, +90°), analysis = West (−44.741, 12.9125, −90°).
  두 책상은 같은 모델을 180° 돌린 것이라 도킹 관계가 대칭이다 → 팔 base 기준 책상 좌표(`DESK_SLOTS`)는 양쪽에 그대로 맞는다.
- P&P 를 맞춘 씬 USD 자세(20.270, 13.746)는 도킹 자세보다 책상에 8.5 cm 가깝다. 씬 트레이 위치 그대로면 도킹 자세에서
  파지 거리 0.96 m, 카메라 거리 0.39 m 가 되어 중앙 정렬 재검출이 빗나가고(후보 23 cm 벗어남 3회) 트레이가 10.9° 기운 채 놓였다.
- 수정: 책상 트레이 자리를 도킹한 로봇의 팔 base 기준 `TRAY_ORIGIN_BASE = (0.831, 0.101)` 로 정의 (P2 에서 검증된 상대 배치,
  책상 모서리에서 약 11 cm 안쪽). `run_fleet_sim.py --pose I:collection|analysis`, `--preload-rack I`(하역 단독 시험) 추가.
- 요청 반영: 놓기 전 대기 `PLACE_WAIT_STEPS` 120 → 60 (2 s → 1 s, 랙·책상 공통). 원본 `pick_and_place_detection.py` 에도
  P2 알고리즘 수정(트레이 축 파지, 기운 트레이 정렬 금지, 랙 놓는 점 +3.9 cm)과 같이 반영.

**검증** (robot1 @ East 도킹: load → unload, robot2 @ West 도킹 + 랙 미리 싣기: unload, 동시 실행, 대기 1 s)

| 실행 | robot1 적재 | robot1 하역 | robot2 West 하역 | 파지 거리 |
|---|---|---|---|---|
| p3a (수정 전 트레이 위치) | 2/3 (3번 칸 기움, "제자리 아님" 보고) | 2/2 | 3/3 | 0.96–0.97 m |
| p3b | 3/3, 94–103 s | 3/3 | 3/3 | 0.88–0.90 m |
| p3d | 3/3 | 3/3 | 3/3 | 0.88–0.91 m |

- 랙 칸 폭 방향 오차 ≤ 0.3 cm, 놓을 때 yaw 어긋남 ≤ 1.8°, 재시도·복구 0.
- 원본 파일은 적재 1–2번 칸까지 로그로 확인(트레이 축 파지, 어긋남 0°). 전 구간 확인은 남았다 — 원본은 매 물리 스텝 렌더라 1회 30분 이상.
- Isaac 을 종료 직후 연달아 띄울 때 NVIDIA Vulkan 렌더러에서 세그폴트 2회(약 15회 중). 코드와 무관한 드라이버 쪽 충돌로 보이며,
  시험 스크립트에 20 s 간격과 실행 중 종료 감지를 넣었다.
- 다음 사이클용 트레이 재배치(책상이 비면 새 트레이)는 P4/P6 에서 다룬다 — 지금은 시작 시 한 번 배치한다.

### P4 단일 로봇 E2E — 완료 (2026-09-26)
구성: `run_fleet_sim.py --robots 1 --pose 1:collection --robot1-global-nav` + `tray_detector`(/robot1) +
`hospital_navigation.launch.py`(RViz, `start_pose_path`) + `hospital_system/robot_agent`(/robot1).

- 새 패키지 `src/hospital_system`: `robot_agent`(PICKING → DELIVERING → PLACING → RETURNING → IDLE, DB 상태값과 같은 이름),
  `stations.py`(collection/analysis ↔ 주행 코드 lab/specimen 대응을 한 곳에). 주행은 `hospital_mission` 을 하위 프로세스로
  돌리고 종료 코드로 판단. 적재 경고가 있으면 주행하지 않는다(트레이가 그리퍼에 걸린 채 달린 사례).
- `--robot1-global-nav`: 로봇 1 주행 토픽을 전역 이름으로 두어 검증된 Nav2 를 그대로 쓴다(네임스페이스는 P7).
- RViz 에 `/hospital/reference_plan`(초록, 구간 전체 기준 경로) 표시 추가.

**실행 중 발견·수정한 문제**

| 증상 | 원인 | 수정 |
|---|---|---|
| 복귀 중 콘 3개 앞에서 15 s 양보 후 실패 | 09-25 "서 있는 사람 인식" 이후 지도에 없는 정지 물체(콘)가 영원히 사람 추적 | 한 번도 안 움직인 물체는 8 s 뒤 정적 장애물로(`obstacle_tracking.STANDING_STATIC_S`) |
| 3번 트레이가 랙에 걸려 그리퍼에 매달림 | '놓기 위' IK 보간 중 손목 특이점 근처에서 joint_4 가 한 틱 20° 이상 튐 → 트레이 흔들림 | 운반 마지막 구간을 관절 보간(IK 1회)으로 +7 cm 경유점까지, 거기서 수직 하강(`joint_ik`). 오프라인 IK 재생 33경로: 틱당 최악 11.6 → 1.9°, 충돌 후보 0. 검토했던 "툴 yaw 회전(A안)"은 이미 실린 트레이 위를 가로질러 13/33 충돌 후보라 폐기 |
| 놓기 전 대기 1 s 로 줄인 뒤 흔들리는 채 하강 | 고정 대기 | 최소 1 s + 트레이가 멈출 때까지(각속도 < 0.1 rad/s, 기울기 < 5°) 최대 2 s. 실측 1.00–1.27 s |
| 보행자와 서로 양보하며 118 s 정지(rosbag 확인) | 이미 최소 간격(0.4 m) 안이면 멀어지는 명령도 모두 거부 | 거리를 좁히지 않는 명령/경로는 0.3 m/s 이하로 허용(`escape_command`, `path_escapes`, 가드 상태 `ESCAPING`). 단위 테스트만 — 실주행에서 아직 발동 안 함 |
| 양보 예산 초과 시 미션 전체 실패, 재시도 불가 | `hospital_mission` 은 도킹 자세에서만 출발 | `resume:=true` — 현재 위치에서 같은 경로를 이어 감(`resume_stages`). 에이전트가 최대 3회 재시도 |

| 회피 뒤 차선 합류 직전에 곧게 가도 되는 곳에서 한 번 더 옆으로 비킴 (사용자 관찰) | 합류 검사(`tail_clear_for_rejoin`)가 복도 전폭(±2.8 m) × 로봇 뒤 2.8 m ~ 앞 2 m 안의 아무 추적이나 '나란히 있음' 으로 봄 — 복도 반대편 사람, 이미 지나친 사람도 걸림 | 실제 합류 경로를 차체 전체로 롤아웃해 간격이 min(1.0 m, 현재 간격) 아래로 줄지 않으면 합류(`rejoin_clear`, 순수 기하). 차체 뒷부분 옆에 선 사람은 합류하며 꺾을 때 꼬리가 그쪽으로 돌아 0.8 m 까지 가까워지므로 여전히 막는다 |
| 적재 명령이 20 s 동안 Isaac 에 전달 안 됨 (p4f) | Nav2 동시 기동 중 DDS 발견 지연 | 에이전트가 구독 연결을 기다린 뒤 최대 60 s 재전송 |

**결과**: p4e 1사이클 무개입 성공 556 s (적재 75 s, 운송 225 s, 하역 48 s, 복귀 207 s). 적재 3/3·하역 3/3 제자리,
양쪽 주행 첫 시도에 도킹. 합류 검사 수정 뒤 p4g 548 s 로 다시 성공(연속 2회) — 회피 4회 모두 합류 후 차선 복귀 완료,
합류 직후 재회피 없음(그 뒤 회피 2회는 새로 만난 보행자 1명, 콘 3개). `ESCAPING` 이 실주행에서 1회 발동.
콘은 추적 범위(8 m) 안에 들어와 8 s 가 지나기 전(약 3.7 m 앞)까지 사람으로 보여 한 번 비켜 간다 — 실패는 아니고
보수적인 우회. 이전 실행(p4a–d, p4f)은 위 표의 문제들로 실패했다.
원본 `pick_and_place_detection.py` 에는 이번 운반·대기 수정(관절 보간 경유점, 안정 대기)이 아직 없다.

### P5 DB 연동 — 완료 (2026-09-26)
구성: P4 + `db_worker`. 컨테이너 `robotdb3_sql`/`robotdb3_nosql`(feature/note 절차로 이미 구축, 스키마 적용됨).

- `DB_container/hospital_amr_db_v5_module.py` → `hospital_system/db.py` 로 이동. 접속 주소는 `HOSPITAL_PG_DSN` /
  `HOSPITAL_REDIS_URL`(없으면 localhost). `manage.py` 는 `from hospital_system import db` 로.
- 스키마 `tray.priority DEFAULT 3 → 1`(D2). 실행 중 DB 에는 `ALTER TABLE ... SET DEFAULT 1` 로 반영.
- `db.loaded_task_create`: 트레이 + 작업을 한 트랜잭션으로. tray_id 에 task_id 가 들어가고(D3) 작업 insert 는 트레이가 먼저
  있어야 하므로(트리거) 시퀀스에서 task_id 를 먼저 받는다. 상태 이력 NULL → WAITING → ASSIGNED.
- `records.py`(에이전트 쪽 기록): Redis `robot:{id}:state`(단계·task_id·AMCL 위치), heartbeat 1 s(TTL 3 s), 이벤트 스트림
  (`STAGE`, `TASK_CREATED`, `MISSION_RESUME`, `UNLOADED`); Postgres `robot_info`(이름=네임스페이스, 실행 중 is_active),
  작업 IN_TRANSIT(출발) → ARRIVED(도킹 완료) → COMPLETED(하역 제자리, 아니면 FAILED), 실패 시 FAILED.
  시작 시 DB 가 꺼져 있으면 종료(`-p use_db:=false` 로 끌 수 있음), 도중 기록 실패는 경고만.
- `PICKING_UP` 은 쓰지 않는다 — D4 로 작업이 적재·긴급도 판독 뒤에 생기기 때문. P6 에서 관제가 적재 전에 작업을 만들면 쓴다.
- 도킹 단계: `hospital_mission` 이 도킹 시작 때 `[MISSION] DOCKING: <책상>` 을 출력 → 에이전트가 PLACE_DOCKING / PICK_DOCKING.
- `db_worker`(관제 PC, ROS 없음): 스트림 → `robot_event_log`(컨슈머 그룹 + ACK), 5 s 마다 heartbeat 살아 있는 로봇의 state →
  `robot_state_history`. P6 에서 fleet_manager 로 흡수.
- 의존성: `python3-psycopg2`, `python3-redis`(package.xml). 이 PC 시스템 파이썬에는 아직 없어 시험은 `--system-site-packages`
  venv(psycopg2-binary, redis)로 돌렸다.

**결과**: p5a 1사이클 무개입 성공 573 s(적재 84 s, 운송 207 s + 도킹 24 s, 하역 51 s, 복귀 181 s + 도킹 27 s),
적재 3/3·하역 3/3 제자리. DB 정합: task 2 = 트레이 3개 `TR20260926-2-S1..3`(긴급도 3, 1, 3 = 판독값), 상태 이력
WAITING → ASSIGNED → IN_TRANSIT → ARRIVED → COMPLETED, departed_at/arrived_at 기록, 이벤트 9건 전부 `robot_event_log` 로 이동,
위치 이력 모든 단계 기록, 종료 뒤 Redis state IDLE·task_id 없음, is_active FALSE.
- 남은 점: Redis 위치는 `/amcl_pose` 라 AMCL 이 갱신할 때만 바뀐다 — 종료 시 y 13.28 로 도킹 자세(13.81)보다 0.5 m 전 값이 남았다.
  관제가 위치로 판단하는 P6 에서는 TF(map→base_link) 주기 조회로 바꾼다.
