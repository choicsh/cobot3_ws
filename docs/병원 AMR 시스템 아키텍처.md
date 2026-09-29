# 병원 AMR 검체 운송 시스템 — 아키텍처와 흐름도

작성 2026-09-29, `feature/system-integration` 브랜치 코드 기준.

관련 문서:

- 실행 방법: [`docs/SYSTEM_RUN_GUIDE.md`](SYSTEM_RUN_GUIDE.md)
- 설계 배경과 단계별 기록: [`docs/SYSTEM_INTEGRATION_PLAN.md`](SYSTEM_INTEGRATION_PLAN.md)
- 주행 알고리즘 세부: [`docs/병원 AMR 주행 알고리즘.md`](병원%20AMR%20주행%20알고리즘.md)
- 관제(구역 예약) 세부: [`docs/FLEET_CONTROL_ALGORITHM.md`](FLEET_CONTROL_ALGORITHM.md)

---

## 0. 한 줄 요약

> 모바일 매니퓰레이터(Nova Carter + Doosan M0609 + RG2) **1~3대**가 **채취실(East)** 책상에서 검체 트레이 3개를
> 랙에 싣고, 트레이의 QR로 **긴급도**를 읽은 뒤 **분석실(West)** 책상까지 운송·하역하고 돌아온다.
> 관제는 **긴급도 높은 로봇부터** 문·복도를 먼저 쓰게 하고, 모든 작업·상태는 PostgreSQL / Redis에 남아 관제 웹에 보인다.

![차선 지도](images/lanes_map.png)

| 이름 (시스템) | 책상 | 역할 | 주행 코드 이름 |
|---|---|---|---|
| `collection` 채취실 | East_DockDesk (x ≈ 20.8) | 트레이 적재 + 긴급도 판독 | `lab` |
| `analysis` 분석실 | West_DockDesk (x ≈ −45.4) | 트레이 하역 | `specimen` |

---

## 1. 배치도 (프로세스와 런타임)

```mermaid
flowchart TB
    subgraph CTRL["관제 PC"]
        subgraph DOCKER["Docker"]
            PG[("PostgreSQL 16<br/>robotdb3_sql :5432<br/>작업 · 트레이 · 이력")]
            RD[("Redis 8.8<br/>robotdb3_nosql :6379<br/>상태 · heartbeat · 예약 · 이벤트")]
        end
        FM["fleet_manager<br/>(Python 3.12, rclpy)"]
        DW["db_worker<br/>(Python 3.12, ROS 없음)"]
        WEB["monitoring_web/server.py<br/>(표준 라이브러리 HTTP + SSE)<br/>:8080"]
    end

    subgraph SIMPC["로봇 / 시뮬레이션 PC"]
        ISAAC["Isaac Sim 5.1<br/>run_fleet_sim.py<br/>(번들 Python 3.11 · rclpy 불가)<br/>병원 씬 · 로봇 N대 · 보행자 · 팔 컨트롤러"]
        subgraph PER["로봇마다 (/robotN)"]
            DET["tray_detector<br/>(~/yolo-venv, Python 3.12)<br/>YOLO + QR"]
            NAV["Nav2 + 병원 노드<br/>hospital_navigation.launch.py<br/>(+ RViz)"]
            AG["robot_agent<br/>(Python 3.12)"]
            MIS["hospital_mission<br/>(하위 프로세스)"]
            AG --> MIS
        end
    end

    BROWSER["브라우저"]

    ISAAC <-- "ROS 2 (DDS, ROS_DOMAIN_ID=136)<br/>OmniGraph 노드" --> DET
    ISAAC <-- "ROS 2" --> NAV
    ISAAC <-- "ROS 2" --> AG
    MIS <-- "ROS 2" --> NAV
    AG <-- "ROS 2" --> FM
    FM -- "Redis" --> RD
    AG -- "psycopg2 / redis" --> PG
    AG -- "redis" --> RD
    DW -- "stream 소비" --> RD
    DW -- "INSERT" --> PG
    WEB -- "docker exec psql / redis-cli" --> PG
    WEB -- "docker exec" --> RD
    BROWSER -- "HTTP · SSE" --> WEB
```

| 런타임 | 쓰는 프로세스 | 이유 |
|---|---|---|
| Isaac 번들 Python 3.11 | `run_fleet_sim.py` | Isaac Sim 5.1이 강제. 시스템 ROS 2 Jazzy의 rclpy(3.12)를 쓸 수 없어 **ROS 입출력은 전부 OmniGraph 노드** |
| `~/yolo-venv` (3.12) | `tray_detector` | ultralytics를 시스템 파이썬에 섞지 않는다. ROS는 `PYTHONPATH`로 시스템 site-packages를 붙인다 |
| 시스템 Python 3.12 | Nav2, `robot_agent`, `hospital_mission`, `fleet_manager`, `db_worker` | psycopg2 / redis는 apt(`python3-psycopg2`, `python3-redis`) |
| Docker | PostgreSQL, Redis | 볼륨에 데이터 유지 |

- 여러 PC로 나눌 때: DB·관제·웹은 관제 PC 한 대, 로봇 스택(검출기·Nav2·에이전트)은 **그 로봇의 Isaac과 같은 PC**
  (라이다·카메라가 PC 사이로 흐르지 않게). DB 주소는 `HOSPITAL_PG_DSN` / `HOSPITAL_REDIS_URL`.
- 모든 ROS 프로세스는 `ROS_DOMAIN_ID=136`, 로봇 토픽은 `/robot1..3` 네임스페이스, TF도 로봇마다 `/robotN/tf`.

---

## 2. 계층 아키텍처

```mermaid
flowchart TB
    L1["표현<br/>monitoring_web — 지도 · 로봇 · 예약 구역 · 작업 · 이벤트"]
    L2["관제<br/>fleet_manager — 사이클 지시 · 복도 배정 · 구역 예약 · 긴급도 우선순위"]
    L3["로봇 조율<br/>robot_agent — 적재 → 운송 → 하역 → 복귀, 실패 이어 가기, DB 기록"]
    subgraph L4["행동"]
        direction LR
        L4A["주행<br/>hospital_mission + Nav2<br/>차선 추종 · 사람 회피 · 책상 도킹"]
        L4B["팔<br/>ArmTaskController<br/>트레이 적재 · 긴급도 관측 · 하역"]
    end
    subgraph L5["인지"]
        direction LR
        L5A["라이다 → 스캔 · AMCL · 사람 추적/예측"]
        L5B["손목 카메라 → YOLO 트레이 3D · QR 긴급도"]
    end
    L6["시뮬레이션<br/>Isaac Sim — 병원 씬 · Nova Carter · M0609 + RG2 · 보행자 · 센서"]
    DATA[("데이터<br/>PostgreSQL + Redis")]

    L1 --> L2 --> L3 --> L4
    L4A --> L5A
    L4B --> L5B
    L5 --> L6
    L4 --> L6
    L2 -. 기록 .-> DATA
    L3 -. 기록 .-> DATA
    L1 -. 조회 .-> DATA
```

책임 분리 원칙:

- **관제는 "언제·어디로"만** 정한다(작업 지시, 복도, 어디까지 가도 되는지). 어떻게 달릴지는 모른다.
- **에이전트는 순서만** 조율한다. 팔과 주행은 각자의 프로세스가 결과(`arm/status`, 종료 코드)로 알려 준다.
- **주행 코드는 관제·DB를 모른다.** `hospital_mission`은 파라미터와 `zone_hold` 토픽만 받는 독립 실행 파일이라 단독 시험이 된다.

---

## 3. 컴포넌트와 토픽 연결 (로봇 한 대, `/robotN`)

```mermaid
flowchart LR
    subgraph ISAAC["Isaac Sim (OmniGraph)"]
        CHASSIS["Nova Carter<br/>차체 그래프"]
        WRIST["손목 카메라 그래프<br/>(적재 중에만 켬)"]
        ARMIO["팔 I/O 그래프"]
        ARM["ArmTaskController<br/>(60 Hz 물리 콜백)"]
        ARMIO <--> ARM
    end
    DET["tray_detector"]
    subgraph NAV2["Nav2 + 병원 노드"]
        NAVS["AMCL · costmap · controller<br/>smoother · guard · collision_monitor"]
    end
    MIS["hospital_mission"]
    AG["robot_agent"]
    FM["fleet_manager"]

    CHASSIS -- "front_3d_lidar/lidar_points<br/>chassis/odom · tf · clock" --> NAVS
    NAVS -- "cmd_vel" --> CHASSIS
    WRIST -- "wrist_camera/color/image_raw<br/>wrist_camera/depth/image_raw<br/>wrist_camera/color/camera_info" --> DET
    DET -- "tray_detection<br/>aruco_markers" --> ARMIO
    AG -- "arm/command" --> ARMIO
    ARMIO -- "arm/status" --> AG
    MIS -- "follow_path 액션 · compute_path_to_pose<br/>initialpose (도킹 재측위)" --> NAVS
    NAVS -- "hospital/tracked_obstacles<br/>local_costmap/costmap_raw · map" --> MIS
    AG -- "하위 프로세스 실행<br/>(stdout 읽기)" --> MIS
    AG -- "agent_status · fleet_pose" --> FM
    FM -- "task · route" --> AG
    FM -- "zone_hold" --> MIS
```

### 3.1 시스템 인터페이스 (전부 `std_msgs/String` JSON, 네임스페이스 `/robotN`)

| 토픽 | 방향 | QoS | 내용 |
|---|---|---|---|
| `arm/command` | agent → Isaac | reliable | `"load:<ms>"`, `"unload:<ms>"`, `"stop:<ms>"` — **문자열이 바뀔 때만** 새 명령 (OmniGraph 구독은 마지막 값을 계속 들고 있다) |
| `arm/status` | Isaac → agent | | `{robot, cmd, cmd_text, result: idle/running/done/failed/stopped, state, slot, loaded[3], aruco[3], urgency[3], unloaded[3], warnings[], detail}` — 바뀔 때 + 0.5 s마다 |
| `tray_detection` | detector → Isaac | `Float32MultiArray` | `[seq, n, x,y,z,conf, …]` 카메라 광학 프레임, 가까운 순 |
| `aruco_markers` | detector → Isaac | `Float32MultiArray` | `[seq, n, id,slot, …]` id 0/1/2 = 하/중/상, slot = 랙 ROI 3등분 |
| `task` | fleet → agent | latched | `{seq, cmd: "cycle"}` |
| `route` | fleet → agent | latched | `{leg, track, version}` — 복도 배정, version↑ = 경로 변경 |
| `zone_hold` | fleet → mission | latched | `{leg#version, stop: [x,y,yaw] \| null, dock, zones}` |
| `agent_status` | agent → fleet | 1 Hz + 변화 시 | `{robot, stage, cycle, leg, cargo{loaded, urgency}, task_seq, fleet, history, …}` |
| `fleet_pose` | agent → fleet | 5 Hz | `PoseStamped` (TF `map → base_link`) |

latched = RELIABLE + TRANSIENT_LOCAL. 주행 하위 프로세스가 새로 떠도 마지막 허가를 바로 받는다.

---

## 4. 데이터 흐름과 저장

```mermaid
flowchart LR
    AG["robot_agent<br/>(Recorder)"]
    FM["fleet_manager"]
    subgraph REDIS["Redis"]
        ST["robot:{id}:state<br/>(해시: status, x, y, theta, task_id)"]
        HB["robot:{id}:heartbeat<br/>(1 s, TTL 3 s)"]
        RT["robot:{id}:route<br/>(1 m 간격 경로점)"]
        ZN["fleet:zones<br/>(구역 → 로봇)"]
        SE["stream:robot_events<br/>(STAGE, TASK_CREATED,<br/>MISSION_RESUME, REROUTED, UNLOADED …)"]
    end
    subgraph PGDB["PostgreSQL"]
        RI["robot_info"]
        TR["tray"]
        TT["transport_task"]
        TL["task_status_log"]
        EL["robot_event_log"]
        SH["robot_state_history"]
    end
    DW["db_worker"]
    WEB["monitoring_web"]

    AG --> ST
    AG --> HB
    AG --> SE
    AG -- "시작 시 등록 / is_active" --> RI
    AG -- "적재 후 생성" --> TR
    AG -- "생성 · 상태 갱신" --> TT
    TT -- "상태 바뀔 때 같이" --> TL
    FM --> RT
    FM --> ZN
    SE -- "컨슈머 그룹 + ACK" --> DW --> EL
    ST -- "5 s마다, heartbeat 살아 있는 로봇만" --> DW --> SH
    REDIS -- "SSE 1 s" --> WEB
    PGDB -- "SSE 1 s" --> WEB
```

- **실시간은 Redis, 기록은 PostgreSQL.** 에이전트는 자주 바뀌는 것(단계·위치·heartbeat)을 Redis에만 쓰고,
  이벤트는 스트림에 올린다. `db_worker`가 스트림을 `robot_event_log`로, 상태를 `robot_state_history`로 옮긴다.
- 스트림은 **컨슈머 그룹 + ACK**라 워커가 죽었다 살아도 이벤트가 빠지지 않는다.
- **DB 기록 실패는 경고만** 한다 — 기록 때문에 로봇이 멈추지 않는다. 시작할 때 연결 확인은 실패하면 종료한다(`use_db:=false`로 끌 수 있다).

### 4.1 스키마 (`DB_container/hospital_amr_db_v5_schema.sql`)

```mermaid
erDiagram
    robot_info ||--o{ transport_task : "배정 (robot_id)"
    robot_info ||--o{ robot_event_log : "robot_id"
    robot_info ||--o{ robot_state_history : "robot_id"
    transport_task ||--o{ task_status_log : "task_id"
    transport_task }o--|{ tray : "tray_ids[] (트리거로 검사)"

    robot_info {
        serial robot_id PK
        varchar robot_name UK "robot1..3"
        varchar model "nova_carter"
        smallint container_slots "1~3"
        boolean is_active
        timestamptz registered_at
    }
    tray {
        varchar tray_id PK "TR{YYYYMMDD}-{task}-S{slot}"
        varchar test_type
        smallint priority "1~3, 3 = 긴급"
        smallint specimen_count
        timestamptz packed_at
        timestamptz created_at
    }
    transport_task {
        bigserial task_id PK
        varchar_array tray_ids "1~3개"
        smallint_array slot_nos "1~3, 중복 없음"
        int robot_id FK
        varchar origin "collection"
        varchar destination "analysis"
        timestamptz departed_at
        timestamptz arrived_at
        task_status status
        timestamptz cancelled_at
        varchar cancel_reason
    }
    task_status_log {
        bigserial log_id PK
        bigint task_id FK
        task_status from_status
        task_status to_status
        timestamptz changed_at
    }
    robot_event_log {
        bigserial event_id PK
        int robot_id FK
        varchar event_type
        jsonb detail
        timestamptz occurred_at
    }
    robot_state_history {
        int robot_id FK
        timestamptz recorded_at
        float x
        float y
        varchar status "IDLE..ERROR"
    }
```

---

## 5. 기동과 종료 (`scripts/run_system.sh`)

```mermaid
flowchart TD
    S([run_system.sh]) --> C1{"이전 실행 프로세스 남음?<br/>(Isaac · Nav2 · RViz · 주행 · 검출기)"}
    C1 -- 예 --> X1([중단 — 목록 출력])
    C1 -- 아니오 --> C2{"DB 드라이버 import 가능?"}
    C2 -- 아니오 --> X2([중단])
    C2 -- 예 --> D["docker start robotdb3_sql robotdb3_nosql<br/>Redis fleet:zones 삭제"]
    D --> I["① Isaac run_fleet_sim.py --robots N --pose …"]
    I --> T["② tray_detector × N"]
    T --> W["③ db_worker"]
    W --> WAIT{"sim.log에 '재생 시작'?<br/>(start_pose_robotN.json 기록됨)"}
    WAIT -- "아직" --> WAIT
    WAIT -- 예 --> N2["④ Nav2 × N (8 s 간격)<br/>AMCL 초기 위치 = start_pose_robotN.json"]
    N2 --> B["(BAG=1) rosbag 기록"]
    B --> F["⑤ fleet_manager robots, cycles"]
    F --> A["⑥ robot_agent × N (fleet:=true, start:=collection | return)"]
    A --> WB["⑦ (WEB=1) 관제 웹"]
    WB --> MON["3 s마다 감시: 로봇별 단계 변화 출력"]
    MON --> E{"모든 에이전트 종료<br/>or 모두 N사이클 + 예약 대기 60 s<br/>or Isaac 종료 or Ctrl-C"}
    E -- 아니오 --> MON
    E -- 예 --> STOP["stop_all (역순)"]
```

```mermaid
flowchart LR
    A["에이전트 SIGTERM<br/>(돌던 주행 정지,<br/>작업 CANCELLED)"] --> B["관제 · db_worker · 웹<br/>(관제는 fleet:zones 삭제)"] --> C["rosbag"] --> D["Nav2 프로세스 그룹"] --> E["검출기"] --> F["Isaac SIGINT<br/>30 s 뒤 SIGKILL"] --> G["DB 작업 요약 ·<br/>관제 복도 배정 출력"]
```

순서가 중요한 이유:

- **Isaac 재생 → Nav2**: Isaac이 재생을 시작하며 로봇별 시작 자세 파일을 쓰고, Nav2의 AMCL이 그걸 초기 위치로 읽는다.
- **Nav2 8 s 간격**: RViz 두 개를 같은 순간에 띄우면 하나가 세그폴트했다.
- **이전 실행 검사**: 다른 도메인에 남은 Nav2가 지도 전달을 막은 적이 있다.

---

## 6. 한 사이클 전체 흐름 (end-to-end)

```mermaid
sequenceDiagram
    autonumber
    participant FM as fleet_manager
    participant AG as robot_agent
    participant ISA as Isaac (팔)
    participant DET as tray_detector
    participant MIS as hospital_mission
    participant DB as Redis / PostgreSQL

    Note over AG: 채취실 도킹 자세, stage = IDLE
    FM->>AG: task {seq, cmd: cycle}
    AG->>DB: state PICKING, event STAGE
    AG->>ISA: arm/command "load:<ms>"
    ISA-->>AG: arm/status result=running
    loop 랙 3칸
        ISA->>DET: 손목 카메라 color · depth
        DET-->>ISA: tray_detection (가까운 순 3D)
        Note over ISA: 스캔 → 하강 → 중앙 정렬 → 재검출 → 파지 → 랙 적재
    end
    ISA->>DET: 랙 관측 영상
    DET-->>ISA: aruco_markers (칸별 id)
    ISA-->>AG: result=done, loaded[3], urgency[3]
    AG->>DB: tray × n + transport_task (WAITING → ASSIGNED) → IN_TRANSIT, departed_at
    AG->>FM: agent_status DELIVERING, leg, cargo
    FM-->>AG: route {leg, track, version}
    AG->>MIS: 실행 (route_id, route_track, zone_leg)
    loop 주행
        FM-->>MIS: zone_hold (정지점 / dock)
        MIS-->>AG: stdout "[MISSION] DOCKING"
    end
    AG->>DB: state PLACE_DOCKING
    MIS-->>AG: exit 0 (분석실 도킹 완료)
    AG->>DB: task ARRIVED, arrived_at
    AG->>ISA: arm/command "unload:<ms>"
    ISA-->>AG: result=done, unloaded[3]
    AG->>DB: task COMPLETED (제자리 아니면 FAILED), event UNLOADED
    AG->>FM: agent_status RETURNING
    FM-->>AG: route (복귀 복도)
    AG->>MIS: 실행 (specimen_to_lab)
    MIS-->>AG: exit 0 (채취실 도킹 완료)
    AG->>DB: state IDLE
    AG->>FM: agent_status IDLE, task_seq
```

---

## 7. `robot_agent` 단계 상태도

단계 이름은 DB `robot_state_history.status`와 같다.

```mermaid
stateDiagram-v2
    [*] --> RETURNING: start:=return<br/>(복귀 차선 위에서 시작)
    [*] --> IDLE: start:=collection<br/>(채취실 도킹 자세)
    RETURNING --> IDLE: 첫 복귀 완료

    IDLE --> PICKING: task 수신 (관제 모드)<br/>또는 run_cycles 남음
    PICKING --> DELIVERING: arm load done<br/>+ 트레이 경고 없음
    PICKING --> ERROR: load 실패 · 시간 초과<br/>· 트레이가 랙 제자리 아님
    DELIVERING --> PLACE_DOCKING: 주행 stdout "DOCKING"
    PLACE_DOCKING --> PLACING: 주행 exit 0
    DELIVERING --> ERROR: 주행 실패 (이어 가기 3회 후)
    PLACE_DOCKING --> ERROR: 도킹 실패
    PLACING --> RETURNING2: unload 끝 (실패여도 복귀)
    state "RETURNING" as RETURNING2
    RETURNING2 --> PICK_DOCKING: 주행 stdout "DOCKING"
    PICK_DOCKING --> IDLE: 주행 exit 0
    RETURNING2 --> ERROR: 주행 실패
    ERROR --> [*]
```

주행 호출 안쪽 (`drive` / `_drive`):

```mermaid
flowchart TD
    A["wait_route<br/>(관제가 복도 줄 때까지, 30 s)"] --> B["hospital_mission 실행<br/>(자기 프로세스 그룹)"]
    B --> C{"종료/사건"}
    C -- "exit 0" --> OK([성공])
    C -- "route version↑" --> RR["주행 SIGINT → REROUTED<br/>새 track으로 resume=true"] --> B
    C -- "exit 1 (도중 실패)" --> R{"이어 가기 < 3회?"}
    R -- 예 --> W["5 s 대기, event MISSION_RESUME<br/>resume=true (현재 위치에서)"] --> B
    R -- 아니오 --> F([실패])
    C -- "exit 2 (잘못된 route)" --> F
    C -- "1500 s 초과" --> T["프로세스 그룹 SIGINT → 10 s → SIGKILL"] --> F
```

- 경로 변경(REROUTED)은 실패 횟수로 세지 않는다.
- 에이전트가 끝나면(Ctrl-C, 시간 초과) **주행 하위 프로세스도 같이 멈춘다** — 에이전트만 죽고 로봇이 계속 달린 적이 있다.

---

## 8. 팔 작업 상태기계 (`isaacpjt/system/arm/controller.py`)

```mermaid
stateDiagram-v2
    [*] --> idle
    idle --> scan: load 명령<br/>(책상 비었으면 보관소 트레이 재공급,<br/>손목 카메라 켬)
    idle --> unload: unload 명령<br/>(분석실 책상의 지난 트레이 치움)

    state "적재 — 랙 칸 1 → 3" as LOAD {
        scan --> descend: joint 회전 끝
        descend --> settle_c: 하강 + 카메라 숙임
        settle_c: settle (→ center)<br/>새 검출 seq 대기
        settle_c --> center: 트레이 찾음
        center --> settle_p: 검출 x만큼 수평 이동
        settle_p: settle (→ pick)<br/>재검출 → 파지점
        settle_p --> pick: 파지점 확정
        pick --> scan: 칸 남음<br/>(랙 제자리 확인 → loaded[k])
    }
    settle_c --> recover: 재시도 초과 / 새 검출 없음
    settle_p --> recover: 재시도 초과
    recover --> scan: 실패 < 2회
    recover --> observe_go: 실패 2회 — 남은 칸 포기

    pick --> observe_go: 랙 3칸 참
    state "긴급도 관측" as OBS {
        observe_go --> observe_settle: 랙 위 관측 자세
        observe_settle: 새 QR 메시지마다 한 표<br/>칸별 최빈 id
        observe_settle --> observe_home
    }
    observe_home --> idle: result = done (하나라도 실림)<br/>/ failed

    state "하역 — 랙 칸 3 → 1" as UNL {
        unload --> unload: 다음 칸 (책상 제자리 확인 → unloaded[k])
    }
    unload --> idle: result = done (모두 제자리)<br/>/ failed

    idle --> idle: stop (언제든, 팔은 마지막 목표에 멈춤)
```

- 스텝 실행(`PickPlaceSequence`)은 코사인 S-커브 보간 + Lula IK. 스텝 도달에 실패하면 **손목을 특이점에서 풀고 재시도**,
  그래도 안 되면 그 스텝을 건너뛴다(그리퍼 상태 유지).
- `tick()`은 **60 Hz 물리 콜백**에서 돈다. 손목 카메라는 렌더 30 Hz의 4프레임에 한 번, 적재 중에만 켠다(3대 동시 발행 부하).
- 적재/하역 뒤 **트레이 실제 위치로 제자리 여부를 확인**한다. 시퀀스가 끝났다고 트레이가 제자리에 있는 건 아니다(랙 테두리 걸림).
  이 결과가 `loaded[]` / `unloaded[]` / `warnings[]`로 에이전트에 간다.

---

## 9. 검출기 프레임 처리 (`admin_ws/src/tray_detector/detect_node.py`)

```mermaid
flowchart TD
    C0["wrist_camera/color 수신"] --> C1{"depth · camera_info 있음?"}
    C1 -- 아니오 --> C0
    C1 -- 예 --> Y["YOLO 추론 (conf ≥ 0.85)"]
    Y --> D["박스마다 깊이 하위 15퍼센타일<br/>0.15~1.0 m 게이트 → 역투영 (광학 프레임)"]
    D --> P["카메라에서 가까운 순, 최대 3개<br/>tray_detection [seq, n, x,y,z,conf …]"]
    C0 --> A["RACK_ROI 크롭 ×3 확대 → QR 코드 판독"]
    A --> M["마커 중심 x → ROI 가로 3등분 칸<br/>aruco_markers [seq, n, id,slot …]"]
    P --> S{"검출/마커 있음?"}
    M --> S
    S -- 예 --> L["로그(바뀔 때) · 이미지 저장<br/>(마커 프레임 전부 + 10장마다 1장)"]
```

- 메시지에 타임스탬프가 없어서 `seq`(발행 카운터)로 **"관측 자세로 옮긴 뒤 새로 찍은 값인지"** 를 구분한다.
- 가까운 순으로 보내는 이유: 왼쪽부터 집던 때 앞의 트레이를 건드렸다.
- 긴급도 변환 (D2): QR id 2(상) → 3, 1(중) → 2, 0(하) → 1, 미검출 → 1.

---

## 10. 작업(`transport_task`) 상태와 기록 시점

```mermaid
stateDiagram-v2
    [*] --> WAITING: 적재 + 긴급도 판독 끝<br/>loaded_task_create (tray n개 + 작업 1건)
    WAITING --> ASSIGNED: 같은 트랜잭션에서 그 로봇에 배정
    ASSIGNED --> IN_TRANSIT: 운송 출발 (departed_at)
    IN_TRANSIT --> ARRIVED: 분석실 도킹 완료 (arrived_at)
    ARRIVED --> COMPLETED: 하역 — 모두 책상 제자리
    ARRIVED --> FAILED: 하역 — 제자리 아님
    IN_TRANSIT --> FAILED: 주행 실패
    ASSIGNED --> CANCELLED: 에이전트 종료 (Ctrl-C)
    IN_TRANSIT --> CANCELLED: 에이전트 종료 / 재시작 시 열린 작업 정리
    ARRIVED --> CANCELLED: 에이전트 종료
    COMPLETED --> [*]
    FAILED --> [*]
    CANCELLED --> [*]
```

- 상태가 바뀔 때마다 `task_status_log`에 한 줄(`from → to`).
- 트레이 ID는 QR이 긴급도 3종뿐이라 개체 식별이 안 되므로 `TR{YYYYMMDD}-{task_id}-S{slot}`로 만든다 (D3).
- 작업은 **적재가 끝난 뒤 그 로봇의 작업으로 만든다**(D4). 생성과 배정(`WAITING → ASSIGNED`)이 한 트랜잭션이라 대기 상태로 남지 않는다. 스키마의 `PICKING_UP`은 쓰지 않는다.

---

## 11. 관제 tick과 주행 요약

### 11.1 관제 (`fleet_manager`, 5 Hz) — 세부는 [관제 문서](FLEET_CONTROL_ALGORITHM.md)

```mermaid
flowchart LR
    A["fleet_pose · agent_status 수신"] --> B["처음 본 로봇이면 모든 경로에 투영<br/>새 구간이면 복도 배정 (+ 긴급도 양보)"]
    B --> C["경로 위 위치 s 갱신"]
    C --> D["Reservations.tick<br/>점유 갱신 → 우선순위 순 앞 6 m 예약"]
    D --> E["zone_hold 발행<br/>(정지점 = 받은 구역 끝 − 앞 차체)"]
    E --> F["IDLE이면 다음 task"]
    F --> G["Redis fleet:zones"]
```

- 긴급도 점수 = 실린 트레이의 `(3 개수, 2 개수, 1 개수)` 사전식 비교. 빈 로봇 = `(0,0,0)`.
- 로봇끼리 서로 센서로 못 봐도(다른 PC의 Isaac) **구역 예약만으로** 충돌하지 않는다(철도 폐색 방식).

### 11.2 주행 (`hospital_mission`) — 세부는 [주행 알고리즘 문서](병원%20AMR%20주행%20알고리즘.md)

```mermaid
flowchart LR
    A["출발 책상 옆<br/>라이다 재측위"] --> B["출발 (DWB)<br/>방 · 문"] --> C["복도 직선 (MPPI)<br/>사람 회피 · 측방 S자"] --> D["도착 (DWB)<br/>정류장 정지"] --> E["예약 dock 허가 대기"] --> F["책상 옆 직진 도킹<br/>간격 0.15 m (재시도 2회)"]
```

명령 체인: `controller → velocity_smoother → hospital_velocity_guard(사람 예측) → collision_monitor → cmd_vel`.

---

## 12. 실패와 복구 경로

| 어디서 | 무엇이 | 어떻게 복구 | 끝내 안 되면 |
|---|---|---|---|
| 팔 — 검출 | 트레이를 못 찾음 / 새 검출이 안 옴 | 칸당 3번 재시도 → `recover`(들고 있으면 제자리에 두고 홈) | 2번 실패하면 남은 칸 포기, 실린 것만 들고 간다 |
| 팔 — 스텝 | IK 미수렴 · 도달 실패 | 손목 풀고 같은 스텝 1번 재시도 → 그래도 안 되면 건너뜀 | — |
| 팔 — 결과 | 트레이가 랙 제자리 아님 | — | 에이전트가 **주행하지 않고** ERROR (걸린 채 달리면 떨어진다) |
| 팔 — 명령 | Isaac이 명령을 못 받음 (DDS 발견 지연) | 구독자 붙을 때까지 60 s, 받을 때까지 1 s마다 재발행 | 에이전트 ERROR |
| 주행 — 사람 | 15 s 전진 못 함 | 에이전트가 5 s 뒤 **현재 위치에서 이어 가기** (3회) | ERROR |
| 주행 — 정적 장애물 | 차선 막힘 | 측방 S자 → NavFn 우회 → 1~1.5 m 후진 후 재평가 | 이어 가기 |
| 주행 — 도킹 | 간격 어긋남 · 모서리 간격 · 시간 초과 | 라이다 재측위 → 4 m 곧게 후진 → 재접근 (2회) | ERROR |
| 주행 — 관측 | 라이다 · odom · TF 1.2 s 끊김 | 정지(가드 · Collision Monitor), 2 s 넘으면 스테이지 실패 | 이어 가기 |
| 관제 — 경로 | 긴급도 높은 로봇에 복도 양보 | route version↑ → 현재 위치에서 새 복도로 | — |
| 관제 — 위치 끊김 | `fleet_pose` 3 s 없음 | 경고만, 예약은 유지(위치를 모르므로) | 사람이 정리 |
| DB | 기록 실패 | 경고만, 로봇은 계속 | — |
| DB 워커 | 워커 재시작 | 컨슈머 그룹 ACK로 빠짐없이 이어 받음 | — |
| 재시작 | 지난 실행이 작업 도중 죽음 | 에이전트 시작 시 그 로봇의 열린 작업을 CANCELLED | — |

---

## 13. 코드 위치

| 구성 요소 | 경로 |
|---|---|
| 전체 실행 스크립트 | `scripts/run_system.sh` |
| Isaac 진입점 / 씬 | `isaacpjt/system/run_fleet_sim.py`, `scene.py` |
| 팔 | `isaacpjt/system/arm/` — `controller.py`(상태기계), `motion.py`(스텝·IK), `rosio.py`(OmniGraph I/O), `trays.py`(트레이 보관·재공급), `config.py`, `geometry.py` |
| 트레이 검출기 | `admin_ws/src/tray_detector/tray_detector/detect_node.py` (가중치 `runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt`) |
| 로봇 에이전트 / DB 기록 | `src/hospital_system/hospital_system/robot_agent.py`, `records.py`, `stations.py` |
| 관제 | `src/hospital_system/hospital_system/fleet_manager.py`, `lane_graph.py`, `priority.py` |
| DB 모듈 / 워커 | `src/hospital_system/hospital_system/db.py`, `db_worker.py` |
| DB 스키마 / 컨테이너 | `DB_container/hospital_amr_db_v5_schema.sql`, `DB_container/*.txt` |
| 주행 | `src/nova_carter/nav_to_goal/nav_to_goal/hospital_*.py` |
| Nav2 설정 · 런치 · 맵 | `src/nova_carter/carter_navigation/{params,launch,maps}/` |
| 예측 코스트맵 레이어 | `src/nova_carter/hospital_dynamic_layer/` |
| 관제 웹 | `monitoring_web/server.py`, `monitoring_web/static/` |
