# 병원 검체 운송 시스템 — 실행 명령어 가이드

로봇 1~3대가 채취실(East)에서 트레이를 싣고 분석실(West)에 내려 주는 전체 시스템을 띄우는 순서다.
설계와 시험 기록은 [SYSTEM_INTEGRATION_PLAN.md](SYSTEM_INTEGRATION_PLAN.md).

## 0. 구성

| # | 프로세스 | 파이썬 | 로봇마다 | 하는 일 |
|---|---|---|---|---|
| 1 | PostgreSQL `robotdb3_sql`, Redis `robotdb3_nosql` | Docker | | 작업·트레이·이력 / 실시간 상태·예약 |
| 2 | Isaac Sim `run_fleet_sim.py` | Isaac 번들 3.11 | 한 프로세스에 N대 | 병원 씬, 로봇, 사람, 팔(적재·하역) |
| 3 | `tray_detector` | `~/yolo-venv` | ○ | 손목 카메라 트레이·ArUco 검출 |
| 4 | Nav2 `hospital_navigation.launch.py` (+ RViz) | 시스템 3.12 | ○ | `/robotN` 주행 스택 |
| 5 | `db_worker` | 시스템 3.12 | | Redis 이벤트 → PostgreSQL |
| 6 | `fleet_manager` | 시스템 3.12 | | 사이클 지시, 복도 배정, 구역 예약 |
| 7 | `robot_agent` | 시스템 3.12 | ○ | 적재 → 운송 → 하역 → 복귀 (주행은 `hospital_mission` 하위 프로세스) |
| 8 | 관제 웹 `monitoring_web/server.py` | 시스템 3.12 | | 브라우저 대시보드 |

모든 ROS 프로세스는 **같은 `ROS_DOMAIN_ID=136`** 이어야 한다. 순서가 중요하다 — Isaac 이 재생을 시작하고 로봇별 시작 자세
파일을 쓴 **뒤에** Nav2 를 띄운다(AMCL 초기 위치).

## 1. 처음 한 번

```bash
# ROS 2 Jazzy, Isaac Sim 5.1 (~/isaacsim), Docker 가 설치돼 있다고 가정
cd ~/cobot3_ws
sudo apt install python3-psycopg2 python3-redis          # 관제·에이전트의 DB 드라이버 (시스템 파이썬)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select carter_navigation nav_to_goal hospital_dynamic_layer hospital_system
```

- DB 컨테이너 만들기와 스키마 적용: `DB_container/2_DB 컨테이너 구축.txt`
  (`docker exec -i robotdb3_sql psql -U rokey -d robotdb3_sql < DB_container/hospital_amr_db_v5_schema.sql`).
- 검출기 파이썬: `python3 -m venv ~/yolo-venv && ~/yolo-venv/bin/pip install ultralytics` (`admin_ws/README.md`).
  가중치는 저장소의 `runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt`.

## 한 번에 실행 — `scripts/run_system.sh`

§3 의 순서를 스크립트 하나로 돌린다. 로그는 `~/cobot3_logs/<시각>/` (Isaac, 검출기·Nav2·에이전트 로봇별, 관제, DB 워커).

```bash
scripts/run_system.sh                              # 로봇 2대, 2사이클
ROBOTS=3 CYCLES=1 RVIZ_MAX=1 scripts/run_system.sh # 3대, RViz 는 robot1 만
ROBOTS=3 CYCLES=0 WEB=1 scripts/run_system.sh      # 계속 운용 + 관제 웹 — Ctrl-C 로 끝
```

- 시작 전에 이전 실행이 남아 있는지 확인하고, 남아 있으면 시작하지 않는다(§6).
- 로봇별 단계 변화를 한 줄씩 찍는다. 사이클을 다 돌면(마지막 로봇은 채취실 앞 줄에서 대기) 또는 Ctrl-C 면 역순으로 모두 멈추고
  이번 실행의 DB 작업과 관제의 복도 배정을 요약한다.
- 옵션(환경 변수): `ROBOTS` 1..3, `CYCLES`(0 = 계속), `RVIZ`/`RVIZ_MAX`, `WEB`, `BAG`(rosbag), `POSE1..3`/`START1..3`(§4),
  `SIM_EXTRA`(예: `--no-people`), `LOG_DIR`, `DB_PYTHON`(psycopg2/redis 가 든 파이썬, 기본 `python3`), `ISAAC`, `YOLO_PY`,
  `ROS_DOMAIN_ID`(기본 136). 자세한 설명은 스크립트 머리말.

아래는 같은 일을 터미널마다 손으로 하는 방법이다.

## 2. 매 터미널 공통

```bash
cd ~/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=136
```

아래 명령은 모두 이 설정이 된 터미널에서 실행한다 (Isaac 터미널은 `export ROS_DOMAIN_ID=136` 만 있으면 된다).

## 3. 실행 순서 (로봇 3대 예)

### 3.1 DB 켜기 — 재부팅하면 컨테이너가 꺼져 있다

```bash
docker start robotdb3_sql robotdb3_nosql
docker exec robotdb3_nosql redis-cli --user rokey --pass rokey --no-auth-warning del fleet:zones   # 지난 예약 표시 지우기
```

### 3.2 Isaac Sim

```bash
export ROS_DOMAIN_ID=136
~/isaacsim/python.sh isaacpjt/system/run_fleet_sim.py --robots 3 \
  --pose 1:collection --pose 2:0.0,14.9,0 --pose 3:-15.0,14.9,0
```

- `[FLEET] 재생 시작` 이 찍힐 때까지 기다린다 (1분 안팎). 이때 `isaacpjt/assets/start_pose_robotN.json` 이 쓰인다.
- 로봇 배치는 §4. 트레이는 채취실 책상에 3개 + 예비 `(로봇 수 − 1)` 세트가 바닥 아래 보관소에 생긴다(`--reserve-sets N`).
- 그 밖의 옵션: `--no-people`(보행자 끔), `--headless`, `--front-cam`, `--preload-rack I`(로봇 I 랙에 트레이를 미리 싣기).

### 3.3 트레이 검출기 — 로봇마다

```bash
for i in 1 2 3; do
  PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH ~/yolo-venv/bin/python \
    admin_ws/src/tray_detector/tray_detector/detect_node.py runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt \
    --ros-args -r __ns:=/robot$i &
done
```

### 3.4 Nav2 — 로봇마다, 8 s 간격

```bash
for i in 1 2 3; do
  ros2 launch carter_navigation hospital_navigation.launch.py namespace:=robot$i use_rviz:=True \
    start_pose_path:=$HOME/cobot3_ws/isaacpjt/assets/start_pose_robot$i.json &
  sleep 8        # RViz 두 개를 같은 순간에 띄우면 하나가 세그폴트한다
done
```

- RViz 는 로봇마다 하나씩 뜬다. 3대에서 무거우면 두 번째부터 `use_rviz:=False` (RViz 가 라이다 처리량을 먹는다 — 계획서 §7 P6b).
- 3D 라이다 포인트클라우드·카메라 표시는 기본 꺼짐(체크박스로 켤 수 있다).

### 3.5 DB 워커

```bash
ros2 run hospital_system db_worker
```

### 3.6 관제

```bash
ros2 run hospital_system fleet_manager --ros-args -p "robots:=['robot1','robot2','robot3']" -p cycles:=2
```

- `cycles` = 로봇마다 지시할 사이클 수, `0` 이면 계속.
- 로그의 `via upper / upper_reserve / lower / lower_reserve` 가 로봇마다 배정한 가운데 복도, `yields` 가 긴급도 양보다.

### 3.7 로봇 에이전트 — 로봇마다

```bash
ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot1 -p fleet:=true -p run_cycles:=2 -p start:=collection &
ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot2 -p fleet:=true -p run_cycles:=2 -p start:=return &
ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot3 -p fleet:=true -p run_cycles:=2 -p start:=return &
```

- `start:=collection` — 채취실 도킹 자세에서 시작(적재부터), `start:=return` — 복귀 차선 위에서 시작(먼저 채취실로).
- 에이전트는 시작할 때 DB 연결을 확인하고, 없으면 종료한다(`-p use_db:=false` 로 끌 수 있다).
  지난 실행이 끝내지 못한 작업은 CANCELLED 로 정리한다.

### 3.8 관제 웹

```bash
python3 monitoring_web/server.py            # --host 0.0.0.0 이면 다른 PC 에서도
```

브라우저에서 <http://127.0.0.1:8080> — 로봇 위치·단계, 예약 구역, 복도, 작업·트레이·이벤트.

## 4. 로봇 배치

| 로봇 수 | Isaac `--pose` | 에이전트 `start` |
|---|---|---|
| 1 | `--pose 1:collection` | robot1 `collection` |
| 2 | `--pose 1:collection --pose 2:0.0,14.9,0` | robot1 `collection`, robot2 `return` |
| 3 | 위 + `--pose 3:-15.0,14.9,0` | robot3 `return` |

- robot1 은 반드시 채취실 도킹 자세(`collection`)에 둔다 — 책상 트레이가 robot1 의 팔 기준으로 놓인다.
- `X,Y,YAW` 는 map 좌표·도(°). 복귀 차선(위 복도 y=14.9) 위에 동쪽(0°)을 보게 두면 `start:=return` 으로 이어 간다.
- `--pose I:analysis` 는 분석실 도킹 자세.

## 5. 보면서 확인하기

```bash
ros2 topic echo /robot1/agent_status        # 단계, 구간(leg), 실은 트레이, 사이클 기록
ros2 topic echo /robot1/route               # 관제가 준 복도 {"leg","track","version"}
ros2 topic echo /robot1/zone_hold           # 예약 정지점 (stop=null 이면 도킹까지 허가)
ros2 topic echo /robot1/hospital/mission_state
docker exec robotdb3_nosql redis-cli --user rokey --pass rokey --no-auth-warning hgetall robot:1:state
docker exec robotdb3_sql psql -U rokey -d robotdb3_sql -c \
  "select task_id, robot_id, status, departed_at, arrived_at from transport_task order by task_id desc limit 10"
```

## 6. 종료

에이전트 → 관제·DB 워커 → Nav2 → 검출기 → Isaac 순서로 Ctrl-C.

- 에이전트를 멈추면 돌던 주행(`hospital_mission`)도 같이 멈추고, 진행 중이던 작업은 DB 에 CANCELLED 로 남는다.
- 관제를 멈추면 Redis 예약 표시(`fleet:zones`)를 지운다.
- 다음 실행 전에 남은 프로세스가 없는지 확인한다 — 다른 도메인에 남은 Nav2 가 지도 전달을 막은 적이 있다:
  ```bash
  pgrep -af 'component_container_[i]solated|[r]viz2 |hospital_[m]ission|detect_[n]ode|kit/python/bin/python[3]'
  ```
  (`pgrep -f` 패턴에 `[ ]` 를 넣지 않으면 그 명령을 친 셸 자신이 잡힌다.)

## 7. 로봇 1대, 관제 없이

관제·DB 없이 적재 → 운송 → 하역 → 복귀 1사이클 (기본 경로: 운송 아래 복도, 복귀 위 복도).

```bash
~/isaacsim/python.sh isaacpjt/system/run_fleet_sim.py --robots 1 --pose 1:collection
# 검출기 /robot1 (§3.3), Nav2 namespace:=robot1 (§3.4)
ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot1 -p run_cycles:=1 -p use_db:=false
```

주행만 따로: `ros2 run nav_to_goal hospital_mission --ros-args -r __ns:=/robot1 -r /tf:=tf -r /tf_static:=tf_static
-p route_id:=lab_to_specimen` (`specimen_to_lab` = 복귀, `-p route_track:=upper_reserve` 로 복도 지정, `-p resume:=true` 로 현재 위치에서 이어 가기).

## 8. 여러 PC 에 나눌 때

- 모든 PC `ROS_DOMAIN_ID=136`, 같은 서브넷.
- DB·관제·웹은 관제 PC 한 대에. 로봇 PC(에이전트, db 를 쓰는 프로세스)에서:
  ```bash
  export HOSPITAL_PG_DSN=postgresql://rokey:rokey@<관제PC>:5432/robotdb3_sql
  export HOSPITAL_REDIS_URL=redis://rokey:rokey@<관제PC>:6379/0
  ```
- 로봇 스택(검출기, Nav2, 에이전트)은 그 로봇의 Isaac 과 같은 PC 에 두는 편이 좋다 — 라이다·카메라가 PC 사이로 흐르지 않는다.

## 9. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| 에이전트가 `postgres off` / `redis off` 로 바로 끝남 | DB 컨테이너가 꺼져 있다 — §3.1 |
| `No module named 'psycopg2'` | `sudo apt install python3-psycopg2 python3-redis` |
| Nav2 가 `map -> base_link TF 대기 중` 에서 멈춤 | Isaac 재생 전에 띄웠거나 도메인이 다르다. `start_pose_path` 확인 |
| RViz 하나가 시작하자마자 죽음 | 두 RViz 를 같은 순간에 띄움 — §3.4 처럼 간격을 둔다 |
| 출발에서 `initial_observations_unavailable` | 지도(`/robotN/map`)를 못 받음 — 다른 도메인·이전 실행의 Nav2 가 남아 있는지 §6 확인 |
| 적재 명령이 한참 안 먹힘 | Nav2 가 뜨는 동안 DDS 발견이 늦다 — 에이전트가 60 s 까지 다시 보낸다 |
| 로봇이 정류장 앞에서 계속 서 있음 | 앞 로봇이 책상에 있다(예약 정지, 정상). 사이클을 다 마친 로봇은 채취실에서 다음 적재를 기다린다 |
| `yield_budget_exceeded` 로 주행 실패 | 사람 앞 양보 15 s 초과 — 에이전트가 현재 위치에서 3번까지 이어 간다 |
| `DOCK_RETRY` | 도킹 실패(정류장에서 책상 간격이 6 cm 넘게 어긋남, 모서리 간격, 시간 초과) — 라이다로 AMCL 을 맞추고 도킹 직선을 따라 정류장 1.5 m 뒤로 곧게 물러나 다시 접근한다(2번까지). 도킹 도중 멈춘 자리에서 이어 가도 같은 방식 |
| 라이다가 느리다 | RViz 부하. 3대면 RViz 를 줄인다(`use_rviz:=False`) |
