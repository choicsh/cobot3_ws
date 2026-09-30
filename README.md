# rokey_cobot3 — 병원 검체 운송 AMR 시스템

Isaac Sim 5.1 병원 씬에서 로봇 팔(Doosan M0609)을 얹은 Nova Carter 로봇 최대 3대가
**채취실(East)** 책상에서 검체 트레이를 싣고 **분석실(West)** 책상에 내려 주는 통합 시스템.

- **픽 앤 플레이스**: YOLO 로 트레이 위치 확인 → 파지 → 로봇 랙 적재, 랙 QR 로 긴급도(3 = 긴급) 판독
- **자율주행**: Nav2 FollowPath 고정 차선 + MPPI, 책상 옆 라이다 도킹, 보행자 회피
- **관제**: 구역 예약으로 교차로·문 통과 조정, 긴급도 기반 우선순위, PostgreSQL + Redis 기록
- **관제 웹**: 로봇 위치·예약 구역·작업 현황 실시간 대시보드, rosbag 재생

## 구성

| 프로세스 | 위치 | 하는 일 |
|---|---|---|
| Isaac Sim `run_fleet_sim.py` | `isaacpjt/system/` | 병원 씬, 로봇 N대, 보행자, 팔 제어 |
| `tray_detector` | `admin_ws/src/tray_detector/` | 손목 카메라 트레이·QR 검출 (YOLO) |
| Nav2 `hospital_navigation.launch.py` | `src/nova_carter/carter_navigation/` | 로봇별 `/robotN` 주행 스택 |
| `fleet_manager` | `src/hospital_system/` | 사이클 지시, 복도 배정, 구역 예약 |
| `robot_agent` | `src/hospital_system/` | 적재 → 운송 → 하역 → 복귀 |
| `db_worker` | `src/hospital_system/` | Redis 이벤트 → PostgreSQL |
| 관제 웹 `server.py` | `monitoring_web/` | 브라우저 대시보드 (http://127.0.0.1:8080) |
| PostgreSQL / Redis | Docker (`DB_container/`) | 작업·트레이·이력 / 실시간 상태·예약 |

## 환경

- Ubuntu + ROS 2 Jazzy
- Isaac Sim 5.1 (`~/isaacsim`)
- Docker (PostgreSQL 16, Redis 8.8)
- 검출기용 파이썬 venv (`~/yolo-venv`, ultralytics)

## 빠른 시작

두 PC 모두 같은 서브넷, `ROS_DOMAIN_ID=136`. 에이전트가 시작할 때 DB 를 확인하므로 **관제 PC 를 먼저** 띄운다. 매 터미널에서:

```bash
cd ~/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash          # 처음이면 먼저 colcon build --packages-select carter_navigation nav_to_goal hospital_dynamic_layer hospital_system
export ROS_DOMAIN_ID=136
```

### 관제 PC — DB, 관제, 관제 웹

```bash
docker start robotdb3_sql robotdb3_nosql      # 처음이면 docs/SYSTEM_RUN_GUIDE.md §1.2 로 컨테이너 생성
docker exec robotdb3_nosql redis-cli --user rokey --pass rokey --no-auth-warning del fleet:zones

ros2 run hospital_system db_worker &
ros2 run hospital_system fleet_manager --ros-args -p "robots:=['robot1','robot2','robot3']" -p cycles:=2 &
python3 monitoring_web/server.py --host 0.0.0.0   # http://<관제PC>:8080
```

### GPU PC — Isaac Sim, 검출기, Nav2, 로봇 에이전트

```bash
export HOSPITAL_PG_DSN=postgresql://rokey:rokey@<관제PC>:5432/robotdb3_sql
export HOSPITAL_REDIS_URL=redis://rokey:rokey@<관제PC>:6379/0

# 1) Isaac Sim — "[FLEET] 재생 시작" 이 찍힐 때까지 기다린다
~/isaacsim/python.sh isaacpjt/system/run_fleet_sim.py --robots 3 \
  --pose 1:collection --pose 2:0.0,14.9,0 --pose 3:-15.0,14.9,0

# 2) 트레이 검출기 — 로봇마다
for i in 1 2 3; do
  PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH ~/yolo-venv/bin/python \
    admin_ws/src/tray_detector/tray_detector/detect_node.py runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt \
    --ros-args -r __ns:=/robot$i &
done

# 3) Nav2 — 로봇마다 8 s 간격 (RViz 는 robot1 만)
for i in 1 2 3; do
  ros2 launch carter_navigation hospital_navigation.launch.py namespace:=robot$i use_rviz:=$([ $i = 1 ] && echo True || echo False) \
    start_pose_path:=$HOME/cobot3_ws/isaacpjt/assets/start_pose_robot$i.json &
  sleep 8
done

# 4) 로봇 에이전트 (2사이클) — robot1 은 채취실에서 적재부터, 2·3 은 복귀 차선에서 시작
ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot1 -p fleet:=true -p run_cycles:=2 -p start:=collection &
ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot2 -p fleet:=true -p run_cycles:=2 -p start:=return &
ros2 run hospital_system robot_agent --ros-args -r __ns:=/robot3 -p fleet:=true -p run_cycles:=2 -p start:=return &
```

한 PC 에서 모두 돌릴 때는 `ROBOTS=3 CYCLES=0 WEB=1 scripts/run_system.sh` 한 줄로 위 순서를 대신한다.
옵션, 로봇 수별 배치, 종료, 문제 해결은 [docs/SYSTEM_RUN_GUIDE.md](docs/SYSTEM_RUN_GUIDE.md).

## 디렉터리

```
isaacpjt/        Isaac Sim 스크립트·에셋 (system/ = 통합 실행, sdg/ = 트레이 합성 데이터·YOLO 학습)
src/             ROS 2 패키지 (hospital_system, nova_carter/*, m0609)
admin_ws/        트레이 검출 노드
monitoring_web/  관제 대시보드
DB_container/    DB 스키마(v5)·컨테이너 구축 안내
scripts/         통합 실행(run_system.sh), 관제 재생(replay_dashboard.sh)
runs/            학습된 YOLO 가중치 (tray-2/weights/best.pt)
docs/            설계·실행·작업 기록
```

## 문서

- [SYSTEM_RUN_GUIDE.md](docs/SYSTEM_RUN_GUIDE.md) — 실행 명령어 가이드
- [SYSTEM_INTEGRATION_PLAN.md](docs/SYSTEM_INTEGRATION_PLAN.md) — 통합 설계와 시험 기록
- [FLEET_CONTROL_ALGORITHM.md](docs/FLEET_CONTROL_ALGORITHM.md) — 관제(구역 예약·우선순위) 알고리즘
- [ALGORITHM.md](ALGORITHM.md) — 주행 알고리즘
- [monitoring_web/README.md](monitoring_web/README.md) — 관제 웹
