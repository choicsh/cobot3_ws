# FollowPath GPU PC 사전 검증 — 2026-09-23

> 아래는 지도 교체 전 사전 점검 기록이다. 이후 새 지도·기존 방 안 책상 도킹과
> guard 제거를 적용한 최신 실행 방법/결과는
> [HOSPITAL_NEWMAP_DOCKING_2026-09-23.md](HOSPITAL_NEWMAP_DOCKING_2026-09-23.md)를 따른다.

## 범위

`HOSPITAL_AMR_AVOIDANCE_HANDOFF_2026-09-23.md`를 최신 기준으로 읽고,
9월 22일 인수인계와 `hospital_people_stage1_review.md`의 과거 결과도 확인했다.
현재 `feature/followpath` 소스를 ROS 2 Jazzy / Isaac Sim 5.1 / RTX 5080 Laptop
환경에서 빌드하고, Isaac headless + 병원 Nav2를 실제 실행했다.
FollowPath 주행 goal은 보내지 않았다. 이 결과는 완주·회피 성능 합격이 아니다.

## 확인 결과

- `hospital_dynamic_layer`, `nav_to_goal`, `carter_navigation` symlink 빌드 성공.
- 아래 pytest 6개 파일의 테스트 **40개 통과**. 기존 CPU 검증 37개에 실제 ROS
  메시지/import를 사용하는 `test_hospital_routes.py` 3개가 포함된다.
  - `test_obstacle_tracking.py`
  - `test_hospital_avoidance.py`
  - `test_hospital_stage_policy.py`
  - `test_hospital_route_geometry_cpu.py`
  - `test_amcl_initial_pose_cpu.py`
  - `test_hospital_routes.py`
- 설치된 mission/safety/stage runner/guard/AMCL/predictor/self-filter import,
  Python AST, YAML, launch 인자 해석, `git diff --check` 통과.
- 설치된 `hospital_mission`을 잘못된 route ID로 실행하여 ROS entry point와
  인자 검사를 확인했다. 예상한 오류 메시지를 출력하고 goal 없이 종료했다.
- 실제 controller에서 `StoppedGoalChecker`, DWB `ObstacleFootprint`, MPPI,
  Planner `GridBased`, `simple_smoother` 로딩 확인.
- Isaac 병원 사람 씬 로딩, NavMesh bake, lab 시작 자세 생성 및 Play 확인.
  여섯 사람의 명령/루프 원점 로딩은 확인했지만 각각의 실제 이동량은 측정하지 않았다.
- AMCL이 `/initialpose`를 받고 `/amcl_pose`를 발행했다. 초기화 노드의 재발행 중단
  로그도 확인했다. 이 노드는 발행을 멈춘 뒤에도 프로세스로 남는다.
- AMCL/controller/planner/collision monitor/velocity smoother 모두 `active`.
  `/follow_path` action server 준비 확인. static/local costmap 수신 확인.
- 25초 수신 관측: `/clock` 742개, `/chassis/odom` 743개, `/scan` 191개,
  `/scan_body_filtered` 198개, `/hospital/tracked_obstacles` 199개.
  서로 독립적으로 구독을 연결한 구간의 개수이며 처리율/누락률 비교는 아니다.
- 늦게 연결한 volatile 구독에서는 정지 중 `/amcl_pose`가 새로 오지 않았다.
  transient-local 구독으로 저장된 초기 pose를 확인했다: x=12.00235,
  y=7.99816, x/y 분산 약 0.242/0.247 m². 실제 scan-map 정합 정확도를 검증한 값은 아니다.
- 별도 15초 timer 관측: guard 0 명령 298개 수신, 비영 속도 0개.
  timer 277회 중 odom 기준 safety snapshot 276회, map 기준 237회 유효.
  map TF를 아직 받지 못한 초기 40회를 제외하면 map snapshot은 모두 유효했다.
  track age 0.033–0.400초, odom/수신 후 TF age 0–0.033초였다.
- `map → base_link`와 `odom → base_link` 확인. TF publisher 목록에 AMCL이 있고
  기존 `ground_truth_localization`은 없다.
- `/cmd_vel_smoothed → hospital_velocity_guard → /cmd_vel_human_checked
  → collision_monitor → /cmd_vel → Isaac`의 publisher/subscriber 연결 확인.
  기본 Nav2 bringup의 docking server도 `/cmd_vel` publisher로 존재하지만
  이번 점검에서 docking action을 호출하지 않았다.

초기 AMCL TF 생성 전에 local costmap의 map-frame 오류 2건이 있었고,
초기화 후 추가 오류는 관측하지 않았다. Isaac 기동 중 Python 3.11/시스템
ROS Python 3.12의 rclpy import 경고도 있었지만, 이후 C++ ROS bridge의
실제 센서/clock/TF 통신은 위와 같이 동작했다.

## 실행 명령

빌드는 완료했다. 새 터미널마다 workspace 환경을 불러온다.

터미널 1 — Isaac GUI:

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/home/rokey/isaacsim/python.sh isaacpjt/pjt_alpha/run_hospital_sim.py
```

터미널 2 — Isaac의 병원 씬 Play 이후 Nav2/RViz:

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch carter_navigation hospital_navigation.launch.py
```

`AMCL pose received; initial-pose publishing stopped` 및 navigation의
`Managed nodes are active`를 확인한다. RViz에서 지도/스캔 위치도 확인한다.

터미널 3 — 새 시뮬레이션의 lab에서 lower로 출발:

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run nav_to_goal hospital_mission --ros-args -p route_id:=lab_to_specimen
```

`[MISSION] SUCCEEDED: station reached`로 specimen 도착을 확인한 뒤,
Isaac/Nav2를 유지한 같은 터미널에서 upper 복귀:

```bash
ros2 run nav_to_goal hospital_mission --ros-args -p route_id:=specimen_to_lab
```

기본 route ID는 `specimen_to_lab`이므로 **새 시뮬레이션의 첫 실행에는 반드시
`lab_to_specimen`을 명시한다.** 중도 실패 후 현재 위치 확인 없이 반대 경로를 실행하지 않는다.
현재 launcher는 사람 포함 씬이다. 사람 없는 기준 주행, 짧은 직선/회전 중 AMCL 정합,
전체 경로 완주, 실제 측방 회피·양보·정류장 정지는 별도의 주행 검증이 남아 있다.

## 로그와 변경

- 점검용 Isaac 로그: `/tmp/hospital_preflight_isaac_20260923.log`
- 점검용 Nav2 로그: `/tmp/hospital_preflight_nav2_20260923.log`
- 이번 점검은 자동 대용량 센서/audit 기록기를 추가하지 않았다.
- 제어 소스/파라미터는 변경하지 않았다. 기존 지도 preview PNG의 삭제 상태를 보존했다.
- 사용자 실행과 중복되지 않도록 점검용 Isaac/Nav2 프로세스를 종료했고 잔존하지 않음을 확인했다.
  종료 시 self-filter의 rclpy 예외와 Nav2 container의 SIGINT 종료 지연/SIGTERM
  강제 종료(exit -6)가 로그에 남았다. 기동·정지 상태 통신 확인과 별개로,
  종료 처리가 깨끗하게 완료됐다고 판단하지 않는다.
