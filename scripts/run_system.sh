#!/bin/bash
# 병원 검체 운송 시스템 전체 실행 — DB, Isaac, 검출기, Nav2(로봇별), db_worker, 관제, 로봇 에이전트, (관제 웹)
#
#   scripts/run_system.sh                    # 로봇 2대, 2사이클
#   ROBOTS=3 CYCLES=1 scripts/run_system.sh
#   ROBOTS=3 CYCLES=0 WEB=1 scripts/run_system.sh      # 계속 운용 + 관제 웹, Ctrl-C 로 끝
#
# 환경 변수 (기본값)
#   ROBOTS=2          로봇 수 1..3
#   CYCLES=2          로봇마다 사이클 수 (0 = 계속, Ctrl-C 로 끝)
#   RVIZ=True         Nav2 RViz. RVIZ_MAX=N 이면 로봇 N 까지만 띄운다 (3대에서 무거우면 RVIZ_MAX=1)
#   WEB=0             1 이면 관제 웹(http://127.0.0.1:8080)도 띄운다
#   BAG=0             1 이면 로봇별 주요 토픽을 rosbag 으로 남긴다
#   POSE1..3 START1..3  로봇 배치 (docs/SYSTEM_RUN_GUIDE.md §4) — 기본: robot1 채취실 도킹, 2·3 복귀 차선 위
#   SIM_EXTRA=""      run_fleet_sim.py 에 더 줄 옵션 (예: --no-people, --headless)
#   LOG_DIR=~/cobot3_logs/<시각>   로그 폴더
#   DB_PYTHON=python3 관제·에이전트·db_worker 파이썬 — psycopg2/redis 가 있어야 한다 (sudo apt install python3-psycopg2
#                     python3-redis, 또는 그 둘이 든 --system-site-packages venv 의 python)
#   ISAAC=~/isaacsim/python.sh   YOLO_PY=~/yolo-venv/bin/python   ROS_DOMAIN_ID=136
#
# 순서와 이유는 docs/SYSTEM_RUN_GUIDE.md. 끝나면(사이클 다 돔 / Ctrl-C) 역순으로 모두 멈추고 DB 작업 요약을 찍는다.
# 사이클 수를 정한 실행은, 마지막에 복귀한 로봇이 앞 로봇(채취실에서 다음 적재를 기다림) 뒤에 줄을 선 채 끝난다 —
# N번째 복귀를 시작한 로봇이 예약 정지(HOLDING)로 60 s 서 있으면 끝난 것으로 본다.

WS=$(cd "$(dirname "$0")/.." && pwd)
N=${ROBOTS:-2}; CYC=${CYCLES:-2}
LOG=${LOG_DIR:-$HOME/cobot3_logs/$(date +%Y%m%d_%H%M%S)}
ISAAC=${ISAAC:-$HOME/isaacsim/python.sh}
YOLO_PY=${YOLO_PY:-$HOME/yolo-venv/bin/python}
WEIGHTS=$WS/runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-136} PYTHONUNBUFFERED=1
declare -A POSE=([1]=${POSE1:-collection} [2]=${POSE2:-0.0,14.9,0} [3]=${POSE3:--15.0,14.9,0})
declare -A START=([1]=${START1:-collection} [2]=${START2:-return} [3]=${START3:-return})

[[ $N =~ ^[123]$ ]] || { echo "ROBOTS 는 1..3"; exit 1; }
[ -x "$ISAAC" ] || { echo "Isaac Sim 이 없다: $ISAAC"; exit 1; }
[ -x "$YOLO_PY" ] || { echo "검출기 파이썬이 없다: $YOLO_PY (admin_ws/README.md)"; exit 1; }
# 다른 도메인에 남은 Nav2 가 지도 전달을 막은 적이 있다 — 이전 실행이 남아 있으면 시작하지 않는다
if pgrep -f 'kit/python/bin/python[3] |component_container_[i]solated|[r]viz2 |hospital_[m]ission|detect_[n]ode' >/dev/null; then
  echo "이전 실행의 Isaac/Nav2/RViz/주행/검출기 프로세스가 남아 있다:"
  pgrep -af 'kit/python/bin/python[3] |component_container_[i]solated|[r]viz2 |hospital_[m]ission|detect_[n]ode' | cut -c1-120
  exit 1
fi
DB_PYTHON=${DB_PYTHON:-python3}
"$DB_PYTHON" -c "import psycopg2, redis" 2>/dev/null ||
  { echo "$DB_PYTHON 에 DB 드라이버가 없다: sudo apt install python3-psycopg2 python3-redis (또는 DB_PYTHON=...)"; exit 1; }
# ros2 run 대신 파이썬으로 바로 exec — 백그라운드 PID 가 곧 노드라 멈출 때 신호가 노드에 간다
# (비대화형 셸의 백그라운드 서브셸·ros2 run 래퍼는 SIGINT 를 무시해 노드까지 전달되지 않는다)
hs() { exec "$DB_PYTHON" -m "hospital_system.$1" "${@:2}"; }

cd "$WS"
source /opt/ros/jazzy/setup.bash >/dev/null
source install/setup.bash
mkdir -p "$LOG"
T0=$(date +%s); ts() { printf '[%4ds]' $(( $(date +%s) - T0 )); }
echo "로그: $LOG   (로봇 $N대, 사이클 $CYC, ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"

docker start robotdb3_sql robotdb3_nosql >/dev/null || { echo "DB 컨테이너를 켜지 못했다 (DB_container/)"; exit 1; }
docker exec robotdb3_nosql redis-cli --user rokey --pass rokey --no-auth-warning del fleet:zones >/dev/null

SIM= DETS= NAVS= WORKER= FLEET= AGENTS= BAGP= WEBP=
stop_all() {
  trap - INT TERM
  echo "$(ts) 종료 중..."
  # rclpy 노드는 SIGTERM 에서도 정리한다 — 에이전트는 돌던 주행을 멈추고 작업을 CANCELLED 로, 관제는 예약 표시를 지운다
  for p in $AGENTS; do kill -TERM $p 2>/dev/null; done
  for p in $AGENTS; do wait $p 2>/dev/null; done
  kill -TERM $FLEET $WORKER $WEBP 2>/dev/null
  [ -n "$BAGP" ] && { kill -TERM $BAGP 2>/dev/null; wait $BAGP 2>/dev/null; }
  for p in $NAVS; do kill -TERM -- -$p 2>/dev/null; done
  kill -TERM $DETS 2>/dev/null
  for p in $(pgrep -f 'kit/python/bin/python[3] '); do kill -INT $p; done
  for i in $(seq 1 30); do pgrep -f 'kit/python/bin/python[3] ' >/dev/null || break; sleep 1; done
  sleep 3
  for p in $(pgrep -f 'kit/python/bin/python[3] '); do kill -9 $p; done
  for p in $NAVS; do kill -9 -- -$p 2>/dev/null; done
  kill -9 $DETS $FLEET $WORKER $WEBP 2>/dev/null
  pgrep -f 'kit/python/bin/python[3] |component_container_[i]solated|[r]viz2 |hospital_[m]ission|detect_[n]ode' >/dev/null &&
    echo "경고: 아직 남은 프로세스가 있다 — pgrep -af 'component_container_[i]solated|[r]viz2 '" || echo "$(ts) 모두 멈춤"
  echo "── 작업 (최근 2시간)"
  docker exec robotdb3_sql psql -U rokey -d robotdb3_sql -c "select t.task_id, r.robot_name, t.status,
    t.departed_at::time(0) as departed, t.arrived_at::time(0) as arrived, t.cancel_reason
    from transport_task t join robot_info r using(robot_id)
    where t.task_id in (select task_id from task_status_log where changed_at > now() - interval '2 hours') order by 1"
  echo "── 관제 경로 배정"
  grep -a -E 'via|yields' "$LOG/fleet.log" | sed 's/.*\[FLEET\] //'
  echo "로그: $LOG"
  exit 0
}
trap stop_all INT TERM

# 1. Isaac Sim — 재생을 시작하면 로봇별 시작 자세 파일(start_pose_robotN.json)을 쓴다
POSE_ARGS=""; for i in $(seq 1 $N); do POSE_ARGS="$POSE_ARGS --pose $i:${POSE[$i]}"; done
"$ISAAC" isaacpjt/system/run_fleet_sim.py --robots $N $POSE_ARGS $SIM_EXTRA > "$LOG/sim.log" 2>&1 & SIM=$!
# 2. 트레이 검출기 (로봇마다, yolo-venv — ROS 는 시스템 site-packages 에서)
for i in $(seq 1 $N); do
  PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH "$YOLO_PY" \
    admin_ws/src/tray_detector/tray_detector/detect_node.py "$WEIGHTS" --ros-args -r __ns:=/robot$i \
    > "$LOG/detector$i.log" 2>&1 & DETS="$DETS $!"
done
# 3. DB 워커
hs db_worker > "$LOG/db_worker.log" 2>&1 & WORKER=$!
echo "$(ts) Isaac 시작 대기 (1분 안팎)..."
for i in $(seq 1 400); do
  grep -q '재생 시작' "$LOG/sim.log" && break
  kill -0 $SIM 2>/dev/null || { echo "Isaac 이 시작 중에 끝났다 — $LOG/sim.log"; stop_all; }
  sleep 1
done
echo "$(ts) Isaac 재생"; grep -a '\[FLEET\] /robot' "$LOG/sim.log" | sed 's/^/        /'
# 4. Nav2 (로봇마다, 8 s 간격 — RViz 두 개를 같은 순간에 띄우면 하나가 세그폴트한다)
for i in $(seq 1 $N); do
  RV=${RVIZ:-True}; [ "$i" -gt "${RVIZ_MAX:-9}" ] && RV=False
  setsid ros2 launch carter_navigation hospital_navigation.launch.py namespace:=robot$i use_rviz:=$RV \
    start_pose_path:=$WS/isaacpjt/assets/start_pose_robot$i.json > "$LOG/nav$i.log" 2>&1 & NAVS="$NAVS $!"
  sleep 8
done
if [ "${BAG:-0}" = 1 ]; then
  TOPICS=""
  for i in $(seq 1 $N); do for t in agent_status arm/status route zone_hold fleet_pose task amcl_pose chassis/odom \
      cmd_vel plan hospital/reference_plan hospital/mission_state hospital/human_guard_state hospital/tracked_obstacles; do
    TOPICS="$TOPICS /robot$i/$t"; done; done
  ros2 bag record -o "$LOG/bag" $TOPICS > "$LOG/bag.log" 2>&1 & BAGP=$!
fi
# 5. 관제
RL=$(seq -s, -f "'robot%g'" 1 $N)
hs fleet_manager --ros-args -p "robots:=[$RL]" -p cycles:=$CYC > "$LOG/fleet.log" 2>&1 & FLEET=$!
# 6. 로봇 에이전트 (로봇마다)
RC=$CYC; [ "$CYC" = 0 ] && RC=1000000
for i in $(seq 1 $N); do
  hs robot_agent --ros-args -r __ns:=/robot$i -p fleet:=true -p run_cycles:=$RC -p start:=${START[$i]} \
    > "$LOG/agent$i.log" 2>&1 & AGENTS="$AGENTS $!"
done
# 7. 관제 웹
if [ "${WEB:-0}" = 1 ]; then
  exec_web() { exec python3 -u monitoring_web/server.py; }
  exec_web > "$LOG/web.log" 2>&1 & WEBP=$!
  echo "$(ts) 관제 웹 http://127.0.0.1:8080"
fi
echo "$(ts) 실행 중 — Ctrl-C 로 멈춘다"

queued_done() {   # 로봇 i 가 N번째 복귀를 시작한 뒤 예약 정지로 60 s 서 있다
  local i=$1 need=$CYC; [ "${START[$i]}" = return ] && need=$(( CYC + 1 ))
  [ "$(grep -a -c -E '\[RETURNING\] analysis -> collection$' "$LOG/agent$i.log")" -ge $need ] || return 1
  local last; last=$(grep -a -o -E '^\[mission\] \[INFO\] \[[0-9]+\.[0-9]+\].*state=[A-Z_]+' "$LOG/agent$i.log" | tail -1)
  [[ "$last" == *state=HOLDING ]] || return 1
  local t; t=$(echo "$last" | grep -o -E '\[[0-9]+\.' | head -1 | tr -d '[.')
  [ $(( $(date +%s) - t )) -ge 60 ]
}
declare -A LAST
while :; do
  kill -0 $SIM 2>/dev/null || { echo "$(ts) Isaac 이 끝났다 — $LOG/sim.log"; stop_all; }
  alive=0; done_all=1; i=0
  for p in $AGENTS; do
    i=$((i + 1))
    if kill -0 $p 2>/dev/null; then
      alive=1
      { [ "$CYC" != 0 ] && queued_done $i; } || done_all=0
    fi
    NOW=$(grep -a -o -E '\[(PICKING|DELIVERING|PLACE_DOCKING|PLACING|RETURNING|PICK_DOCKING|IDLE|ERROR)\].*' "$LOG/agent$i.log" | tail -1)
    [ "$NOW" != "${LAST[$i]}" ] && { echo "$(ts) robot$i $NOW"; LAST[$i]=$NOW; }
  done
  [ $alive = 0 ] && { echo "$(ts) 모든 에이전트가 끝났다"; stop_all; }
  [ $done_all = 1 ] && { echo "$(ts) 모든 로봇이 사이클을 마쳤다 (마지막 로봇은 채취실 앞 줄에서 대기)"; stop_all; }
  sleep 3
done
