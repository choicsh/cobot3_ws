#!/bin/bash
# 관제 웹 재생 — 로봇 PC rosbag + 관제 DB 덤프로 관제 화면을 원래 시간 순서대로 다시 만든다 (화면 녹화용).
#
#   scripts/replay_dashboard.sh up   <실행 로그 폴더>        # 재생 DB·관제·웹 준비 (http://<이 PC>:8081)
#   scripts/replay_dashboard.sh play <실행 로그 폴더> [배속]  # 녹화를 켠 뒤 재생 (기본 1배)
#   scripts/replay_dashboard.sh down <실행 로그 폴더>         # 모두 멈추고 재생 DB 컨테이너 삭제
#
# <폴더>/bag              로봇 PC rosbag (agent_status, fleet_pose, arm/status 가 있어야 한다)
# <폴더>/replay/control_db.dump   관제 DB pg_dump -Fc — 이번 실행(첫 agent_status) 이후 행은 잘라 내고 재생이 다시 쓴다
#
# 관제 PC·로봇 PC 의 기존 DB 와 ROS 는 건드리지 않는다: 재생 DB 는 따로 컨테이너(5433/6380),
# ROS 는 ROS_DOMAIN_ID=137 + localhost 전용. 기록은 robot_agent 와 같은 Recorder 함수(replay_recorder.py),
# 예약 구역·복도 경로는 실제 fleet_manager 가 같은 입력으로 다시 계산한다.
# 원본과 다른 점: 화면의 시각은 재생하는 지금 기준, 예약 구역은 재계산이라 수백 ms 어긋날 수 있다.

CMD=$1; RUN=$(cd "${2:?실행 로그 폴더}" && pwd); RATE=${3:-1.0}
WS=$(cd "$(dirname "$0")/.." && pwd)
R=$RUN/replay; mkdir -p "$R"
SQL=robotdb3_replay_sql; RED=robotdb3_replay_nosql
cd "$WS"
source /opt/ros/jazzy/setup.bash >/dev/null
source install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=137 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONUNBUFFERED=1
export HOSPITAL_PG_DSN=postgresql://rokey:rokey@localhost:5433/robotdb3_sql
export HOSPITAL_REDIS_URL=redis://rokey:rokey@localhost:6380/0
ROBOTS="robot1 robot2 robot3"
psql() { docker exec -i $SQL psql -X -q -U rokey -d robotdb3_sql -v ON_ERROR_STOP=1 "$@"; }
bg() { local name=$1; shift; setsid "$@" > "$R/$name.log" 2>&1 & echo "$name $!" >> "$R/pids"; }

case $CMD in
up)
  [ -f "$R/control_db.dump" ] || { echo "$R/control_db.dump 가 없다 (관제 DB pg_dump -Fc)"; exit 1; }
  [ -f "$R/pids" ] && { echo "이미 떠 있다 — 먼저 down"; exit 1; }
  read -r T0 OFFSET < <(python3 scripts/replay_recorder.py --bag-start "$RUN/bag")
  echo "$T0 $OFFSET" > "$R/start"
  echo "재생 시작점: $T0 (bag 시작 뒤 ${OFFSET}s, 첫 agent_status)"
  docker rm -f $SQL $RED >/dev/null 2>&1
  docker run -d --name $SQL -e POSTGRES_DB=robotdb3_sql -e POSTGRES_USER=rokey -e POSTGRES_PASSWORD=rokey \
    -p 5433:5432 postgres:16 >/dev/null || exit 1
  docker run -d --name $RED -p 6380:6379 redis:8.8 redis-server --user rokey on '>rokey' '~*' +@all >/dev/null || exit 1
  for i in $(seq 1 60); do docker exec $SQL pg_isready -U rokey -d robotdb3_sql >/dev/null 2>&1 && break; sleep 1; done
  sleep 2
  docker exec -i $SQL pg_restore -U rokey -d robotdb3_sql --no-owner < "$R/control_db.dump" || exit 1
  # 이번 실행 이후 행을 잘라 실행 직전 상태로 — 재생이 같은 순서로 다시 쓴다
  psql -v t0="$T0" >/dev/null <<'EOF' || exit 1
DELETE FROM transport_task t
 WHERE (SELECT min(changed_at) FROM task_status_log l WHERE l.task_id = t.task_id) >= :'t0';
DELETE FROM tray WHERE created_at >= :'t0';
DELETE FROM robot_event_log WHERE occurred_at >= :'t0';
DELETE FROM robot_state_history WHERE recorded_at >= :'t0';
UPDATE robot_info SET is_active = false;
SELECT setval(pg_get_serial_sequence('transport_task', 'task_id'), (SELECT coalesce(max(task_id), 1) FROM transport_task));
SELECT setval(pg_get_serial_sequence('task_status_log', 'log_id'), (SELECT coalesce(max(log_id), 1) FROM task_status_log));
SELECT setval(pg_get_serial_sequence('robot_event_log', 'event_id'), (SELECT coalesce(max(event_id), 1) FROM robot_event_log));
EOF
  psql -At -c "select 'robots: ' || string_agg(robot_id || '=' || robot_name, ', ' order by robot_id) from robot_info;
               select 'tasks before run: ' || coalesce(string_agg(task_id || ' ' || status, ', ' order by task_id), 'none') from transport_task;"
  bg db_worker python3 -m hospital_system.db_worker
  bg fleet ros2 run hospital_system fleet_manager --ros-args -p "robots:=['robot1','robot2','robot3']" -p cycles:=0
  bg recorder python3 scripts/replay_recorder.py $ROBOTS
  ROBOT_DB_CONTAINER=$SQL ROBOT_REDIS_CONTAINER=$RED bg web python3 -u monitoring_web/server.py --host 0.0.0.0 --port 8081
  sleep 3
  echo "관제 웹(재생): http://127.0.0.1:8081  (다른 PC: http://$(hostname -I | awk '{print $1}'):8081)"
  echo "녹화를 켠 뒤: scripts/replay_dashboard.sh play $RUN [배속]"
  ;;
play)
  [ -f "$R/start" ] || { echo "먼저 up"; exit 1; }
  read -r T0 OFFSET < "$R/start"
  TOPICS=""; for r in $ROBOTS; do TOPICS="$TOPICS /$r/agent_status /$r/fleet_pose /$r/arm/status"; done
  START=$(python3 -c "print(max(0.0, $OFFSET - 2.0))")
  echo "재생: ${RATE}배, bag ${START}s 부터"
  ros2 bag play "$RUN/bag" --topics $TOPICS --start-offset "$START" --rate "$RATE" > "$R/play.log" 2>&1
  sleep 2
  # 원본처럼 에이전트 종료 = 열린 작업 CANCELLED
  kill -TERM "$(awk '$1=="recorder"{print $2}' "$R/pids")" 2>/dev/null
  echo "재생 끝 (열린 작업은 취소로 기록). 멈추려면: scripts/replay_dashboard.sh down $RUN"
  ;;
down)
  if [ -f "$R/pids" ]; then
    while read -r name pid; do kill -TERM -- -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null; done < "$R/pids"
    sleep 3
    while read -r name pid; do kill -9 -- -"$pid" 2>/dev/null; done < "$R/pids"
    rm -f "$R/pids"
  fi
  docker rm -f $SQL $RED >/dev/null 2>&1
  echo "재생 정리 끝"
  ;;
*)
  sed -n '2,8p' "$0"; exit 1 ;;
esac
