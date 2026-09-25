# 검체 이송 로봇 관제 웹

`feature/HJ` 의 관제 대시보드를 통합 시스템(DB v5 + 관제 구역 예약)에 맞춘 것. 파이썬 표준 라이브러리만 쓰고
DB 드라이버 없이 Docker 컨테이너 안의 `psql` / `redis-cli` 로 조회한다.

## 실행

작업공간 루트에서 (차선 구역을 그리려면 ROS 워크스페이스를 source 한 셸에서):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 monitoring_web/server.py            # --host 0.0.0.0 으로 다른 PC 에서도 접속
```

브라우저에서 <http://127.0.0.1:8080>.

## 보여 주는 것

| 화면 | 출처 |
|---|---|
| 로봇 위치·방향·단계·작업 번호, 통신(heartbeat) | Redis `robot:{id}:state`, `robot:{id}:heartbeat` (없으면 PostgreSQL `robot_state_history` 최신 행) |
| 지도 위 차선 구역, 충돌 구역(두 문, 주황 점선) | `hospital_system.lane_graph` (로봇이 따르는 같은 기하) |
| 로봇별 예약 구역(굵은 색 선), 지금 구간 경로(가는 선) | Redis `fleet:zones`, `robot:{id}:route` (`fleet_manager` 가 기록) |
| 이송 작업(트레이 수·칸·긴급도), 상태 로그, 로봇 이벤트, 트레이 | PostgreSQL `transport_task`, `tray`, `task_status_log`, `robot_event_log` |

- 긴급도는 **3 = 긴급**, 2 = 우선, 1 = 일반 (랙 ArUco 판독, 미검출 = 1).
- 지도는 Nav2 가 쓰는 `hospital_integration_human.yaml` — 원점·해상도·크기를 YAML/PNG 에서 읽는다.
- 브라우저는 `/api/stream`(Server-Sent Events, 1 s)으로 바뀐 내용만 받는다. `/api/dashboard`, `/api/lanes`, `/api/health`.

환경 변수: `ROBOT_DB_CONTAINER`, `ROBOT_DB_NAME`, `ROBOT_DB_USER`, `ROBOT_REDIS_CONTAINER`, `ROBOT_REDIS_USER`,
`ROBOT_REDIS_PASSWORD`, `ROBOT_MAP_YAML`.

`feature/HJ` 의 `simulate_realtime.py` 와 `seed_robotdb3_demo.sql` 은 옛 스키마(`specimen`, 한글 상태) 기준이고
시드 SQL 은 전 테이블을 TRUNCATE 하므로 가져오지 않았다. 움직이는 화면은 실제 시스템(또는 `robot_agent` 가짜 팔/주행 시험)으로 본다.
