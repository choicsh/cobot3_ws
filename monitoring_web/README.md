# 검체 이송 로봇 관제 웹

`robotdb3_sql` Docker 컨테이너의 PostgreSQL 정보를 조회해 localhost에서 보여주는 관제 대시보드입니다.

## 실행

작업공간 루트에서 다음 명령을 실행합니다.

```bash
python3 monitoring_web/server.py
```

브라우저에서 <http://127.0.0.1:8080>으로 접속합니다.

실시간 이동을 확인하려면 별도 터미널에서 더미 위치 생성기를 실행합니다.

```bash
python3 monitoring_web/simulate_realtime.py
```

생성기는 1.5초마다 로봇 3대의 새 위치 이력을 기록합니다. `Ctrl+C`로 종료할 수 있으며,
데이터가 무한히 증가하지 않도록 기본적으로 로봇당 최근 120개 이력만 유지합니다.

기본 연결 대상은 다음과 같습니다.

- Docker 컨테이너: `robotdb3_sql`
- 데이터베이스: `robotdb3_sql`
- 사용자: `rokey`
- 지도: `src/nova_carter/carter_navigation/maps/integration_hospital.png`

필요하면 환경 변수 `ROBOT_DB_CONTAINER`, `ROBOT_DB_NAME`, `ROBOT_DB_USER`, `ROBOT_MAP_PATH`로 변경할 수 있습니다.

## 실시간 갱신

브라우저는 `/api/stream`의 Server-Sent Events 스트림에 연결됩니다. 서버는 DB 스냅샷을 약 1.5초마다 확인하고 변경된 내용을 즉시 전송합니다. 연결이 끊기면 브라우저가 자동으로 재연결합니다.

지도 좌표는 ROS 맵 YAML의 해상도 `0.05 m/px`와 원점 `[-49.975, -9.475]`를 사용해 `robot_state_history`의 최신 `(x, y)` 값을 픽셀 위치로 변환합니다.
