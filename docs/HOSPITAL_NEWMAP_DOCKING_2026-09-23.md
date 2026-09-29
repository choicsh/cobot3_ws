# 새 병원 맵·기존 테이블 도킹 적용 — 2026-09-23

## 사용한 파일과 기존 테이블

사용자 Downloads의 `hospital_integration_human.usd`, `.yaml`, `.png`를
workspace에 복사했다. 세 파일의 SHA-256이 원본과 같음을 확인했다.
USD 형상, 테이블 배치, 로봇 원래 시작 자세를 수정하지 않았다.

- USD: `isaacpjt/assets/hospital_integration_human.usd`
- 지도: `src/nova_carter/carter_navigation/maps/hospital_integration_human.yaml/png`
- 도킹 대상: `/World/NewRooms/Desks/East_DockDesk`, `West_DockDesk`
- 비활성 `/World/Desks`는 사용하지 않는다.
- 누락된 검사 장비 두 참조는 사용자가 승인한 대로 없는 상태에서 시험한다.
- 새 USD 시작: `(20.27, 13.74534, +90°)`. 처음에는 옆 테이블에서 충분히
  북쪽으로 벗어난 뒤 lower에 합류한다.
- 기존 6명 명령 대신 새 씬의 `Character`, `Character_01`, `Character_02`에
  맞춘 3명 명령 파일을 연결했다. 원본 씬의 사람 수를 늘리지 않았다.

## 속도 guard

`hospital_velocity_guard`를 병원 launch에서 제거하고 Collision Monitor의
입력을 `/cmd_vel_smoothed`로 돌렸다. guard 소스/단위시험은 남아 있지만
기본 병원 실행에는 이 노드가 없다.

사람 없는 실험에서 테이블이 이동 track으로 잡혀 guard가
`NO_SAFE_COMMAND predicted_gap=-0.506`을 출력한 사례가 있었다.
상시 직선 상한을 낮추는 장치는 아니었지만, 이 경우 불필요한 추가 정지를 만들었다.
기존 Collision Monitor, 자기 반사 제거, MPPI 직선 구간의 경로 감독/회피는 유지했다.
방 안 DWB 구간은 full-footprint 장애물 검사와 Collision Monitor를 사용한다.
추가 예측 guard를 제거했으므로 과거 guard 포함 시험의 사람 간격 결과를
현재 설정의 보장으로 사용할 수 없다.

```text
Nav2 / 도킹 직선 제어 → cmd_vel_nav → velocity_smoother
                    → cmd_vel_smoothed → collision_monitor → cmd_vel → Isaac
```

MPPI 직선 상한 0.6 m/s, 일반 DWB 0.8 m/s, 방 접근 DWB 0.5 m/s는 유지한다.
마지막 테이블 옆 직선 이동은 별도 정밀 제어(상한 0.18 m/s)다.
이 속도가 복도 전체에 적용되는 것은 아니다.

## 경로와 도킹

지도 원점이 기존과 다르며 새 좌우 벽에 옛 경로가 걸렸다.
전체 footprint로 벽을 검사해 출입구를 통과하는 upper/lower 좌표를 새로 잡았다.
모든 원호/방 진입은 DWB, 중앙 직선은 MPPI를 사용한다.
경로 그림: `docs/hospital_newmap_routes_2026-09-23.png`.

두 테이블의 남쪽 면은 `y=12.16249995`다. 요청한 map yaw 180°(-180°와 동일)는
이 남쪽 면과 평행하다. 로봇 반폭 0.5m + 옆면 간격 0.05m를 빼서
도킹 기준 y를 `11.61249995`로 계산했다.

| 대상 | 도킹 x | 도킹 y | 목표 yaw | 도킹 전 정렬 x |
|---|---:|---:|---:|---:|
| lab / East_DockDesk | 21.285 | 11.61250 | 180° | 18.7 |
| specimen / West_DockDesk | -46.741 | 11.61250 | 180° | -43.4 |

긴 후방 때문에 `base_link`가 차체 길이 중심이 아니다. 목표 x에는
뒤 1.38m / 앞 0.48m의 비대칭을 반영했다.

- 서쪽: 테이블 동쪽 열린 공간에서 180°로 정렬하고 서진하여 도킹한다.
- 동쪽: 테이블 오른쪽 벽이 가까워 서쪽 열린 공간에서 180°로 정렬한 뒤
  자세를 유지하며 후진(월드 +x)하여 도킹한다.
- 다음 출발에는 먼저 같은 직선으로 테이블에서 빠져나온다. 서쪽은 열린
  정렬 지점에서 북향으로 회전한 뒤 upper를 출발한다.
- 테이블 근처 제어도 Collision Monitor를 거친다. 최종 cmd_vel을 직접 덮어쓰지 않는다.
- 기존 라이다의 원본 3D 포인트에서 테이블 남쪽 경계를 선으로 추정한다.
  상판의 촘촘한 반사점을 옆면으로 착각하지 않도록 길이 방향 구간별로 로봇에
  가장 가까운 경계를 추출한다. 취득 시각과 현재 TF 시각 사이의 운동도 보상한다.
  위치만 맞았다고 성공하지 않고,
  측면 간격 5±1cm, 평행 오차 0.5° 이내, 종방향 오차 1.5cm 이내,
  실제 정지 상태가 0.5초 유지되어야 도킹 성공으로 처리한다.
- 신선한 scan/odom/TF 또는 충분한 테이블 면 관측이 없으면 정지·실패 처리한다.
- 지도 해상도 자체가 5cm라 최종 옆면 간격은 지도 픽셀만으로 판정하지 않는다.
  픽셀 경계 보수 검사와 footprint padding을 더하면 5cm 도킹 구간은 지도상
  차단으로 나올 수 있다. 이 구간은 원본 라이다 경계와 실제 USD 기하로 별도 검증한다.
  종방향 오차는 AMCL 좌표 기준이며, 절대 위치가 1.5cm 정확하다는 의미는 아니다.
- Pick & Place는 추가하지 않았다.

## 실행 중 확인한 결함 수정

- 미션이 루프당 ROS callback 한 개만 처리하면 여러 TF/clock/odom 입력을
  따라가지 못했다. 제한된 시간 안에서 준비된 callback을 처리한 뒤 신선도를 검사한다.
- 서로 다른 토픽 도착 순서 때문에 30Hz clock보다 odom/TF가 한 tick 앞서는
  경우가 있었다. 40ms 이내만 허용하며, 0.6초 초과 stale 및 더 먼 미래 입력은 거부한다.
- Isaac의 정확한 시작 자세에도 AMCL 초기 분산이 0.5m/10°라 인접 테이블 안쪽으로
  약 0.2m 보정되는 사례가 있었다. 초기 분산만 2cm/1°로 변경했다.
  이는 실제 localization 오차가 그 이내임을 보장하는 설정이 아니다.
- 새 방 출입구의 벽 모서리가 이동 track으로 인식되어 불필요한 양보가 발생했다.
  정적 지도 점유점 주변 18cm는 이동 추적 입력에서 제외한다. 원본 scan,
  costmap 및 Collision Monitor의 장애물 입력은 그대로 유지한다.
  방의 DWB 구간에는 MPPI 전용 예측 경로 선택/양보를 적용하지 않는다.
- 기존 전방 slowdown은 회전 속도도 40%로 줄여 아주 작은 정렬 명령이 정지 마찰을
  넘지 못했다. 열린 정렬 지점에서 제한된 회전 명령으로 먼저 자세를 맞추고,
  도킹 중에는 관측한 책상 경계에 평행하게 직선 제어한다.

## 검증

- 단위/ROS import/새 지도 경로/라이다 경계 fitting/시각 skew 시험 50개 통과.
- 세 패키지 빌드 성공.
- 기존 테이블·새 벽을 바꾼 것이 아닌 원본 USD/지도에서 시험한다.
- 실제 주행 및 도킹 결과는 아래 실행 결과 절에 기록한다.

## 실행

터미널 1:

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/home/rokey/isaacsim/python.sh isaacpjt/pjt_alpha/run_hospital_sim.py
```

사람 없는 기준 시험은 마지막 줄 앞에 `HOSPITAL_PEOPLE_ENABLED=0`을 붙인다.
기본은 사람 3명을 포함한다.

터미널 2:

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch carter_navigation hospital_navigation.launch.py
```

터미널 3, AMCL 초기화/Nav2 active 확인 후:

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run nav_to_goal hospital_mission --ros-args -p route_id:=lab_to_specimen
```

서쪽 도킹 성공 뒤 동일한 Isaac/Nav2를 유지하고:

```bash
ros2 run nav_to_goal hospital_mission --ros-args -p route_id:=specimen_to_lab
```

출발점이 새 USD spawn이나 해당 정렬/도킹 지점과 다르면 goal을 보내지 않고 실패한다.
중도 실패 후 반대 방향 명령을 무조건 보내지 않는다.

## 실행 결과

- 사람 없는 Isaac/Nav2 실주행: 새 시작점 departure와 lower MPPI 직선 통과.
  직선 실제 odom 속도 중앙값 약 0.598m/s.
- 수정 후 서쪽 도킹 단독 시험 성공: 라이다 간격 0.04994m,
  평행 오차 0.045°, AMCL 종방향 오차 0.0097m.
  별도 Isaac 실제 자세 기록에서도 마지막 모서리 간격 약 4.9cm를 확인했다.
- 서쪽 도킹 해제 → upper departure → upper MPPI → 동쪽 방 staging 도착 성공.
  upper에서 우회 및 전방 합류도 관측했다. 사람 없는 시험이므로 사람 회피
  검증으로 해석하지 않는다.
- **동쪽 책상 최종 도킹은 실패**: 열린 staging에서 정렬 제한시간 35초를 넘겼다.
  서쪽 도킹 성공을 양쪽 도킹 완료로 표현하지 않는다. 마지막 정렬은 추가 보완 대상이다.
- 최신 수정 전체를 적용한 새 시작점부터의 lower 완주 및 사람 포함 완주는 아직
  확정하지 않았다. 사용자가 직접 보도록 headless 시험을 종료하고 GUI 실행으로 전환했다.
