# 병원 AMR 회피 강화 인수인계 — 2026-09-23

## 먼저 알아야 할 상태

- 기준 저장소: `cobot3_ws`, 브랜치 `feature/followpath`. 최초 회피 강화 구현은 `febcecb`로 커밋했다.
- 이 문서는 9월 22일 실주행 기록 이후 **노트북에서 추가 구현한 변경**이다. 아래 센서·Planner 흐름 보강은 그 후속 변경이다.
- 로컬 검증은 순수 Python 기하·예측·상태 로직과 지도 충돌 검사다. **새 코드로 Nav2/Isaac 실주행에 성공했다는 뜻이 아니다.**
- 노트북에 `nav2_msgs`가 없어 ROS 경로 테스트의 실제 import 및 Nav2 통합 실행은 검증하지 못했다. 원본 센서/audit JSONL도 확보하지 못했다. 과거 결과는 인수인계 문서에 기록된 사실로만 인용한다.
- 사용자 의도: 회피 가능한 정면/측면 조우에서는 눈에 보이는 측방 회피를 우선한다. 긴 후방과 운동 제한 때문에 대응하기 어려운 급진입까지 완벽한 회피를 보장하는 것이 목표는 아니다. 그렇더라도 겹침을 성공으로 바꿔 기록하지 않는다.
- 사용자 최신 지시: **정류장 도착 전진 상한은 0.5m/s.** 마지막 1m를 0.15~0.20m/s로 낮추는 이전 제안은 채택하지 않았다. 0.5는 검증된 물리 임계값이 아니라 사용자 지정 상한이며 목표점에서는 정지해야 한다.
- 렉, 설치 환경, 실행 기본 경로 변경은 이번 개선 대상에서 제외했다.
- 정상 운행은 기존 고정 Path + FollowPath. 기존 로봇/Prim/센서 입력 이름 유지. 도킹/Pick & Place 통합과 관제용 완전 차단 판정은 보류.

## 실행 대상 혼동 금지

현재 병원 구현 대상은 아래다. 예전 `move_test.py`나 `carter_navigation_params.yaml`은 이전 지도용 내용이 남아 있으므로 이번 변경의 실행 대상으로 혼동하지 않는다.

- `isaacpjt/pjt_alpha/run_hospital_sim.py` — 이번에는 변경하지 않음.
- `src/nova_carter/carter_navigation/launch/hospital_navigation.launch.py`
- `src/nova_carter/carter_navigation/params/hospital_navigation_params.yaml`
- `src/nova_carter/nav_to_goal/nav_to_goal/hospital_mission.py`
- 지도 `integration_hospital.yaml/png` — 변경하지 않음.
- 정류장: specimen(-36,8), lab(12,8). upper/lower 기준 경로와 다음 출발 접선 방향 유지.

## 실제 구현한 회피 강화

### A. 사람 위치의 시간 정보 보존

기존 `/hospital/predicted_obstacles`는 radius 채널만 있는 soft-cost용 PointCloud로 그대로 유지했다. C++ PredictionLayer는 변경하지 않았다.

새 `/hospital/tracked_obstacles`는 관측 scan 시각, odom frame의 현재 위치, `track_id`, `vx`, `vy`, `radius`, `observation_age`를 전달한다. 위치는 메시지 stamp까지 보상된 값이며, 소비자는 메시지 age만큼 추가 예측한다. uncertainty에는 총 관측 age를 적용한다. 시간 정보 없는 미래 점 묶음을 충돌 검사에 쓰지 않는다.

추적 거리 기본값을 6→8m로 넓혔다. 기존 scan 범위 안의 정보를 더 일찍 쓰는 것이며 센서 하드웨어/센서 토픽을 바꾼 것이 아니다. 노드 파라미터 `tracking_range`로 조정 가능하다.

한 번 움직임이 확인된 track은 관측이 계속되는 동안 정지해도 보호된다. 영구 유령 track은 만들지 않으며 미관측 0.6초 이후 만료된다. 이는 사람 의미 인식기가 아니다. 가림·cluster 분리·다른 사람과 ID 교환·벽 모서리의 겉보기 이동은 남은 한계다.

### B. 전방 측방 회피 후보 선택

`hospital_avoidance.py`는 ROS 없이 동작한다. MPPI 직선 구간의 기준 경로를 진행거리 s와 횡방향 d로 표현한다.

- 좌우 1.5/1.8/2.0m offset 후보 생성.
- 3/4/5m 구간에서 quintic 곡선으로 이동, 유지 구간 후 4m에 걸쳐 전방 합류.
- 선택 lane 중심선 기준 ±2.8m 안에 **차체까지** 들어와야 한다. 다른 lane으로 우회하지 않는다. 실제 지도와 관측 costmap도 추가 검사한다.
- 후진 진행, 유턴, 고리, 급격한 접선 변화, 반경 1.2m 미만의 후보를 거부한다. 이 반경은 후보 경로 제약이며 DiffDrive 설정을 Ackermann으로 바꾸지 않았다.
- 같은 시각의 사람 예측과 비대칭 전체 차체 사이 간격을 3초간 검사한다. 가속 0.5, 감속 0.6, 각가속 1.5, 반응 지연 0.2초의 근사 추종 모델을 사용한다.
- Collision Monitor가 동작할 때 시간상 움직임이 달라지는 영향을 줄이기 위해 정상 상한 0.6m/s와 40% 속도의 rollout을 모두 검사한다. 모든 중간 속도와 실제 MPPI 궤적을 완전히 검증한 것은 아니다.
- 표면 간격 목표 0.6m, 후보 배제 임계 0.4m. 안전 후보가 있으면 회피를 먼저 선택한다.
- 회피 방향을 유지하는 선호를 주되 그쪽 후보가 불가능하면 다른 쪽도 검토한다.

이것은 MPPI 내부 critic을 새로 구현한 것이 아니라 **FollowPath에 줄 reference 후보를 선택하는 감독 로직**이다. 실제 MPPI 명령은 아래 출력 guard가 따로 검사한다. 후보의 기하/짧은 예측이 통과했다고 전체 회피가 성공했다고 말하면 안 된다.

### C. 가까운 점으로 돌아가지 않는 합류

현재 위치에 가장 가까운 XY 점을 합류점으로 사용하지 않는다. 이미 지난 기준 경로 인덱스는 되돌리지 않는다. 앞쪽 S자 경로 끝과 그 이후의 reference만 이어 붙인다.

정적 차단 위치보다 차체 후방 길이·여유·추가 0.5m를 지난 뒤 합류하도록 offset 유지 구간을 연장한다. 동적 track이 후미/측면에 남아 있으면 합류 단계에서 다시 회피 후보를 검토하거나 대기한다. 조건이 나쁘면 무조건 회전해서 복귀하지 않는다.

정적 Planner fallback도 유지하되, 이제 현재 위치의 약 10m 전방 합류점을 요청하고 반환 경로를 동일한 차선·진행·곡률·차체 검사로 검증한다. 검사를 통과하지 못하는 거친 NavFn 경로는 거부한다. 이는 선택 차선 안에서 항상 대안을 찾아주는 constrained planner는 아니다. 안전한 경로가 있어도 후보 집합/NavFn 결과의 한계로 거부될 수 있다.

### D. 실행 명령 재검사

병원 launch에 `hospital_velocity_guard`를 추가했다.

```text
controller → velocity_smoother → /cmd_vel_smoothed
          → hospital_velocity_guard → /cmd_vel_human_checked
          → collision_monitor → /cmd_vel → 기존 로봇
```

Collision Monitor의 입력만 새 중간 토픽으로 연결했다. 최종 `/cmd_vel`, 센서 입력, 로봇 구독은 유지한다. guard를 실행하지 않은 채 새 params만 사용하면 정상 체인이 완성되지 않는다. 새 병원 launch와 params는 함께 사용해야 한다.

- measured odom 속도부터 후보 명령까지 가감속을 적용해 차체와 사람의 미래 간격을 검사한다.
- 기본 1.5초, 필요하면 현재 속도의 제동시간+반응시간까지 늘린다. 현재 순간 명령을 3초 내내 유지한다고 가정해 계획된 회전을 과도하게 막지 않도록 경로 선택(3초)보다 짧게 설정했다.
- 안전 여유가 있으면 입력 명령 유지, 부족하면 비율 감속 후보/정지를 검사한다. 회피 조향을 임의로 생성하지 않는다.
- 감속/양보 후보는 1.0m 목표 간격을 적용한다. 0.4m 이상이어도 1.0m에 못 미치는 정지는 `YIELD_MARGIN_SHORTFALL`, 0.6m 목표보다 가까운 전진은 `PASS_MARGIN_SHORTFALL`로 표시한다. 수 cm 통과를 충분한 양보로 표현하지 않는다.
- 허용되는 명령이 없으면 0 명령과 `NO_SAFE_COMMAND`를 낸다. 상대가 계속 접근하면 정지해도 충돌할 수 있으므로 이를 안전 성공 상태로 해석하면 안 된다.
- 명령이 0.5초 이상 오래되거나, track/odom/TF가 유효하지 않으면 정지한다. 신선한 빈 track 메시지와 메시지 누락을 구분한다.
- odom twist의 child frame이 base_link와 다르면 회전과 레버암을 보정한다. 임의로 모든 twist를 base_link 값이라고 간주하지 않는다.

이 guard는 선형 사람 예측·기하 근사에 기반한 추가 방어이며 안전 인증 장치가 아니다. 0.1초 샘플 사이의 연속 충돌, 미관측 사람, 모델 오차, 최종 Collision Monitor에 의한 추가 감속까지 완전히 보증하지 않는다.

### E. 양보/재출발/표시

- `TRACKING`, `AVOIDING`, `YIELDING`, `REJOINING`, `WAITING_DATA`를 구분한다.
- 안전한 후보를 찾지 못하면 대기하고, 위험이 해소된 상태가 0.5초 유지되면 진행 중이던 경로의 앞쪽으로 재개한다.
- 단순 6초 정체로 Planner를 부르는 로직은 제거했다. 신선한 관측과 같은 위치의 차단 1.5초 지속이 있어야 Planner fallback을 고려한다. 계속 움직이는 track 근처에서는 억제하고, 움직이다 멈춘 track이 1.5초간 정지한 뒤에는 경로 차단으로 취급할 수 있다.
- 대기 15초 초과는 `FAILED/yield_budget_exceeded_not_proof_of_lane_blockage`. 관제용 `BLOCKED` 판정으로 둔갑시키지 않는다. 이는 영구 정체 방지를 위한 작업 예산이지 통행 불가능의 증거가 아니다.
- `/plan`: 실제 실행 중인 경로의 남은 부분. `/hospital/reference_plan`: 원래 기준 경로. 우회 실행 중 원경로를 실제 경로처럼 다시 표시하던 문제를 제거했다.
- `/hospital/mission_state`, `/hospital/human_guard_state`에 상태 전이를 발행하고 로그를 남긴다. 상시 대용량 recorder를 다시 넣지는 않았다.

### F. 후속 센서·우회 경로 흐름 보강

```text
/front_3d_lidar/lidar_points → pointcloud_to_laserscan → /scan
  ├─ local/global costmap: 현재 점유·Planner 비용
  └─ scan_self_filter → /scan_body_filtered
       ├─ Collision Monitor: 실제 근접 제동
       └─ moving_obstacle_predictor → /hospital/tracked_obstacles: 회피 예측·guard
                                    → /hospital/predicted_obstacles: MPPI soft cost
/chassis/odom + TF → 회피 감독·velocity guard
```

현재 병원 주행 회피에 카메라 인식 입력은 없다. 별도 프로세스의 guard와 Collision Monitor가 최종 명령을 계속 검사하므로, mission이 잠시 경로를 계산해도 마지막 명령 검사는 계속된다. 단, 필수 scan/odom/TF가 끊기면 정지한다.

- 사람이든 콘이든 원래 FollowPath 전방의 점유 차단이 보이면 먼저 측방 offset 후보를 검사한다. Planner 호출만 1.5초의 위치 지속 조건을 기다린다. 후보의 유지 구간은 첫 차단점을 **차체 꼬리+여유**만큼 지나서 합류하도록 잡는다.
- 이미 움직이는 것으로 추적했던 사람이 멈춰 서면 1.5초간 정지 관측 후 경로 차단 후보가 된다. 같은 위치의 점유가 추가로 1.5초 지속되고 측방 후보가 없으면 Planner도 허용한다. 이것은 사람/콘 식별이 아니라 통행 가능한 우회 경로의 판단이다.
- Planner의 NavFn 경로는 설정된 `simple_smoother`에 한 번 전달한다. 서버가 없거나 smoothing이 실패하면 원본을 사용하되, 두 경로 모두 촘촘한 점·진행 방향으로 변환한 다음 전진 진행·곡률·선택 차선·지도/관측 costmap·track 간격을 통과해야 FollowPath에 보낸다. NavFn이 0.25m 허용오차 안에서 끝나면 앞쪽 기준 차선까지 짧은 연결을 추가하고 똑같이 검사한다. 검사 실패 사유는 `[DETOUR]` 로그로 남긴다.
- 후보 경로 검사는 지도와 local costmap에 대한 좌표 변환을 각 후보당 한 번만 수행한다. 최종 전송 전에 관측을 갱신하고 로봇이 0.2m 이상 이동했거나 비용/예측 간격이 바뀌었으면 그 후보를 보내지 않는다. mission의 고빈도 관측 구독은 depth 1로 최신 메시지를 우선한다. `/plan`의 남은 경로 반복 표시는 최대 1Hz다.
- 후보 평가가 0.25초를 넘으면 `[AVOIDANCE]`에 소요 시간을 남기고, Planner+Smoother 시간은 `[DETOUR]`에 남긴다. 이 로그로 실제 GPU PC에서 계산 지연과 센서 누락을 분리해 볼 수 있다.
- scan self-filter 또는 tracker에서 TF 부족으로 scan을 버리면 2초 간격으로 원인을 기록한다. 실제 센서/TF가 0.6초 이상 없으면 기존 velocity guard의 정지 동작은 유지된다. 이는 불충분한 관측에서 우회를 강행하지 않기 위한 조건이다.

## 차체와 정류장 제어

- footprint: 앞 0.48m, 뒤 1.38m, 폭 1.0m. 실제 USD collision mesh/질량/휠 제약은 이번 노트북에서 재측정하지 못했다.
- DWB `FollowPath`, `FollowPathDock`의 `BaseObstacle`을 `ObstacleFootprint`로 교체했다.
- 최종 정류장 DWB의 전진 상한과 max_speed_xy를 0.55→**0.5**로 맞췄다. 각속도 0.6은 유지했다. 추가 극저속 stage는 없다.
- 최종 goal checker를 `StoppedGoalChecker`로 변경. 위치 0.05m, yaw 0.10rad, 정지 판정 직선 0.03m/s·회전 0.05rad/s.
- action 성공 후에도 fresh pose/odom으로 위치·방향·정지 상태가 0.3초 유지되는지 확인한다. 확인하지 못하면 성공 처리하지 않는다. 다음 단계의 Pick & Place 호출은 추가하지 않았다.

## GPU PC AI가 알아야 할 값: 적용 vs 미적용 후보

| 항목 | 실제 적용 | 판단/다음 조정 근거 |
|---|---:|---|
| 도착 DWB max_vel_x/max_speed_xy | 0.5 | 사용자 지정. 0.5가 항상 임계 최댓값이라는 증거는 없음 |
| 도착 DWB max_vel_theta | 0.6 유지 | 이번에 극저속/극저각속도로 바꾸지 않음 |
| MPPI vx_max/vx_min | 0.6 / 0 유지 | 전진 상한 유지, 후진 금지 |
| MPPI wz_max | 1.2→0.9 | 긴 꼬리 회전 성분 감소, 조기 회피로 시간 확보 |
| smoother 각속도 상한 | ±0.9 | controller와 일치 |
| smoother 선형 accel/decel | +0.5 / −0.6 유지 | 새 외부 예측도 이 값 사용 |
| smoother angular accel/decel | ±1.5 유지 | 새 외부 예측도 이 값 사용 |
| MPPI ax_max/ax_min/az_max | 30 / −30 / 20 유지 | 과거 작은 값에서 속도 정체 보고. 이 불일치는 미해결 |
| MPPI 가속도 정합화 후보 | **0.5 / −0.6 / 1.5 미적용** | 실제 odom과 출력 램프, MPPI 최적화/시퀀스 처리 확인 후 검토 |
| MPPI wz_std | 0.2 유지 | 분산 증가만으로 회피를 만들지 않음 |
| MPPI horizon | 4.5초 유지 | 경로 선택기 예측 3초와 역할이 다름 |
| MPPI CostCritic near_goal_distance | 1.0→0.0 | MPPI 구간 끝에서도 낮은 장애물 비용 선호를 생략하지 않음 |
| 일반 inflation | radius 1.6, scaling 3.0 유지 | 벽/테이블 접근까지 함께 방해하지 않음 |
| SlowdownFront | 3.5m, ratio 0.4 유지 | 아래 설명 참조. 먼저 끄지 않음 |
| 추적 거리 | 6→8m | 기존 스캔을 일찍 활용 |
| 사람 간격 | 최소 0.4 / 선호 0.6 / 양보 목표 1.0m | 모두 차체 표면 기준. SafetySettings에서 공유 |

### 중요한 미해결 사항

`SlowdownFront`는 전진과 회전을 함께 40%로 줄인다. 각속도만 높여도 이 영역에 들어오면 둔해질 수 있다. 이번에는 선제 경로 선택과 시간 모델을 추가하되 이 방어를 먼저 해제하지 않았다. 따라서 "전방 감속을 TTC 방식으로 완전히 대체했다"고 설명하면 안 된다.

또한 기존 MPPI 가속도 30/−30/20은 실제 smoother와 불일치한다. Jazzy 공식 구현은 첫 제어값뿐 아니라 예측 단계 전체에 이 제한을 적용한다. 이전 YAML의 첫 단계만 제한한다는 주석을 수정했다. 후보값을 한 번에 낮춘 뒤 속도 정체가 재발하면 사람 비용이나 목표 오차를 다시 무작정 바꾸지 말고, 명령→smoother→guard→Collision Monitor→odom 중 어디서 달라지는지 구분해야 한다.

관측이 늦거나 궤적이 모두 불가능한 상황에서는 여전히 대기가 많을 수 있다. 이를 없애려고 minimum_gap을 즉시 줄이지 않는다. 유효 관측 거리, 선택된 경로의 실제 횡이동, 최종 회전 명령이 먼저 확인 대상이다.

## 로컬 검증 결과

**CPU-only 35개 통과**: 최초 33개에 정적 차단 및 멈춰 선 사람의 Planner 수락 사례 2개를 기존 stage 시험에 추가했다. 후속 변경 Python 6개 AST와 `git diff --check`도 통과했다. Nav2/Isaac 런타임 동작은 아직 검증하지 않았다.

- 기존 Tracker 4개.
- 신규 차체/회전 쓸림, 시간차 횡단, 유한 제동거리, 명령 판단, 양방향 전방 후보, 후진/유턴 거부, 조기/늦은 정면 접근, 회피 방향 유지/변경, 정지한 track 유지, costmap 내부 점유/unknown, YAML 계약.
- 실제 stage 함수의 action/관측 I/O를 대역으로 바꾼 상태 시험: 정상 경로 유지, 회피 1회 preemption, 실제 실행 경로 표시, 사람 대기 중 6초 Planner 억제, 관측 누락 시 초기 goal 금지. 실제 ROS action 통신을 시험한 것은 아니다.
- 실제 소스에서 추출한 upper/lower 경로의 지도+전체 footprint와 연결부 검사, 두 lane 중간의 offset 후보가 지도에 들어오는지 검사, Nav2와 외부 모델 footprint 일치.
- Python AST 10개, YAML, package.xml 검사 통과. `git diff --check` 통과.

구체적인 합성 사례: 로봇 0.6m/s, 정면 물체 1.1m/s, 반경 0.4m, 초기 간격 6.8m에서 원경로의 3초 예측 최소 여유 약 0.52m, 선택 offset 후보는 약 0.68m였다. 동일 모델의 5m 시작에서는 후보를 거부했다. 이는 ground-truth를 제어에 넣은 Isaac 실험이 아니라, 알려진 입력으로 계산한 단위시험이다. 실제로 6.8m면 반드시 회피한다는 보증은 아니다.

아직 하지 못한 검증:

- 실제 설치된 Nav2 버전의 StoppedGoalChecker/ObstacleFootprint 플러그인 로딩과 전체 ROS 통신.
- Isaac의 실제 개조 로봇 동역학·충돌 형상, 센서 가림/track 정확도.
- MPPI가 제안 reference를 따라 실제로 충분한 측방 이동을 만드는지.
- guard와 기존 Collision Monitor가 겹쳐 불필요한 정체를 만드는지.
- 정류장 0.5m/s 상한에서 5cm/0.10rad 및 실제 정지 조건 충족.

## 변경 파일 안내

| 파일 | 역할 |
|---|---|
| `nav_to_goal/hospital_avoidance.py` | 순수 기하, 전방 후보, 시간별 사람 간격, 명령 검사 |
| `nav_to_goal/hospital_costmap.py` | 지도 셀과 전체 polygon 교차 검사 |
| `nav_to_goal/hospital_safety.py` | ROS 관측 신선도/좌표계/track 변환 |
| `nav_to_goal/hospital_stage_runner.py` | 실제 FollowPath 상태 전환, 후보 선택, 전방 합류, 정지 확인 |
| `nav_to_goal/hospital_velocity_guard.py` | smoother 출력의 사람 접근 검사 |
| `nav_to_goal/hospital_mission.py` | 기존 경로 유지, stage runner 연결, 오래된 costmap 무시 |
| `nav_to_goal/obstacle_tracking.py` | ID, 멈춘 이동체 유지, 현재 상태 snapshot |
| `nav_to_goal/moving_obstacle_predictor.py` | 기존 soft cloud 유지 + 시간정보 track 스트림, 8m 추적 |
| `hospital_navigation.launch.py` | guard 실행 |
| `hospital_navigation_params.yaml` | 전체 차체 DWB, 도착 0.5, 각속도 0.9, guard 연결 |
| `nav_to_goal/setup.py`, `package.xml` | entry point, std_msgs/numpy 의존성 |
| `test_hospital_avoidance.py`, `test_hospital_stage_policy.py`, `test_hospital_route_geometry_cpu.py` | 신규 CPU-only 검증 |

파일 표의 `nav_to_goal/`은 `src/nova_carter/nav_to_goal/nav_to_goal/` 기준이며 launch/params는 `src/nova_carter/carter_navigation/` 아래다.

## 다음 AI에게 전달할 요청

> 이 인수인계서를 먼저 읽고 실제 Git diff를 확인해라. 기존 개조 Carter와 고정 upper/lower FollowPath를 유지한다. 도착 상한 0.5m/s는 사용자가 선택했으며 0.15m/s 극저속 구간으로 바꾸지 마라. CPU 검증 통과를 실제 회피 성공이라고 보고하지 마라. 새 병원 launch/params/entry point가 함께 적용되는지 확인하고, 조기 정면 회피·횡단 양보·전방 합류·정류장 도착의 실제 동작과 명령 흐름을 확인해라. 관제 BLOCKED, Pick & Place와 실제 테이블 도킹은 이번 범위 밖이다. SlowdownFront와 MPPI 가속도 불일치는 남아 있으므로 표의 적용값과 미적용 후보값을 혼동하지 마라. 사람을 안 만난 완주를 회피 성공으로 계산하지 말고, 미관측 급진입과 관측 가능한 정상 조우를 구분해 원인을 설명해라.
