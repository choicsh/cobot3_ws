# Hospital AMR 프로젝트 — 다음 AI를 위한 인수인계

> 2026-09-23 후속 구현: [회피 강화 인수인계](HOSPITAL_AMR_AVOIDANCE_HANDOFF_2026-09-23.md)를 먼저 읽는다. 아래는 이전 실주행 기록이며, 새 회피 코드의 실주행 성공을 의미하지 않는다.

> 최신 사용자 정정: 상세 기록은 개발 진단용으로만 사용한다. hospital launch의 기록 옵션/노드, 시뮬레이션 루프의 사람 audit, nav_to_goal 기록기 entry point는 제거했다. 관련 도구는 저장소 밖 `/home/rokey/.local/share/hospital-amr-diagnostics/`로 이동했다. 필요한 순간에만 별도 실행하고 종료한다. 아래 과거의 “항상 자동 기록” 설명은 더 이상 적용하지 않는다. 기존 로그는 보존한다.

작성: 2026-09-22, 로컬 작업 상태 기준. 작업 디렉터리: `/home/rokey/cobot3_ws`.

### 21:52 화면 지연 완화 수정

- `run_hospital_sim.py`: 물리 60 Hz / 렌더링 30 Hz를 명시하고 앱 루프를 30 Hz로 제한, viewport AA를 FXAA로 변경했다. 사람/센서/회피 기능은 제거하지 않았다. 렌더 기반 센서의 발행 빈도는 영향을 받을 수 있으므로 주행 회귀 검증이 필요하다.
- `robot_nova.usd`: `/Flattened_Prototype_*`의 `material:binding` 9개가 존재하지 않는 `/World/robot/...` 및 참조 범위 밖을 가리켜 USD에서 무시되고 있었다. 이 무효 관계만 제거했다. 형상·물리·유효 재질은 변경하지 않았다. 이전 실행 Kit 로그 약 667 MiB → 새 실행 초기 약 4 MiB, 같은 참조 경고 0건. 시작 시 경고 폭주는 확인했으나 지속 렉의 유일 원인으로 단정하지 않는다.
- 일시적인 외부 ROS 구독 측정(파일 기록 없음): Isaac 단독 12초 동안 clock 29.78 Hz, 실시간 배율 0.993; lidar 6.43 Hz, camera 16.89 Hz 수신. Nav2+RViz 기동 후 clock 29.77 Hz, 실시간 배율 0.992, 양쪽 lifecycle active 확인. 구독 빈도는 센서 설정 주기와 동일하다고 단정하지 않는다.
- 자동 주행 명령은 보내지 않았다. 실제 로봇 이동 중 화면 체감 및 회피 회귀 검증은 아직 남아 있다. 진단 구독 프로세스는 종료했고 Isaac/Nav2/RViz는 확인용으로 실행 중이다.

이 문서는 사용자의 요구, 지금까지의 구현, 실제 시험 결과, 미해결 문제를 한 파일로 전달하기 위한 문서다. **소스 구현 / 빌드 성공 / 실제 성능 검증을 반드시 구분해서 읽어야 한다.**

## 0. 최신 상태 덮어쓰기 — 후속 세션 반영

이 절은 아래 본문의 직선 왕복 사람 구성과 “최신 재주행 전”이라는 오래된 설명보다 우선한다. 사용자 제공 추가 문서 `/home/rokey/Downloads/HOSPITAL_AMR_HANDOFF_ADDENDUM_2026-09-22_session2.md`를 로컬 소스·로그와 대조했고, 그 뒤 보강 및 재주행까지 했다. 변경은 여전히 미커밋이며 푸시하지 않았다.

### 현재 환경과 다른 AI가 추가한 내용

- 사람 6명은 고정 직선 왕복이 아니라 각 upper/lower 구역의 무작위 GoTo 10곳을 NavMesh로 배회한다. 사람끼리/로봇을 피하는 dynamic avoidance는 계속 꺼져 있다.
- `hospital_people.usd` lower lane의 `(-12.0,-1.45)`에 충돌·라이다 인식 가능한 콘이 있다.
- MPPI `CostCritic 25`, `PathAlign 4`, `wz_max 1.2`; 전방 3.5m×±1.1m `SlowdownFront` 40%; 도착은 `FollowPathDock`을 사용한다.
- MPPI 정체 6초 뒤 NavFn `GridBased` 우회 경로를 1회 요청하는 복구가 추가됐다. 선택 lane을 바꾸는 기능은 아니다.
- 실주행 `log/hospital_people/20260922_204128_423699/people.jsonl`의 첫 lower 시험에서 우회는 성공했으나, 구간 경계 yaw 수렴으로 18초 정체했고 콘은 감속부터 통과까지 약 24초 걸렸다. 사람 접촉 2건은 정지한 로봇의 측면→후방 진입이었다.

### 추가 문서에서 바로잡은 좌표계 해석

`hospital_sensor_recorder`의 `prediction_raw`는 메시지 원본인 **odom**, 변환된 `prediction`은 **map** 좌표다. 예를 들어 sim t≈51초의 raw `(3.57,-9.24)`를 당시 map←odom 변환하면 기록된 map `(2.76,4.43)`과 일치한다. 10m 이상 변환 오류가 아니라 서로 다른 frame을 직접 비교한 결과다. 혼동 방지를 위해 이후 변환 레코드에는 `frame: "map"`을 명시한다. 분석 시 둘 중 하나만 무조건 옳다고 간주하지 말고 frame을 맞춰 비교한다.

### 이번 보강

- 로봇 물리 질량, 사람 collider, 후방 센서를 추가하지 않았다. 실제 센서 하드웨어를 달면 실제 로봇 질량·전력·연산 부담이 늘 수 있지만 이번 소프트웨어 보강은 질량을 바꾸지 않는다.
- MPPI 구간의 local raw costmap에서 남은 reference 전방 0.8~5.0m를 검사한다. lethal/inscribed(253/254, unknown 255 제외)가 같은 map 위치에 1.5초 지속되고 움직이는 예측 track과 겹치지 않을 때만 **정적 차단 조기 detour**를 요청한다.
- 움직이는 사람 예측과 겹치면 planner snapshot을 호출하지 않고 MPPI/SlowdownFront에 맡긴다. 기존 “0.15m 미만 진행 6초”는 costmap/분류 실패의 백업으로 유지한다.
- `transit_goal_checker.yaw_goal_tolerance`를 0.3→0.8rad로 완화했다. 최종 정류장의 0.10rad(5.7°)는 그대로다.
- `FollowPathMPPI.wz_max=1.2`가 velocity smoother 0.9에서 잘리고 있던 불일치를 고쳐 smoother의 각속도 상한만 ±1.2로 맞췄다. DWB 자체 상한은 0.9, 도착 전용은 0.6이라 기존 정류장 제어에는 적용되지 않는다.
- recorder의 변환 scan/prediction/costmap 행에 `frame: "map"`을 추가했다.

### 이번 실주행 결과

동일하게 계속 재생 중이던 Isaac 세션에서 새 Nav2를 빌드·재시작해 양방향을 주행했다. 센서 로그는 `log/hospital_sensors/20260922_210547_933331.jsonl`이다.

| 경로 | sim 구간 | 결과 | 사람 간격 |
|---|---:|---|---|
| upper | 952.850–1056.217 | 3구간 SUCCEEDED | 최소 +0.660m, 겹침 0 |
| lower | 1071.250–1190.083 | 3구간 SUCCEEDED | 최소 -0.341m, 겹침 16샘플/1episode |

- upper 출발 DWB는 12.43초. lower 출발 DWB는 24.52초로, 이전 약 39초보다 14.5초 짧았다. 두 경로 모두 이전의 18초 제자리 yaw 대기는 재현되지 않았다.
- lower 콘 접근: 최종 출력 속도는 t=1114.15에 0.233m/s로 감속(콘 중심 5.73m). t=1115.833에 reference 전방 4.6m 정적 차단 1.5초 조건으로 detour 발동(로봇-콘 중심 5.29m). 기존처럼 사실상 정지한 뒤 6초를 기다리지 않았다.
- detour는 888 poses, 콘 중심 x를 지난 시각 t≈1125.8. 첫 감속부터 콘 중심 통과까지 약 11.65초로 이전 24초보다 줄었다. 정의가 완전히 같은 “콘 전체 이탈” 시간이 아니므로 2배 개선이라고 단정하지 않는다.
- detour 최대 reference 횡이탈은 약 1.40m, 32.5m MPPI stage는 SUCCEEDED.
- lower의 유일한 접촉은 stage 전환 직후 `Lower_Cross_1`이 로봇 **후방**(접근 bearing 약 ±175°→-145°)으로 걸어 들어온 사건이다. 사용자가 현재 우선순위에서 제외한 사각/관통 유형이며, 안전 기록에서는 삭제하지 않는다.
- 관측된 측면 최근접: upper 두 사람 +0.660/+0.677m(bearing 약 133°/97°), lower Cross_2 +1.297m(bearing 약 89°). 접촉은 없었다.
- 이번 두 주행에는 가까운 **정면 조우가 없었다.** 따라서 조기 정적 우회와 구간 회전 개선은 확인됐지만 정면 사람 회피가 최종 검증됐다고 말하면 안 된다. 사용자의 현재 우선순위는 정면 최대 대응, 측면 실질 대응이며 후방 완전 보장은 범위 밖이다.
- launch와 함께 시작된 recorder는 정상적으로 로그를 남기고 launch 종료 시 같이 종료됐다. 과거 PID 153507 수동 중복 recorder도 종료했다. 사용자 Isaac 프로세스는 종료하지 않았다.

### 검증

- `colcon build --packages-select hospital_dynamic_layer nav_to_goal carter_navigation --symlink-install`: 성공.
- 경로·tracker·audit pytest: 9개 통과. 새 테스트는 raw Costmap에서 unknown 255를 차단으로 오인하지 않고 254 정적 차단을 검출하는지 포함한다.
- invalid route 실행으로 설치된 entry point/import 확인, YAML 및 controller/smoother 각속도 상한 일치 확인, `git diff --check` 통과.
- 다음 필수 시험은 정면 사람이 실제로 접근하는 위상 3개 이상이다. `CostCritic 25`와 `SlowdownFront`의 기여도는 아직 분리하지 않았고, 시간별 사람 예측 대 후보 footprint의 hard safety critic도 아직 없다.

## 1. 가장 중요한 결론

1. 병원 정류장 간 독립 upper/lower 순환 경로, 사람 6명, MPPI/DWB 분담, 평가·센서 기록 기반이 있다.
2. **사람 회피는 아직 해결되지 않았다.** 최신 사용자 시험에서 두 경로 모두 도착했지만 사람과 몸체 영역이 겹쳤다. Nav2 `SUCCEEDED`는 안전한 회피 성공을 뜻하지 않는다.
3. 마지막으로 수정한 내용은 **MPPI를 복도 직선에 한정, 모든 원호는 DWB, 정류장 각도 허용 완화, 횡단 보행 범위 확대, launch 자동 기록**이다. 단위 시험과 빌드는 통과했으나 이 변경 묶음의 Isaac 재주행은 아직 하지 않았다.
4. 예측 soft-cost를 추가했다는 사실만으로 human-aware planning이 구현됐다고 말하면 안 된다. 현재는 시간별 사람 위치와 로봇 후보 궤적을 직접 비교하는 안전 판정이 없다.
5. 사용자 요구는 “더 빨리 달려서 사람을 피하기”가 아니라 **정면·측면 사람에 대해 눈에 보이는 안전한 회피/양보/재출발/경로 복귀**다. 정지만 하는 결과도 능동 회피 성과로 인정하지 않는다.

## 2. 사용자 의도와 지켜야 할 제약

### 2.1 환경·정류장·경로

- 사용자 배치가 기준이다. 기존 그림의 테이블 앞 dock/pre-dock/exit 좌표를 다시 사용하지 않는다.
- 환경의 실제 이름은 `hospital_integration.usd`다. 초기 자료에 있던 `integration_hospital.usd`와 혼동하지 않는다. 지도 파일은 별도로 `integration_hospital.yaml/png`라는 이름이다.
- 현재 목표점은 서쪽 specimen `(-36, 8)`, 동쪽 lab `(12, 8)`이다.
- 도킹·팔 작업·테이블 접근은 사용자가 나중에 따로 맞춘다. **현재 미션에 도킹을 끼워 넣지 않는다.**
- 두 차선은 수학적으로 같은 경로를 뒤집는 것이 아니라 독립된 물리 경로다.
  - `specimen_to_lab` → `lane_upper` → 서쪽에서 동쪽.
  - `lab_to_specimen` → `lane_lower` → 동쪽에서 서쪽.
- 가운데 회색 벽/구조물의 위·아래로 각각 지나 하나의 순환 고리를 이룬다. 양쪽 경로는 같은 정류장 점을 공유하지만 그림처럼 불필요하게 교차하지 않는다.
- 막혀도 다른 lane으로 자동 전환하지 않는다. 선택된 lane 주변의 국소 회피 및 원래 경로 복귀를 목표로 한다.
- 다로봇 대기열은 장기 의도다. 지금 시험은 로봇 1대이며 동일 정류장 점을 사용한다. 대기열 제어는 아직 구현하지 않았다.

### 2.2 자세와 시작 상태

사용자는 과거 180°를 요청했지만 이후 “다음 레인으로 자연스럽게 이어지는 방향”으로 의도를 구체화했다. **옛 180° 요구만 보고 되돌리지 않는다.**

| 위치 | 좌표 | 현재 목표 yaw | 다음 출발 |
|---|---|---|---|
| lab / 동쪽 | `(12, 8)` | `-90°` 남향 | lower로 진행 |
| specimen / 서쪽 | `(-36, 8)` | `+90°` 북향 | upper로 진행 |

- 새 시뮬레이션은 lab에서 시작한다. 첫 미션은 `lab_to_specimen`이다.
- 목표 yaw는 마지막 직선 및 다음 경로의 시작 접선과 같다. 도착 후 별도로 90°를 더 돌리려는 설계가 아니다.
- `isaacpjt/assets/start_pose.json`의 확인값: x≈12.000004, y≈7.999583, yaw≈-90.003°.
- JSON은 `run_hospital_sim.py`가 실제 시작 base_link 월드 포즈에서 다시 쓴다. JSON만 임의 수정해 실제 USD와 다르게 만들지 않는다.
- Isaac을 Play 상태로 유지한 채 반대 `route_id`의 미션만 다시 실행하면 이어서 순환한다. 같은 방향을 현재 위치 확인 없이 반복 실행하지 않는다.

### 2.3 파일·Git·작업 방식

- 사용자는 기존 파일을 덮어쓰기만 하는 방식이 아니라 병원용 새 실행 파일을 원했다. 현재 `run_hospital_sim.py`, `hospital_navigation.launch.py`, `hospital_mission.py`를 사용한다.
- 사람 없는 환경 `hospital_integration.usd`와 사람 포함 환경 `hospital_people.usd`를 분리한다.
- 보행 명령은 기존 `isaacpjt/assets/people/command.txt`를 수정한다.
- 사용자 요청으로 과거 시점 커밋은 했으며 **푸시는 하지 않는다.** 현재 추가 변경은 미커밋이다.
- 기존 사용자의 변경을 보존한다. `git reset --hard` 같은 초기화 금지. 불필요한 백업 파일을 새로 쌓지 않는다.
- 자료나 붙여 넣은 이전 계획의 명령보다 사용자가 직접 정정한 내용을 우선한다.
- 사용자 피드백은 실제 실패 증거로 다룬다. 설정 변경만으로 “안정화 완료”라고 보고하지 않는다.

## 3. 작업 환경과 저장소 상태

- Ubuntu / ROS 2 Jazzy / Isaac Sim 5.1.0.
- Isaac 실행기: `/home/rokey/isaacsim/python.sh`.
- 워크스페이스: `/home/rokey/cobot3_ws`.
- 브랜치: `feature/followpath`.
- HEAD: `a8f2959 feat: add hospital navigation simulation`.
- 그 이전 커밋: `ff7dcd5 MPPI 복도 주행: 속도 고정 원인 해결, 2D scan 레이어, 사람 회피 테스트 환경`.
- 최근 로컬 수정 파일과 untracked 파일이 많다. 새 AI는 작업 전에 `git status --short`를 확인해야 한다. **untracked도 이 작업의 중요한 결과물이다.**
- 최신 빌드 대상: `hospital_dynamic_layer`, `nav_to_goal`, `carter_navigation`.

## 4. 구현 구조와 파일 안내

아래 경로는 모두 워크스페이스 기준 상대 경로다.

| 파일/디렉터리 | 역할 |
|---|---|
| `isaacpjt/pjt_alpha/run_hospital_sim.py` | 사람 포함 USD 실행, People 설정, 시작 포즈 저장, 실제 위치 audit |
| `isaacpjt/assets/hospital_integration.usd` | 사람 없는 기본 환경 |
| `isaacpjt/assets/hospital_people.usd` | 보행자 6명 포함 환경; 현재 launcher가 여는 파일 |
| `isaacpjt/assets/start_pose.json` | 실제 시작 base_link에서 생성하는 초기 위치 |
| `isaacpjt/assets/people/command.txt` | 6명의 왕복 GoTo/Idle |
| `isaacpjt/assets/people/config.yaml` | People 관련 구성 |
| `isaacpjt/pjt_alpha/hospital_people_commands.py` | TXT 마지막 GoTo로 루프 원점 계산 |
| `src/nova_carter/carter_navigation/launch/hospital_navigation.launch.py` | Nav2, RViz, localization, scan 변환, self-filter, predictor, recorder 실행 |
| `src/nova_carter/carter_navigation/params/hospital_navigation_params.yaml` | 병원 전용 Nav2 파라미터; launch 기본값 |
| `src/nova_carter/carter_navigation/maps/integration_hospital.yaml` | 현재 정적 지도 |
| `src/nova_carter/nav_to_goal/nav_to_goal/hospital_mission.py` | 두 독립 lane의 경로와 DWB→MPPI→DWB 실행 |
| `src/nova_carter/nav_to_goal/nav_to_goal/scan_self_filter.py` | 로봇 몸체 내부 스캔점 제거, monitor/predictor 입력 |
| `src/nova_carter/nav_to_goal/nav_to_goal/obstacle_tracking.py` | scan cluster, 움직임 추적, 예측 계산 |
| `src/nova_carter/nav_to_goal/nav_to_goal/moving_obstacle_predictor.py` | ROS scan→odom 보정→예측 cloud |
| `src/nova_carter/hospital_dynamic_layer/` | C++ Nav2 costmap 예측 soft-cost 레이어 |
| `src/nova_carter/nav_to_goal/nav_to_goal/hospital_sensor_recorder.py` | ROS 센서·비용장·속도·로그 자동 기록 |
| `isaacpjt/pjt_alpha/record_hospital_sensors.py` | 위 기록기의 호환 실행 래퍼 |
| `isaacpjt/pjt_alpha/hospital_people_audit.py` | 실제 로봇·사람 위치, 몸체 간격 기록 |
| `isaacpjt/pjt_alpha/report_hospital_people.py` | audit 구간 요약 |
| `isaacpjt/pjt_alpha/run_hospital_trial.py` | 미션 실행·출력·audit를 묶는 선택적 평가 도구 |
| `docs/hospital_people_stage1_review.md` | 상세 중간 분석 보고서; 본 인수인계에 핵심을 모두 포함함 |

### 4.1 최신 컨트롤러 분담

경로 전체는 좌표·접선이 연속인 line/arc 배열이다. 마지막 수정은 경로 기하를 바꾸지 않고 구간 배정만 바꿨다.

| lane | DWB 출발 구간 | MPPI 구간 | DWB 도착 구간 |
|---|---|---|---|
| upper | 정류장부터 첫 R=4 원호까지 | `(-32,14.9) → (8,14.9)` | 출구 R=4 원호부터 lab까지 |
| lower | 정류장부터 중앙 진입 R=3 원호까지 | `(4,-1.5) → (-28.55,-1.5)` | 출구 R=3 원호부터 specimen까지 |

`split_route()`의 최신 슬라이스:

```python
lane_upper: route[:2], route[2:3], route[3:]
lane_lower: route[:6], route[6:7], route[7:]
```

- MPPI의 **기준 경로**가 직선이라는 뜻이다. 회피할 때 국소적으로 곡선을 만드는 것까지 금지하지 않는다.
- 이전에는 upper 원호 2개와 lower 중앙 진입/출구 원호가 MPPI에 포함돼 있었다. 사용자가 턴 이전에 DWB로 넘기길 원해 변경했다.
- 출발/중앙 목표는 `transit_goal_checker`, 최종 정류장은 `general_goal_checker`.
- 최종 위치 허용 0.05m는 유지했다. 최종 yaw 허용은 0.03rad(1.7°)→0.10rad(5.7°). 다음 레인 방향은 유지하면서 과도한 미세 정렬을 줄이려는 조정이다.
- 이 완화만으로 끝점 지연이 사라졌다는 실측은 아직 없다. 마지막 5cm 위치 수렴도 지연 원인일 수 있다.
- 미션이 단계별 FollowPath action을 따로 보내므로 전환 시 순간적인 감속/정지는 가능하다. 끊김 없는 단일 action 전환을 구현한 것은 아니다.

### 4.2 결과 상태와 재시도

- `SUCCEEDED`, `BLOCKED`, `CANCELED`, `FAILED` enum이 있다.
- 과거 `TaskResult.FAILED → BLOCKED` 단순 매핑을 제거했다. 실패 원인이 장애물이라고 단정하지 않는다.
- 현재 FollowPath 실패는 최종적으로 `FAILED`다. `BLOCKED`는 향후 실제 lane 완전 차단 판정용으로 남겨뒀으며 그 판정은 아직 구현하지 않았다.
- MPPI 단계는 최대 3회, 실패 시 2초 wall-time 대기 후 남은 reference path로 재시도한다.
- 이는 **새 우회 경로를 생성하는 전역 재계획이 아니다.** 같은 reference를 재시도한다.
- `/plan` 표시는 진행 지점 이후 남은 경로를 사용한다. 남은 거리도 reference arclength로 계산하도록 수정했다.

## 5. 사람 구성과 최신 TXT 변경

### 5.1 사용자 요구

- upper: 동일 직선 위에 2명. lane을 두 구간으로 나눠 서로 겹치지 않는 x 구간에서 왕복.
- lower: 서로 다른 x 위치의 횡단자 4명. 로봇 진행 방향과 수직 왕복.
- 사람 속도는 기존보다 조금 느리게, 조절 가능하면 조절. 로봇 속도를 올려 회피 문제를 덮지 않는다.

### 5.2 현재 지정값

| 사람 | 왕복 경로 |
|---|---|
| Upper_West | y=14.9, x=-30 ↔ -13.5 |
| Upper_East | y=14.9, x=-10.5 ↔ 6 |
| Lower_Cross_1 | x=-24, y=-7.3 ↔ 3.3 |
| Lower_Cross_2 | x=-16, y=-7.3 ↔ 4.3 |
| Lower_Cross_3 | x=-8, y=-7.3 ↔ 4.4 |
| Lower_Cross_4 | x=0, y=-7.3 ↔ 4.4 |

- lower는 원래 y=-4↔1, 편도 5m였다. 최신은 10.6/11.6/11.7m다.
- 중앙 구조물/벽에 가깝게 끝점을 넓히되 사람 반경 0.5m의 2D free 영역을 유지했다. 모든 새 경로를 0.05m 간격으로 검사해 통과했다.
- lower 양 끝 대기는 최소 4초다. 루프 시작 쪽은 사람별 4/5.5/7/8.5초로 위상을 달리한다. 이 Idle은 처음 한 번만이 아니라 반복마다 적용된다.
- `HOSPITAL_PEOPLE_WALK_BLEND` 환경변수, 기본 0.75, 허용 0.1~1.0. **0.75m/s 또는 정확한 75% 속도가 아니다.** 이전 실제 이동 속도 중앙값은 약 1.09~1.10m/s였다.
- People 자체 dynamic avoidance는 꺼져 있다. 사람은 로봇을 피해 주지 않는다. 사람 물리 collider가 없어 시뮬에서 로봇을 통과할 수 있다. 따라서 눈에 보이는 관통도 별도 몸체 겹침 평가로 실패 처리한다.
- NavMesh 경로 투영이 upper y=14.9 직선을 벗어나게 하여 navigation navmesh 사용은 끄고 지정 직선을 따르게 했다. 초기화 의존성 때문에 NavMesh bake 자체는 유지했다.

### 5.3 보행자가 안 보이거나 안 걷던 문제와 조치

- 보행 behavior script 연결/실행 프롬프트, stage 및 NavMesh 초기화 순서를 점검했다.
- 설치된 NVIDIA `character_behavior.py` 6개만 연결됐는지 preflight로 검증한 뒤 실행한다.
- 검증된 씬 로딩/reset 동안만 scripting 경고창 설정을 조정하고 기존 설정으로 복원한다.
- bake 완료 후 world.reset을 한다.
- 실제 animation graph 캐릭터 위치를 audit한다. USD spawn 좌표를 사람이 움직인 위치처럼 읽지 않는다.
- 20초 후 6명 각각 최대 이동량이 0.5m 이상인지 확인하고, 움직이지 않으면 시험을 유효하다고 처리하지 않는다.
- 최신 TXT 확대에 맞춰 `hospital_people_commands.py`가 마지막 GoTo를 읽고 **실행 중 USD session layer에 루프 시작 위치**로 설정한다. NVIDIA 무한 루프의 자동 원점 복귀가 예전 -4/1 중간점으로 향하지 않도록 하기 위함이다.
- 이 최신 원점 적용 코드는 컴파일 및 TXT 계산 검증만 했고 Isaac 실제 재시작 검증은 남았다. 이번 변경에서 두 USD 파일을 저장/덮어쓰지는 않았다. GUI로 USD만 직접 열면 TXT 기반 session 위치 보정은 자동 적용되지 않는다.

## 6. 사람 대응 구현과 중요한 파라미터

### 6.1 입력과 제어 흐름

```text
Isaac lidar → pointcloud_to_laserscan → /scan → local/global obstacle layer
                                      └→ self-filter → /scan_body_filtered
                                                        ├→ collision monitor
                                                        └→ moving predictor
                                                             → predicted cloud
                                                             → prediction costmap layer
Nav2 DWB/MPPI → smoothing → /cmd_vel_smoothed → monitor → /cmd_vel
```

- self-filter는 실제 몸체 내부의 스캔점만 제거한다. costmap은 기존 `/scan`을 유지한다.
- monitor에 원본 scan을 넣었을 때 자기 몸체 반사점 112개가 걸려 출발부터 멈췄던 문제가 있었다. 그 원인을 제거하려고 필터를 넣었다.
- localization은 기존 ground-truth 흐름을 유지한다. 평가용 사람 ground truth는 제어 입력에 사용하지 않는다.

### 6.2 병원 전용 파라미터 요약

| 항목 | 현재 값/상태 |
|---|---|
| 로봇 footprint | x=-1.38~0.48, y=-0.5~0.5, 길이 1.86m |
| MPPI 모델 | DiffDrive |
| MPPI max vx | 0.6m/s, min vx=0 |
| MPPI horizon | 90×0.05=4.5초 |
| MPPI batch | 1000 |
| CostCritic | weight=5, footprint=true, trajectory_point_step=1 |
| PathAlign / PathFollow / PathAngle | 8 / 8 / 2 |
| local costmap | 12×12m, 0.05m, update 10Hz, publish 2Hz, odom frame |
| local layers | static → scan → prediction → inflation |
| inflation | radius=1.6m, scaling=3.0 |
| prediction timeout | 0.6초 |
| controller failure tolerance | 6초 |
| progress allowance | 45초, PoseProgressChecker |
| monitor | approach, 1.5초, min_points=6, source_timeout=0.6초 |
| DWB FollowPath max vx | 0.8m/s (이번 MPPI 직선화에서도 변경하지 않음) |

DWB 최대 속도는 **0.8m/s 그대로**다. MPPI 0.6m/s와 혼동하지 않는다. 예전 사용자는 collision monitor를 당장 바꾸지 말라고 했지만 이후 사람 대응 구현 단계에서 현재 6으로 조정된 상태다. 이것을 초기 상태 유지라고 잘못 보고하지 않는다.

- 공통 `carter_navigation_params.yaml`도 과거 수정이 남아 있으나 현재 hospital launch의 기본은 **별도 hospital params**다. 잘못된 파일만 수정하면 효과가 없다.
- lane mask 서버는 hospital launch에 없다. 다른 사용처가 있는 마스크 자산까지 전부 삭제했다고 단정하지 않는다.
- static planner가 등록돼 있다고 해서 미션이 실제 전역 planner 우회를 호출하는 것은 아니다.

### 6.3 예측기의 실제 수준

- `/scan_body_filtered`를 scan 시각 TF로 odom에 보정한다.
- 연속 scan점 cluster를 만들고 크기/점 수 필터를 쓴다. 사람 의미 분류기가 아니다. 움직이는 물체 cluster 추정이다.
- track timeout≈0.6초, history 기반 속도 평활화, 3회 이상 관측·최소 움직임 조건을 쓴다.
- 미래 1.8초를 0.2초 간격으로 예측, 반경 기본 0.4m에 시간 불확실성을 더한다.
- `/hospital/predicted_obstacles` PointCloud를 custom costmap layer가 받는다.
- 초기 구현은 미래 영역을 lethal로 만들어 모든 후보가 막히고 로봇이 갇혔다. 수정본은 현재 관측 장애물은 lethal 유지, 미래 영역은 최대 240의 soft-cost로 처리한다.
- **한 장에 미래 궤적을 합친 비용 지도**이므로 시간별 충돌 판정이 아니다. 사람의 반전 의도나 모든 후미 접근을 예측하지 못한다.
- 현재 soft-cost만으로 안전 우선순위가 보장되지 않는다. Jazzy CostCritic은 가중치를 254와 샘플 수로 정규화한다. 가중치 숫자가 크다는 것만으로 안전하다고 해석하지 않는다.

## 7. 실제 시험 결과 — 해결됐다고 말하면 안 되는 근거

평가는 로봇 사각 footprint와 반경 0.35m 사람 원의 부호 있는 거리다. 음수는 몸체 영역 겹침이며 정확한 메시 침투 깊이가 아니다. 기록 간격 약 0.05 시뮬레이션 초. 아래 샘플 수는 충돌 횟수가 아니다.

| 시험 | 결과 | 겹침 증거 | 판단 |
|---|---|---|---|
| 기존 반응형 lower | Nav SUCCEEDED | 31샘플, 최소 -0.834m | 사람 대응 실패 |
| 반응형 설정 조정 lower | Nav SUCCEEDED | 107샘플, 최소 -0.728m | 튜닝만으로 해결 안 됨 |
| 초기 lethal 예측 upper | Nav FAILED | 정지/실패 후 사람과 겹침도 관측 | 미래 hard 점유가 로봇을 가둠 |
| 사용자 최신 soft 예측 lower | Nav 도착 | Cross_1 31샘플, Cross_3 32샘플, 최소 -0.847m | 횡단 대응 실패 |
| 사용자 최신 soft 예측 upper | Nav 도착 | Upper_West 179샘플, Upper_East 39샘플, 최소 -0.847m | 정면 대응 실패 |

시험 시작 시간·사람 위상이 다르므로 단순 샘플 수로 개선율/악화율을 계산하지 않는다.

### 7.1 최신 사용자 세션 분석

- audit: `log/hospital_people/20260922_191435_951048/people.jsonl`.
- lower 관측 구간 근사: sim 55.68~180.02초.
- upper: sim 207.43~328.52초.
- controller wall-time 로그와 센서 wall/sim 대응으로 구간을 복원했다. 완벽한 밀리초 경계는 아니다.
- lower Cross_2는 겹치지는 않았지만 최소 여유 0.168m로 목표 0.25m 미달.
- lower 접촉 시작 99.35 / 126.85초 근처 `/cmd_vel`은 약 0.60m/s였다. 감속·회피가 충분하지 않았다.
- upper 접촉 시작 230.65 / 274.05초 근처 명령은 약 0.085 / 0.054m/s였다. 멈추려는 반응은 있지만 정면 통과 공간을 만들지 못했다.
- 두 미션 시간대 예측 메시지 2,134개 중 808개가 비어 있지 않았다. **노드가 꺼져서 아무 출력도 없었다고 설명할 수 없다.** 다만 올바른 사람을 정확히 추적한 증거도 아니다.

### 7.2 마지막 정류장 지연

- 10cm 안에 처음 들어온 뒤 완료까지 lower 약 20.5초, upper 약 12.1초.
- lower yaw 약 71.2°→88.3°, upper -86.3°→-88.3°.
- 마지막 위치 수렴도 포함하므로 이 시간이 전부 제자리 회전이라고 단정하지 않는다.
- 최신 구간 분담·yaw 허용 수정은 이 문제를 줄이기 위한 것이며 아직 재측정 전이다.

### 7.3 짧은 횡단 왕복/후미 재접근

- 기존 편도 5m에서 동일 사람이 레인을 반대로 다시 지나는 간격 최솟값 약 5.4초.
- 실제 속도 약 1.1m/s, 반전/대기 포함 주기는 사람마다 달랐다.
- 예: Cross_3가 152.60초 상향, 158.65초 하향 재횡단. 겹침은 157.85초부터.
- 길이 1.86 + 사람 지름 0.70 + 양쪽 여유 0.50 = 3.06m. 단순 등속 근사로 로봇 0.6m/s면 5.1초, 0.3m/s면 10.2초가 전체 몸체 이탈에 필요하다. 정지·곡선은 더 걸린다.
- 따라서 앞부분만 통과한 것을 완료로 보면 안 된다. 그렇다고 후미가 빠질 때까지 로봇을 정지시키라는 뜻도 아니다.
- 이번 TXT 확대는 시험 입력을 정상화하는 변경이다. 어려운 재접근을 없애서 회피가 좋아진 것처럼 주장하면 안 된다. 짧은 왕복은 이후 스트레스 조건으로 별도 재현해야 한다. 전용 스트레스 프리셋 파일은 아직 만들지 않았다.

## 8. 기록 체계, 수정 이유, 아직 검증할 점

### 8.1 기존 기록 결함

- 이전 수동 기록기 PID 153507이 남아 새 테스트를 예전 폴더 `20260922_184102_627907/sensors_prediction.jsonl`에 기록했다.
- Isaac 시간이 초기화됐지만 그 기록기의 TF 캐시에는 이전 sim≈1065초 데이터가 남았다.
- 새 세션 scan 변환은 과거 외삽 오류로 실패했고, 원본 scan을 남기지 않아 센서 근거가 유실됐다.
- 이전 기록기는 lethal costmap만 저장해 soft 예측 비용이 실제로 들어갔는지도 확인할 수 없었다.
- 이 기록기 결함을 제어기 자체 TF 실패라고 확대 해석하지 않는다. 제어기/예측기는 별도 프로세스다.

### 8.2 최신 자동 기록

- `hospital_sensor_recorder`를 nav_to_goal entry point로 등록, hospital launch에 항상 포함.
- launch 내에서 죽으면 2초 후 respawn. 사용자에게 네 번째 기록 터미널을 요구하지 않는다.
- 출력: `log/hospital_sensors/<실행시각>.jsonl`.
- 실제 위치 audit: 기존 `log/hospital_people/<세션>/people.jsonl`.
- 실행마다 새 파일. 시뮬 시간 역행 감지 시 파일 회전 및 TF 캐시 초기화.
- raw scan, 예측 원본/변환점, soft+lethal costmap, `/chassis/odom`, `/cmd_vel_smoothed`, `/cmd_vel`, `/plan`, 관련 `/rosout` 저장.
- scan 시각의 TF를 잠시 기다리고 실패도 기록. 최신 TF로 조용히 대체하지 않는다.
- 시뮬 시간이 멈춰도 wall-clock heartbeat 2초마다 저장.
- mission 단계 시작 이벤트에 컨트롤러명을 넣어 ROS logger로도 출력하도록 수정.
- 녹화 파일의 실제 scan/odom/costmap/heartbeat 카운트 증가까지 확인해야 “기록 정상”이다. 파일 존재만으로 판단하지 않는다.
- 자동 파일 삭제/용량 순환 정책은 아직 없다. raw costmap 저장량이 크므로 긴 시험에서는 디스크와 기록 부하를 확인해야 한다.

### 8.3 작성 시점 프로세스 사실

- 점검 도중 사용자가 실행했던 Isaac/Nav2 프로세스가 종료된 상태로 바뀌었다. AI가 이번 점검에서 그 두 프로세스를 종료한 것은 아니다.
- 새 기록기를 기존 launch PID 158740에 임시 연결했으나 해당 launch가 이미 끝나 자동 종료했다. `20260922_194214_358570.jsonl`은 **metadata만 있는 파일**이며 정상 주행 센서 로그가 아니다.
- 기존 수동 기록기 PID 153507은 마지막 확인에서 남아 있었다. 새 AI는 PID를 다시 확인한 후 해당 프로세스만 정리해야 한다. PID는 재사용될 수 있으므로 번호만 믿고 kill하지 않는다.
- “항상 기록”은 테스트 launcher 실행 동안의 자동 기록을 뜻한다. Isaac/Nav2가 꺼진 동안의 무인 감시 서비스나 AI의 지속 모니터링을 설정한 것은 아니다.

## 9. 개선한 것과 성과의 수준

| 변경 | 이유 | 확인 수준 |
|---|---|---|
| 별도 hospital 실행 흐름 | 사용자 프로젝트 분리 요구 | 기존 실주행 사용 |
| 두 독립 lane + 동일 정류장 | 교차 동선·도킹 가정 제거 | 경로 구성/이전 주행 확인 |
| 보행 scripts/reset/bake 점검 | 사람이 안 걷는 무효 시험 방지 | 과거 6명 실제 이동 확인 |
| 몸체 scan self-filter | 자기 반사로 monitor가 출발부터 멈춤 | 해당 원인 제거 확인 |
| FAILED/BLOCKED 분리 | FollowPath 실패 원인 오판 방지 | 코드 반영 |
| footprint audit | collider 없는 사람 관통을 성공 처리하지 않기 | 실제 접촉 실패 발견 |
| 움직임 예측 soft-cost | 현재 점유만 보는 한계 보완 시도 | **최신 실제 회피 실패** |
| 모든 턴 DWB, MPPI 직선 | 사용자가 요청한 분담, 원호 이후 전환 문제 | 단위 시험 통과, 재주행 전 |
| yaw 허용 5.7° | 정류장 불필요한 미세 정렬 완화 | 설정 검증, 실측 전 |
| 횡단 범위 확대 | 너무 빠른 반전/후미 재진입과 기본 회피 분리 | TXT/지도 검증, 새 보행 재생 전 |
| launch 자동 기록 | 누락·잔존 기록기·TF 시간 초기화 문제 | 설치/기동/정지시간 heartbeat 확인, 실센서 재검증 필요 |

## 10. 평가 기준과 사용자가 우려한 대기 비용

아래는 AI가 보고서에 정리한 **초기 개발·합격 기준안**이다. 사용자의 숫자별 승인이나 안전 인증 기준이 아니며 제어기에 모두 구현된 것도 아니다. 다음 AI는 사용자에게 이 구분을 유지해야 한다.

### 10.1 안전과 효율을 분리

- 공통: 몸체 겹침 0, 최소 여유 목표 0.25m.
- 음수/접촉은 안전 실패. 양수라도 0.25m 미만은 여유 미달.
- 목적지 도착, 안전 통과, 능동 회피, 양보 대기는 별도 지표다.
- “후미까지 안전해야 완료”는 평가 조건이다. 이를 후미 이탈 전 정지 명령으로 연결하지 않는다.
- 위험한 후보는 배제하고, 안전한 후보 중 진행/복귀/조향/대기 비용을 비교하는 구조를 목표로 한다. 위험이 남아 있는데 대기 벌점 때문에 무리하게 출발시키지 않는다.

### 10.2 접근 유형별 기준안

| 상황 | 요구 행동/평가 |
|---|---|
| 측면 횡단 | 앞쪽으로 무리하게 끼어들지 않기. 감속·양보 또는 여유 있는 국소 회피. 안전 전진 후보가 1초 유지된 뒤 2초 내 재출발을 효율 기준으로 제안 |
| 정면 | 통과 공간이 있으면 측방 경로 선택. 안전한 후보가 있는데 3초 넘게 정지만 하면 비효율 실패로 제안 |
| 통과 공간 없음 | 정지가 정당할 수 있으나 이후 사람이 박으면 접촉 실패는 그대로 남김 |
| 회피 완료 | 전 몸체 여유 0.25m, 위험 해소 1초 지속, 후미 교차 영역 이탈, 기준선 오차 0.3m 내 복귀를 제안 |
| 후미/반전 재접근 | 별도 사건으로 분리, 관측 시각·접근 속도·접촉까지 시간 기록. 빠르다는 이유만으로 충돌 면제 금지 |

- 기본 시험 범위 제안: 사람 0~1.2m/s, 잠재 접촉 최소 2초 전부터 유효 관측. 이것만으로 회피 가능성을 보장하지 않는다.
- 범위 밖 급진입은 스트레스/한계 조건으로 분류하되 실패를 성공으로 바꾸지 않는다.
- 정면/측면 각각 최소 3개 위상 × 3회 = 9회의 실제 조우를 확보하는 반복 시험을 제안했다.
- 전부 안전 기준을 통과해야 하며, 능동 회피 가능한 시나리오에서는 실제 측방 회피도 보여야 한다. 사람을 안 만난 완주는 제외한다.
- **이 효율/사건 분류 전체가 자동 평가 코드로 구현돼 있지는 않다.** 현재 report는 간격·겹침·이동량·기록 유효성 중심이다.

## 11. 아직 못 고친 이유와 다음 해야 할 일

### 11.1 미해결 원인: 확인된 것과 가설

확인된 구조적 한계:

- future soft-cost는 안전 제약이 아니다.
- 시간 없이 합친 미래 점유는 정지 후보/통과 후보의 실제 시간 충돌을 구분하지 못한다.
- Collision Monitor는 최종 방어지만 정면으로 계속 걸어오는 사람에게 통과 공간을 만들어 주지는 않는다.
- raw scan과 soft costmap 기록이 부족해 이전 시험의 감지→추적→비용→명령 연결을 완전히 재구성할 수 없었다.

아직 가설인 것:

- 추적 cluster 분리/ID 변경, velocity 추정 오차, soft-cost 영향 부족, path 추종 비용 우세 중 무엇이 주된 실패 원인인지 확정하지 못했다.
- 비어 있지 않은 예측 메시지가 특정 사람을 정확히 예측했다는 증거는 없다.
- 새 DWB 분담과 yaw 완화가 정류장 지연을 얼마나 줄일지는 미측정이다.

즉, 단순 파라미터 수정을 완료라고 보고할 근거가 없고, 마지막 수정 뒤 실제 재주행도 아직 없기 때문에 “미해결”로 남겨둔 것이다.

### 11.2 우선순위

**1단계 — 현재 사람 대응 안정화 (아직 미통과)**

1. 최신 빌드로 전체 재시작, 새 보행 범위/루프 원점/6명 이동/자동 recorder 실제 수신 확인.
2. 동일 조건에서 upper 정면, lower 횡단 각각 재시험. 단계·명령·실제 속도·실제 사람 위치를 같은 sim/wall 시간으로 정렬.
3. scan에서 사람 관측→track 지속성→예측 위치→local costmap soft 값→MPPI/monitor 출력 순으로 원인을 좁힘.
4. 필요한 곳에 시간별 인간 예측과 로봇 전체 footprint를 비교하는 후보 안전 판정을 구현. 현재 예측 cloud에는 시간 정보가 없으므로 critic 연동 또는 데이터 구조 변경부터 설계해야 한다.
5. 안전 후보 중 정면 측방 통과/측면 양보/재출발을 선택하도록 개선. 대기 비용은 안전 조건과 분리.
6. 기본 반복 시험을 통과한 뒤 짧은 왕복·반전·후미 스트레스 시험. 속도 증가로 통과시키지 않는다.

**2단계 — 선택된 운행선 안의 복구/우회 구조**

- 실제 차단 여부 판정과 일시 보행자 대기/복구를 구분.
- 기존 3회 동일 경로 재시도 이상의 재계획이 필요한지 검토.
- 반대 lane 자동 전환 금지, reference 복귀 유지.
- 이 단계의 전역 우회 fallback은 아직 구현하지 않았다.

**3단계 — 관제 연결·반복 평가·다로봇 확장 준비**

- 상태/이벤트 인터페이스, 정류장 대기열, 장기 기록·회귀 시험.
- 사용자가 맡겠다고 한 도킹은 별도 범위로 둔다.
- 이 단계 분리는 현재 상태에 맞춘 인계용 우선순위다. 이전 첨부 계획의 모든 세부 항목이 구현됐다는 의미가 아니다.

## 12. 검증 현황

완료:

- `colcon build --packages-select hospital_dynamic_layer nav_to_goal carter_navigation --symlink-install`: 3개 패키지 성공.
- pytest 8개 통과: `test_hospital_routes.py`, `test_obstacle_tracking.py`, `test_hospital_people_audit.py`.
- 경로 검사: MPPI는 단일 직선, 모든 원호 DWB, 구간 좌표/접선 연속, 양쪽 정류장 도착 yaw와 다음 출발 yaw 일치.
- Python 컴파일 검사: recorder, launch, simulator runner, People command helper.
- 새 횡단 경로 4개: 지도상 반경 0.5m, 샘플 0.05m free sweep 통과.
- recorder 분리 ROS domain에서 기동 및 sim clock 정지 중 heartbeat 점검. 실제 센서 수신 검증과는 별개다.

미완료:

- 새 TXT/session-layer 원점의 Isaac 실제 반복 보행.
- 새 DWB/MPPI 분담의 양방향 실주행·끝점 소요 시간.
- 자동 recorder가 새 실제 주행에서 scan/odom/soft map을 빠짐없이 받는지.
- recorder clock-reset 파일 회전의 실제 Isaac 재시작 시험.
- 정면/측면의 무접촉·여유·능동 회피 성과.

## 13. 실행 및 재현 명령

### 빌드

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select hospital_dynamic_layer nav_to_goal carter_navigation --symlink-install
source install/setup.bash
```

### 터미널 1 — 시뮬레이션

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/home/rokey/isaacsim/python.sh isaacpjt/pjt_alpha/run_hospital_sim.py
```

### 터미널 2 — Navigation 및 자동 센서 기록

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch carter_navigation hospital_navigation.launch.py
```

### 터미널 3 — lab에서 lower로 첫 미션

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run nav_to_goal hospital_mission --ros-args -p route_id:=lab_to_specimen
```

서쪽 도착 뒤 Isaac과 Nav2를 유지한 채 복귀:

```bash
ros2 run nav_to_goal hospital_mission --ros-args -p route_id:=specimen_to_lab
```

이번 TXT/시뮬 초기화 코드 변경을 적용하려면 Isaac도 다시 실행해야 한다. YAML/launch 변경은 Nav2 재시작이 필요하다. symlink 소스 반영과 이미 실행 중인 프로세스의 설정 갱신을 혼동하지 않는다.

## 14. 주요 근거 파일과 참고 자료

### 로컬 로그

- 기존 lower: `log/hospital_people/20260922_183555_650099/baseline_lower_report.json`.
- 조정 lower: `log/hospital_people/20260922_184102_627907/tuned_reactive_lower_report.json`.
- 최신 사용자 GT: `log/hospital_people/20260922_191435_951048/people.jsonl`.
- 최신 사용자 ROS: `/home/rokey/.ros/log/component_container_isolated_158769_1790072092363.log`.
- 최신 사용자 미션 ROS: `/home/rokey/.ros/log/python3_159452_1790072163716.log`, `python3_159786_1790072406287.log`.
- 혼합 센서 로그: `log/hospital_people/20260922_184102_627907/sensors_prediction.jsonl`. 반드시 wall time으로 세션 분리. scan TF 결함 주의.
- 초기 hard prediction 실패: `/tmp/hospital_prediction_upper1.log`.
- `/tmp` 근거는 지워질 수 있다. 필요하면 사용자가 정한 로그 보존 위치로 관리하되 무단 덮어쓰지 않는다.

### 사용자 제공/검토 자료

- [Isaac ROS2 Navigation](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/robot_control/tutorial_ros2_navigation.html): latest 문서와 로컬 5.1 버전 차이 주의.
- [Nav2 MPPI Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/).
- [Jazzy CostCritic 구현](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/critics/cost_critic.cpp).
- [Nav2 Route Server](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/): 장기 참고, rolling 기능을 Jazzy에서 확인 없이 도입하지 않는다.
- [The Marathon 2](https://arxiv.org/abs/2003.00368).
- [MPPI 원 논문](https://doi.org/10.2514/1.G001921).
- [Human-aware navigation review](https://pmc.ncbi.nlm.nih.gov/articles/PMC11086376/).
- 이전 사용자 계획 문서: `/home/rokey/Downloads/협동3_AMR_자율주행_3단계_구현계획.md`. 이후 사용자 정정이 우선이다.

## 15. 다음 AI에게 마지막 주의

**사용자는 실제 사람 회피가 좋아지지 않았다고 정확히 지적했고 로그도 그것을 뒷받침한다.** 설정을 추가했다는 사실을 성과로 포장하지 말고, 재현 가능한 정면·측면 시험과 센서 연결 증거로 개선을 보여 줘야 한다. 도킹이나 옛 좌표/180°로 돌아가지 말고, 현재 사람 대응 실패를 해결하기 전 다음 단계 완료를 선언하지 않는다. 커밋/푸시는 사용자 지시 범위를 다시 확인한다.
