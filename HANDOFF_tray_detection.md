# 트레이 검출 기반 Pick & Place — 인수인계 문서

작성 2026-09-22 · 대상: 다음 세션 / 이 코드를 처음 보는 작업자

Isaac Sim 안에서 **합성 데이터로 YOLO를 학습시키고, 그 모델로 트레이를 검출해
로봇팔이 집어서 랙에 넣는** 파이프라인이다. 이 문서는 전체 구조, **좌표계 구분**,
실행 방법, 그리고 삽질하며 알아낸 것들을 정리한 것이다.

> **가장 먼저 읽을 곳**: 아래 [2. 좌표계](#2-좌표계--여기서-제일-많이-틀렸다).
> 이 프로젝트에서 발생한 버그의 절반 이상이 좌표계 혼동이었다.

---

## 1. 전체 구조

```
[1단계 데이터 생성]  Isaac Replicator ──> 합성 이미지 + 자동 라벨 ──> YOLO 데이터셋
[2단계 학습]         YOLO26s 학습 ──> best.pt
[3단계 실행]         Isaac Sim (로봇/카메라)  <──ROS2──>  관제 PC (YOLO 추론)
```

3단계는 **반드시 두 프로세스**여야 한다. Isaac Sim은 자체 Python 3.11 + numpy 1.26을
쓰고 ultralytics는 Python 3.12 환경이라 한 프로세스에 합칠 수 없다.
(Isaac 쪽에 ultralytics를 설치하면 numpy가 2.x로 덮여 Isaac이 깨진다 — 시도하지 말 것.)

### 파일 지도

| 파일 | 역할 |
|---|---|
| `isaacpjt/sdg/object_based_sdg.py` | NVIDIA 공식 예제 복사본 + 패치 3곳 (로컬 USD 경로 허용, nested rigid body 제거, yaml 리스트→튜플) |
| `isaacpjt/sdg/object_based_sdg_utils.py` | 위 예제의 유틸 (원본 그대로) |
| `isaacpjt/sdg/tray_sdg.yaml` | 데이터 생성 설정 (해상도, 프레임 수, distractor 등) |
| `isaacpjt/sdg/to_yolo.py` | Replicator 출력(npy/json) → YOLO 라벨(txt) 변환 |
| `isaacpjt/sdg/train_yolo.py` | YOLO26s 학습 |
| `admin_ws/src/tray_detector/tray_detector/detect_node.py` | **관제 PC 노드** — 이미지 구독, YOLO 추론, 3D 좌표 발행 |
| `isaacpjt/M0609/teleop/pick_and_place_detection.py` | **Isaac 메인** — 카메라 발행, 검출 구독, 자동 pick & place |

**건드리지 않은 원본** (참고용, 검출 기능 없음):
`pick_and_place.py`, `pnp_teleop.py`, `pickup_place_go_nova.py`(네비게이션 포함), `nav_mission.py`

> `pick_and_place_detection.py`는 `pick_and_place.py`를 복사해 만들었지만,
> 씬을 `integration_human.usd`로 바꾸면서 **nova_carter(모바일 베이스) 구조**로
> 전환했다. 네비게이션(Nav2 주행)은 이 파일에 **없다** — 그건 `pickup_place_go_nova.py` 계열이다.

---

## 2. 좌표계 — 여기서 제일 많이 틀렸다

### 2.1 한눈에 보기

```
world (Isaac 월드)
 └─ /World/robot_nova/nova_carter                 ← ROBOT_PRIM_PATH (검색 범위)
     ├─ chassis_link                              ← ART_ROOT_PATH  (Articulation 루트)
     └─ Robot/m0609_camera/m0609/
         ├─ base_link                             ← ARM_BASE_PATH  (IK 기준 = "base 프레임")
         └─ .../realsense_d455/RSD455             ← 마운트 Xform (조그 축용, 비표준 축!)
             └─ Camera_OmniVision_OV9782_Color    ← 실제 렌더링 카메라 (표준 USD 축)
```

### 2.2 chassis_link vs base_link — **가장 중요**

카터와 팔이 **하나의 아티큘레이션**(dof 19개)으로 병합돼 있다.

| 용도 | 써야 하는 프림 | 이유 |
|---|---|---|
| Articulation 등록 (`SingleManipulator(prim_path=...)`) | **`chassis_link`** | 실제 아티큘레이션 루트 |
| IK 기준 프레임 (`lula.set_robot_base_pose`) | **`base_link`** | 팔의 실제 시작점 |
| 드라이브/EE/카메라 프림 검색 | `nova_carter` 서브트리 | 둘 다 포함하는 상위 |

`robot.get_world_pose()`는 **chassis_link**를 돌려준다. IK 기준으로 쓰면 안 된다.
반드시 `arm_base_pose()`(= `SingleXFormPrim(ARM_BASE_PATH).get_world_pose()`)를 쓸 것.

**이 문서에서 "base 기준"이라고 하면 전부 `base_link` 기준이다.**
`POINT1~6`, `TRAY_HALF_DEPTH_M`, `SCAN_DESCEND_Z_M` 등 모든 좌표 상수가 base 기준이다.

> 실제로 겪은 버그: `SCAN_DESCEND_Z_M`을 "base 기준 z"라고 문서화해놓고
> 코드에서는 월드 z에 그대로 대입했다. base_link의 월드 높이가 ~0.41m라서
> 팔이 마운트보다 낮은 바닥 근처로 내려갔다. → `build_descend_step()`에서
> `world_to_base_pos()`로 변환 후 z를 바꾸고 다시 월드로 되돌리는 것으로 수정.

### 2.3 카메라 광학 프레임 (ROS) vs USD 카메라

| | x | y | z |
|---|---|---|---|
| **ROS 광학** (관제 PC가 보내는 좌표) | 오른쪽 + | **아래 +** | 앞(전방) + |
| **USD 카메라** (Isaac 프림) | 오른쪽 + | **위 +** | **뒤 +** (전방은 −z) |

변환은 `optical_to_world()`가 담당하며 **y와 z의 부호를 뒤집는다**:

```python
usd_point = Gf.Vec3d(x, -y, -z)      # ROS 광학 → USD 카메라 로컬
world = cam_prim_local_to_world.Transform(usd_point)
```

이 변환은 실측 로그로 검증됐다 (`world`와 `cam_origin + fwd*z`의 차이가
`cam`의 횡방향 성분 크기와 소수점 셋째 자리까지 일치).

### 2.4 RSD455(마운트) vs Color 카메라 — 축이 다르다

USD에서 직접 측정한 결과:

| 비교 | 내적 | 결론 |
|---|---|---|
| RSD455 `up` vs 영상 up | **+1.0** | 일치 |
| RSD455 `right` vs 영상 right | **+1.0** | 일치 |
| RSD455 `forward` vs 영상 forward | **−1.0** | **180° 반대** |

- `RSD455` Xform: 로컬 **+X가 시선의 반대**, +Y가 right, +Z가 up (비표준)
- `Camera_..._Color`: 표준 USD 카메라 (+X right, +Y up, **−Z** forward)

**영상/깊이 관련 계산은 반드시 Color 카메라 프림을 쓸 것.**
`camera_world_right()`가 Color 카메라의 월드 right 축을 돌려준다.

### 2.5 TCP vs 플랜지

`TCP_OFFSET = [0, 0, 0.21671]` — link_6(플랜지) 로컬 +Z 기준 손가락 패드 끝까지.
IK는 플랜지 목표를 받으므로 `tcp_to_flange()`로 변환해서 넘긴다.
카메라는 TCP보다 약 21.7cm 뒤에 있다(= 관측 거리 계산 시 고려).

### 2.6 변환 함수 정리

| 함수 | 방향 |
|---|---|
| `base_to_world(tcp, rpy, base_pos, base_quat)` | base → world (위치+자세) |
| `base_pose_to_world(tcp, quat, base_pos, base_quat)` | base → world (자세를 쿼터니언으로 받음) |
| `world_to_base_pos(world_pos, base_pos, base_quat)` | world → base (**위치만**) |
| `optical_to_world(point_optical, color_camera_path)` | ROS 광학 → world |
| `tcp_to_flange(tcp, quat)` | TCP → 플랜지 |

`base_to_world`와 `world_to_base_pos`는 정확한 역함수다 (왕복 검증 완료).

---

## 3. 실행 방법

### 3-0. 환경 (매 터미널 공통)

```bash
cd ~/cobot3_ws && source /opt/ros/jazzy/setup.bash
```

**ROS2 source를 빠뜨리면 토픽이 하나도 안 나간다.** `isaac-sim.sh`와 달리
`python.sh`는 `setup_ros_env.sh`를 source하지 않아서, 브리지가 ROS2 라이브러리를
못 찾고 **에러 없이 조용히** 실패한다. `ROS_DOMAIN_ID=136`은 `~/.bashrc`에 있다.

### 3-1. 실행 (터미널 2개)

**A. 관제 PC 검출 노드** (먼저 켜기)
```bash
cd ~/cobot3_ws && source /opt/ros/jazzy/setup.bash && PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH ~/yolo-venv/bin/python admin_ws/src/tray_detector/tray_detector/detect_node.py runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt
```

**B. Isaac Sim**
```bash
cd ~/cobot3_ws && source /opt/ros/jazzy/setup.bash && ~/isaacsim/python.sh isaacpjt/M0609/teleop/pick_and_place_detection.py
```
뷰포트 클릭 후 **Play**만 누르면 자동 진행. 완료 후 Play를 다시 누르면 재시도.

### 3-2. 데이터 재생성 / 재학습 (필요할 때만)

```bash
cd ~/cobot3_ws && ~/isaacsim/python.sh isaacpjt/sdg/object_based_sdg.py --config isaacpjt/sdg/tray_sdg.yaml
```
```bash
cd ~/cobot3_ws && python3 isaacpjt/sdg/to_yolo.py isaacpjt/sdg/_out_tray isaacpjt/sdg/dataset_tray
```
```bash
cd ~/cobot3_ws && ~/yolo-venv/bin/python isaacpjt/sdg/train_yolo.py
```

프레임 수를 **줄여서** 재생성할 때는 `_out_tray`를 반드시 지울 것 (옛 실행의 높은 번호
파일이 남아 두 실행이 섞인다). 늘릴 때는 덮어써지므로 안전하다.

### 3-3. 자체 검증 (Isaac/ROS2 없이 즉시)

```bash
~/yolo-venv/bin/python isaacpjt/sdg/to_yolo.py demo
```
```bash
source /opt/ros/jazzy/setup.bash && PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH ~/yolo-venv/bin/python admin_ws/src/tray_detector/tray_detector/detect_node.py demo
```

`pick_and_place_detection.py`는 시작 시 `check_math()`가 자동 실행된다
(파지 yaw 보정 부호, `rack_yaw_delta_deg` 구조 검증). 틀리면 씬 로드 전에 멈춘다.

### 3-4. 디버깅

```bash
source /opt/ros/jazzy/setup.bash && ros2 topic list | grep -E "wrist|tray"
```
```bash
source /opt/ros/jazzy/setup.bash && ros2 topic echo /tray_detection --once
```

**검출 이미지가 `~/tray_detections/`에 저장된다** (10프레임마다 1장).
"검출이 이상하다" 싶으면 **추측하지 말고 이 이미지를 먼저 열어볼 것.**
실제로 "depth 노이즈 문제"로 오진했다가, 이미지를 보니 박스가 벽을 덮고 있어서
프레이밍 문제였던 적이 있다.

---

## 4. 동작 시퀀스

Play를 누르면 상태 기계가 아래 순서로 진행한다 (`main()` 안).

| 상태 | 하는 일 |
|---|---|
| `scan` | `joint_1`을 `SCAN_ROTATE_DEG`(−90°) 상대 회전 → 손목 카메라가 트레이 쪽을 봄 |
| `descend` | z를 `SCAN_DESCEND_Z_M`(base 기준)로 낮추고 카메라를 `PITCH_DOWN_DEG`만큼 기울임 |
| `settle` | 90프레임 정지 후 **새로 찍힌** 검출 프레임을 기다림 (`seq` 카운터로 판단) |
| `center` | 검출점의 광학 x만큼 **카메라 수평축**으로 이동 → ROI를 화면 중앙에 맞춤 |
| `settle` | 다시 정지 + 새 검출 대기 |
| `pick` | 재측정한 좌표로 접근→파지→들어올리기→회전→**슬롯 위**→1초 대기→**수직 하강**→놓기→후퇴→홈 |

### 왜 2번 검출하나
비스듬히 본 박스의 중심은 트레이 중심과 어긋난다. 중앙에 맞춘 뒤 다시 재면
훨씬 안정적이다. 그래서 `center`로 정렬 → 재검출 → 최종 파지점 확정 순서다.

### `seq` 카운터
관제 PC가 발행할 때마다 1씩 올린다. Isaac이 "**이동 후에 새로 찍은 값인지**"를
구분하는 데 쓴다 (메시지에 타임스탬프가 없어서). `FRESH_SEQ_ADVANCE=2` 이상
올라가야 인정한다.

### 놓기는 반드시 2단계
대각선으로 내려가면 트레이가 랙 테두리에 걸린다. **슬롯 바로 위**(`POINT4_TCP`에
z만 `RACK_ABOVE_Z_M` 더한 점)로 먼저 간 뒤 **수직으로만** 내려간다.
`p4_above`는 `POINT4_TCP`에서 파생되므로 놓는 좌표를 바꿔도 수직성이 자동 유지된다.

---

## 5. 통신 규약

```
Isaac ──> 관제 PC
  /wrist_camera/color/image_raw    sensor_msgs/Image   rgb8, 640x480
  /wrist_camera/depth/image_raw    sensor_msgs/Image   32FC1, 미터
  /wrist_camera/color/camera_info  sensor_msgs/CameraInfo

관제 PC ──> Isaac
  /tray_detection   std_msgs/Float32MultiArray
      data = [seq, n, x1,y1,z1,conf1, x2,y2,z2,conf2, ...]
      좌표는 카메라 광학 프레임(ROS 규약) 미터, 화면 왼쪽부터 정렬
```

depth와 color는 **같은 render product**에 붙어 있어 픽셀 단위로 정렬돼 있다.
별도 정합이 필요 없다. (`type="depth"`는 `DistanceToImagePlane` = 광축 방향 Z라
표준 핀홀 역투영 식이 그대로 맞는다. 유클리드 거리가 아니다.)

---

## 6. 튜닝 상수

### `pick_and_place_detection.py`

| 상수 | 현재값 | 의미 |
|---|---|---|
| `SCAN_ROTATE_DEG` | `-90.0` | 스캔 회전량 (실측으로 확정, 씬 바뀌면 재확인) |
| `SCAN_DESCEND_Z_M` | `0.11` | 스캔 후 내려갈 높이 (**base 기준**) |
| `PITCH_DOWN_DEG` | `-30.0` | 카메라 하향 기울기 (실측으로 부호 확정) |
| `DETECT_SETTLE_FRAMES` | `90` | 도착 후 정지 대기 (60Hz 가정 ≈ 1.5초) |
| `FRESH_SEQ_ADVANCE` | `2` | 새 프레임 인정 기준 |
| `FRESH_TIMEOUT_FRAMES` | `300` | 검출 안 오면 포기 |
| `MAX_DETECT_RETRIES` | `3` | 게이트 실패 시 재시도 횟수 |
| `DETECT_DEPTH_MIN/MAX_M` | `0.15` / `1.5` | 깊이 게이트 (그리퍼/먼 배경 차단) |
| `GRASP_REACH_MIN/MAX_M` | `0.25` / `0.95` | **파지점 도달거리 게이트** (벽 오검출 차단) |
| `TRAY_HALF_DEPTH_M` | `0.057` | 검출점(앞면)에서 접근 방향으로 더 들어가는 양 |
| `GRASP_ABOVE_CENTER_M` | `0.04` | 파지 높이 보정 |
| `APPROACH_BACKOFF_M` | `0.082` | 파지 전 수평 후퇴 거리 |
| `RACK_ABOVE_Z_M` | `0.10` | 슬롯 위 안전 지점 높이 (= 수직 하강 거리) |
| `RACK_PLACE_WAIT_STEPS` | `60` | 하강 전 대기 (60Hz 가정 ≈ 1초) |
| `MAX_IK_JOINT_JUMP_DEG` | `20.0` | 손목 특이점 가드 임계값 |

**랙 좌표** (base 기준):
```
POINT4_TCP = [-0.0093, 0.7890, 0.2500]   놓는 위치 (랙 가운데 슬롯)
POINT5_TCP = [-0.0093, 0.6390, 0.2500]   후퇴 (수평 15cm)
POINT4/5_RPY = (-89.3, 0.1, 180.0)
```
`POINT4` y 조정 이력: `0.7690 → 0.8190(+5) → 0.7590(−6) → 0.7890(+3)`

> `POINT1/2/3/6`은 **이제 런타임에 쓰이지 않는다** (파지는 검출값으로 결정).
> `POINT2_RPY`(파지 기준 자세)와 `POINT3_TCP[2]`(`LIFT_Z_M`)만 쓴다.

### `detect_node.py`

| 상수 | 현재값 | 의미 |
|---|---|---|
| `CONF_THRESHOLD` | `0.85` | YOLO 신뢰도 하한 |
| `DEPTH_PERCENTILE` | `15` | 박스 안 깊이 표본 중 **가까운 쪽** 퍼센타일 |
| `MIN_DEPTH_SAMPLES` | `20` | 유효 표본 하한 |
| `MIN/MAX_DEPTH_M` | `0.15` / `1.5` | 깊이 게이트 |
| `MAX_TRAYS` | `3` | 랙 슬롯 수 |

---

## 7. 삽질 기록 — 같은 함정 다시 밟지 말 것

### 환경
- **`python.sh`는 ROS2 환경을 설정하지 않는다.** source 없이 실행하면 토픽이
  하나도 안 나가고 **에러도 안 난다**.
- Isaac Sim 번들 Python에 ultralytics를 설치하지 말 것 (numpy 1.26 → 2.4로 덮여 깨짐).
  대신 `~/yolo-venv`(Python 3.12) + Jazzy의 rclpy 조합을 쓴다 (검증됨).
- 종료 시 `omni.graph`/`syntheticdata` 해제 중 segfault가 난다. Ctrl+C를 받아
  그래프 프림을 먼저 지우고 닫도록 처리했지만 완전히 사라지진 않는다.
  **이 크래시가 진짜 파이썬 트레이스백을 덮을 수 있으니 로그 위쪽을 볼 것.**

### OmniGraph / ROS2 브리지
- 제네릭 `ROS2Publisher`/`ROS2Subscriber`는 메시지 타입을 **`og.Controller.edit`의
  `SET_VALUES`로 한꺼번에 넣으면 조용히 안 올라온다.** 공식 테스트대로
  `messageName=""` → app update → package/subfolder/name 순서 → app update 로
  설정해야 한다 (`set_generic_message_type()`).
- `ROS2CameraHelper`는 render product에 writer를 붙이는 구조라 "한 장만 발행"이
  안 된다. 온디맨드가 필요하면 SDG 파이프라인의 `IsaacSimulationGate`를 여닫아야
  하는데, 그 경로는 이 프로젝트에서 실패했다(게이트 노드를 못 찾음).
  현재는 **계속 발행 + `seq`로 신선도 판단** 방식을 쓴다.
- `cameraPrim`은 relationship이라 `SET_VALUES`로 못 넣는다 → `set_targets()`.
- yaml에서 온 리스트는 `GfVec2f` 속성에 그대로 못 넣는다 → 튜플 변환 필요.

### 로봇 / IK
- **손목 특이점**: `joint_5`가 0 근처면 `joint_4`/`joint_6` 축이 일직선이 되어
  IK가 다른 해로 튀고 팔이 뒤틀린다. `SingularityGuardedIK`가 한 프레임에 20°
  넘게 튀는 해를 걸러낸다. 다만 이건 **안전망이지 해결이 아니다** — 걸리면
  그 스텝은 목표에 못 미친 채 끝난다.
- `default_q`의 `joint_5`가 0(특이점)이지만 warm start는 현재 관절값이라 큰 영향은 없다.
- 드라이브가 한 스텝에 목표를 못 따라와 몇 cm 못 미친 채로 그리퍼가 닫히는 일이
  있다 → 같은 pose를 한 번 더 주는 패턴이 원본 코드에 있다.
- `joint_1` 회전량을 **고정값으로 쓰면 안 된다.** 파지 위치가 검출값이라 매번
  달라지므로 `rack_yaw_delta_deg()`로 파지 방향과 place 방향의 차이를 계산한다.

### 검출 / 깊이
- **트레이는 손잡이+랙+시험관이 붙은 비대칭 조립체**라, 2D 박스의 기하학적
  중앙이 물체가 아니라 **빈틈**에 떨어지기 쉽다. 거기를 찍으면 뒤 배경이 잡힌다.
  → 중앙 패치 median을 버리고 **박스 전체의 하위 퍼센타일**을 쓴다.
- 재시도했는데 **값이 매번 완전히 동일**하면 노이즈가 아니라 **같은 것을 계속 정확히
  보고 있다**는 뜻이다. 그때는 카메라가 무엇을 보고 있는지(저장 이미지)를 확인할 것.
- 화면 아래쪽 그리퍼가 오검출로 잡힌다 (신뢰도 0.85를 넘김). 깊이 게이트로 거른다.
- 학습 데이터가 전부 합성이라 **합성 val의 mAP는 의미가 거의 없다** (0.805 나왔지만
  실물 성능 지표가 아니다). 실물 사진으로 별도 val 셋을 만들어야 한다.

### 좌표 실측
- 좌표가 안 맞으면 **추측하지 말고 실측할 것.** `pnp_teleop_detection.py`(수동 조작판)의
  `1`(기록) / `2`(base 기준 출력) 키를 쓰면 된다. 기록에 쓰는 변환과 실행에 쓰는
  변환이 정확한 역함수라, 기록한 값을 그대로 넣으면 반드시 그 자리로 돌아간다.
- `V` 키를 누르면 팔을 움직이지 않고 검출점(빨강)/랙 자리(파랑)/후퇴점(노랑)
  마커를 씬에 표시한다. **파지 전에 이걸로 먼저 확인할 것.**

### 파이썬
- `grasp, world = found` 로 언팩했다가 `main()`의 `world`(시뮬레이션 World 객체)를
  numpy 배열로 덮어써서 다음 프레임 `world.step()`에서 죽은 적이 있다.
  → `tray_world` / `_tray_world`로 이름을 분리했다. **`world`는 예약어처럼 취급할 것.**

---

## 8. 현재 상태와 남은 일

### 동작하는 것
- SDG 데이터 생성 → YOLO 학습 → 검출 → ROS2 통신 → 파지 → 랙 적재 전 구간 연결됨
- 검출 이미지가 손목 카메라 화면임을 실제 저장 이미지로 확인
- 좌표 변환(광학→월드→base) 수치 검증 완료

### 검증 안 된 것 / 주의
- **`POINT4`/`POINT5`(랙 좌표)는 다른 씬에서 실측한 값을 옮겨온 것**이다
  (`pnp_teleop_detection.py`의 랙 가운데 슬롯). `integration_human.usd`에서
  정확한지 `V` 키로 재확인이 필요하다.
- 놓은 뒤 후퇴가 **수평 15cm**다. 그리퍼를 연 뒤 같은 높이로 빠지므로
  손가락이 테두리를 스칠 수 있다. 걸리면 "수직으로 먼저 올린 뒤 후퇴"로 바꿀 것.
- 트레이 여러 개 처리는 이 파일에 **없다** (한 개만 집는다).
  다중 슬롯 적재는 `pnp_teleop_detection.py`의 `RACK_SLOTS` 로직 참고.

### 개선 후보
1. **실물 val 셋** — 지금 지표는 전부 합성 기준이라 sim2real 갭이 안 보인다.
2. **트레이 자세(yaw) 추정** — 2D 박스로는 방향을 알 수 없어서 "base에서 트레이를
   향하는 방향"을 정면으로 가정한다. 트레이가 비스듬하면 어긋난다.
   깊이 점군 PCA로 주축을 뽑는 방법이 있다.
3. **깊이 클러스터링** — 퍼센타일로 부족하면 1D gap 기반 군집 분리.
4. **특이점 회피 경유점** — 현재는 가드(거부)만 있고 회피 경로는 없다.

---

## 9. 참고 수치 (`integration_human.usd`)

```
base_link 월드 위치   (1.658, 3.363, 0.408)
base_link 월드 자세   pure yaw (w=0.408, z=0.913)
트레이 월드 위치      (2.033, 4.194, 0.820)
트레이 base 기준      (0.369, -0.833, 0.413)
```

현재 랙 관련 지점의 base 기준 거리:

| 지점 | 거리 |
|---|---|
| 놓기 위 안전 (`p4_above`) | 0.863 m |
| 놓는 위치 (`POINT4`) | 0.828 m |
| 후퇴 (`POINT5`) | 0.686 m |

실제로 파지에 성공한 지점들은 대략 **0.7 ~ 0.87 m** 대역이었다. `GRASP_REACH_*`
게이트(0.25~0.95)는 이보다 넉넉하게 잡아둔 것이고, 벽 오검출(1.9m대)을 거르는 게
목적이다. 게이트가 정상 검출을 자꾸 거르면 **범위를 넓히기 전에 먼저 저장 이미지로
무엇을 보고 있는지 확인할 것** — 실제로 그 상황은 게이트 문제가 아니라 카메라가
벽을 보고 있던 문제였다.
