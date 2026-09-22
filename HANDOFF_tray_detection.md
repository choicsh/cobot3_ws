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
                                    └──ROS2──>  Nav2 스택 + 주행 미션 (적재 후 주행/도킹)
```

전체 시나리오는 **적재(3개) → 랙 ArUco 관측 → 주행 → 도킹 → 책상 하역** 이고,
프로세스 4개로 나뉜다 (아래 [3-1](#3-1-실행-전체-시나리오-터미널-4개)).

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
| `isaacpjt/sdg/make_markers.py` | 긴급도 ArUco 마커 PNG 생성 (1회) |
| `admin_ws/src/tray_detector/tray_detector/detect_node.py` | **관제 PC 노드** — 이미지 구독, YOLO 추론, 3D 좌표 발행 |
| `isaacpjt/M0609/teleop/pick_and_place_detection.py` | **Isaac 메인** — 씬/사람/후방 라이다 + 카메라 발행, 검출 구독, 적재 → 관측 → (주행 대기) → 하역 |
| `src/nova_carter/nav_to_goal/nav_to_goal/through_pose_human_test.py` | **주행 미션** — 적재 완료 신호 대기 → undock → goThroughPoses → dock → 완료 신호 |
| `src/nova_carter/nav_to_goal/nav_to_goal/straight_drive.py` | undock/dock 구간용 cmd_vel 직진 드라이버 (회전하지 않는다) |
| `src/nova_carter/carter_navigation/launch/nav2_human_test.launch.py` | 단일 로봇 Nav2 스택 (+ pointcloud_to_laserscan, rviz) |

**건드리지 않은 원본** (참고용, 검출 기능 없음):
`pick_and_place.py`, `pnp_teleop.py`, `pickup_place_go_nova.py`(네비게이션 포함), `nav_mission.py`

> `pick_and_place_detection.py`는 `pick_and_place.py`를 복사해 만들었지만,
> 씬을 `integration_human.usd`로 바꾸면서 **nova_carter(모바일 베이스) 구조**로
> 전환했다. 주행(Nav2)은 이 파일이 직접 하지 않는다 — Isaac 번들 파이썬에서 rclpy 를
> 못 쓰기 때문이다. 대신 **`/mission_state` · `/nav_done` 두 토픽으로 주행 프로세스와
> 손을 잡는다**(아래 5. 통신 규약). 이 파일이 `run_human_scene.py` 의 일(사람 확장 ·
> navmesh 베이크 · 후방 라이다 발행)까지 겸하므로 주행용 씬을 따로 띄우지 않는다.
> (`run_human_scene.py` 는 주행만 단독 시험할 때 계속 쓴다.)

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

### 3-1. 실행 (전체 시나리오, 터미널 4개)

순서대로 띄운다. **A → B(Play) → C → D.** C(Nav2)는 Isaac 이 `/clock` 과 TF 를
내보내기 시작한 뒤에 띄워야 AMCL 이 정상적으로 초기화된다.

**A. 관제 PC 검출 노드** (먼저 켜기)
```bash
cd ~/cobot3_ws && source /opt/ros/jazzy/setup.bash && PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH ~/yolo-venv/bin/python admin_ws/src/tray_detector/tray_detector/detect_node.py runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt
```

**B. Isaac Sim** (씬 + 사람 + 팔 + 후방 라이다)
```bash
cd ~/cobot3_ws && source /opt/ros/jazzy/setup.bash && ~/isaacsim/python.sh isaacpjt/M0609/teleop/pick_and_place_detection.py
```
뷰포트 클릭 후 **Play**. 적재 3개 → 랙 관측 → 홈 복귀까지 자동 진행하고,
그 다음은 D 가 도킹을 끝낼 때까지 기다린다.

**C. Nav2 스택**
```bash
cd ~/cobot3_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch carter_navigation nav2_human_test.launch.py
```

**D. 주행 미션**
```bash
cd ~/cobot3_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run nav_to_goal through_pose_human_test
```
`/mission_state` 가 1 이 될 때까지 대기하다가 undock → 경유지 4개 → dock 을 하고
`/nav_done` 1 을 발행한다. 그러면 Isaac 이 하역을 시작한다.
주행만 단독으로 시험하려면 `--solo` 를 붙인다(대기 없이 바로 출발).

> **다시 돌릴 때는 Isaac 에서 Stop 을 먼저 누를 것.** 정지 시점에
> `/mission_state` 0 발행과 `/nav_done` 값 초기화(`clear_nav_done`)를 한다.
> Stop 없이 Play 만 다시 누르면 이전 주행의 `/nav_done` 1 이 구독 노드에 남아
> **주행을 건너뛰고 바로 하역이 시작된다.**
> Isaac 을 재시작하지 않고 반복할 때는 D 도 매번 다시 실행한다.

수동 진행: 주행 없이 하역만 보고 싶으면 대기 상태에서 뷰포트를 클릭하고 `U` 키.

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

랙 3칸이 다 차면 `scan`으로 돌아가지 않고 아래로 넘어간다.

| 상태 | 하는 일 |
|---|---|
| `observe_go/tilt/settle/home` | 랙을 대각선 위에서 보고 ArUco로 칸별 긴급도를 읽어 출력한 뒤 홈 복귀 (7.의 긴급도 절 참고) |
| (홈 복귀 직후) | `/mission_state` 1 발행 — 주행 프로세스가 이걸 보고 출발한다 |
| `wait_unload` | 홈에서 대기. `/nav_done` 1 수신(또는 `U` 키)이면 하역 시작 |
| `unload` | 랙 **3번부터** 꺼내 `DESK_SLOTS`(base 기준)에 가로로 놓는다. 3칸 끝나면 종료 |

> `DESK_SLOTS`/`DESK_Z_M`은 **출발 책상**에서 실측한 base 기준 값이다. 도착 책상에서
> 그대로 맞는지는 도킹 후 `V` 키로 확인할 것 — 책상 높이나 도킹 정확도가 다르면 여기가 어긋난다.

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
      좌표는 카메라 광학 프레임(ROS 규약) 미터, **카메라에서 가까운 순** 정렬
      (왼쪽부터 집던 때는 왼쪽 트레이를 잡으러 들어가다 앞의 오른쪽 트레이를 건드렸다.
       정렬 키는 `cam_dist2` = x²+y²+z². 광축 z 만 보면 양옆으로 벌어진 트레이를 잘못 고른다.)
  /aruco_markers    std_msgs/Float32MultiArray
      data = [seq, n, id1,slot1, id2,slot2, ...]
      트레이 윗판 ArUco(DICT_4X4_50). id = 긴급도 0/1/2(하/중/상).
      slot = 관제 PC 가 RACK_ROI 를 가로 3등분한 칸 0/1/2 (화면 왼쪽 = 랙 1번).
      적재/하역과 무관 — 적재 완료 후 랙 관측 상태에서만 읽는다.

Isaac <──> 주행 프로세스 (through_pose_human_test.py)   둘 다 std_msgs/Float32MultiArray, data[0] 만 쓴다
  /mission_state  Isaac -> 주행 : 1 = 적재+관측 끝났다, 출발해도 된다 (매 틱 발행)
  /nav_done       주행 -> Isaac : 1 = 도킹까지 끝났다, 하역해도 된다 (5초간 반복 발행)
```

### 주행 핸드셰이크 (2026-09-22 추가)
- Isaac 쪽은 검출과 같은 제네릭 ROS2 노드(`pub_mission` / `sub_nav`)를 쓴다. 번들 파이썬에
  rclpy 가 없어서 노드를 따로 못 띄운다.
- **제네릭 `ROS2Subscriber` 는 마지막 값을 계속 들고 있다.** 그래서 `/nav_done` 은
  Stop 시점에 `clear_nav_done()` 으로 0 으로 지운다. 안 지우면 다음 Play 때 주행을 건너뛴다.
- `/nav_done` 을 5초간 반복 발행하는 이유: Isaac 쪽이 OmniGraph 구독이라 latched(transient
  local)를 못 믿는다. 한 번만 쏘면 놓칠 수 있다.

### 긴급도 ArUco 마커 (2026-09-22 추가)
- 마커 PNG: `isaacpjt/sdg/make_markers.py` → `isaacpjt/assets/markers/aruco_{0,1,2}.png` (흰 여백 포함 7셀, 검은 마커 6셀)
- 부착: `pick_and_place_detection.py`의 `attach_aruco()` — `spawn_tray_copies()`가 원본·복제본 전부에
  `random.choice(0,1,2)`로 붙인다. 손잡이 윗판(`handle/Cube_01`, 5×10cm, 윗면 z=0.1474 handle 기준) 위
  4.9cm 사각형 + `UsdPreviewSurface` 텍스처. 시각 전용(콜리전 없음). 배정 결과는 시작 로그 `aruco` 줄.
- 읽기: `detect_node.py`의 `read_markers()` → `/aruco_markers`로 따로 발행. 저장 이미지에 `aruco N`
  텍스트가 같이 찍힌다 — **마커가 안 읽히면 이 이미지부터 볼 것**.
- **스캔 자세에서는 못 읽는다** (앙각 ≈15°라 윗판 마커가 40×10px로 찌그러짐, 실측 27프레임 0검출).
  수직판(`Cube_02`)은 로봇 쪽에서 보면 기둥에 가려 불가. 그래서 **적재 완료 후 별도 관측 상태**로 읽는다.
- 관측 흐름 (`observe_go → observe_tilt → observe_settle → observe_home → wait_unload`):
  `RACK_SLOTS[1]`에서 로봇 쪽 `OBSERVE_BACK_M`(0.30), 위 `OBSERVE_UP_M`(0.35)로 이동(POINT4_RPY) →
  `build_descend_step` 재사용으로 `OBSERVE_PITCH_DEG`(−50°) 숙임 → `OBSERVE_COLLECT_FRAMES` 동안
  마커 메시지를 누적해 칸별 최빈 id (`assign_slots`/`merge_urgencies`) → 로그
  `observe  랙 1번: 긴급도 상(2)  2번: 미검출(-1)  3번: ...` → 홈 → `U` 대기.
  못 읽어도 `-1`로 찍고 진행한다. `OBSERVE_UP_M`은 0.35→0.30 실측 조정. 랙에 닿거나 IK 안 풀리면 이것부터.
- 관제 PC 는 `RACK_ROI`(관측 자세에서 랙이 보이는 픽셀 영역)만 `ARUCO_ZOOM`(3)배 키워서 읽는다 —
  원본 크기(~28px, 4px/셀)로는 2번 칸이 0/56 프레임이었다. **관측 자세를 바꾸면 저장 이미지의
  노란 ROI 사각형이 랙을 덮는지 보고 `RACK_ROI`를 맞출 것.** 칸 배정도 이 ROI 의 가로 3등분이다.
  마커를 3D 로 옮겨 `RACK_SLOTS` 와 x 거리를 재던 방식은 깊이 오차로 양옆 칸이 버려져([0,11,2]) 폐기.
- 마커가 읽힌 프레임은 `~/tray_detections/` 에 전부 저장된다(진단용). 필요 없어지면
  `_log_and_save` 의 `if markers or ...` 에서 `markers or` 를 빼면 된다.

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

### 사람(omni.anim.people)
- **확장·carb 설정만으로는 안 걷는다.** 명령을 읽어 실행하는 주체는 캐릭터 SkelRoot 에 붙은
  `character_behavior.py`(omni.kit.scripting BehaviorScript) 다. GUI People 확장의
  'Setup characters' 를 눌러야 USD 에 저장되는데 `integration_human.usd` 에는 **없다**
  (실측: `/World/Characters/Character{,_01}` 에 `omni:scripting:scripts` 도 `AnimationGraphAPI` 도 없다).
  → `setup_characters()` 가 실행할 때마다 붙인다. `load_scene()` **뒤**, `bake_navmesh()` 앞.
  `run_human_scene.py` / `pickup_place_go_nova.py` 에는 아직 이 단계가 없다 — 그쪽으로 사람을
  움직이려면 같은 함수를 옮겨 넣을 것.
- 순서: 확장 + carb 설정(씬 로드 **앞**) → 씬 로드(`is_stage_loading()` 이 끝날 때까지 대기) →
  캐릭터 연결 → navmesh 베이크 → Play. navmesh 가 없으면 `GoTo` 가 전부 invalid command 로 거부된다.
- `command.txt` 의 첫 토큰은 **캐릭터 프림 이름**이다 (`Character`, `Character_01`). 이름이 다르면 조용히 무시된다.
- `people/config.yaml` 은 Replicator Agent(UI) 가 읽는 파일이라 standalone 실행에서는 아무도 안 읽는다.

### 로봇 / IK
- **손목 특이점 — 회전 전 후퇴는 관절 공간으로 (2026-09-22)**: `로봇 쪽 후퇴(25cm)`는 자세를
  고정한 채 TCP 를 당기는 직교 보간이었다. 그러면 `joint_5 ≈ −(joint_2+joint_3)` 로 묶여
  **경로가 joint_5 = 0 을 관통**한다 (실측: `IK 거부 161틱, 25cm 중 7cm 만 가고 중단`).
  가드를 풀면 팔이 뒤틀리므로 가드 문제가 아니다. → `joint_retreat_step()` 으로 바꿨다.
  어깨(`RETREAT_SHOULDER_DEG`)/팔꿈치(`RETREAT_ELBOW_DEG`)를 접고 **`joint_5` 로 같은 양을 되돌려
  도구 기울기(= 트레이 기울기)를 유지**한다. IK 를 안 거치니 특이점이 없다. 접는 **부호는 FK 로
  양쪽을 미리 재서** 수평 도달거리가 줄어드는 쪽을 고른다. 크기는 실측 튜닝값 —
  로그 `retreat  +1: 수평 도달 0.870 -> 0.6xx m, z ...` 를 보고 맞출 것. 적재·하역이 같은 함수를 쓴다.
- **스텝이 실패해도 미션은 안 멈춘다 (2026-09-22)**: 예전엔 `sequence.failed` 면
  `pick_state = None` 으로 정지했다. 주행까지 묶인 뒤로는 그러면 안 된다.
  1) `wrist_unlock_step()` 으로 `joint_5` 를 0 에서 `WRIST_UNLOCK_DEG` 띄우고(joint_3 로 기울기 보상)
     **실패한 스텝부터** 재개 (`MAX_STEP_RETRIES` 회).
  2) 적재 계열이면 `begin_recover()` — 들고 있으면 **원래 집은 자리에 되돌려 놓고**
     (`build_return_steps`), 아니면 홈으로만 빠진 뒤 다음 트레이 스캔.
  3) 관측/하역이면 홈으로만 빼고 **상태는 유지** — 홈 복귀가 끝나면 원래 흐름이 그대로 이어진다.
  4) 적재를 `MAX_STAGE_FAILS` 회 실패하면 남은 칸을 포기하고 관측 → 주행 → 하역으로 넘어간다.
  검출 실패(`FRESH_TIMEOUT_FRAMES` / `MAX_DETECT_RETRIES` 초과)도 중단이 아니라 2) 로 간다.
  **복구 시퀀스로 갈아끼울 때 `sequence.gripper` 를 반드시 넘길 것** — `reset()` 이 `open` 으로
  시작하므로 안 넘기면 들고 있던 트레이를 그 자리에서 놓아버린다.
- **손목 특이점**: `joint_5`가 0 근처면 `joint_4`/`joint_6` 축이 일직선이 되어
  IK가 다른 해로 튀고 팔이 뒤틀린다. `SingularityGuardedIK`가 한 프레임에 20°
  넘게 튀는 해를 걸러낸다. 다만 이건 **안전망이지 해결이 아니다**.
- **스텝 도달 확인 (2026-09-22 추가)**: 예전엔 `PickPlaceSequence`가 틱 수만 세고 넘겨서,
  IK 가 계속 거부된 스텝(특히 `로봇 쪽 후퇴 25cm`)에서 팔이 제자리인데 다음 스텝(joint_1 회전)이
  시작됐다 — "올린 뒤 후퇴 없이 회전" 증상. 이제 pose 스텝 끝에 TCP 오차(`STEP_POS_TOL_M` 1cm /
  `STEP_ROT_TOL_DEG` 5°)를 재서 크면 `STEP_EXTRA_TICKS`(120) 더 기다리고, 그래도 못 가면
  `step N '…' 도달 실패 — 위치 오차 …, IK 거부 …틱` 을 찍고 **시퀀스를 중단**한다.
  바닥에 닿는 두 스텝(랙/책상 수직 하강)은 `tol=STEP_CONTACT_TOL_M`(3cm)으로 느슨하다.
  이 로그가 자주 뜨는 스텝이 있으면 그 구간에 경유점을 넣거나 관절 공간으로 바꿀 것.
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
