# admin_ws — 관제 PC 워크스페이스

Isaac Sim 이 도는 PC 와 **같은 저장소**에서 관리하되, 실행은 관제 PC 에서 한다.

## 역할

```
Isaac PC                                    관제 PC (여기)
  /wrist_camera/color/image_raw    ──►        YOLO 검출
  /wrist_camera/depth/image_raw    ──►        깊이 역투영
  /wrist_camera/color/camera_info  ──►        내부 파라미터
  /tray_detection                  ◄──        [n, x,y,z,conf ...]
```

검출 결과는 **카메라 광학 프레임**(x 우, y 하, z 전방) 미터 단위이고,
**화면 왼쪽부터 정렬**해서 보낸다. 로봇이 왼쪽 트레이부터 집는다.

좌표를 월드/베이스로 바꾸는 일은 Isaac 쪽이 한다 — 카메라 프림의 월드 트랜스폼을
가진 쪽이 거기이기 때문이다. 관제 PC 는 TF 를 몰라도 된다.

## 준비

두 PC 의 `ROS_DOMAIN_ID` 가 같아야 하고 같은 서브넷이어야 한다.

```bash
python3 -m venv ~/yolo-venv && ~/yolo-venv/bin/pip install ultralytics
```

학습된 가중치를 관제 PC 로 복사한다 (기본 경로 `~/tray_best.pt`):

```bash
scp <IsaacPC>:~/cobot3_ws/runs/detect/isaacpjt/sdg/runs/tray-2/weights/best.pt ~/tray_best.pt
```

## 빌드 / 실행

ROS2 패키지로 빌드해서 쓰는 경우:

```bash
cd ~/cobot3_ws/admin_ws && colcon build && source install/setup.bash
```
```bash
ros2 run tray_detector detect_node
```

`colcon build` 로 만든 환경에는 ultralytics 가 없으므로, venv 를 쓰려면 스크립트를
직접 돌리는 편이 간단하다:

```bash
source /opt/ros/jazzy/setup.bash && PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH \
    ~/yolo-venv/bin/python ~/cobot3_ws/admin_ws/src/tray_detector/tray_detector/detect_node.py ~/tray_best.pt
```

## 자체 검증

Isaac 도 ROS2 도 없이 역투영/깊이 표본/인코딩 변환의 부호를 확인한다:

```bash
~/yolo-venv/bin/python src/tray_detector/tray_detector/detect_node.py demo
```

## 확인 순서

토픽이 흐르는지 먼저 본다. 여기서 안 나오면 검출을 봐야 소용없다.

```bash
ros2 topic hz /wrist_camera/color/image_raw /wrist_camera/depth/image_raw
```
```bash
ros2 topic echo /tray_detection --once
```
