# 트레이 Pick & Place + 긴급도 관측 — 전체 알고리즘 구조도

작성 2026-09-22. 상세 설명·좌표계·튜닝값은 `HANDOFF_tray_detection.md`.

## 1. 프로세스 / 통신

```mermaid
flowchart LR
    subgraph ISAAC["Isaac Sim (python.sh · Py3.11)  pick_and_place_detection.py"]
        CAM[손목 카메라<br/>Camera_OmniVision_OV9782_Color<br/>640x480, frameSkip 4]
        SM[상태기계 main loop<br/>world.step → sync_base_pose → 상태 → sequence.tick]
        IK[Lula IK (base_link 기준)<br/>+ SingularityGuardedIK]
        SEQ[PickPlaceSequence<br/>pose/hold/joint 스텝 실행기<br/>+ 도달 확인]
        SM --> SEQ --> IK
    end
    subgraph NAV["주행 (Jazzy rclpy)  nav2_human_test.launch.py + through_pose_human_test.py"]
        N2[Nav2 스택<br/>map_server · AMCL · planner/controller<br/>pointcloud_to_laserscan → /scan]
        MIS[주행 미션<br/>undock 3m → goThroughPoses 4점 → dock 5m<br/>StraightDriver = cmd_vel 직진]
        N2 --- MIS
    end
    subgraph ADMIN["관제 PC (~/yolo-venv · Py3.12)  detect_node.py"]
        YOLO[YOLO26s best.pt<br/>conf ≥ 0.85]
        DEPTH[박스 깊이<br/>하위 15퍼센타일<br/>게이트 0.15~1.0m]
        ARUCO[ArUco DICT_4X4_50<br/>RACK_ROI 크롭 ×3 확대<br/>칸 = ROI 가로 3등분]
        SAVE[(~/tray_detections/*.png<br/>마커 프레임 전부 + 10장마다 1장)]
        YOLO --> DEPTH
    end
    CAM -- "/wrist_camera/color/image_raw (rgb8)" --> YOLO
    CAM -- "/wrist_camera/depth/image_raw (32FC1 m)" --> DEPTH
    CAM -- "/wrist_camera/color/camera_info" --> DEPTH
    CAM -- color --> ARUCO
    DEPTH -- "/tray_detection<br/>[seq, n, x,y,z,conf …] 광학 프레임, 가까운 순" --> SM
    ARUCO -- "/aruco_markers<br/>[seq, n, id,slot …]" --> SM
    YOLO --> SAVE
    ARUCO --> SAVE
    SM -- "/mission_state<br/>1 = 적재+관측 완료, 출발해도 된다" --> MIS
    MIS -- "/nav_done<br/>1 = 도킹 완료, 하역해도 된다 (5초간 반복)" --> SM
```

주행 프로세스가 따로인 이유는 검출과 같다 — Isaac 번들 파이썬(3.11)에서 rclpy 를 못 쓴다.
Isaac 쪽 핸드셰이크는 검출과 같은 제네릭 ROS2 노드(`pub_mission` / `sub_nav`)다.

## 2. Isaac 상태기계

```mermaid
stateDiagram-v2
    [*] --> init: python.sh 실행
    init: check_math → setup_people → load_scene → setup_characters<br/>→ bake_navmesh → spawn_tray_copies(+attach_aruco) → register_robot → IK → 그래프
    init --> scan: Play

    state "적재 (slot_index 0→2)" as LOAD {
        scan: joint_1 −90° 상대 회전
        descend: base z=0.11 로 하강 + 카메라 −30° 숙임
        settle1: 90프레임 정지 + seq 신선(≥2) 대기<br/>find_first_tray_optical (깊이 게이트)
        center: 검출 x 만큼 카메라 수평축 이동
        settle2: 재검출 → first_tray_grasp<br/>optical→world→base, +TRAY_HALF_DEPTH, reach 게이트
        pick: build_pick_steps 12스텝 실행<br/>회전 전 후퇴는 관절 공간 (손목 특이점 회피)
        scan --> descend
        descend --> settle1
        settle1 --> center: 검출됨
        settle1 --> settle1: 못 찾음 (재시도 ≤3)
        center --> settle2
        settle2 --> pick: 파지점 확정
        settle2 --> settle2: 못 찾음 (재시도 ≤3)
        pick --> scan: 칸 남음 (slot_index++)
    }
    settle1 --> recover: 재시도 초과 / 300프레임 검출 없음
    settle2 --> recover: 재시도 초과
    pick --> recover: 손목 풀고 재시도해도 실패
    recover: 들고 있으면 원래 자리에 되돌려 놓고, 아니면 홈으로<br/>미션은 안 멈춘다
    recover --> scan: 칸 남음 (실패 < 2회)
    recover --> observe_go: 실패 2회 — 남은 칸 포기

    state "긴급도 관측 (적재/하역과 무관)" as OBS {
        observe_go: RACK_SLOTS[1] 기준 −y 0.30, +z 0.30, POINT4_RPY
        observe_tilt: 현재 자세에서 −50° 숙임 (build_descend_step 재사용)
        observe_settle: 첫 신선 메시지부터 90프레임 투표<br/>assign_slots → merge_urgencies(칸별 최빈 id)
        observe_home: 홈 복귀 + 로그 출력 + /mission_state 1 발행
        observe_go --> observe_tilt
        observe_tilt --> observe_settle
        observe_settle --> observe_home
    }
    pick --> observe_go: 랙 3칸 참
    observe_home --> wait_unload

    wait_unload: 홈에서 /nav_done 1 대기 (U 키로 수동 진행 가능)
    state "주행 프로세스 (별도 프로세스)" as NAVP {
        undock: cmd_vel 직진 3m (회전 불가 구간)
        through: goThroughPoses 경유지 4개 (사람 회피)
        dock: 마지막 경유지 1m 이내 확인 후 cmd_vel 직진 5m
        undock --> through
        through --> dock
    }
    wait_unload --> NAVP: /mission_state 1
    NAVP --> wait_unload: /nav_done 1
    state "하역 (slot 3→1)" as UNLOAD {
        unload: build_unload_steps 13스텝 실행
        unload --> unload: 다음 칸
    }
    wait_unload --> unload: /nav_done 1 또는 U
    unload --> [*]: 3칸 완료 (Stop → Play 로 처음부터.<br/>Stop 시 /mission_state 0 + clear_nav_done)
    unload --> [*]: 스텝 도달 실패 → 중단
```

## 3. 적재 시퀀스 (build_pick_steps) / 하역 시퀀스 (build_unload_steps)

```mermaid
flowchart TB
    subgraph PICK["적재 pick — 파지점 grasp_base(검출) → RACK_SLOTS[slot]"]
        direction TB
        P0[0 안전 위치<br/>파지점 −8.2cm, 그리퍼 open] --> P1[1 파지 위치] --> P2[2 hold 그리퍼 닫기 120틱]
        P2 --> P3[3 들어올리기 z=0.45] --> P4[4 로봇 쪽 후퇴 25cm<br/>★ IK 거부 잦은 구간] --> P5[5 joint_1 회전<br/>파지 방위각 → 랙 방위각]
        P5 --> P6[6 슬롯 위 +11cm] --> P7[7 hold 1s] --> P8[8 수직 하강 tol 3cm] --> P9[9 hold 그리퍼 열기]
        P9 --> P10[10 수평 후퇴 15cm] --> P11[11 홈 복귀 joint]
    end
    subgraph UNL["하역 unload — RACK_SLOTS[slot] → DESK_SLOTS[slot]"]
        direction TB
        U0[0 랙 앞 후퇴점, open] --> U1[1 랙 진입] --> U2[2 hold 닫기] --> U3[3 수직 +11cm]
        U3 --> U4[4 로봇 쪽 후퇴 25cm] --> U5[5 joint_1 회전<br/>랙 → 책상 방위각] --> U6[6 책상 위 후퇴 반경]
        U6 --> U7[7 책상 위] --> U8[8 수직 하강 tol 3cm] --> U9[9 hold 열기] --> U10[10 후퇴]
        U10 --> U11[11 후퇴 반경으로 올리기] --> U12[12 홈 복귀 joint]
    end
```

## 4. 스텝 실행기 (PickPlaceSequence.tick)

```mermaid
flowchart TB
    T0[tick] --> T1{스텝 시작?}
    T1 -- 예 --> T2[_enter_step: 현재 플랜지 실측 → start<br/>n_steps = 거리/속도]
    T1 -- 아니오 --> T3
    T2 --> T3{타입}
    T3 -- pose/hold --> T4[코사인 S-커브 보간<br/>tcp_to_flange → IK]
    T4 --> T5{IK solved?}
    T5 -- 예 --> T6[apply_action]
    T5 -- 아니오<br/>특이점 가드/도달불가 --> T7[지령 없음, step_rejects++]
    T3 -- joint --> T8[관절각 보간 → apply_action]
    T6 --> T9
    T7 --> T9
    T8 --> T9
    T9[그리퍼 지령] --> T10{step_tick ≥ n_steps?}
    T10 -- 아니오 --> T0
    T10 -- 예, pose --> T11{TCP 오차 ≤ tol?<br/>1cm / 5° (접촉 스텝 3cm)}
    T10 -- 예, hold/joint --> T13
    T11 -- 예 --> T13[다음 스텝]
    T11 -- 아니오, 추가 120틱 이내 --> T12[목표 계속 지령] --> T0
    T11 -- 아니오, 초과 --> T14[failed 사유 기록 → 시퀀스 종료<br/>main 이 pick_state=None 으로 중단]
    T13 --> T15{마지막 스텝?}
    T15 -- 예 --> T16[done]
    T15 -- 아니오 --> T0
```

## 5. 관제 PC 프레임 처리 (detect_node._on_color)

```mermaid
flowchart TB
    C0[color 프레임 수신] --> C1{depth · camera_info 있음?}
    C1 -- 아니오 --> C0
    C1 -- 예 --> C2[YOLO 추론 conf ≥ 0.85]
    C2 --> C3[박스마다: 하위 15퍼센타일 깊이<br/>0.15~1.0m 게이트 → 역투영]
    C3 --> C4[카메라에서 가까운 순 정렬, 최대 3개<br/>/tray_detection 발행]
    C0 --> C5[RACK_ROI 크롭 ×3 → ArUco]
    C5 --> C6[마커 중심 x → ROI 3등분 칸<br/>/aruco_markers 발행]
    C4 --> C7{검출 또는 마커 있음?}
    C6 --> C7
    C7 -- 예 --> C8[로그: 내용 바뀔 때만<br/>저장: 마커 프레임 전부, 그 외 10장마다]
```
