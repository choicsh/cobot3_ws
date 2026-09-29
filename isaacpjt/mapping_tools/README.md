# 중앙 방 + 리셉션 데스크 맵 제외본

## 열 파일

- 새 USD: `../assets/hospital_integration_human_map_excluded.usd`
- 검증용 정적 맵: `../../src/nova_carter/carter_navigation/maps/hospital_integration_human_map_excluded.yaml`
- 같은 이름의 PNG가 맵 이미지입니다.

원본 `hospital_integration_human.usd`, 기존 PNG/YAML, 참조 자산은 수정하지 않았습니다. 새 USD는 원본과 같은 폴더에 저장하여 기존 상대 참조 경로를 유지합니다. 다른 PC로 옮길 때는 기존 참조 자산도 필요합니다.

## 수정 내용

1. 중앙 방 8개 바닥 타일이 차지하는 범위(x -24.84~3.16, y 5.35~8.85m)를 맵에서 제외합니다. 바닥과 방 내부 지오메트리는 보존했습니다.
2. `/World/hospital/ReceptionDesk` 그룹을 새 USD에서만 비활성화합니다. 데스크와 그 그룹에 포함된 부속물 자리(x -24.95~-19.44, y 9.07~12.50m)는 회색으로 나옵니다.
3. `/World/OccupancyMapExclusions` 아래에 눈에 보이지 않는 10cm 두께의 충돌 경계를 추가했습니다. 채워진 박스가 아니라 속이 빈 경계여서 내부는 unknown, 테두리는 occupied가 됩니다. 양쪽 새 방의 작업대는 수정하지 않았습니다.

**이 USD는 맵 생성용 사본입니다.** 보이지 않는 충돌 경계가 있으므로 실제 로봇 운용 장면을 대신하는 용도로 쓰지 마세요. 원본 시뮬레이션에는 경계가 없습니다.

## Isaac Sim에서 다시 맵 생성

현재 열려 있는 원본 대신 **새 USD**를 여세요. 원본에 저장하지 마세요.

| 항목 | X | Y | Z |
|---|---:|---:|---:|
| Origin | 0 | 0 | 0 |
| Lower Bound | -49.50 | -9.10 | 0.05 |
| Upper Bound | 24.95 | 23.15 | 1.00 |

- Cell Size: `0.05`
- **Use PhysX Collision Geometry: ON** (필수: 숨겨진 충돌 경계를 사용합니다)
- CALCULATE → VISUALIZE IMAGE, ROS 방향에 맞춰 `Rotate Image: 180°`.
- unknown 색은 회색, freespace 흰색, occupied 검정색.
- 로봇/사람 등 움직이는 물체를 정적 맵에 남기지 않으려면 맵 생성 시에만 제외하세요.

바닥을 지우는 것만으로 unknown 영역을 지정할 수 없습니다. 바닥이 없는 테스트 장면에서도 열린 공간은 free, 닫힌 경계 내부는 unknown으로 생성되는 것을 실제 Isaac Sim으로 확인했습니다.

## 검증 범위와 주의

`static_collision_validation.usdc`는 새 USD에서 활성 정적 충돌 형상 350개를 추출한 검증 장면입니다. 원래 메시 좌표·월드 변환·충돌 근사 종류를 유지하며, 로봇·사람·트레이·트래픽콘은 제외했습니다. 원격 인물 자산을 로드하거나 현재 GUI 장면을 변경하지 않고 검증하기 위함입니다. 따라서 제공 PNG는 이 **정적 형상 검증용 맵**이며 전체 GUI 장면의 동적 객체는 나타나지 않습니다.

PNG는 Isaac Sim의 실제 Occupancy Map 확장에서 생성한 버퍼를 ROS 방향으로 180° 회전해 저장했습니다. 회색을 손으로 덧칠한 이미지가 아닙니다. 해상도는 0.05m, 실제 출력 크기는 1488×645px입니다. 이 설치의 생성기가 요청 bounds의 가로 셀 수를 절삭하므로 수식상 1489px와 1px 차이가 있습니다. PNG/YAML을 항상 한 쌍으로 사용하세요.

YAML origin은 이 설치의 UI `compute_coordinates()` 내보내기 규칙(min bound + 반 셀)에 맞춘 `[-49.475, -9.075000381469727, 0]`입니다. 패널의 Lower Bound를 그대로 YAML origin에 복사하지 마세요.

`map_validation.json`에는 영역별 unknown/free/occupied 개수가 기록됩니다. 두 제외 영역 내부에 free 셀이 0개인지, PNG가 원본 생성 버퍼와 같은지, 원본 USD SHA-256이 보존됐는지 자동 검증했습니다. 방 내부 기존 벽/가구는 검정색으로 남을 수 있습니다.

실행 도구:

- `prepare_excluded_stage.py`: 새 USD 및 정적 충돌 추출. 출력 USD가 이미 있으면 덮어쓰지 않고 중단.
- `validate_excluded_map.py`: Isaac Sim의 Python 환경에서 실제 맵 생성. 이 작업이 생성한 검증 PNG/YAML만 갱신.
- `verify_excluded_result.py`: 시스템 Python(numpy/Pillow)에서 결과 재검증.
- `test_unknown_region.py`: 바닥 없이 닫힌 경계 내부가 unknown이 되는 최소 재현 예제.

사용한 설치에서는 headless Kit 종료 시 X 연결/종료 오류가 발생할 수 있어, 프로세스 종료 코드만이 아니라 저장된 버퍼와 `verify_excluded_result.py`의 검증 통과 여부를 확인했습니다. 기존 GUI Isaac Sim 세션은 조작하지 않았습니다.
