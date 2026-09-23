# 검체 처리 장비 소품 — LAB / 03

제공된 사진의 탁상형 실험실 장비 외형을 참고해 Blender로 새로 제작한 모델입니다.
사진의 원본 모델을 내려받거나 화면 이미지를 텍스처로 사용하지 않았습니다.

## 파일

- `specimen_processor.blend`: 편집 가능한 Blender 5.2 파일. 미리보기 카메라와 스튜디오 포함.
- `specimen_processor.usd`: 장비만 포함한 USD 바이너리. 외부 텍스처나 조명·바닥·카메라 없음.
- `preview.png`: 완성 모델의 렌더 미리보기.
- `build_specimen_processor.py`: Blender에서 모델을 다시 만드는 스크립트.
- `model_info.json`: USD에서 확인한 크기, 폴리곤 수, 좌표계 정보.

## 구성과 크기

흰색 패널 외장, 속이 빈 스테인리스 챔버, 180개 실제 구멍이 있는 트레이,
103도 열린 덮개, 2개 조작 다이얼, 상태 표시등, 통풍구, 고무 받침,
별도로 숨길 수 있는 시험관 6개로 구성됩니다.

- 미터 단위, Z-up, 전면 방향 -Y
- 원점: 장비 중앙의 바닥 높이. 받침 최하단 Z=0
- 본체 폭 약 0.64m, 열린 덮개를 포함한 전체 높이 약 0.703m
- 돌출 조작부와 열린 덮개를 포함한 전체 크기: 약 0.644 × 0.630 × 0.703m
- 실제 기기의 규격을 측정한 것이 아닌 사진 기반의 추정 크기
- USD: 약 82,702개 삼각형에 해당하는 메시, 14개 재질

## Blender에서 편집

`specimen_processor.blend`를 열면 카메라 구도로 장비를 볼 수 있습니다.
`SpecimenProcessor` 컬렉션이 장비이며, `PreviewStudio_NOT_EXPORTED`는 렌더 전용입니다.

- 덮개: `LidHinge_ROTATE_LOCAL_X`의 로컬 X 회전. 기본 -103도, 0도는 닫힌 방향.
- 트레이: `RemovablePerforatedTray`와 그 자식들.
- 시험관: `OptionalSpecimenTubes_HIDE_OR_DELETE`의 자식들. 모두 선택해 숨기거나 제거 가능.
- 조작부: `FrontControls`의 자식들.

메시 재질은 단순한 Principled BSDF로 구성되며 USD에는 USD Preview Surface로 저장됩니다.

## Isaac Sim / USD 장면에서 사용

`specimen_processor.usd`를 USD 레퍼런스로 추가해 사용합니다.
기본 프림은 `/LabEquipment`이며, 필요한 다른 텍스처 파일은 없습니다.
사용하는 장면과 이 파일의 단위가 모두 미터이면 스케일 1.0으로 배치합니다.

시각 소품으로 제작했으므로 충돌체, rigid body, 관절, 실제 분석 기능은 설정하지 않았습니다.
기존 병원 장면 파일은 수정하지 않았습니다.

## 재생성

```bash
/home/rokey/blender-5.2.2-linux-x64/blender --background --factory-startup \
  --python /home/rokey/cobot3_ws/isaacpjt/assets/specimen_processor/build_specimen_processor.py
```

스크립트 실행 시 같은 폴더의 모델과 미리보기가 다시 생성됩니다.
다른 PC에서는 Blender 실행 경로와 스크립트 경로를 해당 PC에 맞게 바꾸면 됩니다.
