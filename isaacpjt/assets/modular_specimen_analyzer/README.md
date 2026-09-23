# 모듈형 검체 분석 장비

사용자가 제공한 긴 모듈형 분석 장비 사진의 외형을 참고해 Blender로 새로 만든 시각 소품입니다.
하부 흰색 캐비닛, 남색 받침, 곡선 프레임과 어두운 투명 커버, 중앙 이송 연결부,
검체 랙, 반응부, 피펫 및 출력 랙으로 구성했습니다.
내부 전체를 채운 덩어리가 아닌 패널형 외장과 단순화한 내부 기구로 제작했습니다.

## 제공 파일

- `modular_specimen_analyzer.blend`: 편집 가능한 Blender 파일. 렌더용 스튜디오 포함.
- `modular_specimen_analyzer.usd`: 장비만 포함하는 USD. 바닥, 조명, 카메라는 제외.
- `preview.png`: 사선에서 본 완성 모델 미리보기.
- `preview_front.png`: 사진과 비교하기 쉬운 정면 미리보기.
- `build_modular_analyzer.py`: 모델 생성 및 내보내기 스크립트.
- `model_info.json`: 내보낸 USD에서 확인한 크기와 메시 정보.

## 장면 배치

- 단위: 미터 / 위쪽: +Z / 전면: -Y
- 전체 모듈의 명목 폭: 6.5m / 캐비닛 깊이: 약 0.82m / 높이: 약 1.8m
- 돌출부를 포함한 USD 실제 경계 크기: 약 6.552 × 0.874 × 1.806m
- 메시: 삼각형 환산 약 123,958개, 재질 19개. USD 용량 약 1.1MB.
- 폭 중앙의 바닥 높이가 원점이며 받침 최하단은 Z=0입니다.
- 실제 제품의 도면이 아닌 사진 비율에 근거한 추정 크기입니다.
- USD 기본 프림은 `/LabAnalyzer`입니다.
- 외부 텍스처 파일 없이 USD Preview Surface 재질을 포함합니다.
- 충돌체, 강체, 모터, 실제 분석 기능은 설정하지 않은 외형용 소품입니다.

Isaac Sim의 미터 단위 장면에는 스케일 1.0으로 레퍼런스를 추가하고 원하는 위치로 옮기면 됩니다.
반투명 커버의 표현은 사용하는 렌더러의 투명 재질 지원 및 조명에 따라 다를 수 있습니다.

## Blender 편집

장비는 `ModularSpecimenAnalyzer` 컬렉션에, 미리보기 조명과 바닥은
`PreviewStudio_NOT_EXPORTED` 컬렉션에 들어 있습니다.

최상위 `ModularSpecimenAnalyzer` 아래에 다음 10개 모듈이 각각 분리되어 있습니다.

1. `01_Intake`: 검체 투입부
2. `02_Chemistry_A`: 분석 모듈 A
3. `03_Chemistry_B`: 분석 모듈 B
4. `04_TransportBridge`: 낮은 이송 연결부
5. `05_Immunoassay`: 분석 모듈 C
6. `06_TransferTower`: 검체 이동 모듈
7. `07_ReactionModule`: 반응부 및 내부 표시 패널
8. `08_WashStation`: 세척부 외형
9. `09_SampleProcessing`: 피펫 및 검체 처리부
10. `10_OutputRack`: 검체 배출 랙

모듈별 빈 오브젝트와 자식들을 함께 선택해 이동하거나 복제할 수 있습니다.
내부 형상을 자세히 보려면 `SmokedUpperDoor`, `SmokedLowerDoor` 메시를 숨깁니다.
표시된 모듈 용도는 외형 제작을 위한 이름이며 실제 제품의 기능 명세가 아닙니다.

## 재생성

```bash
/home/rokey/blender-5.2.2-linux-x64/blender --background --factory-startup \
  --python /home/rokey/cobot3_ws/isaacpjt/assets/modular_specimen_analyzer/build_modular_analyzer.py
```

같은 폴더의 출력 파일이 갱신됩니다. 다른 PC에서는 Blender와 스크립트 경로를 변경해 실행합니다.
이전에 만든 탁상형 장비와 기존 병원 장면은 수정하지 않습니다.
