# 검체 채취실 3인용 워크스테이션

제공된 사진의 형태를 참고하여 새로 만든 외형 모델입니다. 제조사 원본 모델이나 실제 의료 장비 설계도가 아니며 치수는 사진 비율에서 추정했습니다.

## 파일

- `specimen_collection_workstation.blend`: 편집 가능한 Blender 원본과 촬영용 조명·카메라.
- `specimen_collection_workstation.usd`: 외부 텍스처가 필요 없는 USD 모델. 촬영용 바닥·카메라·조명 제외.
- `preview.png`, `preview_front.png`: 사선/정면 미리보기.
- `build_collection_workstation.py`: Blender에서 재생성할 수 있는 독립 스크립트.
- `model_info.json`: 실제 출력 치수, 메시 수, 단위, 의존성 검사 결과.

## 구성과 사용

왼쪽 수납·공급 캐비닛, 터치 단말, 긴 이송 작업대, 3개의 작업 단말과 상부 모니터, 3개의 이동식 선반으로 구성했습니다. 서랍 색상표, 키보드, 화면, 아크릴 칸막이, 모니터 암을 포함합니다. 화면은 개인정보가 없는 예시 도형입니다.

단위는 미터, 위쪽은 Z, 전면은 -Y이며 바퀴 하단은 Z=0입니다. USD를 장면에 불러오거나 Reference로 추가하면 됩니다. 기본 prim은 `/CollectionWorkstation`입니다. 각 스테이션과 선반은 별도 상위 객체로 묶여 있습니다.

물리 충돌, 관절, 서랍 개폐 애니메이션 및 의료 기능은 포함하지 않았습니다. 내부는 외형 표현에 필요한 부분만 구현했습니다. 로봇 시뮬레이션에서 장애물로 사용하려면 대상 시뮬레이터에서 단순화된 충돌체를 별도로 설정하세요.

Blender 실행 예:

```bash
/home/rokey/blender-5.2.2-linux-x64/blender --background --factory-startup --python /home/rokey/cobot3_ws/isaacpjt/assets/specimen_collection_workstation/build_collection_workstation.py
```

스크립트를 다시 실행하면 같은 폴더의 생성 파일들을 덮어씁니다.
