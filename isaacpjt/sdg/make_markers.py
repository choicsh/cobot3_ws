#!/usr/bin/env python3
"""트레이 긴급도 ArUco 마커 PNG 생성 (1회 실행).  id 0/1/2 = 긴급도 하/중/상.

  ~/yolo-venv/bin/python isaacpjt/sdg/make_markers.py

DICT_4X4_50, 테두리 1셀 포함 6셀 + 사방 0.5셀 흰 여백 = 7셀. 이미지를 한 변 L 의
정사각형에 붙이면 검은 마커 한 변은 L*6/7 (solvePnP 쓸 때 markerLength)."""
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).resolve().parents[1] / "assets/markers"
CELL_PX = 50
IDS = (0, 1, 2)

if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for i in IDS:
        m = cv2.aruco.generateImageMarker(d, i, 6 * CELL_PX)
        m = cv2.copyMakeBorder(m, *[CELL_PX // 2] * 4, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(str(OUT / f"aruco_{i}.png"), m)
        print(OUT / f"aruco_{i}.png", m.shape)
