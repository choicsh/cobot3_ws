#!/usr/bin/env python3
"""tray 합성 데이터셋으로 YOLO26s 학습.

    python3 isaacpjt/sdg/train_yolo.py [data.yaml] [epochs]

증강(HSV/flip/mosaic/scale 등)은 ultralytics 기본값을 그대로 쓴다.
조명/배경/카메라 각도 randomization 은 이미 Replicator 단계에서 끝났음.
"""
import sys

from ultralytics import YOLO

data = sys.argv[1] if len(sys.argv) > 1 else "isaacpjt/sdg/dataset_tray/data.yaml"
epochs = int(sys.argv[2]) if len(sys.argv) > 2 else 150

YOLO("yolo26s.pt").train(
    data=data,
    epochs=epochs,
    imgsz=640,
    batch=32,  # 16GB VRAM 기준. OOM 나면 16
    cache="ram",  # 1000장 규모는 전부 RAM 적재 가능
    patience=30,  # 기본 100 은 사실상 early stop 이 안 걸림
    # optimizer=auto 는 lr0/momentum 을 무시하고, iterations = ceil(n_train/64)*epochs 가
    # 10000 을 넘는 순간 AdamW@0.002 에서 MuSGD@0.01 로 튄다 (trainer.py:1136-1144).
    # 데이터나 에폭을 늘려도 LR 이 안 바뀌도록 auto 가 고르던 값을 그대로 고정한다.
    optimizer="AdamW",
    lr0=0.002,  # = 0.002 * 5 / (4 + nc), nc=1
    momentum=0.9,  # AdamW 에서는 beta1. 기본값 0.937 은 SGD 용이라 명시 필요
    project="isaacpjt/sdg/runs",
    name="tray",
)
