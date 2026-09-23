#!/usr/bin/env python3
"""Replicator BasicWriter 출력 -> YOLO 데이터셋 변환. Isaac Sim 불필요.

    python3 isaacpjt/sdg/to_yolo.py isaacpjt/sdg/_out_tray isaacpjt/sdg/dataset_tray
"""
import glob
import json
import os
import shutil
import sys

import numpy as np

MAX_OCCLUSION = 0.9  # 이 이상 가려진 박스는 버림
VAL_EVERY = 10  # 10장 중 1장을 val 로


def boxes_to_yolo(bbox_npy, labels_json, w, h, names):
    """구조화 배열 -> ['<cls> <cx> <cy> <bw> <bh>', ...] (정규화, 화면 밖 클리핑)"""
    lines = []
    for row in bbox_npy:
        if "occlusionRatio" in bbox_npy.dtype.names and row["occlusionRatio"] > MAX_OCCLUSION:
            continue
        cls_name = labels_json[str(row["semanticId"])]["class"]
        if cls_name not in names:
            continue
        x0, y0 = max(0, int(row["x_min"])), max(0, int(row["y_min"]))
        x1, y1 = min(w, int(row["x_max"])), min(h, int(row["y_max"]))
        if x1 <= x0 or y1 <= y0:  # 완전 가림은 INT_MAX 등으로 나옴
            continue
        lines.append(
            f"{names.index(cls_name)} {(x0 + x1) / 2 / w:.6f} {(y0 + y1) / 2 / h:.6f} "
            f"{(x1 - x0) / w:.6f} {(y1 - y0) / h:.6f}"
        )
    return lines


def convert(src, dst, names=("tray",)):
    names = list(names)
    # 이전 변환 잔재가 섞이지 않도록 비운다. data.yaml 존재 = 이 스크립트가 만든 디렉터리라는 증거이므로
    # 경로를 잘못 줘도 남의 디렉터리를 지우지 않는다.
    if os.path.isfile(f"{dst}/data.yaml"):
        for sub in ("images", "labels"):
            shutil.rmtree(f"{dst}/{sub}", ignore_errors=True)
    elif os.path.isdir(dst) and os.listdir(dst):
        sys.exit(f"'{dst}' 가 비어있지 않고 이 스크립트의 출력도 아님. 직접 지우고 다시 실행할 것")

    for split in ("train", "val"):
        os.makedirs(f"{dst}/images/{split}", exist_ok=True)
        os.makedirs(f"{dst}/labels/{split}", exist_ok=True)

    npys = sorted(
        p
        for p in glob.glob(f"{src}/**/bounding_box_2d_tight_*.npy", recursive=True)
        if "_labels_" not in p and "_prim_paths_" not in p
    )
    if not npys:
        sys.exit(f"'{src}' 아래에 bounding_box_2d_tight npy 가 없음")

    kept = 0
    for i, npy in enumerate(npys):
        # BasicWriter 의 두 레이아웃 모두 이 치환으로 rgb 경로가 나옴
        rgb = npy.replace("bounding_box_2d_tight", "rgb").replace(".npy", ".png")
        labels_path = npy.replace("bounding_box_2d_tight_", "bounding_box_2d_tight_labels_").replace(".npy", ".json")
        if not (os.path.isfile(rgb) and os.path.isfile(labels_path)):
            continue

        from PIL import Image  # rgb 해상도 확인용 (ultralytics 가 이미 의존)

        with Image.open(rgb) as im:
            w, h = im.size
        with open(labels_path) as f:
            lines = boxes_to_yolo(np.load(npy), json.load(f), w, h, names)
        if not lines:  # 빈 프레임은 이미지째 제외
            continue

        split = "val" if kept % VAL_EVERY == 0 else "train"
        stem = f"{kept:06d}"
        link = f"{dst}/images/{split}/{stem}.png"
        if not os.path.lexists(link):
            os.symlink(os.path.abspath(rgb), link)
        with open(f"{dst}/labels/{split}/{stem}.txt", "w") as f:
            f.write("\n".join(lines) + "\n")
        kept += 1

    with open(f"{dst}/data.yaml", "w") as f:
        f.write(
            f"path: {os.path.abspath(dst)}\ntrain: images/train\nval: images/val\nnames:\n"
            + "".join(f"  {i}: {n}\n" for i, n in enumerate(names))
        )
    print(f"{kept}/{len(npys)} 프레임 사용 -> {dst}/data.yaml")


def demo():
    dt = np.dtype(
        [("semanticId", "<u4"), ("x_min", "<i4"), ("y_min", "<i4"), ("x_max", "<i4"), ("y_max", "<i4"), ("occlusionRatio", "<f4")]
    )
    rows = np.array(
        [
            (1, 100, 200, 300, 400, 0.1),  # 정상
            (1, 0, 0, 0, 0, 0.0),  # 면적 0
            (1, 10, 10, 50, 50, 0.95),  # 과도한 가림
            (1, -20, -20, 100, 100, 0.0),  # 화면 밖 클리핑
        ],
        dtype=dt,
    )
    out = boxes_to_yolo(rows, {"1": {"class": "tray"}}, 640, 640, ["tray"])
    assert out == [
        "0 0.312500 0.468750 0.312500 0.312500",
        "0 0.078125 0.078125 0.156250 0.156250",
    ], out
    print("demo ok")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "demo":
        demo()
    elif len(sys.argv) == 3:
        convert(sys.argv[1], sys.argv[2])
    else:
        sys.exit(__doc__)
