"""ㄷ자 동선(차선) 선호 마스크 생성 -> lane_mask.png / lane_mask.yaml

global_costmap 의 KeepoutFilter 가 이 마스크를 읽어 차선 밖에 '치명적이지 않은' 비용을 얹는다.
planner 는 평소엔 차선 가운데로 경로를 만들고, 차선이 막히면 비용을 감수하고 벗어나 우회한다.

    python3 make_lane_mask.py            # 맵(intergration_nova.yaml)과 같은 크기/원점으로 생성

마스크 값: 흰색(255)=0% 비용(차선), 회색(153)=40% 비용(차선 밖). 벽/책상은 정적 맵이 담당하므로 여기선 무시.
"""
import os
import yaml
import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_YAML = os.path.join(HERE, 'intergration_nova.yaml')
OUT_PNG = os.path.join(HERE, 'lane_mask.png')
OUT_YAML = os.path.join(HERE, 'lane_mask.yaml')

LANE_HALF_WIDTH = 1.3        # [m] 로봇 폭 1.0 + 여유. 코너에서도 planner 가 차선 안에서 호를 그릴 수 있게
OFF_LANE_PERCENT = 40        # 차선 밖 비용 (0~99). 100 이면 진입 금지가 되어 우회를 못 한다

# 차선 중심선 (map 좌표, m). move_test.py 의 route 와 같은 동선.
LANES = [
    [(1.3, 3.363), (-1.6, 3.363), (-2.2, 2.7), (-2.2, 1.2), (-1.6, 0.6), (5.5, 0.6),   # 책상 탈출 -> U턴 -> 아래 방
     (7.0, 2.1), (7.0, 17.7), (5.5, 19.163), (1.2, 19.163)],                            # 복도 -> 도킹 라인
]


def main():
    with open(MAP_YAML) as f:
        m = yaml.safe_load(f)
    res = float(m['resolution'])
    ox, oy = float(m['origin'][0]), float(m['origin'][1])
    w, h = Image.open(os.path.join(HERE, m['image'])).size

    def to_px(p):
        return ((p[0] - ox) / res, h - (p[1] - oy) / res)

    off = int(round(255 * (1 - OFF_LANE_PERCENT / 100)))
    img = Image.new('L', (w, h), off)
    draw = ImageDraw.Draw(img)
    r = LANE_HALF_WIDTH / res
    for lane in LANES:
        pts = [to_px(p) for p in lane]
        draw.line(pts, fill=255, width=int(round(2 * r)), joint='curve')
        for x, y in pts:
            draw.ellipse([x - r, y - r, x + r, y + r], fill=255)
    img.save(OUT_PNG)

    with open(OUT_YAML, 'w') as f:
        yaml.safe_dump({
            'image': os.path.basename(OUT_PNG),
            'mode': 'scale',
            'resolution': res,
            'origin': [ox, oy, 0.0],
            'negate': 0,
            'occupied_thresh': 1.0,
            'free_thresh': 0.0,
        }, f, sort_keys=False)

    a = np.array(img)
    print(f'{OUT_PNG}: {w}x{h}, lane px {(a == 255).sum()}, off-lane value {off} ({OFF_LANE_PERCENT}%)')


if __name__ == '__main__':
    main()
