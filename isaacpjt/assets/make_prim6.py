"""hospital_integration.usd -> hospital_integration_v2.usd (원본은 손대지 않음)

사용자 확정 요구사항:
1. 가운데 작은 방 8칸(Geo_M2_Floor2~8, Geo_M2_Floor_582) -> 4칸으로. 각 칸을 이웃과 합쳐 폭을 2배(약 7.0m)로
   늘린다 (관제방 등 큰 물건이 들어갈 수 있게). y(안쪽 깊이 3.5m)는 건드리지 않는다.
2. 흰색 바깥 벽 사각형(ClosingWalls + 그 안의 병원 동/서 폭)만 가로(x) 폭을 현재의 2/3 로 줄인다.
   남북(y, 복도 폭)은 절대 건드리지 않는다. 빨간 동/서 끝방(East_Room/West_Room) 자체 크기도 그대로 -
   위치만 좁아진 벽을 따라 안쪽으로 당긴다.
3. 동/서 끝방을 바깥벽 전체의 y 중심(≈7.15)에 맞춰 남/북 대칭으로 재배치 (현재는 북쪽 모서리에 치우쳐 있음).
4. 그 외에는 어떤 것도 옮기거나 바꾸지 않는다.

실행: ~/blender-5.2.2-linux-x64/blender -b --python make_prim6.py
"""
import math
from pxr import Usd, UsdGeom, Sdf, Gf

SRC, DST = 'hospital_integration.usd', 'hospital_integration_v2.usd'
WIDTH_RATIO = 2.0 / 3.0

stage = Usd.Stage.Open(SRC)
layer = stage.GetRootLayer()
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'], useExtentsHint=True)


def ops_of(prim):
    return {o.GetOpName(): o for o in UsdGeom.Xformable(prim).GetOrderedXformOps()}


def bbox(path):
    bc.Clear()
    r = bc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


def set_scalar(op, val):
    """op 의 기존 타입(double/float, Vec3 등)에 맞춰 값을 넣는다."""
    tn = op.GetAttr().GetTypeName()
    cur = op.Get()
    if tn == Sdf.ValueTypeNames.Double3:
        op.Set(Gf.Vec3d(*val))
    elif tn == Sdf.ValueTypeNames.Float3:
        op.Set(Gf.Vec3f(*val))
    else:
        raise TypeError(tn)


# ============================================================ 0. 현재 좌표 읽기 (하드코딩 없이 라이브로)
ew_mn, ew_mx = bbox('/World/ClosingWalls/East_Wall')
ww_mn, ww_mx = bbox('/World/ClosingWalls/West_Wall')
nw_mn, nw_mx = bbox('/World/ClosingWalls/North_Wall')
sw_mn, sw_mx = bbox('/World/ClosingWalls/South_Wall')
ew_ops = ops_of(stage.GetPrimAtPath('/World/ClosingWalls/East_Wall'))
ww_ops = ops_of(stage.GetPrimAtPath('/World/ClosingWalls/West_Wall'))
EW_CX = ew_ops['xformOp:translate'].Get()[0]
WW_CX = ww_ops['xformOp:translate'].Get()[0]
Y_CENTER = ((sw_mn[1] + sw_mx[1]) / 2 + (nw_mn[1] + nw_mx[1]) / 2) / 2   # 바깥벽 전체 y 중심 (≈7.15)
print(f'현재 East_Wall x={EW_CX:.3f}  West_Wall x={WW_CX:.3f}  span={EW_CX - WW_CX:.3f}  y중심={Y_CENTER:.3f}')

OLD_SPAN = EW_CX - WW_CX
NEW_SPAN = OLD_SPAN * WIDTH_RATIO
SHRINK_EACH = (OLD_SPAN - NEW_SPAN) / 2
NEW_EW_CX = EW_CX - SHRINK_EACH
NEW_WW_CX = WW_CX + SHRINK_EACH
print(f'-> 목표 span={NEW_SPAN:.3f} (2/3), 양쪽 각 {SHRINK_EACH:.3f} 씩 안으로. new East x={NEW_EW_CX:.3f} West x={NEW_WW_CX:.3f}')

# ============================================================ 1. 동/서 벽: x 위치만 이동 (두께/길이 불변)
for wall, new_cx in [('East_Wall', NEW_EW_CX), ('West_Wall', NEW_WW_CX)]:
    o = ops_of(stage.GetPrimAtPath('/World/ClosingWalls/' + wall))
    t = o['xformOp:translate'].Get()
    set_scalar(o['xformOp:translate'], (new_cx, t[1], t[2]))
    print(f'{wall}: x {t[0]:.3f} -> {new_cx:.3f} (위치만, 두께/길이 그대로)')

# ============================================================ 2. 남/북 벽: 길이(scale.x)만 줄이고, 새 동/서 벽에 맞춰 재배치
# 이 두 벽은 orient 로 ~90도 돌아 있어서 world 의 x(길이) 는 로컬 scale.y 가 담당하고
# scale.x 는 두께다 (East/West_Wall 의 scale.x=0.15=두께 와 동일 값인 것으로 확인). 그래서 길이는 scale[1] 만 바꾼다.
for wall in ('North_Wall', 'South_Wall'):
    p = stage.GetPrimAtPath('/World/ClosingWalls/' + wall)
    o = ops_of(p); t = o['xformOp:translate'].Get(); s = o['xformOp:scale'].Get()
    overhang_w = t[0] - s[1] / 2 - WW_CX     # 기존에 서쪽 벽 중심선을 얼마나 더 넘어갔는지 (모서리 이음새)
    overhang_e = (t[0] + s[1] / 2) - EW_CX
    new_len = NEW_SPAN + overhang_w + overhang_e
    new_cx = (NEW_WW_CX + overhang_w + NEW_EW_CX + overhang_e) / 2
    set_scalar(o['xformOp:scale'], (s[0], new_len, s[2]))
    set_scalar(o['xformOp:translate'], (new_cx, t[1], t[2]))
    print(f'{wall}: 길이 {s[1]:.3f} -> {new_len:.3f}, 중심 x {t[0]:.3f} -> {new_cx:.3f} (두께 x={s[0]:.3f} 그대로)')

# ============================================================ 3. 병원 바닥판: x 만 새 동/서 폭에 맞춰 축소 (기존 관행과 동일하게 이 프림만 스케일)
fl = stage.GetPrimAtPath('/World/hospital/Geo_Floor_Costum22_95')
o = ops_of(fl); t = o['xformOp:translate'].Get(); s = o['xformOp:scale'].Get()
mn, mx = bbox('/World/hospital/Geo_Floor_Costum22_95')
old_dx = mx[0] - mn[0]
# 바닥판 로컬 원점이 동쪽 끝(East 근처)에 있고 -x 방향으로 뻗는 구조(기존 파일들에서 확인된 패턴)
new_dx = NEW_SPAN + (mx[0] - EW_CX) + (WW_CX - mn[0])   # 벽 바깥까지 살짝 덮던 여유는 비율 유지
ratio = new_dx / old_dx
set_scalar(o['xformOp:scale'], (s[0] * ratio, s[1], s[2]))
set_scalar(o['xformOp:translate'], (t[0] - SHRINK_EACH, t[1], t[2]))
print(f'Floor: x scale x{ratio:.4f} ({old_dx:.2f} -> {new_dx:.2f} m), origin x {t[0]:.3f} -> {t[0] - SHRINK_EACH:.3f}')

# ============================================================ 4. 동/서 끝방: 위치만 이동 (x: 좁아진 벽 따라, y: 전체 중심으로 대칭)
for room, delta_x in [('East_Room', -SHRINK_EACH), ('West_Room', SHRINK_EACH)]:
    p = stage.GetPrimAtPath(f'/World/NewRooms/{room}')
    o = ops_of(p); t = o['xformOp:translate'].Get()
    set_scalar(o['xformOp:translate'], (t[0] + delta_x, Y_CENTER, t[2]))
    print(f'{room}: x {t[0]:.3f} -> {t[0] + delta_x:.3f}, y {t[1]:.3f} -> {Y_CENTER:.3f} (크기는 그대로, 위치만)')

# ============================================================ 5. 가운데 작은 방 8 -> 4 (각 폭 2배, 이웃과 합침)
# 짝: (Floor8+Floor7)->room1, (Floor6+Floor5)->room2, (Floor4+Floor3)->room3, (Floor2+Floor_582)->room4
PAIRS = [
    ('Geo_M2_Floor8', 'Geo_M2_Floor7'),
    ('Geo_M2_Floor6', 'Geo_M2_Floor5'),
    ('Geo_M2_Floor4', 'Geo_M2_Floor3'),
    ('Geo_M2_Floor2', 'Geo_M2_Floor_582'),
]
for keep, drop in PAIRS:
    kp = f'/World/hospital/{keep}'; dp = f'/World/hospital/{drop}'
    kmn, kmx = bbox(kp); dmn, dmx = bbox(dp)
    new_center = (min(kmn[0], dmn[0]) + max(kmx[0], dmx[0])) / 2
    old_center = (kmn[0] + kmx[0]) / 2
    o = ops_of(stage.GetPrimAtPath(kp)); t = o['xformOp:translate'].Get(); s = o['xformOp:scale'].Get()
    set_scalar(o['xformOp:scale'], (s[0] * 2.0, s[1], s[2]))
    set_scalar(o['xformOp:translate'], (t[0] + (new_center - old_center), t[1], t[2]))
    stage.GetPrimAtPath(dp).SetActive(False)
    print(f'방 합침: {keep}(유지,폭2배,중심{old_center:.2f}->{new_center:.2f}) + {drop}(비활성화)')

# 이웃과 합쳐지며 내부에 남는 칸막이벽 제거 (room1|room2 경계였던 -14.4 부근, 이제 room2 내부가 됨)
# (다른 칸막이 -10.9, -3.9 는 새 4방 경계와 거의 일치하므로 그대로 둔다)
removed = 0
for k in stage.GetPrimAtPath('/World/hospital').GetChildren():
    n = k.GetName()
    if n in ('Geo_M2_BaseWallSide7', 'Geo_M2_BaseWallSide8'):
        k.SetActive(False); removed += 1
print(f'내부 칸막이 벽 {removed}개 비활성화 (room2 내부, -14.4 부근)')

# room1|room2 사이(x=-17.84)에는 원래 칸막이가 없었으므로 새로 하나 복사해서 세운다
src_partition = '/World/hospital/Geo_M2_BaseWallSide4'   # 정상적인 칸막이 한 장 복사 (두께/재질/높이 그대로)
new_partition = '/World/hospital/Geo_M2_BaseWallSide_new_17_84'
assert Sdf.CopySpec(layer, Sdf.Path(src_partition), layer, Sdf.Path(new_partition))
po = ops_of(stage.GetPrimAtPath(new_partition)); pt = po['xformOp:translate'].Get()
TARGET_X = -17.84
src_mn, src_mx = bbox(src_partition)
src_cx = (src_mn[0] + src_mx[0]) / 2
set_scalar(po['xformOp:translate'], (pt[0] + (TARGET_X - src_cx), pt[1], pt[2]))
print(f'새 칸막이 벽 추가: {src_partition} 복사 -> x={TARGET_X} (room1|room2 경계)')

# ============================================================ 6. 검증
print('\n--- 최종 확인 ---')
for p in ['/World/ClosingWalls/East_Wall', '/World/ClosingWalls/West_Wall', '/World/ClosingWalls/North_Wall',
          '/World/ClosingWalls/South_Wall', '/World/NewRooms/East_Room', '/World/NewRooms/West_Room']:
    mn, mx = bbox(p)
    print(f'{p:32s} x[{mn[0]:7.2f},{mx[0]:7.2f}] y[{mn[1]:7.2f},{mx[1]:7.2f}]')
print('새 East_Wall~West_Wall span:', bbox('/World/ClosingWalls/East_Wall')[0][0] - bbox('/World/ClosingWalls/West_Wall')[1][0])
for keep, drop in PAIRS:
    mn, mx = bbox(f'/World/hospital/{keep}')
    print(f'{keep:16s} (합쳐진 방) x[{mn[0]:7.2f},{mx[0]:7.2f}] dx={mx[0]-mn[0]:.2f}')

stage.GetRootLayer().Export(DST)
print('\nsaved', DST)
