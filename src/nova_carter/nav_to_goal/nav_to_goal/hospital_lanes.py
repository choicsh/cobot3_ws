"""병원 차선 기하 — 방(도킹·언도킹, 한 방향) + 가운데 복도(양방향 4개). ROS 없이 import 한다.

    채취실(East) 방  --EAST_JUNCTION(10, 12.5)--  복도 4개  --WEST_JUNCTION(-34.8, 12.5)--  분석실(West) 방

방 구간은 한 방향이다: 책상 도킹에서 나와 문 바깥 분기점까지(OUT), 분기점에서 정류장까지(IN). 두 방 모두 문을
한 줄(y=12.5)로 지나고 방 안은 사각형 한 바퀴다(서쪽은 동쪽을 180도 돌린 모양).
복도(TRACKS)는 두 분기점을 잇고 양방향으로 쓴다 — 본선 upper(y=14.9), lower(y=-1.5) 와 2 m 옆 예비선
upper_reserve(y=16.9, 북쪽), lower_reserve(바깥쪽: x=9, y=-3.5, x=-33.55). 기하는 서 -> 동 방향으로 적고
동 -> 서는 뒤집어 쓴다. 관제(hospital_system.lane_graph)가 출발 때 복도 하나를 골라 준다.

경로 = 출발 방 OUT + 복도 + 도착 방 IN. 주행 단계: 출발(DWB, 원호 포함) / 복도 긴 직선(MPPI, 사람 회피) / 도착(DWB).
MPPI 로 넘기기 전 1.5 m 는 DWB 직선(방향을 맞춘다), 도착 단계는 원호로 시작한다 — 그래서 긴 직선을 방향마다
다른 곳에서 자른다.
"""
import math

# Existing NewRooms desks, unchanged (1.8 m x 2.4 m, long sides north-south).
# The robot docks beside a long side with the desk on its right, like the
# USD spawn at the lab desk. *_DOCK must match hospital_docking.TABLES; each
# *_STATION is the staging pose 2.5 m before the dock on the same straight.
# The lab is entered heading north and left northward; the specimen desk is
# entered heading south and left southward, so neither needs an undock.
LAB_DOCK = (20.185, 13.8125)
LAB_STATION = (20.185, 11.3125)
SPECIMEN_DOCK = (-44.741, 12.9125)
SPECIMEN_STATION = (-44.741, 15.4125)
# 서쪽 문(x≈-35.9)은 동쪽 문(y=12.5)처럼 두 차선이 한 줄로 지난다. 방 안은 사각형 한 바퀴
# (도킹 줄 x=-44.741 남행 → 아래 y=8.9 동행 → x=-39.5 북행 → 위 y=18.0125 서행), 분기점은 (-38.0, 12.5).
# 동쪽 방(도킹 줄 x=20.185, 분기점 (13.5, 12.5))을 180도 돌린 모양이다 (2026-09-26 사용자 요청).
WEST_DOOR_Y = 12.5
WEST_LOOP_X = -39.5
# 문 바깥 분기점: 여기서 방 구간과 복도가 만난다 (문 줄 y=12.5 위)
EAST_JUNCTION = (10.0, 12.5)
WEST_JUNCTION = (-34.8, WEST_DOOR_Y)
HANDOVER_M = 1.5          # MPPI 직선 앞 DWB 직선

# 아크끼리 바로 잇지 않고(아크-직선-아크) 한 아크는 90도 이하로 둔다. 도착 경로는
# 정류장 앞 1.6 m 직진으로 끝나 제자리 정렬 없이 TableDocking 직진으로 이어진다.
COLLECTION_OUT = [('line', LAB_DOCK, (20.185, 16.5)),
                  ('arc', (18.685, 16.5), 1.5, 0, 90),
                  ('line', (18.685, 18), (16, 18)),
                  ('arc', (16, 16.55), 1.45, 90, 180),
                  ('line', (14.55, 16.55), (14.55, 13.95)),
                  ('arc', (13.1, 13.95), 1.45, 0, -90),
                  ('line', (13.1, 12.5), EAST_JUNCTION)]
COLLECTION_IN = [('line', EAST_JUNCTION, (13.5, 12.5)),
                 ('arc', (13.5, 11.5), 1.0, 90, 0),
                 ('line', (14.5, 11.5), (14.5, 9.8)),
                 ('arc', (15.5, 9.8), 1.0, 180, 270),
                 ('line', (15.5, 8.8), (19.285, 8.8)),
                 ('arc', (19.285, 9.7), 0.9, 270, 360),
                 ('line', (20.185, 9.7), LAB_STATION)]
ANALYSIS_OUT = [('line', SPECIMEN_DOCK, (-44.741, 10.4)),
                ('arc', (-43.241, 10.4), 1.5, 180, 270),
                ('line', (-43.241, 8.9), (-41.0, 8.9)),
                ('arc', (-41.0, 10.4), 1.5, 270, 360),
                ('line', (WEST_LOOP_X, 10.4), (WEST_LOOP_X, 11.0)),
                ('arc', (-38.0, 11.0), 1.5, 180, 90),
                ('line', (-38.0, WEST_DOOR_Y), WEST_JUNCTION)]
ANALYSIS_IN = [('line', WEST_JUNCTION, (-38.0, WEST_DOOR_Y)),
               ('arc', (-38.0, WEST_DOOR_Y + 1.5), 1.5, 270, 180),
               ('line', (WEST_LOOP_X, WEST_DOOR_Y + 1.5), (WEST_LOOP_X, 16.5125)),
               ('arc', (WEST_LOOP_X - 1.5, 16.5125), 1.5, 0, 90),
               ('line', (WEST_LOOP_X - 1.5, 18.0125), (-43.741, 18.0125)),
               ('arc', (-43.741, 17.0125), 1.0, 90, 180),
               ('line', (-44.741, 17.0125), SPECIMEN_STATION)]

# 복도 (서 -> 동): pre = WEST_JUNCTION 에서 긴 직선 시작까지, straight = (시작, 끝), post = 직선 끝에서 EAST_JUNCTION 까지
TRACKS = {
    # 위 본선 — 복귀 차선이던 y=14.9
    "upper": {
        "pre": [('arc', (-34.8, 13.4), 0.9, 270, 360),
                ('line', (-33.9, 13.4), (-33.9, 14.0)),
                ('arc', (-33.0, 14.0), 0.9, 180, 90)],
        "straight": ((-33.0, 14.9), (7.9, 14.9)),
        "post": [('arc', (7.9, 14.0), 0.9, 90, 0),
                 ('line', (8.8, 14.0), (8.8, 13.4)),
                 ('arc', (9.7, 13.4), 0.9, 180, 270),
                 ('line', (9.7, 12.5), EAST_JUNCTION)],
    },
    # 위 예비선 — 본선 북쪽 2 m (남쪽은 x -24..-21 에서 2.45 m 밖에 안 비었다)
    "upper_reserve": {
        "pre": [('arc', (-34.8, 13.4), 0.9, 270, 360),
                ('line', (-33.9, 13.4), (-33.9, 16.0)),
                ('arc', (-33.0, 16.0), 0.9, 180, 90)],
        "straight": ((-33.0, 16.9), (7.9, 16.9)),
        "post": [('arc', (7.9, 16.0), 0.9, 90, 0),
                 ('line', (8.8, 16.0), (8.8, 13.4)),
                 ('arc', (9.7, 13.4), 0.9, 180, 270),
                 ('line', (9.7, 12.5), EAST_JUNCTION)],
    },
    # 아래 본선 — 운송 차선이던 x=-31.55 / y=-1.5 / x=7
    "lower": {
        "pre": [('line', WEST_JUNCTION, (-34.55, WEST_DOOR_Y)),
                ('arc', (-34.55, WEST_DOOR_Y - 3), 3, 90, 0),
                ('line', (-31.55, WEST_DOOR_Y - 3), (-31.55, 1.5)),
                ('arc', (-28.55, 1.5), 3, 180, 270)],
        "straight": ((-28.55, -1.5), (4, -1.5)),
        "post": [('arc', (4, 1.5), 3, 270, 360),
                 ('line', (7, 1.5), (7, 9.5)),
                 ('arc', (10, 9.5), 3, 180, 90)],
    },
    # 아래 예비선 — 본선 바깥 2 m (안쪽 x=5 는 벽까지 0.1 m 라 못 쓴다)
    "lower_reserve": {
        "pre": [('line', WEST_JUNCTION, (-34.55, WEST_DOOR_Y)),
                ('arc', (-34.55, WEST_DOOR_Y - 1), 1.0, 90, 0),
                ('line', (-33.55, WEST_DOOR_Y - 1), (-33.55, 1.5)),
                ('arc', (-28.55, 1.5), 5, 180, 270)],
        "straight": ((-28.55, -3.5), (4, -3.5)),
        "post": [('arc', (4, 1.5), 5, 270, 360),
                 ('line', (9, 1.5), (9, 11.5)),
                 ('arc', (10, 11.5), 1.0, 180, 90)],
    },
}
TO_ANALYSIS = "to_analysis"        # 채취실 -> 분석실 (운송), 복도를 동 -> 서로
TO_COLLECTION = "to_collection"    # 분석실 -> 채취실 (복귀), 복도를 서 -> 동으로
DEFAULT_TRACK = {TO_ANALYSIS: "lower", TO_COLLECTION: "upper"}   # 관제 없이 돌 때 (예전 lane_lower / lane_upper)


def reverse(segments):
    """경로 원소를 거꾸로 — 직선은 끝점을, 원호는 시작/끝 각을 바꾼다."""
    out = []
    for seg in reversed(segments):
        if seg[0] == 'line':
            out.append(('line', seg[2], seg[1]))
        else:
            out.append(('arc', seg[1], seg[2], seg[4], seg[3]))
    return out


def _toward(a, b, d):
    """a 에서 b 쪽으로 d 간 점"""
    length = math.dist(a, b)
    return (a[0] + (b[0] - a[0]) * d / length, a[1] + (b[1] - a[1]) * d / length)


def corridor(track, direction):
    """복도 원소 목록과 MPPI 직선의 인덱스. 긴 직선의 앞 1.5 m 는 DWB 직선으로 떼고, 뒤쪽은 도착 원호로 바로 잇는다."""
    t = TRACKS[track]
    a, b = t["straight"]
    if direction == TO_COLLECTION:
        pre, post = t["pre"], t["post"]
    else:
        pre, post, a, b = reverse(t["post"]), reverse(t["pre"]), b, a
    mid = _toward(a, b, HANDOVER_M)
    segs = pre + [('line', a, mid), ('line', mid, b)] + post
    return segs, len(pre) + 1


def compose(direction, track):
    """(원소 목록, (MPPI 시작, MPPI 끝)) — 출발 방 OUT + 복도 + 도착 방 IN.

    원소 [:i] 출발(DWB), [i:j] 복도 긴 직선(MPPI), [j:] 도착(DWB)."""
    segs, transit = corridor(track, direction)
    out, room_in = ((ANALYSIS_OUT, COLLECTION_IN) if direction == TO_COLLECTION
                    else (COLLECTION_OUT, ANALYSIS_IN))
    i = len(out) + transit
    return out + segs + room_in, (i, i + 1)


def segments_length(segments):
    return sum(math.dist(s[1], s[2]) if s[0] == 'line' else s[2] * abs(math.radians(s[4] - s[3]))
               for s in segments)
