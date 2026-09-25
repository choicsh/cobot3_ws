"""책상/경로 이름 — 시스템 이름(collection/analysis) 과 주행 코드 이름(lab/specimen) 의 대응은 여기 한 곳.

주행 코드(nav_to_goal/hospital_mission, hospital_docking)는 East_DockDesk 를 'lab', West_DockDesk 를
'specimen' 이라 부른다. 요구사항은 East = 검체 채취실(적재), West = 검체 분석실(하역)이라 뜻이 반대다.
물리 경로는 그대로 쓰고 이름만 옮긴다 (docs/SYSTEM_INTEGRATION_PLAN.md §2.4).
"""

COLLECTION = "collection"   # East_DockDesk — 트레이 적재 + 긴급도 판독
ANALYSIS = "analysis"       # West_DockDesk — 트레이 하역

# (출발, 도착) -> hospital_mission route_id
MISSION_ROUTE = {
    (COLLECTION, ANALYSIS): "lab_to_specimen",   # lane_lower, 운송
    (ANALYSIS, COLLECTION): "specimen_to_lab",   # lane_upper, 복귀
}


def mission_route(origin, destination):
    try:
        return MISSION_ROUTE[(origin, destination)]
    except KeyError:
        raise ValueError(f"no route {origin} -> {destination}; known: {list(MISSION_ROUTE)}") from None


# hospital_mission route_id -> 차선 그래프(lane_graph) 출발/도착 노드
ROUTE_NODES = {
    "lab_to_specimen": ("collection_dock", "analysis_dock"),
    "specimen_to_lab": ("analysis_dock", "collection_dock"),
}
