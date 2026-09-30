"""긴급도 -> 작업/로봇 우선순위 (docs/SYSTEM_INTEGRATION_PLAN.md §3.3).

점수 = (긴급도 3 개수, 2 개수, 1 개수) 를 사전식으로 비교, 클수록 먼저.
    [3,1,1] > [2,2,2] (높은 것이 있다),  [3,3,1] > [3,2,2] (높은 것이 많다).
트레이를 싣지 않은 로봇은 (0, 0, 0) — 실은 로봇이 문·합류 구역을 먼저 지난다.
"""


def score(urgencies):
    """실려 있는 트레이들의 긴급도 목록(1~3) -> 비교용 튜플"""
    u = list(urgencies)
    return (u.count(3), u.count(2), u.count(1))


def loaded_score(loaded, urgency):
    """arm/status 의 loaded[k], urgency[k] -> 실린 칸만 센 점수"""
    return score(urgency[k] for k, ok in enumerate(loaded) if ok and k < len(urgency))
