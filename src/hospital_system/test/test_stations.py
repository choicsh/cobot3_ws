import pytest

from hospital_system.stations import ANALYSIS, COLLECTION, mission_route


def test_routes_map_to_hospital_mission_ids():
    # East(collection) = 주행 코드의 'lab', West(analysis) = 'specimen'
    assert mission_route(COLLECTION, ANALYSIS) == "lab_to_specimen"
    assert mission_route(ANALYSIS, COLLECTION) == "specimen_to_lab"


def test_unknown_route():
    with pytest.raises(ValueError):
        mission_route(COLLECTION, COLLECTION)
