from datetime import datetime

import pytest

from hospital_system.records import trays_from_load


def test_only_seated_slots_become_trays():
    assert trays_from_load([True, False, True], [3, 2, 1]) == [(1, 3), (3, 1)]


def test_missing_urgency_is_lowest():
    assert trays_from_load([True, True, True], [2]) == [(1, 2), (2, 1), (3, 1)]


def test_nothing_loaded():
    assert trays_from_load([False] * 3, [1, 1, 1]) == []


def test_tray_id_format():
    db = pytest.importorskip("hospital_system.db")   # psycopg2 / redis 가 있어야 import 된다
    assert db.tray_id_for(42, 3, datetime(2026, 9, 26)) == "TR20260926-42-S3"
