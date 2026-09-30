# -*- coding: utf-8 -*-
"""임시 가정 관측 범위 (사용자 결정 2026-09-30): 드론이 목표 칸 위를 원으로 한 바퀴 돌았다고 보고 90×90m.
실제 원 비행·경로 증거가 아니라 가정이다 (UAV-08 계약 전까지). 원래 범위(TEST_THERMAL_90M_V1)는 보존한다."""

import importlib

import pytest

from orchestrator import config

from .conftest import submit
from .test_area_completion import BIG, _ev, _map, _monitor, _target, _visit, THERMAL


@pytest.fixture
def orbit(monkeypatch):
    monkeypatch.setattr(config, "DEFAULT_SENSOR_PROFILE_ID", "ASSUMED_ORBIT_90M_V1")


def test_default_profile_is_orbit_assumption_and_original_is_kept(monkeypatch):
    monkeypatch.delenv("ORCH_SENSOR_PROFILE", raising=False)
    fresh = importlib.reload(config)
    try:
        assert fresh.DEFAULT_SENSOR_PROFILE_ID == "ASSUMED_ORBIT_90M_V1"
        orig = fresh.SENSOR_PROFILES["TEST_THERMAL_90M_V1"]
        assert (orig["agl_m"], orig["footprint_width_m"], orig["footprint_height_m"]) == (90.0, 52.0, 42.0)
        assert fresh.SENSOR_PROFILES["ASSUMED_ORBIT_90M_V1"]["status"] == "TEST_ONLY"
    finally:
        importlib.reload(config)


def test_area_monitor_of_90m_cell_completes_in_one_visit_and_records_the_assumption(world, orbit):
    orch, lg = world["orch"], world["ledger"]
    _map(world, BIG)
    task = _monitor(orch, ["BIG"], c=BIG)
    _visit(orch, task.task_id)
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    obs = _ev(lg, task.task_id, "OBSERVATION")[0]["detail"]
    assert obs["sensor_profile_id"] == "ASSUMED_ORBIT_90M_V1" and obs["covered_cells"] == ["BIG"]
    fp = obs["footprint"]
    assert (fp["width_m"], fp["height_m"]) == (90.0, 90.0) and fp["method"] == "ASSUMED_FIXED_FOOTPRINT"
    assert "ASSUMED_ONE_ORBIT" in fp["assumption"] and "ASSUMED_ONE_ORBIT" in obs["footprint_basis"]
    assert obs["sensor_status"] == "TEST_ONLY" and obs["source"] == "SIMULATED"


def test_recon_without_fire_on_90m_cell_is_observed_clear_under_orbit_assumption(world, orbit):
    orch, lg = world["orch"], world["ledger"]
    _map(world, BIG)
    task = submit(orch, "RECON-BIG", target=_target(BIG), requirements=THERMAL)
    _visit(orch, task.task_id)
    assert orch.kb.fire_states(1e9)["BIG"]["status"] == "OBSERVED_CLEAR"
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
