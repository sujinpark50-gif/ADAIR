# -*- coding: utf-8 -*-
"""관측 계획 밖 UGV 열화상 임무도 지상 시선 모델(config.UGV_VIEW, 600 m ~ 2 km, 사용자 결정 2026-10-08)로 관측한다.

UGV 는 도착 위치만 보고하고, 무엇이 보였는지는 총괄 모의 센서(observation.simulate_ground_los)가 환경 정답에서 읽는다.
"""
import math

import pytest

from orchestrator import config
from orchestrator.engine import Orchestrator
from orchestrator.env_adapter import FixtureEnv
from orchestrator.knowledge import InjectedAnalysis
from orchestrator.ledger import Ledger
from orchestrator.resources import UavClient, UgvClient

from .conftest import TARGET, submit
from .fakes import FakeUav, FakeUgv, normal_flight

REQ = {"resource_types": ["UGV"], "sensor": "THERMAL"}
N, MID, CELL_M = 41, 20, 90.0          # 41×41 칸(약 3.7 km) 격자, 가운데 칸(20_20) = TARGET, 불


def _grid_cells():
    dlat = CELL_M / 111_320.0
    dlon = CELL_M / (111_320.0 * math.cos(math.radians(TARGET["lat"])))
    fire, rest = [], []
    for c in range(N):
        for r in range(N):
            cell = {"cell_id": f"{c}_{r}", "lat": TARGET["lat"] + (MID - r) * dlat, "lon": TARGET["lon"] + (c - MID) * dlon,
                    "cell_size_m": CELL_M, "ground_amsl_m": 600.0}
            if (c, r) == (MID, MID):
                fire.append({**cell, "fire_state": "BURNING"})
            else:
                rest.append({**cell, "fire_state": "UNBURNED"})
    return fire, rest, dlat


@pytest.fixture
def world(tmp_path):
    """conftest.world 와 같되, 시선 모델이 읽을 수 있는 격자 지도(칸 이름 col_row, 지면 높이)를 쓴다."""
    fire, rest, _ = _grid_cells()
    env = FixtureEnv(fire_cells=fire, risk_cells=rest)
    uav, ugv = FakeUav(), FakeUgv()
    uav.default_script = normal_flight()
    ledger = Ledger(str(tmp_path / "state.sqlite3"))
    orch = Orchestrator(ledger, env, UavClient(uav.endpoints(), uav.client()), UgvClient("http://ugv", ugv.client()),
                        analysis=InjectedAnalysis())
    return {"uav": uav, "ugv": ugv, "env": env, "ledger": ledger, "orch": orch}


@pytest.fixture
def thermal_on(monkeypatch):
    sensors = tuple(dict.fromkeys(tuple(config.SIMULATED_CAPABILITIES["UGV"]) + ("THERMAL",)))
    monkeypatch.setattr(config, "SIMULATED_CAPABILITIES", {**config.SIMULATED_CAPABILITIES, "UGV": sensors})


def _ugv_reports_arrival(world, position):
    world["ugv"].default_script = lambda rid: [
        {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
        {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
         "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "RP:test:0.0", "position": position},
         "_unit": {"state": "READY", "current_task_id": None}}]


def _run(world, position, polls=4):
    orch, lg = world["orch"], world["ledger"]
    _ugv_reports_arrival(world, position)
    task = submit(orch, requirements=REQ)
    out = orch.dispatch(task.task_id)
    for _ in range(polls):
        orch.poll()
    thermal = [e["detail"] for e in lg.events(task.task_id)
               if e["event_type"] == "OBSERVATION" and (e["detail"] or {}).get("sensor_type") == "THERMAL"]
    return out, task, thermal


def test_unplanned_ugv_thermal_uses_ground_line_of_sight_model(world, thermal_on):
    out, task, obs = _run(world, {"lat": TARGET["lat"] - 4 * _grid_cells()[2], "lon": TARGET["lon"]})   # 4칸(360 m) 남쪽
    assert out["resource_id"] == "A-ugv1"
    assert len(obs) == 1
    o = obs[0]
    assert o["sensor_profile_id"] == config.UGV_VIEW["sensor_profile_id"] == "ASSUMED_GROUND_LOS_V1"
    assert o["sensor_status"] == "TEST_ONLY"
    assert o["result"] == "DETECTED"                                   # min_m(600 m) 안은 항상 보인다


def test_unplanned_ugv_thermal_beyond_max_range_sees_nothing(world, thermal_on):
    _, task, obs = _run(world, {"lat": TARGET["lat"] - 19 * _grid_cells()[2], "lon": TARGET["lon"] + 19 * _grid_cells()[2]})  # 남 1.7 km·동 1.3 km = 약 2.2 km (2 km 밖)
    assert len(obs) == 1 and obs[0]["result"] != "DETECTED"
    assert world["ledger"].get_task(task.task_id).purpose_status != "COMPLETED"


def test_unplanned_ugv_thermal_without_position_fails(world, thermal_on):
    _, task, obs = _run(world, {"lat": None, "lon": None})
    assert len(obs) == 1
    assert obs[0]["result"] == "FAILED" and obs[0]["failure_reason"] == "POSITION_UNKNOWN"


def test_old_square_ground_profile_is_gone():
    assert "ASSUMED_GROUND_VIEW_V1" not in config.SENSOR_PROFILES
    assert not hasattr(config, "GROUND_SENSOR_PROFILE_ID")
