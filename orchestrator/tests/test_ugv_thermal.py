# -*- coding: utf-8 -*-
"""UGV 지상 열화상 (모의, ORCH_UGV_THERMAL=1 일 때만). 2026-10-05.

- 능력: 켜면 UGV 가 THERMAL 임무 후보가 된다. 끄면(기본) 지금처럼 REQUIRED_CAPABILITY_UNAVAILABLE.
- 관측: 고도 대신 차 위치 중심 고정 범위(ASSUMED_GROUND_VIEW_V1, 기본 450 m)로 환경 정답을 읽는다.
- 범위 밖이면 탐지하지 못하고 임무는 완료되지 않는다 (지어내지 않음).
"""
import pytest

from orchestrator import config

from .conftest import TARGET, submit

THERMAL_UGV = {"resource_types": ["UGV"], "sensor": "THERMAL"}


@pytest.fixture
def ugv_thermal(monkeypatch):
    caps = dict(config.SIMULATED_CAPABILITIES)
    caps["UGV"] = tuple(caps["UGV"]) + ("THERMAL",)
    monkeypatch.setattr(config, "SIMULATED_CAPABILITIES", caps)


def _arrive_at(ugv, lat, lon):
    ugv.default_script = lambda rid: [
        {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
        {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
         "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1",
                         "position": {"lat": lat, "lon": lon}},
         "_unit": {"state": "READY", "current_task_id": None}}]


def _thermal_obs(lg, task_id):
    return [e["detail"] for e in lg.events(task_id)
            if e["event_type"] == "OBSERVATION" and (e["detail"] or {}).get("sensor_type") == "THERMAL"]


def _poll(orch, n=4):
    for _ in range(n):
        orch.poll()


def test_ugv_excluded_for_thermal_by_default(world):
    out = world["orch"].dispatch(submit(world["orch"], requirements=THERMAL_UGV).task_id)
    assert out["status"] == "HOLD"
    assert out["excluded"] == {"A-ugv1": "REQUIRED_CAPABILITY_UNAVAILABLE"}


def test_ugv_thermal_detects_fire_within_ground_view(world, ugv_thermal):
    orch, lg = world["orch"], world["ledger"]
    _arrive_at(world["ugv"], TARGET["lat"] - 0.0015, TARGET["lon"])      # 불난 칸 남쪽 약 170 m 도로
    task = submit(orch, requirements=THERMAL_UGV)
    out = orch.dispatch(task.task_id)
    assert out["resource_id"] == "A-ugv1"
    _poll(orch)
    obs = _thermal_obs(lg, task.task_id)
    assert len(obs) == 1
    o = obs[0]
    assert o["sensor_profile_id"] == "ASSUMED_GROUND_VIEW_V1" and o["sensor_status"] == "TEST_ONLY"
    assert o["result"] == "DETECTED" and o["target_covered"]
    assert o["footprint"]["agl_m"] is None and o["footprint"]["width_m"] == 450.0
    assert o["footprint_center"] == {"lat": TARGET["lat"] - 0.0015, "lon": TARGET["lon"]}
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"


def test_ugv_thermal_out_of_view_does_not_complete(world, ugv_thermal):
    orch, lg = world["orch"], world["ledger"]
    _arrive_at(world["ugv"], TARGET["lat"] - 0.01, TARGET["lon"])        # 약 1.1 km 떨어진 도로
    task = submit(orch, requirements=THERMAL_UGV)
    orch.dispatch(task.task_id)
    _poll(orch)
    o = _thermal_obs(lg, task.task_id)[0]
    assert o["result"] == "NO_COVERAGE" and not o["target_covered"] and o["detections"] == []
    assert lg.get_task(task.task_id).purpose_status != "COMPLETED"


def test_ugv_thermal_without_position_fails_without_inventing(world, ugv_thermal):
    orch, lg = world["orch"], world["ledger"]
    world["ugv"].default_script = lambda rid: [
        {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
         "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1"},
         "_unit": {"state": "READY", "current_task_id": None}}]
    task = submit(orch, requirements=THERMAL_UGV)
    orch.dispatch(task.task_id)
    _poll(orch)
    o = _thermal_obs(lg, task.task_id)[0]
    assert o["result"] == "FAILED" and o["failure_reason"] == "POSITION_UNKNOWN"
    assert lg.get_task(task.task_id).purpose_status != "COMPLETED"
