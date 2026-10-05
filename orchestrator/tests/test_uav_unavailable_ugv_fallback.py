# -*- coding: utf-8 -*-
"""시나리오: 드론이 모두 갈 수 없을 때 총괄이 UGV 를 골라 보낸다.

현장 기상 측정(ENV_SENSE/WEATHER) 은 드론·UGV 둘 다 할 수 있는 임무다.
(열화상 정찰은 UGV 에 센서가 없어 대신 보낼 수 없다 — test_live_ugv_not_sent_for_thermal_task)
드론 2대가 강풍으로 거절하면 UGV 가 선택되고, 드론에는 출동 요청이 가지 않는다.
UGV 기상 측정은 도착 위치 기준이므로 목표 칸 안에 도착해야 완료로 인정한다.
"""

from .conftest import submit
from .test_env_sense import DLAT, LAT0, LON0, _run, _setup, _weather_obs

BOTH = {"resource_types": ["UAV", "UGV"], "sensor": "WEATHER"}
TARGET = {"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0, "cell_id": "C1"}
IN_CELL = {"lat": LAT0 + 20 * DLAT, "lon": LON0}      # 목표 칸(90 m) 안을 지나는 도로 지점
FAR_ROAD = {"lat": 38.07, "lon": 128.20}               # 목표에서 약 5 km 떨어진 도로 끝


def _ugv_far_and_arrives(world, arrive_at=IN_CELL):
    # 총괄은 직선거리 순으로 묻는다. UGV 를 드론(약 9~10 km)보다 멀리(약 21 km) 두어 드론에게 먼저 묻게 한다
    world["ugv"].units["A-ugv1"].update(lat=38.20, lon=128.10)
    world["ugv"].default_script = lambda rid: [
        {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
        {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
         # ugv/server.py 처럼 도착 위치를 함께 준다 (기상 측정은 도착 위치 기준)
         "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1", "position": dict(arrive_at)},
         "_unit": {"state": "READY", "current_task_id": None}}]


def _all_uavs_reject(world):
    for rid in world["uav"].ids:
        world["uav"].verdicts[rid] = {"verdict": "REJECT", "reason": "STRONG_WIND"}


def test_all_uavs_reject_strong_wind_then_ugv_selected(world):
    orch, uav, ugv, lg = world["orch"], world["uav"], world["ugv"], world["ledger"]
    _setup(world)
    _ugv_far_and_arrives(world)
    _all_uavs_reject(world)

    t = submit(orch, "FALLBACK-1", kind="ENV_SENSE", target=dict(TARGET), requirements=BOTH)
    out = orch.dispatch(t.task_id)

    # 드론은 둘 다 물어봤지만 거절했고, 출동 요청은 UGV 에만 갔다
    assert {c[1] for c in uav.eval_calls()} == set(uav.ids)
    assert uav.exec_calls() == []
    assert out["status"] == "STARTED" and out["resource_id"] == "A-ugv1"
    rejects = [e for e in lg.events(t.task_id) if e["event_type"] == "LOCAL_RESPONSE" and e["result"] == "REJECT"]
    assert {(e["resource_id"], e["reason"]) for e in rejects} == {(rid, "STRONG_WIND") for rid in uav.ids}

    _run(orch)
    assert lg.get_task(t.task_id).purpose_status == "COMPLETED"
    obs = _weather_obs(lg, t.task_id)
    assert obs and obs[0]["resource_id"] == "A-ugv1" and obs[0]["target_covered"]


def test_ugv_stopping_far_from_target_does_not_complete(world):
    """UGV 가 목표 칸 밖 도로에서 멈추면 측정은 남기되 임무는 다시 대기로 돌린다."""
    orch, lg = world["orch"], world["ledger"]
    _setup(world)
    _ugv_far_and_arrives(world, arrive_at=FAR_ROAD)
    _all_uavs_reject(world)
    t = submit(orch, "FALLBACK-3", kind="ENV_SENSE", target=dict(TARGET), requirements=BOTH)
    assert orch.dispatch(t.task_id)["resource_id"] == "A-ugv1"
    _run(orch)
    assert lg.get_task(t.task_id).purpose_status == "PENDING"
    assert any(e["event_type"] == "TASK_REOPENED" and e["reason"] == "OBSERVATION_REQUIREMENT_NOT_MET"
               for e in lg.events(t.task_id))


def test_uavs_accept_so_ugv_not_needed(world):
    """대조군: 드론이 갈 수 있으면 같은 임무에 드론이 간다."""
    orch, uav = world["orch"], world["uav"]
    _setup(world)
    _ugv_far_and_arrives(world)
    t = submit(orch, "FALLBACK-2", kind="ENV_SENSE", target=dict(TARGET), requirements=BOTH)
    out = orch.dispatch(t.task_id)
    assert out["status"] == "STARTED" and out["resource_id"] in uav.ids
