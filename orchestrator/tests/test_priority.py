# -*- coding: utf-8 -*-
"""우선순위: 위험 셀 ↔ 보호대상 경계 거리, 가중치 없는 우월 관계, 엇갈림 보류 (요청서 §8)."""

import pytest

from orchestrator import priority
from orchestrator.env_adapter import FixtureEnv
from orchestrator.models import Target, Task

from .conftest import submit

LAT0, LON0 = 38.05, 128.25
DLAT = 1 / 111_320.0                     # 1 m 에 해당하는 위도
DLON = 1 / (111_320.0 * 0.78801)         # 38.05° 에서 1 m 에 해당하는 경도 (cos 38.05°)


def cell(cid, dy_m, dx_m, score, size=90.0):
    return {"cell_id": cid, "lat": LAT0 + dy_m * DLAT, "lon": LON0 + dx_m * DLON, "cell_size_m": size,
            "risk_score": score, "fire_state": "UNBURNED", "ground_amsl_m": 600.0}


def square_site(sid, dy_m, dx_m, half_m, rank=None):
    c = (LAT0 + dy_m * DLAT, LON0 + dx_m * DLON)
    b = [[c[0] - half_m * DLAT, c[1] - half_m * DLON], [c[0] - half_m * DLAT, c[1] + half_m * DLON],
         [c[0] + half_m * DLAT, c[1] + half_m * DLON], [c[0] + half_m * DLAT, c[1] - half_m * DLON]]
    return {"site_id": sid, "site_type": "VILLAGE", "boundary": b, "official_priority_rank": rank}


def _task(tid, cid, created):
    return Task(task_id=tid, incident_id="I", kind="RECON", target=Target(LAT0, LON0, 600.0, cid),
                created_wall=created)


# ---------------------------------------------------------------------------
# 거리 계산
# ---------------------------------------------------------------------------
def test_cell_to_boundary_distance_uses_edges_not_centers():
    # 셀 중심에서 동쪽 500m 에 한 변 200m 짜리 마을. 셀 동쪽 변(45m) ~ 마을 서쪽 변(400m) = 355m
    env = FixtureEnv(risk_cells=[cell("R1", 0, 0, 0.8)], protected_sites=[square_site("V1", 0, 500, 100)])
    ctx = priority.risk_context(env.read())
    d = ctx["cells"]["R1"]["distances"][0]
    assert d["site_id"] == "V1" and d["distance_m"] == pytest.approx(355.0, abs=1.0)
    assert ctx["method"] == priority.METHOD


def test_overlap_is_zero_point_site_and_missing_geometry_recorded():
    sites = [square_site("V1", 0, 0, 30), {"site_id": "P1", "site_type": "FACILITY",
                                           "location": {"lat": LAT0, "lon": LON0 + 245 * DLON}},
             {"site_id": "X1", "site_type": "FACILITY"}]
    ctx = priority.risk_context(FixtureEnv(risk_cells=[cell("R1", 0, 0, 0.5)], protected_sites=sites).read())
    ds = {d["site_id"]: d["distance_m"] for d in ctx["cells"]["R1"]["distances"]}
    assert ds["V1"] == 0.0
    assert ds["P1"] == pytest.approx(200.0, abs=1.0)            # 점: 셀 변(45m)에서 245m 지점
    assert ctx["sites"]["P1"]["geometry"] == "POINT_ONLY"
    assert {"site_id": "X1", "reason": "SITE_GEOMETRY_MISSING"} in ctx["missing"]
    assert "X1" not in ds                                        # 거리를 지어내지 않는다


# ---------------------------------------------------------------------------
# 비교 규칙
# ---------------------------------------------------------------------------
def test_dominating_task_comes_first():
    env = FixtureEnv(risk_cells=[cell("HIGH_NEAR", 0, 0, 0.9), cell("LOW_FAR", 2000, 0, 0.5)],
                     protected_sites=[square_site("V1", 0, 300, 50)])
    plan = priority.order_tasks([_task("T_LOW", "LOW_FAR", 1), _task("T_HIGH", "HIGH_NEAR", 2)], env.read())
    assert plan["fronts"] == [["T_HIGH"], ["T_LOW"]]
    assert plan["conflicting_fronts"] == [] and plan["criteria"] == ["risk_score", "distance_m"]


def test_crossing_criteria_form_one_conflicting_front():
    # A: 위험 높음·보호대상 멀리 / B: 위험 낮음·보호대상 가까이 → 가중치 없이는 정할 수 없음
    env = FixtureEnv(risk_cells=[cell("A", 3000, 0, 0.9), cell("B", 0, 0, 0.5)],
                     protected_sites=[square_site("V1", 0, 300, 50)])
    plan = priority.order_tasks([_task("TA", "A", 1), _task("TB", "B", 2)], env.read())
    assert plan["fronts"] == [["TA", "TB"]] and plan["conflicting_fronts"] == [["TA", "TB"]]


def test_missing_risk_goes_last_and_is_recorded():
    env = FixtureEnv(risk_cells=[cell("A", 0, 0, 0.3), cell("N", 0, 200, None)])
    plan = priority.order_tasks([_task("TN", "N", 1), _task("TA", "A", 2)], env.read())
    assert plan["fronts"] == [["TA"], ["TN"]]
    assert plan["evidence"]["TN"]["risk_score_missing"] is True
    assert plan["criteria"] == ["risk_score"]                   # 보호대상 registry 없음 → 거리 기준 제외


def test_equal_evidence_uses_created_order_without_conflict():
    env = FixtureEnv(risk_cells=[cell("A", 0, 0, 0.7), cell("B", 0, 0, 0.7)])
    plan = priority.order_tasks([_task("T2", "B", 2), _task("T1", "A", 1)], env.read())
    assert plan["fronts"] == [["T1", "T2"]] and plan["conflicting_fronts"] == []


def test_official_rank_only_used_when_all_tasks_have_it():
    env = FixtureEnv(risk_cells=[cell("A", 0, 0, 0.7), cell("B", 5000, 0, 0.7)],
                     protected_sites=[square_site("V1", 0, 300, 50, rank=1), square_site("V2", 5000, 300, 50)])
    plan = priority.order_tasks([_task("TA", "A", 1), _task("TB", "B", 2)], env.read())
    assert "official_priority_rank" not in plan["criteria"]


# ---------------------------------------------------------------------------
# 엔진: 우선순위 층 순서 배정
# ---------------------------------------------------------------------------
def _fire_world(world, cells, sites):
    world["env"].risk_cells = cells
    world["env"].protected_sites = sites


def _submit_cell(orch, rid, cid):
    return submit(orch, rid, target={"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0, "cell_id": cid},
                  requirements={"resource_types": ["UAV"], "sensor": "THERMAL"})


def test_conflict_with_too_few_resources_holds_whole_front(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    _fire_world(world, [cell("A", 3000, 0, 0.9), cell("B", 0, 0, 0.5)], [square_site("V1", 0, 300, 50)])
    uav.state["A-uav2"]["failsafe"] = True                      # 쓸 수 있는 UAV 1대뿐
    ta, tb = _submit_cell(orch, "R-A", "A"), _submit_cell(orch, "R-B", "B")
    out = orch.dispatch_pending()
    assert out["results"][ta.task_id]["reason"] == "PRIORITY_CONFLICT"
    assert out["results"][tb.task_id]["reason"] == "PRIORITY_CONFLICT"
    assert lg.get_task(ta.task_id).resume_condition == "MANUAL_RESOLUTION"
    assert uav.exec_calls() == []


def test_conflict_with_enough_resources_dispatches_both(world):
    orch, uav = world["orch"], world["uav"]
    _fire_world(world, [cell("A", 3000, 0, 0.9), cell("B", 0, 0, 0.5)], [square_site("V1", 0, 300, 50)])
    ta, tb = _submit_cell(orch, "R-A", "A"), _submit_cell(orch, "R-B", "B")
    out = orch.dispatch_pending()
    assert out["results"][ta.task_id]["status"] == "STARTED"
    assert out["results"][tb.task_id]["status"] == "STARTED"


def test_dominating_task_gets_scarce_resource(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    _fire_world(world, [cell("HIGH", 0, 0, 0.9), cell("LOW", 2000, 0, 0.5)], [square_site("V1", 0, 300, 50)])
    uav.state["A-uav2"]["failsafe"] = True
    t_low, t_high = _submit_cell(orch, "R-L", "LOW"), _submit_cell(orch, "R-H", "HIGH")   # 낮은 쪽이 먼저 접수
    out = orch.dispatch_pending()
    assert out["fronts"] == [[t_high.task_id], [t_low.task_id]]
    assert out["results"][t_high.task_id]["status"] == "STARTED"
    assert out["results"][t_low.task_id]["status"] == "HOLD"
    assert out["results"][t_low.task_id]["excluded"]["A-uav1"] == "OCCUPIED"
    ev = [e for e in lg.events() if e["event_type"] == "PRIORITY_ORDER"][0]["detail"]
    assert ev["rule"].startswith("PARETO_DOMINANCE_NO_WEIGHTS")
