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
def _plan(cells, sites, tasks, delta=0.1):
    return priority.order_tasks(tasks, FixtureEnv(risk_cells=cells, protected_sites=sites).read(), delta)


def test_residential_cell_always_first_even_with_lower_risk():
    cells = [cell("HIGH", 0, 0, 0.9), dict(cell("HOME", 3000, 0, 0.3), building_type=1)]
    plan = _plan(cells, [], [_task("T_HIGH", "HIGH", 1), _task("T_HOME", "HOME", 2)])
    assert plan["auto_order"] == ["T_HOME", "T_HIGH"]
    ev = plan["evidence"]["T_HOME"]
    assert ev["human_risk"] is True and ev["human_risk_sources"][0]["source"] == "ENV_CELL_RESIDENTIAL"
    assert plan["groups"] == []                                  # 인명 지역은 묶지 않고 항상 먼저


def test_overlap_with_occupied_site_counts_as_human_risk_and_unknown_is_recorded():
    site = dict(square_site("V1", 0, 0, 30), human_occupied=True)
    plan = _plan([cell("A", 0, 0, 0.2), cell("B", 5000, 0, 0.8)], [site],
                 [_task("TA", "A", 1), _task("TB", "B", 2)])
    assert plan["auto_order"][0] == "TA"
    assert plan["evidence"]["TA"]["human_risk_sources"][0]["source"] == "OVERLAPS_OCCUPIED_SITE"
    assert plan["evidence"]["TB"]["human_risk"] is None          # 정보 없음 — '없음'으로 단정하지 않음


def test_similar_scores_form_group_and_auto_prefers_slightly_higher():
    plan = _plan([cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.65)], [],
                 [_task("TB", "B", 1), _task("TA", "A", 2)])
    assert plan["auto_order"] == ["TA", "TB"]
    assert plan["groups"] == [{"task_ids": ["TA", "TB"], "kinds": ["SIMILAR"], "auto_choice": "TA"}]


def test_clear_gap_is_not_grouped():
    plan = _plan([cell("A", 0, 0, 0.9), cell("B", 0, 500, 0.5)], [], [_task("TA", "A", 1), _task("TB", "B", 2)])
    assert plan["groups"] == [] and plan["units"] == [["TA"], ["TB"]]


def test_crossing_criteria_form_conflict_group():
    # A: 위험 높음·보호대상 멀리 / B: 위험 낮음·보호대상 가까이 (차이 0.4 → 비슷하진 않음)
    plan = _plan([cell("A", 3000, 0, 0.9), cell("B", 0, 0, 0.5)], [square_site("V1", 0, 300, 50)],
                 [_task("TA", "A", 1), _task("TB", "B", 2)])
    assert plan["groups"][0]["kinds"] == ["CONFLICT"] and plan["groups"][0]["auto_choice"] == "TA"


def test_missing_risk_goes_last_and_is_recorded():
    plan = _plan([cell("A", 0, 0, 0.3), cell("N", 0, 200, None)], [], [_task("TN", "N", 1), _task("TA", "A", 2)])
    assert plan["auto_order"] == ["TA", "TN"]
    assert plan["evidence"]["TN"]["risk_score_missing"] is True
    assert plan["criteria"] == ["risk_score"]                   # 보호대상 registry 없음 → 거리 기준 제외


def test_official_rank_only_used_when_all_tasks_have_it():
    plan = _plan([cell("A", 0, 0, 0.7), cell("B", 5000, 0, 0.7)],
                 [square_site("V1", 0, 300, 50, rank=1), square_site("V2", 5000, 300, 50)],
                 [_task("TA", "A", 1), _task("TB", "B", 2)])
    assert "official_priority_rank" not in plan["criteria"]


# ---------------------------------------------------------------------------
# 엔진: 두 버전
# ---------------------------------------------------------------------------
def _fire_world(world, cells, sites=()):
    world["analysis"].risk_cells = list(cells)      # 분석 주입값 (진짜 세계 아님)
    world["env"].protected_sites = list(sites)


def _submit_cell(orch, rid, cid):
    return submit(orch, rid, target={"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0, "cell_id": cid},
                  requirements={"resource_types": ["UAV"], "sensor": "THERMAL"})


def _one_uav(world):
    world["uav"].state["A-uav2"]["failsafe"] = True


def test_human_choice_mode_waits_when_resources_short_then_human_picks(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    orch.set_priority_mode("HUMAN_CHOICE", "수동 버전 시험")
    _fire_world(world, [cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.65)])
    _one_uav(world)
    ta, tb = _submit_cell(orch, "R-A", "A"), _submit_cell(orch, "R-B", "B")
    out = orch.dispatch_pending()
    assert out["mode"] == "HUMAN_CHOICE" and out["groups"][0]["status"] == "AWAITING_CHOICE"
    assert {out["results"][t]["reason"] for t in (ta.task_id, tb.task_id)} == {"AWAITING_PRIORITY_CHOICE"}
    assert uav.exec_calls() == []
    # 사람이 위험도가 낮은 B 를 먼저 고름 → B 출동, A 는 계속 대기
    r = orch.choose_priority([tb.task_id], "B 옆 도로가 대피로")
    assert r["results"][tb.task_id]["status"] == "STARTED"
    assert lg.get_task(ta.task_id).hold_reason == "AWAITING_PRIORITY_CHOICE"
    assert [e for e in lg.events() if e["event_type"] == "MANUAL_PRIORITY_CHOICE"][0]["reason"] == "B 옆 도로가 대피로"


def test_human_choice_mode_dispatches_all_when_resources_enough(world):
    orch = world["orch"]
    orch.set_priority_mode("HUMAN_CHOICE", "수동 버전 시험")
    _fire_world(world, [cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.65)])
    ta, tb = _submit_cell(orch, "R-A", "A"), _submit_cell(orch, "R-B", "B")
    out = orch.dispatch_pending()
    assert out["groups"][0]["status"] == "ALL_DISPATCHED"
    assert out["results"][ta.task_id]["status"] == out["results"][tb.task_id]["status"] == "STARTED"


def test_auto_mode_sends_slightly_higher_first_and_marks_auto_decided(world):
    orch = world["orch"]
    orch.set_priority_mode("AUTO_HIGHER_RISK", "시험")
    _fire_world(world, [cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.65)])
    _one_uav(world)
    tb, ta = _submit_cell(orch, "R-B", "B"), _submit_cell(orch, "R-A", "A")   # 낮은 쪽이 먼저 접수
    out = orch.dispatch_pending()
    assert out["groups"][0]["status"] == "AUTO_DECIDED" and out["groups"][0]["auto_choice"] == ta.task_id
    assert out["results"][ta.task_id]["status"] == "STARTED"
    assert out["results"][tb.task_id]["excluded"]["A-uav1"] == "OCCUPIED"


def test_residential_task_gets_scarce_resource_in_both_modes(world):
    orch = world["orch"]
    _fire_world(world, [cell("HIGH", 0, 0, 0.9), dict(cell("HOME", 3000, 0, 0.85), building_type=1)])
    _one_uav(world)
    th, thome = _submit_cell(orch, "R-H", "HIGH"), _submit_cell(orch, "R-HOME", "HOME")
    out = orch.dispatch_pending()
    assert out["units"][0] == [thome.task_id] and out["groups"] == []
    assert out["results"][thome.task_id]["status"] == "STARTED"


def test_board_api_and_mode_switch(world):
    from fastapi.testclient import TestClient
    from orchestrator.api import create_app
    c = TestClient(create_app(world["orch"]))
    assert c.get("/priority/board").json()["mode"] == "AUTO_HIGHER_RISK"      # 기본은 자동
    assert c.post("/priority/mode", json={"mode": "HUMAN_CHOICE"}).json()["mode"] == "HUMAN_CHOICE"
    _fire_world(world, [cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.65)])
    _one_uav(world)
    ta, tb = _submit_cell(world["orch"], "R-A", "A"), _submit_cell(world["orch"], "R-B", "B")
    c.post("/dispatch_pending")
    b = c.get("/priority/board").json()
    assert b["mode"] == "HUMAN_CHOICE" and b["similar_delta"] == 0.1
    g = b["groups"][0]
    assert g["status"] == "AWAITING_CHOICE" and set(g["awaiting"]) == {ta.task_id, tb.task_id}
    assert [t["risk_score"] for t in g["tasks"]] == [0.72, 0.65]
    assert c.post("/priority/mode", json={"mode": "NOPE"}).json()["status"] == "REFUSED"
    assert c.post("/priority/mode", json={"mode": "AUTO_HIGHER_RISK"}).json()["mode"] == "AUTO_HIGHER_RISK"
    r = c.post("/priority/choose", json={"order": [ta.task_id], "reason": "관제 판단"}).json()
    assert r["results"][ta.task_id]["status"] == "STARTED"
    assert "총괄 우선순위 판단" in c.get("/board").text


def test_board_shows_recon_results(world):
    from fastapi.testclient import TestClient
    from orchestrator.api import create_app
    orch = world["orch"]
    c = TestClient(create_app(orch))
    world["env"].fire_cells[0]["cell_id"] = "C1"
    t = submit(orch)                               # conftest: 불타는 셀 C1 정찰
    orch.dispatch(t.task_id)
    for _ in range(3):
        orch.poll()
    b = c.get("/priority/board").json()
    r = b["results"][0]
    assert r["task_id"] == t.task_id
    o = r["observation"]
    assert o["result"] == "DETECTED" and o["detections"] == [{"cell_id": "C1", "fire_state": "BURNING"}]
    assert o["source"] == "SIMULATED" and o["env_apply"]["result"] == "ACK"
    assert o["footprint"] == {"width_m": 52.0, "height_m": 42.0, "agl_m": 90.0}
    assert c.get(f"/tasks/{t.task_id}").json()["observation"]["result"] == "DETECTED"


# ---------------------------------------------------------------------------
# 기본 자동 + 수동 보내기 (사용자 결정 2026-09-30)
# ---------------------------------------------------------------------------
def test_default_mode_is_auto_and_ambiguous_group_departs(world):
    orch = world["orch"]
    assert orch.priority_mode == "AUTO_HIGHER_RISK"
    _fire_world(world, [cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.65)])
    _one_uav(world)
    ta, tb = _submit_cell(orch, "R-A", "A"), _submit_cell(orch, "R-B", "B")
    out = orch.dispatch_pending()
    assert out["groups"][0]["status"] == "AUTO_DECIDED"          # AI 없음 → 규칙으로 자동 출발
    assert out["results"][ta.task_id]["status"] == "STARTED"


def test_manual_dispatch_sends_chosen_task_through_local_and_safety(world):
    orch, lg, uav = world["orch"], world["ledger"], world["uav"]
    _fire_world(world, [cell("A", 0, 0, 0.9), cell("B", 0, 500, 0.3)])
    _one_uav(world)
    ta, tb = _submit_cell(orch, "R-A", "A"), _submit_cell(orch, "R-B", "B")
    world["uav"].state["A-uav1"]["failsafe"] = True                # 잠시 모든 기체 불가 → 둘 다 보류
    orch.dispatch_pending()
    assert lg.get_task(tb.task_id).purpose_status == "HOLD"
    world["uav"].state["A-uav1"]["failsafe"] = False
    r = orch.manual_dispatch(tb.task_id, "B 쪽 주민 신고가 먼저 들어옴")   # 사람이 낮은 쪽을 먼저
    assert r["status"] == "STARTED" and r["resource_id"] == "A-uav1"
    kinds = [e["event_type"] for e in lg.events(tb.task_id)]
    assert "MANUAL_DISPATCH" in kinds and "SAFETY_JUDGEMENT" in kinds and "LOCAL_RESPONSE" in kinds
    assert [e for e in lg.events(tb.task_id) if e["event_type"] == "MANUAL_DISPATCH"][0]["reason"] == "B 쪽 주민 신고가 먼저 들어옴"


def test_manual_dispatch_refused_for_unknown_execution(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.execute_mode["A-uav1"] = "timeout"
    t = submit(orch)
    orch.dispatch(t.task_id)
    assert orch.manual_dispatch(t.task_id, "재시도")["status"] == "REFUSED"   # 불명은 수동 해소로만
