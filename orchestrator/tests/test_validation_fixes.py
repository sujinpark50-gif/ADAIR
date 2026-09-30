# -*- coding: utf-8 -*-
"""검토 2026-09-30 B06(다각형 거리)·B07(환경 ACK)·B08(LLM 출력 검증)."""

import json

import pytest
from fastapi.testclient import TestClient

from orchestrator import llm, priority
from orchestrator.api import create_app
from orchestrator.engine import TaskRejected
from orchestrator.env_adapter import FixtureEnv
from orchestrator.llm import LlmPlanner

from .conftest import TARGET, submit
from .test_flow import UGV_REQ
from .test_llm import FakeOpenAI, _llm_event, _world_with_llm, prefer_lower_risk
from .test_priority import DLAT, DLON, LAT0, LON0, _task, cell, square_site

# ---------------------------------------------------------------------------
# B06 다각형 거리
# ---------------------------------------------------------------------------
H_RECT = [(-2, -0.5), (2, -0.5), (2, 0.5), (-2, 0.5)]
V_RECT = [(-0.5, -2), (0.5, -2), (0.5, 2), (-0.5, 2)]
UNIT = [(0, 0), (1, 0), (1, 1), (0, 1)]


def test_crossing_rectangles_without_vertices_inside_each_other_have_zero_distance():
    assert priority.polygon_distance_m(H_RECT, V_RECT) == 0.0       # 이전 결과: 1.5
    assert priority.polygon_distance_m(V_RECT, H_RECT) == 0.0


@pytest.mark.parametrize("a,b,expected", [
    (UNIT, [(1, 0), (2, 0), (2, 1), (1, 1)], 0.0),                   # 변 접촉
    (UNIT, [(1, 1), (2, 1), (2, 2), (1, 2)], 0.0),                   # 꼭짓점 접촉
    (UNIT, [(0.25, 0.25), (0.75, 0.25), (0.75, 0.75), (0.25, 0.75)], 0.0),   # 포함
    ([(0.25, 0.25), (0.75, 0.25), (0.75, 0.75), (0.25, 0.75)], UNIT, 0.0),
    (UNIT, [(3, 0), (4, 0), (4, 1), (3, 1)], 2.0),                   # 분리
    (UNIT, [(4, 5), (5, 5), (5, 6), (4, 6)], 5.0),                   # 분리 (꼭짓점 사이 3-4-5)
    (UNIT, [(3, 0.5)], 2.0),                                         # 대표점 입력
    (UNIT, [(0.5, 0.5)], 0.0),                                       # 점이 안에 있음
    ([(0, 0)], [(3, 4)], 5.0),                                       # 점-점
    (UNIT, [(0.5, -1), (0.5, 2)], 0.0),                              # 선분이 가로지름
])
def test_polygon_distance_cases(a, b, expected):
    assert priority.polygon_distance_m(a, b) == pytest.approx(expected)
    assert priority.polygon_distance_m(b, a) == pytest.approx(expected)


@pytest.mark.parametrize("bad", [[], [(float("nan"), 0)], [(0, float("inf"))], [(1,)], [("a", 0)], [(True, 0)]])
def test_invalid_geometry_raises_instead_of_inventing_a_distance(bad):
    with pytest.raises(ValueError, match="INVALID_GEOMETRY"):
        priority.polygon_distance_m(UNIT, bad)


def test_protected_site_crossing_a_cell_counts_as_overlap_in_priority():
    # 셀(90m 정사각형)을 남북으로 길게 가로지르는 폭 20m 띠 모양 보호대상: 서로의 꼭짓점이 상대 안에 없다
    strip = {"site_id": "ROAD-VILLAGE", "site_type": "VILLAGE", "human_occupied": True, "boundary": [
        [LAT0 - 300 * DLAT, LON0 - 10 * DLON], [LAT0 - 300 * DLAT, LON0 + 10 * DLON],
        [LAT0 + 300 * DLAT, LON0 + 10 * DLON], [LAT0 + 300 * DLAT, LON0 - 10 * DLON]]}
    snap = FixtureEnv(risk_cells=[cell("R1", 0, 0, 0.4), cell("FAR", 0, 2000, 0.9)], protected_sites=[strip]).read()
    ctx = priority.risk_context(snap)
    assert ctx["cells"]["R1"]["distances"][0] == {"site_id": "ROAD-VILLAGE", "distance_m": 0.0}
    plan = priority.order_tasks([_task("T_FAR", "FAR", 1), _task("T_R1", "R1", 2)], snap, 0.1)
    assert plan["auto_order"][0] == "T_R1"                          # 겹침 → 인명 위험 → 먼저
    assert plan["evidence"]["T_R1"]["human_risk_sources"][0]["source"] == "OVERLAPS_OCCUPIED_SITE"


def test_invalid_site_geometry_is_recorded_as_missing():
    bad = {"site_id": "BAD", "site_type": "FACILITY", "boundary": [[LAT0, LON0], [LAT0, float("nan")], [LAT0, LON0]]}
    ctx = priority.risk_context(FixtureEnv(risk_cells=[cell("R1", 0, 0, 0.4)], protected_sites=[bad]).read())
    assert {"site_id": "BAD", "reason": "SITE_GEOMETRY_INVALID"} in ctx["missing"]
    assert ctx["cells"]["R1"]["distances"] == []


# ---------------------------------------------------------------------------
# B07 환경 반영 ACK
# ---------------------------------------------------------------------------
ARRIVED = {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
           "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1"},
           "_unit": {"state": "READY", "current_task_id": None}}


def _ev(lg, task_id, etype):
    return [e for e in lg.events(task_id) if e["event_type"] == etype]


def test_env_ack_flag_is_resolved_or_rejected_at_intake(world):
    orch = world["orch"]
    road = {"resource_types": ["UGV"], "sensor": "ROAD_STATUS"}
    assert submit(orch, "ROAD-DEFAULT", requirements=road).requirements.needs_env_ack is False   # 생성 시 명시
    assert submit(orch, "THERMAL-DEFAULT").requirements.needs_env_ack is True
    assert submit(orch, "ARRIVE-ONLY", requirements={"resource_types": ["UGV"], "sensor": None}
                  ).requirements.needs_env_ack is False
    with pytest.raises(TaskRejected, match="ENV_ACK_UNSUPPORTED_FOR_SENSOR:ROAD_STATUS"):
        submit(orch, "ROAD-ACK", requirements={**road, "needs_env_ack": True})
    c = TestClient(create_app(orch))
    r = c.post("/tasks", json={"request_id": "ROAD-ACK-HTTP", "target": TARGET,
                               "requirements": {**road, "needs_env_ack": True}})
    assert r.status_code == 422 and r.json()["detail"]["reason"] == "ENV_ACK_UNSUPPORTED_FOR_SENSOR:ROAD_STATUS"
    assert world["ledger"].task_by_request_key("ROAD-ACK-HTTP") is None


def test_ground_task_carrying_ack_flag_is_not_completed_without_ack(world):
    """이전 장부에 남은 조합: needs_env_ack=True 인 도로 관측 임무. ACK 없이 완료시키지 않는다."""
    orch, ugv, lg = world["orch"], world["ugv"], world["ledger"]
    ugv.default_script = lambda rid: [dict(ARRIVED)]
    task = submit(orch, requirements=UGV_REQ)
    stored = lg.get_task(task.task_id)
    stored.requirements.needs_env_ack = True
    lg.save_task(stored)
    out = orch.dispatch(task.task_id)
    for _ in range(2):
        orch.poll()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "HOLD" and t.hold_reason == "ENV_ACK_UNSUPPORTED_FOR_SENSOR:ROAD_STATUS"
    apply = _ev(lg, task.task_id, "ENVIRONMENT_APPLY")[0]
    assert apply["result"] == "UNSUPPORTED" and apply["detail"]["fake_ack_created"] is False
    assert _ev(lg, task.task_id, "TASK_COMPLETE") == []
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"      # 자원 READY 확인·반납은 별개 조건


def test_ground_task_without_ack_requirement_completes(world):
    orch, ugv, lg = world["orch"], world["ugv"], world["ledger"]
    ugv.default_script = lambda rid: [dict(ARRIVED)]
    task = submit(orch, requirements={"resource_types": ["UGV"], "sensor": "ROAD_STATUS"})
    orch.dispatch(task.task_id)
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert "NO_ENV_ACK_REQUIRED" in _ev(lg, task.task_id, "TASK_COMPLETE")[0]["detail"]["basis"]


def _quiet_target(world):
    world["env"].fire_cells[0]["fire_state"] = "UNBURNED"            # 불 발견에 따른 추가 측정 없이 ACK 만 본다


def test_ack_required_task_is_not_complete_when_env_rejects(world):
    orch, lg, env = world["orch"], world["ledger"], world["env"]
    _quiet_target(world)
    env.apply = lambda obs: {"accepted": False, "reason": "REJECTED_BY_ENV"}
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(3):
        orch.poll()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "ENV_APPLY_NOT_ACKED"
    assert _ev(lg, task.task_id, "ENVIRONMENT_APPLY")[0]["result"] == "NACK"


def test_ack_timeout_keeps_task_incomplete_then_retry_with_same_observation_completes(world):
    orch, lg, env = world["orch"], world["ledger"], world["env"]
    _quiet_target(world)
    real, calls = env.apply, []

    def flaky(obs):
        calls.append(obs["observation_id"])
        if len(calls) == 1:
            raise TimeoutError("env apply timed out")
        return real(obs)
    env.apply = flaky
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    for _ in range(3):
        orch.poll()                                                  # 도착·관측, 환경 반영 응답 없음
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"   # ACK 전에는 완료 아님
    assert _ev(lg, task.task_id, "ENVIRONMENT_APPLY")[0]["result"] == "TIMEOUT"
    assert _ev(lg, task.task_id, "TASK_COMPLETE") == []
    orch.poll()                                                      # 같은 관측 ID 로 다시 반영 → ACK
    assert calls[0] == calls[1] and len(set(calls)) == 1
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert _ev(lg, task.task_id, "ENVIRONMENT_APPLY")[-1]["result"] == "ACK"
    assert len(_ev(lg, task.task_id, "OBSERVATION")) == 1            # 관측을 다시 만들지 않는다
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"


def test_ack_never_confirmed_before_resource_returns_reopens_task(world):
    orch, lg, env = world["orch"], world["ledger"], world["env"]
    _quiet_target(world)

    def down(obs):
        raise ConnectionError("env unreachable")
    env.apply = down
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"      # 기체 READY·반납은 ACK 와 별개
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "ENV_APPLY_TIMEOUT"
    assert _ev(lg, task.task_id, "TASK_COMPLETE") == []


def test_programming_error_in_env_apply_is_not_swallowed(world):
    orch, env = world["orch"], world["env"]
    _quiet_target(world)

    def broken(obs):
        raise KeyError("bug")
    env.apply = broken
    orch.dispatch(submit(orch).task_id)
    orch.poll()
    orch.poll()
    with pytest.raises(KeyError):
        orch.poll()


# ---------------------------------------------------------------------------
# B08 LLM 출력 검증
# ---------------------------------------------------------------------------
INP = {"state_version": 3, "groups": [
    {"group_index": 0, "kinds": ["SIMILAR"], "tasks": [
        {"task_id": "T-A", "cells": ["A"], "nearest_protected_site_id": "V1"},
        {"task_id": "T-B", "cells": ["B"], "nearest_protected_site_id": None}]},
    {"group_index": 1, "kinds": ["CONFLICT"], "tasks": [
        {"task_id": "T-C", "cells": ["C"]}, {"task_id": "T-D", "cells": ["D"]}]}]}


def _good():
    return {"state_version": 3, "groups": [
        {"group_index": 0, "order": ["T-B", "T-A"], "rationale": "B 가 마을에 가깝다", "input_refs": ["T-B", "V1"]},
        {"group_index": 1, "order": ["T-C", "T-D"], "rationale": "C 먼저", "input_refs": []}]}


def _mut(path, value):
    out = _good()
    target = out
    for k in path[:-1]:
        target = target[k]
    if value is _DELETE:
        del target[path[-1]]
    else:
        target[path[-1]] = value
    return out


_DELETE = object()
G0 = ("groups", 0)
BAD_OUTPUTS = {
    "group_index_array": (_mut(G0 + ("group_index",), []), "GROUP_INDEX_TYPE"),          # 이전: TypeError
    "group_index_object": (_mut(G0 + ("group_index",), {"i": 0}), "GROUP_INDEX_TYPE"),
    "group_index_bool": (_mut(G0 + ("group_index",), False), "GROUP_INDEX_TYPE"),        # False == 0 으로 통과 금지
    "group_index_string": (_mut(G0 + ("group_index",), "0"), "GROUP_INDEX_TYPE"),
    "group_index_float": (_mut(G0 + ("group_index",), 0.0), "GROUP_INDEX_TYPE"),
    "order_string": (_mut(G0 + ("order",), "T-A,T-B"), "ORDER_TYPE"),
    "order_missing": (_mut(G0 + ("order",), _DELETE), "SCHEMA_MISMATCH"),
    "order_mixed_types": (_mut(G0 + ("order",), ["T-A", 7]), "ORDER_TYPE"),
    "order_nested": (_mut(G0 + ("order",), [["T-A"], "T-B"]), "ORDER_TYPE"),
    "order_duplicate": (_mut(G0 + ("order",), ["T-A", "T-A"]), "ORDER_NOT_PERMUTATION"),
    "order_missing_task": (_mut(G0 + ("order",), ["T-A"]), "ORDER_NOT_PERMUTATION"),
    "order_other_group_task": (_mut(G0 + ("order",), ["T-A", "T-C"]), "ORDER_NOT_PERMUTATION"),
    "order_unknown_task": (_mut(G0 + ("order",), ["T-A", "T-FAKE"]), "ORDER_NOT_PERMUTATION"),
    "input_refs_string": (_mut(G0 + ("input_refs",), "T-A"), "INPUT_REFS_TYPE"),
    "input_refs_mixed": (_mut(G0 + ("input_refs",), ["T-A", {"id": 1}]), "INPUT_REFS_TYPE"),
    "input_refs_null": (_mut(G0 + ("input_refs",), None), "INPUT_REFS_TYPE"),
    "input_refs_unknown": (_mut(G0 + ("input_refs",), ["HOSPITAL-99"]), "UNKNOWN_REFS"),
    "rationale_not_string": (_mut(G0 + ("rationale",), ["x"]), "RATIONALE_INVALID"),
    "state_version_bool": (_mut(("state_version",), True), "STATE_VERSION_TYPE"),
    "state_version_string": (_mut(("state_version",), "3"), "STATE_VERSION_TYPE"),
    "state_version_mismatch": (_mut(("state_version",), 2), "STATE_VERSION_MISMATCH"),
    "groups_object": (_mut(("groups",), {"0": {}}), "SCHEMA_MISMATCH"),
    "group_not_object": (_mut(G0, ["group"]), "SCHEMA_MISMATCH"),
    "group_missing": (_mut(("groups",), _good()["groups"][:1]), "GROUPS_MISSING"),
    "group_repeated": (_mut(("groups", 1, "group_index"), 0), "GROUP_INDEX_INVALID"),
    "not_object": ([1, 2], "SCHEMA_MISMATCH"),
    "null": (None, "SCHEMA_MISMATCH"),
}


def test_valid_permutation_passes_validation():
    assert llm.validate(_good(), INP) == []


@pytest.mark.parametrize("case", sorted(BAD_OUTPUTS))
def test_bad_llm_output_yields_violation_without_raising(case):
    out, expected = BAD_OUTPUTS[case]
    violations = llm.validate(out, INP)                              # 예외 없이 위반 목록으로 수렴
    assert any(v.startswith(expected) for v in violations), violations


@pytest.mark.parametrize("mutate", [
    lambda o: o["groups"][0].__setitem__("group_index", []),
    lambda o: o["groups"][0].__setitem__("order", "T-A"),
    lambda o: o["groups"][0].__setitem__("input_refs", [1, None]),
    lambda o: o["groups"][0].__setitem__("order", o["groups"][0]["order"] + [3]),
    lambda o: o.__setitem__("state_version", True),
])
def test_bad_llm_output_becomes_invalid_and_rule_fallback_in_engine(world, mutate):
    def reply(kw):
        out = prefer_lower_risk(kw)
        mutate(out)
        return out
    fake, ta, tb = _world_with_llm(world, reply)
    out = world["orch"].dispatch_pending()                           # 예외로 끝나지 않는다
    g = out["groups"][0]
    assert g["status"] == "AUTO_DECIDED" and g["llm"]["status"] == "INVALID" and g["llm"]["fallback"] == "RULE_ORDER"
    assert out["results"][ta.task_id]["status"] == "STARTED"         # 공식 규칙 순서(위험도 높은 A)
    ev = _llm_event(world)
    assert ev["result"] == "INVALID" and ev["reason"]


def test_validator_defect_is_contained_as_invalid(monkeypatch):
    def boom(out, inp):
        raise TypeError("unhashable type: 'list'")
    monkeypatch.setattr(llm, "validate", boom)
    monkeypatch.setattr(llm, "build_input", lambda *a: INP)
    planner = LlmPlanner(client=FakeOpenAI(lambda kw: _good()), timeout_s=30, max_calls=5)
    prop = planner.propose(None, None, None)
    assert prop["status"] == "INVALID" and prop["violations"] == ["VALIDATOR_ERROR:TypeError"]


def test_llm_output_cannot_carry_extra_authority_fields():
    out = _good()
    out["groups"][0]["completed"] = True                             # 완료 판정·안전 조건을 바꾸는 칸은 받지 않는다
    assert "SCHEMA_MISMATCH:groups[0]" in llm.validate(out, INP)
    out = _good()
    out["safety_override"] = "ALLOW"
    assert llm.validate(out, INP) == ["SCHEMA_MISMATCH"]
    assert json.dumps(llm.OUTPUT_SCHEMA).count("additionalProperties") == 2
