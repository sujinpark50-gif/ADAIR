# -*- coding: utf-8 -*-
"""LLM 제안 → 결정론적 검증 → 채택 또는 규칙 fallback (사용자 결정 1A, 요청서 §9·§12)."""

import json

from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis
from orchestrator.llm import LlmPlanner

from .test_priority import LAT0, LON0, _fire_world, _one_uav, _submit_cell, cell


class FakeResponses:
    def __init__(self, outer):
        self.outer = outer

    def create(self, **kw):
        self.outer.requests.append(kw)
        r = self.outer.reply(kw)
        if isinstance(r, Exception):
            raise r
        return type("Resp", (), {"output_text": r if isinstance(r, str) else json.dumps(r, ensure_ascii=False)})()


class FakeOpenAI:
    """reply(kw) → dict/str/Exception. kw['input'] 은 총괄이 보낸 JSON 문자열"""

    def __init__(self, reply):
        self.reply, self.requests = reply, []
        self.responses = FakeResponses(self)


def _inp(kw):
    return json.loads(kw["input"])


def prefer_lower_risk(kw):
    """묶음마다 위험도가 낮은 쪽을 먼저 (규칙 순서와 반대) — LLM 이 실제로 반영되는지 보려는 것"""
    inp = _inp(kw)
    return {"state_version": inp["state_version"], "groups": [
        {"group_index": g["group_index"],
         "order": [t["task_id"] for t in sorted(g["tasks"], key=lambda t: t["risk_score"])],
         "rationale": "B 칸이 마을 진입로에 가까워 먼저 확인", "input_refs": [g["tasks"][0]["task_id"]]}
        for g in inp["groups"]]}


def _world_with_llm(world, reply, mode="AUTO_HIGHER_RISK"):
    fake = FakeOpenAI(reply)
    world["orch"].llm = LlmPlanner(client=fake, model="gpt-5.6-luna", timeout_s=30, max_calls=5)
    world["orch"].set_priority_mode(mode, "시험")
    _fire_world(world, [cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.65)])
    _one_uav(world)
    ta, tb = _submit_cell(world["orch"], "R-A", "A"), _submit_cell(world["orch"], "R-B", "B")
    return fake, ta, tb


def _llm_event(world):
    return [e for e in world["ledger"].events() if e["event_type"] == "LLM_PLAN"][-1]


def test_valid_llm_order_is_used_in_auto_mode(world):
    fake, ta, tb = _world_with_llm(world, prefer_lower_risk)
    out = world["orch"].dispatch_pending()
    g = out["groups"][0]
    assert g["status"] == "LLM_DECIDED" and g["llm"]["order"] == [tb.task_id, ta.task_id]
    assert out["results"][tb.task_id]["status"] == "STARTED"            # AI 가 고른 B 가 드론을 받음
    assert out["results"][ta.task_id]["excluded"]["A-uav1"] == "OCCUPIED"
    req = fake.requests[0]
    assert req["model"] == "gpt-5.6-luna" and req["timeout"] == 30
    assert req["text"]["format"]["type"] == "json_schema" and req["text"]["format"]["strict"] is True
    inp = _inp(req)
    assert set(inp["groups"][0]["tasks"][0]) >= {"task_id", "human_risk", "risk_score"}
    ev = _llm_event(world)
    assert ev["result"] == "OK" and ev["detail"]["model"] == "gpt-5.6-luna"


def _bad(mutate):
    def reply(kw):
        out = prefer_lower_risk(kw)
        mutate(out, _inp(kw))
        return out
    return reply


def test_invalid_llm_outputs_fall_back_to_rule_order(world):
    cases = {
        "UNKNOWN_TASK": _bad(lambda o, i: o["groups"][0]["order"].__setitem__(0, "TASK-FAKE")),
        "STALE_VERSION": _bad(lambda o, i: o.__setitem__("state_version", i["state_version"] - 1)),
        "INVENTED_REF": _bad(lambda o, i: o["groups"][0]["input_refs"].append("HOSPITAL-99")),
        "MISSING_GROUP": _bad(lambda o, i: o.__setitem__("groups", [])),
        "NOT_JSON": lambda kw: "먼저 A 로 보내세요",
        "TIMEOUT": lambda kw: TimeoutError("timed out"),
    }
    for name, reply in cases.items():
        from orchestrator.engine import Orchestrator
        from orchestrator.ledger import Ledger
        from orchestrator.resources import UavClient, UgvClient
        from .conftest import fire_env
        from .fakes import FakeUav, FakeUgv, normal_flight
        uav = FakeUav()
        uav.default_script = normal_flight()
        w = {"uav": uav, "env": fire_env(), "ledger": Ledger(":memory:"), "analysis": InjectedAnalysis()}
        from .conftest import TARGET
        w["orch"] = Orchestrator(w["ledger"], w["env"], UavClient(uav.endpoints(), uav.client()),
                                 UgvClient("http://ugv", FakeUgv().client()),
                                 analysis=w["analysis"],
                                 weather_feeds=[EnvStationFeed(w["env"], TARGET["lat"], TARGET["lon"])])
        fake, ta, tb = _world_with_llm(w, reply)
        out = w["orch"].dispatch_pending()
        g = out["groups"][0]
        assert g["status"] == "AUTO_DECIDED", name                      # 규칙 순서로 회수
        assert g["llm"]["fallback"] == "RULE_ORDER", name
        assert out["results"][ta.task_id]["status"] == "STARTED", name  # 규칙: 위험도 높은 A
        assert _llm_event(w)["result"] in ("INVALID", "ERROR"), name


def test_human_choice_mode_shows_llm_recommendation_but_waits(world):
    fake, ta, tb = _world_with_llm(world, prefer_lower_risk, mode="HUMAN_CHOICE")
    out = world["orch"].dispatch_pending()
    g = out["groups"][0]
    assert g["status"] == "AWAITING_CHOICE" and g["llm"]["order"][0] == tb.task_id
    assert world["uav"].exec_calls() == []                             # 추천만, 출동은 사람이


def test_llm_not_called_when_not_configured_or_limit_reached(world):
    calls = []
    fake = FakeOpenAI(lambda kw: calls.append(1) or prefer_lower_risk(kw))
    for planner, why in ((LlmPlanner(client=fake, timeout_s=None, max_calls=5), "TIMEOUT_NOT_CONFIGURED"),
                         (LlmPlanner(client=fake, timeout_s=30, max_calls=None), "CALL_LIMIT_NOT_CONFIGURED"),
                         (LlmPlanner(client=fake, timeout_s=30, max_calls=0), "CALL_LIMIT_REACHED")):
        assert planner.status() == why
    assert calls == []
    assert LlmPlanner(timeout_s=30, max_calls=5).status() == "API_KEY_MISSING"


def test_no_llm_call_without_choice_groups(world):
    fake = FakeOpenAI(prefer_lower_risk)
    world["orch"].llm = LlmPlanner(client=fake, timeout_s=30, max_calls=5)
    _fire_world(world, [cell("A", 0, 0, 0.9), cell("B", 0, 500, 0.4)])
    _submit_cell(world["orch"], "R-A", "A"), _submit_cell(world["orch"], "R-B", "B")
    world["orch"].dispatch_pending()
    assert fake.requests == []                                          # 규칙으로 정해지면 부르지 않음 (비용)


def test_llm_recommendation_reused_for_same_group_and_situation(world):
    from orchestrator import config, priority
    fake, ta, tb = _world_with_llm(world, prefer_lower_risk, mode="HUMAN_CHOICE")
    orch = world["orch"]

    def recommend():
        snap = orch.view()
        tasks = orch.ledger.list_tasks(["PENDING", "HOLD"])
        plan = priority.order_tasks(tasks, snap, config.PRIORITY_SIMILAR_RISK_DELTA)
        return orch._llm_recommend(snap, plan, {t.task_id: t for t in tasks})
    assert recommend() and recommend()                                 # 같은 묶음·같은 상황 두 번
    assert len(fake.requests) == 1
    assert [e for e in world["ledger"].events() if e["event_type"] == "LLM_PLAN_REUSED"]
    world["env"].advance()                                             # 상황이 바뀌면 다시 호출
    recommend()
    assert len(fake.requests) == 2
