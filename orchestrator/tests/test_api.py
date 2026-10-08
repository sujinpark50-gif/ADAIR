# -*- coding: utf-8 -*-
from fastapi.testclient import TestClient

from orchestrator.api import create_app

from .conftest import TARGET

BODY = {"request_id": "WEB-1", "incident_id": "INC-INJE", "target": TARGET,
        "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL"}}


def test_api_task_lifecycle(world):
    c = TestClient(create_app(world["orch"]))
    h = c.get("/health").json()
    assert h["status"] == "ok" and h["env"]["source"] == "ORCHESTRATOR_VIEW"
    assert h["view"]["analysis_source"] == "TEST_INJECTED"
    assert h["f1_official_rule"] == "NOT_READY" and h["f2_suppression"] == "NOT_READY"

    r = c.post("/tasks", json=BODY)
    assert r.status_code == 202
    d = r.json()
    assert d["created"] and d["dispatch"]["status"] == "STARTED"
    tid = d["task"]["task_id"]
    assert d["task"]["common_state"] == "RUNNING"

    # 같은 request_id 재전송 → 같은 Task, 재출동 없음
    r2 = c.post("/tasks", json=BODY).json()
    assert r2["task"]["task_id"] == tid and not r2["created"] and r2["dispatch"] is None
    assert len(world["uav"].exec_calls()) == 1
    # 내용이 다르면 409
    assert c.post("/tasks", json={**BODY, "kind": "MONITOR"}).status_code == 409

    for _ in range(4):
        c.post("/poll")
    t = c.get(f"/tasks/{tid}").json()
    assert t["purpose_status"] == "COMPLETED" and t["attempts"][0]["substatus"] == "RELEASED"
    assert len(c.get(f"/tasks/{tid}/decisions").json()) == 1
    kinds = [e["event_type"] for e in c.get(f"/tasks/{tid}/events").json()]
    for k in ("LOCAL_RESPONSE", "SAFETY_JUDGEMENT", "EXECUTION_SENT", "OBSERVATION", "ENVIRONMENT_APPLY", "TASK_COMPLETE"):
        assert k in kinds
    assert c.get("/state").json()["reservations"] == {}


def test_api_hold_and_event_resume(world):
    world["env"].wind_ms = None
    c = TestClient(create_app(world["orch"]))
    d = c.post("/tasks", json=BODY).json()
    assert d["dispatch"]["status"] == "HOLD"
    tid = d["task"]["task_id"]
    assert c.get("/state").json()["holds"][0]["task_id"] == tid

    world["env"].wind_ms = 4.0
    e = c.post("/events", json={"event_id": "EV-1", "type": "ENV_UPDATED"}).json()
    assert e["redispatched"][0]["status"] == "STARTED"
    dup = c.post("/events", json={"event_id": "EV-1", "type": "ENV_UPDATED"})
    assert dup.status_code == 200 and dup.json()["duplicate"]
    assert len(world["uav"].exec_calls()) == 1


def test_post_task_returns_without_waiting_when_dispatcher_runs(world, monkeypatch):
    """배정 작업자가 돌면 POST /tasks 는 접수만 하고 바로 응답한다 — 작업자가 LLM 관측 계획으로 dispatch_gate 를 쥔
    동안 응답이 수 분 늦어 운용자 요청이 시간 초과되던 문제 (2026-10-08 트윈). 배정은 작업자가 이어서 한다."""
    import time
    from orchestrator import config
    monkeypatch.setattr(config, "AUTO_DISPATCH_ON_CHANGE", True)
    with TestClient(create_app(world["orch"], poll_interval_s=3600)) as c:     # 추적 루프는 사실상 쉼
        d = c.post("/tasks", json=BODY).json()
        assert d["created"] and d["dispatch"] == {"status": "QUEUED_FOR_DISPATCHER"}
        tid = d["task"]["task_id"]
        for _ in range(50):                               # 작업자가 이어서 배정한다
            if world["uav"].exec_calls():
                break
            time.sleep(0.1)
        assert len(world["uav"].exec_calls()) == 1
        assert c.get(f"/tasks/{tid}").json()["common_state"] == "RUNNING"
