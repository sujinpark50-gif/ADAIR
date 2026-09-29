# -*- coding: utf-8 -*-
from fastapi.testclient import TestClient

from orchestrator.api import create_app

from .conftest import TARGET

BODY = {"request_id": "WEB-1", "incident_id": "INC-INJE", "target": TARGET,
        "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL"}}


def test_api_task_lifecycle(world):
    c = TestClient(create_app(world["orch"]))
    h = c.get("/health").json()
    assert h["status"] == "ok" and h["env"]["source"] == "FIXTURE"
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
