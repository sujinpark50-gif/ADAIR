# -*- coding: utf-8 -*-
"""실제 UGV Local Agent(ugv/server.py, UGV_DRIVER=sim) 와의 HTTP 계약 시험.

실행 전 (저장소 루트):
  UGV_DRIVER=sim python -m uvicorn ugv.server:app --port 8100
  ORCH_LIVE_UGV=1 python -m pytest orchestrator/tests/test_live_ugv.py -s

PX4 주행이 아니다. UGV 서버의 SimDriver(시연 속도)와 실제 도로망 판단을 검증한다.
"""

import os
import time

import httpx
import pytest

from orchestrator.engine import Orchestrator
from orchestrator.env_adapter import FixtureEnv
from orchestrator.ledger import Ledger
from orchestrator.resources import UgvClient

pytestmark = pytest.mark.skipif(os.getenv("ORCH_LIVE_UGV") != "1", reason="실제 UGV 서버 필요")

URL = "http://127.0.0.1:8100"
ROAD_OK = {"lat": 38.07, "lon": 128.20, "cell_id": "ROAD_OK"}            # 도로 노드 약 530m
FAR_RIDGE = {"lat": 38.05, "lon": 128.25, "cell_id": "RIDGE"}            # 가장 가까운 노드가 2km 초과


@pytest.fixture(autouse=True)
def idle_vehicles():
    deadline = time.time() + 30
    while time.time() < deadline:
        if all(u["current_task_id"] is None for u in httpx.get(f"{URL}/ugv", timeout=5).json()):
            break
        time.sleep(0.5)
    yield


def _orch(tmp_path, name):
    return Orchestrator(Ledger(str(tmp_path / f"{name}.db")), FixtureEnv(), ugv=UgvClient(URL))


def _submit(orch, rid, target, sensor="ROAD_STATUS", types=("UGV",)):
    task, _ = orch.submit_task({"request_id": rid, "incident_id": "INC-LIVE", "kind": "GROUND_RECON",
                                "target": dict(target),
                                "requirements": {"resource_types": list(types), "sensor": sensor,
                                                 "needs_env_ack": False}})
    return task


def _run_until_released(orch, attempt_id, timeout=30):
    deadline = time.time() + timeout
    while time.time() < deadline:
        orch.poll()
        if orch.ledger.get_attempt(attempt_id)["substatus"] == "RELEASED":
            return True
        time.sleep(0.5)
    return False


def test_live_ugv_road_status_closed_loop(tmp_path):
    orch = _orch(tmp_path, "ugv1")
    task = _submit(orch, "UGV-1", ROAD_OK)
    out = orch.dispatch(task.task_id)
    print("\ndispatch:", out)
    assert out["status"] == "STARTED" and out["resource_id"] == "A-ugv1"
    assert _run_until_released(orch, out["attempt_id"])
    ev = orch.ledger.events(task.task_id)
    for e in ev:
        print(" ", e["event_type"], e["resource_id"] or "", e["result"] or "", e["reason"] or "")
    obs = [e for e in ev if e["event_type"] == "OBSERVATION"][0]
    assert obs["result"] == "ROAD_STATUS" and obs["detail"]["is_fire_observation"] is False
    assert orch.ledger.get_task(task.task_id).purpose_status == "COMPLETED"
    assert orch.ledger.reservations() == {}


def test_live_ugv_unreachable_target_holds(tmp_path):
    orch = _orch(tmp_path, "ugv2")
    out = orch.dispatch(_submit(orch, "UGV-2", FAR_RIDGE).task_id)
    print("\n", out)
    assert out["status"] == "HOLD"
    assert out["excluded"]["A-ugv1"] == "LOCAL_REJECT:TARGET_UNREACHABLE"
    assert out["excluded"]["A-fire1"] == "TYPE_NOT_ALLOWED"


def test_live_ugv_not_sent_for_thermal_task(tmp_path):
    orch = _orch(tmp_path, "ugv3")
    out = orch.dispatch(_submit(orch, "UGV-3", ROAD_OK, sensor="THERMAL", types=("UGV", "FIRE_ENGINE")).task_id)
    print("\n", out)
    assert out["status"] == "HOLD"
    assert set(out["excluded"].values()) == {"REQUIRED_CAPABILITY_UNAVAILABLE"}
    assert not any(e["event_type"] == "LOCAL_RESPONSE" for e in orch.ledger.events())


def test_live_two_tasks_do_not_share_vehicle(tmp_path):
    orch = _orch(tmp_path, "ugv4")
    a = orch.dispatch(_submit(orch, "UGV-4a", ROAD_OK).task_id)
    b = orch.dispatch(_submit(orch, "UGV-4b", ROAD_OK).task_id)
    print("\n", a, "\n", b)
    assert a["resource_id"] == "A-ugv1"
    assert b.get("resource_id") != "A-ugv1"          # 점유 중인 차는 두 번째 Task 후보에서 빠진다
    for r in (a, b):
        if r.get("attempt_id"):
            assert _run_until_released(orch, r["attempt_id"], timeout=60)
