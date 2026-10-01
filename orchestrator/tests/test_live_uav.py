# -*- coding: utf-8 -*-
"""실제 UAV Local Agent(uav/uav-agent, UAV_MODE=mock) 서버와의 HTTP 계약 시험.

실행 전 서버 2대를 띄운다 (저장소 루트의 .venv 기준):
  cd uav/uav-agent
  UAV_ID=A-uav1 UAV_MODE=mock python -m uvicorn main:app --port 8000
  UAV_ID=A-uav2 UAV_MODE=mock python -m uvicorn main:app --port 8001
  ORCH_LIVE_UAV=1 python -m pytest orchestrator/tests/test_live_uav.py -s

PX4/Gazebo 비행이 아니다. UAV 서버의 Mock 비행(배속 100)과 실제 API 형식을 검증한다.
"""

import os
import time

import pytest

from orchestrator.engine import Orchestrator
from orchestrator.env_adapter import FixtureEnv
from orchestrator.knowledge import EnvStationFeed
from orchestrator.ledger import Ledger
from orchestrator.resources import UavClient

pytestmark = pytest.mark.skipif(os.getenv("ORCH_LIVE_UAV") != "1", reason="실제 UAV 서버 필요")

ENDPOINTS = {"A-uav1": "http://127.0.0.1:8000", "A-uav2": "http://127.0.0.1:8001"}
# uav/uav-agent/README.md 의 관측 목표(능선). 좌표는 근사이며 새 인제 맵 전 시험용이다.
RIDGE = {"lat": 38.0425, "lon": 128.2541, "ground_amsl_m": 804.0, "cell_id": "RIDGE"}
# 원통 홈(38.1205,128.2018) 에서 약 3km. 지면고도 400m 는 시험 가정값 (DEM 조회 아님)
NEAR = {"lat": 38.1000, "lon": 128.2150, "ground_amsl_m": 400.0, "cell_id": "NEAR"}


@pytest.fixture(autouse=True)
def idle_drones():
    """앞 시험의 비행이 끝나 착륙할 때까지 기다리고, 배터리를 요청서 §4.2 예시값 95% 로 맞춘다."""
    import httpx
    deadline = time.time() + 90
    for rid, url in ENDPOINTS.items():
        while time.time() < deadline:
            s = httpx.get(f"{url}/uav/{rid}/state", timeout=5).json()
            if s["current_task_id"] is None and not s["armed"]:
                break
            time.sleep(1)
        httpx.post(f"{url}/mock/set", params={"battery_pct": 95}, timeout=5)
    yield


def _env(wind):
    cells = [{**t, "cell_size_m": 90.0, "fire_state": "BURNING"} for t in (RIDGE, NEAR)]
    return FixtureEnv(wind_ms=wind, fire_cells=cells)


def _orch(lg, env):
    # 시험 관측소: 환경 풍속을 목표 근처 관측소 값처럼 전달 (HTTP 계약 시험용)
    return Orchestrator(lg, env, UavClient(ENDPOINTS), weather_feeds=[EnvStationFeed(env, NEAR["lat"], NEAR["lon"])])


def _submit(orch, rid="LIVE-1", target=NEAR):
    task, _ = orch.submit_task({"request_id": rid, "incident_id": "INC-LIVE", "kind": "RECON",
                                "target": dict(target),
                                "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL"}})
    return task


def test_live_normal_recon(tmp_path):
    lg = Ledger(str(tmp_path / "live.db"))
    orch = _orch(lg, _env(3.0))
    task = _submit(orch)
    out = orch.dispatch(task.task_id)
    print("\ndispatch:", out)
    for e in lg.events(task.task_id):
        if e["event_type"] in ("LOCAL_RESPONSE", "SAFETY_JUDGEMENT", "EXECUTION_SENT"):
            print(" ", e["event_type"], e["resource_id"], e["result"], e["reason"])
    assert out["status"] == "STARTED"
    deadline = time.time() + 90
    while time.time() < deadline:
        orch.poll()
        if lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED":
            break
        time.sleep(1)
    subs = [e["result"] for e in lg.events(task.task_id) if e["event_type"] == "EXECUTION_PROGRESS"]
    print("progress:", subs)
    print("task:", lg.get_task(task.task_id).purpose_status)
    obs = [e for e in lg.events(task.task_id) if e["event_type"] == "OBSERVATION"]
    if obs:
        d = obs[0]["detail"]
        print("observation:", d["result"], "agl", round(d["footprint"]["agl_m"], 1) if d["footprint"] else None,
              "covered", d["target_covered"])
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"


def test_live_ridge_counter_not_executable_holds(tmp_path):
    """요청서 §4.2 예시: 능선 목표는 복귀 여유 부족 COUNTER(observe_duration_s) → 실어 보낼 칸이 없어 보류"""
    lg = Ledger(str(tmp_path / "live3.db"))
    orch = _orch(lg, _env(3.0))
    out = orch.dispatch(_submit(orch, "LIVE-3", RIDGE).task_id)
    print("\n", out)
    assert out["status"] == "HOLD"
    assert all(v == "COUNTER_FIELD_UNSUPPORTED:observe_duration_s" for v in out["excluded"].values())


def test_live_high_wind_holds(tmp_path):
    lg = Ledger(str(tmp_path / "live2.db"))
    orch = _orch(lg, _env(12.4))
    out = orch.dispatch(_submit(orch, "LIVE-2").task_id)
    print("\n", out)
    assert out["status"] == "HOLD"
    assert all(v == "LOCAL_REJECT:HIGH_WIND" for v in out["excluded"].values())
