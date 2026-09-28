"""UAV Local Agent — REST API.

UAV 1대당 프로세스 1개다. 여러 대를 운용하면 UAV_ID 와 포트를 달리해 따로 띄운다.

실행:
  UAV_ID=A-uav1 UAV_MODE=mock uvicorn main:app --host 0.0.0.0 --port 8000
  UAV_ID=A-uav2 UAV_MODE=mock uvicorn main:app --host 0.0.0.0 --port 8001

  # PX4 연동. PX4 1개가 약 1.1 코어를 쓰므로 real 은 서버당 1대만 올린다.
  UAV_ID=A-uav1 UAV_MODE=real PX4_ADDRESS=udpin://0.0.0.0:14540 \
      uvicorn main:app --host 0.0.0.0 --port 8000

문서: http://<host>:8000/docs
"""
import asyncio
import os
from datetime import datetime, timezone

from fastapi import FastAPI, HTTPException

import config
import evaluator
from drone import get_provider
from models import (
    EvaluateRequest, EvaluateResponse,
    ExecuteRequest, ExecuteResponse,
    TaskStatus, UavState,
)

app = FastAPI(
    title="UAV Local Agent",
    description="산불 관측 시뮬레이터 — UAV 상태 제공 및 수행 가능성 판단",
    version="0.1.0",
)

provider = get_provider()

# 이 프로세스가 담당하는 UAV 1대. 여러 대를 운용하면 서버를 나누고 각자 다른 값을 준다.
#   UAV_ID=A-uav1 ... --port 8000
#   UAV_ID=A-uav2 ... --port 8001
UAV_ID = os.getenv("UAV_ID", "A-uav1")

# 진행 중 Task. 프로세스 재시작 시 사라짐 (Mock 단계 한정)
_tasks: dict[str, dict] = {}


def _check_id(uav_id: str) -> None:
    """경로의 uav_id 가 이 서버 담당인지 확인한다.

    서버를 여러 대 띄우면 호출측이 주소를 잘못 고를 수 있다. 그대로 두면 엉뚱한
    기체의 상태를 반환하므로 404 로 끊는다. 조용히 틀린 값을 주는 쪽이 더 위험하다.
    """
    if uav_id != UAV_ID:
        raise HTTPException(
            404, f"이 서버는 {UAV_ID} 담당이다. 요청된 {uav_id} 는 다른 주소를 확인할 것"
        )


@app.get("/health")
async def health():
    return {"status": "ok", "mode": os.getenv("UAV_MODE", "mock"), "uav_id": UAV_ID}


@app.get("/uav/{uav_id}/state", response_model=UavState)
async def get_state(uav_id: str):
    _check_id(uav_id)
    return await provider.get_state(uav_id)


@app.post("/uav/{uav_id}/evaluate", response_model=EvaluateResponse)
async def evaluate_mission(uav_id: str, req: EvaluateRequest):
    """수행 가능성 판단. 실행하지 않는다.

    ACCEPT 는 Safety 검증을 거친 뒤 /execute 로 들어온다.
    """
    _check_id(uav_id)
    state = await provider.get_state(uav_id)
    return evaluator.evaluate(state, req)


async def _run_task(uav_id: str, req: ExecuteRequest):
    tid = req.task_id
    try:
        _tasks[tid] = {"task_id": tid, "status": "IN_PROGRESS",
                       "progress": {"phase": "ENROUTE"}, "observation": None}
        alt = req.target.alt_m_amsl + config.MIN_CLEARANCE_M
        await provider.goto(req.target.lat, req.target.lon, alt)

        _tasks[tid]["progress"] = {"phase": "OBSERVING"}
        await asyncio.sleep(2)  # Mock: 관측 소요를 짧게 흉내

        _tasks[tid] = {
            "task_id": tid,
            "status": "COMPLETED",
            "progress": {"phase": "DONE"},
            "observation": {
                "observed_at": datetime.now(timezone.utc).strftime(
                    "%Y-%m-%dT%H:%M:%SZ"),
                "position": {
                    "lat": req.target.lat,
                    "lon": req.target.lon,
                    "alt_m_amsl": req.target.alt_m_amsl,
                },
                "sensor_type": req.observation_type,
                # Mock 고정값. 실연동 시 실제 센서 결과로 교체
                "values": {"max_temp_c": 312.5, "hotspot_detected": True},
            },
        }
    except Exception as e:  # noqa: BLE001
        _tasks[tid] = {"task_id": tid, "status": "FAILED",
                       "progress": None, "observation": None, "error": str(e)}


@app.post("/uav/{uav_id}/execute", response_model=ExecuteResponse)
async def execute_mission(uav_id: str, req: ExecuteRequest):
    """Safety ALLOW 이후 호출. 비동기로 시작하고 즉시 반환한다."""
    _check_id(uav_id)
    if req.task_id in _tasks and _tasks[req.task_id]["status"] in (
        "STARTED", "IN_PROGRESS"
    ):
        raise HTTPException(409, f"task {req.task_id} already running")

    _tasks[req.task_id] = {"task_id": req.task_id, "status": "STARTED",
                           "progress": None, "observation": None}
    asyncio.create_task(_run_task(uav_id, req))
    return {
        "task_id": req.task_id,
        "status": "STARTED",
        "tracking_url": f"/uav/{uav_id}/task/{req.task_id}",
    }


@app.get("/uav/{uav_id}/task/{task_id}", response_model=TaskStatus)
async def get_task(uav_id: str, task_id: str):
    if task_id not in _tasks:
        raise HTTPException(404, f"task {task_id} not found")
    return _tasks[task_id]


# ── Mock 전용: 테스트를 위해 상태를 바꾼다 ──────────────────
@app.post("/mock/set")
async def mock_set(battery_pct: float | None = None,
                   wind_ms: float | None = None,
                   failsafe: bool | None = None,
                   gps_ok: bool | None = None,
                   link_quality: str | None = None):
    """IT-02 등 시나리오 재현용. UAV_MODE=real 에서는 사용 불가."""
    if os.getenv("UAV_MODE", "mock").lower() == "real":
        raise HTTPException(400, "mock mode only")
    if battery_pct is not None:
        provider.battery_pct = battery_pct
    if wind_ms is not None:
        provider.wind_ms = wind_ms
    if failsafe is not None:
        provider.failsafe = failsafe
    if gps_ok is not None:
        provider.gps_ok = gps_ok
    if link_quality is not None:
        provider.link_quality = link_quality
    return await provider.get_state(UAV_ID)
