# -*- coding: utf-8 -*-
"""
orchestrator/api.py
===================
총괄 서버 (FastAPI). 요청서 §9: health/status, Task 접수, 외부 event 접수,
사건/Task/decision/attempt/보류 이유 조회, 수동 해소·resume.

실행 (저장소 루트):
  python -m orchestrator.api                  # 기본 127.0.0.1:8200 (PROVISIONAL)
  ORCH_ENV_FIXTURE=path.json python -m orchestrator.api

주의
- 접수(202)는 출동 성공이 아니다. 출동 여부는 dispatch 결과·Task 상태로 확인한다.
- 같은 request_id 재전송은 같은 Task 로 수렴한다. 내용이 다르면 409.
- 환경은 팀 READ 계약 전까지 fixture 다 (/health 의 env.contract_complete 로 표시).
"""

import json
import os
import threading
from typing import List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from . import config
from .engine import Orchestrator
from .env_adapter import FixtureEnv
from .ledger import Ledger, RequestConflict
from .resources import UavClient, UgvClient


class TargetIn(BaseModel):
    lat: float
    lon: float
    ground_amsl_m: Optional[float] = Field(None, description="목표 지점 지면 해발고도(m). 비행고도 아님")
    cell_id: Optional[str] = None


class RequirementsIn(BaseModel):
    resource_types: List[str] = ["UAV"]
    sensor: Optional[str] = "THERMAL"
    target_agl_m: Optional[float] = None
    agl_range_m: Optional[List[float]] = None
    needs_env_ack: bool = True


class TaskIn(BaseModel):
    request_id: str = Field(..., description="멱등 키. 같은 값으로 재전송하면 같은 Task")
    incident_id: str = "INC-DEFAULT"
    kind: str = "RECON"
    target: TargetIn
    requirements: RequirementsIn = RequirementsIn()
    area_cell_ids: List[str] = Field(default_factory=list, description="임무가 다루는 환경 셀 목록")
    dispatch: bool = Field(True, description="접수 직후 배정까지 시도")


class ResolveIn(BaseModel):
    action: str = Field(..., description="RETRY / CANCEL / CONFIRM_RESOURCE_IDLE")
    reason: str
    attempt_id: Optional[str] = None


class EventIn(BaseModel):
    event_id: str
    type: str = Field(..., description="ENV_UPDATED / RESOURCE_CHANGED / ...")
    simulation_time_s: Optional[float] = None
    payload: dict = {}


def _task_view(orch: Orchestrator, task) -> dict:
    d = task.to_dict()
    d["attempts"] = orch.ledger.list_attempts(task.task_id)
    return d


def create_app(orch: Orchestrator, poll_interval_s: Optional[float] = None) -> FastAPI:
    app = FastAPI(title="ADAIR Orchestrator", version=config.SCHEMA_VERSION,
                  description="총괄 오케스트레이터 — 규칙 판단 + Local/Safety 검증")
    lock = threading.Lock()          # 판단·추적은 한 번에 하나씩 (장부 순서 보장)
    stop = threading.Event()

    if poll_interval_s:
        def loop():
            while not stop.wait(poll_interval_s):
                try:
                    with lock:
                        orch.poll()
                except Exception as e:  # noqa: BLE001 — 추적 루프는 멈추지 않는다
                    orch.ledger.log("POLL_ERROR", reason=type(e).__name__, detail={"error": str(e)})

        @app.on_event("startup")
        def _start():
            threading.Thread(target=loop, daemon=True, name="orch-poller").start()

        @app.on_event("shutdown")
        def _stop():
            stop.set()

    @app.get("/health")
    def health():
        snap = orch.env.read()
        return {"status": "ok", "schema_version": config.SCHEMA_VERSION,
                "env": snap.ref(),
                "uav_endpoints": orch.uav.resource_ids() if orch.uav else [],
                "ugv_server": getattr(orch.ugv, "base_url", None),
                "llm": {"provider": config.LLM_PROVIDER, "model": config.LLM_MODEL,
                        "key_present": bool(os.getenv(config.LLM_API_KEY_ENV)),
                        "ready": bool(os.getenv(config.LLM_API_KEY_ENV)) and config.LLM_TIMEOUT_S is not None},
                "f1_official_rule": config.OFFICIAL_RULE_SOURCE or "NOT_READY",
                "f2_suppression": "READY" if config.SUPPRESSION_CONTRACT_READY else "NOT_READY"}

    @app.post("/tasks", status_code=202)
    def create_task(body: TaskIn):
        data = body.model_dump()
        do_dispatch = data.pop("dispatch")
        with lock:
            try:
                task, created = orch.submit_task(data)
            except RequestConflict:
                raise HTTPException(409, "같은 request_id 에 다른 내용이 들어왔습니다")
            result = orch.dispatch(task.task_id) if (do_dispatch and created) else None
            task = orch.ledger.get_task(task.task_id)
        return {"accepted": True, "created": created, "task": _task_view(orch, task), "dispatch": result}

    @app.get("/tasks")
    def list_tasks(status: Optional[str] = None):
        tasks = orch.ledger.list_tasks([status] if status else None)
        return [t.to_dict() for t in tasks]

    @app.get("/tasks/{task_id}")
    def get_task(task_id: str):
        task = orch.ledger.get_task(task_id)
        if not task:
            raise HTTPException(404, "task not found")
        return _task_view(orch, task)

    @app.get("/tasks/{task_id}/decisions")
    def decisions(task_id: str):
        return orch.ledger.list_decisions(task_id)

    @app.get("/tasks/{task_id}/events")
    def task_events(task_id: str, after_seq: int = 0):
        return orch.ledger.events(task_id, after_seq)

    @app.post("/tasks/{task_id}/dispatch")
    def dispatch(task_id: str):
        with lock:
            return orch.dispatch(task_id)

    @app.post("/tasks/{task_id}/resolve")
    def resolve(task_id: str, body: ResolveIn):
        with lock:
            return orch.resolve(task_id, body.action, body.reason, body.attempt_id)

    @app.get("/attempts/{attempt_id}")
    def attempt(attempt_id: str):
        a = orch.ledger.get_attempt(attempt_id)
        if not a:
            raise HTTPException(404, "attempt not found")
        return a

    @app.get("/state")
    def state():
        tasks = orch.ledger.list_tasks()
        return {"env": orch.env.read().ref(),
                "tasks": {s: [t.task_id for t in tasks if t.purpose_status == s]
                          for s in sorted({t.purpose_status for t in tasks})},
                "holds": [{"task_id": t.task_id, "reason": t.hold_reason, "resume_condition": t.resume_condition}
                          for t in tasks if t.purpose_status == "HOLD"],
                "reservations": orch.ledger.reservations(),
                "active_attempts": [a for a in orch.ledger.list_attempts()
                                    if a["substatus"] not in ("RELEASED", "NOT_SENT", "FAILED", "CANCELLED")]}

    @app.post("/dispatch_pending")
    def dispatch_pending():
        with lock:
            return orch.dispatch_pending()

    @app.post("/poll")
    def poll():
        with lock:
            return orch.poll()

    @app.post("/events", status_code=202)
    def ingest_event(body: EventIn):
        with lock:
            first = orch.ledger.record_external_event(body.event_id, body.model_dump())
            if not first:
                return JSONResponse({"accepted": True, "duplicate": True}, status_code=200)
            redispatched = []
            if body.type in ("ENV_UPDATED", "RESOURCE_CHANGED"):
                # 새 입력·자원 변화가 재개 조건인 보류 Task 만 다시 평가한다
                for t in orch.ledger.list_tasks(["HOLD", "PENDING"]):
                    if t.purpose_status == "PENDING" or t.resume_condition in (
                            "NEW_RESOURCE_OR_INPUT", "ENV_SNAPSHOT_COMPLETE"):
                        redispatched.append({"task_id": t.task_id, **orch.dispatch(t.task_id)})
            return {"accepted": True, "duplicate": False, "redispatched": redispatched}

    return app


def build_default() -> Orchestrator:
    fixture = os.getenv("ORCH_ENV_FIXTURE")
    if fixture:
        with open(fixture, encoding="utf-8") as f:
            env = FixtureEnv(**json.load(f))
    else:
        env = FixtureEnv()
    return Orchestrator(Ledger(config.DB_PATH), env, UavClient(), UgvClient())


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(create_app(build_default(), poll_interval_s=config.POLL_INTERVAL_S),
                host=config.ORCH_HOST, port=config.ORCH_PORT)
