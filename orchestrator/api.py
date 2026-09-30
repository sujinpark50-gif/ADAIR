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
- /events: 잘못된 요청(422)은 event_id 를 쓰지 않는다 — 고쳐서 같은 event_id 로 다시 보낼 수 있다.
  같은 event_id·같은 내용 재전송은 200(duplicate), 같은 event_id·다른 내용은 409(EVENT_ID_CONFLICT).
- 잠금 두 개: lock = 추적·접수·취소 같은 짧은 장부 작업. dispatch_gate = 배정(기체 평가·출동 HTTP, LLM 호출).
  배정은 lock 을 잡지 않으므로, 기체 평가·출동 요청이나 LLM 응답이 느려도 추적(poll)·취소·접수가 기다리지 않는다
  (내부 과제 2). 배정끼리는 dispatch_gate 로 한 번에 하나씩이다. 장부는 자체 트랜잭션으로 스레드 간 일관성을 지키고,
  출동 요청을 보내는 중인 실행시도는 추적이 건드리지 않는다. LLM 추천은 배정 직전 입력과 같을 때만 쓴다.
- 환경은 팀 READ 계약 전까지 fixture 다 (/health 의 env.contract_complete 로 표시).
"""

import json
import os
import threading
from datetime import datetime, timezone
from typing import List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel, Field

from . import config
from .board import BOARD_HTML
from .engine import Orchestrator, TaskRejected
from .env_adapter import FixtureEnv
from .ledger import ACTIVE, Ledger, RequestConflict, RunNotActive
from .knowledge import InjectedAnalysis, KmaAsosReplay
from .llm import LlmPlanner
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
    needs_env_ack: Optional[bool] = Field(
        None, description="완료에 환경 반영 ACK 가 필요한지. 비우면 관측 종류로 정한다 (열화상·기상 = 필요, "
                          "도로 상황·관측 없음 = 불필요). 반영 계약이 없는 관측에 true 를 주면 422")
    extra_sensors: List[str] = Field(default_factory=list, description="같은 방문에서 함께 할 측정 (예: WEATHER)")


class TaskIn(BaseModel):
    request_id: str = Field(..., description="멱등 키. 같은 값으로 재전송하면 같은 Task")
    incident_id: str = "INC-DEFAULT"
    kind: str = "RECON"
    target: TargetIn
    requirements: RequirementsIn = RequirementsIn()
    area_cell_ids: List[str] = Field(default_factory=list,
                                     description="임무와 관련된 환경 셀 (우선순위 근거용). 완료 요구 범위가 아니다")
    required_cell_ids: List[str] = Field(
        default_factory=list, description="구역 임무(MONITOR)가 완료되려면 전체를 관측해야 하는 셀. "
                                          "비우면 area_cell_ids → 목표 칸. 정찰(RECON)에는 줄 수 없다")
    run_id: Optional[str] = Field(None, description="이 요청이 속한 실행(run). 주면 활성 run 과 같아야 한다 (다르면 409)")
    dispatch: bool = Field(True, description="접수 직후 배정까지 시도")


class ResolveIn(BaseModel):
    action: str = Field(..., description="RETRY / CANCEL / CONFIRM_RESOURCE_IDLE / RECHECK")
    reason: str
    attempt_id: Optional[str] = None
    request_id: Optional[str] = Field(None, description="RECHECK 멱등 키 (같은 값 재전송은 같은 재관측 Task)")


class ModeIn(BaseModel):
    mode: str = Field(..., description="HUMAN_CHOICE / AUTO_HIGHER_RISK")
    reason: str = ""


class ChooseIn(BaseModel):
    order: List[str] = Field(..., description="먼저 보낼 Task 순서. 빠진 Task 는 계속 대기")
    reason: str


class ManualIn(BaseModel):
    reason: str = Field(..., description="사람이 직접 보내는 이유 (기록)")


class EventIn(BaseModel):
    event_id: str
    type: str = Field(..., description="ENV_UPDATED / RESOURCE_CHANGED / ...")
    simulation_time_s: Optional[float] = None
    payload: dict = {}
    run_id: Optional[str] = Field(None, description="이벤트가 속한 실행(run). 주면 활성 run 과 같아야 한다 (다르면 409)")


def _one_observation(obs: dict, ack: Optional[dict]) -> dict:
    d = obs["detail"] or {}
    fp = d.get("footprint") or {}
    return {"result": obs["result"], "source": d.get("source"), "sensor_status": d.get("sensor_status"),
            "sensor_type": d.get("sensor_type"), "failure_reason": d.get("failure_reason"),
            "covered_cells": d.get("covered_cells"), "partial_cells": d.get("partial_cells"),
            "cell_coverage": d.get("cell_coverage"), "detections": d.get("detections"),
            "received_after_purpose": d.get("received_after_purpose"),
            "target_covered": d.get("target_covered"), "resource_id": obs["resource_id"],
            "footprint": {k: fp.get(k) for k in ("width_m", "height_m", "agl_m")} if fp else None,
            "simulation_time_s": d.get("simulation_time_s"), "observed_wall": d.get("observed_wall"),
            "is_fire_observation": d.get("is_fire_observation", True),
            "values": d.get("values"), "scope": d.get("scope"), "same_visit_as": d.get("same_visit_as"),
            "env_apply": None if ack is None else {"result": ack["result"],
                                                   "state_version": (ack["detail"] or {}).get("state_version")}}


def observation_summary(orch: Orchestrator, task_id: str) -> Optional[dict]:
    """Task 의 주 관측(임무 센서) 요약 + 같은 방문에서 함께 한 측정(extras)"""
    task = orch.ledger.get_task(task_id)
    main_sensor = task.requirements.sensor if task else None
    items, by_oid = [], {}                       # [(obs_event, ack_event)] — 반영 결과는 관측 ID 로 짝짓는다
    for e in orch.ledger.events(task_id):
        if e["event_type"] == "OBSERVATION":
            items.append([e, None])
            by_oid[(e["detail"] or {}).get("observation_id")] = items[-1]
        elif e["event_type"] == "ENVIRONMENT_APPLY":
            oid = (e["detail"] or {}).get("observation_id")
            it = by_oid.get(oid) if oid else (items[-1] if items and items[-1][1] is None else None)
            if it is not None:
                it[1] = e                        # 같은 관측의 마지막 결과 (시간 초과 뒤 ACK 등)
    if not items:
        return None

    def kind(e):
        d = e["detail"] or {}
        return d.get("sensor_type") or ("ROAD_STATUS" if e["result"] in ("ROAD_STATUS", "NONE") else None)
    mains = [it for it in items if kind(it[0]) == main_sensor or main_sensor is None
             or (main_sensor == "ROAD_STATUS" and kind(it[0]) == "ROAD_STATUS")]
    main = (mains or items)[-1]
    extras = {}
    for it in items:
        if it is not main and kind(it[0]) != kind(main[0]):
            extras[kind(it[0]) or "OTHER"] = _one_observation(*it)
    out = _one_observation(*main)
    out["extras"] = list(extras.values())
    out["completion"] = next((e["detail"] for e in reversed(orch.ledger.events(task_id))
                              if e["event_type"] == "TASK_COMPLETE"), None)
    return out


def _task_view(orch: Orchestrator, task) -> dict:
    d = task.to_dict()
    d["attempts"] = orch.ledger.list_attempts(task.task_id)
    if task.required_cell_ids:
        p = orch.area_progress(task)
        d["area_progress"] = {k: p[k] for k in ("required", "fraction", "remaining", "geometry_missing",
                                                "unvisited", "visited", "complete")}
    d["observation"] = observation_summary(orch, task.task_id)
    return d


def create_app(orch: Orchestrator, poll_interval_s: Optional[float] = None) -> FastAPI:
    app = FastAPI(title="ADAIR Orchestrator", version=config.SCHEMA_VERSION,
                  description="총괄 오케스트레이터 — 규칙 판단 + Local/Safety 검증")
    lock = threading.Lock()          # 판단·추적은 한 번에 하나씩 (장부 순서 보장)
    dispatch_gate = threading.Lock() # 배정(평가·출동 HTTP·LLM)끼리 직렬화. 추적·취소·접수는 이 잠금을 기다리지 않는다
    stop = threading.Event()

    def dispatch_all() -> dict:
        """대기 임무 배정. LLM 호출과 기체 평가·출동 HTTP 모두 lock 밖에서 한다 (dispatch_gate 로만 직렬화)."""
        with dispatch_gate:
            req = orch.llm_request()
            if req:
                orch.llm_store(req, orch.llm_call(req))
            return orch.dispatch_pending(allow_llm_call=False)

    @app.on_event("startup")
    def _recover_on_start():
        # 송신 전후에 멈춘 PREPARED 시도를 제공자 상태와 대조 (재송신 없음)
        with dispatch_gate, lock:
            orch.recover()

    if poll_interval_s:
        dispatch_wanted = threading.Event()

        def dispatcher():
            # 자동 배정 전용 작업자. 추적 루프는 배정을 부탁만 하고(dispatch_wanted) 기다리지 않는다 —
            # 평가·출동 요청이 느려도 주기적 추적은 계속 돈다 (내부 과제 2)
            while not stop.is_set():
                if not dispatch_wanted.wait(0.2):
                    continue
                dispatch_wanted.clear()
                try:
                    dispatch_all()
                except Exception as e:  # noqa: BLE001 — 배정 작업자는 멈추지 않는다 (기록만)
                    orch.ledger.log("DISPATCH_ERROR", reason=type(e).__name__, detail={"error": str(e)})

        def loop():
            while not stop.wait(poll_interval_s):
                try:
                    with lock:
                        changed = bool(orch.poll())
                        created = bool(orch.sync()["created"])     # 그 시각까지의 신고 → 최초 정찰 임무
                    if (changed or created) and config.AUTO_DISPATCH_ON_CHANGE:
                        dispatch_wanted.set()         # 진행 상황이 바뀌면(반납·재대기·새 신고 등) 배정 작업자에게 부탁
                except Exception as e:  # noqa: BLE001 — 추적 루프는 멈추지 않는다
                    orch.ledger.log("POLL_ERROR", reason=type(e).__name__, detail={"error": str(e)})

        @app.on_event("startup")
        def _start():
            threading.Thread(target=loop, daemon=True, name="orch-poller").start()
            threading.Thread(target=dispatcher, daemon=True, name="orch-dispatcher").start()

        @app.on_event("shutdown")
        def _stop():
            stop.set()

    @app.get("/health")
    def health():
        snap = orch.view()
        env_source = getattr(orch.env, "source", "UNKNOWN")
        return {"status": "ok", "schema_version": config.SCHEMA_VERSION,
                "truth_env": {"class": type(orch.env).__name__, "source": env_source,
                              "shared_team_env": env_source not in ("FIXTURE", "UNKNOWN"),
                              "note": "시험용 가짜 환경. 팀 공유 환경(ENV-01) 연결 아님" if env_source == "FIXTURE" else None},
                "env": snap.ref(),
                "run": {"active": orch.ledger.active_run(), "policy": "SINGLE_ACTIVE_RUN", "gate": snap.run_gate,
                        "legacy_rows_quarantined": orch.ledger.migrated_from_legacy},
                "weather_policy": orch.weather_policy(),
                "view": {"source": snap.source, "analysis_source": snap.analysis_source,
                         "scenario_start_kst": snap.scenario_start_kst,
                         "weather_feeds": [getattr(f, "source", type(f).__name__) for f in orch.weather_feeds]},
                "uav_endpoints": orch.uav.resource_ids() if orch.uav else [],
                "ugv_server": getattr(orch.ugv, "base_url", None),
                "llm": {"provider": config.LLM_PROVIDER, "model": config.LLM_MODEL,
                        "key_present": bool(os.getenv(config.LLM_API_KEY_ENV)),
                        "timeout_s": config.LLM_TIMEOUT_S, "max_calls": config.LLM_MAX_CALLS_PER_RUN,
                        "not_ready_reason": orch.llm.status() if orch.llm else "NOT_CONNECTED",
                        "calls_used": orch.llm.calls if orch.llm else 0},
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
            except TaskRejected as e:
                raise HTTPException(422, {"accepted": False, "reason": str(e)})
            except RunNotActive as e:
                raise HTTPException(409, {"accepted": False, "reason": f"RUN_NOT_ACTIVE:{e}",
                                          "active_run": orch.ledger.active_run()})
        result = None
        if do_dispatch and created:
            if config.AUTO_DISPATCH_ON_CHANGE:
                # 다른 대기 임무와 함께 우선순위 순서로 배정
                result = dispatch_all()["results"].get(task.task_id)
            else:
                with dispatch_gate:
                    result = orch.dispatch(task.task_id)
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
        with dispatch_gate:
            return orch.dispatch(task_id)

    @app.post("/tasks/{task_id}/manual_dispatch")
    def manual_dispatch(task_id: str, body: ManualIn):
        with dispatch_gate:
            return orch.manual_dispatch(task_id, body.reason)

    @app.post("/tasks/{task_id}/resolve")
    def resolve(task_id: str, body: ResolveIn):
        # RETRY 는 재배정(평가·출동 HTTP)을 하므로 배정 잠금으로, 나머지(취소·idle 확인·재관측 생성)는 짧은 장부 작업
        with (dispatch_gate if body.action == "RETRY" else lock):
            return orch.resolve(task_id, body.action, body.reason, body.attempt_id, body.request_id)

    @app.get("/attempts/{attempt_id}")
    def attempt(attempt_id: str):
        a = orch.ledger.get_attempt(attempt_id)
        if not a:
            raise HTTPException(404, "attempt not found")
        return a

    @app.get("/state")
    def state():
        tasks = orch.ledger.list_tasks()
        snap = orch.view()
        return {"env": snap.ref(),
                "run": {"active": orch.ledger.active_run(), "gate": snap.run_gate},
                "tasks": {s: [t.task_id for t in tasks if t.purpose_status == s]
                          for s in sorted({t.purpose_status for t in tasks})},
                "holds": [{"task_id": t.task_id, "reason": t.hold_reason, "resume_condition": t.resume_condition}
                          for t in tasks if t.purpose_status == "HOLD"],
                "reservations": orch.ledger.reservations(),
                "active_attempts": [a for a in orch.ledger.list_attempts()
                                    if a["substatus"] not in ("RELEASED", "NOT_SENT", "FAILED", "CANCELLED")]}

    @app.post("/dispatch_pending")
    def dispatch_pending():
        return dispatch_all()

    @app.get("/priority/board")
    def priority_board():
        """관제 화면용: 최근 우선순위 판단의 선택 묶음과 각 Task 의 현재 상태·근거"""
        last = next((e for e in reversed(orch.ledger.events(run_id=ACTIVE))
                     if e["event_type"] == "PRIORITY_ORDER"), None)
        detail = (last or {}).get("detail") or {}
        evidence = detail.get("evidence", {})

        def row(tid):
            t = orch.ledger.get_task(tid)
            ev = evidence.get(tid, {})
            atts = orch.ledger.list_attempts(tid)
            return {"task_id": tid, "kind": t.kind if t else None, "cell_id": t.target.cell_id if t else None,
                    "purpose_status": t.purpose_status if t else None, "hold_reason": t.hold_reason if t else None,
                    "resume_condition": t.resume_condition if t else None,
                    "human_risk": ev.get("human_risk"), "human_risk_sources": ev.get("human_risk_sources"),
                    "human_risk_basis": ev.get("human_risk_basis"), "forecast_arrival_s": ev.get("forecast_arrival_s"),
                    "risk_score": ev.get("risk_score"), "distance_m": ev.get("distance_m"),
                    "nearest_protected": ev.get("nearest_protected"),
                    "resource_id": atts[-1]["resource_id"] if atts else None,
                    "observation": observation_summary(orch, tid)}
        groups = [{**g, "tasks": [row(t) for t in g["task_ids"]],
                   "awaiting": [t for t in g["task_ids"]
                                if (orch.ledger.get_task(t) or None) and orch.ledger.get_task(t).hold_reason
                                == "AWAITING_PRIORITY_CHOICE"]} for g in detail.get("groups", [])]
        results = []
        for t in orch.ledger.list_tasks():
            o = observation_summary(orch, t.task_id)
            if o:
                results.append({**row(t.task_id), "observation": o})
        results.sort(key=lambda r: r["observation"].get("observed_wall") or "", reverse=True)
        llm_state = orch.llm.status() if orch.llm else "NOT_CONNECTED"
        snap = orch.view()
        now = snap.simulation_time_s or 0.0
        field = orch.kb.field_weather(now) if not snap.run_gate else {"valid": [], "excluded": []}
        # 기상은 분석 입력(belief)과 같은 함수로 만든다 — 화면 값과 판단 근거가 어긋나지 않는다
        known = {"fires": [{"cell_id": c, **v} for c, v in sorted(orch.kb.fire_states(now).items())],
                 "stations": orch.kb.latest_station_obs(now), "field_weather": field["valid"],
                 "field_weather_excluded": field["excluded"], "weather_policy": orch.weather_policy(),
                 "scenario_start_kst": snap.scenario_start_kst, "run_id": orch.ledger.active_run(),
                 "run_gate": snap.run_gate,
                 "simulation_time_s": now, "analysis_source": snap.analysis_source}
        premon_tasks = {t.request_key: t.task_id for t in orch.ledger.list_tasks()
                        if (t.request_key or "").startswith("AUTO-PREMON:")}
        forecast = {"ref": snap.forecast_ref, "cell_count": len(snap.spread_forecast),
                    "sim_time_s": snap.simulation_time_s,
                    "preemptive_monitor_enabled": config.PREEMPTIVE_MONITOR_ENABLED,
                    "threats": [{**c, "monitor_task_id": premon_tasks.get(f"AUTO-PREMON:{snap.run_id}:{c['key']}")}
                                for c in orch.preemptive_candidates(snap)]}
        return {"mode": orch.priority_mode, "modes": list(config.PRIORITY_MODES), "results": results,
                "llm": {"model": config.LLM_MODEL, "not_ready_reason": llm_state}, "forecast": forecast,
                "known": known,
                "similar_delta": config.PRIORITY_SIMILAR_RISK_DELTA, "decided_seq": (last or {}).get("seq"),
                "auto_order": [row(t) for t in detail.get("auto_order", [])], "groups": groups,
                "criteria": detail.get("criteria"), "rule": detail.get("rule")}

    @app.post("/priority/mode")
    def priority_mode(body: ModeIn):
        with lock:
            return orch.set_priority_mode(body.mode, body.reason)

    @app.post("/priority/choose")
    def priority_choose(body: ChooseIn):
        with dispatch_gate:
            return orch.choose_priority(body.order, body.reason)

    @app.get("/board", response_class=HTMLResponse)
    def board():
        return BOARD_HTML

    @app.post("/poll")
    def poll():
        with lock:
            return orch.poll()

    @app.post("/events", status_code=202)
    def ingest_event(body: EventIn):
        # 검증 → 접수키 확정 → 반영(신고면 '신고됨' 기록 + 최초 정찰 Task)까지가 접수. 배정은 그 뒤에 따로 한다.
        with lock:
            code, out = orch.ingest_event(body.model_dump())
        if code != 202:
            return JSONResponse(out, status_code=code)
        redispatched = []
        if out.pop("needs_dispatch"):
            if config.AUTO_DISPATCH_ON_CHANGE:
                redispatched = [{"task_id": k, **v} for k, v in dispatch_all()["results"].items()]
            else:
                with dispatch_gate:
                    # 새 입력·자원 변화가 재개 조건인 보류 Task 만 다시 평가한다
                    for t in orch.ledger.list_tasks(["HOLD", "PENDING"]):
                        if t.purpose_status == "PENDING" or t.resume_condition in (
                                "NEW_RESOURCE_OR_INPUT", "ENV_SNAPSHOT_COMPLETE", "MAP_INFO", "RUN_ACTIVE"):
                            redispatched.append({"task_id": t.task_id, **orch.dispatch(t.task_id)})
        return {**out, "redispatched": redispatched}

    return app


def build_default() -> Orchestrator:
    fixture = os.getenv("ORCH_ENV_FIXTURE")
    injected = {}
    if fixture:
        with open(fixture, encoding="utf-8") as f:
            data = json.load(f)
        # 분석 시험 주입값은 진짜 세계(환경)와 분리해서 읽는다
        injected = data.pop("injected_analysis", {}) or {}
    else:
        data = {}
    # 가짜 환경은 서버를 켤 때마다 0초·버전 1부터 다시 시작한다 → 같은 run_id 를 다시 쓰면 이전 실행의
    # 신고·관측·임무가 섞인다. ORCH_RUN_ID 를 주지 않으면 켤 때마다 새 run_id 를 만든다 (B05).
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    data["run_id"] = config.FIXTURE_RUN_ID or f"{data.get('run_id', 'RUN-FIXTURE')}-{stamp}"
    env = FixtureEnv(**data)
    feeds = []
    if config.KMA_ASOS_CSV.is_file() and config.KMA_ASOS_STATIONS.is_file():
        feeds.append(KmaAsosReplay(config.KMA_ASOS_CSV, config.KMA_ASOS_STATIONS))
    return Orchestrator(Ledger(config.DB_PATH), env, UavClient(), UgvClient(), llm=LlmPlanner(),
                        analysis=InjectedAnalysis(**injected), weather_feeds=feeds)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(create_app(build_default(), poll_interval_s=config.POLL_INTERVAL_S),
                host=config.ORCH_HOST, port=config.ORCH_PORT)
