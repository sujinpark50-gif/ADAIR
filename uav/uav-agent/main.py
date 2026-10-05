"""UAV Local Agent — REST API.

UAV 1대당 프로세스 1개다. 여러 대를 운용하면 UAV_ID 와 포트를 달리해 따로 띄운다.

실행:
  UAV_ID=A-uav1 UAV_MODE=mock uvicorn main:app --host 0.0.0.0 --port 8000
  UAV_ID=A-uav2 UAV_MODE=mock uvicorn main:app --host 0.0.0.0 --port 8001

  # PX4 연동. PX4 1개가 약 1.1 코어를 쓰므로 real 은 서버당 1대만 올린다.
  UAV_ID=A-uav1 UAV_MODE=real PX4_ADDRESS=udpin://0.0.0.0:14540 \
      uvicorn main:app --host 0.0.0.0 --port 8000

엔드포인트:
  GET  /uav/{id}/state                  상태
  GET  /uav/{id}/capabilities           지원 기능 (센서·측정·체류 범위·중단·경로·카메라)
  POST /uav/{id}/evaluate               수행 가능성 판단 (실행 안 함). ACCEPT 면 route_id·route_hash
  POST /uav/{id}/execute                실행 (Safety ALLOW 이후). 같은 task_id 재요청은 재출동 없음
  GET  /uav/{id}/task/{task_id}         실행 상태
  POST /uav/{id}/task/{task_id}/abort   중단 요청 → 관측 전이면 임무를 끊고 기지로 복귀

문서: http://<host>:8000/docs
"""
import asyncio
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException

import config
import evaluator
import route as route_mod
from drone import get_provider
from models import (
    AbortRequest,
    EvaluateRequest, EvaluateResponse,
    ExecuteRequest, ExecuteResponse,
    TaskStatus, UavState,
)

app = FastAPI(
    title="UAV Local Agent",
    description="산불 관측 시뮬레이터 — UAV 상태 제공 및 수행 가능성 판단",
    version="0.2.0",
)

# 이 프로세스가 담당하는 UAV 1대. 여러 대를 운용하면 서버를 나누고 각자 다른 값을 준다.
#   UAV_ID=A-uav1 ... --port 8000
#   UAV_ID=A-uav2 ... --port 8001
# 앞글자가 소속 기지다(A=원통119, B=기린119). Mock 은 그 기지에서 출발·복귀한다.
UAV_ID = os.getenv("UAV_ID", "A-uav1")

provider = get_provider(UAV_ID)
UAV_MODE = os.getenv("UAV_MODE", "mock").lower()

# ── 실행 기록 (UAV-01): 실행 키 = task_id. 파일에 저장해 재시작 뒤에도 같은 키로 조회·재요청 가능 ──
# 저장소 루트의 interfaces/exec_store.py 를 쓴다 (UGV 서버와 같은 규칙). 루트는 sys.path 맨 뒤에 붙여
# 이 폴더의 config.py 가 루트 config.py 에 가려지지 않게 한다. (route.py 의 gz_bridge 도 루트에 있다)
_REPO_ROOT = str(Path(__file__).resolve().parents[2])
if _REPO_ROOT not in sys.path:
    sys.path.append(_REPO_ROOT)
from interfaces.exec_store import ExecStore  # noqa: E402

_STATE_DIR = Path(os.getenv("UAV_STATE_DIR", str(Path(__file__).resolve().parent / ".state")))
STORE = ExecStore(_STATE_DIR / f"tasks_{UAV_ID}.json")
_tasks: dict[str, dict] = STORE.tasks
_save = STORE.save
# 평가한 경로 (UAV-03). 실행 요청의 route_id·route_hash 를 여기서 찾는다
ROUTES = route_mod.RouteStore(_STATE_DIR / f"routes_{UAV_ID}.json")

# 재시작 시 끊긴 실행 정리. 비행 코루틴은 프로세스와 함께 사라졌다.
#   mock: MockDrone 은 재시작하면 기지에 서 있다 → 물리 상태를 안다 (phase=DONE)
#   real: 기체가 아직 떠 있을 수 있다 → 모른다고 보고한다 (phase=AGENT_RESTARTED, 총괄은 점유 유지·수동 해소)
_RESTART_PHASE = "DONE" if UAV_MODE != "real" else "AGENT_RESTARTED"


def _was_active(t: dict) -> bool:
    phase = (t.get("progress") or {}).get("phase")
    return t["status"] in ("STARTED", "IN_PROGRESS") or phase == "RETURNING"


def _close_after_restart(t: dict) -> None:
    if t["status"] in ("STARTED", "IN_PROGRESS"):
        # 관측 전에 끊김 → 관측 결과를 비우고 실패로 (총괄은 다른 기체로 인계)
        t.update({"status": "FAILED", "reason": "AGENT_RESTARTED_BEFORE_OBSERVATION", "observation": None,
                  "error": "agent restarted before observation"})
    # 관측 뒤(COMPLETED, 복귀 중)에 끊긴 실행은 관측 ID·내용·시각을 그대로 둔다 (UAV-10 ②)
    t["progress"] = {"phase": _RESTART_PHASE}
    if _RESTART_PHASE != "DONE":
        t["physical_failure_reason"] = "AGENT_RESTARTED"


RESTART_CLOSED = STORE.reconcile_after_restart(
    _was_active, _close_after_restart,
    basis="MOCK_PROVIDER_RESET_TO_HOME" if _RESTART_PHASE == "DONE" else "PHYSICAL_STATE_UNKNOWN_AFTER_RESTART")


def _check_id(uav_id: str) -> None:
    """경로의 uav_id 가 이 서버 담당인지 확인한다.

    서버를 여러 대 띄우면 호출측이 주소를 잘못 고를 수 있다. 그대로 두면 엉뚱한
    기체의 상태를 반환하므로 404 로 끊는다. 조용히 틀린 값을 주는 쪽이 더 위험하다.
    """
    if uav_id != UAV_ID:
        raise HTTPException(
            404, f"이 서버는 {UAV_ID} 담당이다. 요청된 {uav_id} 는 다른 주소를 확인할 것"
        )


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ── 상태 보고 (UAV-10): 관측 결과와 물리 진행을 따로 본다 ──────────
# progress.phase 는 기존 값을 그대로 쓰고(총괄은 phase=DONE 을 착륙으로 본다), 같은 사실을 physical_state 로도 준다.
_PHASE_TO_PHYSICAL = {
    "ENROUTE": "ENROUTE", "OBSERVING": "OBSERVING", "RETURNING": "RETURNING", "DONE": "LANDED",
    "RETURN_FAILED": "RETURN_FAILED", "RETASKED": "RETASKED", "AGENT_RESTARTED": "UNKNOWN",
}


def _report(t: dict) -> dict:
    status = t["status"]
    phase = (t.get("progress") or {}).get("phase")
    if phase is None:
        physical = "PREPARING" if status == "STARTED" else "UNKNOWN"
    else:
        physical = _PHASE_TO_PHYSICAL.get(phase, phase)
    result = {"COMPLETED": "OBSERVED", "FAILED": "NOT_OBSERVED"}.get(status, "PENDING")
    return {**t, "mission_result": result, "physical_state": physical}


# 기지로 돌아오는 중인 비행. 새 임무가 오면 끊고 그 자리에서 출발한다 (UAV-06, RETASK_WHILE_RETURNING).
_returning: asyncio.Task | None = None
# 실행 키 → 비행 코루틴. 중단 요청(UAV-09)이 이것을 끊는다
_flights: dict[str, asyncio.Task] = {}
# 실행 키 → 비행 코루틴이 중단을 처리했다는 신호
_abort_done: dict[str, asyncio.Event] = {}


def _running_task_id() -> str | None:
    """진행 중인 Task id. 드론 provider 는 Task 를 모르므로 여기서 채운다.

    복귀 중(RETURNING)은 여기 들지 않는다. 복귀 중 재배정 여부는 _returning_task_id 와
    config.RETASK_WHILE_RETURNING 으로 따로 본다 (기본: 착륙까지 거절, UAV-06).
    """
    for tid, t in _tasks.items():
        if t["status"] in ("STARTED", "IN_PROGRESS"):
            return tid
    return None


def _returning_task_id() -> str | None:
    for tid, t in _tasks.items():
        if (t.get("progress") or {}).get("phase") == "RETURNING":
            return tid
    return None


async def _state(uav_id: str) -> dict:
    state = await provider.get_state(uav_id)
    state["current_task_id"] = _running_task_id()
    state["returning_task_id"] = _returning_task_id()
    return state


@app.get("/health")
async def health():
    return {"status": "ok", "mode": os.getenv("UAV_MODE", "mock"), "uav_id": UAV_ID,
            "exec_store": {"path": str(STORE.path), "restarts": STORE.restarts,
                           "closed_after_restart": RESTART_CLOSED, "tasks": len(_tasks)},
            "routes": len(ROUTES.routes)}


@app.get("/uav/{uav_id}/state", response_model=UavState)
async def get_state(uav_id: str):
    _check_id(uav_id)
    return await _state(uav_id)


@app.get("/uav/{uav_id}/capabilities")
async def get_capabilities(uav_id: str):
    """이 기체가 받는 요청 형식과 지원 기능. 총괄이 하드코딩하지 않고 여기서 읽을 수 있다."""
    _check_id(uav_id)
    return {
        "uav_id": UAV_ID,
        "mode": UAV_MODE,
        "sensors": config.available_sensors(),
        "measurements_per_visit": True,                # UAV-07: measurements=[...]
        "observe_duration_s": {"default": config.OBSERVE_DURATION_S, "min": config.OBSERVE_DURATION_MIN_S,
                               "max": config.OBSERVE_DURATION_MAX_S},   # UAV-02
        "counter_fields": ["observe_duration_s"],      # COUNTER 가 제안하는 필드 (요청에 그대로 실린다)
        "deadline_field": "remaining_time_s",          # UAV-04 (시뮬레이션 초)
        "abort": True,                                 # UAV-09
        "retask_while_returning": config.RETASK_WHILE_RETURNING,   # UAV-06
        "route": {"version": config.ROUTE_VERSION, "dem": route_mod._gb() is not None,
                  "required_on_execute": config.ROUTE_REQUIRED},   # UAV-03
        "observation_values": {                        # UAV-05: 값이 실측인지
            "THERMAL": "MOCK_FIXED_PLACEHOLDER" if config.placeholder_thermal() else "NO_THERMAL_PIPELINE",
            "WEATHER": "MOCK_PROVIDER_VALUE" if "WEATHER" in config.available_sensors() else None,
            "RGB": "NO_SENSOR_PIPELINE",
        },
        "camera": config.CAMERA,                       # UAV-08: 실제 카메라 미확인
        "patrol": False,                               # UAV-08 순찰·스캔 비행은 향후
    }


@app.post("/uav/{uav_id}/evaluate", response_model=EvaluateResponse)
async def evaluate_mission(uav_id: str, req: EvaluateRequest):
    """수행 가능성 판단. 실행하지 않는다.

    ACCEPT 는 Safety 검증을 거친 뒤 /execute 로 들어온다. 그때 route_id·route_hash 를 실으면
    평가한 경로 그대로 난다 (UAV-03).
    """
    _check_id(uav_id)
    state = await _state(uav_id)
    resp, rt = evaluator.evaluate(state, req, await provider.home())
    if rt is not None:
        ROUTES.put(rt)
    return resp


class _ReturnMarginLow(Exception):
    pass


async def _monitored(uav_id: str, home, coro) -> None:
    """비행·체류 중 배터리를 감시한다 (시나리오 5).

    지금 돌아가도 복귀 후 잔량이 BATTERY_RESERVE_PCT 미만이 되면 동작을 끊는다.
    출발 전 evaluate 가 여유를 확인했으므로, 예측보다 빨리 닳았을 때만 걸린다.
    """
    task = asyncio.create_task(coro)
    try:
        while True:
            done, _ = await asyncio.wait({task}, timeout=config.MONITOR_INTERVAL_S)
            if done:
                return task.result()
            margin = evaluator.return_margin_now(await provider.get_state(uav_id), home)
            if margin < config.BATTERY_RESERVE_PCT:
                raise _ReturnMarginLow(
                    f"return margin {margin:.1f}% < reserve {config.BATTERY_RESERVE_PCT:.0f}%")
    finally:
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


def _measure(kind: str, tid: str, observed_at: str) -> dict:
    """측정 하나의 결과. 실제 센서 처리가 없으므로 값이 어디서 왔는지 value_status·basis 로 밝힌다 (UAV-05)."""
    if kind == "THERMAL" and config.placeholder_thermal():
        values, status, basis = dict(config.MOCK_THERMAL_VALUES), "SIMULATED", "MOCK_FIXED_PLACEHOLDER"
    elif kind == "WEATHER":
        # mock 전용 (실기체는 평가에서 거절). 풍속만 MockDrone 값, 나머지는 없음
        values = {"wind_ms": getattr(provider, "wind_ms", None), "wind_dir_deg": None,
                  "temperature_c": None, "humidity_pct": None}
        status, basis = "SIMULATED", "MOCK_PROVIDER_VALUE"
    else:
        values, status = {}, "NO_DATA"
        basis = "NO_THERMAL_PIPELINE" if kind == "THERMAL" else "NO_SENSOR_PIPELINE"
    return {"measurement_id": f"{UAV_ID}:{tid}:{kind}", "sensor_type": kind, "values": values,
            "value_status": status, "basis": basis, "observed_at": observed_at}


async def _run_task(uav_id: str, req: ExecuteRequest, rt: dict):
    """순항고도로 목표 상공 → 관측고도로 하강·체류 → (결과 보고) → 순항고도로 기지 복귀.

    COMPLETED 는 관측을 마친 시점이다. 이후 복귀하는 동안 progress.phase 는
    RETURNING 이고, 착륙하면 DONE. 복귀에 실패해도 COMPLETED 와 관측은 바뀌지 않는다 (UAV-10).
    중단 요청(UAV-09)을 관측 전에 받으면 FAILED/ABORTED 로 바꾸고 기지로 돌아온다.
    """
    tid = req.task_id
    t = _tasks[tid]                       # execute 가 등록한 기록을 그대로 고친다 (실행 키 기록 유지)
    t.update({"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}, "observation": None})
    _save()
    obs_alt, cruise = rt["obs_alt_m_amsl"], rt["cruise_alt_m_amsl"]
    measurements = evaluator.measurements_of(req)
    try:
        home = await provider.home()
        await _monitored(uav_id, home, provider.goto(req.target.lat, req.target.lon, cruise))
        if cruise > obs_alt:
            await _monitored(uav_id, home, provider.goto(req.target.lat, req.target.lon, obs_alt))

        t["progress"] = {"phase": "OBSERVING"}
        _save()
        await _monitored(uav_id, home, provider.hold(evaluator.observe_seconds(req)))

        pos = (await provider.get_state(uav_id))["position"]
        observed_at = _now()
        results = [_measure(m, tid, observed_at) for m in measurements]
        primary = results[0] if results else None
        t.update({
            "status": "COMPLETED",
            "progress": {"phase": "RETURNING"},
            "observation": {
                # 관측 ID·내용·시각은 이후 복귀 실패·재시작에도 바뀌지 않는다 (UAV-10 ②)
                "observation_id": f"{UAV_ID}:{tid}:OBS",
                "observed_at": observed_at,
                # 목표 좌표가 아니라 관측 시점의 실제 기체 위치
                "position": {
                    "lat": pos["lat"],
                    "lon": pos["lon"],
                    "alt_m_amsl": pos["alt_m_amsl"],
                },
                # 기존 형식 호환: 첫 측정을 sensor_type·values 로도 준다. 측정 전체는 measurements
                "sensor_type": primary["sensor_type"] if primary else None,
                "values": primary["values"] if primary else {},
                "value_status": primary["value_status"] if primary else "NO_MEASUREMENT",
                "measurements": results,
            },
        })
        _save()
    except asyncio.CancelledError:
        if not t.get("abort"):
            raise                                  # 중단 요청이 아닌 취소(서버 종료 등)는 그대로 전파
        asyncio.current_task().uncancel()          # 중단을 처리했다 — 복귀 비행은 계속한다
        t.update({"status": "FAILED", "reason": "ABORTED", "observation": None,
                  "error": "aborted by request before observation", "progress": {"phase": "RETURNING"}})
        _save()
        _abort_done.get(tid, asyncio.Event()).set()
    except _ReturnMarginLow as e:
        # 임무를 끊고 돌아온다. 총괄은 FAILED + reason 을 보고 다른 기체에 인계한다.
        t.update({"status": "FAILED", "reason": "RETURN_MARGIN_INSUFFICIENT",
                  "error": str(e), "progress": {"phase": "RETURNING"}})
        _save()
    except Exception as e:  # noqa: BLE001
        t.update({"status": "FAILED", "progress": None, "error": str(e),
                  "physical_failure_reason": "FLIGHT_ERROR"})
        _save()
        return

    global _returning
    _returning = asyncio.current_task()
    try:
        await provider.return_home(cruise)
        t["progress"] = {"phase": "DONE"}
    except asyncio.CancelledError:
        t["progress"] = {"phase": "RETASKED"}   # 복귀 중 새 임무를 받음
    except Exception as e:  # noqa: BLE001
        t["progress"] = {"phase": "RETURN_FAILED"}
        t["error"] = f"return failed: {e}"
        t["physical_failure_reason"] = "RETURN_FAILED"
    finally:
        _save()
        _flights.pop(tid, None)
        if _returning is asyncio.current_task():
            _returning = None


def _resolve_route(req: ExecuteRequest, state: dict, home, obs_alt: float, observe_s: int,
                   measurements: list[str]) -> dict:
    """실행할 경로. route_id·route_hash 를 주면 평가한 경로와 맞는지 확인하고, 안 주면 지금 새로 잡는다."""
    if req.route_id is None and req.route_hash is None:
        if config.ROUTE_REQUIRED:
            raise HTTPException(409, {"reason": "ROUTE_REQUIRED",
                                      "detail": "평가 응답의 route_id·route_hash 를 실행 요청에 실어야 한다"})
        rt = route_mod.plan(UAV_ID, state["position"], req.target, home, obs_alt, observe_s, measurements)
        ROUTES.put(rt)
        return rt
    rt = ROUTES.find(req.route_id, req.route_hash)
    if rt is None:
        raise HTTPException(409, {"reason": "ROUTE_NOT_FOUND", "route_id": req.route_id,
                                  "route_hash": req.route_hash, "detail": "이 경로로 평가한 기록이 없다 (다시 평가)"})
    if rt["uav_id"] != UAV_ID or not route_mod.params_match(rt, req.target, observe_s, measurements):
        raise HTTPException(409, {"reason": "ROUTE_MISMATCH", "route_id": rt["route_id"],
                                  "detail": "실행 요청의 목표·체류·측정이 평가한 경로와 다르다"})
    pos = state["position"]
    drift = evaluator.haversine_m(pos["lat"], pos["lon"], rt["start"]["lat"], rt["start"]["lon"])
    if drift > config.ROUTE_START_TOLERANCE_M:
        raise HTTPException(409, {"reason": "ROUTE_STALE", "route_id": rt["route_id"], "drift_m": round(drift),
                                  "detail": "평가 뒤 기체 위치가 바뀌었다 (다시 평가)"})
    return rt


@app.post("/uav/{uav_id}/execute", response_model=ExecuteResponse)
async def execute_mission(uav_id: str, req: ExecuteRequest):
    """Safety ALLOW 이후 호출. 비동기로 시작하고 즉시 반환한다.

    실행 키 = task_id (UAV-01). 같은 키·같은 내용 재요청은 새로 출동하지 않고 기존 실행을 돌려준다
    (duplicate=true). 같은 키·다른 내용은 409 — 기존 실행은 그대로 둔다. 재시작 뒤에도 같다.
    거절(400·409)은 실행하지 않았음이 확실한 경우만이며, 키를 쓰지 않는다.
    """
    _check_id(uav_id)
    if req.execution_attempt_id is not None and req.execution_attempt_id != req.task_id:
        raise HTTPException(422, "execution_attempt_id 를 주려면 task_id 와 같아야 한다 (실행 키는 하나)")
    body = req.model_dump()
    verdict, meta = STORE.check(req.task_id, body)
    if verdict == "CONFLICT":
        raise HTTPException(409, {"reason": "EXECUTION_ID_CONFLICT", "task_id": req.task_id,
                                  "detail": "같은 실행 키로 다른 내용의 실행이 이미 있다 (기존 실행 유지)"})
    if verdict == "DUPLICATE":
        cur = _tasks.get(req.task_id, {})
        return {**meta["response"], "status": cur.get("status", meta["response"]["status"]),
                "duplicate": True, "uav_id": UAV_ID}
    running = _running_task_id()
    if running is not None:
        raise HTTPException(409, f"task {running} already running")
    returning = _returning_task_id()
    if returning is not None and not config.RETASK_WHILE_RETURNING:
        raise HTTPException(409, {"reason": "BUSY", "detail": f"returning after {returning} (UAV-06)"})
    obs_alt = evaluator.flight_alt_amsl(req.target)
    if obs_alt is None:
        raise HTTPException(
            400, "target 에 alt_m_amsl(지면 해발고도)이 필요하다")
    measurements = evaluator.measurements_of(req)
    missing = [m for m in measurements if m not in config.available_sensors()]
    if missing:
        raise HTTPException(400, {"reason": "REQUIRED_CAPABILITY_UNAVAILABLE", "detail": f"{missing}"})
    rt = _resolve_route(req, await provider.get_state(uav_id), await provider.home(), obs_alt,
                        evaluator.observe_seconds(req), measurements)
    if _returning is not None and not _returning.done():
        _returning.cancel()
        await asyncio.gather(_returning, return_exceptions=True)

    resp = {"task_id": req.task_id, "status": "STARTED",
            "tracking_url": f"/uav/{uav_id}/task/{req.task_id}", "uav_id": UAV_ID, "duplicate": False,
            "route_id": rt["route_id"], "route_hash": rt["route_hash"]}
    # 출동 전에 기록부터 저장한다 — 저장 뒤에 멈추면 재시작이 '관측 전 끊김'으로 정리하고, 같은 키 재요청은 재출동하지 않는다
    STORE.register(req.task_id, body, {"task_id": req.task_id, "uav_id": UAV_ID, "status": "STARTED",
                                       "progress": None, "observation": None,
                                       "route_id": rt["route_id"], "route_hash": rt["route_hash"]}, resp)
    _flights[req.task_id] = asyncio.create_task(_run_task(uav_id, req, rt))
    return resp


@app.get("/uav/{uav_id}/task/{task_id}", response_model=TaskStatus)
async def get_task(uav_id: str, task_id: str):
    _check_id(uav_id)
    if task_id not in _tasks:
        raise HTTPException(404, f"task {task_id} not found")
    return _report(_tasks[task_id])


@app.post("/uav/{uav_id}/task/{task_id}/abort", response_model=TaskStatus)
async def abort_task(uav_id: str, task_id: str, req: Optional[AbortRequest] = None):
    """중단 요청 (UAV-09). 같은 요청을 다시 보내면 처음 결과를 그대로 돌려준다.

    abort.result
      ABORTED_BEFORE_OBSERVATION  관측 전 → status=FAILED, reason=ABORTED, 관측 없음, 기지로 복귀
                                  (physical_state RETURNING → LANDED)
      ALREADY_RETURNING           관측을 마쳤거나 이미 끊겨 복귀 중 — 멈출 임무가 없다
      ALREADY_FINISHED            이미 끝난 실행 (착륙·복귀 실패·재시작 정리)
    점유 해제는 지금처럼 phase=DONE(착륙) 확인으로 한다.
    """
    _check_id(uav_id)
    t = _tasks.get(task_id)
    if t is None:
        raise HTTPException(404, f"task {task_id} not found")
    if t.get("abort"):
        return _report(t)
    record = {"requested_at": _now(), "reason": req.reason if req else None}
    phase = (t.get("progress") or {}).get("phase")
    if t["status"] in ("STARTED", "IN_PROGRESS"):
        t["abort"] = {**record, "result": "ABORTED_BEFORE_OBSERVATION"}
        _save()
        flight = _flights.get(task_id)
        if flight is not None and not flight.done():
            done = _abort_done.setdefault(task_id, asyncio.Event())
            flight.cancel()
            try:
                await asyncio.wait_for(done.wait(), timeout=5)
            except TimeoutError:
                pass
            _abort_done.pop(task_id, None)
        if t["status"] in ("STARTED", "IN_PROGRESS"):
            # 비행 코루틴이 시작하기 전에 끊겼다 — 기체는 움직이지 않았다
            t.update({"status": "FAILED", "reason": "ABORTED", "observation": None,
                      "error": "aborted by request before takeoff", "progress": {"phase": "DONE"}})
    elif phase == "RETURNING":
        t["abort"] = {**record, "result": "ALREADY_RETURNING"}
    else:
        t["abort"] = {**record, "result": "ALREADY_FINISHED"}
    _save()
    return _report(t)


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
    return await _state(UAV_ID)
