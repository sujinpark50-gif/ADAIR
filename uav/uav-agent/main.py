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
import sys
from datetime import datetime, timezone
from pathlib import Path

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

# 이 프로세스가 담당하는 UAV 1대. 여러 대를 운용하면 서버를 나누고 각자 다른 값을 준다.
#   UAV_ID=A-uav1 ... --port 8000
#   UAV_ID=A-uav2 ... --port 8001
# 앞글자가 소속 기지다(A=원통119, B=기린119). Mock 은 그 기지에서 출발·복귀한다.
UAV_ID = os.getenv("UAV_ID", "A-uav1")

provider = get_provider(UAV_ID)
UAV_MODE = os.getenv("UAV_MODE", "mock").lower()

# ── 실행 기록 (UAV-01): 실행 키 = task_id. 파일에 저장해 재시작 뒤에도 같은 키로 조회·재요청 가능 ──
# 저장소 루트의 interfaces/exec_store.py 를 쓴다 (UGV 서버와 같은 규칙). 루트는 sys.path 맨 뒤에 붙여
# 이 폴더의 config.py 가 루트 config.py 에 가려지지 않게 한다.
_REPO_ROOT = str(Path(__file__).resolve().parents[2])
if _REPO_ROOT not in sys.path:
    sys.path.append(_REPO_ROOT)
from interfaces.exec_store import ExecStore  # noqa: E402

_STATE_DIR = Path(os.getenv("UAV_STATE_DIR", str(Path(__file__).resolve().parent / ".state")))
STORE = ExecStore(_STATE_DIR / f"tasks_{UAV_ID}.json")
_tasks: dict[str, dict] = STORE.tasks
_save = STORE.save

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


# 기지로 돌아오는 중인 비행. 새 임무가 오면 끊고 그 자리에서 출발한다.
_returning: asyncio.Task | None = None


def _running_task_id() -> str | None:
    """진행 중인 Task id. 드론 provider 는 Task 를 모르므로 여기서 채운다.

    복귀 중(RETURNING)은 바쁘지 않다 — 배터리가 되면 다른 임무를 받을 수 있다
    (시나리오 4·7 재배치). evaluate 가 현재 위치 기준으로 판단한다.
    """
    for tid, t in _tasks.items():
        if t["status"] in ("STARTED", "IN_PROGRESS"):
            return tid
    return None


async def _state(uav_id: str) -> dict:
    state = await provider.get_state(uav_id)
    state["current_task_id"] = _running_task_id()
    return state


@app.get("/health")
async def health():
    return {"status": "ok", "mode": os.getenv("UAV_MODE", "mock"), "uav_id": UAV_ID,
            "exec_store": {"path": str(STORE.path), "restarts": STORE.restarts,
                           "closed_after_restart": RESTART_CLOSED, "tasks": len(_tasks)}}


@app.get("/uav/{uav_id}/state", response_model=UavState)
async def get_state(uav_id: str):
    _check_id(uav_id)
    return await _state(uav_id)


@app.post("/uav/{uav_id}/evaluate", response_model=EvaluateResponse)
async def evaluate_mission(uav_id: str, req: EvaluateRequest):
    """수행 가능성 판단. 실행하지 않는다.

    ACCEPT 는 Safety 검증을 거친 뒤 /execute 로 들어온다.
    """
    _check_id(uav_id)
    state = await _state(uav_id)
    return evaluator.evaluate(state, req, await provider.home())


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


async def _run_task(uav_id: str, req: ExecuteRequest):
    """목표 이동 → 관측 체류 → (결과 보고) → 기지 복귀.

    COMPLETED 는 관측을 마친 시점이다. 이후 복귀하는 동안 progress.phase 는
    RETURNING 이고 이 기체는 BUSY 다. 착륙하면 DONE.
    """
    tid = req.task_id
    t = _tasks[tid]                       # execute 가 등록한 기록을 그대로 고친다 (실행 키 기록 유지)
    t.update({"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}, "observation": None})
    _save()
    try:
        home = await provider.home()
        alt = evaluator.flight_alt_amsl(req.target)
        await _monitored(uav_id, home, provider.goto(req.target.lat, req.target.lon, alt))

        t["progress"] = {"phase": "OBSERVING"}
        _save()
        await _monitored(uav_id, home, provider.hold(config.OBSERVE_DURATION_S))

        pos = (await provider.get_state(uav_id))["position"]
        t.update({
            "status": "COMPLETED",
            "progress": {"phase": "RETURNING"},
            "observation": {
                "observed_at": datetime.now(timezone.utc).strftime(
                    "%Y-%m-%dT%H:%M:%SZ"),
                # 목표 좌표가 아니라 관측 시점의 실제 기체 위치
                "position": {
                    "lat": pos["lat"],
                    "lon": pos["lon"],
                    "alt_m_amsl": pos["alt_m_amsl"],
                },
                "sensor_type": req.observation_type,
                # Mock 고정값. 실연동 시 실제 센서 결과로 교체
                "values": {"max_temp_c": 312.5, "hotspot_detected": True},
            },
        })
        _save()
    except _ReturnMarginLow as e:
        # 임무를 끊고 돌아온다. 총괄은 FAILED + reason 을 보고 다른 기체에 인계한다.
        t.update({"status": "FAILED", "reason": "RETURN_MARGIN_INSUFFICIENT",
                  "error": str(e), "progress": {"phase": "RETURNING"}})
        _save()
    except Exception as e:  # noqa: BLE001
        t.update({"status": "FAILED", "progress": None, "error": str(e)})
        _save()
        return

    global _returning
    _returning = asyncio.current_task()
    try:
        await provider.return_home()
        t["progress"] = {"phase": "DONE"}
    except asyncio.CancelledError:
        t["progress"] = {"phase": "RETASKED"}   # 복귀 중 새 임무를 받음
    except Exception as e:  # noqa: BLE001
        t["progress"] = {"phase": "RETURN_FAILED"}
        t["error"] = f"return failed: {e}"
    finally:
        _save()
        if _returning is asyncio.current_task():
            _returning = None


@app.post("/uav/{uav_id}/execute", response_model=ExecuteResponse)
async def execute_mission(uav_id: str, req: ExecuteRequest):
    """Safety ALLOW 이후 호출. 비동기로 시작하고 즉시 반환한다.

    실행 키 = task_id (UAV-01). 같은 키·같은 내용 재요청은 새로 출동하지 않고 기존 실행을 돌려준다
    (duplicate=true). 같은 키·다른 내용은 409 — 기존 실행은 그대로 둔다. 재시작 뒤에도 같다.
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
    if evaluator.flight_alt_amsl(req.target) is None:
        raise HTTPException(
            400, "target 에 alt_m_amsl(지면 해발고도)이 필요하다")
    if _returning is not None and not _returning.done():
        _returning.cancel()
        await asyncio.gather(_returning, return_exceptions=True)

    resp = {"task_id": req.task_id, "status": "STARTED",
            "tracking_url": f"/uav/{uav_id}/task/{req.task_id}", "uav_id": UAV_ID, "duplicate": False}
    # 출동 전에 기록부터 저장한다 — 저장 뒤에 멈추면 재시작이 '관측 전 끊김'으로 정리하고, 같은 키 재요청은 재출동하지 않는다
    STORE.register(req.task_id, body, {"task_id": req.task_id, "uav_id": UAV_ID, "status": "STARTED",
                                       "progress": None, "observation": None}, resp)
    asyncio.create_task(_run_task(uav_id, req))
    return resp


@app.get("/uav/{uav_id}/task/{task_id}", response_model=TaskStatus)
async def get_task(uav_id: str, task_id: str):
    _check_id(uav_id)
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
    return await _state(UAV_ID)
