# ugv/server.py — UGV Local Agent REST API
#
# 서버 1개가 지상자원 전부(config.RESOURCES)를 맡는다. UAV 는 기체 1대당 서버 1개지만,
# UGV 는 자원들이 도로망 그래프 하나를 공유해야 도로 차단이 모든 판단에 동시에 반영된다.
#
# 실행 (저장소 루트에서):
#   UGV_DRIVER=sim uvicorn ugv.server:app --port 8100          # PX4 없이
#   UGV_DRIVER=px4 uvicorn ugv.server:app --port 8100          # px4_port 가 있는 자원만 PX4, 나머지는 sim
# 문서: http://localhost:8100/docs
#
# 호출 순서: state → evaluate → (총괄·Safety 판단) → execute → task 폴링

import asyncio
import logging
import os
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException

from . import config
from .agent import GroundResourceAgent
from .api_models import (
    CongestionRequest, EvaluateRequest, EvaluateResponse, ExecuteRequest, ExecuteResponse,
    FireCellsRequest, LatLon, TargetNode, TaskStatus, UgvState,
)
from .fleet import GroundFleet
from .geo import distance_m
from . import graph_gpkg
from .graph_gpkg import ROAD_CELLS
from .road_status import RoadStatus
from interfaces.exec_store import ExecStore

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger(__name__)

DRIVER = os.getenv("UGV_DRIVER", "sim").lower()     # "sim" | "px4"
REFRESH_S = 1.0                                     # 텔레메트리 → 자원 상태 반영 주기
MAX_TARGET_CANDIDATES = 20                          # evaluate: 목표 반경 안에서 시도할 도로 노드 수

fleet: GroundFleet | None = None
roads: RoadStatus | None = None
_task_of: dict[str, str | None] = {}                # resource_id → 수행 중 task_id

# 실행 기록 (UGV-02·04): 실행 키 = task_id. 파일에 저장해 재시작 뒤에도 같은 키로 조회·재요청이 같은 결과.
# 재시작으로 끊긴 주행은 FAILED + phase=FAILED 로 닫는다 → 총괄은 FAULTED(점유 유지)로 두고,
# 차가 READY 로 확인될 때 반납한다. 관측(도착) 전 끊김이므로 목적은 다른 자원으로 인계된다.
_STATE_DIR = os.getenv("UGV_STATE_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), ".state"))
STORE = ExecStore(os.path.join(_STATE_DIR, "ugv_tasks.json"))
_tasks: dict[str, dict] = STORE.tasks                # task_id → TaskStatus 내용
_save = STORE.save


def _close_after_restart(t: dict) -> None:
    t.update({"status": "FAILED", "error": f"AGENT_RESTARTED: 서버 재시작으로 주행 감시가 끊김 (driver={DRIVER})",
              "progress": {**(t.get("progress") or {}), "phase": "FAILED"}})


RESTART_CLOSED = STORE.reconcile_after_restart(
    lambda t: t["status"] in ("STARTED", "IN_PROGRESS"), _close_after_restart,
    basis="SIM_DRIVER_RESET" if DRIVER == "sim" else "PHYSICAL_STATE_FROM_TELEMETRY_AFTER_RESTART")
_watchers: dict[str, asyncio.Task] = {}             # task_id → 도착 감시 태스크


async def _refresh_loop() -> None:
    """드라이버 위치·연료를 자원 상태로 옮기고 도착을 확정한다."""
    while True:
        try:
            fleet.refresh_all()
        except Exception:
            log.exception("refresh 실패")
        await asyncio.sleep(REFRESH_S)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global fleet, roads
    fleet = GroundFleet(use_px4=(DRIVER == "px4"), graph_data=graph_gpkg)   # 실제 도로망
    roads = RoadStatus(fleet.graph, ROAD_CELLS)
    await fleet.connect_all()                       # PX4 는 연결될 때까지 대기
    loop = asyncio.create_task(_refresh_loop())
    log.info("UGV 서버 준비: driver=%s, 자원 %s", DRIVER, list(fleet.agents))
    yield
    loop.cancel()


app = FastAPI(
    title="UGV Local Agent",
    description="산불 대응 시뮬레이터 — 지상자원(UGV·소방차) 상태 제공, 도로망 기반 수행 가능성 판단, 주행",
    version="0.1.0",
    lifespan=lifespan,
)


def _agent(resource_id: str) -> GroundResourceAgent:
    """없는 자원이면 404. 조용히 다른 자원 값을 주는 것보다 끊는 쪽이 안전하다."""
    agent = fleet.agents.get(resource_id)
    if agent is None:
        raise HTTPException(404, f"자원 {resource_id} 없음. 담당 자원: {list(fleet.agents)}")
    return agent


def _driver_kind(agent: GroundResourceAgent) -> str:
    return "px4" if type(agent.driver).__name__ == "PX4Driver" else "sim"


@app.get("/health")
async def health():
    return {
        "status": "ok",
        "driver": DRIVER,
        "resources": {rid: _driver_kind(a) for rid, a in fleet.agents.items()},
        "graph": {"nodes": len(fleet.graph._nodes), "roads": len(fleet.graph._roads)},
        "approach": {"target_snap_m": config.TARGET_SNAP_M, "fallback": config.APPROACH_FALLBACK,
                     "approach_max_m": config.APPROACH_MAX_M},
    }


def _state(agent: GroundResourceAgent) -> UgvState:
    agent.refresh()
    r = agent.resource
    return UgvState(
        resource_id=r.resource_id,
        resource_type=r.resource_type,
        base=r.base,
        state=r.state,
        position=LatLon(lat=r.lat, lon=r.lon),
        fuel_pct=r.fuel_pct,
        current_node=r.current_node,
        current_task_id=_task_of.get(r.resource_id),
        driver=_driver_kind(agent),
        driver_status=agent.driver.status() if agent.driver else "NONE",
        updated_at=r.updated_at,
        fault=agent.fault,
    )


@app.get("/ugv", response_model=list[UgvState])
async def list_state(base: str | None = None):
    """거점별 자원 목록. 총괄의 get_ugv_status(base) 에 대응한다."""
    return [_state(a) for a in fleet.agents.values() if base is None or a.resource.base == base]


@app.get("/ugv/{resource_id}/state", response_model=UgvState)
async def get_state(resource_id: str):
    return _state(_agent(resource_id))


def _decide(agent: GroundResourceAgent, target: LatLon | None, target_node: str | None) -> dict:
    """목적지 도로 노드를 고르고 갈 수 있는지 판단한다. evaluate·execute 가 같은 규칙을 쓴다.

    target(화재 좌표)을 주면 화재에서 가장 가까운 도로 노드를 목적지로 삼는다.
      - 그 노드가 TARGET_SNAP_M 보다 멀면 TARGET_UNREACHABLE
      - 그 노드로 못 가면 기본은 REJECT. APPROACH_FALLBACK=1 이면 화재에서
        APPROACH_MAX_M 안의 다음 노드들을 가까운 순으로 시도한다 (ugv/config.py)
    반환: verdict, eta_sec, reason, detail, target_node(TargetNode|None), path
    """
    rid = agent.resource.resource_id

    def reject(reason, detail, tn=None):
        return dict(verdict="REJECT", eta_sec=None, reason=reason, detail=detail, target_node=tn, path=None)

    if target_node is not None:
        try:
            n = fleet.graph.node(target_node)
        except KeyError:
            raise HTTPException(404, f"도로 노드 {target_node} 없음")
        candidates = [(n, 0.0)]
    elif target is not None:
        nearest, d = fleet.graph.nearest_node(target.lat, target.lon)
        if d > config.TARGET_SNAP_M:
            return reject("TARGET_UNREACHABLE",
                          f"가장 가까운 도로 노드가 {d:.0f} m 떨어짐 (허용 {config.TARGET_SNAP_M:.0f} m)")
        candidates = [(nearest, d)]
        if config.APPROACH_FALLBACK:   # 가장 가까운 노드로 못 가면 반경 안 다음 노드로
            candidates += [c for c in fleet.graph.nodes_within(target.lat, target.lon, config.APPROACH_MAX_M)
                           if c[0].node_id != nearest.node_id][:MAX_TARGET_CANDIDATES - 1]
    else:
        raise HTTPException(422, "target 또는 target_node 중 하나가 필요하다")

    first_reject = None
    for node, snap_m in candidates:
        tn = TargetNode(node_id=node.node_id, lat=node.lat, lon=node.lon, snap_m=round(snap_m, 1))
        result = agent.evaluate(node.node_id)
        if result["response"] == "ACCEPT":
            return dict(verdict="ACCEPT", eta_sec=round(result["eta_s"]), reason=None,
                        detail=None if first_reject is None else f"가장 가까운 노드는 도달 불가, {snap_m:.0f} m 지점으로 접근",
                        target_node=tn, path=result["path"])
        if result["reason"] == "BUSY":
            return reject("BUSY", f"수행 중 task {_task_of.get(rid)}")
        if result.get("fault"):                 # UNAVAILABLE — 주행 중 이상, /stop 으로 해제
            return reject(result["reason"], f"자원 이상 [{result['fault']}] — 확인 후 /stop 으로 복귀")
        first_reject = first_reject or (result, tn)

    result, tn = first_reject
    blocked = result.get("blocked_road_id")
    return reject(result["reason"],
                  (f"후보 노드 {len(candidates)}개 모두 도달 불가" if len(candidates) > 1
                   else f"화재에서 가장 가까운 노드 {tn.node_id}({tn.snap_m:.0f} m) 도달 불가")
                  + (f", 차단 도로 {blocked}" if blocked else ""), tn)


@app.post("/ugv/{resource_id}/evaluate", response_model=EvaluateResponse)
async def evaluate(resource_id: str, req: EvaluateRequest):
    """수행 가능성 판단. 차를 움직이지 않는다. 목적지 선정 규칙은 _decide 참고."""
    agent = _agent(resource_id)
    return EvaluateResponse(task_id=req.task_id, decision_id=req.decision_id, resource_id=resource_id,
                            **_decide(agent, req.target, req.target_node))


@app.get("/ugv/{resource_id}/observation")
async def observation(resource_id: str):
    """현재 위치의 도로 상태 관측. 공통 Observation 필드 이름을 따른다 (simulation_time_s 는 호출측이 붙인다)."""
    agent = _agent(resource_id)
    agent.refresh()
    r = agent.resource
    return {
        "resource_id": r.resource_id,
        "location_lat": r.lat,
        "location_lon": r.lon,
        "observation_type": "ROAD_STATUS",
        "value": {"state": r.state, "current_node": r.current_node,
                  "current_task_id": _task_of.get(resource_id),
                  "blocked_roads": len(roads.blocked()), "burning_cells": roads.burning_cells},
    }


@app.get("/graph/nodes/{node_id}")
async def get_node(node_id: str):
    """도로 노드 좌표 조회 (총괄 GroundAdapter.position_of_node 대응)."""
    try:
        n = fleet.graph.node(node_id)
    except KeyError:
        raise HTTPException(404, f"도로 노드 {node_id} 없음")
    return {"node_id": n.node_id, "lat": n.lat, "lon": n.lon, "name": n.name}


# --- 실행 -------------------------------------------------------------------

def _check_run(agent: GroundResourceAgent, w: dict, now: float) -> str | None:
    """주행 감시 1회. 이상이 확정되면 'CODE: 설명', 아니면 None.
    w: task 별 감시 상태 (started, mark_t, mark_pos, mark_wp, fault_since)."""
    r, (cur, _) = agent.resource, agent.driver.progress()
    pos = (r.lat, r.lon)

    # 1) 멈춤 — STALL_MOVE_M 이상 움직이거나 웨이포인트를 넘기면 기준점을 새로 잡는다
    if cur != w["mark_wp"] or distance_m(pos, w["mark_pos"]) >= config.STALL_MOVE_M:
        w.update(mark_t=now, mark_pos=pos, mark_wp=cur)
    elif now - w["mark_t"] > config.STALL_TIMEOUT_S:
        return (f"STALLED: {config.STALL_TIMEOUT_S:.0f}초간 이동 "
                f"{distance_m(pos, w['mark_pos']):.0f} m, 웨이포인트 {cur} 에서 멈춤")

    # 2) 경로 이탈
    off = agent.off_route_m()
    if off > config.OFF_ROUTE_M:
        return f"OFF_ROUTE: 경로에서 {off:.0f} m 벗어남 (허용 {config.OFF_ROUTE_M:.0f} m)"

    # 3) 차량 이상 (드라이버가 판정) — 출발 직후 유예, 일정 시간 계속될 때만 확정
    fault = agent.driver.fault() if now - w["started"] > config.FAULT_GRACE_S else None
    if fault is None:
        w["fault_since"] = None
    elif w["fault_since"] is None:
        w["fault_since"] = now
    elif now - w["fault_since"] >= config.FAULT_CONFIRM_S:
        return fault
    return None


async def _watch_task(task_id: str, agent: GroundResourceAgent, target_node: str) -> None:
    """도착할 때까지 진행률을 기록한다. 도착하면 COMPLETED, 이상이 확정되면 FAILED.

    이상(_check_run)이 나면 차를 세우고 자원을 UNAVAILABLE 로 둔다.
    떨어지거나 고장 난 차가 곧바로 READY 가 되면 총괄이 또 배정하므로, 운영자가 /stop 으로 풀어야 한다.
    """
    rid = agent.resource.resource_id
    t = _tasks[task_id]
    now = time.monotonic()
    w = dict(started=now, mark_t=now, mark_pos=(agent.resource.lat, agent.resource.lon),
             mark_wp=0, fault_since=None)
    try:
        while True:
            agent.refresh()
            agent.check_arrival()
            cur, total = agent.driver.progress()
            new = {"phase": "ENROUTE", "waypoint": cur, "total": total}
            if t["status"] != "IN_PROGRESS" or t.get("progress") != new:
                t["status"], t["progress"] = "IN_PROGRESS", new
                _save()
            if agent.resource.state == "READY" and agent.resource.current_node == target_node:
                break
            fault = _check_run(agent, w, time.monotonic())
            if fault:
                await agent.fail(fault)
                t["status"], t["error"] = "FAILED", fault
                t["progress"] = {"phase": "FAILED", "waypoint": cur, "total": total}
                log.warning("task %s 실패 — %s", task_id, fault)
                return
            await asyncio.sleep(0.5)
        r = agent.resource
        t["status"] = "COMPLETED"
        t["progress"] = {"phase": "ARRIVED", "waypoint": total, "total": total}
        # 지상자원 관측은 도로 상태다. 도착했다고 화재를 관측한 것은 아니다.
        t["observation"] = {
            "observation_type": "ROAD_STATUS",
            "arrived_node": target_node,
            "position": {"lat": r.lat, "lon": r.lon},
            "fuel_pct": r.fuel_pct,
        }
    except asyncio.CancelledError:
        pass                                  # /stop 이 상태를 CANCELLED 로 기록한다
    except Exception as e:   # noqa: BLE001
        t["status"], t["error"] = "FAILED", str(e)
        log.exception("task %s 실패", task_id)
    finally:
        _save()
        _task_of[rid] = None
        _watchers.pop(task_id, None)


@app.post("/ugv/{resource_id}/execute", response_model=ExecuteResponse)
async def execute(resource_id: str, req: ExecuteRequest):
    """Safety ALLOW 이후 호출. 주행을 시작하고 즉시 반환한다. 진행은 tracking_url 로 조회.

    UAV 와 같이 target(화재 좌표)을 받는다. 목적지 노드는 evaluate 와 같은 규칙(_decide)으로
    다시 고른다 — evaluate 이후 도로가 막혔으면 여기서 409 로 거절된다.
    """
    agent = _agent(resource_id)
    # 실행 키 확인이 먼저다: 같은 키 재요청은 수행 중이어도 기존 실행을 돌려준다 (UGV-04)
    body = {"resource_id": resource_id, **req.model_dump()}
    verdict, meta = STORE.check(req.task_id, body)
    if verdict == "CONFLICT":
        raise HTTPException(409, {"reason": "EXECUTION_ID_CONFLICT", "task_id": req.task_id,
                                  "detail": "같은 실행 키로 다른 내용의 실행이 이미 있다 (기존 실행 유지)"})
    if verdict == "DUPLICATE":
        return ExecuteResponse(**{**meta["response"], "duplicate": True,
                                  "current_status": _tasks.get(req.task_id, {}).get("status")})
    if _task_of.get(resource_id):
        raise HTTPException(409, f"{resource_id} 는 task {_task_of[resource_id]} 수행 중")

    d = _decide(agent, req.target, req.target_node)
    if d["verdict"] != "ACCEPT":
        raise HTTPException(409, f"실행 불가: {d['reason']} — {d['detail']}")   # 실행 안 함 → 키를 쓰지 않는다
    node_id = d["target_node"].node_id
    url = f"/ugv/{resource_id}/task/{req.task_id}"
    if len(d["path"]) == 1:     # 이미 목적지 노드에 서 있다 — 움직이지 않고 바로 완료
        r = agent.resource
        resp = ExecuteResponse(task_id=req.task_id, resource_id=resource_id, status="STARTED", tracking_url=url,
                               target_node=d["target_node"], eta_sec=0)
        STORE.register(req.task_id, body, {
            "task_id": req.task_id, "resource_id": resource_id, "status": "COMPLETED", "target_node": node_id,
            "progress": {"phase": "ARRIVED", "waypoint": 0, "total": 0},
            "observation": {"observation_type": "ROAD_STATUS", "arrived_node": node_id,
                            "position": {"lat": r.lat, "lon": r.lon}, "fuel_pct": r.fuel_pct}},
            resp.model_dump())
        return resp
    if not await agent.execute(node_id):
        raise HTTPException(500, "드라이버가 주행을 시작하지 못했다")

    resp = ExecuteResponse(task_id=req.task_id, resource_id=resource_id, status="STARTED", tracking_url=url,
                           target_node=d["target_node"], eta_sec=d["eta_sec"])
    STORE.register(req.task_id, body, {"task_id": req.task_id, "resource_id": resource_id, "status": "STARTED",
                                       "target_node": node_id, "progress": None, "observation": None},
                   resp.model_dump())
    _task_of[resource_id] = req.task_id
    _watchers[req.task_id] = asyncio.create_task(_watch_task(req.task_id, agent, node_id))
    return resp


@app.post("/ugv/{resource_id}/stop")
async def stop(resource_id: str):
    """주행 중지. 진행 중 task 는 CANCELLED. 멈춘 곳에서 가장 가까운 도로 노드에 선 것으로 본다."""
    agent = _agent(resource_id)
    task_id = _task_of.get(resource_id)
    watcher = _watchers.get(task_id) if task_id else None
    if watcher:
        watcher.cancel()
    await agent.stop()
    if task_id and task_id in _tasks:
        _tasks[task_id]["status"] = "CANCELLED"
        _tasks[task_id]["error"] = "stopped by request"
        _save()
    _task_of[resource_id] = None
    r = agent.resource
    return {"resource_id": resource_id, "cancelled_task": task_id, "state": r.state,
            "current_node": r.current_node, "position": {"lat": r.lat, "lon": r.lon}}


@app.get("/ugv/{resource_id}/task/{task_id}", response_model=TaskStatus)
async def get_task(resource_id: str, task_id: str):
    t = _tasks.get(task_id)
    if t is None or t["resource_id"] != resource_id:
        raise HTTPException(404, f"{resource_id} 의 task {task_id} 없음")
    return t


# --- 외부 사정 반영 (화재·혼잡) ----------------------------------------------

@app.post("/env/fire_cells")
async def update_fire(req: FireCellsRequest):
    """현재 BURNING 셀 전체를 받아 도로 차단을 다시 계산한다.
    도로가 지나는 칸이나 그 8방향 이웃이 타면 차단. 이미 주행 중인 경로는 바꾸지 않는다."""
    return roads.apply_fire([(c.x, c.y) for c in req.cells])


@app.post("/env/congestion")
async def update_congestion(req: CongestionRequest):
    """시나리오용 혼잡. 환경에 혼잡 정보가 없으므로 직접 준다."""
    try:
        return roads.apply_congestion(req.preset, req.seed, req.ratio, req.min_factor,
                                      req.max_factor, req.roads)
    except (ValueError, KeyError) as e:
        raise HTTPException(422, str(e))


@app.get("/env/roads")
async def road_status():
    return {"burning_cells": roads.burning_cells, "blocked": roads.blocked(),
            "congested": roads.congested()}
