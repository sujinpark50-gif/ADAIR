# -*- coding: utf-8 -*-
"""ugv/server.py — UGV Local Agent HTTP 서버 (포트 8100). 총괄(orchestrator/resources.py UgvClient)과
공통 커넥터(connectors/ugv_connector.py real 모드)가 부르는 경로·응답 형식을 따른다.

    UGV_DRIVER=sim python -m uvicorn ugv.server:app --port 8100
    UGV_DRIVER=px4 python -m uvicorn ugv.server:app --port 8100     # 먼저 WSL 에서 ugv/tools/px4-start.sh
    환경 화재 정보: UGV_ENV_URL=http://127.0.0.1:8300 (환경 계약 서버). 없으면 화재 없음으로 판단한다.

| 메서드 | 경로 | 용도 |
|---|---|---|
| GET  | /health | 드라이버·도로망·화재 정보 출처·차량 목록 |
| GET  | /ugv?base=A | 자원 상태 목록 |
| GET  | /ugv/{id}/state | 자원 1대 상태 |
| POST | /ugv/{id}/evaluate | 수행 가능성·접근 지점 (움직이지 않음) |
| POST | /ugv/{id}/execute | 출발 (Safety ALLOW 뒤). 진행 중 임무가 있으면 취소하고 새로 계획 |
| GET  | /ugv/{id}/task/{task_id} | 실행 진행·결과 |
| POST | /ugv/{id}/task/{task_id}/abort | 실행 중단 (UGV-03) |
| POST | /ugv/{id}/stop | 주행 중지 + 고장·위험 상태 운영자 해제 |
| GET  | /ugv/{id}/route | 지금 계획 경로 선형·구간 속도 |
| GET  | /ugv/{id}/events | 최근 사건 (도착·재계획·이탈·고장) |
| GET  | /ugv/{id}/observation | 현재 위치 도로 상태 (화재 관측 아님, 커넥터 호환) |
| POST | /ugv/{id}/heartbeat | 총괄 연락 (통신 단절 판단) |

총괄 통신 단절 (vehicle._comm_watch, 벽시계): /ugv 아래 요청은 총괄 연락으로 센다 (차량 경로면 그 차량, /ugv 목록이면
전 차량). 감시·시험 도구는 헤더 `X-ADAIR-Monitor: 1` 을 붙여 연락으로 세지 않게 한다.
차량은 서로 독립이다: 차량별 드라이버(PX4 인스턴스·포트)·임무 상태·사건 기록. 임무 저장 파일은 하나지만 기록마다
resource_id 로 나뉘고 한 차량의 재시작 정리는 그 차량 기록만 건드린다.
"""

import asyncio
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import Body, FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse

from interfaces.exec_store import ExecStore

from . import config
from .clock import SimClock
from .fire import build_source
from .roads import RoadNetwork
from .vehicle import HttpError, PlacementError, Vehicle


class Fleet:
    def __init__(self, vehicles_cfg=None, *, net=None, fire_source=None, clock=None, store=None):
        self.net = net or RoadNetwork()
        self.fire_source = fire_source or build_source()
        self.clock = clock or SimClock()
        self.store = store or ExecStore(config.STATE_DIR / "ugv_tasks.json")
        self.vehicles = {}
        self.unplaced = {}                       # 배치 불가 차량: 사유 (소속 거점 출입 지점·대기 위치 없음)
        for c in (vehicles_cfg or config.load_vehicles()):
            try:
                self.vehicles[c["resource_id"]] = Vehicle(c, self.net, self.fire_source, self.clock, self.store)
            except PlacementError as e:
                self.unplaced[c["resource_id"]] = str(e)

    async def start(self):
        for v in self.vehicles.values():
            await v.start()

    async def close(self):
        for v in self.vehicles.values():
            await v.close()

    def get(self, rid) -> Vehicle:
        v = self.vehicles.get(rid)
        if v is None:
            raise HTTPException(404, f"자원 {rid} 없음")
        return v


def create_app(fleet: Optional[Fleet] = None, *, autostart: bool = True) -> FastAPI:
    holder = {"fleet": fleet}

    @asynccontextmanager
    async def lifespan(app):
        if holder["fleet"] is None:
            holder["fleet"] = Fleet()
        if autostart:
            await holder["fleet"].start()
        yield
        await holder["fleet"].close()

    app = FastAPI(title="ADAIR UGV Local Agent", version="0.2.0", lifespan=lifespan)

    def F() -> Fleet:
        return holder["fleet"]

    @app.middleware("http")
    async def contact(request: Request, call_next):
        parts = request.url.path.strip("/").split("/")
        f = holder["fleet"]
        if f is not None and parts and parts[0] == "ugv" and request.headers.get("x-adair-monitor") != "1":
            if len(parts) >= 2 and parts[1] in f.vehicles:
                f.vehicles[parts[1]].note_contact()
            elif len(parts) == 1:
                for v in f.vehicles.values():
                    if request.query_params.get("base") in (None, v.station["base_id"]):
                        v.note_contact()
        return await call_next(request)

    @app.post("/ugv/{rid}/heartbeat")
    def heartbeat(rid: str):
        v = F().get(rid)
        return {"resource_id": rid, "comm": v.state_view()["comm"]}

    async def call(coro):
        try:
            return await coro
        except HttpError as e:
            return JSONResponse({"detail": e.detail}, status_code=e.code)

    @app.get("/health")
    def health():
        f = F()
        src = f.fire_source
        return {"status": "ok", "drivers": {r: v.driver.kind for r, v in f.vehicles.items()},
                "vehicles": {r: {"resource_type": v.resource_type, "profile": v.profile.name, "base": v.station["base_id"],
                                 "access_point": v.station["access_point"]} for r, v in f.vehicles.items()},
                "unplaced": f.unplaced,
                "road_network": {"source": f.net.source, "roads": len(f.net.roads), "nodes": len(f.net.nodes)},
                "fire_source": src.status() if hasattr(src, "status") else src.area().summary(),
                "clock": {"sim_time_s": round(f.clock.now(), 2), "time_scale": f.clock.scale},
                "rules": {"max_speed_mps": round(config.MAX_SPEED_MPS, 2), "fire_standoff_m": config.FIRE_STANDOFF_M,
                          "min_road_speed_kmh": config.MIN_ROAD_SPEED_KMH, "lat_accel_mps2": config.LAT_ACCEL_MPS2}}

    @app.get("/ugv")
    def list_ugv(base: Optional[str] = None):
        return [v.state_view() for v in F().vehicles.values() if base is None or v.station["base_id"] == base]

    @app.get("/ugv/{rid}/state")
    def state(rid: str):
        return F().get(rid).state_view()

    @app.post("/ugv/{rid}/evaluate")
    async def evaluate(rid: str, body: dict = Body(...)):
        return await call(F().get(rid).evaluate(body))

    @app.post("/ugv/{rid}/execute")
    async def execute(rid: str, body: dict = Body(...)):
        return await call(F().get(rid).execute(body))

    @app.get("/ugv/{rid}/task/{task_id}")
    def task(rid: str, task_id: str):
        t = F().store.tasks.get(task_id)
        if t is None or t.get("resource_id") != rid:
            raise HTTPException(404, f"task {task_id} 없음")
        return t

    @app.post("/ugv/{rid}/task/{task_id}/abort")
    async def abort(rid: str, task_id: str, body: dict = Body(default={})):
        return await call(F().get(rid).abort(task_id, (body or {}).get("reason") or "ABORT_REQUESTED"))

    @app.post("/ugv/{rid}/stop")
    async def stop(rid: str, body: dict = Body(default={})):
        return await call(F().get(rid).stop((body or {}).get("reason") or "STOP_REQUESTED"))

    @app.get("/ugv/{rid}/route")
    def route(rid: str):
        v = F().get(rid)
        if v.speed is None:
            return {"resource_id": rid, "route": None}
        from .geo import FRAME
        sp = v.speed
        return {"resource_id": rid, "mission_state": v.state, "task_id": v.task_key, "eta_sec": round(sp.eta_s),
                "length_m": round(sp.cum[-1], 1), "approach": v.plan.report() if v.plan else None,
                "items": [{"lat": round(FRAME.to_ll(x, y)[0], 7), "lon": round(FRAME.to_ll(x, y)[1], 7),
                           "speed_mps": s} for x, y, s in sp.items]}

    @app.get("/ugv/{rid}/events")
    def events(rid: str, limit: int = 50):
        return list(F().get(rid).events)[-limit:]

    @app.get("/ugv/{rid}/observation")
    def observation(rid: str):
        s = F().get(rid).state_view()
        return {"resource_id": rid, "location_lat": s["position"]["lat"], "location_lon": s["position"]["lon"],
                "observation_type": "ROAD_STATUS",
                "value": {"state": s["state"], "mission_state": s["mission_state"], "current_node": s["current_node"],
                          "current_task_id": s["current_task_id"]}}

    return app


app = create_app()
