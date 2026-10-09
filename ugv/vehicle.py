# -*- coding: utf-8 -*-
"""ugv/vehicle.py — 차량 1대의 임무 상태 전이, 취소 규칙, 주행 감시, 화재 안전거리 처리.

임무 상태 (mission_state)
  IDLE        정차 대기 (도착 후 정차 관측 포함 — 다음 명령을 기다린다)
  DRIVING     목표 접근 지점으로 주행
  EVADING     화재 안전거리 위반 → 안전한 도로 지점으로 이탈 주행
  STOPPING    정지 명령 → 감속 중
  FAULT       주행 이상 확정 (경로 이탈·정지 고장·텔레메트리 끊김·차량 이상). 운영자 /stop 으로만 해제
  DANGER      자동 이탈 불가 상태 — 안전 확보 완료가 아니다. 화재 안전거리 위반인데 진행 방향을 지키는 이탈 경로가 없다.
              도로를 따라 제동해 멈추고 운영자 개입을 요청한다. 자동 재출발 없음. 원격 운전 기능은 아직 없다
FAULT·DANGER 해제: 운영자 /stop. 차량이 멈췄고(0.2 m/s 미만) 텔레메트리가 살아 있을 때만 해제한다
총괄에 보이는 자원 상태 (계약 값): READY / RUNNING / UNAVAILABLE

진행 방향 (ugv/drive_graph.py, 2026-10-09 사용자 결정)
  - 계획 출발점 = 지금 위치를 진행 방향으로 명령 교체 지연 동안 갈 거리만큼 앞으로 옮긴 곳. 주행 중이면 계획 경로의
    방향, 아니면 텔레메트리 진행 방위를 쓴다. 정차 중에도 방향 제약을 유지한다
  - 새 목표에 진행 방향을 지키는 경로가 없으면: 실행을 거절(409)하고, 주행 중이던 임무가 있으면 취소한 뒤 도로를 따라
    감속 정지한다 (도로 밖으로 돌아서라도 수행하지 않는다)
  - 정지는 도로를 따른다: 남은 계획 경로(없으면 도로 앞쪽)를 따라 제동거리만큼 가는 감속 미션

취소 규칙
  - 차량당 진행 중 임무는 하나. 새 execute 는 잠금 안에서: 계획(부작용 없음) → 기존 임무 CANCELLED(SUPERSEDED) →
    세대 번호 증가 → 드라이버 미션 교체(follow 는 기존 미션을 통째로 바꾼다). 주행 명령이 둘 남지 않는다.
  - 같은 실행 키·같은 내용 = 기존 실행 반환, 같은 키·다른 내용 = 409 (interfaces/exec_store.py, UGV-04).
  - 감시 루프는 세대 번호가 바뀐 이전 임무의 진행·도착을 무시한다.
"""

import asyncio
import math
import time
from collections import deque
from datetime import datetime, timezone
from typing import Optional

from interfaces.exec_store import ExecStore

from . import config
from .approach import ApproachPlan, node_id_of, parse_node_id, plan_approach, plan_evasion
from .drive_graph import DirPos, access_point
from .drivers import DriverError, build_driver
from .geo import FRAME, bearing_deg
from .profiles import get as get_profile
from .roads import RoadNetwork, RoadPos, Route, plan_speeds

ACTIVE = ("STARTED", "IN_PROGRESS")
DANGER_MEANING = "자동 이탈 불가 상태 (안전 확보 완료 아님). 도로를 따라 제동·정지, 자동 재출발 없음, 운영자 개입 필요"


class PlacementError(Exception):
    """소속 거점에 차량을 둘 도로 출입 지점·대기 위치가 없다."""


def config_station_m():
    from .drive_graph import STATION_SEARCH_M
    return STATION_SEARCH_M


class HttpError(Exception):
    def __init__(self, code: int, detail):
        super().__init__(detail)
        self.code, self.detail = code, detail


def _now_iso():
    return datetime.now(timezone.utc).isoformat()


def _dist_to_polyline(x, y, pts, lo=0, hi=None):
    best, bi = math.inf, lo
    hi = len(pts) - 1 if hi is None else hi
    for i in range(max(lo, 0), max(min(hi, len(pts) - 1), 0)):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        if d < best:
            best, bi = d, i
    return best, bi


class Vehicle:
    def __init__(self, cfg: dict, net: RoadNetwork, fire_source, clock, store: ExecStore):
        self.cfg, self.net, self.fire_source, self.clock, self.store = cfg, net, fire_source, clock, store
        self.resource_id = cfg["resource_id"]
        self.resource_type = cfg.get("resource_type", "UGV")
        # 차량 특성 (차체·회전반경·속도·가감속). 경로 탐색·회전 가능성·차체 검사·속도 계획을 이 값으로 한다
        self.profile = get_profile(cfg.get("profile") or ("adair_firetruck" if self.resource_type == "FIRE_TRUCK"
                                                          else "adair_ugv"))
        self.graph = net.graph_for(self.profile)
        self.cmd_latency_s = float(cfg.get("cmd_latency_s", config.CMD_LATENCY_S))
        st = config.load_station(cfg["station_base_id"])
        sx, sy = FRAME.to_xy(st["lat"], st["lon"])
        nearest = net.snap(sx, sy)
        # 소방서 좌표와 차량 출입 지점을 구분한다. 좌표에서 가장 가까운 도로가 막다른 길일 수 있다 (drive_graph.access_point)
        acc = access_point(net, st["base_id"], graph=self.graph, slot=int(cfg.get("station_slot", 0)),
                           slot_lengths=cfg.get("slot_lengths"))
        if acc is None:
            # 임의의 먼 도로에 두지 않는다 — 배치 불가로 보고 (Fleet 가 이 차량을 빼고 사유를 남긴다)
            raise PlacementError(f"{self.resource_id}: 소방서 {st['base_id']} 좌표 {config_station_m():g} m 안에 출입 지점 없음 "
                                 f"(양방향·서로 오갈 수 있는 도로·차량 {self.profile.name} 급커브 아님 조건) 또는 대기 위치 "
                                 f"{cfg.get('station_slot', 0)} 를 겹치지 않게 둘 도로 길이 없음")
        self.access: RoadPos = acc[0]
        self.access_heading = acc[1] if acc[1] is not None else 0.0
        self.station = {"base_id": st["base_id"], "name": st.get("name"), "lat": st["lat"], "lon": st["lon"],
                        "coordinate_status": st.get("coordinate_status"), "source_file": st.get("source_file"),
                        "road_point": dict(zip(("lat", "lon"), nearest.ll())), "road_offset_m": round(nearest.dist, 1),
                        "access_point": {**dict(zip(("lat", "lon"), self.access.ll())), "road_id": self.access.road_id,
                                         "s_m": round(self.access.s, 1), "heading_deg": round(self.access_heading, 1),
                                         "from_station_m": round(math.hypot(sx - self.access.x, sy - self.access.y), 1),
                                         "source": acc[2]}}
        init = cfg.get("initial", "STATION")
        start = self.access if init == "STATION" else net.snap_ll(init["lat"], init["lon"])
        heading = self.access_heading if init == "STATION" else float(init.get("heading_deg", 0.0))
        self.driver = build_driver(cfg, (start.x, start.y), clock, heading, self.profile)
        self.lock = asyncio.Lock()
        self.state = "IDLE"
        self.fault: Optional[str] = None
        self.gen = 0
        self.task_key: Optional[str] = None
        self.plan: Optional[ApproachPlan] = None
        self.speed = None
        self.seq: Optional[int] = None
        self.target_ll = None
        self.events = deque(maxlen=200)
        self._off_since = None
        self._move_ref = None
        self._last_safety = -math.inf
        self._near_i = 0
        self._drive_started_sim = None
        self._task: Optional[asyncio.Task] = None
        self.last_clearance_m = None
        self.last_stop: dict = {}
        self.danger: Optional[dict] = None

    # ------------------------------------------------------------------ 수명
    async def start(self):
        await self.driver.start()
        self._reconcile_restart()
        self._task = asyncio.create_task(self._run())

    async def close(self):
        if self._task:
            self._task.cancel()
        await self.driver.close()

    async def _run(self):
        while True:
            await asyncio.sleep(max(0.05, self.clock.to_wall(config.SUPERVISE_PERIOD_S)))
            try:
                await self.tick()
            except Exception as e:  # noqa: BLE001 — 감시 루프는 멈추지 않는다
                self._event("SUPERVISOR_ERROR", detail=f"{type(e).__name__}: {e}")

    def _reconcile_restart(self):
        def close(t):
            # 실행 키 계약 (UGV-04): 재시작 전 진행 중이던 임무는 FAILED(AGENT_RESTARTED). 같은 키 재전송은 이 기록을 돌려준다
            t["status"] = "FAILED"
            t["error"] = "AGENT_RESTARTED: 서버 재시작으로 주행 감시가 끊김 (차량 실제 상태는 텔레메트리로 다시 읽음)"
            t["physical_state"] = "UNKNOWN_AFTER_RESTART"
            t["progress"] = {**(t.get("progress") or {}), "phase": "FAILED"}
        basis = "SIM_DRIVER_RESET" if self.driver.kind == "sim" else "PX4_TELEMETRY_REREAD"
        self.store.reconcile_after_restart(
            lambda t: t.get("resource_id") == self.resource_id and t.get("status") in ACTIVE, close, basis=basis)

    # ------------------------------------------------------------------ 보조
    def _event(self, kind, **kw):
        e = {"sim_time_s": round(self.clock.now(), 2), "timestamp": _now_iso(), "event": kind,
             "resource_id": self.resource_id, "task_id": self.task_key, **kw}
        self.events.append(e)
        t = self.store.tasks.get(self.task_key) if self.task_key else None
        if t is not None:
            t.setdefault("events", []).append(e)
            t["events"] = t["events"][-50:]
        return e

    def here(self) -> Optional[RoadPos]:
        t = self.driver.telemetry()
        return None if t is None else self.net.snap(t.x, t.y)

    def _plan_heading(self, tel) -> Optional[float]:
        """주행 중이고 계획 경로 위에 있으면 계획 경로의 진행 방위 (교차로에서 차체 방향보다 믿을 만하다)."""
        if self.speed is None or self.state not in ("DRIVING", "EVADING", "STOPPING"):
            return None
        pts = self.speed.pts
        d, i = _dist_to_polyline(tel.x, tel.y, pts)
        if d > config.OFF_ROUTE_M / 3 or i + 1 >= len(pts):
            return None
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        return bearing_deg(bx - ax, by - ay) if math.hypot(bx - ax, by - ay) > 0 else None

    def here_state(self):
        """계획 출발 상태: (DirPos — 진행 방향, 명령 교체 지연 동안 갈 거리만큼 앞, 속도 m/s, 오류 사유)."""
        tel = self.driver.telemetry()
        if tel is None:
            return None, 0.0, "COMMUNICATION_FAILURE: 차량 위치 없음"
        pos = self.net.snap(tel.x, tel.y)
        hd = self._plan_heading(tel)
        dp = self.graph.dirpos(pos, hd if hd is not None else tel.heading_deg)
        if dp is None:
            return None, tel.speed_mps, f"WRONG_WAY_ON_ONEWAY: 일방통행 도로 {pos.road_id} 를 거꾸로 향함"
        v = max(tel.speed_mps, 0.0)
        # 지연 동안 갈 거리: 주행 중이면 지금 계획 경로를 따라, 아니면 진행 방향 도로를 따라 교차로 너머까지 예측한다
        prefer = None
        if hd is not None:
            _, i = _dist_to_polyline(tel.x, tel.y, self.speed.pts)
            prefer = [(tel.x, tel.y)] + self.speed.pts[i + 1:]
        return self.graph.project(dp, v * self.cmd_latency_s, prefer), v, None

    def current_task(self) -> Optional[dict]:
        t = self.store.tasks.get(self.task_key) if self.task_key else None
        return t if t and t.get("status") in ACTIVE else None

    def resource_state(self) -> str:
        if self.state in ("FAULT", "DANGER"):
            return "UNAVAILABLE"
        if self.current_task() or self.state in ("DRIVING", "EVADING", "STOPPING"):
            return "RUNNING"
        return "READY"

    def _approach(self, body: dict, start: DirPos, v0: float) -> ApproachPlan:
        fire = self.fire_source.area()
        tgt = body.get("target") or {}
        fixed = None
        if body.get("target_node"):
            nid = body["target_node"]["node_id"] if isinstance(body["target_node"], dict) else body["target_node"]
            fixed = parse_node_id(self.net, nid)
            if fixed is None:
                raise HttpError(404, f"target_node 없음: {nid}")
        if tgt.get("lat") is None or tgt.get("lon") is None:
            if fixed is None:
                raise HttpError(422, "target(lat, lon) 또는 target_node 필요")
            tgt = dict(zip(("lat", "lon"), fixed.ll()))
        return plan_approach(self.net, fire, start, float(tgt["lat"]), float(tgt["lon"]), v0=v0, access=self.access,
                             fixed_dest=fixed, graph=self.graph)

    def _target_node(self, plan: ApproachPlan) -> Optional[dict]:
        if plan.dest is None:
            return None
        lat, lon = plan.dest.ll()
        return {"node_id": node_id_of(plan.dest), "lat": round(lat, 7), "lon": round(lon, 7),
                "snap_m": round(plan.dist_to_target_m or 0.0, 1)}

    # ------------------------------------------------------------------ 판단 (움직이지 않음)
    async def evaluate(self, body: dict) -> dict:
        base = {"task_id": body.get("task_id"), "decision_id": body.get("decision_id"), "resource_id": self.resource_id}
        if self.state in ("FAULT", "DANGER"):
            return {**base, "verdict": "REJECT", "eta_sec": None, "reason": "FAILSAFE_ACTIVE",
                    "detail": f"{self.state}: {self.fault}", "target_node": None, "path": None}
        start, v0, err = self.here_state()
        if start is None:
            return {**base, "verdict": "REJECT", "eta_sec": None, "reason": err.split(":")[0],
                    "detail": err, "target_node": None, "path": None}
        plan = self._approach(body, start, v0)
        if plan.status != "OK":
            return {**base, "verdict": "REJECT", "eta_sec": None, "reason": "TARGET_UNREACHABLE",
                    "detail": f"{plan.status}: {plan.reason}", "target_node": None, "path": None,
                    "approach": plan.report()}
        sp = plan_speeds(plan.route, v0, profile=self.profile)
        out = {**base, "verdict": "ACCEPT", "eta_sec": int(round(sp.eta_s)), "reason": None, "detail": None,
               "target_node": self._target_node(plan), "path": plan.route.road_ids, "approach": plan.report()}
        if self.current_task():
            out["detail"] = f"실행하면 진행 중 임무 {self.task_key} 를 취소하고 새로 계획한다"
            out["supersedes_task_id"] = self.task_key
        return out

    # ------------------------------------------------------------------ 실행
    async def execute(self, body: dict) -> dict:
        key = body.get("task_id")
        if not key:
            raise HttpError(422, "task_id 필요")
        if not body.get("target") and not body.get("target_node"):
            raise HttpError(422, "target(lat, lon) 또는 target_node 필요")
        async with self.lock:
            kind, meta = self.store.check(key, body)
            if kind == "DUPLICATE":
                t = self.store.tasks.get(key, {})
                return {**meta["response"], "duplicate": True, "current_status": t.get("status")}
            if kind == "CONFLICT":
                raise HttpError(409, {"reason": "EXECUTION_ID_CONFLICT",
                                      "detail": f"같은 task_id {key} 로 다른 내용이 이미 실행됨"})
            if self.state in ("FAULT", "DANGER"):
                raise HttpError(409, f"FAILSAFE_ACTIVE: {self.state} {self.fault} (/stop 으로 운영자 해제 필요)")
            start, v0, err = self.here_state()
            plan = self._approach(body, start, v0) if start is not None else None
            if plan is None or plan.status != "OK":
                # 진행 방향을 지키는 경로가 없다 → 새 목표를 수행하지 않는다. 주행 중이던 임무는 취소하고 도로를 따라 멈춘다
                cancelled, action = None, None
                if self.current_task():
                    cancelled = self.task_key
                    self.gen += 1
                    reason = err or f"{plan.status}: {plan.reason}"
                    self._finish(cancelled, "CANCELLED", error=f"NEW_TARGET_NO_FEASIBLE_ROUTE:{key}: {reason}",
                                 physical_state="STOPPING")
                    await self._safe_stop()
                    action = "STOPPING_ALONG_ROAD"
                self._event("NEW_TARGET_REJECTED", rejected_task_id=key, detail=err or plan.reason,
                            cancelled_task_id=cancelled, vehicle_action=action)
                raise HttpError(409, {"reason": "TARGET_UNREACHABLE",
                                      "detail": err or f"{plan.status}: {plan.reason}",
                                      "approach": plan.report() if plan else None,
                                      "cancelled_task_id": cancelled, "vehicle_action": action})
            prev = self.task_key if self.current_task() else None
            if prev:
                self._finish(prev, "CANCELLED", error=f"SUPERSEDED_BY:{key}", physical_state="REPLANNED")
            tel = self.driver.telemetry()
            here_d = math.hypot(tel.x - plan.dest.x, tel.y - plan.dest.y) if tel is not None else math.inf
            already = tel is not None and tel.speed_mps < 0.5 and here_d <= config.ARRIVE_M
            self.gen += 1
            if already:
                # 이미 목적지 도착 반경 안에 서 있다 → 주행하지 않고 바로 도착 처리
                # (길이 0 경로로 출발시키면 차가 움직이지 않아 정지 고장(STALLED)이 된다, 2026-10-10 실서버 시험)
                self.plan, self.speed = plan, plan_speeds(plan.route, 0.0, profile=self.profile)
                seq = self.driver.progress().mission_seq
            else:
                try:
                    seq = await self._drive(plan)
                except DriverError as e:
                    await self._safe_stop()
                    self.task_key = None
                    raise HttpError(500, f"DRIVER_START_FAILED: {e}")
            self.task_key = key
            self.target_ll = (body.get("target") or {}) or dict(zip(("lat", "lon"), plan.dest.ll()))
            tn = self._target_node(plan)
            resp = {"task_id": key, "resource_id": self.resource_id, "status": "STARTED",
                    "tracking_url": f"/ugv/{self.resource_id}/task/{key}", "target_node": tn,
                    "eta_sec": int(round(self.speed.eta_s)), "approach": plan.report(),
                    "superseded_task_id": prev, "duplicate": False}
            task = {"task_id": key, "decision_id": body.get("decision_id"), "resource_id": self.resource_id,
                    "status": "STARTED", "target": body.get("target"), "target_node": tn["node_id"] if tn else None,
                    "approach": plan.report(), "mission_result": "NOT_ARRIVED", "physical_state": "DRIVING",
                    "progress": {"phase": "ENROUTE", "waypoint": 0, "total": len(self.speed.items)},
                    "observation": None, "error": None, "replans": [], "events": [],
                    "timing": {"start_sim_s": round(self.clock.now(), 2), "start_wall": _now_iso(),
                               "eta_sec": resp["eta_sec"]},
                    "superseded_task_id": prev, "mission_seq": seq}
            self.store.register(key, body, task, resp)
            self._event("TASK_STARTED", approach=plan.report(), eta_sec=resp["eta_sec"], superseded=prev)
            if already:
                self.state = "IDLE"
                await self._arrived(tel, here_d)
            return resp

    async def _drive(self, plan: ApproachPlan, state="DRIVING") -> int:
        # 실행 경계의 최종 확인: 최종 검증(approach.verify_route)을 통과한 경로만 드라이버로 보낸다.
        # 검사 결과를 보고에만 싣고 실행은 하는 우회를 막는다 (2026-10-10 검토 — 예전 화재 이탈이 그랬다)
        chk = (plan.detail or {}).get("route_check") or {}
        if chk.get("ok") is not True:
            raise DriverError(f"UNVERIFIED_ROUTE: 최종 경로 검사 통과 기록 없음 ({plan.mode}, route_check={chk.get('ok')})")
        t = self.driver.telemetry()
        self.speed = plan_speeds(plan.route, t.speed_mps if t else 0.0, profile=self.profile)
        self.plan = plan
        self.seq = await self.driver.follow(self.speed.items)
        self.state = state
        self._off_since, self._near_i = None, 0
        self._move_ref = (self.clock.now(), t.x, t.y) if t else None
        self._drive_started_sim = self.clock.now()
        return self.seq

    async def _safe_stop(self) -> dict:
        """도로를 따라 감속 정지. 남은 계획 경로가 있으면 그것을, 없으면 진행 방향 도로를 따라 제동거리만큼 간다.
        즉시 멈춘다고 가정하지 않는다. 도로 위치·방향을 모르면 드라이버 기본 정지(진행 방향 직선)로 물러선다.
        정지 경로가 제동거리보다 짧거나(short) 회전반경·차체 도로 포함 검사를 통과하지 못하면(drivable=false) 그 사실을
        돌려주고 기록한다 — 정지 명령은 그래도 보낸다 (다른 선택지가 없다). 정지가 안전 확보를 뜻하지 않는다."""
        info = {}
        try:
            tel = self.driver.telemetry()
            items = None
            if tel is not None and tel.speed_mps >= 0.3:
                v = tel.speed_mps
                need = self.profile.stop_distance(v)
                g = self.graph
                prefer = None
                if self.speed is not None and self._plan_heading(tel) is not None:
                    _, i = _dist_to_polyline(tel.x, tel.y, self.speed.pts)
                    prefer = [(tel.x, tel.y)] + self.speed.pts[i + 1:]
                pos = self.net.snap(tel.x, tel.y)
                dp = g.dirpos(pos, tel.heading_deg) if prefer is None else None
                info = {"basis": "NO_ROAD_DIRECTION", "needed_m": round(need, 1), "speed_mps": round(v, 2)}
                if prefer is not None or dp is not None:
                    pts, got = g.stop_path(dp, need, prefer)
                    if len(pts) >= 2 and got > 1.0:
                        route = Route(pts, [self.profile.max_speed_mps] * len(pts), [], pos, pos, 0.0)
                        items = plan_speeds(route, v, profile=self.profile).items
                        chk = g.check_route(pts, label="STOP_PATH")
                        info = {"basis": "PLANNED_ROUTE" if prefer else "ROAD_AHEAD", "length_m": round(got, 1),
                                "needed_m": round(need, 1), "short": got < need - 0.5, "speed_mps": round(v, 2),
                                "drivable": chk["ok"], "steering_violations": chk["steering_violations"],
                                "body_violations": chk["body_violations"], "worst_excess_m": chk["worst_excess_m"]}
            if items:
                self.seq = await self.driver.follow(items)
            else:
                self.seq = await self.driver.stop()
                info = {**info, "basis": info.get("basis", "STANDSTILL") if tel is not None and tel.speed_mps < 0.3
                        else "DRIVER_DEFAULT (진행 방향 직선 — 도로를 따르지 않을 수 있음)"}
                if tel is not None and tel.speed_mps >= 0.3:
                    info["drivable"] = None
            self.state = "STOPPING"
            problems = []
            if info.get("short"):
                problems.append(f"정지 경로 {info['length_m']} m < 필요 {info['needed_m']} m")
            if info.get("drivable") is False:
                problems.append("정지 경로가 회전반경·차체 도로 포함 검사를 통과하지 못함")
            if info.get("drivable") is None and "DRIVER_DEFAULT" in str(info.get("basis")):
                problems.append("도로 방향을 몰라 직선 정지")
            info["problems"] = problems
            self._event("STOP_PATH", **info)
        except DriverError as e:
            info = {"basis": "FAILED", "problems": [f"정지 명령 실패: {e}"]}
            self._event("STOP_FAILED", detail=str(e))
        self.last_stop = info
        return info

    def _finish(self, key, status, *, error=None, physical_state=None, mission_result=None):
        t = self.store.tasks.get(key)
        if not t or t.get("status") not in ACTIVE:
            return
        t["status"] = status
        if error:
            t["error"] = error
        if physical_state:
            t["physical_state"] = physical_state
        if mission_result:
            t["mission_result"] = mission_result
        t.setdefault("timing", {})["end_sim_s"] = round(self.clock.now(), 2)
        t["timing"]["end_wall"] = _now_iso()
        self._event(f"TASK_{status}", detail=error)
        self.store.save()

    # ------------------------------------------------------------------ 중단·정지
    async def abort(self, key: str, reason: str = "ABORT_REQUESTED") -> dict:
        async with self.lock:
            t = self.store.tasks.get(key)
            if t is None or t.get("resource_id") != self.resource_id:
                raise HttpError(404, f"task {key} 없음")
            if key == self.task_key and t.get("status") in ACTIVE:
                self.gen += 1
                await self._safe_stop()
                self._finish(key, "CANCELLED", error=reason, physical_state="STOPPING")
            return t

    async def stop(self, reason: str = "STOP_REQUESTED") -> dict:
        """주행 중지 + 운영자 해제. FAULT·DANGER 는 차량이 멈췄고(0.2 m/s 미만) 텔레메트리가 살아 있을 때만 해제한다.
        아직 움직이면 도로를 따라 멈추게 하고 해제는 보류한다 (다시 /stop 으로 해제)."""
        async with self.lock:
            self.gen += 1
            if self.current_task():
                self._finish(self.task_key, "CANCELLED", error=reason, physical_state="STOPPING")
            tel = self.driver.telemetry()
            stopped = tel is not None and tel.speed_mps < 0.2 and tel.age_s() <= config.TELEMETRY_STALE_S
            released, pending = None, None
            if self.fault is not None and not stopped:
                pending = "NOT_STOPPED_OR_NO_TELEMETRY: 정지·텔레메트리 확인 뒤 다시 /stop"
                held = self.state
                if tel is not None and tel.speed_mps >= 0.2:
                    await self._safe_stop()
                self.state = held                      # 고장·위험 상태 유지 (자동 재출발 없음)
            else:
                released = self.fault
                self.fault = None
                self.danger = None
                if self.state in ("FAULT", "DANGER"):
                    self.state = "IDLE"
                await self._safe_stop()                # 멈춰 있으면 드라이버가 제자리 정지로 처리한다
            self._event("STOP", detail=reason, released_fault=released, release_pending=pending)
            return {"resource_id": self.resource_id, "state": self.resource_state(), "released_fault": released,
                    "release_pending": pending}

    # ------------------------------------------------------------------ 감시
    def _fail(self, code: str, detail: str):
        self.fault = f"{code}: {detail}"
        if self.current_task():
            self._finish(self.task_key, "FAILED", error=self.fault, physical_state=code)
        self.state = "FAULT"
        self._event("FAULT", code=code, detail=detail)

    async def tick(self):
        now = self.clock.now()
        tel = self.driver.telemetry()
        if tel is None:
            return
        moving = self.state in ("DRIVING", "EVADING")
        if moving and tel.age_s() > config.TELEMETRY_STALE_S:
            async with self.lock:
                self._fail("TELEMETRY_LOST", f"위치 수신 {tel.age_s():.1f} s 없음")
                await self._safe_stop()
                self.state = "FAULT"
            return
        if self.state == "STOPPING" and tel.speed_mps < 0.2:
            self.state = "IDLE"
            self._event("STOPPED", position={"lat": tel.lat, "lon": tel.lon})
        if moving:
            await self._supervise_drive(now, tel)
        if now - self._last_safety >= config.SAFETY_CHECK_PERIOD_S:
            self._last_safety = now
            await self._safety(now, tel)

    async def _supervise_drive(self, now, tel):
        sp = self.speed
        prog = self.driver.progress()
        # 진행: 계획 선형 위 가장 가까운 점 (지난 위치 근처만 본다)
        d, i = _dist_to_polyline(tel.x, tel.y, sp.pts, self._near_i - 20, self._near_i + 200)
        if d > config.OFF_ROUTE_M:
            d, i = _dist_to_polyline(tel.x, tel.y, sp.pts)
        self._near_i = i
        remaining = sp.cum[-1] - sp.cum[min(i + 1, len(sp.cum) - 1)] + math.dist((tel.x, tel.y), sp.pts[min(i + 1, len(sp.pts) - 1)])
        t = self.current_task()
        if t is not None:
            t["status"] = "IN_PROGRESS"
            eta_rem = sum((sp.cum[k] - sp.cum[k - 1]) / max((sp.v[k] + sp.v[k - 1]) / 2, 0.5)
                          for k in range(max(i + 1, 1), len(sp.v)))
            t["progress"] = {"phase": "EVADING" if self.state == "EVADING" else "ENROUTE",
                             "waypoint": prog.current, "total": prog.total, "remaining_m": round(remaining, 1),
                             "eta_remaining_sec": int(round(eta_rem)), "speed_mps": round(tel.speed_mps, 2),
                             "sim_time_s": round(now, 2), "off_route_m": round(d, 1)}
        # 도착
        dest = self.plan.dest
        to_dest = math.hypot(tel.x - dest.x, tel.y - dest.y)
        if prog.mission_seq == self.seq and prog.finished and tel.speed_mps < 0.5:
            async with self.lock:
                if prog.mission_seq != self.seq:
                    return
                if to_dest <= config.ARRIVE_M * 3:
                    await self._arrived(tel, to_dest)
                else:
                    self._fail("ARRIVAL_MISMATCH", f"미션 끝, 목적지까지 {to_dest:.1f} m")
            return
        # 경로 이탈
        if d > config.OFF_ROUTE_M:
            self._off_since = self._off_since if self._off_since is not None else now
            if now - self._off_since >= config.OFF_ROUTE_CONFIRM_S:
                async with self.lock:
                    self._fail("OFF_ROUTE", f"계획 경로에서 {d:.1f} m")
                    await self._safe_stop()
                    self.state = "FAULT"
                return
        else:
            self._off_since = None
        # 정지 고장
        if self._move_ref is None or math.hypot(tel.x - self._move_ref[1], tel.y - self._move_ref[2]) >= config.STALL_MOVE_M:
            self._move_ref = (now, tel.x, tel.y)
        elif now - self._move_ref[0] >= config.STALL_TIMEOUT_S:
            async with self.lock:
                self._fail("STALLED", f"{config.STALL_TIMEOUT_S:g} s 동안 {config.STALL_MOVE_M:g} m 미만 이동")
                await self._safe_stop()
                self.state = "FAULT"
            return
        # 차량 이상 (PX4): 주행 중 시동 꺼짐
        if self.driver.kind == "px4" and not tel.armed and now - (self._drive_started_sim or now) > 5.0:
            async with self.lock:
                self._fail("VEHICLE_FAULT", "주행 중 시동 꺼짐 (disarm)")
            return

    async def _arrived(self, tel, to_dest):
        evading = self.state == "EVADING"
        self.state = "IDLE"
        obs = {"observation_type": "ROAD_STATUS", "arrived_node": node_id_of(self.plan.dest),
               "position": {"lat": tel.lat, "lon": tel.lon}, "fuel_pct": None,
               "note": "도로 지점 도착 기록. 화재 관측이 아니다 (화재 관측은 4단계 관측 보고)"}
        t = self.current_task()
        self._event("ARRIVED" if not evading else "EVADED", to_dest_m=round(to_dest, 1),
                    position={"lat": tel.lat, "lon": tel.lon})
        if t is None:
            return
        if evading:
            # 이탈을 마쳤다 → 원래 목표 접근을 다시 계획한다
            await self._replan_task(t, "AFTER_EVASION")
            return
        t["observation"] = obs
        t["progress"] = {**(t.get("progress") or {}), "phase": "ARRIVED", "remaining_m": round(to_dest, 1),
                         "eta_remaining_sec": 0}
        self._finish(self.task_key, "COMPLETED", physical_state="STOPPED_ON_STATION", mission_result="ARRIVED")

    async def _replan_task(self, t, why):
        here, v0, err = self.here_state()
        plan = None if here is None else plan_approach(
            self.net, self.fire_source.area(), here, float(self.target_ll["lat"]), float(self.target_ll["lon"]),
            v0=v0, access=self.access, graph=self.graph)
        rec = {"sim_time_s": round(self.clock.now(), 2), "why": why, "result": plan.status if plan else "NO_START_STATE",
               "approach": plan.report() if plan else {"reason": err}}
        t.setdefault("replans", []).append(rec)
        if plan is None or plan.status != "OK":
            await self._safe_stop()
            self._finish(self.task_key, "FAILED", error=f"NO_SAFE_APPROACH: {plan.reason if plan else err}",
                         physical_state="STOPPING")
            return
        try:
            await self._drive(plan)
        except DriverError as e:
            await self._safe_stop()
            self._finish(self.task_key, "FAILED", error=f"DRIVER: {e}", physical_state="STOPPING")
            return
        tn = self._target_node(plan)
        t["target_node"] = tn["node_id"] if tn else None
        t["approach"] = plan.report()
        self._event("REPLANNED", why=why, approach=plan.report())
        self.store.save()

    async def _safety(self, now, tel):
        fire = self.fire_source.area()
        if not len(fire):
            self.last_clearance_m = None
            return
        here_c = fire.distance(tel.x, tel.y)
        self.last_clearance_m = here_c
        if self.state in ("FAULT", "DANGER", "EVADING"):
            return
        async with self.lock:
            if here_c < config.FIRE_STANDOFF_M:
                here, v0, err = self.here_state()
                ev = plan_evasion(self.net, fire, here, v0=v0, access=self.access, graph=self.graph) if here is not None else None
                if ev is None or ev.status != "OK":
                    # 자동 이탈 불가 — 안전 확보가 아니다. 도로를 따라 제동하고 운영자 개입을 요청한다 (자동 재출발 없음)
                    await self._enter_danger(here_c, ev, err)
                    return
                self.gen += 1
                try:
                    await self._drive(ev, state="EVADING")
                except DriverError as e:
                    await self._enter_danger(here_c, ev, f"EVASION_START_FAILED: {e}")
                    return
                self._event("EVADING", clearance_m=round(here_c, 1), evasion=ev.report())
                return
            if self.state == "DRIVING" and self.speed is not None:
                rest = self.speed.pts[self._near_i:]
                c = fire.clearance_along(rest[::4] + rest[-1:], max_m=config.FIRE_STANDOFF_M + 1.0)
                if c < config.FIRE_STANDOFF_M and self.current_task():
                    self.gen += 1
                    await self._replan_task(self.current_task(), f"ROUTE_CLEARANCE_{c:.0f}M")

    async def _enter_danger(self, here_c, ev, err=None):
        """DANGER: 실행 가능한 이탈 경로 없음 → 도로를 따라 가능한 범위에서 감속 정지 + 운영자 개입 요청.
        정지가 탈출·안전 확보를 뜻하지 않는다. 정지 경로의 길이 부족·주행 불가도 함께 보고한다."""
        self.gen += 1
        stop = await self._safe_stop()
        why = (ev.reason if ev is not None and ev.reason else None) or err or "UNKNOWN"
        self.fault = f"NO_SAFE_EVASION_ROUTE: 화재 경계 {here_c:.0f} m (안전거리 {config.FIRE_STANDOFF_M:g} m 안), {why}"
        if stop.get("problems"):
            self.fault += " / 정지: " + "; ".join(stop["problems"])
        if self.current_task():
            self._finish(self.task_key, "FAILED", error=self.fault, physical_state="DANGER")
        self.state = "DANGER"
        self.danger = {"clearance_m": round(here_c, 1), "evasion": ev.report() if ev else {"reason": err},
                       "stop_path": stop, "meaning": DANGER_MEANING, "operator_action_required": True,
                       "failure_reasons": [why] + stop.get("problems", [])}
        self._event("DANGER", **self.danger)

    # ------------------------------------------------------------------ 상태
    def state_view(self) -> dict:
        tel = self.driver.telemetry()
        t = self.current_task()
        pos = {"lat": tel.lat, "lon": tel.lon} if tel else {"lat": None, "lon": None}
        here = self.net.snap(tel.x, tel.y) if tel else None
        stopped = tel is not None and tel.speed_mps < 0.2 and self.state == "IDLE"
        return {
            "resource_id": self.resource_id, "resource_type": self.resource_type, "base": self.station["base_id"],
            "state": self.resource_state(), "position": pos, "fuel_pct": None,
            "current_node": node_id_of(here) if (stopped and here) else None,
            "current_task_id": t["task_id"] if t else None, "driver": self.driver.kind,
            "driver_status": tel.mode if tel else "NO_TELEMETRY",
            "updated_at": (time.time() - tel.age_s()) if tel else 0.0, "fault": self.fault,
            "sim_time_s": round(self.clock.now(), 2), "mission_state": self.state,
            "speed_mps": round(tel.speed_mps, 2) if tel else None, "heading_deg": round(tel.heading_deg, 1) if tel else None,
            "station": self.station,
            "operator_intervention": {
                "required": self.state in ("FAULT", "DANGER"), "reason": self.fault,
                "meaning": DANGER_MEANING if self.state == "DANGER" else None,
                "danger": self.danger if self.state == "DANGER" else None,
                "release": "POST /ugv/{id}/stop — 차량 정지(0.2 m/s 미만)·텔레메트리 확인 뒤에만 해제. 원격 운전 기능 없음"},
            "safety": {"fire_clearance_m": None if self.last_clearance_m is None or math.isinf(self.last_clearance_m)
                       else round(self.last_clearance_m, 1), "standoff_m": config.FIRE_STANDOFF_M,
                       **(self.fire_source.area().summary())},
        }
