# -*- coding: utf-8 -*-
"""ugv/approach.py — 목표 좌표에 대한 도로 접근 지점 선택 (화재 안전거리 포함)과 안전 이탈 지점.

규칙 (프롬프트 §4, 확정):
- 목표 좌표에 무조건 가지 않는다. 알려진 화재 영역 경계에서 FIRE_STANDOFF_M(100 m) 이상 떨어진, 도달 가능한 도로 지점을 고른다.
- 경로 전체도 같은 안전거리를 지킨다. 출발 도로의 남은 구간만 예외 (차량이 이미 거기 있다 — 위반이면 이탈 처리로 간다).
- 후보: 목표에서 APPROACH_SEARCH_M(2 km = 관측 최대 거리) 안의 도로 표본 + 목표에서 가장 가까운 도로 지점.
- 선택 기준 (관측에 유리한 지점의 1차 근사): 목표까지 직선거리가 가장 짧은 지점, 25 m 차이 안이면 도착이 빠른 쪽.
  여기에 잠정 ETA 여유 규칙을 건다: 기준 도착 시간(목표 1 km 안 후보 중 가장 빠른 도착)보다 최대 25 %(최소 60 s)
  늦는 후보만 본다. 확정된 관측 우선 기준이 아니고 우회를 줄이는 임시 정책이다 (값은 config, 5단계 지형 가시성
  연결 때 다시 평가). 비교 ETA 는 도로 제한속도만 본 값이라 최종 도착 예정(속도 계획 eta_sec)과 다르다.
- 경로는 진행 방향을 유지한다 (ugv/drive_graph.py: 즉시 U턴 금지, 일방통행, 차체가 들어오는 회전만, 첫 교차로 감속 거리).
- 도착 지점은 도착 방향에서 소속 거점 출입 지점으로 돌아올 경로가 계획 시점에 있어야 고른다 (막다른 길에 들어가 갇히지
  않게). 화재로 나중에 끊길 수 있으므로 복귀를 보장하지 않는다.
- 적절한 지점이 없으면 NO_SAFE_APPROACH (화재 때문) / TARGET_UNREACHABLE (도로가 멀거나 끊김·방향상 갈 수 없음·복귀 경로 없음).
"""

import math
from dataclasses import dataclass, field
from typing import Optional

from . import config
from .drive_graph import DirPos
from .fire import FireArea
from .geo import FRAME
from .roads import RoadNetwork, RoadPos, Route


@dataclass
class ApproachPlan:
    status: str                         # OK / TARGET_UNREACHABLE / NO_SAFE_APPROACH
    reason: Optional[str] = None
    mode: Optional[str] = None          # NEAREST_ROAD (가장 가까운 도로 지점이 안전) / FIRE_STANDOFF (떨어진 지점)
    dest: Optional[RoadPos] = None
    route: Optional[Route] = None
    target_xy: Optional[tuple] = None
    nearest_road_m: Optional[float] = None
    dist_to_target_m: Optional[float] = None
    fire_clearance_m: Optional[float] = None    # 목적지 지점 ~ 화재 경계
    route_min_clearance_m: Optional[float] = None
    candidates_checked: int = 0
    detail: dict = field(default_factory=dict)

    def report(self) -> dict:
        d = {"status": self.status, "reason": self.reason, "mode": self.mode,
             "nearest_road_m": _r(self.nearest_road_m), "dist_to_target_m": _r(self.dist_to_target_m),
             "fire_clearance_m": _r(self.fire_clearance_m), "route_min_clearance_m": _r(self.route_min_clearance_m),
             "standoff_m": config.FIRE_STANDOFF_M, "candidates_checked": self.candidates_checked,
             "selection_basis": "CLOSEST_TO_TARGET_WITHIN_ETA_SLACK (지형 가시성 미반영)",
             "selection_rule": {"type": "PROVISIONAL_ETA_SLACK", "status": "PROVISIONAL",
                                "slack_ratio": config.APPROACH_ETA_SLACK_RATIO, "slack_min_s": config.APPROACH_ETA_SLACK_S,
                                "near_m": config.APPROACH_NEAR_M,
                                "eta_basis": "ROAD_SPEED_ONLY (곡선 감속·가감속 미반영. 최종 도착 예정은 eta_sec)",
                                "ref_eta_s": _r(self.detail.get("_ref_eta_s")),
                                "limit_eta_s": _r(self.detail.get("_limit_eta_s")),
                                "chosen_eta_s": _r(self.detail.get("_chosen_eta_s"))},
             **{k: v for k, v in self.detail.items() if not k.startswith("_")}}
        if self.dest:
            lat, lon = self.dest.ll()
            d["point"] = {"lat": round(lat, 7), "lon": round(lon, 7), "road_id": self.dest.road_id,
                          "s_m": round(self.dest.s, 1)}
        if self.route:
            d["route_length_m"] = round(self.route.length, 1)
        return d


def _r(v):
    return None if v is None or (isinstance(v, float) and math.isinf(v)) else round(v, 1)


def node_id_of(pos: RoadPos) -> str:
    """도로 위 지점 식별자 (평가 → 실행에서 같은 지점으로 가기 위한 target_node, UGV-01)."""
    return f"RP:{pos.road_id}:{pos.s:.1f}"


def parse_node_id(net: RoadNetwork, node_id: str) -> Optional[RoadPos]:
    if not isinstance(node_id, str):
        return None
    if node_id.startswith("RP:"):
        try:
            _, rid, s = node_id.split(":")
            return net.at(rid, float(s)) if rid in net.roads else None
        except ValueError:
            return None
    if node_id in net.nodes:
        return net.snap(*net.nodes[node_id], max_m=1.0)
    return None


class SafetyIndex:
    """도로별 화재 안전 여부 (도로 표본 중 하나라도 경계 100 m 안이면 그 도로 전체를 막는다 — 보수적)."""

    def __init__(self, net: RoadNetwork, fire: FireArea, standoff: float = config.FIRE_STANDOFF_M):
        self.net, self.fire, self.standoff = net, fire, standoff
        self._clear = {}
        self.unsafe_roads = set()
        if len(fire):
            for p in net.samples():
                c = fire.distance(p.x, p.y, max_m=standoff + 1.0)
                if c < standoff:
                    self.unsafe_roads.add(p.road_id)

    def road_ok(self, rid: str) -> bool:
        return rid not in self.unsafe_roads

    def clearance(self, x, y, max_m=5000.0) -> float:
        return self.fire.distance(x, y, max_m=max_m)

    def segment_ok(self, road_id: str, s0: float, s1: float) -> bool:
        if road_id not in self.unsafe_roads:
            return True
        return self.fire.clearance_along(self.net.polyline(road_id, s0, s1), max_m=self.standoff + 1.0) >= self.standoff


def _direction_report(start: DirPos, v0: float, route: Optional[Route], ret_ok, access) -> dict:
    return {"start_road": start.road_id, "start_toward_node": start.toward, "v0_mps": round(v0, 2),
            "arrive_toward_node": route.arrive_toward if route else None,
            "u_turn": "FORBIDDEN (같은 도로 즉시 되짚기·후진·3점 회전 없음)",
            "return_path_at_plan_time": ret_ok,
            "return_basis": None if access is None else
            "계획 시점 화재 기준 출입 지점까지 경로 있음 여부 — 이후 화재로 끊길 수 있어 복귀를 보장하지 않는다"}


def plan_approach(net: RoadNetwork, fire: FireArea, start: DirPos, target_lat: float, target_lon: float, *,
                  v0: float = 0.0, access: Optional[RoadPos] = None,
                  standoff: float = config.FIRE_STANDOFF_M, search_m: float = config.APPROACH_SEARCH_M,
                  snap_limit: float = config.TARGET_SNAP_M, fixed_dest: Optional[RoadPos] = None) -> ApproachPlan:
    """start: 진행 방향을 포함한 출발 위치 (명령 교체 지연만큼 앞으로 옮긴 것). access: 소속 거점 출입 지점 (복귀 검사)."""
    g = net.graph
    tx, ty = FRAME.to_xy(target_lat, target_lon)
    near = net.snap(tx, ty, max_m=snap_limit)
    if near is None:
        return ApproachPlan("TARGET_UNREACHABLE", "NO_ROAD_WITHIN_SNAP_LIMIT", target_xy=(tx, ty),
                            detail={"snap_limit_m": snap_limit})
    safety = SafetyIndex(net, fire, standoff)
    tree = g.search(start, v0, safety.road_ok)
    rs = g.return_set(access, safety.road_ok) if access is not None else None

    near_clear = safety.clearance(near.x, near.y)
    cands = [fixed_dest] if fixed_dest is not None else (
        [near] + [p for p in net.samples() if math.hypot(p.x - tx, p.y - ty) <= search_m])
    ok, checked, no_route, no_return = [], 0, 0, 0
    for p in cands:
        c = safety.clearance(p.x, p.y)
        if c < standoff:
            continue
        checked += 1
        arrs = g.arrivals(tree, p, safety.segment_ok)
        if not arrs:
            no_route += 1
            continue
        if rs is not None:
            arrs = [a for a in arrs if g.can_return(rs, p.road_id, p.s, a.toward)]
            if not arrs:
                no_return += 1
                continue
        a = min(arrs, key=lambda a: a.time_s)
        ok.append((p, math.hypot(p.x - tx, p.y - ty), c, a.time_s, a))
    counts = {"candidates_no_directed_route": no_route, "candidates_no_return_path": no_return}
    if not ok:
        if fixed_dest is not None:
            reason = "FIXED_DEST_UNSAFE_UNREACHABLE_OR_NO_RETURN"
        elif no_return and not no_route:
            reason = "NO_APPROACH_WITH_RETURN_PATH"
        elif near_clear >= standoff and not safety.unsafe_roads:
            reason = "NO_DIRECTED_ROUTE (진행 방향·일방통행·회전 가능 조건으로 갈 수 없음)"
        elif near_clear >= standoff:
            reason = "NO_DIRECTED_ROUTE_OR_ROUTE_THROUGH_FIRE"
        else:
            reason = "NO_ROAD_POINT_BEYOND_STANDOFF_REACHABLE"
        status = "NO_SAFE_APPROACH" if len(fire) and (near_clear < standoff or safety.unsafe_roads) else "TARGET_UNREACHABLE"
        return ApproachPlan(status, reason, target_xy=(tx, ty), nearest_road_m=near.dist, candidates_checked=checked,
                            detail={"nearest_road_fire_clearance_m": _r(near_clear),
                                    "unsafe_roads": len(safety.unsafe_roads), **counts,
                                    "direction": _direction_report(start, v0, None, None, access)})
    # 기준 시간 = 목표 APPROACH_NEAR_M 안 후보 중 가장 빠른 도착 (없으면 전체). 그보다 최대 ETA 여유만큼 늦는
    # 후보 중 목표에 가장 가까운 지점. 목표에 150 m 더 가까워지려고 수십 분 돌아가는 일을 막는다 (2026-10-09 실측:
    # 트윈 화재에서 271 m·27 km 우회 36분 대신 523 m·5 km 6분). 비교 시간은 방향을 반영한 도로 시간이다
    near_set = [o for o in ok if o[1] <= config.APPROACH_NEAR_M] or ok
    t_ref = min(o[3] for o in near_set)
    limit = t_ref + max(config.APPROACH_ETA_SLACK_S, config.APPROACH_ETA_SLACK_RATIO * t_ref)
    dest, d, c, t, arr = min((o for o in ok if o[3] <= limit), key=lambda o: (round(o[1] / 25.0), o[3]))
    sel = {"_ref_eta_s": t_ref, "_limit_eta_s": limit, "_chosen_eta_s": t}
    route = g.build_route(tree, dest, arr)
    # 출발 도로에서 벗어나기 전 구간은 검사에서 뺀다 (이미 있는 곳). 그 뒤 구간의 최소 이격을 기록한다
    route_clear = fire.clearance_along(route.pts[1:]) if len(fire) else math.inf
    mode = "NEAREST_ROAD" if near_clear >= standoff and math.hypot(dest.x - near.x, dest.y - near.y) <= 25.0 else "FIRE_STANDOFF"
    return ApproachPlan("OK", None, mode, dest, route, (tx, ty), near.dist, d, c, route_clear, checked,
                        detail={"nearest_road_fire_clearance_m": _r(near_clear), "unsafe_roads": len(safety.unsafe_roads),
                                "fire_cells_known": len(fire), "fire_source": fire.source, **counts, **sel,
                                "direction": _direction_report(start, v0, route, True if rs is not None else None, access)})


def plan_evasion(net: RoadNetwork, fire: FireArea, here: DirPos, *, v0: float = 0.0, access: Optional[RoadPos] = None,
                 standoff: float = config.FIRE_STANDOFF_M, search_m: float = config.EVADE_SEARCH_M) -> ApproachPlan:
    """현재 위치가 안전거리를 위반할 때: 진행 방향을 유지하며 불 칸 안을 지나지 않는 경로로 갈 수 있는 가장 빠른 안전 지점.
    출입 지점으로 돌아올 경로가 있는 지점을 먼저 고르고, 없으면 복귀 경로 없는 지점이라도 고른다 (화재 이탈이 먼저 —
    결과에 return_path_at_plan_time=false 로 남긴다). 그것도 없으면 NO_SAFE_EVASION_ROUTE (→ DANGER)."""
    g = net.graph
    burning_free = SafetyIndex(net, fire, standoff=0.5)        # 칸 안(경계 0.5 m 이내)만 막는다
    tree = g.search(here, v0, burning_free.road_ok)
    rs = g.return_set(access, SafetyIndex(net, fire, standoff).road_ok) if access is not None else None
    best = None
    for p in net.samples():
        if math.hypot(p.x - here.pos.x, p.y - here.pos.y) > search_m:
            continue
        c = fire.distance(p.x, p.y)
        if c < standoff:
            continue
        for a in g.arrivals(tree, p):
            ret = g.can_return(rs, p.road_id, p.s, a.toward) if rs is not None else None
            key = (0 if ret in (True, None) else 1, a.time_s)
            if best is not None and key >= best[0]:
                continue
            r = g.build_route(tree, p, a)
            # 이미 불 칸 안이면 빠져나가는 첫 구간은 허용한다. 한 번 나온 뒤 다시 불 칸에 들어가는 경로는 버린다
            inside = [fire.distance(x, y, max_m=1.0) < 0.5 for x, y in r.pts]
            k = next((i for i, v in enumerate(inside) if not v), len(inside))
            if any(inside[k:]):
                continue
            best = (key, p, r, c, ret)
    if best is None:
        return ApproachPlan("NO_SAFE_APPROACH", "NO_SAFE_EVASION_ROUTE",
                            detail={"here_clearance_m": _r(fire.distance(here.pos.x, here.pos.y)),
                                    "direction": _direction_report(here, v0, None, None, access)})
    _, p, r, c, ret = best
    return ApproachPlan("OK", None, "EVADE", p, r, None, None, None, c, fire.clearance_along(r.pts), 0,
                        detail={"here_clearance_m": _r(fire.distance(here.pos.x, here.pos.y)),
                                "direction": _direction_report(here, v0, r, ret, access)})
