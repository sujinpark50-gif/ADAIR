# -*- coding: utf-8 -*-
"""ugv/route_preview.py — UGV 서버 없이 같은 프로세스에서 쓰는 도로 경로 미리보기 (관제판·통합 시연·시나리오 도구).

실제 주행 판단은 UGV 서버(ugv/server.py)가 한다. 이 모듈은 같은 도로망(원자료 gpkg)과 같은 진행 방향 규칙
(ugv/drive_graph.py)으로 '어디로 어떻게 갈 수 있는지'만 계산한다. 화재 안전거리·접근 지점 선택은 하지 않는다.

    from ugv import route_preview as rp
    rp.station("A")                                   # 거점 출입 지점
    rp.plan(38.06, 128.17, 38.0277, 128.1303)         # 도로 경로, ETA(도로 속도 기준), 경로 점
"""

import math
import threading
from typing import Callable, List, Optional

from . import config
from .drive_graph import DirPos, access_point
from .geo import FRAME
from .roads import RoadNetwork, RoadPos

_lock = threading.Lock()
_net: Optional[RoadNetwork] = None


def network() -> RoadNetwork:
    global _net
    with _lock:
        if _net is None:
            _net = RoadNetwork()
        return _net


def node_id(pos: RoadPos) -> str:
    return f"RP:{pos.road_id}:{pos.s:.1f}"


def nearest_road(lat: float, lon: float) -> dict:
    """가장 가까운 도로 지점 {node_id, lat, lon, dist_m, road_id}."""
    p = network().snap_ll(lat, lon)
    la, lo = p.ll()
    return {"node_id": node_id(p), "lat": la, "lon": lo, "dist_m": p.dist, "road_id": p.road_id}


def station(base_id: str) -> Optional[dict]:
    """거점 출입 지점 {base_id, name, lat, lon, heading_deg, station_lat, station_lon, note}."""
    acc = access_point(network(), base_id)
    if acc is None:
        return None
    st = config.load_station(base_id)
    la, lo = acc[0].ll()
    return {"base_id": base_id, "name": st.get("name"), "lat": la, "lon": lo, "heading_deg": acc[1],
            "station_lat": st["lat"], "station_lon": st["lon"], "node_id": node_id(acc[0]), "note": acc[2]}


def plan(from_lat: float, from_lon: float, to_lat: float, to_lon: float, *, heading_deg: Optional[float] = None,
         road_ok: Callable[[str], bool] = lambda r: True, snap_limit_m: float = config.TARGET_SNAP_M) -> dict:
    """출발 → 목표에 가장 가까운 도로 지점까지 진행 방향을 지키는 경로.
    heading_deg 를 모르면(미리보기) 두 방향을 다 보고 빠른 쪽을 쓴다 — 결과 basis 에 남긴다.
    road_ok(도로) 가 False 인 도로는 지나지 않는다 (막힌 도로 시연)."""
    net = network()
    g = net.graph
    start = net.snap_ll(from_lat, from_lon)
    goal = net.snap_ll(to_lat, to_lon)
    out = {"reachable": False, "eta_s": None, "length_m": None, "path": [], "t": [], "start_node": node_id(start),
           "target_node": node_id(goal), "snap_m": round(goal.dist, 1), "start_offset_m": round(start.dist, 1),
           "basis": "DIRECTED_ROAD_TIME (진행 방향 유지·즉시 U턴 없음, 도로 제한속도 기준 — 곡선 감속·가감속 미반영)",
           "reason": None}
    if goal.dist > snap_limit_m:
        out["reason"] = "TARGET_TOO_FAR_FROM_ROAD"
        return out
    r0 = net.roads[start.road_id]
    if heading_deg is None:
        starts = [DirPos(start, to) for to in (r0.a, r0.b) if g.oneway_ok(start.road_id, g.other(start.road_id, to))]
        out["basis"] += " — 출발 방향 모름: 두 방향 중 빠른 쪽"
    else:
        dp = g.dirpos(start, heading_deg)
        starts = [dp] if dp is not None else []
    best = None
    for sp in starts:
        tree = g.search(sp, 0.0, road_ok)
        arrs = g.arrivals(tree, goal)
        if arrs:
            a = min(arrs, key=lambda a: a.time_s)
            if best is None or a.time_s < best[0]:
                best = (a.time_s, tree, a)
    if best is None:
        out["reason"] = "NO_DIRECTED_ROUTE"
        return out
    route = g.build_route(best[1], goal, best[2])
    t, acc = [0.0], 0.0
    for (p, q), v in zip(zip(route.pts, route.pts[1:]), route.vlim[1:]):
        acc += math.dist(p, q) / max(v, 0.5)
        t.append(acc)
    out.update(reachable=True, eta_s=round(acc, 1), length_m=round(route.length, 1), road_ids=route.road_ids,
               path=[dict(zip(("lat", "lon"), FRAME.to_ll(x, y))) for x, y in route.pts], t=[round(v, 1) for v in t])
    return out


def segments() -> List[List[float]]:
    """도로 끝점 쌍 [[lat_a, lon_a, lat_b, lon_b], ...] (지도에 도로를 그릴 때)."""
    net = network()
    out = []
    for r in net.roads.values():
        (la, lo), (lb, lob) = FRAME.to_ll(*r.xy[0]), FRAME.to_ll(*r.xy[-1])
        out.append([la, lo, lb, lob])
    return out
