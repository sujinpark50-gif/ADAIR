# -*- coding: utf-8 -*-
"""
integration/bridge_ugv.py
=========================
UGV Connector의 "integrated" 구현.

박유홍님의 실제 도로 그래프(ugv/road_graph.py, Dijkstra)와 시연 도로망
(ugv/graph_data_demo.py)을 팀 스키마(ResourceStatus / LocalResponse / Observation)로 연결한다.
mock의 random ACCEPT/REJECT 대신, 실제 경로탐색 결과(도달성·ETA·막힌 도로)를 사용한다.
PX4는 쓰지 않는다(SimDriver조차 불필요 — 판단은 그래프만으로 가능).
"""

from __future__ import annotations

import math

import config
from interfaces.schema import ResourceStatus, LocalResponse, Observation

from ugv.road_graph import RoadGraph
from ugv.graph_gpkg import NODES, ROADS   # 실제 도로망 (이전: graph_data_demo 시연용 6노드)
from ugv import config as ugv_config
from ugv.geo import distance_m

_graph: RoadGraph | None = None


def _g() -> RoadGraph:
    global _graph
    if _graph is None:
        _graph = RoadGraph(NODES, ROADS)
    return _graph


def _nearest_node(lat: float, lon: float) -> tuple[str | None, float]:
    """목표 좌표에 가장 가까운 그래프 노드 id 와 거리(m).

    거리는 미터 단위(ugv.geo.distance_m)로 비교한다. 위경도 차이 제곱합은
    위도 38°에서 경도 거리를 약 1.6배 과대평가해 엉뚱한 노드를 고를 수 있다.
    """
    best, best_d = None, float("inf")
    for n in NODES:
        d = distance_m((lat, lon), (n["lat"], n["lon"]))
        if d < best_d:
            best, best_d = n["node_id"], d
    return best, best_d


def _home_node(resource_id: str) -> str:
    for r in ugv_config.RESOURCES:
        if r["resource_id"] == resource_id:
            return r["home_node"]
    return "A"


# ---------------------------------------------------------------------------
# 공개 함수 (ugv_connector 가 integrated 모드에서 호출)
# ---------------------------------------------------------------------------

def get_ugv_status(base: str) -> list:
    g = _g()
    statuses = []
    for r in ugv_config.RESOURCES:
        if r["base"] != base:
            continue
        node = g.node(r["home_node"])
        # 상태 조회 시점에는 목적지가 정해지지 않았다 → ETA 는 evaluate 에서 산출
        # (이전 동작: 고정 노드 F1 기준 ETA 를 상태로 보고 → 실제 화재 위치와 무관한 값)
        statuses.append(
            ResourceStatus(
                resource_id=r["resource_id"],
                resource_type=r["resource_type"],
                base=base,
                location_lat=node.lat,
                location_lon=node.lon,
                state="READY",
                capability={
                    "eta_sec": None,
                    "road_blocked": None,
                    "reachable": None,
                },
            )
        )
    return statuses


def send_ugv_command(task_id: str, assignment, force_response=None, force_reason=None) -> LocalResponse:
    if force_response is not None:
        return LocalResponse(resource_id=assignment.resource_id, response=force_response, reason=force_reason)

    g = _g()
    start = _home_node(assignment.resource_id)
    goal, snap_m = _nearest_node(assignment.target_lat, assignment.target_lon)
    limit = getattr(config, "UGV_TARGET_SNAP_M", 2000.0)
    if goal is None or snap_m > limit:
        # 목표 근처에 도로 노드가 없다 → 임의 노드(F1)로 대체하지 않고 거절 (fail-closed)
        return LocalResponse(resource_id=assignment.resource_id, response="REJECT",
                             reason="TARGET_UNREACHABLE",
                             evidence={"target_node": goal, "snap_m": snap_m, "snap_limit_m": limit})
    # goal == start 는 '이미 목표 노드에 있음'이므로 그대로 둔다 (ETA 0)
    route = g.find_route(start, goal)

    evidence = {"start_node": start, "target_node": goal, "snap_m": round(snap_m, 1),
                "path_len": len(route.path) if route.path else None}
    if route.reachable:
        return LocalResponse(resource_id=assignment.resource_id, response="ACCEPT",
                             evidence={**evidence, "eta_sec": route.eta_s})
    if route.blocked_road_id is not None:
        return LocalResponse(resource_id=assignment.resource_id, response="REJECT", reason="ROAD_BLOCKED",
                             evidence={**evidence, "blocked_road_id": route.blocked_road_id})
    return LocalResponse(resource_id=assignment.resource_id, response="REJECT", reason="TARGET_UNREACHABLE",
                         evidence=evidence)


def get_ugv_observation(resource_id: str, sim_time_s: float, task_id: str = None, decision_id: str = None) -> Observation:
    # 도로 상태 관측. 목적지별 ETA·경로는 send_ugv_command(evaluate) 에서만 산출한다.
    # (이전 동작: 고정 노드 F1 까지 경로 → 실제 도로망에는 F1 이 없다)
    g = _g()
    node = g.node(_home_node(resource_id))
    blocked = sum(1 for r in ROADS if g.get_road(r["road_id"]).blocked)
    return Observation(
        resource_id=resource_id,
        location_lat=node.lat,
        location_lon=node.lon,
        simulation_time_s=sim_time_s,
        observation_type="ROAD_STATUS",
        value={"node_id": node.node_id, "blocked_roads": blocked},
        task_id=task_id,
        decision_id=decision_id,
    )


# ---------------------------------------------------------------------------
# 시연 헬퍼 (run_integrated.py 가 UGV 실제 로직을 직접 보여줄 때 사용)
# ---------------------------------------------------------------------------

def demo() -> list[str]:
    """Dijkstra + 도로차단 재탐색을 실제 도로망에서 보여준다. 로그 라인 리스트 반환.

    거점 A(원통119) → B(기린119) 최단경로 위 도로를 하나씩 막아 보고
    ① 우회로가 있는 도로  ② 막히면 도달 불가인 도로(산간 단일 도로) 를 하나씩 보여준다.
    """
    g = _g()
    lines = []

    def fmt(r):
        return f"ETA={r.eta_s / 60:.1f}분  경유노드 {len(r.path)}개"

    base = g.find_route("A", "B")
    lines.append(f"  A→B 정상:  도달={base.reachable}  {fmt(base) if base.reachable else ''}")
    if not base.reachable:
        return lines

    shown_detour = shown_cut = False
    for a, b in zip(base.path, base.path[1:]):
        road = _road_between(g, a, b)
        g.set_blocked(road, True)
        r = g.find_route("A", "B")
        g.set_blocked(road, False)
        if r.reachable and r.eta_s - base.eta_s >= 60 and not shown_detour:   # 상·하행 병행 링크(+0분)는 우회로로 보지 않음
            lines.append(f"  도로 {road} 차단: 우회 성공  {fmt(r)}  (+{(r.eta_s - base.eta_s) / 60:.1f}분)")
            shown_detour = True
        elif not r.reachable and not shown_cut:
            lines.append(f"  도로 {road} 차단: 도달 불가  원인도로={r.blocked_road_id}  (우회로 없는 단일 도로)")
            shown_cut = True
        if shown_detour and shown_cut:
            break
    return lines


def _road_between(g: RoadGraph, a: str, b: str) -> str:
    """인접 노드 a-b 를 잇는 도로 중 가장 빠른 것의 id."""
    return min((road for nid, road in g.neighbors(a) if nid == b), key=lambda r: r.cost_s).road_id


def reset():
    global _graph
    _graph = None
