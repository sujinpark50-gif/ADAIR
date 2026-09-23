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
from ugv.graph_data_demo import NODES, ROADS
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
                             reason="TARGET_UNREACHABLE")
    # goal == start 는 '이미 목표 노드에 있음'이므로 그대로 둔다 (ETA 0)
    route = g.find_route(start, goal)

    if route.reachable:
        return LocalResponse(resource_id=assignment.resource_id, response="ACCEPT")
    if route.blocked_road_id is not None:
        return LocalResponse(resource_id=assignment.resource_id, response="REJECT", reason="ROAD_BLOCKED")
    return LocalResponse(resource_id=assignment.resource_id, response="REJECT", reason="TARGET_UNREACHABLE")


def get_ugv_observation(resource_id: str, timestamp: float, task_id: str = None, decision_id: str = None) -> Observation:
    g = _g()
    node = g.node(_home_node(resource_id))
    route = g.find_route(_home_node(resource_id), "F1")
    return Observation(
        resource_id=resource_id,
        location_lat=node.lat,
        location_lon=node.lon,
        timestamp=timestamp,
        observation_type="ROAD_STATUS",
        value={"eta_sec": route.eta_s, "path": route.path, "reachable": route.reachable},
        task_id=task_id,
        decision_id=decision_id,
    )


# ---------------------------------------------------------------------------
# 시연 헬퍼 (run_integrated.py 가 UGV 실제 로직을 직접 보여줄 때 사용)
# ---------------------------------------------------------------------------

def demo() -> list[str]:
    """박유홍님 Dijkstra + 도로차단 재탐색을 눈으로 보여준다. 로그 라인 리스트 반환."""
    g = _g()
    lines = []
    r1 = g.find_route("A", "F1")
    lines.append(f"  A→F1 정상:  도달={r1.reachable}  ETA={r1.eta_s}s  경로={'→'.join(r1.path or [])}")

    g.set_blocked("E3", True)   # 최단 도로(J1-F1) 차단
    r2 = g.find_route("A", "F1")
    if r2.reachable:
        lines.append(f"  E3 차단 후: 우회 성공  ETA={r2.eta_s}s  경로={'→'.join(r2.path or [])}")
    else:
        lines.append(f"  E3 차단 후: 도달 불가  막힌 도로={r2.blocked_road_id}")

    # 우회로까지 모두 차단 → 도달 불가 + 원인 도로 역추적
    g.set_blocked("E2", True)
    g.set_blocked("E4", True)
    r3 = g.find_route("A", "F1")
    lines.append(f"  E2·E4까지 차단: 도달={r3.reachable}  원인도로={r3.blocked_road_id}")

    for rid in ("E3", "E2", "E4"):
        g.set_blocked(rid, False)
    return lines


def reset():
    global _graph
    _graph = None
