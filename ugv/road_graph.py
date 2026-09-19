# ugv/road_graph.py — 도로망 그래프와 경로 탐색
# 간선은 무향, 비용은 Road.cost_s 그대로 사용 (속도·차량 상태는 고려하지 않음).
# 탐색 알고리즘은 RouteAlgorithm 으로 분리되어 있어 교체 가능.

import heapq
from typing import Protocol
from .models import Node, Road, RouteResult


class RouteAlgorithm(Protocol):
    """경로 탐색 전략. 도달 가능하면 (총비용, node_id 경로), 불가하면 None."""

    def search(
        self, graph: "RoadGraph", start: str, goal: str
    ) -> tuple[float, list[str]] | None: ...


class DijkstraAlgorithm:
    def search(
        self, graph: "RoadGraph", start: str, goal: str
    ) -> tuple[float, list[str]] | None:
        dist: dict[str, float] = {start: 0.0}
        prev: dict[str, str] = {}
        heap: list[tuple[float, str]] = [(0.0, start)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist.get(u, float("inf")):
                continue  # 이미 더 짧은 경로로 확정된 낡은 항목
            if u == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(prev[path[-1]])
                path.reverse()
                return d, path
            for v, road in graph.neighbors(u):
                nd = d + road.cost_s
                if nd == float("inf"):
                    continue  # blocked
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))
        return None


class RoadGraph:
    def __init__(
        self,
        nodes: list[dict],
        roads: list[dict],
        algorithm: RouteAlgorithm | None = None,
    ):
        self.algorithm: RouteAlgorithm = algorithm or DijkstraAlgorithm()
        self._nodes: dict[str, Node] = {}
        self._roads: dict[str, Road] = {}
        self._adj: dict[str, list[tuple[str, Road]]] = {}

        for d in nodes:
            node = Node(**d)
            if node.node_id in self._nodes:
                raise ValueError(f"중복 node_id: {node.node_id}")
            self._nodes[node.node_id] = node
            self._adj[node.node_id] = []

        for d in roads:
            road = Road(**d)
            if road.road_id in self._roads:
                raise ValueError(f"중복 road_id: {road.road_id}")
            for nid in (road.node_a, road.node_b):
                if nid not in self._nodes:
                    raise ValueError(
                        f"road {road.road_id}: 존재하지 않는 node_id {nid}"
                    )
            self._roads[road.road_id] = road
            # 무향: 같은 Road 객체를 양쪽에 등록해 상태 갱신이 양방향에 반영되게 한다
            self._adj[road.node_a].append((road.node_b, road))
            self._adj[road.node_b].append((road.node_a, road))

    # --- 조회 ---

    def node(self, node_id: str) -> Node:
        try:
            return self._nodes[node_id]
        except KeyError:
            raise KeyError(f"존재하지 않는 node_id: {node_id}") from None

    def get_road(self, road_id: str) -> Road:
        try:
            return self._roads[road_id]
        except KeyError:
            raise KeyError(f"존재하지 않는 road_id: {road_id}") from None

    def neighbors(self, node_id: str) -> list[tuple[str, Road]]:
        return self._adj[node_id]

    # --- 도로 상태 갱신 (환경모듈용) ---

    def set_blocked(self, road_id: str, blocked: bool) -> None:
        self.get_road(road_id).blocked = blocked

    def set_congestion(self, road_id: str, congestion: float) -> None:
        if congestion <= 0:
            raise ValueError(f"congestion 은 0 보다 커야 함: {congestion}")
        self.get_road(road_id).congestion = congestion

    # --- 경로 탐색 ---

    def find_route(self, start_id: str, goal_id: str) -> RouteResult:
        self.node(start_id)
        self.node(goal_id)

        found = self.algorithm.search(self, start_id, goal_id)
        if found is not None:
            eta, path = found
            return RouteResult(reachable=True, eta_s=eta, path=path)
        return RouteResult(
            reachable=False,
            eta_s=None,
            path=None,
            blocked_road_id=self._find_blocking_road(start_id, goal_id),
        )

    def _find_blocking_road(self, start_id: str, goal_id: str) -> str | None:
        """blocked 를 무시하면 도달 가능한 경우, 그 경로 위 첫 blocked 도로의 id."""
        blocked = [r for r in self._roads.values() if r.blocked]
        for r in blocked:
            r.blocked = False
        try:
            found = self.algorithm.search(self, start_id, goal_id)
        finally:
            for r in blocked:
                r.blocked = True
        if found is None:
            return None  # blocked 와 무관하게 끊긴 그래프

        _, path = found
        blocked_ids = {r.road_id for r in blocked}
        for a, b in zip(path, path[1:]):
            for nid, road in self._adj[a]:
                if nid == b and road.road_id in blocked_ids:
                    return road.road_id
        return None
