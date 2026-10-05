# ugv/road_status.py — 도로 환경(차단·혼잡) 상태
#
# 도로 환경은 UGV 쪽 것이다. 환경 모듈(화재 CA)과 별개이며, 화재는 도로를 막지 않는다 (팀 결정).
# 도로 환경을 바꾸는 길은 셋이다. 차단은 출처별로 따로 기록하고, 출처가 하나라도 있으면 막힌다.
#   scenario : 시나리오 파일의 시간대 (ugv/scenario.py) — 기본
#   manual   : API 로 직접 (도로·노드 하나씩)
#   cells    : API 로 격자 칸 묶음을 넘겨 그 칸을 지나는 도로를 한꺼번에 차단 (구역 통제용)
#              — 예전 '화재 셀 → 도로 차단' 기능을 이름만 바꿔 남긴 것
# 노드 차단 = 그 노드에 닿은 도로 전부 차단 (출처 태그 "<출처>-node:<노드id>" 로 도로 차단과 구분).
#
# 혼잡은 도로별 통과시간 배율(1.0 = 정상). 시나리오가 건드린 도로만 시나리오가 되돌린다.

import random

from .road_graph import RoadGraph


class RoadStatus:
    def __init__(self, graph: RoadGraph, road_cells: dict[str, list[list[int]]]):
        self.graph = graph
        self.road_cells = road_cells
        self._cell_blocked: set[str] = set()     # 칸 묶음으로 막은 도로 (다음 호출 때 교체)
        self._by: dict[str, set[str]] = {}       # road_id → 차단 출처 태그
        self.closed_cells = 0

    # --- 차단 (출처별) ------------------------------------------------------

    def set_blocked(self, road_id: str, blocked: bool, source: str) -> None:
        self.graph.get_road(road_id)             # 없는 도로면 KeyError
        srcs = self._by.setdefault(road_id, set())
        (srcs.add if blocked else srcs.discard)(source)
        self.graph.set_blocked(road_id, bool(srcs))

    def incident_roads(self, node_id: str) -> list[str]:
        self.graph.node(node_id)                 # 없는 노드면 KeyError
        return sorted({road.road_id for _, road in self.graph.neighbors(node_id)})

    def set_node_blocked(self, node_id: str, blocked: bool, source: str) -> list[str]:
        """노드를 경유 불가로. 닿은 도로를 모두 막는다. 막은/푼 도로 id 를 돌려준다."""
        roads = self.incident_roads(node_id)
        for rid in roads:
            self.set_blocked(rid, blocked, f"{source}-node:{node_id}")
        return roads

    def clear_source(self, prefix: str) -> None:
        """출처 태그가 prefix 로 시작하는 차단을 모두 푼다 ("scenario" 면 scenario-node:* 도 포함)."""
        for rid, srcs in list(self._by.items()):
            for s in [s for s in srcs if s == prefix or s.startswith(prefix + "-")]:
                self.set_blocked(rid, False, s)

    def blocked_by(self) -> dict[str, list[str]]:
        return {rid: sorted(srcs) for rid, srcs in sorted(self._by.items()) if srcs}

    def blocked_nodes(self) -> dict[str, list[str]]:
        """경유 불가 노드 → 출처."""
        out: dict[str, set[str]] = {}
        for srcs in self._by.values():
            for s in srcs:
                if "-node:" in s:
                    src, nid = s.split("-node:", 1)
                    out.setdefault(nid, set()).add(src)
        return {n: sorted(v) for n, v in sorted(out.items())}

    # --- 구역(격자 칸) 차단 ---------------------------------------------------

    def apply_cells(self, cells: list[tuple[int, int]], margin: int = 1) -> dict:
        """격자 칸 묶음(스냅샷)을 받아 그 칸(과 margin 칸 이웃)을 지나는 도로를 막는다.
        이전 호출로 막은 도로는 풀고 새로 계산한다. 빈 목록이면 전부 해제."""
        near = {(x + dx, y + dy) for x, y in cells
                for dx in range(-margin, margin + 1) for dy in range(-margin, margin + 1)}
        now = {rid for rid, rc in self.road_cells.items() if any((x, y) in near for x, y in rc)}
        for rid in self._cell_blocked - now:
            self.set_blocked(rid, False, "cells")
        for rid in now - self._cell_blocked:
            self.set_blocked(rid, True, "cells")
        added, cleared = sorted(now - self._cell_blocked), sorted(self._cell_blocked - now)
        self._cell_blocked, self.closed_cells = now, len(cells)
        return {"cells": len(cells), "blocked_roads": len(now), "newly_blocked": added, "cleared": cleared}

    # --- 혼잡 ---------------------------------------------------------------

    def apply_congestion(self, preset: str = "none", seed: int = 0, ratio: float = 0.2,
                         min_factor: float = 1.5, max_factor: float = 3.0,
                         roads: dict[str, float] | None = None) -> dict:
        """preset: none(전부 1.0) | random(시드 고정, 도로의 ratio 만큼 min~max 배 느리게).
        roads 를 주면 그 도로만 지정 배율로 덮어쓴다."""
        ids = sorted(self.road_cells)
        for rid in ids:
            self.graph.set_congestion(rid, 1.0)
        if preset == "random":
            rng = random.Random(seed)
            for rid in rng.sample(ids, int(len(ids) * ratio)):
                self.graph.set_congestion(rid, round(rng.uniform(min_factor, max_factor), 2))
        elif preset != "none":
            raise ValueError(f"알 수 없는 preset: {preset} (none | random)")
        for rid, f in (roads or {}).items():
            self.graph.set_congestion(rid, f)
        return {"preset": preset, "congested_roads": len(self.congested())}

    # --- 조회 ---------------------------------------------------------------

    def blocked(self) -> list[str]:
        return sorted(rid for rid in self.road_cells if self.graph.get_road(rid).blocked)

    def congested(self) -> dict[str, float]:
        return {rid: self.graph.get_road(rid).congestion for rid in self.road_cells
                if self.graph.get_road(rid).congestion != 1.0}
