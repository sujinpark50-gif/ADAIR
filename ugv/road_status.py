# ugv/road_status.py — 외부 사정(화재·혼잡)을 도로 그래프 상태로 반영
#
# 환경은 "어디가 타는가"(화재 셀)만 알고, 도로 통행 가능 여부는 모른다.
# 통행 가능 여부는 차량 기준 판단이므로 UGV 가 여기서 해석한다.
#
# 화재 → 차단 규칙: 도로가 지나는 칸, 또는 그 8방향 이웃 칸이 BURNING 이면 그 도로는 차단.
#   이웃까지 보는 이유: 환경 CA 에서 도로 칸은 대부분 연료 0 이라 직접 타지 않는다
#   (도로 칸 2,297개 중 1,634개). 옆 칸이 타면 실제로도 통행이 불가능하다.
#
# 혼잡: 환경에 혼잡 정보가 없으므로 시나리오용으로 직접 준다 (고정값 또는 시드 고정 랜덤).

import random

from .road_graph import RoadGraph


class RoadStatus:
    def __init__(self, graph: RoadGraph, road_cells: dict[str, list[list[int]]]):
        self.graph = graph
        self.road_cells = road_cells
        self._fire_blocked: set[str] = set()     # 화재로 막은 도로 (다음 갱신 때 해제 대상)
        self.burning_cells = 0

    # --- 화재 ---------------------------------------------------------------

    def apply_fire(self, burning: list[tuple[int, int]]) -> dict:
        """현재 화재 셀 전체(스냅샷)를 받아 차단 상태를 다시 계산한다. 이전 화재 차단은 해제한다."""
        hot = {(x + dx, y + dy) for x, y in burning for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
        now = {rid for rid, cells in self.road_cells.items() if any((x, y) in hot for x, y in cells)}

        for rid in self._fire_blocked - now:
            self.graph.set_blocked(rid, False)
        for rid in now - self._fire_blocked:
            self.graph.set_blocked(rid, True)

        added, cleared = sorted(now - self._fire_blocked), sorted(self._fire_blocked - now)
        self._fire_blocked, self.burning_cells = now, len(burning)
        return {"burning_cells": len(burning), "blocked_roads": len(now),
                "newly_blocked": added, "cleared": cleared}

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
