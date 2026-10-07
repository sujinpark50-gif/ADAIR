# -*- coding: utf-8 -*-
"""
orchestrator/map_evaluation.py
==============================
인지 지도 평가 — 테두리 재현율 (사용자 결정 2026-10-07).

  테두리 재현율 = |진짜 불 테두리 칸 ∩ 총괄이 관측으로 불 확인한 칸| / |진짜 불 테두리 칸|

- 진짜 불 테두리 칸: 평가 시각에 BURNING 이고 8방향 이웃 중 UNBURNED 가 있는 칸 (드론 모의 센서의 테두리와 같은 정의).
- 총괄이 아는 불: kb.fire_states 의 CONFIRMED (관측으로 확인). 신고·LLM 가설·BURNED 는 넣지 않는다.
- 평가 범위는 지도 전체 (불타는 칸끼리만 비교하므로 범위를 따로 정하지 않는다).
- 정답을 읽는 읽기 전용 계산이다. 결과를 관측 계획 입력으로 넘기지 않는다.
- 진짜 테두리가 없으면(분모 0) 재현율은 None (N/A).
"""

from typing import Dict, List, Optional

from .observation import parse_cell_key


def truth_edge(fire_cells: List[dict], map_ids: Optional[set] = None) -> set:
    burning = {c["cell_id"] for c in fire_cells if c.get("fire_state") == "BURNING"}
    burned = {c["cell_id"] for c in fire_cells if c.get("fire_state") == "BURNED"}
    edge = set()
    for cid in burning:
        p = parse_cell_key(cid)
        if p is None:
            continue
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if not (dc or dr):
                    continue
                n = f"{p[0] + dc}_{p[1] + dr}"
                if n in burning or n in burned or (map_ids is not None and n not in map_ids):
                    continue
                edge.add(cid)
                break
            if cid in edge:
                break
    return edge


def edge_recall(snapshot, fire_states: Dict[str, dict], map_ids: Optional[set] = None) -> dict:
    edge = truth_edge(snapshot.fire_cells, map_ids)
    known = {c for c, f in fire_states.items() if f.get("status") == "CONFIRMED"}
    hit = edge & known
    return {"metric": "EDGE_RECALL", "simulation_time_s": snapshot.simulation_time_s,
            "run_id": snapshot.run_id, "state_version": snapshot.state_version,
            "truth_edge_cells": len(edge), "known_burning_cells": len(known), "edge_cells_found": len(hit),
            "edge_recall_pct": None if not edge else round(100.0 * len(hit) / len(edge), 1),
            "definition": "진짜 테두리(BURNING 이며 8방향 이웃에 UNBURNED) 중 관측으로 불 확인한 칸의 비율",
            "used_for_planning": False}
