# -*- coding: utf-8 -*-
"""
orchestrator/map_evaluation.py
==============================
인지 지도 평가 — 테두리 재현율 (사용자 결정 2026-10-07).

  테두리 재현율 = |진짜 불 테두리 칸 ∩ 총괄이 관측으로 불 확인한 칸| / |진짜 불 테두리 칸|

- 진짜 불 테두리 칸: 평가 시각에 BURNING 이고 8방향 이웃 중 UNBURNED 가 있는 칸 (드론 모의 센서의 테두리와 같은 정의).
- 총괄이 아는 불: 인지 지도(belief_layers)의 BURNING (관측으로 확인, 다 타고 꺼짐 추정 제외).
  신고·LLM 가설·BURNED·PRESUMED_BURNED 는 넣지 않는다.
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


def _pct(hit, total):
    return None if not total else round(100.0 * hit / total, 1)


def edge_recall(snapshot, layers: Dict[str, dict], map_ids: Optional[set] = None,
                ever_observed_fire: Optional[set] = None) -> dict:
    """세 점수 (사용자 결정 2026-10-07 — 테두리만 보면 불이 커질수록 실패처럼 보인다):
      ① 테두리 재현율  : 진짜 테두리 칸 중 지금 '불타는 중'으로 아는 칸
      ② 불타는 곳 발견율: 진짜 불타는 칸 중 지금 '불타는 중'으로 아는 칸
      ③ 화재 영역 발견율: 진짜 불타는 칸+다 탄 칸 중 한 번이라도 관측으로 불·탄 곳을 확인한 칸 (누적, 추정 제외)
    layers = observation_planner.belief_layers(...), ever_observed_fire = 관측으로 불·탄 곳을 본 적 있는 칸"""
    edge = truth_edge(snapshot.fire_cells, map_ids)
    burning = {c["cell_id"] for c in snapshot.fire_cells if c.get("fire_state") == "BURNING"}
    area = {c["cell_id"] for c in snapshot.fire_cells if c.get("fire_state") in ("BURNING", "BURNED")}
    known = {c for c, f in layers.items() if f.get("state") == "BURNING"}
    ever = set(ever_observed_fire or ())
    hit = edge & known
    return {"metric": "MAP_DISCOVERY", "simulation_time_s": snapshot.simulation_time_s,
            "run_id": snapshot.run_id, "state_version": snapshot.state_version,
            "truth_edge_cells": len(edge), "known_burning_cells": len(known), "edge_cells_found": len(hit),
            "edge_recall_pct": _pct(len(hit), len(edge)),
            "truth_burning_cells": len(burning), "burning_found": len(burning & known),
            "burning_found_pct": _pct(len(burning & known), len(burning)),
            "truth_fire_area_cells": len(area), "area_found": len(area & ever),
            "fire_area_found_pct": _pct(len(area & ever), len(area)),
            "definition": {"edge_recall_pct": "진짜 테두리 중 지금 불타는 중으로 아는 칸",
                           "burning_found_pct": "진짜 불타는 칸 중 지금 불타는 중으로 아는 칸",
                           "fire_area_found_pct": "진짜 불타는+다 탄 칸 중 관측으로 한 번이라도 불·탄 곳을 확인한 칸 (누적)"},
            "used_for_planning": False}
