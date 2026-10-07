# -*- coding: utf-8 -*-
"""
orchestrator/observation_planner.py
===================================
관측 계획 (사용자 결정 2026-10-07, docs/common/총괄_관측계획_개편_계획서_완성본.md).

신고 → 드론이 불 테두리를 따라 날며 불 좌표 보고 → 총괄이 아는 사실(불 좌표·관측 시각·기상·지형·자원)로
LLM 에 묻는다 → LLM 이 '다음에 불이 있을 것 같은 구역(예상 지역)'과 '누가 어느 구역을 볼지'를 고른다 → 반복.

이 모듈이 하는 일 (모두 결정론, 정답을 보지 않는다)
- 인지 지도 요약: 칸별 믿음(kb.fire_states)을 미관측/불 없음/일부만 봄/불 확인/탄 곳/신고로 나눈다.
- 후보 구역: 알려진 불(확인·신고)에서 OBS_CANDIDATE_RADIUS_M 안의 칸을 OBS_BLOCK_CELLS×OBS_BLOCK_CELLS 구역으로 묶는다.
  LLM 은 좌표를 만들지 않고 구역 ID 만 고른다. 구역의 이동 목표 칸은 코드가 정한다 (구역 가운데 칸).
- LLM 입력·출력 계약, 출력 검증, 규칙 대체.

입력 해시에는 시뮬레이션 시각을 넣지 않는다 — 환경 시계가 흐른 것만으로 다시 계획하지 않는다 (계획서 §6).
"""

import math
from typing import Dict, List, Optional, Tuple

from . import config
from .ledger import content_hash
from .observation import parse_cell_key

PURPOSES = ("INITIAL_REPORT", "BOUNDARY_CHECK", "PREDICTED_SPREAD", "RECHECK")
_M_PER_DEG_LAT = 111_320.0


def _xy(lat0, lon0, lat, lon):
    return ((lon - lon0) * _M_PER_DEG_LAT * math.cos(math.radians(lat0)), (lat - lat0) * _M_PER_DEG_LAT)


def _bearing(lat1, lon1, lat2, lon2) -> float:
    p1, p2, dl = math.radians(lat1), math.radians(lat2), math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _dist_m(lat1, lon1, lat2, lon2) -> float:
    return math.hypot(*_xy(lat1, lon1, lat2, lon2))


class MapIndex:
    """정적 지도 칸 색인 ('{col}_{row}' 형식 칸만). 불·위험 같은 정답은 들어 있지 않다."""

    def __init__(self, map_cells):
        self.by_id, self.by_cr = {}, {}
        sizes = []
        for c in map_cells:
            p = parse_cell_key(c.get("cell_id"))
            if p is None or c.get("lat") is None or c.get("lon") is None:
                continue
            self.by_id[c["cell_id"]] = c
            self.by_cr[p] = c
            if c.get("cell_size_m"):
                sizes.append(c["cell_size_m"])
        self.cell_size_m = max(sizes) if sizes else None

    @staticmethod
    def block_id(cid: str) -> Optional[str]:
        p = parse_cell_key(cid)
        if p is None:
            return None
        b = config.OBS_BLOCK_CELLS
        return f"B{p[0] // b}_{p[1] // b}"


# ---------------------------------------------------------------------------
# 인지 지도 요약
# ---------------------------------------------------------------------------
BELIEF_LABEL = {"REPORTED": "REPORTED", "CONFIRMED": "BURNING", "CONFIRMED_BURNED": "BURNED",
                "OBSERVED_CLEAR": "CLEAR", "NOT_DETECTED_PARTIAL": "PARTIAL"}


def belief_layers(fire_states: Dict[str, dict]) -> Dict[str, dict]:
    """칸 → {state, sim_time_s, observation_id, source}. 미관측 칸은 들어 있지 않다 (불 없음으로 보지 않는다)."""
    out = {}
    for cid, f in fire_states.items():
        out[cid] = {"state": BELIEF_LABEL.get(f.get("status"), f.get("status")), "sim_time_s": f.get("sim_time_s"),
                    "observation_id": f.get("observation_id"), "source": f.get("source")}
    return out


def known_edge(idx: MapIndex, belief: Dict[str, dict]):
    """총괄이 아는 테두리 (사용자 결정 2026-10-07: 테두리를 보고 예측하는 것이 핵심).
    known_edge: 관측으로 불 확인(BURNING)한 칸 중 8방향 이웃에 불 확인·탄 곳이 아닌 칸(미관측·불 없음·일부만 봄)이 있는 칸.
    frontier : 아직 아무도 보지 않은 칸 중 8방향 이웃에 불 확인 칸이 있는 칸 — 번졌는지 먼저 볼 후보.
    정답이 아니라 아는 세계(belief)로만 계산한다."""
    burning = {c for c, b in belief.items() if b["state"] == "BURNING" and c in idx.by_id}
    edge, frontier = set(), set()
    for cid in burning:
        col, row = parse_cell_key(cid)
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                n = idx.by_cr.get((col + dc, row + dr))
                if not (dc or dr) or n is None:
                    continue
                st = belief.get(n["cell_id"])
                if st is None or st["state"] not in ("BURNING", "BURNED"):
                    edge.add(cid)
                if st is None:
                    frontier.add(n["cell_id"])
    return edge, frontier


def build_input(*, idx: MapIndex, fire_states: Dict[str, dict], reports: List[dict], weather: dict,
                available: List[dict], busy: List[dict], rejections: List[dict], sim_time_s: float,
                run_id: str) -> dict:
    """LLM·규칙 공통 계획 입력. available/busy 는 자원 요약, reports 는 [{report_id, cell_id, sim_time_s}].
    전체 격자·관측 원문은 넣지 않는다. 결과에 input_hash 를 넣는다 (시각 제외)."""
    belief = belief_layers(fire_states)
    edge, frontier = known_edge(idx, belief)
    anchors = [cid for cid, b in belief.items() if b["state"] in ("BURNING", "REPORTED") and cid in idx.by_id]
    blocks: Dict[str, dict] = {}         # LLM 이 고를 수 있는 구역
    context: Dict[str, dict] = {}        # 참고용 (고를 수 없음)
    groups: Dict[str, dict] = {}
    if anchors and idx.cell_size_m:
        size = idx.cell_size_m
        r = int(math.ceil(config.OBS_CANDIDATE_RADIUS_M / size))
        offsets = [(dc, dr, math.hypot(dc, dr) * size) for dc in range(-r, r + 1) for dr in range(-r, r + 1)
                   if math.hypot(dc, dr) * size <= config.OBS_CANDIDATE_RADIUS_M]
        near: Dict[str, float] = {}
        for a in anchors:
            ac, ar = parse_cell_key(a)
            for dc, dr, d in offsets:
                c = idx.by_cr.get((ac + dc, ar + dr))
                if c is not None and d < near.get(c["cell_id"], math.inf):
                    near[c["cell_id"]] = d
        lat_c = sum(idx.by_id[a]["lat"] for a in anchors) / len(anchors)
        lon_c = sum(idx.by_id[a]["lon"] for a in anchors) / len(anchors)
        fire_blocks = {MapIndex.block_id(a) for a in anchors if belief[a]["state"] == "BURNING"}
        for cid, d in near.items():
            bid = MapIndex.block_id(cid)
            b = groups.setdefault(bid, {"block_id": bid, "cells": [], "dist": math.inf})
            b["cells"].append(cid)
            b["dist"] = min(b["dist"], d)
        rows = {}
        for bid, b in groups.items():
            counts = {"UNOBSERVED": 0, "CLEAR": 0, "PARTIAL": 0, "BURNING": 0, "BURNED": 0, "REPORTED": 0}
            times, last_obs = [], None
            n_edge = sum(1 for cid in b["cells"] if cid in edge)
            n_front = sum(1 for cid in b["cells"] if cid in frontier)
            for cid in b["cells"]:
                st = belief.get(cid)
                counts[st["state"] if st else "UNOBSERVED"] = counts.get(st["state"] if st else "UNOBSERVED", 0) + 1
                if st and st["state"] != "REPORTED" and st["sim_time_s"] is not None:
                    times.append(st["sim_time_s"])
                    if last_obs is None or st["sim_time_s"] >= last_obs[0]:
                        last_obs = (st["sim_time_s"], st["observation_id"])
            bc, br = (int(x) for x in bid[1:].split("_"))
            k = config.OBS_BLOCK_CELLS
            centre = idx.by_cr.get((bc * k + k // 2, br * k + k // 2))
            if centre is None or centre.get("ground_amsl_m") is None:
                cands = [idx.by_id[c] for c in b["cells"] if idx.by_id[c].get("ground_amsl_m") is not None]
                ccx, ccy = bc * k + (k - 1) / 2, br * k + (k - 1) / 2
                centre = min(cands, key=lambda c: (math.hypot(parse_cell_key(c["cell_id"])[0] - ccx,
                                                              parse_cell_key(c["cell_id"])[1] - ccy), c["cell_id"]),
                             default=None)
            grounds = [idx.by_id[c]["ground_amsl_m"] for c in b["cells"] if idx.by_id[c].get("ground_amsl_m") is not None]
            neighbour_fire = any(f"B{bc + dc}_{br + dr}" in fire_blocks for dc in (-1, 0, 1) for dr in (-1, 0, 1))
            selectable = centre is not None and (counts["UNOBSERVED"] + counts["CLEAR"] + counts["PARTIAL"]
                                                 + counts["REPORTED"]) > 0
            row = {"block_id": bid, "selectable": selectable,
                   "target_cell": centre["cell_id"] if centre else None,
                   "cells_in_radius": len(b["cells"]), "counts": counts,
                   "known_edge_cells": n_edge, "unobserved_next_to_fire": n_front,
                   "dist_to_known_fire_m": round(b["dist"]),
                   "bearing_from_fire_centre_deg": None if not centre else round(
                       _bearing(lat_c, lon_c, centre["lat"], centre["lon"])),
                   "mean_ground_m": round(sum(grounds) / len(grounds), 1) if grounds else None,
                   "touches_known_fire_block": neighbour_fire,
                   "last_observed_sim_s": last_obs[0] if last_obs else None,
                   "oldest_observed_sim_s": min(times) if times else None,
                   "last_observation_id": last_obs[1] if last_obs else None,
                   "in_progress_by": sorted(x["resource_id"] for x in busy if x.get("block_id") == bid),
                   "distance_km_from": {}}
            if centre:
                for res in available:
                    if res.get("lat") is not None:
                        row["distance_km_from"][res["resource_id"]] = round(
                            _dist_m(res["lat"], res["lon"], centre["lat"], centre["lon"]) / 1000, 2)
            if config.OBS_STALE_S is not None and row["oldest_observed_sim_s"] is not None:
                row["has_stale_observation"] = sim_time_s - row["oldest_observed_sim_s"] > config.OBS_STALE_S
            rows[bid] = row
        # LLM 이 고를 수 있는 구역만 blocks 에 남긴다 (사용자 결정 2026-10-07: 안 되는 것은 미리 뺀다).
        #   빼는 것: 볼 것이 없는 구역(전부 불 확인·탄 곳), 목표 칸을 정할 수 없는 구역, 다른 자원이 보러 가는 구역,
        #   보낼 수 있는 자원이 하나도 없는 구역(방금 거절당한 조합만 남은 경우).
        #   자원별로는 allowed_resources 에 방금 거절당한 자원을 넣지 않는다.
        #   빠진 구역 중 불·진행 중 정보가 있는 곳은 context_blocks(참고용, 고를 수 없음)로 준다.
        rejected = {(r["resource_id"], r["block_id"]) for r in rejections}
        avail_ids = [r["resource_id"] for r in available]
        for bid, row in rows.items():
            allowed = [rid for rid in avail_ids if (rid, bid) not in rejected]
            if row.pop("selectable") and not row["in_progress_by"] and allowed:
                row["allowed_resources"] = allowed
                row["distance_km_from"] = {k: v for k, v in row["distance_km_from"].items() if k in allowed}
                row.pop("in_progress_by")
                blocks[bid] = row
            elif row["counts"]["BURNING"] or row["counts"]["BURNED"] or row["in_progress_by"]:
                context[bid] = {k: row[k] for k in ("block_id", "counts", "known_edge_cells", "dist_to_known_fire_m",
                                                    "bearing_from_fire_centre_deg", "mean_ground_m",
                                                    "last_observed_sim_s", "last_observation_id", "in_progress_by")}
    fire_summary = {
        "burning_cells": sum(1 for b in belief.values() if b["state"] == "BURNING"),
        "burned_cells": sum(1 for b in belief.values() if b["state"] == "BURNED"),
        "observed_clear_cells": sum(1 for b in belief.values() if b["state"] == "CLEAR"),
        "partial_cells": sum(1 for b in belief.values() if b["state"] == "PARTIAL"),
        "reported_cells": sum(1 for b in belief.values() if b["state"] == "REPORTED"),
        "known_edge_cells": len(edge), "unobserved_next_to_fire_cells": len(frontier),
        "unobserved_note": "목록에 없는 칸과 counts.UNOBSERVED 는 아직 아무도 보지 않은 칸이다 (불 없음이 아니다)",
    }
    inp = {
        "run_id": run_id, "simulation_time_s": sim_time_s,
        "demo_end_sim_s": config.OBS_DEMO_END_SIM_S,
        "block_size_m": None if not idx.cell_size_m else idx.cell_size_m * config.OBS_BLOCK_CELLS,
        "candidate_radius_m": config.OBS_CANDIDATE_RADIUS_M,
        "reports": reports, "weather": weather, "fire_summary": fire_summary,
        "resources_available": available, "resources_busy": busy,
        "blocks": [blocks[k] for k in sorted(blocks)],
        "context_blocks": [context[k] for k in sorted(context)],
        "allowed_purposes": list(PURPOSES),
    }
    hashed = {k: v for k, v in inp.items() if k != "simulation_time_s"}
    inp["input_hash"] = content_hash(hashed)[:16]
    return inp


def known_ids(inp: dict) -> set:
    rows = inp["blocks"] + inp["context_blocks"]
    ids = {b["block_id"] for b in rows}
    ids |= {b["last_observation_id"] for b in rows if b.get("last_observation_id")}
    ids |= {r["report_id"] for r in inp["reports"]}
    ids |= {r["resource_id"] for r in inp["resources_available"] + inp["resources_busy"]}
    ids |= {w.get("key") for w in (inp["weather"].get("items") or {}).values() if isinstance(w, dict) and w.get("key")}
    return ids


# ---------------------------------------------------------------------------
# LLM 계약
# ---------------------------------------------------------------------------
SYSTEM = (
    "너는 산불 관측을 지휘하는 총괄 오케스트레이터의 계획 보조다. 스타크래프트 정찰처럼, 드론(UAV)과 지상 로봇(UGV)의 "
    "부분 관측을 모아 지금 불이 어디까지 타고 있는지 알아내는 것이 목표다. 불을 끄는 일은 하지 않는다.\n"
    "입력은 총괄이 아는 사실뿐이다: 신고, 관측으로 확인한 불(BURNING)·탄 곳(BURNED)·불 없음(CLEAR)·일부만 봄(PARTIAL), "
    "아직 아무도 보지 않은 칸(UNOBSERVED), 기상(wind_dir_deg 는 바람이 불어오는 방향, 북 기준 시계방향 — 불은 대체로 "
    "그 반대쪽(풍하)과 오르막으로 번진다), 지형(mean_ground_m), 자원 위치와 상태.\n"
    "후보 구역(blocks)은 알려진 불 주변을 3×3 칸으로 묶은 것이다. 할 일 두 가지:\n"
    "1) predicted_blocks: 지금 불이 있을 것 같지만 아직 확인하지 않은 구역(예상 지역)을 고른다.\n"
    "2) assignments: resources_available 의 자원마다 다음에 볼 구역 하나를 고른다. 드론은 도착하면 불 테두리를 따라 "
    "140 m 를 날며 폭 52 m 를 보고, 테두리가 없으면 신고 지점 쪽으로 140 m 를 난다. UGV 는 가까운 도로에 서서 주변 "
    "450 m 사각형을 본다. 자원마다 서로 다른 구역을 고르고, 그 구역의 allowed_resources 에 있는 자원만 보낸다. "
    "blocks 는 지금 고를 수 있는 구역 전부다. context_blocks 는 불이 확인됐거나 다른 자원이 보러 가는 참고용 구역이며 "
    "assignments 와 predicted_blocks 에 쓸 수 없다.\n"
    "우선 원칙: 불은 테두리에서 번진다. known_edge_cells 는 지금 아는 불의 가장자리, unobserved_next_to_fire 는 "
    "그 바로 바깥의 아직 안 본 칸 수다. 알려진 테두리 바깥, 특히 풍하 쪽과 오르막 쪽의 미관측 칸을 우선 확인하라.\n"
    "규칙: 입력에 없는 ID·좌표·수치를 만들지 않는다. evidence_refs 에는 근거로 쓴 입력 ID(구역·관측·신고·자원)만 넣는다. "
    "rationale 은 관제 요원이 읽는 한국어 1~2문장이다. input_hash 는 입력 값을 그대로 돌려준다."
)

OUTPUT_SCHEMA = {
    "type": "object", "additionalProperties": False,
    "required": ["input_hash", "predicted_blocks", "prediction_rationale", "assignments"],
    "properties": {
        "input_hash": {"type": "string"},
        "predicted_blocks": {"type": "array", "items": {"type": "string"}},
        "prediction_rationale": {"type": "string"},
        "assignments": {"type": "array", "items": {
            "type": "object", "additionalProperties": False,
            "required": ["resource_id", "block_id", "purpose", "evidence_refs", "rationale"],
            "properties": {"resource_id": {"type": "string"}, "block_id": {"type": "string"},
                           "purpose": {"type": "string", "enum": list(PURPOSES)},
                           "evidence_refs": {"type": "array", "items": {"type": "string"}},
                           "rationale": {"type": "string"}}}},
    },
}


def _str_list(v) -> bool:
    return isinstance(v, list) and all(isinstance(x, str) for x in v)


def _text_ok(v) -> bool:
    return isinstance(v, str) and v.strip() != "" and len(v) <= config.OBS_RATIONALE_MAX_CHARS


def validate(out, inp: dict) -> List[str]:
    """위반 목록 (비면 통과). 예외를 던지지 않는다."""
    if not isinstance(out, dict) or set(out) != set(OUTPUT_SCHEMA["required"]):
        return ["SCHEMA_MISMATCH"]
    v = []
    if out["input_hash"] != inp["input_hash"]:
        v.append("INPUT_HASH_MISMATCH")
    blocks = {b["block_id"]: b for b in inp["blocks"]}
    if not _str_list(out["predicted_blocks"]):
        v.append("PREDICTED_BLOCKS_TYPE")
    else:
        bad = [b for b in out["predicted_blocks"] if b not in blocks]
        if bad:
            v.append("PREDICTED_UNKNOWN_BLOCK:" + ",".join(bad[:5]))
    if not _text_ok(out["prediction_rationale"]):
        v.append("PREDICTION_RATIONALE_INVALID")
    avail = {r["resource_id"] for r in inp["resources_available"]}
    ids = known_ids(inp)
    if not isinstance(out["assignments"], list):
        return v + ["ASSIGNMENTS_TYPE"]
    used_r, used_b = set(), set()
    for n, a in enumerate(out["assignments"]):
        if not isinstance(a, dict) or set(a) != set(OUTPUT_SCHEMA["properties"]["assignments"]["items"]["required"]):
            v.append(f"SCHEMA_MISMATCH:assignments[{n}]")
            continue
        rid, bid = a["resource_id"], a["block_id"]
        if not isinstance(rid, str) or rid not in avail:
            v.append(f"RESOURCE_NOT_AVAILABLE:{rid}")
        elif rid in used_r:
            v.append(f"RESOURCE_DUPLICATE:{rid}")
        if not isinstance(bid, str) or bid not in blocks:
            v.append(f"BLOCK_NOT_SELECTABLE:{bid}")
        elif bid in used_b:
            v.append(f"BLOCK_DUPLICATE:{bid}")
        elif isinstance(rid, str) and rid in avail and rid not in blocks[bid]["allowed_resources"]:
            v.append(f"RESOURCE_NOT_ALLOWED_FOR_BLOCK:{rid}:{bid}")
        if a["purpose"] not in PURPOSES:
            v.append(f"PURPOSE_INVALID:{n}")
        if not _str_list(a["evidence_refs"]):
            v.append(f"EVIDENCE_TYPE:{n}")
        else:
            unknown = [x for x in a["evidence_refs"] if x not in ids]
            if unknown:
                v.append(f"UNKNOWN_REFS:{n}:{','.join(unknown[:5])}")
        if not _text_ok(a["rationale"]):
            v.append(f"RATIONALE_INVALID:{n}")
        if isinstance(rid, str):
            used_r.add(rid)
        if isinstance(bid, str):
            used_b.add(bid)
    # 고를 수 있는 구역이 남아 있는데 놀리는 자원이 있으면 위반 (자원마다 다음 관측 하나 — 계획서 §5)
    missing = sorted(r for r in avail - used_r
                     if any(r in b["allowed_resources"] and k not in used_b for k, b in blocks.items()))
    if missing:
        v.append("RESOURCE_NOT_ASSIGNED:" + ",".join(missing))
    return v


# ---------------------------------------------------------------------------
# 규칙 대체 (LLM 실패·검증 실패·한도 소진). 정답을 보지 않고 입력만 쓴다.
#   테두리 바로 바깥 미관측이 많은 순 → 미관측이 있는 구역 → 미관측이 많은 순 → 알려진 불에 가까운 순 → 자원에서 가까운 순 → 구역 ID
# ---------------------------------------------------------------------------
def rule_plan(inp: dict) -> dict:
    taken, out = set(), []
    for res in sorted(inp["resources_available"], key=lambda r: r["resource_id"]):
        rid = res["resource_id"]
        cands = [b for b in inp["blocks"] if b["block_id"] not in taken and rid in b["allowed_resources"]]
        if not cands:
            continue
        b = min(cands, key=lambda b: (-b["unobserved_next_to_fire"],
                                      b["counts"]["UNOBSERVED"] == 0, -b["counts"]["UNOBSERVED"],
                                      b["dist_to_known_fire_m"], b["distance_km_from"].get(rid, math.inf),
                                      b["block_id"]))
        taken.add(b["block_id"])
        purpose = ("INITIAL_REPORT" if b["counts"]["REPORTED"] else
                   "BOUNDARY_CHECK" if b["touches_known_fire_block"] else
                   "PREDICTED_SPREAD" if b["counts"]["UNOBSERVED"] else "RECHECK")
        out.append({"resource_id": rid, "block_id": b["block_id"], "purpose": purpose,
                    "evidence_refs": [b["block_id"]],
                    "rationale": "규칙 대체: 알려진 테두리 바로 바깥 미관측이 많은 구역"})
    return {"input_hash": inp["input_hash"], "predicted_blocks": [], "prediction_rationale": "규칙 대체 (예측 없음)",
            "assignments": out}
