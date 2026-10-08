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


def burn_out_s(tick_s) -> Optional[float]:
    """'불타는 중' 확인 후 다 타고 꺼졌다고 볼 시간 (config.OBS_BURN_OUT_STEPS × 환경 단계 길이). 모르면 None"""
    if config.OBS_BURN_OUT_STEPS is None or not isinstance(tick_s, (int, float)) or tick_s <= 0:
        return None
    return config.OBS_BURN_OUT_STEPS * float(tick_s)


def belief_layers(fire_states: Dict[str, dict], now_s: Optional[float] = None,
                  burn_out: Optional[float] = None) -> Dict[str, dict]:
    """칸 → {state, sim_time_s, observation_id, source}. 미관측 칸은 들어 있지 않다 (불 없음으로 보지 않는다).
    불 확인(BURNING)·신고(REPORTED) 뒤 burn_out 초가 지나도록 다시 '불'로 확인되지 않은 칸은
    PRESUMED_BURNED(다 타고 꺼짐, 추정)로 둔다 — 관측 사실이 아니라 화재 모델 규칙에 따른 추정이다."""
    out = {}
    for cid, f in fire_states.items():
        st = BELIEF_LABEL.get(f.get("status"), f.get("status"))
        row = {"state": st, "sim_time_s": f.get("sim_time_s"), "observation_id": f.get("observation_id"),
               "source": f.get("source")}
        if (st in ("BURNING", "REPORTED") and burn_out is not None and now_s is not None
                and f.get("sim_time_s") is not None and now_s - f["sim_time_s"] > burn_out):
            row.update({"state": "PRESUMED_BURNED", "last_state": st,
                        "presumed_since_s": f["sim_time_s"] + burn_out, "basis": "BURN_OUT_RULE"})
        out[cid] = row
    return out


def known_edge(idx: MapIndex, belief: Dict[str, dict]):
    """총괄이 아는 테두리 (사용자 결정 2026-10-07: 테두리를 보고 예측하는 것이 핵심).
    known_edge: 관측으로 불 확인(BURNING)한 칸 중 8방향 이웃에 불 확인·탄 곳이 아닌 칸(미관측·불 없음·일부만 봄)이 있는 칸.
    frontier : 아직 아무도 보지 않은 칸 중 8방향 이웃에 불 확인 칸 또는 불이 지나간 칸(다 탄 곳·꺼짐 추정)이 있는 칸 —
               번졌는지 먼저 볼 후보. 불이 지나간 칸을 넣지 않으면 아는 불이 모두 꺼진 것으로 바뀌는 순간 '바로 바깥'
               정보가 사라져 먼 구역부터 보게 된다 (사용자 결정 2026-10-07, 실제 환경 실행에서 확인).
    정답이 아니라 아는 세계(belief)로만 계산한다."""
    burning = {c for c, b in belief.items() if b["state"] == "BURNING" and c in idx.by_id}
    passed = {c for c, b in belief.items() if b["state"] in ("BURNED", "PRESUMED_BURNED") and c in idx.by_id}
    edge, frontier = set(), set()
    for cid in passed:
        col, row = parse_cell_key(cid)
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                n = idx.by_cr.get((col + dc, row + dr))
                if (dc or dr) and n is not None and belief.get(n["cell_id"]) is None:
                    frontier.add(n["cell_id"])
    for cid in burning:
        col, row = parse_cell_key(cid)
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                n = idx.by_cr.get((col + dc, row + dr))
                if not (dc or dr) or n is None:
                    continue
                st = belief.get(n["cell_id"])
                if st is None or st["state"] not in ("BURNING", "BURNED", "PRESUMED_BURNED"):
                    edge.add(cid)
                if st is None:
                    frontier.add(n["cell_id"])
    return edge, frontier


def fire_progress(idx: MapIndex, belief: Dict[str, dict], origin_cell: Optional[str] = None) -> Optional[dict]:
    """관측으로 본 불의 진행 방향 (사용자 결정 2026-10-07). 아는 세계만 쓴다 (정답 아님).
    지난 불 = 다 탄 곳(관측)·다 타고 꺼짐(추정)·신고 칸, 지금 불 = 불타는 중(관측).
    지난 불이 없으면 '불타는 중' 칸을 확인 시각으로 나눠 (가장 이른 시각 묶음 → 가장 늦은 시각 묶음) 비교한다.
    지난 불이 없고 처음 발화 지점(origin_cell = 첫 신고 칸)을 알면 '발화 지점 → 지금 불 중심'을 쓴다 — 첫 드론이 원으로
    돌고 오면 본 칸이 모두 같은 시각이라 시각 비교로는 방향이 안 나오기 때문 (사용자 결정 2026-10-07).
    두 묶음의 중심을 잇는 방향·거리를 준다. 계산할 수 없으면 None."""
    def centre(cids):
        cs = [idx.by_id[c] for c in cids if c in idx.by_id]
        if not cs:
            return None
        return sum(c["lat"] for c in cs) / len(cs), sum(c["lon"] for c in cs) / len(cs)
    now_cells = [c for c, b in belief.items() if b["state"] == "BURNING" and c in idx.by_id]
    past_cells = [c for c, b in belief.items() if b["state"] in ("BURNED", "PRESUMED_BURNED", "REPORTED") and c in idx.by_id]
    basis = "PAST_FIRE_TO_BURNING"
    if now_cells and not past_cells and origin_cell in idx.by_id:
        past_cells, basis = [origin_cell], "IGNITION_TO_BURNING"
    if now_cells and not past_cells:
        times = sorted({belief[c]["sim_time_s"] for c in now_cells if belief[c]["sim_time_s"] is not None})
        if len(times) < 2:
            return None
        past_cells = [c for c in now_cells if belief[c]["sim_time_s"] == times[0]]
        now_cells = [c for c in now_cells if belief[c]["sim_time_s"] == times[-1]]
        basis = "EARLIEST_TO_LATEST_BURNING"
    a, b = centre(past_cells), centre(now_cells)
    if a is None or b is None:
        return None
    dist = _dist_m(a[0], a[1], b[0], b[1])
    newest = sorted(now_cells, key=lambda c: (-(belief[c]["sim_time_s"] or 0.0), c))
    blocks = []
    for c in newest:
        bid = MapIndex.block_id(c)
        if bid not in [x["block_id"] for x in blocks]:
            blocks.append({"block_id": bid, "last_burning_sim_s": belief[c]["sim_time_s"]})
    return {"basis": basis, "moved_toward_deg": None if dist < 1 else round(_bearing(a[0], a[1], b[0], b[1])),
            "moved_m": round(dist), "past_cells": len(past_cells), "burning_cells": len(now_cells),
            "newest_burning_blocks": blocks,
            "note": "관측으로 확인한 이동이다 (예측 아님). 방향은 북 기준 시계방향, 불이 '향해 간' 쪽"}


def build_input(*, idx: MapIndex, fire_states: Dict[str, dict], reports: List[dict], weather: dict,
                available: List[dict], busy: List[dict], rejections: List[dict], sim_time_s: float,
                run_id: str, burn_out: Optional[float] = None, reserve: Optional[dict] = None,
                charging: Optional[List[dict]] = None) -> dict:
    """LLM·규칙 공통 계획 입력. available/busy 는 자원 요약, reports 는 [{report_id, cell_id, sim_time_s}].
    available 에는 지금 지시서의 한 지점을 보러 가는 자원도 들어간다 (now=ON_STOP, block_id=가는 구역) — 새 회의는
    덜 끝난 지시서를 바꾸기 때문이다 (사용자 결정 2026-10-08). charging = 충전 중 드론 [{resource_id, ready_in_s}].
    전체 격자·관측 원문은 넣지 않는다. 결과에 input_hash 를 넣는다 (시각 제외)."""
    belief = belief_layers(fire_states, sim_time_s, burn_out)
    edge, frontier = known_edge(idx, belief)
    # 후보 구역의 기준 = 불이 있거나 최근까지 있던 곳. '다 타고 꺼짐(추정)'과 관측으로 본 '다 탄 곳'도 넣는다 — 빼면
    # 아는 불이 모두 꺼진 것으로 바뀌는 순간 후보가 사라져 계획이 멈춘다 (2026-10-07 실제 환경 실행에서 두 번 확인)
    anchors = [cid for cid, b in belief.items() if b["state"] in ("BURNING", "REPORTED", "PRESUMED_BURNED", "BURNED")
               and cid in idx.by_id]
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
        # 알려진 불(확인·신고)의 평균 지면고도 — 구역이 그보다 높으면 오르막 (불은 오르막으로 빨리 번진다)
        fire_ground = [idx.by_id[a]["ground_amsl_m"] for a in anchors if idx.by_id[a].get("ground_amsl_m") is not None]
        fire_ground_m = sum(fire_ground) / len(fire_ground) if fire_ground else None
        for cid, d in near.items():
            bid = MapIndex.block_id(cid)
            b = groups.setdefault(bid, {"block_id": bid, "cells": [], "dist": math.inf})
            b["cells"].append(cid)
            b["dist"] = min(b["dist"], d)
        rows = {}
        for bid, b in groups.items():
            counts = {"UNOBSERVED": 0, "CLEAR": 0, "PARTIAL": 0, "BURNING": 0, "BURNED": 0, "PRESUMED_BURNED": 0,
                      "REPORTED": 0}
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
            # 정적 지도에 도로·연료가 있으면 쓴다 (환경 /map_cells 계약에 아직 없음 — 없으면 넣지 않는다)
            roads = [bool(idx.by_id[c]["is_road"]) for c in b["cells"] if idx.by_id[c].get("is_road") is not None]
            fuels = [idx.by_id[c]["fuel_amount"] for c in b["cells"] if idx.by_id[c].get("fuel_amount") is not None]
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
                   "elev_above_known_fire_m": (round(sum(grounds) / len(grounds) - fire_ground_m)
                                               if grounds and fire_ground_m is not None else None),
                   "touches_known_fire_block": neighbour_fire,
                   "last_observed_sim_s": last_obs[0] if last_obs else None,
                   "oldest_observed_sim_s": min(times) if times else None,
                   "last_observation_id": last_obs[1] if last_obs else None,
                   "in_progress_by": sorted(x["resource_id"] for x in busy + available if x.get("block_id") == bid),
                   "distance_km_from": {}}
            if centre:
                for res in available:
                    if res.get("lat") is not None:
                        row["distance_km_from"][res["resource_id"]] = round(
                            _dist_m(res["lat"], res["lon"], centre["lat"], centre["lon"]) / 1000, 2)
            if roads:
                row["road_cells"] = sum(roads)
            if fuels:
                row["mean_fuel"] = round(sum(fuels) / len(fuels), 2)
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
            elif (row["counts"]["BURNING"] or row["counts"]["BURNED"] or row["counts"]["PRESUMED_BURNED"]
                  or row["in_progress_by"]):
                context[bid] = {k: row[k] for k in ("block_id", "counts", "known_edge_cells", "dist_to_known_fire_m",
                                                    "bearing_from_fire_centre_deg", "mean_ground_m",
                                                    "elev_above_known_fire_m",
                                                    "last_observed_sim_s", "last_observation_id", "in_progress_by")}
    # 후보 구역 상한 (OBS_MAX_CANDIDATE_BLOCKS): 진행 방향 앞쪽 → 테두리 바깥 미관측 → 미관측 → 가까운 순
    progress = fire_progress(idx, belief, reports[0]["cell_id"] if reports else None)
    ahead = None if not progress else progress.get("moved_toward_deg")

    def prio(b):
        brg = b.get("bearing_from_fire_centre_deg")
        if ahead is None or brg is None:
            tier, diff = 0, 0
        else:
            diff = abs((brg - ahead + 180) % 360 - 180)
            tier = 0 if diff <= 45 else 1 if diff <= 90 else 2
        b["ahead_of_progress"] = None if ahead is None or brg is None else tier == 0
        return (tier, -b["unobserved_next_to_fire"], -b["counts"]["UNOBSERVED"], b["dist_to_known_fire_m"], diff,
                b["block_id"])
    ordered = sorted(blocks.values(), key=prio)
    limit = config.OBS_MAX_CANDIDATE_BLOCKS
    chosen = ordered if not limit else ordered[:limit]
    for res in available:                                # 상한 때문에 고를 구역이 없어진 자원이 없게 한 개씩 보탠다
        rid = res["resource_id"]
        if not any(rid in b["allowed_resources"] for b in chosen):
            extra = next((b for b in ordered if rid in b["allowed_resources"] and b not in chosen), None)
            if extra:
                chosen.append(extra)
    deferred = len(ordered) - len(chosen)
    fire_summary = {
        "burning_cells": sum(1 for b in belief.values() if b["state"] == "BURNING"),
        "burned_cells": sum(1 for b in belief.values() if b["state"] == "BURNED"),
        "presumed_burned_out_cells": sum(1 for b in belief.values() if b["state"] == "PRESUMED_BURNED"),
        "burn_out_after_s": burn_out,
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
        "fire_progress": progress,
        "reserve_uavs": reserve,
        "resources_available": available, "resources_busy": busy, "resources_charging": list(charging or []),
        "order_max_stops": config.OBS_ORDER_MAX_STOPS,
        "blocks": chosen, "deferred_blocks": deferred,
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
    "너는 산불 관측 총괄의 계획 보조다. 목표: 드론(UAV)·지상 로봇(UGV)의 관측으로 지금 불이 어디까지 타는지 알아내기.\n"
    "입력은 총괄이 아는 사실뿐이다 (미관측 U 는 불 없음이 아니다. PX 는 관측이 아닌 추정).\n"
    "할 일 1) predicted_blocks: 불이 있을 것 같은데 아직 확인 안 한 구역.\n"
    "할 일 2) assignments = 작업 지시서. 너는 약 35분에 한 번만 불린다. res 의 자원마다 볼 구역을 순서대로 "
    "1~order_max_stops 개(block_ids). 한 구역은 한 번만, ok 에 있는 자원만. 배터리(bat)가 적은 드론에는 가까운 구역을 적게.\n"
    "우선순위: emergency(인명피해 예상) 근처 → progress 방향 앞쪽 → 테두리 바로 바깥 미관측(front) → 풍하·오르막(up+).\n"
    "불은 도로(road)를 못 넘고 연료(fuel) 없는 곳으로 안 번진다. 입력에 없는 값은 모르는 것이다.\n"
    "입력에 없는 ID·수치를 만들지 않는다. evidence_refs 는 입력 ID 만. "
    "reserve.active=false 이고 불이 그 기지 쪽으로 오면 request_reserve_uavs=true + reserve_reason, 아니면 false·빈 문자열.\n"
    "rationale 은 한국어 1문장. input_hash 는 그대로 돌려준다."
)

OUTPUT_SCHEMA = {
    "type": "object", "additionalProperties": False,
    "required": ["input_hash", "predicted_blocks", "prediction_rationale", "assignments", "request_reserve_uavs",
                 "reserve_reason"],
    "properties": {
        "input_hash": {"type": "string"},
        "predicted_blocks": {"type": "array", "items": {"type": "string"}},
        "prediction_rationale": {"type": "string"},
        "request_reserve_uavs": {"type": "boolean"},
        "reserve_reason": {"type": "string"},
        "assignments": {"type": "array", "items": {
            "type": "object", "additionalProperties": False,
            "required": ["resource_id", "block_ids", "purpose", "evidence_refs", "rationale"],
            "properties": {"resource_id": {"type": "string"},
                           "block_ids": {"type": "array", "items": {"type": "string"}},
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
    if not isinstance(out["request_reserve_uavs"], bool):
        v.append("REQUEST_RESERVE_TYPE")
    elif out["request_reserve_uavs"] and not _text_ok(out["reserve_reason"]):
        v.append("RESERVE_REASON_INVALID")
    elif not isinstance(out["reserve_reason"], str):
        v.append("RESERVE_REASON_TYPE")
    avail = {r["resource_id"] for r in inp["resources_available"]}
    ids = known_ids(inp)
    if not isinstance(out["assignments"], list):
        return v + ["ASSIGNMENTS_TYPE"]
    used_r, used_b = set(), set()
    max_stops = inp.get("order_max_stops") or config.OBS_ORDER_MAX_STOPS
    for n, a in enumerate(out["assignments"]):
        if not isinstance(a, dict) or set(a) != set(OUTPUT_SCHEMA["properties"]["assignments"]["items"]["required"]):
            v.append(f"SCHEMA_MISMATCH:assignments[{n}]")
            continue
        rid, bids = a["resource_id"], a["block_ids"]
        if not isinstance(rid, str) or rid not in avail:
            v.append(f"RESOURCE_NOT_AVAILABLE:{rid}")
        elif rid in used_r:
            v.append(f"RESOURCE_DUPLICATE:{rid}")
        if not _str_list(bids) or not 1 <= len(bids) <= max_stops:
            v.append(f"BLOCK_IDS_INVALID:{n}:1~{max_stops}")
            bids = [b for b in bids if isinstance(b, str)] if isinstance(bids, list) else []
        for bid in bids:
            if bid not in blocks:
                v.append(f"BLOCK_NOT_SELECTABLE:{bid}")
            elif bid in used_b:
                v.append(f"BLOCK_DUPLICATE:{bid}")
            elif isinstance(rid, str) and rid in avail and rid not in blocks[bid]["allowed_resources"]:
                v.append(f"RESOURCE_NOT_ALLOWED_FOR_BLOCK:{rid}:{bid}")
            used_b.add(bid)
        if a["purpose"] not in PURPOSES:
            v.append(f"PURPOSE_INVALID:{n}")
        if not _str_list(a["evidence_refs"]):
            v.append(f"EVIDENCE_TYPE:{n}")
        else:
            # 입력에 없는 근거 ID 는 다시 묻지 않고 뺀다 (2026-10-08 run2: 근거 번호 하나 때문에 30초짜리 재질문).
            # 배정 자체(자원·구역)는 위에서 엄격히 검사한다
            a["evidence_refs"] = [x for x in a["evidence_refs"] if x in ids]
        if not _text_ok(a["rationale"]):
            v.append(f"RATIONALE_INVALID:{n}")
        if isinstance(rid, str):
            used_r.add(rid)
    # 고를 수 있는 구역이 남아 있는데 놀리는 자원이 있으면 위반 (자원마다 지시서 하나 — 계획서 §5)
    missing = sorted(r for r in avail - used_r
                     if any(r in b["allowed_resources"] and k not in used_b for k, b in blocks.items()))
    if missing:
        v.append("RESOURCE_NOT_ASSIGNED:" + ",".join(missing))
    return v


# ---------------------------------------------------------------------------
# 규칙 대체 (LLM 실패·검증 실패·한도 소진). 정답을 보지 않고 입력만 쓴다.
#   테두리 바로 바깥 미관측이 많은 순 → 미관측이 있는 구역 → 미관측이 많은 순 → 알려진 불에 가까운 순 → 자원에서 가까운 순 → 구역 ID
# ---------------------------------------------------------------------------
def rule_plan(inp: dict, only: Optional[List[str]] = None) -> dict:
    """자원마다 구역을 최대 order_max_stops 개 (돌아가며 한 개씩 골라 앞 순위 구역이 한 자원에 몰리지 않게).
    only 를 주면 그 자원만 계획한다 (충전을 마친 드론 — 사용자 결정 2026-10-08)."""
    taken, picks = set(), {}
    res = sorted((r for r in inp["resources_available"] if only is None or r["resource_id"] in only),
                 key=lambda r: r["resource_id"])
    for _ in range(inp.get("order_max_stops") or config.OBS_ORDER_MAX_STOPS):
        for r in res:
            rid = r["resource_id"]
            cands = [b for b in inp["blocks"] if b["block_id"] not in taken and rid in b["allowed_resources"]]
            if not cands:
                continue
            b = min(cands, key=lambda b: (-b["unobserved_next_to_fire"],
                                          b["counts"]["UNOBSERVED"] == 0, -b["counts"]["UNOBSERVED"],
                                          b["dist_to_known_fire_m"], b["distance_km_from"].get(rid, math.inf),
                                          b["block_id"]))
            taken.add(b["block_id"])
            picks.setdefault(rid, []).append(b)
    out = []
    for rid, bs in picks.items():
        b = bs[0]
        purpose = ("INITIAL_REPORT" if b["counts"]["REPORTED"] else
                   "BOUNDARY_CHECK" if b["touches_known_fire_block"] else
                   "PREDICTED_SPREAD" if b["counts"]["UNOBSERVED"] else "RECHECK")
        out.append({"resource_id": rid, "block_ids": [x["block_id"] for x in bs], "purpose": purpose,
                    "evidence_refs": [x["block_id"] for x in bs],
                    "rationale": "규칙 대체: 알려진 테두리 바로 바깥 미관측이 많은 구역"})
    return {"input_hash": inp["input_hash"], "predicted_blocks": [], "prediction_rationale": "규칙 대체 (예측 없음)",
            "assignments": out, "request_reserve_uavs": False, "reserve_reason": ""}


# ---------------------------------------------------------------------------
# LLM 에 보내는 짧은 형태 (사용자 결정 2026-10-07: 토큰 절약). 내부 계산·검증·해시는 원래 입력(build_input)으로 한다.
# 항목 이름을 줄이고 0·빈 값은 뺀다. 뜻은 LEGEND 로 지시문에 준다.
# ---------------------------------------------------------------------------
LEGEND = (
    "약어: blocks[] 고를 수 있는 구역 — id, n 칸 수(U 미관측·C 불없음·P 일부·B 불타는중·X 탄곳·PX 꺼짐추정·R 신고), "
    "edge 테두리 칸, front 테두리 바로 바깥 미관측 칸, d 아는 불까지 m, brg 불 중심에서 방향°, ahead 진행 방향 앞쪽, "
    "up 아는 불보다 높은 m, road 도로 칸, fuel 연료, t 마지막 관측 s, ok 보낼 수 있는 자원(없으면 전부), near [가까운 자원, km]. "
    "ctx[] 참고용(고를 수 없음). res[] [ID, 종류, bat 배터리%, now(ON_STOP=지금 지점 본 뒤 시작)]. "
    "chg[] 충전 중 [ID, 남은 s]. busy[] [ID, 구역]. weather {항목: [값, 기준, 관측 s]} (wind_dir_deg 는 불어오는 방향). "
    "UAV: 테두리를 다 탄 쪽 20 m 안에서 따라 날며 폭 52 m 를 본다. UGV: 가까운 도로에서 반지름 600 m~2 km 를 본다 (산에 가리면 안 보임)."
)
_COUNT_KEYS = {"UNOBSERVED": "U", "CLEAR": "C", "PARTIAL": "P", "BURNING": "B", "BURNED": "X", "PRESUMED_BURNED": "PX",
               "REPORTED": "R"}


def _short_counts(counts: dict) -> dict:
    return {_COUNT_KEYS.get(k, k): v for k, v in counts.items() if v}


def llm_view(inp: dict) -> dict:
    avail = [r["resource_id"] for r in inp["resources_available"]]

    def row(b, ctx=False):
        out = {"id": b["block_id"], "n": _short_counts(b["counts"]), "edge": b.get("known_edge_cells"),
               "front": b.get("unobserved_next_to_fire"), "d": b.get("dist_to_known_fire_m"),
               "brg": b.get("bearing_from_fire_centre_deg"), "ahead": b.get("ahead_of_progress"),
               "up": b.get("elev_above_known_fire_m"), "road": b.get("road_cells"), "fuel": b.get("mean_fuel"),
               "t": b.get("last_observed_sim_s"), "obs": b.get("last_observation_id")}
        if ctx:
            out["busy"] = b.get("in_progress_by") or None
        else:
            if sorted(b["allowed_resources"]) != sorted(avail):
                out["ok"] = b["allowed_resources"]
            km = b.get("distance_km_from") or {}
            if km:
                rid = min(km, key=lambda k: (km[k], k))
                out["near"] = [rid, km[rid]]
        return {k: v for k, v in out.items() if v not in (None, 0, [], {}, False) or k in ("id", "ahead") and v is not None}
    w = inp.get("weather") or {}
    weather = {i: [r.get("value"), r.get("basis"), r.get("observed_sim_s")] for i, r in (w.get("items") or {}).items()}
    prog = {k: v for k, v in (inp.get("fire_progress") or {}).items() if k != "note"} or None
    rsv = inp.get("reserve_uavs")
    rsv = rsv and {k: rsv.get(k) for k in ("ids", "active", "bearing_fire_to_base_deg", "distance_km")}
    out = {"input_hash": inp["input_hash"], "t": inp["simulation_time_s"], "end": inp["demo_end_sim_s"],
           "order_max_stops": inp.get("order_max_stops"),
           "reports": [[r["report_id"], r["cell_id"], r["sim_time_s"]] for r in inp["reports"]],
           "weather": weather, "fire": {k: v for k, v in inp["fire_summary"].items()
                                        if v and k not in ("unobserved_note", "burn_out_after_s")},
           "progress": prog, "reserve": rsv,
           "res": [[r["resource_id"], r["resource_type"], r.get("battery_pct"), r.get("now")]
                   for r in inp["resources_available"]],
           "chg": [[r["resource_id"], r.get("ready_in_s")] for r in inp.get("resources_charging") or []],
           "busy": [[r["resource_id"], r.get("block_id")] for r in inp["resources_busy"]],
           "blocks": [row(b) for b in inp["blocks"]], "ctx": [row(b, True) for b in inp["context_blocks"]],
           "purposes": inp["allowed_purposes"]}
    if inp.get("emergency"):
        out["emergency"] = inp["emergency"]
    return {k: v for k, v in out.items() if v not in (None, [], {})}
