# -*- coding: utf-8 -*-
"""
orchestrator/priority.py
========================
임무 우선순위 판단 근거 (요청서 §8, 사용자 결정 2026-09-29).

입력 (같은 환경 snapshot 안에서만)
- risk_cells      : 화선 주변 UNBURNED 위험 셀. cell_id, lat, lon, cell_size_m, risk_score,
                    인명 노출(human_exposure bool 또는 building_type, 1=주거 — 환경팀 제공 합의 2026-09-30)
- protected_sites : 보호대상 registry. site_id, site_type, boundary[[lat,lon],...] 또는 location{lat,lon},
                    official_priority_rank(공식 문서에 순위가 있을 때만), official_source{...}

계산
- 각 위험 셀(정사각형)과 각 보호대상 경계의 최소 거리(m). 좌표가 없으면 계산하지 않는다.
- 임무별 기준: 위험도(높을수록), 보호대상까지 거리(가까울수록), 가장 가까운 보호대상의 공식 순위(높을수록)

비교 규칙 (사용자 결정 2026-09-30) — 가중치·합성식 없음
- 인명피해 예상 지역이 항상 먼저 (소방청 SOP 인명 보호 최우선).
- 그다음 위험도(조금이라도 높은 쪽) → 보호대상 거리 → 공식 순위 → 접수 순서.
- 인명 여부가 같은 임무끼리 위험도 차이가 기준(0.1) 이내이거나 기준이 엇갈리면 "선택 묶음"으로
  화면에 나란히 표시한다. 사람 선택 버전은 자원이 모자랄 때 사람이 고를 때까지 대기,
  자동 버전은 위 순서대로 바로 보내고 '자동 결정'으로 표시한다 (engine.dispatch_pending).
- 위험도 누락은 가장 뒤로 보고 기록한다. 거리·공식 순위는 모든 임무에 값이 있을 때만 쓴다.

공간 계산: 대상들의 중심 위도 기준 국지 평면 근사(등장방형). 방법을 결과에 기록한다.
"""

import math
from typing import Dict, List, Optional, Tuple

from .models import Snapshot, Task

_M_PER_DEG_LAT = 111_320.0
METHOD = "LOCAL_EQUIRECTANGULAR_POLYGON_MIN_DISTANCE"


class _Proj:
    def __init__(self, lat0: float):
        self.kx = _M_PER_DEG_LAT * math.cos(math.radians(lat0))

    def xy(self, lat, lon):
        return lon * self.kx, lat * _M_PER_DEG_LAT


def _cell_polygon(proj: _Proj, cell: dict) -> Optional[List[Tuple[float, float]]]:
    if cell.get("lat") is None or cell.get("lon") is None or not cell.get("cell_size_m"):
        return None
    cx, cy = proj.xy(cell["lat"], cell["lon"])
    h = cell["cell_size_m"] / 2
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


def _site_geometry(proj: _Proj, site: dict):
    """(polygon 점 목록, 형식). 경계가 없고 대표점만 있으면 점 하나. 둘 다 없으면 (None, 'MISSING')"""
    b = site.get("boundary")
    if b and len(b) >= 3:
        return [proj.xy(p[0], p[1]) for p in b], "BOUNDARY"
    loc = site.get("location")
    if loc and loc.get("lat") is not None and loc.get("lon") is not None:
        return [proj.xy(loc["lat"], loc["lon"])], "POINT_ONLY"
    return None, "MISSING"


def _seg_dist(p, a, b) -> float:
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    if dx == dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _inside(p, poly) -> bool:
    x, y, inside = p[0], p[1], False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _edges(poly):
    return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))] if len(poly) > 1 else []


def polygon_distance_m(a: List[Tuple[float, float]], b: List[Tuple[float, float]]) -> float:
    """두 다각형(또는 점) 사이 최소 거리. 겹치면 0."""
    if len(a) >= 3 and any(_inside(p, a) for p in b):
        return 0.0
    if len(b) >= 3 and any(_inside(p, b) for p in a):
        return 0.0
    best = math.inf
    for p in a:
        for s in _edges(b) or [(b[0], b[0])]:
            best = min(best, _seg_dist(p, *s))
    for p in b:
        for s in _edges(a) or [(a[0], a[0])]:
            best = min(best, _seg_dist(p, *s))
    return best


# ---------------------------------------------------------------------------
# 위험 셀 ↔ 보호대상 거리 표
# ---------------------------------------------------------------------------

# 환경 코드(environment/src/risk.py)의 건물 유형 코드: 0 없음, 1 주거, 2 중요시설, 3 핵심시설
RESIDENTIAL_BUILDING_TYPE = 1


def _human_flag(cell: dict) -> Optional[bool]:
    """셀의 인명 노출 여부. 환경이 준 값만 쓴다: human_exposure(bool) 또는 building_type(1=주거)."""
    if isinstance(cell.get("human_exposure"), bool):
        return cell["human_exposure"]
    bt = cell.get("building_type")
    if isinstance(bt, int) and not isinstance(bt, bool):
        return bt == RESIDENTIAL_BUILDING_TYPE
    return None


def risk_context(snapshot: Snapshot) -> dict:
    sites = snapshot.protected_sites
    forecast = {f["cell_id"]: f for f in snapshot.spread_forecast if f.get("cell_id")}
    merged = {}
    for c in snapshot.fire_cells + snapshot.risk_cells + snapshot.spread_forecast:
        merged.setdefault(c["cell_id"], {}).update({k: v for k, v in c.items() if v is not None})
    cells = list(merged.values())
    lats = [c["lat"] for c in cells if c.get("lat") is not None] + \
           [s.get("location", {}).get("lat") for s in sites if s.get("location")] + \
           [p[0] for s in sites for p in (s.get("boundary") or [])]
    ctx = {"snapshot": snapshot.ref(), "crs": snapshot.crs, "method": METHOD,
           "registry_present": bool(sites), "cells": {}, "sites": {}, "missing": [],
           "forecast_ref": snapshot.forecast_ref, "forecast_threats": []}
    if not lats:
        return ctx
    proj = _Proj(sum(lats) / len(lats))
    geoms = {}
    for s in sites:
        g, kind = _site_geometry(proj, s)
        ctx["sites"][s["site_id"]] = {"site_type": s.get("site_type"), "geometry": kind,
                                      "official_priority_rank": s.get("official_priority_rank"),
                                      "human_occupied": s.get("human_occupied"),
                                      "official_source": s.get("official_source")}
        if g is None:
            ctx["missing"].append({"site_id": s["site_id"], "reason": "SITE_GEOMETRY_MISSING"})
        else:
            geoms[s["site_id"]] = g
    for c in cells:
        poly = _cell_polygon(proj, c)
        entry = {"risk_score": c.get("risk_score"), "fire_state": c.get("fire_state"),
                 "human_exposure": _human_flag(c), "building_type": c.get("building_type"), "distances": []}
        if poly is None:
            ctx["missing"].append({"cell_id": c.get("cell_id"), "reason": "CELL_GEOMETRY_MISSING"})
        else:
            for sid, g in geoms.items():
                entry["distances"].append({"site_id": sid, "distance_m": round(polygon_distance_m(poly, g), 1)})
            entry["distances"].sort(key=lambda d: (d["distance_m"], d["site_id"]))
        f = forecast.get(c["cell_id"])
        if f is not None:
            entry["forecast"] = {"expected_arrival_s": f.get("expected_arrival_s"),
                                 "probability": f.get("probability")}
        ctx["cells"][c["cell_id"]] = entry
    # 예측 확산 셀이 주거지이거나 사람 있는 보호대상과 겹치면 '예측된 인명 위험'
    for cid, e in ctx["cells"].items():
        if "forecast" not in e:
            continue
        eta = e["forecast"]["expected_arrival_s"]
        if e.get("human_exposure") is True:
            ctx["forecast_threats"].append({"cell_id": cid, "basis": "FORECAST_REACHES_RESIDENTIAL",
                                            "expected_arrival_s": eta})
        for d in e["distances"]:
            if d["distance_m"] == 0.0 and ctx["sites"].get(d["site_id"], {}).get("human_occupied") is True:
                ctx["forecast_threats"].append({"cell_id": cid, "site_id": d["site_id"],
                                                "basis": "FORECAST_REACHES_OCCUPIED_SITE", "expected_arrival_s": eta})
    return ctx


# ---------------------------------------------------------------------------
# 임무 비교
# ---------------------------------------------------------------------------

def task_evidence(task: Task, ctx: dict) -> dict:
    """임무 대상 셀들의 기준 값. 값이 없으면 None 으로 둔다 (지어내지 않음)."""
    ids = list(task.area_cell_ids or ([task.target.cell_id] if task.target.cell_id else []))
    entries = [(cid, ctx["cells"][cid]) for cid in ids if cid in ctx["cells"]]
    scores = [e["risk_score"] for _, e in entries if isinstance(e.get("risk_score"), (int, float))]
    nearest = None
    for cid, e in entries:
        if e["distances"] and (nearest is None or e["distances"][0]["distance_m"] < nearest["distance_m"]):
            nearest = {"cell_id": cid, **e["distances"][0]}
    if nearest:
        nearest["site_type"] = ctx["sites"].get(nearest["site_id"], {}).get("site_type")
    rank = ctx["sites"].get(nearest["site_id"], {}).get("official_priority_rank") if nearest else None
    # 인명피해 예상: 환경 셀의 human_exposure, 또는 사람이 있는 보호대상과 셀이 겹침(거리 0)
    human_sources = []
    flags = [e.get("human_exposure") for _, e in entries]
    for cid, e in entries:
        if e.get("human_exposure") is True:
            human_sources.append({"cell_id": cid, "source": "ENV_CELL_RESIDENTIAL" if e.get("building_type") == 1
                                  else "ENV_CELL_HUMAN_EXPOSURE"})
        for d in e["distances"]:
            if d["distance_m"] == 0.0 and ctx["sites"].get(d["site_id"], {}).get("human_occupied") is True:
                human_sources.append({"cell_id": cid, "site_id": d["site_id"], "source": "OVERLAPS_OCCUPIED_SITE"})
    for cid, e in entries:
        if "forecast" in e:
            for th in ctx.get("forecast_threats", []):
                if th["cell_id"] == cid:
                    human_sources.append({**th, "source": "FORECAST"})
    if human_sources:
        human = True
    elif flags and all(f is False for f in flags):
        human = False
    else:
        human = None                         # 정보 없음 — 인명 없음으로 단정하지 않고 기록
    return {"task_id": task.task_id, "cells_used": [c for c, _ in entries],
            "human_risk": human, "human_risk_sources": human_sources,
            "risk_score": max(scores) if scores else None, "risk_score_missing": not scores,
            "human_risk_basis": sorted({"FORECAST" if s["source"] == "FORECAST" else "EXPOSURE"
                                        for s in human_sources}),
            "forecast_arrival_s": min((e["forecast"]["expected_arrival_s"] for _, e in entries
                                       if "forecast" in e and e["forecast"]["expected_arrival_s"] is not None),
                                      default=None),
            "nearest_protected": nearest, "distance_m": nearest["distance_m"] if nearest else None,
            "official_priority_rank": rank}


def _criteria(evs: List[dict]) -> List[str]:
    """모든 임무에 값이 있는 기준만 비교에 쓴다. 위험도는 누락을 '가장 뒤'로 보고 항상 쓴다."""
    crit = ["risk_score"] if any(e["risk_score"] is not None for e in evs) else []
    if evs and all(e["distance_m"] is not None for e in evs):
        crit.append("distance_m")
    if evs and all(e["official_priority_rank"] is not None for e in evs):
        crit.append("official_priority_rank")
    return crit


def _key(ev: dict, c: str) -> float:
    """작을수록 앞. 위험도는 높을수록 앞이라 부호를 뒤집고, 누락은 +inf (가장 뒤)."""
    if c == "risk_score":
        return math.inf if ev["risk_score"] is None else -ev["risk_score"]
    return ev[c]


def _dominates(a, b, crit) -> bool:
    ka, kb = [_key(a, c) for c in crit], [_key(b, c) for c in crit]
    return all(x <= y for x, y in zip(ka, kb)) and any(x < y for x, y in zip(ka, kb))


def order_tasks(tasks: List[Task], snapshot: Snapshot, similar_delta: Optional[float] = None) -> dict:
    """우선순위 계획.

    auto_order : 인명 예상 → 위험도(조금이라도 높은 쪽) → 거리 → 공식 순위 → 접수 순서 (자동 버전)
    groups     : 사람 판단이 필요할 수 있는 묶음. 인명 여부가 같은 Task 끼리
                 - 위험도 차이가 similar_delta 이내 (SIMILAR)
                 - 기준이 엇갈림 (CONFLICT, 우월 관계 없음)
    units      : 배정 단위 순서. 묶음은 한 단위, 나머지는 한 Task 씩. auto_order 에서 가장 앞선
                 구성원의 위치로 정렬한다.
    """
    ctx = risk_context(snapshot)
    evs = {t.task_id: task_evidence(t, ctx) for t in tasks}
    crit = _criteria(list(evs.values()))
    created = {t.task_id: (t.created_wall, t.task_id) for t in tasks}

    def human_rank(tid):
        return 0 if evs[tid]["human_risk"] is True else 1

    auto_order = sorted(evs, key=lambda tid: (human_rank(tid), *(_key(evs[tid], c) for c in crit), created[tid]))
    pos = {tid: i for i, tid in enumerate(auto_order)}

    parent = {tid: tid for tid in evs}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    kinds = {}
    ids = list(evs)
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            if human_rank(a) != human_rank(b):
                continue                         # 인명 예상 지역은 항상 먼저 — 묶지 않는다
            ea, eb = evs[a], evs[b]
            why = set()
            if (similar_delta is not None and ea["risk_score"] is not None and eb["risk_score"] is not None
                    and abs(ea["risk_score"] - eb["risk_score"]) <= similar_delta + 1e-12):
                why.add("SIMILAR")
            same = all(_key(ea, c) == _key(eb, c) for c in crit)
            if crit and not same and not _dominates(ea, eb, crit) and not _dominates(eb, ea, crit):
                why.add("CONFLICT")
            if why:
                ra, rb = find(a), find(b)
                parent[ra] = rb
                kinds.setdefault(frozenset((a, b)), set()).update(why)
    comps = {}
    for tid in ids:
        comps.setdefault(find(tid), []).append(tid)
    groups, units = [], []
    for members in comps.values():
        members.sort(key=lambda t: pos[t])
        if len(members) > 1:
            k = set()
            for pair, why in kinds.items():
                if pair <= set(members):
                    k |= why
            groups.append({"task_ids": members, "kinds": sorted(k), "auto_choice": members[0]})
        units.append(members)
    units.sort(key=lambda m: pos[m[0]])
    groups.sort(key=lambda g: pos[g["task_ids"][0]])
    return {"auto_order": auto_order, "units": units, "groups": groups, "criteria": crit,
            "similar_delta": similar_delta, "evidence": evs,
            "rule": "HUMAN_RISK_FIRST; then risk_score desc, distance asc, official rank; equal → created order; "
                    "risk missing → last; groups = same human class & (risk diff <= delta or crossing criteria)",
            "context": {k: ctx[k] for k in ("snapshot", "crs", "method", "registry_present", "missing",
                                            "forecast_ref", "forecast_threats")}}
