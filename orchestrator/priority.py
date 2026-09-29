# -*- coding: utf-8 -*-
"""
orchestrator/priority.py
========================
임무 우선순위 판단 근거 (요청서 §8, 사용자 결정 2026-09-29).

입력 (같은 환경 snapshot 안에서만)
- risk_cells      : 화선 주변 UNBURNED 위험 셀. cell_id, lat, lon, cell_size_m, risk_score
- protected_sites : 보호대상 registry. site_id, site_type, boundary[[lat,lon],...] 또는 location{lat,lon},
                    official_priority_rank(공식 문서에 순위가 있을 때만), official_source{...}

계산
- 각 위험 셀(정사각형)과 각 보호대상 경계의 최소 거리(m). 좌표가 없으면 계산하지 않는다.
- 임무별 기준: 위험도(높을수록), 보호대상까지 거리(가까울수록), 가장 가까운 보호대상의 공식 순위(높을수록)

비교 규칙 — 가중치·합성식 없음
- 모든 기준에서 같거나 낫고 하나 이상 더 나으면 먼저 (우월 관계). 이 관계로 층(front)을 나눈다.
- 같은 층 안의 임무는 서로 엇갈리는 것이다. 자원이 모자라 둘 중 하나만 갈 수 있을 때
  총괄이 임의로 고르지 않고 "사람 판단 필요"로 보류한다 (엔진 dispatch_pending).
- 기준 값이 없는 임무: 위험도 누락은 사용자 결정대로 그 기준에서 가장 뒤로 보고 기록한다.
  거리·공식 순위는 모든 임무에 값이 있을 때만 비교 기준으로 쓴다.
- 완전히 같은 경우에만 접수 순서로 정한다 (사용자 결정 6번째 기준).

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

def risk_context(snapshot: Snapshot) -> dict:
    cells, sites = snapshot.risk_cells, snapshot.protected_sites
    lats = [c["lat"] for c in cells if c.get("lat") is not None] + \
           [s.get("location", {}).get("lat") for s in sites if s.get("location")] + \
           [p[0] for s in sites for p in (s.get("boundary") or [])]
    ctx = {"snapshot": snapshot.ref(), "crs": snapshot.crs, "method": METHOD,
           "registry_present": bool(sites), "cells": {}, "sites": {}, "missing": []}
    if not lats:
        return ctx
    proj = _Proj(sum(lats) / len(lats))
    geoms = {}
    for s in sites:
        g, kind = _site_geometry(proj, s)
        ctx["sites"][s["site_id"]] = {"site_type": s.get("site_type"), "geometry": kind,
                                      "official_priority_rank": s.get("official_priority_rank"),
                                      "official_source": s.get("official_source")}
        if g is None:
            ctx["missing"].append({"site_id": s["site_id"], "reason": "SITE_GEOMETRY_MISSING"})
        else:
            geoms[s["site_id"]] = g
    for c in cells:
        poly = _cell_polygon(proj, c)
        entry = {"risk_score": c.get("risk_score"), "fire_state": c.get("fire_state"), "distances": []}
        if poly is None:
            ctx["missing"].append({"cell_id": c.get("cell_id"), "reason": "CELL_GEOMETRY_MISSING"})
        else:
            for sid, g in geoms.items():
                entry["distances"].append({"site_id": sid, "distance_m": round(polygon_distance_m(poly, g), 1)})
            entry["distances"].sort(key=lambda d: (d["distance_m"], d["site_id"]))
        ctx["cells"][c["cell_id"]] = entry
    return ctx


# ---------------------------------------------------------------------------
# 임무 비교
# ---------------------------------------------------------------------------

def task_evidence(task: Task, ctx: dict) -> dict:
    """임무 대상 셀들의 기준 값. 값이 없으면 None 으로 둔다."""
    ids = list(task.area_cell_ids or ([task.target.cell_id] if task.target.cell_id else []))
    entries = [(cid, ctx["cells"][cid]) for cid in ids if cid in ctx["cells"]]
    scores = [e["risk_score"] for _, e in entries if isinstance(e.get("risk_score"), (int, float))]
    nearest = None
    for cid, e in entries:
        if e["distances"] and (nearest is None or e["distances"][0]["distance_m"] < nearest["distance_m"]):
            nearest = {"cell_id": cid, **e["distances"][0]}
    rank = None
    if nearest:
        rank = ctx["sites"].get(nearest["site_id"], {}).get("official_priority_rank")
    return {"task_id": task.task_id, "cells_used": [c for c, _ in entries],
            "risk_score": max(scores) if scores else None,
            "risk_score_missing": not scores,
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


def order_tasks(tasks: List[Task], snapshot: Snapshot) -> dict:
    """{fronts: [[task_id,...], ...], evidence: {...}, criteria: [...], context: ...}
    front 안은 접수 순서. 같은 front 에서 기준 값이 서로 다르면 엇갈림(conflict)으로 표시한다."""
    ctx = risk_context(snapshot)
    evs = {t.task_id: task_evidence(t, ctx) for t in tasks}
    crit = _criteria(list(evs.values()))
    created = {t.task_id: (t.created_wall, t.task_id) for t in tasks}
    remaining, fronts = list(evs), []
    while remaining:
        front = [a for a in remaining if not any(_dominates(evs[b], evs[a], crit) for b in remaining if b != a)]
        front.sort(key=lambda tid: created[tid])
        fronts.append(front)
        remaining = [t for t in remaining if t not in front]
    conflicts = []
    for f in fronts:
        vals = {tuple(_key(evs[t], c) for c in crit) for t in f}
        if len(vals) > 1:
            conflicts.append(f)
    return {"fronts": fronts, "conflicting_fronts": conflicts, "criteria": crit, "evidence": evs,
            "rule": "PARETO_DOMINANCE_NO_WEIGHTS; equal → created order; risk missing → last",
            "context": {k: ctx[k] for k in ("snapshot", "crs", "method", "registry_present", "missing")}}
