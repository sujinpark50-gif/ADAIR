# -*- coding: utf-8 -*-
"""ugv/tools/terrain_compare.py — 두 타일 세트(수정 전·후)의 도로별 정적 지표 비교: '고치면서 주변을 망가뜨리지 않았나'.

    python -m ugv.tools.terrain_compare --before <타일 폴더> --after <타일 폴더> [--out compare.json]

- 악화: 같은 도로의 종단(10 m 창)·횡단(교차로 밖/안)·2차 차분이 허용 차를 넘게 커짐. 새 실패 = 수정 전 기준 안 → 수정 후 기준 밖.
- 경사 꺾임(차량 pitch 변화): 2 m 표본 종단에서 앞뒤 10 m 경사 차 |g(s+) − g(s−)| 의 최대 — 교차로 노드 30 m 안 / 교량 양 끝 30 m 안 / 그 밖.
- 기준값은 정적 검사와 같다 (종단 10 %, 횡단 6 %, 2차 차분 0.10 m) — 완화 없음.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from ugv.roads import RoadNetwork
from ugv.tools import build_terrain_tiles as BT

TOL = {"max_long_grade": 0.005, "max_cross_slope": 0.005, "max_cross_slope_junction_zone": 0.005, "max_second_diff_m": 0.02}
LIMIT = {"max_long_grade": 0.10, "max_cross_slope": 0.06, "max_cross_slope_junction_zone": 0.06, "max_second_diff_m": 0.10}
NEAR_M = 30.0


class Surf:
    """지정 폴더의 생성 타일 표면 (BT.Surface 와 같은 식, 폴더만 다름)."""

    def __init__(self, d):
        self.d = Path(d)
        self.meta = json.loads((self.d / "tiles.json").read_text(encoding="utf-8"))
        self.cache = {}

    def z(self, x, y):
        i, j = int((x - BT.X0) // BT.TILE_M), int((y - BT.Y0) // BT.TILE_M)
        name = BT.tile_name(i, j)
        t = self.meta["tiles"].get(name)
        if t is None:
            return math.nan
        g = self.cache.get(name)
        if g is None:
            g = self.cache[name] = t["zmin"] + BT.read_png16(self.d / name / "heightmap.png").astype(float) / 65535.0 * t["span"]
            if len(self.cache) > 60:
                self.cache.pop(next(iter(self.cache)))
        x0, y0, x1, y1 = BT.tile_bounds(i, j)
        cell = BT.TILE_M / (BT.N - 1)
        fx, fy = (x - x0) / cell, (y1 - y) / cell
        c0, r0 = min(int(fx), BT.N - 2), min(int(fy), BT.N - 2)
        tx, ty = fx - c0, fy - r0
        return float(g[r0, c0] * (1 - tx) * (1 - ty) + g[r0, c0 + 1] * tx * (1 - ty) + g[r0 + 1, c0] * (1 - tx) * ty
                     + g[r0 + 1, c0 + 1] * tx * ty)


def grade_breaks(net, S, rid, near_pts, step=2.0, half=10.0):
    """도로 종단 경사 꺾임 최대 (교차로·교량 끝 30 m 안, 그 밖)."""
    r = net.roads[rid]
    if r.length < 2 * half + 2 * step:
        return None
    s = np.arange(0.0, r.length + 1e-9, step)
    P = np.array([net.at(rid, v)[2:4] for v in s])
    z = np.array([S.z(x, y) for x, y in P])
    if np.isnan(z).any():
        return None
    k = int(half / step)
    g = (z[k:] - z[:-k]) / half                     # 구간 [i, i+k] 경사
    brk = np.abs(g[k:] - g[:-k])                     # 중심 i+k 앞뒤 10 m 경사 차
    c = P[k:len(P) - k]
    if len(near_pts):
        dn = np.sqrt(((c[:, None, :] - near_pts[None]) ** 2).sum(2)).min(1)
    else:
        dn = np.full(len(c), np.inf)
    near = brk[dn <= NEAR_M]
    far = brk[dn > NEAR_M]
    return (float(near.max()) if near.size else 0.0, float(far.max()) if far.size else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--breaks", action="store_true", help="경사 꺾임(차량 pitch 변화)도 계산 (느림)")
    a = ap.parse_args()
    cb = json.loads((Path(a.before) / "static_checks.json").read_text(encoding="utf-8"))["per_road"]
    ca = json.loads((Path(a.after) / "static_checks.json").read_text(encoding="utf-8"))["per_road"]
    worse, new_fail, fixed = {}, {}, {}
    for rid in sorted(set(cb) & set(ca)):
        b, c = cb[rid], ca[rid]
        if "error" in b or "error" in c:
            continue
        for k, tol in TOL.items():
            if c[k] > b[k] + tol:
                worse.setdefault(rid, []).append([k, b[k], c[k]])
                if b[k] <= LIMIT[k] < c[k]:
                    new_fail.setdefault(rid, []).append([k, b[k], c[k]])
            if b[k] > LIMIT[k] >= c[k]:
                fixed.setdefault(rid, []).append([k, b[k], c[k]])
    out = {"before": a.before, "after": a.after, "tol": TOL, "limit": LIMIT,
           "roads_compared": len(set(cb) & set(ca)), "roads_only_after": sorted(set(ca) - set(cb)),
           "worse_roads": len(worse), "new_fail_roads": len(new_fail), "fixed_roads": len(fixed),
           "worse": worse, "new_fail": new_fail, "fixed": fixed}
    meta_a = json.loads((Path(a.after) / "tiles.json").read_text(encoding="utf-8"))
    dc = meta_a.get("design_cap") or {}
    if dc:
        rr = dc.get("roads", {})
        res = [x for x in rr.values() if x.get("status") == "RESTORED"]
        out["design_cap"] = {"over_cap": len(rr), "restored": len(res),
                             "not_restored": sum(1 for x in rr.values() if x.get("status") == "NOT_RESTORED"),
                             "max_road_dz_m": max((x["max_dev_m"] for x in res), default=0.0),
                             "nodes_moved": len(dc.get("node_shift_m", {})),
                             "max_node_dz_m": max((abs(v) for v in dc.get("node_shift_m", {}).values()), default=0.0)}
    if a.breaks:
        net = RoadNetwork()
        Sb, Sa = Surf(a.before), Surf(a.after)
        inc = [r for r in ca if "error" not in ca[r]]
        jn = BT.junction_nodes(net, inc)
        assumed = meta_a.get("roads_assumed") or {}
        bridge_ends = [net.nodes[n] for r in assumed for n in (net.roads[r].a, net.roads[r].b)]
        near_pts = np.array([net.nodes[n] for n in jn] + bridge_ends) if (jn or bridge_ends) else np.zeros((0, 2))
        bridge_pts = np.array(bridge_ends) if bridge_ends else np.zeros((0, 2))
        rows = {}
        for rid in inc:
            if rid not in cb:
                continue
            gb, ga = grade_breaks(net, Sb, rid, near_pts), grade_breaks(net, Sa, rid, near_pts)
            if gb is None or ga is None:
                continue
            rows[rid] = {"near_before": round(gb[0], 4), "near_after": round(ga[0], 4), "far_before": round(gb[1], 4),
                         "far_after": round(ga[1], 4)}
        nb = np.array([v["near_before"] for v in rows.values()])
        na = np.array([v["near_after"] for v in rows.values()])
        out["grade_breaks"] = {"roads": len(rows), "basis": "앞뒤 10 m 경사 차 (교차로 노드·교량 끝 30 m 안)",
                               "near_p95_before": round(float(np.percentile(nb, 95)), 4), "near_p95_after": round(float(np.percentile(na, 95)), 4),
                               "near_max_before": round(float(nb.max()), 4), "near_max_after": round(float(na.max()), 4),
                               "near_worse_roads(+0.02)": sorted(r for r, v in rows.items() if v["near_after"] > v["near_before"] + 0.02),
                               "per_road": rows}
        # 교량 접속부만 따로: 교량에 붙은 일반 도로들의 끝 30 m
        if len(bridge_pts):
            att = [r for r in rows if any(n in {net.roads[b].a, net.roads[b].b} for b in assumed for n in (net.roads[r].a, net.roads[r].b))
                   and r not in assumed]
            out["bridge_approach"] = {"roads": len(att),
                                      "near_max_after": round(max((rows[r]["near_after"] for r in att), default=0.0), 4),
                                      "near_p95_after": round(float(np.percentile([rows[r]["near_after"] for r in att], 95)), 4) if att else None}
    p = Path(a.out) if a.out else Path(a.after) / "compare.json"
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps({k: v for k, v in out.items() if k not in ("worse", "new_fail", "fixed", "grade_breaks")}, ensure_ascii=False))
    if "grade_breaks" in out:
        print(json.dumps({k: v for k, v in out["grade_breaks"].items() if k not in ("per_road",)}, ensure_ascii=False)[:800])


if __name__ == "__main__":
    main()
