# -*- coding: utf-8 -*-
"""ugv/tools/road_candidates.py — 차량 직선(60 km/h·가감속·제동)·경사 시험에 쓸 도로 후보를 생성 타일 지면에서 고른다.

    python -m ugv.tools.road_candidates --tiles logs/ugv/stage5/tiles_v2 [--subset <road_subset.json>] [--out cand.json]

2 m 표본으로 도로마다: 길이, 누적 방향 변화(°), 최소 곡률 반경(10 m 현), 종단 경사(10 m 창) 최대·평균, 횡경사(차로 반폭 좌우 높이 차)
최대, 지나가는 교차로 노드 수(양 끝 제외). 기준은 결과를 보기 전에 정한 것 (차량 개조 구현 계획 6.3절):
  직선: 길이 ≥ 400 m, 방향 변화 ≤ 3°, |종단| ≤ 1.5 %, 횡경사 ≤ 2 %, 중간 교차로 0 → 상위 3개 (길이 순)
  경사: 길이 ≥ 150 m, 횡경사 ≤ 2 %, 방향 변화 ≤ 15°, 종단 평균 등급 3~4 %, 5~6 %, 7~8 % 각 1곳 (그 등급 안에서 가장 일정한 것)
한 도로(OSM way) 안만 본다 — 여러 도로를 이은 직선은 고르지 않는다. 지면 = 타일 높이맵 (Gazebo 충돌면과 같은 파일).
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from ugv.roads import RoadNetwork
from ugv.tools import build_terrain_tiles as BT

STRAIGHT = {"min_len_m": 400.0, "max_turn_deg": 3.0, "max_grade": 0.015, "max_cross": 0.02, "max_mid_junctions": 0}
SLOPE = {"min_len_m": 150.0, "max_cross": 0.02, "max_turn_deg": 15.0, "bands": [(0.03, 0.04), (0.05, 0.06), (0.07, 0.08)]}
STEP = 2.0


class Surf:
    """생성 타일 폴더의 지면 높이 (쌍선형, build_terrain_tiles 와 같은 격자)."""

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


def road_metrics(net, S, rid, junctions):
    r = net.roads[rid]
    s = np.arange(0.0, r.length + 1e-9, STEP)
    P = [net.at(rid, v) for v in s]
    xy = np.array([[p.x, p.y] for p in P])
    hw = BT.est_half_width(r)
    hd = np.unwrap(np.arctan2(np.diff(xy[:, 1]), np.diff(xy[:, 0])))
    turn = float(np.degrees(np.abs(hd[-1] - hd[0]))) if len(hd) else 0.0
    turn_total = float(np.degrees(np.abs(np.diff(hd)).sum())) if len(hd) > 1 else 0.0
    z = np.array([S.z(x, y) for x, y in xy])
    if np.isnan(z).any():
        return None
    k = int(round(10.0 / STEP))
    g = (z[k:] - z[:-k]) / (s[k:] - s[:-k]) if len(z) > k else np.array([0.0])
    cross = []
    for i in range(len(xy)):
        a, b = xy[max(i - 1, 0)], xy[min(i + 1, len(xy) - 1)]
        tx, ty = b - a
        n = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / n, tx / n
        cross.append(abs(S.z(xy[i, 0] + nx * hw, xy[i, 1] + ny * hw) - S.z(xy[i, 0] - nx * hw, xy[i, 1] - ny * hw)) / (2 * hw))
    rmin = math.inf
    for i in range(k, len(xy) - k):
        a, b, c = xy[i - k], xy[i], xy[i + k]
        ab, bc, ca = np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c)
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        if area2 > 1e-9:
            rmin = min(rmin, ab * bc * ca / (2 * area2))
    mid = sum(1 for n in junctions if n not in (r.a, r.b) and min(np.hypot(*(xy - net.nodes[n]).T)) < 3.0)
    return {"road": rid, "a": r.a, "b": r.b, "len_m": round(r.length, 1), "lanes": r.attrs.get("lanes"), "rank": r.attrs.get("road_rank"),
            "oneway": r.attrs.get("oneway"), "turn_deg": round(turn, 2), "turn_total_deg": round(turn_total, 2),
            "min_radius_m": None if rmin == math.inf else round(rmin, 1),
            "grade_max": round(float(np.abs(g).max()), 4), "grade_mean": round(float((z[-1] - z[0]) / r.length), 4),
            "grade_std": round(float(np.std(g)), 4), "cross_max": round(float(max(cross)), 4), "mid_junctions": mid,
            "start_xy": [round(float(xy[0, 0]), 2), round(float(xy[0, 1]), 2)], "end_xy": [round(float(xy[-1, 0]), 2), round(float(xy[-1, 1]), 2)],
            "tiles": sorted({BT.tile_name(int((x - BT.X0) // BT.TILE_M), int((y - BT.Y0) // BT.TILE_M)) for x, y in xy[::25]})}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tiles", required=True)
    ap.add_argument("--subset", default=None, help="도로 부분집합 JSON (기본: 타일 폴더 road_subset_all_included.json)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    S = Surf(a.tiles)
    sub = Path(a.subset) if a.subset else Path(a.tiles) / "road_subset_all_included.json"
    roads = set(json.loads(sub.read_text(encoding="utf-8"))["roads"])
    net = RoadNetwork()
    roads &= set(net.roads)
    junctions = BT.junction_nodes(net, list(roads))
    rows = []
    for rid in sorted(roads):
        if net.roads[rid].length < SLOPE["min_len_m"]:
            continue
        m = road_metrics(net, S, rid, junctions)
        if m:
            rows.append(m)
    st = [m for m in rows if m["len_m"] >= STRAIGHT["min_len_m"] and m["turn_total_deg"] <= STRAIGHT["max_turn_deg"]
          and m["grade_max"] <= STRAIGHT["max_grade"] and m["cross_max"] <= STRAIGHT["max_cross"] and m["mid_junctions"] <= STRAIGHT["max_mid_junctions"]]
    st.sort(key=lambda m: -m["len_m"])
    slopes = {}
    for lo, hi in SLOPE["bands"]:
        c = [m for m in rows if m["len_m"] >= SLOPE["min_len_m"] and m["cross_max"] <= SLOPE["max_cross"]
             and m["turn_total_deg"] <= SLOPE["max_turn_deg"] and lo <= abs(m["grade_mean"]) < hi]
        c.sort(key=lambda m: (m["grade_std"], -m["len_m"]))
        slopes[f"{lo * 100:.0f}-{hi * 100:.0f}%"] = c[:3]
    out = {"tiles": str(a.tiles), "subset": str(sub), "criteria": {"straight": STRAIGHT, "slope": SLOPE}, "roads_scanned": len(rows),
           "straight": st[:3], "straight_count": len(st), "slope": slopes,
           "near_straight": sorted([m for m in rows if m["len_m"] >= 300 and m["turn_total_deg"] <= 6 and m["grade_max"] <= 0.03],
                                   key=lambda m: -m["len_m"])[:5]}
    txt = json.dumps(out, ensure_ascii=False, indent=1)
    if a.out:
        Path(a.out).write_text(txt, encoding="utf-8")
    print(f"검사 도로 {len(rows)}, 직선 후보 {len(st)}:", [(m['road'], m['len_m'], m['turn_total_deg'], m['grade_max']) for m in st[:3]])
    for k, v in slopes.items():
        print(f"경사 {k}:", [(m['road'], m['len_m'], m['grade_mean'], m['cross_max']) for m in v])


if __name__ == "__main__":
    main()
