# -*- coding: utf-8 -*-
"""ugv/tools/junction_turns.py — 교차로 회전(진입 도로 → 진출 도로)마다 지면 지표를 두 타일 세트(전·후)로 비교 (트윈 지도 정제 3안).

    python -m ugv.tools.junction_turns --before <타일 폴더> --after <타일 폴더> --out turns.json

배경 (2026-10-12): v2 교차로 평탄화(level) 뒤 두 곡선 교차로(495759, 494846)에서 같은 미션이 v1·평탄 지면에서는 통과,
v2 에서만 곡선 바깥으로 밀려 이탈했다. 노드 하나만 terrain 혼합으로 되돌리면 통과. 기존 정적 지표(도로 중심선 종단·횡단·
2차 차분, 교차로 30 m 경사 꺾임)에는 걸리지 않았다 → 회전 경로 둘레 지면을 보는 후보 지표를 만들고 Gazebo 표본으로 가린다.

회전 경로 = 진입 도로 마지막 TURN_HALF_M + 진출 도로 처음 TURN_HALF_M (중심선; 실제 계획 원호는 안쪽으로 짧다).
지표 (경로 둘레 ±CORR_M, 0.5 m 간격 표본):
  hf_rms_mm     : 2 m 보다 짧은 굴곡(지면 − σ 2 m 평활) rms — 교차로 둘레 패치에서
  dz_max_m      : 전후 높이 차 최댓값
  bank_pct      : 꼭짓점 ±5 m 의 횡경사 평균 (+ = 회전 안쪽이 낮음, 곡선에 맞는 방향)
  sag_pp        : 경로 종단 경사의 10 m 창 변화 최댓값 (pp)
  grade_in_pct  : 진입 마지막 20 m 평균 종단 경사 (− = 내리막 진입)
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from ugv.roads import RoadNetwork
from ugv.tools import build_terrain_tiles as BT

TURN_HALF_M = 25.0
CORR_M = 3.0
MIN_TURN_DEG, MAX_TURN_DEG = 40.0, 140.0
MIN_ROAD_M = 50.0          # 진입 도로: 출발(교차로 45 m 앞)을 둘 길이
MIN_OUT_M = 26.0           # 진출 도로: 목표(교차로 20 m 뒤)와 회전 경로 반쪽(25 m)을 둘 길이
PATCH_M = 40.0             # 굴곡 계산 패치 반 폭


class Grid:
    def __init__(self, d):
        self.d = Path(d)
        self.meta = json.loads((self.d / "tiles.json").read_text(encoding="utf-8"))
        self.cache = {}

    def tile(self, name):
        g = self.cache.get(name)
        if g is None:
            t = self.meta["tiles"].get(name)
            p = self.d / name / "heightmap.png"
            if t is None or not p.exists():
                return None
            g = self.cache[name] = t["zmin"] + BT.read_png16(p).astype(float) / 65535.0 * t["span"]
            if len(self.cache) > 40:
                self.cache.pop(next(iter(self.cache)))
        return g

    def patch(self, x, y, half):
        """(x, y) 중심 패치 (타일 하나 안일 때만) → (격자, x0, y_top, cell) 또는 None."""
        i, j = int((x - BT.X0) // BT.TILE_M), int((y - BT.Y0) // BT.TILE_M)
        name = BT.tile_name(i, j)
        g = self.tile(name)
        if g is None:
            return None
        x0, y0, x1, y1 = BT.tile_bounds(i, j)
        cell = BT.TILE_M / (BT.N - 1)
        c, r, n = int((x - x0) / cell), int((y1 - y) / cell), int(half / cell)
        if c - n < 0 or r - n < 0 or c + n >= BT.N or r + n >= BT.N:
            return None
        return g[r - n:r + n + 1, c - n:c + n + 1], x0 + (c - n) * cell, y1 - (r - n) * cell, cell


def sample(P, pts):
    """패치 P 에서 점들 높이 (쌍선형)."""
    g, x0, yt, cell = P
    fx = (pts[:, 0] - x0) / cell
    fy = (yt - pts[:, 1]) / cell
    c0 = np.clip(fx.astype(int), 0, g.shape[1] - 2)
    r0 = np.clip(fy.astype(int), 0, g.shape[0] - 2)
    tx, ty = fx - c0, fy - r0
    return (g[r0, c0] * (1 - tx) * (1 - ty) + g[r0, c0 + 1] * tx * (1 - ty) + g[r0 + 1, c0] * (1 - tx) * ty
            + g[r0 + 1, c0 + 1] * tx * ty)


def turn_path(net, rin, rout, node):
    """진입 도로 rin 의 node 쪽 끝 TURN_HALF_M + 진출 도로 rout 의 node 쪽 시작 TURN_HALF_M → (점들 N×2, 꼭짓점 번호)."""
    a, b = net.roads[rin], net.roads[rout]
    sa = np.arange(TURN_HALF_M, 0, -0.5)
    pin = [net.at(rin, a.length - s if a.b == node else s) for s in sa]
    pout = [net.at(rout, s if b.a == node else b.length - s) for s in np.arange(0.0, TURN_HALF_M + 1e-9, 0.5)]
    pts = np.array([(p.x, p.y) for p in pin + pout])
    return pts, len(pin)


def metrics(P, pts, apex, Pb=None):
    from scipy.ndimage import gaussian_filter
    d = np.gradient(pts, axis=0)
    d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
    nrm = np.stack([-d[:, 1], d[:, 0]], 1)                       # 진행 왼쪽
    offs = np.arange(-CORR_M, CORR_M + 1e-9, 0.5)
    corr = (pts[:, None, :] + offs[None, :, None] * nrm[:, None, :]).reshape(-1, 2)
    g, x0, yt, cell = P
    hf = g - gaussian_filter(g, 2.0 / cell, mode="nearest")
    hf_c = sample((hf, x0, yt, cell), corr)
    z = sample(P, pts)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    gr = np.gradient(z, s)
    k = int(10 / 0.5)
    sag = np.abs(gr[k:] - gr[:-k]).max() if len(gr) > k else 0.0
    # 회전 방향: 진입 방향 × 진출 방향 (+ = 왼쪽 회전)
    din, dout = pts[apex - 1] - pts[0], pts[-1] - pts[apex]
    left = np.sign(din[0] * dout[1] - din[1] * dout[0])
    lo, hi = max(apex - 10, 0), min(apex + 10, len(pts) - 1)
    zl = sample(P, pts[lo:hi] + 3 * nrm[lo:hi])
    zr = sample(P, pts[lo:hi] - 3 * nrm[lo:hi])
    cross = (zl - zr) / 6.0                                          # + = 왼쪽이 높음
    bank = float(np.mean(-cross * left)) * 100                       # + = 안쪽이 낮음
    out = {"hf_rms_mm": round(float(np.sqrt(np.mean(hf_c ** 2))) * 1000, 1),
           "hf_max_mm": round(float(np.abs(hf_c).max()) * 1000, 1),
           "bank_pct": round(bank, 2), "sag_pp": round(float(sag) * 100, 2),
           "grade_in_pct": round(float((z[apex] - z[max(apex - 40, 0)]) / max(s[apex] - s[max(apex - 40, 0)], 1e-6)) * 100, 2)}
    if Pb is not None:
        out["dz_max_m"] = round(float(np.abs(sample(P, corr) - sample(Pb, corr)).max()), 3)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    Gb, Ga = Grid(a.before), Grid(a.after)
    ck_a = json.loads((Path(a.after) / "static_checks.json").read_text(encoding="utf-8"))["per_road"]
    ck_b = json.loads((Path(a.before) / "static_checks.json").read_text(encoding="utf-8"))["per_road"]
    inc = [r for r in ck_a if "error" not in ck_a[r] and r in ck_b and "error" not in ck_b[r]]
    net = RoadNetwork()
    inc = [r for r in inc if r in net.roads]
    jn = BT.junction_nodes(net, inc)
    by_node = {}
    for r in inc:
        for n in (net.roads[r].a, net.roads[r].b):
            by_node.setdefault(n, []).append(r)
    rows = []
    for node in jn:
        roads = [r for r in by_node.get(node, []) if net.roads[r].length >= MIN_OUT_M]
        x, y = net.nodes[node]
        Pa, Pb = Ga.patch(x, y, PATCH_M), Gb.patch(x, y, PATCH_M)
        if Pa is None or Pb is None:
            continue
        for rin in roads:
            if net.roads[rin].length < MIN_ROAD_M:
                continue
            ow_in = net.roads[rin].attrs.get("oneway")
            if ow_in == "A_TO_B" and net.roads[rin].b != node or ow_in == "B_TO_A" and net.roads[rin].a != node:
                continue
            for rout in roads:
                if rout == rin:
                    continue
                ow = net.roads[rout].attrs.get("oneway")
                if ow == "A_TO_B" and net.roads[rout].a != node or ow == "B_TO_A" and net.roads[rout].b != node:
                    continue
                pts, apex = turn_path(net, rin, rout, node)
                din, dout = pts[apex - 1] - pts[apex - 11], pts[apex + 10] - pts[apex]
                ang = abs(math.degrees(math.atan2(din[0] * dout[1] - din[1] * dout[0], din @ dout)))
                if not (MIN_TURN_DEG <= ang <= MAX_TURN_DEG):
                    continue
                ma, mb = metrics(Pa, pts, apex, Pb), metrics(Pb, pts, apex)
                rows.append({"node": node, "in": rin, "out": rout, "turn_deg": round(ang, 1), "xy": [round(x, 1), round(y, 1)],
                             "after": ma, "before": mb,
                             "d_hf_rms_mm": round(ma["hf_rms_mm"] - mb["hf_rms_mm"], 1),
                             "d_bank_pct": round(ma["bank_pct"] - mb["bank_pct"], 2),
                             "d_sag_pp": round(ma["sag_pp"] - mb["sag_pp"], 2)})
    Path(a.out).write_text(json.dumps({"before": a.before, "after": a.after, "turns": rows}, ensure_ascii=False, indent=0),
                           encoding="utf-8")
    print("교차로", len(jn), "회전", len(rows))


if __name__ == "__main__":
    main()
