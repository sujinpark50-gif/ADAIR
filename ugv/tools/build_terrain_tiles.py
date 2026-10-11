# -*- coding: utf-8 -*-
"""ugv/tools/build_terrain_tiles.py — 트윈 전체 지형을 1 km 타일 높이장으로 (B 월드 확장).

    python -m ugv.tools.build_terrain_tiles                    # 전체 타일 → logs/ugv/stage5/tiles (git 밖, 수백 MB)
    python -m ugv.tools.build_terrain_tiles --only T07_10,T08_10
    python -m ugv.tools.build_terrain_tiles --world b_zone1 --tiles T17_04,T18_04,T17_05,T18_05   # 구역 월드 SDF
    python -m ugv.tools.build_terrain_tiles --check            # 생성된 타일(실제 충돌면 값)로 정적 검사만

방식 — ugv/tools/build_terrain_world.py (구역 하나 = 높이장 하나) 와 같은 면을 **위치만의 함수**로 정의하고 타일마다 계산한다.
그래서 이웃 타일이 공유하는 가장자리 격자점은 같은 값이다 (높이장을 크게 늘리지 않고 해상도 약 0.98 m 를 유지).
- 지형 D(x, y) = DEM 쌍선형 (셀 중심 기준, DEM 밖은 nan). 지역 평면 → EPSG:5186 은 타일마다 5×5 점 정확 변환으로 맞춘 아핀 (잔차 기록).
- 도로면 Z(x, y) = DEM 을 10 m 격자로 쌍선형 표본 → 가우스 σ 40 m → 쌍선형. 모든 도로·교차로가 같은 면 → 평면교차로 턱 없음.
- 도로 띠 = 포함 도로 중심선 ± est_half_width (차로 수 × 3.25 / 2 + 0.5 m, 완화 없음). 띠 안 = Z, 띠 밖 BLEND_M 까지 Z → D 선형 전이, 그 밖 = D.
- 제외 (도로면을 만들지 않고 지형 D 그대로): 시설 코드 FACIL_KIND 1·2 (교량·터널 추정 — 높이·연결 미확인, 임의 생성 안 함),
  자동차전용(AUTO_EXCLU, 입체교차 정보 없음), DEM 밖에 걸친 링크.
- 16 bit PNG 양자화: 타일마다 실제 높이 범위 / 65535 (수 mm). 이웃 가장자리 차이 ≤ 두 간격 절반 합 (static_checks 가 잰다).
  (모든 타일 공통 1 cm·세로 655 m 는 이음매 0 이었지만 Gazebo 높이장 충돌이 깨져 차량이 튕겨 나갔다)
- 90 m DEM 을 보간한 면이다. 고해상도 측량처럼 보이지만 지형 세부(사면·하천·옹벽)는 없다 (메타데이터에 남긴다).
결과: <out>/<타일>/{heightmap.png, model.config, model.sdf}, <out>/tiles.json (타일·제외 도로·생성 시간), --check 는 static_checks.json.
"""

import argparse
import json
import os
import math
import time
from pathlib import Path

import numpy as np

from ugv.drive_graph import est_half_width
from ugv.geo import FRAME
from ugv.roads import RoadNetwork
from ugv.terrain.dem import _K5186, _dem
from ugv.tools.build_terrain_world import WORLD_ORIGIN_ALT, write_png16

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(os.getenv("ADAIR_TILES_OUT") or (ROOT / "logs" / "ugv" / "stage5" / "tiles"))   # 다른 폴더로 만들어 비교할 때 (기존 타일 보존)
TILE_M = 1000.0
N = 1025                     # 2^10 + 1 → 0.977 m 간격
Q = 0.01                     # 높이 양자화 간격 (m) — 모든 타일 공통
BLEND_M = 12.0               # build_terrain_world 와 같음
JUNCTION_MODE = os.getenv("ADAIR_JUNCTION_MODE", "terrain")   # terrain: 교차로 안 = 다듬은 지형 면 Z / level: 도로 중심선 고도 혼합을 평활 (횡단 수평 유지)
JUNC_LEVEL_SIGMA_M = float(os.getenv("ADAIR_JUNC_LEVEL_SIGMA_M", "0"))          # level 모드 교차로 면 평활 (둔덕 제거). 2026-10-11 표본 25: σ2 교차로 횡경사 중앙값 4.0 %, σ4 4.5 %, σ7 6.0 % (σ7 은 턱 0.25 m)
JUNC_IN_M, JUNC_OUT_M = 15.0, 30.0   # 교차로 노드에서 JUNC_IN_M 안은 다듬은 지형 면 Z, JUNC_OUT_M 밖은 중심선 고도(횡단 평평), 사이는 연속 전이.
                                     # 교차로에서 여러 도로 중심선 고도를 섞으면 둔덕(2 m 간격 2차 차분 p95 0.22 m)이 생겨 회전 중 바퀴가 들려
                                     # 미끄러졌다 (2026-10-11 Gazebo: 요율/기구학 하위 10 % 0.28, 단일 높이장 b1 월드 0.44~0.61)
SIGMA_M = 40.0
Z10_STEP = 10.0
X0, Y0 = -620.0, -10631.0    # 타일 원점 (DEM 범위의 지역 평면 최소 근처)
NX, NY = 28, 22              # 28 km × 22 km (DEM 회전 범위를 덮음)


def tile_name(i, j):
    return f"T{i:02d}_{j:02d}"


def tile_bounds(i, j):
    return X0 + i * TILE_M, Y0 + j * TILE_M, X0 + (i + 1) * TILE_M, Y0 + (j + 1) * TILE_M


def to5186_exact(x, y):
    la, lo = FRAME.to_ll(x, y)
    e, n = _K5186.to_xy(la, lo)
    return e + 200000.0, n + 600000.0


def affine_5186(x0, y0, x1, y1):
    """타일 안 지역 평면 → 5186 아핀 (5×5 점 최소제곱). 반환 (A 2×3, 최대 잔차 m)."""
    xs, ys = np.meshgrid(np.linspace(x0, x1, 5), np.linspace(y0, y1, 5))
    P = np.stack([xs.ravel(), ys.ravel(), np.ones(25)], 1)
    T = np.array([to5186_exact(x, y) for x, y in zip(xs.ravel(), ys.ravel())])
    A, *_ = np.linalg.lstsq(P, T, rcond=None)
    res = np.abs(P @ A - T).max()
    return A.T, float(res)


class Fields:
    def __init__(self):
        r, a = _dem()
        self.r, self.a = r, a
        self.valid = a != r.nodata
        # 도로면 Z: 10 m 격자 DEM 쌍선형 표본 → 가우스
        from scipy.ndimage import gaussian_filter
        ne = int((r.bounds.right - r.bounds.left) / Z10_STEP)
        nn = int((r.bounds.top - r.bounds.bottom) / Z10_STEP)
        E = r.bounds.left + Z10_STEP / 2 + np.arange(ne) * Z10_STEP
        Nn = r.bounds.top - Z10_STEP / 2 - np.arange(nn) * Z10_STEP
        EE, NN = np.meshgrid(E, Nn)
        d10 = self.dem(EE, NN, clamp=True)
        self.z10 = gaussian_filter(d10, sigma=SIGMA_M / Z10_STEP, mode="nearest")
        self.z10_e0, self.z10_n0 = E[0], Nn[0]

    def dem(self, E, N, clamp=False):
        """DEM 쌍선형 (셀 중심 기준, ugv/terrain/dem.z_at 과 같은 식). clamp=False 면 DEM 밖 nan."""
        r, a = self.r, self.a
        fx = (E - r.bounds.left) / r.res[0] - 0.5
        fy = (r.bounds.top - N) / r.res[1] - 0.5
        outside = (fx < -0.5) | (fy < -0.5) | (fx > r.width - 0.5) | (fy > r.height - 0.5)
        fx = np.clip(fx, 0, r.width - 1 - 1e-9)
        fy = np.clip(fy, 0, r.height - 1 - 1e-9)
        c0, r0 = np.floor(fx).astype(int), np.floor(fy).astype(int)
        tx, ty = fx - c0, fy - r0
        c1, r1 = np.minimum(c0 + 1, r.width - 1), np.minimum(r0 + 1, r.height - 1)
        v = (a[r0, c0] * (1 - tx) * (1 - ty) + a[r0, c1] * tx * (1 - ty) + a[r1, c0] * (1 - tx) * ty + a[r1, c1] * tx * ty)
        if not clamp:
            v = np.where(outside, np.nan, v)
        return v

    def road_z(self, E, N):
        from scipy.ndimage import map_coordinates
        col = (E - self.z10_e0) / Z10_STEP
        row = (self.z10_n0 - N) / Z10_STEP
        return map_coordinates(self.z10, [row, col], order=1, mode="nearest")


def road_sets(net, F):
    inc, exc = [], {}
    for rid, r in net.roads.items():
        a = r.attrs
        if str(a.get("facil_kind")) in ("1", "2"):
            exc[rid] = f"FACIL_KIND_{a.get('facil_kind')} (교량·터널 추정 — 높이·연결 미확인, 도로면 만들지 않음)"
            continue
        if a.get("motorway"):
            exc[rid] = "MOTORWAY (자동차전용, 입체교차 정보 없음 — 도로면 만들지 않음, 통행 안 함)"
            continue
        xy = np.array(r.xy)
        E, Nn = zip(*[to5186_exact(x, y) for x, y in xy])
        d = F.dem(np.array(E), np.array(Nn))
        if np.isnan(d).any():
            exc[rid] = "OUTSIDE_DEM (DEM 범위 밖 꼭짓점)"
            continue
        inc.append(rid)
    return inc, exc


_CORNER = {}


def corner_5186(i, j):
    """타일 모서리 격자점 (i, j) 의 정확 변환 — 이웃 타일이 같은 값을 쓴다."""
    c = _CORNER.get((i, j))
    if c is None:
        c = _CORNER[(i, j)] = to5186_exact(X0 + i * TILE_M, Y0 + j * TILE_M)
    return c


def tile_5186(i, j, X, Y):
    """타일 안 지역 평면 → 5186: 네 모서리 정확 변환의 쌍선형. 공유 가장자리에서는 그 가장자리 두 모서리만으로 같은 식이 되어
    이웃 타일과 값이 비트까지 같다 (타일마다 맞춘 아핀은 0.6 mm 달라 1 cm 양자화에서 한 칸씩 어긋났다). 반환 (E, N, 최대 잔차 m)."""
    x0, y0 = X0 + i * TILE_M, Y0 + j * TILE_M
    tx, ty = (X - x0) / TILE_M, (Y - y0) / TILE_M
    c00, c10, c01, c11 = corner_5186(i, j), corner_5186(i + 1, j), corner_5186(i, j + 1), corner_5186(i + 1, j + 1)
    E = (1 - ty) * ((1 - tx) * c00[0] + tx * c10[0]) + ty * ((1 - tx) * c01[0] + tx * c11[0])
    Nn = (1 - ty) * ((1 - tx) * c00[1] + tx * c10[1]) + ty * ((1 - tx) * c01[1] + tx * c11[1])
    res = 0.0
    for fx, fy in ((0.5, 0.5), (0.25, 0.75), (0.75, 0.25)):
        e, n = to5186_exact(x0 + fx * TILE_M, y0 + fy * TILE_M)
        ee = (1 - fy) * ((1 - fx) * c00[0] + fx * c10[0]) + fy * ((1 - fx) * c01[0] + fx * c11[0])
        nn = (1 - fy) * ((1 - fx) * c00[1] + fx * c10[1]) + fy * ((1 - fx) * c01[1] + fx * c11[1])
        res = max(res, math.hypot(e - ee, n - nn))
    return E, Nn, res


DENSE_M = 4.0
_DENSE = {}


def road_dense(net, F, rid):
    """도로 중심선 4 m 이하 간격 점과 그 점의 도로면 고도 Z (한 번만 계산 — 모든 타일이 같은 값). 횡단은 이 중심선 고도로 평평."""
    d = _DENSE.get(rid)
    if d is None:
        xy = net.roads[rid].xy
        pts = []
        for (ax, ay), (bx, by) in zip(xy, xy[1:]):
            n = max(1, math.ceil(math.hypot(bx - ax, by - ay) / DENSE_M))
            pts += [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(n)]
        pts.append(tuple(xy[-1]))
        EN = np.array([to5186_exact(x, y) for x, y in pts])
        z = F.road_z(EN[:, 0], EN[:, 1])
        flat = net.roads[rid].attrs.get("flat_at_node")
        if flat:
            # 시험 전용 회전 공터: 붙는 노드의 도로면 고도로 수평 (깎고 메운 평탄면)
            fe, fn = to5186_exact(*net.nodes[flat])
            z = np.full(len(pts), float(F.road_z(np.array([fe]), np.array([fn]))[0]))
        d = _DENSE[rid] = (np.array(pts), z)
    return d


_JN = {}


def junction_nodes(net, inc):
    """포함 도로가 셋 이상 만나는 노드 (교차로)."""
    key = len(inc)
    v = _JN.get(key)
    if v is None:
        deg = {}
        pads = set()
        for rid in inc:
            r = net.roads[rid]
            for n in (r.a, r.b):
                deg[n] = deg.get(n, 0) + 1
            if r.attrs.get("flat_at_node"):
                pads.add(r.attrs["flat_at_node"])
        # 시험 전용 회전 공터가 붙는 노드는 지형 면 혼합에서 뺀다 (산비탈 지형 면이 섞여 공터가 32 % 로 기울었다) — 중심선 고도 혼합
        v = _JN[key] = [n for n, d in deg.items() if d >= 3 and n not in pads]
    return v


def build_tile(net, F, inc, half_w, i, j, junction_mode=None):
    """높이 = 도로 띠 안: 포함 도로들의 '가장 가까운 중심선 점 고도' 가중 평균 (도로마다 가중 = 띠+전이 폭 밖에서 0 으로
    연속 감소 / (거리+1)²). 도로 하나만 있는 곳은 그 도로 중심선 고도 그대로 → 횡단 평평. 교차로(여러 도로)는 연속 혼합.
    띠 밖 BLEND_M 까지 도로면 → 지형 D 선형 전이, 그 밖 D. (2026-10-10 첫 생성: 띠 안에서 다듬은 지형 면을 그대로 써
    산 사면 도로 횡단 경사 최대 59 %, 10 % 넘는 도로 320개)"""
    x0, y0, x1, y1 = tile_bounds(i, j)
    xs = x0 + np.arange(N) * (TILE_M / (N - 1))
    ys = y1 - np.arange(N) * (TILE_M / (N - 1))          # 행 0 = 북쪽
    cell = xs[1] - xs[0]
    X, Y = np.meshgrid(xs, ys)
    E, Nn, res = tile_5186(i, j, X, Y)
    D = F.dem(E, Nn)
    Z = F.road_z(E, Nn)
    edge = np.full((N, N), np.inf)
    num = np.zeros((N, N))
    den = np.zeros((N, N))
    roads_here = []
    for rid in inc:
        r = net.roads[rid]
        hw = half_w[rid]
        R = hw + BLEND_M
        reach = R + cell
        bx0, by0 = np.min(r.xy, axis=0)
        bx1, by1 = np.max(r.xy, axis=0)
        if bx1 < x0 - reach or bx0 > x1 + reach or by1 < y0 - reach or by0 > y1 + reach:
            continue
        P, Zd = road_dense(net, F, rid)
        # 도로 창 (타일 안)
        J0 = max(0, int((bx0 - reach - x0) / cell))
        J1 = min(N - 1, int((bx1 + reach - x0) / cell) + 1)
        I0 = max(0, int((y1 - (by1 + reach)) / cell))
        I1 = min(N - 1, int((y1 - (by0 - reach)) / cell) + 1)
        if J1 < J0 or I1 < I0:
            continue
        rd = np.full((I1 - I0 + 1, J1 - J0 + 1), np.inf)
        rz = np.zeros(rd.shape)
        hit = False
        for k in range(len(P) - 1):
            (ax, ay), (bx, by) = P[k], P[k + 1]
            if max(ax, bx) < x0 - reach or min(ax, bx) > x1 + reach or max(ay, by) < y0 - reach or min(ay, by) > y1 + reach:
                continue
            j0 = max(J0, int((min(ax, bx) - reach - x0) / cell))
            j1 = min(J1, int((max(ax, bx) + reach - x0) / cell) + 1)
            i0 = max(I0, int((y1 - (max(ay, by) + reach)) / cell))
            i1 = min(I1, int((y1 - (min(ay, by) - reach)) / cell) + 1)
            if j1 < j0 or i1 < i0:
                continue
            hit = True
            sx, sy = X[i0:i1 + 1, j0:j1 + 1], Y[i0:i1 + 1, j0:j1 + 1]
            dx_, dy_ = bx - ax, by - ay
            L2 = dx_ * dx_ + dy_ * dy_
            t = np.clip(((sx - ax) * dx_ + (sy - ay) * dy_) / L2, 0.0, 1.0) if L2 > 0 else np.zeros(sx.shape)
            dd = np.hypot(sx - ax - t * dx_, sy - ay - t * dy_)
            zf = Zd[k] + t * (Zd[k + 1] - Zd[k])
            sub = (slice(i0 - I0, i1 - I0 + 1), slice(j0 - J0, j1 - J0 + 1))
            better = dd < rd[sub]
            rd[sub] = np.where(better, dd, rd[sub])
            rz[sub] = np.where(better, zf, rz[sub])
        if not hit:
            continue
        roads_here.append(rid)
        w = np.clip((R - rd) / BLEND_M, 0.0, 1.0) / (rd + 1.0) ** 2
        win = (slice(I0, I1 + 1), slice(J0, J1 + 1))
        num[win] += w * rz
        den[win] += w
        edge[win] = np.minimum(edge[win], rd - hw)
    road_mask = edge <= 0
    S = np.where(den > 0, num / np.where(den > 0, den, 1.0), Z)
    # 교차로 근처는 다듬은 지형 면 Z (모든 도로가 같은 연속 면 → 둔덕 없음), 멀어지면 중심선 고도로 연속 전이
    dn = np.full((N, N), np.inf)
    for nid in junction_nodes(net, inc):
        nx, ny = net.nodes[nid]
        if nx < x0 - JUNC_OUT_M or nx > x1 + JUNC_OUT_M or ny < y0 - JUNC_OUT_M or ny > y1 + JUNC_OUT_M:
            continue
        j0, j1 = max(0, int((nx - JUNC_OUT_M - x0) / cell)), min(N - 1, int((nx + JUNC_OUT_M - x0) / cell) + 1)
        i0, i1 = max(0, int((y1 - (ny + JUNC_OUT_M)) / cell)), min(N - 1, int((y1 - (ny - JUNC_OUT_M)) / cell) + 1)
        if j1 < j0 or i1 < i0:
            continue
        sub = (slice(i0, i1 + 1), slice(j0, j1 + 1))
        dn[sub] = np.minimum(dn[sub], np.hypot(X[sub] - nx, Y[sub] - ny))
    tj = np.clip((dn - JUNC_IN_M) / (JUNC_OUT_M - JUNC_IN_M), 0.0, 1.0)
    tj = tj * tj * (3 - 2 * tj)                         # 매끈한 전이 (기울기 연속)
    if (junction_mode or JUNCTION_MODE) == "level":
        # 교차로 안도 도로 중심선 고도 혼합(횡단 수평)을 쓰고, 도로마다 고도가 달라 생기는 둔덕만 공간 평활로 없앤다.
        # terrain 모드는 지형 면 Z 를 써 산비탈 교차로가 원지형 횡경사(중앙값 10.6 %)를 물려받았다 (2026-10-11 지도 대조)
        from scipy.ndimage import gaussian_filter
        # σ 0 이면 평활 없음 — 값이 위치만으로 정해져 타일 이음매가 맞는다 (σ>0 은 타일마다 따로 평활해 이음매 최대 0.73 m 어긋남)
        Sl = gaussian_filter(S, sigma=JUNC_LEVEL_SIGMA_M / cell, mode="nearest") if JUNC_LEVEL_SIGMA_M > 0 else S
        S = Sl * (1 - tj) + S * tj
    else:
        S = Z * (1 - tj) + S * tj
    H = D.copy()
    H[road_mask] = S[road_mask]
    bm = (~road_mask) & (edge <= BLEND_M)
    t = edge[bm] / BLEND_M
    H[bm] = S[bm] * (1 - t) + D[bm] * t
    outside = np.isnan(H)
    if outside.any():
        H[outside] = Z[outside]                 # DEM 밖: 다듬은 면으로 채움 (도로 없음 — 포함 도로는 DEM 안만)
    return {"H": H, "D": D, "road_mask": road_mask, "outside_dem": outside, "roads": roads_here, "affine_res_m": res,
            "xs": xs, "ys": ys}


def quantize(H):
    """타일마다 실제 높이 범위로 16 bit 양자화 (간격 = 범위 / 65535, 범위 110 m 면 1.7 mm). 반환 (최저, 범위, 영상).
    모든 타일을 같은 간격(1 cm, 세로 크기 655 m)으로 맞추면 공유 가장자리 정수가 같았지만, 그 세로 크기에서 Gazebo(DART) 높이장
    충돌이 깨져 차량이 출발하자마자 튕겨 나갔다 (2026-10-11: 같은 타일 하나를 세로 655 m → 튕김, 실제 범위 110 m → 정상 주행).
    대신 이웃 타일 가장자리 차이는 두 간격 절반 이하 (수 mm) — static_checks 가 실제 값으로 잰다."""
    zmin = float(np.nanmin(H))
    span = max(float(np.nanmax(H)) - zmin, 1.0)
    return zmin, span, np.round((H - zmin) / span * 65535).astype(np.uint16)


def write_tile(name, zmin, span, img, i, j):
    d = OUT / name
    d.mkdir(parents=True, exist_ok=True)
    write_png16(d / "heightmap.png", img)
    x0, y0, x1, y1 = tile_bounds(i, j)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    model = f"kangwon_tile_{name}"
    (d / "model.config").write_text(f"""<?xml version="1.0"?>
<model><name>{model}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>트윈 지형 타일 {name} (ugv/tools/build_terrain_tiles.py 생성, 90 m DEM 보간 — 측량 자료 아님)</description></model>
""", encoding="utf-8")
    geo = (f"<heightmap><uri>model://{name}/heightmap.png</uri><size>{TILE_M:.3f} {TILE_M:.3f} {span:.4f}</size>"
           f"<pos>0 0 0</pos></heightmap>")
    (d / "model.sdf").write_text(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model}">
    <static>true</static>
    <pose>{cx:.3f} {cy:.3f} {zmin - WORLD_ORIGIN_ALT:.3f} 0 0 0</pose>
    <link name="link">
      <collision name="collision"><geometry>{geo}</geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="visual"><geometry>{geo}</geometry></visual>
    </link>
  </model>
</sdf>
""", encoding="utf-8")


def build_all(only=None):
    net = RoadNetwork()
    t0 = time.time()
    F = Fields()
    inc, exc = road_sets(net, F)
    half_w = {rid: est_half_width(net.roads[rid]) for rid in inc}
    t_fields = time.time() - t0
    OUT.mkdir(parents=True, exist_ok=True)
    meta_p = OUT / "tiles.json"
    meta = json.loads(meta_p.read_text(encoding="utf-8")) if meta_p.exists() else {"tiles": {}}
    meta.update({"tile_m": TILE_M, "grid": N, "cell_m": round(TILE_M / (N - 1), 4), "q_m": Q, "origin_xy": [X0, Y0],
                 "nx": NX, "ny": NY, "blend_m": BLEND_M, "road_surface_sigma_m": SIGMA_M,
                 "world_origin_alt_m": WORLD_ORIGIN_ALT, "roads_included": len(inc), "roads_excluded": exc,
                 "road_width": "est_half_width = 차로 수 × 3.25 / 2 + 0.5 m (완화 없음)", "fields_s": round(t_fields, 1),
                 "not_verified": ["교량·터널·입체교차 (FACIL_KIND 1·2·자동차전용 도로면 없음)", "DEM 수직 기준 미확인",
                                  "90 m DEM 보간 — 사면·하천·옹벽 세부 없음", "편경사 없음 (도로면 = 다듬은 지형 면)"]})
    import tracemalloc
    for i in range(NX):
        for j in range(NY):
            name = tile_name(i, j)
            if only and name not in only:
                continue
            ts = time.time()
            tracemalloc.start()
            T = build_tile(net, F, inc, half_w, i, j)
            if T["outside_dem"].all():
                tracemalloc.stop()
                continue
            zmin, span, img = quantize(T["H"])
            write_tile(name, zmin, span, img, i, j)
            _, peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()
            rm = T["road_mask"]
            cut = (T["D"] - T["H"])[rm & ~np.isnan(T["D"])]
            meta["tiles"][name] = {
                "i": i, "j": j, "bounds": list(tile_bounds(i, j)), "zmin": zmin, "span": span, "q_m": round(span / 65535, 5),
                "z_range_m": round(float(T["H"].max() - T["H"].min()), 2), "roads": sorted(T["roads"]),
                "road_cells": int(rm.sum()), "outside_dem_cells": int(T["outside_dem"].sum()),
                "dem_minus_surface_in_road_m": None if cut.size == 0 else
                {"max_cut": round(float(cut.max()), 2), "max_fill": round(float(-cut.min()), 2),
                 "p95_abs": round(float(np.percentile(np.abs(cut), 95)), 2)},
                "affine_res_m": round(T["affine_res_m"], 4), "build_s": round(time.time() - ts, 2),
                "peak_mem_mb": round(peak / 1e6, 1)}
            print(name, meta["tiles"][name]["build_s"], "s roads", len(T["roads"]), flush=True)
    meta["total_s"] = round(time.time() - t0, 1)
    meta_p.write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    return meta


# ---------------------------------------------------------------- 실제 충돌면(생성된 PNG) 정적 검사
def read_png16(path):
    """write_png16 이 쓴 16 bit 회색조 PNG 읽기 (필터 0 만 — 추가 패키지 없음). 실제 파일 값을 그대로 검사한다."""
    import struct
    import zlib
    data = Path(path).read_bytes()
    assert data[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    pos, idat = 8, b""
    w = h = 0
    while pos < len(data):
        ln, tag = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + ln]
        if tag == b"IHDR":
            w, h, bd, ct = struct.unpack(">IIBB", body[:10])
            assert bd == 16 and ct == 0
        elif tag == b"IDAT":
            idat += body
        pos += 12 + ln
    raw = zlib.decompress(idat)
    stride = 1 + 2 * w
    rows = np.frombuffer(raw, dtype=np.uint8).reshape(h, stride)
    assert (rows[:, 0] == 0).all(), "필터 0 이 아닌 PNG"
    return rows[:, 1:].copy().view(">u2").reshape(h, w)


class Surface:
    """생성된 타일 PNG 값으로 표면 높이 (Gazebo 높이장과 같은 격자점, 격자 사이 쌍선형 — Gazebo 는 삼각형 분할이라 근사)."""

    def __init__(self):
        self.meta = json.loads((OUT / "tiles.json").read_text(encoding="utf-8"))
        self.cache = {}

    def grid(self, name):
        g = self.cache.get(name)
        if g is None:
            img = read_png16(OUT / name / "heightmap.png").astype(float)
            t = self.meta["tiles"][name]
            g = t["zmin"] + img / 65535.0 * t["span"]
            self.cache[name] = g
            if len(self.cache) > 40:
                self.cache.pop(next(iter(self.cache)))
        return g

    def z(self, x, y):
        i, j = int((x - X0) // TILE_M), int((y - Y0) // TILE_M)
        name = tile_name(i, j)
        if name not in self.meta["tiles"]:
            return math.nan
        g = self.grid(name)
        x0, y0, x1, y1 = tile_bounds(i, j)
        cell = TILE_M / (N - 1)
        fc, fr = (x - x0) / cell, (y1 - y) / cell
        c0, r0 = min(int(fc), N - 2), min(int(fr), N - 2)
        tx, ty = fc - c0, fr - r0
        return float(g[r0, c0] * (1 - tx) * (1 - ty) + g[r0, c0 + 1] * tx * (1 - ty)
                     + g[r0 + 1, c0] * (1 - tx) * ty + g[r0 + 1, c0 + 1] * tx * ty)


def static_checks():
    net = RoadNetwork()
    S = Surface()
    meta = S.meta
    out = {"basis": "생성된 16 bit PNG 값(실제 충돌면 격자점) + 격자 사이 쌍선형. 표본 검사 — 연속 보장 아님"}
    # 1) 타일 이음매: 공유 가장자리 값이 같은가
    seam = {"pairs": 0, "max_abs_m": 0.0, "mismatch_pairs": []}
    for name, t in meta["tiles"].items():
        for di, dj in ((1, 0), (0, 1)):
            nb = tile_name(t["i"] + di, t["j"] + dj)
            if nb not in meta["tiles"]:
                continue
            a, b = S.grid(name), S.grid(nb)
            ea, eb = (a[:, -1], b[:, 0]) if di else (a[0, :], b[-1, :])
            m = float(np.abs(ea - eb).max())
            tol = (t["q_m"] + meta["tiles"][nb]["q_m"]) / 2 + 1e-6
            seam["pairs"] += 1
            seam["max_abs_m"] = max(seam["max_abs_m"], round(m, 5))
            if m > tol:
                seam["mismatch_pairs"].append([name, nb, round(m, 5), round(tol, 5)])
    out["tile_seams"] = seam
    # 2) 도로 띠 안 격자 경사 (실제 최대 경사) — 타일별
    cell = TILE_M / (N - 1)
    grade = {}
    # 3) 도로별 종단·횡단 경사 (중심선 2 m 간격, 횡단 = ±반폭 지점 높이 차)
    inc = [rid for rid in net.roads if rid not in meta["roads_excluded"]]
    per_road, worst_long, worst_cross = {}, [], []
    jn = junction_nodes(net, inc)
    jxy = np.array([net.nodes[n] for n in jn]) if jn else np.zeros((0, 2))
    for rid in inc:
        r = net.roads[rid]
        hw = est_half_width(r)
        L = r.length
        ss = np.arange(0.0, L + 1e-9, 2.0)
        if ss[-1] < L:
            ss = np.append(ss, L)
        zc, zl, zr = [], [], []
        for s in ss:
            p = net.at(rid, s)
            q = net.at(rid, min(L, s + 0.5)) if s < L else net.at(rid, max(0.0, s - 0.5))
            tx, ty = (q.x - p.x), (q.y - p.y)
            if s >= L:
                tx, ty = -tx, -ty
            nrm = math.hypot(tx, ty) or 1.0
            nx, ny = -ty / nrm, tx / nrm
            zc.append(S.z(p.x, p.y))
            zl.append(S.z(p.x + nx * hw, p.y + ny * hw))
            zr.append(S.z(p.x - nx * hw, p.y - ny * hw))
        zc, zl, zr = np.array(zc), np.array(zl), np.array(zr)
        if np.isnan(zc).any():
            per_road[rid] = {"error": "타일 없음"}
            continue
        # 종단: 10 m 창 (2 m 표본의 양자화 잡음 줄임)
        k = 5
        g_long = np.abs(zc[k:] - zc[:-k]) / (ss[k:] - ss[:-k]) if len(ss) > k else np.abs(np.diff(zc) / np.diff(ss))
        g_cross_all = np.abs(zl - zr) / (2 * hw)
        # 횡단: 교차로 구역(노드 JUNC_OUT_M 안 — 여러 방향 도로가 만나 다듬은 지형 면을 그대로 씀) 밖만. 교차로 구역은 따로
        cxy = np.array([net.at(rid, v)[2:4] for v in ss])
        if len(jxy):
            dj = np.sqrt(((cxy[:, None, :] - jxy[None]) ** 2).sum(2)).min(1)
        else:
            dj = np.full(len(ss), np.inf)
        g_cross = g_cross_all[dj > JUNC_OUT_M] if (dj > JUNC_OUT_M).any() else np.zeros(1)
        g_cross_j = g_cross_all[dj <= JUNC_OUT_M] if (dj <= JUNC_OUT_M).any() else np.zeros(1)
        # 표면 급변: 2 m 간격 높이 2차 차분 (턱)
        jump = np.abs(np.diff(zc, 2)) if len(zc) > 2 else np.zeros(1)
        per_road[rid] = {"len_m": round(L, 1), "max_long_grade": round(float(g_long.max()), 4) if g_long.size else 0.0,
                         "max_cross_slope": round(float(g_cross.max()), 4),
                         "max_cross_slope_junction_zone": round(float(g_cross_j.max()), 4),
                         "max_second_diff_m": round(float(jump.max()), 3),
                         "z_range_m": round(float(zc.max() - zc.min()), 1)}
    vals = [v for v in per_road.values() if "error" not in v]
    out["roads_checked"] = len(vals)
    out["roads_missing_tile"] = [r for r, v in per_road.items() if "error" in v]
    for key in ("max_long_grade", "max_cross_slope", "max_second_diff_m"):
        arr = np.array([v[key] for v in vals])
        out[key] = {"max": round(float(arr.max()), 4), "p95": round(float(np.percentile(arr, 95)), 4),
                    "over": {thr: int((arr > thr).sum()) for thr in ((0.08, 0.10, 0.12, 0.15, 0.20) if "grade" in key else
                                                                     (0.02, 0.04, 0.06, 0.10) if "cross" in key else
                                                                     (0.02, 0.05, 0.10))}}
    out["worst_long"] = sorted(((v["max_long_grade"], r) for r, v in per_road.items() if "error" not in v), reverse=True)[:15]
    out["worst_cross"] = sorted(((v["max_cross_slope"], r) for r, v in per_road.items() if "error" not in v), reverse=True)[:15]
    # 4) 교차로 고도: 노드에서 붙은 포함 도로 각각 2 m 안쪽 점 높이와 노드 높이 차
    spread = []
    for n, arms in net.adj.items():
        arms = [(rid, o) for rid, o in arms if rid in per_road and "error" not in per_road[rid]]
        if len(arms) < 2:
            continue
        x, y = net.nodes[n]
        z0 = S.z(x, y)
        zs = []
        for rid, _ in arms:
            r = net.roads[rid]
            s = 2.0 if r.a == n else r.length - 2.0
            p = net.at(rid, max(0.0, min(r.length, s)))
            zs.append(S.z(p.x, p.y))
        spread.append((max(abs(z - z0) for z in zs), n))
    spread.sort(reverse=True)
    out["junction_2m_dz"] = {"nodes": len(spread), "max_m": round(spread[0][0], 3) if spread else None,
                             "p95_m": round(float(np.percentile([s for s, _ in spread], 95)), 3) if spread else None,
                             "worst": [[round(s, 3), n] for s, n in spread[:10]]}
    out["per_road"] = per_road
    (OUT / "static_checks.json").write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    return out


def write_world(name, tiles, spawn_note=""):
    meta = json.loads((OUT / "tiles.json").read_text(encoding="utf-8"))
    missing = [t for t in tiles if t not in meta["tiles"]]
    if missing:
        raise SystemExit(f"없는 타일: {missing}")
    inc = sorted({r for t in tiles for r in meta["tiles"][t]["roads"]})
    # 구역 안 도로 = 모든 꼭짓점이 구역 타일 안 (경계에서 25 m 안쪽)
    net = RoadNetwork()
    boxes = [tile_bounds(meta["tiles"][t]["i"], meta["tiles"][t]["j"]) for t in tiles]

    def in_union(x, y):
        return any(b[0] <= x <= b[2] and b[1] <= y <= b[3] for b in boxes)

    def inside(x, y, m=25.0):
        """구역(타일 합집합) 바깥 경계에서 m 넘게 안쪽 (구역 안 타일 이음매는 경계가 아니다)."""
        return all(in_union(x + dx, y + dy) for dx, dy in ((0, 0), (m, 0), (-m, 0), (0, m), (0, -m), (m, m), (m, -m), (-m, m), (-m, -m)))
    subset = [r for r in inc if all(inside(x, y) for x, y in net.roads[r].xy)]
    includes = "\n".join(f"    <include><uri>model://{t}</uri></include>" for t in tiles)
    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- {name}: 트윈 지형 타일 {len(tiles)}개 ({', '.join(tiles)}). 생성: python -m ugv.tools.build_terrain_tiles --world {name}
     충돌 = 타일 높이장 (1 km, {N}×{N}, {TILE_M / (N - 1):.3f} m), 이음매 공유 격자점 값 같음. 90 m DEM 보간 — 측량 자료 아님. {spawn_note} -->
<sdf version="1.9">
  <world name="{name}">
    <physics type="ode"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate></physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene><grid>false</grid><ambient>0.4 0.4 0.4 1</ambient><background>0.62 0.78 0.92 1</background><shadows>false</shadows></scene>
{includes}
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose><cast_shadows>false</cast_shadows><intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction><diffuse>0.904 0.904 0.904 1</diffuse><specular>0.271 0.271 0.271 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>38.0112</latitude_deg><longitude_deg>128.122643</longitude_deg><elevation>{WORLD_ORIGIN_ALT}</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""
    (OUT / f"{name}.sdf").write_text(sdf, encoding="utf-8")
    (OUT / f"road_subset_{name}.json").write_text(json.dumps({"world": name, "tiles": tiles, "roads": subset}, indent=1),
                                                  encoding="utf-8")
    print(OUT / f"{name}.sdf", "도로", len(subset))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="", help="쉼표로 타일 이름")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--world", default=None)
    ap.add_argument("--tiles", default="")
    a = ap.parse_args()
    if a.world:
        write_world(a.world, [t for t in a.tiles.split(",") if t])
    elif a.check:
        out = static_checks()
        print(json.dumps({k: v for k, v in out.items() if k != "per_road"}, ensure_ascii=False, indent=1))
    else:
        meta = build_all(set(t for t in a.only.split(",") if t) or None)
        print("타일", len(meta["tiles"]), "총", meta["total_s"], "s")


if __name__ == "__main__":
    main()
