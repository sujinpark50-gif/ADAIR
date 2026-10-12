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
# level 모드에서 교차로별로 terrain 혼합을 쓸 노드 (붙은 도로들이 level 에서 더 나빠지는 교차로 — 2026-10-11 비교로 고름)
# 설계형 교차로면 (D 후보, 지정 노드만 — 시험용): 주도로 우선(주도로 중심선 종단을 교차로 안으로 연장, 부도로는 15~30 m 에서
# 연결) 또는 같은 위계면 진입부 점들에 맞춘 기울어진 공통 평면. 실제 도로 설계 관행(TxDOT 4.8.5 table-topping·주도로 우선,
# Maine DOT 5-1.02) 참고 — 측량 자료 아님. docs/ugv/UGV_도로면_설계기준_적용_20261012.md
JUNC_DESIGN_NODES = set(json.loads(Path(os.environ["ADAIR_JUNC_DESIGN_NODES"]).read_text(encoding="utf-8")))     if os.getenv("ADAIR_JUNC_DESIGN_NODES") else set()
DESIGN_STRAIGHT_DEG = 135.0     # 주도로 쌍: 두 도로가 이 각도 이상으로 이어질 때 (직진 쌍)
DESIGN_PLANE_RING = (15.0, 25.0)
DESIGN_WARP_GRADE = 0.04        # 연결(비틀기)이 더하는 종단 경사 상한 (Maine DOT 신호교차로 4 %, 일시정지 6 %) — 매끈한 전이 최대 기울기 1.5·Δ/L
DESIGN_OUT_MAX_M = 60.0         # 전이 바깥 반경 상한 — 넘으면 그 노드는 설계면 후보 실패로 기록하고 기존 처리 유지
DESIGN_REPORT = {}              # 노드 → 선택(주도로/평면)·전이 반경·부도로 높이 차 (tiles.json 에 기록)  # 공통 평면 맞춤: 노드에서 이 거리의 진입부 중심선 고도
JUNC_TERRAIN_NODES = set(json.loads(Path(os.environ["ADAIR_JUNC_TERRAIN_NODES"]).read_text(encoding="utf-8")))     if os.getenv("ADAIR_JUNC_TERRAIN_NODES") else set()
# 종단 다듬기를 하지 않을 도로 (다듬으면 최종 표면 검사에서 악화 — 2차 확인으로 되돌림)
LONG_SKIP = set(json.loads(Path(os.environ["ADAIR_LONG_SKIP"]).read_text(encoding="utf-8"))) if os.getenv("ADAIR_LONG_SKIP") else set()
JUNC_IN_M, JUNC_OUT_M = 15.0, 30.0   # 교차로 노드에서 JUNC_IN_M 안은 다듬은 지형 면 Z, JUNC_OUT_M 밖은 중심선 고도(횡단 평평), 사이는 연속 전이.
                                     # 교차로에서 여러 도로 중심선 고도를 섞으면 둔덕(2 m 간격 2차 차분 p95 0.22 m)이 생겨 회전 중 바퀴가 들려
                                     # 미끄러졌다 (2026-10-11 Gazebo: 요율/기구학 하위 10 % 0.28, 단일 높이장 b1 월드 0.44~0.61)
SIGMA_M = 40.0
Z10_STEP = 10.0
X0, Y0 = -620.0, -10631.0    # 타일 원점 (DEM 범위의 지역 평면 최소 근처)
NX, NY = 28, 22              # 28 km × 22 km (DEM 회전 범위를 덮음)


def build_params():
    """생성 설정 (tiles.json params) — 같은 결과를 다시 만들 수 있게 (예전 tiles_level 은 모드·σ 기록이 없어 재현 불가)."""
    import subprocess
    try:
        head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, timeout=10).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "ugv"], cwd=ROOT, capture_output=True, text=True,
                                    timeout=10).stdout.strip())
    except Exception:  # noqa: BLE001
        head, dirty = None, None
    from ugv import config
    return {"junction_mode": JUNCTION_MODE, "junc_level_sigma_m": JUNC_LEVEL_SIGMA_M, "road_surface_sigma_m": SIGMA_M,
            "long_sigma_m": LONG_SIGMA_M, "design_cap": DESIGN_CAP,
            "junc_terrain_nodes": len(JUNC_TERRAIN_NODES), "junc_design_nodes": sorted(JUNC_DESIGN_NODES), "long_skip": len(LONG_SKIP),
            "bridges": {"on": BRIDGES, "max_deck_above_dem_m": BRIDGE_MAX_DECK_ABOVE_DEM_M, "near_road_m": BRIDGE_NEAR_ROAD_M,
                        "near_road_dz_m": BRIDGE_NEAR_ROAD_DZ_M},
            "blend_m": BLEND_M, "git_head": head, "git_dirty_ugv": dirty, "extra_roads": config.EXTRA_ROADS or None}


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


BRIDGES = os.getenv("ADAIR_BRIDGES", "0") == "1"     # 자료 교량(FACIL_KIND=1)을 직선 상판 도로면으로 (가정 — 상판 높이 자료 없음)
BRIDGE_MAX_DECK_ABOVE_DEM_M = 6.0     # 상판이 지형보다 이보다 높으면 성토 표현이 가짜 산이 된다 → 제외 (계곡 고가, 별도 모델 필요)
BRIDGE_NEAR_ROAD_M = 8.0              # 노드를 공유하지 않는 다른 도로가 이 안에 있고
BRIDGE_NEAR_ROAD_DZ_M = 0.5           # 그 자리 상판과 도로면 높이 차가 이보다 크면 제외 (하부·평행 도로와 높이 상충)
_BRIDGE_INC = set()


def _deck(net, F, rid, step=2.0):
    """교량 상판 직선 가정: 양 끝 노드의 도로면 고도 road_z 를 누적 거리로 직선 연결. (점, 누적 s, 상판 z, 그 점 DEM, 그 점 road_z)."""
    r = net.roads[rid]
    s = np.arange(0.0, r.length + 1e-9, step)
    if s[-1] < r.length:
        s = np.append(s, r.length)
    P = np.array([net.at(rid, v)[2:4] for v in s])
    EN = np.array([to5186_exact(x, y) for x, y in P])
    rz = F.road_z(EN[:, 0], EN[:, 1])
    deck = rz[0] + (rz[-1] - rz[0]) * s / s[-1]
    return P, s, deck, F.dem(EN[:, 0], EN[:, 1], clamp=True), rz


def bridge_rule(net, F, rid, normal_inc, near_index):
    """자료 교량 하나를 도로면으로 넣을지 (사유, 가정 기록). 사유가 None 이면 포함."""
    r = net.roads[rid]
    if r.attrs.get("motorway"):
        return "FACIL_KIND_1 MOTORWAY (자동차전용)", None
    xy = np.array(r.xy)
    E, Nn = zip(*[to5186_exact(x, y) for x, y in xy])
    if np.isnan(F.dem(np.array(E), np.array(Nn))).any():
        return "FACIL_KIND_1 OUTSIDE_DEM", None
    P, s, deck, dem, rz = _deck(net, F, rid)
    above = float((deck - dem).max())
    if above > BRIDGE_MAX_DECK_ABOVE_DEM_M:
        return f"BRIDGE_DECK_ABOVE_DEM_{above:.1f}m (성토 표현 부적합 — 계곡 고가, 상판·하부 자료 없음)", None
    near = []
    ends = {r.a, r.b}
    lo, hi = P.min(0) - BRIDGE_NEAR_ROAD_M, P.max(0) + BRIDGE_NEAR_ROAD_M
    worst_dz = 0.0
    for oid, (q, bb0, bb1) in near_index.items():
        o = net.roads[oid]
        if oid == rid or ends & {o.a, o.b} or bb1[0] < lo[0] or bb0[0] > hi[0] or bb1[1] < lo[1] or bb0[1] > hi[1]:
            continue
        d = np.sqrt(((P[:, None, :] - q[None, :, :]) ** 2).sum(-1)).min(1)
        m = d <= BRIDGE_NEAR_ROAD_M
        if m.any():
            dz = float(np.abs(deck[m] - rz[m]).max())
            near.append([oid, round(float(d.min()), 1), round(dz, 2)])
            worst_dz = max(worst_dz, dz)
    if near and worst_dz > BRIDGE_NEAR_ROAD_DZ_M:
        return f"BRIDGE_NEAR_ROAD (하부·평행 도로와 높이 상충 {worst_dz:.2f} m)", None
    return None, {"kind": "BRIDGE_DECK_STRAIGHT", "len_m": round(r.length, 1),
                  "deck_grade": round(abs(deck[-1] - deck[0]) / max(r.length, 1e-6), 4),
                  "deck_above_dem_max_m": round(above, 2), "deck_minus_roadz_max_m": round(float(np.abs(deck - rz).max()), 2),
                  "near_roads": near}


def road_sets(net, F):
    """(포함, 제외 {도로: 사유}, 가정 {도로: 기록}). ADAIR_BRIDGES=1 이면 자료 교량 중 성토로 표현해도 자료와 모순 없는 것을 포함 (가정)."""
    inc, exc, assumed = [], {}, {}
    bridges = []
    for rid, r in net.roads.items():
        a = r.attrs
        if BRIDGES and str(a.get("facil_kind")) == "1":
            bridges.append(rid)
            continue
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
    _BRIDGE_INC.clear()
    if bridges:
        idx = {}
        for oid in inc:
            q = _dense_xy(net.roads[oid].xy, 1.0)
            idx[oid] = (q, q.min(0), q.max(0))
        for rid in bridges:
            why, rec = bridge_rule(net, F, rid, inc, idx)
            if why:
                exc[rid] = why
            else:
                inc.append(rid)
                assumed[rid] = rec
                _BRIDGE_INC.add(rid)
    return inc, exc, assumed


def _dense_xy(xy, step):
    out = [np.asarray(xy[0], float)]
    for a, b in zip(xy[:-1], xy[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.hypot(*(b - a)) / step))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


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
        r_ = net.roads[rid]
        if _NODE_Z and r_.a in _NODE_Z and r_.b in _NODE_Z:
            # 설계 상한 복구로 옮긴 노드 높이에 양 끝을 맞춘다 (누적 거리 비례 선형 보정)
            P0 = np.array(pts)
            s0 = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P0, axis=0).T))]
            f = s0 / max(s0[-1], 1e-9)
            z = z + (_NODE_Z[r_.a] - z[0]) * (1 - f) + (_NODE_Z[r_.b] - z[-1]) * f
        flat = net.roads[rid].attrs.get("flat_at_node")
        if flat:
            # 시험 전용 회전 공터: 붙는 노드의 도로면 고도로 수평 (깎고 메운 평탄면)
            fe, fn = to5186_exact(*net.nodes[flat])
            z = np.full(len(pts), float(_NODE_Z[flat]) if flat in _NODE_Z else float(F.road_z(np.array([fe]), np.array([fn]))[0]))
        elif rid in _BRIDGE_INC:
            # 자료 교량 상판 직선 가정: 양 끝 도로면 고도를 누적 거리 비례로 (분할 간격이 균일하지 않아 번호 비례 아님)
            P = np.array(pts)
            s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
            z = z[0] + (z[-1] - z[0]) * s / max(s[-1], 1e-9)
        elif LONG_SIGMA_M > 0 and rid not in LONG_SKIP:
            P = np.array(pts)
            s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
            z0 = z
            z, used = adaptive_smooth(s, z0, LONG_SIGMA_M)
            _LONG_STATS[rid] = {"sigma_m": used, "max_dev_m": round(float(np.abs(z - z0).max()), 3),
                                "grade300_before": round(grade_window(s, z0, 300.0), 4),
                                "grade300_after": round(grade_window(s, z, 300.0), 4)}
        cap = design_cap(r_) if not flat else None
        if cap:
            P = np.array(pts)
            s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
            g_before = float(np.abs(np.diff(z) / np.maximum(np.diff(s), 1e-9)).max()) if len(z) > 1 else 0.0
            if g_before > cap:
                rec = {"cap": cap, "rank": r_.attrs.get("road_rank"), "grade_before": round(g_before, 4),
                       "grade300_before": round(grade_window(s, z, 300.0), 4)}
                if rid in DESIGN_CAP_SKIP:
                    rec.update(status="NOT_RESTORED", why="복구하면 최종 표면 검사에서 악화 (2차 확인으로 되돌림)")
                elif abs(z[-1] - z[0]) > cap * s[-1] + 1e-6:
                    rec.update(status="NOT_RESTORED", why="끝(노드) 고도차가 상한 × 길이를 넘음 — 노드를 옮겨야 함(기본 끔): 실제 급경사·형상 단순화·노드 높이 조사 대상")
                else:
                    zc = smooth_profile(s, cap_profile(s, z, cap), CAP_CURVE_SIGMA_M)
                    dev = float(np.abs(zc - z).max())
                    jb = float(np.abs(np.diff(np.interp(np.arange(0, s[-1], 2.0), s, z), 2)).max()) if s[-1] > 6 else 0.0
                    ja = float(np.abs(np.diff(np.interp(np.arange(0, s[-1], 2.0), s, zc), 2)).max()) if s[-1] > 6 else 0.0
                    ga = float(np.abs(np.diff(zc) / np.maximum(np.diff(s), 1e-9)).max())
                    if ga > cap + CAP_CURVE_TOL:
                        rec.update(status="NOT_RESTORED", why=f"종단 곡선을 넣으면 상한을 넘음 ({ga * 100:.1f} %)")
                    elif ja > max(jb + 0.02, 0.10):
                        rec.update(status="NOT_RESTORED", why=f"복구하면 턱이 커짐 (2차 차분 {jb:.3f} → {ja:.3f} m)")
                    elif dev > RESTORE_MAX_DEV_M:
                        rec.update(status="NOT_RESTORED", why=f"상한까지 복구하려면 높이 {dev:.1f} m 변경 — 실제 급경사 또는 형상 단순화 의심",
                                   needed_dev_m=round(dev, 1))
                    else:
                        rec.update(status="RESTORED", grade300_after=round(grade_window(s, zc, 300.0), 4), max_dev_m=round(dev, 2))
                        z = zc
                _CAP_STATS[rid] = rec
        d = _DENSE[rid] = (np.array(pts), z)
    return d


DESIGN_CAP = os.getenv("ADAIR_DESIGN_CAP", "0") == "1"
# 「도로의 구조·시설 기준에 관한 규칙」 제25조 최대 종단경사 (산지, 표에서 가장 낮은 설계속도 = 가장 너그러운 값) + 단서 1 %p.
# 도로 등급(ROAD_RANK) → 도로 구분: 103 일반국도 = 간선(40 km/h 9 %), 106 지방도 = 집산(30 km/h 12 %), 107 시·군도 = 국지(20 km/h 16 %).
# 90 m DEM 이 이 값보다 가파르게 만든 구간만 복구 (가정 — 실측 종단 아님). 108 기타·교량·시험 시설은 복구하지 않음
DESIGN_CAP_BY_RANK = {103: 0.10, 106: 0.13, 107: 0.17}
# 안전장치 (2026-10-11 1차 적용: 높이 변화 최대 89 m·노드 이동 17 m — DEM 오류가 아니라 실제 급경사·형상 단순화로 보이는 곳까지 깎았다).
# 이 한도를 넘어야 상한을 맞출 수 있는 도로는 복구하지 않고 '실제 급경사 또는 형상 단순화 의심' 으로 남긴다
NODE_SHIFT_MAX_M = 3.0      # 노드 높이 이동 상한 (한 노드를 옮기면 붙은 모든 도로 경사가 바뀐다)
# 노드 이동은 기본 끔 (2026-10-11 2차: 노드 65개 이동이 이웃 도로 30개를 악화 — 끝 근처 꺾임·횡경사. 복구 안 된 도로의 노드도 움직였다)
DESIGN_CAP_NODES = os.getenv("ADAIR_DESIGN_CAP_NODES", "0") == "1"
CAP_CURVE_SIGMA_M = 10.0    # 상한으로 묶은 뒤 종단 곡선(꺾임 완화) — 묶은 경계의 급한 경사 변화가 턱(2차 차분 0.24 m)을 만들었다
CAP_CURVE_TOL = 0.01        # 곡선 뒤 허용 경사 = 상한 + 1 %p (상한은 DEM 오류 판별용, 주행 기준 아님)
# 2차 확인: 최종 표면 비교(ugv.tools.terrain_compare)에서 악화된 복구 도로 — 다음 생성에서 복구하지 않는다 (되돌림)
DESIGN_CAP_SKIP = set(json.loads(Path(os.environ["ADAIR_DESIGN_CAP_SKIP"]).read_text(encoding="utf-8")))     if os.getenv("ADAIR_DESIGN_CAP_SKIP") else set()
RESTORE_MAX_DEV_M = 5.0     # 도로 종단 복구 높이 변화 상한
_NODE_Z = {}
_CAP_STATS = {}


def design_cap(r):
    if not DESIGN_CAP or r.attrs.get("test_only") or r.road_id in _BRIDGE_INC:
        return None
    return DESIGN_CAP_BY_RANK.get(r.attrs.get("road_rank"))


def node_heights(net, F, inc, margin=0.98, sweeps=2000):
    """노드 높이 = road_z(노드) 에서 시작해, 양 끝 고도차가 설계 상한 × 길이를 넘는 도로의 두 노드를 서로 가깝게 옮긴다 (반씩).
    도로 안 종단을 상한으로 묶으려면 끝 고도차가 먼저 상한 안이어야 한다. 반환 {노드: z}, {노드: 옮긴 m}."""
    nodes = {n for rid in inc for n in (net.roads[rid].a, net.roads[rid].b)}
    pts = np.array([to5186_exact(*net.nodes[n]) for n in nodes])
    z0 = dict(zip(nodes, F.road_z(pts[:, 0], pts[:, 1])))
    z = dict(z0)
    cons = [(net.roads[rid].a, net.roads[rid].b, design_cap(net.roads[rid]) * net.roads[rid].length * margin)
            for rid in inc if design_cap(net.roads[rid])]
    def clamp(n):
        z[n] = min(max(z[n], z0[n] - NODE_SHIFT_MAX_M), z0[n] + NODE_SHIFT_MAX_M)

    for _ in range(sweeps):
        moved = 0.0
        for a, b, lim in cons:
            d = z[a] - z[b]
            e = abs(d) - lim
            if e > 1e-4:
                sgn = 1.0 if d > 0 else -1.0
                za, zb = z[a], z[b]
                z[a] -= sgn * e / 2
                z[b] += sgn * e / 2
                clamp(a)
                clamp(b)
                moved = max(moved, abs(z[a] - za), abs(z[b] - zb))
        if moved <= 1e-4:
            break                         # 상한 안이 됐거나, 이동 한도에 막혀 더 못 움직임 (남는 도로는 복구 안 함)
    return z, {n: round(z[n] - z0[n], 3) for n in nodes if abs(z[n] - z0[n]) > 1e-3}


def cap_profile(s, z, cap, passes=200):
    """양 끝 고정, 2 m 간격 경사 ≤ cap 이 되도록 앞뒤로 번갈아 묶는다 (끝 고도차는 node_heights 로 이미 상한 안)."""
    su = np.arange(0.0, s[-1] + 1e-9, 2.0)
    if su[-1] < s[-1]:
        su = np.append(su, s[-1])
    zu = np.interp(su, s, z)
    S, z0, zN = su[-1], zu[0], zu[-1]
    lo = np.maximum(z0 - cap * su, zN - cap * (S - su))
    hi = np.minimum(z0 + cap * su, zN + cap * (S - su))
    zu = np.clip(zu, lo, hi)
    ds = np.diff(su)
    for _ in range(passes):
        prev = zu.copy()
        for i in range(1, len(zu) - 1):
            zu[i] = min(max(zu[i], zu[i - 1] - cap * ds[i - 1]), zu[i - 1] + cap * ds[i - 1])
        for i in range(len(zu) - 2, 0, -1):
            zu[i] = min(max(zu[i], zu[i + 1] - cap * ds[i]), zu[i + 1] + cap * ds[i])
        if np.abs(zu - prev).max() < 1e-6:
            break
    return np.interp(s, su, zu)


LONG_SIGMA_M = float(os.getenv("ADAIR_LONG_SIGMA_M", "0"))   # 도로 방향 종단 다듬기 σ (m, 0 = 끔). level 모드에서만
LONG_KEEP_GRADE = 0.005    # 다듬기 전후 300 m 창 최대 경사 차 상한 (지속 경사 보존)
LONG_MAX_DEV_M = 3.0       # 다듬기 높이 변화 상한 (넘으면 σ 를 줄이고, σ/4 도 넘으면 다듬지 않음)
_LONG_STATS = {}


def adaptive_smooth(s, z0, sigma_m):
    """도로마다 보존 조건을 지키는 가장 큰 σ (σ, σ/2, σ/4 순): 300 m 창 최대 경사 변화 ≤ LONG_KEEP_GRADE, 높이 변화 ≤ LONG_MAX_DEV_M.
    다 넘으면 다듬지 않는다 — 실제 고도차(지속 경사)를 깎지 않기 위해. 반환 (z, 쓴 σ)."""
    g0 = grade_window(s, z0, 300.0)
    for sg in (sigma_m, sigma_m / 2, sigma_m / 4):
        zt = smooth_profile(s, z0, sg)
        if abs(grade_window(s, zt, 300.0) - g0) <= LONG_KEEP_GRADE and np.abs(zt - z0).max() <= LONG_MAX_DEV_M:
            return zt, sg
    return z0, 0.0


def smooth_profile(s, z, sigma_m, step=2.0):
    """도로 종단 1차원 가우스 다듬기 — 90 m DEM 보간 요철 제거. 양 끝 = 원래 값(노드 road_z) 고정 (교차로 혼합 불변량:
    모든 도로의 노드 높이가 같다). 끝 고정은 다듬은 값과 원래 끝값 차이를 선형으로 되돌려 맞춘다."""
    from scipy.ndimage import gaussian_filter1d
    if sigma_m <= 0 or s[-1] < 4 * step:
        return z
    su = np.arange(0.0, s[-1] + 1e-9, step)
    if su[-1] < s[-1]:
        su = np.append(su, s[-1])
    zu = np.interp(su, s, z)
    zs = gaussian_filter1d(zu, sigma_m / step, mode="nearest")
    f = su / su[-1]
    zs = zs + (zu[0] - zs[0]) * (1 - f) + (zu[-1] - zs[-1]) * f
    return np.interp(s, su, zs)


def grade_window(s, z, w, step=2.0):
    """창 w m 최대 |Δz|/w (도로가 w 보다 짧으면 양 끝 평균 경사)."""
    if s[-1] <= w:
        return float(abs(z[-1] - z[0]) / max(s[-1], 1e-9))
    su = np.arange(0.0, s[-1] + 1e-9, step)
    zu = np.interp(su, s, z)
    k = int(round(w / step))
    return float(np.abs(zu[k:] - zu[:-k]).max() / w)


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


def _road_away(net, rid, node, dist):
    """도로 rid 의 node 쪽 끝에서 dist m 안쪽 점 (x, y)."""
    r = net.roads[rid]
    s = min(dist, r.length) if r.a == node else max(r.length - dist, 0.0)
    p = net.at(rid, s)
    return p.x, p.y


def design_junction(net, F, inc, node):
    """설계형 교차로면 정의 → ('major', 점열 P, 고도 Zp, 주도로 쌍) 또는 ('plane', (a, b, c)) 또는 None.
    주도로 쌍: 노드에 붙은 포함 도로 중 위계(road_rank 작을수록 상위)·차로 수가 가장 높고 DESIGN_STRAIGHT_DEG 이상으로 이어지는 쌍.
    없으면 공통 평면 z = a + b·x + c·y (진입부 DESIGN_PLANE_RING 구간 중심선 고도 최소제곱)."""
    roads = [r for r in inc if node in (net.roads[r].a, net.roads[r].b)]
    if len(roads) < 3:
        return None
    nx, ny = net.nodes[node]
    info = {}
    for rid in roads:
        ax, ay = _road_away(net, rid, node, 10.0)
        a = net.roads[rid].attrs
        info[rid] = (math.atan2(ay - ny, ax - nx), int(a.get("road_rank") or 999), int(a.get("lanes") or 1))
    best = None
    for i1 in range(len(roads)):
        for i2 in range(i1 + 1, len(roads)):
            r1, r2 = roads[i1], roads[i2]
            ang = abs((math.degrees(info[r1][0] - info[r2][0]) + 180.0) % 360.0 - 180.0)
            if ang < DESIGN_STRAIGHT_DEG:
                continue
            key = (-max(info[r1][1], info[r2][1]), info[r1][2] + info[r2][2], ang)
            if best is None or key > best[0]:
                best = (key, r1, r2)
    others = [r for r in roads if best is None or r not in best[1:]]
    if best is not None and all(info[r][1] >= max(info[best[1]][1], info[best[2]][1]) for r in others) and             any(info[r][1] > max(info[best[1]][1], info[best[2]][1]) or info[r][2] < min(info[best[1]][2], info[best[2]][2])
                for r in others):
        # 주도로 우선: 주도로가 부도로보다 위계가 높거나 차로가 많을 때
        pts, zs = [], []
        for k, rid in enumerate(best[1:]):
            P, Zd = road_dense(net, F, rid)
            P, Zd = list(P), list(Zd)
            r = net.roads[rid]
            if (k == 0) == (r.a == node):          # 첫 도로는 노드 쪽이 끝, 둘째는 노드 쪽이 시작이 되게
                P, Zd = P[::-1], Zd[::-1]
            if k == 1:
                P, Zd = P[1:], Zd[1:]
            pts += [tuple(q) for q in P]
            zs += list(Zd)
        return ("major", np.array(pts), np.array(zs), best[1:])
    A, b = [], []
    for rid in roads:
        P, Zd = road_dense(net, F, rid)
        r = net.roads[rid]
        cum = np.r_[0, np.cumsum(np.hypot(*np.diff(np.asarray(P), axis=0).T))]
        dist = cum if r.a == node else cum[-1] - cum
        sel = (dist >= DESIGN_PLANE_RING[0]) & (dist <= DESIGN_PLANE_RING[1])
        for (x, y), z in zip(np.asarray(P)[sel], np.asarray(Zd)[sel]):
            A.append([1.0, x - nx, y - ny])
            b.append(z)
    if len(b) < 3:
        return None
    coef, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
    return ("plane", (float(coef[0]), float(coef[1]), float(coef[2])), (nx, ny))


def design_out_radius(net, F, inc, node, dj):
    """설계면과 각 도로 자기 면의 높이 차(노드에서 JUNC_IN_M 지점) → 전이 바깥 반경. 실패면 None."""
    nx, ny = net.nodes[node]
    worst = 0.0
    for rid in [r for r in inc if node in (net.roads[r].a, net.roads[r].b)]:
        if dj[0] == "major" and rid in dj[3]:
            continue
        P, Zd = road_dense(net, F, rid)
        P, Zd = np.asarray(P), np.asarray(Zd)
        k = int(np.argmin(np.abs(np.hypot(P[:, 0] - nx, P[:, 1] - ny) - JUNC_IN_M)))
        zj = float(_design_eval(dj, P[k:k + 1, 0:1], P[k:k + 1, 1:2])[0, 0])
        worst = max(worst, abs(zj - float(Zd[k])))
    L = max(JUNC_OUT_M - JUNC_IN_M, 1.5 * worst / DESIGN_WARP_GRADE)
    r_out = JUNC_IN_M + L
    DESIGN_REPORT[node] = {"type": dj[0], "major": list(dj[3]) if dj[0] == "major" else None,
                           "max_dz_at_in_m": round(worst, 3), "r_out_m": round(r_out, 1), "ok": r_out <= DESIGN_OUT_MAX_M}
    return r_out if r_out <= DESIGN_OUT_MAX_M else None


def _design_eval(dj, X, Y):
    """설계면 고도 (격자 X, Y)."""
    if dj[0] == "plane":
        (a, b, c), (nx, ny) = dj[1], dj[2]
        return a + b * (X - nx) + c * (Y - ny)
    P, Zp = dj[1], dj[2]
    best = np.full(X.shape, np.inf)
    out = np.zeros(X.shape)
    for k in range(len(P) - 1):
        (ax, ay), (bx, by) = P[k], P[k + 1]
        dx_, dy_ = bx - ax, by - ay
        L2 = dx_ * dx_ + dy_ * dy_
        t = np.clip(((X - ax) * dx_ + (Y - ay) * dy_) / L2, 0.0, 1.0) if L2 > 0 else np.zeros(X.shape)
        dd = np.hypot(X - ax - t * dx_, Y - ay - t * dy_)
        z = Zp[k] + t * (Zp[k + 1] - Zp[k])
        m = dd < best
        best = np.where(m, dd, best)
        out = np.where(m, z, out)
    return out          # 주도로 횡단 수평 (기존 도로 단면과 같음)


def build_tile(net, F, inc, half_w, i, j, junction_mode=None, level_sigma=None):
    """높이 = 도로 띠 안: 포함 도로들의 '가장 가까운 중심선 점 고도' 가중 평균 (도로마다 가중 = 띠+전이 폭 밖에서 0 으로
    연속 감소 / (거리+1)²). 도로 하나만 있는 곳은 그 도로 중심선 고도 그대로 → 횡단 평평. 교차로(여러 도로)는 연속 혼합.
    띠 밖 BLEND_M 까지 도로면 → 지형 D 선형 전이, 그 밖 D. (2026-10-10 첫 생성: 띠 안에서 다듬은 지형 면을 그대로 써
    산 사면 도로 횡단 경사 최대 59 %, 10 % 넘는 도로 320개)"""
    mode = junction_mode or JUNCTION_MODE
    sig = JUNC_LEVEL_SIGMA_M if level_sigma is None else level_sigma
    x0, y0, x1, y1 = tile_bounds(i, j)
    cell = TILE_M / (N - 1)
    # level 모드 σ>0: 타일 밖 여백(가우스 반경 4σ 이상)까지 같은 전역 격자로 계산해 평활한 뒤 잘라낸다 → 값이 위치만의 함수라
    # 이웃 타일이 같은 값을 갖는다 (예전 타일 안 평활은 이음매 최대 0.73 m 어긋남). 여백 밖 값만 쓰지 않으므로 σ0·terrain 은 그대로
    p = int(math.ceil(4.0 * sig / cell)) + 2 if (mode == "level" and sig > 0) else 0
    Np = N + 2 * p
    xs = x0 + (np.arange(Np) - p) * cell
    ys = y1 - (np.arange(Np) - p) * cell                # 행 0 = 북쪽
    X, Y = np.meshgrid(xs, ys)
    E, Nn, res = tile_5186(i, j, X, Y)
    D = F.dem(E, Nn)
    Z = F.road_z(E, Nn)
    edge = np.full((Np, Np), np.inf)
    num = np.zeros((Np, Np))
    den = np.zeros((Np, Np))
    xa, xb, ya, yb = x0 - p * cell, x1 + p * cell, y0 - p * cell, y1 + p * cell     # 계산 범위 (여백 포함)
    roads_here = []
    for rid in inc:
        r = net.roads[rid]
        hw = half_w[rid]
        R = hw + BLEND_M
        reach = R + cell
        bx0, by0 = np.min(r.xy, axis=0)
        bx1, by1 = np.max(r.xy, axis=0)
        if bx1 < xa - reach or bx0 > xb + reach or by1 < ya - reach or by0 > yb + reach:
            continue
        P, Zd = road_dense(net, F, rid)
        # 도로 창 (계산 범위 안)
        J0 = max(0, int((bx0 - reach - xa) / cell))
        J1 = min(Np - 1, int((bx1 + reach - xa) / cell) + 1)
        I0 = max(0, int((yb - (by1 + reach)) / cell))
        I1 = min(Np - 1, int((yb - (by0 - reach)) / cell) + 1)
        if J1 < J0 or I1 < I0:
            continue
        rd = np.full((I1 - I0 + 1, J1 - J0 + 1), np.inf)
        rz = np.zeros(rd.shape)
        hit = False
        for k in range(len(P) - 1):
            (ax, ay), (bx, by) = P[k], P[k + 1]
            if max(ax, bx) < xa - reach or min(ax, bx) > xb + reach or max(ay, by) < ya - reach or min(ay, by) > yb + reach:
                continue
            j0 = max(J0, int((min(ax, bx) - reach - xa) / cell))
            j1 = min(J1, int((max(ax, bx) + reach - xa) / cell) + 1)
            i0 = max(I0, int((yb - (max(ay, by) + reach)) / cell))
            i1 = min(I1, int((yb - (min(ay, by) - reach)) / cell) + 1)
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
    dn = np.full((Np, Np), np.inf)
    dn_t = np.full((Np, Np), np.inf)                  # level 모드에서 terrain 혼합을 쓸 교차로까지 거리
    dn_d = np.full((Np, Np), np.inf)                  # level 모드에서 설계형 교차로면을 쓸 교차로까지 거리
    JD = np.zeros((Np, Np))
    RO = np.full((Np, Np), JUNC_OUT_M)               # 설계 노드별 전이 바깥 반경
    for nid in junction_nodes(net, inc):
        nx, ny = net.nodes[nid]
        reach_n = DESIGN_OUT_MAX_M if nid in JUNC_DESIGN_NODES else JUNC_OUT_M
        if nx < xa - reach_n or nx > xb + reach_n or ny < ya - reach_n or ny > yb + reach_n:
            continue
        if mode == "level" and nid in JUNC_DESIGN_NODES:
            dj = design_junction(net, F, inc, nid)
            ro = design_out_radius(net, F, inc, nid, dj) if dj is not None else None
            if ro is not None:
                j0d, j1d = max(0, int((nx - ro - xa) / cell)), min(Np - 1, int((nx + ro - xa) / cell) + 1)
                i0d, i1d = max(0, int((yb - (ny + ro)) / cell)), min(Np - 1, int((yb - (ny - ro)) / cell) + 1)
                sd = (slice(i0d, i1d + 1), slice(j0d, j1d + 1))
                if j1d < j0d or i1d < i0d:
                    continue
                dd = np.hypot(X[sd] - nx, Y[sd] - ny)
                m = dd < dn_d[sd]
                JD[sd] = np.where(m, _design_eval(dj, X[sd], Y[sd]), JD[sd])
                RO[sd] = np.where(m, ro, RO[sd])
                dn_d[sd] = np.where(m, dd, dn_d[sd])
                continue
        j0, j1 = max(0, int((nx - JUNC_OUT_M - xa) / cell)), min(Np - 1, int((nx + JUNC_OUT_M - xa) / cell) + 1)
        i0, i1 = max(0, int((yb - (ny + JUNC_OUT_M)) / cell)), min(Np - 1, int((yb - (ny - JUNC_OUT_M)) / cell) + 1)
        if j1 < j0 or i1 < i0:
            continue
        sub = (slice(i0, i1 + 1), slice(j0, j1 + 1))
        tgt = dn_t if (mode == "level" and nid in JUNC_TERRAIN_NODES) else dn
        tgt[sub] = np.minimum(tgt[sub], np.hypot(X[sub] - nx, Y[sub] - ny))

    def _tj(d):
        t = np.clip((d - JUNC_IN_M) / (JUNC_OUT_M - JUNC_IN_M), 0.0, 1.0)
        return t * t * (3 - 2 * t)                      # 매끈한 전이 (기울기 연속)
    tj = _tj(dn)
    if mode == "level":
        # 교차로 안도 도로 중심선 고도 혼합(횡단 수평)을 쓰고, 도로마다 고도가 달라 생기는 둔덕만 공간 평활로 없앤다.
        # terrain 모드는 지형 면 Z 를 써 산비탈 교차로가 원지형 횡경사(중앙값 10.6 %)를 물려받았다 (2026-10-11 지도 대조)
        if sig > 0:
            from scipy.ndimage import gaussian_filter
            # 정규화 평활: 도로 혼합이 있는 곳(den>0)만 섞는다 — 타일별 좌표 변환에 걸린 지형 면 Z 를 섞지 않아 위치만의 함수
            m = (den > 0).astype(float)
            gm = gaussian_filter(m, sigma=sig / cell, mode="nearest")
            gs = gaussian_filter(np.where(den > 0, S, 0.0), sigma=sig / cell, mode="nearest")
            Sl = np.where(gm > 1e-9, gs / np.where(gm > 1e-9, gm, 1.0), S)
        else:
            Sl = S
        S = Sl * (1 - tj) + S * tj
        if JUNC_TERRAIN_NODES:
            tjt = _tj(dn_t)
            S = Z * (1 - tjt) + S * tjt                 # 고른 교차로만 terrain 혼합 (평활 뒤 — 평활에는 Z 를 섞지 않음)
        if JUNC_DESIGN_NODES:
            td = np.clip((dn_d - JUNC_IN_M) / (RO - JUNC_IN_M), 0.0, 1.0)
            tjd = td * td * (3 - 2 * td)
            S = JD * (1 - tjd) + S * tjd                # 설계형 교차로면: 15 m 안 설계면, 30 m 밖 각 도로 자기 면, 사이 매끈한 전이
    else:
        S = Z * (1 - tj) + S * tj
    if p:
        cut = (slice(p, p + N), slice(p, p + N))
        S, D, Z, edge, road_mask = S[cut], D[cut], Z[cut], edge[cut], road_mask[cut]
        xs, ys = xs[p:p + N], ys[p:p + N]
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
    if LONG_SIGMA_M > 0 and JUNCTION_MODE != "level":
        # terrain 모드 교차로는 2차원 면 Z 를 써서 다듬은 도로 프로파일과 어긋나 턱이 생긴다
        raise SystemExit("ADAIR_LONG_SIGMA_M 은 ADAIR_JUNCTION_MODE=level 에서만")
    net = RoadNetwork()
    test_roads = sorted(rid for rid, r in net.roads.items() if r.attrs.get("test_only"))
    if test_roads and os.getenv("ADAIR_ALLOW_TEST_ROADS") != "1":
        raise SystemExit(f"시험 전용 도로가 섞인 트윈 타일 생성 거부 ({len(test_roads)}개) — 시험 월드면 ADAIR_ALLOW_TEST_ROADS=1")
    t0 = time.time()
    F = Fields()
    inc, exc, assumed = road_sets(net, F)
    node_shift = {}
    _NODE_Z.clear()
    if DESIGN_CAP and DESIGN_CAP_NODES:
        nz, node_shift = node_heights(net, F, inc)
        _NODE_Z.update(nz)
    half_w = {rid: est_half_width(net.roads[rid]) for rid in inc}
    t_fields = time.time() - t0
    OUT.mkdir(parents=True, exist_ok=True)
    meta_p = OUT / "tiles.json"
    meta = json.loads(meta_p.read_text(encoding="utf-8")) if meta_p.exists() else {"tiles": {}}
    meta["params"] = build_params()
    if JUNC_DESIGN_NODES:
        meta["design_junctions"] = DESIGN_REPORT     # 설계형 교차로면 노드별 선택·전이 반경·실패 (아래 타일 생성 중 채워진다)
    meta["test_only_roads"] = test_roads
    meta.update({"tile_m": TILE_M, "grid": N, "cell_m": round(TILE_M / (N - 1), 4), "q_m": Q, "origin_xy": [X0, Y0],
                 "nx": NX, "ny": NY, "blend_m": BLEND_M, "road_surface_sigma_m": SIGMA_M,
                 "world_origin_alt_m": WORLD_ORIGIN_ALT, "roads_included": len(inc), "roads_excluded": exc,
                 "road_width": "est_half_width = 차로 수 × 3.25 / 2 + 0.5 m (완화 없음)", "fields_s": round(t_fields, 1),
                 "roads_assumed": assumed,
                 "not_verified": ([f"교량 {len(assumed)}개 상판 높이 = 양 끝 도로면 고도 직선 (가정 — 상판·교각 자료 없음, 높이장 성토로 표현)",
                                   "제외 교량(계곡 고가·하부 도로 상충)·터널·입체교차 (FACIL_KIND 2·자동차전용 도로면 없음)"]
                                  if BRIDGES else ["교량·터널·입체교차 (FACIL_KIND 1·2·자동차전용 도로면 없음)"])
                                 + ["DEM 수직 기준 미확인", "90 m DEM 보간 — 사면·하천·옹벽 세부 없음", "편경사 없음 (도로면 = 다듬은 지형 면)"]})
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
    if _LONG_STATS:
        meta["long_smooth"] = dict(_LONG_STATS)
    if DESIGN_CAP:
        meta["design_cap"] = {"basis": "도로의 구조·시설 기준에 관한 규칙 제25조 (산지, 가장 낮은 설계속도) + 1 %p — 가정, 실측 종단 아님",
                              "cap_by_rank": DESIGN_CAP_BY_RANK, "node_shift_max_m": NODE_SHIFT_MAX_M,
                              "restore_max_dev_m": RESTORE_MAX_DEV_M, "node_moves": DESIGN_CAP_NODES,
                              "curve_sigma_m": CAP_CURVE_SIGMA_M, "node_shift_m": node_shift, "roads": dict(_CAP_STATS)}
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
        # 정보 필드 (판정 기준 불변): 300 m 창 지속 경사 — 도로가 300 m 미만이면 양 끝 평균 경사
        k3 = 150
        g300 = (float(np.abs(zc[k3:] - zc[:-k3]).max() / 300.0) if len(zc) > k3
                else float(abs(zc[-1] - zc[0]) / max(L, 1e-6)))
        g10 = round(float(g_long.max()), 4) if g_long.size else 0.0
        per_road[rid] = {"len_m": round(L, 1), "max_long_grade": g10,
                         "max_cross_slope": round(float(g_cross.max()), 4),
                         "max_cross_slope_junction_zone": round(float(g_cross_j.max()), 4),
                         "max_second_diff_m": round(float(jump.max()), 3),
                         "z_range_m": round(float(zc.max() - zc.min()), 1),
                         "max_long_grade_300m": round(g300, 4),
                         "grade_fail_kind": (None if g10 <= 0.10 else ("SUSTAINED" if g300 > 0.10 else "LOCAL"))}
        if rid in meta.get("roads_assumed", {}):
            per_road[rid]["assumed"] = meta["roads_assumed"][rid].get("kind")
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
    out["grade_over_10"] = {"sustained_300m": sum(1 for v in vals if v.get("grade_fail_kind") == "SUSTAINED"),
                            "local_only": sum(1 for v in vals if v.get("grade_fail_kind") == "LOCAL")}
    out["params"] = meta.get("params")
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
