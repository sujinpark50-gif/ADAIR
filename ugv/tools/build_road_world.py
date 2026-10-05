# ugv/tools/build_road_world.py — UGV 전용 Gazebo 월드: 강원 지형 + 실제 도로 면 (개발용, 1회 실행)
#
# 왜 필요한가 (2026-10-02 WSL 시험)
#   UAV 월드(uav/gazebo/kangwon.sdf)의 지형은 90 m DEM 을 56 m 간격 heightmap 으로 보간한 것이라 도로를 모른다.
#   마을이 산자락과 만나는 곳에서는 원본 DEM 의 같은 칸(경사 0)인데도 보간 때문에 62 m 에 8 m 오르는 가짜
#   오르막이 생겨 r1_rover 가 같은 지점에서 두 번 멈췄다(STALLED). 지형으로 도로를 맞출 수 없으니
#   반대로 도로 데이터로 주행 면을 만든다.
#
# 만드는 것 (ugv/gazebo/ 아래, UAV 파일은 건드리지 않는다)
#   models/kangwon_ugv/roads.obj             도로 면 (폭 ROAD_WIDTH_M, 교차로 패드 포함) — 시각·충돌 공용
#   models/kangwon_ugv/heightmap.png         도로 밑을 도로보다 낮게 깎은 지형 (지형이 도로를 뚫고 올라오지 않게)
#   models/kangwon_ugv/texture.png, normal.png   UAV 것 복사
#   kangwon_ugv.sdf                          월드 (지형 + 도로 모델)
#   ugv/data/road_network.json 의 world·spawn 갱신 (스폰 높이 = 도로 면 + SPAWN_Z_OFFSET_M)
#
# 도로 높이
#   1) 노드 높이 = 원본 DEM(90 m) bilinear
#   2) 도로를 DENSIFY_M 간격으로 쪼개 DEM 을 찍고, SMOOTH_M 창으로 이동평균 (90 m 격자 계단 제거)
#   3) 양 끝을 노드 높이에 맞춘다 (선형 보정 — 교차로에서 도로끼리 높이가 이어진다)
#   4) 경사를 MAX_GRADE 로 제한 (양 끝 고정, 앞뒤로 반복 적용). 양 끝 높이차가 커서 불가능하면 선형
#   월드 z = 해발 − DATUM_ALT − 곡률 보정(d²/2R). build_world.py 와 같은 규칙이라 GPS 고도 = 해발.
#
# 실행 (build_road_network.py 다음에):
#   python3 -m ugv.tools.build_road_world          # v1: kangwon_ugv  (road_network.json 의 world·spawn 갱신 = 기본 월드)
#   python3 -m ugv.tools.build_road_world --v2     # v2: kangwon_ugv2 (아래 차이. world·spawn 은 건드리지 않는다)
#   px4-start.sh 에서 WORLD=kangwon_ugv2 로 고른다.
#
# v2 (2026-10-03, WSL 주행 뒤): 교차로·코너에서 아슬아슬하던 곳을 넓히고, 길가에서 바로 떨어지지 않게 한다
#   도로 양옆에 SHOULDER_W_M 폭 갓길이 SHOULDER_DROP_M 만큼 비스듬히 내려간다 — 코너를 질러 도로 끝을 넘어도
#   0.6 m 아래 지형으로 떨어지지 않고 완만한 비탈을 탄다. 갓길이 다른 도로 면보다 높으면 바깥 끝을 내린다.
#   넓히기(교차로 패드 JUNCTION_PAD_R, 테이퍼 TAPER_*)는 실험 스위치로만 남겼다 (아래).
#   도로 띠는 폭 방향으로 COLLISION_STRIPS 줄(가운데 꼭짓점) — 끝점 둘로만 보간하면 교차로에서 높이장과 어긋나 턱이 생긴다.
#   길이 방향 꼭짓점은 COLLISION_STRIDE 개마다 하나 (10 m). 이 메시 하나를 시각·충돌 공용으로 쓴다.
#   교차로 패드는 7 m 유지: 12 m·테이퍼(16 m)로 넓히면 주 경로 근처 턱이 2 → 54~107 칸(최대 0.37 m)으로 늘었다.
# 필요: numpy scipy pillow rasterio pyproj (런타임 서버에는 필요 없다)

import json
import math
import re
import shutil
from pathlib import Path

import numpy as np
import rasterio
from PIL import Image
from pyproj import Transformer
from scipy.ndimage import map_coordinates

ROOT = Path(__file__).resolve().parents[2]
NET = ROOT / "ugv/data/road_network.json"
DEM = ROOT / "environment/data/processed/gangwon/dem_gangwon.tif"
SRC_SDF = ROOT / "uav/gazebo/kangwon.sdf"
SRC_MODEL = ROOT / "uav/gazebo/models/kangwon"
OUT_DIR = ROOT / "ugv/gazebo"
OUT_MODEL = OUT_DIR / "models/kangwon_ugv"
OUT_SDF = OUT_DIR / "kangwon_ugv.sdf"
WORLD_NAME = "kangwon_ugv"

DATUM_LAT, DATUM_LON, DATUM_ALT = 38.011200, 128.122643, 172.1     # build_world.py · gz_bridge.py 와 같다
LOCAL = (f"+proj=tmerc +lat_0={DATUM_LAT} +lon_0={DATUM_LON} +k=1 "
         "+x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs")
EARTH_R = 6371000.0

ROAD_WIDTH_M = 10.0       # 도로 면 폭. 실제 2차로(약 7 m)보다 넓게 — PX4 경로 추종이 코너를 안쪽으로 질러도 떨어지지 않게
DENSIFY_M = 5.0           # 도로 높이·면을 이 간격으로 쪼갠다 (교차로에서 면끼리 높이가 맞게 촘촘히)
SMOOTH_M = 150.0          # DEM 이동평균 창 (90 m 격자 계단을 지운다)
MAX_GRADE = 0.08          # 경사 상한 8% (실제 도로 설계 수준). 양 끝 높이차 때문에 불가능하면 선형
NODE_PAD_R = 7.0          # 교차로 패드 반지름 — 도로끼리 이어지는 곳의 틈을 메운다
FIELD_STEP_M = 2.0        # 높이장 표본 간격 (도로 중심선)
FIELD_SIGMA_M = 6.0       # 높이장 가우스 폭 — 교차로·나란한 도로가 이 거리 안에서 같은 높이로 이어진다
FIELD_ROUNDS = 3          # 높이장 ↔ 경사 상한 반복 횟수
LAYER_DZ_M = 3.0          # 자기 도로 높이와 이보다 다른 표본은 섞지 않는다 (입체교차를 평면으로 합치지 않게)
CARVE_CLEAR_M = 0.6       # 지형을 도로 면보다 이만큼 아래로 깎는다
SPAWN_Z_OFFSET_M = 0.4
# v2 전용 (--v2)
JUNCTION_PAD_R = 7.0      # 3갈래 이상 교차로 패드 반지름. 12 m 는 주 경로에 0.2 m 턱·갓길 1 m 솟음이 생겨 7 m 유지 (2026-10-03)
TAPER_W_M = 16.0          # 교차로에 닿는 도로 끝 폭
TAPER_LEN_M = 20.0        # 이 거리에 걸쳐 ROAD_WIDTH_M 로 좁아진다
SHOULDER_W_M = 4.0        # 갓길 폭 (도로 끝에서 바깥으로)
SHOULDER_DROP_M = 0.25    # 갓길 바깥 끝이 도로 끝보다 낮은 정도 (경사 약 6%)
COLLISION_STRIDE = 2      # 충돌 메시: 길이 방향 꼭짓점 간격 = DENSIFY_M × 이 값
RIBBON_STRIPS = 2         # 시각 메시 폭 방향 줄 수 (v1 은 1). 시각은 턱과 무관해 가볍게
COLLISION_STRIPS = 2      # 충돌 메시 폭 방향 줄 수
# 실험용 스위치 (환경변수): UGV_V2_TAPER=0/1, UGV_V2_SHOULDER=0/1, UGV_V2_PAD_R=<m>, UGV_V2_DRY=1(파일 안 씀)
import os as _os
V2_TAPER = _os.getenv("UGV_V2_TAPER", "0") == "1"      # 기본 끔: 넓힌 띠가 교차로 높이장과 어긋나 주 경로 턱이 늘었다 (2026-10-03)
V2_SHOULDER = _os.getenv("UGV_V2_SHOULDER", "1") == "1"
JUNCTION_PAD_R = float(_os.getenv("UGV_V2_PAD_R", JUNCTION_PAD_R))
V2_DRY = _os.getenv("UGV_V2_DRY", "0") == "1"
# v2 산불 표시 — 자리는 CA 발화점(남전약수터), 배치(시드)·텍스처는 UAV 월드(uav/gazebo/add_fire.py)와 같다. 파티클이라 GUI 에서만 보인다.
# 텍스처 경로는 @UAV_TEX@ 자리표시 — px4-start.sh 가 띄울 때 절대 경로로 바꾼다.
FIRE_LAT, FIRE_LON = 38.0277, 128.1303   # 남전약수터 — 환경 2019 트윈 발화점(tools/inje2019_whatif.py NAMJEON_SPRING)과 같은 값
FIRE_N, FIRE_R, SMOKE_N = 8, 300.0, 3
RIBBON_STRIPS = int(_os.getenv("UGV_V2_STRIPS", RIBBON_STRIPS))
COLLISION_STRIPS = int(_os.getenv("UGV_V2_COLL_STRIPS", COLLISION_STRIPS))
COLLISION_STRIDE = int(_os.getenv("UGV_V2_COLL_STRIDE", COLLISION_STRIDE))
SPAWN_GAP_M = 8.0         # 같은 거점 두 번째 차량은 도로를 따라 이만큼 앞에 세운다


# --- 좌표·고도 ----------------------------------------------------------------

to_local = Transformer.from_crs("EPSG:4326", LOCAL, always_xy=True)


class Dem:
    def __init__(self, path):
        with rasterio.open(path) as r:
            self.z = r.read(1).astype(np.float64)
            self.t = r.transform
            self.nodata = r.nodata
            self.to_dem = Transformer.from_crs("EPSG:4326", r.crs, always_xy=True)

    def elev(self, lat, lon):
        x, y = self.to_dem.transform(np.asarray(lon), np.asarray(lat))
        col = (x - self.t.c) / self.t.a - 0.5
        row = (y - self.t.f) / self.t.e - 0.5
        return map_coordinates(self.z, [np.atleast_1d(row), np.atleast_1d(col)], order=1, mode="nearest")


def world_z(elev, x, y):
    return elev - DATUM_ALT - (x * x + y * y) / (2 * EARTH_R)


# --- 도로 높이 ----------------------------------------------------------------

def densify(xy):
    out = [xy[0]]
    for a, b in zip(xy, xy[1:]):
        n = max(1, int(math.ceil(math.dist(a, b) / DENSIFY_M)))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
    return np.array(out)


def road_profile(xy, z_raw, za, zb):
    """이동평균 → 양 끝 노드 높이에 맞춤 → 경사 상한. 반환 (z, 선형으로 대체했는지)."""
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    return _profile(s, z_raw, za, zb)


def cap_grade(s, z):
    """양 끝 고정, 경사 MAX_GRADE 상한 (앞뒤로 반복). 양 끝 높이차가 커서 불가능하면 그대로 둔다."""
    z = z.copy()
    if abs(z[-1] - z[0]) > MAX_GRADE * s[-1]:
        return z
    for _ in range(50):
        prev = z.copy()
        for i in range(1, len(z) - 1):
            d = s[i] - s[i - 1]
            z[i] = np.clip(z[i], z[i - 1] - MAX_GRADE * d, z[i - 1] + MAX_GRADE * d)
        for i in range(len(z) - 2, 0, -1):
            d = s[i + 1] - s[i]
            z[i] = np.clip(z[i], z[i + 1] - MAX_GRADE * d, z[i + 1] + MAX_GRADE * d)
        if np.allclose(prev, z, atol=1e-3):
            break
    return z


def _profile(s, z_raw, za, zb):
    L = s[-1]
    if L < 1e-6:
        return np.full(len(s), za), False
    w = max(1, min(int(round(SMOOTH_M / DENSIFY_M)), len(s) // 2))
    pad = np.pad(z_raw, w, mode="edge")
    z = np.convolve(pad, np.ones(2 * w + 1) / (2 * w + 1), mode="same")[w:-w]
    z = z + (za - z[0]) + (s / L) * ((zb - z[-1]) - (za - z[0]))       # 양 끝 맞춤
    if abs(zb - za) > MAX_GRADE * L:                                    # 상한으로는 불가능
        return za + (zb - za) * s / L, True
    for _ in range(50):
        prev = z.copy()
        for i in range(1, len(z) - 1):                                  # 앞에서부터
            d = s[i] - s[i - 1]
            z[i] = np.clip(z[i], z[i - 1] - MAX_GRADE * d, z[i - 1] + MAX_GRADE * d)
        for i in range(len(z) - 2, 0, -1):                              # 뒤에서부터 (끝 고정)
            d = s[i + 1] - s[i]
            z[i] = np.clip(z[i], z[i + 1] - MAX_GRADE * d, z[i + 1] + MAX_GRADE * d)
        if np.allclose(prev, z, atol=1e-3):
            break
    return z, False


def smooth_nodes(node_z, node_xy, roads, iters=300):
    """노드 높이를 이웃과 경사 MAX_GRADE 안으로 맞춘다 (원본 DEM 값에서 최소한만 움직인다).
    90 m 격자를 bilinear 로 찍은 노드 높이는 가까운 교차로끼리도 몇 m 씩 어긋나, 짧은 도로가 급경사가 된다.
    각 노드를 이웃이 허용하는 구간 [max(z_m − G·L), min(z_m + G·L)] 으로 당기고, 구간이 비면(이웃끼리 모순)
    가운데로 둔다. Gauss-Seidel 로 수렴할 때까지 반복."""
    nbr = {n: [] for n in node_z}
    for r in roads:
        L = max(r["distance_m"], 1.0)
        nbr[r["node_a"]].append((r["node_b"], L))
        nbr[r["node_b"]].append((r["node_a"], L))
    z = dict(node_z)
    for _ in range(iters):
        moved = 0.0
        for n, ms in nbr.items():
            if not ms:
                continue
            lo = max(z[m] - MAX_GRADE * L for m, L in ms)
            hi = min(z[m] + MAX_GRADE * L for m, L in ms)
            new = min(max(z[n], lo), hi) if lo <= hi else (lo + hi) / 2
            moved = max(moved, abs(new - z[n]))
            z[n] = new
        if moved < 0.01:
            break
    shift = np.array([z[n] - node_z[n] for n in z])
    print(f"노드 높이 보정: 평균 {np.abs(shift).mean():.2f} m, 최대 {np.abs(shift).max():.1f} m")
    return z


class RoadField:
    """도로 면 높이장 — 연속 함수 하나로 모든 도로 면의 높이를 정한다.
    도로마다 높이를 따로 정하거나 꼭짓점마다 끊어서 섞으면, 띠가 겹치는 교차로에서 한 띠가 다른 띠
    위로 올라와 턱이 생긴다 (2026-10-02 시험, 교차로마다 턱). 그래서 모든 도로 중심선을 FIELD_STEP_M
    간격으로 찍은 높이를 가우스 가중평균(σ=FIELD_SIGMA_M)한 연속 함수로 만들고, 모든 도로 띠·교차로
    패드의 꼭짓점을 이 함수에서 읽는다. 겹치는 면은 같은 함수를 쓰므로 높이가 같다 (삼각형 보간 오차만 남는다).
    다만 자기 높이와 LAYER_DZ_M 이상 다른 표본(입체교차의 위·아래 도로)은 섞지 않는다."""

    def __init__(self, road_xyz):
        from scipy.spatial import cKDTree
        pts = []
        for v in road_xyz.values():
            s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(v[:, :2], axis=0).T))])
            q = np.arange(0, s[-1] + 1e-9, FIELD_STEP_M)
            pts.append(np.column_stack([np.interp(q, s, v[:, 0]), np.interp(q, s, v[:, 1]),
                                        np.interp(q, s, v[:, 2])]))
        p = np.vstack(pts)
        self.p, self.z = p[:, :2], p[:, 2]
        self.tree = cKDTree(self.p)

    def at(self, xy, z_own=None):
        xy = np.atleast_2d(xy)
        out = np.empty(len(xy))
        for k, nb in enumerate(self.tree.query_ball_point(xy, 3 * FIELD_SIGMA_M)):
            nb = np.asarray(nb)
            if not len(nb):
                out[k] = np.nan if z_own is None else z_own[k]
                continue
            if z_own is not None:                                       # 입체교차: 높이가 크게 다른 층은 섞지 않는다
                nb = nb[np.abs(self.z[nb] - z_own[k]) < LAYER_DZ_M]
                if not len(nb):
                    out[k] = z_own[k]
                    continue
            d2 = ((self.p[nb] - xy[k]) ** 2).sum(axis=1)
            w = np.exp(-d2 / (2 * FIELD_SIGMA_M ** 2))
            out[k] = (w * self.z[nb]).sum() / w.sum()
        return out


def ledge_check(mesh, cell=1.0, near=None):
    """메시를 1 m 격자로 찍어, 한 칸을 덮는 면들의 높이차(턱)를 잰다. 반환 (턱 높이 배열).
    near(kd-tree, 반경) 를 주면 (전체, 그 반경 안 칸만) 두 배열을 돌려준다 — 주 경로 근처 턱 확인용."""
    lo, hi = raster(mesh, cell)
    keys = list(lo)
    led = np.array([hi[k] - lo[k] for k in keys])
    if near is None:
        return led
    tree, rad = near
    d, _ = tree.query(np.array(keys, dtype=float) + 0.5 * cell)
    return led, led[d <= rad]


def raster(mesh, cell=1.0):
    """메시를 cell 격자로 찍어 칸별 (최저, 최고) 면 높이 dict 두 개를 돌려준다."""
    V = np.array(mesh.v); F = np.array(mesh.f) - 1
    lo, hi = {}, {}
    for a, b, c in F:
        A, B, C = V[a], V[b], V[c]
        x0, x1 = np.floor(min(A[0], B[0], C[0])), np.ceil(max(A[0], B[0], C[0]))
        y0, y1 = np.floor(min(A[1], B[1], C[1])), np.ceil(max(A[1], B[1], C[1]))
        gx, gy = np.meshgrid(np.arange(x0, x1 + cell, cell), np.arange(y0, y1 + cell, cell))
        gx, gy = gx.ravel(), gy.ravel()
        den = (B[1] - C[1]) * (A[0] - C[0]) + (C[0] - B[0]) * (A[1] - C[1])
        if abs(den) < 1e-9:
            continue
        l1 = ((B[1] - C[1]) * (gx - C[0]) + (C[0] - B[0]) * (gy - C[1])) / den
        l2 = ((C[1] - A[1]) * (gx - C[0]) + (A[0] - C[0]) * (gy - C[1])) / den
        l3 = 1 - l1 - l2
        ins = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
        z = l1[ins] * A[2] + l2[ins] * B[2] + l3[ins] * C[2]
        for key, h in zip(zip(gx[ins].astype(int), gy[ins].astype(int)), z):
            if key in lo:
                lo[key] = min(lo[key], h); hi[key] = max(hi[key], h)
            else:
                lo[key] = hi[key] = h
    return lo, hi


# --- 메시 ---------------------------------------------------------------------

class Obj:
    def __init__(self):
        self.v, self.f, self.owner = [], [], []
        self.tag = 0                # 꼭짓점이 어느 도로·패드 것인지 (턱 검사용)

    def vert(self, x, y, z):
        self.v.append((x, y, z))
        self.owner.append(self.tag)
        return len(self.v)          # OBJ 는 1부터

    def tri(self, a, b, c):
        self.f.append((a, b, c))

    def quad(self, a, b, c, d):     # 반시계 a-b-c-d (위에서 볼 때)
        self.tri(a, b, c)
        self.tri(a, c, d)

    def write(self, path, name):
        """윗면만 있는 메시라 법선은 위쪽 하나(vn 0 0 1)를 모든 면에 쓴다 — 로더가 법선을 만들지 않아도
        음영이 바르게 나오고, 면마다 법선을 쓰는 것보다 파일이 훨씬 작다."""
        with open(path, "w", encoding="ascii") as f:
            f.write(f"# {name} - generated by ugv/tools/build_road_world.py\no {name}\nvn 0 0 1\n")
            f.writelines(f"v {x:.2f} {y:.2f} {z:.2f}\n" for x, y, z in self.v)
            f.writelines(f"f {a}//1 {b}//1 {c}//1\n" for a, b, c in self.f)


def ribbon(mesh: Obj, xyz, field=None, half_w=None, strips=1):
    """도로 띠: 각 점에서 좌우로 폭/2 (half_w 를 주면 점별 반폭). 이웃 구간이 꼭짓점을 공유해 면이 끊기지 않는다.
    반환 (왼쪽 끝 꼭짓점들, 오른쪽 끝 꼭짓점들, 점별 왼쪽 단위법선) — 갓길을 이어 붙일 때 쓴다."""
    xy = xyz[:, :2]
    d = np.diff(xy, axis=0)
    d /= np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-9)[:, None]
    t = np.vstack([d[:1], d[:-1] + d[1:], d[-1:]])                     # 점별 진행방향 (꺾이는 곳은 평균)
    t /= np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    u = np.stack([-t[:, 1], t[:, 0]], axis=1)                           # 왼쪽 단위법선
    n = u * (ROAD_WIDTH_M / 2 if half_w is None else half_w[:, None])
    zl = zr = xyz[:, 2]
    if field is not None:                                               # 겹치는 띠와 같은 높이가 되게
        zl = field.at(xy + n, xyz[:, 2])
        zr = field.at(xy - n, xyz[:, 2])
    L = [mesh.vert(x + nx, y + ny, z) for (x, y, _), (nx, ny), z in zip(xyz, n, zl)]
    R = [mesh.vert(x - nx, y - ny, z) for (x, y, _), (nx, ny), z in zip(xyz, n, zr)]
    if strips <= 1:
        for i in range(len(xyz) - 1):
            mesh.quad(R[i], R[i + 1], L[i + 1], L[i])                   # 윗면 (위에서 반시계)
    else:
        # v2: 폭 방향으로 strips 줄로 나눈다 — 폭 10 m 를 끝점 둘로만 선형 보간하면 높이장(σ 6 m)과 어긋나
        # 교차로에서 겹친 띠끼리 턱이 생긴다 (4줄이면 주 경로 근처 턱 16 → 2 칸, 2026-10-03)
        cols = [R]
        for f in np.linspace(-1, 1, strips + 1)[1:-1]:
            pxy = xy + f * n
            pz = xyz[:, 2] if field is None else field.at(pxy, xyz[:, 2])
            cols.append([mesh.vert(x, y, z) for (x, y), z in zip(pxy, pz)])
        cols.append(L)
        for A, B in zip(cols, cols[1:]):
            for i in range(len(xyz) - 1):
                mesh.quad(A[i], A[i + 1], B[i + 1], B[i])
    return L, R, u, n


def shoulder(mesh: Obj, xyz, L, R, u, n, field):
    """도로 끝 꼭짓점(L, R)을 안쪽 변으로, 바깥 SHOULDER_W_M 에 높이장 − SHOULDER_DROP_M 인 갓길 띠를 붙인다.
    안쪽 변은 도로 끝과 같은 꼭짓점이라 틈이 없고, 바깥 변은 높이장보다 낮아 다른 도로 위로 올라오지 않는다."""
    xy, z = xyz[:, :2], xyz[:, 2]
    w = u * SHOULDER_W_M
    lo_xy, ro_xy = xy + n + w, xy - n - w
    lz = field.at(lo_xy, z) - SHOULDER_DROP_M
    rz = field.at(ro_xy, z) - SHOULDER_DROP_M
    LO = [mesh.vert(x, y, h) for (x, y), h in zip(lo_xy, lz)]
    RO = [mesh.vert(x, y, h) for (x, y), h in zip(ro_xy, rz)]
    for i in range(len(xyz) - 1):
        mesh.quad(L[i], L[i + 1], LO[i + 1], LO[i])                     # 왼쪽 갓길 (도로 끝 → 바깥)
        mesh.quad(RO[i], RO[i + 1], R[i + 1], R[i])                     # 오른쪽 갓길
    return np.column_stack([lo_xy, lz]), np.column_stack([ro_xy, rz]), LO + RO


def clamp_shoulders(mesh: Obj, outer_ids: set[int], rlo: dict, rhi: dict, rounds: int = 8) -> int:
    """갓길 면이 다른 도로 면보다 높은 곳이 있으면 그 면의 바깥 꼭짓점을 내린다 (안쪽 변은 자기 도로 끝이라 고정).
    면마다 꼭짓점·변 중점·무게중심 7점을 1 m 칸에 찍어 도로 면 최고보다 높으면 0.3 m 씩 내리기를 반복."""
    V = np.array(mesh.v)
    faces = np.array([f for f in mesh.f if any(v in outer_ids for v in f)]) - 1
    is_out = np.zeros(len(V), bool); is_out[np.array(sorted(outer_ids)) - 1] = True
    lowered = set()
    for _ in range(rounds):
        A, B, C = V[faces[:, 0]], V[faces[:, 1]], V[faces[:, 2]]
        samples = [A, B, C, (A + B) / 2, (B + C) / 2, (C + A) / 2, (A + B + C) / 3]
        bad = np.zeros(len(faces), bool)
        for P in samples:
            for i, (x, y, z) in enumerate(P):
                if not bad[i]:
                    h = rhi.get((int(math.floor(x)), int(math.floor(y))))
                    if h is not None and z > h + 0.02 and z < h + LAYER_DZ_M:   # 입체교차 위층은 제외
                        bad[i] = True
        if not bad.any():
            break
        vs = np.unique(faces[bad].ravel())
        vs = vs[is_out[vs]]
        V[vs, 2] -= 0.3
        lowered.update(vs.tolist())
    for i in lowered:
        mesh.v[i] = tuple(V[i])
    return len(lowered)


def taper_half_w(xyz, a_junction, b_junction):
    """교차로 쪽 끝에서 TAPER_W_M → TAPER_LEN_M 에 걸쳐 ROAD_WIDTH_M 로 좁아지는 점별 반폭."""
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xyz[:, :2], axis=0).T))])
    w = np.full(len(s), ROAD_WIDTH_M)
    for is_j, d in ((a_junction, s), (b_junction, s[-1] - s)):
        if is_j:
            w = np.maximum(w, ROAD_WIDTH_M + (TAPER_W_M - ROAD_WIDTH_M) * np.clip(1 - d / TAPER_LEN_M, 0, 1))
    return w / 2


def pad(mesh: Obj, x, y, z, k=12, field=None, r=NODE_PAD_R):
    ang = 2 * math.pi * np.arange(k) / k
    rx, ry = x + r * np.cos(ang), y + r * np.sin(ang)
    rz = np.full(k, z) if field is None else field.at(np.column_stack([rx, ry]), np.full(k, z))
    cz = z if field is None else field.at(np.array([[x, y]]), np.array([z]))[0]
    c = mesh.vert(x, y, cz)
    ring = [mesh.vert(a, b, h) for a, b, h in zip(rx, ry, rz)]
    if r <= NODE_PAD_R or field is None:
        for i in range(k):
            mesh.tri(c, ring[i], ring[(i + 1) % k])
        return
    # 큰 패드(v2)는 안쪽 고리를 하나 더 — 반지름 12 m 부채꼴을 끝점만으로 보간하면 높이장과 어긋난다
    ix, iy = x + r / 2 * np.cos(ang), y + r / 2 * np.sin(ang)
    iz = field.at(np.column_stack([ix, iy]), np.full(k, z))
    inner = [mesh.vert(a, b, h) for a, b, h in zip(ix, iy, iz)]
    for i in range(k):
        j = (i + 1) % k
        mesh.tri(c, inner[i], inner[j])
        mesh.quad(inner[i], ring[i], ring[j], inner[j])


# --- 지형 깎기 ----------------------------------------------------------------

class Heightmap:
    """kangwon.sdf 의 collision heightmap (pose·size) 과 같은 규칙으로 꼭짓점 높이를 다룬다."""

    def __init__(self, sdf_text, png):
        blk = sdf_text[sdf_text.index('<model name="kangwon_terrain_collision">'):]
        self.ox, self.oy, self.oz = [float(v) for v in re.search(r"<pose>([^<]+)</pose>", blk).group(1).split()[:3]]
        self.sx, self.sy, self.sz = [float(v) for v in re.search(r"<size>([^<]+)</size>", blk).group(1).split()]
        h = np.asarray(Image.open(png), dtype=np.float64) / 65535.0
        self.z = self.oz + h * self.sz                                  # 꼭짓점 월드 z
        self.n = self.z.shape[0]
        self.d = self.sx / (self.n - 1)

    def cell(self, x, y):
        fc = (x - (self.ox - self.sx / 2)) / self.d
        fr = ((self.oy + self.sy / 2) - y) / self.d
        return fc, fr

    def surface(self, x, y):
        fc, fr = self.cell(x, y)
        c0, r0 = np.floor(fc).astype(int), np.floor(fr).astype(int)
        dc, dr = fc - c0, fr - r0
        z = self.z
        return (z[r0, c0] * (1 - dc) * (1 - dr) + z[r0, c0 + 1] * dc * (1 - dr)
                + z[r0 + 1, c0] * (1 - dc) * dr + z[r0 + 1, c0 + 1] * dc * dr)

    def carve(self, pts):
        """도로 점(x, y, z)들 아래 지형이 도로 면 − CARVE_CLEAR_M 보다 높으면 그 칸의 네 꼭짓점을 낮춘다.
        꼭짓점을 낮추기만 하므로 반복하면 수렴한다. 도로 폭 양 끝도 검사한다."""
        x, y, zr = pts[:, 0], pts[:, 1], pts[:, 2]
        lowered = 0
        for _ in range(20):
            bad = self.surface(x, y) > zr - CARVE_CLEAR_M
            if not bad.any():
                break
            fc, fr = self.cell(x[bad], y[bad])
            c0, r0 = np.floor(fc).astype(int), np.floor(fr).astype(int)
            lim = zr[bad] - CARVE_CLEAR_M - 0.05
            for dr, dc in ((0, 0), (0, 1), (1, 0), (1, 1)):
                np.minimum.at(self.z, (r0 + dr, c0 + dc), lim)
            lowered += int(bad.sum())
        return lowered

    def save(self, png):
        zmin, zmax = float(self.z.min()), float(self.z.max())
        h16 = np.round((self.z - zmin) / (zmax - zmin) * 65535).astype(np.uint16)
        Image.fromarray(h16).save(png)
        self.oz, self.sz = zmin, zmax - zmin


# --- 산불 표시 (v2) --------------------------------------------------------------

def _emitter(name, x, y, z, *, size, psize, lifetime, rate, vmin, vmax, scale_rate, tex, colors):
    """uav/gazebo/add_fire.py 의 emitter 와 같은 모양. 파티클은 이미터 +X 로 나가므로 pitch -90° 로 위를 향한다."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.1f} {y:.1f} {z:.1f} 0 0 0</pose>
      <link name="link">
        <particle_emitter name="{name}_emitter" type="box">
          <pose>0 0 0 0 -1.5708 0</pose>
          <emitting>true</emitting>
          <size>2 {size} {size}</size>
          <particle_size>{psize} {psize} {psize}</particle_size>
          <lifetime>{lifetime}</lifetime>
          <rate>{rate}</rate>
          <min_velocity>{vmin}</min_velocity>
          <max_velocity>{vmax}</max_velocity>
          <scale_rate>{scale_rate}</scale_rate>
          <material>
            <diffuse>1 1 1</diffuse>
            <pbr><metal><albedo_map>@UAV_TEX@/{tex}</albedo_map></metal></pbr>
          </material>
          <color_range_image>@UAV_TEX@/{colors}</color_range_image>
        </particle_emitter>
      </link>
    </model>"""


def fire_models(hm) -> str:
    """UAV 월드와 같은 산불 (같은 중심·반경·난수 시드). 높이는 이 월드의 지형 충돌면."""
    import random
    cx, cy = to_local.transform(FIRE_LON, FIRE_LAT)
    ground = lambda x, y: float(hm.surface(np.array([x]), np.array([y]))[0])    # noqa: E731
    random.seed(11)
    out = []
    for i in range(FIRE_N):
        rr = FIRE_R * math.sqrt(random.random()) if i else 0.0
        a = random.uniform(0, 2 * math.pi)
        x, y = cx + rr * math.cos(a), cy + rr * math.sin(a)
        out.append(_emitter(f"fire_{i + 1}", x, y, ground(x, y), size=45, psize=18, lifetime=2.5, rate=35,
                            vmin=6, vmax=14, scale_rate=4, tex="fire.png", colors="fire_colors.png"))
    for i in range(SMOKE_N):
        x, y = cx + random.uniform(-150, 150), cy + random.uniform(-150, 150)
        out.append(_emitter(f"smoke_{i + 1}", x, y, ground(x, y) + 30, size=80, psize=45, lifetime=22, rate=4,
                            vmin=5, vmax=9, scale_rate=5, tex="smoke.png", colors="smoke_colors.png"))
    print(f"산불 표시: 불꽃 {FIRE_N}, 연기 {SMOKE_N} — 중심 ({cx:.0f}, {cy:.0f}, {ground(cx, cy):.0f})")
    return "\n".join(out).strip()


# --- 실행 ---------------------------------------------------------------------

def build(v2: bool = False):
    out_model = OUT_DIR / "models/kangwon_ugv2" if v2 else OUT_MODEL
    out_sdf = OUT_DIR / "kangwon_ugv2.sdf" if v2 else OUT_SDF
    world_name = "kangwon_ugv2" if v2 else WORLD_NAME
    net = json.loads(NET.read_text(encoding="utf-8"))
    dem = Dem(DEM)
    sdf = SRC_SDF.read_text(encoding="utf-8")
    hm = Heightmap(sdf, SRC_MODEL / "heightmap.png")

    # 노드 높이
    node_xy, node_z = {}, {}
    for n in net["nodes"]:
        x, y = to_local.transform(n["lon"], n["lat"])
        node_xy[n["node_id"]] = (x, y)
        node_z[n["node_id"]] = float(world_z(dem.elev(n["lat"], n["lon"])[0], x, y))

    node_z = smooth_nodes(node_z, node_xy, net["roads"])

    mesh = Obj()
    all_pts, grades, linear = [], [], []
    road_xyz = {}
    for r in net["roads"]:
        geo = r["geometry"]
        lon = np.array([p[1] for p in geo]); lat = np.array([p[0] for p in geo])
        x, y = to_local.transform(lon, lat)
        xy = densify(list(zip(x, y)))
        xy[0], xy[-1] = node_xy[r["node_a"]], node_xy[r["node_b"]]
        to_ll = Transformer.from_crs(LOCAL, "EPSG:4326", always_xy=True)
        lo, la = to_ll.transform(xy[:, 0], xy[:, 1])
        z_raw = world_z(dem.elev(la, lo), xy[:, 0], xy[:, 1])
        z, lin = road_profile(xy, z_raw, node_z[r["node_a"]], node_z[r["node_b"]])
        if lin:
            linear.append(r["road_id"])
        road_xyz[r["road_id"]] = np.column_stack([xy, z])

    # 높이장 ↔ 경사 상한을 번갈아 몇 번: 높이장으로 겹치는 도로를 맞추면 경사가 조금 커지므로 다시 상한을 걸고,
    # 상한 건 중심선으로 높이장을 다시 만든다. 마지막 높이장에서 모든 꼭짓점을 읽는다.
    for _ in range(FIELD_ROUNDS):
        field = RoadField(road_xyz)
        for rid, xyz in road_xyz.items():
            z = field.at(xyz[:, :2], xyz[:, 2])                         # 중심선도 같은 높이장으로
            s_ = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xyz[:, :2], axis=0).T))])
            xyz[:, 2] = cap_grade(s_, z) if s_[-1] > 0 else z
    field = RoadField(road_xyz)

    degree = {n: 0 for n in node_xy}
    for r in net["roads"]:
        degree[r["node_a"]] += 1; degree[r["node_b"]] += 1
    is_j = {n: d >= 3 for n, d in degree.items()}
    road_ab = {r["road_id"]: (r["node_a"], r["node_b"]) for r in net["roads"]}
    sh_vid: set[int] = set()                # v2 갓길 바깥 꼭짓점 (턱 검사를 도로 면과 따로 하려고)
    coll = Obj() if v2 else None            # v2 충돌 메시 (길이 방향으로 성기게)
    coll_sh: set[int] = set()

    def add_road(m, xyz, hw, sh, strips=1):
        L, R, u, n = ribbon(m, xyz, field, hw, strips)
        if sh is not None:
            return shoulder(sh, xyz, L, R, u, n, field)
        return None

    for rid, xyz in road_xyz.items():
        xy, z = xyz[:, :2], xyz[:, 2]
        mesh.tag += 1
        a_, b_ = road_ab[rid]
        hw = taper_half_w(xyz, is_j[a_], is_j[b_]) if (v2 and V2_TAPER) else np.full(len(xyz), ROAD_WIDTH_M / 2)
        if v2:
            sh = mesh if V2_SHOULDER else None
            res = add_road(mesh, xyz, hw, sh, RIBBON_STRIPS)
            if res:
                sh_vid.update(res[2])
            keep = np.unique(np.r_[np.arange(0, len(xyz), COLLISION_STRIDE), len(xyz) - 1])
            res = add_road(coll, xyz[keep], hw[keep], coll if V2_SHOULDER else None, COLLISION_STRIPS)
            if res:
                coll_sh.update(res[2])
        else:
            add_road(mesh, xyz, hw, None)
        seg = np.hypot(*np.diff(xy, axis=0).T)
        grades += list(np.abs(np.diff(z)) / np.maximum(seg, 1e-6))
        # 깎기 검사점: 중심선 + 양 끝 (폭 방향)
        d = np.diff(xy, axis=0); d = np.vstack([d, d[-1:]])
        d /= np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-9)[:, None]
        nrm = np.stack([-d[:, 1], d[:, 0]], axis=1) * hw[:, None]
        for off in (0, 1, -1):
            all_pts.append(np.column_stack([xy + off * nrm, z]))
    for nid, (x, y) in node_xy.items():
        mesh.tag += 1
        pr = JUNCTION_PAD_R if (v2 and is_j[nid]) else NODE_PAD_R
        pk = 24 if pr > NODE_PAD_R else 12
        pad(mesh, x, y, node_z[nid], k=pk, field=field, r=pr)
        if coll is not None:
            pad(coll, x, y, node_z[nid], k=pk, field=field, r=pr)
        all_pts.append(np.array([[x + pr * math.cos(a), y + pr * math.sin(a), node_z[nid]]
                                 for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)] + [[x, y, node_z[nid]]]))

    # 주 경로(A↔B) 근처 — 실제로 달리는 곳의 턱을 따로 본다
    from scipy.spatial import cKDTree
    from ugv import graph_gpkg
    from ugv.road_graph import RoadGraph
    G0 = RoadGraph(graph_gpkg.NODES, graph_gpkg.ROADS)
    rt0 = G0.find_route("A", "B", 2.0)
    main_pts = np.vstack([road_xyz[G0.road_between(u, v).road_id][:, :2] for u, v in zip(rt0.path, rt0.path[1:])])
    near = (cKDTree(main_pts), 10.0)

    def report(led_all, led_near, label):
        la, ln = led_all[led_all < 1], led_near[led_near < 1]
        print(f"턱 {label}: 0.10~1 m 전체 {((la > 0.10)).sum()} 칸 / 주 경로 10 m 안 {((ln > 0.10)).sum()} 칸"
              f" (그중 0.2 m 이상 {(ln > 0.2).sum()}, 최대 {ln.max() if len(ln) else 0:.2f} m)")

    if v2:
        # 턱 검사는 도로 면(띠+패드)만으로 한다. 갓길은 일부러 낮으니 '갓길이 도로 면보다 높은 칸'을 따로 센다.
        road_only, sh_only = Obj(), Obj()
        road_only.v = mesh.v; sh_only.v = mesh.v
        for f in mesh.f:                    # 갓길 면 = 바깥 꼭짓점을 하나라도 쓰는 면
            (sh_only if any(v in sh_vid for v in f) else road_only).f.append(f)
        led, led_n = ledge_check(road_only, near=near)
        report(led, led_n, "시각 도로 면")
        rlo, rhi = raster(road_only)
        coll_road = Obj(); coll_road.v = coll.v
        coll_road.f = [f for f in coll.f if not any(v in coll_sh for v in f)]
        if sh_only.f:
            clo, chi = raster(coll_road)
            for m_, ids, lo_, hi_ in ((mesh, sh_vid, rlo, rhi), (coll, coll_sh, clo, chi)):
                n_low = clamp_shoulders(m_, ids, lo_, hi_)
                print(f"  갓길 낮춤 ({'시각' if m_ is mesh else '충돌'}): 바깥 꼭짓점 {n_low}개")
            csh = Obj(); csh.v = coll.v
            csh.f = [f for f in coll.f if any(v in coll_sh for v in f)]
            _, cshi = raster(csh)
            ck = [k for k, h in cshi.items() if k in chi and chi[k] + 0.05 < h < chi[k] + LAYER_DZ_M]
            cdn, _ = near[0].query(np.array(ck, dtype=float) + 0.5) if ck else (np.array([]), None)
            print(f"충돌 갓길: 도로 면보다 0.05 m 넘게 높은 칸 {len(ck)} / 주 경로 10 m 안 {(cdn <= near[1]).sum()} 칸")
            slo, shi = raster(sh_only)
            ak = [k for k, h in shi.items() if k in rhi and rhi[k] + 0.05 < h < rhi[k] + LAYER_DZ_M]
            dn, _ = near[0].query(np.array(ak, dtype=float) + 0.5) if ak else (np.array([]), None)
            hmax = max((shi[k] - rhi[k] for k, d in zip(ak, dn) if d <= near[1]), default=0)
            print(f"갓길: {len(sh_only.f)} 삼각형, 도로 면보다 0.05 m 넘게 높은 칸 {len(ak)}"
                  f" / 주 경로 10 m 안 {(dn <= near[1]).sum()} 칸 (최대 {hmax:.2f} m)")
            all_pts.append(np.array([mesh.v[i - 1] for i in sorted(sh_vid)]))     # 낮춘 갓길 밑으로 지형 깎기
        cl, cl_n = ledge_check(coll_road, near=near); report(cl, cl_n, "충돌 도로 면"); cl = cl[cl < LAYER_DZ_M]
        print(f"충돌 메시: {len(coll.f)} 삼각형 (시각 {len(mesh.f)}), 도로 면 턱 0.10~1 m {((cl > 0.10) & (cl < 1)).sum()} 칸")
    else:
        led, led_n = ledge_check(mesh, near=near)
        report(led, led_n, "도로 면")
    sep = led >= LAYER_DZ_M                                             # 입체교차 (위·아래 도로가 따로 있다)
    lg = led[~sep]
    print(f"턱 검사 (1 m 격자 {len(led)} 칸): 겹친 면 높이차 최대 {lg.max():.2f} m, "
          f">0.05 m {(lg > 0.05).sum()} 칸, 0.10~1 m {((lg > 0.10) & (lg < 1)).sum()} 칸, "
          f"1 m 이상(입체교차 경계) {(lg >= 1).sum()} 칸, 입체교차 {sep.sum()} 칸")

    if v2 and V2_DRY:
        print("UGV_V2_DRY=1 — 파일을 쓰지 않는다")
        return
    out_model.mkdir(parents=True, exist_ok=True)
    if coll is not None:
        # v2 는 성긴 메시 하나를 시각·충돌 공용으로 쓴다 — 보이는 면 = 바퀴가 닿는 면, 파일도 절반.
        # 촘촘한 시각 메시(mesh)는 턱 비교용으로만 만들고 쓰지 않는다.
        coll.write(out_model / "roads.obj", "roads")
        (out_model / "roads_collision.obj").unlink(missing_ok=True)
    else:
        mesh.write(out_model / "roads.obj", "roads")
        for old in ("roads_visual.obj", "roads_collision.obj"):
            (out_model / old).unlink(missing_ok=True)

    # 지형 깎기 → 새 heightmap (높이 범위가 바뀌면 pose z·size z 도 바뀐다)
    pts = np.vstack(all_pts)
    before = hm.surface(pts[:, 0], pts[:, 1]) - pts[:, 2]
    z0 = hm.z.copy()
    hm.carve(pts)
    dz = (z0 - hm.z)[(z0 - hm.z) > 0]
    after = hm.surface(pts[:, 0], pts[:, 1]) - pts[:, 2]
    hm.save(out_model / "heightmap.png")
    for f in ("texture.png", "normal.png"):
        shutil.copy(SRC_MODEL / f, out_model / f)

    # 월드 SDF: UAV 월드를 복사해 모델 경로·높이만 바꾸고 도로 모델을 더한다
    w = sdf.replace("model://kangwon/", f"model://{out_model.name}/")
    w = re.sub(r'<world name="[^"]+">', f'<world name="{world_name}">', w, count=1)
    old_z = f"{float(re.search(r'<size>[^ ]+ [^ ]+ ([^<]+)</size>', sdf).group(1)):.2f}"
    w = re.sub(r"<size>([^ <]+) ([^ <]+) [^<]+</size>", rf"<size>\1 \2 {hm.sz:.2f}</size>", w)
    w = re.sub(r"(<pose>[^ ]+ [^ ]+ )[^ ]+( 0 0 0</pose>)", rf"\g<1>{hm.oz:.2f}\g<2>", w, count=1)   # collision pose
    w = re.sub(r"(<pos>(?!0 0 0<)[^ ]+ [^ ]+ )[^<]+(</pos>)", rf"\g<1>{hm.oz:.2f}\g<2>", w)          # visual pos (collision 은 0 0 0 유지)
    roads_model = """
    <model name="ugv_roads">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><mesh><uri>model://MODEL/COLL_OBJ</uri></mesh></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>model://MODEL/roads.obj</uri></mesh></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.33 0.33 0.35 1</diffuse>
                    <specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
    </model>
"""
    roads_model = roads_model.replace("MODEL", out_model.name).replace("COLL_OBJ", "roads.obj")
    w = w.replace("</world>", roads_model + "  </world>", 1)
    if v2:
        i = w.index("<light ")
        w = w[:i] + fire_models(hm) + "\n    " + w[i:]
        # 도로 차단 벽 위치 (ugv/gz_fx.py): 도로 길이의 가운데 점과 진행 방향
        marks = {}
        for rid, xyz in road_xyz.items():
            s_ = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xyz[:, :2], axis=0).T))])
            k = int(min(max(np.searchsorted(s_, s_[-1] / 2) - 1, 0), len(xyz) - 2))
            t_ = (s_[-1] / 2 - s_[k]) / max(s_[k + 1] - s_[k], 1e-9)
            p_ = xyz[k] + t_ * (xyz[k + 1] - xyz[k])
            yaw = math.atan2(xyz[k + 1, 1] - xyz[k, 1], xyz[k + 1, 0] - xyz[k, 0])
            marks[rid] = [round(float(p_[0]), 2), round(float(p_[1]), 2), round(float(p_[2]), 2), round(yaw, 3)]
        (out_model / "road_marks.json").write_text(json.dumps(marks), encoding="utf-8")
    out_sdf.write_text(w, encoding="utf-8")

    # 스폰: 거점 노드에서 나가는 가장 긴 도로를 따라 k × SPAWN_GAP_M 앞, 도로 면 위
    from ugv.config import RESOURCES
    to_ll = Transformer.from_crs(LOCAL, "EPSG:4326", always_xy=True)
    spawn, per_base = {}, {}
    for res in RESOURCES:
        node = res["home_node"]
        leaving = [r for r in net["roads"] if node in (r["node_a"], r["node_b"])]
        r = max(leaving, key=lambda r: r["distance_m"])
        xyz = road_xyz[r["road_id"]] if r["node_a"] == node else road_xyz[r["road_id"]][::-1]
        k = per_base.get(node, 0); per_base[node] = k + 1
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xyz[:, :2], axis=0).T))])
        want = min(SPAWN_GAP_M * k, s[-1] * 0.5)
        i = int(np.searchsorted(s, want, side="right") - 1); i = min(i, len(xyz) - 2)
        t = (want - s[i]) / max(s[i + 1] - s[i], 1e-9)
        p = xyz[i] + t * (xyz[i + 1] - xyz[i])
        yaw = math.atan2(*(xyz[i + 1, 1::-1] - xyz[i, 1::-1]))
        spawn[res["resource_id"]] = {"x": round(float(p[0]), 1), "y": round(float(p[1]), 1),
                                     "z": round(float(p[2]) + SPAWN_Z_OFFSET_M, 2), "yaw": round(yaw, 3),
                                     "node": node}
    if v2:      # v2 를 기본 월드로 (2026-10-03 WSL 주행 비교 뒤 표준). 스폰은 도로 중심 높이라 v1 과 같다 — 확인만
        sd = max(abs(spawn[k]["z"] - net["spawn"][k]["z"]) + abs(spawn[k]["x"] - net["spawn"][k]["x"])
                 + abs(spawn[k]["y"] - net["spawn"][k]["y"]) for k in spawn)
        print(f"스폰 (v1 과 차이 합 최대 {sd:.2f} m — 0 이면 road_network.json spawn 을 그대로 쓴다)")
        net["world"] = {"sdf": str(out_sdf.relative_to(ROOT)), "name": world_name,
                        "datum": [DATUM_LAT, DATUM_LON, DATUM_ALT],
                        "road_width_m": ROAD_WIDTH_M, "max_grade": MAX_GRADE, "shoulder_m": SHOULDER_W_M}
        NET.write_text(json.dumps(net, ensure_ascii=False, indent=1), encoding="utf-8")
    else:
        net["spawn"] = spawn
        net["world"] = {"sdf": str(OUT_SDF.relative_to(ROOT)), "name": WORLD_NAME,
                        "datum": [DATUM_LAT, DATUM_LON, DATUM_ALT],
                        "road_width_m": ROAD_WIDTH_M, "max_grade": MAX_GRADE}
        NET.write_text(json.dumps(net, ensure_ascii=False, indent=1), encoding="utf-8")

    # 대표 경로의 최대 경사 (주행 가능 여부를 미리 본다)
    from ugv import graph_gpkg
    from ugv.road_graph import RoadGraph
    G = RoadGraph(graph_gpkg.NODES, graph_gpkg.ROADS)
    for a_, b_ in (("A", "B"), ("B", "A")):
        rt = G.find_route(a_, b_, 2.0)
        worst = max(((np.abs(np.diff(road_xyz[G.road_between(u, v).road_id][:, 2]))
                      / np.maximum(np.hypot(*np.diff(road_xyz[G.road_between(u, v).road_id][:, :2], axis=0).T), 1e-6)).max(),
                     G.road_between(u, v).road_id) for u, v in zip(rt.path, rt.path[1:]))
        print(f"  경로 {a_}→{b_}: 최대 경사 {worst[0]*100:.1f}% (도로 {worst[1]})")

    g = np.array(grades)
    print(f"도로 메시: {len(mesh.f)} 삼각형 → {out_model.relative_to(ROOT)}/roads.obj")
    print(f"경사: 최대 {g.max()*100:.1f}%  >6% 구간 {(g > 0.06).sum()}/{len(g)}  >8% {(g > 0.0801).sum()}  >10% {(g > 0.10).sum()}"
          f"  (상한 불가로 선형 처리한 도로 {len(linear)}개)")
    print(f"지형이 도로 면을 뚫던 점: 깎기 전 {(before > -CARVE_CLEAR_M).sum()}/{len(pts)} → 후 "
          f"{(after > -CARVE_CLEAR_M + 0.01).sum()}  (낮춘 꼭짓점 {dz.size}개, 평균 {dz.mean():.1f} m, 최대 {dz.max():.1f} m)")
    print(f"heightmap z 범위 {old_z} → {hm.sz:.2f} m, 바닥 {hm.oz:.2f}  → {out_sdf.relative_to(ROOT)}")
    for rid, p in spawn.items():
        print(f"  스폰 {rid}: ({p['x']}, {p['y']}, {p['z']}) yaw {p['yaw']}")
    if linear:
        print("  선형 처리 도로:", ", ".join(linear[:10]), "…" if len(linear) > 10 else "")


if __name__ == "__main__":
    import sys
    build(v2="--v2" in sys.argv[1:])
