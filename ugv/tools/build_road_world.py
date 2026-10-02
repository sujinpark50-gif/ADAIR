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
# 실행 (build_road_network.py 다음에):  python3 -m ugv.tools.build_road_world
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

ROAD_WIDTH_M = 6.0        # 도로 면 폭 (차선 2개 정도). r1_rover 폭 약 0.5 m
DENSIFY_M = 10.0          # 도로 높이·면을 이 간격으로 쪼갠다
SMOOTH_M = 150.0          # DEM 이동평균 창 (90 m 격자 계단을 지운다)
MAX_GRADE = 0.08          # 경사 상한 8% (실제 도로 설계 수준). 양 끝 높이차 때문에 불가능하면 선형
NODE_PAD_R = 5.0          # 교차로 패드 반지름 — 도로끼리 이어지는 곳의 틈을 메운다
BLEND_DZ_M = 2.0          # 이보다 높이차가 큰 다른 도로는 입체교차(고가)로 보고 높이를 섞지 않는다
CARVE_CLEAR_M = 0.6       # 지형을 도로 면보다 이만큼 아래로 깎는다
SPAWN_Z_OFFSET_M = 0.4
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
    """도로 면 높이장: 도로 띠의 모든 꼭짓점 높이를 '근처 도로 중심선 높이의 가중평균'으로 정한다.
    도로마다 따로 높이를 정하면 띠가 겹치는 곳(교차로 주변, 나란히 붙은 도로)에서 한쪽 띠 가장자리가
    다른 띠보다 높아 턱이 생긴다 (2026-10-02 시험). 같은 높이장을 쓰면 겹치는 띠는 같은 높이가 된다.
    자기 도로 높이와 BLEND_DZ_M 이상 다른 도로는 입체교차로 보고 섞지 않는다."""

    def __init__(self, road_xyz):
        from scipy.spatial import cKDTree
        self.p = np.vstack([v[:, :2] for v in road_xyz.values()])
        self.z = np.concatenate([v[:, 2] for v in road_xyz.values()])
        self.tree = cKDTree(self.p)
        self.r = ROAD_WIDTH_M

    def at(self, xy, z_own):
        out = np.array(z_own, dtype=float)
        for k, nb in enumerate(self.tree.query_ball_point(xy, self.r)):
            if not nb:
                continue
            nb = np.asarray(nb)
            zn = self.z[nb]
            keep = np.abs(zn - z_own[k]) < BLEND_DZ_M
            if not keep.any():
                continue
            d = np.hypot(*(self.p[nb[keep]] - xy[k]).T)
            w = (1 - d / self.r) ** 2 + 1e-6
            out[k] = (w * zn[keep]).sum() / w.sum()
        return out


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


def ribbon(mesh: Obj, xyz, field=None):
    """도로 띠: 각 점에서 좌우로 폭/2. 이웃 구간이 꼭짓점을 공유해 면이 끊기지 않는다."""
    xy = xyz[:, :2]
    d = np.diff(xy, axis=0)
    d /= np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-9)[:, None]
    t = np.vstack([d[:1], d[:-1] + d[1:], d[-1:]])                     # 점별 진행방향 (꺾이는 곳은 평균)
    t /= np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    n = np.stack([-t[:, 1], t[:, 0]], axis=1) * (ROAD_WIDTH_M / 2)      # 왼쪽 법선
    zl = zr = xyz[:, 2]
    if field is not None:                                               # 겹치는 띠와 같은 높이가 되게
        zl = field.at(xy + n, xyz[:, 2])
        zr = field.at(xy - n, xyz[:, 2])
    L = [mesh.vert(x + nx, y + ny, z) for (x, y, _), (nx, ny), z in zip(xyz, n, zl)]
    R = [mesh.vert(x - nx, y - ny, z) for (x, y, _), (nx, ny), z in zip(xyz, n, zr)]
    for i in range(len(xyz) - 1):
        mesh.quad(R[i], R[i + 1], L[i + 1], L[i])                       # 윗면 (위에서 반시계)


def pad(mesh: Obj, x, y, z, k=12, field=None):
    ang = 2 * math.pi * np.arange(k) / k
    rx, ry = x + NODE_PAD_R * np.cos(ang), y + NODE_PAD_R * np.sin(ang)
    rz = np.full(k, z) if field is None else field.at(np.column_stack([rx, ry]), np.full(k, z))
    cz = z if field is None else field.at(np.array([[x, y]]), np.array([z]))[0]
    c = mesh.vert(x, y, cz)
    ring = [mesh.vert(a, b, h) for a, b, h in zip(rx, ry, rz)]
    for i in range(k):
        mesh.tri(c, ring[i], ring[(i + 1) % k])


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


# --- 실행 ---------------------------------------------------------------------

def build():
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

    field = RoadField(road_xyz)
    for rid, xyz in road_xyz.items():
        xyz[:, 2] = field.at(xyz[:, :2], xyz[:, 2])                    # 중심선도 같은 높이장으로
    field = RoadField(road_xyz)

    for rid, xyz in road_xyz.items():
        xy, z = xyz[:, :2], xyz[:, 2]
        mesh.tag += 1
        ribbon(mesh, xyz, field)
        seg = np.hypot(*np.diff(xy, axis=0).T)
        grades += list(np.abs(np.diff(z)) / np.maximum(seg, 1e-6))
        # 깎기 검사점: 중심선 + 양 끝 (폭 방향)
        d = np.diff(xy, axis=0); d = np.vstack([d, d[-1:]])
        d /= np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-9)[:, None]
        nrm = np.stack([-d[:, 1], d[:, 0]], axis=1) * (ROAD_WIDTH_M / 2)
        for off in (0, 1, -1):
            all_pts.append(np.column_stack([xy + off * nrm, z]))
    for nid, (x, y) in node_xy.items():
        mesh.tag += 1
        pad(mesh, x, y, node_z[nid], field=field)
        all_pts.append(np.array([[x + NODE_PAD_R * math.cos(a), y + NODE_PAD_R * math.sin(a), node_z[nid]]
                                 for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)] + [[x, y, node_z[nid]]]))

    # 턱 검사: 서로 다른 도로의 꼭짓점이 1.5 m 안에 겹치는데 높이가 다른 곳 (입체교차 BLEND_DZ_M 이상 제외)
    from scipy.spatial import cKDTree
    V = np.array(mesh.v)
    owner = np.array(mesh.owner)
    pr = cKDTree(V[:, :2]).query_pairs(1.5, output_type="ndarray")
    pr = pr[owner[pr[:, 0]] != owner[pr[:, 1]]]
    dzv = np.abs(V[pr[:, 0], 2] - V[pr[:, 1], 2])
    ledge = dzv[dzv < BLEND_DZ_M]
    print(f"겹치는 꼭짓점 {len(pr)} 쌍: 높이차 최대 {ledge.max() if len(ledge) else 0:.2f} m, "
          f">0.10 m {(ledge > 0.10).sum()} 쌍, 입체교차로 둔 곳 {(dzv >= BLEND_DZ_M).sum()} 쌍")

    OUT_MODEL.mkdir(parents=True, exist_ok=True)
    mesh.write(OUT_MODEL / "roads.obj", "roads")
    for old in ("roads_visual.obj", "roads_collision.obj"):
        (OUT_MODEL / old).unlink(missing_ok=True)

    # 지형 깎기 → 새 heightmap (높이 범위가 바뀌면 pose z·size z 도 바뀐다)
    pts = np.vstack(all_pts)
    before = hm.surface(pts[:, 0], pts[:, 1]) - pts[:, 2]
    z0 = hm.z.copy()
    hm.carve(pts)
    dz = (z0 - hm.z)[(z0 - hm.z) > 0]
    after = hm.surface(pts[:, 0], pts[:, 1]) - pts[:, 2]
    hm.save(OUT_MODEL / "heightmap.png")
    for f in ("texture.png", "normal.png"):
        shutil.copy(SRC_MODEL / f, OUT_MODEL / f)

    # 월드 SDF: UAV 월드를 복사해 모델 경로·높이만 바꾸고 도로 모델을 더한다
    w = sdf.replace("model://kangwon/", "model://kangwon_ugv/")
    w = re.sub(r'<world name="[^"]+">', f'<world name="{WORLD_NAME}">', w, count=1)
    old_z = f"{float(re.search(r'<size>[^ ]+ [^ ]+ ([^<]+)</size>', sdf).group(1)):.2f}"
    w = re.sub(r"<size>([^ <]+) ([^ <]+) [^<]+</size>", rf"<size>\1 \2 {hm.sz:.2f}</size>", w)
    w = re.sub(r"(<pose>[^ ]+ [^ ]+ )[^ ]+( 0 0 0</pose>)", rf"\g<1>{hm.oz:.2f}\g<2>", w, count=1)   # collision pose
    w = re.sub(r"(<pos>(?!0 0 0<)[^ ]+ [^ ]+ )[^<]+(</pos>)", rf"\g<1>{hm.oz:.2f}\g<2>", w)          # visual pos (collision 은 0 0 0 유지)
    roads_model = """
    <model name="ugv_roads">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><mesh><uri>model://kangwon_ugv/roads.obj</uri></mesh></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>model://kangwon_ugv/roads.obj</uri></mesh></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.33 0.33 0.35 1</diffuse>
                    <specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
    </model>
"""
    w = w.replace("</world>", roads_model + "  </world>", 1)
    OUT_SDF.write_text(w, encoding="utf-8")

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
    print(f"도로 메시: {len(mesh.f)} 삼각형 (시각·충돌 공용) → {OUT_MODEL.relative_to(ROOT)}/roads.obj")
    print(f"경사: 최대 {g.max()*100:.1f}%  >6% 구간 {(g > 0.06).sum()}/{len(g)}  >8% {(g > 0.0801).sum()}  >10% {(g > 0.10).sum()}"
          f"  (상한 불가로 선형 처리한 도로 {len(linear)}개)")
    print(f"지형이 도로 면을 뚫던 점: 깎기 전 {(before > -CARVE_CLEAR_M).sum()}/{len(pts)} → 후 "
          f"{(after > -CARVE_CLEAR_M + 0.01).sum()}  (낮춘 꼭짓점 {dz.size}개, 평균 {dz.mean():.1f} m, 최대 {dz.max():.1f} m)")
    print(f"heightmap z 범위 {old_z} → {hm.sz:.2f} m, 바닥 {hm.oz:.2f}  → {OUT_SDF.relative_to(ROOT)}")
    for rid, p in spawn.items():
        print(f"  스폰 {rid}: ({p['x']}, {p['y']}, {p['z']}) yaw {p['yaw']}")
    if linear:
        print("  선형 처리 도로:", ", ".join(linear[:10]), "…" if len(linear) > 10 else "")


if __name__ == "__main__":
    build()
