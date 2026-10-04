# ugv/tools/build_road_network.py — 실제 도로망(gpkg) → UGV 그래프 JSON 변환 (개발용, 1회 실행)
#
# 입력: data/inje2019/roads/roads_clipped_2019.gpkg (도로 링크 901개, 환경 2019 트윈과 같은 파일, EPSG:5186, 환경 격자와 같은 좌표계)
#       uav/gazebo/kangwon.sdf + models/kangwon/heightmap.png (Gazebo 지형 — 스폰 높이 계산용)
# 출력: ugv/data/road_network.json
#       {"nodes": [...], "roads": [...], "bases": {...}, "spawn": {...}, "world": {...}}
#
# 런타임(server, fleet, bridge_ugv)은 JSON 만 읽으므로 geopandas 가 필요 없다.
# gpkg·월드·거점·자원이 바뀌었을 때만 다시 실행한다:
#   pip install geopandas rasterio pyproj pillow
#   python3 -m ugv.tools.build_road_network
#
# 처리 순서
#   1) 연결 구조: 링크의 시작/끝 노드 번호(UP_FROM_NO, UP_TO_NODE). 컬럼이 없으면 선형 끝점 좌표(1 m 반올림)로 대신한다
#   2) 노드 좌표 = 링크 선형의 끝점 (WGS84 로 변환)
#   3) 가장 큰 연결 덩어리만 남긴다 (영역 경계에서 잘린 조각 제거)
#   4) 거리 = 선형 길이(m, 5186). LENGTH 컬럼은 단위가 불명확해 쓰지 않는다
#   5) 통과시간 = 거리 / 도로등급별 속도 (잠정값, 아래 SPEED_KMH)
#   6) 거점(인제119·기린119)을 가장 가까운 노드에 붙이고 그 노드 id 를 "A"/"B" 로 바꾼다
#      → ugv/config.py 의 home_node("A"/"B")를 그대로 쓸 수 있다
#   7) 도로마다 지나는 환경 격자 칸 목록(cells: [[x, y], ...])을 기록한다 (화재 셀 ↔ 도로 대응용)
#   8) 도로 선형(geometry: [[lat, lon], ...], node_a → node_b 방향, 3 m 단순화)을 기록한다
#      → 차량이 노드 사이 직선이 아니라 실제 도로를 따라 달리게 한다 (ugv/agent.py)
#   9) 자원별 Gazebo 스폰 자세(spawn: x, y, z, yaw)를 기록한다 — ugv/tools/px4-start.sh 가 읽는다
#      x, y = UAV 월드와 같은 datum 중심 TM(build_world.py 의 LOCAL), z = heightmap 지면 + SPAWN_Z_OFFSET_M

import json
import math
import os
import re
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np
import rasterio
from PIL import Image
from pyproj import Transformer
from shapely.geometry import LineString
from shapely.ops import linemerge

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from ugv.config import RESOURCES  # noqa: E402  (스폰 자세를 자원별로 뽑는다)

# 도로 원본: 환경 2019 트윈과 같은 파일 (901 링크, 2019 표준노드링크). 예전 756 링크로 돌리려면
#   UGV_ROADS_SRC=environment/data/processed/roads_reprojected.gpkg UGV_ROADS_GRID=environment/data/processed/roads_raster.tif
SRC = ROOT / os.getenv("UGV_ROADS_SRC", "data/inje2019/roads/roads_clipped_2019.gpkg")
DST = ROOT / "ugv/data/road_network.json"
GRID = ROOT / os.getenv("UGV_ROADS_GRID", "data/inje2019/roads/roads_raster_2019.tif")   # 환경 격자 기준 (화재 CA 와 같은 격자, 308x236, 90 m)
WORLD_SDF = ROOT / "uav/gazebo/kangwon.sdf"
HEIGHTMAP = ROOT / "uav/gazebo/models/kangwon/heightmap.png"
CELL_SAMPLE_M = 30.0        # 선형을 이 간격으로 찍어 칸을 구한다 (격자 90 m)
SIMPLIFY_M = 3.0            # 도로 선형 단순화 허용오차. 주행 경로점 간격(ugv/drivers/px4.py 10 m)보다 충분히 작게
SPAWN_Z_OFFSET_M = 0.3      # 지면 위로 띄워 스폰 (지형에 박히지 않게)
SPAWN_GAP_M = 8.0           # 같은 거점 두 번째 차량부터 도로 진행방향 뒤쪽으로 이만큼씩 물려 세운다

# gz_bridge.py · uav/gazebo/build_world.py 와 같은 datum. 월드 원점 = 이 점, 월드 z=0 = 해발 DATUM_ALT
DATUM_LAT, DATUM_LON, DATUM_ALT = 38.011200, 128.122643, 172.1
LOCAL = (f"+proj=tmerc +lat_0={DATUM_LAT} +lon_0={DATUM_LON} +k=1 "
         "+x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs")

# 도로등급(ROAD_RANK)별 소방차 주행속도 km/h — 잠정값, 팀 합의 필요
SPEED_KMH = {
    "101": 80,  # 고속국도
    "102": 70,  # 도시고속
    "103": 60,  # 일반국도
    "104": 50,  # 특별·광역시도
    "105": 50,  # 국가지원지방도
    "106": 50,  # 지방도
    "107": 30,  # 시군도
    "108": 20,  # 기타
}
DEFAULT_SPEED_KMH = 30

# 거점 좌표 — 주소로 추정한 값 (인제소방서 홈페이지 주소, 좌표는 도로명주소 기초번호로 역산)
#   인제119 : 인제읍 비봉로44번길 71 → 비봉로44번길 선형에서 추정
#   기린119 : 기린면 대내로 34-14   → 대내로 동쪽 시점에서 약 340 m 지점으로 추정 (±200 m, 현장 확인 필요)
BASES = {
    "A": ("인제119", 38.06180, 128.16826),
    "B": ("기린119", 37.96370, 128.31760),
}


def _haversine_m(lat1, lon1, lat2, lon2):
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def _largest_component(edges):
    adj = {}
    for a, b in edges:
        adj.setdefault(a, set()).add(b)
        adj.setdefault(b, set()).add(a)
    seen, best = set(), set()
    for start in adj:
        if start in seen:
            continue
        comp, stack = set(), [start]
        while stack:
            u = stack.pop()
            if u in comp:
                continue
            comp.add(u)
            stack.extend(adj[u] - comp)
        seen |= comp
        if len(comp) > len(best):
            best = comp
    return best


def _single_line(geom):
    """MultiLineString 이면 이어 붙여 LineString 하나로. 끊겨 있으면 가장 긴 조각."""
    if geom.geom_type == "LineString":
        return geom
    merged = linemerge(geom)
    if merged.geom_type == "LineString":
        return merged
    return max(merged.geoms, key=lambda g: g.length)


def _cells_on_line(line, left, top, res, width, height):
    """선형이 지나는 격자 칸 [x(열), y(행)] 목록. 순서 유지, 중복 제거."""
    cells, n = [], max(1, int(line.length // CELL_SAMPLE_M))
    for i in range(n + 1):
        p = line.interpolate(line.length * i / n)
        x, y = int((p.x - left) // res), int((top - p.y) // res)
        if 0 <= x < width and 0 <= y < height and [x, y] not in cells:
            cells.append([x, y])
    return cells


class Terrain:
    """kangwon.sdf 의 collision heightmap 과 같은 방식으로 월드 (x, y) → 지면 z."""

    def __init__(self):
        sdf = WORLD_SDF.read_text(encoding="utf-8")
        blk = sdf[sdf.index('<model name="kangwon_terrain_collision">'):]
        pose = [float(v) for v in re.search(r"<pose>([^<]+)</pose>", blk).group(1).split()[:3]]
        size = [float(v) for v in re.search(r"<size>([^<]+)</size>", blk).group(1).split()]
        self.ox, self.oy, self.oz = pose
        self.sx, self.sy, self.sz = size
        self.h = np.asarray(Image.open(HEIGHTMAP), dtype=np.float64) / 65535.0
        self.n = self.h.shape[0]

    def z(self, x, y):
        d = self.sx / (self.n - 1)
        fc = (x - (self.ox - self.sx / 2)) / d          # 열0 = 서쪽
        fr = ((self.oy + self.sy / 2) - y) / d          # 행0 = 북쪽(+Y)
        c0, r0 = int(fc), int(fr)
        if not (0 <= c0 < self.n - 1 and 0 <= r0 < self.n - 1):
            raise ValueError(f"({x:.0f}, {y:.0f}) 가 지형 밖")
        dc, dr = fc - c0, fr - r0
        h = self.h
        v = (h[r0, c0] * (1 - dc) * (1 - dr) + h[r0, c0 + 1] * dc * (1 - dr)
             + h[r0 + 1, c0] * (1 - dc) * dr + h[r0 + 1, c0 + 1] * dc * dr)
        return self.oz + v * self.sz


def _spawn_poses(nodes_ll, roads):
    """자원별 스폰 자세. 거점 노드에서 첫 도로 방향을 바라보고, 같은 거점 차량은 뒤로 물려 세운다."""
    to_local = Transformer.from_crs("EPSG:4326", LOCAL, always_xy=True)
    terrain = Terrain()
    out, per_base = {}, {}
    for r in RESOURCES:
        node = r["home_node"]
        lat, lon = nodes_ll[node]
        # 거점에서 나가는 가장 긴 도로의 두 번째 점 → 진행방향
        leaving = [rd for rd in roads if node in (rd["node_a"], rd["node_b"])]
        rd = max(leaving, key=lambda x: x["distance_m"])
        geo = rd["geometry"] if rd["node_a"] == node else rd["geometry"][::-1]
        x0, y0 = to_local.transform(lon, lat)
        x1, y1 = to_local.transform(geo[1][1], geo[1][0])
        yaw = math.atan2(y1 - y0, x1 - x0)             # ENU, 0 = 동쪽
        k = per_base.get(node, 0)
        per_base[node] = k + 1
        x = x0 - math.cos(yaw) * SPAWN_GAP_M * k
        y = y0 - math.sin(yaw) * SPAWN_GAP_M * k
        z = terrain.z(x, y) + SPAWN_Z_OFFSET_M
        out[r["resource_id"]] = {"x": round(x, 1), "y": round(y, 1), "z": round(z, 1),
                                 "yaw": round(yaw, 3), "node": node}
    return out


def build():
    links = gpd.read_file(SRC)
    if links.crs is None or links.crs.to_epsg() != 5186:
        links = links.to_crs("EPSG:5186")               # 5186 은 미터 단위 → 길이·격자 계산을 여기서
    has_topo = {"UP_FROM_NO", "UP_TO_NODE"} <= set(links.columns)
    to_wgs = Transformer.from_crs("EPSG:5186", "EPSG:4326", always_xy=True)
    with rasterio.open(GRID) as g:
        if g.crs.to_epsg() != 5186:
            raise SystemExit(f"격자 좌표계가 5186 이 아님: {g.crs}")
        left, top, res = g.transform.c, g.transform.f, g.transform.a
        width, height = g.width, g.height

    def ll(x, y):
        lon, lat = to_wgs.transform(x, y)
        return lat, lon

    nodes, roads, names = {}, [], {}
    for i, row in links.iterrows():
        line = _single_line(row.geometry)
        (xa, ya), (xb, yb) = line.coords[0][:2], line.coords[-1][:2]
        if has_topo:
            a, b = str(row["UP_FROM_NO"]), str(row["UP_TO_NODE"])
        else:   # 링크 컬럼이 없는 gpkg: 끝점 좌표를 1 m 로 반올림해 같은 점을 같은 노드로 본다
            a, b = f"p{round(xa)}_{round(ya)}", f"p{round(xb)}_{round(yb)}"
        if a == b:
            continue
        nodes.setdefault(a, ll(xa, ya))
        nodes.setdefault(b, ll(xb, yb))

        name = row.get("ROAD_NAME")
        name = name if isinstance(name, str) else ""
        for n in (a, b):
            if name and n not in names:
                names[n] = name

        dist = float(line.length)
        speed = SPEED_KMH.get(str(row.get("ROAD_RANK")), DEFAULT_SPEED_KMH)
        simple = line.simplify(SIMPLIFY_M, preserve_topology=False)
        roads.append({
            "road_id": str(row["LINK_ID"]) if "LINK_ID" in links.columns else f"L{i}",
            "node_a": a,
            "node_b": b,
            "name": name,
            "distance_m": round(dist, 1),
            "speed_kmh": speed,
            "base_time_s": round(dist / (speed / 3.6), 1),
            "cells": _cells_on_line(line, left, top, res, width, height),
            "geometry": [[round(v, 6) for v in ll(x, y)] for x, y, *_ in simple.coords],
        })

    keep = _largest_component((r["node_a"], r["node_b"]) for r in roads)
    roads = [r for r in roads if r["node_a"] in keep]

    # 거점 스냅: 가장 가까운 노드의 id 를 "A"/"B" 로 바꾼다
    rename, bases = {}, {}
    for base_id, (label, lat, lon) in BASES.items():
        nid = min(keep, key=lambda n: _haversine_m(lat, lon, *nodes[n]))
        snap = _haversine_m(lat, lon, *nodes[nid])
        rename[nid] = base_id
        names[nid] = label
        bases[base_id] = {"name": label, "lat": lat, "lon": lon,
                          "source_node": nid, "snap_m": round(snap, 1)}

    def rid(n):
        return rename.get(n, n)

    out_nodes = [
        {"node_id": rid(n), "lat": round(nodes[n][0], 6), "lon": round(nodes[n][1], 6),
         "name": names.get(n, "")}
        for n in sorted(keep)
    ]
    for r in roads:
        # 선형 양 끝을 노드 좌표에 정확히 맞춘다 (같은 노드를 공유하는 도로끼리 끝점이 어긋나지 않게)
        la, lb = nodes[r["node_a"]], nodes[r["node_b"]]
        r["geometry"][0] = [round(la[0], 6), round(la[1], 6)]
        r["geometry"][-1] = [round(lb[0], 6), round(lb[1], 6)]
        r["node_a"], r["node_b"] = rid(r["node_a"]), rid(r["node_b"])

    nodes_ll = {rid(n): nodes[n] for n in keep}
    spawn = _spawn_poses(nodes_ll, roads)

    DST.parent.mkdir(parents=True, exist_ok=True)
    DST.write_text(json.dumps(
        {"source": SRC.name, "speed_kmh": SPEED_KMH, "bases": bases,
         "world": {"sdf": "uav/gazebo/kangwon.sdf", "name": "kangwon",
                   "datum": [DATUM_LAT, DATUM_LON, DATUM_ALT]},
         "spawn": spawn, "nodes": out_nodes, "roads": roads},
        ensure_ascii=False, indent=1), encoding="utf-8")

    total_km = sum(r["distance_m"] for r in roads) / 1000
    pts = sum(len(r["geometry"]) for r in roads)
    print(f"노드 {len(out_nodes)}  도로 {len(roads)}  총 {total_km:.1f} km  선형점 {pts}  → {DST.relative_to(ROOT)}")
    for b, info in bases.items():
        print(f"  거점 {b} {info['name']}: 노드 {info['source_node']} 에 스냅 ({info['snap_m']} m)")
    for rid_, p in spawn.items():
        print(f"  스폰 {rid_}: x={p['x']} y={p['y']} z={p['z']} yaw={p['yaw']}")


if __name__ == "__main__":
    build()
