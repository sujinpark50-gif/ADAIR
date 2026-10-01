# ugv/tools/build_road_network.py — 실제 도로망(gpkg) → UGV 그래프 JSON 변환 (개발용, 1회 실행)
#
# 입력: environment/data/processed/roads_clipped.gpkg (도로 링크 756개, 좌표계 Katech TM128)
# 출력: ugv/data/road_network.json  ({"nodes": [...], "roads": [...], "bases": {...}})
#
# 런타임(bridge_ugv, fleet)은 JSON 만 읽으므로 geopandas 가 필요 없다.
# gpkg 가 바뀌었을 때만 다시 실행한다:
#   pip install geopandas
#   python3 -m ugv.tools.build_road_network
#
# 처리 순서
#   1) 링크의 시작/끝 노드 번호(UP_FROM_NO, UP_TO_NODE)로 연결 구조를 만든다
#   2) 노드 좌표 = 링크 선형의 끝점 (WGS84 로 변환)
#   3) 가장 큰 연결 덩어리만 남긴다 (영역 경계에서 잘린 조각 제거)
#   4) 거리 = 선형 길이(m). LENGTH 컬럼은 단위가 불명확해 쓰지 않는다
#   5) 통과시간 = 거리 / 도로등급별 속도 (잠정값, 아래 SPEED_KMH)
#   6) 거점(원통119·기린119)을 가장 가까운 노드에 붙이고 그 노드 id 를 "A"/"B" 로 바꾼다
#      → ugv/config.py 의 home_node("A"/"B")를 그대로 쓸 수 있다
#   7) 도로마다 지나는 환경 격자 칸 목록(cells: [[x, y], ...])을 기록한다
#      → 환경의 화재 셀(x=열, y=행)을 도로 차단으로 바꿀 때 쓴다 (ugv/road_status.py)

import json
import math
from pathlib import Path

import geopandas as gpd
import rasterio

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "environment/data/processed/roads_clipped.gpkg"
DST = ROOT / "ugv/data/road_network.json"
GRID = ROOT / "environment/data/processed/roads_raster.tif"   # 환경 격자 기준 (화재 CA 와 같은 격자)
CELL_SAMPLE_M = 30.0                                          # 선형을 이 간격으로 찍어 칸을 구한다 (격자 90 m)

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

# 거점 좌표 (ugv/graph_data.py 와 같은 값)
BASES = {
    "A": ("원통119", 38.1155, 128.1566),
    "B": ("기린119", 37.9430, 128.2760),
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


def _cells_on_line(line, left, top, res, width, height):
    """선형이 지나는 격자 칸 [x(열), y(행)] 목록. 순서 유지, 중복 제거."""
    cells, n = [], max(1, int(line.length // CELL_SAMPLE_M))
    for i in range(n + 1):
        p = line.interpolate(line.length * i / n)
        x, y = int((p.x - left) // res), int((top - p.y) // res)
        if 0 <= x < width and 0 <= y < height and [x, y] not in cells:
            cells.append([x, y])
    return cells


def build():
    links = gpd.read_file(SRC)
    length_m = links.geometry.length          # TM128 은 미터 단위 투영좌표계
    wgs = links.to_crs("EPSG:4326")
    with rasterio.open(GRID) as g:
        grid_geom = links.to_crs(g.crs).geometry
        left, top, res = g.transform.c, g.transform.f, g.transform.a
        width, height = g.width, g.height

    nodes, roads, names = {}, [], {}
    for i, row in wgs.iterrows():
        a, b = str(row["UP_FROM_NO"]), str(row["UP_TO_NODE"])
        line = row.geometry.geoms[0] if row.geometry.geom_type == "MultiLineString" else row.geometry
        (lon_a, lat_a), (lon_b, lat_b) = line.coords[0][:2], line.coords[-1][:2]
        nodes.setdefault(a, (lat_a, lon_a))
        nodes.setdefault(b, (lat_b, lon_b))

        name = row["ROAD_NAME"] if isinstance(row["ROAD_NAME"], str) else ""
        for n in (a, b):
            if name and n not in names:
                names[n] = name

        dist = float(length_m.iloc[i])
        speed = SPEED_KMH.get(str(row["ROAD_RANK"]), DEFAULT_SPEED_KMH)
        roads.append({
            "road_id": str(row["LINK_ID"]),
            "node_a": a,
            "node_b": b,
            "distance_m": round(dist, 1),
            "base_time_s": round(dist / (speed / 3.6), 1),
            "cells": [c for part in getattr(grid_geom.iloc[i], "geoms", [grid_geom.iloc[i]])
                      for c in _cells_on_line(part, left, top, res, width, height)],
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
        r["node_a"], r["node_b"] = rid(r["node_a"]), rid(r["node_b"])

    DST.parent.mkdir(parents=True, exist_ok=True)
    DST.write_text(json.dumps(
        {"source": SRC.name, "speed_kmh": SPEED_KMH, "bases": bases,
         "nodes": out_nodes, "roads": roads},
        ensure_ascii=False, indent=1), encoding="utf-8")

    total_km = sum(r["distance_m"] for r in roads) / 1000
    print(f"노드 {len(out_nodes)}  도로 {len(roads)}  총 {total_km:.1f} km  → {DST.relative_to(ROOT)}")
    for b, info in bases.items():
        print(f"  거점 {b} {info['name']}: 노드 {info['source_node']} 에 스냅 ({info['snap_m']} m)")


if __name__ == "__main__":
    build()
