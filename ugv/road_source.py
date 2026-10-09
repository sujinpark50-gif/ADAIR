# -*- coding: utf-8 -*-
"""ugv/road_source.py — 원자료 도로망(GeoPackage)을 직접 읽는다. 변환본 파일을 따로 두지 않는다.

원자료: data/inje2019/roads/roads_clipped_2019.gpkg (표준 노드·링크, 환경 격자 범위로 잘림, EPSG:5186)
  - 링크 하나 = 도로 하나. 노드 = UP_FROM_NO(a) → UP_TO_NODE(b). 형상은 a → b 순서
  - 일방통행: ONEWAY=1 이면 a → b 만 (attrs.oneway = A_TO_B)
  - 차로 수: LANES / UP_LANES / DOWN_LANES. 도로 폭 추정에 쓴다 (drive_graph.est_half_width)
  - BARRIER·WIDTH 는 코드 의미·단위가 확인되지 않아 원본 값만 attrs 에 둔다
  - 자동차전용 도로(AUTO_EXCLU=1, 이 범위에서는 서울양양고속도로)는 config.EXCLUDE_MOTORWAY 면 뺀다
  - 범위 경계에서 잘린 링크 끝은 원래 노드 번호를 쓰지 않는다. 경계 밖 같은 노드를 공유하던 다른 링크와 실제로는
    이어져 있지 않기 때문이다 (원자료에 12곳) → 'EDGE:<링크>:<a|b>' 노드로 따로 둔다
  - 제한속도: MAX_SPD 가 있으면 그 값, 없으면 도로 등급 기본값 (config.ROAD_RANK_SPEED_KMH)
"""

import sqlite3
import struct
from typing import Dict, List, Tuple

from . import config

COLUMNS = ("LINK_ID", "UP_FROM_NO", "UP_TO_NODE", "ONEWAY", "LANES", "UP_LANES", "DOWN_LANES", "BARRIER", "WIDTH",
           "ROAD_RANK", "ROAD_NAME", "MAX_SPD", "AUTO_EXCLU", "geom")
END_MATCH_M = 1.0        # 같은 노드 번호의 끝점끼리 이 거리 안이면 같은 점
CLIP_EDGE_M = 1.0        # 범위 경계에서 이 거리 안이면 잘린 끝


def _gpkg_lines(blob: bytes) -> List[Tuple[float, float]]:
    """GeoPackage 형상 → (x, y) 목록. LINESTRING·MULTILINESTRING (Z·M 포함). 여러 조각이면 이어 붙인다."""
    if blob[:2] != b"GP":
        raise ValueError("GeoPackage 형상이 아님")
    envelope = (blob[3] >> 1) & 0x07
    wkb = memoryview(blob)[8 + (0, 32, 48, 48, 64)[envelope]:]
    out: List[Tuple[float, float]] = []

    def read(off: int) -> int:
        order = "<" if wkb[off] == 1 else ">"
        gtype = struct.unpack_from(order + "I", wkb, off + 1)[0]
        off += 5
        kind, flavour = gtype % 1000, gtype // 1000
        dims = {0: 2, 1: 3, 2: 3, 3: 4}[flavour]
        if kind == 2:
            count = struct.unpack_from(order + "I", wkb, off)[0]
            off += 4
            for _ in range(count):
                xy = struct.unpack_from(order + "dd", wkb, off)
                if not out or out[-1] != xy:
                    out.append(xy)
                off += 8 * dims
            return off
        if kind == 5:
            parts = struct.unpack_from(order + "I", wkb, off)[0]
            off += 4
            for _ in range(parts):
                off = read(off)
            return off
        raise ValueError(f"지원하지 않는 형상 종류 {gtype}")

    read(0)
    return out


def load(path=None) -> dict:
    """{"nodes": {id: (lat, lon)}, "roads": [...], "clip_extent": {...}, "source": 파일명, "excluded": {...}}"""
    from rasterio.warp import transform           # 좌표계 변환 (EPSG:5186 → WGS84)

    path = path or config.ROAD_GPKG
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        table, minx, miny, maxx, maxy, srs = db.execute(
            "select table_name, min_x, min_y, max_x, max_y, srs_id from gpkg_contents where data_type='features'").fetchone()
        rows = db.execute(f"select {', '.join(COLUMNS)} from \"{table}\"").fetchall()
    finally:
        db.close()

    links, excluded = [], {"motorway": 0}
    for row in rows:
        rec = dict(zip(COLUMNS, row))
        if config.EXCLUDE_MOTORWAY and str(rec["AUTO_EXCLU"]) == "1":
            excluded["motorway"] += 1
            continue
        pts = _gpkg_lines(rec.pop("geom"))
        if len(pts) >= 2:
            links.append((rec, pts))

    # 링크 끝 → 노드. 같은 번호의 다른 끝과 떨어져 있고 범위 경계에 붙은 끝은 잘린 끝
    def on_edge(p):
        return min(p[0] - minx, maxx - p[0], p[1] - miny, maxy - p[1]) <= CLIP_EDGE_M

    ends: Dict[str, List[Tuple[float, float]]] = {}
    for rec, pts in links:
        ends.setdefault(str(rec["UP_FROM_NO"]), []).append(pts[0])
        ends.setdefault(str(rec["UP_TO_NODE"]), []).append(pts[-1])

    def node_of(nid: str, p, link, side):
        others = [q for q in ends[nid] if q is not p]
        meets = any(((q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2) ** 0.5 <= END_MATCH_M for q in others)
        return f"EDGE:{link}:{side}" if others and not meets and on_edge(p) else nid

    node_xy: Dict[str, Tuple[float, float]] = {}
    roads = []
    for rec, pts in links:
        lid = str(rec["LINK_ID"])
        a = node_of(str(rec["UP_FROM_NO"]), pts[0], lid, "a")
        b = node_of(str(rec["UP_TO_NODE"]), pts[-1], lid, "b")
        node_xy.setdefault(a, pts[0])
        node_xy.setdefault(b, pts[-1])
        try:
            vmax = float(rec["MAX_SPD"] or 0)
        except ValueError:
            vmax = 0.0
        rank = int(rec["ROAD_RANK"] or 0)
        speed = vmax if vmax > 0 else config.ROAD_RANK_SPEED_KMH.get(rank, min(config.ROAD_RANK_SPEED_KMH.values()))
        roads.append({"road_id": lid, "node_a": a, "node_b": b, "name": rec["ROAD_NAME"] or "", "speed_kmh": speed,
                      "xy5186": pts,
                      "attrs": {"oneway": "A_TO_B" if str(rec["ONEWAY"]) == "1" else "BOTH", "lanes": rec["LANES"],
                                "up_lanes": rec["UP_LANES"], "down_lanes": rec["DOWN_LANES"], "road_rank": rank,
                                "barrier_code": rec["BARRIER"], "width_code": rec["WIDTH"]}})

    # 좌표 변환 한 번에
    flat = [p for r in roads for p in r["xy5186"]] + list(node_xy.values())
    lons, lats = transform(f"EPSG:{srs}", "EPSG:4326", [p[0] for p in flat], [p[1] for p in flat])
    k = 0
    for r in roads:
        n = len(r["xy5186"])
        r["geometry"] = list(zip(lats[k:k + n], lons[k:k + n]))
        k += n
        del r["xy5186"]
    nodes = {}
    for nid in node_xy:
        nodes[nid] = (lats[k], lons[k])
        k += 1
    return {"source": str(path), "nodes": nodes, "roads": roads, "excluded": excluded,
            "clip_extent": {"min_x": minx, "min_y": miny, "max_x": maxx, "max_y": maxy, "srs_id": srs}}
