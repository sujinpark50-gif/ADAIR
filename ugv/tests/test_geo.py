# -*- coding: utf-8 -*-
"""지역 평면 = datum 중심 횡메르카토르 — 도로 월드 도로 면·스폰 좌표와 같은 투영인지."""
import json
import random

import pytest

from ugv import config
from ugv.geo import DATUM_LAT, DATUM_LON, FRAME


def test_matches_road_world_spawn():
    d = json.loads(config.ROAD_NETWORK_JSON.read_text(encoding="utf-8"))
    a = next(n for n in d["nodes"] if n["node_id"] == "A")
    x, y = FRAME.to_xy(a["lat"], a["lon"])
    sp = d["spawn"]["A-ugv1"]                        # 도로 월드 빌더가 같은 노드 위에 둔 스폰 (TM 로컬)
    assert abs(x - sp["x"]) < 0.1 and abs(y - sp["y"]) < 0.1


def test_matches_proj_and_roundtrips():
    warp = pytest.importorskip("rasterio.warp")
    tm = f"+proj=tmerc +lat_0={DATUM_LAT} +lon_0={DATUM_LON} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
    rnd = random.Random(1)
    for _ in range(300):
        lat, lon = rnd.uniform(37.85, 38.2), rnd.uniform(127.95, 128.5)
        xs, ys = warp.transform("EPSG:4326", tm, [lon], [lat])
        x, y = FRAME.to_xy(lat, lon)
        assert abs(x - xs[0]) < 0.001 and abs(y - ys[0]) < 0.001
        la, lo = FRAME.to_ll(x, y)
        assert abs(la - lat) < 1e-8 and abs(lo - lon) < 1e-8
