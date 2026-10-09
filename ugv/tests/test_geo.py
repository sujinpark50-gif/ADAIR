# -*- coding: utf-8 -*-
"""지역 평면 = datum 중심 횡메르카토르 — PROJ 와 같은지, 스폰 자세 도구가 출입 지점을 같은 평면으로 내는지."""
import math
import random

import pytest

from ugv.drive_graph import access_point
from ugv.geo import DATUM_LAT, DATUM_LON, FRAME
from ugv.roads import RoadNetwork


def test_spawn_pose_is_access_point_on_same_plane(capsys):
    from ugv.tools import spawn_pose
    import sys
    argv, sys.argv = sys.argv, ["spawn_pose", "A"]
    try:
        assert spawn_pose.main() == 0
    finally:
        sys.argv = argv
    x, y, z, yaw = map(float, capsys.readouterr().out.strip().split(","))
    pos, heading, _ = access_point(RoadNetwork(), "A")
    assert abs(x - pos.x) < 0.01 and abs(y - pos.y) < 0.01 and z == 0.4
    assert abs((90.0 - math.degrees(yaw) - heading + 180) % 360 - 180) < 0.01   # ENU yaw ↔ 방위


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
