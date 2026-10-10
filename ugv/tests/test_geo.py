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


def test_gazebo_enu_frame_matches_gazebo_navsat_far_from_origin():
    """Gazebo navsat 은 월드 좌표를 datum ECEF 접평면 ENU 로 위경도로 바꾼다. 도로 월드는 TM(FRAME) 좌표라 원점에서 멀면 어긋난다.
    2026-10-10 실측: Gazebo (16829.98, -7262.30) 에 정지한 차의 PX4 위경도 → FRAME (16829.495, -7262.10) — 0.52 m.
    시뮬레이터 PX4 드라이버는 GZ_FRAME 으로 바꿔야 Gazebo 도로 위 같은 자리에 선다."""
    from ugv.geo import FRAME, GZ_FRAME
    la, lo = GZ_FRAME.to_ll(16829.98, -7262.30)
    x, y = FRAME.to_xy(la, lo)
    assert abs(x - 16829.495) < 0.01 and abs(y + 7262.10) < 0.01
    for p in ((16829.98, -7262.30), (4000.0, 5600.0), (-20000.0, 25000.0), (0.0, 0.0)):
        q = GZ_FRAME.to_xy(*GZ_FRAME.to_ll(*p))
        assert abs(q[0] - p[0]) < 1e-3 and abs(q[1] - p[1]) < 1e-3
    # 거점 A 부근(원점에서 약 7 km)도 0.19 m 어긋난다 (A 평탄 월드 시험에도 영향)
    xa, ya = FRAME.to_xy(*GZ_FRAME.to_ll(4000.0, 5600.0))
    assert 0.1 < math.hypot(xa - 4000.0, ya - 5600.0) < 0.25


def test_px4_driver_uses_gazebo_frame_only_for_gazebo_vehicles():
    from ugv.clock import ManualClock
    from ugv.drivers import build_driver
    from ugv.geo import FRAME, GZ_FRAME
    sim = build_driver({"driver": "px4", "px4": {"address": "udpin://0.0.0.0:14599", "gz_world": "kangwon_flat"}}, (0, 0), ManualClock())
    real = build_driver({"driver": "px4", "px4": {"address": "udpin://0.0.0.0:14598"}}, (0, 0), ManualClock())
    assert sim.frame is GZ_FRAME and real.frame is FRAME
