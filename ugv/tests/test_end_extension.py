"""E4 목적지 조기 정지 보정 — 서버 마지막 항목 연장 (vehicle._end_extend, Fable 미해결 1.3 G1~G5).
sim 드라이버는 반경 경계 정지를 하지 않아 기본 offset 0 — 시험에서는 그 인스턴스에 offset 을 넣어 기하·안전 조건을 본다."""
import asyncio
import math

from ugv import config
from ugv.geo import FRAME
from ugv.tests.test_multi_vehicle import FLEET_CFG, body, make_fleet


def _planned(tmp_path):
    f, clock = make_fleet(tmp_path, [FLEET_CFG[0]])
    A = f.vehicles["A-ugv1"]
    asyncio.run(A.execute(body("A1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700))))
    return f, A


def test_offset_zero_means_no_extension(tmp_path):
    f, A = _planned(tmp_path)
    assert A.driver.end_stop_offset_m == 0.0
    route, ext = A._end_extend(A.plan)
    assert route is A.plan.route and ext["applied"] is False and ext["reason"].startswith("OFFSET_0")


def test_extension_follows_arrival_road_and_keeps_service_target(tmp_path):
    f, A = _planned(tmp_path)
    A.driver.end_stop_offset_m = 3.0
    dest = A.plan.dest
    route, ext = A._end_extend(A.plan)
    assert ext["applied"] is True, ext
    end = route.pts[-1]
    assert abs(math.dist(end, (dest.x, dest.y)) - 3.0) < 0.6       # 도로를 따라 3 m (굽은 도로면 직선 거리는 약간 짧다)
    assert A.plan.dest is dest and A.plan.route.pts[-1] != end      # 서비스 목표·원래 경로는 그대로 (G5)
    assert route.arrive_toward == A.plan.route.arrive_toward
    assert ext["service_dest_xy"] == [round(dest.x, 2), round(dest.y, 2)]


def test_road_end_too_close_skips(tmp_path):
    f, A = _planned(tmp_path)
    r = A.net.roads[A.plan.dest.road_id]
    A.driver.end_stop_offset_m = r.length + 10.0          # 도로 잔여보다 긴 연장
    route, ext = A._end_extend(A.plan)
    assert route is A.plan.route and ext["applied"] is False and ext["reason"] == "ROAD_END_TOO_CLOSE"


def test_fire_standoff_skips(tmp_path):
    f, A = _planned(tmp_path)
    A.driver.end_stop_offset_m = 3.0

    class Near:
        def __len__(self):
            return 1

        def distance(self, x, y, **k):
            return config.FIRE_STANDOFF_M - 1.0
    A.fire_source.area = lambda: Near()
    route, ext = A._end_extend(A.plan)
    assert route is A.plan.route and ext["applied"] is False and ext["reason"] == "FIRE_STANDOFF"


def test_flag_off_keeps_route(tmp_path):
    import ugv.vehicle as V
    assert V.END_EXTEND is False                          # 기본 off
    f, A = _planned(tmp_path)
    assert "end_extension" not in A.plan.detail
