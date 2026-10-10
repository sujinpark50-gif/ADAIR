# -*- coding: utf-8 -*-
"""PX4 미션 속도 의미 변환 — MAVSDK speed_m_s 는 '그 지점을 지난 뒤' 속도 (2026-10-09 실측)."""
import math

from ugv.drivers.px4 import ACCEPT_M, LEAD_BASE_M, LEAD_TIME_S, mavsdk_plan


def test_speeds_shift_one_item_earlier_and_lead_point_carries_first_segment():
    items = [(100.0, 0.0, 16.0), (140.0, 0.0, 8.0), (180.0, 0.0, 12.0), (200.0, 0.0, 3.0)]
    out = mavsdk_plan(items, (0.0, 0.0), 10.0)
    lead = LEAD_BASE_M + 10.0 * LEAD_TIME_S
    assert len(out) == 5
    assert math.isclose(out[0][0], lead) and out[0][2] == 16.0 and out[0][3]       # 첫 구간 속도는 선행 지점에
    assert [o[2] for o in out[1:]] == [8.0, 12.0, 3.0, 3.0]                        # 지점 k 에 k+1 구간 속도
    assert [o[3] for o in out] == [True, True, True, True, False]                  # 마지막만 정지


def test_no_lead_point_when_first_item_is_close():
    items = [(LEAD_BASE_M + ACCEPT_M - 1.0, 0.0, 5.0), (60.0, 0.0, 9.0)]
    out = mavsdk_plan(items, (0.0, 0.0), 0.0)
    assert [(o[0], o[2]) for o in out] == [(items[0][0], 9.0), (60.0, 9.0)]


def test_single_stop_point():
    out = mavsdk_plan([(0.0, 75.0, 16.0)], (0.0, 0.0), 16.0)
    assert out[-1] == (0.0, 75.0, 16.0, False) and out[0][2] == 16.0 and out[0][1] < 75.0


def test_first_mission_item_is_not_glued_to_start():
    """첫 미션 지점이 출발점에 붙어 출발 램프 속도(1 m/s)를 싣지 않는다 (2026-10-10 Gazebo G5b 기어감)."""
    import math
    from ugv import config
    from ugv.roads import Route, plan_speeds
    pts = [(0.0, 0.0), (0.3, 0.0), (40.0, 8.0), (120.0, 8.0)]
    sp = plan_speeds(Route(pts, [16.7] * 4, [], None, None, 0.0), 0.0)
    assert math.hypot(*sp.items[0][:2]) >= config.ITEM_FIRST_MIN_M - 1e-6
    assert sp.items[0][2] > 1.5
