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
