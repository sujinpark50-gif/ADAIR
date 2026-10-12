# -*- coding: utf-8 -*-
"""상세 기록기 gz_detail 의 Gazebo 없는 부분: 접촉 상태 분류·힘 분해."""
import math

import pytest

from ugv.tools import gz_detail as G


def test_contact_state_classes():
    assert G.contact_state(0.0, ever=False, has_wrench=False) == G.NO_MSG
    assert G.contact_state(0.01, ever=True, has_wrench=True) == G.VALID
    assert G.contact_state(0.01, ever=True, has_wrench=False) == G.STALE      # 접촉 메시지인데 wrench 없음 → 힘 값으로 쓰지 않음
    assert G.contact_state(0.03, ever=True, has_wrench=True) == G.STALE
    assert G.contact_state(0.5, ever=True, has_wrench=True) == G.STALE
    assert G.contact_state(0.51, ever=True, has_wrench=True) == G.NO_CONTACT
    assert G.contact_state(5.0, ever=True, has_wrench=True) == G.NO_CONTACT


def test_split_force_flat_and_slope():
    fn, ft = G.split_force((0.0, 0.0, 3900.0), (0.0, 0.0, 1.0))
    assert fn == pytest.approx(3900.0) and ft == pytest.approx(0.0)
    # 10° 경사 법선, 힘은 순수 수직 → 법선력 = F cos10°, 접선 = F sin10° (세계 Z 힘과 다름)
    a = math.radians(10)
    fn, ft = G.split_force((0.0, 0.0, 1000.0), (math.sin(a), 0.0, math.cos(a)))
    assert fn == pytest.approx(1000 * math.cos(a)) and ft == pytest.approx(1000 * math.sin(a))
    # 법선 합(여러 접촉점)은 단위화 — 크기와 무관
    assert G.split_force((0, 0, 100.0), (0, 0, 4.0))[0] == pytest.approx(100.0)
    assert math.isnan(G.split_force((0, 0, 1.0), (0, 0, 0))[0])
