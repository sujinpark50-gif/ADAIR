# -*- coding: utf-8 -*-
"""지형 반영 속도 계획 후보 (ugv/terrain_speed.py) — 순수 함수와 횡경사 부호 판정 (합성 지형)."""
import math

import pytest

from ugv import terrain_speed as TS


def test_a_eff_reduces_with_downhill_and_has_floor():
    assert TS.a_eff(1.5, 0.0) == 1.5
    assert TS.a_eff(1.5, 0.05) == pytest.approx(1.5 - 0.49)
    assert TS.a_eff(1.5, 0.5) == TS.A_EFF_MIN


def test_v_for_stop_dist_inverts_stop_dist():
    for v, a in ((9.2, 1.5), (16.7, 0.86), (5.0, 2.0)):
        assert TS.v_for_stop_dist(TS.stop_dist(v, a), a) == pytest.approx(v, rel=1e-9)


def test_downhill_cap_keeps_flat_stop_distance():
    v_flat, a = 16.7, 1.5
    a_d = TS.a_eff(a, 0.0665)
    v_d = TS.v_for_stop_dist(TS.stop_dist(v_flat, a), a_d)
    assert v_d < v_flat and TS.stop_dist(v_d, a_d) == pytest.approx(TS.stop_dist(v_flat, a))


class _Plane:
    """z = sx·x + sy·y 평면."""
    def __init__(self, sx, sy):
        self.sx, self.sy = sx, sy

    def z(self, x, y):
        return self.sx * x + self.sy * y


@pytest.mark.parametrize("turn,cross_left_high,adverse", [(+1, True, True), (+1, False, False), (-1, True, False), (-1, False, True)])
def test_bank_adverse_sign(monkeypatch, turn, cross_left_high, adverse):
    # 동쪽(+x)으로 달림 → 왼쪽 = 북(+y). 북쪽이 높으면 cross > 0
    monkeypatch.setattr(TS, "_SURF", _Plane(0.0, 0.04 if cross_left_high else -0.04))
    pts = [(float(x), 0.0) for x in range(0, 50, 5)]
    cum = [p[0] for p in pts]
    prof = TS.profile_along(pts, cum, lambda i: 0.02 * turn)
    adv = prof[3][1]
    assert (adv > 0.03) == adverse and prof[3][2] is True


def test_unknown_terrain_is_conservative(monkeypatch):
    monkeypatch.setattr(TS, "_SURF", False)
    prof = TS.profile_along([(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)], [0, 5, 10], lambda i: 0.0)
    assert all(p == (TS.GRADE_UNKNOWN, TS.BANK_UNKNOWN, False) for p in prof)


def test_downhill_grade_sign(monkeypatch):
    monkeypatch.setattr(TS, "_SURF", _Plane(-0.05, 0.0))   # 동쪽으로 갈수록 낮다 = 하강
    pts = [(float(x), 0.0) for x in range(0, 50, 5)]
    prof = TS.profile_along(pts, [p[0] for p in pts], lambda i: 0.0)
    assert prof[4][0] == pytest.approx(0.05 + TS.GRADE_UNC)
    monkeypatch.setattr(TS, "_SURF", _Plane(0.05, 0.0))    # 오르막 → 하강 0 (+ 불확실성)
    prof = TS.profile_along(pts, [p[0] for p in pts], lambda i: 0.0)
    assert prof[4][0] == pytest.approx(TS.GRADE_UNC)
