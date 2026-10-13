"""정지 경로 항목화 후보 UGV_STOP_ITEMS_RAMP (Fable 2026-10-14 경사 정지 F1 저속 순항 검토 ③).
기존: 34 m 정지 경로 → 끝 지점 1개, 속도 = 램프 마지막 조각 √(2·a·Δ_last) (Δ_last 는 격자 위치로 0~4.9 m) → F1 상한이 1.4~2.7 m/s.
후보: 8 m 간격 램프 항목, 끝 5 m 안 비최종 항목 제거, 끝 항목 = 진입 속도."""
import math

import pytest

from ugv.profiles import PROFILES
from ugv.roads import Route, plan_speeds
from ugv.vehicle import STOP_ITEM_END_CLEAR_M, stop_ramp_items

P = PROFILES["adair_ugv"]


def straight(L, step=4.9, offset=0.0):
    """정지 경로 = 계획선 격자(≈4.9 m) 를 need 에서 자른 모양: 시작 + offset 뒤 격자점들 + 끝."""
    xs = [0.0] + [offset + k * step for k in range(1, 100) if offset + k * step < L] + [L]
    pts = [(x, 0.0) for x in xs]
    return Route(pts, [9.5] * len(pts), [], None, None, 0.0)


@pytest.mark.parametrize("dlast", [0.3, 0.67, 1.06, 1.51, 2.42, 4.5])
def test_old_rule_single_item_speed_depends_on_last_piece(dlast):
    L = 33.7
    step = 4.9
    off = (L - dlast) - step * int((L - dlast) / step)      # 마지막 격자점이 끝에서 dlast 앞에 오게
    items = plan_speeds(straight(L, step, off), 9.28, profile=P).items
    assert len(items) == 1
    # 램프 마지막 조각 속도 √(2·감속·Δ_last) (Fable 원인 2 — 기록 10/10 일치) — F1 상한이 이 값이 된다
    assert items[0][2] == pytest.approx(max(math.floor(math.sqrt(2 * P.decel_mps2 * dlast) * 100) / 100, 1.0), abs=0.05)


@pytest.mark.parametrize("dlast", [0.3, 0.67, 1.06, 1.51, 2.42, 4.5])
@pytest.mark.parametrize("v0", [5.0, 9.28, 13.9])
def test_ramp_items_start_high_and_end_not_crawl(dlast, v0):
    L = 33.7 if v0 < 12 else 70.0
    step = 4.9
    off = (L - dlast) - step * int((L - dlast) / step)
    r = straight(L, step, off)
    items, info = stop_ramp_items(r, v0, P)
    assert len(items) >= 2 or v0 < 6
    assert items[0][2] >= min(v0, 9.3) - 2.0                   # 첫 항목 = 램프 초반 → F1 상한이 즉시 감속 수준 (저속 순항 아님)
    sp = [it[2] for it in items]
    assert all(a >= b - 1e-9 for a, b in zip(sp[:-1], sp[1:-1]))   # 비최종 항목 속도 단조 비증가 (재가속 없음)
    assert items[-1][2] == pytest.approx(max(items[-2][2] if len(items) >= 2 else v0, 1.0))
    for it in items[:-1]:
        assert L - it[0] >= STOP_ITEM_END_CLEAR_M - 1e-6
    assert info["n_items"] == len(items)


def test_flag_default_off():
    import ugv.vehicle as V
    assert V.STOP_ITEMS_RAMP is False
