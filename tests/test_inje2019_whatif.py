# -*- coding: utf-8 -*-
"""2019 인제 '드론이 있었다면' 시나리오 불변식 — 정답지 방지·풍향 규약.

  python tools/inje2019_whatif.py   # scenario.json 생성 (약 1분)
  python -m pytest tests/test_inje2019_whatif.py -q
"""
import base64
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SCN = ROOT / "web" / "static" / "inje2019" / "scenario.json"
sys.path.insert(0, str(ROOT / "environment"))


def arr(d, k, t=np.int16):
    return np.frombuffer(base64.b64decode(d[k]), dtype=t)


@pytest.fixture(scope="module")
def D():
    if not SCN.is_file():
        pytest.skip("scenario.json 없음 — python tools/inje2019_whatif.py 먼저")
    return json.loads(SCN.read_text(encoding="utf-8"))


def test_wind_convention_fire_spreads_downwind():
    """기상청 풍향 = 불어오는 방향. 서풍(270°)이면 동쪽 이웃으로 더 잘 번져야 한다 (기술문서 3절 '풍향 규약')."""
    pytest.importorskip("rasterio")
    from src.cell import Cell
    from src.fire_model import WildfireCAEngine
    eng = WildfireCAEngine.__new__(WildfireCAEngine)
    eng._config = {"wind": {"speed_ms": 8.0, "direction_deg": 270.0}}
    eng._base_spread_prob, eng._slope_weight, eng._wind_weight, eng._moisture_weight = 0.4, 0.05, 0.1, 1.0

    class G:
        cell_resolution = 90.0
    eng._grid = G()
    mk = lambda x, y: Cell(cell_id=0, x=x, y=y, elevation=100.0, slope=0.0, fuel_type=2, fuel_amount=0.6, moisture=0.2)
    src = mk(5, 5)
    east, west = eng._calculate_spread_probability(src, mk(6, 5)), eng._calculate_spread_probability(src, mk(4, 5))
    assert east > west


def test_knowledge_never_exceeds_truth(D):
    """총괄이 '안다'고 기록한 칸은 그 시각까지 실제로 탄 칸이어야 한다 (지어낸 지식 없음)."""
    ign = arr(D, "ign_step_i16")
    for key in ("knownA_step_i16", "knownB_step_i16", "knownBb_step_i16"):
        k = arr(D, key)
        idx = np.nonzero(k >= 0)[0]
        assert np.all(ign[idx] >= 0) and np.all(ign[idx] <= k[idx]), key


def test_A_learns_nothing_new_at_night(D):
    night = {r["k"] for r in D["timeline"] if not r["daylight"]}
    kA = arr(D, "knownA_step_i16")
    assert night and not (set(kA[kA >= 0].tolist()) & night)


def test_drone_knows_at_least_as_much_as_A(D):
    kA, kB = arr(D, "knownA_step_i16"), arr(D, "knownB_step_i16")
    for r in D["timeline"]:
        k = r["k"]
        a = set(np.nonzero((kA >= 0) & (kA <= k))[0]); b = set(np.nonzero((kB >= 0) & (kB <= k))[0])
        assert a <= b


def test_forecast_excludes_known_cells(D):
    kA = arr(D, "knownA_step_i16")
    for r in D["timeline"]:
        known = set(np.nonzero((kA >= 0) & (kA <= r["k"]))[0].tolist())
        assert not (set(r["A"]["forecast"]) & known)


def test_calibration_anchor_and_summary(D):
    s = D["summary"]
    assert 5.0 <= s["truth_ha_19h"] <= 20.0                  # 19:00 추정 10 ha 근처
    assert s["night_unknown_burning_ha_mean"]["B"] <= s["night_unknown_burning_ha_mean"]["A"]
    assert s["drone_grounded_night_steps"] == 0               # 그날 밤 인제 관측 풍속 < 10 m/s
