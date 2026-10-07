# -*- coding: utf-8 -*-
"""환경 계약 수락시험 — 총괄 요청서 ENV-01~06 의 '수락 시험' 칸과 공동 수락시험 J1·J2 를 그대로 옮겼다.

실행 (저장소 루트):  python -m pytest environment/tests -q
실제 래스터(rasterio)로 김은주님 CA 를 띄운다 (환경 하나 만드는 데 약 4초).
"""

import copy
import shutil
import sys
from pathlib import Path

import pytest

ENV_DIR = Path(__file__).resolve().parents[1]
if str(ENV_DIR) not in sys.path:
    sys.path.insert(0, str(ENV_DIR))

pytest.importorskip("rasterio")

from src.belief_analysis import BeliefSpreadAnalysis   # noqa: E402
from src.contract_env import ContractEnvironment       # noqa: E402

IGN = "19_142"


def make(tmp, **kw):
    return ContractEnvironment(tmp, **kw)


@pytest.fixture(scope="module")
def base_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("env_base")
    make(d)                                  # 0초 상태를 한 번 만들어 둔다 (시험마다 복사해 시간 절약)
    return d


@pytest.fixture
def env(base_dir, tmp_path):
    d = tmp_path / "env"
    shutil.copytree(base_dir, d)
    return make(d)


def fire_set(snap):
    return {(c["cell_id"], c["fire_state"]) for c in snap["fire_cells"]}


def thermal(env, oid, **over):
    snap = env.snapshot()
    o = {"observation_id": oid, "run_id": env.run_id, "simulation_time_s": snap["simulation_time_s"],
         "sensor_type": "THERMAL", "source": "SIMULATED", "covered_cells": [IGN], "partial_cells": ["20_142"],
         "cell_coverage": [{"cell_id": IGN, "fraction": 1.0}, {"cell_id": "20_142", "fraction": 0.27}],
         "detections": [{"cell_id": IGN, "fire_state": "BURNING"}]}
    o.update(over)
    return o


# ---------------------------------------------------------------- ENV-01
def test_read_is_pure(env):
    a = env.snapshot()
    for _ in range(5):
        b = env.snapshot()
    assert b["state_version"] == a["state_version"] and b["simulation_time_s"] == a["simulation_time_s"]
    assert fire_set(a) == fire_set(b) and env.step_count == 0


def test_snapshot_has_contract_fields(env):
    s = env.snapshot()
    for k in ("run_id", "state_version", "simulation_time_s", "map_version", "scenario_start_kst", "crs",
              "vertical_datum", "wind_ms", "wind_dir_deg", "fire_cells", "risk_cells", "protected_sites"):
        assert s.get(k) is not None, k
    assert s["contract_complete"] is True and s["source"] == "TEAM_ENV"
    c = s["fire_cells"][0]
    assert {"cell_id", "lat", "lon", "cell_size_m", "ground_amsl_m", "fire_state"} <= set(c)


def test_advance_moves_clock_and_version(env):
    v0 = env.state_version
    s = env.advance(3)
    assert s["state_version"] == v0 + 1 and s["simulation_time_s"] == 180.0 and env.step_count == 3


def test_restart_resumes_same_run_and_rng(base_dir, tmp_path):
    """복구하는 재시작 → 같은 run_id, 시각·버전 유지, 이어서 진행한 결과가 재시작 안 한 경우와 같다."""
    d1, d2 = tmp_path / "a", tmp_path / "b"
    shutil.copytree(base_dir, d1); shutil.copytree(base_dir, d2)
    a = make(d1); a.advance(4)
    before = (a.run_id, a.state_version, a.simulation_time_s)
    a2 = make(d1)                                      # 재시작
    assert a2.restored and (a2.run_id, a2.state_version, a2.simulation_time_s) == before
    a2.advance(3)
    b = make(d2); b.advance(4); b.advance(3)           # 재시작 없이 같은 진행
    assert fire_set(a2.snapshot()) == fire_set(b.snapshot())


def test_reset_gives_new_run_id(env):
    old = env.run_id
    env.advance(2)
    s = env.reset()
    assert s["run_id"] != old and s["simulation_time_s"] == 0.0 and s["state_version"] == 1


# ---------------------------------------------------------------- ENV-02 / INT-04
def test_map_cells_static_only(env):
    cells = env.map_cells()
    assert len(cells) == env.grid.rows * env.grid.cols
    keys = set().union(*(c.keys() for c in cells[:50]))
    assert keys == {"cell_id", "lat", "lon", "cell_size_m", "ground_amsl_m", "building_type", "human_exposure",
                    "is_road", "fuel_amount"}           # 도로·연료: 총괄 관측 계획 지형 근거 (2026-10-07)
    res = [c for c in cells if c["building_type"] == 1]
    assert res and all(c["human_exposure"] for c in res)
    assert len(env.protected_sites()) == len(res)


def test_map_fuel_is_static_not_truth(env):
    """지도의 연료는 시작 시점 값이다 — 불이 태워도 바뀌지 않는다 (정답 누출 방지)"""
    before = {c["cell_id"]: c["fuel_amount"] for c in env.map_cells()}
    env.advance(3)
    burning = {c["cell_id"] for c in env.snapshot()["fire_cells"]}
    after = {c["cell_id"]: c["fuel_amount"] for c in env.map_cells()}
    assert burning and all(after[cid] == before[cid] for cid in burning)


def test_stations_single_source(env):
    st = env.stations()
    assert {s["base_id"] for s in st} == {"A", "B"} and all("lat" in s and "lon" in s for s in st)


# ---------------------------------------------------------------- ENV-03
def test_truth_wind_follows_kma_inje(env):
    s = env.snapshot()          # 14:45 → 14:00 행: 5.7 m/s, 160°
    assert (s["wind_ms"], s["wind_dir_deg"]) == (5.7, 160.0)
    assert s["weather"]["basis"] == "KMA_ASOS_HOURLY" and s["weather"]["scope"] == "GLOBAL_FIELD"
    s = env.advance(16)         # 960초 → 15:01 → 15:00 행: 6.3 m/s
    assert s["wind_ms"] == 6.3 and s["weather"]["observed_kst"] == "2019-04-04 15:00"
    assert env.engine._config["wind"]["speed_ms"] == 6.3       # CA 가 실제로 그 바람으로 돈다


# ---------------------------------------------------------------- ENV-05 (J1, J2)
def test_apply_ack_and_duplicate(env):
    o = thermal(env, "OBS-1")
    a1 = env.apply(o)
    assert a1["accepted"] and a1["observation_id"] == "OBS-1" and a1["duplicate"] is False
    a2 = env.apply(copy.deepcopy(o))           # J1: ACK 유실 후 같은 관측 재전송
    assert a2["duplicate"] is True and a2["state_version"] == a1["state_version"]
    assert env.state_version == a1["state_version"] and env.applied_count() == 1
    assert a1["applied_cells"] == {"full": 1, "partial": 1}


def test_apply_survives_restart_and_detects_conflict(env):
    o = thermal(env, "OBS-R")
    a1 = env.apply(o)
    env2 = make(env.state_dir)                 # J2: 상태 복구 재시작
    assert env2.run_id == env.run_id and env2.state_version >= a1["state_version"]
    again = env2.apply(copy.deepcopy(o))
    assert again["duplicate"] is True and env2.applied_count() == 1
    bad = env2.apply(thermal(env2, "OBS-R", detections=[]))     # 같은 ID·다른 내용
    assert bad["accepted"] is False and bad["reason"] == "OBSERVATION_ID_CONFLICT"
    assert env2.applied_count() == 1


def test_apply_never_mutates_truth_fire(env):
    before = fire_set(env.snapshot())
    env.apply(thermal(env, "OBS-X", detections=[{"cell_id": "100_100", "fire_state": "BURNING"}],
                      covered_cells=["100_100"]))
    assert fire_set(env.snapshot()) == before            # 관측만으로 UNBURNED→BURNING 금지


def test_apply_rejects_other_run_and_future(env):
    assert env.apply(thermal(env, "OBS-O", run_id="RUN-OTHER"))["reason"] == "RUN_MISMATCH"
    r = env.apply(thermal(env, "OBS-F", simulation_time_s=9999.0))
    assert r["accepted"] is False and r["reason"] == "OBSERVATION_FROM_FUTURE"
    assert env.apply(thermal(env, "OBS-F", simulation_time_s=9999.0))["duplicate"] is True   # NACK 도 멱등


# ---------------------------------------------------------------- ENV-06
def test_reports_until_with_stable_event_id(env):
    r1 = env.reports_until(0.0)
    assert len(r1) == 1 and r1[0]["cell_id"] == IGN and r1[0]["run_id"] == env.run_id
    assert env.reports_until(0.0)[0]["event_id"] == r1[0]["event_id"]    # 재전송 = 같은 event_id
    custom = make(env.state_dir, reports=[{"cell_id": "30_150", "sim_time_s": 600.0, "source": "119_CALL"}])
    assert custom.reports_until(599.0) == [] and len(custom.reports_until(600.0)) == 1


# ---------------------------------------------------------------- ENV-04
def analysis(env):
    return BeliefSpreadAnalysis(env.static, copy.deepcopy(env.engine._config), tick_s=env.tick_s)


def belief(env, fires=(IGN,), field=None, station=True):
    st = [{"station_id": "211", "lat": 38.05991, "lon": 128.1681, "sim_time_s": 0.0, "source": "KMA_ASOS_HOURLY",
           "values": {"wind_ms": 5.7, "wind_dir_deg": 160.0}}] if station else []
    return {"run_id": env.run_id, "simulation_time_s": 0.0, "known_fires": [{"cell_id": c} for c in fires],
            "fire_states": {c: {"status": "REPORTED"} for c in fires}, "station_weather": st,
            "field_weather": field or [], "field_weather_excluded": []}


def test_analysis_ignores_truth_changes(env):
    """① 진짜 세계의 미래 화재·확산만 바꿔도 결과 불변"""
    an, b = analysis(env), belief(env)
    r1 = an.analyze(b)
    env.advance(10)
    env.engine._config["wind"]["speed_ms"] = 30.0      # 진짜 바람을 바꿔도
    assert an.analyze(b) == r1
    assert r1["risk_cells"] and r1["spread_forecast"] and r1["source"] == "ENV_BELIEF_ANALYSIS"


def test_analysis_needs_known_fire(env):
    """② 신고·정찰 전에는 예측이 없다"""
    r = analysis(env).analyze(belief(env, fires=()))
    assert r["risk_cells"] == [] and r["forecast_ref"] is None


def test_field_measurement_changes_forecast(env):
    """③ 같은 진짜 상태에서 현장 측정을 한 경우와 안 한 경우 예측이 다르다"""
    an = analysis(env)
    no_field = an.analyze(belief(env))
    field = [{"kind": "FIELD", "cell_id": "20_142", "lat": env.map_cells()[env.idx_of("20_142")]["lat"],
              "lon": env.map_cells()[env.idx_of("20_142")]["lon"], "source": "SIMULATED", "sim_time_s": 0.0,
              "values": {"wind_ms": 11.0, "wind_dir_deg": 270.0}}]
    with_field = an.analyze(belief(env, field=field))
    assert with_field["forecast_ref"]["wind_basis"] == "FIELD_MEASUREMENT"
    assert no_field["forecast_ref"]["wind_basis"] == "STATION"
    assert with_field["spread_forecast"] != no_field["spread_forecast"]


def test_downwind_cells_arrive_first(env):
    """서풍(270°)이면 불 동쪽 칸이 서쪽 칸보다 먼저 도달 (같은 연료·경사면 비교가 아니므로 평균으로 본다)"""
    an = analysis(env)
    st = [{"station_id": "X", "lat": 38.0, "lon": 128.1, "sim_time_s": 0.0, "values": {"wind_ms": 10.0,
                                                                                        "wind_dir_deg": 270.0}}]
    b = belief(env, station=False); b["station_weather"] = st
    r = an.analyze(b)
    t = {c["cell_id"]: c["expected_arrival_s"] for c in r["spread_forecast"]}
    east = [v for k, v in t.items() if int(k.split("_")[0]) > 19]
    west = [v for k, v in t.items() if int(k.split("_")[0]) < 19]
    assert east and (not west or sum(east) / len(east) < sum(west) / len(west))
