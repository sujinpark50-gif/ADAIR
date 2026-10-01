# -*- coding: utf-8 -*-
"""기상 항목별 최신성·출처·재측정 (사용자 결정 2026-09-30 D02).

시뮬레이션 관측시각으로 결정적으로 시험한다. 여기서 쓰는 현장 측정 유효시간(600초)은 시험 전용 값이다 —
운영 기본값은 미설정이며, 미설정이면 현장 측정을 유효하다고 보지 않는다 (POLICY_NOT_SET).
관측소 유효시간은 자료 제공 간격이다 (기상청 시간자료 3600초).
"""

import json

import pytest
from fastapi.testclient import TestClient

from orchestrator.api import create_app
from orchestrator.knowledge import Knowledge
from orchestrator.ledger import Ledger

from .conftest import TARGET, submit

LAT, LON = 38.05, 128.25
FULL = {"wind_ms": 5.0, "wind_dir_deg": 160.0, "temperature_c": 15.0, "humidity_pct": 27.0}


@pytest.fixture
def kb(tmp_path):
    lg = Ledger(str(tmp_path / "w.db"))
    lg.activate_run("RUN-W")
    return Knowledge(lg)


def station(kb, t, values, sid="S1", interval=3600.0, lat=LAT, lon=LON + 0.01, received=None):
    kb.add_weather(f"ST:{sid}:{t}", t, "KMA_ASOS_HOURLY",
                   {"kind": "STATION", "station_id": sid, "station_name": sid, "lat": lat, "lon": lon,
                    "values": values, "provision_interval_s": interval},
                   received_sim_s=t if received is None else received)


def field(kb, t, values, cell="C1", key=None):
    kb.add_weather(key or f"FIELD:{cell}:{t}", t, "SIMULATED",
                   {"kind": "FIELD", "cell_id": cell, "lat": LAT, "lon": LON, "values": values,
                    "observation_id": f"OBS-{cell}-{t}"}, received_sim_s=t)


def test_expired_field_wind_is_replaced_by_newer_station_value(kb, field_policy):
    """재현: 0초 현장 풍속 3 m/s, 3600초 관측소 풍속 12 m/s → 7200초에 옛 현장값을 계속 고르면 안 된다."""
    field_policy(600.0)
    field(kb, 0.0, {"wind_ms": 3.0})
    station(kb, 3600.0, {"wind_ms": 12.0})
    w = kb.weather_for(LAT, LON, "C1", 7200.0)
    wind = w["items"]["wind_ms"]
    assert w["values"]["wind_ms"] == 12.0 and wind["basis"] == "NEAREST_STATION" and wind["status"] == "VALID"
    assert wind["observed_sim_s"] == 3600.0 and wind["age_s"] == 3600.0 and wind["unit"] == "m/s"
    assert wind["replaced"]["status"] == "EXPIRED" and wind["replaced"]["value"] == 3.0
    assert wind["replaced"]["age_s"] == 7200.0 and wind["replaced"]["max_age_s"] == 600.0
    assert [r["item"] for r in w["remeasure"]] == ["wind_ms"] and w["remeasure"][0]["cell_id"] == "C1"
    # 유효한 동안에는 현장 측정이 우선
    fresh = kb.weather_for(LAT, LON, "C1", 600.0)
    assert fresh["values"] == {"wind_ms": 3.0} and fresh["items"]["wind_ms"]["basis"] == "FIELD_AT_TARGET_CELL"
    assert fresh["remeasure"] == []


def test_partial_field_measurement_does_not_erase_other_station_items(kb, field_policy):
    field_policy(600.0)
    station(kb, 0.0, FULL)
    field(kb, 100.0, {"wind_ms": 11.5})                           # 풍속만 잰 현장 측정
    w = kb.weather_for(LAT, LON, "C1", 200.0)
    assert w["values"] == {**FULL, "wind_ms": 11.5}
    assert w["items"]["wind_ms"]["basis"] == "FIELD_AT_TARGET_CELL"
    assert {w["items"][k]["basis"] for k in ("wind_dir_deg", "temperature_c", "humidity_pct")} == {"NEAREST_STATION"}
    assert w["unknown"] == {} and w["remeasure"] == []


def test_unset_field_policy_is_exposed_and_the_field_value_is_not_treated_as_valid(kb):
    station(kb, 0.0, FULL)
    field(kb, 0.0, {"wind_ms": 14.0})                             # 방금 잰 값이어도 유효시간 정책이 없다
    w = kb.weather_for(LAT, LON, "C1", 0.0)
    assert w["values"]["wind_ms"] == 5.0
    assert w["items"]["wind_ms"]["replaced"]["status"] == "POLICY_NOT_SET"
    assert w["remeasure"] == []                                    # 다시 재도 쓸 수 없으므로 재측정을 요청하지 않는다
    excluded = kb.field_weather(0.0)["excluded"]
    assert excluded[0]["status"] == "POLICY_NOT_SET" and "value" not in excluded[0]


def test_per_item_policy_only_validates_configured_items(kb, field_policy):
    field_policy(wind_ms=600.0)                                    # 풍향 정책은 미설정
    station(kb, 0.0, FULL)
    field(kb, 0.0, {"wind_ms": 9.0, "wind_dir_deg": 270.0})
    w = kb.weather_for(LAT, LON, "C1", 60.0)
    assert w["values"]["wind_ms"] == 9.0 and w["values"]["wind_dir_deg"] == 160.0
    assert w["items"]["wind_dir_deg"]["replaced"]["status"] == "POLICY_NOT_SET"


def test_no_data_and_all_expired_are_unknown_with_reasons(kb, field_policy):
    field_policy(600.0)
    none = kb.weather_for(LAT, LON, "C1", 0.0)
    assert none["values"] == {} and none["basis"] == "NO_WEATHER_KNOWLEDGE"
    assert none["unknown"]["wind_ms"] == {"reason": "NO_VALID_SOURCE", "field": None, "station": None}
    field(kb, 0.0, {"wind_ms": 3.0})
    station(kb, 0.0, {"wind_ms": 5.0})
    w = kb.weather_for(LAT, LON, "C1", 3601.0)                     # 현장(600초)·관측소(3600초) 모두 만료
    assert w["values"] == {} and w["basis"] == "WIND_UNKNOWN"
    assert w["unknown"]["wind_ms"] == {"reason": "NO_VALID_SOURCE", "field": "EXPIRED", "station": "EXPIRED"}
    assert [r["item"] for r in w["remeasure"]] == ["wind_ms"]
    assert kb.weather_for(LAT, LON, "C1", 3600.0)["values"] == {"wind_ms": 5.0}   # 제공 간격 안에서는 유효


def test_future_observations_are_never_used(kb):
    station(kb, 0.0, {"wind_ms": 5.0})
    station(kb, 3600.0, {"wind_ms": 12.0})
    assert kb.weather_for(LAT, LON, "C1", 3599.0)["values"]["wind_ms"] == 5.0
    assert [s["values"] for s in kb.latest_station_obs(3599.0)] == [{"wind_ms": 5.0}]
    assert kb.weather_for(LAT, LON, "C1", 3600.0)["values"]["wind_ms"] == 12.0


def test_out_of_order_arrival_uses_observed_time_not_receive_order(kb):
    station(kb, 3600.0, {"wind_ms": 12.0}, received=3600.0)
    station(kb, 0.0, {"wind_ms": 5.0}, received=3700.0)            # 옛 관측이 나중에 도착
    w = kb.weather_for(LAT, LON, "C1", 3700.0)
    assert w["values"]["wind_ms"] == 12.0
    assert w["items"]["wind_ms"]["observed_sim_s"] == 3600.0 and w["items"]["wind_ms"]["received_sim_s"] == 3600.0
    assert kb.latest_station_obs(3700.0)[0]["values"] == {"wind_ms": 12.0}


def test_observation_without_time_or_with_invalid_value_is_not_used(kb, field_policy):
    field_policy(600.0)
    kb.add_weather("ST:S1:untimed", None, "KMA_ASOS_HOURLY",
                   {"kind": "STATION", "station_id": "S1", "lat": LAT, "lon": LON, "values": {"wind_ms": 7.0},
                    "provision_interval_s": 3600.0})
    w = kb.weather_for(LAT, LON, "C1", 100.0)
    assert w["values"] == {} and w["unknown"]["wind_ms"]["station"] == "OBSERVED_TIME_UNKNOWN"
    field(kb, 50.0, {"wind_ms": -2.0, "wind_dir_deg": 400.0, "humidity_pct": float("nan")})
    w = kb.weather_for(LAT, LON, "C1", 100.0)
    assert w["values"] == {} and {w["unknown"][k]["field"] for k in ("wind_ms", "wind_dir_deg", "humidity_pct")} == {
        "VALUE_INVALID"}


def test_station_without_provision_interval_is_policy_not_set(kb):
    station(kb, 0.0, {"wind_ms": 5.0}, interval=None)
    w = kb.weather_for(LAT, LON, "C1", 10.0)
    assert w["values"] == {} and w["unknown"]["wind_ms"]["station"] == "POLICY_NOT_SET"


# ---------------------------------------------------------------------------
# 엔진: 출동에 쓰는 풍속, 재측정 요청, 분석 입력·화면 일치
# ---------------------------------------------------------------------------
def _add_field(orch, t, values, cell="C1"):
    orch.view()                                                    # run 활성화
    orch.kb.add_weather(f"FIELD:seed:{cell}:{t}", t, "SIMULATED",
                        {"kind": "FIELD", "cell_id": cell, "lat": TARGET["lat"], "lon": TARGET["lon"],
                         "values": values, "observation_id": f"OBS-seed-{t}"}, received_sim_s=t)


def _remeasure_events(lg):
    return [e for e in lg.events() if e["event_type"] == "WEATHER_REMEASURE_REQUESTED"]


def test_dispatch_uses_station_wind_when_field_expired_and_requests_remeasure_once(world, field_policy):
    orch, env, uav, lg = world["orch"], world["env"], world["uav"], world["ledger"]
    field_policy(600.0)
    _add_field(orch, 0.0, {"wind_ms": 9.9})
    env.simulation_time_s = 1200.0                                  # 현장 측정 만료. 시험 관측소는 지금 3.0 m/s
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    assert out["status"] == "STARTED" and uav.exec_calls()[0][3]["wind_ms"] == 3.0
    ref = [e for e in lg.events(task.task_id) if e["event_type"] == "CANDIDATES_FILTERED"][-1]["detail"]["wind_ref"]
    assert ref["basis"] == "NEAREST_STATION" and ref["items"]["wind_ms"]["replaced"]["status"] == "EXPIRED"
    # 재측정: 같은 칸에 가는 이 정찰에 기상 측정을 붙인다 (그 칸에 따로 또 보내지 않는다). 모의 측정임을 기록
    ev = _remeasure_events(lg)
    assert len(ev) == 1 and ev[0]["detail"]["measurement_contract"] == "SIMULATED_WEATHER_SENSOR (TEST_ONLY)"
    assert "WEATHER" in lg.get_task(task.task_id).requirements.extra_sensors
    assert [t for t in lg.list_tasks() if t.kind == "ENV_SENSE"] == []
    second = submit(orch, "REQ-2")
    orch.dispatch(second.task_id)
    assert len(_remeasure_events(lg)) == 1                          # 같은 만료 측정에 대한 요청은 한 번


def test_expired_field_creates_one_sense_task_when_no_open_recon_can_carry_it(world, field_policy):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    field_policy(600.0)
    _add_field(orch, 0.0, {"wind_ms": 9.9})
    env.simulation_time_s = 1200.0
    road = {"resource_types": ["UGV"], "sensor": "ROAD_STATUS"}     # 기상을 함께 잴 수 없는 지상 임무
    orch.dispatch(submit(orch, "GROUND-1", requirements=road).task_id)
    sense = [t for t in lg.list_tasks() if t.kind == "ENV_SENSE"]
    assert len(sense) == 1 and sense[0].target.cell_id == "C1" and sense[0].requirements.sensor == "WEATHER"
    assert sense[0].request_key.startswith("AUTO-REMEASURE:RUN-FIXTURE:C1:")
    assert _remeasure_events(lg)[0]["result"] == "NEW_SENSE_TASK"
    orch.dispatch(submit(orch, "GROUND-2", requirements=road).task_id)
    assert len([t for t in lg.list_tasks() if t.kind == "ENV_SENSE"]) == 1 and len(_remeasure_events(lg)) == 1


def test_analysis_input_and_board_show_the_same_selected_weather(world, field_policy):
    orch, env = world["orch"], world["env"]
    field_policy(600.0)
    _add_field(orch, 0.0, {"wind_ms": 9.9})
    c = TestClient(create_app(orch))

    def both():
        known = c.get("/priority/board").json()["known"]
        belief = json.loads(json.dumps(world["analysis"].last_input))
        return known, belief
    known, belief = both()
    assert belief["field_weather"] == known["field_weather"] and belief["station_weather"] == known["stations"]
    assert belief["field_weather"][0]["values"] == {"wind_ms": 9.9}
    assert belief["field_weather"][0]["items"]["wind_ms"]["status"] == "VALID"
    assert belief["weather_policy"] == known["weather_policy"]

    env.simulation_time_s = 1200.0                                  # 만료 뒤에는 분석에도 넘기지 않는다
    known, belief = both()
    assert belief["field_weather"] == [] == known["field_weather"]
    assert belief["field_weather_excluded"] == known["field_weather_excluded"]
    ex = belief["field_weather_excluded"][0]
    assert (ex["cell_id"], ex["item"], ex["status"]) == ("C1", "wind_ms", "EXPIRED") and "value" not in ex
    assert orch.kb.field_history(1200.0)[0]["values"] == {"wind_ms": 9.9}   # 원본 이력은 따로 보관
    assert "field_history" not in belief


def test_health_reports_unset_field_policy(world):
    h = TestClient(create_app(world["orch"])).get("/health").json()
    assert h["weather_policy"]["field_policy_unset_items"] == ["humidity_pct", "temperature_c", "wind_dir_deg",
                                                               "wind_ms"]
    assert h["run"]["active"] == "RUN-FIXTURE" and h["run"]["policy"] == "SINGLE_ACTIVE_RUN"
