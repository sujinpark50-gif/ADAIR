# -*- coding: utf-8 -*-
"""현장 환경 측정·확산 예측 (사용자 결정 2026-09-30, 2019 재현 필수 기능)."""

from orchestrator import config, priority

from .conftest import submit
from .test_priority import DLAT, DLON, LAT0, LON0, _fire_world, _submit_cell, cell, square_site

FIRE = {"cell_id": "C1", "lat": LAT0, "lon": LON0, "cell_size_m": 90.0, "ground_amsl_m": 600.0,
        "fire_state": "BURNING", "risk_score": 1.0}


def _setup(world, wind_dir=270.0):
    env = world["env"]
    env.fire_cells = [dict(FIRE)]
    env.wind_dir_deg = wind_dir                          # 서풍(270) → 불은 동쪽(90°)으로
    env.weather = {"temperature_c": 18.0, "humidity_pct": 22.0}
    env.risk_cells = [cell("EAST", 0, 180, 0.6), cell("NORTH", 180, 0, 0.8), cell("WEST", 0, -180, 0.9)]


def _run(orch, n=4):
    for _ in range(n):
        orch.poll()


def test_fire_detection_creates_sense_tasks_at_fire_and_downwind_cell(world):
    orch, lg = world["orch"], world["ledger"]
    _setup(world)
    t = submit(orch)                                      # 불타는 C1 정찰
    orch.dispatch(t.task_id)
    _run(orch)
    sense = [x for x in lg.list_tasks() if x.kind == "ENV_SENSE"]
    assert sorted(x.target.cell_id for x in sense) == ["C1", "EAST"]   # 위험도가 더 높은 WEST 가 아니라 바람 방향
    assert all(x.requirements.sensor == "WEATHER" for x in sense)
    ev = [e for e in lg.events() if e["event_type"] == "ENV_SENSE_PLANNED"][0]["detail"]
    assert ev["selection"]["downwind_bearing_deg"] == 90.0 and ev["selection"]["angle_off_deg"] < 1.0
    _run(orch)                                            # 같은 관측을 다시 처리해도 중복 생성 없음
    assert len([x for x in lg.list_tasks() if x.kind == "ENV_SENSE"]) == 2


def test_without_wind_direction_only_fire_cell_is_sensed(world):
    orch, lg = world["orch"], world["ledger"]
    _setup(world, wind_dir=None)
    orch.dispatch(submit(orch).task_id)
    _run(orch)
    assert [x.target.cell_id for x in lg.list_tasks() if x.kind == "ENV_SENSE"] == ["C1"]
    ev = [e for e in lg.events() if e["event_type"] == "ENV_SENSE_PLANNED"][0]["detail"]
    assert ev["selection"] == "WIND_DIRECTION_MISSING"


def test_weather_measurement_is_simulated_applied_and_completes(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    _setup(world)
    t = submit(orch, "SENSE-1", kind="ENV_SENSE", target={"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0,
                                                          "cell_id": "C1"},
               requirements={"resource_types": ["UAV"], "sensor": "WEATHER"})
    out = orch.dispatch(t.task_id)
    assert out["status"] == "STARTED"
    assert "observation_type" not in uav.exec_calls()[0][3]  # Local 에는 기상 센서가 없어 이동 가능성만 평가
    _run(orch)
    assert lg.get_task(t.task_id).purpose_status == "COMPLETED"
    obs = [e for e in lg.events(t.task_id) if e["event_type"] == "OBSERVATION"][0]["detail"]
    assert obs["source"] == "SIMULATED" and obs["sensor_status"] == "TEST_ONLY"
    assert obs["values"] == {"wind_ms": 3.0, "wind_dir_deg": 270.0, "temperature_c": 18.0, "humidity_pct": 22.0}
    assert obs["scope"] == "GLOBAL_FIELD"
    assert [e for e in lg.events(t.task_id) if e["event_type"] == "ENVIRONMENT_APPLY"][0]["result"] == "ACK"


def test_cell_weather_used_when_env_provides_it_and_missing_items_not_invented(world):
    orch, lg = world["orch"], world["ledger"]
    _setup(world)
    world["env"].fire_cells[0]["weather"] = {"wind_ms": 11.5}      # 현장만 강풍, 온도·습도는 제공 안 함
    t = submit(orch, "SENSE-2", kind="ENV_SENSE", target={"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0,
                                                          "cell_id": "C1"},
               requirements={"resource_types": ["UAV"], "sensor": "WEATHER"})
    orch.dispatch(t.task_id)
    _run(orch)
    obs = [e for e in lg.events(t.task_id) if e["event_type"] == "OBSERVATION"][0]["detail"]
    assert obs["scope"] == "CELL" and obs["values"] == {"wind_ms": 11.5}


def test_ugv_can_take_weather_task_but_not_thermal(world):
    orch = world["orch"]
    _setup(world)
    t = submit(orch, "SENSE-3", kind="ENV_SENSE", target={"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0,
                                                          "cell_id": "C1"},
               requirements={"resource_types": ["UGV"], "sensor": "WEATHER"})
    assert orch.dispatch(t.task_id)["resource_id"] == "A-ugv1"


# ---------------------------------------------------------------------------
# 확산 예측
# ---------------------------------------------------------------------------
FORECAST = [dict(cell("F-HOME", 400, 0, 0.5), building_type=1, expected_arrival_s=1800.0),
            dict(cell("F-VIL", 0, 400, 0.4), expected_arrival_s=900.0),
            dict(cell("F-FOREST", -400, 0, 0.7), expected_arrival_s=600.0)]
VILLAGE = dict(square_site("VIL", 0, 400, 30), human_occupied=True)


def _forecast(world, enabled=False):
    world["env"].spread_forecast = [dict(f) for f in FORECAST]
    world["env"].forecast_ref = {"model_version": "FIXTURE-CA-1", "issued_sim_s": 0.0, "horizon_s": 3600.0}
    world["env"].protected_sites = [VILLAGE]
    config.PREEMPTIVE_MONITOR_ENABLED = enabled


def test_forecast_threats_are_residential_or_occupied_site_only(world):
    _forecast(world)
    ctx = priority.risk_context(world["env"].read())
    th = {(t["cell_id"], t["basis"]) for t in ctx["forecast_threats"]}
    assert th == {("F-HOME", "FORECAST_REACHES_RESIDENTIAL"), ("F-VIL", "FORECAST_REACHES_OCCUPIED_SITE")}


def test_forecast_threat_makes_task_human_risk(world):
    orch = world["orch"]
    _forecast(world)
    world["env"].protected_sites = []                   # 마을 겹침 효과를 빼고 예측만 본다
    world["env"].spread_forecast = [dict(FORECAST[0], building_type=None, human_exposure=None)]
    world["env"].spread_forecast[0].pop("building_type")
    world["env"].spread_forecast[0].pop("human_exposure")
    _fire_world(world, [cell("HIGH", 0, 0, 0.9)])
    t_high = _submit_cell(orch, "R-H", "HIGH")
    t_fc = _submit_cell(orch, "R-F", "F-HOME")
    plan = priority.order_tasks(world["ledger"].list_tasks(), world["env"].read(), 0.1)
    assert plan["evidence"][t_fc.task_id]["human_risk"] is None   # 주거 표시 없는 예측 칸은 인명 위험 아님
    world["env"].spread_forecast = [dict(FORECAST[0])]              # 예측 칸이 주거지
    plan = priority.order_tasks(world["ledger"].list_tasks(), world["env"].read(), 0.1)
    ev = plan["evidence"][t_fc.task_id]
    assert ev["human_risk"] is True and "FORECAST" in ev["human_risk_basis"]
    assert ev["forecast_arrival_s"] == 1800.0
    assert plan["auto_order"][0] == t_fc.task_id and plan["auto_order"][1] == t_high.task_id


def test_preemptive_monitor_disabled_logs_candidates_but_creates_nothing(world):
    orch, lg = world["orch"], world["ledger"]
    _forecast(world, enabled=False)
    try:
        orch.dispatch_pending()
        orch.dispatch_pending()
        assert [t for t in lg.list_tasks() if t.kind == "MONITOR"] == []
        sup = [e for e in lg.events() if e["event_type"] == "PREEMPTIVE_MONITOR_SUPPRESSED"]
        assert len(sup) == 1                                   # 같은 예측이면 한 번만 기록
        keys = [c["key"] for c in sup[0]["detail"]["candidates"]]
        assert keys == ["VIL", "F-HOME"]                       # 보호대상별 묶음, 먼저 닿는 곳부터
    finally:
        config.PREEMPTIVE_MONITOR_ENABLED = False


def test_preemptive_monitor_enabled_creates_one_task_per_site(world):
    orch, lg = world["orch"], world["ledger"]
    _forecast(world, enabled=True)
    try:
        orch.dispatch_pending()
        mons = [t for t in lg.list_tasks() if t.kind == "MONITOR"]
        assert sorted(t.target.cell_id for t in mons) == ["F-HOME", "F-VIL"]
        orch.dispatch_pending()
        assert len([t for t in lg.list_tasks() if t.kind == "MONITOR"]) == 2
    finally:
        config.PREEMPTIVE_MONITOR_ENABLED = False


def test_board_shows_forecast_and_weather(world):
    from fastapi.testclient import TestClient
    from orchestrator.api import create_app
    orch = world["orch"]
    _setup(world)
    _forecast(world)
    c = TestClient(create_app(orch))
    t = submit(orch, "SENSE-B", kind="ENV_SENSE", target={"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0,
                                                          "cell_id": "C1"},
               requirements={"resource_types": ["UAV"], "sensor": "WEATHER"})
    orch.dispatch(t.task_id)
    _run(orch)
    b = c.get("/priority/board").json()
    f = b["forecast"]
    assert f["cell_count"] == 3 and f["preemptive_monitor_enabled"] is False
    assert [x["key"] for x in f["threats"]] == ["VIL", "F-HOME"] and f["threats"][0]["monitor_task_id"] is None
    w = [r for r in b["results"] if r["task_id"] == t.task_id][0]["observation"]
    assert w["values"]["temperature_c"] == 18.0 and w["scope"] == "GLOBAL_FIELD"
    assert "확산 예측" in c.get("/board").text


def test_residential_threat_inside_site_is_grouped_with_site(world):
    orch = world["orch"]
    _forecast(world)
    # 마을(VIL) 안에 있는 주거 칸이 예측에 들어옴 → 마을 묶음 하나로
    world["env"].spread_forecast.append(dict(cell("F-IN-VIL", 0, 400, 0.4), building_type=1,
                                             expected_arrival_s=1200.0))
    cands = orch.preemptive_candidates(world["env"].read())
    vil = [c for c in cands if c["key"] == "VIL"][0]
    assert set(vil["cells"]) == {"F-VIL", "F-IN-VIL"} and "F-IN-VIL" not in [c["key"] for c in cands]
    assert vil["basis"] == ["FORECAST_REACHES_OCCUPIED_SITE", "FORECAST_REACHES_RESIDENTIAL"]
