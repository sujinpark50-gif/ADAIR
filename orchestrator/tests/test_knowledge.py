# -*- coding: utf-8 -*-
"""진짜 세계 / 총괄이 아는 세계 분리 (정답지 방지) + 기상청 관측 재생 (사용자 결정 2026-09-30)."""

from pathlib import Path

from orchestrator.engine import Orchestrator
from orchestrator.env_adapter import FixtureEnv
from orchestrator.knowledge import InjectedAnalysis, KmaAsosReplay
from orchestrator.ledger import Ledger
from orchestrator.resources import UavClient, UgvClient

from .conftest import submit
from .fakes import FakeUav, FakeUgv, normal_flight

ROOT = Path(__file__).resolve().parents[2]
KMA_CSV = ROOT / "data/weather/kma_asos_hourly_20190404_20190405.csv"
KMA_STN = ROOT / "data/weather/kma_asos_stations.json"
# 인제 관측소(38.05991, 128.1681) 근처의 시험 화재 칸
FIRE = {"cell_id": "F1", "lat": 38.05, "lon": 128.20, "cell_size_m": 90.0, "ground_amsl_m": 500.0,
        "fire_state": "BURNING", "risk_score": 1.0, "weather": {"wind_ms": 14.0}}


def _world(tmp_path, **env_kw):
    uav = FakeUav()
    uav.default_script = normal_flight()
    env = FixtureEnv(fire_cells=[dict(FIRE)], wind_ms=3.0, wind_dir_deg=90.0,
                     scenario_start_kst="2019-04-04T14:00:00+09:00", **env_kw)
    lg = Ledger(str(tmp_path / "k.db"))
    orch = Orchestrator(lg, env, UavClient(uav.endpoints(), uav.client()), UgvClient("http://ugv", FakeUgv().client()),
                        analysis=InjectedAnalysis(), weather_feeds=[KmaAsosReplay(KMA_CSV, KMA_STN)])
    return orch, env, uav, lg


def _recon(orch, rid="R1"):
    return submit(orch, rid, target={"lat": FIRE["lat"], "lon": FIRE["lon"], "ground_amsl_m": 500.0, "cell_id": "F1"},
                  requirements={"resource_types": ["UAV"], "sensor": "THERMAL"})


def test_view_hides_true_fire_until_reported_or_observed(tmp_path):
    orch, env, uav, lg = _world(tmp_path, reports=[{"cell_id": "F1", "sim_time_s": 2700.0, "source": "119_CALL"}])
    v = orch.view()
    assert v.fire_cells == [] and v.source == "ORCHESTRATOR_VIEW"      # 진짜로 타고 있어도 아직 모름
    assert v.wind_ms is None and v.weather == {}                       # 진짜 기상도 보지 않음
    env.simulation_time_s = 2700.0                                     # 14:45 신고
    v = orch.view()
    assert [c["cell_id"] for c in v.fire_cells] == ["F1"]
    k = v.fire_cells[0]["knowledge"]
    assert k["status"] == "REPORTED" and k["source"] == "119_CALL"
    assert v.fire_cells[0]["risk_score"] is None                       # 환경 내부 점수는 새지 않음
    assert "weather" not in v.fire_cells[0]


def test_recon_confirms_fire_and_analysis_gets_only_known_facts(tmp_path):
    orch, env, uav, lg = _world(tmp_path)
    orch.dispatch(_recon(orch).task_id)
    for _ in range(4):
        orch.poll()
    v = orch.view()
    assert v.fire_cells[0]["knowledge"]["status"] == "CONFIRMED"
    belief = orch.analysis.last_input
    assert [c["cell_id"] for c in belief["known_fires"]] == ["F1"]
    assert "fire_cells" not in belief and "weather" not in belief


def test_observed_clear_removes_reported_fire(tmp_path):
    orch, env, uav, lg = _world(tmp_path, reports=[{"cell_id": "F1", "sim_time_s": 0.0, "source": "119_CALL"}])
    env.fire_cells[0]["fire_state"] = "UNBURNED"                      # 오인 신고
    orch.dispatch(_recon(orch).task_id)
    for _ in range(4):
        orch.poll()
    assert orch.view().fire_cells == []
    # 90m 칸을 54×45m 프레임으로 봤다 → 본 범위에는 불이 없지만 칸 전체를 본 것은 아니다 (D03)
    f = orch.kb.fire_states(1e9)["F1"]
    assert f["status"] == "NOT_DETECTED_PARTIAL" and abs(f["coverage_fraction"] - 54 * 45 / 8100) < 1e-6


def test_full_cell_view_without_fire_is_observed_clear(tmp_path):
    orch, env, uav, lg = _world(tmp_path, reports=[{"cell_id": "F1", "sim_time_s": 0.0, "source": "119_CALL"}])
    env.fire_cells[0].update(fire_state="UNBURNED", cell_size_m=40.0)   # 프레임(54×45m) 안에 칸 전체가 들어옴
    orch.dispatch(_recon(orch).task_id)
    for _ in range(4):
        orch.poll()
    assert orch.kb.fire_states(1e9)["F1"]["status"] == "OBSERVED_CLEAR"


def test_kma_replay_gives_only_past_observations_from_nearest_station(tmp_path):
    orch, env, uav, lg = _world(tmp_path)
    task = _recon(orch)
    v = orch.view_for(task)                                            # 14:00 시점
    assert v.wind_ref["basis"] == "NEAREST_STATION" and v.wind_ref["station_id"] == "211"
    assert v.wind_ref["observed_kst"] == "2019-04-04 14:00"
    assert v.wind_ms == 5.7 and v.wind_dir_deg == 160.0 and v.weather == {"temperature_c": 15.3, "humidity_pct": 27.0}
    assert 2000 < v.wind_ref["distance_m"] < 4000
    env.simulation_time_s = 3599.0                                     # 14:59:59 → 15시 관측은 아직 없음
    assert orch.view_for(task).wind_ref["observed_kst"] == "2019-04-04 14:00"
    env.simulation_time_s = 3600.0
    assert orch.view_for(task).wind_ms == 6.3


def test_uav_gets_observed_wind_not_true_wind(tmp_path):
    orch, env, uav, lg = _world(tmp_path)                              # 진짜 세계 풍속 3.0 / 현장 14.0
    out = orch.dispatch(_recon(orch).task_id)
    assert out["status"] == "STARTED"
    assert uav.exec_calls()[0][3]["wind_ms"] == 5.7                    # 인제 관측소 14시 관측값
    ev = [e for e in lg.events() if e["event_type"] == "CANDIDATES_FILTERED"][0]["detail"]
    assert ev["wind_ref"]["source"] == "KMA_ASOS_HOURLY"


def test_field_measurement_overrides_station_for_that_cell(tmp_path, field_policy):
    field_policy(600.0)                                                # 시험 전용 유효시간 (운영값 아님)
    orch, env, uav, lg = _world(tmp_path)
    t = submit(orch, "S1", kind="ENV_SENSE",
               target={"lat": FIRE["lat"], "lon": FIRE["lon"], "ground_amsl_m": 500.0, "cell_id": "F1"},
               requirements={"resource_types": ["UAV"], "sensor": "WEATHER"})
    orch.dispatch(t.task_id)
    for _ in range(4):
        orch.poll()
    v = orch.view_for(_recon(orch, "R2"))
    assert v.wind_ref["basis"] == "FIELD_AT_TARGET_CELL" and v.wind_ms == 14.0   # 현장만 강풍 — 관측소와 다름
    field = orch.analysis.last_input["field_weather"]                          # 분석 입력에도 현장 측정 포함
    assert field and field[-1]["cell_id"] == "F1" and field[-1]["source"] == "SIMULATED"


def test_fire_report_event_api(tmp_path):
    from fastapi.testclient import TestClient
    from orchestrator.api import create_app
    orch, env, uav, lg = _world(tmp_path)
    c = TestClient(create_app(orch))
    r = c.post("/events", json={"event_id": "EV-CALL-1", "type": "FIRE_REPORT",
                                "payload": {"cell_id": "F1", "source": "119_CALL", "note": "남전리 주민 신고"}})
    assert r.status_code == 202
    b = c.get("/priority/board").json()
    assert b["known"]["fires"][0]["status"] == "REPORTED" and b["known"]["fires"][0]["source"] == "119_CALL"
    assert {s["station_id"] for s in b["known"]["stations"]} == {"90", "100", "211", "212"}
    same = c.post("/events", json={"event_id": "EV-CALL-1", "type": "FIRE_REPORT",
                                   "payload": {"cell_id": "F1", "source": "119_CALL", "note": "남전리 주민 신고"}})
    assert same.status_code == 200 and same.json()["duplicate"] is True
    # 같은 event_id 에 다른 내용 → 중복 성공으로 숨기지 않고 충돌 (B04)
    other = c.post("/events", json={"event_id": "EV-CALL-1", "type": "FIRE_REPORT", "payload": {"cell_id": "F1"}})
    assert other.status_code == 409 and other.json()["reason"] == "EVENT_ID_CONFLICT"
