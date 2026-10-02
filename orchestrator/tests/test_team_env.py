# -*- coding: utf-8 -*-
"""총괄 ↔ 환경팀 계약 환경(실제 CA) 연동 시험.

FixtureEnv 대신 environment/src/contract_env.py(김은주님 CA 래핑)를 진짜 세계로 붙여서
신고 → 최초 정찰 → (가짜 UAV) 도착 → 모의 열화상 → 환경 APPLY ACK → 목적 완료, 그리고
ENV-04 분석(belief 만 입력)이 판단 화면에 들어가는지 확인한다. HTTP 계층은 FastAPI TestClient 로 따로 본다.
"""

import copy
import shutil
import sys
from pathlib import Path

import pytest

pytest.importorskip("rasterio")

ROOT = Path(__file__).resolve().parents[2]
ENV_DIR = ROOT / "environment"
if str(ENV_DIR) not in sys.path:
    sys.path.insert(0, str(ENV_DIR))

from orchestrator import config                                   # noqa: E402
from orchestrator.engine import Orchestrator                      # noqa: E402
from orchestrator.knowledge import KmaAsosReplay                  # noqa: E402
from orchestrator.ledger import Ledger                            # noqa: E402
from orchestrator.resources import UavClient, UgvClient           # noqa: E402
from orchestrator.team_env import HttpTeamEnv, InProcessTeamEnv, TeamAnalysis   # noqa: E402
from orchestrator.tests.fakes import FakeUav, FakeUgv, normal_flight          # noqa: E402
from src.belief_analysis import BeliefSpreadAnalysis              # noqa: E402
from src.contract_env import ContractEnvironment                  # noqa: E402

IGN = "19_142"


@pytest.fixture(scope="module")
def base_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("team_env_base")
    ContractEnvironment(d)
    return d


@pytest.fixture
def team(base_dir, tmp_path):
    d = tmp_path / "env"
    shutil.copytree(base_dir, d)
    c = ContractEnvironment(d)
    uav = FakeUav(positions={"A-uav1": (38.02, 128.15), "A-uav2": (38.03, 128.16)})
    uav.default_script = normal_flight()
    an = TeamAnalysis(BeliefSpreadAnalysis(c.static, copy.deepcopy(c.engine._config), tick_s=c.tick_s))
    orch = Orchestrator(Ledger(str(tmp_path / "orch.sqlite3")), InProcessTeamEnv(c),
                        UavClient(uav.endpoints(), uav.client()), UgvClient("http://ugv", FakeUgv().client()),
                        analysis=an, weather_feeds=[KmaAsosReplay(config.KMA_ASOS_CSV, config.KMA_ASOS_STATIONS)])
    return {"c": c, "orch": orch, "uav": uav, "an": an}


def _ev(orch, task_id, et):
    return [e for e in orch.ledger.events(task_id) if e["event_type"] == et]


def test_report_to_recon_to_env_ack_closed_loop(team):
    c, orch, uav = team["c"], team["orch"], team["uav"]
    created = orch.sync()["created"]                 # t=0 신고(ENV-06) → 최초 정찰
    assert len(created) == 1
    task = orch.ledger.get_task(created[0])
    assert task.target.cell_id == IGN
    out = orch.dispatch(task.task_id)
    assert out["status"] == "STARTED"
    cmd = uav.exec_calls()[0][3]
    cell = c.map_cells()[c.idx_of(IGN)]
    assert cmd["target"]["alt_m_amsl"] == cell["ground_amsl_m"]       # 지도 지면고도 그대로
    assert (cmd["target"]["lat"], cmd["target"]["lon"]) == (cell["lat"], cell["lon"])
    for _ in range(3):
        orch.poll()
    t = orch.ledger.get_task(task.task_id)
    assert t.purpose_status == "COMPLETED"
    applies = _ev(orch, task.task_id, "ENVIRONMENT_APPLY")
    assert applies and applies[0]["result"] == "ACK"
    # 정찰 방문에 기상 측정이 붙어(extra_sensors) 관측이 2건 — 각 관측은 환경에 한 번씩만 반영된다
    sent = {e["detail"]["observation_id"] for e in orch.ledger.events() if e["event_type"] == "ENVIRONMENT_APPLY"}
    assert c.applied_count() == len(sent) >= 1
    orch.poll(); orch.poll()                           # 더 돌려도 재반영 없음
    assert c.applied_count() == len(sent)
    obs = _ev(orch, task.task_id, "OBSERVATION")[0]["detail"]
    assert obs["result"] == "DETECTED" and {"cell_id": IGN, "fire_state": "BURNING"} in obs["detections"]


def test_view_uses_belief_analysis_not_truth(team):
    c, orch = team["c"], team["orch"]
    orch.sync()
    c.advance(15)                                      # 진짜 세계는 크게 번짐
    v = orch.view()
    assert v.analysis_source == "ENV_BELIEF_ANALYSIS"
    assert {f["cell_id"] for f in v.fire_cells} == {IGN}          # 총괄은 신고된 불만 안다 (정답지 없음)
    assert len(c.snapshot()["fire_cells"]) > 1
    assert v.risk_cells and all(r.get("lat") is not None for r in v.risk_cells)   # 지도와 결합됨
    assert v.forecast_ref["wind_basis"] == "STATION"               # 기상청 관측소 값으로 예측
    fed = team["an"].last_input
    assert {f["cell_id"] for f in fed["known_fires"]} == {IGN} and "fire_cells" not in fed


def test_http_contract_roundtrip(base_dir, tmp_path):
    """environment/server.py 를 TestClient 로 띄우고 HttpTeamEnv 로 READ/APPLY/중복/분석."""
    from fastapi.testclient import TestClient
    import os
    os.environ["ENV_SERVER_NO_AUTOBUILD"] = "1"
    sys.path.insert(0, str(ROOT))
    from environment.server import build_analysis, create_app
    d = tmp_path / "env"
    shutil.copytree(base_dir, d)
    c = ContractEnvironment(d)
    client = TestClient(create_app(c, build_analysis(c)))
    env = HttpTeamEnv("http://testserver", client=client)
    s1, s2 = env.read(), env.read()
    assert s1.state_version == s2.state_version and s1.contract_complete and s1.source == "TEAM_API"
    assert len(env.map_cells()) == c.grid.rows * c.grid.cols
    obs = {"observation_id": "OBS-H1", "run_id": s1.run_id, "simulation_time_s": 0.0, "sensor_type": "THERMAL",
           "covered_cells": [IGN], "detections": [{"cell_id": IGN, "fire_state": "BURNING"}]}
    a1, a2 = env.apply(obs), env.apply(obs)
    assert a1["accepted"] and a2["duplicate"] and c.applied_count() == 1
    assert env.advance(2).simulation_time_s == 120.0
    an = TeamAnalysis(base_url="http://testserver", client=client)
    r = an.analyze({"simulation_time_s": 0.0, "known_fires": [{"cell_id": IGN}], "station_weather": [],
                    "field_weather": []})
    assert r["source"] == "ENV_BELIEF_ANALYSIS" and r["forecast_ref"]["wind_basis"] == "NONE_ASSUMED_CALM"
    assert env.reports_until(0.0)[0]["cell_id"] == IGN
