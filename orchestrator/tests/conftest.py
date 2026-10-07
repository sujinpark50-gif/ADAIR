# -*- coding: utf-8 -*-
import os
import sys
from pathlib import Path

os.environ["ORCH_SKIP_DOTENV"] = "1"          # 시험은 로컬 .env 의 실제 API 키를 쓰지 않는다

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from orchestrator.engine import Orchestrator          # noqa: E402
from orchestrator.env_adapter import FixtureEnv       # noqa: E402
from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis  # noqa: E402
from orchestrator.ledger import Ledger                # noqa: E402
from orchestrator.resources import UavClient, UgvClient  # noqa: E402
from orchestrator.tests.fakes import FakeUav, FakeUgv, normal_flight  # noqa: E402

TARGET = {"lat": 38.05, "lon": 128.25, "ground_amsl_m": 600.0, "cell_id": "C1"}


def fire_env(**kw):
    cells = [{"cell_id": "C1", "lat": 38.05, "lon": 128.25, "cell_size_m": 90.0,
              "ground_amsl_m": 600.0, "fire_state": "BURNING"}]
    return FixtureEnv(fire_cells=cells, **kw)


@pytest.fixture(autouse=True)
def single_frame_sensor(monkeypatch):
    """기존 시험은 원래 관측 범위(한 프레임, 지면 위 90m 에서 52×42m)를 기준으로 쓴다.
    기본값인 가정 범위(원 비행 가정 90×90m)는 test_orbit_assumption 에서 따로 시험한다."""
    from orchestrator import config
    monkeypatch.setattr(config, "DEFAULT_SENSOR_PROFILE_ID", "TEST_THERMAL_90M_V1")


@pytest.fixture(autouse=True)
def legacy_dispatch_mode(monkeypatch):
    """기존 시험은 관측 계획 이전 동작(신고 → 최초 정찰 Task, UGV 열화상 꺼짐)을 기준으로 쓴다.
    관측 계획(2026-10-07 기본 켜짐)은 test_observation_planner 에서 obs_planner 픽스처로 켠다."""
    from orchestrator import config
    monkeypatch.setattr(config, "OBS_PLANNER_ENABLED", False)
    caps = dict(config.SIMULATED_CAPABILITIES)
    caps["UGV"] = tuple(c for c in caps["UGV"] if c != "THERMAL")
    monkeypatch.setattr(config, "SIMULATED_CAPABILITIES", caps)
    # 현장 기상 유효시간(2026-10-07 기본 35분)도 기존 시험 기준(미설정)으로 둔다. 필요한 시험은 field_policy 로 정한다
    monkeypatch.setattr(config, "FIELD_WEATHER_MAX_AGE_S", {k: None for k in config.WEATHER_ITEMS})


@pytest.fixture
def field_policy(monkeypatch):
    """현장 측정 유효시간을 시험 안에서만 설정한다 (TEST_ONLY — 운영 기본값은 미설정).
    field_policy(600) 은 모든 항목 600초, field_policy(wind_ms=600) 은 그 항목만."""
    from orchestrator import config

    def set_policy(all_items=None, **per_item):
        policy = {k: all_items for k in config.WEATHER_ITEMS}
        policy.update(per_item)
        monkeypatch.setattr(config, "FIELD_WEATHER_MAX_AGE_S", policy)
        return policy
    return set_policy


@pytest.fixture
def world(tmp_path):
    uav = FakeUav()
    uav.default_script = normal_flight()
    ugv = FakeUgv()
    env = fire_env()
    ledger = Ledger(str(tmp_path / "state.sqlite3"))
    analysis = InjectedAnalysis()          # 분석 시험 주입값 — 환경(진짜 세계)과 분리
    orch = Orchestrator(ledger, env, UavClient(uav.endpoints(), uav.client()), UgvClient("http://ugv", ugv.client()),
                        analysis=analysis,
                        weather_feeds=[EnvStationFeed(env, TARGET["lat"], TARGET["lon"])])
    return {"uav": uav, "ugv": ugv, "env": env, "ledger": ledger, "orch": orch, "analysis": analysis, "db": str(tmp_path / "state.sqlite3")}


def submit(orch, request_id="REQ-1", **over):
    body = {"request_id": request_id, "incident_id": "INC-INJE", "kind": "RECON",
            "target": dict(TARGET), "requirements": {"resource_types": ["UAV", "UGV"], "sensor": "THERMAL"}}
    body.update(over)
    task, _ = orch.submit_task(body)
    return task
