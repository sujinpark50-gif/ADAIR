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
