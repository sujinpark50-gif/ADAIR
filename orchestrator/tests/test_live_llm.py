# -*- coding: utf-8 -*-
"""실제 OpenAI 호출 시험 (비용 발생). .env 의 OPENAI_API_KEY 필요.

  ORCH_LIVE_LLM=1 python -m pytest orchestrator/tests/test_live_llm.py -s
"""

import os

import pytest

pytestmark = pytest.mark.skipif(os.getenv("ORCH_LIVE_LLM") != "1", reason="실제 OpenAI 호출 (비용 발생)")


def test_live_llm_orders_similar_group():
    from orchestrator import config, priority
    from orchestrator.config import ROOT, _load_dotenv
    from orchestrator.env_adapter import FixtureEnv
    from orchestrator.llm import LlmPlanner
    from orchestrator.models import Target, Task

    _load_dotenv(ROOT / ".env")                 # conftest 가 막아 둔 .env 를 이 시험에서만 읽는다
    env = FixtureEnv(risk_cells=[
        {"cell_id": "R-A", "lat": 38.1010, "lon": 128.2170, "cell_size_m": 90.0, "risk_score": 0.72,
         "fire_state": "UNBURNED"},
        {"cell_id": "R-B", "lat": 38.1015, "lon": 128.2130, "cell_size_m": 90.0, "risk_score": 0.65,
         "fire_state": "UNBURNED"}],
        protected_sites=[{"site_id": "VILLAGE-1", "site_type": "VILLAGE", "human_occupied": True,
                          "boundary": [[38.0975, 128.2105], [38.0975, 128.2125], [38.0990, 128.2125],
                                       [38.0990, 128.2105]]}])
    tasks = [Task(task_id=f"TASK-{c}", incident_id="INC", kind="RECON",
                  target=Target(38.1, 128.21, 400.0, c), created_wall=i) for i, c in enumerate(("R-A", "R-B"))]
    snap = env.read()
    plan = priority.order_tasks(tasks, snap, config.PRIORITY_SIMILAR_RISK_DELTA)
    assert plan["groups"], "시험 전제: 선택 묶음이 있어야 함"
    planner = LlmPlanner()
    assert planner.status() is None, planner.status()
    out = planner.propose(snap, plan, {t.task_id: t for t in tasks})
    print("\nstatus:", out["status"], "latency_s:", out.get("latency_s"), "model:", out["model"])
    print("violations/reason:", out.get("violations") or out.get("reason") or out.get("error"))
    print("output:", out.get("raw_output"))
    assert out["status"] == "OK"
