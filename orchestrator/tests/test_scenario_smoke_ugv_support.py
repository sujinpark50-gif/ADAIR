# -*- coding: utf-8 -*-
"""시연 시나리오 파일(orchestrator/tests/scenarios/smoke_ugv_support)이 지금 코드로 그대로 돌아가는지 확인한다.

연기로 드론 관측 불가 → UGV 만, 관측 없음 임무 → UGV 가 출동해 도착하면 완료.
"""

import json
from pathlib import Path

from orchestrator.api import TaskIn
from orchestrator.engine import Orchestrator
from orchestrator.env_adapter import FixtureEnv
from orchestrator.ledger import Ledger
from orchestrator.resources import UavClient, UgvClient

from .fakes import FakeUav, FakeUgv

DIR = Path(__file__).resolve().parent / "scenarios" / "smoke_ugv_support"


def test_smoke_scenario_files_send_ugv_and_complete(tmp_path):
    fixture = json.loads((DIR / "fixture.json").read_text(encoding="utf-8"))
    body = TaskIn(**json.loads((DIR / "task.json").read_text(encoding="utf-8"))).model_dump()
    body.pop("dispatch")
    assert body["requirements"]["resource_types"] == ["UGV"] and body["requirements"]["sensor"] is None

    uav, ugv = FakeUav(), FakeUgv()
    tgt = body["target"]
    ugv.default_script = lambda rid: [
        {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
        {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
         "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1",
                         "position": {"lat": tgt["lat"], "lon": tgt["lon"]}},
         "_unit": {"state": "READY", "current_task_id": None}}]
    ledger = Ledger(str(tmp_path / "state.sqlite3"))
    orch = Orchestrator(ledger, FixtureEnv(**fixture), UavClient(uav.endpoints(), uav.client()),
                        UgvClient("http://ugv", ugv.client()))

    task, _ = orch.submit_task(body)
    assert task.requirements.needs_env_ack is False
    out = orch.dispatch(task.task_id)
    assert out["status"] == "STARTED" and out["resource_id"] == "A-ugv1"
    assert uav.eval_calls() == [] and uav.exec_calls() == []      # 드론에는 묻지도 보내지도 않는다
    for _ in range(3):
        orch.poll()
    assert ledger.get_task(task.task_id).purpose_status == "COMPLETED"
