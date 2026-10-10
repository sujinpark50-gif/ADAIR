# -*- coding: utf-8 -*-
"""총괄 연결 — 총괄의 실제 UGV 클라이언트(orchestrator/resources.py UgvClient)로 UGV 서버(4대, sim 드라이버)를 부른다.

차량별 상태 조회(종류·거점 구분), 차량별 평가·실행·조회, 한 차량 명령이 다른 차량에 적용되지 않음,
소방차 주행 도착 보고가 화재 관측·진화 완료로 취급되지 않음(ROAD_STATUS 만), 자원 종류 거르기.
"""

import json
from pathlib import Path

from fastapi.testclient import TestClient

from interfaces.exec_store import ExecStore
from orchestrator.resources import UgvClient, capabilities
from ugv import config
from ugv.clock import ManualClock
from ugv.fire import StaticFireSource
from ugv.geo import FRAME
from ugv.server import Fleet, create_app
from ugv.tests.test_stage3_drive import NET

FLEET_CFG = [{**c, "driver": "sim"} for c in json.loads((Path(config.UGV_DIR) / "fleet_ab.json").read_text(encoding="utf-8"))]


def test_orchestrator_client_sees_and_commands_each_vehicle(tmp_path):
    fleet = Fleet(FLEET_CFG, net=NET, fire_source=StaticFireSource([], "T"), clock=ManualClock(),
                  store=ExecStore(tmp_path / "t.json"))
    app = create_app(fleet)
    with TestClient(app) as http:
        ugv = UgvClient(base_url="http://testserver", client=http)
        views = {v.resource_id: v for v in ugv.states()}
        assert set(views) == {"A-ugv1", "A-fire1", "B-ugv1", "B-fire1"}
        assert {v.resource_type for v in views.values()} == {"UGV", "FIRE_ENGINE"}
        assert views["A-fire1"].resource_type == "FIRE_ENGINE" and views["B-ugv1"].base == "B"
        # 소방차는 열화상·도로 관측 능력이 없다 → UGV 열화상 임무 후보가 되지 않는다 (총괄 prefilter 가 종류·능력으로 거른다)
        assert "THERMAL" not in views["A-fire1"].sensors and "ROAD_STATUS" not in capabilities("FIRE_ENGINE")
        assert all(v.ready for v in views.values())
        # 소방차 하나에만 명령
        a = fleet.vehicles["A-fire1"]
        tgt = FRAME.to_ll(a.access.x - 900, a.access.y - 700)
        payload = {"task_id": "T-F1", "decision_id": "D-F1", "target": {"lat": tgt[0], "lon": tgt[1]}}
        ev = ugv.evaluate("A-fire1", payload)
        assert ev["verdict"] == "ACCEPT" and ev["echo"]["resource_id"] == "A-fire1"
        kind, body = ugv.execute("A-fire1", {**payload, "target_node": ev["constraints"]["target_node"]["node_id"]})
        assert kind == "ACK" and body["resource_id"] == "A-fire1"
        st, t = ugv.task_status("A-fire1", "T-F1")
        assert t.get("resource_id", "A-fire1") == "A-fire1"
        views = {v.resource_id: v for v in ugv.states()}
        assert views["A-fire1"].current_task_id == "T-F1"
        assert all(views[r].current_task_id is None for r in ("A-ugv1", "B-ugv1", "B-fire1"))
        # 다른 차량 경로로 같은 task_id 조회는 없다
        assert http.get("/ugv/A-ugv1/task/T-F1").status_code == 404
        # 한 차량 중단이 다른 차량에 영향 없음
        r = http.post("/ugv/A-fire1/task/T-F1/abort", json={"reason": "TEST"})
        assert r.status_code == 200 and r.json()["status"] == "CANCELLED"
        assert all(fleet.vehicles[x].state == "IDLE" for x in ("A-ugv1", "B-ugv1", "B-fire1"))
        # 소방차 도착 보고 형식: 도로 상태(ROAD_STATUS)뿐, 화재 관측·진화 아님
        obs = http.get("/ugv/A-fire1/observation").json()
        assert obs["observation_type"] == "ROAD_STATUS"
        h = http.get("/health").json()
        assert h["vehicles"]["B-fire1"]["profile"] == "adair_firetruck" and h["unplaced"] == {}
