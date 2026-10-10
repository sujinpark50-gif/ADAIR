# -*- coding: utf-8 -*-
"""총괄 ↔ UGV 폐루프 — 실제 총괄 엔진(orchestrator.engine.Orchestrator, 장부, UgvClient)과 실제 UGV 서버(4대, sim 드라이버, 30배속)를
HTTP(TestClient)로 잇는다. PX4 주행이 아니다 (Gazebo 폐루프는 보고서 4단계 참고).

시나리오 A (차량 고장): 임무 배정(도로 상황 확인) → A-ugv1 출동(명령 수락) → 주행 중 고장 → UGV 임무 FAILED(phase=FAILED) →
  총괄: 그 시도 FAULTED·임무 재대기 → 다시 판단해 다른 UGV(A-ugv2 — B 거점 UGV 는 A 지역에 갈 경로가 없어 거절) 배정 → 실제 도착 → 임무 완료(ROAD_STATUS, 화재 관측 아님) →
  READY 확인 반납 → 복귀 명령 → 거점 READY. 고장 차는 운영자 해제(/stop) 뒤 READY → 총괄 반납.
시나리오 B (막힘): 출동한 차가 서 있는 차에 막혀 우회 경로가 없다 → UGV 임무 FAILED(BLOCKED_BY_VEHICLE, phase=DONE) →
  총괄: 임무 재대기, 그 차는 READY 확인 뒤 바로 반납 (고장으로 묶지 않음).
명령 수락 / 실제 도착 / 임무 완료 / 복귀 후 READY 를 따로 기록·검사한다. 관측 기능(지형 가시성)은 범위 밖 — 도착 보고는 ROAD_STATUS.
"""

import json
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from interfaces.exec_store import ExecStore
from orchestrator.engine import Orchestrator
from orchestrator.env_adapter import FixtureEnv
from orchestrator.ledger import Ledger
from orchestrator.resources import UgvClient
from ugv import config
from ugv.clock import SimClock
from ugv.fire import StaticFireSource
from ugv.geo import FRAME, bearing_deg
from ugv.server import Fleet, create_app
from ugv.tests.test_stage3_drive import NET

FLEET_CFG = [{**c, "driver": "sim"} for c in json.loads((Path(config.UGV_DIR) / "fleet_ab.json").read_text(encoding="utf-8"))]
# A 거점 UGV 한 대 더 (대기 위치 2): B 거점 차는 A 지역에 진행 방향 조건으로 갈 수 있는 경로가 없어(NO_DIRECTED_ROUTE) 인계를 받지 못한다
FLEET_CFG += [{**FLEET_CFG[0], "resource_id": "A-ugv2", "station_slot": 2}]
SCALE = 30.0


def _loop(tmp_path, monkeypatch, scale=SCALE):
    monkeypatch.setenv("UGV_TEST_HOOKS", "1")
    fleet = Fleet(FLEET_CFG, net=NET, fire_source=StaticFireSource([], "T"), clock=SimClock(scale=scale),
                  store=ExecStore(tmp_path / "ugv_tasks.json"))
    return fleet, create_app(fleet)


def _until(orch, cond, timeout_s, every=0.3):
    end = time.time() + timeout_s
    while time.time() < end:
        orch.poll()
        if cond():
            return True
        time.sleep(every)
    return False


def _submit(orch, rid, lat, lon):
    task, _ = orch.submit_task({"request_id": rid, "incident_id": "INC-LOOP", "kind": "GROUND_RECON",
                                "target": {"lat": lat, "lon": lon, "cell_id": rid},
                                "requirements": {"resource_types": ["UGV"], "sensor": "ROAD_STATUS", "needs_env_ack": False}})
    return task


def test_closed_loop_fault_redecision_other_resource_return_ready(tmp_path, monkeypatch):
    fleet, app = _loop(tmp_path, monkeypatch)
    rec = {}
    with TestClient(app) as http:
        ugv = UgvClient(base_url="http://testserver", client=http)
        orch = Orchestrator(Ledger(str(tmp_path / "orch.db")), FixtureEnv(), ugv=ugv)
        A = fleet.vehicles["A-ugv1"]
        lat, lon = FRAME.to_ll(A.access.x - 900, A.access.y - 700)
        task = _submit(orch, "LOOP-A", lat, lon)
        # 1) 배정·출동 — 명령 수락
        out = orch.dispatch(task.task_id)
        assert out["status"] == "STARTED" and out["resource_id"] == "A-ugv1", out
        att1 = out["attempt_id"]
        rec["accepted_1"] = {"resource": out["resource_id"], "attempt": att1}
        assert _until(orch, lambda: A.state == "DRIVING" and (A.driver.telemetry().speed_mps or 0) > 3, 30)
        # 2) 주행 중 고장
        r = http.post("/ugv/A-ugv1/_test/fault", json={"code": "VEHICLE_FAULT", "detail": "폐루프 시험 주입: 주행 중 고장"})
        assert r.status_code == 200 and r.json()["state"] in ("FAULT", "UNAVAILABLE") or r.json().get("fault")
        ut = [t for t in fleet.store.tasks.values() if t["resource_id"] == "A-ugv1"][-1]
        assert ut["status"] == "FAILED" and ut["progress"]["phase"] == "FAILED"
        # 3) 총괄: 고장 시도 FAULTED, 임무 재대기 (다른 자원에 인계)
        assert _until(orch, lambda: orch.ledger.get_attempt(att1)["substatus"] == "FAULTED", 15)
        assert orch.ledger.get_task(task.task_id).purpose_status == "PENDING"
        rec["fault"] = {"attempt": att1, "substatus": "FAULTED", "task": "PENDING"}
        # 고장 차는 멈춘다 (실제 정지)
        assert _until(orch, lambda: A.driver.telemetry().speed_mps < 0.2, 20)
        # 4) 재판단 → 다른 UGV
        out2 = orch.dispatch(task.task_id)
        assert out2["status"] == "STARTED" and out2["resource_id"] == "A-ugv2", out2
        att2 = out2["attempt_id"]
        assert out2["resource_id"] != "A-ugv1"                              # 고장·점유 차는 후보에서 빠진다
        rec["accepted_2"] = {"resource": "A-ugv2", "attempt": att2, "excluded": out2.get("excluded")}
        B = fleet.vehicles["A-ugv2"]
        # 5) 실제 도착 (UGV ARRIVED) → 총괄 ARRIVED → 관측(ROAD_STATUS) → 임무 완료 → READY 확인 반납
        assert _until(orch, lambda: orch.ledger.get_attempt(att2)["substatus"] in ("ARRIVED", "OBSERVED", "APPLIED", "RELEASED"),
                      240), orch.ledger.get_attempt(att2)
        bt = [t for t in fleet.store.tasks.values() if t["resource_id"] == "A-ugv2"][-1]
        assert bt["status"] == "COMPLETED" and bt["mission_result"] == "ARRIVED"
        rec["arrived_2"] = {"ugv_task": bt["task_id"], "mission_result": bt["mission_result"]}
        assert _until(orch, lambda: orch.ledger.get_attempt(att2)["substatus"] == "RELEASED", 30)
        assert orch.ledger.get_task(task.task_id).purpose_status == "COMPLETED"
        obs = [e for e in orch.ledger.events(task.task_id) if e["event_type"] == "OBSERVATION"]
        assert obs and obs[-1]["result"] == "ROAD_STATUS" and obs[-1]["detail"]["is_fire_observation"] is False
        rec["completed"] = {"purpose": "COMPLETED", "observation": "ROAD_STATUS", "released": att2}
        # 6) 복귀 → 거점 READY
        st = http.get("/ugv/A-ugv2/state").json()
        acc = st["station"]["access_point"]
        kind, body = ugv.execute("A-ugv2", {"task_id": "LOOP-HOME-B", "decision_id": "D-HOME",
                                            "target_node": f"RP:{acc['road_id']}:{acc['s_m']:.1f}"})
        assert kind == "ACK"
        assert _until(orch, lambda: fleet.store.tasks["LOOP-HOME-B"]["status"] == "COMPLETED", 240)
        v = {x.resource_id: x for x in ugv.states()}["A-ugv2"]
        assert v.ready and v.current_task_id is None
        tel = B.driver.telemetry()
        assert ((tel.x - B.access.x) ** 2 + (tel.y - B.access.y) ** 2) ** 0.5 <= config.ARRIVE_M * 3
        rec["returned_ready"] = {"resource": "A-ugv2", "ready": True}
        # 7) 고장 차: 운영자 해제 → READY → 총괄 반납
        r = http.post("/ugv/A-ugv1/stop", json={"reason": "OPERATOR_RELEASE"})
        assert r.status_code == 200
        assert _until(orch, lambda: orch.ledger.get_attempt(att1)["substatus"] == "RELEASED", 30)
        assert orch.ledger.reservations() == {}
        rec["fault_released"] = {"attempt": att1}
    (tmp_path / "closed_loop_record.json").write_text(json.dumps(rec, ensure_ascii=False, indent=1), encoding="utf-8")


def test_closed_loop_blocked_vehicle_is_released_not_faulted(tmp_path, monkeypatch):
    # 차량 간 판단 주기는 벽시계라 30배속이면 한 주기에 차가 너무 많이 나아가 막힘 처리 전에 정지 거리 안으로 들어간다 → 3배속
    fleet, app = _loop(tmp_path, monkeypatch, scale=3.0)
    with TestClient(app) as http:
        ugv = UgvClient(base_url="http://testserver", client=http)
        orch = Orchestrator(Ledger(str(tmp_path / "orch.db")), FixtureEnv(), ugv=ugv)
        A, BU = fleet.vehicles["A-ugv1"], fleet.vehicles["B-ugv1"]      # B-ugv1 을 막는 차로 옮겨 세운다
        lat, lon = FRAME.to_ll(A.access.x - 900, A.access.y - 700)
        task = _submit(orch, "LOOP-B", lat, lon)
        out = orch.dispatch(task.task_id)
        assert out["status"] == "STARTED" and out["resource_id"] == "A-ugv1"
        att = out["attempt_id"]
        assert _until(orch, lambda: A.state == "DRIVING" and 0.3 < (A.driver.telemetry().speed_mps or 0) < 4, 20, every=0.05)
        # 막 출발한 차(저속, 정지 거리 짧음) 앞 30 m(같은 출발 도로)에 임무 없는 차를 세운다 → 즉시 되짚기 금지라 우회 경로 없음
        sp = A.speed
        s_blk = A._s_now() + 30.0
        k = next(i for i, c in enumerate(sp.cum) if c >= s_blk)
        (x0, y0), (x1, y1) = sp.pts[k - 1], sp.pts[k]
        BU.driver.x, BU.driver.y = x1, y1
        BU.driver.heading = bearing_deg(x1 - x0, y1 - y0)
        assert NET.snap(x1, y1).road_id == A.access.road_id              # 출발 도로 위
        assert _until(orch, lambda: orch.ledger.get_attempt(att)["substatus"] in ("RELEASED", "FAULTED", "UNKNOWN"), 240)
        ut = [t for t in fleet.store.tasks.values() if t["resource_id"] == "A-ugv1"][-1]
        assert ut["status"] == "FAILED" and ut["error"].startswith("BLOCKED_BY_VEHICLE"), ut.get("error")
        assert ut["progress"]["phase"] == "DONE"
        assert orch.ledger.get_attempt(att)["substatus"] == "RELEASED"     # 멀쩡한 차 — 고장으로 묶지 않고 반납
        assert orch.ledger.get_task(task.task_id).purpose_status == "PENDING"   # 총괄이 다시 판단할 수 있게 재대기
        ev = [e for e in orch.ledger.events(task.task_id) if e["event_type"] == "TASK_REOPENED"]
        assert ev and ev[-1]["reason"] == "BLOCKED_BY_VEHICLE"
