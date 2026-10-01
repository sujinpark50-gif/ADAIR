# -*- coding: utf-8 -*-
"""외부 이벤트 접수 (검토 2026-09-30 B04) + 신고 → 최초 정찰 Task (사용자 결정 D01).

이벤트 API 는 HTTP(ASGI TestClient)로 응답 코드까지 확인한다.
  422 거절       — event_id 를 쓰지 않는다 (고쳐서 같은 ID 로 다시 보낼 수 있다)
  202 접수·반영  — 신고면 '신고됨' 기록 + 최초 RECON Task 멱등 생성
  200 duplicate  — 같은 run·event_id·같은 내용
  409            — 같은 event_id 에 다른 내용 (EVENT_ID_CONFLICT), run 불일치
접수 성공은 출동 성공이 아니다. 출동은 기존 배정·Safety·점유 조건을 따른다.
"""

import pytest
from fastapi.testclient import TestClient

from orchestrator.api import create_app
from orchestrator.engine import Orchestrator
from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis
from orchestrator.ledger import Ledger
from orchestrator.resources import UavClient, UgvClient

from .conftest import TARGET, submit


class Crash(BaseException):
    """총괄 프로세스 강제 종료 (일반 예외 처리에 잡히지 않는다)"""


def _report(c, event_id, cell="C1", **extra):
    return c.post("/events", json={"event_id": event_id, "type": "FIRE_REPORT",
                                   "payload": {"cell_id": cell, "source": "119_CALL"}, **extra})


def _recons(lg):
    return [t for t in lg.list_tasks() if t.kind == "RECON"]


def _restart(world):
    world["ledger"].close()
    lg = Ledger(world["db"])
    uav, env = world["uav"], world["env"]
    orch = Orchestrator(lg, env, UavClient(uav.endpoints(), uav.client()),
                        UgvClient("http://ugv", world["ugv"].client()), analysis=InjectedAnalysis(),
                        weather_feeds=[EnvStationFeed(env, TARGET["lat"], TARGET["lon"])])
    return orch, lg


# ---------------------------------------------------------------------------
# B04 접수키
# ---------------------------------------------------------------------------
def test_rejected_report_does_not_block_corrected_resend_with_same_event_id(world):
    c, lg = TestClient(create_app(world["orch"])), world["ledger"]
    bad = c.post("/events", json={"event_id": "EV-1", "type": "FIRE_REPORT", "payload": {"source": "119_CALL"}})
    assert bad.status_code == 422 and bad.json()["reason"] == "CELL_ID_MISSING"
    assert world["orch"].kb.fire_states(1e9) == {} and lg.list_tasks() == []
    rejected = [e for e in lg.events() if e["event_type"] == "EXTERNAL_EVENT_REJECTED"][0]
    assert rejected["detail"]["idempotency_key_consumed"] is False

    ok = _report(c, "EV-1")                                       # 같은 event_id 로 고쳐서 재전송
    assert ok.status_code == 202 and ok.json()["duplicate"] is False
    assert world["orch"].kb.fire_states(1e9)["C1"]["status"] == "REPORTED"


def test_same_valid_event_is_idempotent_and_different_content_is_a_conflict(world):
    c, lg, uav = TestClient(create_app(world["orch"])), world["ledger"], world["uav"]
    assert _report(c, "EV-1").status_code == 202
    again = _report(c, "EV-1")
    assert again.status_code == 200 and again.json()["duplicate"] is True
    conflict = _report(c, "EV-1", cell="C2")
    assert conflict.status_code == 409 and conflict.json() == {
        "accepted": False, "reason": "EVENT_ID_CONFLICT", "detail": "같은 event_id 로 다른 내용이 이미 접수되어 있다"}
    assert set(world["orch"].kb.fire_states(1e9)) == {"C1"}        # 충돌 요청은 아무것도 바꾸지 않는다
    assert len(_recons(lg)) == 1 and len(uav.exec_calls()) == 1


@pytest.mark.parametrize("body,status,reason", [
    ({"event_id": "E", "type": "FIRE_REPORT", "payload": {"cell_id": ""}}, 422, "CELL_ID_MISSING"),
    ({"event_id": "E", "type": "FIRE_REPORT", "payload": {"cell_id": 7}}, 422, "CELL_ID_MISSING"),
    ({"event_id": "E", "type": "FIRE_REPORT", "payload": {"cell_id": "C1"}, "simulation_time_s": -1}, 422,
     "SIMULATION_TIME_INVALID"),
    ({"event_id": "E", "type": "FIRE_REPORT", "payload": {"cell_id": "C1"}, "simulation_time_s": 9999}, 422,
     "SIMULATION_TIME_IN_FUTURE"),
    ({"event_id": " ", "type": "FIRE_REPORT", "payload": {"cell_id": "C1"}}, 422, "EVENT_ID_MISSING"),
    ({"event_id": "E", "type": "FIRE_REPORT", "payload": {"cell_id": "C1"}, "run_id": "OTHER-RUN"}, 409,
     "RUN_MISMATCH"),
])
def test_invalid_events_are_rejected_over_http_without_consuming_the_key(world, body, status, reason):
    c = TestClient(create_app(world["orch"]))
    r = c.post("/events", json=body)
    assert r.status_code == status and r.json()["reason"] == reason
    assert world["ledger"].pending_external_events() == [] and world["ledger"].list_tasks() == []
    assert _report(c, "E").status_code == 202                     # 같은 ID 의 올바른 요청은 접수된다


def test_malformed_body_is_rejected_by_schema(world):
    c = TestClient(create_app(world["orch"]))
    assert c.post("/events", json={"type": "FIRE_REPORT", "payload": {"cell_id": "C1"}}).status_code == 422
    assert c.post("/events", json={"event_id": "E", "type": "FIRE_REPORT", "payload": "C1"}).status_code == 422


def test_event_received_but_not_applied_is_recovered_after_restart(world):
    orch, lg, uav = world["orch"], world["ledger"], world["uav"]
    body = {"event_id": "EV-1", "type": "FIRE_REPORT", "payload": {"cell_id": "C1", "source": "119_CALL"}}

    def crash(*a, **k):
        raise Crash()
    orch._initial_recon = crash                                    # 접수·신고 기록 뒤, Task 생성 전에 종료
    with pytest.raises(Crash):
        orch.ingest_event(body)
    assert [e["event_id"] for e in lg.pending_external_events()] == ["EV-1"]
    assert orch.kb.fire_states(1e9)["C1"]["status"] == "REPORTED" and lg.list_tasks() == []

    orch2, lg2 = _restart(world)
    assert orch2.recover()["events_resumed"] == ["EV-1"]           # 재시작 시 마저 반영 — 이벤트가 사라지지 않는다
    assert lg2.pending_external_events() == [] and len(_recons(lg2)) == 1
    code, out = orch2.ingest_event(body)                           # 보낸 쪽이 다시 보내도 한 번만
    assert code == 200 and out["duplicate"] is True and len(_recons(lg2)) == 1
    orch2.dispatch_pending()
    orch2.dispatch_pending()
    assert len(uav.exec_calls()) == 1                              # 중복 출동 없음


def test_resend_of_received_but_unapplied_event_resumes_it(world):
    orch, lg = world["orch"], world["ledger"]
    body = {"event_id": "EV-1", "type": "FIRE_REPORT", "payload": {"cell_id": "C1"}}
    real = orch._initial_recon

    def crash(*a, **k):
        raise Crash()
    orch._initial_recon = crash
    with pytest.raises(Crash):
        orch.ingest_event(body)
    orch._initial_recon = real
    code, out = orch.ingest_event(body)
    assert code == 202 and out["resumed"] is True and out["applied"]["recon"]["decision"] == "NEW_INCIDENT"
    assert len(_recons(lg)) == 1 and lg.pending_external_events() == []


# ---------------------------------------------------------------------------
# D01 신고 → 최초 정찰
# ---------------------------------------------------------------------------
def test_report_on_empty_ledger_creates_recon_and_dispatches_exactly_once(world):
    c, lg, uav = TestClient(create_app(world["orch"])), world["ledger"], world["uav"]
    r = _report(c, "EV-1")
    assert r.status_code == 202
    recon = r.json()["applied"]["recon"]
    run = lg.active_run()
    assert recon["decision"] == "NEW_INCIDENT" and recon["incident_id"] == f"INC-AUTO:{run}:C1:1"
    task = _recons(lg)[0]
    assert task.task_id == recon["task_id"] and task.request_key == f"AUTO-RECON:{run}:EVENT:EV-1"
    assert task.target.cell_id == "C1" and task.target.ground_amsl_m == 600.0     # 지도(정적 정보)에서 읽은 값
    assert r.json()["redispatched"][0]["status"] == "STARTED" and len(uav.exec_calls()) == 1

    assert _report(c, "EV-1").status_code == 200                   # 같은 신고 재전송
    dup = _report(c, "EV-2")                                       # 같은 칸의 별도 중복 신고
    assert dup.status_code == 202 and dup.json()["applied"]["recon"]["decision"] == "DUPLICATE_OF_OPEN_TASK"
    assert dup.json()["applied"]["recon"]["task_id"] == task.task_id
    assert len(_recons(lg)) == 1 and len(uav.exec_calls()) == 1


def test_report_acceptance_is_not_dispatch_success_when_no_resource_then_assigned_later(world):
    c, lg, uav = TestClient(create_app(world["orch"])), world["ledger"], world["uav"]
    for rid in uav.ids:
        uav.state[rid]["failsafe"] = True
    r = _report(c, "EV-1")
    assert r.status_code == 202 and r.json()["redispatched"][0]["status"] == "HOLD"
    t = _recons(lg)[0]
    assert t.purpose_status == "HOLD" and t.hold_reason == "NO_FEASIBLE_CANDIDATE" and uav.exec_calls() == []
    uav.state["A-uav1"]["failsafe"] = False
    e = c.post("/events", json={"event_id": "EV-RES", "type": "RESOURCE_CHANGED"})
    assert e.json()["redispatched"][0]["status"] == "STARTED" and len(uav.exec_calls()) == 1


def test_report_for_cell_without_map_location_is_held_not_sent_to_invented_coordinates(world):
    c, lg, uav, env = TestClient(create_app(world["orch"])), world["ledger"], world["uav"], world["env"]
    r = _report(c, "EV-1", cell="NOT-ON-MAP")
    assert r.status_code == 202
    t = _recons(lg)[0]
    assert t.target.lat is None and t.target.lon is None and t.target.ground_amsl_m is None
    assert t.purpose_status == "HOLD" and t.hold_reason == "TARGET_LOCATION_UNKNOWN"
    assert t.resume_condition == "MAP_INFO" and uav.eval_calls() == [] and uav.exec_calls() == []
    # 지도에 그 칸이 생기면 지도 값으로 채워서 배정한다
    env.risk_cells.append({"cell_id": "NOT-ON-MAP", "lat": 38.06, "lon": 128.26, "cell_size_m": 90.0,
                           "ground_amsl_m": 640.0, "fire_state": "UNBURNED"})
    out = world["orch"].dispatch_pending()["results"][t.task_id]
    assert out["status"] == "STARTED"
    sent = uav.exec_calls()[0][3]["target"]
    assert (sent["lat"], sent["lon"], sent["alt_m_amsl"]) == (38.06, 128.26, 640.0)


def test_new_report_after_clear_observation_is_a_new_incident(world):
    orch, c, lg, env = world["orch"], TestClient(create_app(world["orch"])), world["ledger"], world["env"]
    env.fire_cells[0].update(fire_state="UNBURNED", cell_size_m=40.0)      # 오인 신고, 칸 전체가 관측 범위 안
    _report(c, "EV-1")
    for _ in range(4):
        orch.poll()
    first = _recons(lg)[0]
    assert first.purpose_status == "COMPLETED" and orch.kb.fire_states(1e9)["C1"]["status"] == "OBSERVED_CLEAR"
    r = _report(c, "EV-2")                                          # 같은 칸, 나중의 별개 신고
    recon = r.json()["applied"]["recon"]
    assert recon["decision"] == "NEW_INCIDENT" and recon["incident_id"].endswith(":C1:2")
    assert len(_recons(lg)) == 2 and recon["task_id"] != first.task_id


def test_report_on_already_confirmed_fire_creates_nothing_but_explicit_recheck_does(world):
    orch, c, lg, uav = world["orch"], TestClient(create_app(world["orch"])), world["ledger"], world["uav"]
    _report(c, "EV-1")
    for _ in range(4):
        orch.poll()
    first = _recons(lg)[0]
    assert first.purpose_status == "COMPLETED" and orch.kb.fire_states(1e9)["C1"]["status"] == "CONFIRMED"
    r = _report(c, "EV-2")
    assert r.json()["applied"]["recon"]["decision"] == "ALREADY_CONFIRMED" and len(_recons(lg)) == 1
    re = c.post(f"/tasks/{first.task_id}/resolve", json={"action": "RECHECK", "reason": "재발화 확인",
                                                         "request_id": "OP-1"}).json()
    assert re["status"] == "CREATED" and lg.get_task(re["task_id"]).parent_task_id == first.task_id
    assert lg.get_task(first.task_id).purpose_status == "COMPLETED"


def test_open_manual_recon_at_cell_absorbs_later_report(world):
    orch, lg = world["orch"], world["ledger"]
    manual = submit(orch)                                           # 사람이 먼저 넣은 C1 정찰 (대기 중)
    code, out = orch.ingest_event({"event_id": "EV-1", "type": "FIRE_REPORT", "payload": {"cell_id": "C1"}})
    assert code == 202 and out["applied"]["recon"] == {
        **out["applied"]["recon"], "decision": "DUPLICATE_OF_OPEN_TASK", "task_id": manual.task_id}
    assert [t.task_id for t in _recons(lg)] == [manual.task_id]


def test_scenario_report_creates_recon_once_across_views_sync_and_restart(world):
    orch, lg, env, uav = world["orch"], world["ledger"], world["env"], world["uav"]
    env.reports = [{"cell_id": "C1", "sim_time_s": 120.0, "source": "119_CALL"}]
    assert orch.sync()["created"] == [] and lg.list_tasks() == []   # 아직 신고 시각 전
    env.simulation_time_s = 120.0
    created = orch.sync()["created"]
    assert len(created) == 1 and orch.sync()["created"] == []
    orch.view()
    assert [t.task_id for t in _recons(lg)] == created
    orch2, lg2 = _restart(world)
    orch2.view()
    assert orch2.sync()["created"] == [] and [t.task_id for t in _recons(lg2)] == created
    out = orch2.dispatch_pending()
    assert out["results"][created[0]]["status"] == "STARTED"
    orch2.dispatch_pending()
    assert len(uav.exec_calls()) == 1
