# -*- coding: utf-8 -*-
"""실행(run) 범위 격리 (검토 2026-09-30 B05).

정책: 활성 run 은 하나. 신고·관측·기상·Task·요청/이벤트 키는 run 안에서만 유효하다.
- 다른 run 의 점유·열린 실행이 남아 있으면 run 을 바꾸지 않는다 (추적은 계속, 판단은 보류).
- 닫힌 run 은 다시 활성화하지 않는다 (늦게 온 이전 run 의 snapshot·이벤트는 거절).
- run 범위가 없는 이전 장부의 행은 LEGACY-UNSCOPED 로 격리하고 새 run 에 귀속시키지 않는다.
"""

import json
import sqlite3
import time

import pytest

from orchestrator.engine import Orchestrator
from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis
from orchestrator.ledger import ALL_RUNS, LEGACY_RUN, Ledger, RunClosed, RunNotActive, RunSwitchBlocked
from orchestrator.models import Target, Task
from orchestrator.resources import UavClient, UgvClient

from .conftest import TARGET, submit

REPORT = {"cell_id": "C1", "sim_time_s": 0.0, "source": "119_CALL"}


def _new_orch(world, ledger):
    uav, env = world["uav"], world["env"]
    return Orchestrator(ledger, env, UavClient(uav.endpoints(), uav.client()),
                        UgvClient("http://ugv", world["ugv"].client()), analysis=InjectedAnalysis(),
                        weather_feeds=[EnvStationFeed(env, TARGET["lat"], TARGET["lon"])])


def _switch(env, run_id):
    env.run_id, env.fire_cells, env.reports = run_id, [], []


def test_new_run_does_not_inherit_previous_knowledge_or_tasks(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    env.reports = [dict(REPORT)]
    assert [c["cell_id"] for c in orch.view().fire_cells] == ["C1"]      # 이전 run 에서 C1 신고
    old_task = lg.list_tasks()[0]                                        # 신고로 생긴 최초 정찰 (출동 전)
    assert lg.knowledge("WEATHER")

    _switch(env, "NEW-RUN")
    v = orch.view()
    assert v.run_gate is None and lg.active_run() == "NEW-RUN"
    assert v.fire_cells == [] and orch.kb.fire_states(1e9) == {}        # 이전 C1 이 남지 않는다
    assert lg.list_tasks() == []
    assert all(k["body"].get("received_sim_s") is not None for k in lg.knowledge("WEATHER"))
    assert orch.dispatch_pending()["results"] == {} and world["uav"].exec_calls() == []
    # 이전 run 의 기록은 지우지 않고 그 run 범위에 남긴다
    assert lg.run_status("RUN-FIXTURE") == "CLOSED"
    assert [t.task_id for t in lg.list_tasks(run_id="RUN-FIXTURE")] == [old_task.task_id]
    assert orch.dispatch(old_task.task_id)["reason"] == "TASK_RUN_NOT_ACTIVE"


def test_same_request_and_event_ids_are_independent_per_run(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    env.fire_cells = []
    first = submit(orch, "REQ-1")
    code, _ = orch.ingest_event({"event_id": "EV-1", "type": "FIRE_REPORT", "payload": {"cell_id": "C1"}})
    assert code == 202
    _switch(env, "RUN-B")
    second = submit(orch, "REQ-1", kind="ENV_SENSE", requirements={"resource_types": ["UAV"], "sensor": "WEATHER"})
    assert second.task_id != first.task_id and second.run_id == "RUN-B"  # 다른 내용이어도 충돌 아님
    code, out = orch.ingest_event({"event_id": "EV-1", "type": "FIRE_REPORT", "payload": {"cell_id": "C9"}})
    assert code == 202 and out["duplicate"] is False
    assert set(orch.kb.fire_states(1e9)) == {"C9"}


def test_run_switch_is_blocked_while_resource_is_occupied_and_old_execution_stays_tracked(world):
    orch, env, uav, lg = world["orch"], world["env"], world["uav"], world["ledger"]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    _switch(env, "RUN-B")

    v = orch.view()
    assert v.run_gate == "RUN_SWITCH_BLOCKED" and lg.active_run() == "RUN-FIXTURE"
    assert v.fire_cells == [] and v.risk_cells == []                     # 두 run 의 정보를 섞지 않는다
    with pytest.raises(RunNotActive):
        submit(orch, "REQ-B")
    assert orch.dispatch_pending()["run_gate"] == "RUN_SWITCH_BLOCKED"
    code, body = orch.ingest_event({"event_id": "EV-B", "type": "FIRE_REPORT", "payload": {"cell_id": "C1"}})
    assert code == 409 and body["reason"] == "RUN_GATE:RUN_SWITCH_BLOCKED"
    assert "A-uav1" in lg.reservations()                                 # run 이 바뀌어도 점유를 지우지 않는다
    assert len([e for e in lg.events() if e["event_type"] == "RUN_GATE"]) == 1

    for _ in range(3):
        orch.poll()                                                      # 이전 실행은 계속 추적
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RETURNING"
    skipped = [e for e in lg.events(task.task_id) if e["event_type"] == "OBSERVATION_SKIPPED"]
    assert skipped and skipped[0]["reason"] == "RUN_MISMATCH"            # 다른 run 의 세계로 관측을 만들지 않음
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "HOLD" and t.hold_reason == "RUN_ENDED_BEFORE_OBSERVATION"
    assert [e for e in lg.events(task.task_id) if e["event_type"] in ("OBSERVATION", "TASK_COMPLETE")] == []

    orch.poll()                                                          # 착륙·READY 확인 → 반납
    assert lg.reservations() == {}
    v = orch.view()                                                      # 이제 전환 가능
    assert v.run_gate is None and lg.active_run() == "RUN-B"
    assert lg.list_tasks() == [] and orch.kb.fire_states(1e9) == {}
    assert len(uav.exec_calls()) == 1


def test_closed_run_is_not_reactivated_by_late_snapshot_or_event(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    orch.view()
    _switch(env, "RUN-B")
    orch.view()
    # 늦게 온 이전 run 의 이벤트 (현재 환경은 RUN-B)
    code, body = orch.ingest_event({"event_id": "EV-LATE", "type": "FIRE_REPORT", "run_id": "RUN-FIXTURE",
                                    "payload": {"cell_id": "C1"}})
    assert code == 409 and body["reason"] == "RUN_MISMATCH" and orch.kb.fire_states(1e9) == {}
    with pytest.raises(RunNotActive):
        submit(orch, "REQ-LATE", run_id="RUN-FIXTURE")
    # 늦게 온 이전 run 의 snapshot
    env.run_id = "RUN-FIXTURE"
    assert orch.view().run_gate == "RUN_CLOSED" and lg.active_run() == "RUN-B"
    with pytest.raises(RunClosed):
        lg.activate_run("RUN-FIXTURE")


def test_run_scope_and_version_order_survive_restart(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    env.reports = [dict(REPORT)]
    env.advance()
    orch.view()
    lg.close()
    lg2 = Ledger(world["db"])
    orch2 = _new_orch(world, lg2)
    assert lg2.active_run() == "RUN-FIXTURE"
    assert [c["cell_id"] for c in orch2.view().fire_cells] == ["C1"]     # 같은 run 이면 이어받는다
    assert len(lg2.list_tasks()) == 1
    env.state_version -= 1                                               # 같은 run 에서 버전 역행
    assert orch2.view().run_gate == "SNAPSHOT_STALE"
    assert orch2.dispatch(lg2.list_tasks()[0].task_id)["reason"] == "RUN_GATE:SNAPSHOT_STALE"
    assert world["uav"].exec_calls() == []


def test_ledger_run_activation_rules(tmp_path):
    lg = Ledger(str(tmp_path / "r.db"))
    assert lg.active_run() is None and lg.list_tasks() == []
    with pytest.raises(RunNotActive):
        lg.create_task(Task("T0", "I", "RECON", Target(38.0, 128.0)), {})
    assert lg.activate_run("A") == "ACTIVATED" and lg.activate_run("A") == "ALREADY_ACTIVE"
    lg.create_task(Task("T1", "I", "RECON", Target(38.0, 128.0), request_key="K"), {"a": 1})
    lg.prepare_attempt("ATT-1", "T1", "D1", "A-uav1", {"resource_type": "UAV"})
    with pytest.raises(RunSwitchBlocked):
        lg.activate_run("B")
    assert lg.active_run() == "A"
    lg.set_attempt_status("ATT-1", "RELEASED", release=True)
    assert lg.activate_run("B") == "ACTIVATED" and lg.run_status("A") == "CLOSED"
    t, created = lg.create_task(Task("T2", "I", "RECON", Target(38.0, 128.0), request_key="K"), {"a": 2})
    assert created and t.run_id == "B"                                   # 같은 키·다른 내용이어도 다른 run
    assert lg.note_state_version("B", 5) == "NEW" and lg.note_state_version("B", 5) == "DUPLICATE"
    assert lg.note_state_version("B", 4) == "STALE" and lg.note_state_version("B", 6) == "NEW"


# ---------------------------------------------------------------------------
# 이전 장부(run 범위 없음) 이관
# ---------------------------------------------------------------------------
LEGACY_SCHEMA = """
CREATE TABLE tasks (task_id TEXT PRIMARY KEY, incident_id TEXT NOT NULL, request_key TEXT UNIQUE, request_hash TEXT,
    purpose_status TEXT NOT NULL, body TEXT NOT NULL, created_wall REAL NOT NULL, updated_wall REAL NOT NULL);
CREATE TABLE decisions (decision_id TEXT PRIMARY KEY, task_id TEXT NOT NULL, parent_decision_id TEXT,
    created_wall REAL NOT NULL, body TEXT NOT NULL);
CREATE TABLE attempts (attempt_id TEXT PRIMARY KEY, task_id TEXT NOT NULL, decision_id TEXT NOT NULL,
    resource_id TEXT NOT NULL, command TEXT NOT NULL, command_hash TEXT NOT NULL, substatus TEXT NOT NULL,
    detail TEXT, created_wall REAL NOT NULL, updated_wall REAL NOT NULL);
CREATE TABLE reservations (resource_id TEXT PRIMARY KEY, task_id TEXT NOT NULL, attempt_id TEXT NOT NULL,
    created_wall REAL NOT NULL);
CREATE TABLE events (seq INTEGER PRIMARY KEY AUTOINCREMENT, wall_time REAL NOT NULL, sim_time_s REAL,
    event_type TEXT NOT NULL, task_id TEXT, decision_id TEXT, attempt_id TEXT, resource_id TEXT, result TEXT,
    reason TEXT, detail TEXT);
CREATE TABLE knowledge (seq INTEGER PRIMARY KEY AUTOINCREMENT, kind TEXT NOT NULL, key TEXT NOT NULL UNIQUE,
    subject TEXT, sim_time_s REAL, source TEXT, body TEXT NOT NULL, wall_time REAL NOT NULL);
"""


def _legacy_db(path):
    now = time.time()
    body = {"task_id": "TASK-old", "incident_id": "INC", "kind": "RECON",
            "target": {"lat": 38.05, "lon": 128.25, "ground_amsl_m": 600.0, "cell_id": "C1"},
            "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL", "needs_env_ack": True},
            "area_cell_ids": [], "purpose_status": "HOLD", "hold_reason": "EXECUTION_RESPONSE_LOST",
            "resume_condition": "MANUAL_RESOLUTION", "request_key": "REQ-1", "created_wall": now}
    c = sqlite3.connect(path)
    c.executescript(LEGACY_SCHEMA)
    c.execute("INSERT INTO tasks VALUES (?,?,?,?,?,?,?,?)",
              ("TASK-old", "INC", "REQ-1", "h", "HOLD", json.dumps(body), now, now))
    c.execute("INSERT INTO attempts VALUES (?,?,?,?,?,?,?,?,?,?)",
              ("ATT-old", "TASK-old", "DEC-old", "A-uav1", json.dumps({"resource_type": "UAV", "payload": {
                  "task_id": "ATT-old"}}), "h", "UNKNOWN", None, now, now))
    c.execute("INSERT INTO reservations VALUES (?,?,?,?)", ("A-uav1", "TASK-old", "ATT-old", now))
    c.execute("INSERT INTO knowledge (kind, key, subject, sim_time_s, source, body, wall_time) VALUES (?,?,?,?,?,?,?)",
              ("FIRE", "REPORT:old", "C1", 0.0, "119_CALL", json.dumps({"cell_id": "C1", "status": "REPORTED"}), now))
    c.commit()
    c.close()


def test_legacy_ledger_rows_are_quarantined_and_block_start_until_occupancy_is_resolved(world, tmp_path):
    path = str(tmp_path / "legacy.db")
    _legacy_db(path)
    lg = Ledger(path)
    assert lg.migrated_from_legacy and lg.run_status(LEGACY_RUN) == "QUARANTINED" and lg.active_run() is None
    orch = _new_orch(world, lg)

    # 시작 조건: 범위를 알 수 없는 열린 실행의 점유가 풀리기 전에는 새 run 을 시작하지 않는다
    assert orch.view().run_gate == "RUN_SWITCH_BLOCKED" and lg.active_run() is None
    assert lg.reservations()["A-uav1"]["attempt_id"] == "ATT-old"       # 점유 보존
    assert lg.get_attempt("ATT-old")["run_id"] == LEGACY_RUN
    orch.poll()                                                          # 제공자는 모르는 시도 → 불명 유지
    assert lg.get_attempt("ATT-old")["substatus"] == "UNKNOWN"

    r = orch.resolve("TASK-old", "CONFIRM_RESOURCE_IDLE", "관제가 기체 정지 확인", attempt_id="ATT-old")
    assert r["status"] == "RELEASED" and lg.reservations() == {}
    v = orch.view()
    assert v.run_gate is None and lg.active_run() == "RUN-FIXTURE"
    # 이전 행은 새 run 에 귀속되지 않는다
    assert v.fire_cells == [] and lg.list_tasks() == [] and lg.knowledge("FIRE") == []
    assert [t.task_id for t in lg.list_tasks(run_id=LEGACY_RUN)] == ["TASK-old"]
    assert len(lg.knowledge("FIRE", run_id=LEGACY_RUN)) == 1
    assert len(lg.list_tasks(run_id=ALL_RUNS)) == 1
    fresh = submit(orch, "REQ-1")                                        # 이전 장부와 같은 요청 키도 쓸 수 있다
    assert fresh.run_id == "RUN-FIXTURE" and fresh.task_id != "TASK-old"
    assert orch.dispatch("TASK-old")["reason"] == "TASK_RUN_NOT_ACTIVE"
    Ledger(path).close()                                                 # 다시 열어도 이관을 반복하지 않는다
