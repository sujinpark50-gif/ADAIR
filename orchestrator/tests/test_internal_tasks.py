# -*- coding: utf-8 -*-
"""총괄 내부 과제 (팀별 요청서 '총괄 내부 과제', 2026-10-01).

1 불 발견 뒤 자동 측정 임무를 관측과 함께 장부에 남기고, 멈췄다 켜져도 같은 요청 ID 로 한 번만 만든다.
2 기체 평가·출동 요청이 느려도 추적(poll)·취소·조회가 기다리지 않는다 (배정은 공유 잠금 밖에서 HTTP 호출).
  보내는 중인 실행시도는 추적이 건드리지 않고, 평가 도중 취소된 임무는 보내지 않는다.
3 UGV 는 평가에서 고른 도로 노드(target_node)로 실행한다.
"""

import threading
import time

import pytest
from fastapi.testclient import TestClient

from orchestrator.api import create_app
from orchestrator.engine import Orchestrator
from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis
from orchestrator.ledger import Ledger
from orchestrator.resources import UavClient, UgvClient

from .conftest import TARGET, submit
from .test_env_sense import _setup
from .test_flow import UGV_REQ


class Crash(BaseException):
    """총괄 프로세스 강제 종료"""


def _ev(lg, etype, task_id=None):
    return [e for e in lg.events(task_id) if e["event_type"] == etype]


def _restart(world):
    world["ledger"].close()
    lg = Ledger(world["db"])
    uav, env = world["uav"], world["env"]
    orch = Orchestrator(lg, env, UavClient(uav.endpoints(), uav.client()),
                        UgvClient("http://ugv", world["ugv"].client()), analysis=world["analysis"],
                        weather_feeds=[EnvStationFeed(env, TARGET["lat"], TARGET["lon"])])
    world.update(orch=orch, ledger=lg)
    return orch, lg


def _sense_tasks(lg):
    return [t for t in lg.list_tasks() if t.kind == "ENV_SENSE"]


# ---------------------------------------------------------------------------
# 1 자동 후속 측정 임무 복구
# ---------------------------------------------------------------------------
def test_auto_sense_followup_survives_stop_right_after_observation(world):
    orch, lg = world["orch"], world["ledger"]
    _setup(world)                                                  # 서풍 → 동쪽(EAST) 칸이 바람 방향
    task = submit(orch)
    orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()
    real = orch.run_followups

    def crash():
        orch.run_followups = real
        raise Crash()
    orch.run_followups = crash                                     # 관측 저장 직후, 후속 임무 만들기 전에 종료
    with pytest.raises(Crash):
        orch.poll()
    assert _sense_tasks(lg) == []
    assert [f["status"] for f in lg.followups(task.task_id)] == ["PENDING"]    # 해야 할 일은 장부에 남아 있다
    orch, lg = _restart(world)
    assert orch.recover()["followups_resumed"][0]["created"]
    sense = _sense_tasks(lg)
    assert [t.target.cell_id for t in sense] == ["EAST"]
    assert sense[0].request_key == f"AUTO-SENSE:{lg.active_run()}:EAST"
    assert [f["status"] for f in lg.followups(task.task_id)] == ["DONE"]
    assert len(_ev(lg, "ENV_SENSE_PLANNED")) == 1
    orch.poll()
    orch.recover()
    assert len(_sense_tasks(lg)) == 1 and len(_ev(lg, "ENV_SENSE_PLANNED")) == 1


def test_followup_interrupted_after_task_creation_is_not_duplicated(world):
    orch, lg = world["orch"], world["ledger"]
    _setup(world)
    task = submit(orch)
    orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()
    real = lg.finish_followup

    def crash(*a, **k):
        lg.finish_followup = real
        raise Crash()
    lg.finish_followup = crash                                     # 임무는 만들었고 '끝남' 표시 전에 종료
    with pytest.raises(Crash):
        orch.poll()
    assert len(_sense_tasks(lg)) == 1 and _ev(lg, "ENV_SENSE_PLANNED") == []  # 계획 기록도 함께 되돌아감
    orch, lg = _restart(world)
    orch.recover()
    assert len(_sense_tasks(lg)) == 1                              # 같은 요청 ID → 새로 만들지 않음
    assert len(_ev(lg, "ENV_SENSE_PLANNED")) == 1
    assert [f["status"] for f in lg.followups(task.task_id)] == ["DONE"]


def test_normal_flow_creates_followup_once_and_marks_done(world):
    orch, lg = world["orch"], world["ledger"]
    _setup(world)
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    assert len(_sense_tasks(lg)) == 1 and [f["status"] for f in lg.followups(task.task_id)] == ["DONE"]


# ---------------------------------------------------------------------------
# 2 느린 평가·출동 요청 동안 추적·취소
# ---------------------------------------------------------------------------
def _during(fn, timeout=3.0):
    box = {}
    th = threading.Thread(target=lambda: box.update(result=fn()), daemon=True)
    t0 = time.monotonic()
    th.start()
    th.join(timeout)
    return (not th.is_alive()), box.get("result"), time.monotonic() - t0


def test_slow_evaluate_does_not_block_tracking_of_other_drone_or_cancel(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    first = orch.dispatch(submit(orch, "REQ-1").task_id)          # A-uav1 비행 중
    entered, release = threading.Event(), threading.Event()

    def slow(body):
        entered.set()
        assert release.wait(10)
        return {"verdict": "ACCEPT", "eta_sec": 600}
    uav.verdicts["A-uav2"] = slow
    app = create_app(orch)
    body = {"request_id": "REQ-2", "target": TARGET, "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL"}}
    box = {}
    th = threading.Thread(target=lambda: box.update(r=TestClient(app).post("/tasks", json=body).json()), daemon=True)
    th.start()
    assert entered.wait(5)                                         # A-uav2 평가 요청이 응답을 안 줌

    def side():
        c = TestClient(app)
        polls = [c.post("/poll").status_code for _ in range(2)]
        t2 = lg.task_by_request_key("REQ-2")
        cancel = c.post(f"/tasks/{t2.task_id}/resolve", json={"action": "CANCEL", "reason": "오인 신고"}).json()
        return polls, cancel, c.get("/state").status_code
    finished, res, elapsed = _during(side)
    assert finished, "느린 평가 요청 동안 추적·취소가 공유 잠금에 막혔다"
    polls, cancel, state = res
    assert polls == [200, 200] and state == 200 and cancel["status"] == "CANCELLED"
    assert lg.get_attempt(first["attempt_id"])["substatus"] == "ARRIVED"       # 다른 기체 추적은 계속 진행
    release.set()
    th.join(5)
    t2 = lg.task_by_request_key("REQ-2")
    assert lg.get_task(t2.task_id).purpose_status == "CANCELLED"
    assert [c for c in uav.exec_calls() if c[1] == "A-uav2"] == []            # 평가 도중 취소된 임무는 보내지 않음
    assert "A-uav2" not in lg.reservations()


def test_slow_execute_is_not_misjudged_by_concurrent_tracking(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.state["A-uav2"]["failsafe"] = True
    entered, release = threading.Event(), threading.Event()
    real = orch.uav.execute

    def slow_execute(rid, payload):
        entered.set()
        assert release.wait(10)
        return real(rid, payload)
    orch.uav.execute = slow_execute
    app = create_app(orch)
    task = submit(orch)
    box = {}
    th = threading.Thread(target=lambda: box.update(r=TestClient(app).post(f"/tasks/{task.task_id}/dispatch").json()),
                          daemon=True)
    th.start()
    assert entered.wait(5)                                         # 예약·선저장(PREPARED) 뒤 송신 중
    att = lg.list_attempts(task.task_id)[0]
    assert att["substatus"] == "PREPARED"
    finished, _, _ = _during(lambda: [TestClient(app).post("/poll").status_code for _ in range(3)])
    assert finished
    # 제공자는 아직 이 시도를 모른다(404) — 그래도 보내는 중이므로 불명으로 판정하지 않는다
    assert lg.get_attempt(att["attempt_id"])["substatus"] == "PREPARED"
    assert _ev(lg, "EXECUTION_UNKNOWN", task.task_id) == []
    release.set()
    th.join(5)
    assert box["r"]["status"] == "STARTED" and lg.get_attempt(att["attempt_id"])["substatus"] == "STARTED"
    for _ in range(4):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED" and len(uav.exec_calls()) == 1


def test_concurrent_dispatchers_and_trackers_never_double_execute(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.state["A-uav2"]["failsafe"] = True
    submit(orch, "REQ-1")
    submit(orch, "REQ-2")
    app = create_app(orch)
    calls = [lambda: TestClient(app).post("/dispatch_pending"), lambda: TestClient(app).post("/poll")] * 4
    threads = [threading.Thread(target=f, daemon=True) for f in calls]
    for t in threads:
        t.start()
    for t in threads:
        t.join(10)
    assert len(uav.exec_calls()) == 1 and len(lg.reservations()) == 1          # 드론 1대 → 한 번만 출동


# ---------------------------------------------------------------------------
# 3 UGV 목적지 고정
# ---------------------------------------------------------------------------
def test_ugv_execute_carries_target_node_chosen_in_evaluation(world):
    orch, ugv, lg = world["orch"], world["ugv"], world["ledger"]
    task = submit(orch, requirements=UGV_REQ)
    out = orch.dispatch(task.task_id)
    assert out["status"] == "STARTED"
    execs = [c for c in ugv.calls if c[1].endswith("/execute")]
    assert execs[0][2]["target_node"] == "N1"                      # 평가 응답의 target_node.node_id
    evals = [c for c in ugv.calls if c[1].endswith("/evaluate")]
    assert "target_node" not in evals[0][2]                        # 평가 요청에는 좌표만 (노드는 UGV 가 고름)
    safety = _ev(lg, "SAFETY_JUDGEMENT", task.task_id)[0]
    assert safety["result"] == "ALLOW" and "same_mission" in safety["detail"]["checked"]
    assert lg.get_attempt(out["attempt_id"])["command"]["payload"]["target_node"] == "N1"


def test_ugv_without_target_node_in_evaluation_is_sent_with_coordinates_only(world):
    orch, ugv = world["orch"], world["ugv"]
    ugv.verdicts["A-ugv1"] = {"verdict": "ACCEPT", "eta_sec": 300, "target_node": None}
    task = submit(orch, requirements=UGV_REQ)
    assert orch.dispatch(task.task_id)["status"] == "STARTED"
    execs = [c for c in ugv.calls if c[1].endswith("/execute")]
    assert "target_node" not in execs[0][2] and execs[0][2]["target"]["lat"] == TARGET["lat"]
