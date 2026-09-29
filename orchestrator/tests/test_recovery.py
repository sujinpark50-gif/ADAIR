# -*- coding: utf-8 -*-
"""PREPARED 실행시도 재시작 복구 (피드백 2026-09-30 §1).

프로세스 종료는 Crash(BaseException) 로 흉내 낸다 — 엔진의 일반 예외 처리에 잡히지 않는다.
제공자(UAV)에 멱등키 계약이 없으므로 '물리 명령 최대 1회'를 보장한다고 주장하지 않는다.
여기서 확인하는 것은 총괄이 재송신·재출동하지 않는다는 것이다.
"""

import pytest

from orchestrator.engine import Orchestrator
from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis
from orchestrator.ledger import Ledger
from orchestrator.resources import UavClient, UgvClient

from .conftest import TARGET, submit


class Crash(BaseException):
    """총괄 프로세스 강제 종료"""


def _restart(world):
    world["ledger"].close()
    lg = Ledger(world["db"])
    uav, env = world["uav"], world["env"]
    orch = Orchestrator(lg, env, UavClient(uav.endpoints(), uav.client()),
                        UgvClient("http://ugv", world["ugv"].client()), analysis=InjectedAnalysis(),
                        weather_feeds=[EnvStationFeed(env, TARGET["lat"], TARGET["lon"])])
    return orch, lg


def _only_uav1(world):
    world["uav"].state["A-uav2"]["failsafe"] = True


def _crash_dispatch(world, where):
    orch, uav = world["orch"], world["uav"]
    real_execute = orch.uav.execute

    if where == "before_send":
        def execute(rid, payload):
            raise Crash()
        orch.uav.execute = execute
    elif where == "after_send":
        def execute(rid, payload):
            real_execute(rid, payload)             # 제공자는 받았다
            raise Crash()                          # 응답을 받기 전에 총괄 종료
        orch.uav.execute = execute
    elif where == "after_response":
        real_set = world["ledger"].set_attempt_status

        def set_status(aid, sub, *a, **k):
            if sub == "STARTED":
                raise Crash()                      # ACK 는 받았지만 장부 갱신 전 종료
            return real_set(aid, sub, *a, **k)
        world["ledger"].set_attempt_status = set_status
    task = submit(orch)
    with pytest.raises(Crash):
        orch.dispatch(task.task_id)
    att = world["ledger"].list_attempts(task.task_id)[0]
    assert att["substatus"] == "PREPARED"
    return task, att


def _recovery_events(lg, task_id):
    return [e for e in lg.events(task_id) if e["event_type"] == "RECOVERY_PREPARED"]


def test_crash_before_send_becomes_unknown_and_keeps_occupancy(world):
    _only_uav1(world)
    task, att = _crash_dispatch(world, "before_send")
    assert world["uav"].exec_calls() == []
    orch, lg = _restart(world)
    orch.recover()
    a = lg.get_attempt(att["attempt_id"])
    assert a["substatus"] == "UNKNOWN"                       # 송신 여부를 증명할 수 없음
    assert lg.reservations()["A-uav1"]["attempt_id"] == att["attempt_id"]
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "HOLD" and t.resume_condition == "MANUAL_RESOLUTION"
    ev = _recovery_events(lg, task.task_id)[0]
    assert ev["detail"]["provider_task_id"] == att["attempt_id"] and ev["detail"]["task_id"] == task.task_id
    assert ev["detail"]["provider_status"] == "NOT_FOUND" and ev["result"] == "UNKNOWN"
    orch.dispatch_pending()
    orch.poll()
    assert world["uav"].exec_calls() == []                   # 자동 재송신·재출동 없음


@pytest.mark.parametrize("where", ["after_send", "after_response"])
def test_crash_after_send_resumes_tracking_from_provider(world, where):
    _only_uav1(world)
    task, att = _crash_dispatch(world, where)
    assert len(world["uav"].exec_calls()) == 1
    orch, lg = _restart(world)
    orch.recover()
    a = lg.get_attempt(att["attempt_id"])
    assert a["substatus"] in ("STARTED", "ARRIVED")           # 제공자가 같은 시도를 보고 → 추적 재개
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"
    assert _recovery_events(lg, task.task_id)[0]["result"] == "RESUMED"
    for _ in range(4):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert len(world["uav"].exec_calls()) == 1                # 중복 execute 없음


def test_provider_unreachable_at_restart_is_unknown_then_resumes_when_reachable(world):
    _only_uav1(world)
    task, att = _crash_dispatch(world, "after_send")
    world["uav"].unreachable.add("A-uav1")
    orch, lg = _restart(world)
    orch.recover()
    assert lg.get_attempt(att["attempt_id"])["substatus"] == "UNKNOWN"
    assert _recovery_events(lg, task.task_id)[0]["detail"]["provider_status"] == "UNREACHABLE"
    world["uav"].unreachable.clear()
    orch.poll()                                               # 제공자가 같은 시도를 보고
    assert lg.get_attempt(att["attempt_id"])["substatus"] in ("STARTED", "ARRIVED")
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"
    assert len(world["uav"].exec_calls()) == 1


def test_poll_also_recovers_prepared(world):
    """recover() 를 부르지 않아도 poll 이 PREPARED 를 놓치지 않는다"""
    _only_uav1(world)
    task, att = _crash_dispatch(world, "before_send")
    orch, lg = _restart(world)
    orch.poll()
    assert lg.get_attempt(att["attempt_id"])["substatus"] == "UNKNOWN"
