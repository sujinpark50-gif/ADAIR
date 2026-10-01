# -*- coding: utf-8 -*-
"""LLM 추천 재사용 키와 긴 LLM 호출 동안의 잠금 (검토 2026-09-30 §4 추가 확인).

- 재사용 키는 LLM 에 실제로 주는 입력의 해시다. 환경 state_version 이 그대로여도 위험점수·보호대상 거리가
  바뀌면 옛 추천을 다시 쓰지 않는다.
- 서버는 긴 LLM 호출을 공통 잠금 밖에서 한다. 그 사이 추적(poll)·취소·조회가 진행되고, 돌아온 추천은
  배정 직전의 입력(run·버전·묶음·근거)과 같을 때만 쓴다. 출동(execute)은 계속 잠금 안에서 한 번만 한다.
"""

import threading

from fastapi.testclient import TestClient

from orchestrator import config, priority
from orchestrator.api import create_app

from .test_llm import _world_with_llm, prefer_lower_risk
from .test_priority import cell, square_site


def _recommend(orch):
    snap = orch.view()
    tasks = orch.ledger.list_tasks(["PENDING", "HOLD"])
    plan = priority.order_tasks(tasks, snap, config.PRIORITY_SIMILAR_RISK_DELTA)
    return orch._llm_recommend(snap, plan, {t.task_id: t for t in tasks})


def test_recommendation_is_not_reused_when_evidence_changes_without_env_version_change(world):
    fake, ta, tb = _world_with_llm(world, prefer_lower_risk, mode="HUMAN_CHOICE")
    orch, env = world["orch"], world["env"]
    assert _recommend(orch) and _recommend(orch) and len(fake.requests) == 1     # 같은 입력 → 재사용
    version = env.state_version

    world["analysis"].risk_cells = [cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.70)]   # 위험점수만 바뀜
    assert _recommend(orch) and len(fake.requests) == 2 and env.state_version == version
    assert _recommend(orch) and len(fake.requests) == 2

    env.protected_sites = [square_site("V1", 0, 900, 50)]                         # 보호대상 거리가 생김
    assert _recommend(orch) and len(fake.requests) == 3 and env.state_version == version
    reused = [e for e in world["ledger"].events() if e["event_type"] == "LLM_PLAN_REUSED"]
    assert len(reused) == 2 and all(e["detail"]["input_hash"] for e in reused)


class SlowLlm:
    """LLM 호출이 끝나지 않고 기다리게 한다 (release 가 풀릴 때까지)."""

    def __init__(self):
        self.entered, self.release = threading.Event(), threading.Event()

    def __call__(self, kw):
        self.entered.set()
        assert self.release.wait(10), "시험이 LLM 을 풀어주지 않았다"
        return prefer_lower_risk(kw)


def _start_dispatch(app, box):
    th = threading.Thread(target=lambda: box.update(out=TestClient(app).post("/dispatch_pending").json()))
    th.start()
    return th


def _during(fn, timeout=3.0):
    """fn 을 다른 스레드에서 실행하고 timeout 안에 끝났는지 돌려준다"""
    box = {}
    th = threading.Thread(target=lambda: box.update(result=fn()))
    th.start()
    th.join(timeout)
    return not th.is_alive(), box.get("result")


def test_tracking_and_cancel_proceed_during_slow_llm_call_and_cancelled_task_is_not_dispatched(world):
    slow = SlowLlm()
    fake, ta, tb = _world_with_llm(world, slow)                  # 자동 모드, 드론 1대, 비슷한 두 임무
    orch, lg, uav = world["orch"], world["ledger"], world["uav"]
    app, box = create_app(orch), {}
    th = _start_dispatch(app, box)
    assert slow.entered.wait(5)

    def side():
        c = TestClient(app)
        return {"poll": c.post("/poll").status_code, "state": c.get("/state").status_code,
                "cancel": c.post(f"/tasks/{tb.task_id}/resolve", json={"action": "CANCEL", "reason": "오인"}).json()}
    finished, res = _during(side)
    assert finished, "LLM 호출 동안 추적·취소가 잠금에 막혔다"
    assert res["poll"] == 200 and res["state"] == 200 and res["cancel"]["status"] == "CANCELLED"
    assert th.is_alive() and uav.exec_calls() == []              # 그동안 LLM 은 아직 응답 전, 출동도 없음

    slow.release.set()
    th.join(5)
    out = box["out"]
    # 추천(AI 는 B 먼저)은 B 가 있던 때의 입력에 대한 것 → 취소된 B 는 다시 검증에서 빠지고 출동하지 않는다
    assert tb.task_id not in out["results"] and out["groups"] == []
    assert out["results"][ta.task_id]["status"] == "STARTED" and len(uav.exec_calls()) == 1
    assert lg.get_task(tb.task_id).purpose_status == "CANCELLED"


def test_llm_result_for_outdated_input_is_not_applied(world):
    slow = SlowLlm()
    fake, ta, tb = _world_with_llm(world, slow)
    orch, lg, uav = world["orch"], world["ledger"], world["uav"]
    app, box = create_app(orch), {}
    th = _start_dispatch(app, box)
    assert slow.entered.wait(5)
    world["analysis"].risk_cells = [cell("A", 0, 0, 0.72), cell("B", 0, 500, 0.70)]   # 호출 중에 근거가 바뀜
    slow.release.set()
    th.join(5)
    g = box["out"]["groups"][0]
    assert g["llm"]["status"] == "STALE" and g["llm"]["fallback"] == "RULE_ORDER" and g["status"] == "AUTO_DECIDED"
    assert box["out"]["results"][ta.task_id]["status"] == "STARTED"              # 규칙 순서(A), AI 의 B 가 아님
    assert [e["reason"] for e in lg.events() if e["event_type"] == "LLM_PLAN_NOT_APPLIED"] == [
        "NO_PLAN_FOR_CURRENT_INPUT"]
    assert len(uav.exec_calls()) == 1


def test_llm_result_for_unchanged_input_is_applied_after_out_of_lock_call(world):
    slow = SlowLlm()
    fake, ta, tb = _world_with_llm(world, slow)
    app, box = create_app(world["orch"]), {}
    th = _start_dispatch(app, box)
    assert slow.entered.wait(5)
    finished, code = _during(lambda: TestClient(app).post("/poll").status_code)
    assert finished and code == 200
    slow.release.set()
    th.join(5)
    g = box["out"]["groups"][0]
    assert g["status"] == "LLM_DECIDED" and g["llm"]["order"] == [tb.task_id, ta.task_id]
    assert box["out"]["results"][tb.task_id]["status"] == "STARTED" and len(world["uav"].exec_calls()) == 1
    assert len(fake.requests) == 1


def test_concurrent_dispatch_requests_do_not_double_execute(world):
    fake, ta, tb = _world_with_llm(world, prefer_lower_risk)
    app, uav = create_app(world["orch"]), world["uav"]
    threads = [threading.Thread(target=lambda: TestClient(app).post("/dispatch_pending")) for _ in range(6)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(10)
    assert len(uav.exec_calls()) == 1                            # 드론 1대 → 한 번만 출동
    assert len(world["ledger"].reservations()) == 1
