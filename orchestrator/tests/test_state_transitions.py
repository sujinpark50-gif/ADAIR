# -*- coding: utf-8 -*-
"""목적 상태와 실행시도 진행의 분리 (검토 2026-09-30 B01·B02·B03).

- B01 취소된 목적은 늦게 온 도착·관측·완료 응답, idle 확인, 재시도로 되살아나지 않는다.
       취소해도 실행시도의 추적·점유는 기체 READY 확인까지 이어진다.
- B02 완료·취소된 Task 의 RETRY 는 거절한다. 임무가 열린 시도가 있으면 새 시도를 만들지 않는다.
       끝난 대상의 재관측은 이전 Task 와 연결된 새 Task(RECHECK)다.
- B03 상태 조회 응답은 실행 식별자·자원 ID·상태 값을 검증한 뒤에만 쓴다.
"""

import pytest

from orchestrator.ledger import AttemptConflict, IllegalTransition

from .conftest import submit
from .test_flow import UGV_REQ


def _events(lg, task_id, etype):
    return [e for e in lg.events(task_id) if e["event_type"] == etype]


def _fly_to_release(orch, n=4):
    for _ in range(n):
        orch.poll()


# ---------------------------------------------------------------------------
# B01 취소
# ---------------------------------------------------------------------------
def test_cancelled_purpose_survives_arrival_observation_return_and_ready(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    r = orch.resolve(task.task_id, "CANCEL", "오인 신고로 확인")
    assert r["status"] == "CANCELLED" and r["stop_command_sent"] is False
    assert r["resources_still_occupied"] == ["A-uav1"]           # 취소만으로 반납하지 않는다

    for expected in ("STARTED", "ARRIVED", "RETURNING"):         # 이동 → 도착 → 관측·복귀
        orch.poll()
        assert lg.get_task(task.task_id).purpose_status == "CANCELLED"
        assert lg.get_attempt(out["attempt_id"])["substatus"] == expected
        assert "A-uav1" in lg.reservations()                     # READY 확인 전에는 점유 유지

    # 늦게 온 관측: 출처·'취소 이후 수신'과 함께 보관하되 완료 근거로 쓰지 않는다
    obs = _events(lg, task.task_id, "OBSERVATION")[0]["detail"]
    assert obs["received_after_purpose"] == "CANCELLED" and obs["used_for_completion"] is False
    assert obs["source"] == "SIMULATED" and obs["result"] == "DETECTED"
    assert _events(lg, task.task_id, "TASK_COMPLETE") == []
    assert _events(lg, task.task_id, "ENVIRONMENT_APPLY") == []
    assert _events(lg, task.task_id, "OBSERVATION_AFTER_CLOSE")[0]["reason"] == "NOT_USED_FOR_COMPLETION"
    fact = orch.kb.fire_states(1e9)["C1"]
    assert fact["status"] == "CONFIRMED" and fact["received_after_purpose"] == "CANCELLED"

    orch.poll()                                                  # 착륙·READY 확인 → 그때 반납
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED" and lg.reservations() == {}
    assert lg.get_task(task.task_id).purpose_status == "CANCELLED"
    assert len(uav.exec_calls()) == 1


def test_cancel_is_not_revived_by_idle_confirmation_retry_or_dispatch(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.execute_mode["A-uav1"] = "timeout"                       # 실행 결과 불명 → 수동 해소 대기
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    assert orch.resolve(task.task_id, "CANCEL", "취소")["status"] == "CANCELLED"
    assert "A-uav1" in lg.reservations()                         # 불명 시도의 점유는 그대로

    r = orch.resolve(task.task_id, "CONFIRM_RESOURCE_IDLE", "관제가 정지 확인", attempt_id=out["attempt_id"])
    assert r == {"status": "RELEASED", "purpose_status": "CANCELLED"} and lg.reservations() == {}
    assert orch.resolve(task.task_id, "RETRY", "다시")["reason"] == "TASK_ALREADY_CANCELLED"
    assert orch.dispatch(task.task_id)["status"] == "SKIPPED"
    assert orch.manual_dispatch(task.task_id, "보내기")["status"] == "REFUSED"
    assert orch.resolve(task.task_id, "CANCEL", "또 취소")["reason"] == "TASK_ALREADY_CANCELLED"
    orch.dispatch_pending()
    assert lg.get_task(task.task_id).purpose_status == "CANCELLED" and len(uav.exec_calls()) == 1


def test_ground_arrival_after_cancel_does_not_complete(world):
    orch, ugv, lg = world["orch"], world["ugv"], world["ledger"]
    ugv.default_script = lambda rid: [
        {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
         "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1"},
         "_unit": {"state": "READY", "current_task_id": None}}]
    task = submit(orch, requirements=UGV_REQ)
    out = orch.dispatch(task.task_id)
    orch.resolve(task.task_id, "CANCEL", "취소")
    for _ in range(2):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "CANCELLED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"      # 물리 추적·반납은 끝까지


def test_ledger_boundary_rejects_reviving_a_closed_purpose(world):
    orch, lg = world["orch"], world["ledger"]
    task = submit(orch)
    orch.resolve(task.task_id, "CANCEL", "취소")
    stale = lg.get_task(task.task_id)
    for status in ("PENDING", "IN_EXECUTION", "COMPLETED"):
        stale.purpose_status = status
        with pytest.raises(IllegalTransition):
            lg.save_task(stale)
    assert lg.get_task(task.task_id).purpose_status == "CANCELLED"
    # 엔진 경계는 예외 대신 거절 사유를 기록한다
    assert orch._reopen(lg.get_task(task.task_id), "OBSERVATION_FAILED:X") is False
    rej = _events(lg, task.task_id, "PURPOSE_TRANSITION_REJECTED")[-1]
    assert rej["result"] == "CANCELLED->PENDING" and rej["reason"] == "OBSERVATION_FAILED:X"


# ---------------------------------------------------------------------------
# B02 RETRY·재관측
# ---------------------------------------------------------------------------
def test_retry_of_completed_task_is_refused_and_does_not_redispatch(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task = submit(orch)
    orch.dispatch(task.task_id)
    _fly_to_release(orch)
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED" and lg.reservations() == {}
    r = orch.resolve(task.task_id, "RETRY", "다시 보고 싶음")
    assert r["status"] == "REFUSED" and r["reason"] == "TASK_ALREADY_COMPLETED"
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED" and len(uav.exec_calls()) == 1
    assert _events(lg, task.task_id, "MANUAL_RESOLUTION_REFUSED")[0]["reason"] == "TASK_ALREADY_COMPLETED"


def test_retry_while_attempt_is_open_does_not_create_second_dispatch(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task = submit(orch)
    out = orch.dispatch(task.task_id)                            # STARTED (이동 중)
    r = orch.resolve(task.task_id, "RETRY", "재촉")
    assert r["status"] == "REFUSED" and r["reason"] == "ATTEMPT_STILL_OPEN"
    assert orch.dispatch(task.task_id)["status"] == "SKIPPED"
    assert len(uav.exec_calls()) == 1
    # 장부 경계: 같은 Task 에 임무가 열린 시도가 있으면 다른 자원으로도 시도를 만들 수 없다
    with pytest.raises(AttemptConflict):
        lg.prepare_attempt("ATT-second", task.task_id, "DEC-x", "A-uav2", {"resource_type": "UAV", "payload": {}})
    assert set(lg.reservations()) == {"A-uav1"} and lg.get_attempt(out["attempt_id"])["substatus"] == "STARTED"


def test_retry_after_failure_is_allowed_and_uses_another_resource(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.default_script = lambda rid, body: [
        {"status": "FAILED", "reason": "RETURN_MARGIN_INSUFFICIENT", "progress": {"phase": "RETURNING"}}]
    task = submit(orch)
    first = orch.dispatch(task.task_id)
    orch.poll()                                                   # 실패 확정 → 임무 부분 종료, 기체는 복귀 중
    assert lg.get_attempt(first["attempt_id"])["substatus"] == "RETURNING"
    r = orch.resolve(task.task_id, "RETRY", "후임 배정")
    assert r["status"] == "STARTED" and r["resource_id"] == "A-uav2"
    assert set(lg.reservations()) == {"A-uav1", "A-uav2"}        # 복귀 중인 기체의 점유도 유지


def test_recheck_creates_linked_task_and_leaves_parent_closed(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task = submit(orch)
    assert orch.resolve(task.task_id, "RECHECK", "아직 안 끝남")["reason"] == "PARENT_NOT_CLOSED"
    orch.dispatch(task.task_id)
    _fly_to_release(orch)
    r = orch.resolve(task.task_id, "RECHECK", "재발화 의심", request_id="OP-7")
    assert r["status"] == "CREATED" and r["parent_task_id"] == task.task_id
    again = orch.resolve(task.task_id, "RECHECK", "재발화 의심", request_id="OP-7")
    assert again["status"] == "DUPLICATE" and again["task_id"] == r["task_id"]   # 같은 요청은 하나로 수렴
    child = lg.get_task(r["task_id"])
    assert child.kind == "RECHECK" and child.parent_task_id == task.task_id
    assert child.target.cell_id == "C1" and child.completion_rule == "TARGET_POINT"
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"               # 이전 임무는 그대로
    out = orch.dispatch(child.task_id)
    assert out["status"] == "STARTED" and len(uav.exec_calls()) == 2
    _fly_to_release(orch)
    assert lg.get_task(child.task_id).purpose_status == "COMPLETED"


# ---------------------------------------------------------------------------
# B03 상태 조회 응답 검증
# ---------------------------------------------------------------------------
GOOD_OBS = {"position": {"lat": 38.05, "lon": 128.25, "alt_m_amsl": 690.0}, "sensor_type": "THERMAL", "values": {}}
DONE = {"status": "COMPLETED", "progress": {"phase": "DONE"}, "observation": GOOD_OBS}

UAV_BAD = {
    "OTHER_EXECUTION": ({**DONE, "task_id": "ATT-someone-else"}, "EXECUTION_ID_MISMATCH"),
    "TASK_ID_INSTEAD_OF_ATTEMPT": (lambda task: {**DONE, "task_id": task.task_id}, "EXECUTION_ID_MISMATCH"),
    "ID_MISSING": ({**DONE, "task_id": None}, "EXECUTION_ID_MISSING"),
    "OTHER_RESOURCE": ({**DONE, "uav_id": "A-uav2"}, "RESOURCE_ID_MISMATCH"),
    "BAD_STATUS": ({"status": "FINISHED", "progress": {"phase": "DONE"}}, "STATUS_INVALID"),
    "STATUS_MISSING": ({"status": None}, "STATUS_INVALID"),
    "PROGRESS_NOT_OBJECT": ({"status": "COMPLETED", "progress": "DONE"}, "PROGRESS_INVALID"),
}


@pytest.mark.parametrize("case", sorted(UAV_BAD))
def test_invalid_uav_status_response_is_not_applied_and_later_valid_response_recovers(world, case):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    bad, why = UAV_BAD[case]
    task = submit(orch)
    bad = bad(task) if callable(bad) else bad
    uav.default_script = lambda rid, body: [
        dict(bad), dict(bad),
        {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
        {"status": "COMPLETED", "progress": {"phase": "RETURNING"},
         "observation": {**GOOD_OBS, "position": {**GOOD_OBS["position"], "alt_m_amsl": 690.0}}},
        DONE]
    out = orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()                                                   # 같은 잘못된 응답이 또 와도
    att = lg.get_attempt(out["attempt_id"])
    assert att["substatus"] == "UNKNOWN" and att["detail"]["unknown_reason"] == f"RESPONSE_VALIDATION_FAILED:{why}"
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "HOLD" and t.resume_condition == "MANUAL_RESOLUTION"
    assert "A-uav1" in lg.reservations()                          # 점유 유지, 반납 없음
    for etype in ("OBSERVATION", "ENVIRONMENT_APPLY", "TASK_COMPLETE"):
        assert _events(lg, task.task_id, etype) == []
    rejected = _events(lg, task.task_id, "PROVIDER_RESPONSE_REJECTED")
    assert len(rejected) == 1 and rejected[0]["reason"] == why and rejected[0]["detail"]["applied"] is False

    orch.poll()                                                   # 이후 정상 응답 → 추적 재개
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "STARTED"
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"
    orch.poll()
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED" and len(uav.exec_calls()) == 1


@pytest.mark.parametrize("override,why", [({"resource_id": "A-ugv9"}, "RESOURCE_ID_MISMATCH"),
                                          ({"resource_id": None}, "RESOURCE_ID_MISSING"),
                                          ({"task_id": "ATT-other"}, "EXECUTION_ID_MISMATCH")])
def test_invalid_ugv_status_response_is_not_applied(world, override, why):
    orch, ugv, lg = world["orch"], world["ugv"], world["ledger"]
    arrived = {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
               "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1"}}
    ugv.default_script = lambda rid: [{**arrived, **override},
                                      {**arrived, "_unit": {"state": "READY", "current_task_id": None}}]
    task = submit(orch, requirements=UGV_REQ)
    out = orch.dispatch(task.task_id)
    orch.poll()
    assert lg.get_attempt(out["attempt_id"])["detail"]["unknown_reason"] == f"RESPONSE_VALIDATION_FAILED:{why}"
    assert lg.get_task(task.task_id).purpose_status == "HOLD" and "A-ugv1" in lg.reservations()
    orch.poll()                                                   # 정상 응답으로 회복
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"


def test_recontact_after_observation_does_not_observe_or_apply_twice(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    returning = {"status": "COMPLETED", "progress": {"phase": "RETURNING"}, "observation": GOOD_OBS}
    uav.default_script = lambda rid, body: [returning, {**returning, "task_id": "ATT-other"}, DONE]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()                                                   # 관측·환경 반영·완료, 복귀 중
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    orch.poll()                                                   # 다른 실행의 응답 → 불명 (완료는 그대로)
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "UNKNOWN"
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED" and "A-uav1" in lg.reservations()
    orch.poll()                                                   # 정상 응답 → 지나온 단계를 다시 밟지 않고 반납
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"
    thermal = [e for e in _events(lg, task.task_id, "OBSERVATION") if e["detail"]["sensor_type"] == "THERMAL"]
    assert len(thermal) == 1 and len(_events(lg, task.task_id, "TASK_COMPLETE")) == 1
