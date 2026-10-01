# -*- coding: utf-8 -*-
"""R01~R04 종합 검증 뒤 추가 수정 F01~F07 (검토 기준 92a3153).

F01 관측을 한 번 확정해 원본·사건·아는 세계·OBSERVED·반영 기록을 한 트랜잭션으로 저장
F02 관측 뒤의 물리 실패(FAILED)는 이미 확보한 관측의 판정을 버리지 않는다
F03 함께 한 기상 측정도 APPLY 전에 저장하고 같은 반영 경로를 쓴다
F04 취소와 끝나지 않은 반영 기록 종료를 한 트랜잭션으로 (복구 시에도 같은 결과)
F05 이전 판(094f965)의 ACKED/NACKED 기록 이관
F06 위치 수치 검사가 예외(OverflowError 등)를 내지 않는다
F07 run 활성화와 이전 run 기록 종료를 한 트랜잭션으로, recover 에서 멱등 정리

중단은 Crash(BaseException)로 흉내 낸다 — 엔진의 일반 예외 처리에 잡히지 않는다.
"""

import pytest

from orchestrator.engine import Orchestrator
from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis
from orchestrator.ledger import ALL_RUNS, AttemptConflict, Ledger
from orchestrator.resources import UavClient, UgvClient

from .conftest import TARGET, submit
from .test_area_completion import M1, M2, _map, _monitor

POS = {"position": {"lat": 38.05, "lon": 128.25, "alt_m_amsl": 690.0}}
RETURNING = {"status": "COMPLETED", "progress": {"phase": "RETURNING"}, "observation": POS}
LANDED = {"status": "COMPLETED", "progress": {"phase": "DONE"}, "observation": POS}


class Crash(BaseException):
    """총괄 프로세스 강제 종료"""


def _ev(lg, task_id, etype):
    return [e for e in lg.events(task_id) if e["event_type"] == etype]


def _thermal(lg, task_id):
    return [e for e in _ev(lg, task_id, "OBSERVATION") if e["detail"]["sensor_type"] == "THERMAL"]


def _restart(world):
    world["ledger"].close()
    lg = Ledger(world["db"])
    uav, env = world["uav"], world["env"]
    orch = Orchestrator(lg, env, UavClient(uav.endpoints(), uav.client()),
                        UgvClient("http://ugv", world["ugv"].client()), analysis=InjectedAnalysis(),
                        weather_feeds=[EnvStationFeed(env, TARGET["lat"], TARGET["lon"])])
    world.update(orch=orch, ledger=lg)
    return orch, lg


def _crash_on(obj, name, when=lambda *a, **k: True):
    real = getattr(obj, name)

    def f(*a, **k):
        if when(*a, **k):
            setattr(obj, name, real)                  # 한 번만 멈춘다
            raise Crash()
        return real(*a, **k)
    setattr(obj, name, f)


def _env_calls(world, fail_first=0, exc=ConnectionError):
    env, calls = world["env"], []
    real = env.apply

    def apply(obs):
        calls.append(obs["observation_id"])
        if len(calls) <= fail_first:
            raise exc("env unreachable")
        return real(obs)
    env.apply = apply
    return calls


# ---------------------------------------------------------------------------
# F01 관측 원본 확정·원자적 저장
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("step", ["save_observation", "log", "add_knowledge", "set_attempt_status",
                                  "add_pending_apply"])
def test_stop_inside_observation_save_rolls_back_everything_and_restart_completes_once(world, step):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    world["env"].fire_cells[0]["fire_state"] = "UNBURNED"
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()                                                   # 도착(ARRIVED)
    when = {"log": lambda et, **k: et == "OBSERVATION",
            "set_attempt_status": lambda aid, sub, *a, **k: sub == "OBSERVED"}.get(step, lambda *a, **k: True)
    _crash_on(lg, step, when)
    with pytest.raises(Crash):
        orch.poll()                                               # 관측 저장 도중 종료
    # 반쪽 저장이 남지 않는다: 원본·관측 사건·아는 세계·OBSERVED·반영 기록 모두 없음
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "ARRIVED"
    assert _ev(lg, task.task_id, "OBSERVATION") == [] and lg.knowledge("FIRE") == []
    assert lg.get_observation(out["attempt_id"], "MAIN") is None
    assert lg.pending_applies(task_id=task.task_id, status=None) == []
    orch, lg = _restart(world)
    orch.recover()
    for _ in range(3):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert len(_thermal(lg, task.task_id)) == 1 and len(_ev(lg, task.task_id, "TASK_COMPLETE")) == 1
    assert len(uav.exec_calls()) == 1


def test_saved_observation_is_replayed_unchanged_even_if_world_changes_before_restart(world):
    orch, uav, lg, env = world["orch"], world["uav"], world["ledger"], world["env"]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()
    _crash_on(orch, "_process_apply")                            # 저장은 끝났고 환경에 보내기 직전에 종료
    with pytest.raises(Crash):
        orch.poll()
    original = lg.get_observation(out["attempt_id"], "MAIN")
    assert original["result"] == "DETECTED" and lg.pending_applies(task_id=task.task_id)[0]["status"] == "PENDING"
    env.fire_cells[0]["fire_state"] = "UNBURNED"                  # 재시작 사이에 진짜 세계가 바뀜
    env.advance(5)
    calls = _env_calls(world)
    orch, lg = _restart(world)
    orch.recover()
    aux = lg.get_observation(out["attempt_id"], "AUX")            # 불 발견 → 같은 방문의 기상 측정도 저장돼 있었다
    assert sorted(calls) == sorted([original["observation_id"], aux["observation_id"]])   # 원래 관측을 원래 ID 로 반영
    assert lg.get_observation(out["attempt_id"], "MAIN") == original
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert len(_thermal(lg, task.task_id)) == 1 and orch.kb.fire_states(1e9)["C1"]["status"] == "CONFIRMED"
    assert len(uav.exec_calls()) == 1


def test_area_observation_stop_and_restart_does_not_double_accumulate(world):
    orch, lg = world["orch"], world["ledger"]
    _map(world, M1, M2)
    task = _monitor(orch, ["M1", "M2"])
    orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()
    _crash_on(lg, "add_pending_apply")
    with pytest.raises(Crash):
        orch.poll()
    assert lg.get_task(task.task_id).coverage == {}
    orch, lg = _restart(world)
    orch.recover()
    orch.poll()
    t = lg.get_task(task.task_id)
    assert len(t.coverage["M1"]["rects"]) == 1 and len(t.coverage["M2"]["rects"]) == 1
    assert len(_ev(lg, task.task_id, "AREA_COVERAGE")) == 1 and t.hold_reason == "AREA_COVERAGE_INCOMPLETE"


# ---------------------------------------------------------------------------
# F02 관측 뒤 물리 실패
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("failed_phase", ["RETURNING", "DONE"])
@pytest.mark.parametrize("nack", [False, True])
def test_failure_after_valid_observation_keeps_judgement(world, failed_phase, nack):
    orch, uav, lg, env = world["orch"], world["uav"], world["ledger"], world["env"]
    env.fire_cells[0]["fire_state"] = "UNBURNED"
    fail = {"status": "FAILED", "reason": "RETURN_ABORTED", "progress": {"phase": failed_phase}}
    uav.default_script = lambda rid, body: [dict(RETURNING), dict(fail),
                                            {"status": "FAILED", "progress": {"phase": "DONE"}}]
    real = env.apply

    def down(obs):
        raise ConnectionError("down")
    env.apply = down
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()                                                   # 관측, 환경 응답 대기
    orch.poll()                                                   # 관측 뒤 FAILED
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "IN_EXECUTION" and t.hold_reason == "ENV_APPLY_PENDING"   # 재대기로 바꾸지 않는다
    assert lg.get_attempt(out["attempt_id"])["detail"]["physical_failure_after_observation"] == "RETURN_ABORTED"
    env.apply = (lambda obs: {"accepted": False, "reason": "REJECTED_BY_ENV"}) if nack else real
    for _ in range(2):
        orch.poll()
    t = lg.get_task(task.task_id)
    if nack:
        assert t.purpose_status == "PENDING" and t.hold_reason == "ENV_APPLY_NOT_ACKED"  # 거절은 재관측 대기
    else:
        assert t.purpose_status == "COMPLETED"
        orch.dispatch_pending()
        assert len(uav.exec_calls()) == 1                         # 두 번째 출동 없음
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"


def test_failure_with_unconfirmed_position_after_observation_does_not_block_completion(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    world["env"].fire_cells[0]["fire_state"] = "UNBURNED"
    calls = _env_calls(world, fail_first=1)
    uav.default_script = lambda rid, body: [dict(RETURNING), {"status": "FAILED", "progress": {"phase": "FAILED"},
                                                              "error": "link lost"}]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "UNKNOWN" and "A-uav1" in lg.reservations()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"   # 물리 불명과 목적 판정은 따로
    assert len(set(calls)) == 1


def test_current_uav_return_failed_phase_completes_purpose_and_keeps_occupancy(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.default_script = lambda rid, body: [dict(RETURNING), {"status": "COMPLETED", "progress": {"phase": "RETURN_FAILED"},
                                                              "observation": POS, "error": "return failed: x"}]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    for _ in range(3):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RETURNING" and "A-uav1" in lg.reservations()


def test_failure_before_observation_still_reopens_for_handover(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.default_script = lambda rid, body: [{"status": "FAILED", "reason": "RETURN_MARGIN_INSUFFICIENT",
                                             "progress": {"phase": "RETURNING"}}]
    task = submit(orch)
    orch.dispatch(task.task_id)
    orch.poll()
    assert lg.get_task(task.task_id).hold_reason == "HANDOVER:RETURN_MARGIN_INSUFFICIENT"


# ---------------------------------------------------------------------------
# F03 함께 한 기상 측정
# ---------------------------------------------------------------------------
def _aux_rows(lg, task_id, status=None):
    return [r for r in lg.pending_applies(task_id=task_id, status=status) if not r["purpose_bound"]]


def _fire_recon(world):
    orch = world["orch"]
    task = submit(orch)                                           # 불타는 C1 → 불 발견 → 같은 방문에서 기상 측정
    out = orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()
    return task, out


def test_aux_measurement_is_saved_before_apply_and_recovered_after_stop(world):
    orch, lg, env = world["orch"], world["ledger"], world["env"]
    task, out = _fire_recon(world)
    real = env.apply

    def apply(obs):
        if obs["sensor_type"] == "WEATHER":
            env.apply = real
            raise Crash()                                         # 함께 한 측정을 환경에 보내는 순간 종료
        return real(obs)
    env.apply = apply
    with pytest.raises(Crash):
        orch.poll()
    aux = lg.get_observation(out["attempt_id"], "AUX")
    assert aux is not None and _aux_rows(lg, task.task_id)[0]["observation_id"] == aux["observation_id"]
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"            # 주 관측은 이미 판정됨
    calls = _env_calls(world)
    orch, lg = _restart(world)
    orch.recover()
    assert calls == [aux["observation_id"]]
    row = _aux_rows(lg, task.task_id, status="DONE")[0]
    assert row["reason"] == "AUXILIARY_MEASUREMENT" and row["env_result"] == "ACK"
    assert len([e for e in _ev(lg, task.task_id, "OBSERVATION") if e["detail"]["sensor_type"] == "WEATHER"]) == 1


def test_aux_response_lost_then_resent_with_same_id_is_idempotent(world):
    orch, lg, env = world["orch"], world["ledger"], world["env"]
    task, out = _fire_recon(world)
    real, seen = env.apply, []

    def apply(obs):
        ack = real(obs)
        seen.append(obs["observation_id"])
        if obs["sensor_type"] == "WEATHER" and seen.count(obs["observation_id"]) == 1:
            raise TimeoutError("응답 유실")                        # 환경은 반영, 응답만 유실
        return ack
    env.apply = apply
    orch.poll()
    orch.poll()
    aux = lg.get_observation(out["attempt_id"], "AUX")
    assert seen.count(aux["observation_id"]) == 2 and len(env.applied) == 2        # 주 1 + 부가 1
    assert _aux_rows(lg, task.task_id, status="DONE")[0]["ack"]["duplicate"] is True


def test_aux_response_stored_then_stop_resumes_without_new_apply(world):
    orch, lg = world["orch"], world["ledger"]
    task, out = _fire_recon(world)
    calls = _env_calls(world)
    _crash_on(lg, "finish_pending_apply", lambda oid, status, reason=None: reason == "AUXILIARY_MEASUREMENT")
    with pytest.raises(Crash):
        orch.poll()
    assert _aux_rows(lg, task.task_id)[0]["status"] == "RESPONDED"
    n = len(calls)
    orch, lg = _restart(world)
    orch.recover()
    assert len(calls) == n and _aux_rows(lg, task.task_id, status="DONE")


# ---------------------------------------------------------------------------
# F04 취소
# ---------------------------------------------------------------------------
def _pending_main_and_aux(world):
    orch, lg = world["orch"], world["ledger"]
    calls = _env_calls(world, fail_first=10 ** 6)                 # 환경이 계속 응답하지 않음
    task, out = _fire_recon(world)
    orch.poll()
    rows = lg.pending_applies(task_id=task.task_id)
    assert sorted(r["purpose_bound"] for r in rows) == [False, True] and {r["status"] for r in rows} == {"PENDING"}
    return task, out, calls


def test_cancel_and_record_closing_are_atomic(world):
    orch, lg = world["orch"], world["ledger"]
    task, out, calls = _pending_main_and_aux(world)
    _crash_on(lg, "finish_pending_apply")
    with pytest.raises(Crash):
        orch.resolve(task.task_id, "CANCEL", "취소")
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"           # 둘 다 저장되지 않았다
    assert len(lg.pending_applies(task_id=task.task_id)) == 2
    assert orch.resolve(task.task_id, "CANCEL", "취소")["status"] == "CANCELLED"
    closed = lg.pending_applies(task_id=task.task_id, status="CLOSED")
    assert len(closed) == 2 and {r["reason"] for r in closed} == {"TASK_CANCELLED"}


def test_cancel_saved_without_record_closing_converges_after_restart(world):
    """이전 판처럼 취소만 저장되고 기록 종료 전에 멈춘 장부도 재시작 후 같은 결과가 된다."""
    orch, lg, uav = world["orch"], world["ledger"], world["uav"]
    task, out, calls = _pending_main_and_aux(world)
    t = lg.get_task(task.task_id)
    t.purpose_status = "CANCELLED"
    lg.save_task(t)                                               # 취소만 저장된 상태
    n = len(calls)
    orch, lg = _restart(world)
    orch.recover()
    orch.poll()
    assert len(calls) == n                                        # 주·부가 측정 모두 더 적용하지 않는다
    closed = lg.pending_applies(task_id=task.task_id, status="CLOSED")
    assert len(closed) == 2 and {r["reason"] for r in closed} == {"TASK_CANCELLED"}
    assert lg.get_task(task.task_id).purpose_status == "CANCELLED"
    for _ in range(2):
        orch.poll()                                               # 기체 추적·반납은 계속
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED" and len(uav.exec_calls()) == 1


def test_completed_task_aux_measurement_keeps_retrying(world):
    orch, lg, env = world["orch"], world["ledger"], world["env"]
    real = env.apply

    def apply(obs):
        if obs["sensor_type"] == "WEATHER" and apply.down:
            raise ConnectionError("down")
        return real(obs)
    apply.down = True
    env.apply = apply
    task, out = _fire_recon(world)
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert _aux_rows(lg, task.task_id)[0]["status"] == "PENDING"                 # 완료 임무의 부가 측정은 계속 대기
    apply.down = False
    orch.poll()
    assert _aux_rows(lg, task.task_id, status="DONE")


# ---------------------------------------------------------------------------
# F05 이전 판 기록 이관
# ---------------------------------------------------------------------------
def _legacy_state(world, legacy_status, task_status=None):
    """환경 응답 대기까지 진행한 뒤, 기록을 이전 판 형식(ACKED/NACKED, 응답 본문 없음)으로 바꾼다."""
    orch, lg = world["orch"], world["ledger"]
    world["env"].fire_cells[0]["fire_state"] = "UNBURNED"
    calls = _env_calls(world, fail_first=10 ** 6)
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    oid = lg.pending_applies(task_id=task.task_id)[0]["observation_id"]
    if task_status:
        t = lg.get_task(task.task_id)
        t.purpose_status = task_status
        lg.save_task(t)
    lg._conn.execute("UPDATE pending_applies SET status=?, ack=NULL, reason=NULL WHERE observation_id=?",
                     (legacy_status, oid))
    return task, out, oid, calls


def test_legacy_acked_record_with_unjudged_task_is_resumed_without_new_apply(world):
    task, out, oid, calls = _legacy_state(world, "ACKED")
    n = len(calls)
    orch, lg = _restart(world)
    assert lg.legacy_apply_migration == {oid: "RESPONDED"}
    row = lg.pending_applies(task_id=task.task_id)[0]
    assert row["ack"] == {"accepted": True, "reason": None, "restored_from": "LEGACY_STATUS_ACKED"}
    orch.recover()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED" and len(calls) == n
    assert len(world["uav"].exec_calls()) == 1


def test_legacy_nacked_record_reopens_for_reobservation(world):
    task, out, oid, calls = _legacy_state(world, "NACKED")
    orch, lg = _restart(world)
    orch.recover()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "ENV_APPLY_NOT_ACKED"


@pytest.mark.parametrize("task_status", ["COMPLETED", "CANCELLED"])
def test_legacy_record_of_closed_task_is_not_reapplied(world, task_status):
    task, out, oid, calls = _legacy_state(world, "ACKED", task_status=task_status)
    n = len(calls)
    orch, lg = _restart(world)
    row = lg.pending_applies(task_id=task.task_id, status="DONE")[0]
    assert row["reason"] == f"LEGACY_ACKED_ALREADY_JUDGED:{task_status}"
    orch.recover()
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == task_status and len(calls) == n


def test_legacy_record_without_task_goes_to_manual_review_and_blocks_redispatch(world):
    task, out, oid, calls = _legacy_state(world, "ACKED")
    world["ledger"]._conn.execute("UPDATE pending_applies SET attempt_id='ATT-gone' WHERE observation_id=?", (oid,))
    orch, lg = _restart(world)
    row = lg.pending_applies(task_id=task.task_id, status="MANUAL_REVIEW")[0]
    assert row["reason"] == "LEGACY_ACKED_TASK_OR_ATTEMPT_MISSING" and row["ack"]["restored_from"]
    with pytest.raises(AttemptConflict):
        lg.prepare_attempt("ATT-new", task.task_id, "DEC", "A-uav2", {"resource_type": "UAV", "payload": {}})


def test_legacy_migration_is_repeatable(world):
    task, out, oid, calls = _legacy_state(world, "ACKED")
    orch, lg = _restart(world)
    first = lg.pending_applies(task_id=task.task_id, status=None)
    orch2, lg2 = _restart(world)
    assert lg2.legacy_apply_migration == {} and lg2.pending_applies(task_id=task.task_id, status=None) == first


def test_legacy_pending_record_is_processed_normally(world):
    orch, lg = world["orch"], world["ledger"]
    world["env"].fire_cells[0]["fire_state"] = "UNBURNED"
    env = world["env"]
    real = env.apply

    def down(obs):
        raise ConnectionError("down")
    env.apply = down
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    env.apply = real
    orch, lg = _restart(world)
    orch.recover()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"


# ---------------------------------------------------------------------------
# F06 위치 수치 검사
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("field", ["lat", "lon", "alt_m_amsl"])
@pytest.mark.parametrize("value,why", [(10 ** 400, "POSITION_INVALID"), (-(10 ** 400), "POSITION_INVALID"),
                                       ("38.05", "POSITION_INVALID"), (True, "POSITION_INVALID"),
                                       ([1.0], "POSITION_INVALID"), (None, "POSITION_MISSING")])
def test_invalid_position_values_are_isolated_without_exception(world, field, value, why):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    bad = {"status": "COMPLETED", "progress": {"phase": "RETURNING"},
           "observation": {"position": {**POS["position"], field: value}}}
    uav.default_script = lambda rid, body: ([dict(bad), dict(RETURNING), dict(LANDED)] if rid == "A-uav1"
                                            else [{"status": "IN_PROGRESS", "progress": {"phase": "OBSERVING"}}])
    t1, t2 = submit(orch, "REQ-1"), submit(orch, "REQ-2")
    a1, a2 = orch.dispatch(t1.task_id), orch.dispatch(t2.task_id)
    orch.poll()                                                   # 예외 없이 끝난다
    att = lg.get_attempt(a1["attempt_id"])
    assert att["substatus"] == "UNKNOWN" and att["detail"]["unknown_reason"] == f"RESPONSE_VALIDATION_FAILED:{why}"
    assert "A-uav1" in lg.reservations() and _ev(lg, t1.task_id, "OBSERVATION") == []
    assert lg.pending_applies(task_id=t1.task_id, status=None) == []
    assert lg.get_attempt(a2["attempt_id"])["substatus"] == "ARRIVED"         # 같은 poll 의 다른 자원 추적 계속
    for _ in range(2):
        orch.poll()
    assert lg.get_task(t1.task_id).purpose_status == "COMPLETED"              # 후속 정상 응답으로 회복


@pytest.mark.parametrize("value", [float("inf"), float("-inf"), float("nan"), 10 ** 400, 10 ** 308 * 10])
def test_non_finite_or_overflowing_numbers_are_position_invalid(world, value):
    """비유한 값은 JSON 으로 보낼 수 없어 HTTP 가짜 서버 대신 검사 함수로 직접 확인한다."""
    orch, lg = world["orch"], world["ledger"]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    att = lg.get_attempt(out["attempt_id"])
    data = {"task_id": out["attempt_id"], "status": "COMPLETED", "progress": {"phase": "RETURNING"},
            "observation": {"position": {**POS["position"], "alt_m_amsl": value}}}
    assert orch._response_problem(att, data, lg.get_task(task.task_id)) == "POSITION_INVALID"


# ---------------------------------------------------------------------------
# F07 run 전환
# ---------------------------------------------------------------------------
def _pending_then_switch_env(world):
    orch, lg, env = world["orch"], world["ledger"], world["env"]
    env.fire_cells[0]["fire_state"] = "UNBURNED"
    calls = _env_calls(world, fail_first=10 ** 6)
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()                                               # 기체 반납, 반영 기록만 남음
    env.run_id, env.fire_cells, env.reports = "RUN-B", [], []
    return task, calls


def test_run_activation_and_old_record_closing_are_atomic(world):
    orch, lg = world["orch"], world["ledger"]
    task, calls = _pending_then_switch_env(world)
    _crash_on(lg, "close_pending_applies_of_other_runs")
    with pytest.raises(Crash):
        orch.view()                                               # 활성화 도중 종료
    assert lg.active_run() == "RUN-FIXTURE" and lg.run_status("RUN-B") is None      # 메모리·장부 모두 이전 run
    assert lg.pending_applies(task_id=task.task_id, run_id=ALL_RUNS)[0]["status"] == "PENDING"
    orch, lg = _restart(world)
    orch.recover()
    assert lg.active_run() == "RUN-B"
    row = lg.pending_applies(task_id=task.task_id, run_id=ALL_RUNS, status="CLOSED")[0]
    assert row["reason"] == "RUN_ENDED" and row["run_id"] == "RUN-FIXTURE" and row["observation"]
    n = len(calls)
    orch.poll()
    assert len(calls) == n                                        # 이전 관측을 새 run 에 적용하지 않는다


def test_recover_closes_old_run_records_left_open(world):
    orch, lg = world["orch"], world["ledger"]
    task, calls = _pending_then_switch_env(world)
    orch.view()                                                   # 새 run 활성화 (같은 트랜잭션에서 종료)
    lg._conn.execute("UPDATE pending_applies SET status='RESPONDED', ack='{\"accepted\": true}', resolved_wall=NULL")
    orch, lg = _restart(world)                                    # 이전 run 기록이 열린 채 남은 장부
    orch.recover()
    row = lg.pending_applies(task_id=task.task_id, run_id=ALL_RUNS, status="CLOSED")[0]
    assert row["reason"] == "RUN_ENDED" and row["ack"] == {"accepted": True}   # 받은 응답은 보존
    assert lg.pending_applies(run_id=ALL_RUNS) == []
