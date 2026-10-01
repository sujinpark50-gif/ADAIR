# -*- coding: utf-8 -*-
"""추가 수정 요구 R01~R04 (2026-09-30, 검증 기준 d2e1c80).

R01 인계 뒤 이전 시도가 후임의 목적 상태를 바꾸지 못한다 (목적 담당 시도를 장부에 기록).
R02 칸 일부만 보고 불을 못 본 관측은 기존 신고·확인 화재를 해소하지 않는다.
R03 형식이 틀린 상태 응답은 예외 없이 격리되고 다른 자원의 추적은 계속된다.
R04 환경 반영 대기는 기체 반납과 별개로 남아 같은 관측 ID 로 재반영된다 (새 비행으로 풀지 않는다).

부분 관측 시험은 한 프레임 기준 센서(conftest 가 고정, 90m 칸의 약 27%)로 재현한다. 운영 기본값인
임시 모의 범위 90×90m 는 바꾸지 않는다 (test_orbit_assumption).
"""

import json

import pytest
from fastapi.testclient import TestClient

from orchestrator.api import create_app
from orchestrator.engine import Orchestrator
from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis
from orchestrator.ledger import ALL_RUNS, Ledger
from orchestrator.resources import UavClient, UgvClient

from .conftest import TARGET, submit
from .fakes import normal_flight

FAILED_RETURNING = {"status": "FAILED", "reason": "RETURN_MARGIN_INSUFFICIENT", "progress": {"phase": "RETURNING"}}
FAILED_DONE = {"status": "FAILED", "reason": "RETURN_MARGIN_INSUFFICIENT", "progress": {"phase": "DONE"}}


def _ev(lg, task_id, etype):
    return [e for e in lg.events(task_id) if e["event_type"] == etype]


def _restart(world):
    world["ledger"].close()
    lg = Ledger(world["db"])
    uav, env = world["uav"], world["env"]
    orch = Orchestrator(lg, env, UavClient(uav.endpoints(), uav.client()),
                        UgvClient("http://ugv", world["ugv"].client()), analysis=InjectedAnalysis(),
                        weather_feeds=[EnvStationFeed(env, TARGET["lat"], TARGET["lon"])])
    world.update(orch=orch, ledger=lg)
    return orch, lg


# ---------------------------------------------------------------------------
# R01 인계
# ---------------------------------------------------------------------------
def _handover(world):
    """A-uav1 실패(복귀 중) → A-uav2 로 인계한 상태를 만든다"""
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    flight = normal_flight()
    uav.default_script = lambda rid, body: [dict(FAILED_RETURNING)] if rid == "A-uav1" else flight(rid, body)
    task = submit(orch)
    first = orch.dispatch(task.task_id)
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "PENDING" and "A-uav1" in lg.reservations()
    second = orch.resolve(task.task_id, "RETRY", "후임 배정")
    assert second["status"] == "STARTED" and second["resource_id"] == "A-uav2"
    assert lg.get_task(task.task_id).owner_attempt_id == second["attempt_id"]
    return task, first, second


def test_repeated_failure_report_of_old_attempt_does_not_override_successor(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task, first, second = _handover(world)
    orch.poll()                                                   # A-uav1 이 같은 FAILED/RETURNING 을 또 보고
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"    # 후임의 목적은 그대로
    for _ in range(2):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"       # 후임이 관측·ACK → 완료
    assert _ev(lg, task.task_id, "TASK_COMPLETE")[0]["attempt_id"] == second["attempt_id"]
    orch.poll()
    # 점유는 각자 반납 조건을 만족할 때 따로 풀린다
    assert lg.get_attempt(second["attempt_id"])["substatus"] == "RELEASED"
    assert lg.get_attempt(first["attempt_id"])["substatus"] == "RETURNING" and set(lg.reservations()) == {"A-uav1"}
    uav.scripts[first["attempt_id"]] = [dict(FAILED_DONE)]        # 이전 기체 착륙
    orch.poll()
    assert lg.get_attempt(first["attempt_id"])["substatus"] == "RELEASED" and lg.reservations() == {}
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert len(_ev(lg, task.task_id, "TASK_REOPENED")) == 1        # 재대기는 처음 실패 때 한 번뿐
    assert len(uav.exec_calls()) == 2


def test_old_attempt_unknown_and_late_normal_response_do_not_touch_successor(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task, first, second = _handover(world)
    late = {"status": "COMPLETED", "progress": {"phase": "RETURNING"},
            "observation": {"position": {"lat": 38.05, "lon": 128.25, "alt_m_amsl": 690.0}}}
    uav.scripts[first["attempt_id"]] = [{**FAILED_RETURNING, "task_id": "ATT-other"}, late, late]
    orch.poll()                                                   # 이전 시도가 일시 불명
    assert lg.get_attempt(first["attempt_id"])["substatus"] == "UNKNOWN"
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "IN_EXECUTION" and t.resume_condition is None   # 후임 임무가 보류로 바뀌지 않는다
    orch.poll()                                                   # 이전 시도의 늦은 '정상 완료' 응답
    assert lg.get_attempt(first["attempt_id"])["substatus"] == "RETURNING"
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    obs = _ev(lg, task.task_id, "OBSERVATION")
    assert {e["attempt_id"] for e in obs} == {second["attempt_id"]}     # 완료에 쓴 관측은 후임 것뿐
    assert lg.get_task(task.task_id).owner_attempt_id == second["attempt_id"]


def test_purpose_owner_survives_restart(world):
    task, first, second = _handover(world)
    orch, lg = _restart(world)
    assert lg.get_task(task.task_id).owner_attempt_id == second["attempt_id"]
    orch.recover()
    for _ in range(3):
        orch.poll()                                               # 재시작 뒤에도 이전 시도의 실패 보고는 무시
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert _ev(lg, task.task_id, "TASK_COMPLETE")[0]["attempt_id"] == second["attempt_id"]
    assert "A-uav1" in lg.reservations()


def test_handover_requires_mission_end_evidence_not_just_returning_label(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task = submit(orch)
    first = orch.dispatch(task.task_id)
    # 실패 확정 근거 없이 상태 이름만 RETURNING 인 시도 (임무 종료를 확인하지 못함)
    lg.set_attempt_status(first["attempt_id"], "RETURNING", {})
    stale = lg.get_task(task.task_id)
    orch._set_purpose(stale, "PENDING", basis="시험 준비")
    assert [a["attempt_id"] for a in lg.open_mission_attempts(task.task_id)] == [first["attempt_id"]]
    assert orch.dispatch(task.task_id)["reason"] == "ATTEMPT_STILL_OPEN"
    assert orch.resolve(task.task_id, "RETRY", "재시도")["reason"] == "ATTEMPT_STILL_OPEN"
    assert len(uav.exec_calls()) == 1
    lg.set_attempt_status(first["attempt_id"], "RETURNING", {"mission_end": {"reason": "PROVIDER_FAILED:X"}})
    assert lg.open_mission_attempts(task.task_id) == []


def test_unknown_old_attempt_blocks_handover(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.default_script = lambda rid, body: [dict(FAILED_RETURNING), {**FAILED_RETURNING, "task_id": "ATT-other"}]
    task = submit(orch)
    orch.dispatch(task.task_id)
    orch.poll()
    orch.poll()                                                   # 실패 뒤 불명
    assert orch.resolve(task.task_id, "RETRY", "후임")["reason"] == "UNKNOWN_ATTEMPT_OPEN"
    assert orch.dispatch(task.task_id)["status"] == "SKIPPED" and len(uav.exec_calls()) == 1


def test_attempt_event_from_non_owner_is_rejected_at_the_boundary(world):
    orch, lg = world["orch"], world["ledger"]
    task, first, second = _handover(world)
    t = lg.get_task(task.task_id)
    assert orch._set_purpose(t, "PENDING", basis="OLD_ATTEMPT_EVENT", attempt_id=first["attempt_id"]) is False
    rej = _ev(lg, task.task_id, "PURPOSE_TRANSITION_REJECTED")[-1]
    assert rej["detail"]["why"] == "NOT_PURPOSE_OWNER" and rej["detail"]["owner_attempt_id"] == second["attempt_id"]
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"


# ---------------------------------------------------------------------------
# R02 부분 음성 관측
# ---------------------------------------------------------------------------
REPORT = {"cell_id": "C1", "sim_time_s": 0.0, "source": "119_CALL"}


def _fly(orch, task_id, n=4):
    orch.dispatch(task_id)
    for _ in range(n):
        orch.poll()


def test_partial_negative_observation_keeps_reported_fire_candidate(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    env.reports = [dict(REPORT)]
    env.fire_cells[0]["fire_state"] = "UNBURNED"                  # 본 범위(칸의 약 27%)에는 불이 없다
    orch.view()
    recon = [t for t in lg.list_tasks() if t.kind == "RECON"][0]
    _fly(orch, recon.task_id)
    assert lg.get_task(recon.task_id).purpose_status == "COMPLETED"      # 정찰은 목표 지점 기준으로 끝남
    f = orch.kb.fire_states(1e9)["C1"]
    assert f["status"] == "REPORTED" and f["source"] == "119_CALL"       # 신고는 해소되지 않는다
    lp = f["last_partial_observation"]
    assert lp["coverage_fraction"] == pytest.approx(52 * 42 / 8100, abs=1e-5)
    assert lp["unobserved_fraction"] == pytest.approx(1 - 52 * 42 / 8100, abs=1e-5) and lp["source"] == "SIMULATED"
    assert list(orch.kb.known_fires(1e9)) == ["C1"]
    v = orch.view()
    assert [c["cell_id"] for c in v.fire_cells] == ["C1"]
    assert v.fire_cells[0]["knowledge"]["last_partial_observation"]["observation_id"] == lp["observation_id"]
    # 원본 이력(신고 + 부분 관측)은 그대로 남는다
    assert [k["body"]["status"] for k in lg.knowledge("FIRE")] == ["REPORTED", "NOT_DETECTED_PARTIAL"]


def test_partial_negative_observation_keeps_confirmed_fire(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    first = submit(orch)
    _fly(orch, first.task_id)                                     # 불 확인 (칸 일부만 봤어도 본 사실은 성립)
    assert orch.kb.fire_states(1e9)["C1"]["status"] == "CONFIRMED"
    env.fire_cells[0]["fire_state"] = "UNBURNED"
    env.advance()
    again = orch.resolve(first.task_id, "RECHECK", "다시 확인")
    _fly(orch, again["task_id"])
    f = orch.kb.fire_states(1e9)["C1"]
    assert f["status"] == "CONFIRMED" and f["last_partial_observation"]["sim_time_s"] == env.simulation_time_s
    assert list(orch.kb.known_fires(1e9)) == ["C1"]


def test_partial_positive_is_recorded_and_full_negative_resolves(world):
    orch, env = world["orch"], world["env"]
    env.reports = [dict(REPORT)]
    orch.view()
    t = [x for x in world["ledger"].list_tasks() if x.kind == "RECON"][0]
    _fly(orch, t.task_id)
    f = orch.kb.fire_states(1e9)["C1"]
    assert f["status"] == "CONFIRMED" and "last_partial_observation" not in f   # 부분 양성: 발견 사실 반영
    env.fire_cells[0].update(fire_state="UNBURNED", cell_size_m=40.0)           # 이제 칸 전체가 프레임 안
    env.advance()
    again = orch.resolve(t.task_id, "RECHECK", "진화 확인")
    _fly(orch, again["task_id"])
    assert orch.kb.fire_states(1e9)["C1"]["status"] == "OBSERVED_CLEAR"         # 칸 전체 음성 → 해소
    assert orch.kb.known_fires(1e9) == {} and orch.view().fire_cells == []


def test_partial_negative_without_prior_evidence_is_not_a_fire_candidate(world):
    orch, env = world["orch"], world["env"]
    env.fire_cells[0]["fire_state"] = "UNBURNED"
    _fly(orch, submit(orch).task_id)
    assert orch.kb.fire_states(1e9)["C1"]["status"] == "NOT_DETECTED_PARTIAL"
    assert orch.kb.known_fires(1e9) == {}


def test_api_board_and_analysis_input_agree_on_fire_belief(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    env.reports = [dict(REPORT)]
    env.fire_cells[0]["fire_state"] = "UNBURNED"
    orch.view()
    _fly(orch, [t for t in lg.list_tasks() if t.kind == "RECON"][0].task_id)
    c = TestClient(create_app(orch))
    known = c.get("/priority/board").json()["known"]["fires"]
    belief = json.loads(json.dumps(world["analysis"].last_input))
    assert known == [{"cell_id": "C1", **belief["fire_states"]["C1"]}]
    assert [f["cell_id"] for f in belief["known_fires"]] == ["C1"]
    assert belief["known_fires"][0]["knowledge"] == belief["fire_states"]["C1"]
    assert known[0]["status"] == "REPORTED" and known[0]["last_partial_observation"]["status"] == "NOT_DETECTED_PARTIAL"


# ---------------------------------------------------------------------------
# R03 비정상 응답 격리
# ---------------------------------------------------------------------------
def test_malformed_response_of_one_resource_does_not_stop_tracking_of_another(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    flight = normal_flight()
    bad = {"status": [], "progress": {"phase": "DONE"}}
    uav.default_script = lambda rid, body: ([dict(bad), {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}}]
                                            if rid == "A-uav1" else flight(rid, body))
    t1, t2 = submit(orch, "REQ-1"), submit(orch, "REQ-2")
    a1, a2 = orch.dispatch(t1.task_id), orch.dispatch(t2.task_id)
    assert (a1["resource_id"], a2["resource_id"]) == ("A-uav1", "A-uav2")
    orch.poll()                                                   # 예외 없이 끝난다
    orch.poll()
    assert lg.get_attempt(a2["attempt_id"])["substatus"] == "ARRIVED"           # 다른 자원 추적은 계속
    assert lg.get_attempt(a1["attempt_id"])["substatus"] == "STARTED"           # 후속 정상 응답으로 회복
    assert _ev(lg, t1.task_id, "PROVIDER_RESPONSE_REJECTED")[0]["reason"] == "STATUS_INVALID"
    assert _ev(lg, t1.task_id, "TASK_COMPLETE") == [] and "A-uav1" in lg.reservations()


@pytest.mark.parametrize("bad,why", [
    ({"status": []}, "STATUS_INVALID"), ({"status": {}}, "STATUS_INVALID"), ({"status": 7}, "STATUS_INVALID"),
    ({}, "STATUS_INVALID"),
    ({"status": "COMPLETED", "progress": {"phase": ["DONE"]}}, "PHASE_INVALID"),
    ({"status": "COMPLETED", "progress": {"phase": "RETURNING"}, "observation": {"position": "38,128"}},
     "POSITION_INVALID"),
    ({"status": "COMPLETED", "progress": {"phase": "RETURNING"},
      "observation": {"position": {"lat": "38.05", "lon": 128.25, "alt_m_amsl": 690.0}}}, "POSITION_INVALID"),
    ({"status": "COMPLETED", "progress": {"phase": "RETURNING"},
      "observation": {"position": {"lat": 38.05, "lon": 128.25, "alt_m_amsl": [690.0]}}}, "POSITION_INVALID"),
    ({"status": "FAILED", "reason": {"code": 1}, "progress": {"phase": "RETURNING"}}, "REASON_INVALID"),
    ({"status": "FAILED", "error": ["x"], "progress": {"phase": "RETURNING"}}, "ERROR_INVALID"),
])
def test_malformed_status_or_nested_fields_are_isolated_without_exception(world, bad, why):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    flight = normal_flight()
    uav.default_script = lambda rid, body: [dict(bad)] + flight(rid, body)
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()
    att = lg.get_attempt(out["attempt_id"])
    assert att["substatus"] == "UNKNOWN" and att["detail"]["unknown_reason"] == f"RESPONSE_VALIDATION_FAILED:{why}"
    assert "A-uav1" in lg.reservations()
    for etype in ("OBSERVATION", "ENVIRONMENT_APPLY", "TASK_COMPLETE"):
        assert _ev(lg, task.task_id, etype) == []
    for _ in range(4):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"


def test_internal_error_on_one_attempt_is_logged_others_tracked_and_error_not_hidden(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    t1, t2 = submit(orch, "REQ-1"), submit(orch, "REQ-2")
    a1, a2 = orch.dispatch(t1.task_id), orch.dispatch(t2.task_id)
    real = orch.uav.task_status

    def broken(rid, aid):
        if rid == "A-uav1":
            raise RuntimeError("내부 결함")
        return real(rid, aid)
    orch.uav.task_status = broken
    with pytest.raises(RuntimeError):
        orch.poll()
    assert lg.get_attempt(a2["attempt_id"])["detail"]["last_contact_wall"]      # 같은 poll 에서 A-uav2 는 추적됨
    err = [e for e in lg.events() if e["event_type"] == "TRACK_ERROR"][0]
    assert err["attempt_id"] == a1["attempt_id"] and err["reason"] == "RuntimeError"


# ---------------------------------------------------------------------------
# R04 환경 반영 대기
# ---------------------------------------------------------------------------
def _env_down(world):
    env = world["env"]
    env.fire_cells[0]["fire_state"] = "UNBURNED"                  # 불 발견에 따른 추가 측정 없이 ACK 만 본다
    real, calls = env.apply, []

    def down(obs):
        calls.append(obs["observation_id"])
        raise ConnectionError("env unreachable")
    env.apply = down

    def recover():
        def up(obs):
            calls.append(obs["observation_id"])
            return real(obs)
        env.apply = up
    return calls, recover


def test_pending_apply_survives_resource_release_and_is_reapplied_without_second_flight(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    calls, recover = _env_down(world)
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED" and lg.reservations() == {}   # 기체는 정상 반납
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "IN_EXECUTION" and t.hold_reason == "ENV_APPLY_PENDING"
    row = lg.pending_applies(task_id=task.task_id)[0]
    assert (row["run_id"], row["attempt_id"], row["status"]) == ("RUN-FIXTURE", out["attempt_id"], "PENDING")
    oid = row["observation_id"]

    # 반영 대기 중에는 같은 목적을 다시 출동시키지 않는다
    assert orch.dispatch(task.task_id)["status"] == "SKIPPED"
    assert orch.dispatch_pending()["results"] == {}
    assert orch.resolve(task.task_id, "RETRY", "다시 보내기")["reason"] == "ENV_APPLY_PENDING"
    assert orch.manual_dispatch(task.task_id, "지금 보내기")["status"] == "REFUSED"
    assert len(uav.exec_calls()) == 1

    recover()
    orch.poll()                                                   # 환경 복구 → 같은 관측 ID 로 재반영
    assert set(calls) == {oid} and lg.pending_applies(task_id=task.task_id) == []
    done = lg.pending_applies(task_id=task.task_id, status="DONE")[0]
    assert (done["observation_id"], done["env_result"], done["reason"]) == (oid, "ACK", "TASK_COMPLETED")
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"          # 반납된 시도 상태는 그대로
    assert len(_ev(lg, task.task_id, "OBSERVATION")) == 1 and len(uav.exec_calls()) == 1
    n = len(calls)
    orch.poll()
    orch.poll()
    assert len(calls) == n and len(_ev(lg, task.task_id, "TASK_COMPLETE")) == 1  # 끝난 뒤 다시 반영하지 않는다


def test_pending_apply_is_recovered_after_restart(world):
    calls, recover = _env_down(world)
    orch, uav = world["orch"], world["uav"]
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    orch, lg = _restart(world)
    assert len(lg.pending_applies(task_id=task.task_id)) == 1
    orch.recover()
    orch.dispatch_pending()
    assert len(uav.exec_calls()) == 1                             # 재시작 뒤에도 재출동 없음
    recover()
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED" and len(set(calls)) == 1


def test_duplicate_ack_from_environment_is_idempotent(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    env.fire_cells[0]["fire_state"] = "UNBURNED"
    real, seen = env.apply, []

    def applied_but_response_lost(obs):
        seen.append(obs["observation_id"])
        ack = real(obs)                                           # 환경은 반영했지만
        if len(seen) == 1:
            raise TimeoutError("응답 유실")                        # 총괄은 응답을 못 받음
        return ack
    env.apply = applied_but_response_lost
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(3):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    ack = _ev(lg, task.task_id, "ENVIRONMENT_APPLY")[-1]
    assert ack["result"] == "ACK" and ack["detail"]["duplicate"] is True and len(env.applied) == 1


def test_cancel_while_apply_pending_is_not_revived_by_late_ack(world):
    calls, recover = _env_down(world)
    orch, lg = world["orch"], world["ledger"]
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    n = len(calls)
    assert orch.resolve(task.task_id, "CANCEL", "취소")["status"] == "CANCELLED"
    row = lg.pending_applies(task_id=task.task_id, status="CLOSED")[0]
    assert row["reason"] == "TASK_CANCELLED"                      # 지우지 않고 사유와 함께 닫는다
    recover()
    orch.poll()
    assert len(calls) == n                                        # 취소된 목적의 관측은 환경에 반영하지 않는다
    assert lg.get_task(task.task_id).purpose_status == "CANCELLED" and _ev(lg, task.task_id, "TASK_COMPLETE") == []


def test_pending_apply_is_not_applied_to_another_run(world):
    calls, recover = _env_down(world)
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    n = len(calls)
    recover()
    env.run_id, env.fire_cells, env.reports = "RUN-B", [], []
    orch.poll()                                                   # 환경은 이미 다른 run → 적용하지 않는다
    assert len(calls) == n
    orch.view()                                                   # 점유가 없으므로 run 전환
    assert lg.active_run() == "RUN-B"
    row = lg.pending_applies(task_id=task.task_id, run_id=ALL_RUNS, status="CLOSED")[0]
    assert row["reason"] == "RUN_ENDED" and row["run_id"] == "RUN-FIXTURE"
    orch.poll()
    assert len(calls) == n and lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"


def test_late_ack_is_not_attributed_when_purpose_owner_changed(world):
    calls, recover = _env_down(world)
    orch, lg = world["orch"], world["ledger"]
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    t = lg.get_task(task.task_id)
    t.owner_attempt_id = "ATT-successor"                          # 담당이 다른 시도로 바뀐 상황
    lg.save_task(t)
    recover()
    orch.poll()
    row = lg.pending_applies(task_id=task.task_id, status="CLOSED")[0]           # 응답은 기록에 남고
    assert (row["env_result"], row["reason"], row["ack"]["accepted"]) == ("ACK", "OWNER_CHANGED", True)
    closed = _ev(lg, task.task_id, "ENV_APPLY_RECORD_CLOSED")[-1]
    assert closed["reason"] == "OWNER_CHANGED" and closed["detail"]["attributed_to_purpose"] is False
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"           # 목적은 담당 시도만 바꾼다
    assert _ev(lg, task.task_id, "TASK_COMPLETE") == []


def test_explicit_nack_is_distinguished_from_unknown_timeout(world):
    orch, env, lg = world["orch"], world["env"], world["ledger"]
    env.fire_cells[0]["fire_state"] = "UNBURNED"
    env.apply = lambda obs: {"accepted": False, "reason": "REJECTED_BY_ENV"}
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(3):
        orch.poll()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "ENV_APPLY_NOT_ACKED"   # 거절 → 다시 관측해야 함
    assert lg.pending_applies(task_id=task.task_id) == []                             # 끝나지 않은 기록은 없다
    row = lg.pending_applies(task_id=task.task_id, status=None)[0]
    assert (row["status"], row["env_result"], row["reason"]) == ("DONE", "NACK", "TASK_PENDING")
    assert _ev(lg, task.task_id, "ENVIRONMENT_APPLY")[0]["result"] == "NACK"


# ---------------------------------------------------------------------------
# R04 보강: 환경 응답 저장과 임무 판정을 따로 기록 (피드백 2026-09-30)
#   PENDING(환경 응답 모름) → RESPONDED(응답 저장, 판정 전) → DONE/CLOSED
# ---------------------------------------------------------------------------
class Crash(BaseException):
    """총괄 프로세스 강제 종료"""


OBS_POS = {"position": {"lat": 38.05, "lon": 128.25, "alt_m_amsl": 690.0}}
RETURNING = {"status": "COMPLETED", "progress": {"phase": "RETURNING"}, "observation": OBS_POS}
LANDED = {"status": "COMPLETED", "progress": {"phase": "DONE"}, "observation": OBS_POS}


def _observed_then_unknown(world, nack=False):
    """관측 뒤 환경이 응답하지 않고, 이어서 실행시도가 불명이 된 상태. 그 뒤 환경이 돌아와 응답을 준다."""
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    calls, recover = _env_down(world)
    uav.default_script = lambda rid, body: [dict(RETURNING), {**RETURNING, "task_id": "ATT-other"},
                                            dict(RETURNING), dict(LANDED)]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()                                                   # 도착·관측, 환경 응답 없음
    orch.poll()                                                   # 다른 실행의 응답 → 불명, 임무 보류
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "UNKNOWN"
    assert lg.get_task(task.task_id).purpose_status == "HOLD"
    if nack:
        world["env"].apply = lambda obs: {"accepted": False, "reason": "REJECTED_BY_ENV"}
    else:
        recover()
    assert orch.process_pending_applies() == []                   # 환경 응답은 받아 저장하지만 판정은 미룬다
    row = lg.pending_applies(task_id=task.task_id)[0]
    assert row["status"] == "RESPONDED" and row["env_result"] == ("NACK" if nack else "ACK")
    assert lg.get_task(task.task_id).purpose_status == "HOLD"
    return task, out, calls


def test_ack_received_while_attempt_unknown_is_kept_and_applied_after_recontact(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task, out, calls = _observed_then_unknown(world)
    n = len(calls)
    orch.poll()                                                   # 정상 재접촉 → 저장해 둔 ACK 로 판정
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert len(calls) == n                                        # 환경에 다시 보내지 않았다
    assert lg.pending_applies(task_id=task.task_id, status="DONE")[0]["reason"] == "TASK_COMPLETED"
    assert "A-uav1" in lg.reservations()                          # 점유 해제는 READY 확인으로 따로
    orch.poll()
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED" and len(uav.exec_calls()) == 1
    assert len(_ev(lg, task.task_id, "TASK_COMPLETE")) == 1 and len(_ev(lg, task.task_id, "OBSERVATION")) == 1


def test_nack_received_while_attempt_unknown_reopens_after_recontact(world):
    orch, lg = world["orch"], world["ledger"]
    task, out, _ = _observed_then_unknown(world, nack=True)
    orch.poll()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "ENV_APPLY_NOT_ACKED"
    row = lg.pending_applies(task_id=task.task_id, status="DONE")[0]
    assert row["env_result"] == "NACK" and _ev(lg, task.task_id, "TASK_COMPLETE") == []


def test_stored_ack_does_not_revive_cancelled_purpose(world):
    orch, lg = world["orch"], world["ledger"]
    task, out, calls = _observed_then_unknown(world)
    assert orch.resolve(task.task_id, "CANCEL", "취소")["status"] == "CANCELLED"
    row = lg.pending_applies(task_id=task.task_id, status="CLOSED")[0]
    assert row["reason"] == "TASK_CANCELLED" and row["ack"]["accepted"] is True   # 응답은 보관, 목적은 그대로
    for _ in range(2):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "CANCELLED" and _ev(lg, task.task_id, "TASK_COMPLETE") == []
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"


def test_stored_ack_is_not_applied_after_owner_change(world):
    orch, lg = world["orch"], world["ledger"]
    task, out, _ = _observed_then_unknown(world)
    t = lg.get_task(task.task_id)
    t.owner_attempt_id = "ATT-successor"
    lg.save_task(t)
    orch.process_pending_applies()
    row = lg.pending_applies(task_id=task.task_id, status="CLOSED")[0]
    assert row["reason"] == "OWNER_CHANGED" and row["env_result"] == "ACK"
    assert lg.get_task(task.task_id).purpose_status == "HOLD"


def test_idle_confirmation_with_open_apply_record_continues_judgement_instead_of_redispatch(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    task, out, _ = _observed_then_unknown(world)
    r = orch.resolve(task.task_id, "CONFIRM_RESOURCE_IDLE", "관제가 착륙 확인", attempt_id=out["attempt_id"])
    assert r == {"status": "RELEASED", "purpose_status": "IN_EXECUTION"} and lg.reservations() == {}
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED" and len(uav.exec_calls()) == 1


def test_process_stop_after_ack_stored_is_resumed_after_restart_without_resending(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    calls, recover = _env_down(world)
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    recover()

    def crash(*a, **k):
        raise Crash()
    orch._conclude = crash                                        # 환경 응답 저장 직후, 임무 판정 전에 종료
    with pytest.raises(Crash):
        orch.poll()
    row = lg.pending_applies(task_id=task.task_id)[0]
    assert row["status"] == "RESPONDED" and row["env_result"] == "ACK"
    n = len(calls)
    orch, lg = _restart(world)
    assert [u["task"] for u in orch.recover()["applies_resumed"]] == ["COMPLETED"]
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED" and len(calls) == n
    assert len(_ev(lg, task.task_id, "TASK_COMPLETE")) == 1 and len(uav.exec_calls()) == 1


def test_judgement_is_atomic_stop_in_the_middle_leaves_no_half_applied_result(world):
    orch, lg = world["orch"], world["ledger"]
    calls, recover = _env_down(world)
    task = submit(orch)
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    recover()
    real = lg.finish_pending_apply

    def crash(*a, **k):
        raise Crash()
    lg.finish_pending_apply = crash                               # Task 완료 기록 뒤, 판정 종료 표시 전에 종료
    with pytest.raises(Crash):
        orch.poll()
    lg.finish_pending_apply = real
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"           # 완료 기록도 함께 되돌아갔다
    assert _ev(lg, task.task_id, "TASK_COMPLETE") == []
    assert lg.pending_applies(task_id=task.task_id)[0]["status"] == "RESPONDED"
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert len(_ev(lg, task.task_id, "TASK_COMPLETE")) == 1


def test_area_task_does_not_double_accumulate_across_restart_or_interrupted_judgement(world):
    from .test_area_completion import M1, M2, _map, _monitor
    orch, lg = world["orch"], world["ledger"]
    _map(world, M1, M2)
    env = world["env"]
    real_apply = env.apply

    def down(obs):
        raise ConnectionError("env unreachable")
    env.apply = down
    task = _monitor(orch, ["M1", "M2"])
    orch.dispatch(task.task_id)
    for _ in range(4):
        orch.poll()
    assert lg.get_task(task.task_id).coverage == {}                # 환경 응답 전에는 누적도 판정도 하지 않는다
    env.apply = real_apply
    real = lg.finish_pending_apply

    def crash(*a, **k):
        raise Crash()
    lg.finish_pending_apply = crash
    with pytest.raises(Crash):
        orch.poll()                                               # 누적 기록 중간에 종료
    lg.finish_pending_apply = real
    assert lg.get_task(task.task_id).coverage == {} and _ev(lg, task.task_id, "AREA_COVERAGE") == []
    orch, lg = _restart(world)
    orch.recover()
    orch.poll()
    t = lg.get_task(task.task_id)
    assert t.coverage["M1"]["fraction"] == 1.0 and len(t.coverage["M1"]["rects"]) == 1
    assert len(t.coverage["M2"]["rects"]) == 1 and len(_ev(lg, task.task_id, "AREA_COVERAGE")) == 1
    assert t.purpose_status == "PENDING" and t.hold_reason == "AREA_COVERAGE_INCOMPLETE"
    assert lg.pending_applies(task_id=task.task_id) == []


# ---------------------------------------------------------------------------
# R03 보강: 관측이 필요한 단계의 필수 위치 (피드백 2026-09-30)
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("bad", [
    {"status": "COMPLETED", "progress": {"phase": "RETURNING"}},                                   # observation 없음
    {"status": "COMPLETED", "progress": {"phase": "RETURNING"}, "observation": {"sensor_type": "THERMAL"}},
    {"status": "COMPLETED", "progress": {"phase": "RETURNING"}, "observation": {"position": {}}},
    {"status": "COMPLETED", "progress": {"phase": "RETURNING"},
     "observation": {"position": {"lat": 38.05, "lon": 128.25}}},                                  # 높이 없음
    {"status": "COMPLETED", "progress": {"phase": "DONE"},
     "observation": {"position": {"lat": None, "lon": 128.25, "alt_m_amsl": 690.0}}},
])
def test_uav_completion_without_required_position_is_isolated_before_observation(world, bad):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.default_script = lambda rid, body: [dict(bad), dict(RETURNING), dict(LANDED)]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()
    att = lg.get_attempt(out["attempt_id"])
    assert att["substatus"] == "UNKNOWN" and att["detail"]["unknown_reason"] == "RESPONSE_VALIDATION_FAILED:POSITION_MISSING"
    assert "A-uav1" in lg.reservations() and lg.get_task(task.task_id).purpose_status == "HOLD"
    for etype in ("OBSERVATION", "ENVIRONMENT_APPLY", "TASK_COMPLETE", "TASK_REOPENED"):
        assert _ev(lg, task.task_id, etype) == []
    assert lg.pending_applies(task_id=task.task_id, status=None) == []
    orch.dispatch_pending()
    assert len(uav.exec_calls()) == 1                             # 잘못된 응답 때문에 다시 보내지 않는다
    orch.poll()
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"


def test_position_is_not_required_for_progress_or_post_observation_reports(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.default_script = lambda rid, body: [
        {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},            # 이동 중: 위치 없음
        dict(RETURNING),
        {"status": "COMPLETED", "progress": {"phase": "DONE"}}]                  # 관측 저장 뒤 복귀 보고: 위치 없음
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    for _ in range(3):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"
    assert _ev(lg, task.task_id, "PROVIDER_RESPONSE_REJECTED") == []


def test_weather_task_requires_lat_lon_but_not_altitude(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    req = {"resource_types": ["UAV"], "sensor": "WEATHER"}
    no_alt = {"status": "COMPLETED", "progress": {"phase": "DONE"},
              "observation": {"position": {"lat": 38.05, "lon": 128.25}}}
    uav.default_script = lambda rid, body: [{"status": "COMPLETED", "progress": {"phase": "DONE"},
                                             "observation": {"position": {"lon": 128.25}}}, dict(no_alt)]
    task = submit(orch, "SENSE-1", kind="ENV_SENSE", requirements=req)
    out = orch.dispatch(task.task_id)
    orch.poll()
    assert lg.get_attempt(out["attempt_id"])["detail"]["unknown_reason"].endswith("POSITION_MISSING")
    orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
