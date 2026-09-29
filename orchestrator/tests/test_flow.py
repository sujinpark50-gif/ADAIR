# -*- coding: utf-8 -*-
"""요청서 §12 총괄 단독 시험 (fixture 기반 = 상태 B)."""

from orchestrator.engine import Orchestrator
from orchestrator.ledger import Ledger
from orchestrator.resources import UavClient, UgvClient

from .conftest import TARGET, submit


def _events(ledger, task_id, etype):
    return [e for e in ledger.events(task_id) if e["event_type"] == etype]


# ---------------------------------------------------------------------------
# 시나리오 1: 정상 정찰 폐루프
# ---------------------------------------------------------------------------
def test_scenario1_normal_recon_closed_loop(world):
    orch, uav, lg, env = world["orch"], world["uav"], world["ledger"], world["env"]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    assert out["status"] == "STARTED" and out["resource_id"] == "A-uav1"

    # 고도: 지면고도 그대로 + 임무 AGL 명시, 80/10 을 더하지 않는다
    _, rid, _, cmd = uav.exec_calls()[0]
    assert cmd["target"]["alt_m_amsl"] == 600.0 and cmd["target"]["target_agl_m"] == 80.0
    assert cmd["wind_ms"] == env.wind_ms and cmd["task_id"] == out["attempt_id"]

    orch.poll()                                   # ENROUTE
    orch.poll()                                   # OBSERVING → ARRIVED
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "ARRIVED"
    assert lg.get_task(task.task_id).purpose_status == "IN_EXECUTION"   # 도착 ≠ 완료

    orch.poll()                                   # COMPLETED(RETURNING) → 모의관측 → 환경 ACK
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "COMPLETED"
    att = lg.get_attempt(out["attempt_id"])
    assert att["substatus"] == "RETURNING"                               # 복귀 중: 점유 유지
    assert "A-uav1" in lg.reservations()
    obs = _events(lg, task.task_id, "OBSERVATION")[0]["detail"]
    assert obs["source"] == "SIMULATED" and obs["sensor_status"] == "TEST_ONLY"
    assert obs["result"] == "DETECTED" and obs["target_covered"]
    assert obs["uav_reported_values_ignored"] == {"max_temp_c": 312.5, "hotspot_detected": True}
    assert abs(obs["footprint"]["agl_m"] - 90.0) < 1e-6                  # 실제 높이로 재계산
    assert abs(obs["footprint"]["width_m"] - 54.0) < 1e-6

    orch.poll()                                   # DONE → READY 확인 → 해제
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"
    assert lg.reservations() == {}


# ---------------------------------------------------------------------------
# 시나리오 3: 강풍 → UAV 불가, UGV 는 열화상 능력 없음 → 보류
# ---------------------------------------------------------------------------
def test_scenario3_high_wind_holds_when_substitute_cannot_meet_requirement(world):
    orch, uav, lg, env = world["orch"], world["uav"], world["ledger"], world["env"]
    env.wind_ms = 12.4
    for rid in uav.ids:
        uav.verdicts[rid] = {"verdict": "REJECT", "reason": "HIGH_WIND"}
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    assert out["status"] == "HOLD" and out["reason"] == "NO_FEASIBLE_CANDIDATE"
    assert out["excluded"]["A-ugv1"] == "REQUIRED_CAPABILITY_UNAVAILABLE"
    assert out["excluded"]["A-uav1"] == "LOCAL_REJECT:HIGH_WIND"
    assert uav.exec_calls() == []
    # 거절마다 새 decision
    dids = [e["decision_id"] for e in _events(lg, task.task_id, "LOCAL_RESPONSE")]
    assert len(dids) == 2 and len(set(dids)) == 2


def test_reject_then_next_candidate(world):
    orch, uav = world["orch"], world["uav"]
    uav.verdicts["A-uav1"] = {"verdict": "REJECT", "reason": "LOW_BATTERY"}
    out = orch.dispatch(submit(orch).task_id)
    assert out["status"] == "STARTED" and out["resource_id"] == "A-uav2"


# ---------------------------------------------------------------------------
# COUNTER (3A)
# ---------------------------------------------------------------------------
def test_note_only_counter_is_not_executed(world):
    orch, uav = world["orch"], world["uav"]
    for rid in uav.ids:
        uav.verdicts[rid] = {"verdict": "COUNTER", "reason": "MODERATE_WIND",
                             "counter_offer": {"note": "풍속 주의. 고도 하향 권장"}}
    out = orch.dispatch(submit(orch).task_id)
    assert out["status"] == "HOLD" and out["excluded"]["A-uav1"] == "COUNTER_NOT_ACTIONABLE"
    assert uav.exec_calls() == []


def test_counter_with_unsupported_field_is_not_executed(world):
    orch, uav = world["orch"], world["uav"]
    for rid in uav.ids:
        uav.verdicts[rid] = {"verdict": "COUNTER", "reason": "RETURN_MARGIN_INSUFFICIENT",
                             "counter_offer": {"observe_duration_s": 30, "note": "체류 단축"}}
    out = orch.dispatch(submit(orch).task_id)
    assert out["excluded"]["A-uav1"] == "COUNTER_FIELD_UNSUPPORTED:observe_duration_s"
    assert uav.exec_calls() == []


def test_concrete_counter_is_reevaluated_and_same_values_executed(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]

    def v(body):
        if body["target"]["target_agl_m"] == 60:
            return {"verdict": "ACCEPT", "eta_sec": 500}
        return {"verdict": "COUNTER", "reason": "MODERATE_WIND", "counter_offer": {"target_agl_m": 60}}
    uav.verdicts["A-uav1"] = v
    task = submit(orch, requirements={"resource_types": ["UAV"], "sensor": "THERMAL", "agl_range_m": [50, 100]})
    out = orch.dispatch(task.task_id)
    assert out["status"] == "STARTED" and out["resource_id"] == "A-uav1"
    evals = uav.eval_calls()
    assert [c[3]["target"]["target_agl_m"] for c in evals] == [80.0, 60]
    assert evals[0][3]["decision_id"] != evals[1][3]["decision_id"]
    assert uav.exec_calls()[0][3]["target"]["target_agl_m"] == 60
    assert out["decision_id"] == evals[1][3]["decision_id"]


def test_counter_outside_requirement_or_repeated_is_dropped(world):
    orch, uav = world["orch"], world["uav"]
    uav.verdicts["A-uav1"] = {"verdict": "COUNTER", "reason": "MODERATE_WIND", "counter_offer": {"target_agl_m": 30}}
    uav.verdicts["A-uav2"] = {"verdict": "COUNTER", "reason": "MODERATE_WIND", "counter_offer": {"target_agl_m": 60}}
    out = orch.dispatch(submit(orch, requirements={"resource_types": ["UAV"], "sensor": "THERMAL",
                                                   "agl_range_m": [50, 100]}).task_id)
    assert out["excluded"]["A-uav1"] == "COUNTER_BREAKS_REQUIREMENT"
    assert out["excluded"]["A-uav2"] == "COUNTER_REPEATED"      # 60 으로 재평가해도 같은 COUNTER
    assert uav.exec_calls() == []


# ---------------------------------------------------------------------------
# 입력 누락
# ---------------------------------------------------------------------------
def test_missing_wind_holds_but_zero_wind_is_valid(world):
    orch, env, uav = world["orch"], world["env"], world["uav"]
    env.wind_ms = None
    out = orch.dispatch(submit(orch, requirements={"resource_types": ["UAV"], "sensor": "THERMAL"}).task_id)
    assert out["status"] == "HOLD" and "WIND_MISSING" in out["excluded"]["A-uav1"]
    assert uav.eval_calls() == []
    env.wind_ms = 0.0
    out = orch.dispatch(submit(orch, "REQ-2", requirements={"resource_types": ["UAV"], "sensor": "THERMAL"}).task_id)
    assert out["status"] == "STARTED"


def test_missing_ground_altitude_never_sent(world):
    orch, uav = world["orch"], world["uav"]
    tgt = dict(TARGET, ground_amsl_m=None)
    out = orch.dispatch(submit(orch, target=tgt).task_id)
    assert out["status"] == "HOLD" and "TARGET_ALTITUDE_UNKNOWN" in out["excluded"]["A-uav1"]
    assert uav.eval_calls() == []


def test_stale_resource_state_is_excluded(world):
    orch, uav = world["orch"], world["uav"]
    uav.state["A-uav1"]["timestamp"] = "2026-01-01T00:00:00Z"
    out = orch.dispatch(submit(orch).task_id)
    assert out["resource_id"] == "A-uav2"


# ---------------------------------------------------------------------------
# Safety / snapshot
# ---------------------------------------------------------------------------
def test_snapshot_change_between_eval_and_safety_forces_reevaluation(world):
    orch, uav, env, lg = world["orch"], world["uav"], world["env"], world["ledger"]
    calls = {"n": 0}

    def v(body):
        calls["n"] += 1
        if calls["n"] == 1:
            env.advance()           # 평가 직후 환경이 바뀜
        return {"verdict": "ACCEPT", "eta_sec": 500}
    uav.verdicts["A-uav1"] = v
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    assert out["status"] == "STARTED"
    safeties = _events(lg, task.task_id, "SAFETY_JUDGEMENT")
    assert safeties[0]["result"] == "REJECT" and "SNAPSHOT_CHANGED" in safeties[0]["reason"]
    assert safeties[-1]["result"] == "ALLOW"
    assert len(uav.exec_calls()) == 1


# ---------------------------------------------------------------------------
# 실행 불명(UNKNOWN)·중복·재시작
# ---------------------------------------------------------------------------
def test_execute_timeout_is_unknown_keeps_occupancy_and_never_resends(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.execute_mode["A-uav1"] = "timeout"
    t1 = submit(orch)
    out = orch.dispatch(t1.task_id)
    assert out["status"] == "UNKNOWN"
    assert lg.reservations()["A-uav1"]["attempt_id"] == out["attempt_id"]
    orch.poll()                                   # 조회 404 → UNKNOWN 유지, 수동 해소 대기
    assert lg.get_task(t1.task_id).resume_condition == "MANUAL_RESOLUTION"
    assert orch.dispatch(t1.task_id)["status"] == "SKIPPED"
    assert orch.resolve(t1.task_id, "RETRY", "재시도")["status"] == "REFUSED"
    assert len(uav.exec_calls()) == 1             # 재전송 없음

    # 다른 Task 는 점유된 A-uav1 을 못 쓴다
    uav.execute_mode["A-uav1"] = "ok"
    out2 = orch.dispatch(submit(orch, "REQ-2").task_id)
    assert out2["resource_id"] == "A-uav2"

    # 사람이 기체 정지를 확인해야 해제된다
    r = orch.resolve(t1.task_id, "CONFIRM_RESOURCE_IDLE", "관제가 착륙 확인", attempt_id=out["attempt_id"])
    assert r["status"] == "RELEASED" and "A-uav1" not in lg.reservations()


def test_execute_409_is_refused_and_released(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.execute_mode["A-uav1"] = "409"
    out = orch.dispatch(submit(orch).task_id)
    assert out["resource_id"] == "A-uav2"
    assert set(lg.reservations()) == {"A-uav2"}


def test_duplicate_request_dispatches_once(world):
    orch, uav = world["orch"], world["uav"]
    t1 = submit(orch)
    t2 = submit(orch)
    assert t1.task_id == t2.task_id
    orch.dispatch(t1.task_id)
    orch.dispatch(t2.task_id)
    assert len(uav.exec_calls()) == 1


def test_restart_recovers_occupancy(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    out = orch.dispatch(submit(orch).task_id)
    lg.close()
    lg2 = Ledger(world["db"])
    orch2 = Orchestrator(lg2, world["env"], UavClient(uav.endpoints(), uav.client()),
                         UgvClient("http://ugv", world["ugv"].client()))
    out2 = orch2.dispatch(submit(orch2, "REQ-2").task_id)
    assert out2["resource_id"] == "A-uav2"
    orch2.poll()
    assert lg2.get_attempt(out["attempt_id"])["substatus"] in ("STARTED", "ARRIVED")


# ---------------------------------------------------------------------------
# 관측 요구 미충족 / 실패 인계
# ---------------------------------------------------------------------------
def test_observation_off_target_reopens_task(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]

    def off(rid, body):
        pos = {"lat": body["target"]["lat"] + 0.01, "lon": body["target"]["lon"], "alt_m_amsl": 690.0}
        return [{"status": "COMPLETED", "progress": {"phase": "RETURNING"},
                 "observation": {"position": pos, "sensor_type": "THERMAL", "values": {}}}]
    uav.default_script = off
    task = submit(orch)
    orch.dispatch(task.task_id)
    orch.poll()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "OBSERVATION_REQUIREMENT_NOT_MET"
    assert "A-uav1" in lg.reservations()          # 복귀 확인 전 점유 유지


def test_return_margin_failure_reopens_for_handover_and_keeps_occupancy(world):
    orch, uav, lg = world["orch"], world["uav"], world["ledger"]
    uav.default_script = lambda rid, body: [
        {"status": "FAILED", "reason": "RETURN_MARGIN_INSUFFICIENT", "progress": {"phase": "RETURNING"}}]
    task = submit(orch)
    out = orch.dispatch(task.task_id)
    orch.poll()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "HANDOVER:RETURN_MARGIN_INSUFFICIENT"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RETURNING"
    out2 = orch.dispatch(task.task_id)            # 후임 배정
    assert out2["resource_id"] == "A-uav2"


# ---------------------------------------------------------------------------
# UGV (가짜 서버) — 도로상황 임무, 주행 중 고장
# ---------------------------------------------------------------------------
UGV_REQ = {"resource_types": ["UGV"], "sensor": "ROAD_STATUS", "needs_env_ack": False}


def test_ugv_road_status_task_completes_and_releases(world):
    orch, ugv, lg = world["orch"], world["ugv"], world["ledger"]
    ugv.default_script = lambda rid: [
        {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
        {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
         "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1"},
         "_unit": {"state": "READY", "current_task_id": None}}]
    task = submit(orch, requirements=UGV_REQ)
    out = orch.dispatch(task.task_id)
    assert out["resource_id"] == "A-ugv1"
    for _ in range(3):
        orch.poll()
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"


def test_ugv_fault_reopens_task_and_keeps_vehicle_until_ready(world):
    orch, ugv, lg = world["orch"], world["ugv"], world["ledger"]
    ugv.default_script = lambda rid: [
        {"status": "FAILED", "progress": {"phase": "FAILED"}, "error": "STALLED: 30초간 이동 0 m",
         "_unit": {"state": "UNAVAILABLE", "current_task_id": None}}]
    task = submit(orch, requirements=UGV_REQ)
    out = orch.dispatch(task.task_id)
    orch.poll()
    att = lg.get_attempt(out["attempt_id"])
    assert att["substatus"] == "FAULTED" and "A-ugv1" in lg.reservations()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "HANDOVER:STALLED"
    orch.poll()                                   # 아직 UNAVAILABLE → 계속 점유
    assert "A-ugv1" in lg.reservations()
    ugv.units["A-ugv1"]["state"] = "READY"        # 운영자가 /stop 으로 해제
    orch.poll()
    assert lg.get_attempt(out["attempt_id"])["substatus"] == "RELEASED"
    assert lg.reservations() == {}
