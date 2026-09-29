# -*- coding: utf-8 -*-
"""
orchestrator/engine.py
======================
총괄 폐루프 (규칙 기반). LLM 제안은 이 위에 얹되 같은 검증 관문을 거친다.

dispatch(task_id)  Task 하나: snapshot → 사전필터 → Local 평가/협상 → Safety → 예약·선저장 → 1회 송신
poll()             진행 중 실행시도 추적: 도착 → 모의 관측 → 환경 APPLY → 완료 → 복귀·READY 확인 후 해제
resolve(...)       사람의 수동 해소 (재평가도 Local·Safety 관문을 다시 거친다)

금지 사항 (요청서 §5·§6)
- 송신 결과를 모르면(UNKNOWN) 재전송·재출동하지 않고 점유를 유지한다.
- UAV 의 COMPLETED 만으로 Task 완료나 자원 반납을 하지 않는다.
- 도착만으로 관측 완료, 관측만으로 진화 효과를 만들지 않는다.
"""

import time
from typing import Dict, List, Optional

from . import config, negotiation, observation, safety
from .env_adapter import SnapshotGate, missing_snapshot_fields
from .ledger import Ledger, ReservationConflict, new_id
from .models import Requirements, ResourceView, Target, Task
from .prefilter import filter_candidates, haversine_m
from .resources import Unreachable

ACTIVE_ATTEMPT = ("REQUESTED", "STARTED", "ARRIVED", "OBSERVED", "APPLIED", "RETURNING", "UNKNOWN", "FAULTED")
MANUAL = "MANUAL_RESOLUTION"
MAX_SNAPSHOT_RESTARTS = 2


class Orchestrator:
    def __init__(self, ledger: Ledger, env, uav=None, ugv=None):
        self.ledger, self.env, self.uav, self.ugv = ledger, env, uav, ugv
        self.gate = SnapshotGate()

    # ------------------------------------------------------------------
    # 접수
    # ------------------------------------------------------------------
    def submit_task(self, body: dict):
        """body: {request_id, incident_id, kind, target{lat,lon,ground_amsl_m?,cell_id?}, requirements{}}"""
        req = body.get("requirements") or {}
        task = Task(
            task_id=new_id("TASK"), incident_id=body.get("incident_id") or "INC-DEFAULT",
            kind=body.get("kind") or "RECON", target=Target(**body["target"]),
            requirements=Requirements(**{k: (tuple(v) if k in ("resource_types", "agl_range_m") and v is not None
                                             else v) for k, v in req.items()}),
            request_key=body.get("request_id"),
        )
        snap = self.env.read()
        task.created_sim_s = snap.simulation_time_s
        task, created = self.ledger.create_task(task, body)
        self.ledger.log("TASK_RECEIVED", task_id=task.task_id, result="CREATED" if created else "DUPLICATE",
                        detail={"request_id": task.request_key}, sim_time_s=snap.simulation_time_s)
        return task, created

    # ------------------------------------------------------------------
    # 자원 조회
    # ------------------------------------------------------------------
    def _collect(self, types) -> (List[ResourceView], Dict[str, str], Dict[str, object]):
        views, unreachable, client_of = [], {}, {}
        if "UAV" in types and self.uav:
            for rid in self.uav.resource_ids():
                try:
                    views.append(self.uav.state(rid))
                    client_of[rid] = self.uav
                except Unreachable as e:
                    unreachable[rid] = str(e)
        if ({"UGV", "FIRE_ENGINE"} & set(types)) and self.ugv:
            try:
                for v in self.ugv.states():
                    views.append(v)
                    client_of[v.resource_id] = self.ugv
            except Unreachable as e:
                unreachable["UGV_SERVER"] = str(e)
        return views, unreachable, client_of

    # ------------------------------------------------------------------
    # 요청 payload
    # ------------------------------------------------------------------
    @staticmethod
    def _payload(task: Task, view: ResourceView, snap, decision_id: str, exec_id: str, mods: dict) -> dict:
        tgt = {"lat": task.target.lat, "lon": task.target.lon}
        body = {"task_id": exec_id, "decision_id": decision_id, "target": tgt}
        if view.resource_type == "UAV":
            agl = mods.get("target_agl_m", task.requirements.target_agl_m or config.UAV_DEFAULT_TARGET_AGL_M)
            # 지면고도만 넣는다. 80m·10m 는 UAV 가 더한다 (요청서 §4.1)
            tgt.update({"alt_m_amsl": task.target.ground_amsl_m, "target_agl_m": agl})
            body.update({"observation_type": task.requirements.sensor or "THERMAL", "wind_ms": snap.wind_ms})
        return body

    # ------------------------------------------------------------------
    # 배정
    # ------------------------------------------------------------------
    def dispatch(self, task_id: str, _restart: int = 0) -> dict:
        task = self.ledger.get_task(task_id)
        if task is None:
            return {"status": "NOT_FOUND"}
        # EVALUATING: 재평가 재시작 또는 서버가 평가 도중 멈춘 경우. 송신 전이라 다시 평가해도 안전하다
        # (송신된 시도는 APPROVED/IN_EXECUTION 으로 남는다).
        if task.purpose_status not in ("PENDING", "HOLD", "EVALUATING"):
            return {"status": "SKIPPED", "purpose_status": task.purpose_status}
        if task.purpose_status == "HOLD" and task.resume_condition == MANUAL:
            return {"status": "SKIPPED", "reason": "WAITING_MANUAL_RESOLUTION"}

        snap = self.env.read()
        sim = snap.simulation_time_s
        missing = missing_snapshot_fields(snap)
        if missing:
            return self._hold(task, "SNAPSHOT_INCOMPLETE:" + ",".join(missing), "ENV_SNAPSHOT_COMPLETE", sim)

        task.purpose_status = "EVALUATING"
        task.hold_reason = task.resume_condition = None
        self.ledger.save_task(task)

        views, unreachable, client_of = self._collect(task.requirements.resource_types)
        by_id = {v.resource_id: v for v in views}
        ranked, excluded = filter_candidates(
            task, snap, views, unreachable, self.ledger.reservations(),
            age_of=lambda v: client_of[v.resource_id].source_age_s(v))
        self.ledger.log("CANDIDATES_FILTERED", task_id=task.task_id, sim_time_s=sim, detail={
            "snapshot": snap.ref(), "candidates": [[v.resource_id, round(d, 1)] for d, v in ranked],
            "excluded": excluded, "ordering": "STRAIGHT_LINE_DISTANCE_THEN_ID (정렬용, 도착시간 아님)"})

        parent = None
        for dist, view in ranked:
            rid, client = view.resource_id, client_of[view.resource_id]
            mods, rounds, seen = {}, 0, set()
            while True:
                did = self.ledger.add_decision(task.task_id, {
                    "resource_id": rid, "modifications": mods, "snapshot": snap.ref(),
                    "straight_line_m": round(dist, 1), "policy": "RULES_V1"}, parent)
                parent = did
                payload = self._payload(task, view, snap, did, task.task_id, mods)
                try:
                    local = client.evaluate(rid, payload)
                except Unreachable as e:
                    self.ledger.log("LOCAL_RESPONSE", task_id=task.task_id, decision_id=did, resource_id=rid,
                                    result="UNREACHABLE", reason="COMMUNICATION_FAILURE", detail={"error": str(e)})
                    excluded[rid] = "LOCAL_UNREACHABLE"
                    break
                echo = local["echo"]
                if echo.get("task_id") != task.task_id or echo.get("decision_id") != did or echo.get("resource_id") != rid:
                    self.ledger.log("LOCAL_RESPONSE", task_id=task.task_id, decision_id=did, resource_id=rid,
                                    result="INVALID", reason="LOCAL_ECHO_MISMATCH", detail=local["raw"])
                    excluded[rid] = "LOCAL_ECHO_MISMATCH"
                    break
                self.ledger.log("LOCAL_RESPONSE", task_id=task.task_id, decision_id=did, resource_id=rid,
                                result=local["verdict"], reason=local.get("reason"), sim_time_s=sim,
                                detail={"eta_sec_duration": local.get("eta_sec"), "counter_offer": local.get("counter_offer"),
                                        "constraints": local.get("constraints"), "request": payload})
                self.ledger.update_decision(did, local_verdict=local["verdict"], local_reason=local.get("reason"),
                                            local_eta_sec=local.get("eta_sec"))
                if local["verdict"] == "ACCEPT":
                    out = self._approve_and_send(task, view, client, snap, did, payload, local, mods)
                    if out.get("restart") and _restart < MAX_SNAPSHOT_RESTARTS:
                        return self.dispatch(task.task_id, _restart + 1)
                    if out.get("done"):
                        return out
                    excluded[rid] = out.get("excluded_reason", "SAFETY_REJECT")
                    break
                if local["verdict"] == "COUNTER":
                    action, val = negotiation.decide_counter(task, view.resource_type, local, rounds, seen)
                    self.ledger.log("COUNTER_DECISION", task_id=task.task_id, decision_id=did, resource_id=rid,
                                    result=action, reason=None if action == "RETRY" else val,
                                    detail={"modifications": val if action == "RETRY" else None,
                                            "original_offer": local.get("counter_offer")})
                    if action == "RETRY":
                        mods, rounds = {**mods, **val}, rounds + 1
                        continue
                    excluded[rid] = val
                    break
                excluded[rid] = f"LOCAL_{local['verdict']}:{local.get('reason')}"
                break

        return self._hold(task, "NO_FEASIBLE_CANDIDATE", "NEW_RESOURCE_OR_INPUT", sim, detail={"excluded": excluded})

    def _approve_and_send(self, task, view, client, snap, did, evaluated, local, mods) -> dict:
        rid = view.resource_id
        attempt_id = new_id("ATT")
        command = self._payload(task, view, snap, did, attempt_id, mods)
        current = self.env.read()
        try:
            fresh = client.state(rid) if view.resource_type == "UAV" else next(
                (v for v in client.states() if v.resource_id == rid), None)
        except Unreachable:
            fresh = None
        verdict = safety.check(
            task=self.ledger.get_task(task.task_id), attempt_id=attempt_id, decision_id=did,
            view=fresh or view, view_age_s=client.source_age_s(fresh) if fresh else None,
            reservations=self.ledger.reservations(), eval_snapshot=snap, current_snapshot=current,
            evaluated_payload=evaluated, local=local, command=command)
        self.ledger.log("SAFETY_JUDGEMENT", task_id=task.task_id, decision_id=did, attempt_id=attempt_id,
                        resource_id=rid, result=verdict.response, reason=verdict.reason,
                        sim_time_s=current.simulation_time_s, detail={"checked": verdict.checked})
        self.ledger.update_decision(did, safety=verdict.response, safety_reasons=verdict.reasons)
        if verdict.response != "ALLOW":
            return {"restart": "SNAPSHOT_CHANGED" in verdict.reasons, "excluded_reason": "SAFETY:" + verdict.reason}
        try:
            self.ledger.prepare_attempt(attempt_id, task.task_id, did, rid, {"resource_type": view.resource_type,
                                                                              "payload": command})
        except ReservationConflict as e:
            self.ledger.log("RESERVATION_FAILED", task_id=task.task_id, decision_id=did, attempt_id=attempt_id,
                            resource_id=rid, reason="RESOURCE_OCCUPIED", detail={"error": str(e)})
            return {"excluded_reason": "RESERVATION_CONFLICT"}
        task.purpose_status = "APPROVED"
        self.ledger.save_task(task)
        result, info = client.execute(rid, command)
        self.ledger.log("EXECUTION_SENT", task_id=task.task_id, decision_id=did, attempt_id=attempt_id,
                        resource_id=rid, result=result, detail=info, sim_time_s=current.simulation_time_s)
        if result == "REFUSED":
            # 실행되지 않았음이 확실 → 예약 해제 후 다음 후보
            self.ledger.set_attempt_status(attempt_id, "NOT_SENT", info, release=True)
            task.purpose_status = "EVALUATING"
            self.ledger.save_task(task)
            return {"excluded_reason": f"EXECUTE_REFUSED:{info.get('http_status')}"}
        task.purpose_status = "IN_EXECUTION"
        self.ledger.save_task(task)
        if result == "ACK":
            self.ledger.set_attempt_status(attempt_id, "STARTED", {"last_contact_wall": time.time(),
                                                                  "execute_response": info})
        else:
            att = self.ledger.get_attempt(attempt_id)
            self._unknown(att, task, {"execute_response": info}, "EXECUTION_RESPONSE_LOST")
        sub = "STARTED" if result == "ACK" else "UNKNOWN"
        return {"done": True, "status": sub, "attempt_id": attempt_id, "resource_id": rid, "decision_id": did}

    def _hold(self, task: Task, reason: str, resume: str, sim, detail=None) -> dict:
        task.purpose_status, task.hold_reason, task.resume_condition = "HOLD", reason, resume
        self.ledger.save_task(task)
        self.ledger.log("TASK_HOLD", task_id=task.task_id, result="HOLD", reason=reason, sim_time_s=sim,
                        detail={"resume_condition": resume, **(detail or {})})
        return {"status": "HOLD", "reason": reason, "resume_condition": resume, **(detail or {})}

    # ------------------------------------------------------------------
    # 추적
    # ------------------------------------------------------------------
    def poll(self) -> List[dict]:
        updates = []
        for att in self.ledger.list_attempts(substatuses=ACTIVE_ATTEMPT):
            client = self.uav if att["command"]["resource_type"] == "UAV" else self.ugv
            if client is None:
                continue
            u = self._track(att, client)
            if u:
                updates.append(u)
        return updates

    def _track(self, att: dict, client) -> Optional[dict]:
        aid, rid, tid = att["attempt_id"], att["resource_id"], att["task_id"]
        task = self.ledger.get_task(tid)
        detail = dict(att["detail"] or {})
        status, data = client.task_status(rid, aid)
        now = time.time()

        def move(sub, reason=None, release=False, extra=None):
            if sub == att["substatus"] and not extra and not release:
                return None
            detail.update(extra or {})
            self.ledger.set_attempt_status(aid, sub, detail, release=release)
            self.ledger.log("EXECUTION_PROGRESS", task_id=tid, attempt_id=aid, resource_id=rid,
                            result=sub, reason=reason, detail={"provider_status": status, "provider": data})
            att["substatus"] = sub
            return {"attempt_id": aid, "substatus": sub, "reason": reason}

        if status == "UNREACHABLE":
            last = detail.get("last_contact_wall") or att["created_wall"]
            if att["substatus"] != "UNKNOWN" and now - last >= config.RESOURCE_OFFLINE_S:
                return self._unknown(att, task, detail, "PROVIDER_OFFLINE")
            return None
        if status == "NOT_FOUND":
            return self._unknown(att, task, detail, "PROVIDER_LOST_TASK")

        detail["last_contact_wall"] = now
        pstatus = data.get("status")
        phase = (data.get("progress") or {}).get("phase")
        sub = att["substatus"]
        if sub == "UNKNOWN" and task and task.purpose_status == "HOLD" and task.resume_condition == MANUAL:
            # 실행 파트가 이 시도를 다시 보고함 → 실제 상태를 따라 추적 재개
            task.purpose_status, task.hold_reason, task.resume_condition = "IN_EXECUTION", None, None
            self.ledger.save_task(task)
            self.ledger.log("EXECUTION_RECONTACTED", task_id=tid, attempt_id=aid, resource_id=rid,
                            detail={"provider_status": pstatus, "phase": phase})

        if pstatus in ("STARTED", "IN_PROGRESS"):
            if phase == "OBSERVING" and sub in ("STARTED", "UNKNOWN", "REQUESTED"):
                return move("ARRIVED", "PROVIDER_PHASE_OBSERVING", extra={"arrival_evidence": "UAV phase=OBSERVING"})
            if sub == "UNKNOWN":
                return move("STARTED", "PROVIDER_RECONTACTED")
            self.ledger.set_attempt_status(aid, sub, detail)
            return None

        if pstatus == "COMPLETED":
            out = None
            if sub in ("STARTED", "UNKNOWN", "REQUESTED"):
                # UAV 는 이동+체류를 마친 뒤에만 COMPLETED 를 준다 → 도착 사실은 성립
                out = move("ARRIVED", "PROVIDER_COMPLETED_IMPLIES_ARRIVAL",
                           extra={"arrival_evidence": "UAV COMPLETED after goto+hold"})
            if att["substatus"] == "ARRIVED":
                out = self._observe_and_apply(att, task, data, detail) or out
            if att["substatus"] in ("APPLIED", "OBSERVED", "RETURNING"):
                out = self._maybe_release(att, client, phase, detail, move) or out
            return out

        if sub == "FAULTED":
            # 운영자가 해제해 READY 로 돌아오면 반납
            return self._maybe_release(att, client, phase, detail, move)

        if pstatus == "FAILED":
            reason = data.get("reason") or (data.get("error") or "PROVIDER_FAILED").split(":")[0]
            if task and task.purpose_status == "IN_EXECUTION":
                # 목적 미완료. 인계 필요 — 관측 공백을 남기고 재배정 대기
                task.purpose_status, task.hold_reason = "PENDING", f"HANDOVER:{reason}"
                self.ledger.save_task(task)
                self.ledger.log("TASK_REOPENED", task_id=tid, attempt_id=aid, resource_id=rid,
                                reason=reason, detail={"observation_gap": True})
            if phase == "RETURNING":
                return move("RETURNING", reason, extra={"failed": True})
            if phase == "DONE":
                return self._maybe_release(att, client, phase, detail, move)
            if phase == "FAILED" and att["command"]["resource_type"] != "UAV":
                # UGV 서버: 이상 확정 → 차를 세우고 UNAVAILABLE. /stop 전까지 재배정 불가
                return move("FAULTED", reason, extra={"fault": data.get("error")})
            return self._unknown(att, task, detail, f"FAILED_POSITION_UNCONFIRMED:{reason}", reopen=False)
        return None

    def _unknown(self, att, task, detail, reason, reopen=True) -> Optional[dict]:
        if att["substatus"] == "UNKNOWN":
            return None
        self.ledger.set_attempt_status(att["attempt_id"], "UNKNOWN", {**detail, "unknown_reason": reason})
        self.ledger.log("EXECUTION_UNKNOWN", task_id=att["task_id"], attempt_id=att["attempt_id"],
                        resource_id=att["resource_id"], result="UNKNOWN", reason=reason,
                        detail={"occupancy_kept": True, "automatic_redispatch": False})
        if task and task.purpose_status == "IN_EXECUTION":
            task.purpose_status, task.hold_reason, task.resume_condition = "HOLD", reason, MANUAL
            self.ledger.save_task(task)
        return {"attempt_id": att["attempt_id"], "substatus": "UNKNOWN", "reason": reason}

    def _observe_and_apply(self, att, task, data, detail) -> Optional[dict]:
        aid, rid = att["attempt_id"], att["resource_id"]
        if att["command"]["resource_type"] != "UAV":
            return self._ground_observation(att, task, data, detail)
        if task.requirements.sensor != "THERMAL":
            return None
        snap = self.env.read()
        pos = ((data.get("observation") or {}).get("position")) or {}
        obs = observation.simulate(snapshot=snap, task=task, attempt_id=aid, resource_id=rid,
                                   position=pos, uav_raw_observation=data.get("observation"))
        self.ledger.log("OBSERVATION", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                        result=obs["result"], reason=obs.get("failure_reason"), sim_time_s=snap.simulation_time_s,
                        detail=obs)
        if obs["result"] == "FAILED":
            detail["observation_failed"] = obs.get("failure_reason")
            self.ledger.set_attempt_status(aid, "OBSERVED", detail)
            att["substatus"] = "OBSERVED"
            self._reopen(task, "OBSERVATION_FAILED:" + obs["failure_reason"])
            return {"attempt_id": aid, "substatus": "OBSERVED", "reason": obs["failure_reason"]}
        ack = self.env.apply(obs) if task.requirements.needs_env_ack else {"accepted": None, "skipped": True}
        self.ledger.log("ENVIRONMENT_APPLY", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                        result="ACK" if ack.get("accepted") else ("SKIPPED" if ack.get("skipped") else "NACK"),
                        reason=ack.get("reason"), detail=ack)
        detail.update({"observation_id": obs["observation_id"], "env_ack": ack})
        sub = "APPLIED" if ack.get("accepted") else "OBSERVED"
        self.ledger.set_attempt_status(aid, sub, detail)
        att["substatus"] = sub
        satisfied = obs["target_covered"] and (ack.get("accepted") or not task.requirements.needs_env_ack)
        if satisfied:
            task.purpose_status, task.hold_reason = "COMPLETED", None
            self.ledger.save_task(task)
            self.ledger.log("TASK_COMPLETE", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                            result="COMPLETED", detail={"basis": ["ARRIVED", "SIMULATED_OBSERVATION_TARGET_COVERED",
                                                                  "ENV_ACK" if ack.get("accepted") else "NO_ENV_ACK_REQUIRED"],
                                                        "resource_released": False})
        else:
            self._reopen(task, "OBSERVATION_REQUIREMENT_NOT_MET" if not obs["target_covered"] else "ENV_APPLY_NOT_ACKED")
        return {"attempt_id": aid, "substatus": sub, "task": task.purpose_status}

    def _ground_observation(self, att, task, data, detail) -> dict:
        """UGV/소방차 도착. 제공 관측은 ROAD_STATUS 뿐이며 화재 관측으로 바꾸지 않는다."""
        aid, rid = att["attempt_id"], att["resource_id"]
        raw = data.get("observation")
        need = task.requirements.sensor
        satisfied = need is None or (need == "ROAD_STATUS" and bool(raw))
        self.ledger.log("OBSERVATION", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                        result="ROAD_STATUS" if raw else "NONE",
                        detail={"source": "UGV_PROVIDER", "raw": raw, "is_fire_observation": False})
        detail["observation_source"] = "UGV_PROVIDER"
        self.ledger.set_attempt_status(aid, "OBSERVED", detail)
        att["substatus"] = "OBSERVED"
        if satisfied:
            task.purpose_status, task.hold_reason = "COMPLETED", None
            self.ledger.save_task(task)
            self.ledger.log("TASK_COMPLETE", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                            result="COMPLETED", detail={"basis": ["ARRIVED", "UGV_ROAD_STATUS" if raw else "ARRIVAL_ONLY_TASK"],
                                                        "resource_released": False})
        else:
            self._reopen(task, "ARRIVED_OBSERVATION_PENDING")
        return {"attempt_id": aid, "substatus": "OBSERVED", "task": task.purpose_status}

    def _reopen(self, task, reason):
        task.purpose_status, task.hold_reason = "PENDING", reason
        self.ledger.save_task(task)
        self.ledger.log("TASK_REOPENED", task_id=task.task_id, reason=reason)

    def _maybe_release(self, att, client, phase, detail, move) -> Optional[dict]:
        # UAV 는 착륙(phase=DONE) 뒤에만 반납 확인. UGV 는 도착 후 정지 상태를 READY 로 확인한다.
        if att["command"]["resource_type"] == "UAV" and phase != "DONE":
            return move("RETURNING", "PROVIDER_RETURNING") if att["substatus"] != "RETURNING" else None
        try:
            v = client.state(att["resource_id"]) if att["command"]["resource_type"] == "UAV" else next(
                (x for x in client.states() if x.resource_id == att["resource_id"]), None)
        except Unreachable:
            v = None
        if v is not None and v.ready and v.current_task_id is None:
            return move("RELEASED", "RESOURCE_READY_CONFIRMED", release=True,
                        extra={"released_evidence": {"ready": True, "phase": phase}})
        if att["substatus"] in ("RETURNING", "FAULTED"):
            return None                     # 아직 READY 아님 → 점유 유지
        return move("RETURNING", "LANDED_READY_UNCONFIRMED")

    # ------------------------------------------------------------------
    # 수동 해소
    # ------------------------------------------------------------------
    def resolve(self, task_id: str, action: str, reason: str, attempt_id: Optional[str] = None) -> dict:
        """action: RETRY(재평가 요청) / CANCEL / CONFIRM_RESOURCE_IDLE(사람이 기체 정지 확인 → 점유 해제)"""
        task = self.ledger.get_task(task_id)
        if task is None:
            return {"status": "NOT_FOUND"}
        self.ledger.log("MANUAL_RESOLUTION", task_id=task_id, attempt_id=attempt_id, result=action, reason=reason)
        if action == "CONFIRM_RESOURCE_IDLE":
            att = self.ledger.get_attempt(attempt_id) if attempt_id else None
            if not att or att["task_id"] != task_id or att["substatus"] != "UNKNOWN":
                return {"status": "REFUSED", "reason": "ATTEMPT_NOT_UNKNOWN"}
            self.ledger.set_attempt_status(attempt_id, "FAILED", {"manual_reason": reason}, release=True)
            task.purpose_status, task.resume_condition = "PENDING", None
            task.hold_reason = "MANUAL_IDLE_CONFIRMED"
            self.ledger.save_task(task)
            return {"status": "RELEASED"}
        if action == "RETRY":
            if any(a["substatus"] == "UNKNOWN" for a in self.ledger.list_attempts(task_id)):
                return {"status": "REFUSED", "reason": "UNKNOWN_ATTEMPT_OPEN"}
            task.purpose_status, task.resume_condition = "PENDING", None
            self.ledger.save_task(task)
            return self.dispatch(task_id)
        if action == "CANCEL":
            task.purpose_status = "CANCELLED"
            self.ledger.save_task(task)
            return {"status": "CANCELLED",
                    "note": "진행 중 기체의 중단 명령은 보내지 않았다 (UAV/UGV 중단 API 계약 대기)"}
        return {"status": "REFUSED", "reason": "UNKNOWN_ACTION"}
