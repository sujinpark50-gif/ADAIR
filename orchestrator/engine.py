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

import dataclasses
import time
from typing import Dict, List, Optional

from . import config, negotiation, observation, priority, safety
from .env_adapter import SnapshotGate, missing_snapshot_fields
from .knowledge import Knowledge
from .ledger import Ledger, ReservationConflict, new_id
from .models import Requirements, ResourceView, Target, Task
from .prefilter import filter_candidates, haversine_m
from .resources import Unreachable

ACTIVE_ATTEMPT = ("REQUESTED", "STARTED", "ARRIVED", "OBSERVED", "APPLIED", "RETURNING", "UNKNOWN", "FAULTED")
MANUAL = "MANUAL_RESOLUTION"
MAX_SNAPSHOT_RESTARTS = 2


class Orchestrator:
    def __init__(self, ledger: Ledger, env, uav=None, ugv=None, llm=None, analysis=None, weather_feeds=()):
        self.ledger, self.env, self.uav, self.ugv, self.llm = ledger, env, uav, ugv, llm
        self.kb = Knowledge(ledger)
        self.analysis = analysis              # 위험 칸·확산 예측 (입력은 총괄이 아는 세계만)
        self.weather_feeds = list(weather_feeds)
        self.gate = SnapshotGate()
        self.priority_mode = config.PRIORITY_MODE

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
            area_cell_ids=list(body.get("area_cell_ids") or []),
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
    # 총괄이 보는 화면 (진짜 세계를 직접 보지 않는다)
    # ------------------------------------------------------------------
    def _sync_knowledge(self, truth) -> None:
        """그 시각까지의 신고·기상 관측을 아는 세계에 넣는다 (중복은 key 로 무시)."""
        now = truth.simulation_time_s or 0.0
        for r in getattr(self.env, "reports_until", lambda t: [])(now):
            self.kb.report_fire(r["cell_id"], r.get("sim_time_s", now), r.get("source", "REPORT"),
                                key=f"REPORT:{truth.run_id}:{r['cell_id']}:{r.get('sim_time_s', now)}",
                                detail={"note": r.get("note")})
        for feed in self.weather_feeds:
            for o in feed.observations(now, truth.scenario_start_kst):
                self.kb.add_weather(o["key"], o["sim_time_s"], o["source"], o["body"])

    def view(self):
        """판단용 화면: 지도(변하지 않는 정보) + 아는 세계 + 분석. 불·위험·기상 진짜 값은 들어가지 않는다."""
        truth = self.env.read()
        self._sync_knowledge(truth)
        now = truth.simulation_time_s or 0.0
        cells = {c["cell_id"]: c for c in getattr(self.env, "map_cells", lambda: [])()}
        fires = [{**cells.get(cid, {"cell_id": cid}), "fire_state": "BURNING", "risk_score": None,
                  "knowledge": k} for cid, k in self.kb.known_fires(now).items()]
        belief = {"run_id": truth.run_id, "simulation_time_s": now, "known_fires": fires,
                  "fire_states": self.kb.fire_states(now), "station_weather": self.kb.latest_station_obs(now)}
        an = self.analysis.analyze(belief) if self.analysis else {}

        def geo(c):                           # 분석 결과 칸에 지도 정보를 붙인다
            return {**cells.get(c["cell_id"], {}), **c}
        return dataclasses.replace(
            truth, fire_cells=fires, risk_cells=[geo(c) for c in an.get("risk_cells", [])],
            spread_forecast=[geo(c) for c in an.get("spread_forecast", [])], forecast_ref=an.get("forecast_ref"),
            wind_ms=None, wind_dir_deg=None, weather={}, source="ORCHESTRATOR_VIEW",
            analysis_source=an.get("source"))

    def view_for(self, task: Task, base=None):
        """Task 판단용 화면: 목표 칸의 현장 측정 → 가까운 관측소 관측으로 풍속·풍향을 채운다."""
        v = base or self.view()
        w = self.kb.weather_for(task.target.lat, task.target.lon, task.target.cell_id, v.simulation_time_s or 0.0)
        if not w:
            return dataclasses.replace(v, wind_ref={"basis": "NO_WEATHER_KNOWLEDGE"})
        vals = w.get("values") or {}
        ref = {k: w.get(k) for k in ("basis", "source", "station_id", "station_name", "cell_id", "distance_m",
                                     "sim_time_s", "observed_kst")}
        return dataclasses.replace(v, wind_ms=vals.get("wind_ms"), wind_dir_deg=vals.get("wind_dir_deg"),
                                   weather={k: vals[k] for k in ("temperature_c", "humidity_pct") if k in vals},
                                   wind_ref=ref)

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
            body["wind_ms"] = snap.wind_ms
            otype = config.LOCAL_OBSERVATION_TYPE.get(task.requirements.sensor or "THERMAL", "THERMAL")
            if otype:
                body["observation_type"] = otype
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

        snap = self.view_for(task)
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
            "snapshot": snap.ref(), "wind_ref": snap.wind_ref, "wind_ms": snap.wind_ms,
            "candidates": [[v.resource_id, round(d, 1)] for d, v in ranked],
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

    # ------------------------------------------------------------------
    # 여러 Task 배정 (우선순위 층 순서)
    # ------------------------------------------------------------------
    def _candidate_ids(self, task: Task, snap) -> set:
        views, unreachable, client_of = self._collect(task.requirements.resource_types)
        ranked, _ = filter_candidates(task, snap, views, unreachable, self.ledger.reservations(),
                                      age_of=lambda v: client_of[v.resource_id].source_age_s(v))
        return {v.resource_id for _, v in ranked}

    def set_priority_mode(self, mode: str, reason: str = "") -> dict:
        if mode not in config.PRIORITY_MODES:
            return {"status": "REFUSED", "reason": "UNKNOWN_MODE", "modes": list(config.PRIORITY_MODES)}
        self.ledger.log("PRIORITY_MODE_CHANGED", result=mode, reason=reason or None,
                        detail={"previous": self.priority_mode})
        self.priority_mode = mode
        return {"status": "OK", "mode": mode}

    def dispatch_pending(self) -> dict:
        """대기 Task 를 우선순위 단위(unit) 순서로 배정한다 (priority.order_tasks).

        선택 묶음(비슷함·엇갈림)의 처리는 모드에 따른다.
          HUMAN_CHOICE     묶음이 쓸 수 있는 자원이 묶음 크기보다 적으면 묶음 전체를 사람 선택 대기로 보류.
                           자원이 충분하면 모두 보낸다 (선택할 필요 없음). 사전 점검 뒤 같은 묶음 Task 가
                           자원을 가져가 밀린 경우도 보류.
          AUTO_HIGHER_RISK 인명 → 위험도가 조금이라도 높은 순으로 바로 보내고, 묶음을 '자동 결정'으로 기록.
        """
        snap = self.view()
        self._preemptive_monitor(snap)
        tasks = [t for t in self.ledger.list_tasks(["PENDING", "HOLD"])
                 if not (t.purpose_status == "HOLD" and t.resume_condition == MANUAL)]
        if not tasks:
            return {"mode": self.priority_mode, "units": [], "groups": [], "results": {}}
        plan = priority.order_tasks(tasks, snap, config.PRIORITY_SIMILAR_RISK_DELTA)
        by_id = {t.task_id: t for t in tasks}
        llm_applied = self._llm_recommend(snap, plan, by_id)
        group_of = {tid: g for g in plan["groups"] for tid in g["task_ids"]}
        results, group_status = {}, {}
        for unit in plan["units"]:
            g = group_of.get(unit[0])
            if g and self.priority_mode == "HUMAN_CHOICE":
                pool = set().union(*(self._candidate_ids(by_id[t], self.view_for(by_id[t], snap)) for t in unit))
                if len(pool) < len(unit):
                    for tid in unit:
                        results[tid] = self._hold(by_id[tid], "AWAITING_PRIORITY_CHOICE", MANUAL,
                                                  snap.simulation_time_s,
                                                  detail={"group": unit, "kinds": g["kinds"],
                                                          "available_resources": sorted(pool),
                                                          "evidence": {t: plan["evidence"][t] for t in unit}})
                    group_status[tuple(g["task_ids"])] = "AWAITING_CHOICE"
                    continue
            for tid in unit:
                results[tid] = self.dispatch(tid)
            if g:
                taken = {r.get("resource_id") for r in (results[t] for t in unit) if r.get("resource_id")}
                starved = [t for t in unit if results[t].get("status") == "HOLD" and any(
                    reason == "OCCUPIED" and rid in taken for rid, reason in (results[t].get("excluded") or {}).items())]
                if self.priority_mode == "HUMAN_CHOICE" and starved:
                    for tid in starved:
                        results[tid] = self._hold(self.ledger.get_task(tid), "AWAITING_PRIORITY_CHOICE", MANUAL,
                                                  snap.simulation_time_s,
                                                  detail={"group": unit, "taken_by_group_mates": sorted(taken)})
                    group_status[tuple(g["task_ids"])] = "PARTIAL_AWAITING_CHOICE"
                elif self.priority_mode == "AUTO_HIGHER_RISK":
                    group_status[tuple(g["task_ids"])] = "LLM_DECIDED" if llm_applied else "AUTO_DECIDED"
                else:
                    group_status[tuple(g["task_ids"])] = "ALL_DISPATCHED"
        groups = [{**g, "status": group_status.get(tuple(g["task_ids"]), "UNKNOWN")} for g in plan["groups"]]
        self.ledger.log("PRIORITY_ORDER", sim_time_s=snap.simulation_time_s, result=self.priority_mode,
                        detail={"mode": self.priority_mode, "groups": groups,
                                **{k: plan[k] for k in ("auto_order", "units", "criteria", "similar_delta",
                                                        "rule", "evidence", "context")}})
        return {"mode": self.priority_mode, "units": plan["units"], "groups": groups, "results": results}

    def _llm_recommend(self, snap, plan: dict, by_id: dict) -> bool:
        """선택 묶음이 있으면 LLM 에 순서를 제안받아 검증한다. 검증 통과 시 묶음에 추천을 붙이고,
        자동 모드에서는 그 순서로 배정 단위를 바꾼다. 실패하면 규칙 순서를 그대로 쓴다 (fallback)."""
        if not plan["groups"] or self.llm is None:
            return False
        prop = self.llm.propose(snap, plan, by_id)
        self.ledger.log("LLM_PLAN", sim_time_s=snap.simulation_time_s, result=prop["status"],
                        reason=prop.get("reason") or (",".join(prop.get("violations", [])) or None),
                        detail={k: v for k, v in prop.items() if k not in ("orders", "rationales")})
        if prop["status"] != "OK":
            for g in plan["groups"]:
                g["llm"] = {"status": prop["status"], "fallback": "RULE_ORDER",
                            "reason": prop.get("reason") or prop.get("violations")}
            return False
        for i, g in enumerate(plan["groups"]):
            g["llm"] = {"status": "OK", "order": prop["orders"][i], "rationale": prop["rationales"][i],
                        "input_refs": prop["input_refs"][i], "model": prop["model"]}
        if self.priority_mode == "AUTO_HIGHER_RISK":
            for g in plan["groups"]:
                for unit in plan["units"]:
                    if set(unit) == set(g["task_ids"]):
                        unit[:] = g["llm"]["order"]
        return True

    def choose_priority(self, order: List[str], reason: str) -> dict:
        """사람이 선택 묶음에서 먼저 보낼 순서를 정함. 목록의 Task 만 순서대로 배정하고, 빠진 Task 는 대기 유지.
        사람이 골라도 Local·Safety 관문은 그대로 거친다."""
        results = {}
        self.ledger.log("MANUAL_PRIORITY_CHOICE", result="ORDER", reason=reason, detail={"order": order})
        for tid in order:
            t = self.ledger.get_task(tid)
            if t is None or t.purpose_status != "HOLD" or t.hold_reason != "AWAITING_PRIORITY_CHOICE":
                results[tid] = {"status": "REFUSED", "reason": "NOT_AWAITING_PRIORITY_CHOICE"}
                continue
            t.purpose_status, t.resume_condition = "PENDING", None
            t.hold_reason = "HUMAN_PRIORITY_CHOSEN"
            self.ledger.save_task(t)
            results[tid] = self.dispatch(tid)
        return {"results": results}

    def _approve_and_send(self, task, view, client, snap, did, evaluated, local, mods) -> dict:
        rid = view.resource_id
        attempt_id = new_id("ATT")
        command = self._payload(task, view, snap, did, attempt_id, mods)
        current = self.view_for(task)
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
        sensor = task.requirements.sensor
        pos = ((data.get("observation") or {}).get("position")) or {}
        if sensor == "WEATHER":
            snap = self.env.read()
            obs = observation.simulate_weather(snapshot=snap, task=task, attempt_id=aid, resource_id=rid,
                                               position=pos, provider_raw=data.get("observation"))
        elif att["command"]["resource_type"] != "UAV":
            return self._ground_observation(att, task, data, detail)
        elif sensor != "THERMAL":
            return None
        else:
            snap = self.env.read()
            obs = observation.simulate(snapshot=snap, task=task, attempt_id=aid, resource_id=rid,
                                       position=pos, uav_raw_observation=data.get("observation"))
        self.ledger.log("OBSERVATION", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                        result=obs["result"], reason=obs.get("failure_reason"), sim_time_s=snap.simulation_time_s,
                        detail=obs)
        if sensor == "WEATHER" and obs["result"] == "MEASURED":
            self.kb.add_weather(f"FIELD:{obs['observation_id']}", snap.simulation_time_s, obs["source"], {
                "kind": "FIELD", "cell_id": task.target.cell_id, "lat": pos.get("lat"), "lon": pos.get("lon"),
                "values": obs["values"], "scope": obs.get("scope"), "observation_id": obs["observation_id"],
                "resource_id": rid})
        elif sensor == "THERMAL" and obs["result"] != "FAILED":
            self.kb.record_observation(obs, snap.simulation_time_s)
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
        if obs["result"] == "DETECTED":
            self._auto_sense(task, obs, self.view())
        return {"attempt_id": aid, "substatus": sub, "task": task.purpose_status}

    # ------------------------------------------------------------------
    # 현장 환경 측정 자동 생성 (사용자 결정 2026-09-30)
    # ------------------------------------------------------------------
    @staticmethod
    def _bearing_deg(lat1, lon1, lat2, lon2) -> float:
        import math
        p1, p2, dl = math.radians(lat1), math.radians(lat2), math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        return (math.degrees(math.atan2(x, y)) + 360) % 360

    def sense_points(self, fire_cell: dict, snap) -> dict:
        """불난 칸 + 바람이 향하는 쪽 위험 칸(환경이 표시한 위험 칸 중 방위가 가장 가까운 것)."""
        points, note = [fire_cell], None
        if snap.wind_dir_deg is None:
            note = "WIND_DIRECTION_MISSING"
        elif not snap.risk_cells:
            note = "NO_RISK_CELLS"
        else:
            toward = (snap.wind_dir_deg + 180.0) % 360.0
            best = None
            for c in snap.risk_cells:
                if c.get("lat") is None or c.get("lon") is None:
                    continue
                b = self._bearing_deg(fire_cell["lat"], fire_cell["lon"], c["lat"], c["lon"])
                diff = min(abs(b - toward), 360 - abs(b - toward))
                key = (diff, -(c.get("risk_score") or 0.0), c["cell_id"])
                if best is None or key < best[0]:
                    best = (key, c, round(diff, 1))
            if best:
                points.append(best[1])
                note = {"downwind_bearing_deg": round(toward, 1), "angle_off_deg": best[2]}
        return {"points": points, "note": note}

    def _auto_sense(self, task: Task, obs: dict, snap) -> List[str]:
        if not config.AUTO_ENV_SENSE_ENABLED or task.kind == "ENV_SENSE":
            return []
        cells = {c["cell_id"]: c for c in snap.fire_cells + snap.risk_cells}
        created = []
        for det in obs.get("detections", []):
            fire = cells.get(det["cell_id"])
            if not fire or fire.get("lat") is None:
                continue
            # 풍향은 아는 세계에서: 불난 칸 현장 측정 → 가장 가까운 관측소 (진짜 풍향을 보지 않음)
            w = self.kb.weather_for(fire["lat"], fire["lon"], fire["cell_id"], snap.simulation_time_s or 0.0) or {}
            sp = self.sense_points(fire, dataclasses.replace(
                snap, wind_dir_deg=(w.get("values") or {}).get("wind_dir_deg")))
            sp["wind_ref"] = {k: w.get(k) for k in ("basis", "source", "station_id", "distance_m", "sim_time_s")}
            for c in sp["points"]:
                t, new = self.submit_task({
                    "request_id": f"AUTO-SENSE:{snap.run_id}:{c['cell_id']}", "incident_id": task.incident_id,
                    "kind": "ENV_SENSE",
                    "target": {"lat": c["lat"], "lon": c["lon"], "ground_amsl_m": c.get("ground_amsl_m"),
                               "cell_id": c["cell_id"]},
                    "requirements": {"resource_types": ["UAV", "UGV", "FIRE_ENGINE"], "sensor": "WEATHER",
                                     "needs_env_ack": True}})
                if new:
                    created.append(t.task_id)
            self.ledger.log("ENV_SENSE_PLANNED", task_id=task.task_id, result="CREATED" if created else "EXISTING",
                            detail={"fire_cell": fire["cell_id"], "points": [c["cell_id"] for c in sp["points"]],
                                    "selection": sp["note"], "wind_ref": sp["wind_ref"], "created": created})
        return created

    # ------------------------------------------------------------------
    # 확산 예측 → 사전 감시 임무 (구현 완료, config.PREEMPTIVE_MONITOR_ENABLED 로 꺼 둠)
    # ------------------------------------------------------------------
    def preemptive_candidates(self, snap) -> List[dict]:
        """예측이 주거지·사람 있는 보호대상에 닿는 곳. 보호대상(없으면 주거 셀)마다 하나로 묶는다."""
        ctx = priority.risk_context(snap)
        by_key = {}
        site_of_cell = {th["cell_id"]: th["site_id"] for th in ctx.get("forecast_threats", []) if th.get("site_id")}
        for th in ctx.get("forecast_threats", []):
            # 보호대상 안에 있는 주거 칸은 그 보호대상 묶음으로 (같은 곳을 두 번 감시하지 않음)
            key = th.get("site_id") or site_of_cell.get(th["cell_id"]) or th["cell_id"]
            cur = by_key.get(key)
            if cur is None:
                cur = by_key[key] = {"key": key, "site_id": th.get("site_id") or site_of_cell.get(th["cell_id"]), "cells": [], "basis": set(),
                                     "earliest_arrival_s": None, "target_cell": None}
            if th["cell_id"] not in cur["cells"]:
                cur["cells"].append(th["cell_id"])
            cur["basis"].add(th["basis"])
            eta = th.get("expected_arrival_s")
            if eta is not None and (cur["earliest_arrival_s"] is None or eta < cur["earliest_arrival_s"]):
                cur["earliest_arrival_s"], cur["target_cell"] = eta, th["cell_id"]
            cur["target_cell"] = cur["target_cell"] or th["cell_id"]
        out = []
        for c in sorted(by_key.values(), key=lambda c: (c["earliest_arrival_s"] is None,
                                                        c["earliest_arrival_s"] or 0, c["key"])):
            c["basis"] = sorted(c["basis"])
            out.append(c)
        return out

    def _preemptive_monitor(self, snap) -> dict:
        cands = self.preemptive_candidates(snap)
        if not cands:
            return {"enabled": config.PREEMPTIVE_MONITOR_ENABLED, "candidates": []}
        sig = (snap.run_id, snap.state_version, tuple(c["key"] for c in cands))
        if not config.PREEMPTIVE_MONITOR_ENABLED:
            if getattr(self, "_premon_sig", None) != sig:
                self._premon_sig = sig
                self.ledger.log("PREEMPTIVE_MONITOR_SUPPRESSED", sim_time_s=snap.simulation_time_s,
                                reason="DISABLED_BY_CONFIG", detail={"candidates": cands,
                                                                     "forecast": snap.forecast_ref})
            return {"enabled": False, "candidates": cands}
        cells = {c["cell_id"]: c for c in snap.fire_cells + snap.risk_cells + snap.spread_forecast}
        created = []
        for c in cands:
            tc = cells.get(c["target_cell"]) or {}
            if tc.get("lat") is None:
                continue
            t, new = self.submit_task({
                "request_id": f"AUTO-PREMON:{snap.run_id}:{c['key']}", "incident_id": "AUTO",
                "kind": "MONITOR", "area_cell_ids": c["cells"],
                "target": {"lat": tc["lat"], "lon": tc["lon"], "ground_amsl_m": tc.get("ground_amsl_m"),
                           "cell_id": c["target_cell"]},
                "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL", "needs_env_ack": True}})
            if new:
                created.append(t.task_id)
        if created:
            self.ledger.log("PREEMPTIVE_MONITOR_CREATED", sim_time_s=snap.simulation_time_s,
                            detail={"created": created, "candidates": cands})
        return {"enabled": True, "candidates": cands, "created": created}

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
