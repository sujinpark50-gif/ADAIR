# -*- coding: utf-8 -*-
"""
orchestrator/engine.py
======================
총괄 폐루프 (규칙 기반). LLM 제안은 이 위에 얹되 같은 검증 관문을 거친다.

dispatch(task_id)  Task 하나: snapshot → 사전필터 → Local 평가/협상 → Safety → 예약·선저장 → 1회 송신
poll()             진행 중 실행시도 추적: 도착 → 모의 관측 → 환경 APPLY → 완료 → 복귀·READY 확인 후 해제
resolve(...)       사람의 수동 해소 (재평가도 Local·Safety 관문을 다시 거친다)
ingest_event(...)  외부 이벤트 접수: 검증 → 접수키 확정 → 반영 (신고면 최초 정찰 Task 멱등 생성)

금지 사항 (요청서 §5·§6)
- 송신 결과를 모르면(UNKNOWN) 재전송·재출동하지 않고 점유를 유지한다.
- UAV 의 COMPLETED 만으로 Task 완료나 자원 반납을 하지 않는다.
- 도착만으로 관측 완료, 관측만으로 진화 효과를 만들지 않는다.

상태 두 축 (2026-09-30 B01~B03)
- Task 목적 상태는 _set_purpose 한 곳에서만 바꾼다 (허용 전이는 models.PURPOSE_TRANSITIONS, 장부가 다시 검증).
  종료된 목적(COMPLETED/CANCELLED)은 늦게 온 관측·완료 응답, RETRY, idle 확인으로 되살아나지 않는다.
- 실행시도(attempt)의 물리 진행(도착·복귀·반납)은 목적 상태와 따로 끝까지 추적한다. 취소해도 점유는 그대로다.
- 제공자 응답은 실행 식별자·자원 ID·상태 값을 검증한 뒤에만 쓴다. 검증 실패는 UNKNOWN(점유 유지)이다.

실행(run) 범위 (B05): 활성 run 은 하나. 신고·관측·기상·Task·이벤트 키는 그 run 안에서만 쓴다.
다른 run 의 점유가 남아 있으면 run 을 바꾸지 않고 판단을 보류한다 (_ensure_run).
"""

import dataclasses
import math
import time
from typing import Dict, List, Optional, Tuple

from . import config, llm as llm_mod, negotiation, observation, priority, safety
from .env_adapter import ENV_TRANSIENT_ERRORS, missing_snapshot_fields
from .knowledge import Knowledge
from .ledger import (ACTIVE, OCCUPYING_SUBSTATUSES, AttemptConflict, Ledger, ReservationConflict, RunClosed,
                     RunNotActive, RunSwitchBlocked, content_hash, new_id)
from .models import (AREA_KINDS, COMPLETION_AREA_ALL, COMPLETION_TARGET_POINT, MISSION_OPEN_SUBSTATUSES,
                     TERMINAL_PURPOSES, Requirements, ResourceView, Target, Task, can_transition)
from .prefilter import filter_candidates
from .resources import Unreachable

# PREPARED 도 추적한다: 송신 직전·직후 총괄이 종료되면 재시작 후 이 상태로 남는다 (복구 대상)
ACTIVE_ATTEMPT = ("PREPARED", "REQUESTED", "STARTED", "ARRIVED", "OBSERVED", "APPLIED", "RETURNING", "UNKNOWN", "FAULTED")
# 임무 부분이 끝났고 기체만 점유 중인 상태 (불명에서 다시 연락되면 이 상태로 되돌린다)
POST_MISSION = ("OBSERVED", "APPLIED", "RETURNING", "FAULTED")
MANUAL = "MANUAL_RESOLUTION"
MAX_SNAPSHOT_RESTARTS = 2
_KEEP = object()

# 제공자 상태 조회 응답 계약 (uav/uav-agent/models.py TaskStatus, ugv/api_models.py TaskStatus)
#   task_id 칸에는 총괄이 보낸 실행 식별자(attempt_id)가 돌아온다. UGV 서버는 resource_id 도 준다.
PROVIDER_STATUSES = {"UAV": {"STARTED", "IN_PROGRESS", "COMPLETED", "FAILED"},
                     "GROUND": {"STARTED", "IN_PROGRESS", "COMPLETED", "FAILED", "CANCELLED"}}
EVENT_TYPES_THAT_REDISPATCH = ("ENV_UPDATED", "RESOURCE_CHANGED", "FIRE_REPORT")


class TaskRejected(Exception):
    """접수 단계에서 거절한 요청 (지원하지 않는 조합 등). 사유 코드를 담는다."""


class Orchestrator:
    def __init__(self, ledger: Ledger, env, uav=None, ugv=None, llm=None, analysis=None, weather_feeds=()):
        self.ledger, self.env, self.uav, self.ugv, self.llm = ledger, env, uav, ugv, llm
        self.kb = Knowledge(ledger)
        self.analysis = analysis              # 위험 칸·확산 예측 (입력은 총괄이 아는 세계만)
        self.weather_feeds = list(weather_feeds)
        self.priority_mode = config.PRIORITY_MODE
        self._llm_memo = {}                   # 추천 입력 해시 → 추천 (같은 입력이면 재사용, 호출 절약)
        self._synced = set()                  # 이미 장부에 넣은 신고·기상 키 (같은 run 안에서 다시 쓰지 않음)
        self._gate_logged = None

    # ------------------------------------------------------------------
    # 목적 상태 전이 (공통 경계)
    # ------------------------------------------------------------------
    def _set_purpose(self, task: Task, new: str, *, basis: str, hold_reason=_KEEP, resume=_KEEP,
                     attempt_id: Optional[str] = None, sim=None) -> bool:
        """Task 목적 상태를 바꾼다. 허용되지 않은 전이면 바꾸지 않고 거절 사유를 기록한 뒤 False.
        basis 는 전이 사유(근거 사건) 코드다."""
        stored = self.ledger.get_task(task.task_id)
        current = stored.purpose_status if stored else task.purpose_status
        # 실행시도에서 온 사건은 그 시도가 지금 목적 담당일 때만 목적을 바꾼다 (R01: 인계 뒤의 이전 시도 차단)
        owner = stored.owner_attempt_id if stored else None
        not_owner = bool(attempt_id and owner and owner != attempt_id)
        if not_owner or not can_transition(current, new):
            self.ledger.log("PURPOSE_TRANSITION_REJECTED", task_id=task.task_id, attempt_id=attempt_id,
                            result=f"{current}->{new}", reason=basis, sim_time_s=sim,
                            detail={"current": current, "requested": new, "kept": current,
                                    "why": "NOT_PURPOSE_OWNER" if not_owner else "TRANSITION_NOT_ALLOWED",
                                    "owner_attempt_id": owner})
            task.purpose_status = current
            if stored:
                task.hold_reason, task.resume_condition = stored.hold_reason, stored.resume_condition
            return False
        task.purpose_status = new
        if hold_reason is not _KEEP:
            task.hold_reason = hold_reason
        if resume is not _KEEP:
            task.resume_condition = resume
        self.ledger.save_task(task)
        if current != new:
            self.ledger.log("PURPOSE_TRANSITION", task_id=task.task_id, attempt_id=attempt_id,
                            result=f"{current}->{new}", reason=basis, sim_time_s=sim,
                            detail={"hold_reason": task.hold_reason, "resume_condition": task.resume_condition})
        return True

    @staticmethod
    def _owns(att: dict, task: Optional[Task]) -> bool:
        """이 실행시도가 지금 Task 목적을 맡고 있는지 (담당 기록이 없는 이전 장부의 Task 는 맡은 것으로 본다)"""
        return task is not None and task.owner_attempt_id in (None, att["attempt_id"])

    # ------------------------------------------------------------------
    # 실행(run) 범위
    # ------------------------------------------------------------------
    def _ensure_run(self, truth) -> Optional[str]:
        """이 snapshot 을 판단 입력으로 쓸 수 있으면 None, 아니면 사유 코드.
        새 run 이면 활성화한다. 다른 run 의 점유가 남아 있으면 바꾸지 않는다 (추적은 poll 이 계속한다)."""
        if truth.run_id is None:
            return "RUN_ID_MISSING"
        active = self.ledger.active_run()
        if truth.run_id != active:
            try:
                self.ledger.activate_run(truth.run_id)
            except RunSwitchBlocked as e:
                return self._gate("RUN_SWITCH_BLOCKED", truth, {"active_run": active, "blocking": str(e)})
            except RunClosed:
                return self._gate("RUN_CLOSED", truth, {"active_run": active})
            self._llm_memo, self._synced, self._gate_logged = {}, set(), None
            self._premon_sig = None
            # 이전 run 의 환경 반영 대기는 새 run 의 환경에 적용하지 않는다. 지우지 않고 사유와 함께 닫는다 (R04)
            closed = self.ledger.close_pending_applies_of_other_runs(truth.run_id, "RUN_ENDED")
            self.ledger.log("RUN_ACTIVATED", result=truth.run_id, sim_time_s=truth.simulation_time_s,
                            detail={"previous_run": active, "policy": "SINGLE_ACTIVE_RUN",
                                    "pending_env_applies_closed": closed,
                                    "legacy_rows_quarantined": self.ledger.migrated_from_legacy})
        if truth.state_version is not None:
            if self.ledger.note_state_version(truth.run_id, truth.state_version) == "STALE":
                return self._gate("SNAPSHOT_STALE", truth, {"state_version": truth.state_version})
        self._gate_logged = None
        return None

    def _gate(self, reason: str, truth, detail: dict) -> str:
        sig = (reason, truth.run_id, truth.state_version if reason == "SNAPSHOT_STALE" else None)
        if self._gate_logged != sig:                     # 같은 사유는 한 번만 기록
            self._gate_logged = sig
            self.ledger.log("RUN_GATE", result=reason, sim_time_s=truth.simulation_time_s,
                            detail={"snapshot_run": truth.run_id, **detail})
        return reason

    # ------------------------------------------------------------------
    # 접수
    # ------------------------------------------------------------------
    @staticmethod
    def _completion_scope(body: dict, kind: str, sensor, inherit: Optional[Task]) -> Tuple[str, List[str]]:
        """완료 규칙과 필수 셀 (D03). 정찰류는 목표 지점, 구역 감시는 필수 셀 전체."""
        given = list(body.get("required_cell_ids") or [])
        area = list(body.get("area_cell_ids") or [])
        cell = (body.get("target") or {}).get("cell_id")
        rule = inherit.completion_rule if inherit else (
            COMPLETION_AREA_ALL if kind in AREA_KINDS else COMPLETION_TARGET_POINT)
        if rule == COMPLETION_TARGET_POINT:
            if given:
                raise TaskRejected("REQUIRED_CELLS_ONLY_FOR_AREA_TASKS")
            return rule, []
        if sensor != "THERMAL":
            raise TaskRejected(f"AREA_EVIDENCE_UNSUPPORTED_FOR_SENSOR:{sensor}")
        required = given or area or ([cell] if cell else [])
        if not required:
            raise TaskRejected("AREA_REQUIRED_CELLS_MISSING")
        return rule, list(dict.fromkeys(required))

    def submit_task(self, body: dict, _inherit: Optional[Task] = None):
        """body: {request_id, incident_id, kind, target{lat,lon,ground_amsl_m?,cell_id?}, requirements{},
                  area_cell_ids?, required_cell_ids?, run_id?}"""
        snap = self.env.read()
        gate = self._ensure_run(snap)
        if gate:
            raise RunNotActive(gate)
        if body.get("run_id") and body["run_id"] != self.ledger.active_run():
            raise RunNotActive(f"RUN_MISMATCH:{body['run_id']}")
        kind = body.get("kind") or "RECON"
        req = dict(body.get("requirements") or {})
        sensor = req.get("sensor", "THERMAL")
        ack = req.get("needs_env_ack")
        if ack is None:
            ack = sensor in config.ENV_ACK_SUPPORTED_SENSORS     # 반영 계약이 있는 관측만 ACK 를 요구
        elif ack and sensor not in config.ENV_ACK_SUPPORTED_SENSORS:
            raise TaskRejected(f"ENV_ACK_UNSUPPORTED_FOR_SENSOR:{sensor}")
        req["needs_env_ack"] = bool(ack)
        rule, required = self._completion_scope(body, kind, sensor, _inherit)
        task = Task(
            task_id=new_id("TASK"), incident_id=body.get("incident_id") or "INC-DEFAULT",
            kind=kind, target=Target(**body["target"]),
            requirements=Requirements(**{k: (tuple(v) if k in ("resource_types", "agl_range_m", "extra_sensors")
                                             and v is not None
                                             else v) for k, v in req.items()}),
            request_key=body.get("request_id"),
            area_cell_ids=list(body.get("area_cell_ids") or []),
            completion_rule=rule, required_cell_ids=required,
            parent_task_id=body.get("parent_task_id"),
        )
        task.created_sim_s = snap.simulation_time_s
        task, created = self.ledger.create_task(task, body)
        if created:
            self._merge_pending_sense_into(task)
        self.ledger.log("TASK_RECEIVED", task_id=task.task_id, result="CREATED" if created else "DUPLICATE",
                        detail={"request_id": task.request_key, "completion_rule": task.completion_rule,
                                "required_cell_ids": task.required_cell_ids,
                                "needs_env_ack": task.requirements.needs_env_ack},
                        sim_time_s=snap.simulation_time_s)
        return task, created

    # ------------------------------------------------------------------
    # 외부 이벤트 (B04): 검증 → 접수키 확정 → 반영. 거절한 요청은 접수키를 쓰지 않는다.
    # ------------------------------------------------------------------
    @staticmethod
    def _event_problems(body: dict, now: float) -> List[str]:
        p = []
        if not isinstance(body.get("event_id"), str) or not body["event_id"].strip():
            p.append("EVENT_ID_MISSING")
        if not isinstance(body.get("type"), str) or not body["type"].strip():
            p.append("TYPE_MISSING")
        t = body.get("simulation_time_s")
        if t is not None:
            if isinstance(t, bool) or not isinstance(t, (int, float)) or not math.isfinite(t) or t < 0:
                p.append("SIMULATION_TIME_INVALID")
            elif t > now:
                p.append("SIMULATION_TIME_IN_FUTURE")
        payload = body.get("payload")
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            p.append("PAYLOAD_NOT_OBJECT")
        elif body.get("type") == "FIRE_REPORT":
            cid = payload.get("cell_id")
            if not isinstance(cid, str) or not cid.strip():
                p.append("CELL_ID_MISSING")
            if payload.get("source") is not None and not isinstance(payload["source"], str):
                p.append("SOURCE_NOT_STRING")
        return p

    def ingest_event(self, body: dict) -> Tuple[int, dict]:
        """(HTTP 상태, 응답). 202 접수·반영 / 200 같은 내용 재전송 / 409 같은 ID 다른 내용·run 불일치 / 422 거절."""
        truth = self.env.read()
        now = truth.simulation_time_s or 0.0
        problems = self._event_problems(body, now)
        if problems:
            self.ledger.log("EXTERNAL_EVENT_REJECTED", result=str(body.get("type")), reason=problems[0],
                            sim_time_s=now, detail={"event_id": body.get("event_id"), "problems": problems,
                                                    "idempotency_key_consumed": False})
            return 422, {"accepted": False, "reason": problems[0], "problems": problems, "simulation_time_s": now}
        gate = self._ensure_run(truth)
        if gate:
            return 409, {"accepted": False, "reason": f"RUN_GATE:{gate}", "active_run": self.ledger.active_run()}
        if body.get("run_id") and body["run_id"] != self.ledger.active_run():
            return 409, {"accepted": False, "reason": "RUN_MISMATCH", "active_run": self.ledger.active_run()}
        given = body.get("simulation_time_s")
        meaning = {"type": body["type"], "simulation_time_s": given, "payload": body.get("payload") or {}}
        stored = {**meaning, "effective_sim_time_s": now if given is None else given}
        status, row = self.ledger.receive_external_event(body["event_id"], content_hash(meaning), stored)
        if status == "CONFLICT":
            self.ledger.log("EXTERNAL_EVENT_CONFLICT", result=body["type"], reason=body["event_id"], sim_time_s=now,
                            detail={"stored_hash": row["content_hash"], "new_hash": content_hash(meaning)})
            return 409, {"accepted": False, "reason": "EVENT_ID_CONFLICT",
                         "detail": "같은 event_id 로 다른 내용이 이미 접수되어 있다"}
        if status == "DUPLICATE" and row["status"] == "APPLIED":
            return 200, {"accepted": True, "duplicate": True, "applied": row["result"]}
        applied = self._apply_event(row)       # 새 이벤트, 또는 접수만 되고 반영 전에 멈췄던 이벤트
        return 202, {"accepted": True, "duplicate": status == "DUPLICATE", "resumed": status == "DUPLICATE",
                     "applied": applied, "needs_dispatch": body["type"] in EVENT_TYPES_THAT_REDISPATCH}

    def _apply_event(self, row: dict) -> dict:
        """접수된 이벤트를 아는 세계·Task 에 반영한다. 각 단계가 키로 멱등이라 다시 불러도 같은 결과다."""
        body, eid = row["body"], row["event_id"]
        result = {"type": body["type"]}
        if body["type"] == "FIRE_REPORT":
            p = body["payload"]
            key = f"EVENT:{eid}"
            src = p.get("source", "EXTERNAL_REPORT")
            self.kb.report_fire(p["cell_id"], body["effective_sim_time_s"], src, key=key, detail={"note": p.get("note")})
            result["recon"] = self._initial_recon(p["cell_id"], key, body["effective_sim_time_s"], src)
        self.ledger.mark_external_event_applied(eid, result)
        self.ledger.log("EXTERNAL_EVENT", result=body["type"], reason=eid, sim_time_s=body["effective_sim_time_s"],
                        detail={**body, "applied": result})
        return result

    # ------------------------------------------------------------------
    # 신고 → 최초 정찰 Task (D01). 접수 성공은 출동 성공이 아니다 — 출동은 기존 배정·Safety·점유 조건을 따른다.
    # ------------------------------------------------------------------
    def _open_thermal_tasks_covering(self, cell_id: str) -> List[Task]:
        return [t for t in self.ledger.list_tasks(self.OPEN_PURPOSES)
                if t.requirements.sensor == "THERMAL"
                and (t.target.cell_id == cell_id or cell_id in t.required_cell_ids)]

    def _initial_recon(self, cell_id: str, report_key: str, sim_time_s: float, source: str) -> dict:
        """신고 한 건에 대한 처분을 한 번 정하고 기록한다 (같은 신고를 다시 처리해도 같은 결과).

        사건 ID 가 계약에 없으므로 대체키를 쓴다: INC-AUTO:{run}:{cell}:{그 칸의 n번째 사건}.
          - 그 칸을 볼 열화상 임무가 이미 열려 있으면   → 같은 사건의 중복 신고 (새 임무 없음)
          - 그 칸의 최신 관측이 '불 확인'이면            → 이미 확인된 사건 (새 임무 없음, 재관측은 RECHECK)
          - 그 밖 (처음, 관측 결과 불 없음, 이전 임무 종료) → 새 사건 → RECON 생성
        """
        if not config.AUTO_RECON_ON_REPORT:
            return {"decision": "DISABLED"}
        disp_key = f"DISPOSITION:{report_key}"
        prev = self.ledger.get_knowledge(disp_key)
        if prev:
            return prev["body"]
        run = self.ledger.active_run()
        request_id = f"AUTO-RECON:{run}:{report_key}"
        task = self.ledger.task_by_request_key(request_id)     # 임무는 만들었는데 처분 기록 전에 멈춘 경우
        if task is not None:
            out = {"decision": "NEW_INCIDENT", "task_id": task.task_id, "incident_id": task.incident_id}
        else:
            open_ = self._open_thermal_tasks_covering(cell_id)
            observed = [k for k in self.ledger.knowledge("FIRE")
                        if k["subject"] == cell_id and k["body"].get("observation_id")]
            if open_:
                out = {"decision": "DUPLICATE_OF_OPEN_TASK", "task_id": open_[0].task_id,
                       "incident_id": open_[0].incident_id}
            elif observed and observed[-1]["body"]["status"] == "CONFIRMED":
                out = {"decision": "ALREADY_CONFIRMED", "task_id": None,
                       "confirmed_by": observed[-1]["body"]["observation_id"]}
            else:
                n = 1 + sum(1 for k in self.ledger.knowledge("REPORT_DISPOSITION")
                            if k["subject"] == cell_id and k["body"].get("decision") == "NEW_INCIDENT")
                incident = f"INC-AUTO:{run}:{cell_id}:{n}"
                m = {c["cell_id"]: c for c in getattr(self.env, "map_cells", lambda: [])()}.get(cell_id, {})
                task, _ = self.submit_task({
                    "request_id": request_id, "incident_id": incident, "kind": "RECON",
                    # 지도에 없는 값은 비워 둔다 (좌표·지면고도를 지어내지 않음 → 배정 단계에서 사유와 함께 보류)
                    "target": {"lat": m.get("lat"), "lon": m.get("lon"), "ground_amsl_m": m.get("ground_amsl_m"),
                               "cell_id": cell_id},
                    "requirements": dict(config.AUTO_RECON_REQUIREMENTS)})
                out = {"decision": "NEW_INCIDENT", "task_id": task.task_id, "incident_id": incident}
        out.update({"cell_id": cell_id, "report_key": report_key, "report_source": source,
                    "incident_key_rule": "INC-AUTO:{run}:{cell}:{n}"})
        self.ledger.add_knowledge("REPORT_DISPOSITION", disp_key, cell_id, sim_time_s, "ORCHESTRATOR", out)
        self.ledger.log("INITIAL_RECON", task_id=out.get("task_id"), result=out["decision"], sim_time_s=sim_time_s,
                        detail=out)
        return out

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
    def _sync_knowledge(self, truth) -> List[str]:
        """그 시각까지의 신고·기상 관측을 아는 세계에 넣는다 (중복은 key 로 무시).
        신고가 새로 들어오면 최초 정찰 Task 를 만든다. 새로 만든 Task ID 목록을 돌려준다."""
        now = truth.simulation_time_s or 0.0
        created = []
        for r in getattr(self.env, "reports_until", lambda t: [])(now):
            t = r.get("sim_time_s", now)
            key = f"REPORT:{truth.run_id}:{r['cell_id']}:{t}"
            if key in self._synced:
                continue
            src = r.get("source", "REPORT")
            self.kb.report_fire(r["cell_id"], t, src, key=key, detail={"note": r.get("note")})
            out = self._initial_recon(r["cell_id"], key, t, src)
            if out.get("decision") == "NEW_INCIDENT" and out.get("task_id"):
                created.append(out["task_id"])
            self._synced.add(key)
        for feed in self.weather_feeds:
            for o in feed.observations(now, truth.scenario_start_kst):
                if o["key"] in self._synced:
                    continue
                self.kb.add_weather(o["key"], o["sim_time_s"], o["source"], o["body"], received_sim_s=now)
                self._synced.add(o["key"])
        return created

    def sync(self) -> dict:
        """환경을 읽어 아는 세계를 최신으로 맞춘다 (추적 루프가 부른다). 새로 생긴 임무가 있으면 created 에."""
        truth = self.env.read()
        gate = self._ensure_run(truth)
        if gate:
            return {"gate": gate, "created": []}
        return {"gate": None, "created": self._sync_knowledge(truth)}

    def view(self):
        """판단용 화면: 지도(변하지 않는 정보) + 아는 세계 + 분석. 불·위험·기상 진짜 값은 들어가지 않는다."""
        truth = self.env.read()
        gate = self._ensure_run(truth)
        if gate:
            # 이 snapshot 은 활성 run 의 것이 아니다 → 다른 run 의 지식과 섞지 않고 빈 화면 + 사유만 준다
            return dataclasses.replace(truth, fire_cells=[], risk_cells=[], spread_forecast=[], forecast_ref=None,
                                       wind_ms=None, wind_dir_deg=None, weather={}, source="ORCHESTRATOR_VIEW",
                                       run_gate=gate)
        self._sync_knowledge(truth)
        now = truth.simulation_time_s or 0.0
        cells = {c["cell_id"]: c for c in getattr(self.env, "map_cells", lambda: [])()}
        fires = [{**cells.get(cid, {"cell_id": cid}), "fire_state": "BURNING", "risk_score": None,
                  "knowledge": k} for cid, k in self.kb.known_fires(now).items()]
        field = self.kb.field_weather(now)
        # 분석 입력에는 지금 유효한 기상 항목만 넘긴다 (만료된 현장 측정 원본은 kb.field_history 에만 남는다)
        belief = {"run_id": truth.run_id, "simulation_time_s": now, "known_fires": fires,
                  "fire_states": self.kb.fire_states(now), "station_weather": self.kb.latest_station_obs(now),
                  "field_weather": field["valid"], "field_weather_excluded": field["excluded"],
                  "weather_policy": self.weather_policy()}
        an = self.analysis.analyze(belief) if self.analysis else {}

        def geo(c):                           # 분석 결과 칸에 지도 정보를 붙인다
            return {**cells.get(c["cell_id"], {}), **c}
        return dataclasses.replace(
            truth, fire_cells=fires, risk_cells=[geo(c) for c in an.get("risk_cells", [])],
            spread_forecast=[geo(c) for c in an.get("spread_forecast", [])], forecast_ref=an.get("forecast_ref"),
            wind_ms=None, wind_dir_deg=None, weather={}, source="ORCHESTRATOR_VIEW",
            analysis_source=an.get("source"))

    @staticmethod
    def weather_policy() -> dict:
        unset = sorted(k for k, v in config.FIELD_WEATHER_MAX_AGE_S.items() if v is None)
        return {"clock": "SIMULATION_OBSERVED_TIME", "field_max_age_s": dict(config.FIELD_WEATHER_MAX_AGE_S),
                "field_policy_unset_items": unset,
                "station_max_age": "자료 제공 간격 (provision_interval_s)",
                "order": "유효한 현장 측정 → 가장 가까운 관측소의 유효한 값 → UNKNOWN"}

    def view_for(self, task: Task, base=None):
        """Task 판단용 화면: 갈 지점의 기상을 항목별로 고른다 (유효한 현장 측정 → 가까운 관측소 → 없음)."""
        v = base or self.view()
        if v.run_gate:
            return dataclasses.replace(v, wind_ref={"basis": "RUN_GATE", "reason": v.run_gate})
        nav = task.nav_target
        w = self.kb.weather_for(nav.lat, nav.lon, nav.cell_id, v.simulation_time_s or 0.0)
        vals = w["values"]
        ref = {k: w.get(k) for k in ("basis", "source", "station_id", "station_name", "cell_id", "distance_m",
                                     "sim_time_s", "observed_kst")}
        ref["items"] = {i: {k: r.get(k) for k in ("value", "unit", "basis", "status", "source", "observed_sim_s",
                                                  "received_sim_s", "age_s", "max_age_s", "station_id", "cell_id",
                                                  "distance_m", "replaced")} for i, r in w["items"].items()}
        ref["unknown"], ref["remeasure"] = w["unknown"], w["remeasure"]
        return dataclasses.replace(v, wind_ms=vals.get("wind_ms"), wind_dir_deg=vals.get("wind_dir_deg"),
                                   weather={k: vals[k] for k in ("temperature_c", "humidity_pct") if k in vals},
                                   wind_ref=ref)

    # ------------------------------------------------------------------
    # 요청 payload
    # ------------------------------------------------------------------
    @staticmethod
    def _payload(task: Task, view: ResourceView, snap, decision_id: str, exec_id: str, mods: dict) -> dict:
        nav = task.nav_target
        tgt = {"lat": nav.lat, "lon": nav.lon}
        body = {"task_id": exec_id, "decision_id": decision_id, "target": tgt}
        if view.resource_type == "UAV":
            agl = mods.get("target_agl_m", task.requirements.target_agl_m or config.UAV_DEFAULT_TARGET_AGL_M)
            # 지면고도만 넣는다. 80m·10m 는 UAV 가 더한다 (요청서 §4.1)
            tgt.update({"alt_m_amsl": nav.ground_amsl_m, "target_agl_m": agl})
            body["wind_ms"] = snap.wind_ms
            otype = config.LOCAL_OBSERVATION_TYPE.get(task.requirements.sensor or "THERMAL", "THERMAL")
            if otype:
                body["observation_type"] = otype
        return body

    # ------------------------------------------------------------------
    # 배정
    # ------------------------------------------------------------------
    def _fill_target_from_map(self, task: Task) -> bool:
        """위치를 모르는 목표를 지도(정적 정보)로 채운다. 지도에도 없으면 False — 좌표를 만들지 않는다."""
        nav = task.nav_target
        if nav.lat is not None and nav.lon is not None:
            return True
        m = {c["cell_id"]: c for c in getattr(self.env, "map_cells", lambda: [])()}.get(nav.cell_id or "")
        if not m or m.get("lat") is None or m.get("lon") is None:
            return False
        nav.lat, nav.lon = m["lat"], m["lon"]
        if nav.ground_amsl_m is None:
            nav.ground_amsl_m = m.get("ground_amsl_m")
        self.ledger.save_task(task)
        return True

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
        open_ = self.ledger.open_mission_attempts(task_id)
        if open_:       # 임무 종료가 확인되지 않은 실행시도가 있으면 같은 Task 로 또 보내지 않는다 (B02·R01)
            return {"status": "SKIPPED", "reason": "ATTEMPT_STILL_OPEN", "attempt_id": open_[0]["attempt_id"],
                    "substatus": open_[0]["substatus"]}
        if self.ledger.pending_applies(task_id=task_id):
            # 관측은 했고 환경 반영 확인만 남았다 → 새 비행으로 풀지 않는다 (R04)
            return {"status": "SKIPPED", "reason": "ENV_APPLY_PENDING"}

        base = self.view()
        sim = base.simulation_time_s
        missing = missing_snapshot_fields(base)
        if missing:
            return self._hold(task, "SNAPSHOT_INCOMPLETE:" + ",".join(missing), "ENV_SNAPSHOT_COMPLETE", sim)
        if base.run_gate:
            return self._hold(task, f"RUN_GATE:{base.run_gate}", "RUN_ACTIVE", sim)
        if task.run_id != base.run_id:
            return {"status": "SKIPPED", "reason": "TASK_RUN_NOT_ACTIVE", "task_run": task.run_id,
                    "active_run": base.run_id}
        if not self._fill_target_from_map(task):
            return self._hold(task, "TARGET_LOCATION_UNKNOWN", "MAP_INFO", sim,
                              detail={"cell_id": task.nav_target.cell_id})
        snap = self.view_for(task, base)       # 갈 지점 기준으로 기상을 항목별로 고른다
        if config.WEATHER_REMEASURE_ENABLED and snap.wind_ref.get("remeasure"):
            self._request_remeasure(task, snap)
            task = self.ledger.get_task(task_id)

        if not self._set_purpose(task, "EVALUATING", basis="DISPATCH_EVALUATION", hold_reason=None, resume=None,
                                 sim=sim):
            return {"status": "SKIPPED", "purpose_status": task.purpose_status}

        views, unreachable, client_of = self._collect(task.requirements.resource_types)
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
                    if out.get("abort"):
                        return self._hold(task, out["abort"], MANUAL, sim, detail={"excluded": excluded})
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

    def _pending_plan(self):
        """(snap, 대기 Task 목록, 우선순위 계획, by_id). 대기 Task 가 없으면 plan 은 None."""
        snap = self.view()
        if snap.run_gate:
            return snap, [], None, {}
        self._preemptive_monitor(snap)
        tasks = [t for t in self.ledger.list_tasks(["PENDING", "HOLD"])
                 if not (t.purpose_status == "HOLD" and t.resume_condition == MANUAL)]
        if not tasks:
            return snap, [], None, {}
        plan = priority.order_tasks(tasks, snap, config.PRIORITY_SIMILAR_RISK_DELTA)
        return snap, tasks, plan, {t.task_id: t for t in tasks}

    def dispatch_pending(self, allow_llm_call: bool = True) -> dict:
        """대기 Task 를 우선순위 단위(unit) 순서로 배정한다 (priority.order_tasks).

        선택 묶음(비슷함·엇갈림)의 처리는 모드에 따른다.
          HUMAN_CHOICE     묶음이 쓸 수 있는 자원이 묶음 크기보다 적으면 묶음 전체를 사람 선택 대기로 보류.
                           자원이 충분하면 모두 보낸다 (선택할 필요 없음). 사전 점검 뒤 같은 묶음 Task 가
                           자원을 가져가 밀린 경우도 보류.
          AUTO_HIGHER_RISK 인명 → 위험도가 조금이라도 높은 순으로 바로 보내고, 묶음을 '자동 결정'으로 기록.

        allow_llm_call=False 이면 여기서 LLM 을 부르지 않는다 (서버는 llm_request/llm_call/llm_store 로
        긴 호출을 잠금 밖에서 하고, 그 결과가 지금 입력과 같을 때만 쓴다).
        """
        snap, tasks, plan, by_id = self._pending_plan()
        if snap.run_gate:
            return {"mode": self.priority_mode, "units": [], "groups": [], "results": {}, "run_gate": snap.run_gate}
        if not tasks:
            return {"mode": self.priority_mode, "units": [], "groups": [], "results": {}}
        llm_applied = self._llm_recommend(snap, plan, by_id, allow_llm_call)
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

    # ------------------------------------------------------------------
    # LLM 추천. 추천 재사용 키는 "실제로 LLM 에 주는 입력"의 해시다 — 위험점수·거리·공식 순위·묶음 구성·
    # run·환경 버전 중 하나라도 바뀌면 옛 추천을 쓰지 않는다.
    # ------------------------------------------------------------------
    def llm_request(self) -> Optional[dict]:
        """지금 대기 묶음에 LLM 호출이 필요하면 호출 재료, 아니면 None. 장부 잠금 안에서 부른다."""
        if self.llm is None or self.llm.status():           # 호출할 수 없으면(키 없음·한도 등) 재료도 만들지 않는다
            return None
        snap, tasks, plan, by_id = self._pending_plan()
        if not plan or not plan["groups"]:
            return None
        key = content_hash(llm_mod.build_input(snap, plan, by_id))
        if key in self._llm_memo:
            return None
        return {"key": key, "snap": snap, "plan": plan, "by_id": by_id}

    def llm_call(self, req: dict) -> dict:
        """LLM 호출 (길 수 있다). 장부를 건드리지 않으므로 잠금 밖에서 불러도 된다."""
        return self.llm.propose(req["snap"], req["plan"], req["by_id"])

    def llm_store(self, req: dict, prop: dict) -> None:
        """호출 결과를 입력 해시와 함께 보관한다. 적용은 dispatch_pending 이 그때의 입력으로 다시 확인하고 한다."""
        if prop["status"] in ("OK", "INVALID"):
            self._llm_memo = {req["key"]: prop}           # 최근 1건만 보관
        else:
            self._llm_once = {req["key"]: prop}           # 오류·사용 불가는 재사용하지 않고 이번 판단에만 알린다
        self.ledger.log("LLM_PLAN", sim_time_s=req["snap"].simulation_time_s, result=prop["status"],
                        reason=prop.get("reason") or (",".join(prop.get("violations", [])) or None),
                        detail={**{k: v for k, v in prop.items() if k not in ("orders", "rationales")},
                                "input_hash": req["key"]})

    def _llm_recommend(self, snap, plan: dict, by_id: dict, allow_call: bool = True) -> bool:
        """선택 묶음이 있으면 LLM 에 순서를 제안받아 검증한다. 검증 통과 시 묶음에 추천을 붙이고,
        자동 모드에서는 그 순서로 배정 단위를 바꾼다. 실패하면 규칙 순서를 그대로 쓴다 (fallback)."""
        if not plan["groups"] or self.llm is None:
            return False
        key = content_hash(llm_mod.build_input(snap, plan, by_id))
        prop = self._llm_memo.get(key)
        if prop is None:
            once, self._llm_once = getattr(self, "_llm_once", {}).get(key), {}
            if once is not None:
                prop = once                               # 방금 잠금 밖에서 부른 결과 (오류 등) — 이미 기록됨
            elif not allow_call and self.llm.status():
                prop = {"status": "DISABLED", "reason": self.llm.status()}
                self.ledger.log("LLM_PLAN", sim_time_s=snap.simulation_time_s, result="DISABLED",
                                reason=prop["reason"], detail={"input_hash": key})
            elif not allow_call:
                # 잠금 밖에서 받아 온 추천이 지금 입력(run·버전·묶음·근거)과 맞지 않거나 아직 없다 → 쓰지 않는다
                self.ledger.log("LLM_PLAN_NOT_APPLIED", sim_time_s=snap.simulation_time_s, result="RULE_ORDER",
                                reason="NO_PLAN_FOR_CURRENT_INPUT", detail={"input_hash": key})
                prop = {"status": "STALE", "reason": "NO_PLAN_FOR_CURRENT_INPUT"}
            else:
                req = {"key": key, "snap": snap, "plan": plan, "by_id": by_id}
                prop = self.llm_call(req)
                self.llm_store(req, prop)
        else:
            self.ledger.log("LLM_PLAN_REUSED", sim_time_s=snap.simulation_time_s, result=prop["status"],
                            detail={"input_hash": key})
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

    def manual_dispatch(self, task_id: str, reason: str) -> dict:
        """사람이 '지금 보내기'. 우선순위 순서를 건너뛰지만 Local·Safety 관문은 그대로 거친다.
        실행 결과가 불명(UNKNOWN)인 임무는 수동 해소(resolve)로만 다시 다룬다."""
        t = self.ledger.get_task(task_id)
        if t is None:
            return {"status": "NOT_FOUND"}
        self.ledger.log("MANUAL_DISPATCH", task_id=task_id, result="REQUESTED", reason=reason,
                        detail={"mode": self.priority_mode, "purpose_status": t.purpose_status})
        if t.purpose_status == "HOLD" and t.hold_reason == "AWAITING_PRIORITY_CHOICE":
            return self.choose_priority([task_id], reason)["results"][task_id]
        if t.purpose_status not in ("PENDING", "HOLD") or t.resume_condition == MANUAL:
            return {"status": "REFUSED", "reason": f"NOT_DISPATCHABLE:{t.purpose_status}:{t.hold_reason}"}
        self._set_purpose(t, "PENDING", basis="MANUAL_DISPATCH", hold_reason="HUMAN_DISPATCHED", resume=None)
        return self.dispatch(task_id)

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
            self._set_purpose(t, "PENDING", basis="MANUAL_PRIORITY_CHOICE", hold_reason="HUMAN_PRIORITY_CHOSEN",
                              resume=None)
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
        except AttemptConflict as e:
            # 같은 Task 에 임무가 열린 시도가 이미 있다 (또는 Task 가 종료됨) → 다른 후보로도 보내지 않는다
            self.ledger.log("ATTEMPT_REFUSED", task_id=task.task_id, decision_id=did, attempt_id=attempt_id,
                            resource_id=rid, reason="OPEN_ATTEMPT_OR_CLOSED_TASK", detail={"error": str(e)})
            return {"abort": "OPEN_ATTEMPT_EXISTS"}
        task.owner_attempt_id = attempt_id          # 이제부터 이 시도가 목적 담당 (장부에 저장 → 재시작 후에도 유지)
        self.ledger.save_task(task)
        self._set_purpose(task, "APPROVED", basis="SAFETY_ALLOW_RESERVED", attempt_id=attempt_id,
                          sim=current.simulation_time_s)
        result, info = client.execute(rid, command)
        self.ledger.log("EXECUTION_SENT", task_id=task.task_id, decision_id=did, attempt_id=attempt_id,
                        resource_id=rid, result=result, detail=info, sim_time_s=current.simulation_time_s)
        if result == "REFUSED":
            # 실행되지 않았음이 확실 → 예약 해제 후 다음 후보
            self.ledger.set_attempt_status(attempt_id, "NOT_SENT", info, release=True)
            self._set_purpose(task, "EVALUATING", basis="EXECUTE_REFUSED", attempt_id=attempt_id)
            return {"excluded_reason": f"EXECUTE_REFUSED:{info.get('http_status')}"}
        self._set_purpose(task, "IN_EXECUTION", basis="EXECUTION_SENT", attempt_id=attempt_id)
        if result == "ACK":
            self.ledger.set_attempt_status(attempt_id, "STARTED", {"last_contact_wall": time.time(),
                                                                  "execute_response": info})
        else:
            att = self.ledger.get_attempt(attempt_id)
            self._unknown(att, task, {"execute_response": info}, "EXECUTION_RESPONSE_LOST")
        sub = "STARTED" if result == "ACK" else "UNKNOWN"
        return {"done": True, "status": sub, "attempt_id": attempt_id, "resource_id": rid, "decision_id": did}

    def _hold(self, task: Task, reason: str, resume: str, sim, detail=None) -> dict:
        if not self._set_purpose(task, "HOLD", basis=reason, hold_reason=reason, resume=resume, sim=sim):
            return {"status": "SKIPPED", "purpose_status": task.purpose_status}
        self.ledger.log("TASK_HOLD", task_id=task.task_id, result="HOLD", reason=reason, sim_time_s=sim,
                        detail={"resume_condition": resume, **(detail or {})})
        return {"status": "HOLD", "reason": reason, "resume_condition": resume, **(detail or {})}

    # ------------------------------------------------------------------
    # 추적
    # ------------------------------------------------------------------
    def recover(self) -> dict:
        """서버 시작 시 호출: 송신 전후에 멈춘 PREPARED 시도를 제공자 상태와 대조하고,
        접수만 되고 반영 전에 멈춘 외부 이벤트를 마저 반영한다.
        재송신·재출동은 하지 않는다. 송신을 증명할 수 없으면 UNKNOWN(점유 유지, 수동 해소 대기)."""
        pending = self.ledger.list_attempts(substatuses=["PREPARED"])
        results = []
        for att in pending:
            client = self.uav if att["command"]["resource_type"] == "UAV" else self.ugv
            if client is not None:
                results.append(self._track(att, client))
        events = self.resume_events()
        applies = self.process_pending_applies()      # 환경 응답 대기·임무 반영 대기도 이어서 처리
        self.ledger.log("RECOVERY_SCAN", result="DONE", detail={"prepared_found": [a["attempt_id"] for a in pending],
                                                                "results": results, "events_resumed": events,
                                                                "applies_resumed": applies})
        return {"prepared_found": len(pending), "results": results, "events_resumed": events,
                "applies_resumed": applies}

    def resume_events(self) -> List[str]:
        """활성 run 에서 접수(RECEIVED)만 된 이벤트를 반영한다. 다른 run 의 이벤트는 건드리지 않는다."""
        truth = self.env.read()
        if self._ensure_run(truth):
            return []
        done = []
        for row in self.ledger.pending_external_events():
            self._apply_event(row)
            done.append(row["event_id"])
        return done

    def poll(self) -> List[dict]:
        updates, first_error = [], None
        for att in self.ledger.list_attempts(substatuses=ACTIVE_ATTEMPT):
            client = self.uav if att["command"]["resource_type"] == "UAV" else self.ugv
            if client is None:
                continue
            try:
                u = self._track(att, client)
            except Exception as e:  # noqa: BLE001 — 한 자원의 오류가 다른 자원의 추적을 끊지 않게 한다
                # 숨기지 않는다: 기록하고, 나머지 자원을 다 추적한 뒤 다시 던진다 (외부 응답 형식 오류는
                # _response_problem 이 먼저 걸러 UNKNOWN 으로 격리하므로 여기 오는 것은 내부 결함이다)
                self.ledger.log("TRACK_ERROR", task_id=att["task_id"], attempt_id=att["attempt_id"],
                                resource_id=att["resource_id"], reason=type(e).__name__,
                                detail={"error": str(e)[:300], "other_attempts_still_tracked": True})
                first_error = first_error or e
                continue
            if u:
                updates.append(u)
        updates.extend(self.process_pending_applies())
        if first_error is not None:
            raise first_error
        return updates

    def process_pending_applies(self) -> List[dict]:
        """끝나지 않은 환경 반영 기록을 이어서 처리한다 (R04). poll 과 recover 가 부른다.
        두 단계를 따로 다룬다: ① 환경 응답을 아직 못 받은 관측은 같은 observation_id 로 다시 APPLY,
        ② 응답은 저장했지만 임무 판정이 안 된 것은 저장한 응답으로 판정만 다시 한다 (환경에 다시 보내지 않음).
        재시도 횟수 제한은 두지 않는다 (근거 있는 값이 없음)."""
        rows = self.ledger.pending_applies(run_id=ACTIVE)
        if not rows:
            return []
        env_run = self.env.read().run_id
        return [u for u in (self._process_apply(row, env_run) for row in rows) if u]

    def _process_apply(self, row: dict, env_run, task=None, att=None, detail=None) -> Optional[dict]:
        oid, obs = row["observation_id"], row["observation"]
        task = task or self.ledger.get_task(row["task_id"])
        att = att or self.ledger.get_attempt(row["attempt_id"])
        aid = row["attempt_id"]

        def close(status, reason, note=None):
            self.ledger.finish_pending_apply(oid, status, reason)
            self.ledger.log("ENV_APPLY_RECORD_CLOSED", task_id=row["task_id"], attempt_id=aid, result=status,
                            reason=reason, detail={"observation_id": oid, "env_result": row.get("env_result"),
                                                   "attributed_to_purpose": False, **(note or {})})
            return None

        if row["purpose_bound"] and task and task.purpose_status in TERMINAL_PURPOSES:
            # 취소(종료)된 목적: 환경에 더 보내지 않고, 이미 받은 응답이 있어도 목적을 되살리지 않는다
            return close("CLOSED", f"TASK_{task.purpose_status}")
        owner = bool(task and att and task.owner_attempt_id == aid)
        if row["status"] == "PENDING":
            if env_run != row["run_id"]:
                return None                                    # 다른 run 의 환경에는 적용하지 않는다
            ack = self._apply_to_env(obs)
            if ack.get("pending"):
                first = row["tries"] == 0
                self.ledger.note_pending_apply_try(oid)
                if not (first and row["purpose_bound"]):
                    return None
                self.ledger.log("ENVIRONMENT_APPLY", task_id=row["task_id"], attempt_id=aid,
                                resource_id=att["resource_id"] if att else None, result="TIMEOUT",
                                reason=ack.get("reason"), detail=ack)
                if owner and task.purpose_status == "IN_EXECUTION":
                    self._set_purpose(task, "IN_EXECUTION", basis="ENV_APPLY_PENDING", attempt_id=aid,
                                      hold_reason="ENV_APPLY_PENDING")
                return {"attempt_id": aid, "substatus": att["substatus"] if att else None,
                        "task": task.purpose_status if task else None, "reason": "ENV_APPLY_TIMEOUT"}
            self.ledger.store_apply_response(oid, ack)         # 응답부터 저장 → 여기서 멈춰도 판정을 다시 할 수 있다
            row = {**row, "status": "RESPONDED", "ack": ack, "env_result": "ACK" if ack.get("accepted") else "NACK"}
        ack = row["ack"]
        if not row["purpose_bound"]:
            self.ledger.log("ENVIRONMENT_APPLY", task_id=row["task_id"], attempt_id=aid,
                            result=row["env_result"], reason=ack.get("reason"),
                            detail={**ack, "late": True, "attributed_to_purpose": False})
            self.ledger.finish_pending_apply(oid, "DONE", "AUXILIARY_MEASUREMENT")
            return None
        if not owner:
            # 담당이 다른 시도로 바뀌었다 → 응답은 기록에 남기되 후임의 목적에는 적용하지 않는다
            return close("CLOSED", "OWNER_CHANGED", {"owner_attempt_id": task.owner_attempt_id if task else None})
        if task.purpose_status != "IN_EXECUTION":
            if att["substatus"] == "UNKNOWN":
                return None      # 실행시도가 불명이라 임무가 보류 중 → 응답을 보관하고, 재접촉으로 복구되면 판정한다
            return close("CLOSED", f"PURPOSE_{task.purpose_status}")
        cells = self._map_cells() if task.completion_rule == COMPLETION_AREA_ALL else None   # 환경 조회는 트랜잭션 밖
        if detail is None:
            detail = dict(att["detail"] or {})
        with self.ledger._tx():
            # Task 상태·구역 누적·완료/재대기 사건·판정 종료 표시를 한 트랜잭션으로 묶는다.
            # 도중에 멈추면 전부 되돌아가고 RESPONDED 로 남아 다시 판정한다 (완료 사건·누적이 두 번 생기지 않는다)
            out = self._conclude(att, task, obs, ack, detail, update_attempt=att["substatus"] == "OBSERVED",
                                 cells=cells)
            self.ledger.finish_pending_apply(oid, "DONE", f"TASK_{task.purpose_status}")
        return out

    @staticmethod
    def _response_problem(att: dict, data, task: Optional[Task] = None) -> Optional[str]:
        """제공자 상태 응답이 이 실행시도의 것이고 지금 단계에서 쓸 수 있는지. 문제없으면 None.
        단계별 필수 값: UAV 가 '완료'를 보고했고 아직 관측을 처리하기 전이면 관측 위치가 있어야 한다
        (열화상: lat·lon·alt_m_amsl, 기상: lat·lon). 이동 중 보고, 지상 임무(UGV 관측에는 위치 칸이 없다),
        이미 관측을 저장한 뒤의 복귀 보고에는 위치를 요구하지 않는다."""
        if not isinstance(data, dict):
            return "NOT_AN_OBJECT"
        expected = (att["command"].get("payload") or {}).get("task_id") or att["attempt_id"]
        if data.get("task_id") is None:
            return "EXECUTION_ID_MISSING"
        if data["task_id"] != expected:
            return "EXECUTION_ID_MISMATCH"
        uav = att["command"]["resource_type"] == "UAV"
        if uav:
            if data.get("uav_id") is not None and data["uav_id"] != att["resource_id"]:
                return "RESOURCE_ID_MISMATCH"         # UAV TaskStatus 에는 자원 ID 칸이 없다 (있으면 검사)
        else:
            if data.get("resource_id") is None:
                return "RESOURCE_ID_MISSING"
            if data["resource_id"] != att["resource_id"]:
                return "RESOURCE_ID_MISMATCH"
        status = data.get("status")
        # 문자열인지 먼저 본다 (배열·객체는 집합과 비교하는 것만으로 예외가 난다)
        if not isinstance(status, str) or status not in PROVIDER_STATUSES["UAV" if uav else "GROUND"]:
            return "STATUS_INVALID"
        progress = data.get("progress")
        if progress is not None and not isinstance(progress, dict):
            return "PROGRESS_INVALID"
        if progress and progress.get("phase") is not None and not isinstance(progress["phase"], str):
            return "PHASE_INVALID"
        for key in ("reason", "error"):
            if data.get(key) is not None and not isinstance(data[key], str):
                return f"{key.upper()}_INVALID"
        obs = data.get("observation")
        if obs is not None and not isinstance(obs, dict):
            return "OBSERVATION_INVALID"
        pos = (obs or {}).get("position")
        if pos is not None:
            if not isinstance(pos, dict):
                return "POSITION_INVALID"
            for k in ("lat", "lon", "alt_m_amsl"):
                v = pos.get(k)
                if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)):
                    return "POSITION_INVALID"
        before = (att["detail"] or {}).get("pre_unknown_substatus") if att["substatus"] == "UNKNOWN" else att["substatus"]
        if uav and status == "COMPLETED" and task is not None and before not in POST_MISSION:
            need = {"THERMAL": ("lat", "lon", "alt_m_amsl"), "WEATHER": ("lat", "lon")}.get(task.requirements.sensor, ())
            if need and (not isinstance(pos, dict) or any(pos.get(k) is None for k in need)):
                return "POSITION_MISSING"       # 위치 없이는 관측을 만들 수 없다 → 관측·반영·반납 전에 격리
        return None

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

        if status == "OK":
            why = self._response_problem(att, data, task)
            if why:
                # 다른 실행의 응답이거나 형식이 틀렸다 → 완료·관측 적용·반납의 근거로 쓰지 않는다
                reason = f"RESPONSE_VALIDATION_FAILED:{why}"
                if detail.get("unknown_reason") != reason or att["substatus"] != "UNKNOWN":
                    self.ledger.log("PROVIDER_RESPONSE_REJECTED", task_id=tid, attempt_id=aid, resource_id=rid,
                                    result="REJECTED", reason=why,
                                    detail={"expected_execution_id": (att["command"].get("payload") or {}).get("task_id"),
                                            "expected_resource_id": rid, "applied": False,
                                            "provider": data if isinstance(data, dict) else repr(data)[:300]})
                return self._unknown(att, task, detail, reason)

        if att["substatus"] == "PREPARED":
            # 송신 여부를 장부만으로는 알 수 없다. 제공자가 같은 시도를 확실히 보고할 때만 추적을 이어간다.
            # 제공자가 못 찾는다(404)는 응답만으로 '실행 안 됨'이라 단정하지 않는다 (UAV 상태는 메모리 기반).
            provider_task_id = (att["command"].get("payload") or {}).get("task_id")
            rec = {"attempt_id": aid, "provider_task_id": provider_task_id, "task_id": tid,
                   "decision_id": att["decision_id"], "provider_status": status,
                   "provider_reported_task_id": (data or {}).get("task_id")}
            if not (status == "OK" and data.get("task_id") == provider_task_id):
                self.ledger.log("RECOVERY_PREPARED", task_id=tid, attempt_id=aid, resource_id=rid, result="UNKNOWN",
                                reason="SEND_UNPROVEN", detail={**rec, "occupancy_kept": True,
                                                                "automatic_resend": False})
                return self._unknown(att, task, detail, f"RECOVERY_SEND_UNPROVEN:{status}")
            self.ledger.log("RECOVERY_PREPARED", task_id=tid, attempt_id=aid, resource_id=rid, result="RESUMED",
                            detail=rec)
            detail.update({"recovered_from": "PREPARED", "last_contact_wall": now})
            self.ledger.set_attempt_status(aid, "STARTED", detail)
            att["substatus"] = "STARTED"
            if task and task.owner_attempt_id is None and task.purpose_status == "EVALUATING":
                task.owner_attempt_id = aid       # 예약 직후·담당 기록 전에 멈춘 경우
                self.ledger.save_task(task)
            if self._owns(att, task) and task.purpose_status == "EVALUATING":
                self._set_purpose(task, "APPROVED", basis="RECOVERY_PREPARED_RESUMED", attempt_id=aid)
            if self._owns(att, task) and task.purpose_status in ("APPROVED", "HOLD"):
                self._set_purpose(task, "IN_EXECUTION", basis="RECOVERY_PREPARED_RESUMED", attempt_id=aid,
                                  hold_reason=None, resume=None)

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
        if sub == "UNKNOWN":
            # 실행 파트가 이 시도를 다시 (검증된 응답으로) 보고함 → 실제 상태를 따라 추적 재개
            if (self._owns(att, task) and task.purpose_status == "HOLD" and task.resume_condition == MANUAL
                    and task.hold_reason == detail.get("unknown_reason")):
                self._set_purpose(task, "IN_EXECUTION", basis="EXECUTION_RECONTACTED", attempt_id=aid,
                                  hold_reason=None, resume=None)
                self.ledger.log("EXECUTION_RECONTACTED", task_id=tid, attempt_id=aid, resource_id=rid,
                                detail={"provider_status": pstatus, "phase": phase})
            before = detail.pop("pre_unknown_substatus", None)
            detail.pop("unknown_reason", None)
            if before in POST_MISSION or before == "ARRIVED":
                # 이미 지나온 단계는 다시 밟지 않는다 (관측·환경 반영을 두 번 하지 않음)
                move(before, "PROVIDER_RECONTACTED")
                sub = before

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
            first_report = "mission_end" not in detail
            # 임무 부분 종료 근거: 제공자가 실패를 확정 보고함. 이 근거가 있어야 다른 자원으로 인계할 수 있다
            detail.setdefault("mission_end", {"reason": f"PROVIDER_FAILED:{reason}", "evidence": "PROVIDER_STATUS_FAILED"})
            # 이 시도가 아직 목적 담당일 때만 재대기로 돌린다 — 인계 뒤 같은 실패를 또 보고해도 후임의 목적을 건드리지 않는다
            if self._owns(att, task) and task.purpose_status == "IN_EXECUTION":
                # 목적 미완료. 인계 필요 — 관측 공백을 남기고 재배정 대기
                if self._set_purpose(task, "PENDING", basis=f"PROVIDER_FAILED:{reason}", attempt_id=aid,
                                     hold_reason=f"HANDOVER:{reason}"):
                    self.ledger.log("TASK_REOPENED", task_id=tid, attempt_id=aid, resource_id=rid,
                                    reason=reason, detail={"observation_gap": True})
            if phase == "RETURNING":
                if sub == "RETURNING" and not first_report:
                    return None                      # 같은 보고의 반복 — 점유만 유지
                return move("RETURNING", reason, extra={"failed": True})
            if phase == "DONE":
                return self._maybe_release(att, client, phase, detail, move)
            if phase == "FAILED" and att["command"]["resource_type"] != "UAV":
                # UGV 서버: 이상 확정 → 차를 세우고 UNAVAILABLE. /stop 전까지 재배정 불가
                return move("FAULTED", reason, extra={"fault": data.get("error")})
            return self._unknown(att, task, detail, f"FAILED_POSITION_UNCONFIRMED:{reason}")
        return None

    def _unknown(self, att, task, detail, reason) -> Optional[dict]:
        if att["substatus"] == "UNKNOWN":
            return None
        self.ledger.set_attempt_status(att["attempt_id"], "UNKNOWN", {
            **detail, "unknown_reason": reason, "pre_unknown_substatus": att["substatus"]})
        self.ledger.log("EXECUTION_UNKNOWN", task_id=att["task_id"], attempt_id=att["attempt_id"],
                        resource_id=att["resource_id"], result="UNKNOWN", reason=reason,
                        detail={"occupancy_kept": True, "automatic_redispatch": False,
                                "previous_substatus": att["substatus"]})
        att["substatus"] = "UNKNOWN"
        if self._owns(att, task) and task.purpose_status in ("IN_EXECUTION", "APPROVED", "EVALUATING"):
            self._set_purpose(task, "HOLD", basis=f"EXECUTION_UNKNOWN:{reason}", attempt_id=att["attempt_id"],
                              hold_reason=reason, resume=MANUAL)
        return {"attempt_id": att["attempt_id"], "substatus": "UNKNOWN", "reason": reason}

    # ------------------------------------------------------------------
    # 관측 → 환경 반영 → 목적 판정
    # ------------------------------------------------------------------
    def _observe_and_apply(self, att, task, data, detail) -> Optional[dict]:
        aid, rid = att["attempt_id"], att["resource_id"]
        sensor = task.requirements.sensor
        pos = ((data.get("observation") or {}).get("position")) or {}
        # 이 관측을 목적 완료에 쓸 수 없는 경우: 목적이 이미 종료(취소 등)됐거나, 인계되어 담당이 다른 시도다
        closed = task.purpose_status in TERMINAL_PURPOSES or not self._owns(att, task)
        after_label = task.purpose_status if task.purpose_status in TERMINAL_PURPOSES else "SUPERSEDED"
        detail.setdefault("mission_end", {"reason": "ARRIVED_AND_OBSERVED", "evidence": "PROVIDER_STATUS_COMPLETED"})
        snap = self.env.read()
        if snap.run_id != att.get("run_id"):
            # 이 실행시도의 run 은 끝났다. 다른 run 의 세계를 읽어 관측을 만들지 않는다 (물리 추적만 계속)
            self.ledger.log("OBSERVATION_SKIPPED", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                            reason="RUN_MISMATCH", detail={"attempt_run": att.get("run_id"), "env_run": snap.run_id})
            detail["observation_skipped"] = "RUN_MISMATCH"
            self.ledger.set_attempt_status(aid, "OBSERVED", detail)
            att["substatus"] = "OBSERVED"
            if not closed:
                self._set_purpose(task, "HOLD", basis="RUN_ENDED_BEFORE_OBSERVATION", attempt_id=aid,
                                  hold_reason="RUN_ENDED_BEFORE_OBSERVATION", resume=MANUAL)
            return {"attempt_id": aid, "substatus": "OBSERVED", "reason": "RUN_MISMATCH"}
        if sensor == "WEATHER":
            obs = observation.simulate_weather(snapshot=snap, task=task, attempt_id=aid, resource_id=rid,
                                               position=pos, provider_raw=data.get("observation"))
        elif att["command"]["resource_type"] != "UAV":
            return self._ground_observation(att, task, data, detail, closed)
        elif sensor != "THERMAL":
            return None
        else:
            obs = observation.simulate(snapshot=snap, task=task, attempt_id=aid, resource_id=rid,
                                       position=pos, uav_raw_observation=data.get("observation"))
        if closed:
            obs["received_after_purpose"] = after_label
            obs["used_for_completion"] = False
        self.ledger.log("OBSERVATION", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                        result=obs["result"], reason=obs.get("failure_reason"), sim_time_s=snap.simulation_time_s,
                        detail=obs)
        after = {"received_after_purpose": after_label, "task_id": task.task_id} if closed else None
        if sensor == "WEATHER" and obs["result"] == "MEASURED":
            self.kb.add_weather(f"FIELD:{obs['observation_id']}", snap.simulation_time_s, obs["source"], {
                "kind": "FIELD", "cell_id": task.nav_target.cell_id, "lat": pos.get("lat"), "lon": pos.get("lon"),
                "values": obs["values"], "scope": obs.get("scope"), "observation_id": obs["observation_id"],
                "resource_id": rid, **(after or {})}, received_sim_s=snap.simulation_time_s)
        elif sensor == "THERMAL" and obs["result"] != "FAILED":
            self.kb.record_observation(obs, snap.simulation_time_s, extra=after)
        if closed:
            # 유효한 관측은 출처·'취소 이후 수신'과 함께 보관하되, 끝난 임무의 성공 근거로 쓰지 않는다
            detail.update({"observation_id": obs["observation_id"], "observation_after_purpose": after_label})
            self.ledger.set_attempt_status(aid, "OBSERVED", detail)
            att["substatus"] = "OBSERVED"
            self.ledger.log("OBSERVATION_AFTER_CLOSE", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                            result=after_label, reason="NOT_USED_FOR_COMPLETION",
                            detail={"observation_id": obs["observation_id"], "kept_in_knowledge": obs["result"] != "FAILED"})
            return {"attempt_id": aid, "substatus": "OBSERVED", "task": task.purpose_status}
        if obs["result"] == "FAILED":
            detail["observation_failed"] = obs.get("failure_reason")
            self.ledger.set_attempt_status(aid, "OBSERVED", detail)
            att["substatus"] = "OBSERVED"
            self._reopen(task, "OBSERVATION_FAILED:" + obs["failure_reason"], aid)
            return {"attempt_id": aid, "substatus": "OBSERVED", "reason": obs["failure_reason"]}
        visit = dataclasses.replace(task)      # 이번 방문 기준 (구역 임무는 판정 뒤 다음 방문 지점으로 바뀐다)
        if task.requirements.needs_env_ack:
            # 환경에 보내기 전에 관측을 '반영 기록'으로 장부에 남긴다. 이후 어느 단계에서 멈춰도
            # (환경 응답 전, 응답 저장 뒤 임무 판정 전) 재시작 후 그 단계부터 이어진다 (R04)
            detail["observation_id"] = obs["observation_id"]
            self.ledger.set_attempt_status(aid, "OBSERVED", detail)
            att["substatus"] = "OBSERVED"
            self.ledger.add_pending_apply(obs, att.get("run_id") or task.run_id, task.task_id, aid, True)
            row = self.ledger.pending_applies(task_id=task.task_id, status=None)[-1]
            out = self._process_apply(row, snap.run_id, task=task, att=att, detail=detail)
        else:
            out = self._conclude(att, task, obs, {"accepted": None, "skipped": True}, detail)
        if sensor == "THERMAL" and self._weather_this_visit(visit, att, obs):
            self._measure_weather_same_visit(visit, att, pos, data)
        if obs["result"] == "DETECTED":
            self._auto_sense(visit, obs, self.view(), measured_here=self._weather_this_visit(visit, att, obs))
        return out

    def _apply_to_env(self, obs: dict) -> dict:
        """환경 APPLY. 응답을 못 받았으면(시간 초과·연결 끊김) pending — 거절(NACK)과 구분한다."""
        try:
            return self.env.apply(obs)
        except ENV_TRANSIENT_ERRORS as e:
            return {"accepted": False, "pending": True, "reason": "ENV_APPLY_TIMEOUT",
                    "error": f"{type(e).__name__}: {e}"[:300]}

    def _conclude(self, att, task, obs, ack, detail, update_attempt: bool = True, cells=None) -> dict:
        """관측 + 환경 응답(ACK/NACK/불필요)으로 목적 충족을 판정한다. 환경을 부르지 않는다 (장부만 바꾼다).
        update_attempt=False: 기체를 이미 반납했거나 다른 단계에 있는 경우 — 실행시도 상태는 건드리지 않는다."""
        aid, rid = att["attempt_id"], att["resource_id"]
        need_ack = task.requirements.needs_env_ack
        result = "ACK" if ack.get("accepted") else ("SKIPPED" if ack.get("skipped") else (
            "TIMEOUT" if ack.get("pending") else "NACK"))
        self.ledger.log("ENVIRONMENT_APPLY", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                        result=result, reason=ack.get("reason"), detail=ack)
        detail.update({"observation_id": obs["observation_id"], "env_ack": ack})
        detail.pop("env_apply_pending", None)
        sub = att["substatus"]
        if update_attempt:
            sub = "APPLIED" if ack.get("accepted") else "OBSERVED"
            self.ledger.set_attempt_status(aid, sub, detail)
            att["substatus"] = sub
        acked = bool(ack.get("accepted")) or not need_ack
        if task.completion_rule == COMPLETION_AREA_ALL:
            self._conclude_area(att, task, obs, ack, acked, cells)
        elif obs["target_covered"] and acked:
            if self._set_purpose(task, "COMPLETED", basis="TARGET_OBSERVED", attempt_id=aid, hold_reason=None):
                self.ledger.log("TASK_COMPLETE", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                                result="COMPLETED", detail={
                                    "completion_rule": task.completion_rule,
                                    "basis": ["ARRIVED", "SIMULATED_OBSERVATION_TARGET_COVERED",
                                              "ENV_ACK" if ack.get("accepted") else "NO_ENV_ACK_REQUIRED"],
                                    "resource_released": False, **self._evidence_level(obs.get("source"), ack)})
        else:
            self._reopen(task, "OBSERVATION_REQUIREMENT_NOT_MET" if not obs["target_covered"]
                         else "ENV_APPLY_NOT_ACKED", aid)
        return {"attempt_id": aid, "substatus": sub, "task": task.purpose_status}

    # ------------------------------------------------------------------
    # 구역 임무 완료 판정 (D03): 필수 셀 전체가 누적 관측으로 완전히 덮여야 완료.
    #   - 한 점(프레임 하나) 결과를 구역 완료로 올리지 않는다. 프레임이 칸 일부만 덮으면 그만큼만 쌓는다.
    #   - 같은 관측(observation_id)은 두 번 쌓지 않는다.
    #   - 남은 필수 셀 중 아직 가 보지 않은 칸이 있으면 그 칸을 다음 방문 지점으로 두고 다시 대기.
    #   - 남은 칸을 모두 한 번씩 갔는데도 덮이지 않으면 (프레임 < 칸, 또는 지도에 칸 정보 없음) 구역 관측
    #     증거를 지금 제공자 계약으로는 만들 수 없다 → 사유와 남은 범위를 남기고 보류 (UAV 구역 관측 계약 대기).
    # ------------------------------------------------------------------
    def _map_cells(self) -> dict:
        return {c["cell_id"]: c for c in getattr(self.env, "map_cells", lambda: [])()}

    def area_progress(self, task: Task, cells: Optional[dict] = None) -> dict:
        cells = self._map_cells() if cells is None else cells
        frac = {cid: (task.coverage.get(cid) or {}).get("fraction", 0.0) for cid in task.required_cell_ids}
        remaining = [cid for cid in task.required_cell_ids if frac[cid] < 1.0]
        no_geo = [cid for cid in remaining if cells.get(cid, {}).get("lat") is None
                  or cells.get(cid, {}).get("lon") is None or not cells.get(cid, {}).get("cell_size_m")]
        unvisited = [cid for cid in remaining if cid not in task.visited_cell_ids and cid not in no_geo]
        return {"required": list(task.required_cell_ids), "fraction": frac, "remaining": remaining,
                "geometry_missing": no_geo, "unvisited": unvisited, "visited": list(task.visited_cell_ids),
                "complete": not remaining, "cells": cells}

    def _conclude_area(self, att, task, obs, ack, acked: bool, cells: Optional[dict] = None) -> None:
        aid, rid = att["attempt_id"], att["resource_id"]
        oid, sim = obs["observation_id"], obs.get("simulation_time_s")
        for cc in obs.get("cell_coverage", []):
            if cc["cell_id"] not in task.required_cell_ids:
                continue
            entry = task.coverage.setdefault(cc["cell_id"], {"rects": [], "fraction": 0.0, "sources": [],
                                                             "last_observed_sim_s": None})
            if any(r[4] == oid for r in entry["rects"]):
                continue                                   # 같은 관측 재전송 — 두 번 쌓지 않는다
            entry["rects"].append([*cc["rect"], oid])
            entry["fraction"] = round(observation.union_fraction(entry["rects"], cc["cell_size_m"]), 6)
            entry["last_observed_sim_s"] = sim
            if obs.get("source") not in entry["sources"]:
                entry["sources"].append(obs.get("source"))
        here = task.nav_target.cell_id
        if here and here not in task.visited_cell_ids:
            task.visited_cell_ids.append(here)
        self.ledger.save_task(task)
        p = self.area_progress(task, cells)
        summary = {k: p[k] for k in ("required", "fraction", "remaining", "geometry_missing", "unvisited", "visited")}
        self.ledger.log("AREA_COVERAGE", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                        result="COMPLETE" if p["complete"] else "INCOMPLETE", sim_time_s=sim,
                        detail={**summary, "observation_id": oid, "partial_cells": obs.get("partial_cells"),
                                "rule": "필수 셀 전체가 누적 관측 사각형의 합집합으로 완전히 덮여야 완료 (비율 임계값 없음)"})
        if p["complete"] and acked:
            task.visit_target = None
            if self._set_purpose(task, "COMPLETED", basis="AREA_FULLY_OBSERVED", attempt_id=aid, hold_reason=None):
                self.ledger.log("TASK_COMPLETE", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                                result="COMPLETED", detail={
                                    "completion_rule": task.completion_rule,
                                    "basis": ["ARRIVED", "ALL_REQUIRED_CELLS_FULLY_OBSERVED",
                                              "ENV_ACK" if ack.get("accepted") else "NO_ENV_ACK_REQUIRED"],
                                    "coverage": summary, "resource_released": False,
                                    **self._evidence_level(obs.get("source"), ack)})
            return
        if not acked:
            self._reopen(task, "ENV_APPLY_NOT_ACKED", aid)
            return
        if p["unvisited"]:
            c = p["cells"][p["unvisited"][0]]
            task.visit_target = Target(lat=c["lat"], lon=c["lon"], ground_amsl_m=c.get("ground_amsl_m"),
                                       cell_id=c["cell_id"])
            self.ledger.save_task(task)
            self._reopen(task, "AREA_COVERAGE_INCOMPLETE", aid)
            return
        if self._set_purpose(task, "HOLD", basis="AREA_EVIDENCE_UNSUPPORTED", attempt_id=aid,
                             hold_reason="AREA_EVIDENCE_UNSUPPORTED", resume=MANUAL):
            self.ledger.log("TASK_HOLD", task_id=task.task_id, attempt_id=aid, result="HOLD",
                            reason="AREA_EVIDENCE_UNSUPPORTED", sim_time_s=sim,
                            detail={"resume_condition": MANUAL, **summary,
                                    "why": "남은 필수 셀을 한 번씩 방문했지만 한 프레임으로 칸 전체를 덮지 못했거나 "
                                           "지도에 칸 정보가 없다. 구역 관측 증거(경로·프레임 목록) 계약 필요 (UAV-08)"})

    # ------------------------------------------------------------------
    # 현장 환경 측정 자동 생성 (사용자 결정 2026-09-30)
    # ------------------------------------------------------------------
    @staticmethod
    def _bearing_deg(lat1, lon1, lat2, lon2) -> float:
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

    # ------------------------------------------------------------------
    # 한 번 가서 할 수 있는 측정은 한 번에 (같은 칸 중복 방문 방지)
    # ------------------------------------------------------------------
    OPEN_PURPOSES = ("PENDING", "HOLD", "EVALUATING", "APPROVED", "IN_EXECUTION")

    def _weather_this_visit(self, task: Task, att: dict, obs: dict) -> bool:
        """이번 정찰 방문에서 기상도 잴지: 임무에 붙은 추가 측정이거나, 불을 발견했고 기체가 잴 수 있을 때"""
        from .resources import capabilities
        if obs.get("result") == "FAILED":
            return False
        can = "WEATHER" in capabilities(att["command"]["resource_type"])
        if not can:
            return False
        if "WEATHER" in task.requirements.extra_sensors:
            return True
        return config.AUTO_ENV_SENSE_ENABLED and obs.get("result") == "DETECTED"

    def _measure_weather_same_visit(self, task: Task, att: dict, pos: dict, data: dict) -> dict:
        aid, rid = att["attempt_id"], att["resource_id"]
        truth = self.env.read()
        w = observation.simulate_weather(snapshot=truth, task=task, attempt_id=aid, resource_id=rid,
                                         position=pos, provider_raw=None)
        w["same_visit_as"] = task.kind
        self.ledger.log("OBSERVATION", task_id=task.task_id, attempt_id=aid, resource_id=rid, result=w["result"],
                        reason=w.get("failure_reason"), sim_time_s=truth.simulation_time_s, detail=w)
        if w["result"] == "MEASURED":
            self.kb.add_weather(f"FIELD:{w['observation_id']}", truth.simulation_time_s, w["source"], {
                "kind": "FIELD", "cell_id": task.nav_target.cell_id, "lat": pos.get("lat"), "lon": pos.get("lon"),
                "values": w["values"], "scope": w.get("scope"), "observation_id": w["observation_id"],
                "resource_id": rid, "same_visit_as": task.kind}, received_sim_s=truth.simulation_time_s)
            ack = self._apply_to_env(w)
            if ack.get("pending"):                     # 함께 한 측정도 반영 기록으로 남긴다 (목적 완료와는 무관)
                self.ledger.add_pending_apply(w, att.get("run_id") or task.run_id, task.task_id, aid, False)
            self.ledger.log("ENVIRONMENT_APPLY", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                            result="ACK" if ack.get("accepted") else ("TIMEOUT" if ack.get("pending") else "NACK"),
                            reason=ack.get("reason"), detail=ack)
        return w

    def _open_tasks_at(self, cell_id: str, kinds=None) -> List[Task]:
        return [t for t in self.ledger.list_tasks(self.OPEN_PURPOSES)
                if t.target.cell_id == cell_id and (kinds is None or t.kind in kinds)]

    def _merge_pending_sense_into(self, task: Task) -> None:
        """새 정찰이 들어왔는데 같은 칸에 아직 출발하지 않은 측정 임무가 있으면 취소하고 정찰에 합친다."""
        if task.kind == "ENV_SENSE" or task.requirements.sensor != "THERMAL" or not task.target.cell_id:
            return
        occupied = {r["task_id"] for r in self.ledger.reservations().values()}
        merged = []
        for s in self._open_tasks_at(task.target.cell_id, kinds=("ENV_SENSE",)):
            if s.purpose_status in ("PENDING", "HOLD") and s.task_id not in occupied:
                if self._set_purpose(s, "CANCELLED", basis=f"MERGED_INTO:{task.task_id}",
                                     hold_reason=f"MERGED_INTO:{task.task_id}", resume=None):
                    merged.append(s.task_id)
        if merged:
            task.requirements.extra_sensors = tuple(sorted(set(task.requirements.extra_sensors) | {"WEATHER"}))
            self.ledger.save_task(task)
            self.ledger.log("TASK_MERGED", task_id=task.task_id, result="WEATHER_ATTACHED",
                            detail={"cancelled_sense_tasks": merged, "cell_id": task.target.cell_id})

    def _attach_or_create_sense(self, c: dict, incident_id: str, run_id: str,
                                request_id: Optional[str] = None) -> Optional[str]:
        """측정 지점에 이미 열린 정찰이 있으면 그 정찰에 기상 측정을 붙이고, 없으면 측정 임무를 만든다."""
        for t in self._open_tasks_at(c["cell_id"]):
            if t.kind == "ENV_SENSE" or "WEATHER" in t.requirements.extra_sensors:
                return None                                   # 이미 그 칸에서 잴 예정
            if t.requirements.sensor == "THERMAL":
                t.requirements.extra_sensors = tuple(sorted(set(t.requirements.extra_sensors) | {"WEATHER"}))
                self.ledger.save_task(t)
                self.ledger.log("TASK_MERGED", task_id=t.task_id, result="WEATHER_ATTACHED",
                                detail={"cell_id": c["cell_id"], "reason": "SENSE_POINT_HAS_OPEN_RECON"})
                return None
        t, new = self.submit_task({
            "request_id": request_id or f"AUTO-SENSE:{run_id}:{c['cell_id']}", "incident_id": incident_id,
            "kind": "ENV_SENSE",
            "target": {"lat": c["lat"], "lon": c["lon"], "ground_amsl_m": c.get("ground_amsl_m"),
                       "cell_id": c["cell_id"]},
            "requirements": {"resource_types": ["UAV", "UGV", "FIRE_ENGINE"], "sensor": "WEATHER",
                             "needs_env_ack": True}})
        return t.task_id if new else None

    def _request_remeasure(self, task: Task, snap) -> Optional[dict]:
        """만료된 현장 측정의 재측정 요청 (D02). 같은 칸·같은 만료 측정에 대해 한 번만 요청한다.
        측정은 지금의 모의 기상 센서 계약(SIMULATED / TEST_ONLY)을 쓴다 — 실제 센서가 아니다."""
        nav, items = task.nav_target, snap.wind_ref["remeasure"]
        if not nav.cell_id:
            return None
        sig = content_hash(sorted([r["item"], r["expired_key"]] for r in items))[:16]
        if not self.ledger.add_knowledge("WEATHER_REMEASURE", f"REMEASURE:{nav.cell_id}:{sig}", nav.cell_id,
                                         snap.simulation_time_s, "ORCHESTRATOR", {"items": items}):
            return None                                       # 이미 요청함
        tid = self._attach_or_create_sense(
            {"cell_id": nav.cell_id, "lat": nav.lat, "lon": nav.lon, "ground_amsl_m": nav.ground_amsl_m},
            task.incident_id, snap.run_id, request_id=f"AUTO-REMEASURE:{snap.run_id}:{nav.cell_id}:{sig}")
        out = {"cell_id": nav.cell_id, "items": items, "new_sense_task_id": tid,
               "mode": "NEW_SENSE_TASK" if tid else "ATTACHED_TO_OPEN_TASK_OR_ALREADY_PLANNED",
               "measurement_contract": "SIMULATED_WEATHER_SENSOR (TEST_ONLY)"}
        self.ledger.log("WEATHER_REMEASURE_REQUESTED", task_id=task.task_id, result=out["mode"],
                        sim_time_s=snap.simulation_time_s, detail=out)
        return out

    def _auto_sense(self, task: Task, obs: dict, snap, measured_here: bool = False) -> List[str]:
        if not config.AUTO_ENV_SENSE_ENABLED or task.kind == "ENV_SENSE" or snap.run_gate:
            return []
        cells = {c["cell_id"]: c for c in snap.fire_cells + snap.risk_cells}
        created = []
        for det in obs.get("detections", []):
            fire = cells.get(det["cell_id"])
            if not fire or fire.get("lat") is None:
                continue
            # 풍향은 아는 세계에서: 불난 칸의 유효한 현장 측정 → 가장 가까운 관측소 (진짜 풍향을 보지 않음)
            w = self.kb.weather_for(fire["lat"], fire["lon"], fire["cell_id"], snap.simulation_time_s or 0.0)
            wd = w["items"].get("wind_dir_deg") or {}
            sp = self.sense_points(fire, dataclasses.replace(snap, wind_dir_deg=w["values"].get("wind_dir_deg")))
            sp["wind_ref"] = {k: wd.get(k) for k in ("basis", "source", "station_id", "distance_m", "status")}
            sp["wind_ref"]["sim_time_s"] = wd.get("observed_sim_s")
            for c in sp["points"]:
                if c["cell_id"] == fire["cell_id"] and measured_here and fire["cell_id"] == task.nav_target.cell_id:
                    continue                                  # 불 발견한 그 방문에서 이미 측정함
                tid = self._attach_or_create_sense(c, task.incident_id, snap.run_id)
                if tid:
                    created.append(tid)
            self.ledger.log("ENV_SENSE_PLANNED", task_id=task.task_id, result="CREATED" if created else "EXISTING",
                            detail={"fire_cell": fire["cell_id"], "points": [c["cell_id"] for c in sp["points"]],
                                    "selection": sp["note"], "wind_ref": sp["wind_ref"], "created": created,
                                    "measured_in_same_visit": measured_here})
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
                "kind": "MONITOR", "area_cell_ids": c["cells"], "required_cell_ids": c["cells"],
                "target": {"lat": tc["lat"], "lon": tc["lon"], "ground_amsl_m": tc.get("ground_amsl_m"),
                           "cell_id": c["target_cell"]},
                "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL", "needs_env_ack": True}})
            if new:
                created.append(t.task_id)
        if created:
            self.ledger.log("PREEMPTIVE_MONITOR_CREATED", sim_time_s=snap.simulation_time_s,
                            detail={"created": created, "candidates": cands})
        return {"enabled": True, "candidates": cands, "created": created}

    def _ground_observation(self, att, task, data, detail, closed: bool = False) -> dict:
        """UGV/소방차 도착. 제공 관측은 ROAD_STATUS 뿐이며 화재 관측으로 바꾸지 않는다."""
        aid, rid = att["attempt_id"], att["resource_id"]
        raw = data.get("observation")
        need = task.requirements.sensor
        satisfied = need is None or (need == "ROAD_STATUS" and bool(raw))
        self.ledger.log("OBSERVATION", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                        result="ROAD_STATUS" if raw else "NONE",
                        detail={"source": "UGV_PROVIDER", "raw": raw, "is_fire_observation": False,
                                **({"received_after_purpose": task.purpose_status, "used_for_completion": False}
                                   if closed else {})})
        detail["observation_source"] = "UGV_PROVIDER"
        self.ledger.set_attempt_status(aid, "OBSERVED", detail)
        att["substatus"] = "OBSERVED"
        if closed:
            return {"attempt_id": aid, "substatus": "OBSERVED", "task": task.purpose_status}
        if task.requirements.needs_env_ack:
            # 도로 관측에는 환경 반영 ACK 계약이 없다. 접수에서 거절하는 조합이며(B07), 이전 장부에 남은
            # 임무라면 ACK 없이 완료시키지 않고 사유를 남겨 보류한다 — 플래그를 조용히 무시하지 않는다.
            self.ledger.log("ENVIRONMENT_APPLY", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                            result="UNSUPPORTED", reason=f"ENV_ACK_UNSUPPORTED_FOR_SENSOR:{need}",
                            detail={"accepted": False, "fake_ack_created": False})
            self._hold(task, f"ENV_ACK_UNSUPPORTED_FOR_SENSOR:{need}", MANUAL, None)
        elif satisfied:
            if self._set_purpose(task, "COMPLETED", basis="GROUND_ARRIVAL_OBSERVED", attempt_id=aid, hold_reason=None):
                self.ledger.log("TASK_COMPLETE", task_id=task.task_id, attempt_id=aid, resource_id=rid,
                                result="COMPLETED", detail={"completion_rule": task.completion_rule,
                                                            "basis": ["ARRIVED", "UGV_ROAD_STATUS" if raw else "ARRIVAL_ONLY_TASK",
                                                                      "NO_ENV_ACK_REQUIRED"],
                                                            "resource_released": False,
                                                            **self._evidence_level("UGV_PROVIDER", None)})
        else:
            self._reopen(task, "ARRIVED_OBSERVATION_PENDING", aid)
        return {"attempt_id": aid, "substatus": "OBSERVED", "task": task.purpose_status}

    def _evidence_level(self, observation_source, ack) -> dict:
        """완료 근거의 수준. 모의 센서 또는 가짜 환경 ACK 로 끝난 완료는 '시뮬레이션 시험 완료'다.
        실제 센서 관측·실제(팀 공유) 환경 반영 완료와 구분한다."""
        env_source = getattr(self.env, "source", "UNKNOWN")
        simulated = observation_source == "SIMULATED" or env_source == "FIXTURE"
        return {"evidence_level": "SIMULATION_TEST" if simulated else "INTEGRATED",
                "evidence": {"observation_source": observation_source, "env_source": env_source,
                             "env_ack": None if ack is None else bool(ack.get("accepted"))}}

    def _reopen(self, task, reason, attempt_id=None) -> bool:
        if not self._set_purpose(task, "PENDING", basis=reason, hold_reason=reason, attempt_id=attempt_id):
            return False
        self.ledger.log("TASK_REOPENED", task_id=task.task_id, attempt_id=attempt_id, reason=reason)
        return True

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
            # 기체 반납은 환경 반영 대기와 별개다. 대기 중인 관측은 장부(pending_applies)에 남아 계속 재반영된다
            return move("RELEASED", "RESOURCE_READY_CONFIRMED", release=True,
                        extra={"released_evidence": {"ready": True, "phase": phase}})
        if att["substatus"] in ("RETURNING", "FAULTED"):
            return None                     # 아직 READY 아님 → 점유 유지
        return move("RETURNING", "LANDED_READY_UNCONFIRMED")

    # ------------------------------------------------------------------
    # 수동 해소
    # ------------------------------------------------------------------
    def resolve(self, task_id: str, action: str, reason: str, attempt_id: Optional[str] = None,
                request_id: Optional[str] = None) -> dict:
        """action
          RETRY                  실패·보류 뒤 재평가 (종료된 Task·임무가 열린 시도가 있는 Task 는 거절)
          CANCEL                 목적 취소. 진행 중 기체의 추적·점유는 그대로 이어진다
          CONFIRM_RESOURCE_IDLE  사람이 기체 정지 확인 → 불명 시도의 점유 해제 (종료된 목적은 되살리지 않음)
          RECHECK                종료된 Task 의 대상을 다시 관측할 새 Task 생성 (이전 Task 는 그대로 둠)
        """
        task = self.ledger.get_task(task_id)
        if task is None:
            return {"status": "NOT_FOUND"}
        self.ledger.log("MANUAL_RESOLUTION", task_id=task_id, attempt_id=attempt_id, result=action, reason=reason)
        attempts = self.ledger.list_attempts(task_id)
        open_ = self.ledger.open_mission_attempts(task_id)

        def refuse(why, **extra):
            self.ledger.log("MANUAL_RESOLUTION_REFUSED", task_id=task_id, attempt_id=attempt_id, result=action,
                            reason=why, detail={"purpose_status": task.purpose_status, **extra})
            return {"status": "REFUSED", "reason": why, "purpose_status": task.purpose_status, **extra}

        if action == "CONFIRM_RESOURCE_IDLE":
            att = self.ledger.get_attempt(attempt_id) if attempt_id else None
            if not att or att["task_id"] != task_id or att["substatus"] != "UNKNOWN":
                return refuse("ATTEMPT_NOT_UNKNOWN")
            self.ledger.set_attempt_status(attempt_id, "FAILED", {**(att["detail"] or {}), "manual_reason": reason},
                                           release=True)
            if task.purpose_status == "HOLD" and self.ledger.pending_applies(task_id=task_id):
                # 관측은 이미 했고 환경 반영 판정만 남았다 → 재대기로 돌리지 않고 판정을 이어 간다
                self._set_purpose(task, "IN_EXECUTION", basis="MANUAL_IDLE_CONFIRMED_APPLY_OPEN",
                                  attempt_id=attempt_id, hold_reason="ENV_APPLY_PENDING", resume=None)
            elif task.purpose_status == "HOLD":
                self._set_purpose(task, "PENDING", basis="MANUAL_IDLE_CONFIRMED", attempt_id=attempt_id,
                                  hold_reason="MANUAL_IDLE_CONFIRMED", resume=None)
            return {"status": "RELEASED", "purpose_status": task.purpose_status}
        if action == "RETRY":
            if task.purpose_status in TERMINAL_PURPOSES:
                return refuse(f"TASK_ALREADY_{task.purpose_status}", hint="재관측은 action=RECHECK (새 Task)")
            if any(a["substatus"] == "UNKNOWN" for a in open_):
                return refuse("UNKNOWN_ATTEMPT_OPEN")
            if open_:
                return refuse("ATTEMPT_STILL_OPEN", attempt_id=open_[0]["attempt_id"],
                              substatus=open_[0]["substatus"])
            if self.ledger.pending_applies(task_id=task_id):
                return refuse("ENV_APPLY_PENDING", hint="관측은 끝났고 환경 반영 확인을 기다리는 중 — 다시 보내지 않는다")
            if task.purpose_status not in ("PENDING", "HOLD", "EVALUATING"):
                return refuse(f"TASK_{task.purpose_status}")
            if task.purpose_status == "HOLD":
                self._set_purpose(task, "PENDING", basis="MANUAL_RETRY", resume=None)
            return self.dispatch(task_id)
        if action == "CANCEL":
            if task.purpose_status in TERMINAL_PURPOSES:
                return refuse(f"TASK_ALREADY_{task.purpose_status}")
            self._set_purpose(task, "CANCELLED", basis=f"MANUAL_CANCEL:{reason}")
            for row in self.ledger.pending_applies(task_id=task_id):      # 취소된 목적: 반영·판정을 멈추고 사유와 함께 닫는다
                self.ledger.finish_pending_apply(row["observation_id"], "CLOSED", "TASK_CANCELLED")
            occupying = [a for a in attempts if a["substatus"] in ACTIVE_ATTEMPT]
            return {"status": "CANCELLED", "stop_command_sent": False,
                    "open_attempts": [{"attempt_id": a["attempt_id"], "resource_id": a["resource_id"],
                                       "substatus": a["substatus"]} for a in occupying],
                    "resources_still_occupied": [a["resource_id"] for a in occupying],
                    "note": "진행 중 기체의 중단 명령은 보내지 않았다 (UAV/UGV 중단 API 계약 대기). "
                            "기체가 돌아와 READY 가 확인될 때까지 추적과 점유를 유지한다"}
        if action == "RECHECK":
            return self.recheck(task_id, reason, request_id)
        return refuse("UNKNOWN_ACTION")

    def recheck(self, task_id: str, reason: str, request_id: Optional[str] = None) -> dict:
        """종료된 Task 의 대상을 다시 관측하는 새 Task(RECHECK). 이전 Task 의 상태는 바꾸지 않는다.
        request_id 를 주면 같은 값의 재전송은 같은 RECHECK 로 수렴한다."""
        parent = self.ledger.get_task(task_id)
        if parent is None:
            return {"status": "NOT_FOUND"}
        if parent.purpose_status not in ("COMPLETED", "CANCELLED"):
            return {"status": "REFUSED", "reason": "PARENT_NOT_CLOSED", "purpose_status": parent.purpose_status,
                    "hint": "끝나지 않은 Task 는 RETRY 로 다룬다"}
        n = 1 + sum(1 for t in self.ledger.list_tasks() if t.parent_task_id == task_id)
        r = parent.requirements
        body = {"request_id": f"RECHECK:{task_id}:{request_id or n}", "incident_id": parent.incident_id,
                "kind": "RECHECK", "parent_task_id": task_id,
                "target": dataclasses.asdict(parent.target),
                "requirements": {"resource_types": list(r.resource_types), "sensor": r.sensor,
                                 "target_agl_m": r.target_agl_m,
                                 "agl_range_m": list(r.agl_range_m) if r.agl_range_m else None,
                                 "needs_env_ack": r.needs_env_ack},
                "area_cell_ids": list(parent.area_cell_ids), "required_cell_ids": list(parent.required_cell_ids)}
        task, created = self.submit_task(body, _inherit=parent)
        self.ledger.log("RECHECK_CREATED", task_id=task.task_id, result="CREATED" if created else "DUPLICATE",
                        reason=reason, detail={"parent_task_id": task_id, "parent_status": parent.purpose_status,
                                               "completion_rule": task.completion_rule})
        return {"status": "CREATED" if created else "DUPLICATE", "task_id": task.task_id,
                "parent_task_id": task_id, "parent_status": parent.purpose_status}
