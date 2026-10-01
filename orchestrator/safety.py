# -*- coding: utf-8 -*-
"""
orchestrator/safety.py
======================
규칙 기반 Safety (요청서 §5-3, 실행계획 "Safety 최소 검사").

Local 이 ACCEPT 한 명령만 들어온다. 필요한 정보가 하나라도 없으면 ALLOW 하지 않는다.
MODIFY 는 내지 않는다 — 수정이 필요하면 REJECT 사유로 알리고 총괄이 새 decision 으로
Local 재평가부터 다시 한다 (무검증 수정 명령 실행 금지).

검사 항목
 1. 식별: attempt/task/decision/resource ID, Local 응답 echo 일치
 2. 상태: Task 가 평가 가능한 상태, 자원 미점유, 자원 상태 신선도·READY
 3. 입력: snapshot 필수 필드, 판단 이후 snapshot 버전 변화 없음
 4. 단위·좌표: 목표 위경도, 지면고도(UAV), 좌표계·수직 datum, 풍속
 5. 능력: 필수 관측 능력
 6. 동일성: Local 이 ACCEPT 한 임무 조건 == 실제 보낼 명령 (목표·AGL·풍속)
 7. 고도 중복 가산 금지: alt_m_amsl 은 Task 의 지면고도 그대로 (80m·10m 미포함)
"""

from dataclasses import dataclass, field
from typing import List, Optional

from . import config
from .env_adapter import missing_snapshot_fields, wind_status
from .models import ResourceView, Snapshot, Task
from .prefilter import valid_latlon, valid_number

EVALUABLE_PURPOSES = {"PENDING", "HOLD", "EVALUATING"}


@dataclass
class SafetyResult:
    response: str                       # ALLOW / REJECT
    reasons: List[str] = field(default_factory=list)
    checked: List[str] = field(default_factory=list)

    @property
    def reason(self) -> Optional[str]:
        return ",".join(self.reasons) if self.reasons else None


def mission_fields(payload: dict) -> dict:
    """Local 평가와 실행 명령에서 반드시 같아야 하는 임무 조건"""
    t = payload.get("target") or {}
    return {"lat": t.get("lat"), "lon": t.get("lon"), "alt_m_amsl": t.get("alt_m_amsl"),
            "target_agl_m": t.get("target_agl_m"), "target_node": payload.get("target_node"),
            "observation_type": payload.get("observation_type"), "wind_ms": payload.get("wind_ms")}


def check(*, task: Task, attempt_id: str, decision_id: str, view: ResourceView, view_age_s: Optional[float],
          reservations: dict, eval_snapshot: Snapshot, current_snapshot: Snapshot,
          evaluated_payload: dict, local: dict, command: dict) -> SafetyResult:
    r = SafetyResult("ALLOW")

    def need(ok: bool, name: str, reason: str):
        r.checked.append(name)
        if not ok:
            r.reasons.append(reason)

    rid = view.resource_id
    rtype = view.resource_type
    # 1. 식별
    need(bool(attempt_id) and bool(decision_id) and bool(task.task_id), "ids", "ID_MISSING")
    echo = local.get("echo") or {}
    need(local.get("verdict") == "ACCEPT", "local_accept", "LOCAL_NOT_ACCEPT")
    need(echo.get("decision_id") == decision_id and echo.get("resource_id") == rid,
         "local_echo", "LOCAL_ECHO_MISMATCH")
    need(command.get("decision_id") == decision_id, "command_decision", "COMMAND_DECISION_MISMATCH")
    need(command.get("task_id") == attempt_id, "command_attempt", "COMMAND_ATTEMPT_MISMATCH")
    # 2. 상태
    need(task.purpose_status in EVALUABLE_PURPOSES, "task_state", f"TASK_STATE_{task.purpose_status}")
    need(rid not in reservations, "reservation", "RESOURCE_OCCUPIED")
    need(view_age_s is not None and view_age_s < config.RESOURCE_STALE_S, "freshness", "RESOURCE_STATE_STALE")
    need(view.ready and not view.failsafe, "resource_ready", "RESOURCE_NOT_READY")
    need(rtype in task.requirements.resource_types, "type_allowed", "TYPE_NOT_ALLOWED")
    # 3. 입력 snapshot
    need(not missing_snapshot_fields(current_snapshot), "snapshot_complete", "SNAPSHOT_INCOMPLETE")
    need((current_snapshot.run_id, current_snapshot.state_version)
         == (eval_snapshot.run_id, eval_snapshot.state_version), "snapshot_unchanged", "SNAPSHOT_CHANGED")
    # 4. 단위·좌표
    tgt = command.get("target") or {}
    need(valid_latlon(tgt.get("lat"), tgt.get("lon")), "target_coord", "TARGET_COORD_INVALID")
    need(bool(current_snapshot.crs), "crs", "CRS_UNKNOWN")
    if rtype == "UAV":
        need(valid_number(tgt.get("alt_m_amsl")), "ground_alt", "TARGET_ALTITUDE_UNKNOWN")
        need(bool(current_snapshot.vertical_datum), "vertical_datum", "VERTICAL_DATUM_UNKNOWN")
        need(wind_status(current_snapshot) is None, "wind", "WIND_UNUSABLE")
        need(command.get("wind_ms") == current_snapshot.wind_ms, "wind_same_snapshot", "WIND_NOT_FROM_SNAPSHOT")
        # 7. 고도 중복 가산 금지
        need(tgt.get("alt_m_amsl") == task.nav_target.ground_amsl_m, "no_double_add", "ALTITUDE_NOT_GROUND_LEVEL")
        need(valid_number(tgt.get("target_agl_m")) and tgt.get("target_agl_m") > 0, "agl", "TARGET_AGL_INVALID")
    # 5. 능력
    need(not task.requirements.sensor or task.requirements.sensor in view.sensors,
         "capability", "REQUIRED_CAPABILITY_UNAVAILABLE")
    # 6. 평가한 조건 == 보낼 조건
    need(mission_fields(evaluated_payload) == mission_fields(command), "same_mission", "COMMAND_DIFFERS_FROM_EVALUATED")

    if r.reasons:
        r.response = "REJECT"
    return r
