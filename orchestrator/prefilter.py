# -*- coding: utf-8 -*-
"""
orchestrator/prefilter.py
=========================
Local 평가 전에 "물리적으로 명백히 불가능한" 자원을 사유와 함께 뺀다
(실행계획 §5 후보 거르기, 요청서 §2).

- 확인된 사실로만 뺀다. 필요한 정보가 없거나 오래되면 통과시키지 않는다.
- 배터리·경로·복귀 여유처럼 기체가 직접 판단할 일은 Local 에 맡긴다.
- 남은 후보는 목표까지 직선거리 → resource_id 순으로 정렬한다 (결정론적).
  직선거리는 정렬용일 뿐 도착시간이 아니다. 도착시간은 Local 의 eta_sec 를 쓴다.
"""

import math
from typing import Callable, Dict, List, Optional, Tuple

from . import config
from .env_adapter import wind_status
from .models import ResourceView, Snapshot, Task


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin(math.radians(lat2 - lat1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * 6371000.0 * math.asin(math.sqrt(min(1.0, max(0.0, a))))


def valid_latlon(lat, lon) -> bool:
    return (isinstance(lat, (int, float)) and isinstance(lon, (int, float))
            and math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180)


def valid_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def task_input_problems(task: Task, snapshot: Snapshot, resource_type: str) -> List[str]:
    """이 자원 종류로 Task 를 평가하는 데 빠진 입력. 비어 있으면 평가 가능."""
    problems = []
    if not valid_latlon(task.target.lat, task.target.lon):
        problems.append("TARGET_COORD_INVALID")
    if resource_type == "UAV":
        if not valid_number(task.target.ground_amsl_m):
            problems.append("TARGET_ALTITUDE_UNKNOWN")
        w = wind_status(snapshot)
        if w:
            problems.append(w)
        if not snapshot.vertical_datum:
            problems.append("VERTICAL_DATUM_UNKNOWN")
    return problems


def filter_candidates(
    task: Task,
    snapshot: Snapshot,
    views: List[ResourceView],
    unreachable: Dict[str, str],
    reservations: Dict[str, dict],
    age_of: Callable[[ResourceView], Optional[float]],
    excluded: Optional[Dict[str, str]] = None,
) -> Tuple[List[Tuple[float, ResourceView]], Dict[str, str]]:
    """(정렬된 후보 [(거리m, view)], 제외 {resource_id: 사유})"""
    out: Dict[str, str] = dict(excluded or {})
    for rid, err in unreachable.items():
        out.setdefault(rid, "STATE_UNREACHABLE")
    allowed = set(task.requirements.resource_types)
    need = task.requirements.sensor
    input_problems = {}
    ranked = []
    for v in views:
        rid = v.resource_id
        if rid in out:
            continue
        if v.resource_type not in allowed:
            out[rid] = "TYPE_NOT_ALLOWED"
            continue
        if need and need not in v.sensors:
            out[rid] = "REQUIRED_CAPABILITY_UNAVAILABLE"
            continue
        if rid in reservations:
            out[rid] = "OCCUPIED"
            continue
        age = age_of(v)
        if age is None:
            out[rid] = "STATE_TIME_UNKNOWN"
            continue
        if age >= config.RESOURCE_OFFLINE_S:
            out[rid] = "STATE_OFFLINE"
            continue
        if age >= config.RESOURCE_STALE_S:
            out[rid] = "STATE_STALE"
            continue
        if v.failsafe:
            out[rid] = "FAILSAFE_ACTIVE"
            continue
        if not v.ready:
            out[rid] = "NOT_READY"
            continue
        if not valid_latlon(v.lat, v.lon):
            out[rid] = "POSITION_UNKNOWN"
            continue
        if v.resource_type not in input_problems:
            input_problems[v.resource_type] = task_input_problems(task, snapshot, v.resource_type)
        if input_problems[v.resource_type]:
            out[rid] = "INPUT_MISSING:" + ",".join(input_problems[v.resource_type])
            continue
        ranked.append((haversine_m(v.lat, v.lon, task.target.lat, task.target.lon), v))
    ranked.sort(key=lambda x: (x[0], x[1].resource_id))
    return ranked, out
