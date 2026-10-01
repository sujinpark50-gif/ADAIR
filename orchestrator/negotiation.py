# -*- coding: utf-8 -*-
"""
orchestrator/negotiation.py
===========================
Local 의 COUNTER 처리 (사용자 결정 3A, 요청서 §4.3).

수정안을 재평가하는 조건:
1. 수정 필드가 구체적인 값으로 있다 (note 만 있는 응답은 제외)
2. 그 필드를 총괄이 재평가 요청에 실어 보낼 수 있다 (config.COUNTER_SUPPORTED_FIELDS)
3. 수정 후에도 임무 목적이 유지된다 (Task 요구 범위 안)
4. 같은 수정안의 반복이 아니고, 자원당 협상 예산(2회) 안이다

그 밖의 COUNTER 는 실행하지 않고 다음 후보로 넘긴다. 재평가도 새 decision 으로 한다.
MODERATE_WIND 를 풍속만 보고 그대로 수용하던 v0.1.2 예외는 쓰지 않는다.
"""

import json
from typing import Optional, Set, Tuple

from . import config
from .models import Task

# 수정 제안이 아닌 설명·참고용 키
_INFORMATIONAL_KEYS = {"note", "detail", "eta_sec", "arrival"}


def counter_modifications(counter_offer: Optional[dict]) -> dict:
    return {k: v for k, v in (counter_offer or {}).items() if k not in _INFORMATIONAL_KEYS}


def decide_counter(task: Task, resource_type: str, local: dict, rounds_used: int,
                   seen_offers: Set[str]) -> Tuple[str, object]:
    """("RETRY", 수정필드 dict) 또는 ("DROP", 사유코드)"""
    offer = local.get("counter_offer")
    mods = counter_modifications(offer)
    if not mods:
        return "DROP", "COUNTER_NOT_ACTIONABLE"          # note-only 등
    unsupported = set(mods) - config.COUNTER_SUPPORTED_FIELDS.get(resource_type, set())
    if unsupported:
        return "DROP", "COUNTER_FIELD_UNSUPPORTED:" + ",".join(sorted(unsupported))
    key = json.dumps(mods, sort_keys=True)
    if key in seen_offers:
        return "DROP", "COUNTER_REPEATED"
    if rounds_used >= config.COUNTER_ROUNDS_PER_RESOURCE:
        return "DROP", "COUNTER_BUDGET_EXHAUSTED"
    if "target_agl_m" in mods:
        v = mods["target_agl_m"]
        rng = task.requirements.agl_range_m
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            return "DROP", "COUNTER_VALUE_INVALID"
        if rng is None:
            return "DROP", "COUNTER_PURPOSE_UNVERIFIABLE"   # 허용 범위가 정해지지 않은 임무
        if not (rng[0] <= v <= rng[1]):
            return "DROP", "COUNTER_BREAKS_REQUIREMENT"
    seen_offers.add(key)
    return "RETRY", mods
