# -*- coding: utf-8 -*-
"""
orchestrator/llm.py
===================
LLM 계획 제안 + 결정론적 검증 (사용자 결정 1A, 요청서 §9).

LLM 이 하는 일
- 규칙으로 순서를 정할 수 없는 "선택 묶음"(위험도 비슷·기준 엇갈림) 안에서 먼저 보낼 순서를 추천하고
  근거를 적는다. 인명 우선은 묶음을 나눌 때 이미 적용되어 LLM 이 뒤집을 수 없다.
- 좌표·고도·위험점수 같은 수치나 존재하지 않는 자원·사실을 만들지 않는다 (출력 스키마에 수치 칸이 없다).

검증 (하나라도 어기면 제안을 버리고 규칙 순서 사용)
- 출력 스키마, 같은 환경 state_version
- 묶음마다 정확히 한 번, 순서는 그 묶음 Task 의 순열 (없는 ID·중복·누락 금지)
- 근거가 인용한 ID(input_refs)는 입력에 있던 ID 만
- 근거 문장 길이 제한

모델·timeout·호출 한도는 설정값이다 (사용자 결정 2026-09-30: gpt-5.6-luna, 30초, 서버 1회 실행당 50회).
timeout 이나 호출 한도가 비어 있거나 키가 없으면 LLM 을 부르지 않는다. API 키는 .env 에서 읽고 기록하지 않는다.
"""

import json
import os
import time
from typing import List, Optional

from . import config

MAX_RATIONALE_CHARS = 600

SYSTEM = (
    "너는 산불 대응 총괄 오케스트레이터의 계획 보조다. 주어진 입력 사실만 사용한다. "
    "각 선택 묶음(groups) 안에서 먼저 출동할 임무 순서를 정하고 한국어로 짧은 근거를 적는다. "
    "원칙: 인명 보호 최우선(이미 묶음 분리에 반영됨), 그다음 확산 위험과 보호대상 근접성. "
    "입력에 없는 사실·수치·자원을 만들지 말고, 근거에 인용한 ID 는 input_refs 에 넣는다. "
    "근거 문장은 관제 요원이 읽는다: 임무를 task_id 대신 대상 셀 이름(cells)으로 부르고, 2문장 이내로 쓴다."
)

OUTPUT_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": ["state_version", "groups"],
    "properties": {
        "state_version": {"type": "integer"},
        "groups": {
            "type": "array",
            "items": {
                "type": "object",
                "additionalProperties": False,
                "required": ["group_index", "order", "rationale", "input_refs"],
                "properties": {
                    "group_index": {"type": "integer"},
                    "order": {"type": "array", "items": {"type": "string"}},
                    "rationale": {"type": "string"},
                    "input_refs": {"type": "array", "items": {"type": "string"}},
                },
            },
        },
    },
}


def build_input(snapshot, plan: dict, tasks_by_id: dict) -> dict:
    """LLM 에 줄 입력. 판단에 필요한 사실만, ID 로 추적 가능하게."""
    def task_row(tid):
        ev = plan["evidence"][tid]
        t = tasks_by_id[tid]
        np_ = ev.get("nearest_protected") or {}
        return {"task_id": tid, "kind": t.kind, "cells": ev.get("cells_used"),
                "human_risk": ev.get("human_risk"), "risk_score": ev.get("risk_score"),
                "nearest_protected_site_id": np_.get("site_id"), "nearest_protected_site_type": np_.get("site_type"),
                "distance_to_protected_m": ev.get("distance_m"),
                "official_priority_rank": ev.get("official_priority_rank")}
    return {
        "run_id": snapshot.run_id, "state_version": snapshot.state_version,
        "simulation_time_s": snapshot.simulation_time_s, "wind_ms": snapshot.wind_ms,
        "groups": [{"group_index": i, "kinds": g["kinds"], "tasks": [task_row(t) for t in g["task_ids"]]}
                   for i, g in enumerate(plan["groups"])],
        "allowed_action": "각 묶음의 task_id 순서만 정한다",
    }


def known_ids(inp: dict) -> set:
    ids = set()
    for g in inp["groups"]:
        for t in g["tasks"]:
            ids.add(t["task_id"])
            ids.update(t.get("cells") or [])
            if t.get("nearest_protected_site_id"):
                ids.add(t["nearest_protected_site_id"])
    return ids


def validate(out, inp: dict) -> List[str]:
    v = []
    if not isinstance(out, dict) or set(out) != {"state_version", "groups"}:
        return ["SCHEMA_MISMATCH"]
    if out["state_version"] != inp["state_version"]:
        v.append("STATE_VERSION_MISMATCH")
    groups = out.get("groups")
    if not isinstance(groups, list):
        return v + ["SCHEMA_MISMATCH"]
    want = {g["group_index"]: [t["task_id"] for t in g["tasks"]] for g in inp["groups"]}
    seen = set()
    ids = known_ids(inp)
    for g in groups:
        if not isinstance(g, dict) or set(g) != {"group_index", "order", "rationale", "input_refs"}:
            v.append("SCHEMA_MISMATCH")
            continue
        gi = g["group_index"]
        if gi not in want or gi in seen:
            v.append(f"GROUP_INDEX_INVALID:{gi}")
            continue
        seen.add(gi)
        if sorted(g["order"]) != sorted(want[gi]) or len(set(g["order"])) != len(g["order"]):
            v.append(f"ORDER_NOT_PERMUTATION:{gi}")
        if not isinstance(g["rationale"], str) or not g["rationale"].strip() or len(g["rationale"]) > MAX_RATIONALE_CHARS:
            v.append(f"RATIONALE_INVALID:{gi}")
        unknown = [r for r in g["input_refs"] if r not in ids]
        if unknown:
            v.append(f"UNKNOWN_REFS:{gi}:{','.join(unknown[:5])}")
    missing = set(want) - seen
    if missing:
        v.append("GROUPS_MISSING:" + ",".join(map(str, sorted(missing))))
    return v


_DEFAULT = object()     # 인자를 주지 않으면 config 값, None 을 주면 "설정 안 됨"


class LlmPlanner:
    """OpenAI Responses API 로 선택 묶음 순서를 제안받는다. client 는 시험에서 바꿔 끼운다."""

    def __init__(self, client=None, model: Optional[str] = None, timeout_s=_DEFAULT,
                 max_calls=_DEFAULT, temperature=_DEFAULT):
        self.model = model or config.LLM_MODEL
        self.timeout_s = config.LLM_TIMEOUT_S if timeout_s is _DEFAULT else timeout_s
        self.max_calls = config.LLM_MAX_CALLS_PER_RUN if max_calls is _DEFAULT else max_calls
        self.temperature = config.LLM_TEMPERATURE if temperature is _DEFAULT else temperature
        self._client = client
        self.calls = 0

    def status(self) -> Optional[str]:
        """호출할 수 없는 이유. 호출 가능하면 None."""
        if self._client is None and not os.getenv(config.LLM_API_KEY_ENV):
            return "API_KEY_MISSING"
        if self.timeout_s is None:
            return "TIMEOUT_NOT_CONFIGURED"
        if self.max_calls is None:
            return "CALL_LIMIT_NOT_CONFIGURED"
        if self.calls >= self.max_calls:
            return "CALL_LIMIT_REACHED"
        return None

    def _get_client(self):
        if self._client is None:
            from openai import OpenAI
            self._client = OpenAI()
        return self._client

    def propose(self, snapshot, plan: dict, tasks_by_id: dict) -> dict:
        """{status: OK/DISABLED/ERROR/INVALID, orders: {group_index: [...]}, rationales, ...}"""
        base = {"provider": config.LLM_PROVIDER, "model": self.model, "timeout_s": self.timeout_s,
                "temperature": self.temperature}
        why = self.status()
        if why:
            return {**base, "status": "DISABLED", "reason": why}
        inp = build_input(snapshot, plan, tasks_by_id)
        kwargs = {"model": self.model, "instructions": SYSTEM,
                  "input": json.dumps(inp, ensure_ascii=False),
                  "text": {"format": {"type": "json_schema", "name": "priority_order",
                                      "schema": OUTPUT_SCHEMA, "strict": True}},
                  "timeout": self.timeout_s}
        if self.temperature is not None:
            kwargs["temperature"] = self.temperature
        self.calls += 1
        t0 = time.monotonic()
        try:
            resp = self._get_client().responses.create(**kwargs)
            raw = resp.output_text
        except Exception as e:  # noqa: BLE001 — 모델 실패는 규칙 fallback 으로 회수한다
            return {**base, "status": "ERROR", "reason": type(e).__name__, "error": str(e)[:300],
                    "latency_s": round(time.monotonic() - t0, 3), "input": inp}
        latency = round(time.monotonic() - t0, 3)
        try:
            out = json.loads(raw)
        except (TypeError, ValueError):
            return {**base, "status": "INVALID", "violations": ["NOT_JSON"], "raw_output": str(raw)[:2000],
                    "latency_s": latency, "input": inp}
        violations = validate(out, inp)
        if violations:
            return {**base, "status": "INVALID", "violations": violations, "raw_output": out,
                    "latency_s": latency, "input": inp}
        return {**base, "status": "OK", "latency_s": latency, "input": inp, "raw_output": out,
                "orders": {g["group_index"]: g["order"] for g in out["groups"]},
                "rationales": {g["group_index"]: g["rationale"] for g in out["groups"]},
                "input_refs": {g["group_index"]: g["input_refs"] for g in out["groups"]}}
