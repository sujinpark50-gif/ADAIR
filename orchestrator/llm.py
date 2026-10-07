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
- 출력 스키마와 자료형 (값을 쓰기 전에 검사한다: 정수 칸에 bool·배열·객체, 문자열 목록에 다른 형 금지)
- 같은 환경 state_version
- 묶음마다 정확히 한 번, 순서는 그 묶음 Task 의 순열 (없는 ID·중복·누락 금지)
- 근거가 인용한 ID(input_refs)는 입력에 있던 ID 만
- 근거 문장 길이 제한

모델·timeout·호출 한도는 설정값이다 (사용자 결정: 2026-10-07 gpt-5.5·60초, 2026-09-30 서버 1회 실행당 50회).
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
    "order 에는 입력의 task_id 문자열을 그대로 넣는다 (셀 이름 금지). "
    "rationale 문장은 관제 요원이 읽으므로 임무를 대상 셀 이름(cells)으로 부르고 2문장 이내로 쓴다."
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
                    "order": {"type": "array", "items": {"type": "string"},
                              "description": "먼저 보낼 순서. 입력 tasks[].task_id 값을 그대로 (셀 이름 아님)"},
                    "rationale": {"type": "string"},
                    "input_refs": {"type": "array", "items": {"type": "string"}},
                },
            },
        },
    },
}

SYSTEM_WITH_FORMAT = (
    SYSTEM + " 답은 아래 JSON Schema 를 따르는 JSON 객체 하나만 출력한다 (설명·코드블록 없이).\n"
    + json.dumps(OUTPUT_SCHEMA, ensure_ascii=False)
)


def _strip_code_fence(text):
    """모델이 ```json ... ``` 으로 감싸 보내는 경우만 벗긴다. 그 밖의 내용은 손대지 않는다."""
    if not isinstance(text, str):
        return text
    t = text.strip()
    if t.startswith("```") and t.endswith("```"):
        t = t[3:-3]
        if t[:4].lower() == "json":
            t = t[4:]
    return t.strip()


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


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)      # True/False 를 정수로 받지 않는다


def _is_str_list(v) -> bool:
    return isinstance(v, list) and all(isinstance(x, str) for x in v)


def validate(out, inp: dict) -> List[str]:
    """LLM 출력의 위반 목록 (비어 있으면 통과). 외부 출력은 어떤 형이든 올 수 있으므로
    해시·정렬·비교에 쓰기 전에 자료형부터 검사한다. 예외를 던지지 않는다."""
    v = []
    if not isinstance(out, dict) or set(out) != {"state_version", "groups"}:
        return ["SCHEMA_MISMATCH"]
    if not _is_int(out["state_version"]):
        v.append("STATE_VERSION_TYPE")
    elif out["state_version"] != inp["state_version"]:
        v.append("STATE_VERSION_MISMATCH")
    groups = out["groups"]
    if not isinstance(groups, list):
        return v + ["SCHEMA_MISMATCH"]
    want = {g["group_index"]: [t["task_id"] for t in g["tasks"]] for g in inp["groups"]}
    seen = set()
    ids = known_ids(inp)
    for n, g in enumerate(groups):
        if not isinstance(g, dict) or set(g) != {"group_index", "order", "rationale", "input_refs"}:
            v.append(f"SCHEMA_MISMATCH:groups[{n}]")
            continue
        gi = g["group_index"]
        if not _is_int(gi):
            v.append(f"GROUP_INDEX_TYPE:groups[{n}]")
            continue
        if gi not in want or gi in seen:
            v.append(f"GROUP_INDEX_INVALID:{gi}")
            continue
        seen.add(gi)
        order = g["order"]
        if not _is_str_list(order):
            v.append(f"ORDER_TYPE:{gi}")
        elif len(set(order)) != len(order) or sorted(order) != sorted(want[gi]):
            v.append(f"ORDER_NOT_PERMUTATION:{gi}")       # 중복·누락·다른 묶음 Task·없는 ID
        if not isinstance(g["rationale"], str) or not g["rationale"].strip() or len(g["rationale"]) > MAX_RATIONALE_CHARS:
            v.append(f"RATIONALE_INVALID:{gi}")
        refs = g["input_refs"]
        if not _is_str_list(refs):
            v.append(f"INPUT_REFS_TYPE:{gi}")
        else:
            unknown = [r for r in refs if r not in ids]
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
            self._client = OpenAI(base_url=config.LLM_BASE_URL)
        return self._client

    def propose(self, snapshot, plan: dict, tasks_by_id: dict) -> dict:
        """{status: OK/DISABLED/ERROR/INVALID, orders: {group_index: [...]}, rationales, ...}"""
        base = {"provider": config.LLM_PROVIDER, "model": self.model, "timeout_s": self.timeout_s,
                "temperature": self.temperature}
        why = self.status()
        if why:
            return {**base, "status": "DISABLED", "reason": why}
        inp = build_input(snapshot, plan, tasks_by_id)
        # copa 게이트웨이는 Chat Completions 만 받고 response_format(json_schema·json_object)도 거부한다
        # (2026-10-07 확인). 그래서 출력 형식은 지시문으로 주고, 지키는지는 validate() 가 결정론으로 검사한다.
        kwargs = {"model": self.model,
                  "messages": [{"role": "system", "content": SYSTEM_WITH_FORMAT},
                               {"role": "user", "content": json.dumps(inp, ensure_ascii=False)}],
                  "timeout": self.timeout_s}
        if self.temperature is not None:
            kwargs["temperature"] = self.temperature
        self.calls += 1
        t0 = time.monotonic()
        try:
            resp = self._get_client().chat.completions.create(**kwargs)
            raw = _strip_code_fence(resp.choices[0].message.content)
        except Exception as e:  # noqa: BLE001 — 모델 실패는 규칙 fallback 으로 회수한다
            return {**base, "status": "ERROR", "reason": type(e).__name__, "error": str(e)[:300],
                    "latency_s": round(time.monotonic() - t0, 3), "input": inp}
        latency = round(time.monotonic() - t0, 3)
        try:
            out = json.loads(raw)
        except (TypeError, ValueError):
            return {**base, "status": "INVALID", "violations": ["NOT_JSON"], "raw_output": str(raw)[:2000],
                    "latency_s": latency, "input": inp}
        try:
            violations = validate(out, inp)
        except (TypeError, KeyError, ValueError, AttributeError) as e:
            # 검증기는 자료형을 먼저 검사하므로 여기 오면 검증기 결함이다. 그래도 외부 출력 때문에
            # 판단 루프가 멈추면 안 되므로 제안을 버리고(INVALID) 규칙 순서로 간다. 사유를 남긴다.
            violations = [f"VALIDATOR_ERROR:{type(e).__name__}"]
        if violations:
            return {**base, "status": "INVALID", "violations": violations, "raw_output": out,
                    "latency_s": latency, "input": inp}
        return {**base, "status": "OK", "latency_s": latency, "input": inp, "raw_output": out,
                "orders": {g["group_index"]: g["order"] for g in out["groups"]},
                "rationales": {g["group_index"]: g["rationale"] for g in out["groups"]},
                "input_refs": {g["group_index"]: g["input_refs"] for g in out["groups"]}}

    # ------------------------------------------------------------------
    # 관측 계획 (사용자 결정 2026-10-07): 예상 지역과 자원 분담을 LLM 이 고른다.
    # 형식·참조 오류면 오류 목록을 주고 한 번 고쳐 달라고 한다 (계획서 §7). 그래도 틀리면 INVALID → 규칙 대체.
    # 두 번째 호출도 같은 호출 한도를 쓴다.
    # ------------------------------------------------------------------
    def _chat(self, messages) -> str:
        kwargs = {"model": self.model, "messages": messages, "timeout": self.timeout_s}
        if self.temperature is not None:
            kwargs["temperature"] = self.temperature
        self.calls += 1
        resp = self._get_client().chat.completions.create(**kwargs)
        return _strip_code_fence(resp.choices[0].message.content)

    def plan_observations(self, inp: dict) -> dict:
        from . import observation_planner as op
        base = {"provider": config.LLM_PROVIDER, "model": self.model, "timeout_s": self.timeout_s,
                "kind": "OBSERVATION_PLAN"}
        why = self.status()
        if why:
            return {**base, "status": "DISABLED", "reason": why, "attempts": []}
        system = (op.SYSTEM + "\n답은 아래 JSON Schema 를 따르는 JSON 객체 하나만 출력한다 (설명·코드블록 없이).\n"
                  + json.dumps(op.OUTPUT_SCHEMA, ensure_ascii=False))
        messages = [{"role": "system", "content": system},
                    {"role": "user", "content": json.dumps(inp, ensure_ascii=False)}]
        attempts, t0 = [], time.monotonic()
        for n in range(2):
            if n and self.status():
                attempts.append({"status": "NOT_RETRIED", "reason": self.status()})
                break
            t1 = time.monotonic()
            try:
                raw = self._chat(messages)
            except Exception as e:  # noqa: BLE001 — 모델 실패는 규칙 대체로 회수한다
                attempts.append({"status": "ERROR", "reason": type(e).__name__, "error": str(e)[:300]})
                return {**base, "status": "ERROR", "reason": type(e).__name__, "attempts": attempts,
                        "latency_s": round(time.monotonic() - t0, 3)}
            try:
                out = json.loads(raw)
                violations = op.validate(out, inp)
            except (TypeError, ValueError):
                out, violations = None, ["NOT_JSON"]
            except (KeyError, AttributeError) as e:
                out, violations = None, [f"VALIDATOR_ERROR:{type(e).__name__}"]
            attempts.append({"status": "INVALID" if violations else "OK", "violations": violations,
                             "raw_output": out if out is not None else str(raw)[:2000],
                             "latency_s": round(time.monotonic() - t1, 3)})
            if not violations:
                return {**base, "status": "OK", "plan": out, "attempts": attempts, "repaired": n == 1,
                        "latency_s": round(time.monotonic() - t0, 3)}
            messages = messages + [{"role": "assistant", "content": str(raw)[:8000]},
                                   {"role": "user", "content": "검증 실패. 다음 오류를 고쳐 같은 형식의 JSON 하나만 다시 "
                                                               "출력하라: " + json.dumps(violations, ensure_ascii=False)}]
        last = [a for a in attempts if a["status"] == "INVALID"][-1]
        return {**base, "status": "INVALID", "violations": last["violations"], "attempts": attempts,
                "latency_s": round(time.monotonic() - t0, 3)}
