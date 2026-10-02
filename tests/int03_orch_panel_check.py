# -*- coding: utf-8 -*-
"""
INT-03 확인 (총괄 팀별 요청서): 관제판 우측 "총괄 판단" 요약

- 총괄 서버가 없으면 연결 안 됨으로 응답하고 관제판은 계속 동작
- 총괄 서버가 있으면 /health, /priority/board 를 읽어 요약 (읽기 전용)
- 관제판은 총괄 상태를 바꾸는 요청(POST)을 보내지 않음

실행: python -m tests.int03_orch_panel_check   (총괄·UAV 서버 불필요 — 응답을 흉내 냄)
"""
import tempfile

import config

# 관제판을 불러오면 로그 파일을 새로 만들므로, 실제 logs/web_auto.jsonl 을 지우지 않게 임시 폴더를 쓴다
config.LOG_DIR = tempfile.mkdtemp(prefix="adair_test_")
config.ORCH_URL = "http://127.0.0.1:59999"   # 아무도 없는 주소

import httpx
from fastapi.testclient import TestClient
import web.app as webapp

ok = True


def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({info})" if info else ""))


c = TestClient(webapp.app)

# 1) 총괄 서버 없음
r = c.get("/api/orch").json()
check("총괄 서버가 없으면 connected=False", r["connected"] is False)

# 2) 총괄 서버 응답 흉내
HEALTH = {"truth_env": {"class": "FixtureEnv", "shared_team_env": False}}
BOARD = {"mode": "AUTO_HIGHER_RISK", "llm": {"not_ready_reason": "API_KEY_MISSING"}, "groups": [],
         "known": {"fires": [{"cell_id": "C1"}], "run_id": "RUN-X", "simulation_time_s": 0.0},
         "auto_order": [{"task_id": "T1", "kind": "RECON", "cell_id": "C1", "purpose_status": "HOLD",
                         "hold_reason": "TARGET_LOCATION_UNKNOWN", "resume_condition": "MAP_INFO", "resource_id": None}]}
STATE = {"active_attempts": [{"task_id": "T2", "substatus": "RETURNING", "resource_id": "A-uav1"}]}
BOARD["auto_order"].append({"task_id": "T2", "kind": "RECON", "cell_id": "C2", "purpose_status": "COMPLETED",
                            "hold_reason": None, "resume_condition": None, "resource_id": None})
calls = []


class _R:
    def __init__(self, d): self._d = d
    def json(self): return self._d


real_get = httpx.get
def fake_get(url, **kw):
    calls.append(("GET", url))
    return _R(HEALTH if url.endswith("/health") else STATE if url.endswith("/state") else BOARD)
def fake_post(url, **kw):
    calls.append(("POST", url))
    raise AssertionError("관제판이 총괄에 POST 하면 안 됨")

httpx.get, post_backup, httpx.post = fake_get, httpx.post, fake_post
try:
    r = c.get("/api/orch").json()
finally:
    httpx.get, httpx.post = real_get, post_backup

check("총괄 응답을 받으면 connected=True", r["connected"] is True)
check("환경 종류 표시 (시험용 환경 구분)", r["env_class"] == "FixtureEnv" and r["env_shared"] is False)
check("임무 요약에 상태·보류 사유 포함", r["tasks"][0]["purpose_status"] == "HOLD"
      and r["tasks"][0]["hold_reason"] == "TARGET_LOCATION_UNKNOWN")
check("목적 완료와 기체 복귀를 따로 표시", r["tasks"][1]["purpose_status"] == "COMPLETED"
      and r["tasks"][1]["attempt"] == "RETURNING" and r["tasks"][1]["resource_id"] == "A-uav1")
check("읽기 전용 (POST 요청 없음)", all(m == "GET" for m, _ in calls), f"{len(calls)}회 GET")

html = c.get("/").text
check("화면에 불명·보류를 실패로 칠하지 않는 표기 포함", "UNKNOWN:['확인 필요','wait']" in html and "HOLD:['보류','wait']" in html)

print("\n모두 통과" if ok else "\n실패 항목 있음")
raise SystemExit(0 if ok else 1)   # 자동 시험(GitHub Actions)이 실패를 알아채도록
