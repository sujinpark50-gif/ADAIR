# -*- coding: utf-8 -*-
"""
INT-01 확인 (총괄 팀별 요청서): 관제판 "출동" 버튼을 총괄 경유로

- 스위치 켜짐(WEB_DISPATCH_VIA_ORCH): 총괄 POST /tasks 로 한 번만 보내고, UAV 서버는 부르지 않는다
- 요청 내용: kind=RECON, UAV·열화상, 목표 위경도·지면 해발고도·칸 번호, 클릭마다 새 request_id
- 총괄 서버가 없으면 실패로 답하고 UAV 로 우회하지 않는다
- 스위치 꺼짐(기본): 기존처럼 UAV 직접 경로 (총괄 호출 없음)

실행: python -m tests.int01_orch_dispatch_check   (총괄·UAV 서버 불필요 — 응답을 흉내 냄)
"""
import tempfile

import config

# 관제판을 불러오면 로그 파일을 새로 만들므로, 실제 logs/web_auto.jsonl 을 지우지 않게 임시 폴더를 쓴다
config.LOG_DIR = tempfile.mkdtemp(prefix="adair_test_")
import httpx
from fastapi.testclient import TestClient

import web.app as webapp

ok = True


def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({info})" if info else ""))


calls = []
MODE = {"orch_up": True}


class _Resp:
    def __init__(self, code, data): self.status_code, self._d = code, data
    def json(self): return self._d


class FakeClient:
    """httpx.AsyncClient 대신 쓰는 가짜. 보낸 요청을 기록만 한다."""
    def __init__(self, *a, **kw): pass
    async def __aenter__(self): return self
    async def __aexit__(self, *a): return False

    async def post(self, url, json=None, **kw):
        calls.append((url, json))
        if url.startswith(config.ORCH_URL):
            if not MODE["orch_up"]:
                raise httpx.ConnectError("총괄 서버 없음")
            return _Resp(202, {"accepted": True, "created": True,
                               "task": {"task_id": "TASK-T1", "purpose_status": "HOLD",
                                        "hold_reason": "NO_FEASIBLE_CANDIDATE"}})
        if url.endswith("/evaluate"):
            return _Resp(200, {"verdict": "REJECT", "reason": "HIGH_WIND"})
        return _Resp(200, {})


webapp.httpx.AsyncClient = FakeClient
c = TestClient(webapp.app)
CELL = {"col": 108, "row": 42}

# 1) 스위치 켜짐: 총괄로만
config.WEB_DISPATCH_VIA_ORCH = True
calls.clear()
r = c.post("/api/fly", json=CELL).json()
orch = [x for x in calls if x[0].startswith(config.ORCH_URL)]
uav = [x for x in calls if webapp.UAV_AGENT in x[0]]
check("켜짐: 총괄 /tasks 로 1회 요청", len(orch) == 1 and orch[0][0].endswith("/tasks"), orch[0][0] if orch else "")
check("켜짐: UAV 서버는 직접 부르지 않음", len(uav) == 0)
body = orch[0][1] if orch else {}
check("요청 내용: 정찰·UAV·열화상", body.get("kind") == "RECON"
      and body.get("requirements") == {"resource_types": ["UAV"], "sensor": "THERMAL"})
t = body.get("target", {})
check("요청 내용: 목표 위경도·지면 해발고도·칸 번호", all(k in t for k in ("lat", "lon", "ground_amsl_m"))
      and t.get("cell_id") == "108_42", f"고도 {t.get('ground_amsl_m')} m")
check("응답: 접수 결과와 보류 사유를 그대로 전달", r.get("ok") and r.get("via") == "orch"
      and r.get("task_id") == "TASK-T1" and r.get("hold_reason") == "NO_FEASIBLE_CANDIDATE")

calls.clear()
c.post("/api/fly", json=CELL)
check("클릭마다 새 request_id", calls and calls[0][1]["request_id"] != body.get("request_id"))

# 2) 총괄 서버 없음: 실패로 답하고 UAV 로 우회하지 않음
MODE["orch_up"] = False
calls.clear()
r = c.post("/api/fly", json=CELL).json()
check("총괄 없음: 실패 응답(ORCH_UNREACHABLE)", r.get("ok") is False and r.get("reason") == "ORCH_UNREACHABLE")
check("총괄 없음: UAV 로 우회하지 않음", not any(webapp.UAV_AGENT in u for u, _ in calls))
MODE["orch_up"] = True

# 3) 스위치 꺼짐(기본): 기존 경로
config.WEB_DISPATCH_VIA_ORCH = False
calls.clear()
c.post("/api/fly", json=CELL)
check("꺼짐(기본): 총괄을 부르지 않고 기존처럼 UAV 직접", not any(u.startswith(config.ORCH_URL) for u, _ in calls)
      and any(u.endswith("/evaluate") for u, _ in calls))

print("\n모두 통과" if ok else "\n실패 항목 있음")
raise SystemExit(0 if ok else 1)   # 자동 시험(GitHub Actions)이 실패를 알아채도록
