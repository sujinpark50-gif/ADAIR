# -*- coding: utf-8 -*-
"""
관제 로그 보강 확인 (통합 담당)

1) 폐루프 로그: ENVIRONMENT_UPDATE·TASK_COMPLETE 에도 decision_id 가 남는다
   (이 줄만 보고 어떤 판단의 결과인지 알 수 있어야 한다)
2) 관제판 수동 출동이 로그 파일에 남는다: TASK_CREATED → LOCAL_RESPONSE → EXECUTION (출처 WEB_MANUAL)
3) 차량 도착이 로그 파일에 남는다: ARRIVAL (진화 반영 여부 포함)
4) 총괄 경유 출동은 접수 결과가 남는다: ORCH_TASK_SUBMITTED (ACCEPTED / FAILED)

실행: python -m tests.log_coverage_check   (서버 불필요 — 외부 응답은 흉내 냄)
"""
import io
import json
import contextlib

import tempfile

import config
import httpx

# 실제 logs/ 의 기록(관제판이 쓰는 web_auto.jsonl 포함)을 덮어쓰지 않도록 임시 폴더에 쓴다
config.LOG_DIR = tempfile.mkdtemp(prefix="adair_logtest_")

ok = True


def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({info})" if info else ""))


def read(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(l) for l in f if l.strip()]


# ---- 1) 폐루프 (mock) -------------------------------------------------------
import main
with contextlib.redirect_stdout(io.StringIO()):
    main.run_simulation(total_steps=3)
rows = read(f"{config.LOG_DIR}/{config.LOG_FILE_NAME}")
tail = [r for r in rows if r["event_type"] in ("ENVIRONMENT_UPDATE", "TASK_COMPLETE")
        and r["result"] != "CANCELLED" and r.get("reason") not in ("TARGET_UNRESOLVED", "NO_FIRE_TARGET")]
check("폐루프: ENVIRONMENT_UPDATE·TASK_COMPLETE 에 decision_id", tail and all(r["decision_id"] for r in tail),
      f"{len(tail)}줄")
done = [r for r in tail if r["event_type"] == "TASK_COMPLETE" and r["result"] == "COMPLETED"]
check("폐루프: 완료 줄에 수행 자원(resource_id)", all(r["resource_id"] for r in done), f"{len(done)}건")

# ---- 2~4) 관제판 -----------------------------------------------------------
import web.app as webapp
from fastapi.testclient import TestClient

MODE = {"orch_up": True}


class _R:
    def __init__(self, code, d): self.status_code, self._d = code, d
    def json(self): return self._d


class FakeClient:
    def __init__(self, *a, **k): pass
    async def __aenter__(self): return self
    async def __aexit__(self, *a): return False

    async def post(self, url, json=None, **k):
        if url.startswith(config.ORCH_URL):
            if not MODE["orch_up"]:
                raise httpx.ConnectError("총괄 없음")
            return _R(202, {"created": True, "task": {"task_id": "TASK-T9", "purpose_status": "HOLD",
                                                      "hold_reason": "NO_FEASIBLE_CANDIDATE"}})
        if url.endswith("/evaluate"):
            return _R(200, {"verdict": "ACCEPT", "eta_sec": 120, "constraints": {"distance_km": 1.2}})
        return _R(200, {"task_id": json.get("task_id") if json else None})


webapp.httpx.AsyncClient = FakeClient
c = TestClient(webapp.app)
LOG = webapp._AUTO_LOGGER.log_path
n0 = len(read(LOG))

config.WEB_DISPATCH_VIA_ORCH = False
c.post("/api/fly", json={"col": 108, "row": 42})
new = read(LOG)[n0:]
seq = [(r["event_type"], r["result"]) for r in new if (r.get("detail") or {}).get("source") == "WEB_MANUAL"]
check("수동 출동: TASK_CREATED → LOCAL_RESPONSE → EXECUTION 기록",
      [e for e, _ in seq] == ["TASK_CREATED", "LOCAL_RESPONSE", "EXECUTION"], " → ".join(e for e, _ in seq))
lr = next((r for r in new if r["event_type"] == "LOCAL_RESPONSE"), {})
check("수동 출동: 판단 근거(evidence)와 decision_id", lr.get("decision_id") and (lr.get("detail") or {}).get("evidence"))
ex = next((r for r in new if r["event_type"] == "EXECUTION"), {})
check("수동 출동: 총괄 Safety 를 거치지 않았음을 표시", ex.get("reason") == "MANUAL_DIRECT_NO_SAFETY")

n1 = len(read(LOG))
config.WEB_DEMO_EXTINGUISH = False
c.post("/api/extinguish", json={"col": 108, "row": 42, "resource_id": "A-fire1"})
arr = [r for r in read(LOG)[n1:] if r["event_type"] == "ARRIVAL"]
check("도착: ARRIVAL 기록 (자원 이름·진화 미반영)", arr and arr[0]["resource_id"] == "A-fire1"
      and arr[0]["detail"]["extinguish_applied"] is False)

n2 = len(read(LOG))
webapp._AUTO["target"] = {"col": 108, "row": 42}
c.post("/api/auto/done")
arr = [r for r in read(LOG)[n2:] if r["event_type"] == "ARRIVAL"]
check("자동 모드 도착도 ARRIVAL 기록", arr and arr[0]["detail"]["source"] == "WEB_AUTO")

n3 = len(read(LOG))
config.WEB_DISPATCH_VIA_ORCH = True
c.post("/api/fly", json={"col": 108, "row": 42})
MODE["orch_up"] = False
c.post("/api/fly", json={"col": 108, "row": 42})
sub = [(r["result"], r.get("task_id"), r.get("reason")) for r in read(LOG)[n3:] if r["event_type"] == "ORCH_TASK_SUBMITTED"]
check("총괄 경유: 접수 결과 기록 (ACCEPTED·총괄 임무 번호)", sub and sub[0][0] == "ACCEPTED" and sub[0][1] == "TASK-T9")
check("총괄 경유: 총괄 없음도 기록 (FAILED·ORCH_UNREACHABLE)", len(sub) > 1 and sub[1][0] == "FAILED"
      and sub[1][2] == "ORCH_UNREACHABLE")
config.WEB_DISPATCH_VIA_ORCH = False

print("\n모두 통과" if ok else "\n실패 항목 있음")
raise SystemExit(0 if ok else 1)   # 자동 시험(GitHub Actions)이 실패를 알아채도록
