# -*- coding: utf-8 -*-
"""
INT-02 수락 확인 (총괄 팀별 요청서): 도착 즉시 진화 제거

- 기본(WEB_DEMO_EXTINGUISH 꺼짐): /api/extinguish, /api/auto/done 을 불러도 화재 칸이 그대로여야 한다
- 시연 스위치를 켜면(WEB_DEMO_EXTINGUISH=1): 예전처럼 진화가 반영된다

실행: python -m tests.int02_extinguish_check   (UAV·UGV 서버 불필요)
"""
import tempfile

import config

# 관제판을 불러오면 로그 파일을 새로 만들므로, 실제 logs/web_auto.jsonl 을 지우지 않게 임시 폴더를 쓴다
config.LOG_DIR = tempfile.mkdtemp(prefix="adair_test_")
from fastapi.testclient import TestClient

import web.app as webapp
from connectors import fire_connector

ok = True


def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({info})" if info else ""))


def burning():
    es = fire_connector.get_environment_state(0.0, advance=False)
    return {(c["x"], c["y"]) for c in es.fire_cells}


c = TestClient(webapp.app)
c.get("/api/env")                       # 엔진 준비 (점화)
cells = burning()
x, y = sorted(cells)[0]

# 1) 기본: 진화하지 않음
config.WEB_DEMO_EXTINGUISH = False
r = c.post("/api/extinguish", json={"col": x, "row": y}).json()
check("기본 설정에서 /api/extinguish 는 비활성 응답", r.get("disabled") is True, r.get("reason"))
check("도착 처리 뒤에도 그 칸은 계속 연소 중", (x, y) in burning(), f"칸 {x},{y}")

webapp._AUTO["target"] = {"col": x, "row": y}
c.post("/api/auto/done")
check("자동 모드 도착(/api/auto/done)도 환경을 바꾸지 않음", (x, y) in burning())

# 2) 시연 스위치를 켜면 예전 동작
config.WEB_DEMO_EXTINGUISH = True
r = c.post("/api/extinguish", json={"col": x, "row": y}).json()
check("시연 스위치를 켜면 진화 반영", r.get("ok") is True and (x, y) not in burning(), f"칸 {x},{y}")
config.WEB_DEMO_EXTINGUISH = False

print("\n모두 통과" if ok else "\n실패 항목 있음")
raise SystemExit(0 if ok else 1)   # 자동 시험(GitHub Actions)이 실패를 알아채도록
