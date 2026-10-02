# -*- coding: utf-8 -*-
"""
INT-04·05 통합 쪽 확인 (멘토 패치 0001·0004 와 연결)

INT-04 거점 좌표 단일 출처: config.BASE_* 가 environment/config/fire_stations.json 을 읽는다
INT-05 시계 하나: 관제판이 환경 계약 서버 모드(WEB_ENV_URL)면 /snapshot 을 읽기만 하고
       산불을 진행시키지 않으며, 관제판 자체 자동 루프·진화 시연을 끈다. 기록 시각은 환경 서버 시각.
run_servers.py --twin 구성: 트윈 시계만 /advance, 총괄은 팀 환경(team_http), 관제판은 읽기·총괄 경유

실행: python -m tests.int04_05_check   (서버 불필요 — 환경 서버 응답은 흉내 냄)
"""
import json
import tempfile

import config

config.LOG_DIR = tempfile.mkdtemp(prefix="adair_test_")

ok = True


def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({info})" if info else ""))


# ---- INT-04 --------------------------------------------------------------
st = json.load(open(config.FIRE_STATIONS_PATH, encoding="utf-8"))["stations"]
a = next(s for s in st if s["base_id"] == "A")
check("거점 좌표를 fire_stations.json 에서 읽음", config.STATIONS_SOURCE == "fire_stations.json"
      and (config.BASE_A_LAT, config.BASE_A_LON) == (a["lat"], a["lon"]), f"A {config.BASE_A_LAT}, {config.BASE_A_LON}")
check("파일이 없으면 기존 config 값으로 동작", config._load_stations("없는/경로.json") == {})

# ---- INT-05 (관제판, 환경 서버 모드) ---------------------------------------
import httpx
import web.app as webapp
from fastapi.testclient import TestClient

SNAP = {"run_id": "R", "state_version": 7, "simulation_time_s": 4200.0, "step_count": 2, "tick_s": 2100.0,
        "wind_ms": 6.3, "wind_dir_deg": 160.0,
        "fire_cells": [{"cell_id": "13_101", "risk_score": 1.0}, {"cell_id": "14_101", "risk_score": 0.9}],
        "risk_cells": [{"cell_id": "15_100", "risk_score": 0.2}, {"cell_id": "16_99", "risk_score": 0.6}]}
calls = []
real_get = httpx.get


class _R:
    def __init__(self, d): self._d = d
    def json(self): return self._d


def fake_get(url, *a, **k):
    calls.append(("GET", url))
    if url.startswith("http://env.test"):
        return _R(SNAP)
    return real_get(url, *a, **k)


webapp.httpx.get = fake_get
config.WEB_ENV_URL = "http://env.test"
c = TestClient(webapp.app)

tick0 = webapp._tick
e = c.get("/api/env").json()
check("관제판이 환경 서버 스냅샷을 그대로 표시", e["source"] == "ENV_SERVER" and e["fire"] == [{"x": 13, "y": 101}, {"x": 14, "y": 101}]
      and e["simulation_time_s"] == 4200.0, f"불 {len(e['fire'])}칸, 시뮬 {e['simulation_time_s']}초")
check("위험 칸은 위험도 높은 순", e["risk"][0] == {"x": 16, "y": 99})
for _ in range(5):
    c.get("/api/env")
check("관제판은 산불을 진행시키지 않음 (/advance 호출 없음, 자체 시계 그대로)",
      webapp._tick == tick0 and not any("advance" in u for _, u in calls))
r = c.post("/api/auto/toggle").json()
check("관제판 자체 자동 루프 꺼짐 (자동 판단은 총괄)", r.get("on") is False and r.get("disabled") is True)
config.WEB_DEMO_EXTINGUISH = True
r = c.post("/api/extinguish", json={"col": 13, "row": 101}).json()
check("환경 서버 모드에서는 시연 진화도 환경을 바꾸지 않음", r.get("disabled") is True)
config.WEB_DEMO_EXTINGUISH = False
check("기록 시각 = 환경 서버 시뮬레이션 시각", webapp._current_sim_time() == 4200.0)
m = c.get("/api/modes").json()
check("상단 표시: 산불 시계·거점 좌표 출처", m["env_url"] == "http://env.test" and m["stations_source"] == "fire_stations.json")

config.WEB_ENV_URL = ""
r = c.post("/api/auto/toggle").json()
check("환경 서버 주소가 없으면 기존 자동 루프 그대로", r.get("disabled") is None)
if r.get("on"):
    c.post("/api/auto/toggle")

# ---- run_servers.py --twin 구성 -------------------------------------------
import run_servers as rs
d = rs.server_defs({"twin": True, "twin_dir": tempfile.mkdtemp(), "twin_speed": 100})
check("트윈 구성: 환경 서버·시계·순찰 포함", all(n in d for n in ("env", "clock", "operator")))
check("트윈 구성: /advance 는 시계만", d["clock"]["cmd"][1].endswith("twin_clock.py")
      and "WEB_ENV_URL" in d["web"]["env"] and d["web"]["env"].get("WEB_DISPATCH_VIA_ORCH") == "1")
check("트윈 구성: 총괄은 팀 환경(team_http), 발화점·시각은 시나리오 값",
      d["orch"]["env"].get("ORCH_ENV_MODE") == "team_http" and d["env"]["env"].get("ENV_TICK_S") == "2100")

print("\n모두 통과" if ok else "\n실패 항목 있음")
raise SystemExit(0 if ok else 1)
