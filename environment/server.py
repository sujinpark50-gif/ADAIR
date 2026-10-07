# -*- coding: utf-8 -*-
"""환경 계약 서버 (ENV-01~06) — 총괄·웹이 같은 환경 인스턴스 하나를 공유하기 위한 HTTP 창구.

실행 (저장소 루트에서):
    python -m uvicorn environment.server:app --host 127.0.0.1 --port 8300

포트 8300 (UAV 8000/8001, UGV 8100, 총괄 8200, 웹 8080 과 겹치지 않게 — 임시, INT-03 에서 합의).

| 메서드 | 경로 | 계약 |
|---|---|---|
| GET  | /health          | run·버전·가정값·바람 출처 |
| GET  | /snapshot        | READ — 상태 불변 |
| POST | /advance         | ADVANCE {"steps": n} — 시계 소유자(INT-05)만 부른다 |
| POST | /apply           | APPLY 관측 → ACK (중복·충돌·재시작 후 중복 방지) |
| GET  | /map_cells       | 정적 지도 (map_version 이 같으면 캐시해도 된다) |
| GET  | /stations        | 소방서 좌표 (INT-04 후보 단일 출처) |
| GET  | /reports?until=t | 그 시각까지의 시나리오 신고 (ENV-06) |
| POST | /analyze         | ENV-04 분석 — 입력은 총괄의 belief 뿐 |
| POST | /reset           | 새 시뮬레이션 → 새 run_id |

환경 변수: ENV_STATE_DIR(기본 environment/.state), ENV_TICK_S(60), ENV_SCENARIO_START_KST,
ENV_SEED(42), ENV_IGNITION("19,142"), ENV_REPORT_DELAY_S(0), ENV_RESUME(1),
ENV_SUBTICK_S(비움 = 끔. 예: 60 이면 /advance 한 번에 1분, CA 단계 사이 점화 시각을 채워 넣음 — 총괄 브랜치 2026-10-07).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from fastapi import Body, FastAPI, HTTPException

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from src.belief_analysis import BeliefSpreadAnalysis   # noqa: E402
from src.contract_env import ContractEnvironment       # noqa: E402


def build_env() -> ContractEnvironment:
    ign = os.getenv("ENV_IGNITION", "19,142")
    ignitions = [tuple(int(v) for v in p.split(",")) for p in ign.split(";") if p.strip()]
    return ContractEnvironment(
        os.getenv("ENV_STATE_DIR", str(HERE / ".state")),
        tick_s=float(os.getenv("ENV_TICK_S", "60")),
        scenario_start_kst=os.getenv("ENV_SCENARIO_START_KST", "2019-04-04T14:45:00+09:00"),
        seed=int(os.getenv("ENV_SEED", "42")), ignitions=ignitions,
        report_delay_s=float(os.getenv("ENV_REPORT_DELAY_S", "0")),
        resume=os.getenv("ENV_RESUME", "1") != "0",
        subtick_s=float(os.getenv("ENV_SUBTICK_S")) if os.getenv("ENV_SUBTICK_S") else None)


def build_analysis(env: ContractEnvironment) -> BeliefSpreadAnalysis:
    import copy
    return BeliefSpreadAnalysis(env.static, copy.deepcopy(env.engine._config), tick_s=env.tick_s,
                                horizon_s=float(os.getenv("ENV_FORECAST_HORIZON_S", "1800")))


def create_app(env: ContractEnvironment, analysis: BeliefSpreadAnalysis) -> FastAPI:
    app = FastAPI(title="ADAIR Environment Contract", version="0.1")

    @app.get("/health")
    def health():
        return env.health()

    @app.get("/snapshot")
    def snapshot():
        return env.snapshot()

    @app.post("/advance")
    def advance(body: dict = Body(default={})):
        steps = body.get("steps", 1)
        try:
            return env.advance(steps)
        except ValueError as e:
            raise HTTPException(422, str(e))

    @app.post("/apply")
    def apply(observation: dict = Body(...)):
        return env.apply(observation)

    @app.get("/map_cells")
    def map_cells():
        return {"map_version": env.map_version, "cells": env.map_cells()}

    @app.get("/stations")
    def stations():
        return env.stations_doc

    @app.get("/reports")
    def reports(until: float):
        return {"run_id": env.run_id, "reports": env.reports_until(until)}

    @app.post("/analyze")
    def analyze(belief: dict = Body(...)):
        return analysis.analyze(belief)

    @app.post("/reset")
    def reset():
        return env.reset()

    return app


if os.getenv("ENV_SERVER_NO_AUTOBUILD") != "1":
    _env = build_env()
    app = create_app(_env, build_analysis(_env))
