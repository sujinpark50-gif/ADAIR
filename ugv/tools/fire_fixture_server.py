# -*- coding: utf-8 -*-
"""ugv/tools/fire_fixture_server.py — 환경 계약 /snapshot 형식으로 '정해 둔 화재 칸'을 내주는 시험용 서버.

UGV 는 운영과 같은 경로(UGV_ENV_URL → ugv/fire.py EnvHttpFireSource → GET /snapshot)로 화재를 읽는다.
화재 칸만 시험 입력으로 정한다 (환경 CA 확산 시뮬레이션이 아니다 — 보고서에 '시험 화재 입력'으로 표시).
넣은 칸은 모두 파일로 남긴다 (재현 입력).

    python -m uvicorn ugv.tools.fire_fixture_server:app --port 8310
    POST /set {"label": "...", "cells": [{cell_id, lat, lon, fire_state, cell_size_m}]}   → state_version 증가
    GET  /snapshot  {run_id, state_version, simulation_time_s, fire_cells, fixture: true}
"""

import json
import os
import time
import uuid
from pathlib import Path

from fastapi import Body, FastAPI

LOG = Path(os.getenv("FIRE_FIXTURE_LOG", Path(__file__).resolve().parents[2] / "logs" / "ugv" / "fire_fixture"))
app = FastAPI(title="ADAIR fire fixture (environment /snapshot contract)")
STATE = {"run_id": f"FIXTURE-{uuid.uuid4().hex[:8]}", "state_version": 0, "cells": [], "label": "EMPTY", "t0": time.time()}


@app.get("/health")
def health():
    return {"status": "ok", "fixture": True, "run_id": STATE["run_id"], "state_version": STATE["state_version"],
            "cells": len(STATE["cells"]), "label": STATE["label"]}


@app.get("/snapshot")
def snapshot():
    return {"run_id": STATE["run_id"], "state_version": STATE["state_version"],
            "simulation_time_s": round(time.time() - STATE["t0"], 1), "fire_cells": STATE["cells"],
            "fixture": True, "label": STATE["label"]}


@app.post("/set")
def set_cells(body: dict = Body(...)):
    STATE["cells"] = list(body.get("cells") or [])
    STATE["label"] = body.get("label") or "UNLABELED"
    STATE["state_version"] += 1
    LOG.mkdir(parents=True, exist_ok=True)
    f = LOG / f"fire_{time.strftime('%Y%m%d_%H%M%S')}_v{STATE['state_version']}.json"
    f.write_text(json.dumps({"run_id": STATE["run_id"], "state_version": STATE["state_version"], "label": STATE["label"],
                             "set_wall": time.time(), "fire_cells": STATE["cells"]}, ensure_ascii=False), encoding="utf-8")
    return {"state_version": STATE["state_version"], "cells": len(STATE["cells"]), "saved": str(f)}
