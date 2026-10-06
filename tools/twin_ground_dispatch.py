# -*- coding: utf-8 -*-
"""트윈 지상 출동 운용자 — 불이 확인되면 소방차를 **사람 있는 시설 앞**으로 보내 달라고 총괄에 요청한다.

야간 순찰(twin_operator.py)과 같은 자리의 '사람 역할' 스크립트다 (통합 담당, 시연 B 운용 가정).
2019 실제 기록: 일몰(19:06) 뒤 헬기 철수, 지상 인력으로 민가 방어 (강원 산불백서 142~146쪽).

    python tools/twin_ground_dispatch.py                       # run_servers.py --twin 이 함께 띄운다
    python tools/twin_ground_dispatch.py --trigger sunset      # 일몰 뒤에 출동 (2019 기록과 같은 시점)
    python tools/twin_ground_dispatch.py --site INJE_REST_AREA --types FIRE_ENGINE,UGV

하는 일 / 하지 않는 일
- 출동 시점: 총괄이 '불 확인(CONFIRMED)'을 알게 된 뒤 (드론 정찰 결과). 진짜 불 위치(환경 서버 정답)는 보지 않는다.
- 목적지: scenario.json 의 보호 시설 좌표(공개 정보) → 가장 가까운 트윈 칸. 경로는 UGV 서버가 도로망으로 정한다.
- 어떤 차량이 갈지, 갈 수 있는지, 안전한지는 총괄이 정한다. 이 스크립트는 임무를 '요청'만 한다 (POST /tasks).
- 실행(run) 하나에 시설당 한 번만 요청한다. 진화(물 뿌리기·불 끄기)는 하지 않는다 (ENV-07 미완).
"""
import argparse
import json
import math
import time
from pathlib import Path

import httpx

ROOT = Path(__file__).resolve().parents[1]
START_KST_MIN = 14 * 60 + 45                      # 트윈 시작 = 2019-04-04 14:45
SUNSET_S = (19 * 60 + 6 - START_KST_MIN) * 60     # 19:06 일몰 (twin_operator.py 와 같은 기준)


def load_sites() -> dict:
    d = json.loads((ROOT / "web" / "static" / "inje2019" / "scenario.json").read_text(encoding="utf-8"))
    return {s["site_id"]: s for s in d.get("sites", [])}


def nearest_cell(cells: list, lat: float, lon: float) -> dict:
    k = math.cos(math.radians(lat))
    return min(cells, key=lambda c: (c["lat"] - lat) ** 2 + ((c["lon"] - lon) * k) ** 2)


def confirmed_fires(cx: httpx.Client, orch: str) -> list:
    """총괄이 아는 불 중 확인된 칸 (드론 정찰 결과). 신고만 된 칸은 아직 아님."""
    known = cx.get(f"{orch}/priority/board").json().get("known") or {}
    return [f for f in known.get("fires", []) if str(f.get("status", "")).startswith("CONFIRMED")]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", default="http://127.0.0.1:8300")
    ap.add_argument("--orch", default="http://127.0.0.1:8200")
    ap.add_argument("--site", default="NAMJEON1_HALL", help="보호 시설 site_id (쉼표로 여러 개)")
    ap.add_argument("--types", default="FIRE_ENGINE", help="허용 자원 종류 (쉼표, 예: FIRE_ENGINE,UGV)")
    ap.add_argument("--trigger", choices=("confirmed", "sunset"), default="confirmed",
                    help="confirmed: 총괄이 불 확인을 알게 되면 / sunset: 거기에 더해 19:06 일몰 뒤")
    a = ap.parse_args()

    sites = load_sites()
    wanted = [s.strip() for s in a.site.split(",") if s.strip()]
    unknown = [s for s in wanted if s not in sites]
    if unknown:
        raise SystemExit(f"알 수 없는 시설: {unknown} (가능: {sorted(sites)})")
    types = [t.strip() for t in a.types.split(",") if t.strip()]

    cx = httpx.Client(timeout=30)
    cells, done = None, set()
    print(f"지상 출동 운용자 대기: 시설 {wanted}, 자원 {types}, 시점 {a.trigger}", flush=True)
    while True:
        try:
            s = cx.get(f"{a.env}/snapshot").json()
            run_id, t = s["run_id"], s["simulation_time_s"]
            if cells is None:
                cells = cx.get(f"{a.env}/map_cells").json()["cells"]
            if a.trigger == "sunset" and t < SUNSET_S:
                time.sleep(1.0); continue
            fires = confirmed_fires(cx, a.orch)
            if not fires:
                time.sleep(1.0); continue
            for sid in wanted:
                if (run_id, sid) in done:
                    continue
                site = sites[sid]
                c = nearest_cell(cells, site["lat"], site["lon"])
                body = {"request_id": f"OPERATOR-DEFEND:{run_id}:{sid}", "incident_id": "INC-INJE-2019",
                        "kind": "GROUND_SUPPORT",
                        "target": {k: c[k] for k in ("lat", "lon", "ground_amsl_m", "cell_id")},
                        "requirements": {"resource_types": types, "sensor": None}}
                r = cx.post(f"{a.orch}/tasks", json=body)
                print(f"sim {t / 3600:5.2f}h 불 확인 {[f['cell_id'] for f in fires][:3]} → "
                      f"{site['name']} 방어 출동 요청 (칸 {c['cell_id']}, {types}) → HTTP {r.status_code}", flush=True)
                if r.status_code < 500:
                    done.add((run_id, sid))
        except (httpx.HTTPError, KeyError, ValueError):
            time.sleep(2.0); continue
        time.sleep(1.0)


if __name__ == "__main__":
    main()
