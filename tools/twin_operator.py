# -*- coding: utf-8 -*-
"""트윈 운용자 — 일몰 뒤 드론 순찰을 **총괄 공식 API(POST /tasks)** 로 요청한다 (시나리오 B 운용 가정).

총괄은 신고가 오면 최초 정찰(RECON)만 스스로 만든다. 밤새 화선을 도는 것은 운용 정책이라 사람이 요청하는 몫이다.
이 스크립트가 그 사람 역할을 한다.

정답지 방지: 운용자는 **진짜 불 위치를 보지 않는다.** 신고 칸을 중심으로 스텝마다 반경을 넓히는 탐색 고리를 돌고,
고리 위 칸 중 바람이 불어가는 쪽(기상청 관측, 공개 정보)을 먼저 고른다. 무엇을 찾았는지는 총괄이 관측으로 안다.

    python tools/twin_operator.py               # run_twin.sh 가 함께 띄운다 (TWIN_OPERATOR=0 이면 끔)
"""
import argparse
import math
import time

import httpx

NIGHT_FROM_S, NIGHT_TO_S = (19 * 60 + 6 - (14 * 60 + 45)) * 60, (24 * 60 + 6 * 60 + 10 - (14 * 60 + 45)) * 60


def ring(c0, r0, r, n, wind_from_deg):
    cells = [(c0 + dx, r0 + dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1) if max(abs(dx), abs(dy)) == r]
    to = math.radians((wind_from_deg + 180) % 360)
    ux, uy = math.sin(to), -math.cos(to)                    # 격자: 행이 아래(남)로 증가

    def downwind(c):
        dx, dy = c[0] - c0, c[1] - r0
        d = math.hypot(dx, dy) or 1
        return -(dx * ux + dy * uy) / d
    cells.sort(key=downwind)
    return cells[:n]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", default="http://127.0.0.1:8300")
    ap.add_argument("--orch", default="http://127.0.0.1:8200")
    ap.add_argument("--cells", type=int, default=6, help="스텝마다 순찰할 칸 수")
    a = ap.parse_args()
    cx = httpx.Client(timeout=30)
    done = set()
    while True:
        try:
            s = cx.get(f"{a.env}/snapshot").json()
            reps = cx.get(f"{a.env}/reports", params={"until": s["simulation_time_s"]}).json()["reports"]
        except httpx.HTTPError:
            time.sleep(2); continue
        t, k = s["simulation_time_s"], s["step_count"]
        if reps and NIGHT_FROM_S <= t < NIGHT_TO_S and (s["run_id"], k) not in done:
            c0, r0 = map(int, reps[0]["cell_id"].split("_"))
            r = 1 + int((t - NIGHT_FROM_S) // s["tick_s"]) % 6         # 1~6칸 고리를 반복 (진짜 불 위치를 쓰지 않음)
            cells = [f"{c}_{rr}" for c, rr in ring(c0, r0, r, a.cells, s["wind_dir_deg"])]
            body = {"request_id": f"OPERATOR-PATROL:{s['run_id']}:{k}", "incident_id": "INC-NIGHT-PATROL",
                    "kind": "MONITOR",
                    "area_cell_ids": cells, "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL"}}
            # 목표 지점 = 고리의 첫 칸 (지도 정보로 좌표·지면고도)
            ms = {c["cell_id"]: c for c in _map(cx, a.env)}
            first = ms[cells[0]]
            body["target"] = {k2: first[k2] for k2 in ("lat", "lon", "ground_amsl_m", "cell_id")}
            r2 = cx.post(f"{a.orch}/tasks", json=body)
            print(f"sim {t / 3600:5.2f}h 순찰 요청 반경 {r}칸 {cells} → HTTP {r2.status_code}")
            done.add((s["run_id"], k))
        time.sleep(1.0)


_MAP = None


def _map(cx, env):
    global _MAP
    if _MAP is None:
        _MAP = cx.get(f"{env}/map_cells").json()["cells"]
    return _MAP


if __name__ == "__main__":
    main()
