# -*- coding: utf-8 -*-
"""ugv/tools/park_restart_test.py — 주행 → 정차(시동 끔) N 분 → 재출발 시 초기 경로 이탈 (정차 중 방위 추정 틀어짐 재현).

    python -m ugv.tools.park_restart_test --tag before --park-min 10       (서버·Gazebo 는 미리 띄운다, B 월드 기준 목표)
1구간: 목표 A 로 갔다가 거점 복귀 (road_test), 정차 동안 PX4 방위 vs Gazebo 방위 기록, 2구간: 목표 B (+ 22 s 뒤 취소).
"""
import argparse
import json
import os
import subprocess
import sys
import time

import httpx

from ugv.tools.drive_test import LOG_DIR, ROOT
from ugv.tools.heading_drift import gz_heading


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--park-min", type=float, default=10)
    ap.add_argument("--world", default="kangwon_terrain_a")
    ap.add_argument("--leg1", default="RP:682502008:57.4")
    ap.add_argument("--leg2", default="RP:682502546:39.9")
    a = ap.parse_args()
    env = dict(os.environ)
    py = sys.executable
    tag1, tag2 = f"PR_{a.tag}_leg1", f"PR_{a.tag}_leg2"
    subprocess.run([py, "-m", "ugv.tools.road_test", "--tag", tag1, "--target-node", a.leg1, "--return-home",
                    "--track-s", "600", "--world", a.world], env=env, check=False, cwd=ROOT)
    c = httpx.Client(base_url="http://127.0.0.1:8100", timeout=30, headers={"X-ADAIR-Monitor": "1"})
    drift = []
    t0 = time.time()
    while time.time() - t0 < a.park_min * 60:
        st = c.get("/ugv/A-ugv1/state").json()
        gz = gz_heading(a.world, "adair_ugv_1")
        drift.append({"t_s": round(time.time() - t0), "px4_deg": st["heading_deg"], "gazebo_deg": gz,
                      "err_deg": round((st["heading_deg"] - gz + 180) % 360 - 180, 2), "speed": st["speed_mps"]})
        print(drift[-1], flush=True)
        time.sleep(30)
    subprocess.run([py, "-m", "ugv.tools.road_test", "--tag", tag2, "--target-node", a.leg2, "--abort-after", "22",
                    "--track-s", "200", "--world", a.world], env=env, check=False, cwd=ROOT)
    out = LOG_DIR / f"park_restart_{a.tag}_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.write_text(json.dumps({"tag": a.tag, "park_min": a.park_min, "drift": drift}, ensure_ascii=False, indent=1),
                   encoding="utf-8")
    print("기록", out)


if __name__ == "__main__":
    main()
