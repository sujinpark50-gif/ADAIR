# -*- coding: utf-8 -*-
"""ugv/tools/heading_drift.py — 정차 중 PX4 방위 추정과 Gazebo 실제 방위 차이를 시간에 따라 기록한다.

    python -m ugv.tools.heading_drift --world kangwon_terrain_a --model adair_ugv_1 --instance-addr udpin://0.0.0.0:14541 --minutes 4
결과: logs/ugv/heading_drift_<시각>.json
"""
import argparse
import asyncio
import json
import subprocess
import time

from ugv.clock import SimClock
from ugv.drivers.px4 import Px4Driver
from ugv.tools.drive_test import LOG_DIR, ROOT, to_wsl_path


def gz_heading(world, model):
    sh = to_wsl_path(ROOT / "ugv" / "tools" / "wsl_gz_yaw.sh")
    out = subprocess.run(["wsl.exe", "-d", "Ubuntu-24.04", "--", "bash", sh, world, model], capture_output=True, text=True).stdout
    return float(out.strip().splitlines()[-1])


async def main(a):
    d = Px4Driver(a.addr, SimClock())
    await d.start()
    rec = []
    t0 = time.time()
    while time.time() - t0 < a.minutes * 60:
        tel = d.telemetry()
        gz = gz_heading(a.world, a.model)
        err = (tel.heading_deg - gz + 180) % 360 - 180
        rec.append({"t_s": round(time.time() - t0, 1), "px4_deg": round(tel.heading_deg, 1), "gazebo_deg": gz,
                    "err_deg": round(err, 2), "speed": round(tel.speed_mps, 2)})
        print(rec[-1], flush=True)
        await asyncio.sleep(a.every)
    res = {"world": a.world, "model": a.model, "records": rec,
           "err_first": rec[0]["err_deg"], "err_last": rec[-1]["err_deg"],
           "drift_deg_per_min": round((rec[-1]["err_deg"] - rec[0]["err_deg"]) / max(rec[-1]["t_s"] / 60, 1e-6), 2)}
    out = LOG_DIR / f"heading_drift_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print({k: v for k, v in res.items() if k != "records"}, out)
    await d.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="kangwon_terrain_a")
    ap.add_argument("--model", default="adair_ugv_1")
    ap.add_argument("--addr", default="udpin://0.0.0.0:14541")
    ap.add_argument("--minutes", type=float, default=4)
    ap.add_argument("--every", type=float, default=20)
    asyncio.run(main(ap.parse_args()))
