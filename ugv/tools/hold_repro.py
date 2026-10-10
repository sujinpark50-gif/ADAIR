# -*- coding: utf-8 -*-
"""ugv/tools/hold_repro.py — PX4 rover(adair_ugv) 정차 중 MAVLink HOLD 가 차를 움직이는지 재현 (2026-10-10 사고 확인용).

순서: 앞 25 m 지점 미션 → 끝나고 정차 확인 10 s → action.hold() → 20 s 위치 기록 → 지금 위치 한 점 미션으로 세움.
    python -m ugv.tools.hold_repro [udpin://0.0.0.0:14541]
결과: logs/ugv/hold_repro_<시각>.json
"""
import asyncio
import json
import math
import sys
import time

from ugv.drivers.px4 import Px4Driver
from ugv.clock import SimClock
from ugv.tools.drive_test import LOG_DIR


async def main(addr):
    d = Px4Driver(addr, SimClock())
    await d.start()
    rec = []

    def snap(tag):
        t = d.telemetry()
        rec.append({"t": round(time.time(), 2), "tag": tag, "x": round(t.x, 2), "y": round(t.y, 2),
                    "speed": round(t.speed_mps, 2), "heading": round(t.heading_deg, 1), "mode": t.mode})
    t = d.telemetry()
    hd = math.radians(t.heading_deg)
    await d.follow([(t.x + 25 * math.sin(hd), t.y + 25 * math.cos(hd), 3.0)])
    for _ in range(60):
        await asyncio.sleep(1)
        snap("mission")
        if d.progress().finished and d.telemetry().speed_mps < 0.1:
            break
    if PARK_CHECK:
        # 수정 후 확인: 드라이버가 미션 끝 정차 뒤 시동을 꺼 정차를 유지하는지 (60 s)
        for _ in range(60):
            await asyncio.sleep(1)
            snap("after_mission_parked")
        idle = [r for r in rec if r["tag"] == "after_mission_parked"]
        res = {"address": addr, "mode": "PARK_CHECK", "records": rec, "parked_seq": d.parked_seq, "armed_end": d._armed,
               "moved_m_60s": round(math.dist((idle[0]["x"], idle[0]["y"]), (idle[-1]["x"], idle[-1]["y"])), 2),
               "max_speed_60s": max(r["speed"] for r in idle)}
        out = LOG_DIR / f"park_check_{time.strftime('%Y%m%d_%H%M%S')}.json"
        out.write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
        print(json.dumps({k: v for k, v in res.items() if k != "records"}, ensure_ascii=False), out)
        await d.close()
        return
    for _ in range(10):
        await asyncio.sleep(1)
        snap("after_mission_idle")
    await d._act.hold()
    for _ in range(20):
        await asyncio.sleep(1)
        snap("after_HOLD")
    t = d.telemetry()
    await d.follow([(t.x, t.y, 1.0)])
    for _ in range(15):
        await asyncio.sleep(1)
        snap("one_point_mission_stop")
    idle = [r for r in rec if r["tag"] == "after_mission_idle"]
    hold = [r for r in rec if r["tag"] == "after_HOLD"]
    res = {"address": addr, "records": rec,
           "idle_moved_m": round(math.dist((idle[0]["x"], idle[0]["y"]), (idle[-1]["x"], idle[-1]["y"])), 2),
           "hold_moved_m": round(math.dist((hold[0]["x"], hold[0]["y"]), (hold[-1]["x"], hold[-1]["y"])), 2),
           "hold_max_speed": max(r["speed"] for r in hold)}
    out = LOG_DIR / f"hold_repro_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps({k: v for k, v in res.items() if k != "records"}, ensure_ascii=False), out)
    await d.close()


PARK_CHECK = "--park" in sys.argv

if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    asyncio.run(main(args[0] if args else "udpin://0.0.0.0:14541"))
