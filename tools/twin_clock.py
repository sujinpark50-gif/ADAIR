# -*- coding: utf-8 -*-
"""디지털 트윈 시계 — 진짜 세계(환경 계약 서버)를 진행시키는 **유일한** 주체 (INT-05: 시계 소유자 하나).

    python tools/twin_clock.py                # 100배속: CA 한 스텝(2100 s)마다 실제 21초
    python tools/twin_clock.py --speed 1      # 실시간 (PX4 real 비행과 같은 시간 축)

배속은 드론 쪽 시간과 맞춰야 한다. uav-agent mock 은 MOCK_TIME_SCALE=100 배속으로 날기 때문에 기본값도 100.
PX4 real 로 날리면 --speed 1 (또는 Gazebo 실시간 배속에 맞춘 값)을 쓴다.
"""
import argparse
import time

import httpx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", default="http://127.0.0.1:8300")
    ap.add_argument("--speed", type=float, default=100.0, help="시뮬레이션 초 / 실제 초")
    ap.add_argument("--until-h", type=float, default=32.25, help="시나리오 시작 후 몇 시간까지 (기상청 자료 끝 = 32.25)")
    a = ap.parse_args()
    cx = httpx.Client(timeout=30)
    h = cx.get(f"{a.env}/health").json()
    tick = h["assumptions"]["tick_s"]
    print(f"run {h['run_id']} · 한 스텝 {tick:.0f}s · {a.speed:g}배속 → 실제 {tick / a.speed:.1f}s 마다 진행")
    while True:
        s = cx.get(f"{a.env}/snapshot").json()
        if s["simulation_time_s"] >= a.until_h * 3600:
            print("시나리오 끝"); break
        time.sleep(tick / a.speed)
        s = cx.post(f"{a.env}/advance", json={"steps": 1}).json()
        burning = sum(1 for c in s["fire_cells"] if c["fire_state"] == "BURNING")
        print(f"  sim {s['simulation_time_s'] / 3600:5.2f}h  v{s['state_version']}  타는 칸 {burning:4d}  "
              f"바람 {s['wind_ms']} m/s {s['wind_dir_deg']:.0f}°")


if __name__ == "__main__":
    main()
