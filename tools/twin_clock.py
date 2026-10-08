# -*- coding: utf-8 -*-
"""디지털 트윈 시계 — 진짜 세계(환경 계약 서버)를 진행시키는 **유일한** 주체 (INT-05: 시계 소유자 하나).

    python tools/twin_clock.py                # 100배속: CA 한 스텝(2100 s)마다 실제 21초
    python tools/twin_clock.py --speed 1      # 실시간 (PX4 real 비행과 같은 시간 축)

배속은 드론 쪽 시간과 맞춰야 한다. uav-agent mock 은 MOCK_TIME_SCALE=100 배속으로 날기 때문에 기본값도 100.
PX4 real 로 날리면 --speed 1 (또는 Gazebo 실시간 배속에 맞춘 값)을 쓴다.

총괄이 LLM 을 기다리는 동안(/health llm.in_flight > 0)은 진행하지 않는다 (사용자 결정 2026-10-08). 100배속에서
LLM 60초가 시뮬레이션 1시간 40분이 되어, 실제 운용보다 불이 훨씬 더 번진 뒤에야 계획이 나오는 왜곡을 막는다.
끄기: --no-llm-pause. 총괄이 응답하지 않으면 멈추지 않고 진행한다.
주의: 드론·UGV mock 은 각자 실제 시각으로 움직이므로 시계가 멈춘 동안에도 비행·주행은 계속된다.
"""
import argparse
import time

import httpx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", default="http://127.0.0.1:8300")
    ap.add_argument("--speed", type=float, default=100.0, help="시뮬레이션 초 / 실제 초")
    ap.add_argument("--until-h", type=float, default=32.25, help="시나리오 시작 후 몇 시간까지 (기상청 자료 끝 = 32.25)")
    ap.add_argument("--orch", default="http://127.0.0.1:8200", help="총괄 주소 (LLM 기다리는 동안 멈춤)")
    ap.add_argument("--no-llm-pause", action="store_true", help="LLM 을 기다려도 멈추지 않는다")
    a = ap.parse_args()
    cx = httpx.Client(timeout=30)
    h = cx.get(f"{a.env}/health").json()
    # 단계 사이 채우기(ENV_SUBTICK_S)가 켜져 있으면 /advance 한 번 = subtick 초 (총괄 브랜치 2026-10-07)
    tick = h["assumptions"].get("subtick_s") or h["assumptions"]["tick_s"]
    print(f"run {h['run_id']} · 한 번 진행 {tick:.0f}s (CA 단계 {h['assumptions']['tick_s']:.0f}s) · "
          f"{a.speed:g}배속 → 실제 {tick / a.speed:.1f}s 마다 진행")
    while True:
        s = cx.get(f"{a.env}/snapshot").json()
        if s["simulation_time_s"] >= a.until_h * 3600:
            print("시나리오 끝"); break
        time.sleep(tick / a.speed)
        if not a.no_llm_pause:
            _wait_llm(cx, a.orch)
        s = cx.post(f"{a.env}/advance", json={"steps": 1}).json()
        burning = sum(1 for c in s["fire_cells"] if c["fire_state"] == "BURNING")
        print(f"  sim {s['simulation_time_s'] / 3600:5.2f}h  v{s['state_version']}  타는 칸 {burning:4d}  "
              f"바람 {s['wind_ms']} m/s {s['wind_dir_deg']:.0f}°")


def _wait_llm(cx, orch):
    """총괄이 LLM 을 기다리는 동안 진행하지 않는다. 총괄이 응답하지 않으면 바로 진행."""
    paused = None
    while True:
        try:
            busy = (cx.get(f"{orch}/health", timeout=5).json().get("llm") or {}).get("in_flight")
        except (httpx.HTTPError, ValueError):
            busy = 0
        if not busy:
            if paused is not None:
                print(f"  LLM 응답 — 시계 다시 진행 (멈춘 시간 {time.monotonic() - paused:.0f}s)")
            return
        if paused is None:
            paused = time.monotonic()
            print("  총괄 LLM 대기 중 — 시계 멈춤")
        time.sleep(0.5)


if __name__ == "__main__":
    main()
