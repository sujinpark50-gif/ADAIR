#!/usr/bin/env python3
"""시연: 원통119 → 기린119 를 능선(최고 986 m) 위로 넘어 비행한다. Gazebo 강원 월드 전용.

  1) 현재 위치에서 제자리 상승 → CRUISE_AMSL
  2) CRUISE_AMSL 로 기린까지 직선 비행
  3) 기린 지면 + 60 m 로 하강 후 호버링 (착륙하지 않음)

⚠️ 시뮬레이션 전용. 1,050 m AMSL 일정 고도 비행은 골짜기 위에서 AGL 150 m 를 크게 넘어
   실제로는 승인 없이 불가하다 (NOTE.md / API_DEFINE.md 참고).

실행 (PX4 강원 월드가 떠 있고 기체가 시동·이륙 가능 상태):
  ~/uav-venv4/bin/python uav/tools/demo_fly_over_ridge.py [--profile ridge_profile.json]

--profile: [[lat, lon, 지면고도], ...] 원통→기린 직선 지형 단면. 주면 로그에 지형 여유고도를 찍는다.
MAVSDK 는 GCS 링크(14550)로 붙어 UAV Agent(14540)와 경합하지 않는다.
"""
import argparse
import asyncio
import json
import math
import subprocess

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

WONTONG = (38.1205, 128.2018)
GIRIN = (37.9645, 128.3062, 279.0)      # 팀 DEM 지면고도
CRUISE_AMSL = 1050.0                     # 경로 최고 지형 986 m + 여유
FINAL_AGL = 60.0
PX4_PARAM = "/opt/px4-gazebo/bin/px4-param"


def hav(a, b):
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dp, dl = p2 - p1, math.radians(b[1] - a[1])
    return 2 * 6371000 * math.asin(math.sqrt(math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2))


def param(name, value=None):
    args = ["docker", "exec", "px4", PX4_PARAM] + (["set", name, str(value)] if value is not None else ["show", name])
    out = subprocess.run(args, capture_output=True, text=True).stdout
    for ln in out.splitlines():
        if f" {name} " in ln:
            return float(ln.rsplit(":", 1)[1])
    return None


class Terrain:
    def __init__(self, path):
        self.prof = json.load(open(path)) if path else None

    def below(self, lat, lon):
        """원통→기린 직선에서 가장 가까운 단면 점의 지면고도."""
        if not self.prof:
            return None
        return min(self.prof, key=lambda p: (p[0] - lat) ** 2 + ((p[1] - lon) * math.cos(math.radians(lat))) ** 2)[2]


async def latest(stream):
    gen = stream()
    try:
        return await asyncio.wait_for(anext(gen), 5)
    finally:
        await gen.aclose()


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="udpin://0.0.0.0:14550")
    ap.add_argument("--profile")
    ap.add_argument("--speed", type=float, default=12.0, help="비행 중 MPC_XY_CRUISE (m/s)")
    ap.add_argument("--to", help="목적지 'lat,lon,지면고도' (기본: 기린119)")
    ap.add_argument("--cruise-amsl", type=float, default=CRUISE_AMSL)
    ap.add_argument("--final-agl", type=float, default=FINAL_AGL)
    a = ap.parse_args()
    dest = tuple(map(float, a.to.split(","))) if a.to else GIRIN
    terrain = Terrain(a.profile)

    sdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    await sdk.add_any_connection(a.address)
    system = await sdk.first_autopilot(timeout_s=20)
    tel, act = TelemetryAsync(system), ActionAsync(system)

    old_speed = param("MPC_XY_CRUISE") or 5.0
    param("MPC_XY_CRUISE", a.speed)
    print(f"MPC_XY_CRUISE {old_speed} → {a.speed} m/s", flush=True)
    t0 = asyncio.get_running_loop().time()

    async def log(tag):
        pos = await latest(tel.subscribe_position)
        att = await latest(tel.subscribe_attitude_euler)
        vel = await latest(tel.subscribe_velocity_ned)
        ground = terrain.below(pos.latitude_deg, pos.longitude_deg)
        clr = f"{pos.absolute_altitude_m - ground:7.0f}" if ground is not None else "      -"
        gnd = f"{ground:6.0f}" if ground is not None else "     -"
        t = asyncio.get_running_loop().time() - t0
        print(f"[{t:6.0f}s] {tag:<8} 원통→{hav(WONTONG, (pos.latitude_deg, pos.longitude_deg)) / 1000:5.2f} km "
              f"목적지까지 {hav((pos.latitude_deg, pos.longitude_deg), dest) / 1000:5.2f} km | "
              f"고도 {pos.absolute_altitude_m:6.0f} m 지면 {gnd} 여유 {clr} m | "
              f"속도 {math.hypot(vel.north_m_s, vel.east_m_s):4.1f} m/s 상승 {-vel.down_m_s:4.1f} | "
              f"roll {att.roll_deg:5.1f} pitch {att.pitch_deg:5.1f}", flush=True)
        if abs(att.roll_deg) > 60 or abs(att.pitch_deg) > 60:
            raise RuntimeError("자세 이상 — 충돌 또는 추락 추정")
        return pos

    async def leg(tag, lat, lon, alt, done):
        await act.goto_location(lat, lon, alt, 0.0)
        while True:
            pos = await log(tag)
            if done(pos):
                return
            await asyncio.sleep(5)

    try:
        if not await latest(tel.subscribe_in_air):
            print("지상 상태 → 시동·이륙", flush=True)
            await act.arm()
            await act.takeoff()
            await asyncio.sleep(10)
        p = await latest(tel.subscribe_position)
        await leg("상승", p.latitude_deg, p.longitude_deg, a.cruise_amsl,
                  lambda q: q.absolute_altitude_m > a.cruise_amsl - 5)
        await leg("순항", dest[0], dest[1], a.cruise_amsl,
                  lambda q: hav((q.latitude_deg, q.longitude_deg), dest) < 30)
        final = dest[2] + a.final_agl
        await leg("하강" if final < a.cruise_amsl else "도착", dest[0], dest[1], final,
                  lambda q: abs(q.absolute_altitude_m - final) < 5)
        print("목적지 상공 도착 — 호버링 유지", flush=True)
    finally:
        param("MPC_XY_CRUISE", old_speed)
        print(f"MPC_XY_CRUISE 복원 → {old_speed} m/s", flush=True)
        sdk.destroy()


if __name__ == "__main__":
    asyncio.run(main())
