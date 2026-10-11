# -*- coding: utf-8 -*-
"""고장 정지 경로와 지나침 건너뛰기 안전 조건 (2026-10-12 교차로 v1/v2 독립 검증 문서 5·6항).

1) 고장(FAULT) 정지도 계획선 가까이면 남은 계획 경로를 따라 정지한다 — _fail 이 상태를 먼저 FAULT 로 바꿔
   계획 경로 정지가 늘 빠지고 직선 정지로 떨어지던 결함.
2) 도로를 따르는 정지 경로가 없을 때(도로 밖·도로 방향과 진행 방향 불일치): emergency 모드는 비상 감속 직선
   (일반 직선 10 m 대신 짧게), straight 모드는 기존 드라이버 기본 정지. 도로로 급히 되돌아가는 경로는 만들지 않는다.
3) 지나친 항목은 목표가 차 앞이고 차체가 도로 띠 안일 때만 건너뛴다.
"""
import asyncio
import math

import ugv.vehicle as VM
from ugv.drivers.px4 import pass_allowed
from ugv.geo import FRAME
from ugv.tests.test_multi_vehicle import FLEET_CFG, body, make_fleet


def _go(A, clock, secs):
    async def go():
        for _ in range(int(secs / 0.5)):
            clock.advance(0.5)
            A.driver.step(0.5)
            await A.tick()
    return go()


def test_fault_stop_on_plan_follows_remaining_route(tmp_path):
    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[0]])
        A = f.vehicles["A-ugv1"]
        await A.execute(body("F1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await _go(A, clock, 20)
        assert A.driver.telemetry().speed_mps > 3
        A._fail("TEST_FAULT", "계획선 위 고장")
        info = await A._safe_stop()
        assert info["basis"] == "PLANNED_ROUTE", info
    asyncio.run(go())


def _offroad(tmp_path, mode, monkeypatch):
    monkeypatch.setattr(VM, "OFFROAD_STOP_MODE", mode)

    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[0]])
        A = f.vehicles["A-ugv1"]
        await A.execute(body("F2", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await _go(A, clock, 20)
        d = A.driver
        p = A.net.snap(d.x, d.y)
        r = A.net.roads[p.road_id]
        a, b = A.net.at(p.road_id, max(0.0, p.s - 2)), A.net.at(p.road_id, min(r.length, p.s + 2))
        hd = math.degrees(math.atan2(b.x - a.x, b.y - a.y))
        # 도로에서 옆으로 40 m (계획선·도로 정지 경로 조건 밖), 진행 방향은 도로와 직각
        nx, ny = math.cos(math.radians(hd)), -math.sin(math.radians(hd))
        d.x, d.y, d.heading, d.v = d.x + 40 * nx, d.y + 40 * ny, (hd + 90) % 360, 3.94
        A._fail("OFF_ROAD", "시험")
        return await A._safe_stop(), A
    return asyncio.run(go())


def test_offroad_stop_emergency_is_short_straight(tmp_path, monkeypatch):
    info, A = _offroad(tmp_path, "emergency", monkeypatch)
    assert info["basis"].startswith("EMERGENCY_STRAIGHT"), info
    assert info["length_m"] < A.profile.stop_distance(3.94)                       # 일반 직선(10.2 m)보다 짧다
    assert info["length_m"] >= VM.EMERG_STOP_MIN_M                                # PX4 도착 반경보다 멀게
    assert "도로 방향을 몰라 직선 정지" in info["problems"]


def test_offroad_stop_straight_mode_unchanged(tmp_path, monkeypatch):
    info, _ = _offroad(tmp_path, "straight", monkeypatch)
    assert info["basis"].startswith("DRIVER_DEFAULT"), info


def test_pass_allowed_conditions():
    assert pass_allowed((10.0, 0.0), (0.0, 0.0), 90.0, True)[0]                   # 동쪽 목표, 동쪽 진행
    assert pass_allowed((10.0, 5.0), (0.0, 0.0), 90.0, True)[0]                   # 앞 27°
    assert not pass_allowed((-10.0, 0.0), (0.0, 0.0), 90.0, True)[0]              # 목표가 뒤 (반대 방향)
    assert not pass_allowed((0.0, 10.0), (0.0, 0.0), 90.0, True)[0]               # 옆 90°
    assert not pass_allowed((10.0, 0.0), (0.0, 0.0), 90.0, False)[0]              # 차체가 도로 띠 밖
