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
from ugv.drivers.px4 import ACCEPT_M, pass_allowed, stall_away
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


def test_stall_away_conditions():
    assert not stall_away(2.0, 1.0)                        # 도착 반경 안 — PX4 가 곧 넘긴다
    assert not stall_away(ACCEPT_M + 1.0, ACCEPT_M)        # 아직 최소 거리 + 1.5 m 안 (다가오는 중·옆을 지나는 중)
    assert stall_away(ACCEPT_M + 2.5, ACCEPT_M)            # 놓치고 멀어짐
    assert not stall_away(10.0, 9.5)                       # 멀리서 다가오는 중이었다면 최소 거리 기준으로는 멀어짐 아님


def test_timeline_stop_needs_continuous_low_speed():
    from ugv.tools.failure_timeline import stop_phases
    rows = [{"x": 0.0, "y": 0.0}] * 3
    # 반례: 1 s 경계 표본에서 다시 빨라짐 → 정지 확인 아님
    r = stop_phases([0.0, 0.5, 1.0], [0.1, 0.1, 1.0], rows, 0.0)
    assert not r["stop_done"] and r["stop_low_start_sim_t"] == 0.0
    T = [i * 0.1 for i in range(140)]
    v = [1.0] * 10 + [0.1] * 120 + [0.5] * 10            # 1.0 s 에 저속 시작, 13.0 s 에 재이동
    r = stop_phases(T, v, [{"x": 0.0, "y": 0.0}] * 140, 0.0)
    assert r["stop_done"] and abs(r["stop_confirm_sim_t"] - 2.0) < 1e-6 and abs(r["re_motion_sim_t"] - 13.0) < 1e-6
    assert r["observation_short"] is False or r["observed_after_confirm_s"] >= 10.0


def _drv(monkeypatch, stall=True, skip=False):
    import ugv.drivers.px4 as P
    from ugv.drivers.base import Progress
    monkeypatch.setattr(P, "STALL_DETECT", stall)
    monkeypatch.setattr(P, "PASS_SKIP", skip)
    d = P.Px4Driver("udp://x", None)
    d._armed = True
    sent = [(float(x), 0.0, 4.0, True) for x in range(0, 60, 2)]
    d._sent = (1, sent, (0.0, 0.0))
    d._prog = Progress(10, len(sent), False, 1)
    state = {"x": 18.0, "y": 0.0, "age": 0.0}

    class Tel:
        def __init__(s):
            s.x, s.y, s.heading_deg, s.speed_mps = state["x"], state["y"], 90.0, 4.0

        def age_s(s):
            return state["age"]
    d.telemetry = lambda: Tel()
    return d, state, P, Progress


def test_stall_timer_lifecycle(monkeypatch):
    """정체 타이머: 멀어지면 시작, 항목 변경·새 미션·텔레메트리 지연·시동 꺼짐이면 지운다 (검증 문서 단계 1)."""
    d, st, P, Progress = _drv(monkeypatch)
    run = lambda: asyncio.run(d._pass_step(0.0))
    run()                                             # 항목 10(x=20) 2 m 앞 — 최소 거리 2
    assert d.pass_blocked_since is None
    st["x"], st["y"] = 21.0, 6.0                      # 옆으로 지나쳐 멀어짐 (6.1 m > 3.5, > 2 + 1.5)
    run()
    assert d.pass_blocked_since is not None
    d._prog = Progress(11, d._prog.total, False, 1)  # PX4 가 다음 항목으로 넘김 → 지움
    run()
    assert d.pass_blocked_since is None
    d._prog = Progress(10, d._prog.total, False, 1)
    st["x"], st["y"] = 18.0, 0.0
    run()
    st["x"], st["y"] = 21.0, 6.0
    run()
    assert d.pass_blocked_since is not None
    st["age"] = 5.0                                    # 텔레메트리 지연 → 지움
    run()
    assert d.pass_blocked_since is None
    st["age"] = 0.0
    d._armed = False                                   # 시동 꺼짐 → 지움
    run()
    assert d.pass_blocked_since is None
    d._armed = True
    d._sent = (2, d._sent[1], d._sent[2])              # 새 미션 번호인데 진행 보고는 옛 미션 → 판단 보류·지움
    run()
    assert d.pass_blocked_since is None


def test_stall_detect_off_does_nothing(monkeypatch):
    d, st, *_ = _drv(monkeypatch, stall=False, skip=False)
    st["x"], st["y"] = 21.0, 6.0
    asyncio.run(d._pass_step(0.0))
    assert d.pass_blocked_since is None
