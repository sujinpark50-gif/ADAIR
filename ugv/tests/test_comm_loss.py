# -*- coding: utf-8 -*-
"""총괄 통신 단절 처리 (사용자 결정 2026-10-10) — 모의, 가짜 벽시계로 긴 시간 경계를 본다 (운영 기본값은 줄이지 않는다).

단절 10분(COMM_LOSS_STOP_S) 초과 → 진행 중 임무 취소 + 도로를 따라 감속 정지. 정지 뒤 30분(COMM_LOSS_RETURN_S) 더 → 소속 거점
출입 지점으로 복귀 시도. 복귀 경로가 없으면 FAULT + 운영자 개입 필요 + 사유. 판단은 벽시계(시뮬레이션 시계 아님).
"""

import asyncio

from interfaces.exec_store import ExecStore
from ugv import config
from ugv import vehicle as vehicle_mod
from ugv.clock import ManualClock
from ugv.fire import StaticFireSource
from ugv.roads import RoadNetwork
from ugv.tests.test_stage3_drive import NET, TARGET, VCFG, body
from ugv.vehicle import Vehicle


class Wall:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make(tmp_path):
    clock, wall = ManualClock(), Wall()
    v = Vehicle(dict(VCFG), NET, StaticFireSource([], "T"), clock, ExecStore(tmp_path / "t.json"), wall=wall)
    return v, clock, wall


async def run(v, clock, wall, sim_s, dt=0.5, wall_rate=1.0, until=None):
    for _ in range(int(sim_s / dt)):
        clock.advance(dt)
        wall.t += dt * wall_rate
        v.driver.step(dt)
        await v.tick()
        if until and until():
            return True
    return False


def test_defaults_are_operational_values():
    assert config.COMM_LOSS_STOP_S == 600.0 and config.COMM_LOSS_RETURN_S == 1800.0


def test_no_contact_yet_means_no_comm_loss(tmp_path):
    async def go():
        v, clock, wall = make(tmp_path)
        wall.t += 10 * 3600
        await run(v, clock, wall, 2)
        assert v.comm["phase"] == "OK" and v.comm_lost_s() is None
    asyncio.run(go())


def test_loss_stops_then_returns_to_own_station(tmp_path):
    async def go():
        v, clock, wall = make(tmp_path)
        v.note_contact()
        await v.execute(body("T1"))
        await run(v, clock, wall, 60)                        # 주행 중
        assert v.state == "DRIVING"
        wall.t += config.COMM_LOSS_STOP_S - 61               # 벽시계만 흐른다 (시뮬레이션 시계와 무관)
        await run(v, clock, wall, 0.5)
        assert v.comm["phase"] == "OK"                        # 아직 10분 안
        wall.t += 2.0
        await run(v, clock, wall, 0.5)
        assert v.comm["phase"] == "STOPPING" and v.state == "STOPPING"
        t = v.store.tasks["T1"]
        assert t["status"] == "CANCELLED" and t["error"].startswith("COMM_LOSS") and t["mission_result"] != "ARRIVED"
        assert await run(v, clock, wall, 60, until=lambda: v.comm["phase"] == "STOPPED")
        assert v.driver.telemetry().speed_mps < 0.2
        wall.t += config.COMM_LOSS_RETURN_S - 5
        await run(v, clock, wall, 1)
        assert v.comm["phase"] == "STOPPED"                   # 정지 뒤 30분 전에는 움직이지 않는다
        wall.t += 10
        await run(v, clock, wall, 1)
        assert v.comm["phase"] == "RETURNING" and v.state == "DRIVING" and v.plan.dest == v.access
        assert await run(v, clock, wall, 2400, until=lambda: v.comm["phase"] == "RETURNED")
        tel = v.driver.telemetry()
        import math
        assert math.hypot(tel.x - v.access.x, tel.y - v.access.y) <= config.ARRIVE_M * 3
        assert v.store.tasks["T1"]["status"] == "CANCELLED"   # 취소된 임무가 뒤늦게 완료로 바뀌지 않는다
        v.note_contact()
        assert v.comm["phase"] == "OK"
    asyncio.run(go())


def test_loss_without_return_route_requires_operator(tmp_path, monkeypatch):
    async def go():
        v, clock, wall = make(tmp_path)
        v.note_contact()
        await v.execute(body("T1"))
        await run(v, clock, wall, 40)
        wall.t += config.COMM_LOSS_STOP_S + 1
        assert await run(v, clock, wall, 80, until=lambda: v.comm["phase"] == "STOPPED")
        from ugv.approach import ApproachPlan
        monkeypatch.setattr(vehicle_mod, "plan_approach",
                            lambda *a, **k: ApproachPlan("TARGET_UNREACHABLE", "NO_DIRECTED_ROUTE (시험 주입)"))
        wall.t += config.COMM_LOSS_RETURN_S + 1
        await run(v, clock, wall, 1)
        assert v.comm["phase"] == "FAILED" and v.state == "FAULT"
        op = v.state_view()["operator_intervention"]
        assert op["required"] is True and "COMM_LOSS_NO_RETURN_ROUTE" in op["reason"]
        assert any(e["event"] == "COMM_LOSS_RETURN_FAILED" and e["operator_action_required"] for e in v.events)
    asyncio.run(go())


def test_contact_before_threshold_keeps_mission(tmp_path):
    async def go():
        v, clock, wall = make(tmp_path)
        v.note_contact()
        await v.execute(body("T1"))
        for _ in range(3):
            wall.t += 500
            v.note_contact()
            await run(v, clock, wall, 2)
        assert v.comm["phase"] == "OK" and v.store.tasks["T1"]["status"] in ("STARTED", "IN_PROGRESS")
    asyncio.run(go())
