"""정지 차량 막힘 보고 = 막힘 에피소드(작업, 채택된 주행 경로 세대, 상대)당 한 번 (2026-10-13 sim 고정 장면 무한 재양보, Codex 1705·1712).
traffic._block_report_due 를 직접 부른다 (막힘 판단·보고 키만 — 회피 계획은 기존 시험이 본다)."""
from types import SimpleNamespace

import pytest

import ugv.traffic as TR
from ugv.tests.test_multi_vehicle import FLEET_CFG, make_fleet


def _tm(tmp_path):
    f, _ = make_fleet(tmp_path, [FLEET_CFG[0], FLEET_CFG[2]])
    tm = f.traffic
    tm.vehicles["B-ugv1"]._fail("VEHICLE_FAULT", "시험 — 움직이지 않는 차")      # immovable → 바로 보고 대상
    return tm


def _v(task="T1", gen=1):
    return SimpleNamespace(task_key=task, drive_gen=gen, block_report_void=None)


H = {"other": "B-ugv1", "kind": "STATIC", "s": 50.0}


def test_persistent_block_reported_once(tmp_path):
    tm, v = _tm(tmp_path), _v()
    assert tm._block_report_due("A-ugv1", v, H, 10.0) is not None
    for t in (10.5, 11.0, 20.0, 200.0):
        assert tm._block_report_due("A-ugv1", v, H, t) is None


def test_yield_stop_seq_change_does_not_rereport(tmp_path):
    """양보 정지 미션 전송은 drive_gen 을 올리지 않는다 → 같은 에피소드."""
    tm, v = _tm(tmp_path), _v()
    assert tm._block_report_due("A-ugv1", v, H, 10.0) is not None
    v.seq = 99                                            # 정지 미션으로 순번만 바뀜
    assert tm._block_report_due("A-ugv1", v, H, 10.5) is None


def test_reblock_after_successful_replan_is_reported_again(tmp_path):
    """막힘 처리로 새 주행 경로가 채택되면(세대 +1) 같은 상대에 다시 막혀도 새 에피소드 — 대기 기록이 지워지기 전에(같은 틱 직후)도."""
    tm, v = _tm(tmp_path), _v(gen=1)
    assert tm._block_report_due("A-ugv1", v, H, 10.0) is not None
    v.drive_gen = 2                                       # REPLANNED_AROUND_VEHICLE → _drive
    r = tm._block_report_due("A-ugv1", v, H, 10.2)
    assert r is not None and tm.waits["A-ugv1"]["reported_key"] == ("T1", 2, "B-ugv1")
    assert tm._block_report_due("A-ugv1", v, H, 11.0) is None   # 그 에피소드도 한 번


def test_void_report_can_be_retried_in_same_episode(tmp_path):
    tm, v = _tm(tmp_path), _v()
    assert tm._block_report_due("A-ugv1", v, H, 10.0) is not None
    v.block_report_void = ("T1", 1, "B-ugv1")             # 차량 쪽이 전제 불충족으로 처리하지 않음
    assert tm._block_report_due("A-ugv1", v, H, 10.5) is not None
    assert v.block_report_void is None


def test_stale_void_does_not_affect_new_episode(tmp_path):
    tm, v = _tm(tmp_path), _v(gen=1)
    assert tm._block_report_due("A-ugv1", v, H, 10.0) is not None
    v.drive_gen = 2
    assert tm._block_report_due("A-ugv1", v, H, 10.2) is not None
    v.block_report_void = ("T1", 1, "B-ugv1")             # 이전 세대의 늦은 '처리 안 됨'
    assert tm._block_report_due("A-ugv1", v, H, 10.4) is None   # 새 에피소드의 보고 완료를 되돌리지 않는다


def test_new_task_starts_new_episode(tmp_path):
    tm, v = _tm(tmp_path), _v()
    assert tm._block_report_due("A-ugv1", v, H, 10.0) is not None
    v.task_key, v.drive_gen = "T2", 1
    assert tm._block_report_due("A-ugv1", v, H, 10.5) is not None


def test_movable_blocker_waits_report_time(tmp_path):
    """막은 차가 임무를 가진 주행 차(이동 가능)면 TRAFFIC_BLOCK_REPORT_S 를 기다린 뒤에만 보고."""
    import asyncio as _a
    from ugv.tests.test_multi_vehicle import body as _body
    from ugv.geo import FRAME as _F
    f, _ = make_fleet(tmp_path, [FLEET_CFG[0], FLEET_CFG[2]])
    tm, B = f.traffic, f.vehicles["B-ugv1"]
    _a.run(B.execute(_body("B1", *_F.to_ll(B.access.x + 600, B.access.y + 900))))
    assert not TR._immovable(B), (B.state, B.current_task())
    v = _v()
    assert tm._block_report_due("A-ugv1", v, H, 0.0) is None
    assert tm._block_report_due("A-ugv1", v, H, TR.TRAFFIC_BLOCK_REPORT_S - 1) is None
    assert tm._block_report_due("A-ugv1", v, H, TR.TRAFFIC_BLOCK_REPORT_S + 1) is not None


# ---- 차량·비동기 경계 (Codex 1745) -------------------------------------------------------------
import asyncio
import math

from ugv.geo import FRAME, bearing_deg
from ugv.tests.test_multi_vehicle import NET, body, done, run


def test_stale_episode_report_is_discarded_by_vehicle(tmp_path):
    """잠금을 기다리는 사이 새 주행 경로가 채택됐으면(세대 변화) 오래된 보고는 폐기 — 막힘 이력·임무 상태를 건드리지 않는다."""
    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[0], FLEET_CFG[2]])
        A = f.vehicles["A-ugv1"]
        await A.execute(body("A1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await run(f, clock, 10)
        A.hold = {"s": A._s_now() + 20, "other": "B-ugv1", "kind": "STATIC", "since_sim": clock.now()}
        old = (A.task_key, A.drive_gen - 1, "B-ugv1")
        await A.blocked_by_vehicle("B-ugv1", 15.0, True, episode=old)
        t = f.store.tasks["A1"]
        assert t["status"] == "IN_PROGRESS" and not t.get("blocked")
        assert any(e["event"] == "TRAFFIC_BLOCK_REPORT_STALE" for e in A.events)
        assert A.block_report_void is None
    asyncio.run(go())


def test_void_uses_original_episode_key(tmp_path):
    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[0], FLEET_CFG[2]])
        A = f.vehicles["A-ugv1"]
        await A.execute(body("A1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await run(f, clock, 10)
        A.hold = None                                     # 전제 불충족 (대기 중 아님)
        ep = (A.task_key, A.drive_gen, "B-ugv1")
        await A.blocked_by_vehicle("B-ugv1", 15.0, True, episode=ep)
        assert A.block_report_void == ep
    asyncio.run(go())


def test_reblock_after_detour_ends_with_repeated_block_and_physical_stop(tmp_path):
    """실제 교통 루프·명령 작업·차량 잠금을 거쳐: 막힘 → 우회 재계획 → 새 경로에서 같은 차에 다시 막힘 → 다시 보고 →
    REPEATED_BLOCK 안전 종료 → 정차 확인. (수정 전: 재보고가 억제돼 YIELDING 무한 대기 가능 — 2026-10-13 sim 고정 장면)"""
    async def go():
        A0 = {**FLEET_CFG[0], "resource_id": "X-ugv"}
        far = {"lat": FRAME.to_ll(30000, 30000)[0], "lon": FRAME.to_ll(30000, 30000)[1], "heading_deg": 0.0}
        f, clock = make_fleet(tmp_path, [A0, {**FLEET_CFG[2], "resource_id": "Y-ugv", "initial": far}])
        X, Y = f.vehicles["X-ugv"], f.vehicles["Y-ugv"]
        tgt = FRAME.to_ll(X.access.x - 900, X.access.y - 700)
        await X.execute(body("X1", *tgt))

        def park_on(route_roads, skip):
            mid = next(r for r in route_roads[skip:-1] if NET.roads[r].length > 60)
            rm = NET.roads[mid]
            pm, q = NET.at(mid, rm.length / 2), NET.at(mid, rm.length / 2 + 1)
            Y.driver.x, Y.driver.y = pm.x, pm.y
            Y.driver.heading = bearing_deg(q.x - pm.x, q.y - pm.y)
            return mid
        mid1 = park_on(X.plan.route.road_ids, 4)
        gen0 = X.drive_gen
        ok, _ = await run(f, clock, 600, until=lambda: any(e["event"] == "REPLANNED_AROUND_VEHICLE" for e in X.events) or done(f, "X1")())
        assert any(e["event"] == "REPLANNED_AROUND_VEHICLE" for e in X.events), [e["event"] for e in X.events][-8:]
        assert X.drive_gen > gen0 and mid1 not in X.plan.route.road_ids
        # 새 경로 앞쪽 도로에 같은 차가 다시 선다
        i_now = next((k for k, r in enumerate(X.plan.route.road_ids) if r == X.here().road_id), 0) if X.here() else 0
        park_on(X.plan.route.road_ids, i_now + 2)
        ok, w = await run(f, clock, 600, until=done(f, "X1"))
        t = f.store.tasks["X1"]
        assert t["status"] == "FAILED" and "REPEATED_BLOCK" in t["error"], t.get("error")
        ev = [e["event"] for e in X.events]
        assert ev.count("TRAFFIC_BLOCKED_BY_VEHICLE") == 2
        assert w["overlaps"] == 0, w
        ok, _ = await run(f, clock, 30)                    # 물리 정지 확인
        assert X.driver.telemetry().speed_mps < 0.2 and X.state in ("IDLE", "STOPPING") and X.hold is None
    asyncio.run(go())
