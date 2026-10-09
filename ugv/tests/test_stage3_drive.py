# -*- coding: utf-8 -*-
"""3단계: 도로 경로·구간 감속·접근 지점(화재 100 m)·실행 키·취소/재계획·감시 — 모의 (sim 드라이버, 시간 주입).

원자료 도로망(gpkg, ugv/road_source.py)과 실제 거점 좌표(environment/config/fire_stations.json A)를 쓴다.
화재는 시험용으로 만든 칸이다 (환경 모듈 연결 시험은 따로).
"""

import asyncio
import math

import pytest

from interfaces.exec_store import ExecStore
from ugv import config
from ugv.approach import plan_approach
from ugv.clock import ManualClock
from ugv.drive_graph import access_point
from ugv.fire import FireArea, StaticFireSource
from ugv.geo import FRAME
from ugv.roads import RoadNetwork, plan_speeds
from ugv.vehicle import HttpError, Vehicle

NET = RoadNetwork()
_ACC = access_point(NET, "A")
START_A = NET.graph.dirpos(_ACC[0], _ACC[1])        # 거점 A 출입 지점, 스폰 방향
TARGET = (38.0277, 128.1303)          # 남전약수터 근처 (2019 발화점)
VCFG = {"resource_id": "A-ugv1", "resource_type": "UGV", "station_base_id": "A", "initial": "STATION",
        "driver": "sim"}


def fire_block(lat, lon, n=1, state="BURNING"):
    """(2n+1)² 칸 화재. 도로 칸(도로 중심선이 칸 안을 지나는 칸)은 빼다 — 도로에는 불이 생기지 않는다 (사용자 확인 2026-10-09)."""
    x0, y0 = FRAME.to_xy(lat, lon)
    out = []
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = x0 + 90 * i, y0 + 90 * j
            if NET.snap(x, y, max_m=45.0) is not None:
                continue
            la, lo = FRAME.to_ll(x, y)
            out.append({"cell_id": f"T{i}_{j}", "lat": la, "lon": lo, "fire_state": state, "cell_size_m": 90})
    return out


def beside_route(pts, k, off=70.0):
    """경로 점 k 에서 진행 방향 오른쪽으로 off m 떨어진 곳 (도로 밖 칸 중심)."""
    (ax, ay), (bx, by) = pts[k], pts[k + 1]
    d = math.hypot(bx - ax, by - ay)
    return ax + off * (by - ay) / d, ay - off * (bx - ax) / d


def make(tmp_path, cells=()):
    clock = ManualClock()
    src = StaticFireSource(cells, "TEST")
    v = Vehicle(dict(VCFG), NET, src, clock, ExecStore(tmp_path / "tasks.json"))
    return v, clock, src


async def run_for(v, clock, sim_s, dt=0.5, until=None):
    for _ in range(int(sim_s / dt)):
        clock.advance(dt)
        v.driver.step(dt)
        await v.tick()
        if until and until():
            return True
    return False


def body(key, lat=TARGET[0], lon=TARGET[1], **kw):
    return {"task_id": key, "decision_id": "D1", "target": {"lat": lat, "lon": lon}, **kw}


# ---------------------------------------------------------------------- 경로·속도
def test_route_from_station_and_speed_profile():
    dst = NET.snap_ll(*TARGET)
    g = NET.graph
    tree = g.search(START_A)
    r = g.build_route(tree, dst, min(g.arrivals(tree, dst), key=lambda a: a.time_s))
    assert r is not None and 4000 < r.length < 8000
    sp = plan_speeds(r)
    assert max(sp.v) <= config.MAX_SPEED_MPS + 1e-6
    assert sp.v[0] == 0 and sp.v[-1] == 0
    # 가감속 한계
    for k in range(1, len(sp.v)):
        ds = sp.cum[k] - sp.cum[k - 1]
        assert sp.v[k] ** 2 <= sp.v[k - 1] ** 2 + 2 * config.ACCEL_MPS2 * ds + 1e-6
        assert sp.v[k - 1] ** 2 <= sp.v[k] ** 2 + 2 * config.DECEL_MPS2 * ds + 1e-6
    # 곡선 감속이 실제로 생긴다 (도로망에 곡선이 있다. 꺾인 점은 원호로 다듬어져 있다 — drive_graph._smooth)
    assert min(sp.v[10:-10]) < config.MAX_SPEED_MPS - 3.0
    # 미션 지점 속도는 1 m/s ~ 최고속도, 마지막 지점은 목적지
    assert all(1.0 <= s <= config.MAX_SPEED_MPS + 1e-6 for _, _, s in sp.items)
    assert math.dist(sp.items[-1][:2], (dst.x, dst.y)) < 0.5


def test_station_is_single_source_and_offset_reported(tmp_path):
    v, _, _ = make(tmp_path)
    st = config.load_station("A")
    assert v.station["lat"] == st["lat"] and v.station["name"] == st["name"]
    assert 0 < v.station["road_offset_m"] < 100          # 안전센터 좌표는 도로 밖 → 가장 가까운 도로 지점 거리 보고
    s = v.state_view()
    assert s["state"] == "READY" and s["base"] == "A" and s["driver"] == "sim"
    for k in ("fuel_pct", "current_node", "current_task_id", "driver"):     # ugv_connector real 모드가 읽는 키
        assert k in s


# ---------------------------------------------------------------------- 접근 지점
def test_approach_without_fire_goes_to_nearest_road():
    p = plan_approach(NET, FireArea([], source="T"), START_A, *TARGET)
    assert p.status == "OK" and p.mode == "NEAREST_ROAD" and p.dist_to_target_m < 30


def test_approach_keeps_100m_from_fire_boundary_on_point_and_route():
    fire = FireArea(fire_block(*TARGET, n=1), source="T")
    p = plan_approach(NET, fire, START_A, *TARGET)
    assert p.status == "OK" and p.mode == "FIRE_STANDOFF"
    assert p.fire_clearance_m >= config.FIRE_STANDOFF_M
    assert fire.clearance_along(p.route.pts[1:]) >= config.FIRE_STANDOFF_M
    rep = p.report()
    assert rep["point"]["lat"] and rep["dist_to_target_m"] > 100 and rep["standoff_m"] == 100


def test_approach_impossible_when_fire_covers_area():
    fire = FireArea(fire_block(*TARGET, n=25), source="T")       # 약 4.6 km 사각형: 목표 2 km 안 도로 전부가 100 m 안
    p = plan_approach(NET, fire, START_A, *TARGET)
    assert p.status == "NO_SAFE_APPROACH"


def test_target_far_from_any_road_is_unreachable():
    p = plan_approach(NET, FireArea([], source="T"), START_A, 37.50, 127.50)
    assert p.status == "TARGET_UNREACHABLE"


# ---------------------------------------------------------------------- 실행·도착
def test_execute_drive_and_arrive(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        ev = await v.evaluate(body("E1"))
        assert ev["verdict"] == "ACCEPT" and ev["target_node"]["node_id"].startswith("RP:")
        assert v.driver.path == []                        # 판단은 움직이지 않는다
        r = await v.execute(body("T1", target_node=ev["target_node"]["node_id"]))
        assert r["status"] == "STARTED" and r["target_node"]["node_id"] == ev["target_node"]["node_id"]
        assert v.state_view()["state"] == "RUNNING"
        done = await run_for(v, clock, 1200, until=lambda: v.store.tasks["T1"]["status"] == "COMPLETED")
        t = v.store.tasks["T1"]
        assert done, t
        assert t["mission_result"] == "ARRIVED" and t["physical_state"] == "STOPPED_ON_STATION"
        assert t["observation"]["observation_type"] == "ROAD_STATUS"
        s = v.state_view()
        assert s["state"] == "READY" and s["current_task_id"] is None and s["mission_state"] == "IDLE"
        # 실제 걸린 시뮬레이션 시간이 계획 ETA 근처 (sim 드라이버 가속이 계획보다 크다 → 조금 빠를 수 있다)
        took = t["timing"]["end_sim_s"] - t["timing"]["start_sim_s"]
        assert 0.7 * r["eta_sec"] <= took <= 1.3 * r["eta_sec"]
    asyncio.run(go())


def test_duplicate_and_conflicting_execution_key(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        r1 = await v.execute(body("T1"))
        seq = v.driver.progress().mission_seq
        r2 = await v.execute(body("T1"))
        assert r2["duplicate"] is True and r2["task_id"] == "T1"
        assert v.driver.progress().mission_seq == seq     # 새 주행 없음
        with pytest.raises(HttpError) as e:
            await v.execute(body("T1", lat=38.05, lon=128.15))
        assert e.value.code == 409
        assert r1["eta_sec"] > 0
    asyncio.run(go())


def test_new_target_while_driving_cancels_and_replans_single_mission(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        await v.execute(body("T1"))
        await run_for(v, clock, 60)
        seq1 = v.driver.progress().mission_seq
        r2 = await v.execute(body("T2", lat=38.0450, lon=128.1900))
        assert r2["superseded_task_id"] == "T1"
        t1 = v.store.tasks["T1"]
        assert t1["status"] == "CANCELLED" and t1["error"] == "SUPERSEDED_BY:T2"
        assert v.driver.progress().mission_seq == seq1 + 1          # 미션은 교체 한 번
        assert v.driver.path[-1][:2] == v.speed.items[-1][:2]       # 드라이버에 남은 미션 = 새 경로
        done = await run_for(v, clock, 1500, until=lambda: v.store.tasks["T2"]["status"] == "COMPLETED")
        assert done and t1["status"] == "CANCELLED"                 # 취소된 임무가 뒤늦게 완료되지 않는다
    asyncio.run(go())


def test_abort_stops_vehicle(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        await v.execute(body("T1"))
        await run_for(v, clock, 30)
        assert v.driver.v > 5
        t = await v.abort("T1")
        assert t["status"] == "CANCELLED"
        await run_for(v, clock, 30)
        assert v.driver.v == 0 and v.state == "IDLE" and v.state_view()["state"] == "READY"
        with pytest.raises(HttpError):
            await v.abort("NOPE")
    asyncio.run(go())


# ---------------------------------------------------------------------- 화재 안전
def test_fire_spreading_onto_route_triggers_replan(tmp_path):
    async def go():
        v, clock, src = make(tmp_path)
        await v.execute(body("T1"))
        await run_for(v, clock, 20)
        # 남은 경로 중간, 도로 옆 칸에 불이 난다 (도로 자체는 타지 않지만 도로가 경계 100 m 안에 든다).
        # 2026-10-10: 회전·차체 검사가 촘촘해지며 2/3 지점 불의 27 km 우회로는 차체 2 cm 초과로 주행 불가 판정 →
        # 우회로가 실제로 있는(오프라인 계획으로 확인한) 지점을 고른다. 우회로가 없는 경우는 아래 실패 시험이 본다
        from ugv.approach import plan_approach
        from ugv.fire import FireArea
        cells = None
        for frac in (2 / 3, 1 / 2, 3 / 4, 5 / 6, 0.4):
            k = int(len(v.speed.pts) * frac)
            c = fire_block(*FRAME.to_ll(*beside_route(v.speed.pts, k)), n=0)
            st, v0, _ = v.here_state()
            if c and plan_approach(NET, FireArea(c, source="T"), st, *TARGET, v0=v0, access=v.access).status == "OK":
                cells = c
                break
        assert cells, "우회로가 있는 화재 위치 없음"
        src.set(cells, "TEST")
        await run_for(v, clock, 3)
        t = v.store.tasks["T1"]
        assert t["replans"] and t["replans"][-1]["result"] == "OK"
        fire = src.area()
        assert fire.clearance_along(v.speed.pts[5:]) >= config.FIRE_STANDOFF_M
        # 불이 한 줄뿐인 도로를 막으면 크게 돌아갈 수 있다. 가는 내내 안전거리를 지키는지 본다
        worst = [math.inf]

        def check():
            tel = v.driver.telemetry()
            worst[0] = min(worst[0], fire.distance(tel.x, tel.y))
            return t["status"] in ("COMPLETED", "FAILED")
        done = await run_for(v, clock, 3600, until=check)
        assert done and t["status"] == "COMPLETED"
        assert worst[0] >= config.FIRE_STANDOFF_M
    asyncio.run(go())


def test_fire_reaching_vehicle_on_station_evades(tmp_path):
    async def go():
        v, clock, src = make(tmp_path)
        tel = v.driver.telemetry()
        # 차량 옆 도로 밖 칸에 불 (경계가 차량에서 100 m 안)
        offs = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for r in (60, 75, 90) for a in range(0, 360, 30)]
        cells = next(c for c in (fire_block(*FRAME.to_ll(tel.x + dx, tel.y + dy), n=0) for dx, dy in offs) if c)
        src.set(cells, "TEST")
        await run_for(v, clock, 2)
        assert v.state == "EVADING"
        await run_for(v, clock, 300, until=lambda: v.state == "IDLE")
        tel = v.driver.telemetry()
        assert src.area().distance(tel.x, tel.y) >= config.FIRE_STANDOFF_M
        assert any(e["event"] == "EVADED" for e in v.events)
    asyncio.run(go())


def test_no_safe_evasion_is_danger_and_requires_release(tmp_path):
    async def go():
        v, clock, src = make(tmp_path)
        await v.execute(body("T1"))
        await run_for(v, clock, 20)
        tel = v.driver.telemetry()
        # 도로만 남기고 사방이 불: 30 m 칸으로 반경 3.5 km 를 채운다 (도로 중심선 15 m 안 칸은 뺀다).
        # 모든 도로 지점이 화재 경계 100 m 안 → 안전한 이탈 지점이 없다
        cells = []
        for i in range(-117, 118):
            for j in range(-117, 118):
                x, y = tel.x + 30 * i, tel.y + 30 * j
                if NET.snap(x, y, max_m=15.0) is None:
                    la, lo = FRAME.to_ll(x, y)
                    cells.append({"cell_id": f"D{i}_{j}", "lat": la, "lon": lo, "fire_state": "BURNING", "cell_size_m": 30})
        src.set(cells, "TEST")
        await run_for(v, clock, 2)
        assert v.state == "DANGER" and v.state_view()["state"] == "UNAVAILABLE"
        assert v.store.tasks["T1"]["status"] == "FAILED"
        ev = await v.evaluate(body("E2"))
        assert ev["verdict"] == "REJECT" and ev["reason"] == "FAILSAFE_ACTIVE"
        assert v.state_view()["operator_intervention"]["required"] is True
        assert any(e["event"] == "DANGER" and e["operator_action_required"] for e in v.events)
        src.set([], "TEST")
        if v.driver.v >= 0.2:                                   # 아직 제동 중이면 해제를 미룬다
            r = await v.stop()
            assert r["released_fault"] is None and r["release_pending"] and v.state == "DANGER"
        await run_for(v, clock, 30)
        assert v.state == "DANGER"                              # 자동 재출발·자동 해제 없음
        r = await v.stop()
        assert r["released_fault"].startswith("NO_SAFE_EVASION_ROUTE")
        await run_for(v, clock, 5)
        assert v.state_view()["state"] == "READY"
    asyncio.run(go())


# ---------------------------------------------------------------------- 주행 감시
def test_stall_and_off_route_faults(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        await v.execute(body("T1"))
        await run_for(v, clock, 10)
        v.driver.frozen = True
        await run_for(v, clock, config.STALL_TIMEOUT_S + 2)
        assert v.state == "FAULT" and v.fault.startswith("STALLED")
        assert v.store.tasks["T1"]["status"] == "FAILED"
        v.driver.frozen = False
        await run_for(v, clock, 30)
        await v.stop()
        await run_for(v, clock, 5)
        assert v.state_view()["state"] == "READY" and v.fault is None
        await v.execute(body("T2"))
        await run_for(v, clock, 10)
        v.driver.x += 80.0                                  # 경로에서 80 m 밀려남
        v.driver.path, v.driver.k = [(v.driver.x + 500, v.driver.y, 10.0)], 0
        await run_for(v, clock, config.OFF_ROUTE_CONFIRM_S + 2)
        assert v.state == "FAULT" and v.fault.startswith("OFF_ROUTE")
    asyncio.run(go())


def test_driver_start_failure_is_not_registered(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        v.driver.fail_next_follow = "MISSION_REJECTED"
        with pytest.raises(HttpError) as e:
            await v.execute(body("T1"))
        assert e.value.code == 500 and "T1" not in v.store.tasks
        r = await v.execute(body("T1"))                    # 같은 키로 다시 보낼 수 있다
        assert r["status"] == "STARTED"
    asyncio.run(go())
