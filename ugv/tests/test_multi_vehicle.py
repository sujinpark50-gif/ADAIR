# -*- coding: utf-8 -*-
"""다중 차량 (UGV 2 + 소방차 2, 거점 A 인제119 · 거점 B 기린119) — 모의 (sim 드라이버, 시간 주입).

차량별 분리(특성·거점·출입 지점·드라이버·임무·통신 단절), 각자 소속 거점 복귀, 차량 간 충돌 방지(ugv/traffic.py):
앞뒤 간격, 교차로 우선순위, 같은 도로 반대 방향 진입 조정, 정지 차량 점유, 교착 감지.
차체 겹침 판정 = 두 차체 직사각형(차량별 치수, 진행 방향) 분리축 검사. 표본(0.5 s) 판정이라 근사다.
"""

import asyncio
import json
import math
from pathlib import Path

from interfaces.exec_store import ExecStore
from ugv import config
from ugv.clock import ManualClock
from ugv.drive_graph import DirPos
from ugv.fire import StaticFireSource
from ugv.geo import FRAME, bearing_deg
from ugv.roads import RoadNetwork
from ugv.server import Fleet
from ugv.tests.test_stage3_drive import NET

FLEET_CFG = json.loads((Path(config.UGV_DIR) / "fleet_ab.json").read_text(encoding="utf-8"))


def sim_cfg(cfgs):
    return [{**c, "driver": "sim"} for c in cfgs]


def make_fleet(tmp_path, cfgs=None):
    clock = ManualClock()
    f = Fleet(sim_cfg(cfgs or FLEET_CFG), net=NET, fire_source=StaticFireSource([], "T"), clock=clock,
              store=ExecStore(tmp_path / "tasks.json"))
    return f, clock


def corners(v):
    t = v.driver.telemetry()
    p = v.profile
    yaw = math.radians(90.0 - t.heading_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    # sim 드라이버 위치 = 뒤축 중심 (계획 선형이 뒤축 경로) → 차체 중심은 진행 방향으로 뒤축-중심 거리만큼 앞
    cx, cy = t.x + p.rear_axle_from_center_m * c, t.y + p.rear_axle_from_center_m * s
    hl, hw = p.body_length_m / 2, p.hw
    return [(cx + dx * c - dy * s, cy + dx * s + dy * c) for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def overlap(r1, r2):
    for poly in (r1, r2):
        for i in range(4):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % 4]
            nx, ny = y2 - y1, x1 - x2
            p1 = [nx * x + ny * y for x, y in r1]
            p2 = [nx * x + ny * y for x, y in r2]
            if max(p1) < min(p2) or max(p2) < min(p1):
                return False
    return True


async def run(f, clock, sim_s, dt=0.5, until=None, watch=None):
    worst = {"min_gap_m": math.inf, "overlaps": 0}
    for k in range(int(sim_s / dt)):
        clock.advance(dt)
        for v in f.vehicles.values():
            v.driver.step(dt)
            await v.tick()
        await f.traffic.tick()
        vs = list(f.vehicles.values()) if watch is None else [f.vehicles[r] for r in watch]
        for i in range(len(vs)):
            for j in range(i + 1, len(vs)):
                a, b = vs[i], vs[j]
                ta, tb = a.driver.telemetry(), b.driver.telemetry()
                gap = math.hypot(ta.x - tb.x, ta.y - tb.y) - (a.profile.body_length_m + b.profile.body_length_m) / 2
                worst["min_gap_m"] = min(worst["min_gap_m"], gap)
                if overlap(corners(a), corners(b)):
                    worst["overlaps"] += 1
        if until and until():
            return True, worst
    return False, worst


def body(key, lat, lon):
    return {"task_id": key, "decision_id": "D-" + key, "target": {"lat": lat, "lon": lon}}


def node_body(key, rid, s_m):
    from ugv.approach import node_id_of
    return {"task_id": key, "decision_id": "D-" + key, "target_node": node_id_of(NET.at(rid, s_m))}


def reachable(v, dx, dy, min_m=1500.0):
    """v 가 지금 갈 수 있는 (계획 OK) 목표 — 출입 지점에서 (dx, dy) 방향으로 min_m 넘게 떨어진 도로 표본 중 가까운 것."""
    from ugv.approach import plan_approach
    from ugv.fire import FireArea
    st, v0, _ = v.here_state()
    cands = []
    for ang in [math.atan2(dy, dx) + k * math.pi / 4 for k in range(8)]:
        ux, uy = math.cos(ang), math.sin(ang)
        cands += sorted((p for p in NET.samples()[::9]
                         if (p.x - v.access.x) * ux + (p.y - v.access.y) * uy > min_m),
                        key=lambda p: math.hypot(p.x - v.access.x, p.y - v.access.y))[:10]
    for p in cands:
        ll = FRAME.to_ll(p.x, p.y)
        if v._approach({"target": {"lat": ll[0], "lon": ll[1]}}, st, v0).status == "OK":     # 차량과 같은 조건
            return ll
    raise AssertionError("갈 수 있는 목표 없음")


def done(f, *keys):
    return lambda: all(f.store.tasks.get(k, {}).get("status") in ("COMPLETED", "FAILED", "CANCELLED") for k in keys)


# ---------------------------------------------------------------------- 배치·분리
def test_four_vehicles_two_stations_separate_settings(tmp_path):
    f, _ = make_fleet(tmp_path)
    assert set(f.vehicles) == {"A-ugv1", "A-fire1", "B-ugv1", "B-fire1"} and not f.unplaced
    A, AF, B, BF = (f.vehicles[r] for r in ("A-ugv1", "A-fire1", "B-ugv1", "B-fire1"))
    stations = json.loads(config.STATIONS_JSON.read_text(encoding="utf-8"))["stations"]
    st = {s["base_id"]: s for s in stations}
    assert A.station["name"] == st["A"]["name"] and B.station["name"] == st["B"]["name"] == "기린119안전센터"
    assert (B.station["lat"], B.station["lon"]) == (st["B"]["lat"], st["B"]["lon"])     # 환경 공통 설정에서 읽음
    assert AF.resource_type == BF.resource_type == "FIRE_ENGINE" and AF.profile.name == "adair_firetruck"
    assert A.profile.r_min < AF.profile.r_min and AF.profile.max_speed_mps < A.profile.max_speed_mps
    assert A.graph is not AF.graph and len(AF.graph.blocked_turns) > len(A.graph.blocked_turns)
    # 같은 거점 두 차는 서로 다른 도로의 대기 위치 (한 줄 대기는 서로 막는다), 다른 거점 차는 각자 거점
    for x, y in ((A, AF), (B, BF)):
        assert x.access.road_id != y.access.road_id
        assert math.hypot(x.access.x - y.access.x, x.access.y - y.access.y) > (x.profile.body_length_m + y.profile.body_length_m) / 2 + 4.0
        assert not overlap(corners(x), corners(y))
        assert "대기 위치 1" in y.station["access_point"]["source"]
    assert math.hypot(A.access.x - B.access.x, A.access.y - B.access.y) > 5000
    # PX4 연결 정보가 차량마다 다르다
    ports = {c["px4"]["address"] for c in FLEET_CFG}
    models = {c["px4"]["gz_model"] for c in FLEET_CFG}
    assert len(ports) == len(models) == 4


def test_unplaceable_vehicle_is_reported_not_moved(tmp_path):
    cfg = [dict(FLEET_CFG[0]), {**FLEET_CFG[1], "resource_id": "A-fire9", "station_slot": 400}]
    f, _ = make_fleet(tmp_path, cfg)
    assert "A-fire9" in f.unplaced and "A-fire9" not in f.vehicles and "대기 위치" in f.unplaced["A-fire9"]


def test_separate_routes_concurrent_and_each_returns_home(tmp_path):
    async def go():
        f, clock = make_fleet(tmp_path)
        A, B = f.vehicles["A-ugv1"], f.vehicles["B-ugv1"]
        ta = FRAME.to_ll(A.access.x - 900, A.access.y - 700)
        # B 주변은 도로가 적고 U턴이 없어, 오프셋 목표는 출발 도로 먼 끝으로 잡혀 복귀가 43 km 가 된다 → Gazebo 시험과 같은 목표
        tb = (37.951004, 128.337462)
        ra = await A.execute(body("A1", *ta))
        rb = await B.execute(body("B1", *tb))
        assert ra["status"] == rb["status"] == "STARTED"
        ok, w = await run(f, clock, 1500, until=done(f, "A1", "B1"))
        assert ok and f.store.tasks["A1"]["status"] == f.store.tasks["B1"]["status"] == "COMPLETED"
        # 각자 소속 거점으로 (A 로 고정되지 않는다)
        from ugv.approach import node_id_of
        await A.execute({"task_id": "A2", "decision_id": "D", "target_node": node_id_of(A.access)})
        await B.execute({"task_id": "B2", "decision_id": "D", "target_node": node_id_of(B.access)})
        ok, w2 = await run(f, clock, 1800, until=done(f, "A2", "B2"))
        assert ok and f.store.tasks["A2"]["status"] == f.store.tasks["B2"]["status"] == "COMPLETED"
        for v in (A, B):
            t = v.driver.telemetry()
            assert math.hypot(t.x - v.access.x, t.y - v.access.y) <= config.ARRIVE_M * 3
        assert w["overlaps"] == w2["overlaps"] == 0
    asyncio.run(go())


def test_one_vehicle_cancel_change_and_comm_loss_do_not_touch_others(tmp_path):
    async def go():
        f, clock = make_fleet(tmp_path)
        A, B = f.vehicles["A-ugv1"], f.vehicles["B-ugv1"]
        await A.execute(body("A1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await B.execute(body("B1", *reachable(B, -1.0, 1.0, 3000.0)))                       # B 는 오래 달린다
        await run(f, clock, 30)
        b_plan = B.plan
        await A.execute(body("A2", *FRAME.to_ll(A.access.x + 800, A.access.y + 300)))     # A 만 목표 변경
        assert f.store.tasks["A1"]["status"] == "CANCELLED" and B.plan is b_plan
        assert f.store.tasks["B1"]["status"] in ("STARTED", "IN_PROGRESS")
        await A.abort("A2")                                                               # A 만 취소
        assert f.store.tasks["B1"]["status"] in ("STARTED", "IN_PROGRESS") and B.state == "DRIVING"
        # A 만 통신 단절 (벽시계를 가짜로): B 는 총괄 연락을 계속 받는다
        wall = [1000.0]
        for v in f.vehicles.values():
            v.wall = lambda: wall[0]
            v.note_contact()
        wall[0] += config.COMM_LOSS_STOP_S + 5
        for r, v in f.vehicles.items():
            if r != "A-ugv1":
                v.note_contact()
        await run(f, clock, 1)
        assert A.comm["phase"] in ("STOPPING", "STOPPED") and B.comm["phase"] == "OK"
        assert f.store.tasks["B1"]["status"] in ("STARTED", "IN_PROGRESS") and B.state == "DRIVING"
        # A 고장이 B 에 번지지 않는다
        A._fail("STALLED", "시험 주입")
        assert B.state == "DRIVING" and B.fault is None
    asyncio.run(go())


# ---------------------------------------------------------------------- 공유 도로
def test_same_station_follow_keeps_gap(tmp_path):
    """같은 거점 UGV 와 소방차가 같은 목표로: 앞차(출입 지점) 뒤를 소방차가 간격을 두고 따라간다."""
    async def go():
        f, clock = make_fleet(tmp_path)
        A, AF = f.vehicles["A-ugv1"], f.vehicles["A-fire1"]
        tgt = FRAME.to_ll(A.access.x - 900, A.access.y - 700)
        await A.execute(body("U1", *tgt))                       # 앞차(출입 지점) 먼저 출발
        ru = A.plan.route
        cum = [0.0]
        for p1, p2 in zip(ru.pts, ru.pts[1:]):
            cum.append(cum[-1] + math.dist(p1, p2))
        k = next(i for i, c in enumerate(cum) if c > 0.6 * cum[-1])
        mid = NET.snap(*ru.pts[k])                              # 소방차 목표 = UGV 경로 60 % 지점 (앞차가 지나간 뒤 도착)
        from ugv.approach import node_id_of
        await AF.execute({"task_id": "F1", "decision_id": "D", "target_node": node_id_of(mid)})
        ok, w = await run(f, clock, 1500, until=done(f, "F1", "U1"), watch=["A-ugv1", "A-fire1"])
        assert ok
        assert w["overlaps"] == 0 and w["min_gap_m"] > 1.0, w
        # (대기 위치가 서로 다른 도로라 출발 순서 대기는 없을 수 있다. 같은 길을 앞뒤로 갈 때 간격·겹침만 본다)
    asyncio.run(go())


def _approach_pose(rid, node, dist):
    r = NET.roads[rid]
    s = r.length - dist if node == r.b else dist
    p = NET.at(rid, s)
    q = NET.at(rid, s + 1 if node == r.b else s - 1)
    return {"lat": FRAME.to_ll(p.x, p.y)[0], "lon": FRAME.to_ll(p.x, p.y)[1], "heading_deg": bearing_deg(q.x - p.x, q.y - p.y)}


def _plans_ok(f, jobs):
    """jobs [(vehicle id, body)] 모두 차량 조건(진행 방향·복귀·회피 도로)으로 계획 OK 인가."""
    for rid, bd in jobs:
        v = f.vehicles[rid]
        st, v0, _ = v.here_state()
        if st is None or v._approach(bd, st, v0).status != "OK":
            return False
    return True


# 시험 배치 (ugv 도로망에서 두 차량 모두 진행 방향·복귀 조건으로 계획되는 곳을 찾아 고정, 2026-10-10)
JUNCTION = ("494928", "682501307", "682501539", "682501929")        # 노드, X 진입, Y 진입, 합류 도로
HEAD_ON = ("683400826", "683400838", "683400828", "683401696", "683400719")   # 공유 도로, X 진입, Y 진입, X 진출, Y 진출


def test_junction_merge_one_yields(tmp_path):
    """다른 거점 차량(UGV·소방차)이 같은 교차로로 동시에 들어와 같은 도로로 합류: 하나가 교차로 앞에서 기다린다.
    (이 도로망에는 두 차종 모두 지날 수 있는 긴 4갈래 교차로가 없어 3갈래 합류로 본다. 먼저 도착하는 UGV 목적지가 더 멀어
    늦게 온 차가 서 있는 차를 지나갈 필요가 없다 — 중심선 주행이라 비켜 갈 수 없다)"""
    async def go():
        n, r1, r2, r3 = JUNCTION
        cfg = [{**FLEET_CFG[0], "resource_id": "X-ugv", "initial": _approach_pose(r1, n, 110)},
               {**FLEET_CFG[3], "resource_id": "Y-fire", "initial": _approach_pose(r2, n, 110)}]
        f, clock = make_fleet(tmp_path, cfg)
        X, Y = f.vehicles["X-ugv"], f.vehicles["Y-fire"]
        q3 = NET.roads[r3]
        rx = await X.execute(node_body("X1", r3, 170.0 if q3.a == n else q3.length - 170.0))
        ry = await Y.execute(node_body("Y1", r3, 60.0 if q3.a == n else q3.length - 60.0))
        assert rx["status"] == ry["status"] == "STARTED"
        assert X.plan.route.road_ids[:2] == [r1, r3] and Y.plan.route.road_ids[:2] == [r2, r3]
        ok, w = await run(f, clock, 600, until=done(f, "X1", "Y1"))
        assert ok and w["overlaps"] == 0 and w["min_gap_m"] > 1.0, w
        assert f.store.tasks["X1"]["status"] == f.store.tasks["Y1"]["status"] == "COMPLETED"
        yields = [e for v in (X, Y) for e in v.events if e["event"] == "YIELD"]
        assert yields                     # 합류(같은 도로로 나감)는 먼저 오는 차가 먼저 — 늦은 차가 교차로 앞에서 기다린다
    asyncio.run(go())


def test_head_on_same_road_waits_outside(tmp_path):
    """같은 도로를 반대 방향으로 지나야 하는 UGV 와 소방차: 하나가 그 도로에 들어가기 전에 기다린다 (교행 없음, 소방차 우선).
    X(UGV): 진입 → 공유 도로 → 진출, Y(소방차): 반대쪽 진입 → 공유 도로 → 다른 진출."""
    async def go():
        rid, ra, rb, rc, rd = HEAD_ON
        r = NET.roads[rid]
        cfg = [{**FLEET_CFG[0], "resource_id": "X-ugv", "initial": _approach_pose(ra, r.a, 70)},
               {**FLEET_CFG[1], "resource_id": "Y-fire", "initial": _approach_pose(rb, r.b, 70)}]
        f, clock = make_fleet(tmp_path, cfg)
        X, Y = f.vehicles["X-ugv"], f.vehicles["Y-fire"]
        qc, qd = NET.roads[rc], NET.roads[rd]
        rx = await X.execute(node_body("X1", rc, 60.0 if qc.a == r.b else qc.length - 60.0))
        ry = await Y.execute(node_body("Y1", rd, 60.0 if qd.a == r.a else qd.length - 60.0))
        assert rx["status"] == ry["status"] == "STARTED"
        assert rid in X.plan.route.road_ids and rid in Y.plan.route.road_ids
        ok, w = await run(f, clock, 900, until=done(f, "X1", "Y1"))
        assert ok and w["overlaps"] == 0 and w["min_gap_m"] > 1.0, w
        assert f.store.tasks["X1"]["status"] == f.store.tasks["Y1"]["status"] == "COMPLETED"
        heads = [e for v in (X, Y) for e in v.events if e["event"] == "YIELD" and e.get("kind") == "HEAD_ON"]
        assert heads and all(e["resource_id"] == "X-ugv" for e in heads)
    asyncio.run(go())


def test_stopped_vehicle_is_occupied_and_reported(tmp_path):
    """① 멈춘 차가 있는 도로는 새 계획에서 피한다 (들어가지 않는다).
    ② 출발 도로 앞을 멈춘 차가 막고 있으면(피할 수 없음) 그 뒤에서 기다리고, 오래 막히면 보고한다. 정지 고장으로 오판하지 않는다."""
    async def go():
        A0 = {**FLEET_CFG[0], "resource_id": "X-ugv"}
        f0, _ = make_fleet(tmp_path / "probe", [A0])
        X0 = f0.vehicles["X-ugv"]
        tgt = FRAME.to_ll(X0.access.x - 900, X0.access.y - 700)
        await X0.execute(body("P1", *tgt))
        route0 = X0.plan.route
        # ① 경로 중간 도로(출발 도로 아님)에 멈춘 차
        mid = next(r for r in route0.road_ids[2:-1] if NET.roads[r].length > 60)
        rm = NET.roads[mid]
        pm = NET.at(mid, rm.length / 2)
        q = NET.at(mid, rm.length / 2 + 1)
        cfg = [A0, {**FLEET_CFG[2], "resource_id": "Y-ugv", "initial": {"lat": FRAME.to_ll(pm.x, pm.y)[0],
               "lon": FRAME.to_ll(pm.x, pm.y)[1], "heading_deg": bearing_deg(q.x - pm.x, q.y - pm.y)}}]
        f, clock = make_fleet(tmp_path / "a", cfg)
        X = f.vehicles["X-ugv"]
        f.vehicles["Y-ugv"]._fail("VEHICLE_FAULT", "시험 주입 — 도로 위 고장 정지")
        await X.execute(body("X1", *tgt))
        assert mid not in X.plan.route.road_ids
        assert X.plan.report().get("avoided_roads_occupied_by_vehicles", {}).get(mid) == "Y-ugv"
        # ② 출발 도로 앞 30 m 에 고장 차
        acc = X0.access
        r = NET.roads[acc.road_id]
        p2 = NET.at(acc.road_id, acc.s + 30.0)
        q2 = NET.at(acc.road_id, acc.s + 31.0)
        cfg2 = [A0, {**FLEET_CFG[2], "resource_id": "Y-ugv", "initial": {"lat": FRAME.to_ll(p2.x, p2.y)[0],
                "lon": FRAME.to_ll(p2.x, p2.y)[1], "heading_deg": bearing_deg(q2.x - p2.x, q2.y - p2.y)}}]
        f2, clock2 = make_fleet(tmp_path / "b", cfg2)
        X2, Y2 = f2.vehicles["X-ugv"], f2.vehicles["Y-ugv"]
        Y2._fail("VEHICLE_FAULT", "시험 주입 — 도로 위 고장 정지")
        await X2.execute(body("X2", *tgt))
        # 고장 차는 스스로 움직이지 않는다 → 충돌을 예측하자마자 막힘 처리: 출발 도로라 우회 경로가 없다 → 기다리지 않고 임무 실패로
        # 보고, 차는 그 앞 대기 지점에 선다 (2026-10-10 개선 전: 60 s 뒤 보고만 하고 임무는 끝없이 IN_PROGRESS 로 기다렸다)
        ok, w = await run(f2, clock2, 120)
        assert w["overlaps"] == 0 and w["min_gap_m"] > 1.0, w
        t = f2.store.tasks["X2"]
        assert t["status"] == "FAILED" and t["error"].startswith("BLOCKED_BY_VEHICLE: Y-ugv"), t["error"]
        assert t["physical_state"] == "STOPPED_BLOCKED" and t["blocked"][0]["result"] == "FAILED_NO_DETOUR"
        assert X2.state == "IDLE" and X2.fault is None and X2.hold is None
        ev = [e["event"] for e in X2.events]
        assert ev.count("TRAFFIC_BLOCKED_BY_VEHICLE") == 1 and ev.count("TRAFFIC_BLOCKED_NO_DETOUR") == 1
        assert "REPLANNED_AROUND_VEHICLE" not in ev
        p0 = X2.driver.telemetry()
        ok, w = await run(f2, clock2, 60)                       # 그 뒤 움직이지 않고, 막힘 처리를 되풀이하지 않는다
        p1 = X2.driver.telemetry()
        assert math.hypot(p1.x - p0.x, p1.y - p0.y) < 0.5 and p1.speed_mps < 0.2
        assert [e["event"] for e in X2.events].count("TRAFFIC_BLOCKED_BY_VEHICLE") == 1
        dist_y = math.hypot(p1.x - Y2.driver.telemetry().x, p1.y - Y2.driver.telemetry().y)
        assert dist_y > (X2.profile.body_length_m + Y2.profile.body_length_m) / 2 + 1.0
    asyncio.run(go())


def test_vehicle_stopping_on_route_after_planning_is_bypassed(tmp_path):
    """계획한 뒤에 경로 위(교차로 사이 도로)에 다른 차가 임무 없이 선다 → 막은 차가 스스로 움직이지 않을 차이므로 충돌을 예측하자마자
    그 도로를 뺀 우회 경로로 다시 계획해(그 도로에 들어가기 전) 목적지에 간다. 우회는 한 번만."""
    async def go():
        A0 = {**FLEET_CFG[0], "resource_id": "X-ugv"}
        probe, _ = make_fleet(tmp_path / "probe", [A0])
        X0 = probe.vehicles["X-ugv"]
        tgt = FRAME.to_ll(X0.access.x - 900, X0.access.y - 700)
        await X0.execute(body("P1", *tgt))
        route0 = X0.plan.route
        mid = next(r for r in route0.road_ids[4:-1] if NET.roads[r].length > 60)
        rm = NET.roads[mid]
        far = {"lat": FRAME.to_ll(X0.access.x + 3000, X0.access.y + 3000)[0],
               "lon": FRAME.to_ll(X0.access.x + 3000, X0.access.y + 3000)[1], "heading_deg": 0.0}
        cfg = [A0, {**FLEET_CFG[2], "resource_id": "Y-ugv", "initial": far}]
        f, clock = make_fleet(tmp_path / "a", cfg)
        X, Y = f.vehicles["X-ugv"], f.vehicles["Y-ugv"]
        await X.execute(body("X1", *tgt))
        assert mid in X.plan.route.road_ids
        # 계획 뒤 Y 가 그 도로 가운데에 선다 (임무 없음)
        pm, q = NET.at(mid, rm.length / 2), NET.at(mid, rm.length / 2 + 1)
        Y.driver.x, Y.driver.y = pm.x, pm.y
        Y.driver.heading = bearing_deg(q.x - pm.x, q.y - pm.y)
        ok, w = await run(f, clock, 900, until=done(f, "X1"))
        assert ok and w["overlaps"] == 0 and w["min_gap_m"] > 1.0, w
        t = f.store.tasks["X1"]
        assert t["status"] == "COMPLETED", t
        assert t["blocked"][0]["result"] == "REPLANNED" and t["blocked"][0]["blocked_road"] == mid
        assert mid not in X.plan.route.road_ids
        ev = [e["event"] for e in X.events]
        assert ev.count("REPLANNED_AROUND_VEHICLE") == 1
    asyncio.run(go())


def test_repeated_block_by_same_vehicle_fails_instead_of_replanning_again(tmp_path):
    """같은 임무에서 같은 차에 다시 막히면 또 다시 계획하지 않고 실패로 보고한다 (되풀이 재계획 방지)."""
    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[0], FLEET_CFG[2]])
        A = f.vehicles["A-ugv1"]
        await A.execute(body("A1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await run(f, clock, 10)
        A.hold = {"s": A._s_now() + 20, "other": "B-ugv1", "kind": "STATIC", "since_sim": clock.now()}
        f.store.tasks["A1"]["blocked"] = [{"other": "B-ugv1", "result": "REPLANNED"}]
        await A.blocked_by_vehicle("B-ugv1", 15.0, True)
        t = f.store.tasks["A1"]
        assert t["status"] == "FAILED" and t["error"].startswith("BLOCKED_BY_VEHICLE")
        assert t["blocked"][-1]["result"] == "FAILED_NO_DETOUR" and "REPEATED_BLOCK" in t["blocked"][-1]["why"]
    asyncio.run(go())


def test_mutual_wait_is_reported_as_deadlock(tmp_path):
    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[0], FLEET_CFG[2]])
        A, B = f.vehicles["A-ugv1"], f.vehicles["B-ugv1"]
        await A.execute(body("A1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await B.execute(body("B1", *FRAME.to_ll(B.access.x + 600, B.access.y + 900)))
        await run(f, clock, 10)
        tm = f.traffic
        # 서로를 기다리는 판단을 주입 (실제 기하에서 만들기 어려운 고리) → 교착 보고, 둘 다 기다림 유지
        tm._first_conflict = lambda P, Q: ("CROSS", P.samples[-1], Q.samples[-1])
        tm._decide = lambda kind, P, Q, a, b, prefer=None: (P, Q, max(P.s_now, a[3] - 5), "TEST")
        await tm.tick()
        assert tm.deadlocks == [["A-ugv1", "B-ugv1"]]
        assert any(e["event"] == "TRAFFIC_DEADLOCK" and e["operator_action_required"] for e in A.events)
        assert A.hold and B.hold
    asyncio.run(go())


def test_gazebo_b_station_return_sequence_no_contact(tmp_path):
    """2026-10-10 Gazebo 4대 시험 재현: B-fire1 이 B-ugv1 복귀 길 위 목적지에 서 있고, B-ugv1 복귀 뒤 B-fire1 도 복귀.
    수정 전: 마주 선 두 차가 '피할 수 없음' 으로 각자 경로를 따라 정지하며 접촉 (차체 겹침 1,597 표본).
    수정 후: 서 있는 차는 그 자리에서 기다리고(교착이면 보고), 계획은 서 있는 차가 있는 도로를 피한다. 접촉 없음."""
    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[2], FLEET_CFG[3]])
        U, F = f.vehicles["B-ugv1"], f.vehicles["B-fire1"]
        await U.execute(body("S-B-U", 37.951004, 128.337462))
        await F.execute(body("S-B-F", 37.955735, 128.317915))
        ok, w1 = await run(f, clock, 900, until=done(f, "S-B-U", "S-B-F"))
        assert ok and w1["overlaps"] == 0
        from ugv.approach import node_id_of
        rU = await U.execute({"task_id": "H-U", "decision_id": "D", "target_node": node_id_of(U.access)})
        await run(f, clock, 60)
        try:
            await F.execute({"task_id": "H-F", "decision_id": "D", "target_node": node_id_of(F.access)})
        except Exception as e:                                           # 길이 막혀 거절될 수 있다 (정상 응답)
            assert "TARGET_UNREACHABLE" in str(e)
        ok, w2 = await run(f, clock, 1200, until=done(f, "H-U") if "H-F" not in f.store.tasks else done(f, "H-U", "H-F"))
        assert w2["overlaps"] == 0 and w2["min_gap_m"] > 1.0, w2
        st = {k: f.store.tasks[k]["status"] for k in ("H-U", "H-F") if k in f.store.tasks}
        dead = any(e["event"] == "TRAFFIC_DEADLOCK" for v in (U, F) for e in v.events)
        assert all(s == "COMPLETED" for s in st.values()) or dead or f.traffic.waits, st
    asyncio.run(go())


def test_departing_away_from_close_vehicle_behind_is_not_a_conflict(tmp_path):
    """앞차가 뒤차와 간격보다 가깝게 서 있다가 멀어지며 출발해도 충돌로 보지 않는다 (2026-10-10 Gazebo F4)."""
    async def go():
        A0, F0 = dict(FLEET_CFG[0]), dict(FLEET_CFG[1])
        f0, _ = make_fleet(tmp_path / "p", [A0])
        acc = f0.vehicles["A-ugv1"].access
        p = NET.at(acc.road_id, acc.s - 4.0)                      # 출입 지점 4 m 앞에 선 UGV → 뒤 소방차와 간격 8 m 미만
        q = NET.at(acc.road_id, acc.s - 3.0)
        A0["initial"] = {"lat": FRAME.to_ll(p.x, p.y)[0], "lon": FRAME.to_ll(p.x, p.y)[1],
                         "heading_deg": bearing_deg(q.x - p.x, q.y - p.y)}
        f, clock = make_fleet(tmp_path, [A0, F0])
        A = f.vehicles["A-ugv1"]
        await A.execute(body("U1", *FRAME.to_ll(acc.x - 900, acc.y - 700)))
        ok, w = await run(f, clock, 60)
        assert not any(e["event"] == "TRAFFIC_CONFLICT_UNAVOIDABLE" for e in A.events)
        assert f.store.tasks["U1"]["status"] in ("STARTED", "IN_PROGRESS", "COMPLETED") and w["overlaps"] == 0
    asyncio.run(go())


def test_vehicle_with_lost_telemetry_occupies_where_it_may_be(tmp_path):
    """텔레메트리가 끊긴 차는 마지막 위치가 아니라 남은 경로 위 (속도 × 10 s + 30 m) 안 어디에든 있을 수 있다고 본다."""
    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[0], FLEET_CFG[1]])
        A, AF = f.vehicles["A-ugv1"], f.vehicles["A-fire1"]
        await A.execute(body("U1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await run(f, clock, 25)
        v = A.driver.telemetry().speed_mps
        assert v > 8
        A._fail("TELEMETRY_LOST", "시험 주입")
        P = f.traffic._predict(A)
        assert P.static and P.samples[-1][3] >= v * 10 + 20 - 6          # 경로를 따라 넓은 구간
        occ = AF.occupied_roads()
        assert len([r for r, who in occ.items() if who == "A-ugv1"]) >= 1
    asyncio.run(go())


def test_mutual_static_wait_is_broken_without_replan_loop(tmp_path):
    """두 차가 서로를 '서 있는 차' 로 보고 기다리는 고리 (둘 다 임무 중 → 바로 처리하지 않고 TRAFFIC_BLOCK_REPORT_S 기다림).
    60 s 뒤 막힘 처리로 고리가 풀리고, 충돌 판단이 계속 같아도 재계획을 되풀이하지 않는다 (같은 상대 한 번 → 그다음은 실패 보고)."""
    from ugv.traffic import TRAFFIC_BLOCK_REPORT_S
    async def go():
        f, clock = make_fleet(tmp_path, [FLEET_CFG[0], FLEET_CFG[2]])
        A, B = f.vehicles["A-ugv1"], f.vehicles["B-ugv1"]
        await A.execute(body("A1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        await B.execute(body("B1", *FRAME.to_ll(B.access.x + 600, B.access.y + 900)))
        await run(f, clock, 10)
        tm = f.traffic
        tm._first_conflict = lambda P, Q: ("STATIC", P.samples[-1], Q.samples[-1])
        tm._decide = lambda kind, P, Q, a, b, prefer=None: (P, Q, max(P.s_now, a[3] - 5), "TEST")
        for _ in range(int((TRAFFIC_BLOCK_REPORT_S + 90) / 0.5)):
            clock.advance(0.5)
            for v in (A, B):
                v.driver.step(0.5)
                await v.tick()
            await tm.tick()
        ev = {v.resource_id: [e["event"] for e in v.events] for v in (A, B)}
        handled = {r: e.count("TRAFFIC_BLOCKED_BY_VEHICLE") for r, e in ev.items()}
        replans = {r: e.count("REPLANNED_AROUND_VEHICLE") for r, e in ev.items()}
        assert sum(handled.values()) >= 1                                   # 고리가 풀림 (처리됨)
        assert all(n <= 1 for n in replans.values())                        # 같은 상대로 두 번 이상 다시 계획하지 않음
        st = {k: f.store.tasks[k]["status"] for k in ("A1", "B1")}
        assert any(s == "FAILED" for s in st.values()), st                  # 계속 막히면 결국 실패로 보고 (끝없이 기다리지 않음)
        for k in ("A1", "B1"):
            assert len(f.store.tasks[k].get("blocked", [])) <= 2
    asyncio.run(go())
