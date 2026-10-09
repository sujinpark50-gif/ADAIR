# -*- coding: utf-8 -*-
"""3단계: 진행 방향 유지 경로 (2026-10-09 사용자 결정) — 모의.

즉시 U턴·후진·3점 회전 없음, 일방통행, 차체가 들어오는 회전만, 첫 교차로 감속 거리, 방향별 복귀 가능성,
도로를 따르는 정지, 경로가 없으면 감속 정지 후 보고. 원자료 도로망(ugv/road_source.py, gpkg 직접 읽기)으로 본다.
"""

import asyncio
import math
import random

import pytest

from interfaces.exec_store import ExecStore
from ugv import config
from ugv.approach import node_id_of, plan_approach
from ugv.clock import ManualClock
from ugv.drive_graph import DirPos, access_point
from ugv.fire import FireArea, StaticFireSource
from ugv.geo import FRAME
from ugv.roads import RoadNetwork
from ugv.vehicle import HttpError, Vehicle

NET = RoadNetwork()
G = NET.graph
ACC = access_point(NET, "A")
START_A = G.dirpos(ACC[0], ACC[1])
NOFIRE = FireArea([], source="T")
TARGET = (38.0277, 128.1303)
VCFG = {"resource_id": "A-ugv1", "resource_type": "UGV", "station_base_id": "A", "initial": "STATION", "driver": "sim"}


def legs_ok(route):
    """연속한 두 구간: 이어져 있고, 같은 도로 되짚기 없고, 회전표에 있는 회전이고, 일방통행을 지킨다."""
    for k, (rid, frm, to) in enumerate(route.legs):
        if k > 0:
            assert G.oneway_ok(rid, frm), f"일방통행 역주행 {rid}"
            prid, pfrm, pto = route.legs[k - 1]
            assert pto == frm, "구간이 이어지지 않음"
            assert rid != prid, f"즉시 U턴 {rid}"
            assert any(t.to == (rid, to) for t in G.turns[(prid, pto)]), f"회전표에 없는 회전 {prid}->{rid}"
    return True


def make(tmp_path, cells=(), initial="STATION"):
    clock = ManualClock()
    src = StaticFireSource(cells, "TEST")
    cfg = dict(VCFG, initial=initial)
    v = Vehicle(cfg, NET, src, clock, ExecStore(tmp_path / "tasks.json"))
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


def plan_from(dp, p, v0=0.0):
    tree = G.search(dp, v0)
    arrs = G.arrivals(tree, p)
    return (G.build_route(tree, p, min(arrs, key=lambda a: a.time_s)) if arrs else None), tree


# ---------------------------------------------------------------------- 데이터·그래프
def test_oneway_attributes_loaded_and_turn_table():
    import sqlite3
    ow = [r for r in NET.roads.values() if r.attrs.get("oneway") != "BOTH"]
    db = sqlite3.connect(config.ROAD_GPKG)
    want = db.execute("select count(*) from roads_clipped_2019 where ONEWAY=1 and AUTO_EXCLU!='1'").fetchone()[0]
    assert len(ow) == want > 100                            # 원자료 ONEWAY=1 (자동차전용 도로 제외)
    assert NET.excluded["motorway"] > 0                 # 자동차전용 도로(AUTO_EXCLU=1)는 뺐다
    for (rin, n), ts in G.turns.items():
        for t in ts:
            assert t.to[0] != rin                           # 같은 도로 되짚기 없음
            assert G.allowed(t.to[0], n)                    # 나가는 도로는 일방통행·급커브 조건을 지킨다
    assert G.blocked_turns and all("deflect_deg" in b for b in G.blocked_turns)


def test_random_routes_keep_direction_rules():
    rnd = random.Random(7)
    samples = NET.samples()
    checked = 0
    for _ in range(40):
        p = rnd.choice(samples)
        r, _ = plan_from(START_A, p)
        if r is None:
            continue
        legs_ok(r)
        assert r.legs[0][2] == START_A.toward              # 출발은 지금 진행 방향
        checked += 1
    assert checked >= 25


def test_oneway_road_is_entered_only_in_its_direction():
    hits = 0
    for r in sorted((r for r in NET.roads.values() if r.attrs.get("oneway") == "A_TO_B"), key=lambda r: r.road_id):
        p = NET.at(r.road_id, r.length / 2)
        route, _ = plan_from(START_A, p)
        if route is None:
            continue
        assert route.legs[-1][0] == r.road_id and route.legs[-1][1] == r.a      # a → b 로만 들어간다
        assert route.arrive_toward == r.b
        legs_ok(route)
        hits += 1
        if hits >= 8:
            break
    assert hits >= 8


# ---------------------------------------------------------------------- 반대·뒤쪽 목표
def _long_two_way_road(min_len=400):
    for rid in sorted(NET.roads):
        r = NET.roads[rid]
        if r.length >= min_len and r.attrs.get("oneway") == "BOTH" and rid not in G.sharp_roads \
                and G.turns.get((rid, r.b)) and G.turns.get((rid, r.a)):
            return r
    raise AssertionError("시험 도로 없음")


def test_target_behind_on_same_road_loops_around_instead_of_u_turn():
    """같은 도로 150 m 뒤 목표: 되짚지 않고 앞으로 가서 도로망을 돌아온다. 돌아올 길이 없는 곳이면 경로 없음 (U턴 하지 않는다)."""
    looped = none = 0
    for rid in sorted(NET.roads):
        r = NET.roads[rid]
        if not (r.length >= 400 and r.attrs.get("oneway") == "BOTH" and rid not in G.sharp_roads):
            continue
        dp = DirPos(NET.at(rid, r.length / 2), r.b)                         # 가운데에서 b 쪽으로 가는 중
        behind = NET.at(rid, r.length / 2 - 150)                            # 같은 도로 150 m 뒤
        route, tree = plan_from(dp, behind, v0=10.0)
        assert all(a.kind != "DIRECT" for a in G.arrivals(tree, behind))
        if route is None:
            none += 1
            continue
        assert route.legs[0] == (rid, r.a, r.b)                              # 먼저 앞으로 간다
        assert route.length > r.length / 2 + 150                             # 돌아서 온다
        legs_ok(route)
        looped += 1
        if looped >= 5:
            break
    assert looped >= 5


def test_target_ahead_on_same_road_is_direct_only_beyond_braking_distance():
    r = _long_two_way_road()
    dp = DirPos(NET.at(r.road_id, 50.0), r.b)
    near = NET.at(r.road_id, 80.0)                                          # 30 m 앞
    far = NET.at(r.road_id, 50.0 + 120.0)                                   # 120 m 앞
    tree_fast = G.search(dp, 16.0)                                          # 16 m/s 제동거리 ≈ 85 m
    assert not any(a.kind == "DIRECT" for a in G.arrivals(tree_fast, near))
    assert any(a.kind == "DIRECT" for a in G.arrivals(tree_fast, far))
    assert any(a.kind == "DIRECT" for a in G.arrivals(G.search(dp, 0.0), near))


def test_opposite_target_while_moving_keeps_heading(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        await v.execute(body("T1"))
        await run_for(v, clock, 60)
        tel = v.driver.telemetry()
        assert tel.speed_mps > 5
        i = max(0, v._near_i - 60)                                          # 지나온 경로 약 300 m 뒤
        back_ll = FRAME.to_ll(*v.speed.pts[i])
        r2 = await v.execute(body("T2", lat=back_ll[0], lon=back_ll[1]))
        assert r2["superseded_task_id"] == "T1"
        legs_ok(v.plan.route)
        hx, hy = math.sin(math.radians(tel.heading_deg)), math.cos(math.radians(tel.heading_deg))
        fx, fy = v.speed.items[0][0] - tel.x, v.speed.items[0][1] - tel.y
        assert fx * hx + fy * hy > 0                                       # 첫 미션 지점은 앞쪽
        assert r2["approach"]["direction"]["return_path_at_plan_time"] is True
        done = await run_for(v, clock, 3600, until=lambda: v.store.tasks["T2"]["status"] in ("COMPLETED", "FAILED"))
        assert done and v.store.tasks["T2"]["status"] == "COMPLETED"
    asyncio.run(go())


# ---------------------------------------------------------------------- 막다른 길·좁은 교차로
def _dead_end_road():
    """한쪽 끝이 막다른 노드이고 거점에서 먼 도로 (지도 경계가 아닌 것)."""
    for rid in sorted(NET.roads):
        r = NET.roads[rid]
        for tip, root in ((r.b, r.a), (r.a, r.b)):
            if len(NET.adj[tip]) == 1 and len(NET.adj[root]) >= 3 and r.length > 150 and r.attrs.get("oneway") == "BOTH":
                if math.dist(NET.nodes[tip], (START_A.pos.x, START_A.pos.y)) > 2000:
                    return r, tip, root
    raise AssertionError("시험 도로 없음")


def test_dead_end_is_not_chosen_when_no_return_path():
    r, tip, root = _dead_end_road()
    tip_ll = FRAME.to_ll(*NET.nodes[tip])
    without = plan_approach(NET, NOFIRE, START_A, *tip_ll)                  # 복귀 검사 없음 → 막다른 끝으로 간다
    assert without.status == "OK" and without.dest.road_id == r.road_id
    with_ret = plan_approach(NET, NOFIRE, START_A, *tip_ll, access=ACC[0])
    if with_ret.status == "OK":
        d = with_ret.dest
        assert G.can_return(G.return_set(ACC[0]), d.road_id, d.s, with_ret.route.arrive_toward)
        assert with_ret.report()["direction"]["return_path_at_plan_time"] is True
        assert d.road_id != r.road_id or with_ret.route.arrive_toward != tip
    else:
        assert with_ret.reason in ("NO_APPROACH_WITH_RETURN_PATH",) or "DIRECTED" in with_ret.reason
    assert with_ret.report()["candidates_no_return_path"] > 0


def test_geometrically_blocked_turn_is_never_used():
    """기하로 막힌 회전 (대부분 짧은 연결로의 120° 넘는 꺾임): 그 회전 직전에서 그 회전 너머로 가는 경로가 그 회전을 쓰지 않는다."""
    used = 0
    for b in [b for b in G.blocked_turns if b["entry_legal"]]:
        rin, rout, n = NET.roads[b["from_road"]], NET.roads[b["to_road"]], b["node"]
        dp = DirPos(NET.at(rin.road_id, rin.length / 2), n)
        route, _ = plan_from(dp, NET.at(rout.road_id, rout.length / 2))
        if route is None:
            continue
        for (r1, _, t1), (r2, _, _) in zip(route.legs, route.legs[1:]):
            assert not (r1 == rin.road_id and t1 == n and r2 == rout.road_id), "막힌 회전을 썼다"
        legs_ok(route)
        used += 1
    assert used >= 3


def test_first_junction_turn_needs_braking_distance():
    found = 0
    for (rin, n), ts in sorted(G.turns.items()):
        r = NET.roads[rin]
        slow = [t for t in ts if t.v_turn < 6.0 and NET.roads[t.to[0]].length > 60]
        if not slow or r.length < 120 or not G.oneway_ok(rin, G.other(rin, n)):
            continue
        t = slow[0]
        dp = DirPos(NET.at(rin, r.length - 40 if n == r.b else 40), n)    # 교차로 40 m 앞
        ro = NET.roads[t.to[0]]
        p = NET.at(ro.road_id, 30 if t.to[1] == ro.b else ro.length - 30)
        stopped, _ = plan_from(dp, p, v0=0.0)
        fast, _ = plan_from(dp, p, v0=16.0)                                 # 16 m/s 에서 v_turn 까지 감속 ≈ 70 m 필요
        assert stopped is not None and stopped.legs[1][0] == ro.road_id    # 멈춰 있으면 거기서 돈다
        if fast is not None:
            assert fast.legs[1][0] != ro.road_id                           # 빠르면 첫 교차로에서 돌지 않는다
            legs_ok(fast)
        found += 1
        if found >= 5:
            break
    assert found >= 3


# ---------------------------------------------------------------------- 거점 복귀·정지·보고
def test_station_access_point_is_separate_from_station_coordinates(tmp_path):
    v, _, _ = make(tmp_path)
    st = v.state_view()["station"]
    assert st["road_point"] != {k: st["access_point"][k] for k in ("lat", "lon")}
    acc = NET.roads[st["access_point"]["road_id"]]
    assert (acc.road_id, acc.a) in G.core and (acc.road_id, acc.b) in G.core      # 출입 지점은 서로 오갈 수 있는 도로
    near = NET.snap_ll(st["lat"], st["lon"])                                   # 소방서 좌표에서 가장 가까운 도로
    nr = NET.roads[near.road_id]
    assert near.road_id != acc.road_id and min(len(NET.adj[nr.a]), len(NET.adj[nr.b])) == 1   # 그 도로는 막다른 길


def test_return_to_station_after_arrival(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        await v.execute(body("T1"))
        assert await run_for(v, clock, 1500, until=lambda: v.store.tasks["T1"]["status"] == "COMPLETED")
        home = node_id_of(v.access)
        ev = await v.evaluate({"task_id": "RET", "decision_id": "D", "target_node": home})
        assert ev["verdict"] == "ACCEPT", ev
        await v.execute({"task_id": "RET", "decision_id": "D", "target_node": home})
        legs_ok(v.plan.route)
        assert await run_for(v, clock, 3600, until=lambda: v.store.tasks["RET"]["status"] == "COMPLETED")
        tel = v.driver.telemetry()
        assert math.hypot(tel.x - v.access.x, tel.y - v.access.y) < config.ARRIVE_M
    asyncio.run(go())


def test_abort_on_curve_stops_along_road(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        await v.execute(body("T1"))
        pts, cum = v.speed.pts, v.speed.cum
        import bisect

        def hd(i):
            j = min(i + 1, len(pts) - 1)
            return math.atan2(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1])

        # 앞 70 m 안에서 진행 방향이 18° 넘게 바뀌는 곳 직전까지 달린다 (점 간격이 일정하지 않아 거리로 센다)
        def turn_ahead():
            i = v._near_i
            k = bisect.bisect_left(cum, cum[i] + 70.0)
            if k >= len(pts) - 1 or v.driver.v <= 8:
                return False
            return abs((math.degrees(hd(k) - hd(i)) + 180) % 360 - 180) > 18
        assert await run_for(v, clock, 600, until=turn_ahead)
        tel = v.driver.telemetry()
        straight = (tel.x + 60 * math.sin(math.radians(tel.heading_deg)), tel.y + 60 * math.cos(math.radians(tel.heading_deg)))
        await v.abort("T1")
        stop_ev = [e for e in v.events if e["event"] == "STOP_PATH"][-1]
        assert stop_ev["basis"] == "PLANNED_ROUTE"
        for x, y, _ in v.driver.path:
            assert NET.snap(x, y).dist < 1.0                               # 정지 경로는 도로 중심선 위
        await run_for(v, clock, 40)
        tel2 = v.driver.telemetry()
        assert tel2.speed_mps == 0 and v.state == "IDLE"
        assert NET.snap(tel2.x, tel2.y).dist < 1.0
        assert NET.snap(*straight).dist > NET.snap(*v.driver.path[-1][:2]).dist + 2.0  # 직선 정지보다 도로에 붙어 있다
    asyncio.run(go())


def test_new_target_without_feasible_route_stops_and_reports(tmp_path):
    async def go():
        v, clock, _ = make(tmp_path)
        await v.execute(body("T1"))
        await run_for(v, clock, 40)
        assert v.driver.v > 5
        with pytest.raises(HttpError) as e:
            await v.execute(body("T2", lat=37.50, lon=127.50))             # 도로에서 먼 목표 → 경로 없음
        d = e.value.detail
        assert e.value.code == 409 and d["cancelled_task_id"] == "T1" and d["vehicle_action"] == "STOPPING_ALONG_ROAD"
        assert v.store.tasks["T1"]["status"] == "CANCELLED" and "NEW_TARGET_NO_FEASIBLE_ROUTE" in v.store.tasks["T1"]["error"]
        assert "T2" not in v.store.tasks
        await run_for(v, clock, 40)
        tel = v.driver.telemetry()
        assert tel.speed_mps == 0 and v.state == "IDLE" and NET.snap(tel.x, tel.y).dist < 1.0
        assert any(e["event"] == "NEW_TARGET_REJECTED" for e in v.events)
    asyncio.run(go())


def test_idle_vehicle_keeps_direction_and_wrong_way_on_oneway_is_rejected(tmp_path):
    async def go():
        r = next(r for r in sorted(NET.roads.values(), key=lambda r: r.road_id)
                 if r.attrs.get("oneway") == "A_TO_B" and r.length > 100)
        p = NET.at(r.road_id, r.length / 2)
        q = NET.at(r.road_id, r.length / 2 + 5)
        lat, lon = FRAME.to_ll(p.x, p.y)
        wrong = (math.degrees(math.atan2(p.x - q.x, p.y - q.y))) % 360    # b → a 방향 (역방향)
        v, clock, _ = make(tmp_path, initial={"lat": lat, "lon": lon, "heading_deg": wrong})
        ev = await v.evaluate(body("E1"))
        assert ev["verdict"] == "REJECT" and ev["reason"] == "WRONG_WAY_ON_ONEWAY"
        right = (wrong + 180) % 360
        v2, _, _ = make(tmp_path / "b", initial={"lat": lat, "lon": lon, "heading_deg": right})
        ev2 = await v2.evaluate(body("E2"))
        assert ev2["verdict"] == "ACCEPT"
        assert ev2["approach"]["direction"]["start_toward_node"] == r.b
    (tmp_path / "b").mkdir()
    asyncio.run(go())


# ---------------------------------------------------------------------- 교차로 선형 (2026-10-10)
def test_route_corners_are_smoothed_into_drivable_arcs():
    """꺾인 점을 원호로 다듬는다: 다듬기 전 선형은 최소 회전반경보다 좁은 꺾임투성이, 다듬은 선형은 거의 없다.
    차가 다듬은 선을 따라가면(뒤축이 선 위) 차체 모서리가 추정 도로 띠 안에 남는다 (근사 기하 검사)."""
    import bisect
    from ugv.drive_graph import R_MIN, DriveGraph
    rnd = random.Random(3)
    states = [(r, t) for r, x in NET.roads.items() for t in (x.a, x.b) if G.allowed(r, G.other(r, t))]

    def measure(smooth):
        tight = outside = worst = 0.0
        orig = DriveGraph._smooth
        if not smooth:
            DriveGraph._smooth = lambda self, pts, vlim, legs: (pts, vlim)
        try:
            for _ in range(30):
                rid, to = rnd.choice(states)
                r = NET.roads[rid]
                sp = DirPos(NET.at(rid, r.length / 2), to)
                p = rnd.choice(NET.samples())
                route, _ = plan_from(sp, p)
                if route is None:
                    continue
                pts = route.pts
                assert math.dist(pts[0], (sp.pos.x, sp.pos.y)) < 0.1 and math.dist(pts[-1], (p.x, p.y)) < 0.1
                cum = [0.0]
                for a, b in zip(pts, pts[1:]):
                    cum.append(cum[-1] + math.dist(a, b))

                def at(s):
                    i = max(0, min(len(cum) - 2, bisect.bisect_left(cum, s) - 1))
                    t = (s - cum[i]) / max(cum[i + 1] - cum[i], 1e-9)
                    a, b = pts[i], pts[i + 1]
                    return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
                s = 3.0
                while s < cum[-1] - 3.0:
                    a, b, c = at(s - 3), at(s), at(s + 3)
                    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
                    if area2 > 1e-9 and math.dist(a, b) * math.dist(b, c) * math.dist(a, c) / (2 * area2) < R_MIN:
                        tight += 1
                    yaw = math.atan2(c[1] - a[1], c[0] - a[0])
                    cx, cy = b[0] + 1.35 * math.cos(yaw), b[1] + 1.35 * math.sin(yaw)
                    for fx, fy in ((2.1, 0.9), (2.1, -0.9), (-2.1, 0.9), (-2.1, -0.9)):
                        e = G.outside_m(cx + fx * math.cos(yaw) - fy * math.sin(yaw), cy + fx * math.sin(yaw) + fy * math.cos(yaw))
                        if e > 0:
                            outside += 1
                            worst = max(worst, e)
                    s += 3.0
        finally:
            DriveGraph._smooth = orig
        return tight, outside, worst

    rnd.seed(3)
    raw_tight, _, _ = measure(False)
    rnd.seed(3)
    tight, outside, worst = measure(True)
    assert raw_tight > 100 and tight < raw_tight / 10
    assert worst < 0.5                   # 남는 초과는 교차로 몇 곳의 수십 cm (근사 기하)


def test_mission_items_follow_arcs():
    from ugv.roads import plan_speeds
    dst = NET.snap_ll(*TARGET)
    route, _ = plan_from(START_A, dst)
    sp = plan_speeds(route)
    assert len(sp.items) < 2000                                            # PX4 미션 한도(SITL 10000) 안
    for x, y, _ in sp.items:
        assert G.outside_m(x, y) < -0.5                                    # 미션 지점은 도로 띠 안쪽
