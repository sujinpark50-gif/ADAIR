# -*- coding: utf-8 -*-
"""최종 경로 검증 회귀 시험 (2026-10-10 검토의 두 결함).

결함 1 — 출발 도로 검증 예외가 너무 넓었다: check_route 가 출발 도로 구간(직선거리로 추정한 start_leg)의 위반을 승인
판정에서 뺐다. 출발 도로의 급커브(반경 1.5~3.6 m < 최소 4.68 m)가 위반을 기록하면서도 OK 로 승인됐다.
결함 2 — 화재 이탈 경로는 최종 검사에 실패해도 실행됐다: plan_evasion 이 check_route 실패에도 OK 를 돌려주고 차량이 주행했다.

시험은 함수 반환값뿐 아니라 드라이버에 실제로 보낸 미션과 차량 상태 전이까지 본다.
수정 전 코드(커밋 90084cb)에서 결함 1·2 시험은 실패한다 (docs/ugv 검증 보고서에 실행 기록).
"""

import asyncio
import math
import random

from interfaces.exec_store import ExecStore
from ugv import approach as approach_mod
from ugv.approach import plan_approach, plan_evasion, verify_route
from ugv.clock import ManualClock
from ugv.drive_graph import DirPos, access_point
from ugv.fire import FireArea, StaticFireSource
from ugv.geo import FRAME
from ugv.roads import RoadNetwork, plan_speeds
from ugv.vehicle import Vehicle

NET = RoadNetwork()
G = NET.graph
ACC = access_point(NET, "A")
NOFIRE = FireArea([], source="T")
VCFG = {"resource_id": "A-ugv1", "resource_type": "UGV", "station_base_id": "A", "initial": "STATION", "driver": "sim"}
SHARP_ROAD, SHARP_TOWARD = "684500365", "496432"      # 도로 중간 최소 반경 3.51 m (< 최소 회전반경 4.68 m)


def independent_ok(route, g=G):
    """독립 재검사: 회전반경(조향 한계)·차체 위반 0, 초기 자세 예외 0 이어야 한다."""
    chk = g.check_route(route)
    return chk["ok"] and chk["steering_violations"] == 0 and chk["body_violations"] == 0, chk


def sharp_start(s):
    r = NET.roads[SHARP_ROAD]
    return DirPos(NET.at(SHARP_ROAD, s if SHARP_TOWARD == r.b else r.length - s), SHARP_TOWARD)


def target_on(rid, toward, frac):
    r = NET.roads[rid]
    return NET.at(rid, r.length * frac if toward == r.b else r.length * (1 - frac))


# ---------------------------------------------------------------------- 결함 1: 출발 도로 예외
def test_defect1_sharp_curve_on_start_road_same_road_target_is_not_approved():
    """같은 도로 위 목표(교차로 없음)로 가는 경로가 조향 한계보다 좁은 급커브를 지나면 승인하지 않는다.
    수정 전: OK, 최소 반경 1.56 m, 반경 위반 13건이 '출발 구간' 이라 승인 판정에서 빠졌다."""
    assert SHARP_ROAD in G.sharp_roads and G.min_radius[SHARP_ROAD] < G.p.r_min
    for s in (5.0, 36.0):
        st = sharp_start(s)
        tgt = target_on(SHARP_ROAD, SHARP_TOWARD, 0.6)
        plan = plan_approach(NET, NOFIRE, st, *FRAME.to_ll(tgt.x, tgt.y), access=ACC[0])
        if plan.status == "OK":
            ok, chk = independent_ok(plan.route)
            assert ok, chk                       # 승인했다면 급커브를 지나지 않는 경로여야 한다
        else:
            assert plan.reason.startswith("NO_DRIVABLE_ROUTE") or "NO_DIRECTED" in plan.reason


def test_defect1_start_beyond_the_curve_is_still_approved():
    """급커브를 이미 지난 위치(같은 도로 72 m)에서 앞쪽 목표는 정상 승인 — 불필요한 거절 없음."""
    st = sharp_start(72.0)
    tgt = target_on(SHARP_ROAD, SHARP_TOWARD, 0.6)
    plan = plan_approach(NET, NOFIRE, st, *FRAME.to_ll(tgt.x, tgt.y), access=ACC[0])
    assert plan.status == "OK"
    ok, chk = independent_ok(plan.route)
    assert ok and chk["min_radius_m"] is None or chk["min_radius_m"] >= G.p.r_min * 0.98


def test_defect1_no_start_leg_exemption_anywhere():
    """임의의 출발점(도로 중간, 교차로 5 m 앞 포함)에서 승인된 경로는 출발 도로·첫 교차로 원호 진입부를 포함해 전 구간
    위반 0. 초기 자세 예외는 출발 자세 차체가 이미 도로 밖일 때만 쓰이고, 그 범위는 차체 길이 안이다."""
    rnd = random.Random(5)
    states = [(r, t) for r, x in NET.roads.items() for t in (x.a, x.b)
              if G.oneway_ok(r, G.other(r, t)) and not x.attrs.get("motorway")]
    approved = 0
    samples = NET.samples()
    for k in range(80):
        if approved >= 12:
            break
        rid, to = rnd.choice(states)
        r = NET.roads[rid]
        if k % 2 and r.length > 12:
            s = r.length - 5.0 if to == r.b else 5.0                      # 첫 교차로 5 m 앞 (원호 진입부가 출발 도로 안)
        else:
            s = rnd.uniform(0, r.length)
        st = DirPos(NET.at(rid, s), to)
        near = [q for q in samples[::7] if 200 < math.hypot(q.x - st.pos.x, q.y - st.pos.y) < 3000]
        if not near:
            continue
        p = rnd.choice(near)
        plan = plan_approach(NET, NOFIRE, st, *FRAME.to_ll(p.x, p.y), access=ACC[0])
        if plan.status != "OK":
            continue
        approved += 1
        chk = G.check_route(plan.route)
        ip = chk["initial_pose"]
        assert chk["ok"] and not chk["issues"], chk["issues"][:3]
        assert ip["exempt_samples"] == 0 or (ip["excess_m"] > 0.02 and ip["exempt_until_m"] <= G.p.body_length_m)
        assert plan.report()["route_check"]["ok"] is True
        assert plan.report()["route_check"]["mission_items"]["ok"] is True        # PX4 미션 선도 통과
    assert approved >= 10


def test_defect1_failing_future_segment_is_reported_not_exempted():
    """check_route 는 출발점 근처라도 회전반경 위반을 예외로 빼지 않는다 (예전 start_leg_issues 없음)."""
    st = sharp_start(5.0)
    tgt = target_on(SHARP_ROAD, SHARP_TOWARD, 0.6)
    tree = G.search(st, 0.0)
    arr = next(a for a in G.arrivals(tree, tgt) if a.kind == "DIRECT")
    route = G.build_route(tree, tgt, arr)
    chk = G.check_route(route)
    assert not chk["ok"] and chk["steering_violations"] > 0
    assert "start_leg_issues" not in chk


# ---------------------------------------------------------------------- 결함 2: 화재 이탈 검증 우회
def _fire_near(tel, rings=(60, 75, 90)):
    from ugv.tests.test_stage3_drive import fire_block
    offs = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for r in rings for a in range(0, 360, 30)]
    return next(c for c in (fire_block(*FRAME.to_ll(tel.x + dx, tel.y + dy), n=0) for dx, dy in offs) if c)


def make(tmp_path, cells=()):
    clock = ManualClock()
    src = StaticFireSource(cells, "TEST")
    v = Vehicle(dict(VCFG), NET, src, clock, ExecStore(tmp_path / "tasks.json"))
    sent = []
    orig = v.driver.follow

    async def follow(items):
        sent.append({"state": v.state, "items": list(items)})
        return await orig(items)
    v.driver.follow = follow
    return v, clock, src, sent


async def run_for(v, clock, sim_s, dt=0.5, until=None):
    for _ in range(int(sim_s / dt)):
        clock.advance(dt)
        v.driver.step(dt)
        await v.tick()
        if until and until():
            return True
    return False


def test_defect2_evasion_route_must_pass_final_check():
    """이탈 계획이 돌려주는 경로는 verify_route(회전반경·차체·PX4 미션 선)를 통과한 것뿐이다."""
    here = G.dirpos(ACC[0], ACC[1])
    cells = _fire_near(here.pos)
    ev = plan_evasion(NET, FireArea(cells, source="T"), here, access=ACC[0])
    assert ev.status == "OK"
    assert verify_route(NET, ev.route)["ok"] is True
    assert ev.report()["route_check"]["ok"] is True


def test_defect2_first_candidate_fails_next_is_chosen(monkeypatch):
    """첫(가장 빠른) 이탈 후보가 최종 검사에 실패하면 버리고 다음 후보를 고른다 (검사 실패를 주입)."""
    here = G.dirpos(ACC[0], ACC[1])
    fire = FireArea(_fire_near(here.pos), source="T")
    first = plan_evasion(NET, fire, here, access=ACC[0])
    assert first.status == "OK"
    bad_dest = first.dest
    real = approach_mod.verify_route

    def fake(net, route, graph=None):
        v = real(net, route, graph)
        if route.end == bad_dest:                       # 첫 후보 경로는 차체가 도로 밖으로 나간다고 가정
            v = {**v, "ok": False, "issue": {"kind": "BODY", "s_m": 1.0, "x": route.pts[1][0], "y": route.pts[1][1],
                                             "value": 0.5}}
        return v
    monkeypatch.setattr(approach_mod, "verify_route", fake)
    ev = plan_evasion(NET, fire, here, access=ACC[0])
    assert ev.status == "OK" and ev.dest != bad_dest
    assert ev.detail["evasion_search"]["rejected_route_check"] >= 1
    assert real(NET, ev.route)["ok"] is True


def test_defect2_vehicle_never_drives_failed_evasion_and_enters_danger(tmp_path, monkeypatch):
    """모든 이탈 후보가 검사에 실패하면: 이탈 미션을 보내지 않고, 도로를 따른 감속 정지 미션만 보내고, DANGER·운영자 개입
    요청·구체 사유를 남긴다. 그 뒤 자동 재출발이 없다."""
    async def go():
        v, clock, src, sent = make(tmp_path)
        await v.execute({"task_id": "T1", "decision_id": "D", "target": {"lat": 38.0277, "lon": 128.1303}})
        await run_for(v, clock, 25)
        n_before = len(sent)
        real = approach_mod.verify_route

        def always_fail(net, route, graph=None):
            r = real(net, route, graph)
            return {**r, "ok": False, "issue": {"kind": "STEERING_LIMIT", "s_m": 2.0, "x": route.pts[1][0],
                                                "y": route.pts[1][1], "value": 3.0}}
        monkeypatch.setattr(approach_mod, "verify_route", always_fail)
        tel = v.driver.telemetry()
        src.set(_fire_near(tel), "TEST")
        await run_for(v, clock, 2)
        assert v.state == "DANGER"
        new = sent[n_before:]
        assert len(new) == 1                                       # 정지 미션 하나뿐 (이탈 미션 없음)
        stop = [e for e in v.events if e["event"] == "STOP_PATH"][-1]
        length = sum(math.dist(a[:2], b[:2]) for a, b in zip(new[0]["items"], new[0]["items"][1:]))
        assert length <= stop["needed_m"] + 3.0                    # 제동거리만큼만 가는 정지 경로
        d = v.state_view()["operator_intervention"]
        assert d["required"] is True and d["danger"]["operator_action_required"] is True
        assert "NO_SAFE_EVASION_ROUTE" in v.fault and "검사" in v.fault
        assert v.store.tasks["T1"]["status"] == "FAILED"
        n_after = len(sent)
        await run_for(v, clock, 60)
        assert v.state == "DANGER" and len(sent) == n_after        # 자동 재출발 없음
    asyncio.run(go())


def test_defect2_unverified_plan_is_refused_at_driver_boundary(tmp_path):
    """검사 통과 기록이 없는 계획은 드라이버로 보내지 않는다 (실행 경계 확인)."""
    async def go():
        v, clock, src, sent = make(tmp_path)
        st, v0, _ = v.here_state()
        plan = plan_approach(NET, NOFIRE, st, 38.0277, 128.1303, access=v.access)
        plan.detail["route_check"] = {**plan.detail["route_check"], "ok": False}
        from ugv.drivers import DriverError
        try:
            await v._drive(plan)
            raise AssertionError("검사 실패 계획이 실행됐다")
        except DriverError as e:
            assert "UNVERIFIED_ROUTE" in str(e)
        assert sent == []
    asyncio.run(go())


def test_stop_path_shortfall_is_reported(tmp_path):
    """막다른 길 끝 가까이에서 빠르게 달리다 정지하면 정지 경로가 제동거리보다 짧다 → short 로 보고한다."""
    async def go():
        dead = next((rid, n) for n, arms in NET.adj.items() if len(arms) == 1
                    for rid, _ in arms if NET.roads[rid].length > 120 and G.oneway_ok(rid, G.other(rid, n)))
        rid, n = dead
        r = NET.roads[rid]
        s = r.length - 30.0 if n == r.b else 30.0
        p = NET.at(rid, s)
        q = NET.at(rid, s + 2 if n == r.b else s - 2)
        from ugv.geo import bearing_deg
        hd = bearing_deg(q.x - p.x, q.y - p.y)
        la, lo = FRAME.to_ll(p.x, p.y)
        clock = ManualClock()
        v = Vehicle(dict(VCFG, initial={"lat": la, "lon": lo, "heading_deg": hd}), NET, StaticFireSource([], "T"), clock,
                    ExecStore(tmp_path / "t.json"))
        v.driver.v = 15.0                                            # 15 m/s 로 막다른 길 30 m 앞
        info = await v._safe_stop()
        assert info["short"] is True and info["length_m"] < info["needed_m"]
        assert any("필요" in x for x in info["problems"])
    asyncio.run(go())


# ---------------------------------------------------------------------- 계획 → PX4 미션 연결
def test_mission_items_never_cut_across_checked_arcs():
    """PX4 미션 지점을 이은 직선이 다듬은 선형에서 ITEM_CHORD_TOL_M 이상 벗어나지 않고, 그 선도 차체 검사를 통과한다."""
    from ugv import config
    rnd = random.Random(8)
    st = G.dirpos(ACC[0], ACC[1])
    n = 0
    for _ in range(12):
        p = rnd.choice(NET.samples())
        plan = plan_approach(NET, NOFIRE, st, *FRAME.to_ll(p.x, p.y), access=ACC[0])
        if plan.status != "OK":
            continue
        sp = plan_speeds(plan.route)
        assert sp.max_chord_dev_m <= config.ITEM_CHORD_TOL_M + 1e-6
        line = [plan.route.pts[0]] + [(x, y) for x, y, _ in sp.items]
        assert G.check_route(line, check_radius=False)["ok"]
        n += 1
    assert n >= 5


def test_return_feasibility_does_not_use_overlapping_turn_arcs():
    """짧은 도로에서 들어오는 원호와 나가는 원호가 겹치는 연속 회전은 복귀 가능성의 근거가 되지 않는다:
    복귀 가능 꼬리표 (도로, 들어온 회전) 마다 겹치지 않는 다음 회전으로 이어지는 복귀 가능 꼬리표가 있다."""
    rs = G.return_set(ACC[0])
    acc_road = ACC[0].road_id
    assert rs["labels"]
    for (s, pin) in rs["labels"]:
        if s[0] == acc_road:
            continue
        r = NET.roads[s[0]]
        assert any((t.to, s) in rs["labels"] and G.tan[(pin, s)] + t.tangent_m <= r.length + 1e-6
                   for t in G.turns.get(s, ())), (pin, s)


def test_evasion_route_never_crosses_fire_cell_on_straight_segment():
    """이탈 경로가 화재 칸을 가로지르면 안 된다 — 꼭짓점 사이 긴 직선 선분도 본다 (2026-10-10 Gazebo G5: 꼭짓점만 봐서 놓침)."""
    from ugv.fire import densify
    rnd = random.Random(21)
    from ugv.tests.test_stage3_drive import fire_block
    hit = 0
    for _ in range(25):
        rid = rnd.choice([r for r, x in NET.roads.items() if x.length > 400 and x.attrs["oneway"] == "BOTH"])
        r = NET.roads[rid]
        here = DirPos(NET.at(rid, 60.0), r.b)
        ahead = NET.at(rid, 180.0)
        # 진행 방향 앞 도로 옆에 칸 (칸 사각형이 도로를 덮을 수 있다)
        cells = []
        for off in (50.0, -50.0):
            q = NET.at(rid, 185.0)
            tx, ty = ahead.x - q.x, ahead.y - q.y
            n = math.hypot(tx, ty) or 1.0
            cells += fire_block(*FRAME.to_ll(ahead.x - ty / n * off, ahead.y + tx / n * off), n=0)
        if not cells:
            continue
        fire = FireArea(cells, source="T")
        if fire.distance(here.pos.x, here.pos.y) >= 100:
            continue
        ev = plan_evasion(NET, fire, here, access=ACC[0])
        if ev.status == "OK":
            pts = densify(ev.route.pts)
            inside = [fire.distance(x, y, max_m=1.0) < 0.5 for x, y in pts]
            k = next((i for i, v in enumerate(inside) if not v), len(inside))
            assert not any(inside[k:]), "이탈 경로가 화재 칸을 지난다"
            hit += 1
    assert hit >= 3
