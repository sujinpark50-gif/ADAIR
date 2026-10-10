# -*- coding: utf-8 -*-
"""2026-10-10 재시험 도구 — 소방차 치수 판정, 계획 선형 노출, 이탈 원인 분류.

소방차를 UGV 치수로 판정하면 이탈을 작게 본다 → road_test 는 기록의 차량 특성으로 판정해야 한다.
/ugv/{id}/route 는 계획 선형·속도를 내줘야 PX4 궤적과 비교할 수 있다.
"""

from fastapi.testclient import TestClient

from interfaces.exec_store import ExecStore
from orchestrator.resources import UgvClient
from ugv import config
from ugv.clock import ManualClock
from ugv.fire import StaticFireSource
from ugv.geo import FRAME
from ugv.profiles import get as get_profile
from ugv.server import Fleet, create_app
from ugv.tests.test_orchestrator_link import FLEET_CFG
from ugv.tests.test_stage3_drive import NET
from ugv.tools import offroad_cause, road_test


def test_road_test_judges_with_vehicle_profile():
    try:
        road_test.set_profile("adair_firetruck")
        ft = get_profile("adair_firetruck")
        assert road_test._rac() == ft.rear_axle_from_center_m
        xs = [x for x, _ in road_test.BODY_OUTLINE]
        ys = [y for _, y in road_test.BODY_OUTLINE]
        assert abs(max(xs) - min(xs) - ft.body_length_m) < 1e-6 and abs(max(ys) - min(ys) - ft.body_width_m) < 1e-6
        ugv = get_profile("adair_ugv")
        assert ft.body_length_m > ugv.body_length_m                       # UGV 치수로 판정하면 이탈을 작게 본다
        road_test.set_profile("adair_ugv")
        xs = [x for x, _ in road_test.BODY_OUTLINE]
        assert abs(max(xs) - min(xs) - ugv.body_length_m) < 1e-6
    finally:
        road_test.PROF = None
        road_test.BODY_OUTLINE = tuple((x - config.REAR_AXLE_FROM_CENTER_M, y) for x, y in road_test.BODY_SAMPLES)


def test_route_exposes_plan_polyline_and_speeds(tmp_path):
    fleet = Fleet(FLEET_CFG, net=NET, fire_source=StaticFireSource([], "T"), clock=ManualClock(),
                  store=ExecStore(tmp_path / "t.json"))
    with TestClient(create_app(fleet)) as http:
        assert http.get("/ugv/A-fire1/route").json()["route"] is None
        a = fleet.vehicles["A-fire1"]
        lat, lon = FRAME.to_ll(a.access.x - 900, a.access.y - 700)
        body = {"task_id": "T-R1", "decision_id": "D-R1", "target": {"lat": lat, "lon": lon}}
        client = UgvClient(base_url="http://testserver", client=http)
        ev = client.evaluate("A-fire1", body)
        assert ev["verdict"] == "ACCEPT"
        kind, _ = client.execute("A-fire1", {**body, "target_node": ev["constraints"]["target_node"]["node_id"]})
        assert kind == "ACK"
        rt = http.get("/ugv/A-fire1/route").json()
        assert len(rt["plan_xy"]) == len(rt["plan_v"]) >= 2
        assert rt["plan_v"][-1] == 0.0 and max(rt["plan_v"]) <= get_profile("adair_firetruck").max_speed_mps + 0.01   # 0.01 반올림
        assert "hold" in rt and "last_sent_mission" in rt                  # sim 드라이버는 보낸 미션 기록이 없다 → None


def test_offroad_cause_nearest_and_classification():
    d, i, t = offroad_cause.nearest_on([[0, 0], [10, 0], [10, 10]], 5, 2)
    assert abs(d - 2) < 1e-9 and i == 0 and abs(t - 0.5) < 1e-9
    base = {"x": 0, "y": 0, "heading_err_deg": 0}
    smp = [  # 0.0~0.4 s: 계획은 도로 안, 추종 오차 큼 / 1.0 s: 계획 자체가 밖 / 2.0 s: 둘 다 경계
        {**base, "sim_t": 0.0, "body_excess_actual": 0.2, "cross_track_m": 0.9, "plan_excess_m": -0.8},
        {**base, "sim_t": 0.2, "body_excess_actual": 0.5, "cross_track_m": 1.2, "plan_excess_m": -0.8},
        {**base, "sim_t": 0.4, "body_excess_actual": 0.1, "cross_track_m": 0.7, "plan_excess_m": -0.8},
        {**base, "sim_t": 1.0, "body_excess_actual": 0.3, "cross_track_m": 0.1, "plan_excess_m": 0.2},
        {**base, "sim_t": 1.6, "body_excess_actual": 0.0, "cross_track_m": 0.1, "plan_excess_m": -0.5},
        {**base, "sim_t": 2.0, "body_excess_actual": 0.1, "cross_track_m": 0.2, "plan_excess_m": -0.15},
    ]
    ev = offroad_cause.events(smp)
    assert [e["cause"] for e in ev] == ["PX4_TRACKING", "PLAN_OUTSIDE", "MARGINAL"]
    assert ev[0]["body_excess_actual"] == 0.5 and ev[0]["duration_s"] == 0.6


def test_traffic_time_cache_ignores_reused_id(tmp_path):
    """속도 계획 시간 캐시는 id 만으로 찾지 않는다 — 해제된 옛 계획의 id 를 새 계획이 받으면 옛 시간표(다른 길이)를 쓰던 결함
    (2026-10-10 전체 회귀 시험 test_separate_routes_concurrent_and_each_returns_home 에서 IndexError 로 드러남)."""
    import asyncio
    from ugv.tests.test_multi_vehicle import body, make_fleet
    from ugv.traffic import _tcum

    async def go():
        f, _ = make_fleet(tmp_path)
        A = f.vehicles["A-ugv1"]
        assert (await A.execute(body("C1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700))))["status"] == "STARTED"
        sp = A.speed
        f.traffic._tc[id(sp)] = (object(), [0.0])            # 같은 id 의 옛 계획 시간표 (길이 1)
        pred = f.traffic._predict(A)
        assert pred is not None and len(pred.samples) > 2
        assert f.traffic._tc[id(sp)][0] is sp and f.traffic._tc[id(sp)][1] == _tcum(sp)
    asyncio.run(go())


def test_resume_after_yield_is_not_disarm_fault_and_real_disarm_stops(tmp_path):
    """대기(YIELD) 지점에서 PX4 는 미션 끝 정차로 시동을 끈다. 대기가 풀려 다시 출발할 때 텔레메트리의 시동 상태가 늦게 오면
    '주행 중 시동 꺼짐' 으로 오판했고, 그 고장 처리에는 정지가 없어 FAULT 상태로 남은 미션을 달렸다 (2026-10-10 Gazebo 마주 접근).
    수정: 재출발 시각부터 5 s 유예, 진짜 시동 꺼짐 고장은 다른 고장처럼 정지 미션으로 바꾼다."""
    import asyncio
    import dataclasses
    from ugv.tests.test_multi_vehicle import body, make_fleet

    async def go():
        f, clock = make_fleet(tmp_path)
        clock.advance(100.0)                                  # 시각 0 출발은 따로 (아래)
        A = f.vehicles["A-ugv1"]
        assert (await A.execute(body("D1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700))))["status"] == "STARTED"
        for _ in range(40):                                   # 20 s 주행 (출발 5 s 유예를 지난다)
            clock.advance(0.5)
            A.driver.step(0.5)
            await A.tick()
        drv = A.driver
        real_tel = drv.telemetry
        drv.kind = "px4"
        disarmed = {"on": False}
        drv.telemetry = lambda: dataclasses.replace(real_tel(), armed=False) if disarmed["on"] else real_tel()
        A.hold = {"s": A._s_now() + 30.0, "other": "X", "kind": "HEAD_ON", "since_sim": clock.now()}
        disarmed["on"] = True                                 # 대기 지점 정차 → 시동 꺼짐
        await A.tick()
        assert A.state == "DRIVING" and A.fault is None
        await A.clear_hold()                                  # 재출발 직후: 시동 상태가 아직 꺼짐으로 온다
        clock.advance(0.5)
        await A.tick()
        assert A.state == "DRIVING" and A.fault is None, A.fault
        clock.advance(6.0)                                    # 유예가 지나도 계속 꺼져 있으면 진짜 고장 → 정지 미션
        seq_before = A.seq
        await A.tick()
        assert A.state == "FAULT" and A.fault.startswith("VEHICLE_FAULT")
        assert A.seq != seq_before and A.last_stop.get("basis") is not None
    asyncio.run(go())


def test_disarm_detected_when_drive_started_at_sim_time_zero(tmp_path):
    import asyncio
    import dataclasses
    from ugv.tests.test_multi_vehicle import body, make_fleet

    async def go():
        f, clock = make_fleet(tmp_path)                       # 시계 0.0 에서 출발
        A = f.vehicles["A-ugv1"]
        assert (await A.execute(body("Z1", *FRAME.to_ll(A.access.x - 900, A.access.y - 700))))["status"] == "STARTED"
        assert A._drive_started_sim == 0.0
        real_tel = A.driver.telemetry
        A.driver.kind = "px4"
        A.driver.telemetry = lambda: dataclasses.replace(real_tel(), armed=False)
        clock.advance(6.0)
        await A.tick()
        assert A.state == "FAULT" and A.fault.startswith("VEHICLE_FAULT")
    asyncio.run(go())


def test_progress_index_keeps_earlier_pass_on_self_overlapping_route():
    """복귀 경로가 거점을 지나 한 바퀴 돌아 같은 도로를 되짚어 오면 계획 선형이 같은 곳을 두 번 지난다. 진행 위치가 나중 통과로
    건너뛰면 남은 경로를 다시 보낼 때 첫 지점이 차 뒤가 된다 (2026-10-10 Gazebo 4대 분리 경로: 도로 밖 10 m 회전)."""
    import math
    from ugv.vehicle import _dist_to_polyline, _progress_index
    out = [(float(x), 0.0) for x in range(0, 101, 5)]                  # 동쪽으로 100 m
    loop = [(100.0, 20.0), (100.0, 40.0), (80.0, 40.0), (60.0, 40.0), (60.0, 20.0)]
    back = [(float(x), -0.3) for x in range(60, -1, -5)]               # 같은 도로를 서쪽으로 (선형이 0.3 m 어긋남)
    pts = out + loop + back
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    near_i = 7                                                         # 나가는 통과 x 35~40
    x, y = 41.0, -0.2                                                  # 되짚는 선형이 조금 더 가깝다
    _, old = _dist_to_polyline(x, y, pts, near_i - 20, near_i + 200)
    assert old > len(out)                                              # 예전 방식: 나중 통과로 건너뜀
    d, i = _progress_index(x, y, pts, cum, near_i)
    assert i == 8 and abs(d - 0.2) < 1e-9                              # 나가는 통과의 x 40~45 구간
    # 평소(겹침 없음): 가장 가까운 구간 그대로, 한 칸 뒤로 물러서지 않음
    d, i = _progress_index(97.0, 0.4, pts, cum, 17)
    assert i == 19
    # 실제로 되짚는 구간에 들어선 뒤에는 나중 통과
    k = len(out) + len(loop) + 3                                       # back 의 x 45 지점 근처
    d, i = _progress_index(43.0, -0.3, pts, cum, k)
    assert i >= len(out) + len(loop)


def test_home_route_avoids_same_station_waiting_road(tmp_path):
    """거점 복귀 경로는 같은 거점 다른 차의 대기 도로를 지나지 않는다 — 지금 비어 있어도 그 차가 먼저 돌아와 서면 막힌다
    (2026-10-10 Gazebo 4대 분리 경로: A-ugv1 복귀가 대기 방향 도착을 위해 A-fire1 대기 도로를 도는 경로를 골라 그 앞에서 멈춤)."""
    import asyncio
    from ugv.tests.test_multi_vehicle import body, done, make_fleet, node_body, run

    async def go():
        f, clock = make_fleet(tmp_path)
        A, AF = f.vehicles["A-ugv1"], f.vehicles["A-fire1"]
        assert A.access.road_id != AF.access.road_id
        # Gazebo 와 같은 출동 → 도착 뒤 복귀 (A-fire1 은 출동 중이라 대기 도로가 비어 있음)
        await AF.execute(body("F0", *FRAME.to_ll(AF.access.x + 900, AF.access.y + 600)))
        assert AF.access.road_id in A.occupied_roads(home=True) and AF.access.road_id not in A.occupied_roads()
        await A.execute(body("U0", *FRAME.to_ll(A.access.x - 900, A.access.y - 700)))
        ok, _ = await run(f, clock, 600, until=done(f, "U0"))
        assert ok
        r = await A.execute(node_body("UH", A.access.road_id, A.access.s))
        assert r["status"] == "STARTED"
        assert AF.access.road_id not in A.plan.route.road_ids
        assert A.plan.report()["avoided_roads_occupied_by_vehicles"][AF.access.road_id].startswith("A-fire1")
    asyncio.run(go())


def test_link_loss_reach_covers_acceleration_and_response_delay():
    """링크가 끊긴 차의 점유 구간: PX4 는 끊긴 뒤 약 12 s(COM_DL_LOSS_T 10 s + 판정 지연) 계획 속도대로 가속하며 달린 뒤 시동을 꺼 미끄러진다.
    2026-10-10 Gazebo (link_loss_compare disarm_s): 끊길 때 8.2 m/s → 13.9 m/s 까지 가속, 끊긴 곳에서 158.8 m 에 정지.
    예전 '마지막 속도 × 10 s + 30 m' = 111.6 m 로 모자랐다."""
    import math
    from types import SimpleNamespace
    from ugv.traffic import link_loss_reach
    pts = [(float(x), 0.0) for x in range(0, 601, 5)]
    cum = [p[0] for p in pts]
    v = [min(13.89, 8.16 + 0.11 * c) for c in cum]                       # 약 50 m 동안 8.2 → 13.9 m/s
    sp = SimpleNamespace(pts=pts, cum=cum, v=v)
    r = link_loss_reach(sp, 0, 8.16)
    assert r >= 158.8 + 10.0 and 8.16 * 10 + 30 < 158.8
    # 목적지가 가까우면 계획대로 거기 선다 → 목적지까지 + 여유
    short = SimpleNamespace(pts=pts[:20], cum=cum[:20], v=v[:19] + [0.0])
    assert abs(link_loss_reach(short, 0, 8.16) - (cum[19] + 10.0)) < 1e-9
    assert math.isfinite(link_loss_reach(sp, 110, 0.0))


def test_waiting_slots_of_same_station_use_different_roads_across_profiles(tmp_path):
    """대기 위치 1 소방차와 대기 위치 2 UGV 가 같은 도로·같은 지점을 받던 결함 (차종마다 'k 번째 도로' 목록이 다름).
    앞 대기 위치들이 쓴 도로를 빼고 고른다. 기존 4대 배치는 그대로."""
    from ugv.tests.test_multi_vehicle import make_fleet
    cfg = FLEET_CFG + [{**FLEET_CFG[0], "resource_id": "A-ugv2", "station_slot": 2}]
    f, _ = make_fleet(tmp_path, [{**c, "driver": "sim"} for c in cfg])
    assert not f.unplaced
    roads = [f.vehicles[r].access.road_id for r in ("A-ugv1", "A-fire1", "A-ugv2")]
    assert len(set(roads)) == 3, roads
    f4, _ = make_fleet(tmp_path / "four", [{**c, "driver": "sim"} for c in FLEET_CFG])
    assert f4.vehicles["A-fire1"].access.road_id == "682500724" and f4.vehicles["B-fire1"].access.road_id == "683400570"


def test_rejection_reason_is_specific_and_recorded(tmp_path):
    """거절 사유를 응답에만 싣고 버리지 않는다: 사건 기록·ugv_rejections.jsonl 에 남긴다. 진행 방향 경로는 있었는데 최종 경로 검사가
    회전을 빼서 끊긴 경우는 NO_DIRECTED_ROUTE 가 아니라 NO_DRIVABLE_ROUTE_AFTER_CHECK (2026-10-10: B 거점 → A 지역 목표)."""
    import asyncio
    import json
    from ugv.tests.test_multi_vehicle import make_fleet

    async def go():
        f, _ = make_fleet(tmp_path, [{**FLEET_CFG[2], "driver": "sim"}])
        B = f.vehicles["B-ugv1"]
        ev = await B.evaluate({"task_id": "R1", "decision_id": "D-R1", "target": {"lat": 38.0281751, "lon": 128.1280267}})
        assert ev["verdict"] == "REJECT" and "NO_DRIVABLE_ROUTE_AFTER_CHECK" in ev["detail"]
        assert ev["approach"]["route_check_rejected"]
        assert B.events[-1]["event"] == "EVALUATE_REJECTED" and B.events[-1]["route_check_rejected"]
        rec = json.loads((tmp_path / "ugv_rejections.jsonl").read_text(encoding="utf-8").splitlines()[-1])
        assert rec["task_id"] == "R1" and rec["decision_id"] == "D-R1" and rec["route_check_rejected"]
        assert rec["counts"]["candidates_checked"] > 0
    asyncio.run(go())
