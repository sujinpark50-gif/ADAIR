# -*- coding: utf-8 -*-
"""PX4 미션 속도 의미 변환 — MAVSDK speed_m_s 는 '그 지점을 지난 뒤' 속도 (2026-10-09 실측)."""
import math

from ugv.drivers.px4 import ACCEPT_M, LEAD_BASE_M, LEAD_TIME_S, mavsdk_plan


def test_speeds_shift_one_item_earlier_and_lead_point_carries_first_segment():
    items = [(100.0, 0.0, 16.0), (140.0, 0.0, 8.0), (180.0, 0.0, 12.0), (200.0, 0.0, 3.0)]
    out = mavsdk_plan(items, (0.0, 0.0), 10.0)
    lead = LEAD_BASE_M + 10.0 * LEAD_TIME_S
    assert len(out) == 5
    assert math.isclose(out[0][0], lead) and out[0][2] == 16.0 and out[0][3]       # 첫 구간 속도는 선행 지점에
    assert [o[2] for o in out[1:]] == [8.0, 12.0, 3.0, 3.0]                        # 지점 k 에 k+1 구간 속도
    assert [o[3] for o in out] == [True, True, True, True, False]                  # 마지막만 정지


def test_no_lead_point_when_first_item_is_close():
    items = [(LEAD_BASE_M + ACCEPT_M - 1.0, 0.0, 5.0), (60.0, 0.0, 9.0)]
    out = mavsdk_plan(items, (0.0, 0.0), 0.0)
    assert [(o[0], o[2]) for o in out] == [(items[0][0], 9.0), (60.0, 9.0)]


def test_single_stop_point():
    out = mavsdk_plan([(0.0, 75.0, 16.0)], (0.0, 0.0), 16.0)
    assert out[-1] == (0.0, 75.0, 16.0, False) and out[0][2] == 16.0 and out[0][1] < 75.0


def test_first_mission_item_is_not_glued_to_start():
    """첫 미션 지점이 출발점에 붙어 출발 램프 속도(1 m/s)를 싣지 않는다 (2026-10-10 Gazebo G5b 기어감)."""
    import math
    from ugv import config
    from ugv.roads import Route, plan_speeds
    pts = [(0.0, 0.0), (0.3, 0.0), (40.0, 8.0), (120.0, 8.0)]
    sp = plan_speeds(Route(pts, [16.7] * 4, [], None, None, 0.0), 0.0)
    assert math.hypot(*sp.items[0][:2]) >= config.ITEM_FIRST_MIN_M - 1e-6
    assert sp.items[0][2] > 1.5


def test_park_due_also_parks_a_vehicle_that_keeps_driving_after_mission_end():
    """미션 끝 뒤 계속 달리면 RUNAWAY_PARK_S 뒤 시동을 끈다 (2026-10-11 Gazebo: 정지 경로 끝 2.9 m/s 로 FAULT 직진)."""
    from ugv.drivers.px4 import PARK_AFTER_S, RUNAWAY_PARK_S, park_due
    due, still, fin = park_due(2.9, 100.0, None, None)
    assert not due and still is None and fin == 100.0
    due, still, fin = park_due(2.9, 100.0 + RUNAWAY_PARK_S - 0.5, still, fin)
    assert not due
    assert park_due(2.9, 100.0 + RUNAWAY_PARK_S, still, fin)[0]
    # 정상 정차: PARK_AFTER_S 정지로 끈다
    due, still, fin = park_due(0.1, 200.0, None, None)
    assert not due and still == 200.0
    assert park_due(0.1, 200.0 + PARK_AFTER_S, still, fin)[0]


def test_park_due_is_not_postponed_by_speed_alternating_around_threshold():
    """0.29↔0.31 m/s 교대에도 미션 끝 상한으로 끈다 (2026-10-11 검토: 두 타이머가 서로 지워 60 s 동안 안 끔)."""
    from ugv.drivers.px4 import RUNAWAY_PARK_S, park_due
    still = fin = None
    t, due_at = 0.0, None
    while t < 60.0:
        due, still, fin = park_due(0.31 if int(t * 2) % 2 else 0.29, t, still, fin)
        if due:
            due_at = t
            break
        t += 0.5
    assert due_at is not None and due_at <= RUNAWAY_PARK_S + 1e-9


def test_px4_telemetry_without_velocity_is_not_fresh_standstill():
    """위치만 받고 속도를 못 받았으면 0 m/s 최신 정지가 아니라 나이 무한대 (2026-10-11 검토 재현)."""
    import math as _m
    import time as _t
    from types import SimpleNamespace
    from ugv.drivers.px4 import Px4Driver
    d = Px4Driver.__new__(Px4Driver)
    from ugv.geo import FRAME
    d.frame = FRAME
    d._pos = SimpleNamespace(latitude_deg=38.07, longitude_deg=128.17, absolute_altitude_m=300.0)
    d._pos_mono = _t.monotonic()
    d._vel, d._mono, d._heading, d._armed, d._mode = None, {"position": _t.monotonic()}, 0.0, False, "HOLD"
    tel = d.telemetry()
    assert _m.isinf(tel.age_s()) and tel.armed                 # 속도 없음 → 끊김과 같게, armed 미수신 → 시동 걸림으로
    d._vel = SimpleNamespace(north_m_s=0.0, east_m_s=0.0)
    d._mono.update(velocity=_t.monotonic() - 30.0, armed=_t.monotonic())
    assert d.telemetry().age_s() >= 29.0                       # 오래된 0 m/s 는 최신이 아니다
    d._mono["velocity"] = _t.monotonic()
    tel = d.telemetry()
    assert tel.age_s() < 1.0 and tel.speed_mps == 0.0 and not tel.armed


def test_first_item_ahead_skips_items_passed_during_upload():
    """업로드 동안 지나친 항목은 건너뛴다 (2026-10-11 Gazebo: 첫 항목들이 차 뒤 → 로버 되돌기·도로 이탈)."""
    from ugv.drivers.px4 import first_item_ahead
    sent = [(0.0, 23.0, 9.0, True), (0.0, 6.7, 9.0, True), (0.0, 10.6, 9.0, True), (0.0, 18.1, 9.0, True), (0.0, 42.9, 9.0, True),
            (0.0, 80.0, 9.0, True), (0.0, 120.0, 9.0, False)]
    sent = [(0.0, 5.0, 9.0, True)] + sent[1:]                       # 선행 지점(차 뒤) + 항목들, 북쪽(+y)으로 진행
    k = first_item_ahead(sent, (0.0, 17.0), 9.0)                     # 차는 y 17 — 항목 0~3 은 뒤·밑
    assert sent[k][1] > 17.0 + 3.0 and k == 4
    assert first_item_ahead(sent, (0.0, 4.0), 0.0) == 0              # 아직 첫 항목(선행 지점) 전 — 평소 출발 그대로
    assert first_item_ahead(sent, (200.0, 17.0), 9.0) == 0           # 미션 선에서 멀면 맞추지 않음


def test_first_item_ahead_ignores_route_coming_back_over_parked_position():
    """경로가 블록을 돌아 출발 위치를 반대로 다시 지나면, 그 되돌아오는 선분이 정차한 차와 겹친다.
    업로드 동안 갈 수 있었던 거리 상한 밖 선분은 짝짓지 않는다 (2026-10-11 Gazebo c23 A-ugv1 SK102: 정차 출발인데 뒤쪽 항목 44 부터 시작해
    제자리 선회·도로 이탈 — 원인 검증 Fable)."""
    from ugv.drivers.px4 import first_item_ahead
    sent = [(0.0, 5.0, 3, False), (0.0, 40.0, 5, True), (30.0, 40.0, 5, True), (30.0, 20.0, 5, True), (-30.0, -20.0, 5, True),
            (-30.0, -60.0, 5, True)]
    assert first_item_ahead(sent, (0.0, 0.0), 0.0) > 0                       # 상한 없음: 되돌아오는 선분을 짝지음 (예전 결함)
    assert first_item_ahead(sent, (0.0, 0.0), 0.0, max_along_m=10.0) == 0     # 정차 출발 상한 ≈ 10 m: 평소 출발
    # 원래 사례 (업로드 8.9 s, 9 m/s): 상한 ≈ 90 m 안에서는 그대로 지나친 항목을 건너뛴다
    sent2 = [(0.0, 5.0 + 10.0 * i, 9, True) for i in range(12)]
    assert first_item_ahead(sent2, (0.0, 17.0), 9.0, max_along_m=90.0) == first_item_ahead(sent2, (0.0, 17.0), 9.0) > 0


# ---- 시작 항목 선택 회귀 (2026-10-11 검토 지적 1~7): 고른 항목 번호·위치·차와 거리·차 앞 여부
def _along(sent, k):
    return sum(math.dist(sent[i][:2], sent[i + 1][:2]) for i in range(k))


def _check_ahead(sent, pos, k):
    """고른 항목이 차 앞(미션 선 위 차 위치보다 뒤 진행)이고 수용 반경 밖인가."""
    from ugv.drivers.px4 import ACCEPT_M
    best = min(range(len(sent) - 1), key=lambda i: _seg_dist(sent[i][:2], sent[i + 1][:2], pos)[0])
    d, t = _seg_dist(sent[best][:2], sent[best + 1][:2], pos)
    s_car = _along(sent, best) + t * math.dist(sent[best][:2], sent[best + 1][:2])
    return _along(sent, k) > s_car and math.dist(sent[k][:2], pos) > ACCEPT_M


def _seg_dist(a, b, p):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy), t


def _reach(v, dt):
    from ugv.drivers.px4 import ACCEPT_M, LEAD_BASE_M, START_AHEAD_ACCEL_MPS2
    return LEAD_BASE_M + ACCEPT_M + (v + START_AHEAD_ACCEL_MPS2 * dt) * dt + 2.0


STRAIGHT = [(0.0, 5.0 + 10.0 * i, 9, True) for i in range(15)]
# 블록을 돌아 출발 도로를 '같은 방향'으로 다시 지남 (y 축 위로 → 오른쪽 → 아래 → 왼쪽 → 다시 y 축 위로)
LOOP_SAME = [(0.0, 5.0, 3, False), (0.0, 60.0, 5, True), (60.0, 60.0, 5, True), (60.0, -60.0, 5, True), (0.0, -60.0, 5, True),
             (0.0, 60.0, 5, True), (0.0, 200.0, 5, True)]
LOOP_REV = [(0.0, 5.0, 3, False), (0.0, 40.0, 5, True), (30.0, 40.0, 5, True), (30.0, 20.0, 5, True), (-30.0, -20.0, 5, True),
            (-30.0, -60.0, 5, True)]
ARC = [(30.0 * math.sin(a), 30.0 * (1 - math.cos(a)), 6, True) for a in [0.15 * i + 0.15 for i in range(20)]]   # 출발 직후 곡선


import math  # noqa: E402


def test_start_item_cases_from_standstill():
    from ugv.drivers.px4 import first_item_ahead
    # 계획 시점 위치 here = (0, 0). 정차 방위는 이 함수가 쓰지 않는다 (방위 오차 영향 없음 — 구조상)
    for sent, name in ((STRAIGHT, "직선"), (LOOP_SAME, "같은 방향 재통과"), (LOOP_REV, "반대 방향 재통과"), (ARC, "곡선")):
        for off in ((0.0, 0.0), (0.2, 0.0), (-0.2, 0.0), (0.0, 0.2), (0.0, -0.2)):          # ±0.2 m 위치 오차
            for dt in (1.0, 3.0, 9.0):                                                         # 업로드 시간
                k = first_item_ahead(sent, off, 0.0, max_along_m=_reach(0.0, dt), here=(0.0, 0.0))
                assert k == 0, (name, off, dt, k)                                              # 정차 출발: 평소 출발 (첫 항목 = 선행 지점, 차 앞)
                assert sent[0][1] > 0 or sent[0][0] > 0


def test_start_item_cases_moving_upload_skips_only_passed_items():
    from ugv.drivers.px4 import first_item_ahead
    # 직선·한 번만 쓰는 도로: 9 m/s 로 8.9 s 업로드 동안 80 m 진행 → 지나친 항목만 건너뛰고 고른 항목은 차 앞
    pos = (0.0, 85.0)
    k = first_item_ahead(STRAIGHT, pos, 9.0, max_along_m=_reach(9.0, 8.9), here=(0.0, 0.0))
    assert k > 0 and _check_ahead(STRAIGHT, pos, k) and math.dist(STRAIGHT[k][:2], pos) < 25.0
    # 같은 방향 재통과 경로에서 첫 출현 구간을 달리는 중 (y 30): 두 번째 출현(누적 360 m 뒤)과 짝짓지 않는다
    pos = (0.1, 30.0)
    k = first_item_ahead(LOOP_SAME, pos, 6.0, max_along_m=_reach(6.0, 4.0), here=(0.0, 0.0))
    assert k == 1 and _check_ahead(LOOP_SAME, pos, k)
    # 곡선 위 (호 길이 약 20 m 진행): 고른 항목은 차 앞
    a = 0.75
    pos = (30.0 * math.sin(a) + 0.2, 30.0 * (1 - math.cos(a)))
    k = first_item_ahead(ARC, pos, 6.0, max_along_m=_reach(6.0, 3.0), here=(0.0, 0.0))
    assert k > 0 and _check_ahead(ARC, pos, k)


def test_start_item_sk102_real_mission():
    """c23 SK102 실제 미션 (Fable 재구성: 98 지점, 출발 도로를 반대로 다시 지남). 예전 44 → 0."""
    import json
    from pathlib import Path
    from ugv.drivers.px4 import first_item_ahead
    d = json.loads((Path(__file__).parent / "data" / "sk102_mission.json").read_text(encoding="utf-8"))
    sent = [tuple(x) for x in d["sent"]]
    here = tuple(d["here"])
    assert first_item_ahead(sent, here, 0.0) == 44                                   # 예전 동작 (거리 상한·출발점 없음)
    for dt in (1.1, 3.0, 9.0):
        for off in ((0.0, 0.0), (0.2, 0.0), (-0.2, 0.0), (0.0, 0.2), (0.0, -0.2)):
            pos = (here[0] + off[0], here[1] + off[1])
            assert first_item_ahead(sent, pos, 0.0, max_along_m=_reach(0.0, dt), here=here) == 0
            assert first_item_ahead(sent, pos, 0.0, here=here) == 0                   # 거리 상한 없이도 (경로 순서)


# --- 주행 중 지나침 감시 (passed_item) — 2026-10-12 Gazebo c33 A 교량 경로 항목 51 을 3.34 m 로 지나쳐 PX4 가 되돌아 돌다 이탈
from ugv.drivers.px4 import passed_item  # noqa: E402

C33 = [(4561.55, 6044.95), (4562.81, 6044.86), (4564.04, 6044.62), (4565.25, 6044.24), (4566.38, 6043.69), (4567.45, 6043.03),
       (4568.39, 6042.2), (4569.24, 6041.3), (4571.32, 6038.1), (4580.42, 6025.21), (4601.03, 5992.26), (4610.34, 5979.08)]
C33_SENT = [(x, y, 4.0, True) for x, y in C33]      # 실제 미션 항목 44~55 (여기 번호 7 = 실제 51)


def test_pass_none_while_tracking_normally():
    # 계획선 위·옆 1.5 m 안을 따라가는 동안은 건너뛰지 않는다 (현재 항목은 차 앞 3 m 근처)
    for cur, pos in ((3, (4562.8, 6045.1)), (5, (4564.7, 6045.4)), (7, (4567.6, 6043.4)), (8, (4569.6, 6040.0))):
        assert passed_item(C33_SENT, None, cur, pos) is None


def test_pass_skips_after_missing_item_outside_curve():
    # 기록 위치 (sim 107.0 s): 현재 항목 51 에서 5 m, 경로상 이미 지남 → 차 앞 항목으로
    k = passed_item(C33_SENT, None, 7, (4574.3, 6042.1))
    assert k is not None and k >= 8
    assert math.dist((4574.3, 6042.1), C33[k]) > 0


def test_pass_none_inside_accept_radius_and_last_item():
    assert passed_item(C33_SENT, None, 7, (4570.0, 6043.5)) is None                 # 2.3 m — PX4 가 곧 도착 처리
    assert passed_item(C33_SENT, None, len(C33_SENT) - 1, (4620.0, 5970.0)) is None   # 마지막(정지점)은 건너뛰지 않음


def test_pass_straight_offsets():
    sent = [(float(x), 0.0, 5.0, True) for x in range(0, 60, 2)]
    assert passed_item(sent, None, 10, (21.0, 2.5)) is None       # 옆 2.5 m 로 지나침 → 도착 반경 안이라 PX4 가 처리
    k = passed_item(sent, None, 10, (22.0, 4.0))                  # 옆 4 m 로 지나침 → 진행 위치 + 3 m 넘는 첫 항목
    assert k == 13
    assert passed_item(sent, None, 10, (22.0, 4.0), 5.0) == 15    # 5 m/s 면 1 s 만큼 더 앞
    # 늦은 진행 보고(cur 8)를 받아도 차 앞 + 3 m 밖 항목이라 PX4 실제 현재 항목(차 앞 3 m 안쪽은 이미 넘김)보다 뒤가 아니다
    assert passed_item(sent, None, 8, (22.0, 4.0)) == 13
    assert passed_item(sent, None, 10, (22.0, 20.0)) is None      # 미션 선에서 15 m 넘게 → 서버 이탈 감시 몫
    assert passed_item(sent, None, 10, (17.0, 4.0)) is None       # 아직 항목 앞
