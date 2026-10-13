"""E2 기준 ③ 관측 시험 훅 (Fable 미해결 2.2) — UGV_TEST_HOOKS=1 + UGV_PX4_PARK_AFTER_S 일 때만, PX4 끝 지점 자체 정지 차만, [1.0, 3.0] 로 자름."""
from ugv.drivers.px4 import PARK_AFTER_S, RUNAWAY_PARK_S, park_after_policy, park_due

ACC3 = {"NAV_ACC_RAD": 3.0, "RA_ACC_RAD_MAX": 3.0}
RA_OFF = {"NAV_ACC_RAD": 3.0, "RA_ACC_RAD_MAX": -1.0}


def test_default_without_hooks(monkeypatch):
    monkeypatch.delenv("UGV_TEST_HOOKS", raising=False)
    monkeypatch.setenv("UGV_PX4_PARK_AFTER_S", "2.0")
    p = park_after_policy(ACC3)
    assert p["after_s"] == PARK_AFTER_S == 1.0 and p["test_hook"] is False


def test_hook_applies_and_clamps(monkeypatch):
    monkeypatch.setenv("UGV_TEST_HOOKS", "1")
    monkeypatch.setenv("UGV_PX4_PARK_AFTER_S", "2.0")
    assert park_after_policy(ACC3)["after_s"] == 2.0 and park_after_policy(ACC3)["test_hook"] is True
    monkeypatch.setenv("UGV_PX4_PARK_AFTER_S", "9")
    assert park_after_policy(ACC3)["after_s"] == RUNAWAY_PARK_S - 1.0
    monkeypatch.setenv("UGV_PX4_PARK_AFTER_S", "0.2")
    assert park_after_policy(ACC3)["after_s"] == PARK_AFTER_S


def test_hook_ignored_for_vehicle_without_end_stop_or_unknown(monkeypatch):
    monkeypatch.setenv("UGV_TEST_HOOKS", "1")
    monkeypatch.setenv("UGV_PX4_PARK_AFTER_S", "2.0")
    for info in (RA_OFF, {"basis": "파라미터 읽기 실패"}, None):
        p = park_after_policy(info)
        assert p["after_s"] == PARK_AFTER_S and p["test_hook"] is False


def test_park_due_uses_policy_but_runaway_cap_unchanged():
    due, still, fin = park_due(0.1, 100.0, None, None, 2.0)
    assert not park_due(0.1, 101.5, still, fin, 2.0)[0]          # 기본(1.0)이면 끔, 훅 2.0 이면 아직
    assert park_due(0.1, 101.5, still, fin)[0]
    assert park_due(0.1, 102.0, still, fin, 2.0)[0]
    d2, s2, f2 = park_due(2.9, 200.0, None, None, 3.0)              # RUNAWAY 4 s 상한은 훅과 무관
    assert park_due(2.9, 200.0 + RUNAWAY_PARK_S, s2, f2, 3.0)[0]


def _watch(speeds, after_s=1.0):
    """실제 _park_watch 를 가짜 텔레메트리로 돌려 시동 끔 사유 기록(park_log)을 본다. speeds: 0.5 s 폴링마다 속도."""
    import asyncio
    import types
    from ugv.drivers.base import Progress
    from ugv.drivers.px4 import Px4Driver
    d = Px4Driver("udpin://0.0.0.0:1", clock=None)
    d._armed, d._prog = True, Progress(5, 5, True, 7)
    d.park_policy = {"after_s": after_s, "test_hook": after_s != 1.0}
    it = iter(speeds)
    cur = {"v": next(it)}
    d.telemetry = lambda: types.SimpleNamespace(speed_mps=cur["v"], age_s=lambda: 0.0)

    class Act:
        async def disarm(self):
            d._armed = False
    d._act = Act()

    async def run():
        t = asyncio.create_task(d._park_watch())
        for v in speeds:
            cur["v"] = v
            await asyncio.sleep(0.5)
            if d.parked_seq == 7:
                break
        await asyncio.sleep(0.6)
        t.cancel()
    asyncio.run(run())
    return d.park_log


def test_park_log_normal_reason():
    log = _watch([0.1] * 8, after_s=1.0)
    assert log["reason"] == "NORMAL_STILL" and log["mission_seq"] == 7 and log["cond_still"] and not log["cond_cap"]
    assert log["decide_mono"] - log["still_since_mono"] >= 1.0 and log["disarm_call"] == "OK"
    assert log["disarmed_seen_mono"] is not None and log["disarmed_seen_mono"] >= log["disarm_req_mono"]


def test_park_log_runaway_reason_even_if_slow_at_cap():
    # 미션 끝 뒤 계속 0.5 m/s (저속 연속이 안 채워짐) → 4 s 상한으로 끔 = RUNAWAY
    log = _watch([0.5] * 12, after_s=2.0)
    assert log["reason"] == "RUNAWAY_CAP" and log["cond_cap"] and not log["cond_still"]
    assert log["decide_mono"] - log["finished_first_mono"] >= 4.0


def test_park_decide_cap_and_still_both_true_reports_cap():
    from ugv.drivers.px4 import park_decide
    d = park_decide(0.1, 104.0, 101.0, 100.0, 2.0)          # 저속 3 s (≥ 2.0) 이고 미션 끝 뒤 4 s 상한도 참
    assert d["due"] and d["reason"] == "RUNAWAY_CAP" and d["cond_cap"] and d["cond_still"]


def test_park_decide_still_reset_and_stale_speed():
    from ugv.drivers.px4 import park_decide
    d = park_decide(0.5, 101.0, 100.5, 100.0, 2.0)          # 저속 타이머가 있었는데 0.5 m/s → 지움
    assert not d["due"] and d["still_reset"] and d["still_since"] is None
    d = park_decide(1.0, 102.0, None, 100.0, 2.0)            # 오래된 속도는 호출 쪽이 1.0 으로 넣는다 → 저속 아님
    assert not d["cond_still"] and d["reason"] is None


def test_park_due_unchanged_by_refactor():
    from ugv.drivers.px4 import park_due
    assert park_due(0.1, 100.0, None, None) == (False, 100.0, 100.0)
    assert park_due(0.1, 101.0, 100.0, 100.0)[0] is True
    assert park_due(2.0, 104.0, None, 100.0)[0] is True


def test_park_lock_wait_then_mission_changed_is_recorded_abandoned():
    """판단(상한) 뒤 명령 잠금 대기 중 새 미션이 오면 끄지 않고 ABANDONED 로 기록 — 판단 사유는 판단 순간 값 그대로."""
    import asyncio
    import types
    from ugv.drivers.base import Progress
    from ugv.drivers.px4 import Px4Driver
    d = Px4Driver("udpin://0.0.0.0:1", clock=None)
    d._armed, d._prog = True, Progress(5, 5, True, 3)
    d.park_policy = {"after_s": 2.0}
    d.telemetry = lambda: types.SimpleNamespace(speed_mps=0.6, age_s=lambda: 0.0)
    calls = []

    class Act:
        async def disarm(self):
            calls.append(1)
    d._act = Act()

    async def run():
        await d._cmd.acquire()                       # 다른 명령이 잠금을 쥐고 있음
        t = asyncio.create_task(d._park_watch())
        await asyncio.sleep(5.2)                     # 상한(4 s) 판단이 나고 잠금 대기
        d._prog = Progress(0, 9, False, 4)           # 새 미션
        d._cmd.release()
        await asyncio.sleep(0.3)
        t.cancel()
    asyncio.run(run())
    assert not calls and d.park_log["outcome"] == "ABANDONED_MISSION_CHANGED" and d.park_log["reason"] == "RUNAWAY_CAP"
