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
    assert log["reason"] == "NORMAL" and log["mission_seq"] == 7 and log["still_for_s"] >= 1.0


def test_park_log_runaway_reason_even_if_slow_at_cap():
    # 미션 끝 뒤 계속 0.5 m/s (저속 연속이 안 채워짐) → 4 s 상한으로 끔 = RUNAWAY
    log = _watch([0.5] * 12, after_s=2.0)
    assert log["reason"] == "RUNAWAY" and log["after_finished_s"] >= 4.0
