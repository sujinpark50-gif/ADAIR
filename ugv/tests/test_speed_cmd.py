"""UGV_SPEED_TERRAIN_CMD 구간 속도 명령 (vehicle._speed_cmd_tick) — 진입 전 적용·변화 때만 전송·양보/정지/재계획 중 금지·
실패 재시도와 backoff·성공 뒤 중복 생략·정지 요청과의 잠금 경쟁·시간 초과 (Codex 답변 1423·1436)."""
import asyncio
from types import SimpleNamespace

from ugv import vehicle as V
from ugv.vehicle import Vehicle


class Drv:
    def __init__(self, results=None, delay=0.0):
        self.sent, self.results, self.delay = [], list(results or []), delay

    async def set_speed_cap(self, v):
        self.sent.append(v)
        if self.delay:
            await asyncio.sleep(self.delay)
        return self.results.pop(0) if self.results else True


def stub(drv=None):
    cum = [float(k * 5) for k in range(41)]                      # 0..200 m, 5 m 간격
    v = [12.0 if not (100 <= c <= 130) else 6.0 for c in cum]    # 100~130 m 구간 계획 6 m/s
    st = SimpleNamespace(speed=SimpleNamespace(cum=cum, v=v), hold=None, _stop_sup=None, state="DRIVING", seq=3,
                         lock=asyncio.Lock(), driver=drv or Drv(), events=[], last_error=None, _spd_cmd=None, _spd_ok=None, _spd_fails=0)
    st._event = lambda name, **kw: st.events.append({"event": name, **kw})
    return st


def tel(v):
    return SimpleNamespace(speed_mps=v)


def tick(st, now, v, i):
    return Vehicle._speed_cmd_tick(st, now, tel(v), i)


def run(st, now, v, i):
    asyncio.run(tick(st, now, v, i))


def test_applies_before_segment_and_only_on_change():
    st = stub()
    run(st, 0.0, 12.0, 0)                 # s 0: 앞 12·3+여유 안에 낮은 구간 없음 → 12
    assert st.driver.sent == [12.0]
    run(st, 0.5, 12.0, 2)                 # 변화 없음 → 안 보냄
    run(st, 5.0, 12.0, 3)                 # 시간이 지나도 성공한 같은 목표는 생략
    assert st.driver.sent == [12.0]
    run(st, 6.0, 12.0, 13)                # s 65: 앞 ~36 m+ 안에 100 m 구간 → 진입 전 6
    assert st.driver.sent[-1] == 6.0
    assert st.events[-1]["event"] == "SPEED_CMD" and st.events[-1]["mission_seq"] == 3 and st.events[-1]["ack"] is True


def test_not_sent_during_hold_stop_or_replan():
    for k, val in (("hold", {"s": 1}), ("_stop_sup", {"basis": "x"}), ("state", "STOPPING")):
        st = stub()
        setattr(st, k, val)
        run(st, 0.0, 12.0, 13)
        assert st.driver.sent == [], k


def test_failure_retries_same_target_with_backoff_then_dedups():
    st = stub(Drv(results=[False, False, True]))
    run(st, 0.0, 12.0, 13)                # 실패 1
    assert st.driver.sent == [6.0] and st.events[-1]["ack"] is False and "감독 유지" in st.last_error
    run(st, 0.5, 12.0, 13)                # backoff 1 s 미달 → 안 보냄
    assert st.driver.sent == [6.0]
    run(st, 1.0, 12.0, 13)                # 같은 목표 재시도 (실패 2)
    run(st, 2.5, 12.0, 13)                # backoff 2 s 미달
    assert st.driver.sent == [6.0, 6.0]
    run(st, 3.0, 12.0, 13)                # 재시도 성공
    run(st, 10.0, 12.0, 13)               # 성공 뒤 같은 목표 생략
    assert st.driver.sent == [6.0, 6.0, 6.0] and st._spd_fails == 0 and st._spd_ok["v"] == 6.0


def test_gives_up_after_max_fails_but_keeps_supervision():
    st = stub(Drv(results=[False] * 10))
    t = 0.0
    for _ in range(V.SPEED_CMD_MAX_FAILS + 3):
        run(st, t, 12.0, 13)
        t += V.SPEED_CMD_BACKOFF_MAX_S + 0.1
    assert len(st.driver.sent) == V.SPEED_CMD_MAX_FAILS
    assert any(e["event"] == "SPEED_CMD_GIVEUP" for e in st.events)


def test_stop_request_waits_for_pending_cmd_then_wins():
    async def scenario():
        st = stub(Drv(delay=0.2))
        waited = {}

        async def stop_req():
            await asyncio.sleep(0.05)                   # 명령 ACK 대기 중 정지 요청
            t0 = asyncio.get_running_loop().time()
            async with st.lock:
                waited["s"] = asyncio.get_running_loop().time() - t0
                st._stop_sup, st.seq, st.state = {"basis": "STOP"}, 4, "STOPPING"
        await asyncio.gather(tick(st, 0.0, 12.0, 13), stop_req())
        await tick(st, 5.0, 12.0, 0)                    # 정지 뒤: 이전 주행 목표 재전송 없음
        return st, waited["s"]
    st, w = asyncio.run(scenario())
    assert st.driver.sent == [6.0]
    assert w < 0.3                                      # 정지 대기 = 남은 ACK 대기 (≤ SPEED_CMD_TIMEOUT_S)
    assert st.events[-1]["lock_held_s"] >= 0.15


def test_new_mission_during_ack_wait_not_overwritten():
    async def scenario():
        st = stub(Drv(delay=0.2))

        async def new_mission():
            await asyncio.sleep(0.05)
            async with st.lock:                         # _drive 와 같은 순서: 잠금 뒤 seq·명령 상태 초기화
                st.seq, st._spd_cmd, st._spd_ok, st._spd_fails = 5, None, None, 0
        await asyncio.gather(tick(st, 0.0, 12.0, 13), new_mission())
        return st
    st = asyncio.run(scenario())
    assert st.seq == 5 and st._spd_cmd is None and st._spd_ok is None   # 이전 미션 결과가 새 미션 상태를 덮지 않음


def test_timeout_marks_unknown_and_retries():
    old = V.SPEED_CMD_TIMEOUT_S
    V.SPEED_CMD_TIMEOUT_S = 0.05
    try:
        st = stub(Drv(delay=0.2))
        run(st, 0.0, 12.0, 13)
        assert st.events[-1]["ack"] is None and st._spd_fails == 1
        st.driver.delay = 0.0
        run(st, 1.0, 12.0, 13)
        assert st.events[-1]["ack"] is True and st._spd_fails == 0
    finally:
        V.SPEED_CMD_TIMEOUT_S = old


def test_stop_cap_first_sent_before_upload(monkeypatch):
    """UGV_STOP_CAP_FIRST: 정지 경로 업로드(follow) 전에 min(지금 속도, 첫 구간 계획 속도) 상한을 보낸다 (Fable F1)."""
    import inspect
    src = inspect.getsource(Vehicle._safe_stop)
    i_cap, i_follow = src.index("STOP_CAP_FIRST and tel is not None"), src.index("self.driver.follow(items)")
    assert i_cap < i_follow
    assert "min(tel.speed_mps, float(items[0][2]))" in src
