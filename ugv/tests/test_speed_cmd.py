"""UGV_SPEED_TERRAIN_CMD 구간 속도 명령 (vehicle._speed_cmd_tick) — 진입 전 적용·변화 때만 전송·양보/정지/재계획 중 금지·실패 시 감독 유지."""
import asyncio
from types import SimpleNamespace

from ugv.vehicle import Vehicle


class Drv:
    def __init__(self, ok=True):
        self.sent, self.ok = [], ok

    async def set_speed_cap(self, v):
        self.sent.append(v)
        return self.ok


def stub(ok=True):
    cum = [float(k * 5) for k in range(41)]                      # 0..200 m, 5 m 간격
    v = [12.0 if not (100 <= c <= 130) else 6.0 for c in cum]    # 100~130 m 구간 계획 6 m/s
    st = SimpleNamespace(speed=SimpleNamespace(cum=cum, v=v), hold=None, _stop_sup=None, state="DRIVING", seq=3,
                         lock=asyncio.Lock(), driver=Drv(ok), events=[], last_error=None, _spd_cmd=None)
    st._event = lambda name, **kw: st.events.append({"event": name, **kw})
    return st


def tel(v):
    return SimpleNamespace(speed_mps=v)


def run(st, now, v, i):
    asyncio.run(Vehicle._speed_cmd_tick(st, now, tel(v), i))


def test_applies_before_segment_and_only_on_change():
    st = stub()
    run(st, 0.0, 12.0, 0)                 # s 0: 앞 12·3+여유 안에 낮은 구간 없음 → 12
    assert st.driver.sent == [12.0]
    run(st, 0.5, 12.0, 2)                 # 변화 없음·간격 미달 → 안 보냄
    assert st.driver.sent == [12.0]
    run(st, 2.0, 12.0, 13)                # s 65: 앞 ~36 m+ 안에 100 m 구간 → 진입 전 6
    assert st.driver.sent[-1] == 6.0
    assert st.events[-1]["event"] == "SPEED_CMD" and st.events[-1]["mission_seq"] == 3 and st.events[-1]["ack"] is True


def test_not_sent_during_hold_stop_or_replan():
    for k, val in (("hold", {"s": 1}), ("_stop_sup", {"basis": "x"}), ("state", "STOPPING")):
        st = stub()
        setattr(st, k, val)
        run(st, 0.0, 12.0, 13)
        assert st.driver.sent == [], k


def test_failure_keeps_supervision_and_records():
    st = stub(ok=False)
    run(st, 0.0, 12.0, 13)
    assert st.driver.sent == [6.0] and st.events[-1]["ack"] is False and "감독 유지" in st.last_error
