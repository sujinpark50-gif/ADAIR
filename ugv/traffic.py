# -*- coding: utf-8 -*-
"""ugv/traffic.py — 차량 간 충돌 방지 (최소 구현, 2026-10-10). 한 UGV 서버 안의 차량들(Fleet)을 함께 본다.

방식: 주기마다(TRAFFIC_PERIOD_S) 각 차량의 앞으로 TRAFFIC_HORIZON_S 동안 위치를 속도 계획(SpeedPlan)으로 예측해
시공간에서 서로 가까워지는 곳(충돌 후보)을 찾고, 한 차량을 그 앞에서 멈춰 기다리게(hold) 한다. 길이 비면 다시 출발.
- 간격: 두 차체 길이 절반의 합 + TRAFFIC_GAP_M. 기다리는 차는 정지거리(차량 감속) + 명령 교체 지연 동안 거리 안에서는
  멈출 수 없으므로, 그보다 가까운 충돌은 상대가 양보하게 바꾼다. 둘 다 못 멈추면 둘 다 도로를 따라 감속 정지 + 보고.
- 같은 방향 앞뒤: 뒤차가 기다린다 (추월 없음).
- 교차(교차로): 우선순위가 낮은 차가 충돌 지점 앞에서 기다린다.
- 반대 방향 같은 도로: 차량은 도로 중심선을 따라가고 차로 위치를 쓰지 않는다 → **교행하지 않는다**. 두 차가 같은 도로 구간을
  반대로 지나는 시간이 겹치면 우선순위가 낮은 차가 그 구간에 들어가기 전에 기다린다 (좁은 도로·넓은 도로 모두 — 보수적).
- 정지·고장·대기 차량은 점유 차량(정지 장애물)으로 본다. 그 차가 비킬 때까지 기다리고 TRAFFIC_BLOCK_REPORT_S 넘으면 보고.
- 우선순위: ① 충돌 지점 앞에서 멈출 수 없는 차(이미 진입) ② 소방차(FIRE_ENGINE) ③ 충돌 지점에 먼저 도착하는 차 ④ id 순.
- 서로 기다리는 고리(교착)는 감지해 보고한다 (자동 해소하지 않는다 — 기다림을 유지하고 운영자 개입 요청).
한계: 표본 예측(근사)이다. 속도 계획대로 달린다고 가정하며 PX4 추종 오차·지연 변동은 간격 여유로만 흡수한다.
충돌 보장이 아니다. 서버가 다른 차량(다른 서버의 차, 사람 운전 차량)은 모른다.
"""

import math
import time
from typing import Dict, List, Optional

from . import config

TRAFFIC_PERIOD_S = 0.5           # 판단 주기 (시뮬레이션 초)
TRAFFIC_HORIZON_S = 45.0         # 예측 시간
TRAFFIC_HORIZON_M = 700.0        # 예측 거리 상한
TRAFFIC_SAMPLE_M = 4.0           # 예측 표본 간격
TRAFFIC_GAP_M = 4.0              # 차체 사이 최소 간격 (두 차체 길이 절반 합에 더한다)
TRAFFIC_TIME_SLACK_S = 4.0       # 같은 곳을 이 시간 차 안에 지나면 충돌 후보 (명령 지연은 따로 더한다)
TRAFFIC_FOLLOW_HEADWAY_S = 2.0  # 같은 방향 앞뒤: 같은 곳을 이 시간(+ 명령 지연) 안에 지나면 뒤차가 기다린다
TRAFFIC_LANE_M = 3.5             # 반대 방향 두 경로 중심선이 이보다 가까우면 같은 도로 구간 (교행 안 함)
TRAFFIC_HOLD_MARGIN_M = 3.0      # 기다리는 지점 = 충돌 표본보다 이만큼 앞
TRAFFIC_RELEASE_TICKS = 2        # 연속 이만큼 충돌이 없어야 다시 출발 (흔들림 방지)
TRAFFIC_BLOCK_REPORT_S = 60.0    # 정지 차량 때문에 이보다 오래 기다리면 보고
RANK = {"FIRE_ENGINE": 0, "UGV": 1}


class _Pred:
    """차량 하나의 예측: 표본 [(x, y, t, s, hx, hy)] (t: 지금부터 초, s: 자기 계획 선형 위 위치, (hx, hy): 진행 방향)."""

    def __init__(self, v, samples, static, s_now, v_now, committed_m):
        self.v, self.samples, self.static, self.s_now, self.v_now = v, samples, static, s_now, v_now
        self.committed_m = committed_m           # 지금부터 이 거리 안에서는 멈출 수 없다 (정지거리 + 지연)


def _tcum(sp):
    t = [0.0]
    for i in range(1, len(sp.v)):
        t.append(t[-1] + (sp.cum[i] - sp.cum[i - 1]) / max((sp.v[i] + sp.v[i - 1]) / 2, 0.5))
    return t


class TrafficManager:
    def __init__(self, vehicles: Dict[str, object], clock):
        self.vehicles, self.clock = vehicles, clock
        self._tc = {}                            # id(speed plan) → 누적 시간
        self._clear_ticks: Dict[str, int] = {}
        self.waits: Dict[str, dict] = {}         # 기다리는 차 → {other, kind, since_sim, ...}
        self.deadlocks: List[List[str]] = []
        self.unavoidable: List[dict] = []
        self.last_tick_wall_ms = None
        self.ticks = 0

    # ------------------------------------------------------------------ 예측
    def _predict(self, v, own_view: bool = False) -> Optional[_Pred]:
        """own_view: 자기 충돌 판단용 — 대기 중이어도 대기 지점 너머 원래 경로까지 본다 (대기를 풀어도 되는지).
        아니면 남이 보는 예측 — 대기 지점에서 멈춰 머문다."""
        tel = v.driver.telemetry()
        if tel is None:
            return None
        lat_s = v.latency_s()
        vel = max(tel.speed_mps, 0.0)
        committed = v.profile.stop_distance(vel) + vel * lat_s if vel > 0.3 else 0.0
        hd = math.radians(tel.heading_deg)
        hx, hy = math.sin(hd), math.cos(hd)
        moving_plan = v.state in ("DRIVING", "EVADING") and v.speed is not None
        if not moving_plan:
            # 정지·고장·대기·정지 중: 지금 자리(정지 중이면 앞쪽 정지 경로까지)를 계속 차지한다
            pts = [(tel.x, tel.y, 0.0, 0.0, hx, hy)]
            if vel > 0.3:
                d = v.profile.stop_distance(vel)
                n = max(1, int(d / TRAFFIC_SAMPLE_M))
                pts += [(tel.x + hx * d * k / n, tel.y + hy * d * k / n, 0.0, d * k / n, hx, hy) for k in range(1, n + 1)]
            return _Pred(v, pts, True, 0.0, vel, committed)
        sp = v.speed
        key = id(sp)
        if key not in self._tc:
            self._tc = {key: _tcum(sp), **{k: x for k, x in list(self._tc.items())[-8:]}}
        tc = self._tc[key]
        i0 = max(0, min(v._near_i, len(sp.pts) - 1))
        s_now = sp.cum[i0]
        hold_s = v.hold["s"] if getattr(v, "hold", None) and not own_view else None
        out = [(tel.x, tel.y, 0.0, s_now, hx, hy)]
        last_s = s_now
        # 지연: 명령을 바꿔도 lat_s 동안은 지금 계획대로 간다 → 예측 시간에 지연을 넣지 않고 기다림 판단(committed)에 넣는다
        for k in range(i0 + 1, len(sp.pts)):
            s = sp.cum[k]
            if hold_s is not None and s > hold_s:
                break
            t = tc[k] - tc[i0]
            if t > TRAFFIC_HORIZON_S or s - s_now > TRAFFIC_HORIZON_M:
                break
            if s - last_s < TRAFFIC_SAMPLE_M and k != len(sp.pts) - 1:
                continue
            a, b = sp.pts[k - 1], sp.pts[k]
            d = math.dist(a, b) or 1.0
            out.append((b[0], b[1], t, s, (b[0] - a[0]) / d, (b[1] - a[1]) / d))
            last_s = s
        static_tail = hold_s is not None or (len(sp.pts) and last_s >= sp.cum[-1] - 1e-6)
        return _Pred(v, out, False, s_now, vel, committed) if not static_tail else \
            _Pred(v, out, False, s_now, vel, committed)._with_tail()

    # ------------------------------------------------------------------ 충돌 찾기
    @staticmethod
    def _first_conflict(A: _Pred, B: _Pred):
        """A 경로를 따라 처음으로 B 와 부딪칠 수 있는 곳. (kind, A 표본, B 표본) 또는 None.
        kind: FOLLOW(같은 방향) / CROSS / HEAD_ON(반대 방향 같은 구간) / STATIC(B 가 멈춰 있음)."""
        la, lb = A.v.profile.body_length_m, B.v.profile.body_length_m
        d_safe = (la + lb) / 2 + TRAFFIC_GAP_M
        slack = TRAFFIC_TIME_SLACK_S + max(A.v.latency_s(), B.v.latency_s())
        bs = B.samples
        b_end_t = bs[-1][2] if bs else 0.0
        for a in A.samples:
            ax, ay, at, as_, ahx, ahy = a
            for b in bs:
                bx, by, bt, bs_, bhx, bhy = b
                dist = math.hypot(ax - bx, ay - by)
                dot = ahx * bhx + ahy * bhy
                if dist < TRAFFIC_LANE_M and dot < -0.5:
                    # 반대 방향 같은 도로 구간. A 가 이 지점에 오기 전에 B 가 이 지점을 지나 구간을 벗어났으면(bt < at) 그
                    # 뒤 지점들도 마찬가지라 만나지 않는다. B 가 이 지점에 A 와 같은 때나 그 뒤에 오면 구간 안에서 마주친다
                    if B.static or bt >= at - slack:
                        return ("HEAD_ON" if not B.static else "STATIC", a, b)
                if dist < d_safe:
                    tb = bt if not (B.tail and b is bs[-1]) else None
                    if B.static:
                        return ("STATIC", a, b)
                    if tb is None:                      # B 가 그 자리(목적지·대기 지점)에 머문다
                        if at >= bt - slack:
                            return ("STATIC" if B.v_now < 0.3 else "FOLLOW", a, b)
                        continue
                    if dot > 0.5:
                        if abs(at - tb) < TRAFFIC_FOLLOW_HEADWAY_S + max(A.v.latency_s(), B.v.latency_s()):
                            return ("FOLLOW", a, b)
                    elif abs(at - tb) < slack:
                        return ("CROSS", a, b)
        return None

    def _priority(self, P: _Pred, at_t: float):
        return (RANK.get(P.v.resource_type, 9), at_t, P.v.resource_id)

    # ------------------------------------------------------------------ 판단·적용
    async def tick(self):
        w0 = time.perf_counter()
        self.ticks += 1
        preds = {rid: self._predict(v) for rid, v in self.vehicles.items()}
        preds = {r: p for r, p in preds.items() if p is not None}
        own = {rid: self._predict(self.vehicles[rid], own_view=True) for rid in preds}
        holds: Dict[str, dict] = {}
        unavoidable = []
        movers = [r for r, p in preds.items() if not p.static]
        for i, ra in enumerate(movers):
            for rb in list(preds):
                if rb == ra:
                    continue
                A, B = own[ra], preds[rb]
                c = self._first_conflict(A, B)
                if c is None:
                    continue
                kind, a, b = c
                prefer = ra if (self.waits.get(ra) or {}).get("other") == rb else                     rb if (self.waits.get(rb) or {}).get("other") == ra else None
                yielder, other, ys, why = self._decide(kind, A, B, a, b, prefer)
                if yielder is None:
                    unavoidable.append({"vehicles": [ra, rb], "kind": kind, "at": [round(a[0], 1), round(a[1], 1)]})
                    continue
                cur = holds.get(yielder.v.resource_id)
                if cur is None or ys < cur["s"]:
                    holds[yielder.v.resource_id] = {"s": ys, "other": other.v.resource_id, "kind": kind, "why": why,
                                                    "at": [round(a[0], 1), round(a[1], 1)] if yielder is A
                                                    else [round(b[0], 1), round(b[1], 1)]}
        # 교착: 기다림 고리
        edges = {r: h["other"] for r, h in holds.items()}
        cycles = []
        for r in edges:
            seen, cur = [], r
            while cur in edges and cur not in seen:
                seen.append(cur)
                cur = edges[cur]
            if cur == r and sorted(seen) not in cycles:
                cycles.append(sorted(seen))
        now = self.clock.now()
        for cyc in cycles:
            if cyc not in self.deadlocks:
                for r in cyc:
                    self.vehicles[r]._event("TRAFFIC_DEADLOCK", members=cyc, operator_action_required=True,
                                            priorities={x: list(map(str, self._priority(preds[x], 0.0))) for x in cyc})
        self.deadlocks = cycles
        for u in unavoidable:
            if u not in self.unavoidable:
                for r in u["vehicles"]:
                    v = self.vehicles[r]
                    v._event("TRAFFIC_CONFLICT_UNAVOIDABLE", **u, operator_action_required=True)
                    if v.state in ("DRIVING", "EVADING"):
                        await v.traffic_emergency_stop(u)
        self.unavoidable = unavoidable
        # 적용
        for rid, v in self.vehicles.items():
            h = holds.get(rid)
            if h is not None:
                self._clear_ticks[rid] = 0
                prev = self.waits.get(rid)
                since = prev["since_sim"] if prev and prev["other"] == h["other"] else now
                self.waits[rid] = {**h, "since_sim": since}
                if (h["kind"] == "STATIC" and now - since > TRAFFIC_BLOCK_REPORT_S
                        and not (prev or {}).get("reported")):
                    self.waits[rid]["reported"] = True
                    v._event("TRAFFIC_BLOCKED_BY_VEHICLE", other=h["other"], waited_s=round(now - since, 1),
                             operator_action_required=True)
                elif prev and prev.get("reported"):
                    self.waits[rid]["reported"] = True
                await v.set_hold(h["s"], {k: x for k, x in h.items() if k != "s"})
            elif getattr(v, "hold", None):
                self._clear_ticks[rid] = self._clear_ticks.get(rid, 0) + 1
                if self._clear_ticks[rid] >= TRAFFIC_RELEASE_TICKS:
                    self.waits.pop(rid, None)
                    await v.clear_hold()
            else:
                self.waits.pop(rid, None)
        self.last_tick_wall_ms = round((time.perf_counter() - w0) * 1000, 1)

    def _decide(self, kind, A: _Pred, B: _Pred, a, b, prefer: Optional[str] = None):
        """(기다릴 차, 상대, 기다릴 자기 계획 선형 위치 s, 사유). 안전하게 기다릴 차가 없으면 (None, ...).
        - 멈춰 있는 상대(대기 중 포함) 앞: 다가가는 차가 충돌 지점 앞에서 선다.
        - 같은 방향(FOLLOW): 늦게 오는 차가 선다. 충돌 지점 앞에서 못 서면 최대한 빨리 선다 (앞차는 멀어진다).
        - 교차·반대 방향(CROSS·HEAD_ON): 기다리는 자리는 상대가 앞으로 지날 길(교차로에서 도는 길 포함)에서 벗어난 곳 —
          자기 길을 따라 충돌 지점 앞쪽으로 물러나며 상대 길과 (두 차체 길이 절반 합 + 간격) 넘게 떨어진 첫 자리.
          우선순위가 낮은 차가 그런 자리에서 멈출 수 있으면 그 차가, 아니면 상대가 기다린다.
        prefer: 이미 상대를 기다리던 차 — 계속 기다릴 수 있으면 그대로 (양보가 번갈아 바뀌지 않게)."""
        clear_d = (A.v.profile.body_length_m + B.v.profile.body_length_m) / 2 + TRAFFIC_GAP_M

        def stoppable(P, ws):
            if ws is None:
                return False
            h = getattr(P.v, "hold", None)
            if h is not None and h["s"] <= ws + 1.0:
                return True                      # 이미 그 앞에 서도록 명령해 두었다
            return P.v_now < 0.3 or ws - P.s_now >= P.committed_m

        def simple_wait(P, smp):
            return max(P.s_now, smp[3] - TRAFFIC_HOLD_MARGIN_M)

        def clear_wait(P, smp, Q):
            """P 의 길에서 충돌 지점(smp) 앞쪽으로, Q 의 앞으로 길에서 clear_d 넘게 떨어진 가장 가까운 자리 (없으면 None)."""
            qs = Q.samples[1:] or Q.samples
            for x in sorted((x for x in P.samples if x[3] <= smp[3]), key=lambda x: -x[3]):
                if all(math.hypot(x[0] - q[0], x[1] - q[1]) >= clear_d for q in qs):
                    return max(P.s_now, x[3] - TRAFFIC_HOLD_MARGIN_M)
            return None

        b_stopped = B.static or (B.v_now < 0.3 and (b[2] == 0.0 or (B.tail and b is B.samples[-1])))
        if b_stopped:
            ws = simple_wait(A, a)
            return (A, B, ws, "STOPPED_VEHICLE_AHEAD") if stoppable(A, ws) else (None, B, None, "CANNOT_STOP_FOR_STOPPED")
        if kind == "FOLLOW":
            P, smp, Q = (A, a, B) if b[2] <= a[2] else (B, b, A)
            if prefer is not None and prefer == Q.v.resource_id:
                P, smp, Q = Q, (b if Q is B else a), P
            return P, Q, max(simple_wait(P, smp), P.s_now + P.committed_m), "FOLLOWING"
        def wait_point(P, smp, Q):
            ws = clear_wait(P, smp, Q)
            if ws is None and P.v_now < 0.3:
                ws = P.s_now          # 멈춰 있는 차는 그 자리에서 기다린다 (비킬 곳이 없어도 움직이지 않는 것이 안전 — 교착은 보고)
            return ws
        opts = {A.v.resource_id: (A, B, wait_point(A, a, B)), B.v.resource_id: (B, A, wait_point(B, b, A))}
        if prefer in opts and stoppable(opts[prefer][0], opts[prefer][2]):
            P, Q, ws = opts[prefer]
            return P, Q, ws, f"{kind}_KEEP_YIELDING"
        a_can, b_can = stoppable(A, opts[A.v.resource_id][2]), stoppable(B, opts[B.v.resource_id][2])
        order = sorted([(A, a, a_can), (B, b, b_can)],
                       key=lambda o: (o[2], self._priority(o[0], o[1][2])))   # 기다릴 수 없는 차(False)가 우선
        hi, lo = order
        for yi, ot, tag in ((lo, hi, "PRIORITY"), (hi, lo, "OTHER_CANNOT_WAIT")):
            P, Q, ws = opts[yi[0].v.resource_id]
            if stoppable(P, ws):
                return P, Q, ws, f"{kind}_{tag}"
        return None, None, None, "NO_SAFE_WAIT_POINT"

    def status(self) -> dict:
        return {"waits": self.waits, "deadlocks": self.deadlocks, "unavoidable": self.unavoidable,
                "tick_ms": self.last_tick_wall_ms, "ticks": self.ticks,
                "rules": {"gap_m": TRAFFIC_GAP_M, "time_slack_s": TRAFFIC_TIME_SLACK_S, "lane_m": TRAFFIC_LANE_M,
                          "follow_headway_s": TRAFFIC_FOLLOW_HEADWAY_S, "horizon_s": TRAFFIC_HORIZON_S,
                          "head_on": "같은 도로 반대 방향은 교행하지 않음 (차로 위치 미구현)",
                          "priority": "멈춘 차 앞은 다가가는 차가 멈춤 > 멈출 수 없는 차 > 소방차 > 먼저 도착 > id. "
                                      "기다릴 자리가 상대 길 위면 상대가 기다림",
                          "basis": "속도 계획 기반 시공간 표본 예측 (근사, 충돌 보장 아님)"}}


def _with_tail(self):
    """마지막 표본(목적지·대기 지점)에 머문다고 표시."""
    self.tail = True
    return self


_Pred.tail = False
_Pred._with_tail = _with_tail
