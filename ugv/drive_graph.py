# -*- coding: utf-8 -*-
"""ugv/drive_graph.py — 진행 방향을 유지하는 도로 경로 그래프 (2026-10-09 사용자 결정).

규칙
1. 경로는 지금 진행 방향으로 시작한다. 위치는 명령 교체 지연(CMD_LATENCY_S) 동안 갈 거리만큼 앞으로 옮겨 잡는다.
   정차 중에도 방향 제약을 유지한다 (차량 방향 = 텔레메트리 진행 방향).
2. 같은 도로를 즉시 되짚는 U턴·후진·3점 회전은 없다. 교차로에서는 다른 도로로만 나간다. 필요하면 도로망을 돌아 접근한다.
3. 회전은 차체 전체가 도로 안에 들어올 때만 허용한다 (최소 회전반경·차체·도로 폭. 아래 '회전 검사').
   첫 교차로는 지금 속도에서 회전 속도까지 감속할 거리가 있어야 회전할 수 있다:
       필요 거리 = (v² − v_회전²) / (2 × DECEL_MPS2) + TURN_MARGIN_M   (지연 거리는 1에서 이미 반영)
   도로 중간 급커브도 같은 기준으로 본다 (최소 회전반경보다 좁으면 그 도로를 쓰지 않는다).
4. 일방통행(원자료 ONEWAY)을 지킨다.
5. 도착 지점은 도착했을 때의 방향에서 소속 거점 출입 지점으로 돌아올 경로가 있어야 고른다 (계획 시점 기준.
   화재로 나중에 끊길 수 있으므로 복귀를 보장하지 않는다 → 결과에 '계획 시점 복귀 경로 있음' 으로만 남긴다).

회전 검사 (근사): 뒤축 중심이 두 도로 중심선에 접하는 원호(반경 R)를 따라 돈다고 보고, 차체 둘레 표본(0.45 m 간격,
모서리 포함)이 진행 방향 2.5° 간격마다 그 노드에 붙은 모든 도로 띠(중심선 ± 추정 반폭) 안에 있는지 본다. R 은 최소 회전반경부터 넓혀 가며 맞는 것 중 가장 큰 값을 회전 속도
계산에 쓴다 (v = √(LAT_ACCEL × R)). 교차로 확장면·차로 위치(우측 통행)는 반영하지 않는다.
도로 폭은 원자료 차로 수 × LANE_WIDTH_EST_M + 길어깨로 추정한 값이다 (원자료 WIDTH 는 1~4 코드라 쓰지 않는다).
"""

import bisect
import heapq
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from . import config
from .geo import bearing_deg
from .profiles import DEFAULT_PROFILE, VehicleProfile
from .roads import Road, RoadNetwork, RoadPos, Route

State = Tuple[str, str]          # (road_id, 향하는 노드)

# 모듈 수준 값은 기본 UGV 특성(ugv/profiles.py adair_ugv)의 값 — 기존 도구 호환용. 계획·검사는 DriveGraph.p (차량별) 를 쓴다
_P0 = DEFAULT_PROFILE
R_MIN = _P0.r_min
_HW = _P0.hw
BODY_SAMPLES = _P0.body_samples
BODY_REACH = _P0.body_reach
# 최종 경로 검사 간격 (근사 표본 검사 — 연속 충돌 판정이 아니다). 곡률반경 CHECK_FINE_R_M 보다 좁은 곳은 촘촘히:
# 반경 4.7 m 회전에서 1 m 진행하면 차체 끝(뒤축에서 약 3.6 m)이 약 0.8 m 움직여 표본 사이 누락이 커진다 → 0.25 m 면 약 0.2 m
CHECK_STEP_M = 1.0
CHECK_FINE_STEP_M = 0.25
CHECK_FINE_R_M = 25.0
CHECK_TOL_M = 0.02               # 수치 오차 허용
YAW_WINDOW_M = 1.0               # 차체 방향 = 앞뒤 이 거리 두 점의 방향
RADIUS_WINDOW_M = 3.0            # 곡률반경 = 앞뒤 이 거리 세 점의 외접원 (경로 양 끝에서는 창을 안쪽으로 민다)
# 초기 자세 예외 (최소): 출발 자세의 차체가 이미 도로 띠 밖으로 나가 있으면(사실), 출발점부터 차체 길이만큼 가는 동안
# 그 초과량을 넘지 않는 차체 표본만 승인 판정에서 뺀다. 초과가 허용치 아래로 내려오거나 차체 길이를 지나면 끝난다.
# 회전반경은 예외가 없다. 출발 도로 전체를 빼던 예전 규칙(직선거리로 추정한 start_leg)은 없앴다 (2026-10-10 검토)
TURN_FIT_STEP_DEG = 2.5          # 교차로 회전 검사의 진행 방향 표본 간격 (5° 는 차체 끝 약 0.3 m 씩 건너뛰어 놓쳤다)
R_STEPS = (1.0, 1.15, 1.3, 1.5, 1.8, 2.2, 3.0, 4.0, 6.0, 9.0, 14.0, 20.0, 30.0)
STRAIGHT_DEG = 5.0
STRIP_LEN_M = 60.0
DIR_LEN_M = 15.0
TURN_TANGENT_MAX_M = 25.0       # 교차로 회전 원호가 노드 앞뒤로 차지하는 길이 상한
BEND_R_M = 50.0                 # 도로 선형의 꺾인 점(교차로 아님)을 다듬는 원호 반경 목표
BEND_TANGENT_SHARE = 0.45       #   이웃 선분 길이의 이 비율까지만 쓴다
ARC_STEP_M = 2.0                # 원호 표본 간격 상한
ARC_SAGITTA_M = 0.02            # 원호 표본을 이은 현이 원호에서 벗어나는 최대 거리


@dataclass(frozen=True)
class DirPos:
    """도로 위 지점 + 향하는 노드 (진행 방향)."""
    pos: RoadPos
    toward: str

    @property
    def road_id(self):
        return self.pos.road_id


@dataclass
class Turn:
    to: State
    v_turn: float               # 이 회전의 최고 속도 (m/s). 직진에 가까우면 inf
    deflect_deg: float
    radius_m: Optional[float]   # 경로에 실제로 그리는 원호 반경 (차체 검사를 통과한 값). 직진이면 None
    tangent_m: float = 0.0      # 그 원호가 노드 앞뒤로 차지하는 길이


@dataclass
class Arrival:
    time_s: float
    kind: str                   # DIRECT (출발 도로 앞쪽) / VIA (그 도로에 들어와서)
    state: Optional[tuple]      # VIA 일 때 목적지 도로에 들어선 꼬리표 (상태, 직전 상태)
    toward: str                 # 도착할 때 향하던 노드


def est_half_width(road: Road) -> float:
    lanes = road.attrs.get("lanes") or 2
    return max(int(lanes), 1) * config.LANE_WIDTH_EST_M / 2 + config.SHOULDER_EST_M


def _seg_dist(q, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / L2))
    return math.hypot(q[0] - a[0] - t * dx, q[1] - a[1] - t * dy)


def _radius3(a, b, c):
    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    if area2 < 1e-9:
        return math.inf
    return math.dist(a, b) * math.dist(b, c) * math.dist(a, c) / (2.0 * area2)


class DriveGraph:
    def __init__(self, net: RoadNetwork, profile: VehicleProfile = DEFAULT_PROFILE):
        self.net = net
        self.p = profile
        self.half_w = {rid: est_half_width(r) for rid, r in net.roads.items()}
        self.min_radius: Dict[str, float] = {}
        self.sharp_roads = set()                # 도로 중간 급커브로 쓰지 않는 도로
        for rid, r in net.roads.items():
            self.min_radius[rid] = self._road_min_radius(r)
            if not self._bend_ok(self.min_radius[rid], self.half_w[rid]):
                self.sharp_roads.add(rid)
        self.turns: Dict[State, List[Turn]] = {}
        self.blocked_turns: List[dict] = []     # 기하로 막힌 회전 (감사용)
        self._build_turns()
        self.rev: Dict[State, List[State]] = {}
        self.tan: Dict[Tuple[State, State], float] = {}     # (지금 상태, 다음 상태) → 회전 원호가 노드 앞뒤로 차지하는 길이
        for s, ts in self.turns.items():
            for t in ts:
                self.rev.setdefault(t.to, []).append(s)
                self.tan[(s, t.to)] = t.tangent_m

    # ------------------------------------------------------------------ 정적 속성
    def states(self) -> List[State]:
        """들어가 달릴 수 있는 방향 상태 전부 (일방통행·급커브 반영)."""
        return [(rid, to) for rid, r in self.net.roads.items() for to in (r.a, r.b) if self.allowed(rid, self.other(rid, to))]

    def components(self) -> List[List[State]]:
        """방향 상태 그래프의 강연결 성분 (서로 오갈 수 있는 묶음), 큰 순서."""
        succ = {s: [t.to for t in self.turns.get(s, ())] for s in self.states()}
        index, low, on, stack, comps, counter = {}, {}, set(), [], [], [0]
        for root in succ:
            if root in index:
                continue
            work = [(root, iter(succ[root]))]
            index[root] = low[root] = counter[0]; counter[0] += 1; stack.append(root); on.add(root)
            while work:
                v, it = work[-1]
                pushed = False
                for w in it:
                    if w not in succ:
                        continue
                    if w not in index:
                        index[w] = low[w] = counter[0]; counter[0] += 1; stack.append(w); on.add(w)
                        work.append((w, iter(succ[w])))
                        pushed = True
                        break
                    if w in on:
                        low[v] = min(low[v], index[w])
                if pushed:
                    continue
                work.pop()
                if work:
                    low[work[-1][0]] = min(low[work[-1][0]], low[v])
                if low[v] == index[v]:
                    comp = []
                    while True:
                        w = stack.pop(); on.discard(w); comp.append(w)
                        if w == v:
                            break
                    comps.append(comp)
        return sorted(comps, key=len, reverse=True)

    @property
    def core(self) -> set:
        """가장 큰 강연결 성분 — 그 안 어디서든 그 안 어디로든 갈 수 있다."""
        if getattr(self, "_core", None) is None:
            self._core = set(self.components()[0])
        return self._core

    def road_is_spare(self, rid: str, keep: str, min_share: float = 0.97) -> bool:
        """도로 rid 를 빼도 서로 오갈 수 있는 묶음(가장 큰 강연결 성분)이 거의 그대로(min_share 이상)이고 keep 도로가 그 안에
        남는가 — 대기 차량이 그 도로를 막아도 다른 차의 길이 끊기지 않는지."""
        cache = self.__dict__.setdefault("_spare", {})
        key = (rid, keep)
        if key in cache:
            return cache[key]
        core = self.core
        succ = {s: [t.to for t in self.turns.get(s, ()) if t.to[0] != rid] for s in core if s[0] != rid}
        start = next(((keep, n) for n in (self.net.roads[keep].a, self.net.roads[keep].b) if (keep, n) in succ), None)
        if start is None:
            cache[key] = False
            return False
        fwd, stack = {start}, [start]
        while stack:
            for t in succ.get(stack.pop(), ()):
                if t in succ and t not in fwd:
                    fwd.add(t)
                    stack.append(t)
        rev = {}
        for s_, ts in succ.items():
            for t in ts:
                rev.setdefault(t, []).append(s_)
        bwd, stack = {start}, [start]
        while stack:
            for t in rev.get(stack.pop(), ()):
                if t in succ and t not in bwd:
                    bwd.add(t)
                    stack.append(t)
        ok = len(fwd & bwd) >= min_share * (len(core) - sum(1 for s_ in core if s_[0] == rid))
        cache[key] = ok
        return ok

    def detour_ok(self, rid: str, home: RoadPos, radius_m: float = 6000.0, mean_max: float = 1.10, worst_max: float = 1.5) -> bool:
        """도로 rid 를 막아도 거점 출입 지점(home)에서 반경 안 도로들까지 걸리는 시간이 크게 늘지 않는가 (양 방향 출발 기준).
        대기 차량이 그 도로에 서 있으면 다른 차는 그 도로로 계획하지 않으므로, 막힌 도로 때문에 크게 돌아가는 곳은 고르지 않는다
        (2026-10-10: 거점 B 두 번째 도로를 막자 B-ugv1 복귀가 43 km 가 됨)."""
        cache = self.__dict__.setdefault("_detour", {})
        key = (rid, home.road_id, round(home.s, 1))
        if key in cache:
            return cache[key]
        r = self.net.roads[home.road_id]
        ratios = []
        for to in (r.a, r.b):
            if not self.allowed(home.road_id, self.other(home.road_id, to)):
                continue
            st = DirPos(home, to)
            base = self.search(st, 0.0)["dist"]
            cut = self.search(st, 0.0, road_ok=lambda x: x != rid)["dist"]
            for s_, t0 in base.items():
                if s_[0] == rid or t0 <= 0:
                    continue
                x, y = self.net.nodes[s_[1]]
                if math.hypot(x - home.x, y - home.y) > radius_m:
                    continue
                ratios.append(cut.get(s_, math.inf) / t0)
        ok = bool(ratios) and sum(min(q, 10.0) for q in ratios) / len(ratios) <= mean_max and max(ratios) <= worst_max
        cache[key] = ok
        return ok

    def oneway_ok(self, rid: str, frm: str) -> bool:
        r = self.net.roads[rid]
        ow = r.attrs.get("oneway", "BOTH")
        if ow == "A_TO_B":
            return frm == r.a
        if ow == "B_TO_A":
            return frm == r.b
        return True

    def allowed(self, rid: str, frm: str) -> bool:
        """도로 rid 에 frm 노드에서 들어가 달릴 수 있나 (일방통행·급커브·자동차전용 도로)."""
        if self.net.roads[rid].attrs.get("motorway") and not config.UGV_ON_MOTORWAY:
            return False
        return rid not in self.sharp_roads and self.oneway_ok(rid, frm)

    def other(self, rid: str, node: str) -> str:
        r = self.net.roads[rid]
        return r.b if node == r.a else r.a

    def _road_min_radius(self, r: Road) -> float:
        L = r.length
        if L < 2 * 7.5:
            return math.inf
        best = math.inf
        s = 7.5
        while s <= L - 7.5:
            best = min(best, _radius3(self.net.at(r.road_id, s - 7.5)[2:4], self.net.at(r.road_id, s)[2:4],
                                      self.net.at(r.road_id, s + 7.5)[2:4]))
            s += 2.5
        return best

    def _bend_ok(self, R: float, half_w: float) -> bool:
        hw = self.p.hw
        if math.isinf(R):
            return hw <= half_w
        if R < self.p.r_min:
            return False
        outer = math.hypot(R + hw, self.p.front) - R      # 바깥 앞모서리가 중심선 밖으로 나가는 거리
        return outer <= half_w and hw <= half_w

    def road_speed(self, road: Road) -> float:
        """이 차량의 도로 속도 = min(도로 속도 (제한속도·최저 기준), 차량 최고속도)."""
        return min(self.net.road_speed(road), self.p.max_speed_mps)

    def _dir(self, rid: str, node: str, out: bool):
        r = self.net.roads[rid]
        s = min(DIR_LEN_M, r.length)
        if node == r.a:
            p0, p1 = r.xy[0], self.net.at(rid, s)[2:4]
        else:
            p0, p1 = r.xy[-1], self.net.at(rid, r.length - s)[2:4]
        v = (p1[0] - p0[0], p1[1] - p0[1])
        n = math.hypot(*v) or 1.0
        v = (v[0] / n, v[1] / n)
        return v if out else (-v[0], -v[1])

    def _strips(self, node):
        V = self.net.nodes[node]
        out = []
        for rid, _ in self.net.adj.get(node, []):
            r = self.net.roads[rid]
            pts = r.xy if r.a == node else r.xy[::-1]
            poly, acc = [pts[0]], 0.0
            for p, q in zip(pts, pts[1:]):
                poly.append(q)
                acc += math.dist(p, q)
                if acc > STRIP_LEN_M:
                    break
            out.append(([(p[0] - V[0], p[1] - V[1]) for p in poly], self.half_w[rid]))
        return out

    @staticmethod
    def _inside(q, strips):
        for poly, h in strips:
            for a, b in zip(poly, poly[1:]):
                if _seg_dist(q, a, b) <= h + CHECK_TOL_M:          # 최종 경로 검사와 같은 수치 허용
                    return True
        return False

    def turn_fit(self, u, w, strips, Lin, Lout):
        """(가능 여부, 맞는 가장 큰 반경, 꺾임각°)."""
        D = math.atan2(u[0] * w[1] - u[1] * w[0], u[0] * w[0] + u[1] * w[1])
        ad = abs(D)
        if math.degrees(ad) < STRAIGHT_DEG:
            return True, math.inf, math.degrees(ad)
        sg = 1 if D > 0 else -1
        for f in reversed(R_STEPS):
            R = self.p.r_min * f
            T = R * math.tan(ad / 2)
            if T > min(Lin, STRIP_LEN_M, TURN_TANGENT_MAX_M) + 1 or T > min(Lout, STRIP_LEN_M, TURN_TANGENT_MAX_M) + 1:
                continue
            P1 = (-T * u[0], -T * u[1])
            C = (P1[0] - u[1] * sg * R, P1[1] + u[0] * sg * R)
            ok = True
            n = max(12, math.ceil(math.degrees(ad) / TURN_FIT_STEP_DEG))
            for k in range(n + 1):
                hd = math.atan2(u[1], u[0]) + sg * ad * k / n
                ch, sh = math.cos(hd), math.sin(hd)
                rx, ry = C[0] + R * sh * sg, C[1] - R * ch * sg
                for fx, fy in self.p.body_samples:
                    if not self._inside((rx + fx * ch - fy * sh, ry + fx * sh + fy * ch), strips):
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                return True, R, math.degrees(ad)
        return False, None, math.degrees(ad)

    def _build_turns(self):
        net = self.net
        for n, arms in net.adj.items():
            strips = self._strips(n)
            for rin, frm in arms:
                # 진입 도로는 거르지 않는다 (차가 이미 급커브 도로 위에 있어도 빠져나갈 회전은 있어야 한다).
                # 일방통행 역방향 진입 상태는 어떤 회전으로도 도달하지 않으므로 표에 있어도 쓰이지 않는다
                u = self._dir(rin, n, out=False)
                lst = self.turns.setdefault((rin, n), [])
                for rout, to in arms:
                    if rout == rin or not self.allowed(rout, n):       # 같은 도로 되짚기(U턴) 금지
                        continue
                    w = self._dir(rout, n, out=True)
                    ok, R, deg = self.turn_fit(u, w, strips, net.roads[rin].length, net.roads[rout].length)
                    if not ok:
                        self.blocked_turns.append({"node": n, "from_road": rin, "to_road": rout,
                                                   "deflect_deg": round(deg, 1), "entry_legal": self.allowed(rin, frm)})
                        continue
                    vmax = min(self.road_speed(net.roads[rin]), self.road_speed(net.roads[rout]))
                    v = vmax if math.isinf(R) else min(vmax, math.sqrt(self.p.lat_accel_mps2 * R))
                    T = 0.0 if math.isinf(R) else R * math.tan(math.radians(deg) / 2)
                    lst.append(Turn((rout, to), v, round(deg, 1), None if math.isinf(R) else round(R, 2), round(T, 2)))

    # ------------------------------------------------------------------ 위치·방향
    def dirpos(self, pos: RoadPos, heading_deg: float) -> Optional[DirPos]:
        """도로 위 지점 + 차량 진행 방위(북=0, 시계) → 향하는 노드. 일방통행 역방향이면 None."""
        r = self.net.roads[pos.road_id]
        p0 = self.net.at(pos.road_id, max(0.0, pos.s - 2.0))[2:4]
        p1 = self.net.at(pos.road_id, min(r.length, pos.s + 2.0))[2:4]
        tx, ty = p1[0] - p0[0], p1[1] - p0[1]                       # a → b 방향
        hx, hy = math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg))
        toward = r.b if tx * hx + ty * hy >= 0 else r.a
        if not self.oneway_ok(pos.road_id, self.other(pos.road_id, toward)):
            return None                                             # 일방통행을 거꾸로 향하고 있다
        return DirPos(pos, toward)

    def ahead_m(self, dp: DirPos) -> float:
        r = self.net.roads[dp.road_id]
        return r.length - dp.pos.s if dp.toward == r.b else dp.pos.s

    def advance(self, dp: DirPos, d: float) -> DirPos:
        """진행 방향으로 d m (출발 도로 안에서만. 노드 직전에서 멈춘다). 교차로를 넘는 예측은 project()."""
        r = self.net.roads[dp.road_id]
        d = max(0.0, min(d, self.ahead_m(dp) - 0.1))
        s = dp.pos.s + d if dp.toward == r.b else dp.pos.s - d
        return DirPos(self.net.at(dp.road_id, s), dp.toward)

    def project(self, dp: DirPos, d: float, prefer: Optional[List[Tuple[float, float]]] = None) -> DirPos:
        """명령 교체 지연 동안 갈 거리 d 만큼 앞의 위치·방향. 교차로를 넘으면 다음 도로 위로 잡는다
        (예전에는 교차로 0.1 m 앞에서 잘라 이미 다음 도로에 들어간 차를 이전 도로에서 계획했다, 2026-10-10 검토).
        prefer(지금 위치부터의 계획 경로 점)가 있으면 그 경로를 따라, 없으면 진행 방향 도로를 따라(가장 덜 꺾이는 회전) 간다."""
        if d <= 0.5:
            return dp
        if d < self.ahead_m(dp) - 0.1:
            return self.advance(dp, d)
        pts, got = self.stop_path(dp, d, prefer)
        if len(pts) < 2:
            return self.advance(dp, d)
        (ax, ay), (bx, by) = pts[-2], pts[-1]
        hd = bearing_deg(bx - ax, by - ay) if math.hypot(bx - ax, by - ay) > 1e-6 else None
        q = self.snap_aligned(bx, by, hd) if hd is not None else None
        if q is None:
            return self.advance(dp, d)
        return q

    def snap_aligned(self, x: float, y: float, heading_deg: float, max_m: float = 8.0) -> Optional[DirPos]:
        """(x, y) 근처 도로 중 진행 방위와 방향이 맞는 도로 위 지점 (교차로 한가운데서 가로지르는 도로를 고르지 않게)."""
        hx, hy = math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg))
        best = None
        for rid, dist in self.net.near(x, y, max_m).items():
            r = self.net.roads[rid]
            pos = self.net.on_road(rid, x, y)
            p0 = self.net.at(rid, max(0.0, pos.s - 2.0))[2:4]
            p1 = self.net.at(rid, min(r.length, pos.s + 2.0))[2:4]
            tx, ty = p1[0] - p0[0], p1[1] - p0[1]
            n = math.hypot(tx, ty) or 1.0
            align = (tx * hx + ty * hy) / n
            if abs(align) < 0.7:
                continue
            toward = r.b if align > 0 else r.a
            if not self.oneway_ok(rid, self.other(rid, toward)):
                continue
            key = dist - 2.0 * abs(align)
            if best is None or key < best[0]:
                best = (key, DirPos(pos, toward))
        return best[1] if best else None

    def outside_m(self, x: float, y: float) -> float:
        """점이 추정 도로 띠(근처 모든 도로: 중심선 ± 추정 반폭)의 합집합 밖으로 나간 거리. 안이면 0 이하."""
        near = self.net.near(x, y, 15.0)
        if not near:
            p = self.net.snap(x, y)
            return p.dist - self.half_w[p.road_id]
        return min(d - self.half_w[rid] for rid, d in near.items())

    def brake_need(self, v0: float, v1: float) -> float:
        if v0 <= v1:
            return 0.0
        return (v0 * v0 - v1 * v1) / (2 * self.p.decel_mps2) + config.TURN_MARGIN_M

    # ------------------------------------------------------------------ 탐색
    @staticmethod
    def _lab_ok(tin: float, tout: float, length: float) -> bool:
        """한 도로에서 들어올 때 원호(노드 뒤 tin)와 나갈 때 원호(노드 앞 tout)가 겹치지 않는가.
        겹치면 두 원호를 줄여야 해서 차체 검사한 반경보다 좁아진다 → 그 연속 회전은 쓰지 않는다 (짧은 도로 연속 회전)."""
        return tin + tout <= length + 1e-6

    def search(self, start: DirPos, v0: float = 0.0, road_ok: Callable[[str], bool] = lambda r: True,
               turn_ok: Callable[[State, State], bool] = lambda a, b: True):
        """출발 방향을 유지하는 최소 시간 탐색. 꼬리표 = (상태 (도로, 향하는 노드), 들어온 직전 상태) — 같은 도로라도
        어느 회전으로 들어왔는지(그 원호 길이)에 따라 다음 회전 가능 여부가 다르다 (_lab_ok). 값 = 그 도로에 들어선 시각.
        출발 도로의 남은 구간은 road_ok 와 관계없이 지난다 (차량이 거기 있다) — 대신 최종 경로 검사가 그 형상을 본다.
        turn_ok(지금 상태, 다음 상태): 최종 경로 검사에서 실패한 회전을 빼고 다시 찾을 때 쓴다."""
        net = self.net
        s0 = (start.road_id, start.toward)
        d0 = self.ahead_m(start)
        t0 = d0 / self.road_speed(net.roads[start.road_id])
        dist: Dict[tuple, float] = {}
        prev: Dict[tuple, Optional[tuple]] = {}
        pq = []
        for t in self.turns.get(s0, ()):
            if not road_ok(t.to[0]) or not turn_ok(s0, t.to):
                continue
            # 첫 교차로: 회전 원호가 시작하는 곳(노드 앞 tangent_m)까지 회전 속도로 줄일 거리가 있어야 한다
            if d0 - t.tangent_m < self.brake_need(v0, t.v_turn):
                continue
            lab = (t.to, s0)
            if t0 < dist.get(lab, math.inf):
                dist[lab], prev[lab] = t0, None
                heapq.heappush(pq, (t0, lab))
        done = set()
        while pq:
            T, lab = heapq.heappop(pq)
            if lab in done:
                continue
            done.add(lab)
            s, pin = lab
            r = net.roads[s[0]]
            tin = self.tan.get((pin, s), 0.0)
            Tn = T + r.length / self.road_speed(r)
            for t in self.turns.get(s, ()):
                if not road_ok(t.to[0]) or not turn_ok(s, t.to) or not self._lab_ok(tin, t.tangent_m, r.length):
                    continue
                nl = (t.to, s)
                if Tn < dist.get(nl, math.inf):
                    dist[nl], prev[nl] = Tn, lab
                    heapq.heappush(pq, (Tn, nl))
        labels: Dict[State, List[tuple]] = {}
        for lab, T in dist.items():
            labels.setdefault(lab[0], []).append((T, lab))
        for v in labels.values():
            v.sort()
        best = {s: v[0][0] for s, v in labels.items()}
        return {"start": start, "v0": v0, "dist": best, "labels": labels, "prev": prev}

    def arrivals(self, tree, p: RoadPos, seg_ok: Callable[[str, float, float], bool] = lambda r, a, b: True) -> List[Arrival]:
        """p 에 도착하는 방법들 (방향별 가장 빠른 것). seg_ok(road, s_from, s_to): 목적지 도로에서 지나는 구간 안전.
        목적지 도로에 들어오는 회전 원호가 목적지 앞에서 끝나야 한다 (원호 한가운데 정차하지 않는다)."""
        net = self.net
        st: DirPos = tree["start"]
        r = net.roads[p.road_id]
        v = self.road_speed(r)
        out = []
        if p.road_id == st.road_id:
            d = (p.s - st.pos.s) if st.toward == r.b else (st.pos.s - p.s)
            if d >= self.brake_need(tree["v0"], 0.0) - config.TURN_MARGIN_M and d > 0 and seg_ok(p.road_id, st.pos.s, p.s):
                out.append(Arrival(d / v, "DIRECT", None, st.toward))
        for toward, frm_s in ((r.b, 0.0), (r.a, r.length)):
            s = (p.road_id, toward)
            run = abs(p.s - frm_s)
            for T, lab in tree["labels"].get(s, ()):
                if self.tan.get((lab[1], s), 0.0) <= run and seg_ok(p.road_id, frm_s, p.s):
                    out.append(Arrival(T + run / v, "VIA", lab, toward))
                    break
        return out

    def build_route(self, tree, p: RoadPos, arr: Arrival) -> Route:
        net = self.net
        st: DirPos = tree["start"]
        pts, vlim, rids, legs = [], [], [], []

        def add(seq, v, rid, frm, to):
            for q in seq:
                if pts and math.dist(pts[-1], q) < 0.01:
                    continue
                pts.append(q)
                vlim.append(v)
            if not rids or rids[-1] != rid:
                rids.append(rid)
            legs.append((rid, frm, to))

        r0 = net.roads[st.road_id]
        frm0 = self.other(st.road_id, st.toward)
        if arr.kind == "DIRECT":
            add(net.polyline(st.road_id, st.pos.s, p.s), self.road_speed(r0), st.road_id, frm0, st.toward)
        else:
            chain = []
            lab = arr.state
            while lab is not None:
                chain.append(lab[0])
                lab = tree["prev"][lab]
            chain.reverse()
            add(net.polyline(st.road_id, st.pos.s, r0.length if st.toward == r0.b else 0.0), self.road_speed(r0),
                st.road_id, frm0, st.toward)
            for rid, to in chain[:-1]:
                r = net.roads[rid]
                add(net.polyline(rid, 0.0, r.length) if to == r.b else net.polyline(rid, r.length, 0.0),
                    self.road_speed(r), rid, self.other(rid, to), to)
            rid, to = chain[-1]
            r = net.roads[rid]
            add(net.polyline(rid, 0.0 if to == r.b else r.length, p.s), self.road_speed(r), rid, self.other(rid, to), to)
        if len(pts) == 1:
            pts.append(pts[0])
            vlim.append(vlim[0])
        else:
            pts, vlim = self._smooth(pts, vlim, legs)
        route = Route(pts, vlim, rids, st.pos, p, arr.time_s)
        route.legs, route.arrive_toward = legs, arr.toward
        return route

    # ------------------------------------------------------------------ 주행 선형 다듬기
    def _smooth(self, pts, vlim, legs):
        """도로 중심선 꺾인 점을 차가 따라갈 수 있는 원호로 바꾼다 (2026-10-10).
        PX4 는 꺾인 점에서 스스로 모서리를 질러가는데(RA_ACC_RAD_*) 좁은 도로에서 차체가 도로 밖으로 나갔다 →
        계획에서 원호를 그리고 PX4 질러가기는 끈다 (airframe RA_ACC_RAD_MAX -1).
        1) 교차로: 회전표가 차체 검사한 원호 그대로 — 노드를 지나는 두 방향선(노드에서 DIR_LEN_M 떨어진 점으로 잰 진입·진출
           방향, turn_fit 과 같은 값)에 접하는 반경 R 원호, 노드에서 tangent_m 떨어진 두 접점 사이. 노드 근처 도로 선형의
           작은 꺾임은 이 원호에 흡수된다 (예전에는 원호 옆 꺾임을 줄여 최소 회전반경보다 좁은 원호가 생겼다).
           이웃 교차로 원호와 겹치면 둘 다 줄인다 — 반경이 작아질 수 있고, 그러면 최종 검사(check_route)가 거른다.
        2) 그 밖의 꺾인 점: 반경 BEND_R_M 목표, 이웃 선분의 BEND_TANGENT_SHARE 까지, 교차로 원호 구간은 건드리지 않는다.
        원호는 정확한 원호로 ARC_STEP_M 간격 표본을 둔다."""
        def cum_of(p):
            c = [0.0]
            for a, b in zip(p, p[1:]):
                c.append(c[-1] + math.dist(a, b))
            return c

        def unit(a, b):
            d = math.dist(a, b) or 1.0
            return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)

        def fillet(V, u, w, T):
            """꼭짓점 V, 들어오는 방향 u, 나가는 방향 w 에 접하는 원호 표본 (V-Tu 부터 V+Tw 까지, 양 끝 포함)."""
            D = abs(math.atan2(u[0] * w[1] - u[1] * w[0], u[0] * w[0] + u[1] * w[1]))
            p1 = (V[0] - T * u[0], V[1] - T * u[1])
            p2 = (V[0] + T * w[0], V[1] + T * w[1])
            if D < math.radians(1.0) or T <= 1e-6:
                return [p1, p2]
            R = T / math.tan(D / 2)
            sg = 1 if (u[0] * w[1] - u[1] * w[0]) > 0 else -1
            C = (p1[0] - u[1] * sg * R, p1[1] + u[0] * sg * R)
            a0 = math.atan2(p1[1] - C[1], p1[0] - C[0])
            # 표본 사이 현이 원호 안쪽으로 들어가는 거리(R − √(R² − (step/2)²))를 ARC_SAGITTA_M 이하로
            step = min(ARC_STEP_M, math.sqrt(8 * R * ARC_SAGITTA_M))
            n = max(2, math.ceil(R * D / step))
            return [(C[0] + R * math.cos(a0 + sg * D * k / n), C[1] + R * math.sin(a0 + sg * D * k / n))
                    for k in range(n)] + [p2]

        cum = cum_of(pts)
        L = cum[-1]
        # 1) 교차로 원호: 노드에서 진입·진출 방향선(노드 ~ 도로를 따라 DIR_LEN_M 떨어진 점을 이은 직선) 위 접점 사이 원호.
        #    원호 앞뒤도 그 방향선을 따라 DIR_LEN_M 점까지 곧게 잇는다 (원호 끝에서 꺾이지 않게). 그 점에서 도로 선형으로 돌아간다
        J = []
        for (r1, _, t1), (r2, _, t2) in zip(legs, legs[1:]):
            turn = next((t for t in self.turns.get((r1, t1), ()) if t.to == (r2, t2)), None)
            if turn is None or turn.radius_m is None:
                continue
            nx, ny = self.net.nodes[t1]
            j = min(range(1, len(pts) - 1), key=lambda i: (pts[i][0] - nx) ** 2 + (pts[i][1] - ny) ** 2, default=None)
            if j is None or math.hypot(pts[j][0] - nx, pts[j][1] - ny) >= 0.5:
                continue
            J.append([cum[j], turn.tangent_m, self._dir(r1, t1, out=False), self._dir(r2, t1, out=True), (nx, ny),
                      min(DIR_LEN_M, self.net.roads[r1].length), min(DIR_LEN_M, self.net.roads[r2].length)])
        for a, b in zip(J, J[1:]):                       # 같은 도로 양 끝 원호가 겹치면 (탐색이 피하지만) 둘 다 줄인다
            over = a[1] + b[1] - (b[0] - a[0])
            if over > 0:
                k = (b[0] - a[0]) / max(a[1] + b[1], 1e-9)
                a[1] *= k
                b[1] *= k
        out, ov, protected, s_done = [pts[0]], [vlim[0]], [], 0.0
        for idx, (s0, T, u, w, V, dl_in, dl_out) in enumerate(J):
            T = min(T, s0 - s_done - 0.05, L - s0 - 0.5)   # 경로 양 끝·앞 원호를 넘지 않게 (줄면 최종 검사가 본다)
            if T <= 0.3:
                continue
            nxt = J[idx + 1] if idx + 1 < len(J) else None
            span_in = max(T, min(dl_in, s0 - s_done))
            if span_in > s0 - s_done - 0.5:                     # 경로 시작·앞 원호에 붙으면 방향선 점 없이 원호로 바로
                span_in = T
            span_out = max(T, min(dl_out, L - s0 - 0.5, (nxt[0] - nxt[1] - s0) if nxt else math.inf))
            for k in range(len(pts)):
                if s_done < cum[k] < s0 - span_in - 0.05:
                    out.append(pts[k])
                    ov.append(vlim[k])
            ks = [k for k in range(len(pts)) if s0 - span_in - 1e-6 <= cum[k] <= s0 + span_out + 1e-6]
            v = min(vlim[k] for k in ks) if ks else min(vlim)
            seq = []
            if span_in > T + 0.05:
                seq.append((V[0] - span_in * u[0], V[1] - span_in * u[1]))
            arc = fillet(V, u, w, T)
            start = len(out) + len(seq)
            seq += arc
            end = len(out) + len(seq) - 1
            if span_out > T + 0.05:
                seq.append((V[0] + span_out * w[0], V[1] + span_out * w[1]))
            out += seq
            ov += [v] * len(seq)
            protected.append((start - (1 if span_in > T + 0.05 else 0), end + (1 if span_out > T + 0.05 else 0)))
            s_done = s0 + span_out
        for k in range(len(pts)):
            if cum[k] > s_done + 0.05:
                out.append(pts[k])
                ov.append(vlim[k])
        if math.dist(out[-1], pts[-1]) > 0.05:                  # 경로 끝(목적지)은 반드시 남긴다
            out.append(pts[-1])
            ov.append(vlim[-1])
        pts, vlim = out, ov
        # 중복 점 제거 (보호 구간 번호를 같이 옮긴다)
        keep = [0] + [i for i in range(1, len(pts)) if math.dist(pts[i - 1], pts[i]) >= 0.05]
        remap = {old: new for new, old in enumerate(keep)}

        def nearest_kept(i):
            while i not in remap and i > 0:
                i -= 1
            return remap.get(i, 0)
        protected = [(nearest_kept(a), nearest_kept(b)) for a, b in protected]
        pts, vlim = [pts[i] for i in keep], [vlim[i] for i in keep]
        cum = cum_of(pts)
        prot = set()
        for a, b in protected:
            prot.update(range(a, b + 1))
        # 2) 일반 꺾인 점
        corners = []
        for j in range(1, len(pts) - 1):
            if j in prot:
                continue
            u, w = unit(pts[j - 1], pts[j]), unit(pts[j], pts[j + 1])
            D = abs(math.atan2(u[0] * w[1] - u[1] * w[0], u[0] * w[0] + u[1] * w[1]))
            if math.degrees(D) < 2.0:
                continue
            # 꼭짓점에서 원호까지 거리 e = T·tan(D/4) 를 도로 반폭 − 차체 반폭 − 여유 안으로
            hw = self.half_w[self.net.snap(*pts[j]).road_id]
            e_max = max(0.3, hw - self.p.hw - 0.3)
            T = min(BEND_R_M * math.tan(D / 2),
                    BEND_TANGENT_SHARE * min(cum[j] - cum[j - 1], cum[j + 1] - cum[j]),
                    e_max / max(math.tan(D / 4), 1e-9))
            # 교차로 원호(보호 구간)의 끝점까지는 넘지 않는다
            if j - 1 in prot:
                T = min(T, cum[j] - cum[j - 1])
            if j + 1 in prot:
                T = min(T, cum[j + 1] - cum[j])
            corners.append([j, T, u, w])
        for a, b in zip(corners, corners[1:]):
            over = a[1] + b[1] - (cum[b[0]] - cum[a[0]])
            if over > 0:
                k = (cum[b[0]] - cum[a[0]]) / max(a[1] + b[1], 1e-9)
                a[1] *= k
                b[1] *= k
        by_j = {c[0]: c for c in corners if c[1] > 0.05}
        fp, fv = [], []
        for i, q in enumerate(pts):
            c = by_j.get(i)
            if c is None:
                seq = [q]
            else:
                seq = fillet(q, c[2], c[3], c[1])
            for p in seq:
                if fp and math.dist(fp[-1], p) < 0.05:
                    continue
                fp.append(p)
                fv.append(vlim[i])
        if math.dist(fp[-1], pts[-1]) > 0.05:
            fp.append(pts[-1])
            fv.append(vlim[-1])
        return fp, fv

    # ------------------------------------------------------------------ 최종 경로 검사
    def body_excess(self, x: float, y: float, yaw: float) -> float:
        """뒤축 (x, y)·진행 방향 yaw(ENU rad) 자세에서 차체 둘레 표본이 추정 도로 띠 합집합 밖으로 나간 최대 거리 (안이면 ≤ 0)."""
        near = self.net.near(x, y, 15.0)
        margin = max((self.half_w[r] - d for r, d in near.items()), default=-math.inf)   # 가장 안쪽 도로 띠의 남은 여유
        reach = self.p.body_reach
        if margin >= reach:                              # 차체가 어느 방향으로 돌아도 그 띠 안 → 상한값 (≤ 0)
            return reach - margin
        ch, sh = math.cos(yaw), math.sin(yaw)
        # 둘레 표본 전부를 한 번에: 근처 도로 선분을 한 번 모아 numpy 로 (표본마다 격자를 다시 훑지 않는다 — 소방차처럼
        # 표본이 많으면 최종 검사가 수십 초 걸렸다). 결과는 outside_m 을 표본마다 부른 것과 같다
        rids, A, B = self.net.segments_near(x, y, 15.0 + reach)
        if not rids:
            return max(self.outside_m(x + fx * ch - fy * sh, y + fx * sh + fy * ch) for fx, fy in self.p.body_samples)
        import numpy as np
        S = np.asarray(self.p.body_samples)
        P = np.stack([x + S[:, 0] * ch - S[:, 1] * sh, y + S[:, 0] * sh + S[:, 1] * ch], axis=1)
        A_, B_ = np.asarray(A), np.asarray(B)
        HW = np.asarray([self.half_w[r] for r in rids])
        D = B_ - A_
        L2 = np.maximum((D ** 2).sum(1), 1e-12)
        T = np.clip(((P[:, None, :] - A_[None]) * D[None]).sum(2) / L2[None], 0.0, 1.0)
        Q = A_[None] + T[..., None] * D[None]
        dist = np.hypot(P[:, None, 0] - Q[..., 0], P[:, None, 1] - Q[..., 1])
        return float((dist - HW[None]).min(axis=1).max())

    def check_route(self, route, *, check_radius: bool = True, label: str = "ROUTE") -> dict:
        """최종 주행 선형(다듬은 경로, 또는 PX4 미션 지점을 이은 선)을 차가 따라갈 때(뒤축이 선 위) 주행 가능한지.
        - 회전반경: 앞뒤 RADIUS_WINDOW_M 세 점 외접원 반경 < 최소 회전반경 → STEERING_LIMIT (속도를 낮춰도 못 도는 곳).
          최소 회전반경 이상이면 곡선 속도(√(횡가속 × R))로 해결되는 곳이라 위반이 아니다 (min_curve_speed_mps 로만 보고).
        - 차체: 둘레 표본(모서리 포함 0.45 m 간격)이 추정 도로 띠 합집합 안인지. 출발 자세·최종 정차 자세 포함.
        출발 도로도 예외 없이 본다. 예외는 self.p.body_length_m 의 초기 자세 예외 하나뿐이다 (아래).
        표본 검사라 근사다 — 표본 사이 연속 충돌을 보장하지 않는다.
        route: Route 또는 점 목록. {"ok", "min_radius_m", "steering_violations", "body_violations", "worst_excess_m",
        "initial_pose", "issues"}"""
        pts = route.pts if hasattr(route, "pts") else list(route)
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.dist(a, b))
        L = cum[-1]

        def at(sv):
            i = max(0, min(len(cum) - 2, bisect.bisect_left(cum, sv) - 1))
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg <= 0 else (sv - cum[i]) / seg
            a, b = pts[i], pts[i + 1]
            return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t

        issues, exempt, min_r, worst = [], [], math.inf, -math.inf
        n_rad = n_body = 0
        e0 = None
        exempt_open = True
        sv, last = 0.0, False
        while L > 0:
            q = at(sv)
            w = min(RADIUS_WINDOW_M, L / 2)
            lo = max(0.0, min(sv - w, L - 2 * w))
            a, c = at(lo), at(lo + 2 * w)
            # 진행 방향(접선)은 짧은 대칭 창으로 — 긴 창은 직선·원호 경계에서 차체 방향을 틀리게 잡는다
            ya, yc = at(max(0.0, sv - YAW_WINDOW_M)), at(min(L, sv + YAW_WINDOW_M))
            yaw = math.atan2(yc[1] - ya[1], yc[0] - ya[0]) if math.dist(ya, yc) > 1e-6 else 0.0
            R = math.inf
            if w >= 1.0:
                R = _radius3(a, at(lo + w), c)
                min_r = min(min_r, R)
                if check_radius and R < self.p.r_min * 0.98:
                    n_rad += 1
                    issues.append({"kind": "STEERING_LIMIT", "s_m": round(sv, 1), "x": round(q[0], 1), "y": round(q[1], 1),
                                   "value": round(R, 2)})
            ex = self.body_excess(q[0], q[1], yaw)
            worst = max(worst, ex)
            if e0 is None:
                e0 = ex
            if ex > CHECK_TOL_M:
                rec = {"kind": "BODY", "s_m": round(sv, 1), "x": round(q[0], 1), "y": round(q[1], 1), "value": round(ex, 2)}
                if exempt_open and e0 > CHECK_TOL_M and sv <= self.p.body_length_m and ex <= e0 + CHECK_TOL_M:
                    exempt.append(rec)
                else:
                    exempt_open = False
                    n_body += 1
                    issues.append(rec)
            else:
                exempt_open = False
            if last:
                break
            step = CHECK_FINE_STEP_M if (R < CHECK_FINE_R_M or ex > -0.5) else CHECK_STEP_M
            sv += step
            if sv >= L:
                sv, last = L, True                   # 최종 정차 자세는 반드시 본다
        curve_v = math.sqrt(self.p.lat_accel_mps2 * min_r) if not math.isinf(min_r) else None
        return {"ok": not issues, "label": label, "length_m": round(L, 1),
                "min_radius_m": None if math.isinf(min_r) else round(min_r, 2),
                "steering_violations": n_rad, "radius_violations": n_rad, "body_violations": n_body,
                "worst_excess_m": None if math.isinf(worst) else round(worst, 3),
                "min_curve_speed_mps": None if curve_v is None else round(curve_v, 2),
                "initial_pose": {"excess_m": None if e0 is None else round(e0, 3), "exempt_samples": len(exempt),
                                 "exempt_until_m": exempt[-1]["s_m"] if exempt else None,
                                 "rule": f"출발 자세 차체가 이미 도로 띠 밖일 때만, 출발 {self.p.body_length_m:g} m 안, "
                                         "그 초과량 이하 차체 표본만. 회전반경 예외 없음"},
                "issues": issues[:20],
                "sampling": {"step_m": CHECK_STEP_M, "fine_step_m": CHECK_FINE_STEP_M,
                             "fine_when": f"곡률반경 < {CHECK_FINE_R_M:g} m 또는 차체가 도로 띠 경계 0.5 m 안",
                             "body_sample_spacing_m": 0.45, "body_samples": len(self.p.body_samples), "profile": self.p.name},
                "basis": "뒤축이 선 위, 차체 둘레 표본 vs 추정 도로 띠 합집합(차로 수 추정 폭), 최소 회전반경 — 근사 표본 검사"}

    def blame(self, route: Route, issue: dict):
        """검사 실패 지점을 막을 단위: 가까운 교차로 회전 (지금 상태, 다음 상태) 또는 도로 id."""
        q = (issue["x"], issue["y"])
        best = None
        for (r1, f1, t1), (r2, f2, t2) in zip(route.legs, route.legs[1:]):
            d = math.dist(q, self.net.nodes[t1])
            if d <= TURN_TANGENT_MAX_M + 5 and (best is None or d < best[0]):
                best = (d, ("TURN", (r1, t1), (r2, t2)))
        if best:
            return best[1]
        p = self.net.snap(*q)
        return ("ROAD", p.road_id)

    # ------------------------------------------------------------------ 복귀 가능성
    def return_set(self, access: RoadPos, road_ok: Callable[[str], bool] = lambda r: True,
                   turn_ok: Callable[[State, State], bool] = lambda a, b: True):
        """출입 지점 access 까지 갈 수 있는 꼬리표(상태, 들어온 직전 상태) 집합 (정차 출발 — 첫 교차로 감속 조건 없음).
        회전표(차체 둘레 검사를 통과한 회전)만 쓰고, 짧은 도로의 연속 회전처럼 원호가 겹치는 조합은 쓰지 않는다 (_lab_ok).
        turn_ok/road_ok 로 최종 경로 검사에서 주행 불가로 확인된 회전·도로를 빼서 복귀 가능성의 근거로 다시 쓰이지 않게 한다."""
        r = self.net.roads[access.road_id]
        good = set()
        stack = []
        if road_ok(access.road_id):
            for to, frm_s in ((r.b, 0.0), (r.a, r.length)):
                s = (access.road_id, to)
                if not self.allowed(access.road_id, self.other(access.road_id, to)):
                    continue
                for pin in self.rev.get(s, ()):
                    if turn_ok(pin, s) and self.tan[(pin, s)] <= abs(access.s - frm_s):
                        good.add((s, pin))
                        stack.append((s, pin))
        while stack:
            t, s = stack.pop()                      # 도로 s 에서 t 로 도는 회전 다음은 복귀 가능
            rs_ = self.net.roads[s[0]]
            tout = self.tan[(s, t)]
            if not road_ok(s[0]):
                continue
            for q in self.rev.get(s, ()):
                lab = (s, q)
                if lab in good or not turn_ok(q, s) or not road_ok(q[0]):
                    continue
                if self._lab_ok(self.tan[(q, s)], tout, rs_.length):
                    good.add(lab)
                    stack.append(lab)
        return {"access": access, "labels": good}

    def can_return(self, rs, rid: str, s: float, toward: str) -> bool:
        """rid 의 s 에 toward 방향으로 서 있는 차가 출입 지점까지 갈 수 있나 (계획 시점, 보장 아님).
        다음 회전 원호가 차 앞에서 시작할 거리가 있어야 한다."""
        acc: RoadPos = rs["access"]
        r = self.net.roads[rid]
        if acc.road_id == rid and ((toward == r.b and acc.s >= s) or (toward == r.a and acc.s <= s)):
            return True
        ahead = r.length - s if toward == r.b else s
        st = (rid, toward)
        return any((t.to, st) in rs["labels"] and t.tangent_m <= ahead for t in self.turns.get(st, ()))

    # ------------------------------------------------------------------ 도로를 따르는 정지 경로
    def stop_path(self, start: DirPos, dist_m: float, prefer: Optional[List[Tuple[float, float]]] = None):
        """start 에서 진행 방향으로 도로를 따라 dist_m. prefer(남은 계획 경로 점)가 있으면 그것을 따른다.
        없으면 교차로마다 가장 덜 꺾이는 허용 회전으로 이어 간다. (점 목록, 실제 길이) — 길이가 모자랄 수 있다."""
        if prefer and len(prefer) >= 2:
            pts, acc = [prefer[0]], 0.0
            for p, q in zip(prefer, prefer[1:]):
                d = math.dist(p, q)
                if acc + d >= dist_m:
                    t = (dist_m - acc) / d if d > 0 else 0.0
                    pts.append((p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t))
                    return pts, dist_m
                pts.append(q)
                acc += d
            return pts, acc
        net = self.net
        rid, toward, s = start.road_id, start.toward, start.pos.s
        pts, acc = [], 0.0
        for _ in range(50):
            r = net.roads[rid]
            end_s = r.length if toward == r.b else 0.0
            seg = net.polyline(rid, s, end_s)
            for p, q in zip(seg, seg[1:]):
                if not pts:
                    pts.append(p)
                d = math.dist(p, q)
                if acc + d >= dist_m:
                    t = (dist_m - acc) / d if d > 0 else 0.0
                    pts.append((p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t))
                    return pts, dist_m
                pts.append(q)
                acc += d
            nxt = min(self.turns.get((rid, toward), ()), key=lambda t: t.deflect_deg, default=None)
            if nxt is None:
                return pts or [start.pos[2:4]], acc
            rid, toward = nxt.to
            r = net.roads[rid]
            s = 0.0 if toward == r.b else r.length
        return pts, acc


def heading_of_yaw(yaw_enu_rad: float) -> float:
    """Gazebo ENU yaw(동=0, 반시계) → 방위(북=0, 시계)."""
    return (90.0 - math.degrees(yaw_enu_rad)) % 360.0


STATION_SEARCH_M = 1000.0        # 소방서 좌표에서 출입 지점을 찾는 거리 (이보다 멀면 배치 불가 — 임의의 먼 도로에 두지 않는다)
SLOT_GAP_M = 6.0                 # 같은 거점 대기 위치 사이 차체 간 간격 (같은 도로 한 줄 대기일 때)
SLOT_END_M = 35.0                # 대기 위치는 도로 끝(교차로)에서 이만큼 떨어진 곳 (15 m 일 때 교차로를 지나는 차가 서 있는 차에 막힘)


def access_point(net: RoadNetwork, base_id: str, search_m: float = STATION_SEARCH_M, graph=None, slot: int = 0,
                 slot_lengths: Optional[List[float]] = None):
    """소속 거점의 차량 출입 지점(대기 위치): 소방서 좌표(environment/config/fire_stations.json)에서 가까운 도로 지점 중
    양방향 통행이고 두 방향 모두 가장 큰 강연결 성분(서로 오갈 수 있는 묶음)에 들며, 이 차량 특성(graph.p)으로 급커브가
    아닌 곳. 소방서 좌표와 따로 둔다 — 좌표에서 가장 가까운 도로가 막다른 길일 수 있다 (인제119: 약 44 m 떨어진 도로가 막다른 길).
    진행 방향은 그 도로의 a → b.
    slot: 같은 거점의 k 번째 차량 — 조건 맞는 **서로 다른 도로** 중 k 번째로 가까운 도로 (2026-10-10). 같은 도로에 한 줄로
    세우면 중심선 주행이라 서로 비켜 갈 수 없어, 반대 방향으로 돌아온 앞 차가 뒤 차에 막혔다 (Gazebo 4대 시험).
    다른 도로가 모자라면 0 번 도로에서 진행 방향 뒤로 (앞 차량들 길이 + 간격) 물린 자리 (slot_lengths: 앞 차량 길이).
    (RoadPos, 방위°, 설명) 또는 None (search_m 안에 조건 맞는 도로 없음 — 배치 불가)"""
    st = config.load_station(base_id)
    from .geo import FRAME
    sx, sy = FRAME.to_xy(st["lat"], st["lon"])
    g = graph or net.graph
    best = {}                                       # 도로 → (거리, 지점)
    for p in net.samples() + [net.snap(sx, sy)]:
        d = math.hypot(p.x - sx, p.y - sy)
        if d > search_m:
            continue
        r = net.roads[p.road_id]
        if r.attrs.get("oneway", "BOTH") != "BOTH" or p.road_id in g.sharp_roads or r.length < 2 * SLOT_END_M:
            continue
        if not ((p.road_id, r.a) in g.core and (p.road_id, r.b) in g.core):
            continue
        if p.road_id not in best or d < best[p.road_id][0]:
            best[p.road_id] = (d, p)
    if not best:
        return None
    ranked = sorted(best.values(), key=lambda v: v[0])
    if slot > 0:
        # 두 번째부터는 그 도로를 막아도(차가 서 있으면 다른 차는 그 도로로 계획하지 않는다) 도로망이 끊기지 않는 도로만
        first = ranked[0][1]
        ranked = [ranked[0]] + [v for v in ranked[1:] if g.road_is_spare(v[1].road_id, keep=first.road_id)
                                and g.detour_ok(v[1].road_id, first)]
    note = ""
    if slot < len(ranked):
        d, p = ranked[slot]
        r = net.roads[p.road_id]
        s_ = min(max(p.s, SLOT_END_M), r.length - SLOT_END_M)          # 교차로 원호 구간을 피해 도로 끝에서 떨어진 곳
        p = net.at(p.road_id, s_)
        if slot:
            note = f", 대기 위치 {slot} (같은 거점 {slot + 1}번째 도로)"
    else:
        d, p = ranked[0]
        own = g.p.body_length_m
        k = slot - len(ranked) + 1
        lens = list(slot_lengths or [own] * k)
        back = lens[0] / 2 + sum(lens[1:k]) + SLOT_GAP_M * k + own / 2
        if p.s - back < 5.0:
            return None
        p = net.at(p.road_id, p.s - back)
        note = f", 대기 위치 {slot} (다른 도로 부족 — 첫 도로 출입 지점 뒤 {back:.1f} m, 한 줄 대기)"
    r = net.roads[p.road_id]
    q0 = net.at(p.road_id, max(0.0, p.s - 2.0))[2:4]
    q1 = net.at(p.road_id, min(r.length, p.s + 2.0))[2:4]
    return p, bearing_deg(q1[0] - q0[0], q1[1] - q0[1]),         f"소방서 {st['base_id']} 좌표에서 {d:.0f} m, 서로 오갈 수 있는 도로 {p.road_id} (a→b 방향){note}"
