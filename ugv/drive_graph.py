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

회전 검사 (근사): 뒤축 중심이 두 도로 중심선에 접하는 원호(반경 R)를 따라 돈다고 보고, 차체 네 모서리가 그 노드에 붙은
모든 도로 띠(중심선 ± 추정 반폭) 안에 있는지 본다. R 은 최소 회전반경부터 넓혀 가며 맞는 것 중 가장 큰 값을 회전 속도
계산에 쓴다 (v = √(LAT_ACCEL × R)). 교차로 확장면·차로 위치(우측 통행)는 반영하지 않는다.
도로 폭은 원자료 차로 수 × LANE_WIDTH_EST_M + 길어깨로 추정한 값이다 (원자료 WIDTH 는 1~4 코드라 쓰지 않는다).
"""

import heapq
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from . import config
from .geo import bearing_deg
from .roads import Road, RoadNetwork, RoadPos, Route

State = Tuple[str, str]          # (road_id, 향하는 노드)

R_MIN = config.WHEELBASE_M / math.tan(math.radians(config.STEER_MAX_DEG))
_FRONT = config.REAR_AXLE_FROM_CENTER_M + config.BODY_LENGTH_M / 2      # 뒤축에서 앞 범퍼
_REAR = config.BODY_LENGTH_M / 2 - config.REAR_AXLE_FROM_CENTER_M        # 뒤축에서 뒤 범퍼
_HW = config.BODY_WIDTH_M / 2
CORNERS = ((_FRONT, _HW), (_FRONT, -_HW), (-_REAR, _HW), (-_REAR, -_HW))
R_STEPS = (1.0, 1.15, 1.3, 1.5, 1.8, 2.2, 3.0, 4.0, 6.0, 9.0, 14.0, 20.0, 30.0)
STRAIGHT_DEG = 5.0
STRIP_LEN_M = 60.0
DIR_LEN_M = 15.0
TURN_TANGENT_MAX_M = 25.0       # 교차로 회전 원호가 노드 앞뒤로 차지하는 길이 상한
BEND_R_M = 50.0                 # 도로 선형의 꺾인 점(교차로 아님)을 다듬는 원호 반경 목표
BEND_TANGENT_SHARE = 0.45       #   이웃 선분 길이의 이 비율까지만 쓴다
ARC_STEP_M = 2.0                # 원호 표본 간격


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
    state: Optional[State]      # VIA 일 때 목적지 도로에 들어선 상태
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
    def __init__(self, net: RoadNetwork):
        self.net = net
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
        for s, ts in self.turns.items():
            for t in ts:
                self.rev.setdefault(t.to, []).append(s)

    # ------------------------------------------------------------------ 정적 속성
    def oneway_ok(self, rid: str, frm: str) -> bool:
        r = self.net.roads[rid]
        ow = r.attrs.get("oneway", "BOTH")
        if ow == "A_TO_B":
            return frm == r.a
        if ow == "B_TO_A":
            return frm == r.b
        return True

    def allowed(self, rid: str, frm: str) -> bool:
        """도로 rid 에 frm 노드에서 들어가 달릴 수 있나 (일방통행·급커브)."""
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

    @staticmethod
    def _bend_ok(R: float, half_w: float) -> bool:
        if math.isinf(R):
            return _HW <= half_w
        if R < R_MIN:
            return False
        outer = math.hypot(R + _HW, _FRONT) - R          # 바깥 앞모서리가 중심선 밖으로 나가는 거리
        return outer <= half_w and _HW <= half_w

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
                if _seg_dist(q, a, b) <= h:
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
            R = R_MIN * f
            T = R * math.tan(ad / 2)
            if T > min(Lin, STRIP_LEN_M, TURN_TANGENT_MAX_M) + 1 or T > min(Lout, STRIP_LEN_M, TURN_TANGENT_MAX_M) + 1:
                continue
            P1 = (-T * u[0], -T * u[1])
            C = (P1[0] - u[1] * sg * R, P1[1] + u[0] * sg * R)
            ok = True
            for k in range(13):
                hd = math.atan2(u[1], u[0]) + sg * ad * k / 12
                ch, sh = math.cos(hd), math.sin(hd)
                rx, ry = C[0] + R * sh * sg, C[1] - R * ch * sg
                for fx, fy in CORNERS:
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
                    vmax = min(net.road_speed(net.roads[rin]), net.road_speed(net.roads[rout]))
                    v = vmax if math.isinf(R) else min(vmax, math.sqrt(config.LAT_ACCEL_MPS2 * R))
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
        """진행 방향으로 d m (출발 도로 안에서만. 노드 직전에서 멈춘다)."""
        r = self.net.roads[dp.road_id]
        d = max(0.0, min(d, self.ahead_m(dp) - 0.1))
        s = dp.pos.s + d if dp.toward == r.b else dp.pos.s - d
        return DirPos(self.net.at(dp.road_id, s), dp.toward)

    def outside_m(self, x: float, y: float) -> float:
        """점이 추정 도로 띠(근처 모든 도로: 중심선 ± 추정 반폭)의 합집합 밖으로 나간 거리. 안이면 0 이하."""
        near = self.net.near(x, y, 15.0)
        if not near:
            p = self.net.snap(x, y)
            return p.dist - self.half_w[p.road_id]
        return min(d - self.half_w[rid] for rid, d in near.items())

    @staticmethod
    def brake_need(v0: float, v1: float) -> float:
        if v0 <= v1:
            return 0.0
        return (v0 * v0 - v1 * v1) / (2 * config.DECEL_MPS2) + config.TURN_MARGIN_M

    # ------------------------------------------------------------------ 탐색
    def search(self, start: DirPos, v0: float = 0.0, road_ok: Callable[[str], bool] = lambda r: True):
        """출발 방향을 유지하는 최소 시간 탐색. 상태 (도로, 향하는 노드) 의 값 = 그 도로에 들어선 시각.
        출발 도로의 남은 구간은 road_ok 와 관계없이 지난다 (차량이 거기 있다)."""
        net = self.net
        r0 = net.roads[start.road_id]
        d0 = self.ahead_m(start)
        t0 = d0 / net.road_speed(r0)
        dist: Dict[State, float] = {}
        prev: Dict[State, Optional[State]] = {}
        pq = []
        for t in self.turns.get((start.road_id, start.toward), ()):
            if not road_ok(t.to[0]):
                continue
            if d0 < self.brake_need(v0, t.v_turn):        # 첫 교차로: 회전 속도까지 감속할 거리가 없다
                continue
            if t0 < dist.get(t.to, math.inf):
                dist[t.to], prev[t.to] = t0, None
                heapq.heappush(pq, (t0, t.to))
        done = set()
        while pq:
            T, s = heapq.heappop(pq)
            if s in done:
                continue
            done.add(s)
            r = net.roads[s[0]]
            Tn = T + r.length / net.road_speed(r)
            for t in self.turns.get(s, ()):
                if not road_ok(t.to[0]):
                    continue
                if Tn < dist.get(t.to, math.inf):
                    dist[t.to], prev[t.to] = Tn, s
                    heapq.heappush(pq, (Tn, t.to))
        return {"start": start, "v0": v0, "dist": dist, "prev": prev}

    def arrivals(self, tree, p: RoadPos, seg_ok: Callable[[str, float, float], bool] = lambda r, a, b: True) -> List[Arrival]:
        """p 에 도착하는 방법들 (방향별 가장 빠른 것). seg_ok(road, s_from, s_to): 목적지 도로에서 지나는 구간 안전."""
        net = self.net
        st: DirPos = tree["start"]
        r = net.roads[p.road_id]
        v = net.road_speed(r)
        out = []
        if p.road_id == st.road_id:
            d = (p.s - st.pos.s) if st.toward == r.b else (st.pos.s - p.s)
            if d >= self.brake_need(tree["v0"], 0.0) - config.TURN_MARGIN_M and d > 0 and seg_ok(p.road_id, st.pos.s, p.s):
                out.append(Arrival(d / v, "DIRECT", None, st.toward))
        for toward, frm_s in ((r.b, 0.0), (r.a, r.length)):
            s = (p.road_id, toward)
            if s in tree["dist"] and seg_ok(p.road_id, frm_s, p.s):
                out.append(Arrival(tree["dist"][s] + abs(p.s - frm_s) / v, "VIA", s, toward))
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
            add(net.polyline(st.road_id, st.pos.s, p.s), net.road_speed(r0), st.road_id, frm0, st.toward)
        else:
            chain = []
            s = arr.state
            while s is not None:
                chain.append(s)
                s = tree["prev"][s]
            chain.reverse()
            add(net.polyline(st.road_id, st.pos.s, r0.length if st.toward == r0.b else 0.0), net.road_speed(r0),
                st.road_id, frm0, st.toward)
            for rid, to in chain[:-1]:
                r = net.roads[rid]
                add(net.polyline(rid, 0.0, r.length) if to == r.b else net.polyline(rid, r.length, 0.0),
                    net.road_speed(r), rid, self.other(rid, to), to)
            rid, to = chain[-1]
            r = net.roads[rid]
            add(net.polyline(rid, 0.0 if to == r.b else r.length, p.s), net.road_speed(r), rid, self.other(rid, to), to)
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
        - 교차로: 회전표 반경(차체가 그 노드 도로 띠 안에 들어온다고 검사한 값)으로, 노드 앞뒤 tangent_m
        - 그 밖의 꺾인 점: 반경 BEND_R_M 목표, 이웃 선분의 BEND_TANGENT_SHARE 까지
        원호는 3차 베지어로 근사해 ARC_STEP_M 간격으로 표본을 둔다. 겹치는 원호는 줄인다."""
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.dist(a, b))
        L = cum[-1]

        def dir_at(i):
            a, b = pts[i], pts[i + 1]
            d = math.dist(a, b) or 1.0
            return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)

        def defl(i):
            u, w = dir_at(i - 1), dir_at(i)
            return abs(math.atan2(u[0] * w[1] - u[1] * w[0], u[0] * w[0] + u[1] * w[1]))

        # 교차로 꼭짓점: 연속한 구간 사이 노드 위치와 같은 점
        junction_T = {}
        for (r1, _, t1), (r2, _, t2) in zip(legs, legs[1:]):
            turn = next((t for t in self.turns.get((r1, t1), ()) if t.to == (r2, t2)), None)
            if turn is None or turn.radius_m is None:
                continue
            nx, ny = self.net.nodes[t1]
            j = min(range(1, len(pts) - 1), key=lambda i: (pts[i][0] - nx) ** 2 + (pts[i][1] - ny) ** 2, default=None)
            if j is not None and math.hypot(pts[j][0] - nx, pts[j][1] - ny) < 0.5:
                junction_T[j] = turn.tangent_m
        corners = []
        for j in range(1, len(pts) - 1):
            if math.dist(pts[j - 1], pts[j]) < 1e-6 or math.dist(pts[j], pts[j + 1]) < 1e-6:
                continue
            if j in junction_T:
                corners.append([cum[j], junction_T[j], True])
                continue
            D = defl(j)
            if math.degrees(D) < 2.0:
                continue
            # 꼭짓점에서 원호까지 거리 e = T·tan(D/4) 를 도로 반폭 − 차체 반폭 − 여유 안으로
            hw = self.half_w[self.net.snap(*pts[j]).road_id]
            e_max = max(0.3, hw - _HW - 0.3)
            T = min(BEND_R_M * math.tan(D / 2),
                    BEND_TANGENT_SHARE * min(cum[j] - cum[j - 1], cum[j + 1] - cum[j]),
                    e_max / max(math.tan(D / 4), 1e-9))
            corners.append([cum[j], T, False])
        # 교차로 원호 안에 든 일반 꺾인 점은 교차로 원호에 흡수
        kept = []
        for c in corners:
            if not c[2] and any(o[2] and abs(o[0] - c[0]) < o[1] for o in corners):
                continue
            kept.append(c)
        # 겹침 제거 (일반 꺾인 점을 먼저 줄인다)
        for a, b in zip(kept, kept[1:]):
            over = a[1] + b[1] - (b[0] - a[0])
            if over > 0:
                if not a[2] and not b[2] or a[2] == b[2]:
                    k = (b[0] - a[0]) / max(a[1] + b[1], 1e-9)
                    a[1] *= k
                    b[1] *= k
                elif not b[2]:
                    b[1] = max(0.0, b[1] - over)
                else:
                    a[1] = max(0.0, a[1] - over)
        kept = [c for c in kept if c[1] > 0.3 and c[0] - c[1] > 0 and c[0] + c[1] < L]

        def at(sv):
            i = max(0, min(len(cum) - 2, next((k for k in range(len(cum) - 1) if cum[k + 1] >= sv), len(cum) - 2)))
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg <= 0 else (sv - cum[i]) / seg
            a, b = pts[i], pts[i + 1]
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t), i

        out, ov, s_done = [pts[0]], [vlim[0]], 0.0
        for s0, T, _ in kept:
            (p1, i1), (p2, i2) = at(s0 - T), at(s0 + T)
            for k in range(len(pts)):
                if s_done < cum[k] < s0 - T:
                    out.append(pts[k])
                    ov.append(vlim[k])
            u, w = dir_at(i1), dir_at(min(i2, len(pts) - 2))
            D = abs(math.atan2(u[0] * w[1] - u[1] * w[0], u[0] * w[0] + u[1] * w[1]))
            chord = math.dist(p1, p2)
            v = min(vlim[max(0, i1):i2 + 2])
            if D < math.radians(1.0) or chord < 1e-6:
                out.append(p1); ov.append(v)
            else:
                R = chord / (2 * math.sin(D / 2))
                h = 4.0 / 3.0 * math.tan(D / 4) * R
                c1 = (p1[0] + h * u[0], p1[1] + h * u[1])
                c2 = (p2[0] - h * w[0], p2[1] - h * w[1])
                n = max(2, math.ceil(R * D / ARC_STEP_M))
                for k in range(n):
                    t = k / n
                    a, b, c, d = (1 - t) ** 3, 3 * (1 - t) ** 2 * t, 3 * (1 - t) * t * t, t ** 3
                    out.append((a * p1[0] + b * c1[0] + c * c2[0] + d * p2[0], a * p1[1] + b * c1[1] + c * c2[1] + d * p2[1]))
                    ov.append(v)
            out.append(p2)
            ov.append(v)
            s_done = s0 + T
        for k in range(len(pts)):
            if cum[k] > s_done:
                out.append(pts[k])
                ov.append(vlim[k])
        # 중복 점 제거
        fp, fv = [out[0]], [ov[0]]
        for q, v in zip(out[1:], ov[1:]):
            if math.dist(fp[-1], q) >= 0.05:
                fp.append(q)
                fv.append(v)
        if math.dist(fp[-1], pts[-1]) > 0.05:
            fp.append(pts[-1])
            fv.append(vlim[-1])
        return fp, fv

    # ------------------------------------------------------------------ 복귀 가능성
    def return_set(self, access: RoadPos, road_ok: Callable[[str], bool] = lambda r: True):
        """출입 지점 access 까지 갈 수 있는 상태 집합 (정차 상태에서 출발 — 첫 교차로 감속 조건 없음)."""
        r = self.net.roads[access.road_id]
        seeds = [(access.road_id, to) for to in (r.a, r.b)
                 if self.allowed(access.road_id, self.other(access.road_id, to)) and road_ok(access.road_id)]
        seen = set(seeds)
        stack = list(seeds)
        while stack:
            s = stack.pop()
            for p in self.rev.get(s, ()):
                if p not in seen and road_ok(p[0]):
                    seen.add(p)
                    stack.append(p)
        return {"access": access, "states": seen}

    def can_return(self, rs, rid: str, s: float, toward: str) -> bool:
        """rid 의 s 에 toward 방향으로 서 있는 차가 출입 지점까지 갈 수 있나."""
        acc: RoadPos = rs["access"]
        r = self.net.roads[rid]
        if acc.road_id == rid and ((toward == r.b and acc.s >= s) or (toward == r.a and acc.s <= s)):
            return True
        return any(t.to in rs["states"] for t in self.turns.get((rid, toward), ()))

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


def access_point(net: RoadNetwork, base_id: str):
    """소속 거점의 차량 출입 지점 (도로 위). 소방서 좌표와 따로 둔다 — 소방서 좌표에서 가장 가까운 도로가 막다른 길일 수
    있다 (인제119: 43.5 m 떨어진 682503090 은 176 m 막다른 도로). 도로망의 거점 노드(nodes[base_id], 빌더가 소방서 근처
    교차 도로에 넣은 노드)와 그 거점 스폰 방향을 쓴다. (RoadPos, 방위°, 출처)"""
    if base_id not in net.nodes or not net.adj.get(base_id):
        return None
    yaw = next((sp["yaw"] for sp in net.spawn.values() if sp.get("node") == base_id), None)
    hd = heading_of_yaw(yaw) if yaw is not None else 0.0
    hx, hy = math.sin(math.radians(hd)), math.cos(math.radians(hd))
    g = net.graph
    # 거점 노드는 교차점이라 그 위에서는 방향이 모호하다 → 스폰 방향과 가장 잘 맞게 거점 노드에서 나가는 도로의 시작점
    rid, _ = max(net.adj[base_id], key=lambda e: (g.oneway_ok(e[0], base_id),
                                                   sum(a * b for a, b in zip(g._dir(e[0], base_id, out=True), (hx, hy)))))
    r = net.roads[rid]
    pos = net.at(rid, min(1.0, r.length / 2) if r.a == base_id else max(r.length - 1.0, r.length / 2))
    w = g._dir(rid, base_id, out=True)
    return pos, bearing_deg(*w), f"road_network.json nodes[{base_id}] 에서 나가는 도로 {rid} (spawn 방향 {hd:.0f}° 에 가장 가까움)"
