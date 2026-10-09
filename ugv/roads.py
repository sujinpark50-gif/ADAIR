# -*- coding: utf-8 -*-
"""ugv/roads.py — 정적 도로망: 도로 위 위치, 도로 표본, 구간별 속도 계획.

자료: 원자료 GeoPackage 를 바로 읽는다 (ugv/road_source.py, config.ROAD_GPKG). 경로 탐색은 진행 방향을 지키는
ugv/drive_graph.py 가 한다. 교통 혼잡·동적 장애물은 다루지 않는다 (범위 밖).

좌표: 지역 평면 (ugv/geo.py FRAME, m). 위치는 RoadPos(도로, 도로 시작 노드 a 에서 선형을 따라 잰 거리 s).
도로 속도 = min(max(제한속도, MIN_ROAD_SPEED_KMH), 차량 최고속도).
"""

import bisect
import math
from dataclasses import dataclass, field
from typing import Dict, List, NamedTuple, Optional, Tuple

from . import config
from .geo import FRAME, bearing_deg


class RoadPos(NamedTuple):
    road_id: str
    s: float            # 도로 node_a 에서 선형을 따라 잰 거리 (m)
    x: float
    y: float
    dist: float = 0.0   # 질의 지점 ~ 이 도로 위 지점 거리 (m)

    def ll(self):
        return FRAME.to_ll(self.x, self.y)


@dataclass
class Road:
    road_id: str
    a: str
    b: str
    name: str
    speed_kmh: float
    xy: List[Tuple[float, float]]
    cum: List[float]
    attrs: dict = field(default_factory=dict)      # 원자료 속성 (oneway, lanes ...) — ugv/road_source.py

    @property
    def length(self) -> float:
        return self.cum[-1]


@dataclass
class Route:
    pts: List[Tuple[float, float]]
    vlim: List[float]                 # 각 점이 속한 도로의 속도 상한 (m/s)
    road_ids: List[str]
    start: RoadPos
    end: RoadPos
    time_s: float                     # 도로 속도 기준 시간 (속도 계획 전)
    length: float = field(init=False)
    legs: List[Tuple[str, str, str]] = field(default_factory=list)   # (road_id, 출발 노드, 향하는 노드) — 방향 경로만
    arrive_toward: Optional[str] = None                              # 도착할 때 향하던 노드 (도착 방향)

    def __post_init__(self):
        self.length = sum(math.dist(p, q) for p, q in zip(self.pts, self.pts[1:]))


@dataclass
class SpeedPlan:
    pts: List[Tuple[float, float]]    # 5 m 이하 간격
    v: List[float]                    # 각 점 계획 속도 (m/s)
    cum: List[float]
    eta_s: float
    items: List[Tuple[float, float, float]]   # PX4 미션 지점 (x, y, 그 지점까지 구간 속도)
    v_max: float


class RoadNetwork:
    def __init__(self, path=None):
        from .road_source import load
        doc = load(path)
        self.source = doc["source"]
        self.excluded = doc["excluded"]
        self.clip_extent = doc["clip_extent"]
        self.nodes: Dict[str, Tuple[float, float]] = {n: FRAME.to_xy(*ll) for n, ll in doc["nodes"].items()}
        self.roads: Dict[str, Road] = {}
        self.adj: Dict[str, List[Tuple[str, str]]] = {}
        for r in doc["roads"]:
            xy = [FRAME.to_xy(la, lo) for la, lo in r["geometry"]]
            cum = [0.0]
            for p, q in zip(xy, xy[1:]):
                cum.append(cum[-1] + math.dist(p, q))
            road = Road(r["road_id"], r["node_a"], r["node_b"], r["name"], float(r["speed_kmh"]), xy, cum, r["attrs"])
            if road.length <= 0 or road.a == road.b:
                continue
            self.roads[road.road_id] = road
            self.adj.setdefault(road.a, []).append((road.road_id, road.b))
            self.adj.setdefault(road.b, []).append((road.road_id, road.a))
        self._graph = None
        self._seg_index: Dict[Tuple[int, int], List[Tuple[str, int]]] = {}
        for rid, road in self.roads.items():
            for i, (p, q) in enumerate(zip(road.xy, road.xy[1:])):
                for key in self._cells_of_segment(p, q):
                    self._seg_index.setdefault(key, []).append((rid, i))
        self._samples = None

    @property
    def graph(self):
        """방향 경로 그래프 (진행 방향 유지·회전 가능성). 처음 쓸 때 만든다."""
        if self._graph is None:
            from .drive_graph import DriveGraph
            self._graph = DriveGraph(self)
        return self._graph

    # ------------------------------------------------------------------ 위치
    _CELL = 200.0

    def _cells_of_segment(self, p, q):
        x0, x1 = sorted((p[0], q[0]))
        y0, y1 = sorted((p[1], q[1]))
        for i in range(int(x0 // self._CELL), int(x1 // self._CELL) + 1):
            for j in range(int(y0 // self._CELL), int(y1 // self._CELL) + 1):
                yield i, j

    def road_speed(self, road: Road) -> float:
        if config.MIN_ROAD_SPEED_KMH <= 0:
            return config.MAX_SPEED_MPS
        return min(max(road.speed_kmh, config.MIN_ROAD_SPEED_KMH) / 3.6, config.MAX_SPEED_MPS)

    def snap(self, x: float, y: float, max_m: float = math.inf) -> Optional[RoadPos]:
        """(x, y) 에서 가장 가까운 도로 위 지점. max_m 안에 없으면 None."""
        best = None
        r = self._CELL
        while True:
            ci, cj, k = int(x // self._CELL), int(y // self._CELL), int(r // self._CELL)
            seen = set()
            for i in range(ci - k, ci + k + 1):
                for j in range(cj - k, cj + k + 1):
                    for rid, si in self._seg_index.get((i, j), ()):
                        if (rid, si) in seen:
                            continue
                        seen.add((rid, si))
                        road = self.roads[rid]
                        (ax, ay), (bx, by) = road.xy[si], road.xy[si + 1]
                        dx, dy = bx - ax, by - ay
                        L2 = dx * dx + dy * dy
                        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
                        px, py = ax + t * dx, ay + t * dy
                        d = math.hypot(x - px, y - py)
                        if best is None or d < best.dist:
                            best = RoadPos(rid, road.cum[si] + t * math.sqrt(L2), px, py, d)
            if (best is not None and best.dist <= r - self._CELL) or r >= max_m or r > 64000:
                break
            r *= 2
        if best is None or best.dist > max_m:
            return None
        return best

    def near(self, x: float, y: float, max_m: float) -> Dict[str, float]:
        """(x, y) 에서 max_m 안에 있는 도로들과 그 중심선까지 거리."""
        out: Dict[str, float] = {}
        k = int(max_m // self._CELL) + 1
        ci, cj = int(x // self._CELL), int(y // self._CELL)
        for i in range(ci - k, ci + k + 1):
            for j in range(cj - k, cj + k + 1):
                for rid, si in self._seg_index.get((i, j), ()):
                    road = self.roads[rid]
                    (ax, ay), (bx, by) = road.xy[si], road.xy[si + 1]
                    dx, dy = bx - ax, by - ay
                    L2 = dx * dx + dy * dy
                    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
                    d = math.hypot(x - ax - t * dx, y - ay - t * dy)
                    if d <= max_m and d < out.get(rid, math.inf):
                        out[rid] = d
        return out

    def snap_ll(self, lat: float, lon: float, max_m: float = math.inf) -> Optional[RoadPos]:
        return self.snap(*FRAME.to_xy(lat, lon), max_m=max_m)

    def at(self, road_id: str, s: float) -> RoadPos:
        road = self.roads[road_id]
        s = max(0.0, min(road.length, s))
        i = max(0, min(len(road.cum) - 2, bisect.bisect_right(road.cum, s) - 1))
        seg = road.cum[i + 1] - road.cum[i]
        t = 0.0 if seg == 0 else (s - road.cum[i]) / seg
        (ax, ay), (bx, by) = road.xy[i], road.xy[i + 1]
        return RoadPos(road_id, s, ax + t * (bx - ax), ay + t * (by - ay))

    def polyline(self, road_id: str, s0: float, s1: float) -> List[Tuple[float, float]]:
        """도로 위 s0 → s1 구간 선형 (양 끝 포함, 방향 그대로)."""
        road = self.roads[road_id]
        lo, hi = sorted((s0, s1))
        inner = [road.xy[i] for i in range(len(road.cum)) if lo < road.cum[i] < hi]
        pts = [self.at(road_id, lo)[2:4]] + inner + [self.at(road_id, hi)[2:4]]
        return pts if s0 <= s1 else pts[::-1]

    def samples(self) -> List[RoadPos]:
        """모든 도로를 ROAD_SAMPLE_M 간격으로 나눈 점 (양 끝 포함). 접근 후보·안전 검사에 쓴다."""
        if self._samples is None:
            out = []
            for rid, road in self.roads.items():
                n = max(1, math.ceil(road.length / config.ROAD_SAMPLE_M))
                out.extend(self.at(rid, road.length * k / n) for k in range(n + 1))
            self._samples = out
        return self._samples

    # 경로 탐색은 진행 방향을 지키는 ugv/drive_graph.py 만 쓴다 (이전 무방향 탐색은 U턴·일방통행 역주행을 허용해서 지웠다)


# ---------------------------------------------------------------------- 속도 계획
def _densify(pts, vlim, step=5.0):
    out, ov = [pts[0]], [vlim[0]]
    for (p, q), v in zip(zip(pts, pts[1:]), vlim[1:]):
        d = math.dist(p, q)
        n = max(1, math.ceil(d / step))
        for k in range(1, n + 1):
            out.append((p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n))
            ov.append(v)
    return out, ov


def _curvature(pts, cum, i, w):
    j = bisect.bisect_left(cum, cum[i] - w)
    k = min(len(pts) - 1, bisect.bisect_left(cum, cum[i] + w))
    if j >= i or k <= i:
        return 0.0
    a, b, c = pts[j], pts[i], pts[k]
    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    den = math.dist(a, b) * math.dist(b, c) * math.dist(a, c)
    return 0.0 if den <= 1e-9 else 2.0 * area2 / den


def plan_speeds(route: Route, v0: float = 0.0, *, lat_accel=None, accel=None, decel=None,
                spacing=None, turn_deg=None) -> SpeedPlan:
    """곡선·도착 구간 감속을 반영한 속도 계획과 PX4 미션 지점."""
    lat_accel = lat_accel or config.LAT_ACCEL_MPS2
    accel, decel = accel or config.ACCEL_MPS2, decel or config.DECEL_MPS2
    spacing, turn_deg = spacing or config.ITEM_SPACING_M, turn_deg or config.ITEM_TURN_DEG
    pts, vl = _densify(route.pts, route.vlim)
    cum = [0.0]
    for p, q in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist(p, q))
    v = []
    for i in range(len(pts)):
        k = _curvature(pts, cum, i, config.CURVE_WINDOW_M)
        v.append(min(vl[i], math.sqrt(lat_accel / k) if k > 1e-6 else math.inf))
    v[0] = min(v[0], max(v0, 0.0)) if v0 > 0 else 0.0
    v[-1] = 0.0
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * accel * (cum[i] - cum[i - 1])))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * decel * (cum[i + 1] - cum[i])))
    eta = 0.0
    for i in range(1, len(v)):
        eta += (cum[i] - cum[i - 1]) / max((v[i] + v[i - 1]) / 2, 0.5)
    # PX4 미션 지점: 간격 상한 + 꺾이는 곳. 지점 속도 = 앞 지점부터 이 지점까지 계획 속도의 최소 (구간에 들어가기 전에 낮춘다)
    items, last, last_hd = [], 0, None
    for i in range(1, len(pts)):
        hd = bearing_deg(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) if math.dist(pts[i], pts[i - 1]) > 0 else last_hd
        turn = 0.0 if last_hd is None or hd is None else abs((hd - last_hd + 180) % 360 - 180)
        if i == len(pts) - 1 or cum[i] - cum[last] >= spacing or turn >= turn_deg:
            seg_v = min(v[last + 1:i + 1]) if i > last else v[i]
            if i == len(pts) - 1:
                seg_v = min(v[last + 1:i]) if i - 1 > last else 1.0
            items.append((pts[i][0], pts[i][1], max(math.floor(seg_v * 100) / 100, 1.0)))
            last, last_hd = i, hd
        elif last_hd is None:
            last_hd = hd
    return SpeedPlan(pts, v, cum, eta, items, max(v) if v else 0.0)
