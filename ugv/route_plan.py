# ugv/route_plan.py — 노드 경로 → 도로를 따라가는 주행 웨이포인트
#
# 경로 탐색(road_graph)은 노드 단위다. 노드 사이를 직선으로 이으면 굽은 도로에서 산으로 올라간다
# (강원 도로망: 링크 756개 중 41개가 곡률비 1.2 이상). 그래서 도로 선형(road_network.json 의 geometry)을
# 펼쳐 웨이포인트로 쓰고, 도로마다 속도(min(도로, 차량) / 혼잡)를 붙인다.
#
# 웨이포인트는 WAYPOINT_GAP_M 보다 가까운 점을 솎는다. PX4 rover 는 도착 반경(10 m) 안에
# 다음 점이 있으면 점을 건너뛰거나 도착 판정이 꼬인다 (ugv/drivers/px4.py).
# 솎은 결과는 다시 솎아도 그대로라, 드라이버가 한 번 더 솎아도 진행률 번호가 어긋나지 않는다.

import math
from dataclasses import dataclass, field

from .geo import distance_m, to_ned

WAYPOINT_GAP_M = 8.0      # = 2 × PX4 도착 반경(ACCEPT_RADIUS_M 4 m). 도로 모양 점을 덜 솎아 굽은 길을 더 잘 따라간다


@dataclass
class RoutePlan:
    target_node: str
    path: list[str]                                   # 노드 id, 출발 노드 포함
    eta_s: float                                      # 시뮬레이션 초
    start: tuple[float, float]                        # 출발 좌표 (웨이포인트에는 없다)
    waypoints: list[tuple[float, float]] = field(default_factory=list)   # 출발점 제외
    speeds: list[float] = field(default_factory=list)                    # waypoints[i] 로 가는 구간 속도 m/s
    leg_of: list[int] = field(default_factory=list)                      # waypoints[i] 가 속한 구간 번호
    legs: list[dict] = field(default_factory=list)                       # road_graph.legs 결과

    @property
    def polyline(self) -> list[tuple[float, float]]:
        """이탈 판정용 선형 (솎기 전 도로 선형 전체)."""
        pts = [self.start]
        for leg in self.legs:
            pts.extend(leg["points"][1:])
        return pts

    def road_ahead(self, passed: int) -> list[str]:
        """웨이포인트 passed 개를 지난 지금, 아직 달리지 않은 도로 id (지금 달리는 도로 포함, 순서대로)."""
        if not self.leg_of:
            return []
        first = self.leg_of[min(passed, len(self.leg_of) - 1)]
        return [leg["road_id"] for leg in self.legs[first:]]

    def current_leg(self, passed: int) -> dict | None:
        if not self.leg_of:
            return None
        return self.legs[self.leg_of[min(passed, len(self.leg_of) - 1)]]


def build(legs: list[dict], start: tuple[float, float], target_node: str, path: list[str],
          eta_s: float, gap_m: float = WAYPOINT_GAP_M) -> RoutePlan:
    plan = RoutePlan(target_node=target_node, path=path, eta_s=eta_s, start=start, legs=legs)
    raw = []    # (점, 속도, 구간번호) — 출발점 제외
    for i, leg in enumerate(legs):
        for p in leg["points"][1:]:
            raw.append((tuple(p), leg["speed_mps"], i))
    if not raw:
        return plan

    kept, last = [], start
    for j, (p, v, i) in enumerate(raw):
        is_last = j == len(raw) - 1
        if is_last:
            if kept and distance_m(last, p) < gap_m:
                kept.pop()                      # 마지막 점 바로 앞의 너무 가까운 점을 빼고 마지막 점을 넣는다
            kept.append((p, v, i))
        elif distance_m(last, p) >= gap_m:
            kept.append((p, v, i))
            last = p
    plan.waypoints = [k[0] for k in kept]
    plan.speeds = [k[1] for k in kept]
    plan.leg_of = [k[2] for k in kept]
    return plan


def distance_to_polyline_m(here: tuple[float, float], line: list[tuple[float, float]]) -> float:
    """현재 위치에서 꺾은선까지 최단 거리(m). 점이 2개 미만이면 0."""
    if len(line) < 2:
        return 0.0
    best = math.inf
    for a, b in zip(line, line[1:]):
        ay, ax = to_ned(*a, *here)          # 현재 위치 기준 로컬 좌표 (북, 동)
        by, bx = to_ned(*b, *here)
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / seg2))
        best = min(best, math.hypot(ax + t * dx, ay + t * dy))
    return best
