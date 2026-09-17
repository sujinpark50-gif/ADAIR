@dataclass(frozen=True)
class Node:
    node_id: str
    lat: float
    lon: float
    name: str = ""

@dataclass
class Road:
    road_id: str
    node_a: str
    node_b: str
    distance_m: float
    base_time_s: float          # 평상시 소요

    # 동적 상태 — 환경모듈이 갱신
    blocked: bool = False
    congestion: float = 1.0     # 1.0=정상, 2.0=2배 느림

    @property
    def cost_s(self) -> float:
        if self.blocked:
            return float("inf")
        return self.base_time_s * self.congestion

@dataclass
class RouteResult:
    reachable: bool
    eta_s: float | None
    path: list[str] | None
    blocked_road_id: str | None = None   # 막혀서 실패한 경우