from dataclasses import dataclass, field


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
    base_time_s: float          # 평상시 소요 (도로 제한속도 기준)
    name: str = ""
    speed_kmh: float | None = None      # 도로 제한속도. 없으면 distance_m / base_time_s 로 본다
    # 도로 선형 [[lat, lon], ...], node_a → node_b 방향. 없으면 두 노드를 잇는 직선
    geometry: list | None = field(default=None, repr=False, compare=False)

    # 동적 상태 — 시나리오·환경모듈이 갱신
    blocked: bool = False
    congestion: float = 1.0     # 1.0=정상, 2.0=2배 느림

    @property
    def cost_s(self) -> float:
        if self.blocked:
            return float("inf")
        return self.base_time_s * self.congestion

    def speed_mps(self, max_speed_mps: float | None = None) -> float:
        """이 도로에서 실제로 낼 속도 = min(도로 제한속도, 차량 최고속도) / 혼잡 배율."""
        road = self.speed_kmh / 3.6 if self.speed_kmh else self.distance_m / max(self.base_time_s, 1e-6)
        if max_speed_mps is not None:
            from . import config                       # 느린 도로 하한 (UGV_MIN_ROAD_SPEED_KMH, 기본 50 km/h)
            road = max(road, config.MIN_ROAD_SPEED_KMH / 3.6)
            road = min(road, max_speed_mps)
        return road / self.congestion

    def travel_s(self, max_speed_mps: float | None = None) -> float:
        """통과 시간(시뮬레이션 초). max_speed_mps 를 안 주면 cost_s 와 같다 (기존 동작 유지)."""
        if self.blocked:
            return float("inf")
        if max_speed_mps is None:
            return self.cost_s
        return self.distance_m / self.speed_mps(max_speed_mps)

@dataclass
class RouteResult:
    reachable: bool
    eta_s: float | None
    path: list[str] | None
    blocked_road_id: str | None = None   # 막혀서 실패한 경우