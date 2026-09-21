from dataclasses import dataclass


@dataclass
class GroundResource:
    resource_id: str
    resource_type: str          # "UGV" | "FIRE_ENGINE"
    base: str                   # "A" | "B"
    home_node: str

    # 현재 상태 — 드라이버가 갱신
    lat: float
    lon: float
    fuel_pct: float = 100.0
    state: str = "READY"        # 계약 §6 상태값
    updated_at: float = 0.0     # 마지막 갱신 시각 (STALE 판정용)

    # 경로상 위치
    current_node: str | None = None       # 노드에 정지 중일 때
    current_road_id: str | None = None    # 주행 중일 때
    road_progress: float = 0.0            # 0.0~1.0