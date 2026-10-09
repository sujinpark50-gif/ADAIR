# -*- coding: utf-8 -*-
"""ugv/profiles.py — 차량 종류별 주행 특성 (차체 치수·회전반경·속도·가감속).

경로 탐색·회전 가능성·차체 도로 포함 검사·속도 계획은 차량마다 이 값으로 한다 (drive_graph.DriveGraph(net, profile)).
차체 크기와 회전반경을 전역 상수로 공유하지 않는다 — UGV 가 지나는 회전이라도 소방차가 못 지나면 소방차 경로에서 빠진다.

UGV (adair_ugv): 기존 값 그대로 (ugv/gazebo/models/adair_ugv, airframe 51100, 2단계 실측 조향 30°). 직선 목표 약 60 km/h.
소방차 (adair_firetruck): **시뮬레이션용 가정값** (2026-10-10). 특정 실차 제원이 아니다. 국내 중형 펌프차 규모를 참고한
대략값으로 길이 7.0 m · 폭 2.3 m · 축간거리 3.8 m · 질량 10 t · 최대 조향 35° (뒤축 최소 회전반경 약 5.4 m).
속도 상한 50 km/h, 가속 1.2 m/s², 감속 계획 1.5 m/s², 곡선 횡가속 1.2 m/s² (무게중심이 높아 UGV 보다 낮게).
실제 소방차 자료를 받으면 이 표와 Gazebo 모델·airframe(51101) 을 같이 고친다.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

from . import config


@dataclass(frozen=True)
class VehicleProfile:
    name: str
    vehicle_kind: str                  # UGV / FIRE_TRUCK
    wheelbase_m: float
    steer_max_deg: float
    body_length_m: float
    body_width_m: float
    rear_axle_from_center_m: float     # 뒤축이 차체 중심에서 뒤로 떨어진 거리
    max_speed_mps: float
    lat_accel_mps2: float
    accel_mps2: float
    decel_mps2: float
    mass_kg: float
    basis: str
    gz_model: str                      # Gazebo 모델 (ugv/gazebo/models/<gz_model>)
    px4_airframe: int
    body_samples: Tuple[Tuple[float, float], ...] = field(init=False, repr=False)

    def __post_init__(self):
        object.__setattr__(self, "body_samples", _outline(self.front, self.rear, self.hw))

    @property
    def r_min(self) -> float:
        """뒤축 중심 최소 회전반경."""
        return self.wheelbase_m / math.tan(math.radians(self.steer_max_deg))

    @property
    def front(self) -> float:          # 뒤축에서 앞 범퍼
        return self.rear_axle_from_center_m + self.body_length_m / 2

    @property
    def rear(self) -> float:           # 뒤축에서 뒤 범퍼
        return self.body_length_m / 2 - self.rear_axle_from_center_m

    @property
    def hw(self) -> float:
        return self.body_width_m / 2

    @property
    def body_reach(self) -> float:     # 뒤축에서 차체 끝까지 가장 먼 거리
        return max(math.hypot(x, y) for x, y in self.body_samples)

    def stop_distance(self, v: float) -> float:
        """v 에서 계획 감속으로 멈추는 거리 + 여유 (명령 지연 거리 제외)."""
        return v * v / (2 * self.decel_mps2) + config.STOP_MARGIN_M

    def summary(self) -> dict:
        return {"profile": self.name, "vehicle_kind": self.vehicle_kind, "wheelbase_m": self.wheelbase_m,
                "steer_max_deg": self.steer_max_deg, "body_m": [self.body_length_m, self.body_width_m],
                "r_min_rear_axle_m": round(self.r_min, 2), "max_speed_kmh": round(self.max_speed_mps * 3.6, 1),
                "lat_accel_mps2": self.lat_accel_mps2, "accel_mps2": self.accel_mps2, "decel_mps2": self.decel_mps2,
                "mass_kg": self.mass_kg, "basis": self.basis, "gz_model": self.gz_model,
                "px4_airframe": self.px4_airframe}


def _outline(front, rear, hw, step: float = 0.45):
    """차체 둘레(뒤축 기준)를 step 간격으로 + 중심. 교차로처럼 오목한 도로 영역에서 모서리 사이 변이 밖으로 나가는 것도 본다."""
    nx = max(1, round((front + rear) / step))
    ny = max(1, round(2 * hw / step))
    xs = [-rear + (front + rear) * i / nx for i in range(nx + 1)]
    ys = [-hw + 2 * hw * i / ny for i in range(ny + 1)]
    pts = []
    for x in xs:
        pts += [(x, hw), (x, -hw)]
    for y in ys[1:-1]:
        pts += [(front, y), (-rear, y)]
    pts.append(((front - rear) / 2, 0.0))
    return tuple(pts)


PROFILES: Dict[str, VehicleProfile] = {
    "adair_ugv": VehicleProfile(
        "adair_ugv", "UGV", config.WHEELBASE_M, config.STEER_MAX_DEG, config.BODY_LENGTH_M, config.BODY_WIDTH_M,
        config.REAR_AXLE_FROM_CENTER_M, config.MAX_SPEED_MPS, config.LAT_ACCEL_MPS2, config.ACCEL_MPS2, config.DECEL_MPS2,
        mass_kg=1500.0, basis="ugv/gazebo/models/adair_ugv/model.sdf + 2단계 Gazebo 실측 (조향 30°, 직선 60 km/h)",
        gz_model="adair_ugv", px4_airframe=51100),
    "adair_firetruck": VehicleProfile(
        "adair_firetruck", "FIRE_TRUCK", 3.8, 35.0, 7.0, 2.3, 1.6, 50.0 / 3.6, 1.2, 1.2, 1.5,
        mass_kg=10000.0, basis="시뮬레이션용 가정값 — 국내 중형 펌프차 규모 참고 (실차 제원 아님, 2026-10-10)",
        gz_model="adair_firetruck", px4_airframe=51101),
}
DEFAULT_PROFILE = PROFILES["adair_ugv"]


def get(name: str) -> VehicleProfile:
    if name not in PROFILES:
        raise KeyError(f"차량 특성 {name} 없음 (ugv/profiles.py: {', '.join(PROFILES)})")
    return PROFILES[name]
