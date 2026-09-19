# ugv/geo.py — 위경도 <-> 로컬 NED 미터 변환
# 기준점(ref_lat, ref_lon) 주변을 평면으로 근사하는 등장방형 변환.
# 기준점에서 수 km 이내(도로망 규모)에서 오차가 무시할 만하다. 장거리에는 쓰지 말 것.

import math

EARTH_RADIUS_M = 6_371_000.0


def to_ned(
    lat: float, lon: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    """위경도 -> (north_m, east_m). 기준점이 원점."""
    north = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    east = (
        math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    )
    return north, east


def to_latlon(
    north_m: float, east_m: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    """(north_m, east_m) -> 위경도. to_ned 의 역변환."""
    lat = ref_lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = ref_lon + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat)))
    )
    return lat, lon


def distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    """두 (lat, lon) 사이 수평 거리(m). a 를 기준점으로 삼아 근사."""
    north, east = to_ned(b[0], b[1], a[0], a[1])
    return math.hypot(north, east)
