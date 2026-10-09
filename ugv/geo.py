# -*- coding: utf-8 -*-
"""ugv/geo.py — 좌표 변환·거리.

UGV 내부 계산은 지역 평면 좌표(m)로 한다. 원점 = 팀 공통 datum (gz_bridge.DATUM_*, Gazebo spherical_coordinates).
지역 평면 = datum 중심 횡메르카토르 (WGS84, k=1) — 도로 월드(ugv/gazebo/worlds/kangwon_ugv2.sdf)의 도로 면·스폰 좌표를
만든 투영과 같다 (221a497 ugv/tools/build_road_world.py LOCAL). PROJ 와 mm 단위로 일치 (ugv/tests 에서 대조).
Gazebo/PX4 위경도(구면 좌표 → 지역 ENU)와는 27 km 떨어진 곳에서도 0.6 m 이내.
처음에는 원점 위도 하나로 경도 길이를 잡는 평면 근사를 썼는데, 원점에서 북쪽 5.6 km(거점 A)에서 동서 2.8 m 가
어긋나 도로 월드 주행 중 차가 도로 면 가장자리로 밀렸다 (2026-10-09 실측) → 이 투영으로 바꿨다.
EPSG:5186(환경 격자)과는 축이 약 0.7° 돌아 있어 섞어 쓰지 않는다 — 환경 칸은 위경도로 받아 여기서 바꾼다.
"""

import math

DATUM_LAT, DATUM_LON, DATUM_ALT = 38.011200, 128.122643, 172.1

_A, _F = 6378137.0, 1 / 298.257223563
_E2 = _F * (2 - _F)
_EP2 = _E2 / (1 - _E2)
_E4, _E6 = _E2 * _E2, _E2 ** 3
_M1 = 1 - _E2 / 4 - 3 * _E4 / 64 - 5 * _E6 / 256
_M2 = 3 * _E2 / 8 + 3 * _E4 / 32 + 45 * _E6 / 1024
_M3 = 15 * _E4 / 256 + 45 * _E6 / 1024
_M4 = 35 * _E6 / 3072
_E1 = (1 - math.sqrt(1 - _E2)) / (1 + math.sqrt(1 - _E2))


def _arc(phi):
    return _A * (_M1 * phi - _M2 * math.sin(2 * phi) + _M3 * math.sin(4 * phi) - _M4 * math.sin(6 * phi))


class LocalFrame:
    """datum 중심 횡메르카토르 (Snyder, USGS PP1395 식 8-9~8-25). k0 = 1, x = 동, y = 북."""

    def __init__(self, lat0: float = DATUM_LAT, lon0: float = DATUM_LON):
        self.lat0, self.lon0 = lat0, lon0
        self._phi0, self._lam0 = math.radians(lat0), math.radians(lon0)
        self._m0 = _arc(self._phi0)

    def to_xy(self, lat: float, lon: float):
        phi, lam = math.radians(lat), math.radians(lon)
        s, c = math.sin(phi), math.cos(phi)
        n = _A / math.sqrt(1 - _E2 * s * s)
        t = (s / c) ** 2
        cc = _EP2 * c * c
        a = (lam - self._lam0) * c
        x = n * (a + (1 - t + cc) * a ** 3 / 6 + (5 - 18 * t + t * t + 72 * cc - 58 * _EP2) * a ** 5 / 120)
        y = (_arc(phi) - self._m0 + n * (s / c) * (a * a / 2 + (5 - t + 9 * cc + 4 * cc * cc) * a ** 4 / 24
                                                     + (61 - 58 * t + t * t + 600 * cc - 330 * _EP2) * a ** 6 / 720))
        return x, y

    def to_ll(self, x: float, y: float):
        mu = (self._m0 + y) / (_A * _M1)
        p1 = (mu + (3 * _E1 / 2 - 27 * _E1 ** 3 / 32) * math.sin(2 * mu)
              + (21 * _E1 ** 2 / 16 - 55 * _E1 ** 4 / 32) * math.sin(4 * mu)
              + (151 * _E1 ** 3 / 96) * math.sin(6 * mu) + (1097 * _E1 ** 4 / 512) * math.sin(8 * mu))
        s1, c1 = math.sin(p1), math.cos(p1)
        t1 = (s1 / c1) ** 2
        cc1 = _EP2 * c1 * c1
        n1 = _A / math.sqrt(1 - _E2 * s1 * s1)
        r1 = _A * (1 - _E2) / (1 - _E2 * s1 * s1) ** 1.5
        d = x / n1
        phi = p1 - (n1 * (s1 / c1) / r1) * (d * d / 2 - (5 + 3 * t1 + 10 * cc1 - 4 * cc1 * cc1 - 9 * _EP2) * d ** 4 / 24
                                            + (61 + 90 * t1 + 298 * cc1 + 45 * t1 * t1 - 252 * _EP2 - 3 * cc1 * cc1)
                                            * d ** 6 / 720)
        lam = self._lam0 + (d - (1 + 2 * t1 + cc1) * d ** 3 / 6
                            + (5 - 2 * cc1 + 28 * t1 - 3 * cc1 * cc1 + 8 * _EP2 + 24 * t1 * t1) * d ** 5 / 120) / c1
        return math.degrees(phi), math.degrees(lam)


FRAME = LocalFrame()


def distance_m(a, b) -> float:
    """두 (lat, lon) 사이 거리 (m, 하버사인)."""
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 6371008.8 * math.asin(math.sqrt(h))


def bearing_deg(dx: float, dy: float) -> float:
    """지역 평면 벡터의 방위 (북=0, 시계방향, 0~360)."""
    return math.degrees(math.atan2(dx, dy)) % 360.0
