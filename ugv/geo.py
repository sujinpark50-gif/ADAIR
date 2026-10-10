# -*- coding: utf-8 -*-
"""ugv/geo.py — 좌표 변환·거리.

UGV 내부 계산은 지역 평면 좌표(m)로 한다. 원점 = 팀 공통 datum (gz_bridge.DATUM_*, Gazebo spherical_coordinates).
지역 평면 = datum 중심 횡메르카토르 (WGS84, k=1). Gazebo 도로 월드(kangwon_flat)의 도로 메시·스폰 좌표도 이 평면이다
(ugv/tools/build_road_mesh.py, spawn_pose.py). PROJ 와 mm 단위로 일치 (ugv/tests/test_geo.py 에서 대조).
Gazebo/PX4 위경도(구면 좌표 → ECEF 접평면 ENU)와는 다르다 — 원점에서 7 km(거점 A) 0.19 m, 18.6 km(거점 B) 0.52 m.
Gazebo 시뮬레이터의 PX4 드라이버는 GZ_FRAME 으로 바꿔 도로 월드와 맞춘다 (2026-10-10).
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


class GzEnuFrame:
    """Gazebo 시뮬레이터의 위경도 ↔ 월드 좌표 (gz-math SphericalCoordinates: datum 의 ECEF 접평면 ENU, WGS84).
    Gazebo 도로 월드는 지역 평면(TM, FRAME) 좌표로 만들었지만 Gazebo GPS(navsat)는 월드 좌표를 이 방식으로 위경도로 바꾼다.
    원점에서 멀수록 두 방식이 벌어져, 18.6 km(거점 B 부근)에서 0.52 m (2026-10-10 실측: 정지 차량 Gazebo (16829.98, -7262.30)
    → PX4 위경도 → FRAME (16829.50, -7262.10), 이 변환으로 계산하면 1 mm 안에서 일치). 그만큼 PX4 가 Gazebo 도로에서 옆으로 비켜
    달렸다. 시뮬레이터 차량의 PX4 드라이버만 이것을 쓴다 (실차는 FRAME — 실제 측지 변환)."""

    def __init__(self, lat0: float = DATUM_LAT, lon0: float = DATUM_LON, alt0: float = DATUM_ALT):
        self._p, self._l = math.radians(lat0), math.radians(lon0)
        self._o_h = alt0
        self._o = self._ecef(self._p, self._l, alt0)

    @staticmethod
    def _ecef(p, l, h):
        n = _A / math.sqrt(1 - _E2 * math.sin(p) ** 2)
        return ((n + h) * math.cos(p) * math.cos(l), (n + h) * math.cos(p) * math.sin(l), (n * (1 - _E2) + h) * math.sin(p))

    def to_ll(self, x: float, y: float):
        p, l = self._p, self._l
        dx = -math.sin(l) * x - math.sin(p) * math.cos(l) * y
        dy = math.cos(l) * x - math.sin(p) * math.sin(l) * y
        dz = math.cos(p) * y
        X, Y, Z = self._o[0] + dx, self._o[1] + dy, self._o[2] + dz
        lon = math.atan2(Y, X)
        r = math.hypot(X, Y)
        lat = math.atan2(Z, r * (1 - _E2))
        for _ in range(6):
            n = _A / math.sqrt(1 - _E2 * math.sin(lat) ** 2)
            h = r / math.cos(lat) - n
            lat = math.atan2(Z, r * (1 - _E2 * n / (n + h)))
        return math.degrees(lat), math.degrees(lon)

    def _enu(self, lat, lon, h):
        p, l = self._p, self._l
        X, Y, Z = self._ecef(math.radians(lat), math.radians(lon), h)
        dx, dy, dz = X - self._o[0], Y - self._o[1], Z - self._o[2]
        e = -math.sin(l) * dx + math.cos(l) * dy
        n = -math.sin(p) * math.cos(l) * dx - math.sin(p) * math.sin(l) * dy + math.cos(p) * dz
        u = math.cos(p) * math.cos(l) * dx + math.cos(p) * math.sin(l) * dy + math.sin(p) * dz
        return e, n, u

    def to_xy(self, lat: float, lon: float):
        """월드 평면(U = 0, 원점 고도의 접평면) 위의 점으로 본다 — 평탄 월드 차량 높이(1 m 안팎)의 영향은 mm 수준.
        고도를 모르면(위경도만) 높이를 반복으로 맞춘다 (18.6 km 에서 접평면은 지표보다 약 27 m 높다)."""
        h = self._o_h
        for _ in range(4):
            e, n, u = self._enu(lat, lon, h)
            h -= u
        return e, n


GZ_FRAME = GzEnuFrame()


def distance_m(a, b) -> float:
    """두 (lat, lon) 사이 거리 (m, 하버사인)."""
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 6371008.8 * math.asin(math.sqrt(h))


def bearing_deg(dx: float, dy: float) -> float:
    """지역 평면 벡터의 방위 (북=0, 시계방향, 0~360)."""
    return math.degrees(math.atan2(dx, dy)) % 360.0
