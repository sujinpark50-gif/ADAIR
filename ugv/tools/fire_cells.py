# -*- coding: utf-8 -*-
"""ugv/tools/fire_cells.py — 화재 시험 입력 칸 만들기 (환경 fire_cells 형식). 도로에는 불이 생기지 않으므로 도로 칸은 뺀다.

near_cells: 차량 옆 도로 밖 90 m 칸 하나 (경계가 차량에서 100 m 안) → 안전거리 위반, 이탈 가능한 경우
surround_cells: 차량 둘레 반경 3.5 km 를 30 m 칸으로 채움 (도로 중심선 15 m 안 칸 제외) → 모든 도로 지점이 경계 100 m 안,
                안전한 이탈 지점이 없는 경우 (DANGER)
"""

import math
from functools import lru_cache

from ugv.geo import FRAME


@lru_cache(maxsize=1)
def _net():
    from ugv.roads import RoadNetwork
    return RoadNetwork()


def near_cells(lat, lon, rings=(60, 75, 90)):
    net = _net()
    x0, y0 = FRAME.to_xy(lat, lon)
    for r in rings:
        for a in range(0, 360, 30):
            x, y = x0 + r * math.cos(math.radians(a)), y0 + r * math.sin(math.radians(a))
            if net.snap(x, y, max_m=45.0) is None:
                la, lo = FRAME.to_ll(x, y)
                return [{"cell_id": f"N{r}_{a}", "lat": la, "lon": lo, "fire_state": "BURNING", "cell_size_m": 90}]
    return []


def surround_cells(lat, lon, radius_m=3500.0, cell_m=30.0):
    net = _net()
    x0, y0 = FRAME.to_xy(lat, lon)
    n = int(radius_m // cell_m)
    out = []
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = x0 + cell_m * i, y0 + cell_m * j
            if net.snap(x, y, max_m=cell_m / 2) is None:
                la, lo = FRAME.to_ll(x, y)
                out.append({"cell_id": f"S{i}_{j}", "lat": la, "lon": lo, "fire_state": "BURNING", "cell_size_m": cell_m})
    return out


def behind_cells(lat, lon, heading_deg, rings=(60, 75, 90)):
    """차량 진행 방향 뒤쪽(±60°) 도로 밖 90 m 칸 하나 — 앞으로 이탈할 길이 있는 경우."""
    net = _net()
    x0, y0 = FRAME.to_xy(lat, lon)
    back = math.radians((heading_deg + 180.0) % 360.0)
    for r in rings:
        for da in (0, 20, -20, 40, -40, 60, -60):
            b = back + math.radians(da)
            x, y = x0 + r * math.sin(b), y0 + r * math.cos(b)
            if net.snap(x, y, max_m=45.0) is None:
                la, lo = FRAME.to_ll(x, y)
                return [{"cell_id": f"B{r}_{da}", "lat": la, "lon": lo, "fire_state": "BURNING", "cell_size_m": 90}]
    return []
