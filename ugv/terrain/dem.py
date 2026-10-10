# -*- coding: utf-8 -*-
"""ugv/terrain/dem.py — 환경 DEM(environment/data/processed/dem_clipped.tif) 읽기와 지역 평면 좌표 → 고도.

DEM: 90 m 격자, 투영 = EPSG:5186 과 같은 TM(위도 38°, 경도 127°, k=1, FE 200 000, FN 600 000, GRS80), nodata −9999,
값 단위 m. 수직 기준(정표고·타원체고)은 메타데이터에 없다 (미확인). Gazebo 월드 원점 고도 172.1 m 가 이 DEM 의 원점 값과 같다.
지역 평면(ugv/geo.py FRAME, WGS84 datum 중심 TM) ↔ 5186 은 위경도를 거쳐 바꾼다 (GRS80·WGS84 차이는 여기서 무시할 만함).
"""

import math
from functools import lru_cache
from pathlib import Path

from ..geo import FRAME, LocalFrame

DEM_PATH = Path(__file__).resolve().parents[2] / "environment" / "data" / "processed" / "dem_clipped.tif"
_K5186 = LocalFrame(38.0, 127.0)


@lru_cache(maxsize=1)
def _dem():
    import rasterio
    r = rasterio.open(DEM_PATH)
    return r, r.read(1).astype(float)


def to5186(x, y):
    la, lo = FRAME.to_ll(x, y)
    e, n = _K5186.to_xy(la, lo)
    return e + 200000.0, n + 600000.0


def z_at(x: float, y: float) -> float:
    """지역 평면 (x, y) 의 DEM 고도 (쌍선형 보간, 셀 중심 기준). nodata 면 nan."""
    r, a = _dem()
    E, N = to5186(x, y)
    fx = (E - r.bounds.left) / r.res[0] - 0.5
    fy = (r.bounds.top - N) / r.res[1] - 0.5
    c0, r0 = int(math.floor(fx)), int(math.floor(fy))
    tx, ty = fx - c0, fy - r0

    def g(rr, cc):
        rr = min(max(rr, 0), r.height - 1)
        cc = min(max(cc, 0), r.width - 1)
        v = a[rr, cc]
        return math.nan if v == r.nodata else v
    return (g(r0, c0) * (1 - tx) * (1 - ty) + g(r0, c0 + 1) * tx * (1 - ty)
            + g(r0 + 1, c0) * (1 - tx) * ty + g(r0 + 1, c0 + 1) * tx * ty)


def meta() -> dict:
    r, a = _dem()
    return {"path": str(DEM_PATH), "res_m": list(r.res), "size": [r.width, r.height], "bounds_5186": list(r.bounds),
            "nodata": r.nodata, "crs": "TM 38N 127E k=1 FE200000 FN600000 GRS80 (= EPSG:5186)",
            "vertical_datum": "미확인 (메타데이터 없음)", "z_at_world_origin_m": round(z_at(0.0, 0.0), 2)}
