# -*- coding: utf-8 -*-
"""gz sim 로컬 ENU <-> WGS84 <-> EPSG:5186 + CA 격자 변환 (datum 고정)."""
from __future__ import annotations
from typing import Tuple
import os as _os

DATUM_LAT = 38.011200
DATUM_LON = 128.122643
DATUM_ALT = 172.1
DATUM_E   = 298591.0
DATUM_N   = 601838.0
ENV_CRS   = "EPSG:5186"

# DEM 격자 transform (dem_clipped.tif)
GRID_LEFT = 298096.0
GRID_TOP  = 612773.0
GRID_RES  = 90.0

def _to_world(lat, lon):
    try:
        from connectors import env_geo
        return env_geo.latlon_to_world(lat, lon, dst_crs=ENV_CRS)
    except Exception:
        import rasterio.warp as W
        xs, ys = W.transform("EPSG:4326", ENV_CRS, [lon], [lat])
        return float(xs[0]), float(ys[0])

def _to_latlon(x, y):
    try:
        from connectors import env_geo
        return env_geo.world_to_latlon(x, y, src_crs=ENV_CRS)
    except Exception:
        import rasterio.warp as W
        lons, lats = W.transform(ENV_CRS, "EPSG:4326", [x], [y])
        return float(lats[0]), float(lons[0])

def latlon_to_epsg(lat, lon): return _to_world(lat, lon)
def epsg_to_enu(x, y): return (x - DATUM_E, y - DATUM_N)
def enu_to_epsg(e, n): return (e + DATUM_E, n + DATUM_N)
def latlon_to_enu(lat, lon):
    x, y = _to_world(lat, lon); return epsg_to_enu(x, y)
def enu_to_latlon(e, n):
    x, y = enu_to_epsg(e, n); return _to_latlon(x, y)

def grid_to_epsg(col, row):
    return (GRID_LEFT + (col + 0.5) * GRID_RES, GRID_TOP - (row + 0.5) * GRID_RES)
def grid_cell_to_enu(col, row):
    x, y = grid_to_epsg(col, row); return epsg_to_enu(x, y)
def grid_cell_to_latlon(col, row):
    x, y = grid_to_epsg(col, row); return _to_latlon(x, y)

_DEM = None
_DEM_PATH = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                          "environment", "data", "processed", "dem_clipped.tif")
def terrain_elev(col, row):
    """격자 셀 중심의 지면 고도(AMSL, m).

    fail-closed: DEM 을 읽지 못하거나 셀이 격자 밖이면 예외를 던진다.
    (이전 동작: 조용히 DATUM_ALT=172.1m 반환 / 격자 밖 좌표를 가장자리로 클램프 →
     틀린 고도로 비행 명령이 나갈 수 있어 제거)
    """
    global _DEM
    if _DEM is None:
        import rasterio
        with rasterio.open(_DEM_PATH) as ds:
            _DEM = ds.read(1)
    rows, cols = _DEM.shape
    r, c = int(row), int(col)
    if not (0 <= r < rows and 0 <= c < cols):
        raise IndexError(f"셀({col},{row}) 이 DEM 격자({cols}x{rows}) 밖")
    return float(_DEM[r, c])

def fire_cell_to_target(cell, clearance_m=50.0):
    col, row = int(cell["x"]), int(cell["y"])
    lat, lon = grid_cell_to_latlon(col, row)
    return lat, lon, terrain_elev(col, row) + clearance_m

def amsl_to_local_up(a): return a - DATUM_ALT
def local_up_to_amsl(z): return z + DATUM_ALT

if __name__ == "__main__":
    e, n = latlon_to_enu(DATUM_LAT, DATUM_LON)
    la, lo = enu_to_latlon(0.0, 0.0)
    print(f"datum -> ENU = ({e:.3f}, {n:.3f})  (0,0 이어야 정상)")
    print(f"ENU(0,0) -> latlon = ({la:.6f}, {lo:.6f})  (datum 이어야 정상)")
