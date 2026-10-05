"""비행 경로 — 평가·실행 경로 동일성 (UAV-03).

경로 한 개 = 출발점 → (순항고도로) 목표 상공 → 관측고도로 하강·체류 → 순항고도로 기지 상공 → 착륙.
비행 방식은 MockDrone 과 같다: 제자리에서 고도를 맞춘 뒤 수평 이동한다.

순항고도 = max(관측고도, 경로 최고 지형 + ROUTE_TERRAIN_CLEARANCE_M).
지형은 저장소의 DEM(environment/data/processed/dem_clipped.tif, gz_bridge)을 경로를 따라 샘플한다.
샘플마다 그 칸과 주변 8칸 중 최고값을 써서 경로 옆 능선도 피한다.
DEM 을 읽을 수 없거나 경로 일부가 격자 밖이면 terrain.status 로 그 사실을 알린다 (값을 지어내지 않는다).

route_hash = 경로 내용(출발·목표·고도·체류·측정·경유점·지형 결과)의 SHA-256.
평가 응답에 route_id·route_version·route_hash 를 주고, 실행 요청이 같은 값을 실으면 그 경로 그대로 난다.
"""
from __future__ import annotations

import hashlib
import json
import math
import threading
from pathlib import Path
from typing import Optional

import config


def _gb():
    """gz_bridge (저장소 루트). DEM 을 끄거나 불러올 수 없으면 None."""
    if not config.ROUTE_USE_DEM:
        return None
    try:
        import gz_bridge
        return gz_bridge
    except Exception:  # noqa: BLE001 — rasterio 미설치 등
        return None


def _cell_max(gb, col: int, row: int) -> Optional[float]:
    """칸과 주변 8칸 중 최고 지면고도. 하나도 못 읽으면 None."""
    best = None
    for dc in (-1, 0, 1):
        for dr in (-1, 0, 1):
            try:
                v = gb.terrain_elev(col + dc, row + dr)
            except Exception:  # noqa: BLE001 — 격자 밖·DEM 없음
                continue
            best = v if best is None else max(best, v)
    return best


def terrain_along(legs: list[tuple[tuple[float, float], tuple[float, float]]]) -> dict:
    """구간들((lat,lon)→(lat,lon))을 따라 지형 최고값을 구한다.

    반환: {"status": "OK"|"PARTIAL"|"UNVERIFIED", "max_m", "samples", "unknown_samples", "source"}
    """
    gb = _gb()
    if gb is None:
        return {"status": "UNVERIFIED", "max_m": None, "samples": 0, "unknown_samples": 0,
                "source": "DEM_DISABLED" if not config.ROUTE_USE_DEM else "DEM_UNAVAILABLE"}
    best, n, unknown = None, 0, 0
    try:
        for (lat0, lon0), (lat1, lon1) in legs:
            # 투영 좌표(EPSG:5186, m)에서 직선 보간한다. 수십 km 이내라 대권과 차이가 작다
            x0, y0 = gb.latlon_to_epsg(lat0, lon0)
            x1, y1 = gb.latlon_to_epsg(lat1, lon1)
            k = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / config.ROUTE_SAMPLE_M))
            for i in range(k + 1):
                x, y = x0 + (x1 - x0) * i / k, y0 + (y1 - y0) * i / k
                col = math.floor((x - gb.GRID_LEFT) / gb.GRID_RES)
                row = math.floor((gb.GRID_TOP - y) / gb.GRID_RES)
                v = _cell_max(gb, col, row)
                n += 1
                if v is None:
                    unknown += 1
                else:
                    best = v if best is None else max(best, v)
    except Exception as e:  # noqa: BLE001 — 좌표 변환 실패
        return {"status": "UNVERIFIED", "max_m": None, "samples": n, "unknown_samples": unknown,
                "source": f"DEM_ERROR:{type(e).__name__}"}
    status = "UNVERIFIED" if best is None else ("PARTIAL" if unknown else "OK")
    return {"status": status, "max_m": None if best is None else round(best, 1), "samples": n,
            "unknown_samples": unknown, "source": "DEM_CLIPPED_90M"}


def _hash(body: dict) -> str:
    return hashlib.sha256(json.dumps(body, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def plan(uav_id: str, start: dict, target, home, obs_alt: float, observe_s: int,
         measurements: list[str]) -> dict:
    """평가용 경로. start: 현재 위치 dict(lat, lon, alt_m_amsl), target: models.Target, home: (lat, lon, 지면고도)."""
    s = (round(start["lat"], 6), round(start["lon"], 6))
    t = (round(target.lat, 6), round(target.lon, 6))
    h = (round(home[0], 6), round(home[1], 6))
    terrain = terrain_along([(s, t), (t, h)])
    cruise = obs_alt
    if terrain["max_m"] is not None:
        cruise = max(obs_alt, terrain["max_m"] + config.ROUTE_TERRAIN_CLEARANCE_M)
    cruise = round(cruise, 1)
    obs_alt = round(obs_alt, 1)
    body = {
        "version": config.ROUTE_VERSION,
        "uav_id": uav_id,
        "params": {  # 실행 요청과 맞춰 볼 값
            "target": {"lat": target.lat, "lon": target.lon, "alt_m_amsl": target.alt_m_amsl,
                       "target_agl_m": target.target_agl_m},
            "observe_duration_s": observe_s,
            "measurements": list(measurements),
        },
        "start": {"lat": s[0], "lon": s[1], "alt_m_amsl": round(start["alt_m_amsl"], 1)},
        "home": {"lat": h[0], "lon": h[1], "alt_m_amsl": round(home[2], 1)},
        "obs_alt_m_amsl": obs_alt,
        "cruise_alt_m_amsl": cruise,
        "waypoints": [
            {"lat": s[0], "lon": s[1], "alt_m_amsl": cruise, "action": "CLIMB"},
            {"lat": t[0], "lon": t[1], "alt_m_amsl": cruise, "action": "CRUISE"},
            {"lat": t[0], "lon": t[1], "alt_m_amsl": obs_alt, "action": "OBSERVE", "hold_s": observe_s},
            {"lat": t[0], "lon": t[1], "alt_m_amsl": cruise, "action": "CLIMB"},
            {"lat": h[0], "lon": h[1], "alt_m_amsl": cruise, "action": "CRUISE"},
            {"lat": h[0], "lon": h[1], "alt_m_amsl": round(home[2], 1), "action": "LAND"},
        ],
        "terrain": terrain,
        "terrain_clearance_m": config.ROUTE_TERRAIN_CLEARANCE_M,
        "profile_assumption": "VERTICAL_ALT_CHANGE_THEN_HORIZONTAL",
    }
    digest = _hash(body)
    return {**body, "route_id": "RT-" + digest[:16], "route_hash": digest}


def params_match(route: dict, target, observe_s: int, measurements: list[str]) -> bool:
    p = route["params"]
    return (p["target"] == {"lat": target.lat, "lon": target.lon, "alt_m_amsl": target.alt_m_amsl,
                            "target_agl_m": target.target_agl_m}
            and p["observe_duration_s"] == observe_s and p["measurements"] == list(measurements))


class RouteStore:
    """평가한 경로를 파일에 둔다. 재시작 뒤에도 평가 때 받은 route_id 로 실행할 수 있다."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self.routes: dict[str, dict] = {}
        if self.path.is_file():
            try:
                self.routes = json.loads(self.path.read_text(encoding="utf-8")).get("routes", {})
            except (ValueError, OSError):
                self.routes = {}

    def put(self, route: dict) -> None:
        with self._lock:
            self.routes.pop(route["route_id"], None)
            self.routes[route["route_id"]] = route
            while len(self.routes) > config.ROUTE_KEEP:
                self.routes.pop(next(iter(self.routes)))
            tmp = self.path.with_suffix(self.path.suffix + ".tmp")
            tmp.write_text(json.dumps({"routes": self.routes}, ensure_ascii=False), encoding="utf-8")
            tmp.replace(self.path)

    def find(self, route_id: Optional[str], route_hash: Optional[str]) -> Optional[dict]:
        with self._lock:
            if route_id is not None:
                r = self.routes.get(route_id)
                return r if r is not None and (route_hash is None or r["route_hash"] == route_hash) else None
            return next((r for r in self.routes.values() if r["route_hash"] == route_hash), None)
