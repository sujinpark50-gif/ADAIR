# -*- coding: utf-8 -*-
"""
integration/bridge_fire.py
==========================
환경 Connector의 "integrated" 구현.

두 가지 백엔드를 자동 선택한다:
  1) 김은주님의 실제 산불 CA 모델(environment/src/environment) — rasterio + 지형 래스터가
     있으면 이걸 매 스텝 step() 시켜 진짜 확산을 돌린다.
  2) 위 로드가 실패하면(예: rasterio 미설치) 내장 경량 stateful 산불 모델로 자동 전환한다.
     어느 PC에서도 무조건 실행되도록 하기 위한 fallback이며, mock과 달리 '무상태'가 아니라
     실제로 불이 매 스텝 번진다.

어느 쪽이 활성화됐는지는 최초 1회 콘솔에 출력한다.
"""

from __future__ import annotations

import os
import random
import sys
from pathlib import Path
from typing import Optional

import config
from interfaces.schema import EnvironmentState

_REPO_ROOT = Path(__file__).resolve().parent.parent
_ENV_DIR = _REPO_ROOT / "environment"

_backend = None          # 활성 백엔드(실제 또는 fallback) — 최초 1회 생성 후 재사용
_backend_name = None


# ---------------------------------------------------------------------------
# 백엔드 1: 김은주님 실제 CA 모델 어댑터
# ---------------------------------------------------------------------------

class _RealEnvBackend:
    """environment/src/environment 의 EnvironmentGrid + WildfireCAEngine + API 래퍼."""

    def __init__(self):
        cwd0 = os.getcwd()
        if str(_ENV_DIR) not in sys.path:
            sys.path.insert(0, str(_ENV_DIR))
        try:
            os.chdir(_ENV_DIR)  # yaml의 상대 경로(data/processed/*.tif) 해석을 위해
            from src.environment.grid import EnvironmentGrid          # type: ignore
            from src.environment.fire_model import WildfireCAEngine   # type: ignore
            from src.environment.api import EnvironmentModelAPI       # type: ignore

            cfg = "config/environment_config.yaml"
            self._grid = EnvironmentGrid(cfg, seed=42)
            self._engine = WildfireCAEngine(self._grid, cfg, seed=42)
            cx = getattr(self._grid, "cols", 100) // 2
            cy = getattr(self._grid, "rows", 100) // 2
            self._engine.ignite_at(cx, cy)
            self._api = EnvironmentModelAPI(self._grid, self._engine)
        finally:
            os.chdir(cwd0)

    def get_state(self, timestamp: float) -> EnvironmentState:
        # 매 호출마다 한 스텝 진행 (실제 확산)
        try:
            self._api.step()
        except Exception:
            pass
        fire_cells = []
        try:
            for loc in self._api.get_fire_locations():
                x = loc.get("x", loc.get("col", 0))
                y = loc.get("y", loc.get("row", 0))
                fire_cells.append({
                    "cell_id": loc.get("cell_id", f"{x}_{y}"),
                    "x": x, "y": y,
                    "fire_state": "BURNING",
                    "risk_score": round(float(loc.get("risk_score", loc.get("risk", 0.8))), 2),
                })
        except Exception:
            pass
        wind_speed, wind_dir, spread = 3.0, 270.0, "E"
        try:
            st = self._api.get_environment_state()
            wind_speed = float(st.get("wind_speed", st.get("wind", {}).get("speed_ms", 3.0)))
            wind_dir = float(st.get("wind_direction", st.get("wind", {}).get("direction_deg", 270.0)))
            spread = str(st.get("spread_direction", spread))
        except Exception:
            pass
        return EnvironmentState(
            simulation_time_s=timestamp,
            fire_cells=fire_cells[:80],
            risk_zone=[],
            wind_speed=wind_speed,
            wind_direction=wind_dir,
            spread_direction=spread,
            env_updated=False,
        )

    def apply_observation_latlon(self, obs) -> dict:
        """관측(위경도) → EPSG:5186 → 환경 CA 에 반영 (fire_state 갱신)."""
        lat = getattr(obs, "location_lat", None)
        lon = getattr(obs, "location_lon", None)
        if lat is None or lon is None:
            return {"success": False, "error": "관측에 위치 없음"}
        val = getattr(obs, "value", {}) or {}
        fire_state = (val.get("fire_state", "BURNING") if isinstance(val, dict) else "BURNING")
        import gz_bridge
        x, y = gz_bridge.latlon_to_epsg(float(lat), float(lon))
        report = {
            "reporter_id": getattr(obs, "resource_id", "UAV"),
            "world_x": x, "world_y": y,
            "fire_state": fire_state,
        }
        return self._api.apply_observation(report)


# ---------------------------------------------------------------------------
# 백엔드 2: 내장 경량 stateful 산불 모델 (fallback, 순수 파이썬)
# ---------------------------------------------------------------------------

class _LightFireBackend:
    """격자 위에서 불이 매 스텝 이웃으로 번지는 최소 stateful 모델.

    mock(무상태)과 달리 화재 셀 수가 실제로 늘어나고 바람에 따라 확산이 편향된다.
    지형/과학 상수는 없고, '통합 흐름을 살아있게' 만드는 용도.
    """

    _DIRS = {"N": (0, -1), "S": (0, 1), "E": (1, 0), "W": (-1, 0),
             "NE": (1, -1), "NW": (-1, -1), "SE": (1, 1), "SW": (-1, 1)}

    def __init__(self, seed: int = 42):
        self.w = int(getattr(config, "GRID_WIDTH", 100))
        self.h = int(getattr(config, "GRID_HEIGHT", 100))
        cx, cy = self.w // 2, self.h // 2
        self.burning = {(cx, cy)}
        self.burned: set = set()
        self.step_no = 0
        self._seed = seed

    def _spread_dir(self, step):
        seq = ["E", "SE", "S", "SW", "W", "NW", "N", "NE"]
        return seq[step % len(seq)]

    def get_state(self, timestamp: float) -> EnvironmentState:
        rng = random.Random(self._seed + self.step_no)
        spread = self._spread_dir(self.step_no)
        dx, dy = self._DIRS[spread]
        wind_speed = round(2.0 + 4.0 * abs(((self.step_no % 7) / 7.0) - 0.5) * 2, 1)

        new = set()
        for (x, y) in self.burning:
            for ddx in (-1, 0, 1):
                for ddy in (-1, 0, 1):
                    if ddx == 0 and ddy == 0:
                        continue
                    nx, ny = x + ddx, y + ddy
                    if not (0 <= nx < self.w and 0 <= ny < self.h):
                        continue
                    if (nx, ny) in self.burning or (nx, ny) in self.burned:
                        continue
                    p = 0.28
                    if (ddx, ddy) == (dx, dy):     # 바람 방향으로 확산 가속
                        p += 0.25
                    if rng.random() < p:
                        new.add((nx, ny))
        # 오래 탄 셀 일부는 burned로
        for c in list(self.burning):
            if rng.random() < 0.15:
                self.burned.add(c)
                self.burning.discard(c)
        self.burning |= new
        self.step_no += 1

        fire_cells = [{
            "cell_id": f"{x}_{y}", "x": x, "y": y,
            "fire_state": "BURNING",
            "risk_score": round(0.6 + 0.3 * rng.random(), 2),
        } for (x, y) in sorted(self.burning)[:80]]

        # 위험구역 = 불 경계의 미점화 이웃 (앞 20개)
        risk = []
        seen = set()
        for (x, y) in sorted(self.burning):
            for ddx, ddy in self._DIRS.values():
                nx, ny = x + ddx, y + ddy
                if (0 <= nx < self.w and 0 <= ny < self.h and
                        (nx, ny) not in self.burning and (nx, ny) not in self.burned
                        and (nx, ny) not in seen):
                    seen.add((nx, ny))
                    risk.append({"cell_id": f"{nx}_{ny}", "x": nx, "y": ny,
                                 "risk_score": round(0.4 + 0.2 * rng.random(), 2)})
                if len(risk) >= 20:
                    break
            if len(risk) >= 20:
                break

        wind_dir_map = {"N": 0, "NE": 45, "E": 90, "SE": 135,
                        "S": 180, "SW": 225, "W": 270, "NW": 315}
        return EnvironmentState(
            simulation_time_s=timestamp,
            fire_cells=fire_cells,
            risk_zone=risk,
            wind_speed=wind_speed,
            wind_direction=float(wind_dir_map[spread]),
            spread_direction=spread,
            env_updated=False,
        )


# ---------------------------------------------------------------------------
# 공개 함수 (fire_connector 가 integrated 모드에서 호출)
# ---------------------------------------------------------------------------

def _ensure_backend():
    global _backend, _backend_name
    if _backend is not None:
        return
    if os.getenv("ADAIR_FORCE_LIGHT_ENV") != "1":
        try:
            _backend = _RealEnvBackend()
            _backend_name = "김은주 실제 CA 모델 (environment/src/environment)"
            print(f"[ENV] 환경 백엔드 = {_backend_name}")
            return
        except Exception as e:  # rasterio 미설치·데이터 없음·경로 등 무엇이든
            print(f"[ENV] 김은주 실제 모델 로드 실패 → 내장 산불 모델로 전환. (사유: {type(e).__name__}: {e})")
    _backend = _LightFireBackend()
    _backend_name = "내장 경량 stateful 산불 모델 (fallback)"
    print(f"[ENV] 환경 백엔드 = {_backend_name}")


def get_environment_state(timestamp: float) -> EnvironmentState:
    _ensure_backend()
    return _backend.get_state(timestamp)


def update_environment(env_state: EnvironmentState, observations: list) -> EnvironmentState:
    # 관측을 환경 모델에 되먹인다: 실제 백엔드면 위경도→격자셀로 fire_state 반영.
    if observations:
        env_state.env_updated = True
        try:
            _ensure_backend()
            if isinstance(_backend, _RealEnvBackend):
                for obs in observations:
                    res = _backend.apply_observation_latlon(obs)
                    if not res.get("success"):
                        print(f"[ENV] 관측 되먹임 거부: {res.get('error')}")
        except Exception as e:
            print(f"[ENV] 관측 되먹임 스킵: {type(e).__name__}: {e}")
    return env_state


def reset():
    """새 실행을 위해 백엔드 상태 초기화."""
    global _backend, _backend_name
    _backend = None
    _backend_name = None
