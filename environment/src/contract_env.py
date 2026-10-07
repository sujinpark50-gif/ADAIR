# -*- coding: utf-8 -*-
"""환경 계약 계층 — 총괄 요청서 ENV-01·02·03·05·06 을 김은주님 실제 CA 위에 얹는다.

이 파일은 기존 환경 코드(grid.py·fire_model.py·api.py)를 **수정하지 않고** 감싸기만 한다.

ENV-01  READ(불변) / ADVANCE(진행) / APPLY(관측 반영) 분리.
        run_id·state_version·simulation_time_s·map_version·scenario_start_kst 를 snapshot 에 싣는다.
        상태를 state_dir 에 저장해 두고, 서버를 다시 켜면 같은 run_id 로 이어서 복구한다
        (state_version·시각은 뒤로 가지 않음). reset() 은 새 run_id.
ENV-02  정적 지도 map_cells (칸 위치·크기·지면고도·건물유형), 보호대상(주거 칸), 소방서 stations.
ENV-03  진짜 세계 바람을 기상청 ASOS 시간자료(인제 211)로 시각별 구동. 지도 전체 값 하나(GLOBAL_FIELD)이며
        칸별 바람장(양간지풍)은 아직 아님 — snapshot.weather.scope 로 드러낸다.
ENV-05  APPLY + ACK. (run_id, observation_id) 단위로 sqlite 에 기록 → 재시작 뒤에도 중복 방지.
        같은 ID·같은 내용 = 처음 ACK + duplicate=true / 같은 ID·다른 내용 = 충돌 거절.
        관측만으로 UNBURNED→BURNING 을 바꾸지 않는다 (진짜 세계는 CA 만 바꾼다). 일부만 본 칸은 부분 관측으로만 기록.
ENV-06  시나리오 신고 reports_until(t) — event_id·run_id 포함.

가정값(ASSUMPTION)은 모두 생성자 인자로 노출하고 /health 로 드러낸다:
  tick_s (CA 한 스텝 = 시뮬레이션 몇 초), scenario_start_kst, 점화 위치, 신고 시각, 진짜 세계 화재 칸 주변 범위.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import sqlite3
import sys
import threading
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

ENV_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = ENV_DIR.parent
if str(ENV_DIR) not in sys.path:                 # 환경 코드는 `from src.xxx import` 형식이다
    sys.path.insert(0, str(ENV_DIR))

from src.cell import FireState                    # noqa: E402

KST = timezone(timedelta(hours=9))

DEFAULT_CONFIG = ENV_DIR / "config" / "environment_config.yaml"
DEFAULT_WEATHER_CSV = REPO_ROOT / "data" / "weather" / "kma_asos_hourly_20190404_20190405.csv"
DEFAULT_STATIONS_JSON = REPO_ROOT / "data" / "weather" / "kma_asos_stations.json"
DEFAULT_FIRE_STATIONS_JSON = ENV_DIR / "config" / "fire_stations.json"

MAP_VERSION = "INJE-2019-v1"                     # 래스터(dem_clipped·buildings_inje)가 바뀌면 올린다
CONTRACT_VERSION = "ENV-CONTRACT-0.1"

_FIRE_CODE = {FireState.UNBURNED: 0, FireState.BURNING: 1, FireState.BURNED: 2}
_FIRE_FROM_CODE = {v: k for k, v in _FIRE_CODE.items()}


def cell_key(col: int, row: int) -> str:
    """팀 공통 칸 ID 형식 (connectors/fire_connector.py·web 과 동일): '{x}_{y}' = '{col}_{row}'."""
    return f"{col}_{row}"


def parse_cell_key(cid) -> Optional[Tuple[int, int]]:
    if not isinstance(cid, str) or "_" not in cid:
        return None
    a, _, b = cid.partition("_")
    try:
        return int(a), int(b)
    except ValueError:
        return None


def content_hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, ensure_ascii=False, default=str).encode()).hexdigest()


# ---------------------------------------------------------------------------
# 기상청 시간자료 → 진짜 세계 바람 (ENV-03)
# ---------------------------------------------------------------------------
class KmaWindDriver:
    """한 관측소의 시간자료를 시뮬레이션 시각에 맞춰 돌려준다. 그 시각 이전의 가장 최근 행만 (보간 없음)."""

    FIELDS = {"ws": "wind_ms", "wd": "wind_dir_deg", "ta": "temperature_c", "hm": "humidity_pct"}

    def __init__(self, csv_path: Path, station_id: str, scenario_start_kst: str, stations_json: Optional[Path] = None):
        start = datetime.fromisoformat(scenario_start_kst)
        self.station_id = str(station_id)
        self.rows: List[Tuple[float, str, dict]] = []
        with open(csv_path, encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                if r["stnId"] != self.station_id:
                    continue
                t = datetime.strptime(r["tm"], "%Y-%m-%d %H:%M").replace(tzinfo=KST)
                vals = {}
                for src, dst in self.FIELDS.items():
                    v = (r.get(src) or "").strip()
                    if v != "":
                        vals[dst] = float(v)
                self.rows.append(((t - start).total_seconds(), r["tm"], vals))
        self.rows.sort(key=lambda x: x[0])
        if not self.rows:
            raise ValueError(f"기상청 자료에 관측소 {station_id} 행이 없다: {csv_path}")
        self.station_name = None
        if stations_json and Path(stations_json).is_file():
            meta = json.load(open(stations_json, encoding="utf-8"))
            self.station_name = meta.get("stations", {}).get(self.station_id, {}).get("name")

    def at(self, sim_time_s: float) -> dict:
        prior = [r for r in self.rows if r[0] <= sim_time_s]
        if not prior:
            # 자료 시작 전 시각 — 값을 만들지 않는다 (호출 측이 설정 기본값을 쓰고 그 사실을 드러냄)
            return {"status": "BEFORE_DATA", "values": {}}
        off, tm, vals = prior[-1]
        status = "OK" if sim_time_s - off <= 3600.0 else "STALE_BEYOND_DATA"
        return {"status": status, "values": dict(vals), "observed_kst": tm, "valid_from_sim_s": off,
                "station_id": self.station_id, "station_name": self.station_name,
                "source": "KMA_ASOS_HOURLY", "provision_interval_s": 3600.0}


# ---------------------------------------------------------------------------
# 계약 환경
# ---------------------------------------------------------------------------
class ContractEnvironment:
    """김은주님 CA(WildfireCAEngine)를 계약(READ/ADVANCE/APPLY)으로 감싼 '진짜 세계'."""

    source = "TEAM_ENV"

    def __init__(self, state_dir, *, config_path=DEFAULT_CONFIG, seed: int = 42,
                 ignitions: Iterable[Tuple[int, int]] = ((19, 142),),
                 tick_s: float = 60.0,
                 scenario_start_kst: str = "2019-04-04T14:45:00+09:00",
                 weather_csv=DEFAULT_WEATHER_CSV, stations_json=DEFAULT_STATIONS_JSON,
                 wind_station_id: str = "211",
                 fire_stations_json=DEFAULT_FIRE_STATIONS_JSON,
                 reports: Optional[List[dict]] = None, report_delay_s: float = 0.0,
                 truth_buffer_cells: int = 3, resume: bool = True):
        from src.fire_model import WildfireCAEngine
        from src.grid import EnvironmentGrid

        self._lock = threading.RLock()
        self.state_dir = Path(state_dir)
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self._state_file = self.state_dir / "env_state.json"
        self._db_path = self.state_dir / "env_applied.sqlite3"
        self.config_path = Path(config_path).resolve()
        self.seed = seed
        self.ignitions = [tuple(map(int, p)) for p in ignitions]
        self.tick_s = float(tick_s)
        self.scenario_start_kst = scenario_start_kst
        self.truth_buffer_cells = int(truth_buffer_cells)
        self.map_version = MAP_VERSION
        self.report_delay_s = float(report_delay_s)
        self._custom_reports = reports

        self.grid = EnvironmentGrid(config_path=str(self.config_path), seed=seed)
        self._cells = self.grid.cells
        # grid.cells 는 호출마다 72k 목록을 새로 만든다 → 한 번만 (칸 객체는 같다)
        self.engine = WildfireCAEngine(grid=self.grid, config_path=str(self.config_path), seed=seed)
        self._config_wind = dict(self.engine._config.get("wind", {}))   # 기상청 자료가 없을 때만 쓰는 설정값

        self.wind_csv, self.stations_json = Path(weather_csv), Path(stations_json)
        self.wind = KmaWindDriver(self.wind_csv, wind_station_id, scenario_start_kst, self.stations_json)
        self._build_static()
        self.stations_doc = self._load_fire_stations(Path(fire_stations_json))
        self._init_db()

        restored = False
        if resume and self._state_file.is_file():
            restored = self._restore()
        if not restored:
            self._new_run()
        self.restored = restored

    # ------------------------------------------------------------------ 정적 지도 (ENV-02)
    def _build_static(self):
        from rasterio.warp import transform as warp
        cells = self._cells
        n, res = len(cells), float(self.grid.cell_resolution)
        t = self.grid.transform
        cols = np.array([c.x for c in cells]); rows = np.array([c.y for c in cells])
        xs = t.c + (cols + 0.5) * t.a
        ys = t.f + (rows + 0.5) * t.e
        lons, lats = warp(self.grid.crs, "EPSG:4326", xs.tolist(), ys.tolist())
        self.static = {
            "cols": cols, "rows": rows, "lat": np.round(np.array(lats), 7), "lon": np.round(np.array(lons), 7),
            "elevation": np.array([c.elevation for c in cells], dtype=float),
            "slope": np.array([c.slope for c in cells], dtype=float),
            "fuel_amount": np.array([c.fuel_amount for c in cells], dtype=float),
            "moisture": np.array([c.moisture for c in cells], dtype=float),
            "building_type": np.array([c.building_type for c in cells], dtype=int),
            "n_cols": self.grid.cols, "n_rows": self.grid.rows, "cell_size_m": res,
        }
        self._map_cells = []
        for i in range(n):
            bt = int(self.static["building_type"][i])
            elev = float(self.static["elevation"][i])
            self._map_cells.append({
                "cell_id": cell_key(int(cols[i]), int(rows[i])), "lat": float(self.static["lat"][i]),
                "lon": float(self.static["lon"][i]), "cell_size_m": res,
                # DEM nodata 는 0 으로 채워져 있다 → 지면고도를 모르는 칸으로 둔다 (가짜 고도로 비행 명령 금지)
                "ground_amsl_m": None if elev <= 0.0 else round(elev, 2),
                "building_type": bt, "human_exposure": bt == 1,
                # 총괄 관측 계획의 지형 근거 (2026-10-07, 총괄 브랜치에서 추가 — 환경팀 공유 필요).
                # 둘 다 시작 시점의 정적 지도값이다 (불이 태운 뒤의 연료가 아님 — 정답 누출 없음)
                "is_road": bool(cells[i].is_road), "fuel_amount": round(float(self.static["fuel_amount"][i]), 3),
            })

    def _idx(self, col: int, row: int) -> Optional[int]:
        if 0 <= col < self.static["n_cols"] and 0 <= row < self.static["n_rows"]:
            return row * self.static["n_cols"] + col
        return None

    def idx_of(self, cid) -> Optional[int]:
        p = parse_cell_key(cid)
        return None if p is None else self._idx(*p)

    def map_cells(self) -> List[dict]:
        return self._map_cells           # 변하지 않는 정보만. 호출 측이 고치지 않는다는 전제 (HTTP 는 직렬화 사본)

    def protected_sites(self) -> List[dict]:
        """주거 칸(building_type=1)을 사람이 있는 보호대상으로 낸다. 좌표 출처는 래스터 칸 중심.
        백서 141쪽 시설(남전리 마을회관 등)의 실제 좌표는 근거가 없어 넣지 않았다."""
        out = []
        for i in np.nonzero(self.static["building_type"] == 1)[0]:
            c = self._map_cells[int(i)]
            out.append({"site_id": f"RES-{c['cell_id']}", "site_type": "RESIDENTIAL_CELL", "human_occupied": True,
                        "location": {"lat": c["lat"], "lon": c["lon"]}, "cell_id": c["cell_id"],
                        "coordinate_source": "buildings_inje.tif 칸 중심 (building_type=1)"})
        return out

    @staticmethod
    def _load_fire_stations(path: Path) -> dict:
        if not path.is_file():
            return {"stations": [], "source": None, "note": f"{path} 없음"}
        return json.load(open(path, encoding="utf-8"))

    def stations(self) -> List[dict]:
        return list(self.stations_doc.get("stations", []))

    # ------------------------------------------------------------------ 영속 (ENV-01·05)
    def _db(self):
        con = sqlite3.connect(str(self._db_path), timeout=10)
        con.row_factory = sqlite3.Row
        return con

    def _init_db(self):
        with self._db() as con:
            con.execute("""CREATE TABLE IF NOT EXISTS applied(
                run_id TEXT NOT NULL, observation_id TEXT NOT NULL, content_hash TEXT NOT NULL,
                ack TEXT NOT NULL, received_wall TEXT NOT NULL, PRIMARY KEY(run_id, observation_id))""")
            con.execute("""CREATE TABLE IF NOT EXISTS cell_obs(
                run_id TEXT NOT NULL, observation_id TEXT NOT NULL, cell_id TEXT NOT NULL,
                sim_time_s REAL, coverage TEXT NOT NULL, fraction REAL, observed_state TEXT,
                sensor_type TEXT, PRIMARY KEY(run_id, observation_id, cell_id))""")

    def _fire_codes(self) -> np.ndarray:
        return np.array([_FIRE_CODE[c.fire_state] for c in self._cells], dtype=np.uint8)

    def _save(self):
        codes = self._fire_codes()
        doc = {
            "contract_version": CONTRACT_VERSION, "run_id": self.run_id, "state_version": self.state_version,
            "simulation_time_s": self.simulation_time_s, "step_count": self.step_count,
            "map_version": self.map_version, "scenario_start_kst": self.scenario_start_kst, "tick_s": self.tick_s,
            "seed": self.seed, "ignitions": self.ignitions,
            "burning": np.nonzero(codes == 1)[0].tolist(), "burned": np.nonzero(codes == 2)[0].tolist(),
            "burn_counters": {str(k): v for k, v in self.engine._burn_counters.items()},
            "rng_state": self.engine._rng.bit_generator.state, "saved_wall": datetime.now(timezone.utc).isoformat(),
        }
        tmp = self._state_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(doc), encoding="utf-8")
        os.replace(tmp, self._state_file)        # 원자적 교체 — 쓰다 멈춰도 이전 상태가 남는다

    def _set_fire(self, burning: Iterable[int], burned: Iterable[int]):
        for c in self._cells:
            c.fire_state = FireState.UNBURNED
        cells = self._cells
        for i in burned:
            cells[int(i)].fire_state = FireState.BURNED
        for i in burning:
            cells[int(i)].fire_state = FireState.BURNING

    def _refresh_risk(self):
        from src.risk import update_grid_risk_scores
        update_grid_risk_scores(self.grid, self.engine._config, spread_scores=self.engine.predict_spread())

    def _restore(self) -> bool:
        try:
            doc = json.loads(self._state_file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return False
        if doc.get("map_version") != self.map_version:
            return False                         # 지도가 바뀌면 이어갈 수 없다 → 새 run
        self.run_id = doc["run_id"]
        self.state_version = int(doc["state_version"])
        self.simulation_time_s = float(doc["simulation_time_s"])
        self.step_count = int(doc["step_count"])
        if doc["scenario_start_kst"] != self.scenario_start_kst:     # 저장된 run 의 시계를 따른다
            self.scenario_start_kst = doc["scenario_start_kst"]
            self.wind = KmaWindDriver(self.wind_csv, self.wind.station_id, self.scenario_start_kst, self.stations_json)
        self.tick_s = float(doc["tick_s"])
        self.ignitions = [tuple(p) for p in doc["ignitions"]]
        self._set_fire(doc["burning"], doc["burned"])
        self.engine._burn_counters = {int(k): v for k, v in doc["burn_counters"].items()}
        self.engine._rng.bit_generator.state = doc["rng_state"]
        # APPLY 기록은 ACK 를 먼저 저장하고 상태 파일을 나중에 쓴다. 그 사이에 멈췄으면 ACK 가 준 버전이 더 크다 →
        # 그보다 작은 버전을 내보내지 않는다 (같은 run 안에서 버전 역행 금지).
        with self._db() as con:
            r = con.execute("SELECT ack FROM applied WHERE run_id=?", (self.run_id,)).fetchall()
        for row in r:
            v = json.loads(row["ack"]).get("state_version")
            if isinstance(v, int) and v > self.state_version:
                self.state_version = v
        self._apply_wind()
        self._refresh_risk()
        return True

    def _new_run(self):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.run_id = f"RUN-INJE-{stamp}-{uuid.uuid4().hex[:6]}"
        self.state_version = 1
        self.simulation_time_s = 0.0
        self.step_count = 0
        from src.fire_model import WildfireCAEngine
        self._set_fire([], [])
        self.engine = WildfireCAEngine(grid=self.grid, config_path=str(self.config_path), seed=self.seed)
        for col, row in self.ignitions:
            self.engine.ignite_at(col, row)
        self._apply_wind()
        self._refresh_risk()
        self._save()

    def reset(self) -> dict:
        """새 시뮬레이션 (0초·처음 상태부터) → 반드시 새 run_id (ENV-01 run_id 규칙)."""
        with self._lock:
            self._new_run()
            return self.snapshot()

    # ------------------------------------------------------------------ 바람 (ENV-03)
    def _apply_wind(self) -> dict:
        w = self.wind.at(self.simulation_time_s)
        vals = w.get("values", {})
        cfg = self.engine._config.setdefault("wind", {})
        if "wind_ms" in vals and "wind_dir_deg" in vals:
            cfg["speed_ms"], cfg["direction_deg"] = vals["wind_ms"], vals["wind_dir_deg"]
            basis = "KMA_ASOS_HOURLY"
        else:
            cfg.update(self._config_wind)
            basis = "CONFIG_DEFAULT_ASSUMPTION"
        self._wind_info = {**w, "basis": basis, "applied_speed_ms": cfg.get("speed_ms"),
                           "applied_direction_deg": cfg.get("direction_deg")}
        return self._wind_info

    # ------------------------------------------------------------------ READ (ENV-01)
    def _cell_view(self, i: int, state: str) -> dict:
        c = self._cells[i]
        return {**self._map_cells[i], "fire_state": state, "risk_score": round(float(c.risk_score), 4)}

    def snapshot(self) -> dict:
        """READ. 상태를 바꾸지 않는다 (CA 진행·버전 증가 없음)."""
        with self._lock:
            codes = self._fire_codes()
            fire_idx = np.nonzero(codes > 0)[0]
            fire = [self._cell_view(int(i), _FIRE_FROM_CODE[int(codes[i])].value) for i in fire_idx]
            # 진짜 세계 화재 칸 주변 UNBURNED 칸 — 모의 센서가 '불 없음'을 볼 수 있는 대상 (총괄 판단 입력 아님)
            ring = set()
            nc, nr, b = self.static["n_cols"], self.static["n_rows"], self.truth_buffer_cells
            for i in fire_idx:
                col, row = int(self.static["cols"][i]), int(self.static["rows"][i])
                for dr in range(-b, b + 1):
                    for dc in range(-b, b + 1):
                        j = self._idx(col + dc, row + dr)
                        if j is not None and codes[j] == 0:
                            ring.add(j)
            risk = [self._cell_view(j, "UNBURNED") for j in sorted(ring)]
            w = self._wind_info
            weather = {k: w["values"][k] for k in ("temperature_c", "humidity_pct") if k in w.get("values", {})}
            weather.update({"scope": "GLOBAL_FIELD", "basis": w["basis"], "station_id": w.get("station_id"),
                            "observed_kst": w.get("observed_kst"), "data_status": w.get("status")})
            return {
                "run_id": self.run_id, "state_version": self.state_version,
                "simulation_time_s": self.simulation_time_s, "scenario_start_kst": self.scenario_start_kst,
                "map_version": self.map_version, "crs": self.grid.crs, "cell_coord_crs": "EPSG:4326",
                "vertical_datum": "DEM_AMSL_UNVERIFIED",
                "wind_ms": w.get("applied_speed_ms"), "wind_dir_deg": w.get("applied_direction_deg"),
                "wind_valid_sim_s": w.get("valid_from_sim_s", self.simulation_time_s),
                "weather": weather, "fire_cells": fire, "risk_cells": risk,
                "protected_sites": self.protected_sites(),
                "step_count": self.step_count, "tick_s": self.tick_s,
                "source": self.source, "contract_complete": True,
            }

    # ------------------------------------------------------------------ ADVANCE (ENV-01)
    def advance(self, steps: int = 1) -> dict:
        if not isinstance(steps, int) or isinstance(steps, bool) or steps < 1:
            raise ValueError("steps 는 1 이상의 정수")
        with self._lock:
            for _ in range(steps):
                self._apply_wind()               # 이번 스텝은 스텝 시작 시각의 관측 바람으로 진행
                self.engine.step()
                self.step_count += 1
                self.simulation_time_s = round(self.simulation_time_s + self.tick_s, 6)
            self._apply_wind()                   # 새 시각의 바람을 snapshot 에 반영
            self.state_version += 1
            self._save()
            return self.snapshot()

    # ------------------------------------------------------------------ APPLY (ENV-05)
    def _validate(self, obs: dict) -> Optional[str]:
        t = obs.get("simulation_time_s")
        if t is not None:
            if isinstance(t, bool) or not isinstance(t, (int, float)) or not math.isfinite(t) or t < 0:
                return "SIMULATION_TIME_INVALID"
            if t > self.simulation_time_s + 1e-6:
                return "OBSERVATION_FROM_FUTURE"
        st = obs.get("sensor_type")
        if st not in (None, "THERMAL", "WEATHER"):
            return f"SENSOR_TYPE_UNSUPPORTED:{st}"
        return None

    def apply(self, observation: dict) -> dict:
        with self._lock:
            if not isinstance(observation, dict):
                return {"accepted": False, "reason": "OBSERVATION_NOT_OBJECT", "duplicate": False}
            oid = observation.get("observation_id")
            if not isinstance(oid, str) or not oid.strip():
                return {"accepted": False, "reason": "OBSERVATION_ID_MISSING", "duplicate": False}
            if observation.get("run_id") != self.run_id:
                # 다른 run 의 관측은 기록하지 않고 거절 (그 run 의 장부에 섞지 않는다)
                return {"accepted": False, "reason": "RUN_MISMATCH", "observation_id": oid, "run_id": self.run_id,
                        "state_version": self.state_version, "duplicate": False}
            h = content_hash(observation)
            with self._db() as con:
                row = con.execute("SELECT content_hash, ack FROM applied WHERE run_id=? AND observation_id=?",
                                  (self.run_id, oid)).fetchone()
            if row is not None:
                if row["content_hash"] == h:
                    return {**json.loads(row["ack"]), "duplicate": True}
                return {"accepted": False, "reason": "OBSERVATION_ID_CONFLICT", "observation_id": oid,
                        "run_id": self.run_id, "state_version": self.state_version, "duplicate": False,
                        "conflict": True, "detail": "같은 observation_id 로 다른 내용이 이미 반영되어 있다 (기존 반영 유지)"}
            problem = self._validate(observation)
            sim = observation.get("simulation_time_s", self.simulation_time_s)
            cell_rows, unknown = [], []
            if problem is None and observation.get("sensor_type", "THERMAL") == "THERMAL":
                det = {d.get("cell_id"): d.get("fire_state") for d in observation.get("detections") or []}
                fracs = {c.get("cell_id"): c.get("fraction") for c in observation.get("cell_coverage") or []}
                for cov, ids in (("FULL", observation.get("covered_cells") or []),
                                 ("PARTIAL", observation.get("partial_cells") or [])):
                    for cid in ids:
                        if self.idx_of(cid) is None:
                            unknown.append(cid)
                            continue
                        # 일부만 본 칸은 '본 부분'의 사실만 남긴다 (칸 전체 관측으로 세지 않음)
                        cell_rows.append((cid, cov, fracs.get(cid, 1.0 if cov == "FULL" else None),
                                          det.get(cid) or ("NOT_DETECTED" if cov == "FULL" else "NOT_DETECTED_IN_PART")))
            if problem is None:
                self.state_version += 1
                ack = {"accepted": True, "observation_id": oid, "run_id": self.run_id,
                       "state_version": self.state_version, "duplicate": False,
                       "applied_cells": {"full": sum(1 for r in cell_rows if r[1] == "FULL"),
                                         "partial": sum(1 for r in cell_rows if r[1] == "PARTIAL")},
                       "unknown_cells": unknown, "truth_fire_state_changed": False,
                       "policy": "OBSERVATION_LOGGED_NO_TRUTH_MUTATION"}
            else:
                ack = {"accepted": False, "reason": problem, "observation_id": oid, "run_id": self.run_id,
                       "state_version": self.state_version, "duplicate": False}
            with self._db() as con:                     # ACK 와 칸 기록을 한 트랜잭션으로
                con.execute("INSERT INTO applied VALUES(?,?,?,?,?)",
                            (self.run_id, oid, h, json.dumps(ack, ensure_ascii=False),
                             datetime.now(timezone.utc).isoformat()))
                con.executemany("INSERT OR IGNORE INTO cell_obs VALUES(?,?,?,?,?,?,?,?)",
                                [(self.run_id, oid, cid, sim, cov, fr, st, observation.get("sensor_type", "THERMAL"))
                                 for cid, cov, fr, st in cell_rows])
            if ack["accepted"]:
                self._save()
            return ack

    def applied_count(self, run_id: Optional[str] = None) -> int:
        with self._db() as con:
            return con.execute("SELECT COUNT(*) FROM applied WHERE run_id=? AND json_extract(ack,'$.accepted')=1",
                               (run_id or self.run_id,)).fetchone()[0]

    # ------------------------------------------------------------------ 신고 (ENV-06)
    def reports(self) -> List[dict]:
        if self._custom_reports is not None:
            base = [dict(r) for r in self._custom_reports]
        else:
            # 기본값: 첫 점화 칸을 report_delay_s 뒤 119 신고 (시험값 — 백서 140~146쪽 시각 대조 필요)
            col, row = self.ignitions[0]
            base = [{"cell_id": cell_key(col, row), "sim_time_s": self.report_delay_s, "source": "119_CALL",
                     "note": "시나리오 시험값 (백서 대조 전)"}]
        out = []
        for r in base:
            meaning = {k: r.get(k) for k in ("cell_id", "sim_time_s", "source", "note")}
            out.append({**r, "run_id": self.run_id,
                        "event_id": r.get("event_id") or f"EVT-{self.run_id}-{content_hash(meaning)[:10]}"})
        return out

    def reports_until(self, sim_time_s: float) -> List[dict]:
        return [r for r in self.reports() if r.get("sim_time_s", 0.0) <= sim_time_s]

    # ------------------------------------------------------------------ 상태 설명
    def health(self) -> dict:
        return {"contract_version": CONTRACT_VERSION, "run_id": self.run_id, "state_version": self.state_version,
                "simulation_time_s": self.simulation_time_s, "map_version": self.map_version,
                "restored_from_disk": self.restored, "grid": {"rows": self.grid.rows, "cols": self.grid.cols,
                                                              "cell_size_m": self.static["cell_size_m"]},
                "wind": {k: self._wind_info.get(k) for k in ("basis", "status", "station_id", "station_name",
                                                            "observed_kst", "applied_speed_ms",
                                                            "applied_direction_deg")},
                "assumptions": {"tick_s": self.tick_s, "scenario_start_kst": self.scenario_start_kst,
                                "ignitions": self.ignitions, "seed": self.seed,
                                "truth_buffer_cells": self.truth_buffer_cells,
                                "wind_field": "GLOBAL_FIELD (칸별 바람장·양간지풍 미구현)",
                                "apply_policy": "관측은 기록만, 진짜 화재 상태는 CA 만 바꾼다",
                                "fire_stations_source": self.stations_doc.get("source")}}
