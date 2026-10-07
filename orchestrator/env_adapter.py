# -*- coding: utf-8 -*-
"""
orchestrator/env_adapter.py
===========================
환경 연결 경계 (요청서 §3).

READ    변경 없는 원자적 snapshot 조회. 조회가 시뮬레이션을 진행시키면 안 된다.
ADVANCE 명시적 시뮬레이션 진행.
APPLY   관측·진화 이벤트 적용 → ACK(수용 여부, 새 state_version).

현재 환경팀 코드(environment/src/api.py)에는 run_id·simulation_time·map_version 이 없고
connectors/fire_connector.py 는 조회 때마다 step() 한다. 그래서 여기서는
- FixtureEnv: 계약 형태를 그대로 따르는 시험용 환경 (상태 B: fixture 로만 시험)
- SnapshotGate: 같은 run 안의 중복·역순 snapshot 을 걸러내는 총괄 쪽 검사 (메모리 판). 엔진은 같은 규칙을
  장부에 저장해 쓴다 — 활성 run·마지막 버전은 Ledger.activate_run / note_state_version (재시작 후에도 유지)
만 둔다. 환경팀 API 가 생기면 같은 인터페이스(read/advance/apply)로 어댑터를 추가한다.
"""

import copy
from typing import Optional

from .models import Snapshot

class EnvUnavailable(Exception):
    """환경 호출에 응답이 없음 (시간 초과·연결 끊김). 거절(NACK)과 다르다 — 반영 여부를 모른다."""


# APPLY 응답을 못 받은 것으로 보는 예외. 그 밖의 예외(프로그래밍 오류)는 숨기지 않는다.
ENV_TRANSIENT_ERRORS = (EnvUnavailable, TimeoutError, ConnectionError)

# 판단에 반드시 있어야 하는 snapshot 필드. 없으면 해당 판단은 보류한다.
REQUIRED_SNAPSHOT_FIELDS = ("run_id", "state_version", "simulation_time_s", "map_version")


def missing_snapshot_fields(s: Snapshot) -> list:
    return [f for f in REQUIRED_SNAPSHOT_FIELDS if getattr(s, f) is None]


def wind_status(s: Snapshot) -> Optional[str]:
    """풍속을 쓸 수 없으면 사유 코드, 쓸 수 있으면 None. 0 은 유효값, 누락과 구분한다."""
    w = s.wind_ms
    if w is None:
        return "WIND_MISSING"
    if isinstance(w, bool) or not isinstance(w, (int, float)) or w != w or w < 0 or w == float("inf"):
        return "WIND_INVALID"
    return None


class SnapshotGate:
    """같은 run_id 안에서 state_version 이 올라간 snapshot 만 새 판단 입력으로 받는다."""

    def __init__(self):
        self.run_id = None
        self.version = None

    def check(self, s: Snapshot) -> str:
        """NEW / DUPLICATE / STALE / RUN_CHANGED / INCOMPLETE"""
        if missing_snapshot_fields(s):
            return "INCOMPLETE"
        if self.run_id is not None and s.run_id != self.run_id:
            self.run_id, self.version = s.run_id, s.state_version
            return "RUN_CHANGED"
        if self.version is not None:
            if s.state_version == self.version:
                return "DUPLICATE"
            if s.state_version < self.version:
                return "STALE"
        self.run_id, self.version = s.run_id, s.state_version
        return "NEW"


class FixtureEnv:
    """시험용 환경. READ 는 불변, ADVANCE/APPLY 만 상태를 바꾼다.

    fire_cells / risk_cells / protected_sites 는 시험이 직접 넣는다. 셀 dict 에는
    최소 cell_id, lat, lon, ground_amsl_m, fire_state 가 있다.
    """

    source = "FIXTURE"

    def __init__(self, run_id="RUN-FIXTURE", map_version="FIXTURE-MAP-1", wind_ms=3.0,
                 fire_cells=None, risk_cells=None, protected_sites=None,
                 crs="EPSG:4326", vertical_datum="FIXTURE_AMSL", tick_s=60.0,
                 wind_dir_deg=None, weather=None, spread_forecast=None, forecast_ref=None,
                 scenario_start_kst=None, reports=None):
        self.run_id = run_id
        self.map_version = map_version
        self.wind_ms = wind_ms
        self.crs = crs
        self.vertical_datum = vertical_datum
        self.tick_s = tick_s
        self.state_version = 1
        self.simulation_time_s = 0.0
        self.fire_cells = list(fire_cells or [])
        self.risk_cells = list(risk_cells or [])
        self.protected_sites = list(protected_sites or [])
        self.wind_dir_deg = wind_dir_deg
        self.weather = dict(weather or {})
        self.spread_forecast = list(spread_forecast or [])     # 시험용 가짜 예측 (환경 모델 계약 대기)
        self.forecast_ref = forecast_ref
        self.scenario_start_kst = scenario_start_kst
        self.reports = list(reports or [])       # 시나리오 신고 이벤트 [{cell_id, sim_time_s, source}]
        self.applied = {}          # observation_id → ACK (중복 적용 방지)
        self.read_count = 0

    def read(self) -> Snapshot:
        self.read_count += 1
        return Snapshot(
            run_id=self.run_id, state_version=self.state_version,
            simulation_time_s=self.simulation_time_s, map_version=self.map_version,
            wind_ms=self.wind_ms, wind_valid_sim_s=self.simulation_time_s,
            crs=self.crs, vertical_datum=self.vertical_datum,
            fire_cells=copy.deepcopy(self.fire_cells), risk_cells=copy.deepcopy(self.risk_cells),
            protected_sites=copy.deepcopy(self.protected_sites),
            wind_dir_deg=self.wind_dir_deg, weather=copy.deepcopy(self.weather),
            spread_forecast=copy.deepcopy(self.spread_forecast), forecast_ref=copy.deepcopy(self.forecast_ref),
            scenario_start_kst=self.scenario_start_kst,
            source=self.source, contract_complete=True,
        )

    # 지도(변하지 않는 정보): 칸 위치·크기·지면고도·건물 유형. 불·위험·기상 같은 진짜 상태는 뺀다.
    STATIC_CELL_KEYS = ("cell_id", "lat", "lon", "cell_size_m", "ground_amsl_m", "building_type", "human_exposure",
                        "is_road", "fuel_amount")      # 도로·연료: 관측 계획의 지형 근거 (정적 지도 정보)

    def map_cells(self) -> list:
        out = {}
        for c in self.fire_cells + self.risk_cells + self.spread_forecast:
            out.setdefault(c["cell_id"], {k: c[k] for k in self.STATIC_CELL_KEYS if c.get(k) is not None})
        return list(out.values())

    def reports_until(self, sim_time_s: float) -> list:
        """그 시각까지 들어온 신고만 (시나리오 이벤트)"""
        return [r for r in self.reports if r.get("sim_time_s", 0.0) <= sim_time_s]

    def advance(self, steps: int = 1) -> Snapshot:
        self.simulation_time_s += self.tick_s * steps
        self.state_version += 1
        return self.read()

    def apply(self, observation: dict) -> dict:
        """관측 적용. 같은 observation_id 는 한 번만 반영하고 기존 ACK 를 돌려준다."""
        oid = observation.get("observation_id")
        if not oid:
            return {"accepted": False, "reason": "OBSERVATION_ID_MISSING"}
        if oid in self.applied:
            return {**self.applied[oid], "duplicate": True}
        if observation.get("run_id") != self.run_id:
            return {"accepted": False, "reason": "RUN_MISMATCH"}
        changed = False
        for det in observation.get("detections", []):
            for cell in self.fire_cells + self.risk_cells:
                if cell.get("cell_id") == det.get("cell_id") and det.get("fire_state"):
                    # 관측으로 확인된 상태만 기록한다. UNBURNED→BURNING 임의 변경은 하지 않는다.
                    cell["last_observed_sim_s"] = self.simulation_time_s
                    changed = True
        self.state_version += 1
        ack = {"accepted": True, "observation_id": oid, "state_version": self.state_version,
               "run_id": self.run_id, "changed_cells": changed, "duplicate": False}
        self.applied[oid] = ack
        return ack
