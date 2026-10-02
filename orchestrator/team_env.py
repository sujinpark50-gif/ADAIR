# -*- coding: utf-8 -*-
"""
orchestrator/team_env.py
========================
환경팀 계약 환경(environment/src/contract_env.py, environment/server.py)을 총괄의 환경 교체 지점
(read / advance / apply / map_cells / reports_until)과 분석 교체 지점(analyze)에 붙이는 어댑터.

- InProcessTeamEnv : 같은 프로세스에서 ContractEnvironment 를 직접 부른다 (시험·단독 실행용)
- HttpTeamEnv      : environment/server.py 를 HTTP 로 부른다 (팀 공유 환경, INT-05 시계 하나)
- TeamAnalysis     : ENV-04 분석. 총괄이 만든 belief 만 넘긴다 (진짜 세계 참조 없음)

HTTP 응답을 못 받으면 EnvUnavailable 을 던진다 → 엔진이 '반영 여부 모름'으로 다루고 같은 observation_id 로 재전송.
"""

from typing import Optional

from .env_adapter import EnvUnavailable
from .models import Snapshot

_SNAPSHOT_FIELDS = ("run_id", "state_version", "simulation_time_s", "map_version", "wind_ms", "wind_valid_sim_s",
                    "crs", "vertical_datum", "fire_cells", "risk_cells", "protected_sites", "wind_dir_deg",
                    "weather", "scenario_start_kst")


def to_snapshot(d: dict, source: str) -> Snapshot:
    return Snapshot(**{k: d.get(k) for k in _SNAPSHOT_FIELDS if k in d},
                    source=source, contract_complete=bool(d.get("contract_complete")))


class InProcessTeamEnv:
    source = "TEAM_ENV_IN_PROCESS"

    def __init__(self, contract_env):
        self.c = contract_env

    @property
    def tick_s(self):
        return self.c.tick_s

    def read(self) -> Snapshot:
        return to_snapshot(self.c.snapshot(), self.source)

    def advance(self, steps: int = 1) -> Snapshot:
        return to_snapshot(self.c.advance(steps), self.source)

    def apply(self, observation: dict) -> dict:
        return self.c.apply(observation)

    def map_cells(self) -> list:
        return self.c.map_cells()

    def reports_until(self, sim_time_s: float) -> list:
        return self.c.reports_until(sim_time_s)


class HttpTeamEnv:
    """environment/server.py 클라이언트. map_cells 는 map_version 이 같으면 다시 받지 않는다."""

    source = "TEAM_API"

    def __init__(self, base_url: str = "http://127.0.0.1:8300", client=None, timeout_s: float = 10.0):
        import httpx
        self.base = base_url.rstrip("/")
        self.http = client or httpx.Client(timeout=timeout_s)
        self._map = (None, [])
        self._last_tick = None

    def _call(self, method, path, **kw):
        import httpx
        try:
            r = self.http.request(method, self.base + path, **kw)
        except (httpx.TimeoutException, httpx.TransportError) as e:
            raise EnvUnavailable(f"{method} {path}: {type(e).__name__}") from e
        if r.status_code >= 500:
            raise EnvUnavailable(f"{method} {path}: HTTP {r.status_code}")
        r.raise_for_status()
        return r.json()

    @property
    def tick_s(self):
        return self._last_tick

    def read(self) -> Snapshot:
        d = self._call("GET", "/snapshot")
        self._last_tick = d.get("tick_s")
        return to_snapshot(d, self.source)

    def advance(self, steps: int = 1) -> Snapshot:
        return to_snapshot(self._call("POST", "/advance", json={"steps": steps}), self.source)

    def apply(self, observation: dict) -> dict:
        return self._call("POST", "/apply", json=observation)

    def map_cells(self) -> list:
        ver, cells = self._map
        if ver is None:
            d = self._call("GET", "/map_cells")
            self._map = (d["map_version"], d["cells"])
            return d["cells"]
        return cells

    def reports_until(self, sim_time_s: float) -> list:
        try:
            return self._call("GET", "/reports", params={"until": sim_time_s})["reports"]
        except EnvUnavailable:
            return []                 # 신고 조회 실패는 다음 sync 에서 다시 (key 로 중복 무시)


class TeamAnalysis:
    """ENV-04. in-process 분석 객체(analyze 메서드) 또는 HTTP 주소 중 하나로 만든다."""

    def __init__(self, analyzer=None, base_url: Optional[str] = None, client=None, timeout_s: float = 10.0):
        self.analyzer, self.base = analyzer, (base_url or "").rstrip("/") or None
        self.http = client
        if self.base and self.http is None:
            import httpx
            self.http = httpx.Client(timeout=timeout_s)
        self.source = getattr(analyzer, "source", "ENV_BELIEF_ANALYSIS")
        self.last_input = None
        self.last_error = None

    def analyze(self, belief: dict) -> dict:
        self.last_input = belief
        if self.analyzer is not None:
            return self.analyzer.analyze(belief)
        try:
            r = self.http.post(self.base + "/analyze", json=belief)
            r.raise_for_status()
            self.last_error = None
            return r.json()
        except Exception as e:  # noqa: BLE001 — 분석 실패는 판단을 막지 않는다 (예측 없음으로 표시)
            self.last_error = f"{type(e).__name__}: {e}"[:200]
            return {"risk_cells": [], "spread_forecast": [], "forecast_ref": None,
                    "source": "ENV_ANALYSIS_UNAVAILABLE"}
