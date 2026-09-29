# -*- coding: utf-8 -*-
"""
orchestrator/knowledge.py
=========================
"총괄이 아는 세계" (사용자 결정 2026-09-30: 정답지 방지).

환경 시뮬레이터는 진짜 세계다. 총괄의 판단은 진짜 상태를 직접 보지 않고,
아래 경로로 알게 된 사실만 쓴다. 모든 사실은 시뮬레이션 시각·출처와 함께 장부(knowledge 표)에 쌓인다.

  FIRE    신고(시나리오 이벤트·외부 API), 정찰 결과(불 발견 / 관측했지만 불 없음)
  WEATHER 기상청 관측소 관측(재생기, 그 시각까지 나온 값만), 현장 모의 측정

판단에 쓰는 "화면(view)" 은 engine.view() 가 만든다:
  지도(변하지 않는 지형·건물·보호대상) + 여기 쌓인 사실 + 분석(위험 칸·확산 예측, 입력은 이 사실만)
"""

import csv
import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Dict, List, Optional

from .ledger import Ledger

KST = timezone(timedelta(hours=9))
BURNING_STATUSES = ("REPORTED", "CONFIRMED")


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin(math.radians(lat2 - lat1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * 6371000.0 * math.asin(math.sqrt(min(1.0, max(0.0, a))))


class Knowledge:
    def __init__(self, ledger: Ledger):
        self.ledger = ledger

    # ------------------------------------------------------------------ 화재
    def report_fire(self, cell_id: str, sim_time_s: float, source: str, key: str, detail: dict = None) -> bool:
        return self.ledger.add_knowledge("FIRE", key, cell_id, sim_time_s, source,
                                         {"cell_id": cell_id, "status": "REPORTED", **(detail or {})})

    def record_observation(self, obs: dict, sim_time_s: float) -> None:
        """정찰 결과: 발견한 칸은 CONFIRMED, 관측 범위 안에서 불이 없던 칸은 OBSERVED_CLEAR"""
        found = {d["cell_id"]: d.get("fire_state") for d in obs.get("detections", [])}
        for cid in set(obs.get("covered_cells", [])) | set(found):
            status = "CONFIRMED" if found.get(cid) == "BURNING" else (
                "CONFIRMED_BURNED" if found.get(cid) == "BURNED" else "OBSERVED_CLEAR")
            self.ledger.add_knowledge("FIRE", f"{obs['observation_id']}:{cid}", cid, sim_time_s,
                                      obs.get("source", "OBSERVATION"),
                                      {"cell_id": cid, "status": status, "observation_id": obs["observation_id"],
                                       "resource_id": obs.get("resource_id")})

    def fire_states(self, now_s: float) -> Dict[str, dict]:
        """칸별 가장 최근 사실 (now 이전)"""
        out = {}
        for k in self.ledger.knowledge("FIRE"):
            if k["sim_time_s"] is not None and k["sim_time_s"] > now_s:
                continue
            out[k["subject"]] = {**k["body"], "sim_time_s": k["sim_time_s"], "source": k["source"]}
        return out

    def known_fires(self, now_s: float) -> Dict[str, dict]:
        return {c: v for c, v in self.fire_states(now_s).items() if v["status"] in BURNING_STATUSES}

    # ------------------------------------------------------------------ 기상
    def add_weather(self, key: str, sim_time_s: float, source: str, body: dict) -> bool:
        subject = body.get("station_id") or body.get("cell_id")
        return self.ledger.add_knowledge("WEATHER", key, subject, sim_time_s, source, body)

    def weather_for(self, lat: float, lon: float, cell_id: Optional[str], now_s: float) -> Optional[dict]:
        """목표 칸의 최신 현장 측정 → 없으면 목표에서 가장 가까운 관측소의 최신 관측. 보간하지 않는다."""
        obs = [k for k in self.ledger.knowledge("WEATHER") if k["sim_time_s"] is None or k["sim_time_s"] <= now_s]
        field = [k for k in obs if k["body"].get("kind") == "FIELD" and cell_id and k["body"].get("cell_id") == cell_id]
        if field:
            k = field[-1]
            return {**k["body"], "sim_time_s": k["sim_time_s"], "source": k["source"], "basis": "FIELD_AT_TARGET_CELL"}
        latest = {}
        for k in obs:
            if k["body"].get("kind") == "STATION":
                latest[k["subject"]] = k
        best = None
        for k in latest.values():
            b = k["body"]
            if b.get("lat") is None or lat is None:
                continue
            d = _haversine_m(lat, lon, b["lat"], b["lon"])
            if best is None or (d, k["subject"]) < best[0]:
                best = ((d, k["subject"]), k)
        if best is None:
            return None
        k = best[1]
        return {**k["body"], "sim_time_s": k["sim_time_s"], "source": k["source"],
                "basis": "NEAREST_STATION", "distance_m": round(best[0][0], 1)}

    def latest_station_obs(self, now_s: float) -> List[dict]:
        latest = {}
        for k in self.ledger.knowledge("WEATHER"):
            if k["body"].get("kind") == "STATION" and (k["sim_time_s"] is None or k["sim_time_s"] <= now_s):
                latest[k["subject"]] = {**k["body"], "sim_time_s": k["sim_time_s"], "source": k["source"]}
        return sorted(latest.values(), key=lambda b: b.get("station_id") or "")


# ---------------------------------------------------------------------------
# 기상 관측 공급원
# ---------------------------------------------------------------------------

class KmaAsosReplay:
    """기상청 ASOS 시간자료 재생기. 시뮬레이션 시각까지 관측된 행만 내보낸다 (미래 값 금지)."""

    source = "KMA_ASOS_HOURLY"
    FIELDS = {"ta": "temperature_c", "ws": "wind_ms", "wd": "wind_dir_deg", "hm": "humidity_pct"}

    def __init__(self, csv_path: Path, stations_path: Path):
        self.rows = list(csv.DictReader(open(csv_path, encoding="utf-8-sig")))
        meta = json.load(open(stations_path, encoding="utf-8"))
        self.stations = meta["stations"]
        self.station_source = meta.get("source")

    def observations(self, sim_time_s: float, scenario_start_kst: Optional[str]) -> List[dict]:
        if not scenario_start_kst:
            return []
        start = datetime.fromisoformat(scenario_start_kst)
        out = []
        for r in self.rows:
            t = datetime.strptime(r["tm"], "%Y-%m-%d %H:%M").replace(tzinfo=KST)
            offset = (t - start).total_seconds()
            if offset > sim_time_s:
                continue
            st = self.stations.get(r["stnId"], {})
            values = {}
            for src, dst in self.FIELDS.items():
                v = (r.get(src) or "").strip()
                if v != "":
                    values[dst] = float(v)
            out.append({"key": f"KMA:{r['stnId']}:{r['tm']}", "sim_time_s": offset, "source": self.source,
                        "body": {"kind": "STATION", "station_id": r["stnId"], "station_name": r.get("stnNm"),
                                 "lat": st.get("lat"), "lon": st.get("lon"), "observed_kst": r["tm"],
                                 "values": values, "station_coord_source": self.station_source}})
        return out


class EnvStationFeed:
    """시험용: 환경(진짜 세계)의 기상값을 한 지점 관측소가 잰 것처럼 내보낸다 (TEST_ONLY).
    실제 운용에서는 KmaAsosReplay 같은 관측 자료를 쓴다."""

    source = "FIXTURE_STATION"

    def __init__(self, env, lat: float, lon: float, station_id: str = "FIXTURE"):
        self.env, self.lat, self.lon, self.station_id = env, lat, lon, station_id

    def observations(self, sim_time_s: float, scenario_start_kst: Optional[str]) -> List[dict]:
        values = {k: v for k, v in {"wind_ms": self.env.wind_ms, "wind_dir_deg": self.env.wind_dir_deg,
                                    **(self.env.weather or {})}.items() if v is not None}
        key = f"FIXTURE:{sim_time_s}:{json.dumps(values, sort_keys=True)}"
        return [{"key": key, "sim_time_s": sim_time_s, "source": self.source,
                 "body": {"kind": "STATION", "station_id": self.station_id, "station_name": "시험 관측소",
                          "lat": self.lat, "lon": self.lon, "values": values}}]


class InjectedAnalysis:
    """분석(위험 칸·확산 예측) 자리의 시험 주입값 (TEST_INJECTED). 환경팀 분석 기능(ENV-04) 전까지 쓴다.

    - 환경(진짜 세계)을 참조하지 않는다. 값은 생성 시 또는 시험이 직접 넣는다 (시나리오 파일의
      injected_analysis). 따라서 진짜 세계의 화재·위험·확산이 바뀌어도 이 값은 바뀌지 않는다.
    - 칸마다 requires_known_fire 를 두면, 총괄이 그 불을 알게 된 뒤(신고·정찰 확인)에만 내보낸다.
    - 내보내는 칸에는 derived_from_observation=False, analysis_source=TEST_INJECTED 를 붙인다.
      관측으로부터 계산한 예측이 아니며, 이 값으로 총괄 예측 성능이 검증됐다고 표시하지 않는다.
    """

    source = "TEST_INJECTED"

    def __init__(self, risk_cells=None, spread_forecast=None, forecast_ref=None):
        self.risk_cells = list(risk_cells or [])
        self.spread_forecast = list(spread_forecast or [])
        self.forecast_ref = forecast_ref
        self.last_input = None

    def _visible(self, cells, known):
        out = []
        for c in cells:
            need = c.get("requires_known_fire")
            needs = [need] if isinstance(need, str) else list(need or [])
            if all(n in known for n in needs):
                out.append({**c, "derived_from_observation": False, "analysis_source": self.source})
        return out

    def analyze(self, belief: dict) -> dict:
        self.last_input = belief
        known = {c["cell_id"] for c in belief.get("known_fires", [])}
        spread = self._visible(self.spread_forecast, known)
        return {"risk_cells": self._visible(self.risk_cells, known), "spread_forecast": spread,
                "forecast_ref": ({**self.forecast_ref, "injected": True} if (self.forecast_ref and spread) else None),
                "source": self.source, "derived_from_observation": False}
