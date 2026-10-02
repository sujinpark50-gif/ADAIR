# -*- coding: utf-8 -*-
"""ENV-04 분석 기능 — `analyze(belief) → {risk_cells, spread_forecast, forecast_ref, source}`.

정답지 방지 규칙 (총괄 요청서 §0, ENV-04):
- 진짜 세계(CA 의 화재 상태·진짜 바람)를 **읽지 않는다.** 생성할 때 정적 지도 배열(지면고도·연료량·습도·건물유형)
  사본만 받아 두고, 그 뒤로는 belief(총괄이 아는 세계)만 입력으로 쓴다. 이 클래스는 grid·engine 참조를 갖지 않는다.
- 화재 출발점 = belief.known_fires (신고·정찰 확인으로 총괄이 지금 불이라고 믿는 칸).
- 바람 = belief.field_weather(유효한 현장 측정) 중 불에 가장 가까운 것 → 없으면 station_weather(관측소) 중 가장 가까운 것.
  둘 다 없으면 바람 0 으로 가정하고 그 사실을 forecast_ref.wind_basis 에 남긴다.
- 확산식은 김은주님 CA 의 칸→이웃 확산 확률식(fire_model._calculate_spread_probability)과 같은 식을 쓴다.
  확률 p 로 매 스텝(tick_s) 번질 때 '절반의 경우 번지는' 대기 시간(기하분포 중앙값, 최소 1스텝)
  tick_s · max(1, ln0.5/ln(1-p)) 를 간선 비용으로 두고, 알려진 불에서 최단 도달 시간(Dijkstra)을
  '예상 도달 시각'으로 낸다. 결정론 — 같은 belief 면 같은 결과.
  비용식 선택 근거(오프라인 보정, 2026-10, seed 42·점화 19_142·인제 5.7m/s 160°·30분):
    평균 대기 tick_s/p     → 실제 CA 화재 칸 대비 재현율 0.16, 정밀도 1.00
    중앙값 대기(채택)       → 재현율 0.45, 정밀도 0.99
  여러 불 칸이 동시에 번지는 효과를 넣지 않아 여전히 **과소 예측**이다 (환경팀 보정 대상).

결과는 예측이며 진짜 세계와 다를 수 있다 (그래서 현장 측정의 가치가 생긴다).
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

MODEL_VERSION = "BELIEF-CA-EXPECTED-ARRIVAL-v1"

_NEIGHBORS_8 = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
_NEIGHBORS_4 = [(0, -1), (-1, 0), (1, 0), (0, 1)]


def _parse(cid) -> Optional[Tuple[int, int]]:
    if not isinstance(cid, str) or "_" not in cid:
        return None
    a, _, b = cid.partition("_")
    try:
        return int(a), int(b)
    except ValueError:
        return None


def _dist_m(lat1, lon1, lat2, lon2) -> float:
    k = 111_320.0
    return math.hypot((lon2 - lon1) * k * math.cos(math.radians((lat1 + lat2) / 2)), (lat2 - lat1) * k)


class BeliefSpreadAnalysis:
    source = "ENV_BELIEF_ANALYSIS"

    def __init__(self, static: dict, ca_config: dict, *, tick_s: float = 60.0, horizon_s: float = 1800.0,
                 max_forecast_cells: int = 200, n_risk_cells: int = 10):
        # 정적 배열만 복사해 둔다 (진짜 세계 참조 없음)
        self.n_cols, self.n_rows = int(static["n_cols"]), int(static["n_rows"])
        self.cell_size_m = float(static["cell_size_m"])
        self.lat = np.array(static["lat"], dtype=float)
        self.lon = np.array(static["lon"], dtype=float)
        self.elev = np.array(static["elevation"], dtype=float)
        self.fuel = np.array(static["fuel_amount"], dtype=float)
        self.moist = np.array(static["moisture"], dtype=float)
        self.btype = np.array(static["building_type"], dtype=int)
        ca = dict(ca_config.get("fire_ca", {}))
        self.base_p = float(ca.get("base_spread_probability", 0.2))
        self.slope_w = float(ca.get("slope_factor_weight", 0.05))
        self.wind_w = float(ca.get("wind_factor_weight", 0.1))
        self.moist_w = float(ca.get("moisture_dampening_weight", 1.0))
        self.neighbors = _NEIGHBORS_4 if ca.get("neighbor_mode") == "4-neighbor" else _NEIGHBORS_8
        rs = dict(ca_config.get("risk_scoring", {}))
        w = dict(rs.get("composite_weights", {}))
        tot = sum(float(v) for v in w.values()) or 1.0
        self.w = {k: float(v) / tot for k, v in w.items()}
        self.bvals = dict(rs.get("building_risk_values", {}))
        self.tick_s, self.horizon_s = float(tick_s), float(horizon_s)
        self.max_forecast_cells, self.n_risk_cells = int(max_forecast_cells), int(n_risk_cells)
        self.last_input = None

    # ------------------------------------------------------------------
    def _idx(self, col, row) -> Optional[int]:
        if 0 <= col < self.n_cols and 0 <= row < self.n_rows:
            return row * self.n_cols + col
        return None

    def _pick_wind(self, belief: dict, fires: List[int]) -> Tuple[float, float, dict]:
        lat0 = float(np.mean([self.lat[i] for i in fires]))
        lon0 = float(np.mean([self.lon[i] for i in fires]))

        def best(items, kind):
            cands = []
            for it in items or []:
                v = it.get("values") or {}
                if it.get("lat") is None or it.get("lon") is None:
                    continue
                if not all(isinstance(v.get(k), (int, float)) and not isinstance(v.get(k), bool)
                           for k in ("wind_ms", "wind_dir_deg")):
                    continue
                cands.append((_dist_m(lat0, lon0, it["lat"], it["lon"]),
                              str(it.get("cell_id") or it.get("station_id")), it))
            if not cands:
                return None
            d, _, it = min(cands, key=lambda c: (c[0], c[1]))
            v = it["values"]
            return float(v["wind_ms"]), float(v["wind_dir_deg"]), {
                "wind_basis": kind, "wind_from": it.get("cell_id") or it.get("station_id"),
                "wind_source": it.get("source"), "wind_observed_sim_s": it.get("sim_time_s"),
                "distance_to_fire_m": round(d, 1)}

        got = best(belief.get("field_weather"), "FIELD_MEASUREMENT") or best(belief.get("station_weather"), "STATION")
        if got is None:
            return 0.0, 270.0, {"wind_basis": "NONE_ASSUMED_CALM"}
        return got

    def _p(self, s: int, t: int, dc: int, dr: int, wx: float, wy: float, wind: float) -> float:
        # fire_model._calculate_spread_probability 와 같은 식 (입력만 아는 세계의 바람)
        if self.fuel[t] <= 0.0:
            return 0.0
        mf = max(0.0, 1.0 - self.moist_w * float(self.moist[t]))
        if mf <= 0.0:
            return 0.0
        dist = math.hypot(dc, dr)
        grad = float(self.elev[t] - self.elev[s]) / (dist * self.cell_size_m)
        align = (dc / dist) * wx + (dr / dist) * wy
        p = self.base_p * float(self.fuel[t]) * mf * math.exp(self.slope_w * grad) * math.exp(self.wind_w * wind * align)
        return float(max(0.0, min(1.0, p)))

    def _edge_s(self, p: float) -> float:
        if p >= 1.0:
            return self.tick_s
        return self.tick_s * max(1.0, math.log(0.5) / math.log(1.0 - p))

    def _exposure(self, i: int) -> Dict[str, float]:
        bt = int(self.btype[i])
        m = {1: ("human", "residential"), 2: ("infrastructure", "important"), 3: ("infrastructure", "critical"),
             4: ("secondary_hazard", "secondary_hazard"), 5: ("property", "commercial"),
             6: ("property", "industrial")}.get(bt)
        return {m[0]: float(self.bvals.get(m[1], 0.0))} if m else {}

    # ------------------------------------------------------------------
    def analyze(self, belief: dict) -> dict:
        self.last_input = belief
        now = float(belief.get("simulation_time_s") or 0.0)
        fires, ignored = [], []
        for c in belief.get("known_fires") or []:
            p = _parse(c.get("cell_id"))
            i = None if p is None else self._idx(*p)
            (fires if i is not None else ignored).append(i if i is not None else c.get("cell_id"))
        fires = sorted({int(i) for i in fires})
        empty = {"risk_cells": [], "spread_forecast": [], "forecast_ref": None, "source": self.source,
                 "derived_from_observation": True, "input_summary": {"known_fires": 0, "ignored_cells": ignored}}
        if not fires:
            return empty
        wind, wdir, wref = self._pick_wind(belief, fires)
        towards = math.radians((wdir + 180.0) % 360.0)
        wx, wy = math.sin(towards), -math.cos(towards)      # 격자 좌표(행이 아래로 증가) 기준, CA 와 동일

        # 알려진 불에서 예상 도달 시각 (Dijkstra, horizon 까지)
        best = {i: 0.0 for i in fires}
        heap = [(0.0, i) for i in fires]
        heapq.heapify(heap)
        while heap:
            t, s = heapq.heappop(heap)
            if t > best.get(s, math.inf):
                continue
            sc, sr = s % self.n_cols, s // self.n_cols
            for dc, dr in self.neighbors:
                j = self._idx(sc + dc, sr + dr)
                if j is None:
                    continue
                p = self._p(s, j, dc, dr, wx, wy, wind)
                if p <= 0.0:
                    continue
                nt = t + self._edge_s(p)
                if nt <= self.horizon_s and nt < best.get(j, math.inf):
                    best[j] = nt
                    heapq.heappush(heap, (nt, j))
        fire_set = set(fires)
        reach = sorted(((float(t), int(i)) for i, t in best.items() if i not in fire_set), key=lambda x: (x[0], x[1]))

        def cid(i):
            return f"{i % self.n_cols}_{i // self.n_cols}"

        spread = [{"cell_id": cid(i), "expected_arrival_s": round(float(t), 1),
                   "expected_arrival_sim_s": round(now + float(t), 1)} for t, i in reach[:self.max_forecast_cells]]
        scored = []
        for t, i in reach:
            s_spread = 1.0 - t / self.horizon_s
            exp = self._exposure(i)
            score = self.w.get("spread", 0.0) * s_spread + sum(self.w.get(k, 0.0) * v for k, v in exp.items())
            basis = "DOWNWIND_EXPECTED_ARRIVAL" + ("+BUILDING:" + ",".join(sorted(exp)) if exp else "")
            scored.append((-score, t, i, round(float(score), 4), basis))
        scored.sort()
        risk = [{"cell_id": cid(i), "risk_score": sc, "expected_arrival_s": round(float(t), 1), "basis": basis}
                for _, t, i, sc, basis in scored[:self.n_risk_cells]]
        return {
            "risk_cells": risk, "spread_forecast": spread, "source": self.source, "derived_from_observation": True,
            "forecast_ref": {"model_version": MODEL_VERSION, "issued_sim_s": now, "horizon_s": self.horizon_s,
                             "tick_s_assumption": self.tick_s, "wind_ms_used": wind, "wind_dir_deg_used": wdir,
                             **wref, "cells_reached": len(reach), "cells_returned": len(spread)},
            "input_summary": {"known_fires": len(fires), "ignored_cells": ignored,
                              "fire_states": len(belief.get("fire_states") or {})},
        }
