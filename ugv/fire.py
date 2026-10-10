# -*- coding: utf-8 -*-
"""ugv/fire.py — 안전 판단에 쓰는 '알려진 화재 영역'과 그 경계까지의 거리.

출처 (FireSource):
  EnvHttpFireSource : 환경 계약 서버 GET /snapshot 의 fire_cells (총괄과 같은 환경 인스턴스, 권장)
  StaticFireSource  : 시험·수동 주입용. connectors/fire_connector.py 의 EnvironmentState.fire_cells 형식
                      ({cell_id:"x_y", x, y, fire_state, location_lat, location_lon}) 도 받는다

주의: 여기서 읽는 화재 칸은 환경이 가진 전체 정보다. **안전거리 판단에만 쓰고 차량 관측 결과로 보고하지 않는다**
(보고에는 safety_basis 로만 출처를 남긴다). 차량 관측은 ugv/observe.py 가 따로 만든다 (4단계).

칸 모양: 환경 격자(EPSG:5186) 90 m 정사각형. 지역 평면(ugv/geo.py)에서는 약 0.7° 돌아 있지만 축 정렬 정사각형으로
근사한다 (경계 거리 오차 1 m 미만).
"""

import math
import time
from typing import Iterable, List, Optional

from . import config
from .geo import FRAME

_BUCKET_M = 250.0
ALONG_STEP_M = 5.0          # 선 위 이격 검사 간격 (칸 반폭 45 m 에 비해 충분히 촘촘)


def densify(pts, step_m: float = ALONG_STEP_M):
    """점열을 step_m 이하 간격으로 (끝점 포함)."""
    pts = list(pts)
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        n = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / step_m)))
        out += [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(1, n + 1)]
    return out


class FireArea:
    """화재 칸 묶음. distance(x, y) = 가장 가까운 화재 칸 경계까지 거리 (칸 안이면 0, 칸이 없으면 inf)."""

    def __init__(self, cells: Iterable[dict], *, source: str, states=config.FIRE_STATES_FOR_SAFETY,
                 version=None, sim_time_s: Optional[float] = None):
        self.source, self.version, self.sim_time_s = source, version, sim_time_s
        self.cells: List[dict] = []
        self._grid: dict = {}
        for c in cells:
            if c.get("fire_state") not in states:
                continue
            lat = c.get("lat", c.get("location_lat"))
            lon = c.get("lon", c.get("location_lon"))
            if lat is None or lon is None:
                continue
            x, y = FRAME.to_xy(float(lat), float(lon))
            half = float(c.get("cell_size_m") or 90.0) / 2.0
            rec = {"cell_id": c.get("cell_id"), "x": x, "y": y, "half": half, "fire_state": c.get("fire_state")}
            self.cells.append(rec)
            self._grid.setdefault((int(x // _BUCKET_M), int(y // _BUCKET_M)), []).append(rec)

    def __len__(self):
        return len(self.cells)

    @staticmethod
    def _d(rec, x, y):
        return math.hypot(max(abs(x - rec["x"]) - rec["half"], 0.0), max(abs(y - rec["y"]) - rec["half"], 0.0))

    def distance(self, x: float, y: float, max_m: float = 5000.0) -> float:
        """가장 가까운 화재 칸 경계까지 거리 (m). max_m 보다 멀면 inf."""
        if not self.cells:
            return math.inf
        if max_m >= 20000.0:
            best = min(self._d(r, x, y) for r in self.cells)
        else:
            bx, by, k = int(x // _BUCKET_M), int(y // _BUCKET_M), int(max_m // _BUCKET_M) + 1
            best = min((self._d(rec, x, y) for i in range(bx - k, bx + k + 1) for j in range(by - k, by + k + 1)
                        for rec in self._grid.get((i, j), ())), default=math.inf)
        return best if best <= max_m else math.inf

    def clearance_along(self, pts, max_m: float = 5000.0, step_m: float = ALONG_STEP_M) -> float:
        """선 위 최소 이격. 꼭짓점만 보지 않고 선분을 step_m 간격으로 나눠 본다 (예전에는 꼭짓점만 봐서 긴 직선 선분이
        화재 칸을 가로지르는 것을 놓쳤다 — 2026-10-10 Gazebo G5 이탈 경로가 칸 안을 지남)."""
        return min((self.distance(x, y, max_m) for x, y in densify(pts, step_m)), default=math.inf)

    def summary(self) -> dict:
        return {"source": self.source, "version": self.version, "sim_time_s": self.sim_time_s,
                "cells": len(self.cells), "states": list(config.FIRE_STATES_FOR_SAFETY),
                "used_for": "SAFETY_ONLY_NOT_REPORTED_AS_OBSERVATION"}


EMPTY = FireArea([], source="NONE")


class StaticFireSource:
    def __init__(self, cells: Iterable[dict] = (), source: str = "STATIC"):
        self.set(cells, source)

    def set(self, cells: Iterable[dict], source: str = "STATIC", sim_time_s: Optional[float] = None):
        self._area = FireArea(list(cells), source=source, version=time.time(), sim_time_s=sim_time_s)

    def area(self) -> FireArea:
        return self._area


class EnvHttpFireSource:
    """환경 계약 서버 /snapshot. state_version 이 같으면 다시 만들지 않는다. 연결 실패 시 마지막 값 + stale 표시.
    읽기는 별도 스레드가 poll_s 마다 한다 — area() 는 마지막 값을 바로 돌려준다. (예전에는 감시 루프 안에서 HTTP 로 읽고
    화재 칸 묶음을 만들어, 칸이 수만 개면 이벤트 루프가 막혀 MAVSDK 콜백이 밀리고 서버가 죽었다 — 2026-10-10 Gazebo G6)"""

    def __init__(self, base_url: str, client=None, poll_s: float = config.ENV_POLL_S, background: bool = True):
        import httpx
        self.url = base_url.rstrip("/")
        self.client = client or httpx.Client(timeout=10.0)
        self.poll_s = poll_s
        self._area, self._fetched, self._error = EMPTY, 0.0, None
        self.run_id = None
        self.last_fetch_s = None
        self._thread = None
        if background:
            import threading
            self._thread = threading.Thread(target=self._loop, name="env-fire-poll", daemon=True)
            self._thread.start()

    def _fetch(self):
        t0 = time.monotonic()
        try:
            r = self.client.get(f"{self.url}/snapshot")
            r.raise_for_status()
            snap = r.json()
            if snap.get("state_version") != self._area.version or snap.get("run_id") != self.run_id:
                self._area = FireArea(snap.get("fire_cells") or [], source=f"ENV_SNAPSHOT:{self.url}",
                                      version=snap.get("state_version"), sim_time_s=snap.get("simulation_time_s"))
                self.run_id = snap.get("run_id")
            self._error = None
        except Exception as e:  # noqa: BLE001 — 환경이 잠깐 안 돼도 마지막 값으로 안전 판단을 계속한다
            self._error = f"{type(e).__name__}: {e}"
        self._fetched = time.monotonic()
        self.last_fetch_s = round(self._fetched - t0, 3)

    def _loop(self):
        while True:
            self._fetch()
            time.sleep(self.poll_s)

    def area(self) -> FireArea:
        if self._thread is None and time.monotonic() - self._fetched >= self.poll_s:
            self._fetch()
        return self._area

    def status(self) -> dict:
        return {"url": self.url, "run_id": self.run_id, "error": self._error, "last_fetch_s": self.last_fetch_s,
                "age_s": round(time.monotonic() - self._fetched, 1) if self._fetched else None, **self._area.summary()}


def build_source():
    if config.ENV_URL:
        return EnvHttpFireSource(config.ENV_URL)
    return StaticFireSource(source="NONE (UGV_ENV_URL 미설정)")
