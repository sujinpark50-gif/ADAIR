# -*- coding: utf-8 -*-
"""
orchestrator/resources.py
=========================
UAV·UGV Local Agent HTTP 연결. 요청/응답 형식은 각 담당 코드 기준이다.
  UAV: uav/uav-agent/models.py  (1대 = 서버 1개, config.UAV_ENDPOINTS)
  UGV: ugv/api_models.py        (서버 1개가 여러 대 관리, config.UGV_SERVER_URL)

원칙:
- 응답 원문(raw)을 버리지 않는다.
- 네트워크 오류·timeout 은 "실패"가 아니라 "불명(UNKNOWN)"이다 (요청서 §5-5).
- 실행 상태 조회 404 도 FAILED 가 아니다. UAV 는 재시작하면 기록을 잊는다.
- 실행 요청의 task_id 칸에는 총괄 attempt_id 를 넣는다. UAV/UGV 에 attempt 전용
  칸이 없어서, 실행시도마다 서로 다른 식별자로 추적하기 위함이다 (UAV·UGV 계약 대기).
"""

import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import httpx

from . import config
from .models import ResourceView

# 자원 종류별 관측 능력. 상태 API 가 센서 목록을 주지 않아 담당 코드 값을 적는다.
#   UAV: uav/uav-agent/config.py AVAILABLE_SENSORS (UAV_CODE)
#   UGV: ugv/api_models.py — 도착 시 ROAD_STATUS 관측만 제공, 화재 관측 아님
TYPE_CAPABILITIES = {
    "UAV": ("THERMAL", "RGB"),
    "UGV": ("ROAD_STATUS",),
    "FIRE_ENGINE": (),
}


def capabilities(resource_type: str) -> tuple:
    """담당 코드 기준 능력 + 총괄 쪽 모의 센서 능력(TEST_ONLY, config.SIMULATED_CAPABILITIES)"""
    return TYPE_CAPABILITIES.get(resource_type, ()) + tuple(config.SIMULATED_CAPABILITIES.get(resource_type, ()))


class Unreachable(Exception):
    """연결 실패·timeout. 상대 상태를 알 수 없음"""


def _parse_iso(ts: Optional[str]) -> Optional[float]:
    if not ts:
        return None
    try:
        return datetime.fromisoformat(ts.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def _call(client: httpx.Client, method: str, url: str, **kw) -> httpx.Response:
    try:
        return client.request(method, url, **kw)
    except (httpx.TimeoutException, httpx.TransportError) as e:
        raise Unreachable(f"{method} {url}: {type(e).__name__}: {e}") from e


class UavClient:
    resource_type = "UAV"

    def __init__(self, endpoints: Optional[Dict[str, str]] = None, client: Optional[httpx.Client] = None):
        self.endpoints = dict(config.UAV_ENDPOINTS if endpoints is None else endpoints)
        self.client = client or httpx.Client(timeout=config.HTTP_TIMEOUT_S)

    def resource_ids(self) -> List[str]:
        return sorted(r for r, u in self.endpoints.items() if u and u != "TBD")

    def _url(self, rid: str, path: str) -> str:
        return f"{self.endpoints[rid].rstrip('/')}/uav/{rid}{path}"

    def state(self, rid: str) -> ResourceView:
        r = _call(self.client, "GET", self._url(rid, "/state"))
        if r.status_code != 200:
            raise Unreachable(f"state {rid}: HTTP {r.status_code}")
        d = r.json()
        pos, health = d.get("position") or {}, d.get("health") or {}
        failsafe = health.get("failsafe")
        return ResourceView(
            resource_id=rid, resource_type="UAV", base=rid.split("-")[0],
            lat=pos.get("lat"), lon=pos.get("lon"), alt_amsl_m=pos.get("alt_m_amsl"),
            ready=(failsafe is False and d.get("current_task_id") is None),
            current_task_id=d.get("current_task_id"),
            battery_pct=(d.get("battery") or {}).get("percent"),
            sensors=capabilities("UAV"), comm=d.get("link_quality"), failsafe=failsafe,
            phase=d.get("flight_mode"), fetched_wall=time.time(),
            source_timestamp=d.get("timestamp"), raw=d,
        )

    def source_age_s(self, view: ResourceView) -> Optional[float]:
        t = _parse_iso(view.source_timestamp)
        return None if t is None else max(0.0, time.time() - t)

    def evaluate(self, rid: str, payload: dict) -> dict:
        r = _call(self.client, "POST", self._url(rid, "/evaluate"), json=payload)
        if r.status_code != 200:
            raise Unreachable(f"evaluate {rid}: HTTP {r.status_code} {r.text[:200]}")
        d = r.json()
        return {"verdict": d.get("verdict"), "reason": d.get("reason"), "eta_sec": d.get("eta_sec"),
                "counter_offer": d.get("counter_offer"), "constraints": d.get("constraints") or {},
                "echo": {"task_id": d.get("task_id"), "decision_id": d.get("decision_id"),
                         "resource_id": d.get("uav_id")},
                "raw": d}

    def execute(self, rid: str, payload: dict) -> Tuple[str, dict]:
        """("ACK"|"REFUSED"|"UNKNOWN", 상세). REFUSED 는 실행되지 않았음이 확실한 경우만."""
        try:
            r = _call(self.client, "POST", self._url(rid, "/execute"), json=payload)
        except Unreachable as e:
            return "UNKNOWN", {"error": str(e)}
        if r.status_code == 200:
            return "ACK", r.json()
        if r.status_code in (400, 404, 409, 422):
            return "REFUSED", {"http_status": r.status_code, "body": r.text[:500]}
        return "UNKNOWN", {"http_status": r.status_code, "body": r.text[:500]}

    def abort(self, rid: str, exec_task_id: str, reason: str) -> Tuple[str, dict]:
        """중단 요청 (드론팀 UAV-09). ("OK"|"NOT_FOUND"|"UNREACHABLE", TaskStatus). 같은 요청 반복은 처음 결과."""
        try:
            r = _call(self.client, "POST", self._url(rid, f"/task/{exec_task_id}/abort"), json={"reason": reason})
        except Unreachable as e:
            return "UNREACHABLE", {"error": str(e)}
        if r.status_code == 200:
            return "OK", r.json()
        if r.status_code == 404:
            return "NOT_FOUND", {"body": r.text[:300]}
        return "UNREACHABLE", {"http_status": r.status_code}

    def task_status(self, rid: str, exec_task_id: str) -> Tuple[str, dict]:
        """("OK"|"NOT_FOUND"|"UNREACHABLE", 상세)"""
        try:
            r = _call(self.client, "GET", self._url(rid, f"/task/{exec_task_id}"))
        except Unreachable as e:
            return "UNREACHABLE", {"error": str(e)}
        if r.status_code == 200:
            return "OK", r.json()
        if r.status_code == 404:
            return "NOT_FOUND", {"body": r.text[:300]}
        return "UNREACHABLE", {"http_status": r.status_code}


class UgvClient:
    resource_type = "UGV"

    def __init__(self, base_url: Optional[str] = None, client: Optional[httpx.Client] = None):
        self.base_url = (base_url or config.UGV_SERVER_URL or "").rstrip("/")
        self.client = client or httpx.Client(timeout=config.HTTP_TIMEOUT_S)

    def states(self) -> List[ResourceView]:
        if not self.base_url or self.base_url == "TBD":
            return []
        r = _call(self.client, "GET", f"{self.base_url}/ugv")
        if r.status_code != 200:
            raise Unreachable(f"ugv list: HTTP {r.status_code}")
        out = []
        for d in r.json():
            pos = d.get("position") or {}
            rtype = d.get("resource_type") or "UGV"
            out.append(ResourceView(
                resource_id=d["resource_id"], resource_type=rtype, base=d.get("base"),
                lat=pos.get("lat"), lon=pos.get("lon"),
                ready=(d.get("state") == "READY" and d.get("current_task_id") is None),
                current_task_id=d.get("current_task_id"),
                sensors=capabilities(rtype), comm=None,
                phase=d.get("driver_status"), fetched_wall=time.time(),
                source_timestamp=str(d.get("updated_at")), raw=d,
            ))
        return out

    @staticmethod
    def source_age_s(view: ResourceView) -> Optional[float]:
        t = view.raw.get("updated_at")
        if not isinstance(t, (int, float)) or t <= 0:
            return None          # 0 은 텔레메트리 미수신
        return max(0.0, time.time() - t)

    def evaluate(self, rid: str, payload: dict) -> dict:
        r = _call(self.client, "POST", f"{self.base_url}/ugv/{rid}/evaluate", json=payload)
        if r.status_code != 200:
            raise Unreachable(f"ugv evaluate {rid}: HTTP {r.status_code}")
        d = r.json()
        return {"verdict": d.get("verdict"), "reason": d.get("reason"), "eta_sec": d.get("eta_sec"),
                "counter_offer": None, "constraints": {"target_node": d.get("target_node"),
                                                       "path": d.get("path")},
                "echo": {"task_id": d.get("task_id"), "decision_id": d.get("decision_id"),
                         "resource_id": d.get("resource_id")},
                "raw": d}

    def execute(self, rid: str, payload: dict) -> Tuple[str, dict]:
        try:
            r = _call(self.client, "POST", f"{self.base_url}/ugv/{rid}/execute", json=payload)
        except Unreachable as e:
            return "UNKNOWN", {"error": str(e)}
        if r.status_code == 200:
            return "ACK", r.json()
        if r.status_code in (400, 404, 409, 422):
            return "REFUSED", {"http_status": r.status_code, "body": r.text[:500]}
        return "UNKNOWN", {"http_status": r.status_code, "body": r.text[:500]}

    def task_status(self, rid: str, exec_task_id: str) -> Tuple[str, dict]:
        try:
            r = _call(self.client, "GET", f"{self.base_url}/ugv/{rid}/task/{exec_task_id}")
        except Unreachable as e:
            return "UNREACHABLE", {"error": str(e)}
        if r.status_code == 200:
            return "OK", r.json()
        if r.status_code == 404:
            return "NOT_FOUND", {"body": r.text[:300]}
        return "UNREACHABLE", {"http_status": r.status_code}
