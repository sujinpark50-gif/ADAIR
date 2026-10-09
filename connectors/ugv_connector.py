# -*- coding: utf-8 -*-
"""
connectors/ugv_connector.py
==============================
Ground Resource Connector (Contract 7번): UGV·소방차에 공통 사용.
단, resource_type / capability / state 의미는 구분한다.

연결 방식은 config.UGV_CONNECTION_MODE 로 고른다.
  mock        무작위 응답 (자리 확인용)
  integrated  같은 프로세스에서 도로 경로만으로 판단 (integration/bridge_ugv.py, 주행 없음)
  real        UGV 서버(ugv/server.py) HTTP. 주소 config.UGV_SERVER_URL
              판단 = POST /ugv/{id}/evaluate (움직이지 않음),
              관측 = POST /ugv/{id}/execute 뒤 GET /ugv/{id}/task/{key} 로 끝날 때까지 기다려 도착 관측을 돌려준다
              (main.py 는 Safety ALLOW 뒤에만 관측 함수를 부른다)
"""

import random
import time

import config
from interfaces.schema import ResourceStatus, LocalResponse, Observation


def get_ugv_status(base: str) -> list:
    if config.UGV_CONNECTION_MODE == "integrated":
        from integration import bridge_ugv
        return bridge_ugv.get_ugv_status(base)
    if config.UGV_CONNECTION_MODE == "mock":
        return _get_ugv_status_mock(base)
    return _real().status(base)


def send_ugv_command(task_id: str, assignment, force_response: str = None,
                     force_reason: str = None, decision_id: str = None) -> LocalResponse:
    if config.UGV_CONNECTION_MODE == "integrated":
        from integration import bridge_ugv
        return bridge_ugv.send_ugv_command(task_id, assignment, force_response, force_reason)
    if force_response is not None or config.UGV_CONNECTION_MODE == "mock":
        return _send_ugv_command_mock(assignment, force_response, force_reason)
    return _real().evaluate(task_id, decision_id, assignment)


def get_ugv_observation(resource_id: str, sim_time_s: float, task_id: str = None, decision_id: str = None) -> Observation:
    if config.UGV_CONNECTION_MODE == "integrated":
        from integration import bridge_ugv
        return bridge_ugv.get_ugv_observation(resource_id, sim_time_s, task_id, decision_id)
    if config.UGV_CONNECTION_MODE == "mock":
        return Observation(
            resource_id=resource_id,
            location_lat=config.BASE_A_LAT,
            location_lon=config.BASE_A_LON,
            simulation_time_s=sim_time_s,
            observation_type="ROAD_STATUS",
            value={"note": "mock observation"},
            task_id=task_id,
            decision_id=decision_id,
        )
    return _real().observe(resource_id, sim_time_s, task_id, decision_id)


def _get_ugv_status_mock(base: str) -> list:
    """거점당 UGV 2대 + 소방차 1대 (Contract 자원 구조 반영)"""
    base_lat = config.BASE_A_LAT if base == "A" else config.BASE_B_LAT
    base_lon = config.BASE_A_LON if base == "A" else config.BASE_B_LON

    statuses = []
    for i in range(1, 3):
        statuses.append(
            ResourceStatus(
                resource_id=f"{base}-ugv{i}",
                resource_type="UGV",
                base=base,
                location_lat=base_lat + random.uniform(-0.01, 0.01),
                location_lon=base_lon + random.uniform(-0.01, 0.01),
                state="READY",
                capability={
                    "eta_sec": round(random.uniform(180, 900), 1),
                    "road_blocked": random.choice([False, False, False, True]),
                    "reachable": True,
                },
            )
        )
    statuses.append(
        ResourceStatus(
            resource_id=f"{base}-fire1",
            resource_type="FIRE_ENGINE",
            base=base,
            location_lat=base_lat + random.uniform(-0.01, 0.01),
            location_lon=base_lon + random.uniform(-0.01, 0.01),
            state="READY",
            capability={
                "eta_sec": round(random.uniform(300, 1200), 1),
                "reachable": True,
            },
        )
    )
    return statuses


def _send_ugv_command_mock(assignment, force_response, force_reason) -> LocalResponse:
    if force_response is not None:
        return LocalResponse(resource_id=assignment.resource_id, response=force_response, reason=force_reason)

    response = random.choices(["ACCEPT", "REJECT"], weights=[0.85, 0.15])[0]
    reason = None
    if response == "REJECT":
        reason = random.choice(["ROAD_BLOCKED", "TARGET_UNREACHABLE"])
    return LocalResponse(resource_id=assignment.resource_id, response=response, reason=reason)


# 실행 키 (UAV-01·UGV-04). UAV/UGV 서버는 실행 기록을 파일에 남기고 '같은 키·다른 내용'을 409 로 거절한다.
# 이 커넥터의 task_id 는 프로세스마다 TASK_001 부터 다시 세므로, 실행 키에 이 프로세스 표식과 decision_id 를
# 붙여 실행(결정)마다 고유하게 만든다. 로그·관측의 task_id 는 그대로다.
import uuid as _uuid
_RUN_TAG = _uuid.uuid4().hex[:8]


def exec_key(task_id, decision_id):
    return f"{_RUN_TAG}:{task_id}:{decision_id}"


class _RealUgv:
    """UGV 서버 HTTP 연결. 판단(evaluate)에서 받아들인 목표를 기억했다가 관측 요청 때 실행(execute)한다
    (main.py 는 관측 호출에 목표를 넘기지 않으므로 (task_id, resource_id) 로 찾는다)."""

    def __init__(self, url: str):
        import requests                                  # real 모드에서만 필요
        self.http, self.url = requests, url.rstrip("/")
        self.accepted: dict = {}

    def _get(self, path, **kw):
        r = self.http.get(self.url + path, timeout=kw.pop("timeout", 5), **kw)
        r.raise_for_status()
        return r.json()

    def status(self, base: str) -> list:
        """서버에 닿지 않으면 빈 목록 (그 거점 지상자원 없음으로 본다)."""
        try:
            units = self._get("/ugv", params={"base": base})
        except self.http.RequestException:
            return []
        return [ResourceStatus(resource_id=u["resource_id"], resource_type=u["resource_type"], base=u["base"],
                               location_lat=u["position"]["lat"], location_lon=u["position"]["lon"], state=u["state"],
                               capability={"eta_sec": None, "reachable": None, "road_blocked": None,   # 목적지별 값은 evaluate 에서
                                           "fuel_pct": u.get("fuel_pct"), "current_node": u.get("current_node"),
                                           "current_task_id": u.get("current_task_id"), "driver": u.get("driver"),
                                           "mission_state": u.get("mission_state")},
                               comm_status="OK") for u in units]

    def evaluate(self, task_id, decision_id, assignment) -> LocalResponse:
        rid = assignment.resource_id
        target = {"lat": assignment.target_lat, "lon": assignment.target_lon}
        r = self.http.post(f"{self.url}/ugv/{rid}/evaluate", timeout=10,
                           json={"task_id": task_id, "decision_id": decision_id or task_id, "target": target})
        r.raise_for_status()
        ev = r.json()
        if ev["verdict"] == "ACCEPT":
            self.accepted[(task_id, rid)] = {"target": target, "decision_id": decision_id or task_id,
                                             "eta_sec": ev.get("eta_sec"), "target_node": ev.get("target_node")}
        else:
            self.accepted.pop((task_id, rid), None)
        return LocalResponse(resource_id=rid, response=ev["verdict"], reason=ev.get("reason"),
                             evidence={"eta_sec": ev.get("eta_sec"), "target_node": ev.get("target_node"),
                                       "path_len": len(ev["path"]) if ev.get("path") else None,
                                       "detail": ev.get("detail"), "approach": ev.get("approach")})

    def observe(self, rid, sim_time_s, task_id, decision_id, poll_s: float = 1.0, timeout_s: float = None) -> Observation:
        job = self.accepted.pop((task_id, rid), None) if task_id else None
        if job is None:                                  # 실행할 목표가 없다 → 지금 위치의 도로 상태만
            d = self._get(f"/ugv/{rid}/observation")
            return Observation(resource_id=rid, location_lat=d["location_lat"], location_lon=d["location_lon"],
                               simulation_time_s=sim_time_s, observation_type=d["observation_type"], value=d["value"],
                               task_id=task_id, decision_id=decision_id)
        dec = decision_id or job["decision_id"]
        key = exec_key(task_id, dec)
        # 평가 때 고른 도로 지점으로 실행한다 (서버가 같은 지점으로 다시 계획)
        node = (job.get("target_node") or {}).get("node_id")
        r = self.http.post(f"{self.url}/ugv/{rid}/execute", timeout=10,
                           json={"task_id": key, "decision_id": dec, "target": job["target"], "target_node": node})
        if r.status_code == 409:
            raise RuntimeError(f"UGV 실행 거부: {r.json().get('detail')}")
        r.raise_for_status()
        until = time.monotonic() + (timeout_s or getattr(config, "UGV_TASK_TIMEOUT_S", 120.0))
        while time.monotonic() < until:
            t = self._get(f"/ugv/{rid}/task/{key}")
            if t["status"] == "COMPLETED":
                o = t.get("observation") or {}
                pos = o.get("position") or {}
                # 지상자원 도착 기록은 도로 상태 관측이다 — 화재 관측으로 쓰지 않는다
                return Observation(resource_id=rid, location_lat=pos.get("lat"), location_lon=pos.get("lon"),
                                   simulation_time_s=sim_time_s, observation_type=o.get("observation_type", "ROAD_STATUS"),
                                   value={"arrived_node": o.get("arrived_node"), "fuel_pct": o.get("fuel_pct"),
                                          "eta_sec_planned": job.get("eta_sec")},
                                   task_id=task_id, decision_id=decision_id)
            if t["status"] in ("FAILED", "CANCELLED"):
                raise RuntimeError(f"UGV 임무 {t['status']}: {t.get('error')}")
            time.sleep(poll_s)
        raise TimeoutError(f"UGV {rid} 도착 대기 시간 초과 (task_id={task_id})")


_real_conn = None


def _real() -> _RealUgv:
    global _real_conn
    url = getattr(config, "UGV_SERVER_URL", "") or ""
    if not url or url == "TBD":
        raise RuntimeError("config.UGV_SERVER_URL 이 설정되지 않았습니다 (예: http://localhost:8100)")
    if _real_conn is None or _real_conn.url != url.rstrip("/"):
        _real_conn = _RealUgv(url)
    return _real_conn
