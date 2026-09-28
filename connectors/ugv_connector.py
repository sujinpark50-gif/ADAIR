# -*- coding: utf-8 -*-
"""
connectors/ugv_connector.py
==============================
Ground Resource Connector (Contract 7번): UGV·소방차에 공통 사용.
단, resource_type / capability / state 의미는 구분한다.

모드 (config.UGV_CONNECTION_MODE)
  mock       : 랜덤 응답
  integrated : integration/bridge_ugv.py — 같은 프로세스에서 도로 그래프만으로 판단 (주행 없음)
  real       : UGV Local Agent 서버(ugv/server.py) 에 HTTP 요청. config.UGV_SERVER_URL
               서버 실행: UGV_DRIVER=sim uvicorn ugv.server:app --port 8100

real 모드 흐름은 UAV 커넥터와 같다.
  send_ugv_command    → POST /ugv/{id}/evaluate  (판단만, 움직이지 않음)
  get_ugv_observation → POST /ugv/{id}/execute → GET /task 폴링 → 도착 관측 반환
                        (main.py 는 Safety ALLOW 이후에만 이 함수를 부른다)
"""

import random
import time

import config
from interfaces.schema import ResourceStatus, LocalResponse, Observation

try:
    import requests
except ImportError:
    requests = None  # real 모드를 쓰기 전까지는 없어도 됨


def get_ugv_status(base: str) -> list:
    if config.UGV_CONNECTION_MODE == "integrated":
        from integration import bridge_ugv
        return bridge_ugv.get_ugv_status(base)
    if config.UGV_CONNECTION_MODE == "mock":
        return _get_ugv_status_mock(base)
    return _get_ugv_status_real(base)


def send_ugv_command(task_id: str, assignment, force_response: str = None, force_reason: str = None,
                     decision_id: str = None) -> LocalResponse:
    if config.UGV_CONNECTION_MODE == "integrated":
        from integration import bridge_ugv
        return bridge_ugv.send_ugv_command(task_id, assignment, force_response, force_reason)
    if config.UGV_CONNECTION_MODE == "mock" or force_response is not None:
        return _send_ugv_command_mock(assignment, force_response, force_reason)
    return _send_ugv_command_real(task_id, decision_id, assignment)


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
    return _get_ugv_observation_real(resource_id, sim_time_s, task_id, decision_id)


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

# ---------------------------------------------------------------------------
# Real 구현 — ugv/server.py REST API 연동
# ---------------------------------------------------------------------------

# ACCEPT 된 task 의 목표 좌표를 execute 때 쓰기 위해 기억한다.
# main.py 는 UGV 관측 호출에 목표(assignment)를 넘기지 않으므로 (task_id, resource_id) 로 찾는다.
_evaluated: dict[tuple[str, str], dict] = {}


def _server() -> str:
    url = getattr(config, "UGV_SERVER_URL", "TBD")
    if not url or url == "TBD":
        raise RuntimeError("config.UGV_SERVER_URL 이 설정되지 않았습니다 (예: http://localhost:8100)")
    if requests is None:
        raise RuntimeError("real 모드에는 requests 패키지가 필요합니다")
    return url.rstrip("/")


def _get_ugv_status_real(base: str) -> list:
    """GET /ugv?base= → ResourceStatus 목록. 서버에 연결되지 않으면 빈 목록 (자원 없음 취급)."""
    try:
        resp = requests.get(f"{_server()}/ugv", params={"base": base}, timeout=5)
        resp.raise_for_status()
    except requests.RequestException:
        return []
    return [
        ResourceStatus(
            resource_id=d["resource_id"],
            resource_type=d["resource_type"],
            base=d["base"],
            location_lat=d["position"]["lat"],
            location_lon=d["position"]["lon"],
            state=d["state"],
            capability={
                "fuel_pct": d["fuel_pct"],
                "current_node": d["current_node"],
                "current_task_id": d["current_task_id"],
                "driver": d["driver"],
                # 목적지별 값은 evaluate 에서만 나온다
                "eta_sec": None, "reachable": None, "road_blocked": None,
            },
            comm_status="OK",
        )
        for d in resp.json()
    ]


def _send_ugv_command_real(task_id: str, decision_id: str, assignment) -> LocalResponse:
    """POST /ugv/{id}/evaluate — 수행가능성만 판단, 실행하지 않음"""
    rid = assignment.resource_id
    body = {
        "task_id": task_id,
        "decision_id": decision_id or task_id,   # main.py 는 UGV 에 decision_id 를 넘기지 않는다
        "target": {"lat": assignment.target_lat, "lon": assignment.target_lon},
    }
    resp = requests.post(f"{_server()}/ugv/{rid}/evaluate", json=body, timeout=10)
    resp.raise_for_status()
    data = resp.json()

    if data["verdict"] == "ACCEPT":
        _evaluated[(task_id, rid)] = {**data, "target": body["target"]}
    else:
        _evaluated.pop((task_id, rid), None)

    # LocalResponse 에는 eta·path 칸이 없어 counter_constraint 에 판단 근거로 싣는다 (R01 확장 전 임시)
    return LocalResponse(
        resource_id=rid,
        response=data["verdict"],
        reason=data.get("reason"),
        counter_constraint={"eta_sec": data.get("eta_sec"), "target_node": data.get("target_node"),
                            "path_len": len(data["path"]) if data.get("path") else None,
                            "detail": data.get("detail")},
    )


def _get_ugv_observation_real(resource_id, sim_time_s, task_id, decision_id,
                              poll_interval_sec=1.0, timeout_sec=None) -> Observation:
    """ACCEPT 된 task 면 POST /execute → GET /task 폴링 → 도착 관측 반환.
    evaluate 기록이 없으면 현재 위치의 도로 상태만 조회한다 (주행하지 않음)."""
    url = _server()
    evaluated = _evaluated.pop((task_id, resource_id), None) if task_id else None

    if evaluated is None:
        resp = requests.get(f"{url}/ugv/{resource_id}/observation", timeout=5)
        resp.raise_for_status()
        d = resp.json()
        return Observation(resource_id=resource_id, location_lat=d["location_lat"],
                           location_lon=d["location_lon"], simulation_time_s=sim_time_s,
                           observation_type=d["observation_type"], value=d["value"],
                           task_id=task_id, decision_id=decision_id)

    # UAV 와 같이 화재 좌표로 실행을 요청한다. 목적지 노드는 서버가 evaluate 와 같은 규칙으로 다시 고른다
    exec_body = {"task_id": task_id, "decision_id": decision_id or evaluated["decision_id"],
                 "target": evaluated["target"]}
    resp = requests.post(f"{url}/ugv/{resource_id}/execute", json=exec_body, timeout=10)
    if resp.status_code == 409:   # evaluate 이후 도로가 막혔거나 다른 task 수행 중
        raise RuntimeError(f"UGV 실행 거부: {resp.json().get('detail')}")
    resp.raise_for_status()

    timeout_sec = timeout_sec or getattr(config, "UGV_TASK_TIMEOUT_S", 120.0)
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        t = requests.get(f"{url}/ugv/{resource_id}/task/{task_id}", timeout=5)
        t.raise_for_status()
        t = t.json()
        if t["status"] == "COMPLETED":
            obs = t.get("observation") or {}
            pos = obs.get("position") or {}
            # 지상자원 관측은 도로 상태다. 화재 관측으로 쓰지 않는다 (fire_connector 는 ROAD_STATUS 를 반영하지 않음)
            return Observation(resource_id=resource_id, location_lat=pos.get("lat"),
                               location_lon=pos.get("lon"), simulation_time_s=sim_time_s,
                               observation_type=obs.get("observation_type", "ROAD_STATUS"),
                               value={"arrived_node": obs.get("arrived_node"), "fuel_pct": obs.get("fuel_pct"),
                                      "eta_sec_planned": evaluated.get("eta_sec")},
                               task_id=task_id, decision_id=decision_id)
        if t["status"] == "FAILED":
            raise RuntimeError(f"UGV 주행 실패: {t.get('error')}")
        time.sleep(poll_interval_sec)

    raise TimeoutError(f"UGV {resource_id} 도착 대기 시간 초과 (task_id={task_id})")
