# -*- coding: utf-8 -*-
"""
connectors/uav_connector.py
==============================
UAV Connector. Mock 모드에 더해, 김동현님이 올리신 UAV Local Agent
(px4/uav-agent, FastAPI 서버)와 실제로 통신하는 real 모드를 구현했다.

실제 서버 엔드포인트 (px4/uav-agent/README.md 기준):
  GET  /uav/{uav_id}/state              — 상태 조회
  POST /uav/{uav_id}/evaluate           — 수행가능성 판단 (ACCEPT/REJECT/COUNTER)
  POST /uav/{uav_id}/execute            — 실행 시작 (Safety ALLOW 이후에만 호출)
  GET  /uav/{uav_id}/task/{task_id}     — 실행 진행/결과 조회

주의: UAV는 "1대 = 프로세스 1개 = 포트 1개" 구조라, config.UAV_ENDPOINTS에서
resource_id별로 다른 주소를 찾아 호출한다 (단일 UAV_SERVER_URL 아님).
"""

import random
import time

import config
from interfaces.schema import ResourceStatus, LocalResponse, Observation

try:
    import requests
except ImportError:
    requests = None  # real 모드를 쓰기 전까지는 없어도 됨


# ---------------------------------------------------------------------------
# 공개 함수 (mock/real 스위치)
# ---------------------------------------------------------------------------

def get_uav_status(base: str) -> list:
    if config.UAV_CONNECTION_MODE == "mock":
        return _get_uav_status_mock(base)
    return _get_uav_status_real(base)


def send_uav_command(
    task_id: str,
    decision_id: str,
    assignment,
    wind_ms: float = None,
    force_response: str = None,
    force_reason: str = None,
) -> LocalResponse:
    """
    decision_id, wind_ms 인자가 추가됨 (real 모드에서 UAV 서버 API가 요구함).
    - decision_id: /evaluate 요청 바디에 포함해야 함
    - wind_ms: PX4/Gazebo가 풍속을 안 주므로, 환경모델 값을 여기로 주입해야 함
      (config.WIND_SOURCE 참고, 없으면 UAV가 WIND_UNKNOWN으로 REJECT함)
    """
    if config.UAV_CONNECTION_MODE == "mock":
        return _send_uav_command_mock(assignment, force_response, force_reason)
    return _send_uav_command_real(task_id, decision_id, assignment, wind_ms)


def get_uav_observation(resource_id: str, timestamp: float, task_id: str = None,
                         decision_id: str = None, assignment=None) -> Observation:
    """
    real 모드에서는 이 호출 시점(Safety ALLOW 이후)에 /execute를 트리거하고
    /task를 폴링해서 완료를 기다린 뒤 관측 결과를 반환한다.
    assignment: 실행 목표(target_lat/lon/alt_m)가 필요해서 추가된 인자
    """
    if config.UAV_CONNECTION_MODE == "mock":
        # 드론은 배정된 화재 타깃 위치에서 관측한다 → 그 좌표를 담아 환경 되먹임이 격자 안에 들어가게 함
        obs_lat = getattr(assignment, "target_lat", config.BASE_A_LAT) if assignment is not None else config.BASE_A_LAT
        obs_lon = getattr(assignment, "target_lon", config.BASE_A_LON) if assignment is not None else config.BASE_A_LON
        return Observation(
            resource_id=resource_id,
            location_lat=obs_lat,
            location_lon=obs_lon,
            simulation_time_s=timestamp,
            observation_type="FIRE_BOUNDARY",
            value={"fire_state": "BURNING", "note": "mock observation @ target"},
            task_id=task_id,
            decision_id=decision_id,
        )
    return _get_uav_observation_real(resource_id, timestamp, task_id, decision_id, assignment)


# ---------------------------------------------------------------------------
# Mock 구현 (기존 그대로)
# ---------------------------------------------------------------------------

def _get_uav_status_mock(base: str) -> list:
    base_lat = config.BASE_A_LAT if base == "A" else config.BASE_B_LAT
    base_lon = config.BASE_A_LON if base == "A" else config.BASE_B_LON

    statuses = []
    for i in range(1, 3):
        statuses.append(
            ResourceStatus(
                resource_id=f"{base}-uav{i}",
                resource_type="UAV",
                base=base,
                location_lat=base_lat + random.uniform(-0.01, 0.01),
                location_lon=base_lon + random.uniform(-0.01, 0.01),
                state="READY",
                capability={
                    "battery_pct": round(random.uniform(30, 100), 1),
                    "eta_sec": round(random.uniform(60, 600), 1),
                    "px4_failsafe": False,
                },
                comm_status=random.choice(["OK", "OK", "OK", "WEAK"]),
            )
        )
    return statuses


def _send_uav_command_mock(assignment, force_response, force_reason) -> LocalResponse:
    if force_response is not None:
        return LocalResponse(resource_id=assignment.resource_id, response=force_response, reason=force_reason)

    response = random.choices(["ACCEPT", "REJECT"], weights=[0.8, 0.2])[0]
    reason = None
    if response == "REJECT":
        reason = random.choice(["LOW_BATTERY", "HIGH_WIND", "SENSOR_FAILURE"])
    return LocalResponse(resource_id=assignment.resource_id, response=response, reason=reason)


# ---------------------------------------------------------------------------
# Real 구현 — px4/uav-agent REST API 연동
# ---------------------------------------------------------------------------

def _endpoint_for(resource_id: str) -> str:
    url = config.UAV_ENDPOINTS.get(resource_id)
    if not url or url == "TBD":
        raise RuntimeError(
            f"{resource_id}의 UAV 서버 주소가 아직 config.UAV_ENDPOINTS에 설정되지 않았습니다."
        )
    return url


def _get_uav_status_real(base: str) -> list:
    """base에 속한 UAV들(resource_id가 f'{base}-uav'로 시작)의 상태를 실제 서버에서 조회"""
    statuses = []
    for resource_id, url in config.UAV_ENDPOINTS.items():
        if not resource_id.startswith(f"{base}-uav"):
            continue
        if not url or url == "TBD":
            # 아직 배포 안 된 UAV는 자원 목록에서 제외 (OFFLINE 취급)
            continue

        resp = requests.get(f"{url}/uav/{resource_id}/state", timeout=5)
        resp.raise_for_status()
        data = resp.json()

        statuses.append(
            ResourceStatus(
                resource_id=resource_id,
                resource_type="UAV",
                base=base,
                location_lat=data["position"]["lat"],
                location_lon=data["position"]["lon"],
                state="OFFLINE" if data["health"]["failsafe"] else "READY",
                capability={
                    "battery_pct": data["battery"]["percent"],
                    "px4_failsafe": data["health"]["failsafe"],
                    "alt_m_amsl": data["position"]["alt_m_amsl"],
                },
                comm_status="OK" if data["link_quality"] == "OK" else data["link_quality"],
            )
        )
    return statuses


def _send_uav_command_real(task_id: str, decision_id: str, assignment, wind_ms: float) -> LocalResponse:
    """POST /uav/{id}/evaluate — 수행가능성만 판단, 실행하지 않음"""
    url = _endpoint_for(assignment.resource_id)
    body = {
        "task_id": task_id,
        "decision_id": decision_id,
        "target": {
            "lat": assignment.target_lat,
            "lon": assignment.target_lon,
            "alt_m_amsl": assignment.target_alt_m,
        },
        "observation_type": "THERMAL",
        "wind_ms": wind_ms,  # None이면 UAV 서버가 WIND_UNKNOWN으로 REJECT함
    }
    resp = requests.post(f"{url}/uav/{assignment.resource_id}/evaluate", json=body, timeout=10)
    resp.raise_for_status()
    data = resp.json()

    return LocalResponse(
        resource_id=assignment.resource_id,
        response=data["verdict"],
        reason=data.get("reason"),
        counter_alternative=data.get("counter_offer"),
        counter_constraint=data.get("constraints"),
    )


def _get_uav_observation_real(resource_id, timestamp, task_id, decision_id, assignment,
                                poll_interval_sec=1.0, timeout_sec=30.0) -> Observation:
    """POST /execute 로 실행 시작 → GET /task 폴링 → 완료 시 관측 결과 반환"""
    url = _endpoint_for(resource_id)

    exec_body = {
        "task_id": task_id,
        "decision_id": decision_id,
        "target": {
            "lat": assignment.target_lat,
            "lon": assignment.target_lon,
            "alt_m_amsl": assignment.target_alt_m,
        },
        "observation_type": "THERMAL",
    }
    resp = requests.post(f"{url}/uav/{resource_id}/execute", json=exec_body, timeout=10)
    resp.raise_for_status()

    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        task_resp = requests.get(f"{url}/uav/{resource_id}/task/{task_id}", timeout=5)
        task_resp.raise_for_status()
        task_data = task_resp.json()

        if task_data["status"] == "COMPLETED":
            obs = task_data.get("observation") or {}
            pos = obs.get("position") or {}
            # 관측 위치가 없으면 목표 좌표로 채우지 않는다(없는 관측을 만들지 않음).
            # 위치가 None 인 관측은 fire_connector.update_environment 가 반영하지 않는다.
            return Observation(
                resource_id=resource_id,
                location_lat=pos.get("lat"),
                location_lon=pos.get("lon"),
                simulation_time_s=timestamp,
                observation_type=obs.get("sensor_type", "THERMAL"),
                value=obs.get("values", {}),
                task_id=task_id,
                decision_id=decision_id,
            )
        if task_data["status"] == "FAILED":
            raise RuntimeError(f"UAV 실행 실패: {task_data.get('error')}")

        time.sleep(poll_interval_sec)

    raise TimeoutError(f"UAV {resource_id} 실행 결과 대기 시간 초과 (task_id={task_id})")