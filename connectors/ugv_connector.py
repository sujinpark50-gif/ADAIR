# -*- coding: utf-8 -*-
"""
connectors/ugv_connector.py
==============================
Ground Resource Connector (Contract 7번): UGV·소방차에 공통 사용.
단, resource_type / capability / state 의미는 구분한다.
"""

import random
import config
from interfaces.schema import ResourceStatus, LocalResponse, Observation


def get_ugv_status(base: str) -> list:
    if config.UGV_CONNECTION_MODE == "integrated":
        from integration import bridge_ugv
        return bridge_ugv.get_ugv_status(base)
    if config.UGV_CONNECTION_MODE == "mock":
        return _get_ugv_status_mock(base)
    raise NotImplementedError("UGV 실제 연동이 아직 구현되지 않았습니다.")


def send_ugv_command(task_id: str, assignment, force_response: str = None, force_reason: str = None) -> LocalResponse:
    if config.UGV_CONNECTION_MODE == "integrated":
        from integration import bridge_ugv
        return bridge_ugv.send_ugv_command(task_id, assignment, force_response, force_reason)
    if config.UGV_CONNECTION_MODE == "mock":
        return _send_ugv_command_mock(assignment, force_response, force_reason)
    raise NotImplementedError("UGV 실제 명령 전송이 아직 구현되지 않았습니다.")


def get_ugv_observation(resource_id: str, timestamp: float, task_id: str = None, decision_id: str = None) -> Observation:
    if config.UGV_CONNECTION_MODE == "integrated":
        from integration import bridge_ugv
        return bridge_ugv.get_ugv_observation(resource_id, timestamp, task_id, decision_id)
    if config.UGV_CONNECTION_MODE == "mock":
        return Observation(
            resource_id=resource_id,
            location_lat=config.BASE_A_LAT,
            location_lon=config.BASE_A_LON,
            simulation_time_s=timestamp,
            observation_type="ROAD_STATUS",
            value={"note": "mock observation"},
            task_id=task_id,
            decision_id=decision_id,
        )
    raise NotImplementedError("UGV 실제 관측 결과 반환이 아직 구현되지 않았습니다.")


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