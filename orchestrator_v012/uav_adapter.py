"""UAV 원본 HTTP 입력을 보존하는 총괄 전용 Adapter.

기존 Connector 반환값에는 일부 정보가 이미 없으므로 원본 API를 조회한다.
상태 조회와 Local evaluate는 실행을 일으키지 않는다.
execute_and_wait는 총괄이 기존 Safety ALLOW를 확인한 뒤 별도로 호출한다.
HTTP 연결 여부와 UAV 서버 내부의 Mock/PX4 모드는 별개다.
"""

from copy import deepcopy
import asyncio
import time

import config
from interfaces.schema import Assignment
from .models import UavEvaluation, UavStatus


def adapt_uav_state(data: dict, base: str) -> UavStatus:
    """원값과 단위를 보존한다. 파생 state는 Local verdict를 대체하지 않는다."""
    return UavStatus(
        resource_id=data["uav_id"],
        resource_type="UAV",
        base=base,
        location_lat=data["position"]["lat"],
        location_lon=data["position"]["lon"],
        state="OFFLINE" if data["health"]["failsafe"] else "READY",
        state_source="uav_adapter:health.failsafe",
        source_timestamp=data["timestamp"],
        current_task_id=data.get("current_task_id"),
        capability={
            "battery_pct": data["battery"]["percent"],
            "px4_failsafe": data["health"]["failsafe"],
            "alt_m_amsl": data["position"]["alt_m_amsl"],
            "gps_ok": data["health"]["gps_ok"],
            "sensors_ok": data["health"]["sensors_ok"],
            "velocity_ms": data["velocity_ms"],
            "flight_mode": data["flight_mode"],
        },
        comm_status=data["link_quality"],
    )


def adapt_uav_evaluation(data: dict) -> UavEvaluation:
    """요청 ID로 응답 ID를 덮어쓰거나 COUNTER를 ACCEPT로 바꾸지 않는다."""
    return UavEvaluation(
        resource_id=data["uav_id"],
        response=data["verdict"],
        reason=data.get("reason"),
        counter_alternative=deepcopy(data.get("counter_offer")),
        counter_constraint=deepcopy(data.get("constraints")),
        task_id=data["task_id"],
        decision_id=data["decision_id"],
        eta_sec=data.get("eta_sec"),
        detail=data.get("detail"),
    )


class UavHttpAdapter:
    """기존 UAV API와 설정을 사용하는 입력 경계. http 주입으로 독립 검증 가능."""

    def __init__(self, endpoints=None, http=None):
        self.endpoints = dict(config.UAV_ENDPOINTS if endpoints is None else endpoints)
        if http is None:
            import requests
            http = requests
        self.http = http

    def _endpoint_for(self, resource_id: str) -> str:
        url = self.endpoints.get(resource_id)
        if not url or url == "TBD":
            raise RuntimeError(f"{resource_id}의 UAV 서버 주소가 설정되지 않았습니다.")
        return url.rstrip("/")

    def get_status(self, base: str) -> list[UavStatus]:
        statuses = []
        for resource_id, url in self.endpoints.items():
            # 기존 Connector와 같이 다른 거점 및 주소 미설정 자원은 조회하지 않는다.
            if not resource_id.startswith(f"{base}-uav") or not url or url == "TBD":
                continue
            response = self.http.get(
                f"{self._endpoint_for(resource_id)}/uav/{resource_id}/state", timeout=5,
            )
            response.raise_for_status()
            statuses.append(adapt_uav_state(response.json(), base))
        return statuses

    def evaluate(
        self, task_id: str, decision_id: str, assignment: Assignment,
        wind_ms: float | None = None,
    ) -> UavEvaluation:
        resource_id = assignment.resource_id
        body = {
            "task_id": task_id,
            "decision_id": decision_id,
            "target": {
                "lat": assignment.target_lat,
                "lon": assignment.target_lon,
                "alt_m_amsl": assignment.target_alt_m,
            },
            "observation_type": "THERMAL",
            "wind_ms": wind_ms,
        }
        response = self.http.post(
            f"{self._endpoint_for(resource_id)}/uav/{resource_id}/evaluate",
            json=body, timeout=10,
        )
        response.raise_for_status()
        return adapt_uav_evaluation(response.json())

    async def execute_and_wait(self, task_id, decision_id, assignment, timeout_s=30):
        """호출자는 Safety ALLOW 후에만 진입한다. 완료 상태를 실제 API로 조회한다."""
        rid = assignment.resource_id
        url = self._endpoint_for(rid)
        response = self.http.post(f'{url}/uav/{rid}/execute', json={
            'task_id': task_id, 'decision_id': decision_id,
            'target': {'lat': assignment.target_lat, 'lon': assignment.target_lon,
                       'alt_m_amsl': assignment.target_alt_m},
            'observation_type': 'THERMAL',
        }, timeout=10)
        response.raise_for_status()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            response = self.http.get(f'{url}/uav/{rid}/task/{task_id}', timeout=5)
            response.raise_for_status()
            result = response.json()
            if result.get('task_id') != task_id:
                raise ValueError('실행 응답 task_id 불일치')
            if result['status'] in ('COMPLETED', 'FAILED'):
                return result
            await asyncio.sleep(.1)
        return {'status': 'UNKNOWN', 'task_id': task_id}
