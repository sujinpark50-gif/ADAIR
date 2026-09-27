# ugv/api_models.py — UGV Local Agent REST API 의 요청·응답 형식
# UAV Agent(uav/uav-agent/models.py)와 필드 이름·의미를 맞춘다: verdict, eta_sec, reason, detail.
# 단위: 거리 m, 시간 s, 연료 %(0~100), 좌표 WGS84

from typing import Literal

from pydantic import BaseModel


class LatLon(BaseModel):
    lat: float
    lon: float


class UgvState(BaseModel):
    resource_id: str
    resource_type: str                 # "UGV" | "FIRE_ENGINE"
    base: str
    state: str                         # READY / RUNNING / UNAVAILABLE(주행 중 이상, /stop 으로 해제)
    position: LatLon
    fuel_pct: float
    current_node: str | None           # 노드에 정지 중일 때만 값이 있다 (주행 중 None)
    current_task_id: str | None
    driver: str                        # "sim" | "px4"
    driver_status: str                 # SimDriver: IDLE/MISSION/ARRIVED, PX4: flight mode
    updated_at: float                  # 마지막 텔레메트리 반영 시각 (epoch s, 0 이면 미수신)
    fault: str | None = None           # state=UNAVAILABLE 일 때 이상 내용 'CODE: 설명'. /stop 으로 해제


class EvaluateRequest(BaseModel):
    task_id: str
    decision_id: str
    target: LatLon | None = None       # 화재 좌표 (권장). 반경 안 도로 노드 중 도달 가능한 가장 가까운 곳으로 판단
    target_node: str | None = None     # 도로 노드 id 를 이미 알 때 (target 대신)


class TargetNode(BaseModel):
    node_id: str
    lat: float
    lon: float
    snap_m: float                      # 요청 좌표 ~ 선택된 도로 노드 거리


class EvaluateResponse(BaseModel):
    task_id: str
    decision_id: str
    resource_id: str
    verdict: Literal["ACCEPT", "REJECT"]
    eta_sec: int | None                # ACCEPT 일 때만 값이 있다
    reason: str | None                 # REJECT: BUSY / ROAD_BLOCKED / TARGET_UNREACHABLE
    detail: str | None
    target_node: TargetNode | None     # 실제로 향할 도로 노드 (스냅 실패 시 None)
    path: list[str] | None             # 노드 id 목록, 출발 노드 포함


class ExecuteRequest(BaseModel):
    task_id: str
    decision_id: str
    target: LatLon | None = None       # 화재 좌표 (권장, UAV 와 같은 방식). evaluate 와 같은 규칙으로 목적지를 고른다
    target_node: str | None = None     # 도로 노드 id 를 직접 지정할 때 (target 대신)


class ExecuteResponse(BaseModel):
    task_id: str
    resource_id: str
    status: Literal["STARTED"]
    tracking_url: str
    target_node: TargetNode            # 실제로 향하는 도로 노드
    eta_sec: int                       # 출발 시점 도로망 기준 ETA


class TaskStatus(BaseModel):
    task_id: str
    resource_id: str
    status: Literal["STARTED", "IN_PROGRESS", "COMPLETED", "FAILED", "CANCELLED"]
    target_node: str
    progress: dict | None              # {"phase": "ENROUTE", "waypoint": i, "total": n}
    observation: dict | None           # 도착 시 ROAD_STATUS 관측. 화재 관측이 아니다
    error: str | None = None


class FireCell(BaseModel):
    x: int                             # 환경 격자 열 (get_fire_locations 의 x)
    y: int                             # 환경 격자 행 (get_fire_locations 의 y)


class FireCellsRequest(BaseModel):
    cells: list[FireCell]              # 현재 BURNING 셀 전체 (스냅샷). get_fire_locations() 결과를 그대로 넣어도 된다


class CongestionRequest(BaseModel):
    preset: Literal["none", "random"] = "none"
    seed: int = 0
    ratio: float = 0.2                 # random: 전체 도로 중 혼잡 도로 비율
    min_factor: float = 1.5            # random: 통과시간 배율 범위
    max_factor: float = 3.0
    roads: dict[str, float] | None = None   # 특정 도로 배율 직접 지정 (preset 적용 후 덮어씀)
