# -*- coding: utf-8 -*-
"""
interfaces/schema.py
=====================
박수진님의 "01. Integration Contract" 문서를 기준으로 전면 재작성한
공통 데이터 형식입니다. 이 파일이 곧 Contract 2번 "통합 담당"의 산출물
(Schema/Enum, 공통 ID, 좌표·단위·시간 형식)에 해당합니다.

문서 대조:
- 4번(공통 메시지 식별 정보) → MessageHeader
- 5번(공통 데이터 의미) → EnvironmentState, ResourceStatus
- 6번(Task와 Resource 상태값) → TaskState, ResourceState
- Local Response / Safety Response / Observation → 아래 각 섹션
"""

from dataclasses import dataclass, field
from typing import List, Literal, Optional


# ---------------------------------------------------------------------------
# 4. 공통 메시지 식별 정보 (Contract 4번)
# ---------------------------------------------------------------------------

@dataclass
class MessageHeader:
    """모든 통합 메시지가 최소로 가져야 하는 공통 식별 정보"""
    message_id: str
    sender: str              # 어느 모듈이 보냈는지 (예: "fire_connector", "uav_connector")
    message_type: str        # 예: "ENVIRONMENT_STATE", "LOCAL_RESPONSE"
    schema_version: str
    timestamp: float
    run_id: str
    # 상황별 추가 (필요한 메시지에만)
    task_id: Optional[str] = None
    resource_id: Optional[str] = None
    decision_id: Optional[str] = None
    correlation_id: Optional[str] = None


# ---------------------------------------------------------------------------
# 6. Task 상태값 (Contract 6번)
# ---------------------------------------------------------------------------

TaskState = Literal["READY", "ASSIGNED", "RUNNING", "COMPLETED", "FAILED", "CANCELLED"]


@dataclass
class Task:
    """
    프로젝트 상위 업무 단위. 환경 상태/사건에서 생성되고,
    Orchestrator가 후보 자원을 평가해 할당한다.
    """
    task_id: str
    created_at: float
    state: TaskState = "READY"
    description: str = ""     # 예: "화재 셀(50,50) 관측"
    target_lat: float = 0.0
    target_lon: float = 0.0


# ---------------------------------------------------------------------------
# 6. Resource 상태값 (Contract 6번) + 5번(resource_type)
# ---------------------------------------------------------------------------

ResourceState = Literal[
    "READY", "ASSIGNED", "RUNNING", "RETURNING", "STALE", "OFFLINE", "UNAVAILABLE"
]
ResourceType = Literal["UAV", "UGV", "FIRE_ENGINE"]


# ---------------------------------------------------------------------------
# 5. 환경/산불 모델 상태 (Contract 5번 "Environment State")
# ---------------------------------------------------------------------------

@dataclass
class EnvironmentState:
    """환경모델이 매 스텝마다 제공하는 화재 상태 정보"""
    timestamp: float
    fire_cells: List[dict]
    risk_zone: List[dict]
    wind_speed: float          # m/s
    wind_direction: float      # degree
    spread_direction: str
    env_updated: bool = False


# ---------------------------------------------------------------------------
# 5. Resource 공통 상태 (Contract 5번 "Resource")
#    UAV/UGV/FIRE_ENGINE가 공통으로 갖는 필드 + 각자의 Capability 차이는
#    capability 딕셔너리로 구분한다 (Contract: "Capability·에너지/연료·센서 의미는 구분")
# ---------------------------------------------------------------------------

@dataclass
class ResourceStatus:
    """UAV·UGV·소방차 공통 Resource 표현"""
    resource_id: str                 # 예: "A-uav1", "A-ugv1", "A-truck1"
    resource_type: ResourceType
    base: str                        # 소속 거점 ("A" 또는 "B")
    location_lat: float
    location_lon: float
    state: ResourceState = "READY"   # Task/Resource 생명주기 상태
    task_state: Optional[TaskState] = None  # 현재 수행 중인 Task 상태 (있을 경우)
    capability: dict = field(default_factory=dict)
    # capability 예시:
    #   UAV: {"battery_pct": 80, "eta_sec": 120, "px4_failsafe": False}
    #   UGV: {"eta_sec": 400, "road_blocked": False, "reachable": True}
    #   FIRE_ENGINE: {"eta_sec": 900, "reachable": True}
    comm_status: str = "OK"          # "OK" / "WEAK" / "LOST"


# ---------------------------------------------------------------------------
# Local Response (Contract "Local Response")
# ---------------------------------------------------------------------------

LocalResponseType = Literal["ACCEPT", "REJECT", "COUNTER"]

RejectReason = Literal[
    "LOW_BATTERY",
    "RETURN_MARGIN_INSUFFICIENT",
    "HIGH_WIND",
    "ROAD_BLOCKED",
    "TARGET_UNREACHABLE",
    "SENSOR_FAILURE",
    "COMMUNICATION_FAILURE",
    "TIMEOUT",
    "REQUIRED_CAPABILITY_UNAVAILABLE",
    # UAV 브랜치(김동현) 연동 과정에서 추가 합의된 사유 (px4/uav-agent/README.md 참고)
    "FAILSAFE_ACTIVE",   # PX4 failsafe 작동 중
    "BUSY",              # 다른 Task 수행 중
    "WIND_UNKNOWN",       # 풍속 정보 없음 — 요청에 wind_ms 미포함
    "DEADLINE_TIGHT",     # (COUNTER) 기한 초과하나 수행 가능
    "MODERATE_WIND",      # (COUNTER) 풍속 주의
]


@dataclass
class LocalResponse:
    """Local Agent(UAV/UGV)의 수행가능성 응답"""
    resource_id: str
    response: LocalResponseType
    reason: Optional[RejectReason] = None      # REJECT일 때만
    counter_alternative: Optional[dict] = None  # COUNTER일 때 대안 (초기 범위 제외, 자리만 확보)
    counter_constraint: Optional[dict] = None   # COUNTER일 때 제한조건 (〃)


# ---------------------------------------------------------------------------
# Safety Response (Contract "Safety Response")
# ---------------------------------------------------------------------------

SafetyResponseType = Literal["ALLOW", "MODIFY", "REJECT"]


@dataclass
class SafetyVerdict:
    """Safety Layer의 판단 결과"""
    response: SafetyResponseType
    reason: Optional[str] = None
    modified_assignment: Optional[dict] = None  # MODIFY일 때 수정안


# ---------------------------------------------------------------------------
# Observation (Contract "Observation")
# ---------------------------------------------------------------------------

@dataclass
class Observation:
    """실행 이후 자원이 반환하는 관측 결과"""
    resource_id: str
    location_lat: float
    location_lon: float
    timestamp: float
    observation_type: str        # 관측 종류 (예: "FIRE_BOUNDARY", "WIND")
    value: dict                  # 관측 값
    task_id: Optional[str] = None
    decision_id: Optional[str] = None


# ---------------------------------------------------------------------------
# 자원 배정 / 총괄 판단 결과
# ---------------------------------------------------------------------------

@dataclass
class Assignment:
    """Task 하나에 대한 자원 배정 제안"""
    task_id: str
    resource_id: str
    target_lat: float
    target_lon: float
    task_type: str = "OBSERVE"
    # UAV는 goto(위도,경도,고도) 형태로 명령해야 해서 고도가 필수임 (px4/uav-agent 요청 반영)
    # TBD: 실제 지형 고도(DEM)는 김은주님 환경모델 완성 후 반영, 그 전까지는 0.0(해수면)
    target_alt_m: float = 0.0


@dataclass
class Decision:
    """
    오케스트레이터가 한 번의 판단 흐름에서 내리는 결과.
    decision_id는 "하나의 총괄 판단 흐름"을 추적하는 단위이며,
    REJECT 이후 재평가가 발생하면 새 decision_id를 부여한다 (Contract "Decision ID 원칙").
    """
    decision_id: str
    task_id: str
    timestamp: float
    assignments: List[Assignment]
    safety_verdict: Optional[SafetyVerdict] = None


# ---------------------------------------------------------------------------
# 거점 전체 자원 컨테이너 (기존 유지, resource_type 기반으로 조회 가능)
# ---------------------------------------------------------------------------

@dataclass
class ResourcePool:
    base_a: List[ResourceStatus] = field(default_factory=list)
    base_b: List[ResourceStatus] = field(default_factory=list)

    def all_resources(self) -> List[ResourceStatus]:
        return self.base_a + self.base_b

    def by_type(self, resource_type: ResourceType) -> List[ResourceStatus]:
        return [r for r in self.all_resources() if r.resource_type == resource_type]