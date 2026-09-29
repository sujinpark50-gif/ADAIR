# -*- coding: utf-8 -*-
"""
orchestrator/models.py
======================
총괄 내부 데이터 모델.

공통 TaskState(interfaces/schema.py)는 재정의하지 않는다 (요청서 §6).
총괄은 Task "목적 상태(purpose_status)"와 실행 시도 "substatus"를 따로 두고
공통 상태로 매핑만 한다.
"""

from dataclasses import asdict, dataclass, field
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Task 목적 상태 → 공통 TaskState 매핑 (구현준비 문서 §3)
# ---------------------------------------------------------------------------
PURPOSE_TO_COMMON = {
    "PENDING": "READY",
    "HOLD": "READY",
    "EVALUATING": "READY",
    "APPROVED": "ASSIGNED",
    "IN_EXECUTION": "RUNNING",
    "COMPLETED": "COMPLETED",
    "FAILED": "FAILED",
    "CANCELLED": "CANCELLED",
}

# 실행 시도 substatus. 도착·관측·환경반영·완료는 별도 사건이다 (요청서 §6).
ATTEMPT_SUBSTATUSES = (
    "PREPARED",    # Safety ALLOW + 예약 + 선저장, 아직 미전송
    "REQUESTED",   # 송신함, 응답 대기
    "STARTED",     # 실행 파트가 접수 확인
    "ARRIVED",     # 물리 도착 확인
    "OBSERVED",    # 유효 관측 확인
    "APPLIED",     # 환경 반영 ACK
    "RETURNING",   # 목적 종료, 기체 복귀 중 (점유 유지)
    "RELEASED",    # 기체 READY/반납 확인 → 예약 해제
    "UNKNOWN",     # 응답 유실·조회 불능. 점유 유지, 재전송 금지
    "FAULTED",     # 실패 확인 + 기체 정지·사용불가 (예: UGV 주행 이상). 운영자 해제로 READY 될 때까지 점유
    "FAILED",      # 실패가 확인된 경우만
    "NOT_SENT",    # 송신 전 중단이 확인됨 (예약 해제 가능)
    "CANCELLED",
)


def common_state(purpose_status: str) -> str:
    return PURPOSE_TO_COMMON[purpose_status]


# ---------------------------------------------------------------------------
# Task
# ---------------------------------------------------------------------------

@dataclass
class Target:
    lat: float
    lon: float
    # 목표 지점 지면 해발고도(m, AMSL). 비행고도가 아니다. 없으면 None (0 으로 대체 금지)
    ground_amsl_m: Optional[float] = None
    cell_id: Optional[str] = None


@dataclass
class Requirements:
    resource_types: Tuple[str, ...] = ("UAV",)
    sensor: Optional[str] = "THERMAL"          # 필수 관측 능력. None 이면 관측 요구 없음
    target_agl_m: Optional[float] = None       # 임무 AGL. None 이면 UAV 기본값(80) 명시 전송
    # COUNTER 로 target_agl_m 을 바꾸자고 할 때 임무 목적이 유지되는 범위.
    # None 이면 AGL 변경 COUNTER 는 목적 충족을 판단할 수 없어 수용하지 않는다.
    agl_range_m: Optional[Tuple[float, float]] = None
    needs_env_ack: bool = True                 # 완료 조건에 환경 반영 ACK 필요 여부


@dataclass
class Task:
    task_id: str
    incident_id: str
    kind: str                                  # RECON / MONITOR / RECHECK / GROUND_SUPPORT ...
    target: Target
    requirements: Requirements = field(default_factory=Requirements)
    # 임무가 다루는 환경 셀들 (위험 셀 여러 칸을 한 임무로 묶을 때). 비면 target.cell_id 만 본다
    area_cell_ids: List[str] = field(default_factory=list)
    purpose_status: str = "PENDING"
    hold_reason: Optional[str] = None
    resume_condition: Optional[str] = None
    request_key: Optional[str] = None
    created_wall: float = 0.0
    created_sim_s: Optional[float] = None

    @property
    def common_state(self) -> str:
        return common_state(self.purpose_status)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["common_state"] = self.common_state
        return d

    @staticmethod
    def from_dict(d: dict) -> "Task":
        req = dict(d.get("requirements") or {})
        if "resource_types" in req:
            req["resource_types"] = tuple(req["resource_types"])
        if req.get("agl_range_m") is not None:
            req["agl_range_m"] = tuple(req["agl_range_m"])
        return Task(
            task_id=d["task_id"],
            incident_id=d["incident_id"],
            kind=d["kind"],
            target=Target(**d["target"]),
            requirements=Requirements(**req),
            area_cell_ids=list(d.get("area_cell_ids") or []),
            purpose_status=d.get("purpose_status", "PENDING"),
            hold_reason=d.get("hold_reason"),
            resume_condition=d.get("resume_condition"),
            request_key=d.get("request_key"),
            created_wall=d.get("created_wall", 0.0),
            created_sim_s=d.get("created_sim_s"),
        )


# ---------------------------------------------------------------------------
# 환경 snapshot (요청서 §3: run_id·state_version·simulation_time·map_version 연결)
# ---------------------------------------------------------------------------

@dataclass
class Snapshot:
    run_id: Optional[str]
    state_version: Optional[int]
    simulation_time_s: Optional[float]
    map_version: Optional[str]
    wind_ms: Optional[float]
    wind_valid_sim_s: Optional[float] = None
    crs: Optional[str] = None
    vertical_datum: Optional[str] = None
    fire_cells: List[dict] = field(default_factory=list)       # BURNING 셀
    risk_cells: List[dict] = field(default_factory=list)       # 화선 주변 UNBURNED 위험 셀
    protected_sites: List[dict] = field(default_factory=list)  # 보호대상 registry
    wind_dir_deg: Optional[float] = None   # 바람이 불어오는 방향 (기상 관례, degree)
    weather: dict = field(default_factory=dict)            # 지도 전체 기상값 (환경이 준 항목만)
    spread_forecast: List[dict] = field(default_factory=list)  # 환경 모델의 확산 예상 셀
    forecast_ref: Optional[dict] = None    # {model_version, issued_sim_s, horizon_s}
    source: str = "UNKNOWN"          # FIXTURE / IN_PROCESS / TEAM_API
    contract_complete: bool = False  # 요청서 §3 READ 계약을 만족하는 출처인지

    def ref(self) -> dict:
        """결정 근거에 남길 snapshot 식별 정보"""
        return {
            "run_id": self.run_id,
            "state_version": self.state_version,
            "simulation_time_s": self.simulation_time_s,
            "map_version": self.map_version,
            "source": self.source,
            "contract_complete": self.contract_complete,
        }


# ---------------------------------------------------------------------------
# 자원 상태 (총괄이 조회한 시점의 관찰값. 원문은 raw 로 보존)
# ---------------------------------------------------------------------------

@dataclass
class ResourceView:
    resource_id: str
    resource_type: str                  # UAV / UGV / FIRE_ENGINE
    base: Optional[str]
    lat: Optional[float]
    lon: Optional[float]
    alt_amsl_m: Optional[float] = None
    ready: bool = False                 # 실행 파트가 새 임무를 받을 수 있다고 보고했는지
    current_task_id: Optional[str] = None
    battery_pct: Optional[float] = None
    sensors: Tuple[str, ...] = ()
    comm: Optional[str] = None          # OK / WEAK / LOST ...
    failsafe: Optional[bool] = None
    phase: Optional[str] = None
    fetched_wall: float = 0.0           # 총괄이 조회에 성공한 실제 시각
    source_timestamp: Optional[str] = None
    raw: dict = field(default_factory=dict)
