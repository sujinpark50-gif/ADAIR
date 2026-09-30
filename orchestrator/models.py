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


# 실행시도의 "임무 부분"이 아직 열려 있는 상태. 이 상태의 시도가 있으면 같은 Task 에 새 시도를 만들지 않는다.
# OBSERVED/APPLIED/RETURNING/FAULTED 는 임무 부분이 끝났고 기체만 점유 중인 상태다 (인계 배정은 가능).
MISSION_OPEN_SUBSTATUSES = ("PREPARED", "REQUESTED", "STARTED", "ARRIVED", "UNKNOWN")

# ---------------------------------------------------------------------------
# Task 목적 상태 허용 전이 (2026-09-30 B01/B02). 장부(save_task)가 이 표로 검증한다.
# 종료 상태(COMPLETED/CANCELLED/FAILED)에서는 어떤 상태로도 나가지 않는다 — 늦게 온 관측·완료 응답,
# 수동 RETRY, idle 확인이 종료된 목적을 되살리지 못한다. 재관측은 새 Task(RECHECK)로 만든다.
# ---------------------------------------------------------------------------
TERMINAL_PURPOSES = ("COMPLETED", "CANCELLED", "FAILED")
PURPOSE_TRANSITIONS = {
    "PENDING": {"EVALUATING", "HOLD", "CANCELLED"},
    "HOLD": {"PENDING", "EVALUATING", "IN_EXECUTION", "CANCELLED"},      # IN_EXECUTION: 불명 시도가 다시 보고됨
    "EVALUATING": {"APPROVED", "HOLD", "PENDING", "CANCELLED"},
    "APPROVED": {"IN_EXECUTION", "EVALUATING", "HOLD", "CANCELLED"},     # EVALUATING: 실행 거절 확정 → 다음 후보
    "IN_EXECUTION": {"COMPLETED", "PENDING", "HOLD", "CANCELLED"},       # PENDING: 관측 미충족·인계
    "COMPLETED": set(),
    "CANCELLED": set(),
    "FAILED": set(),
}

# 완료 판정 규칙 (2026-09-30 D03)
COMPLETION_TARGET_POINT = "TARGET_POINT"                    # 목표 지점이 유효 관측 범위 안
COMPLETION_AREA_ALL = "AREA_ALL_REQUIRED_CELLS"             # 필수 셀 전체가 (누적) 완전 관측
AREA_KINDS = ("MONITOR",)


def can_transition(current: str, new: str) -> bool:
    """같은 상태 유지(사유 갱신)는 종료 상태가 아니어도·이어도 허용한다 (상태가 바뀌지 않음)."""
    return current == new or new in PURPOSE_TRANSITIONS.get(current, set())


def common_state(purpose_status: str) -> str:
    return PURPOSE_TO_COMMON[purpose_status]


# ---------------------------------------------------------------------------
# Task
# ---------------------------------------------------------------------------

@dataclass
class Target:
    # 위치를 모르면 None (신고 칸이 지도에 없을 때). 좌표를 지어내지 않고 배정을 보류한다
    lat: Optional[float]
    lon: Optional[float]
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
    # 같은 방문에서 함께 하는 측정 (예: 정찰하면서 기상도). 같은 칸에 두 번 보내지 않기 위함
    extra_sensors: Tuple[str, ...] = ()


@dataclass
class Task:
    task_id: str
    incident_id: str
    kind: str                                  # RECON / MONITOR / RECHECK / GROUND_SUPPORT ...
    target: Target
    requirements: Requirements = field(default_factory=Requirements)
    # 임무와 관련된 환경 셀들 (우선순위 근거용 메타데이터). 완료 요구 범위가 아니다 → required_cell_ids
    area_cell_ids: List[str] = field(default_factory=list)
    purpose_status: str = "PENDING"
    hold_reason: Optional[str] = None
    resume_condition: Optional[str] = None
    request_key: Optional[str] = None
    created_wall: float = 0.0
    created_sim_s: Optional[float] = None
    run_id: Optional[str] = None               # 이 Task 가 속한 실행(run). 다른 run 의 판단에 쓰지 않는다
    parent_task_id: Optional[str] = None       # 재관측(RECHECK)이 이어받은 이전 Task
    # 완료 판정 (D03). TARGET_POINT: 목표 지점 관측 / AREA_ALL_REQUIRED_CELLS: required_cell_ids 전체 완전 관측
    completion_rule: str = COMPLETION_TARGET_POINT
    required_cell_ids: List[str] = field(default_factory=list)
    # 구역 임무의 누적 관측 {cell_id: {"rects": [[x0,y0,x1,y1,observation_id]], "fraction", "last_observed_sim_s", "sources"}}
    coverage: dict = field(default_factory=dict)
    visited_cell_ids: List[str] = field(default_factory=list)   # 기체가 관측 프레임 중심을 둔 필수 셀
    visit_target: Optional[Target] = None      # 구역 임무의 다음 방문 지점 (없으면 target)

    @property
    def common_state(self) -> str:
        return common_state(self.purpose_status)

    @property
    def nav_target(self) -> Target:
        """이번 실행시도가 실제로 갈 지점"""
        return self.visit_target or self.target

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
        if "extra_sensors" in req:
            req["extra_sensors"] = tuple(req["extra_sensors"] or ())
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
            run_id=d.get("run_id"),
            parent_task_id=d.get("parent_task_id"),
            # 이전 장부의 MONITOR 는 규칙 칸이 없다 → 구역 규칙으로 읽는다 (한 점 관측으로 완료시키지 않음)
            completion_rule=d.get("completion_rule") or (
                COMPLETION_AREA_ALL if d["kind"] in AREA_KINDS else COMPLETION_TARGET_POINT),
            required_cell_ids=list(d.get("required_cell_ids") or (
                (d.get("area_cell_ids") or ([d["target"].get("cell_id")] if d["target"].get("cell_id") else []))
                if (not d.get("completion_rule") and d["kind"] in AREA_KINDS) else [])),
            coverage=dict(d.get("coverage") or {}),
            visited_cell_ids=list(d.get("visited_cell_ids") or []),
            visit_target=Target(**d["visit_target"]) if d.get("visit_target") else None,
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
    scenario_start_kst: Optional[str] = None  # 시뮬레이션 0초에 해당하는 실제 시각 (재현 사건 시계)
    wind_ref: Optional[dict] = None        # 판단에 쓴 풍속의 출처 (view_for)
    analysis_source: Optional[str] = None  # 위험 칸·확산 예측을 만든 곳
    source: str = "UNKNOWN"          # FIXTURE / IN_PROCESS / TEAM_API
    contract_complete: bool = False  # 요청서 §3 READ 계약을 만족하는 출처인지
    # 이 snapshot 을 판단 입력으로 쓸 수 없는 이유 (run 전환 차단·버전 역행 등). None 이면 사용 가능
    run_gate: Optional[str] = None

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
