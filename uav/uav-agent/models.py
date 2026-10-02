"""요청/응답 스키마.

대응 문서: docs/intergration/01. Integration Contract.md
대응 코드: interfaces/schema.py (ResourceStatus / Assignment / LocalResponse)

계약 문서는 명명·프로세스 규약이라 UAV 필드를 규정하지 않는다("세부 통신 구현은
여기서 확정하지 않는다", 계약 §1). 아래는 그 공백을 채운 UAV 측 잠정안이며,
팀 스키마와 어긋나는 지점은 **합의 필요** 로 표시했다.

단위는 계약 §8 을 따른다 — 거리/고도 m, 속도·풍속 m/s, 시간 s, 배터리 %, 각도 degree.
"""
from typing import Literal, Optional
from pydantic import BaseModel, Field

import config


class Position(BaseModel):
    """위치. 단위 m, 좌표 WGS84."""
    lat: float
    lon: float
    alt_m_amsl: float = 0.0   # 해수면 기준
    alt_m_agl: float = 0.0    # 지면 기준


class Target(BaseModel):
    """임무 목표. 비행고도 = alt_m_amsl + target_agl_m + config.AGL_MARGIN_M.

    팀 합의안(docs/uav/uav_final 4.1): 고도는 AGL 로 전달하고 Orchestrator 가 정한다.
    지면 고도는 팀 Assignment.target_alt_m 이 alt_m_amsl 로 들어온다.
    target_agl_m 이 없으면 시나리오 기본값을 쓴다. 지면 고도는 추측하면 지형에
    충돌할 수 있으므로 없으면 거절한다.
    """
    lat: float
    lon: float
    alt_m_amsl: Optional[float] = None    # 목표 지점 지면의 해발고도 — 없으면 거절
    target_agl_m: float = config.DEFAULT_TARGET_AGL_M  # 지면 위 목표 높이


class Battery(BaseModel):
    percent: float
    voltage: float


class Health(BaseModel):
    gps_ok: bool
    sensors_ok: bool
    failsafe: bool


class UavState(BaseModel):
    """UAV 상태. 팀 ResourceStatus 에 대응한다.

    합의 필요(시간형식): 여기서는 ISO8601 UTC 문자열을 쓰나, 팀 schema.py 는
    timestamp: float (시뮬레이션 시작 후 경과 초)다. 둘 중 하나로 통일해야 한다.
    """
    uav_id: str                 # 팀 표기: resource_id (예: "A-uav1")
    timestamp: str              # ISO8601 UTC — 합의 필요
    position: Position
    battery: Battery
    velocity_ms: float
    flight_mode: str
    armed: bool
    health: Health
    wind_ms: Optional[float] = None   # 실연동 시 PX4 가 풍속을 주지 않아 None
    link_quality: str
    current_task_id: Optional[str] = None


class EvaluateRequest(BaseModel):
    task_id: str
    decision_id: str
    target: Target
    observation_type: str = "THERMAL"
    deadline: Optional[str] = None
    # 풍속 주입. PX4 는 풍속을 발행하지 않으므로(실측 확인) 외부 제공이 유일한 출처다.
    # 팀 스키마의 EnvironmentState.wind_speed (m/s) 값을 그대로 실어 보내면 된다.
    # 미지정 시 REJECT / WIND_UNKNOWN. (NOTE.md 6-4)
    wind_ms: Optional[float] = Field(
        default=None,
        description="환경모델의 wind_speed (m/s). 미지정 시 판단 불가로 거절",
    )


class EvaluateResponse(BaseModel):
    task_id: str
    decision_id: str
    uav_id: str
    verdict: Literal["ACCEPT", "REJECT", "COUNTER"]
    eta_sec: Optional[int] = None
    reason: Optional[str] = None
    detail: Optional[str] = None
    counter_offer: Optional[dict] = None
    constraints: dict = {}


class ExecuteRequest(BaseModel):
    task_id: str                # 실행 키 (총괄은 실행시도 ID 를 넣는다). 같은 키 = 같은 실행
    decision_id: str
    target: Target
    observation_type: str = "THERMAL"
    execution_attempt_id: Optional[str] = None   # 주면 task_id 와 같아야 한다 (UAV-01 ① 명시용)


class ExecuteResponse(BaseModel):
    task_id: str
    status: str
    tracking_url: str
    uav_id: Optional[str] = None
    duplicate: bool = False     # 같은 키·같은 내용 재요청이면 True (새로 출동하지 않음)


class TaskStatus(BaseModel):
    task_id: str
    uav_id: Optional[str] = None   # UAV-01 ④
    status: Literal["STARTED", "IN_PROGRESS", "COMPLETED", "FAILED"]
    progress: Optional[dict] = None
    observation: Optional[dict] = None
    reason: Optional[str] = None   # FAILED 사유 코드 (예: RETURN_MARGIN_INSUFFICIENT)
    error: Optional[str] = None
    agent_restart: Optional[dict] = None   # 재시작으로 끊긴 실행이면 근거 (physical_basis)
