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


class Position(BaseModel):
    """위치. 단위 m, 좌표 WGS84.

    드론은 goto(위도, 경도, 고도) 형태로 명령하므로 목표 고도가 필수다.
    팀 Assignment.target_alt_m 이 여기 alt_m_amsl 로 들어온다(기본 0.0 이면 지형 무시).
    팀 합의안(docs/uav/uav_final 4.1)은 AGL 전달이나, 현재는 목표 지면 AMSL 을 받는다.
    """
    lat: float
    lon: float
    alt_m_amsl: float = 0.0   # 해수면 기준 — 지형 위 여유고도 계산에 사용
    alt_m_agl: float = 0.0    # 지면 기준


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
    target: Position
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
    task_id: str
    decision_id: str
    target: Position
    observation_type: str = "THERMAL"


class ExecuteResponse(BaseModel):
    task_id: str
    status: str
    tracking_url: str


class TaskStatus(BaseModel):
    task_id: str
    status: Literal["STARTED", "IN_PROGRESS", "COMPLETED", "FAILED"]
    progress: Optional[dict] = None
    observation: Optional[dict] = None
    error: Optional[str] = None
