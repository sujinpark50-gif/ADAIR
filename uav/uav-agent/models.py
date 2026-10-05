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
    returning_task_id: Optional[str] = None   # 관측을 마치고 복귀 중인 Task (UAV-06)


Measurement = str   # "THERMAL" | "RGB" | "WEATHER". 지원 여부는 evaluator 가 본다 (모르면 REJECT)

_OBSERVE_FIELD = Field(
    default=None, ge=config.OBSERVE_DURATION_MIN_S, le=config.OBSERVE_DURATION_MAX_S,
    description="관측 체류시간(s). 없으면 config.OBSERVE_DURATION_S. "
                "RETURN_MARGIN_INSUFFICIENT COUNTER 가 제안한 값을 그대로 실어 재평가·실행한다 (UAV-02)",
)
_MEASUREMENTS_FIELD = Field(
    default=None, min_length=1,
    description='한 방문에서 할 측정 목록. 예) ["THERMAL","WEATHER"]. 결과도 측정별로 돌려준다 (UAV-07). '
                "observation_type 도 주면 둘을 합친다. 둘 다 없으면 이동만 평가·수행한다",
)


class EvaluateRequest(BaseModel):
    task_id: str
    decision_id: str
    target: Target
    # 없으면 이동 가능성만 평가한다 (기상 전용 임무 — 측정은 총괄 모의 센서, UAV-07 ②)
    observation_type: Optional[str] = None
    measurements: Optional[list[Measurement]] = _MEASUREMENTS_FIELD
    observe_duration_s: Optional[int] = _OBSERVE_FIELD
    # 기한: 남은 시뮬레이션 시간(s). 실제 시계(UTC)와 비교하지 않는다 (UAV-04). 이전 deadline(ISO 문자열)은 받지 않는다
    remaining_time_s: Optional[float] = Field(
        default=None, description="기한까지 남은 시뮬레이션 시간(s). eta_sec 보다 작으면 REJECT")
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
    counter_offer: Optional[dict] = None   # 재평가 요청에 그대로 실을 수 있는 필드만 담는다 (UAV-02)
    constraints: dict = {}
    # ACCEPT 일 때만. 실행 요청에 같은 route_id·route_hash 를 실으면 평가한 경로 그대로 난다 (UAV-03)
    route_id: Optional[str] = None
    route_version: Optional[str] = None
    route_hash: Optional[str] = None
    route: Optional[dict] = None           # 순항고도·경유점·지형 확인 결과


class ExecuteRequest(BaseModel):
    task_id: str                # 실행 키 (총괄은 실행시도 ID 를 넣는다). 같은 키 = 같은 실행
    decision_id: str
    target: Target
    observation_type: Optional[str] = None
    measurements: Optional[list[Measurement]] = _MEASUREMENTS_FIELD
    observe_duration_s: Optional[int] = _OBSERVE_FIELD
    execution_attempt_id: Optional[str] = None   # 주면 task_id 와 같아야 한다 (UAV-01 ① 명시용)
    route_id: Optional[str] = None     # 평가 응답의 값. 주면 그 경로로만 실행한다 (UAV-03)
    route_hash: Optional[str] = None


class AbortRequest(BaseModel):
    reason: Optional[str] = None


class ExecuteResponse(BaseModel):
    task_id: str
    status: str
    tracking_url: str
    uav_id: Optional[str] = None
    duplicate: bool = False     # 같은 키·같은 내용 재요청이면 True (새로 출동하지 않음)
    route_id: Optional[str] = None
    route_hash: Optional[str] = None


class TaskStatus(BaseModel):
    """실행 상태.

    status 와 mission_result 는 같은 규칙이다 (UAV-10):
      COMPLETED ⇔ mission_result=OBSERVED     관측(도착+체류)을 마쳤다. 이후 복귀에 실패해도 COMPLETED 와
                                              관측(observation_id·내용·시각)은 바뀌지 않는다
      FAILED    ⇔ mission_result=NOT_OBSERVED 관측 전에 끝났다. observation 은 비어 있다
      STARTED·IN_PROGRESS ⇔ PENDING
    물리 진행은 physical_state 로 따로 본다: PREPARING → ENROUTE → OBSERVING → RETURNING → LANDED,
    실패·불명은 RETURN_FAILED / RETASKED / UNKNOWN (+ physical_failure_reason).
    progress.phase 는 기존 값 그대로 둔다 (LANDED = phase DONE).
    """
    task_id: str
    uav_id: Optional[str] = None   # UAV-01 ④
    status: Literal["STARTED", "IN_PROGRESS", "COMPLETED", "FAILED"]
    progress: Optional[dict] = None
    observation: Optional[dict] = None
    reason: Optional[str] = None   # FAILED 사유 코드 (예: RETURN_MARGIN_INSUFFICIENT, ABORTED)
    error: Optional[str] = None
    agent_restart: Optional[dict] = None   # 재시작으로 끊긴 실행이면 근거 (physical_basis)
    mission_result: Optional[Literal["OBSERVED", "NOT_OBSERVED", "PENDING"]] = None   # UAV-10
    physical_state: Optional[str] = None
    physical_failure_reason: Optional[str] = None
    abort: Optional[dict] = None           # 중단 요청을 받았으면 그 기록 (UAV-09)
    route_id: Optional[str] = None
    route_hash: Optional[str] = None
