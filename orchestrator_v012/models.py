"""공통 스키마를 수정하지 않고 총괄 입력에 필요한 필드만 확장한다."""

from dataclasses import dataclass
from typing import Optional

from interfaces.schema import LocalResponse, ResourceStatus, Task


@dataclass
class UavStatus(ResourceStatus):
    # UAV API의 ISO 시각 원문. 공통 로그의 float timestamp로 변환하지 않는다.
    source_timestamp: Optional[str] = None
    # READY/OFFLINE은 UAV 원본 state가 아니라 Adapter 파생값이다.
    state_source: Optional[str] = None
    current_task_id: Optional[str] = None


@dataclass
class UavEvaluation(LocalResponse):
    task_id: Optional[str] = None
    decision_id: Optional[str] = None
    eta_sec: Optional[float] = None
    detail: Optional[str] = None


@dataclass
class FireTask(Task):
    target_alt_m: float = 0.0
    target_node: Optional[str] = None
    grid_x: Optional[int] = None
    grid_y: Optional[int] = None
