# -*- coding: utf-8 -*-
"""ugv/drivers/base.py — 주행 드라이버 공통 인터페이스.

드라이버는 '받은 미션 지점 목록을 따라 달리고, 정지 명령에 멈추고, 위치를 알려 주는 것'만 한다.
경로 계획·임무 상태·취소 규칙은 ugv/vehicle.py 가 갖는다. 한 드라이버에 동시에 미션은 하나뿐이다:
follow() 는 기존 미션을 통째로 교체한다 (주행 명령이 둘 남지 않는다).
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .. import config
from ..geo import FRAME

Item = Tuple[float, float, float]      # (x, y, 그 지점까지 구간 속도 m/s), 지역 평면


class DriverError(Exception):
    pass


@dataclass
class Telemetry:
    x: float
    y: float
    lat: float
    lon: float
    speed_mps: float
    heading_deg: float
    armed: bool
    mode: str
    updated_mono: float = field(default_factory=time.monotonic)   # 마지막 수신 (위치·속도 중 오래된 쪽, 벽시계 단조. 속도 미수신이면 -inf)
    alt_amsl_m: Optional[float] = None

    def age_s(self) -> float:
        return time.monotonic() - self.updated_mono


@dataclass
class Progress:
    current: int = 0
    total: int = 0
    finished: bool = False
    mission_seq: int = 0               # follow() 호출마다 1 증가 — 지난 미션의 '끝남'을 새 미션으로 오해하지 않게


class Driver:
    kind = "base"

    async def start(self) -> None: ...
    async def close(self) -> None: ...
    def telemetry(self) -> Optional[Telemetry]: ...
    def progress(self) -> Progress: ...
    async def follow(self, items: List[Item]) -> int:
        """미션 교체 후 출발. 새 mission_seq 를 돌려준다."""
        raise NotImplementedError

    # 끝 지점 정지 반경 보정 (E4): PX4 가 마지막 지점 '도착 반경' 경계에서 스스로 서는 차(RA_ACC_RAD_MAX ≥ NAV_ACC_RAD)만 그 반경 (m).
    # 기본 드라이버·sim 은 0 — 연장하지 않는다
    end_stop_offset_m: float = 0.0
    end_stop_info: dict = {"basis": "기본 드라이버 — 끝 지점 연장 없음"}
    park_policy: dict = {"after_s": None, "test_hook": False, "basis": "기본 드라이버 — 시동 끔 정차 없음"}

    async def set_speed_cap(self, v: float) -> bool:
        """현재 미션의 속도 상한 명령 (PX4: MAV_CMD_DO_CHANGE_SPEED). 기본 드라이버는 없음 → False. UGV_STOP_SPEED_CMD 후보용."""
        return False

    async def stop(self) -> int:
        """진행 방향 앞 제동거리 지점에서 멈추는 미션으로 교체 (PX4 rover HOLD 는 U턴하므로 쓰지 않는다)."""
        t = self.telemetry()
        if t is None:
            raise DriverError("TELEMETRY_UNAVAILABLE")
        v = max(t.speed_mps, 0.0)
        if v < 0.3:
            return await self.halt_in_place()
        d = v * v / (2 * (getattr(self, "decel_mps2", None) or config.DECEL_MPS2)) + config.STOP_MARGIN_M
        hd = math.radians(t.heading_deg)
        return await self.follow([(t.x + d * math.sin(hd), t.y + d * math.cos(hd), max(v, 1.0))])

    async def halt_in_place(self) -> int:
        raise NotImplementedError

    async def cut_drive(self) -> int:
        """구동 차단 (정지 감독의 마지막 수단 — 정지 미션으로 서지 않는 차). 실패하면 DriverError."""
        raise NotImplementedError

    @staticmethod
    def to_ll(x, y):
        return FRAME.to_ll(x, y)
