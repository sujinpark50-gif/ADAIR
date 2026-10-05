# ugv/drivers/sim.py — PX4 없이 동작하는 시뮬레이션 드라이버 (테스트·시연용)
# 웨이포인트 사이를 직선 이동한다. 위치는 tick(벽시계) 마다 속도 × time_scale × tick_s 만큼 갱신.
#   속도: goto 의 speeds[i](구간별, 시뮬레이션 m/s) 가 있으면 그것, 없으면 speed_mps
#   time_scale: 시뮬레이션 초 / 벽시계 초 (config.TIME_SCALE). PX4_SIM_SPEED_FACTOR 와 같은 의미

import asyncio

from ..geo import distance_m, to_latlon, to_ned
from .base import MotionDriver

BATTERY_DRAIN_PCT_PER_KM = 1.0


class SimDriver(MotionDriver):
    def __init__(
        self,
        start: tuple[float, float],
        speed_mps: float = 3.0,
        tick_s: float = 0.1,
        time_scale: float = 1.0,
    ):
        self._pos = start
        self.speed_mps = speed_mps
        self.time_scale = time_scale
        self.tick_s = tick_s
        self._battery = 100.0
        self._status = "IDLE"
        self._progress = (0, 0)
        self._task: asyncio.Task | None = None
        self._speed = 0.0

    async def connect(self) -> None:
        pass

    def position(self) -> tuple[float, float]:
        return self._pos

    def battery(self) -> float:
        return self._battery

    def status(self) -> str:
        return self._status

    def progress(self) -> tuple[int, int]:
        return self._progress

    def telemetry(self) -> dict:
        return {"speed_mps": self._speed if self._status == "MISSION" else 0.0}

    async def goto(self, waypoints: list[tuple[float, float]],
                   speeds: list[float] | None = None) -> bool:
        if not waypoints:
            return False
        if self._task:
            self._task.cancel()
        self._progress = (0, len(waypoints))
        self._status = "MISSION"
        speeds = list(speeds) if speeds else [self.speed_mps] * len(waypoints)
        self._task = asyncio.create_task(self._run(list(waypoints), speeds))
        return True

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            self._task = None
        self._status = "STOPPED"

    async def _run(self, waypoints: list[tuple[float, float]], speeds: list[float]) -> None:
        """tick 마다 '구간 속도 × time_scale × tick_s' 만큼 이동한다. 한 tick 에 웨이포인트를
        여러 개 지날 수 있다 — 고배속에서 웨이포인트마다 tick 을 하나씩 쓰면 시계보다 늦어진다."""
        i, n = 0, len(waypoints)
        while i < n:
            budget = self.tick_s * self.time_scale          # 이번 tick 의 시뮬레이션 초
            self._speed = speeds[i]
            while i < n and budget > 0:
                target, v = waypoints[i], speeds[i]
                remaining = distance_m(self._pos, target)
                if remaining <= v * budget:                 # 이번 tick 안에 도착 — 남은 시간은 다음 구간으로
                    budget -= remaining / v if v > 0 else budget
                    self._advance(remaining, target)
                    i += 1
                    self._progress = (i, n)
                else:                                       # 목표 방향으로 v * budget 전진
                    step_m = v * budget
                    north, east = to_ned(target[0], target[1], *self._pos)
                    ratio = step_m / remaining
                    self._advance(step_m, to_latlon(north * ratio, east * ratio, *self._pos))
                    budget = 0
            if i < n:
                await asyncio.sleep(self.tick_s)
        self._status = "ARRIVED"

    def _advance(self, moved_m: float, new_pos: tuple[float, float]) -> None:
        self._pos = new_pos
        drain = moved_m / 1000.0 * BATTERY_DRAIN_PCT_PER_KM
        self._battery = max(0.0, self._battery - drain)
