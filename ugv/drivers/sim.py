# ugv/drivers/sim.py — PX4 없이 동작하는 시뮬레이션 드라이버 (테스트·시연용)
# 웨이포인트 사이를 등속 직선 이동한다. 위치는 tick 마다 speed_mps * tick_s 만큼 갱신.

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
    ):
        self._pos = start
        self.speed_mps = speed_mps
        self.tick_s = tick_s
        self._battery = 100.0
        self._status = "IDLE"
        self._progress = (0, 0)
        self._task: asyncio.Task | None = None

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

    async def goto(self, waypoints: list[tuple[float, float]]) -> bool:
        if not waypoints:
            return False
        if self._task:
            self._task.cancel()
        self._progress = (0, len(waypoints))
        self._status = "MISSION"
        self._task = asyncio.create_task(self._run(list(waypoints)))
        return True

    async def _run(self, waypoints: list[tuple[float, float]]) -> None:
        step_m = self.speed_mps * self.tick_s
        for i, target in enumerate(waypoints, start=1):
            while True:
                remaining = distance_m(self._pos, target)
                if remaining <= step_m:
                    self._advance(remaining, target)
                    break
                # 현재 위치 기준 로컬 좌표에서 목표 방향으로 step_m 전진
                north, east = to_ned(target[0], target[1], *self._pos)
                ratio = step_m / remaining
                nxt = to_latlon(north * ratio, east * ratio, *self._pos)
                self._advance(step_m, nxt)
                await asyncio.sleep(self.tick_s)
            self._progress = (i, len(waypoints))
        self._status = "ARRIVED"

    def _advance(self, moved_m: float, new_pos: tuple[float, float]) -> None:
        self._pos = new_pos
        drain = moved_m / 1000.0 * BATTERY_DRAIN_PCT_PER_KM
        self._battery = max(0.0, self._battery - drain)
