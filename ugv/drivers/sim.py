# -*- coding: utf-8 -*-
"""ugv/drivers/sim.py — PX4 없이 도는 운동학 드라이버 (시험, 트윈 배속 실행).

미션 지점을 이은 선을 그대로 따라간다 (경로 추종 오차 없음). 속도는 지점별 구간 속도, 가감속 한계는
2단계 Gazebo 실측(가속 PX4 RO_ACCEL_LIM 2.5, 도착 감속 실측 약 2.1 m/s²)에 맞췄다.
시간은 clock(시뮬레이션 초)을 따른다. 실행 중에는 run() 이 주기적으로 step() 을 부르고, 시험은 step(dt) 를 직접 부른다.
"""

import asyncio
import math
from typing import List, Optional

from ..geo import FRAME, bearing_deg
from .base import Driver, DriverError, Item, Progress, Telemetry

ACCEL = 2.5
DECEL = 2.1
TICK_WALL_S = 0.05


class SimDriver(Driver):
    kind = "sim"

    def __init__(self, start_xy, clock, heading_deg: float = 0.0, accel: float = ACCEL, decel: float = DECEL):
        self.clock = clock
        self.accel, self.decel = accel, decel     # 차량 특성별 (UGV 기본 2.5 / 2.1 — 2단계 실측)
        self.x, self.y = start_xy
        self.v = 0.0
        self.heading = heading_deg            # 방위 (북=0, 시계). 정차 중에도 진행 방향 제약에 쓴다
        self.path: List[Item] = []
        self.k = 0
        self._prog = Progress()
        self._last_sim = clock.now()
        self._task: Optional[asyncio.Task] = None
        self.fail_next_follow: Optional[str] = None   # 시험용: 다음 follow 를 거부
        self.frozen = False                            # 시험용: 차가 움직이지 않음 (정지 고장)

    async def start(self):
        if self._task is None:
            self._last_sim = self.clock.now()
            self._task = asyncio.create_task(self._run())

    async def close(self):
        if self._task:
            self._task.cancel()
            self._task = None

    async def _run(self):
        while True:
            await asyncio.sleep(TICK_WALL_S)
            now = self.clock.now()
            self.step(now - self._last_sim)
            self._last_sim = now

    def telemetry(self) -> Telemetry:
        lat, lon = FRAME.to_ll(self.x, self.y)
        return Telemetry(self.x, self.y, lat, lon, self.v, self.heading, armed=bool(self.path),
                         mode="MISSION" if self.path and not self._prog.finished else "HOLD")

    def progress(self) -> Progress:
        return self._prog

    async def follow(self, items: List[Item]) -> int:
        if self.fail_next_follow:
            msg, self.fail_next_follow = self.fail_next_follow, None
            raise DriverError(msg)
        if not items:
            raise DriverError("EMPTY_MISSION")
        self.path, self.k = list(items), 0
        self._prog = Progress(0, len(items), False, self._prog.mission_seq + 1)
        return self._prog.mission_seq

    async def halt_in_place(self) -> int:
        self.path, self.k, self.v = [], 0, 0.0
        self._prog = Progress(0, 0, True, self._prog.mission_seq + 1)
        return self._prog.mission_seq

    def _remaining(self) -> float:
        if not self.path:
            return 0.0
        d = math.hypot(self.path[self.k][0] - self.x, self.path[self.k][1] - self.y)
        for (ax, ay, _), (bx, by, _) in zip(self.path[self.k:], self.path[self.k + 1:]):
            d += math.hypot(bx - ax, by - ay)
        return d

    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        if not self.path or self._prog.finished or self.frozen:
            self.v = max(0.0, self.v - self.decel * dt) if not self.frozen else 0.0
            return
        tx, ty, seg_v = self.path[self.k]
        v_target = min(seg_v, math.sqrt(2 * self.decel * self._remaining()))
        self.v = min(self.v + self.accel * dt, v_target) if self.v < v_target else max(self.v - self.decel * dt, v_target)
        move = self.v * dt
        while move > 0 and self.path:
            tx, ty, _ = self.path[self.k]
            d = math.hypot(tx - self.x, ty - self.y)
            if d > 1e-9:
                self.heading = bearing_deg(tx - self.x, ty - self.y)
            if move < d:
                self.x += (tx - self.x) * move / d
                self.y += (ty - self.y) * move / d
                move = 0
            else:
                self.x, self.y = tx, ty
                move -= d
                if self.k + 1 < len(self.path):
                    self.k += 1
                else:
                    self.v = 0.0
                    self._prog = Progress(len(self.path), len(self.path), True, self._prog.mission_seq)
                    break
        if not self._prog.finished:
            self._prog = Progress(self.k, len(self.path), False, self._prog.mission_seq)
