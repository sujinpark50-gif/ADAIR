# -*- coding: utf-8 -*-
"""ugv/clock.py — UGV 시뮬레이션 시계.

시뮬레이션 초 = 기준 + (벽시계 단조 경과) × TIME_SCALE. PX4·Gazebo 주행은 실시간(배율 1)이다.
환경 시각에 맞추는 것(sync)은 4단계 관측에서 쓴다. 통신 단절 판단은 이 시계가 아니라 벽시계 단조 시간을 쓴다 (7단계).
시험에서는 ManualClock 으로 시간을 직접 진행한다.
"""

import time

from . import config


class SimClock:
    def __init__(self, scale: float = config.TIME_SCALE, start_s: float = 0.0):
        self.scale = scale
        self._base_sim = start_s
        self._base_mono = time.monotonic()

    def now(self) -> float:
        return self._base_sim + (time.monotonic() - self._base_mono) * self.scale

    def sync(self, sim_time_s: float) -> float:
        """외부(환경) 시각에 맞춘다. 맞추기 전과의 차이(내 시계 - 외부)를 돌려준다."""
        drift = self.now() - sim_time_s
        self._base_sim, self._base_mono = sim_time_s, time.monotonic()
        return drift

    def to_wall(self, sim_s: float) -> float:
        return sim_s / self.scale


class ManualClock:
    scale = 1.0

    def __init__(self, start_s: float = 0.0):
        self.t = start_s

    def now(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt

    def sync(self, sim_time_s: float) -> float:
        d, self.t = self.t - sim_time_s, sim_time_s
        return d

    def to_wall(self, sim_s: float) -> float:
        return 0.0
