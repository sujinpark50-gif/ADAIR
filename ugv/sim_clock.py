# ugv/sim_clock.py — UGV 가 쓰는 시뮬레이션 시계
#
# 기준은 환경의 시계(sim_step)다. 총괄은 시계가 없고, 환경 CA 가 스텝을 센다.
# 다만 환경은 스텝 번호만 주고 1 스텝이 몇 초인지 정의가 없다(INT-05 미정) → SECONDS_PER_ENV_STEP 설정값.
#
#   시뮬레이션 초 = sim_step × SECONDS_PER_ENV_STEP        (환경이 알려 준 순간)
#                 + (그 뒤 흐른 벽시계 초) × TIME_SCALE      (다음 스텝을 받기 전까지)
#
# 환경이 스텝을 안 알려 주면(단독 실행) 서버 시작 시각을 0 으로 벽시계 × TIME_SCALE 로만 간다.
# TIME_SCALE 은 Gazebo/PX4 가 도는 배율(PX4_SIM_SPEED_FACTOR)과 같아야 차량 속도와 시계가 맞는다.
#
# 시나리오 시각(ugv/scenario.py), ETA(eta_sec), 차량 속도(m/s)는 모두 이 시뮬레이션 초 기준이다.

import time


class SimClock:
    def __init__(self, time_scale: float = 1.0, seconds_per_env_step: float = 60.0):
        if time_scale <= 0 or seconds_per_env_step <= 0:
            raise ValueError("time_scale, seconds_per_env_step 는 0 보다 커야 한다")
        self.time_scale = time_scale
        self.seconds_per_env_step = seconds_per_env_step
        self._anchor_wall = time.monotonic()
        self._anchor_sim = 0.0
        self.env_step: int | None = None          # 마지막으로 받은 환경 스텝
        self.env_synced_at: float | None = None   # 그때의 벽시계 (monotonic)
        self.last_drift_s: float | None = None    # 동기화 직전 내 시계 - 환경 시계 (시뮬레이션 초)

    def now(self) -> float:
        """현재 시뮬레이션 초."""
        return self._anchor_sim + (time.monotonic() - self._anchor_wall) * self.time_scale

    def env_step_now(self) -> float:
        """현재 시각을 환경 스텝 단위로 (소수). 시각화·로그용."""
        return self.now() / self.seconds_per_env_step

    def sync_env(self, sim_step: int) -> dict:
        """환경 스텝을 받아 시계를 맞춘다. 스텝이 줄었으면 새 실행으로 보고 reset=True 를 돌려준다."""
        env_s = sim_step * self.seconds_per_env_step
        reset = self.env_step is not None and sim_step < self.env_step
        self.last_drift_s = None if reset else round(self.now() - env_s, 1)
        self._anchor_sim, self._anchor_wall = env_s, time.monotonic()
        self.env_step, self.env_synced_at = sim_step, self._anchor_wall
        return {"sim_time_s": env_s, "drift_s": self.last_drift_s, "reset": reset}

    def reset(self, sim_time_s: float = 0.0) -> None:
        self._anchor_sim, self._anchor_wall = sim_time_s, time.monotonic()
        self.env_step = self.env_synced_at = self.last_drift_s = None

    def info(self) -> dict:
        return {
            "sim_time_s": round(self.now(), 1),
            "env_step": round(self.env_step_now(), 2),
            "time_scale": self.time_scale,
            "seconds_per_env_step": self.seconds_per_env_step,
            "source": "environment" if self.env_step is not None else "local",
            "last_env_step": self.env_step,
            "last_drift_s": self.last_drift_s,
        }
