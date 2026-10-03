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
#
# PX4 시간 따라가기 (set_source, 2026-10-03)
#   Gazebo 배속은 '상한'이라 CPU 가 밀리면 순간 0.6~4 로 흔들린다. 벽시계 × TIME_SCALE 로 세면 서버 시계만
#   앞서 간다 (실측: 10분 주행에 PX4 보다 6~10% 빠름 → 시나리오 사건이 그만큼 일찍 발동).
#   그래서 PX4 시각(lockstep 이라 = Gazebo 시뮬레이션 시간)이 들어오면 '흐름'은 그걸 따른다:
#     시뮬레이션 초 = 기준점 + (지금 PX4 시각 − 기준점의 PX4 시각)
#   기준점(환경 스텝 동기화·reset)은 그대로라, 환경 시계를 우리가 바꾸는 일은 없다 — 읽기만 한다.
#   PX4 시각이 STALE_S(벽시계) 넘게 안 바뀌면(연결 끊김) 벽시계 × TIME_SCALE 로 이어서 간다.
#   소스가 바뀌거나(다른 차량) PX4 를 다시 띄워 시각이 줄면 그 순간 값으로 이어 붙인다 (시계가 뒤로 가지 않게).

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
        self._source = None                       # () -> (소스 이름, 시뮬레이션 초) | None
        self._src_key = None                      # 기준점을 잡은 소스 이름. None 이면 벽시계로 흐른다
        self._src_anchor = 0.0                    # 기준점에서의 소스 시각
        self._src_last = None                     # (값, 벽시계) — 멈춤 판정용
        self.source_switches = 0
        self._last_value = 0.0

    STALE_S = 10.0

    def set_source(self, fn) -> None:
        """fn() → (이름, 시뮬레이션 초) 또는 None. 예: PX4 imu 시각."""
        self._source = fn

    def _read_source(self):
        if self._source is None:
            return None
        try:
            got = self._source()
        except Exception:   # noqa: BLE001 — 소스가 이상하면 벽시계로
            return None
        if not got or got[1] is None:
            return None
        key, t = got
        wall = time.monotonic()
        if self._src_last is None or t != self._src_last[0]:
            self._src_last = (t, wall)
        elif wall - self._src_last[1] > self.STALE_S:
            return None                           # 값이 오래 안 바뀜 → 끊긴 것으로 본다
        return key, float(t)

    def _wall_now(self) -> float:
        return self._anchor_sim + (time.monotonic() - self._anchor_wall) * self.time_scale

    def _rebase(self, sim_s: float, src=None) -> None:
        """시뮬레이션 초 sim_s 를 지금 순간에 고정하고 이후 흐름의 기준을 잡는다."""
        self._anchor_sim, self._anchor_wall = sim_s, time.monotonic()
        self._last_value = sim_s
        if src is None:
            self._src_key = None
        else:
            self._src_key, self._src_anchor = src

    def now(self) -> float:
        """현재 시뮬레이션 초. PX4 시각이 있으면 그 흐름, 없으면 벽시계 × TIME_SCALE. 뒤로 가지 않는다."""
        src = self._read_source()
        if src is None:
            if self._src_key is not None:         # 소스가 끊김 → 마지막 값에서 벽시계로 이어 간다
                self._rebase(self._last_value)
                self.source_switches += 1
            v = self._wall_now()
        else:
            key, t = src
            if key != self._src_key or t < self._src_anchor:   # 새 소스 / PX4 재시작 → 지금 값에서 이어 붙인다
                base = self._wall_now() if self._src_key is None else self._last_value
                self._rebase(base, (key, t))
                self.source_switches += 1
            v = self._anchor_sim + (t - self._src_anchor)
        self._last_value = v
        return v

    def env_step_now(self) -> float:
        """현재 시각을 환경 스텝 단위로 (소수). 시각화·로그용."""
        return self.now() / self.seconds_per_env_step

    def sync_env(self, sim_step: int) -> dict:
        """환경 스텝을 받아 시계를 맞춘다. 스텝이 줄었으면 새 실행으로 보고 reset=True 를 돌려준다."""
        env_s = sim_step * self.seconds_per_env_step
        reset = self.env_step is not None and sim_step < self.env_step
        self.last_drift_s = None if reset else round(self.now() - env_s, 1)
        self._rebase(env_s, self._read_source())
        self.env_step, self.env_synced_at = sim_step, self._anchor_wall
        return {"sim_time_s": env_s, "drift_s": self.last_drift_s, "reset": reset}

    def reset(self, sim_time_s: float = 0.0) -> None:
        self._rebase(sim_time_s, self._read_source())
        self.env_step = self.env_synced_at = self.last_drift_s = None

    def info(self) -> dict:
        return {
            "sim_time_s": round(self.now(), 1),
            "env_step": round(self.env_step_now(), 2),
            "time_scale": self.time_scale,
            "seconds_per_env_step": self.seconds_per_env_step,
            "source": "environment" if self.env_step is not None else "local",
            "rate_source": self._src_key or "wall",   # 시계가 무엇의 흐름을 따르는지 (px4:<자원> | wall)
            "rate_source_switches": self.source_switches,
            "last_env_step": self.env_step,
            "last_drift_s": self.last_drift_s,
        }
