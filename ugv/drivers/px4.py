# ugv/drivers/px4.py — PX4(SITL/실기체) MAVSDK 드라이버
# 흐름은 tools/try_move.py 에서 검증한 것과 같다: 업로드 -> arm -> 2초 대기 -> 미션 시작.
# 텔레메트리는 백그라운드 태스크가 구독해 snapshot 을 갱신하고, 조회 메서드는 snapshot 만 읽는다.

import asyncio
import logging
import time
from dataclasses import dataclass

from mavsdk_grpc import System
from mavsdk_grpc.mission import MissionItem, MissionPlan

from .. import config
from ..geo import distance_m
from .base import MotionDriver

log = logging.getLogger(__name__)

TELEMETRY_RATE_HZ = 1.0
ARM_SETTLE_S = 2.0   # arm 직후 곧바로 start_mission 하면 DENIED
ACCEPT_RADIUS_M = 10.0                  # 웨이포인트 도착 반경. 2m 는 지나쳐 버린다
# 웨이포인트 간격(2 × 도착 반경)은 ugv/route_plan.py 가 맞춰서 넘긴다. 여기서 다시 솎으면
# 진행률 번호가 경로 계획과 어긋나므로 드라이버는 받은 그대로 올린다.


@dataclass
class Snapshot:
    lat: float = float("nan")
    lon: float = float("nan")
    battery_pct: float = float("nan")   # MAVSDK remaining_percent 원값 그대로
    flight_mode: str = "UNKNOWN"
    armed: bool = False
    rel_alt_m: float = 0.0              # 홈 기준 상대고도. 지형 월드에서는 도로를 따라 수백 m 바뀐다
    updated_at: float = 0.0             # 마지막 위치 수신 시각 (monotonic)
    descent_mps: float = 0.0            # 최근 하강 속도 (시뮬레이션 초당 m, 양수 = 내려감). 추락 판정용


class PX4Driver(MotionDriver):
    def __init__(
        self,
        address: str = "udpin://0.0.0.0:14540",
        speed_mps: float = 3.0,
        alt_m: float = 2.0,
        time_scale: float = 1.0,
    ):
        self.address = address
        self.time_scale = time_scale    # 하강 속도를 시뮬레이션 초 기준으로 바꿀 때 쓴다
        self.speed_mps = speed_mps
        self.alt_m = alt_m
        self.snapshot = Snapshot()
        self._drone = System()
        self._tasks: list[asyncio.Task] = []
        self._progress_task: asyncio.Task | None = None
        self._progress = (0, 0)

    # --- 연결 / 텔레메트리 ---------------------------------------------

    async def connect(self) -> None:
        log.info("PX4 연결 대기: %s (PX4 SITL 이 떠 있어야 한다)", self.address)
        await self._drone.connect(system_address=self.address)
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                break
        log.info("연결됨: %s", self.address)

        # GCS 없이 arm 하기 위한 파라미터.
        # 확인 사항 (v1.18-beta):
        #   SYS_HAS_MAG / FD_FAIL_* 는 건드리지 않는다 — yaw_align 이 깨져 arm 이 거부된다
        #   RD_* 는 기본값이 적절하므로 설정하지 않는다
        #   (RD_TRANS_DRV_TRN 기본값 3.1416 은 rad 이며, 문서의 deg 표기는 v1.16 기준)
        for name, val in (
            ("NAV_RCL_ACT", 0),
            ("NAV_DLL_ACT", 0),
            ("COM_RCL_EXCEPT", 4),
            ("SDLOG_MODE", -1),
        ):
            try:
                await self._drone.param.set_param_int(name, val)
            except Exception as e:
                log.warning("param %s 설정 실패: %s", name, e)

        for name in ("set_rate_position", "set_rate_battery"):
            try:
                await getattr(self._drone.telemetry, name)(TELEMETRY_RATE_HZ)
            except Exception as e:
                log.warning("%s 실패: %s", name, e)

        self._tasks = [
            asyncio.create_task(self._watch_position()),
            asyncio.create_task(self._watch_battery()),
            asyncio.create_task(self._watch_flight_mode()),
            asyncio.create_task(self._watch_armed()),
        ]

    async def _watch_position(self) -> None:
        try:
            async for p in self._drone.telemetry.position():
                now, s = time.monotonic(), self.snapshot
                if s.updated_at and now > s.updated_at:
                    wall = now - s.updated_at
                    s.descent_mps = (s.rel_alt_m - p.relative_altitude_m) / (wall * self.time_scale)
                s.lat = p.latitude_deg
                s.lon = p.longitude_deg
                s.rel_alt_m = p.relative_altitude_m
                s.updated_at = now
        except Exception:
            log.exception("position 구독 종료")

    async def _watch_battery(self) -> None:
        try:
            async for b in self._drone.telemetry.battery():
                self.snapshot.battery_pct = b.remaining_percent
        except Exception:
            log.exception("battery 구독 종료")

    async def _watch_flight_mode(self) -> None:
        try:
            async for m in self._drone.telemetry.flight_mode():
                self.snapshot.flight_mode = str(m)
        except Exception:
            log.exception("flight_mode 구독 종료")

    async def _watch_armed(self) -> None:
        try:
            async for a in self._drone.telemetry.armed():
                self.snapshot.armed = a
        except Exception:
            log.exception("armed 구독 종료")

    async def close(self) -> None:
        """백그라운드 태스크 정리."""
        tasks = self._tasks + ([self._progress_task] if self._progress_task else [])
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        self._tasks = []
        self._progress_task = None

    # --- MotionDriver ---------------------------------------------------

    def position(self) -> tuple[float, float]:
        return self.snapshot.lat, self.snapshot.lon

    def battery(self) -> float:
        return self.snapshot.battery_pct

    def status(self) -> str:
        return self.snapshot.flight_mode

    def progress(self) -> tuple[int, int]:
        return self._progress

    def fault(self) -> str | None:
        """미션 수행 중(웨이포인트가 남아 있을 때)만 판정한다. 확정 지연은 server 가 한다."""
        s, (cur, total) = self.snapshot, self._progress
        if s.updated_at and time.monotonic() - s.updated_at > config.STALE_AFTER_S:
            return f"TELEMETRY_LOST: 위치 수신 {time.monotonic() - s.updated_at:.0f}초 없음"
        if s.descent_mps > config.FALL_RATE_MPS:
            return f"VEHICLE_FAULT: 초당 {s.descent_mps:.1f} m 하강 (추락, 상대고도 {s.rel_alt_m:.0f} m)"
        if total and cur < total:
            if not s.armed:
                return "VEHICLE_FAULT: 미션 중 disarm"
            if "MISSION" not in s.flight_mode:
                return f"VEHICLE_FAULT: 미션 중 모드 이탈 ({s.flight_mode})"
        return None

    async def goto(self, waypoints: list[tuple[float, float]],
                   speeds: list[float] | None = None) -> bool:
        """주행 중 다시 부르면 새 미션으로 바꾼다 (경로 재탐색). 미션은 처음 항목부터 시작한다."""
        if not waypoints:
            return False

        if self._progress_task:
            self._progress_task.cancel()
            self._progress_task = None
        self._progress = (0, len(waypoints))

        # MAVSDK 는 항목 i 의 속도를 항목 i 에 도착한 뒤(DO_CHANGE_SPEED)부터 적용한다.
        # speeds[i] 는 'i 로 가는 구간' 속도이므로 항목 i 에는 speeds[i+1](다음 구간)을 싣고,
        # 첫 구간 속도는 기본 순항속도(self.speed_mps)를 그 구간 속도로 바꿔 쓴다.
        speeds = list(speeds) if speeds else [self.speed_mps] * len(waypoints)
        nxt = speeds[1:] + speeds[-1:]
        self._first_speed = speeds[0]
        # 중간 웨이포인트는 통과형(is_fly_through=True): 멈추지 않고 지나가며 다음 점으로 꺾는다.
        # 멈춤형이면 점마다 정지 → 제자리 회전을 하는데, 경사진 지형에서 회전을 못 끝내 멈춘 채로
        # 남는 일이 있었다 (2026-10-02 WSL 시험, 61° 꺾임·경사 7.5° 지점에서 STALLED). 마지막 점만 멈춤형.
        last = len(waypoints) - 1
        plan = MissionPlan([self._waypoint(lat, lon, v, fly_through=(i < last))
                            for i, ((lat, lon), v) in enumerate(zip(waypoints, nxt))])
        try:
            await self._drone.mission.upload_mission(plan)
            log.info("미션 업로드 성공 (%d 개)", len(waypoints))
            try:    # 첫 구간 속도. 실패해도 미션은 기본 순항속도로 간다
                await self._drone.action.set_current_speed(self._first_speed)
            except Exception as e:
                log.warning("set_current_speed 실패: %s", e)
            await self._drone.action.arm()
            log.info("arm 성공")
            await asyncio.sleep(ARM_SETTLE_S)
            await self._drone.mission.start_mission()
            log.info("미션 시작")
        except Exception as e:
            log.error("goto 실패: %s", e)
            return False

        self._progress_task = asyncio.create_task(self._watch_progress())
        return True

    async def stop(self) -> None:
        """미션을 멈추고 HOLD 로 세운 뒤 disarm 한다. 하나가 실패해도 나머지는 시도한다."""
        if self._progress_task:
            self._progress_task.cancel()
            self._progress_task = None
        for name, call in (("pause_mission", self._drone.mission.pause_mission),
                           ("hold", self._drone.action.hold),
                           ("disarm", self._drone.action.disarm)):
            try:
                await call()
                log.info("stop: %s 성공", name)
            except Exception as e:
                log.warning("stop: %s 실패: %s", name, e)

    @staticmethod
    def _thin(waypoints: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """직전 점과 MIN_WAYPOINT_GAP_M 보다 가까운 중간점을 뺀다. 마지막 점은 항상 남긴다.
        도로망 교차로 노드는 수 m 간격으로 몰려 있어, 도착 반경(ACCEPT_RADIUS_M) 안에
        다음 점이 들어가 있으면 rover 가 점을 건너뛰거나 도착 판정이 꼬인다."""
        kept = [waypoints[0]]
        for p in waypoints[1:-1]:
            if distance_m(kept[-1], p) >= MIN_WAYPOINT_GAP_M:
                kept.append(p)
        if len(waypoints) > 1:
            if distance_m(kept[-1], waypoints[-1]) < MIN_WAYPOINT_GAP_M and len(kept) > 1:
                kept.pop()
            kept.append(waypoints[-1])
        return kept

    def _waypoint(self, lat: float, lon: float, speed_mps: float | None = None,
                  fly_through: bool = False) -> MissionItem:
        nan = float("nan")
        return MissionItem(
            lat, lon, self.alt_m,
            speed_mps or self.speed_mps,
            fly_through,           # is_fly_through — True 면 이 점에서 멈추지 않는다
            nan, nan,
            MissionItem.CameraAction.NONE,
            nan, nan,
            ACCEPT_RADIUS_M, nan, nan,
            MissionItem.VehicleAction.NONE,
        )

    async def _watch_progress(self) -> None:
        try:
            async for prog in self._drone.mission.mission_progress():
                self._progress = (prog.current, prog.total)
                if prog.current == prog.total:
                    break
        except Exception:
            log.exception("mission_progress 구독 종료")