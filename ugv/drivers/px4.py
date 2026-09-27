# ugv/drivers/px4.py — PX4(SITL/실기체) MAVSDK 드라이버
# 흐름은 tools/try_move.py 에서 검증한 것과 같다: 업로드 -> arm -> 2초 대기 -> 미션 시작.
# 텔레메트리는 백그라운드 태스크가 구독해 snapshot 을 갱신하고, 조회 메서드는 snapshot 만 읽는다.

import asyncio
import logging
from dataclasses import dataclass

from mavsdk_grpc import System
from mavsdk_grpc.mission import MissionItem, MissionPlan

from ..geo import distance_m
from .base import MotionDriver

log = logging.getLogger(__name__)

TELEMETRY_RATE_HZ = 1.0
ARM_SETTLE_S = 2.0   # arm 직후 곧바로 start_mission 하면 DENIED
ACCEPT_RADIUS_M = 10.0                  # 웨이포인트 도착 반경. 2m 는 지나쳐 버린다
MIN_WAYPOINT_GAP_M = 2 * ACCEPT_RADIUS_M


@dataclass
class Snapshot:
    lat: float = float("nan")
    lon: float = float("nan")
    battery_pct: float = float("nan")   # MAVSDK remaining_percent 원값 그대로
    flight_mode: str = "UNKNOWN"


class PX4Driver(MotionDriver):
    def __init__(
        self,
        address: str = "udpin://0.0.0.0:14540",
        speed_mps: float = 3.0,
        alt_m: float = 2.0,
    ):
        self.address = address
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
        ]

    async def _watch_position(self) -> None:
        try:
            async for p in self._drone.telemetry.position():
                self.snapshot.lat = p.latitude_deg
                self.snapshot.lon = p.longitude_deg
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

    async def goto(self, waypoints: list[tuple[float, float]]) -> bool:
        if not waypoints:
            return False

        if self._progress_task:
            self._progress_task.cancel()
            self._progress_task = None
        self._progress = (0, len(waypoints))

        waypoints = self._thin(waypoints)
        self._progress = (0, len(waypoints))
        plan = MissionPlan([self._waypoint(lat, lon) for lat, lon in waypoints])
        try:
            await self._drone.mission.upload_mission(plan)
            log.info("미션 업로드 성공 (%d 개)", len(waypoints))
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

    def _waypoint(self, lat: float, lon: float) -> MissionItem:
        nan = float("nan")
        return MissionItem(
            lat, lon, self.alt_m,
            self.speed_mps,
            False,                 # is_fly_through
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