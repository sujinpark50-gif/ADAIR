"""UAV 상태 제공자.

MockDrone 과 Px4Drone 이 같은 인터페이스를 구현한다.
환경변수 UAV_MODE=mock|real 로 교체하며, 호출측 코드는 바뀌지 않는다.
"""
import asyncio
import math
import os
from abc import ABC, abstractmethod
from datetime import datetime, timezone

import config
from evaluator import haversine_m


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class DroneProvider(ABC):
    """임무 한 번 = goto → hold → return_home. 모두 끝날 때까지 기다렸다가 반환한다.

    취소(asyncio.CancelledError)되면 그 자리에서 멈춘다. 비행 중 배터리 감시가
    임무를 끊고 return_home 을 부를 때 쓴다(main._run_task).
    """

    @abstractmethod
    async def get_state(self, uav_id: str) -> dict: ...

    @abstractmethod
    async def home(self) -> tuple[float, float, float]:
        """기지 (lat, lon, 지면 해발고도)."""

    @abstractmethod
    async def goto(self, lat: float, lon: float, alt_amsl: float) -> None:
        """목표 도착까지 기다린다."""

    @abstractmethod
    async def hold(self, seconds: float) -> None:
        """제자리 체류 (관측)."""

    @abstractmethod
    async def return_home(self) -> None:
        """현재 고도로 기지 상공까지 간 뒤 착륙한다."""


class MockDrone(DroneProvider):
    """PX4/Gazebo 없이 비행을 흉내낸다.

    evaluator 와 같은 모델로 움직인다 — 상승(CLIMB_SPEED) 후 수평(CRUISE_SPEED),
    초당 BATTERY_DRAIN_PCT_S 소모. MOCK_TIME_SCALE 배속으로 진행한다.
    테스트 편의를 위해 배터리·풍속·health 를 런타임에 바꿀 수 있다(/mock/set).
    """

    def __init__(self, home: tuple[float, float, float]) -> None:
        self.home_pos = home
        self.lat, self.lon, self.alt_amsl = home
        self.battery_pct = 95.0   # 기본 시나리오에 여유를 두기 위한 값
        self.wind_ms = 3.1
        self.failsafe = False
        self.gps_ok = True
        self.sensors_ok = True
        self.link_quality = "OK"
        self.current_task_id = None
        self.armed = False
        self.flight_mode = "HOLD"
        self.velocity_ms = 0.0

    async def get_state(self, uav_id: str) -> dict:
        return {
            "uav_id": uav_id,
            "timestamp": _now(),
            "position": {
                "lat": self.lat,
                "lon": self.lon,
                "alt_m_amsl": self.alt_amsl,
                "alt_m_agl": 0.0,
            },
            "battery": {"percent": self.battery_pct, "voltage": 15.8},
            "velocity_ms": self.velocity_ms,
            "flight_mode": self.flight_mode,
            "armed": self.armed,
            "health": {
                "gps_ok": self.gps_ok,
                "sensors_ok": self.sensors_ok,
                "failsafe": self.failsafe,
            },
            "wind_ms": self.wind_ms,
            "link_quality": self.link_quality,
            "current_task_id": self.current_task_id,
        }

    async def home(self) -> tuple[float, float, float]:
        return self.home_pos

    async def _run(self, sim_s: float, step) -> None:
        """sim_s 초(시뮬레이션 시간)를 배속으로 흘리며 step(진행률)과 배터리를 갱신한다."""
        elapsed = 0.0
        while elapsed < sim_s:
            dt = min(0.05 * config.MOCK_TIME_SCALE, sim_s - elapsed)
            await asyncio.sleep(dt / config.MOCK_TIME_SCALE)
            elapsed += dt
            self.battery_pct -= dt * config.BATTERY_DRAIN_PCT_S
            step(elapsed / sim_s)

    async def _vertical(self, alt_amsl: float) -> None:
        z0 = self.alt_amsl
        await self._run(abs(alt_amsl - z0) / config.CLIMB_SPEED_MS,
                        lambda f: setattr(self, "alt_amsl", z0 + (alt_amsl - z0) * f))

    async def goto(self, lat: float, lon: float, alt_amsl: float) -> None:
        self.armed, self.flight_mode = True, "GOTO"
        try:
            await self._vertical(alt_amsl)
            lat0, lon0 = self.lat, self.lon

            def move(f):
                self.lat = lat0 + (lat - lat0) * f
                self.lon = lon0 + (lon - lon0) * f

            self.velocity_ms = config.CRUISE_SPEED_MS
            await self._run(haversine_m(lat0, lon0, lat, lon) / config.CRUISE_SPEED_MS, move)
        finally:
            self.velocity_ms, self.flight_mode = 0.0, "HOLD"

    async def hold(self, seconds: float) -> None:
        await self._run(seconds, lambda f: None)

    async def return_home(self) -> None:
        await self.goto(self.home_pos[0], self.home_pos[1], self.alt_amsl)
        self.flight_mode = "LAND"
        await self._vertical(self.home_pos[2])
        self.armed, self.flight_mode = False, "HOLD"
        # Mock 가정: 기지에 내리면 배터리를 교체한다. 없으면 한 번 출동한 기체가
        # 다음 임무를 계속 거절해 연속 시나리오를 재현할 수 없다.
        self.battery_pct = 100.0


class Px4Drone(DroneProvider):
    """MAVSDK 4.x(네이티브 바인딩) 실연동.

    주의:
      - mavsdk>=4 전용이다. 3.x(gRPC) 의 `System().connect()` / `drone.telemetry.*` 는
        4.x 에서 동작하지 않는다(`Mavsdk` + `TelemetryAsync(system)` 방식).
      - battery.remaining_percent 는 4.x 에서 0~100 이다(PX4 원값 0~1 을 ×100, 실측 확인).
        원값 그대로 전달하며 여기서 변환하지 않는다.
      - PX4 SITL 은 telemetry.wind 를 발행하지 않는다(실측 확인). wind_ms 는 None 으로
        반환하며, 판단에 쓸 풍속은 요청(EvaluateRequest.wind_ms)으로 주입받는다.
      - 2 vCPU 환경에서 PX4 가 CPU 고갈로 MAVLink 송신을 멈추는 사례를 확인했다.
        모든 조회에 타임아웃을 두고, 실패 시 연결을 버려 다음 호출에서 재연결한다.
    """

    def __init__(self, address: str = "udpin://0.0.0.0:14540",
                 timeout_s: float = 8.0) -> None:
        self.address = address
        self.timeout_s = timeout_s
        self._sdk = None
        self._tel = None
        self._act = None
        self._param = None

    async def _ensure(self):
        if self._tel is None:
            from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
            from mavsdk.asyncio.plugins.action import ActionAsync
            from mavsdk.asyncio.plugins.param import ParamAsync
            from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

            sdk = Mavsdk(Configuration.create_with_component_type(
                ComponentType.GROUND_STATION))
            try:
                await sdk.add_any_connection(self.address)
                system = await sdk.first_autopilot(timeout_s=self.timeout_s * 2)
                if system is None:
                    raise RuntimeError(f"{self.address} 에서 PX4 를 찾지 못함")
            except Exception:
                sdk.destroy()
                raise
            self._sdk = sdk
            self._tel = TelemetryAsync(system)
            self._act = ActionAsync(system)
            self._param = ParamAsync(system)
        return self._tel, self._act

    def _reset(self) -> None:
        """연결을 버린다. 다음 호출에서 재연결한다."""
        if self._sdk is not None:
            try:
                self._sdk.destroy()
            except Exception:  # noqa: BLE001
                pass
        self._sdk = self._tel = self._act = self._param = None

    async def _first(self, stream, default=None):
        """구독 스트림의 첫 값을 타임아웃과 함께 읽고 구독을 해제한다."""
        gen = stream()
        try:
            async with asyncio.timeout(self.timeout_s):
                return await anext(gen)
        except (TimeoutError, Exception):
            return default
        finally:
            await gen.aclose()   # 4.x 는 aclose 시점에 unsubscribe 한다

    async def get_state(self, uav_id: str) -> dict:
        try:
            tel, _ = await self._ensure()
        except Exception as e:
            self._reset()
            raise RuntimeError(f"PX4 연결 실패: {e}") from e

        pos = await self._first(tel.subscribe_position)
        bat = await self._first(tel.subscribe_battery)
        health = await self._first(tel.subscribe_health)
        mode = await self._first(tel.subscribe_flight_mode)
        armed = await self._first(tel.subscribe_armed, default=False)
        vel = await self._first(tel.subscribe_velocity_ned)

        if pos is None or bat is None or health is None:
            # PX4 가 응답을 멈춘 상태. 연결을 버리고 다음 호출에서 재연결한다.
            self._reset()
            raise RuntimeError("PX4 텔레메트리 타임아웃 (송신 중단 가능성)")

        sensors_ok = bool(
            health.is_gyrometer_calibration_ok
            and health.is_accelerometer_calibration_ok
            and health.is_magnetometer_calibration_ok
        )
        speed = (
            math.hypot(vel.north_m_s, vel.east_m_s) if vel is not None else 0.0
        )

        return {
            "uav_id": uav_id,
            "timestamp": _now(),
            "position": {
                "lat": pos.latitude_deg,
                "lon": pos.longitude_deg,
                "alt_m_amsl": pos.absolute_altitude_m,
                "alt_m_agl": pos.relative_altitude_m,
            },
            "battery": {
                "percent": bat.remaining_percent,
                "voltage": bat.voltage_v,
            },
            "velocity_ms": round(speed, 2),
            # 4.x FlightMode 는 IntEnum 이라 str() 이 "3" 이 된다. 이름("HOLD")으로 보낸다.
            "flight_mode": mode.name if mode is not None else "UNKNOWN",
            "armed": bool(armed),
            "health": {
                "gps_ok": bool(health.is_global_position_ok),
                "sensors_ok": sensors_ok,
                # PX4 failsafe 플래그는 telemetry.health 에 없다. 별도 확인 필요(TBD).
                "failsafe": False,
            },
            # PX4 SITL 은 풍속을 발행하지 않는다. 판단 시 요청값으로 주입받는다.
            "wind_ms": None,
            "link_quality": "OK",
            "current_task_id": None,
        }

    async def home(self) -> tuple[float, float, float]:
        tel, _ = await self._ensure()
        h = await self._first(tel.subscribe_home)
        if h is None:
            raise RuntimeError("PX4 home 위치를 읽지 못함")
        return (h.latitude_deg, h.longitude_deg, h.absolute_altitude_m)

    async def _wait(self, done, timeout_s: float, what: str) -> None:
        """done() 이 참이 될 때까지 1초마다 확인한다. PX4 명령은 도착을 기다리지 않는다."""
        try:
            async with asyncio.timeout(timeout_s):
                while not await done():
                    await asyncio.sleep(1.0)
        except TimeoutError:
            raise RuntimeError(f"{what} 대기 시간 초과 ({timeout_s:.0f} s)") from None

    async def _in_air(self) -> bool:
        tel, _ = await self._ensure()
        return bool(await self._first(tel.subscribe_in_air, default=False))

    async def goto(self, lat: float, lon: float, alt_amsl: float) -> None:
        tel, act = await self._ensure()
        # goto_location 은 속도를 받지 않아 PX4 가 MPC_XY_CRUISE(기본 5 m/s)로 난다.
        # evaluator 의 ETA·배터리 계산과 같은 속도로 날도록 맞춘다.
        await self._param.set_param_float("MPC_XY_CRUISE", config.CRUISE_SPEED_MS)
        if not await self._in_air():
            await act.arm()
            await act.takeoff()
            await self._wait(self._in_air, 60, "이륙")

        pos = await self._first(tel.subscribe_position)
        if pos is None:
            raise RuntimeError("PX4 위치를 읽지 못함")
        eta = (haversine_m(pos.latitude_deg, pos.longitude_deg, lat, lon) / config.CRUISE_SPEED_MS
               + abs(alt_amsl - pos.absolute_altitude_m) / config.CLIMB_SPEED_MS)
        await act.goto_location(lat, lon, alt_amsl, 0)

        async def arrived() -> bool:
            p = await self._first(tel.subscribe_position)
            return (p is not None
                    and haversine_m(p.latitude_deg, p.longitude_deg, lat, lon)
                    <= config.ARRIVAL_TOLERANCE_M
                    and abs(p.absolute_altitude_m - alt_amsl) <= config.ARRIVAL_TOLERANCE_M)

        await self._wait(arrived, eta * 2 + 60, "목표 도착")

    async def hold(self, seconds: float) -> None:
        # goto_location 목표점에서 PX4 가 스스로 제자리 비행한다.
        await asyncio.sleep(seconds)

    async def return_home(self) -> None:
        # PX4 RTL 은 지형을 모르고 RTL_RETURN_ALT 로 내려와 능선에 닿을 수 있다.
        # 지금 고도(목표 상공 비행고도)를 유지해 기지 상공까지 간 뒤 착륙한다.
        tel, act = await self._ensure()
        h_lat, h_lon, h_alt = await self.home()
        pos = await self._first(tel.subscribe_position)
        if pos is None:
            raise RuntimeError("PX4 위치를 읽지 못함")
        await self.goto(h_lat, h_lon, pos.absolute_altitude_m)
        await act.land()

        async def landed() -> bool:
            return not await self._in_air()

        # 착륙은 MPC_Z_VEL_MAX_DN(1.5 m/s)보다 느리게 내려온다. 넉넉히 잡는다.
        await self._wait(landed, (pos.absolute_altitude_m - h_alt) / 0.5 + 120, "착륙")


def get_provider(uav_id: str) -> DroneProvider:
    """UAV 1대 = 프로세스 1개. 여러 대를 띄울 때는 PX4_ADDRESS 로 각자의 PX4 를 가리킨다.

    한 서버에 PX4 를 2개 올리면 CPU 가 부족하고(1 인스턴스당 약 1.1 코어) 같은 UDP
    포트를 두 프로세스가 물어 텔레메트리가 튄다. 드론이 여러 대면 서버도 나눈다.
    """
    mode = os.getenv("UAV_MODE", "mock").lower()
    if mode != "real":
        base = uav_id[:1].upper()
        if base not in config.HOME_BASES:
            raise ValueError(f"UAV_ID {uav_id} 의 소속 기지를 알 수 없다 "
                             f"(앞글자 {list(config.HOME_BASES)} 중 하나여야 함)")
        return MockDrone(config.HOME_BASES[base])
    return Px4Drone(address=os.getenv("PX4_ADDRESS", "udpin://0.0.0.0:14540"))
