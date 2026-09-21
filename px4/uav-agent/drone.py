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


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class DroneProvider(ABC):
    @abstractmethod
    async def get_state(self, uav_id: str) -> dict: ...

    @abstractmethod
    async def goto(self, lat: float, lon: float, alt_amsl: float) -> None: ...

    @abstractmethod
    async def land(self) -> None: ...


class MockDrone(DroneProvider):
    """PX4/Gazebo 없이 고정값을 반환한다.

    테스트 편의를 위해 배터리·풍속·health 를 런타임에 바꿀 수 있다.
    """

    def __init__(self) -> None:
        # 원통119 근처 (NOTE.md 4절)
        self.lat = 38.1205
        self.lon = 128.2018
        self.alt_amsl = 235.0
        self.battery_pct = 95.0   # 기본 시나리오에 여유를 두기 위한 값
        self.wind_ms = 3.1
        self.failsafe = False
        self.gps_ok = True
        self.sensors_ok = True
        self.link_quality = "OK"
        self.current_task_id = None

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
            "velocity_ms": 0.0,
            "flight_mode": "HOLD",
            "armed": False,
            "health": {
                "gps_ok": self.gps_ok,
                "sensors_ok": self.sensors_ok,
                "failsafe": self.failsafe,
            },
            "wind_ms": self.wind_ms,
            "link_quality": self.link_quality,
            "current_task_id": self.current_task_id,
        }

    async def goto(self, lat: float, lon: float, alt_amsl: float) -> None:
        self.lat, self.lon, self.alt_amsl = lat, lon, alt_amsl

    async def land(self) -> None:
        pass


class Px4Drone(DroneProvider):
    """MAVSDK 실연동.

    주의:
      - PX4 SITL 은 telemetry.wind 를 발행하지 않는다(실측 확인). wind_ms 는 None 으로
        반환하며, 판단에 쓸 풍속은 요청(EvaluateRequest.wind_ms)으로 주입받는다.
      - 2 vCPU 환경에서 PX4 가 CPU 고갈로 MAVLink 송신을 멈추는 사례를 확인했다.
        모든 조회에 타임아웃을 두고, 실패 시 연결을 버려 다음 호출에서 재연결한다.
    """

    def __init__(self, address: str = "udpin://0.0.0.0:14540",
                 timeout_s: float = 8.0) -> None:
        self.address = address
        self.timeout_s = timeout_s
        self._drone = None

    async def _ensure(self):
        if self._drone is None:
            from mavsdk import System

            drone = System()
            await drone.connect(system_address=self.address)
            async with asyncio.timeout(self.timeout_s * 2):
                async for s in drone.core.connection_state():
                    if s.is_connected:
                        break
            self._drone = drone
        return self._drone

    async def _first(self, stream, default=None):
        """스트림의 첫 값을 타임아웃과 함께 읽는다."""
        try:
            async with asyncio.timeout(self.timeout_s):
                async for v in stream():
                    return v
        except (TimeoutError, Exception):
            return default
        return default

    async def get_state(self, uav_id: str) -> dict:
        try:
            drone = await self._ensure()
        except Exception as e:
            self._drone = None
            raise RuntimeError(f"PX4 연결 실패: {e}") from e

        pos = await self._first(drone.telemetry.position)
        bat = await self._first(drone.telemetry.battery)
        health = await self._first(drone.telemetry.health)
        mode = await self._first(drone.telemetry.flight_mode)
        armed = await self._first(drone.telemetry.armed, default=False)
        vel = await self._first(drone.telemetry.velocity_ned)

        if pos is None or bat is None or health is None:
            # PX4 가 응답을 멈춘 상태. 연결을 버리고 다음 호출에서 재연결한다.
            self._drone = None
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
            "flight_mode": str(mode) if mode is not None else "UNKNOWN",
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

    async def goto(self, lat: float, lon: float, alt_amsl: float) -> None:
        drone = await self._ensure()
        await drone.action.arm()
        await drone.action.takeoff()
        await drone.action.goto_location(lat, lon, alt_amsl, 0)

    async def land(self) -> None:
        drone = await self._ensure()
        await drone.action.land()


def get_provider() -> DroneProvider:
    """UAV 1대 = 프로세스 1개. 여러 대를 띄울 때는 PX4_ADDRESS 로 각자의 PX4 를 가리킨다.

    한 서버에 PX4 를 2개 올리면 CPU 가 부족하고(1 인스턴스당 약 1.1 코어) 같은 UDP
    포트를 두 프로세스가 물어 텔레메트리가 튄다. 드론이 여러 대면 서버도 나눈다.
    """
    mode = os.getenv("UAV_MODE", "mock").lower()
    if mode != "real":
        return MockDrone()
    return Px4Drone(address=os.getenv("PX4_ADDRESS", "udpin://0.0.0.0:14540"))
