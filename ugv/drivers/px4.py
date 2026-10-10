# -*- coding: utf-8 -*-
"""ugv/drivers/px4.py — PX4 SITL(adair_ugv, Gazebo) 드라이버. mavsdk 4.x (네이티브 바인딩, UAV 와 같은 라이브러리).

주행 = PX4 Mission 모드. 미션 지점마다 구간 속도(speed_m_s)를 싣는다. 경로 추종(pure pursuit)·꺾임 감속은 PX4 가 한다.
- 속도 의미 변환 (mavsdk_plan): 공통 Item 속도는 '그 지점까지 가는 구간' 속도인데, MAVSDK/PX4 는 지점의 speed_m_s 를
  '그 지점을 지난 뒤' 쓴다 (2026-10-09 평탄 도로 월드 실측: 구간 k-1→k 실제 속도가 지점 k-1 속도와 평균 0.06 m/s 차,
  지점 k 속도와 2.9 m/s 차). 그대로 실으면 감속이 한 구간(최대 40 m) 늦어 곡선에 빠르게 들어간다
  (계획 8.7 m/s 곡선을 11.3 m/s, 횡가속 4.75 m/s²) → 한 칸 당겨 싣고, 첫 구간 속도를 정하려고 앞쪽에 선행 지점을 하나 넣는다.
- follow(): 미션을 통째로 교체 → (시동) → 시작. 업로드 직후 PX4 가 검증을 끝내기 전에는 시작을 거부하므로 재시도한다.
- stop(): 진행 방향 앞 제동거리 지점 하나짜리 미션으로 교체 (base.Driver.stop). HOLD 는 명령 시점 위치로 U턴한다.
- 정차 유지 = 시동 끄기(disarm): 미션이 끝나 멈추면(0.3 m/s 미만 PARK_AFTER_S) 시동을 끈다. 이 rover 는 미션 끝 '대기(loiter)'
  에서 조금 지나친 마지막 지점으로 돌아가려다 다시 달리거나 맴돌았다 (2026-10-10 Gazebo: 정차 5 s 뒤 스스로 출발, 26분 6 km).
  시동이 꺼지면 바퀴 속도 명령이 0 이라 그 자리에 선다. 다음 미션(follow)에서 다시 시동을 건다.
- 텔레메트리는 구독을 계속 열어 두고 마지막 값을 보관한다. 위치가 끊기면 Telemetry.age_s() 가 커진다 (감시는 vehicle.py).
검증 (2026-10-09, 평지 월드): 직선 60.5 km/h, 정지 57 m, 90°·45°·135° 꺾임 17.9/36.1/17.7 km/h. ugv/tools/drive_test.py
"""

import asyncio
import math
import os
import time
from typing import List, Optional

from ..geo import FRAME, GZ_FRAME
from .base import Driver, DriverError, Item, Progress, Telemetry

START_RETRIES = 20
PARK_AFTER_S = 1.0          # 미션 끝 + 0.3 m/s 미만이 이만큼 이어지면 시동을 꺼 정차 유지
LEAD_BASE_M = 5.0           # 선행 지점 = 지금 위치에서 경로를 따라 LEAD_BASE_M + 속도 × LEAD_TIME_S 앞
LEAD_TIME_S = 2.0           #   (업로드·시작에 걸리는 동안 지나쳐 뒤에 남지 않게. 뒤에 남으면 rover 가 되돌아간다)
ACCEPT_M = float(os.getenv("UGV_PX4_ACCEPT_M", "3.0"))   # 미션 지점 도착 반경 (airframe NAV_ACC_RAD 와 맞춘다. 환경변수는 시험용)
START_RETRY_S = 0.5
ARM_RETRIES = 15
ARM_RETRY_S = 1.0


def mavsdk_plan(items: List[Item], here, v_now: float):
    """공통 Item[(x, y, 그 지점까지 구간 속도)] → MAVSDK 미션 [(x, y, 그 지점을 지난 뒤 속도, 통과 여부)].

    지점 k 에 지점 k+1 의 구간 속도를 싣는다 (마지막 지점은 정지점이라 지난 뒤 속도가 없다 — 자기 값).
    첫 구간(지금 위치 → 첫 지점)의 속도는 실을 곳이 없으므로, 경로를 따라 조금 앞에 선행 지점을 넣고 거기에
    첫 지점의 구간 속도를 싣는다. 첫 지점이 선행 거리보다 가까우면 넣지 않는다 (짧은 첫 구간은 이전 속도 그대로)."""
    if not items:
        return []
    out = []
    lead = LEAD_BASE_M + max(v_now, 0.0) * LEAD_TIME_S
    d0 = math.dist(here, items[0][:2])
    if d0 > lead + ACCEPT_M:
        t = lead / d0
        out.append((here[0] + (items[0][0] - here[0]) * t, here[1] + (items[0][1] - here[1]) * t, items[0][2], True))
    for k, (x, y, v) in enumerate(items):
        last = k == len(items) - 1
        out.append((x, y, v if last else items[k + 1][2], not last))
    return out


def _mission_item(lat, lon, speed, through):
    from mavsdk.plugins.mission.mission import MissionItem
    return MissionItem(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=0.0, speed_m_s=speed,
                       is_fly_through=through, gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
                       camera_action=MissionItem.CameraAction.NONE, loiter_time_s=0.0,
                       camera_photo_interval_s=0.0, acceptance_radius_m=ACCEPT_M, yaw_deg=float("nan"),
                       camera_photo_distance_m=0.0, vehicle_action=MissionItem.VehicleAction.NONE)


class Px4Driver(Driver):
    kind = "px4"

    def __init__(self, address: str, clock, connect_timeout_s: float = 30.0, decel_mps2=None, gazebo: bool = False):
        self.address, self.clock, self.connect_timeout_s = address, clock, connect_timeout_s
        # Gazebo SITL 이면 Gazebo 와 같은 위경도 변환 (geo.GzEnuFrame — 도로 월드 좌표와 PX4 위치를 맞춘다). 실차는 FRAME
        self.frame = GZ_FRAME if gazebo else FRAME
        self.decel_mps2 = decel_mps2
        self._sdk = self._tel = self._act = self._mis = None
        self._pos = self._vel = None
        self._heading = 0.0
        self._armed = False
        self._mode = "UNKNOWN"
        self._pos_mono = 0.0
        self._prog = Progress()
        self._expected_total = 0
        self._seen_running = False
        self._tasks: List[asyncio.Task] = []
        self._cmd = asyncio.Lock()
        self.connected = False
        self.last_error: Optional[str] = None
        self.parked_seq = None                  # 정차 유지(시동 끔)한 미션 번호

    async def start(self):
        from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
        from mavsdk.asyncio.plugins.action import ActionAsync
        from mavsdk.asyncio.plugins.mission import MissionAsync
        from mavsdk.asyncio.plugins.telemetry import TelemetryAsync
        sdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
        await sdk.add_any_connection(self.address)
        system = await sdk.first_autopilot(timeout_s=self.connect_timeout_s)
        if system is None:
            sdk.destroy()
            raise DriverError(f"PX4_NOT_FOUND: {self.address} (ugv/tools/px4-start.sh, 방화벽 확인)")
        self._sdk, self._tel = sdk, TelemetryAsync(system)
        self._act, self._mis = ActionAsync(system), MissionAsync(system)
        await self._mis.set_return_to_launch_after_mission(False)
        for name, stream in (("position", self._tel.subscribe_position), ("velocity", self._tel.subscribe_velocity_ned),
                             ("heading", self._tel.subscribe_heading), ("armed", self._tel.subscribe_armed),
                             ("mode", self._tel.subscribe_flight_mode),
                             ("progress", self._mis.subscribe_mission_progress)):
            self._tasks.append(asyncio.create_task(self._pump(name, stream)))
        for _ in range(100):
            if self._pos is not None:
                break
            await asyncio.sleep(0.1)
        self.connected = self._pos is not None
        if not self.connected:
            raise DriverError("PX4_NO_POSITION")
        self._tasks.append(asyncio.create_task(self._park_watch()))

    async def test_link(self, cut: bool) -> dict:
        """시험용 (UGV_TEST_HOOKS=1): 이 차량의 MAVLink 연결만 끊거나 잇는다. 끊으면 이 차량의 MAVSDK 인스턴스를 닫아 서버는 이 차에
        아무것도 보내지 못하고(하트비트 포함) PX4 는 지상국 연결 끊김(COM_DL_LOSS_T)으로 판단한다 — PX4 안의 MAVLink 모듈은 그대로라
        실제 무선 단절과 같다. 마지막 텔레메트리는 그대로 남아 낡아 간다 (서버는 TELEMETRY_LOST 로 판단).
        (remove_connection 만으로는 하트비트가 계속 나가 PX4 가 끊김을 몰랐다. PX4 MAVLink 인스턴스를 멈추는 방법은 dataman 읽기
        시간 초과로 미션 추종이 흔들려 결과를 바꿨다, 2026-10-10)"""
        if cut and self._sdk is not None:
            await self.close()
        elif not cut and self._sdk is None:
            await self.start()
        return {"linked": self._sdk is not None}

    async def _park_watch(self):
        """미션이 끝나고 멈춘 차의 시동을 끈다 (정차 유지). 다음 follow 가 다시 시동을 건다."""
        still_since = None
        while True:
            await asyncio.sleep(0.5)
            try:
                tel = self.telemetry()
                prog = self._prog
                if tel is None or not self._armed or not prog.finished or self.parked_seq == prog.mission_seq:
                    still_since = None
                    continue
                if tel.speed_mps >= 0.3:
                    still_since = None
                    continue
                import time
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since < PARK_AFTER_S:
                    continue
                async with self._cmd:
                    if self._prog.mission_seq != prog.mission_seq or not self._prog.finished:
                        continue
                    await self._act.disarm()
                    self.parked_seq = prog.mission_seq
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001
                self.last_error = f"park: {type(e).__name__}: {e}"

    async def _pump(self, name, stream):
        import time
        while True:
            try:
                async for v in stream():
                    if name == "position":
                        self._pos, self._pos_mono = v, time.monotonic()
                    elif name == "velocity":
                        self._vel = v
                    elif name == "heading":
                        self._heading = v.heading_deg
                    elif name == "armed":
                        self._armed = bool(v)
                    elif name == "mode":
                        from mavsdk.plugins.telemetry.telemetry import FlightMode
                        try:
                            self._mode = FlightMode(int(v)).name
                        except (ValueError, TypeError):
                            self._mode = getattr(v, "name", str(v))
                    elif name == "progress":
                        # 새 미션을 보낸 뒤 '진행 중'(current < total)을 한 번 본 다음에만 '끝남'을 받아들인다
                        # (지난 미션의 끝남 보고가 늦게 와도 새 미션 완료로 오해하지 않게)
                        if v.total != self._expected_total or v.total == 0:
                            continue
                        if v.current < v.total:
                            self._seen_running = True
                        fin = self._seen_running and v.current >= v.total
                        self._prog = Progress(v.current, v.total, fin, self._prog.mission_seq)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 — 구독이 끊기면 다시 연다
                self.last_error = f"{name}: {type(e).__name__}: {e}"
                await asyncio.sleep(1.0)

    async def close(self):
        for t in self._tasks:
            t.cancel()
        self._tasks = []
        if self._sdk is not None:
            self._sdk.destroy()
            self._sdk = None

    def telemetry(self) -> Optional[Telemetry]:
        if self._pos is None:
            return None
        import time
        x, y = self.frame.to_xy(self._pos.latitude_deg, self._pos.longitude_deg)
        v = math.hypot(self._vel.north_m_s, self._vel.east_m_s) if self._vel is not None else 0.0
        return Telemetry(x, y, self._pos.latitude_deg, self._pos.longitude_deg, v, self._heading, self._armed,
                         self._mode, self._pos_mono, self._pos.absolute_altitude_m)

    def progress(self) -> Progress:
        return self._prog

    async def _ensure_armed(self):
        for k in range(ARM_RETRIES):
            if self._armed:
                return
            try:
                await self._act.arm()
                return
            except Exception as e:  # noqa: BLE001 — EKF 안정 전에는 거절
                self.last_error = f"arm: {e}"
                await asyncio.sleep(ARM_RETRY_S)
        raise DriverError(f"ARM_FAILED: {self.last_error}")

    async def follow(self, items: List[Item]) -> int:
        if not items:
            raise DriverError("EMPTY_MISSION")
        from mavsdk.plugins.mission.mission import MissionPlan
        tel = self.telemetry()
        here = (tel.x, tel.y) if tel is not None else items[0][:2]
        plan = []
        sent = mavsdk_plan(items, here, tel.speed_mps if tel is not None else 0.0)
        self.last_sent = {"wall": time.time(), "here": [round(here[0], 2), round(here[1], 2)],
                          "items": [[round(x, 2), round(y, 2), round(v, 2), th] for x, y, v, th in sent]}
        for x, y, v, through in sent:
            lat, lon = self.frame.to_ll(x, y)
            plan.append(_mission_item(lat, lon, float(v), through=through))
        async with self._cmd:
            seq = self._prog.mission_seq + 1
            self._expected_total = len(plan)
            self._seen_running = False
            self._prog = Progress(0, len(plan), False, seq)
            try:
                await self._mis.upload_mission(MissionPlan(plan))
                await self._ensure_armed()
                for k in range(START_RETRIES):
                    try:
                        await self._mis.start_mission()
                        break
                    except Exception as e:  # noqa: BLE001
                        if k == START_RETRIES - 1:
                            raise
                        await asyncio.sleep(START_RETRY_S)
            except DriverError:
                raise
            except Exception as e:  # noqa: BLE001
                self.last_error = f"follow: {type(e).__name__}: {e}"
                raise DriverError(f"MISSION_START_FAILED: {e}") from e
            return seq

    async def halt_in_place(self) -> int:
        """이미 거의 멈춘 차의 '제자리 정지'. MAVLink HOLD(action.hold) 는 보내지 않는다:
        이 rover(ackermann)에서 HOLD 는 정지가 아니라 주행을 일으켰다 — 2026-10-10 Gazebo 에서 정차 중인 차에 HOLD 를 보내자
        3.8 m/s 로 26분 동안 약 6 km 를 달렸다 (통신 단절 정지 처리가 정차 차량에 정지 명령을 보낸 경우).
        미션이 끝나 멈춰 있으면 아무것도 보내지 않는다 (미션 끝 정차 상태 유지). 아직 미션을 따라 움직이는 중이면
        지금 위치 한 점짜리 미션으로 바꿔 그 자리에서 끝나게 한다."""
        async with self._cmd:
            seq = self._prog.mission_seq + 1
            tel = self.telemetry()
            prog = self._prog
            if tel is not None and (prog.finished or prog.total == 0) and tel.speed_mps < 0.3:
                self._prog = Progress(prog.current, prog.total, True, seq)
                return seq
        if tel is None:
            raise DriverError("TELEMETRY_UNAVAILABLE")
        return await self.follow([(tel.x, tel.y, 1.0)])
