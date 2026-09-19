import asyncio
import mavsdk_grpc
from mavsdk_grpc import System
from mavsdk_grpc.mission import MissionItem, MissionPlan

HOME_LAT, HOME_LON = 47.397971, 8.546164   # SITL 기본 홈


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    print("연결됨")

    # 1) arm
    try:
        await drone.action.arm()
        print("arm 성공")
        await asyncio.sleep(2)              # ← 추가
    except Exception as e:
        print(f"arm 실패: {e}")
        return

    # 2) 미션 업로드 — 약 50m, 100m 북동쪽
    def wp(lat, lon):
        return MissionItem(
            lat, lon, 2.0,
            3.0,                  # speed m/s
            True,                 # is_fly_through
            float("nan"), float("nan"),
            MissionItem.CameraAction.NONE,
            float("nan"), float("nan"),
            2.0, float("nan"), float("nan"),
            MissionItem.VehicleAction.NONE,
        )

    items = [
        wp(HOME_LAT + 0.00045, HOME_LON + 0.00045),
        wp(HOME_LAT + 0.00090, HOME_LON + 0.00000),
    ]

    try:
        await drone.mission.upload_mission(MissionPlan(items))
        print("미션 업로드 성공")
    except Exception as e:
        print(f"미션 업로드 실패: {e}")
        return

    try:
        await drone.mission.start_mission()
        print("미션 시작")
    except Exception as e:
        print(f"미션 시작 실패: {e}")
        return

    # 3) 진행 추적
    async for prog in drone.mission.mission_progress():
        print(f"진행 {prog.current}/{prog.total}")
        if prog.current == prog.total:
            break


asyncio.run(main())