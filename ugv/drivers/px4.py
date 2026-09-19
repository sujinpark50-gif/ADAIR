"""PX4 SITL 연결 확인. 무엇이 읽히는지 눈으로 보는 용도."""
import asyncio
from mavsdk import System


async def main():
    drone = System()
    print("연결 시도: udp://:14540")
    await drone.connect(system_address="udpin://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("연결됨")
            break

    async for p in drone.telemetry.position():
        print(f"position   lat={p.latitude_deg:.6f} lon={p.longitude_deg:.6f}")
        break

    async for b in drone.telemetry.battery():
        print(f"battery    {b.remaining_percent}")
        break

    async for h in drone.telemetry.health():
        print(f"health     {h}")
        break

    async for a in drone.telemetry.armed():
        print(f"armed      {a}")
        break

    async for o in drone.telemetry.odometry():
        print(f"odometry   x={o.position_body.x_m:.2f} y={o.position_body.y_m:.2f}")
        break


asyncio.run(main())