# -*- coding: utf-8 -*-
"""
demo_fly_to_fire.py  ·  강원 kangwon 월드에서 드론을 '화재 셀'로 실제 비행시키는 데모
=====================================================================================
목적: 이번에 배선한 좌표 체인이 실제 PX4-gz 에서 도는지 눈으로 확인한다.
      화재 셀(격자 x,y) → gz_bridge → 위경도 + 지형고도 → MAVSDK goto_location.

전제 (팀 EC2/WSL, 아래 런북 순서대로):
  1) PX4 SITL + Gazebo 가 kangwon 월드로 떠 있고 (PX4_HOME = datum),
  2) 이 스크립트를 실행하는 환경에 mavsdk 설치 (uav-venv),
  3) 저장소 루트에서 실행 (gz_bridge import 위해).

주의: 이 스크립트는 현재 개발 컨테이너에서 실행/검증되지 않았다(문법만 확인).
      PX4·Gazebo 가 실제로 떠 있는 팀 환경에서 돌려야 한다.

사용:
  python demo_fly_to_fire.py                 # 실 환경에서 화재셀 자동 선택
  python demo_fly_to_fire.py --col 60 --row 70
  python demo_fly_to_fire.py --lat 38.05 --lon 128.20
  python demo_fly_to_fire.py --addr udpin://0.0.0.0:14540 --clearance 60 --no-land
"""

import argparse
import asyncio
import math
import sys

try:
    from mavsdk import System
except ImportError:
    print("mavsdk 가 없습니다. uav-venv 활성화 후:  pip install mavsdk", file=sys.stderr)
    sys.exit(1)

import gz_bridge


def _haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def resolve_target(args):
    """화재 타깃 (lat, lon, alt_amsl, 설명) 을 정한다."""
    # 1) 직접 위경도 지정
    if args.lat is not None and args.lon is not None:
        # 위경도 → 격자로 되짚어 지형고도 산정
        try:
            e, n = gz_bridge.latlon_to_enu(args.lat, args.lon)
            # 근사 격자 인덱스로 지형고도 조회
            x, y = gz_bridge.enu_to_epsg(e, n)
            col = int((x - gz_bridge.GRID_LEFT) / gz_bridge.GRID_RES)
            row = int((gz_bridge.GRID_TOP - y) / gz_bridge.GRID_RES)
            alt = gz_bridge.terrain_elev(col, row) + args.clearance
        except Exception:
            alt = gz_bridge.DATUM_ALT + args.clearance
        return args.lat, args.lon, alt, f"수동 위경도 ({args.lat:.5f},{args.lon:.5f})"

    # 2) 격자 셀 지정
    if args.col is not None and args.row is not None:
        lat, lon = gz_bridge.grid_cell_to_latlon(args.col, args.row)
        alt = gz_bridge.terrain_elev(args.col, args.row) + args.clearance
        return lat, lon, alt, f"격자셀 ({args.col},{args.row})"

    # 3) 실 환경에서 최고위험 화재셀 자동 선택
    try:
        import config
        config.FIRE_CONNECTION_MODE = "integrated"
        from connectors import fire_connector
        es = None
        for t in range(3):
            es = fire_connector.get_environment_state(float(t))
        cell = max(es.fire_cells, key=lambda c: c.get("risk_score", 0.0))
        lat, lon, alt = gz_bridge.fire_cell_to_target(cell, clearance_m=args.clearance)
        return lat, lon, alt, f"최고위험 화재셀 ({cell['x']},{cell['y']})"
    except Exception as e:
        print(f"[경고] 실 환경에서 화재셀 자동선택 실패({e}) → datum 근처 기본 셀 사용")
        lat, lon = gz_bridge.grid_cell_to_latlon(60, 70)
        alt = gz_bridge.terrain_elev(60, 70) + args.clearance
        return lat, lon, alt, "기본 셀 (60,70)"


async def main(args):
    tgt_lat, tgt_lon, tgt_alt, tgt_desc = resolve_target(args)
    print(f"■ 화재 타깃: {tgt_desc}")
    print(f"  → lat={tgt_lat:.6f}, lon={tgt_lon:.6f}, alt(AMSL)={tgt_alt:.1f} m "
          f"(지형+{args.clearance:.0f}m)")

    drone = System()
    print(f"■ PX4 연결 시도: {args.addr}")
    await drone.connect(system_address=args.addr)
    async for st in drone.core.connection_state():
        if st.is_connected:
            print("  연결됨")
            break

    # 홈(=datum 이어야 정합)
    async for h in drone.telemetry.home():
        print(f"■ 홈: lat={h.latitude_deg:.6f}, lon={h.longitude_deg:.6f}, "
              f"alt={h.absolute_altitude_m:.1f}  (datum {gz_bridge.DATUM_LAT},{gz_bridge.DATUM_LON} 와 일치해야 정합)")
        break

    # headless 이륙을 막는 GCS failsafe 완화(시뮬 전용) — 실패해도 진행
    try:
        await drone.param.set_param_int("NAV_RCL_ACT", 0)
        await drone.param.set_param_int("NAV_DLL_ACT", 0)
        print("■ 시뮬 failsafe 파라미터 완화 적용")
    except Exception as e:
        print(f"  (파라미터 설정 생략: {e})")

    # health 대기
    print("■ 위치추정/홈 확정 대기...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    print("■ arm → takeoff")
    await drone.action.set_takeoff_altitude(max(20.0, args.clearance))
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("■ 화재 타깃으로 goto_location")
    await drone.action.goto_location(tgt_lat, tgt_lon, tgt_alt, 0.0)

    # 도착 모니터링
    reached = False
    for _ in range(int(args.timeout)):
        async for pos in drone.telemetry.position():
            d = _haversine_m(pos.latitude_deg, pos.longitude_deg, tgt_lat, tgt_lon)
            dz = abs(pos.absolute_altitude_m - tgt_alt)
            print(f"  거리 {d:7.1f} m | 고도차 {dz:6.1f} m | "
                  f"현재 alt {pos.absolute_altitude_m:6.1f}")
            break
        if d < args.arrive_radius:
            reached = True
            break
        await asyncio.sleep(1)

    if reached:
        print(f"✅ 화재 타깃 도착 (반경 {args.arrive_radius:.0f} m 이내). 관측 지점 도달.")
    else:
        print("⚠️ 제한 시간 내 미도착 — 거리/배터리/지형 고도 확인 필요.")

    if not args.no_land:
        print("■ 복귀(RTL)")
        await drone.action.return_to_launch()
        await asyncio.sleep(5)

    print("■ 데모 종료")


def parse():
    p = argparse.ArgumentParser(description="kangwon 월드에서 드론을 화재셀로 비행")
    p.add_argument("--addr", default="udpin://0.0.0.0:14540", help="PX4 MAVSDK 주소")
    p.add_argument("--col", type=int, help="화재 격자 col(x)")
    p.add_argument("--row", type=int, help="화재 격자 row(y)")
    p.add_argument("--lat", type=float, help="화재 위도(직접)")
    p.add_argument("--lon", type=float, help="화재 경도(직접)")
    p.add_argument("--clearance", type=float, default=50.0, help="지형 위 여유고도(m)")
    p.add_argument("--arrive-radius", type=float, default=80.0, help="도착 판정 반경(m)")
    p.add_argument("--timeout", type=int, default=600, help="도착 대기 최대 초")
    p.add_argument("--no-land", action="store_true", help="도착 후 RTL 생략")
    return p.parse_args()


if __name__ == "__main__":
    asyncio.run(main(parse()))
