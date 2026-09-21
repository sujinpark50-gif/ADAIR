# ugv/demo_ugv.py — 지상자원 재할당 시나리오 시연.
#   python -m ugv.demo_ugv            (SimDriver, PX4 불필요)
#   python -m ugv.demo_ugv --px4      (PX4 SITL 연동)
import logging
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

import argparse
import asyncio

from .fleet import GroundFleet

TARGET = "F1"


def show(fleet: GroundFleet, label: str) -> None:
    print(f"\n[{label}]")
    for rid in ("A-ugv1", "B-ugv1"):
        r = fleet.evaluate(rid, TARGET)
        if r["response"] == "ACCEPT":
            print(f"  {rid:10} ACCEPT  eta={r['eta_s']:7.0f}s  "
                  f"경로={' -> '.join(r['path'])}")
        else:
            print(f"  {rid:10} REJECT  {r['reason']}")


async def main(use_px4: bool) -> None:
    fleet = GroundFleet(use_px4=use_px4)
    if use_px4:
        await fleet.connect_all()

    print("=== 자원 목록 (거점 A) ===")
    for s in fleet.status_by_base("A"):
        print(f"  {s['resource_id']:10} {s['resource_type']:12} "
              f"{s['state']:8} ({s['location_lat']:.4f}, {s['location_lon']:.4f})")

    show(fleet, "평상시")

    fleet.set_road_blocked("E3", True)
    show(fleet, "E3 차단 — 화재 확산으로 북부 진입로 통제")

    fleet.set_road_blocked("E2", True)
    show(fleet, "E2 추가 차단 — A 거점 도달 불가")

    print("\n=== 재할당: B-ugv1 실행 ===")
    ok = await fleet.execute("B-ugv1", TARGET)
    print(f"  주행 시작: {ok}")

    for _ in range(600):
        await asyncio.sleep(1.0)
        fleet.refresh_all()
        a = fleet.agents["B-ugv1"]
        cur, total = a.driver.progress()
        lat, lon = a.driver.position()
        print(f"  {cur}/{total}  ({lat:.5f}, {lon:.5f})  "
              f"연료 {a.resource.fuel_pct:.1f}%  {a.resource.state}")
        if total and cur >= total:
            print("  완주")
            break


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--px4", action="store_true", help="PX4 SITL 연동")
    args = p.parse_args()
    asyncio.run(main(args.px4))