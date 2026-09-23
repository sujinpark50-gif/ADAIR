# -*- coding: utf-8 -*-
"""
tests/it_patch_checks.py
========================
통합 패치(누락분 복원 + 결함 수정) 회귀 확인.

    python3 -m tests.it_patch_checks

real 환경(rasterio + environment/data/processed/*.tif)이 없으면 real 항목은 SKIP 한다.
"""

import importlib

import config
from interfaces.schema import EnvironmentState, Observation, Assignment

results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond), detail))


def skip(name, why):
    results.append((name, None, why))


def _env(cells):
    return EnvironmentState(simulation_time_s=0.0, fire_cells=cells, risk_zone=[], wind_speed=3.0,
                            wind_direction=270.0, spread_direction="E")


def run():
    from connectors import orchestrator_connector as oc, fire_connector as fc

    # 1) 화재셀이 없으면 배정하지 않는다 (NO_FIRE_TARGET → CANCELLED)
    t = oc.create_task(0.0, _env([]))
    check("화재 없음 → target_resolved=False(NO_FIRE_TARGET)",
          (not t.target_resolved) and t.unresolved_reason == "NO_FIRE_TARGET")

    # 2) 격자 밖 셀: real 모드에서는 원점으로 대체하지 않는다
    prev = config.FIRE_CONNECTION_MODE
    config.FIRE_CONNECTION_MODE = "real"
    try:
        t = oc.create_task(0.0, _env([{"x": 99999, "y": 99999, "risk_score": 0.9}]))
        check("변환 실패(real) → TARGET_UNRESOLVED, 원점 대체 없음",
              (not t.target_resolved) and t.unresolved_reason == "TARGET_UNRESOLVED"
              and t.target_lat == 0.0)
    finally:
        config.FIRE_CONNECTION_MODE = prev

    # 3) mock 모드에서만 placeholder 허용 + 출처 명시
    t = oc.create_task(0.0, _env([{"x": 99999, "y": 99999, "risk_score": 0.9}]))
    check("변환 실패(mock) → MOCK_PLACEHOLDER 로 명시", t.target_source == "MOCK_PLACEHOLDER")

    # 4) 관측 근거 없는 관측은 환경에 반영하지 않는다
    obs_none = Observation("A-uav1", 38.0, 128.2, 0.0, "THERMAL", {"hotspot_detected": False})
    obs_road = Observation("A-ugv1", 38.0, 128.2, 0.0, "ROAD_STATUS", {"passable": True})
    obs_noloc = Observation("A-uav1", None, None, 0.0, "THERMAL", {"hotspot_detected": True})
    obs_fire = Observation("A-uav1", 38.0, 128.2, 0.0, "THERMAL", {"hotspot_detected": True})
    check("hotspot=False·도로관측·위치없음 → env_updated=False",
          not fc.update_environment(_env([]), [obs_none, obs_road, obs_noloc]).env_updated)
    check("hotspot=True + 위치 → env_updated=True (mock)",
          fc.update_environment(_env([]), [obs_fire]).env_updated)

    # 5) UGV: 목표 근처 노드가 없으면 임의 노드로 대체하지 않고 거절
    try:
        from integration import bridge_ugv
        far = bridge_ugv.send_ugv_command("T", Assignment("T", "A-ugv1", 37.99, 128.14))
        near = bridge_ugv.send_ugv_command("T", Assignment("T", "A-ugv1", 38.11465, 128.15694))
        check("UGV 목표 15km 밖 → REJECT TARGET_UNREACHABLE (F1 대체 없음)",
              far.response == "REJECT" and far.reason == "TARGET_UNREACHABLE")
        check("UGV 목표가 도로 노드 근처 → ACCEPT", near.response == "ACCEPT")
    except Exception as e:  # noqa: BLE001
        skip("UGV bridge", f"로드 실패: {e}")

    # ---- real 환경 필요 항목 ----
    try:
        import gz_bridge
        gz_bridge.terrain_elev(19, 142)
    except Exception as e:  # noqa: BLE001
        skip("real 항목(DEM·CA)", f"DEM/rasterio 없음: {e}")
        return

    # 6) 타깃 고도 = 지면 고도(AMSL). 여유고도는 UAV Agent 가 한 번만 더한다
    ground = gz_bridge.terrain_elev(19, 142)
    t = oc.create_task(0.0, _env([{"x": 19, "y": 142, "risk_score": 0.9}]))
    check("타깃 고도 = DEM 지면 고도 (여유 미포함)",
          t.target_source == "DEM" and abs(t.target_alt_m - ground) < 1e-6,
          f"ground={ground:.1f} m, target_alt_m={t.target_alt_m:.1f} m")

    # 7) terrain_elev: 격자 밖이면 클램프하지 않고 예외
    try:
        gz_bridge.terrain_elev(-1, 0)
        check("terrain_elev 격자 밖 → 예외", False)
    except IndexError:
        check("terrain_elev 격자 밖 → 예외", True)

    # 8) advance=False 재조회는 CA 를 진행시키지 않는다
    config.FIRE_CONNECTION_MODE = "real"
    try:
        importlib.reload(fc)
        s0 = fc.get_environment_state(0.0)                  # 최초(점화 직후)
        s1 = fc.get_environment_state(1.0)                  # 한 스텝 진행
        n_before = len(s1.fire_cells)
        for _ in range(5):
            s = fc.get_environment_state(1.0, advance=False)
        check("advance=False 5회 재조회 → sim_step·화재셀 불변",
              s.sim_step == s1.sim_step and len(s.fire_cells) == n_before,
              f"sim_step {s0.sim_step}->{s1.sim_step}->{s.sim_step}, cells={n_before}")
    finally:
        config.FIRE_CONNECTION_MODE = prev


if __name__ == "__main__":
    run()
    fails = 0
    for name, ok, detail in results:
        tag = "SKIP" if ok is None else ("PASS" if ok else "FAIL")
        fails += ok is False
        print(f"[{tag}] {name}" + (f"  ({detail})" if detail else ""))
    print("\n실패 없음" if not fails else f"\n실패 {fails}건")
    raise SystemExit(1 if fails else 0)
