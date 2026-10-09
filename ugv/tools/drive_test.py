# -*- coding: utf-8 -*-
"""ugv/tools/drive_test.py — adair_ugv 차량 시험 주행 (2단계: 기본 이동·정지·최고속도·곡선 감속)

PX4 에는 MAVLink(mavsdk 4.x)로 미션을 보내고, 결과는 Gazebo 가 발행한 차량 자세와 시뮬레이션 시각
(ugv/tools/gz_track.py, WSL 에서 실행)으로만 잰다. PX4 가 알려 주는 속도나 배속 설정값은 판정에 쓰지 않는다.

    # 먼저 WSL 에서 ugv/tools/px4-start.sh 로 평지 월드·차량·PX4 를 띄운다
    python -m ugv.tools.drive_test straight      # 직선 1.2 km → 마지막 지점에서 정지
    python -m ugv.tools.drive_test brake         # 직선 최고속도 도달 뒤 정지 명령(HOLD)
    python -m ugv.tools.drive_test curve         # 90°·45°·135° 꺾임이 있는 경로
    python -m ugv.tools.drive_test --analyze logs/ugv/curve_xxx.csv --plan logs/ugv/curve_xxx.plan.json

결과: logs/ugv/<시험>_<시각>.csv (Gazebo 자세), .plan.json (경로), .json (요약)
"""

import argparse
import asyncio
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs" / "ugv"
WSL_DISTRO = "Ubuntu-24.04"
# 정지 명령: 정지점까지 거리 = v²/(2·STOP_DECEL) + 여유. PX4 도착 감속(RO_DECEL_LIM·RO_JERK_LIM)이 실제로 내는 값(약 2 m/s²) 기준
STOP_DECEL_MPS2 = 2.0
STOP_MARGIN_M = 5.0

# 좌표: UGV 지역 평면 (datum 중심 TM, ugv/geo.py). 평지 월드 원점도 같은 datum 이다
from ugv.geo import FRAME  # noqa: E402


def enu_to_latlon(e, n):
    return FRAME.to_ll(e, n)


def latlon_to_enu(lat, lon):
    return FRAME.to_xy(lat, lon)


def to_wsl_path(p: Path) -> str:
    s = str(p.resolve()).replace("\\", "/")
    return f"/mnt/{s[0].lower()}{s[2:]}"


# ---------------------------------------------------------------------------- 기록
def start_tracker(out_csv: Path, duration_s: float, world="adair_flat", model="adair_ugv_1"):
    cmd = ["wsl.exe", "-d", WSL_DISTRO, "--", "env", "GZ_PARTITION=adair_ugv", "GZ_IP=127.0.0.1",
           "python3", to_wsl_path(ROOT / "ugv" / "tools" / "gz_track.py"), "--world", world, "--model", model,
           "--out", to_wsl_path(out_csv), "--duration", str(duration_s)]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def load_track(path: Path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


# ---------------------------------------------------------------------------- 분석
def _dist_to_polyline(x, y, pts):
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def analyze(rows, plan_xy=None, corners=None, window_s=0.5, sustain_s=3.0):
    """Gazebo 자세(시뮬레이션 시각)만으로 속도·정지·추종 오차를 계산한다."""
    if len(rows) < 3:
        return {"error": "기록 부족", "rows": len(rows)}
    t = [r["sim_t"] for r in rows]
    # 속도: window_s 간격 중심 차분 (수평 거리 / 시뮬레이션 시간)
    spd = []
    j0 = 0
    for i, r in enumerate(rows):
        while t[i] - t[j0] > window_s / 2 and j0 < i:
            j0 += 1
        j1 = i
        while j1 + 1 < len(rows) and t[j1 + 1] - t[i] <= window_s / 2:
            j1 += 1
        dt = t[j1] - t[j0]
        spd.append(math.hypot(rows[j1]["x"] - rows[j0]["x"], rows[j1]["y"] - rows[j0]["y"]) / dt if dt > 0 else 0.0)
    # 지속 최고속도: sustain_s 구간 평균의 최대
    best_sus, k = 0.0, 0
    for i in range(len(rows)):
        while t[i] - t[k] > sustain_s:
            k += 1
        dt = t[i] - t[k]
        if dt >= sustain_s * 0.95:
            path = sum(math.hypot(rows[q + 1]["x"] - rows[q]["x"], rows[q + 1]["y"] - rows[q]["y"]) for q in range(k, i))
            best_sus = max(best_sus, path / dt)
    moving = [i for i, s in enumerate(spd) if s > 0.3]
    total = sum(math.hypot(rows[i + 1]["x"] - rows[i]["x"], rows[i + 1]["y"] - rows[i]["y"]) for i in range(len(rows) - 1))
    out = {
        "samples": len(rows), "sim_duration_s": round(t[-1] - t[0], 2),
        "distance_m": round(total, 1),
        "max_speed_mps_0p5s": round(max(spd), 2), "max_speed_kmh_0p5s": round(max(spd) * 3.6, 1),
        "max_sustained_speed_mps_3s": round(best_sus, 2), "max_sustained_speed_kmh_3s": round(best_sus * 3.6, 1),
        "max_abs_roll_deg": round(max(abs(math.degrees(r["roll"])) for r in rows), 2),
        "max_abs_pitch_deg": round(max(abs(math.degrees(r["pitch"])) for r in rows), 2),
        "max_abs_steer_deg": round(max((abs(math.degrees(r["steer_rad"])) for r in rows
                                        if not math.isnan(r["steer_rad"])), default=float("nan")), 1),
        "z_range_m": [round(min(r["z"] for r in rows), 3), round(max(r["z"] for r in rows), 3)],
    }
    if moving:
        s0 = moving[0]
        for thr in (10.0, 15.0, 16.0):
            hit = next((i for i in range(s0, len(spd)) if spd[i] >= thr), None)
            out[f"time_to_{thr:g}mps_s"] = None if hit is None else round(t[hit] - t[s0], 2)
        last = moving[-1]
        out["final_speed_mps"] = round(spd[-1], 3)
        out["stopped_at_end"] = spd[-1] < 0.1
        # 마지막 감속: 지속 최고속도 근처에서 멈출 때까지
        peak = max(range(s0, last + 1), key=lambda i: spd[i])
        dec0 = next((i for i in range(peak, len(spd)) if spd[i] < spd[peak] * 0.97), None)
        stop = next((i for i in range(peak, len(spd)) if spd[i] < 0.1), None)
        if dec0 is not None and stop is not None:
            out["final_decel"] = {
                "from_speed_mps": round(spd[dec0], 2), "sim_time_s": round(t[stop] - t[dec0], 2),
                "distance_m": round(sum(math.hypot(rows[q + 1]["x"] - rows[q]["x"], rows[q + 1]["y"] - rows[q]["y"])
                                        for q in range(dec0, stop)), 1),
                "mean_decel_mps2": round(spd[dec0] / (t[stop] - t[dec0]), 2) if t[stop] > t[dec0] else None}
    if plan_xy:
        cte = [_dist_to_polyline(r["x"], r["y"], plan_xy) for r in rows]
        srt = sorted(cte)
        out["cross_track_m"] = {"mean": round(sum(cte) / len(cte), 2), "p95": round(srt[int(0.95 * (len(srt) - 1))], 2),
                                "max": round(srt[-1], 2)}
        out["end_error_m"] = round(math.hypot(rows[-1]["x"] - plan_xy[-1][0], rows[-1]["y"] - plan_xy[-1][1]), 2)
    if corners:
        cs = []
        for name, (cx, cy), turn_deg in corners:
            near = [i for i, r in enumerate(rows) if math.hypot(r["x"] - cx, r["y"] - cy) <= 40.0]
            if near:
                i_min = min(near, key=lambda i: spd[i])
                cs.append({"corner": name, "turn_deg": turn_deg, "min_speed_mps": round(spd[i_min], 2),
                           "min_speed_kmh": round(spd[i_min] * 3.6, 1),
                           "closest_m": round(min(math.hypot(rows[i]["x"] - cx, rows[i]["y"] - cy) for i in near), 1),
                           "max_cross_track_m_within_40m": round(max(_dist_to_polyline(rows[i]["x"], rows[i]["y"], plan_xy)
                                                                     for i in near), 2) if plan_xy else None})
            else:
                cs.append({"corner": name, "turn_deg": turn_deg, "reached": False})
        out["corners"] = cs
    return out


# ---------------------------------------------------------------------------- 주행
async def connect(address: str):
    from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
    sdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    await sdk.add_any_connection(address)
    system = await sdk.first_autopilot(timeout_s=20.0)
    if system is None:
        raise RuntimeError(f"{address} 에서 PX4 를 찾지 못함 (px4-start.sh, 방화벽 확인)")
    return sdk, system


async def first(stream, timeout=10.0):
    gen = stream()
    try:
        async with asyncio.timeout(timeout):
            return await anext(gen)
    finally:
        await gen.aclose()


def mission_item(lat, lon, speed, through=True):
    from mavsdk.plugins.mission.mission import MissionItem
    return MissionItem(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=0.0, speed_m_s=speed,
                       is_fly_through=through, gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
                       camera_action=MissionItem.CameraAction.NONE, loiter_time_s=0.0,
                       camera_photo_interval_s=0.0, acceptance_radius_m=3.0, yaw_deg=float("nan"),
                       camera_photo_distance_m=0.0, vehicle_action=MissionItem.VehicleAction.NONE)


async def start_mission(mis, tries=20):
    """업로드 직후에는 PX4 가 미션 검증을 끝내지 않아 시작을 거부할 수 있다 ('Switching to Mission is currently
    not available', 2026-10-09 실측) → 짧게 재시도한다."""
    for k in range(tries):
        try:
            return await mis.start_mission()
        except Exception as e:  # noqa: BLE001
            if k == tries - 1:
                raise
            print(f"start_mission 재시도: {e}")
            await asyncio.sleep(0.5)


SCENARIOS = {
    # (상대 꺾임점 목록 [east, north] m, 속도 m/s)
    "straight": ([(1200.0, 0.0)], 16.7),
    "brake": ([(1500.0, 0.0)], 16.7),
    "curve": ([(400.0, 0.0), (400.0, 400.0), (683.0, 683.0), (283.0, 683.0)], 16.7),
}


async def run(name: str, address: str, track_s: float):
    from mavsdk.asyncio.plugins.action import ActionAsync
    from mavsdk.asyncio.plugins.mission import MissionAsync
    from mavsdk.asyncio.plugins.telemetry import TelemetryAsync
    from mavsdk.plugins.mission.mission import MissionPlan

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_csv = LOG_DIR / f"{name}_{stamp}.csv"
    sdk, system = await connect(address)
    tel, act, mis = TelemetryAsync(system), ActionAsync(system), MissionAsync(system)
    pos = await first(tel.subscribe_position)
    e0, n0 = latlon_to_enu(pos.latitude_deg, pos.longitude_deg)
    rel, speed = SCENARIOS[name]
    # 시험 경로(동쪽 +x 기준)를 차량의 지금 진행 방향으로 돌린다 → 출발 때 제자리 회전 없이 바로 시험 구간
    att = await first(tel.subscribe_heading)
    yaw = math.radians(90.0 - att.heading_deg)          # 나침반 방위(북=0, 시계) → ENU 각(동=0, 반시계)
    c, s_ = math.cos(yaw), math.sin(yaw)
    plan_xy = [(e0, n0)] + [(e0 + de * c - dn * s_, n0 + de * s_ + dn * c) for de, dn in rel]
    items = []
    for k, (x, y) in enumerate(plan_xy[1:]):
        lat, lon = enu_to_latlon(x, y)
        items.append(mission_item(lat, lon, speed, through=k < len(plan_xy) - 2))
    corners = []
    for k in range(1, len(plan_xy) - 1):
        a, b, c = plan_xy[k - 1], plan_xy[k], plan_xy[k + 1]
        h1, h2 = math.atan2(b[1] - a[1], b[0] - a[0]), math.atan2(c[1] - b[1], c[0] - b[0])
        turn = abs(math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi))
        corners.append((f"C{k}", b, round(turn)))
    (LOG_DIR / f"{name}_{stamp}.plan.json").write_text(json.dumps(
        {"scenario": name, "speed_mps": speed, "plan_xy": plan_xy, "corners": corners,
         "start_latlon": [pos.latitude_deg, pos.longitude_deg]}, ensure_ascii=False, indent=1), encoding="utf-8")

    health = await first(tel.subscribe_health)
    print(f"[{name}] 시작 위치 gz≈({e0:.1f},{n0:.1f}) health local_ok={health.is_local_position_ok}")
    tracker = start_tracker(out_csv, track_s)
    await asyncio.sleep(3.0)                      # 기록기가 구독을 시작할 시간
    await mis.clear_mission()
    await mis.set_return_to_launch_after_mission(False)
    await mis.upload_mission(MissionPlan(items))
    for _ in range(20):
        try:
            await act.arm()
            break
        except Exception as e:  # noqa: BLE001  — EKF 안정 전에는 arm 거절
            print(f"[{name}] arm 재시도: {e}")
            await asyncio.sleep(2.0)
    await start_mission(mis)
    t_start = time.time()
    braked = False
    while time.time() - t_start < track_s - 8:
        if name == "brake" and not braked:
            v = await first(tel.subscribe_velocity_ned)
            p = await first(tel.subscribe_position)
            e, n = latlon_to_enu(p.latitude_deg, p.longitude_deg)
            vh = math.hypot(v.north_m_s, v.east_m_s)
            if vh >= 16.0 and math.hypot(e - e0, n - n0) >= 400.0:
                # 정지 = 진행 방향 앞 제동거리 지점 하나짜리 미션으로 교체. PX4 rover HOLD 는 명령 시점 위치로
                # 되돌아가려고 U턴한다 (2026-10-09 실측) → 도로 위 정지에 쓰지 않는다
                d = vh * vh / (2 * STOP_DECEL_MPS2) + STOP_MARGIN_M
                se, sn = e + d * v.east_m_s / vh, n + d * v.north_m_s / vh
                lat, lon = enu_to_latlon(se, sn)
                await mis.upload_mission(MissionPlan([mission_item(lat, lon, vh, through=False)]))
                await start_mission(mis)
                braked = True
                print(f"[{name}] 정지 명령: {d:.1f} m 앞 정지점 (PX4 속도 {vh:.2f} m/s, 참고용)")
        else:
            if await mis.is_mission_finished():
                await asyncio.sleep(6.0)          # 정지 후 기록
                break
        await asyncio.sleep(0.2)
    try:
        await act.disarm()
    except Exception:  # noqa: BLE001
        await act.hold()
    print(f"[{name}] 주행 끝, 기록기 종료 대기")
    tracker.wait(timeout=track_s + 30)
    print(tracker.stdout.read().decode(errors="replace").strip())
    sdk.destroy()
    return out_csv


def report(csv_path: Path, plan_path: Path):
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    res = analyze(load_track(csv_path), plan["plan_xy"], [(c[0], tuple(c[1]), c[2]) for c in plan["corners"]])
    res = {"scenario": plan["scenario"], "commanded_speed_mps": plan["speed_mps"], "measured_from": "gazebo_pose+sim_time",
           "track_csv": str(csv_path.relative_to(ROOT)), **res}
    csv_path.with_suffix(".json").write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=1))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", nargs="?", choices=sorted(SCENARIOS))
    ap.add_argument("--address", default="udpin://0.0.0.0:14541")
    ap.add_argument("--track-s", type=float, default=200.0, help="기록 최대 시간 (벽시계 초)")
    ap.add_argument("--analyze", type=Path)
    ap.add_argument("--plan", type=Path)
    a = ap.parse_args()
    if a.analyze:
        report(a.analyze, a.plan or a.analyze.with_suffix(".plan.json"))
        return
    if not a.scenario:
        ap.error("scenario 또는 --analyze 필요")
    out = asyncio.run(run(a.scenario, a.address, a.track_s))
    report(out, out.with_suffix(".plan.json"))


if __name__ == "__main__":
    sys.exit(main())
