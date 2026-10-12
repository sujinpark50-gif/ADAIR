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
import os
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
def start_tracker(out_csv: Path, duration_s: float, world="adair_flat", model="adair_ugv_1", detail=False):  # noqa: E501
    """detail=True 면 gz_detail (바퀴 접촉·서스펜션 관절, 접촉 센서 있는 후보 모델) — 아니면 gz_track (자세만)."""
    tool = ["-m", "ugv.tools.gz_detail"] if detail else [to_wsl_path(ROOT / "ugv" / "tools" / "gz_track.py")]
    cmd = ["wsl.exe", "-d", WSL_DISTRO, "--cd", to_wsl_path(ROOT), "--exec", "env",
           f"GZ_PARTITION={os.getenv('UGV_GZ_PARTITION', 'adair_ugv')}", "GZ_IP=127.0.0.1",
           "python3", *tool, "--world", world, "--model", model,
           "--out", to_wsl_path(out_csv), "--duration", str(duration_s)]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def load_track(path: Path):
    with open(path, newline="") as f:
        rows = []
        for r in csv.DictReader(f):
            d = {}
            for k, v in r.items():       # gz_detail 기록의 문자열 열(접촉 상태)·빈칸은 건너뛴다
                try:
                    d[k] = float(v)
                except (TypeError, ValueError):
                    pass
            if "steer_rad" not in d and "steer_l" in d and "steer_r" in d:
                d["steer_rad"] = (d["steer_l"] + d["steer_r"]) / 2      # gz_detail: 좌우 조향 평균 = gz_track 의 조향
            rows.append(d)
        return rows


# ---------------------------------------------------------------------------- 분석
def _dist_to_polyline(x, y, pts):
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def command_brake(rows, t, spd, events):
    """정지 명령(취소·비상) 시각부터: 명령 -> 감속 시작(명령 때 속도에서 0.3 m/s 떨어짐) -> 정지(0.1 m/s 미만) 시간·거리.
    시각 대응 = gz_detail 의 wall_t (같은 PC 의 WSL 시계). 명령 때 PX4 위치와 그 시각 Gazebo 위치 차를 함께 남겨 대응을 확인한다."""
    cmd = next((e for e in events or [] if e.get("ev") == "stop_command"), None)
    if cmd is None:
        return {}
    if not rows or "wall_t" not in rows[0]:
        return {"command_brake": {"error": "기록에 wall_t 없음 — 명령 시각 대응 불가 (미확인)"}}
    k = next((i for i, r in enumerate(rows) if r.get("wall_t", 0) >= cmd["wall"]), None)
    if k is None:
        return {"command_brake": {"error": "명령 시각이 기록 밖 (미확인)"}}
    v0 = spd[k]
    kd = next((i for i in range(k, len(spd)) if spd[i] < v0 - 0.3), None)
    ks = next((i for i in range(k, len(spd)) if spd[i] < 0.1), None)

    def d(a, b):
        return round(sum(math.hypot(rows[q + 1]["x"] - rows[q]["x"], rows[q + 1]["y"] - rows[q]["y"]) for q in range(a, b)), 2)
    out = {"kind": cmd.get("kind"), "speed_at_command_mps": round(v0, 2), "planned_stop_dist_m": cmd.get("dist_m"),
           "planned_decel_mps2": cmd.get("decel_mps2"), "item_speed_mps": cmd.get("item_speed_mps"),
           "px4_vs_gz_pos_at_command_m": round(math.hypot(rows[k]["x"] - cmd["px4_en"][0], rows[k]["y"] - cmd["px4_en"][1]), 2),
           "upload_latency_wall_s": round(cmd["wall_started"] - cmd["wall"], 2) if cmd.get("wall_started") else None}
    if kd is not None:
        out.update({"cmd_to_decel_s": round(t[kd] - t[k], 2), "cmd_to_decel_m": d(k, kd)})
    if ks is not None:
        sp = rows[ks]
        stop_pt = cmd.get("stop_en")
        out.update({"cmd_to_stop_s": round(t[ks] - t[k], 2), "cmd_to_stop_m": d(k, ks),
                    "stop_vs_planned_point_m": round(math.hypot(sp["x"] - stop_pt[0], sp["y"] - stop_pt[1]), 2) if stop_pt else None})
        after = [i for i in range(ks, len(spd)) if t[i] >= t[ks] + 1.0]
        out["observed_after_stop_s"] = round(t[-1] - t[ks], 2)
        out["re_motion_after_stop"] = any(spd[i] >= 0.3 for i in after)
    else:
        out["cmd_to_stop_s"] = None
    return {"command_brake": out}


def analyze(rows, plan_xy=None, corners=None, window_s=0.5, sustain_s=3.0, events=None):
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
        # 감속 시작 = 정지 전에 속도가 (최고 − 0.5 m/s) 이상이던 마지막 표본의 다음 (2026-10-13 S1·S3 독립 검증 R2: 첫 구현은 문턱 아래로
        # '처음' 내려간 시각이라 최고속도 뒤 일시 속도 조절을 감속 시작으로 잡았다 — 소방차 susp_a straight50_a 603 m). 휴리스틱이며 명령 시각이 아니다
        peak = max(range(s0, last + 1), key=lambda i: spd[i])
        stop = next((i for i in range(peak, len(spd)) if spd[i] < 0.1), None)
        hi = [i for i in range(peak, stop if stop is not None else len(spd)) if spd[i] >= spd[peak] - 0.5]   # 판정기(vehicle_drive_report)와 같은 문턱
        dec0 = (hi[-1] + 1) if hi and hi[-1] + 1 < len(spd) else None
        if dec0 is not None and stop is not None:
            out["final_decel"] = {
                "from_speed_mps": round(spd[dec0], 2), "sim_time_s": round(t[stop] - t[dec0], 2),
                "distance_m": round(sum(math.hypot(rows[q + 1]["x"] - rows[q]["x"], rows[q + 1]["y"] - rows[q]["y"])
                                        for q in range(dec0, stop)), 1),
                "mean_decel_mps2": round(spd[dec0] / (t[stop] - t[dec0]), 2) if t[stop] > t[dec0] else None,
                "definition": "last_above_peak-0.5mps_v3 (= vehicle_drive_report brake_metrics; heuristic, not command time)"}
        out.update(command_brake(rows, t, spd, events))
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
    # 2026-10-12 차량 개조 검증 (계획 4.4 S3·6.2 T4): 요철 월드 adair_flat_susp 의 x=+30 m 띠를 5 m/s 로 지나 80 m 에서 정지 /
    # 30 km/h 직선 300 m 끝 정지
    "bump": ([(80.0, 0.0)], 5.0),
    "stop30": ([(300.0, 0.0)], 8.33),
    # 소방차 (계획 8.1: 목표 50 km/h = RO_SPEED_LIM 13.9, airframe 51101) 직선 1.2 km 끝 정지
    "straight50": ([(1200.0, 0.0)], 13.9),
    # 2026-10-13 후속 계획 S4: 명령 기준 제동 — 순항 중 정지 명령을 서버와 같은 방식(정지점 하나짜리 미션으로 교체)으로 보낸다.
    #   cancel* = 명령 취소 정지: 정지점 v^2/(2·차종 decel_mps2) + 5 m, 지점 속도 = 지금 속도 (서버 계획 정지와 같은 감속 가정)
    #   estop*  = 비상 정지: 정지점 max(v^2/(2·차종 emergency_decel_mps2) + 1 m, 4 m), 지점 속도 1 m/s (vehicle.py EMERGENCY_STRAIGHT)
    "cancel60": ([(1500.0, 0.0)], 16.7), "estop60": ([(1500.0, 0.0)], 16.7),
    "cancel50": ([(1500.0, 0.0)], 13.9), "estop50": ([(1500.0, 0.0)], 13.9),
    "cancel30": ([(600.0, 0.0)], 8.33), "estop30": ([(600.0, 0.0)], 8.33),
}
STOP_TRIGGER = {"60": 400.0, "50": 400.0, "30": 150.0}
# 정차 유지 정책 (환경변수 UGV_DRIVE_PARK=server | immediate 일 때만, 기본은 예전 동작 = 미션 끝 뒤 기록 대기 후 시동 끔).
# 서버 드라이버(ADAIR_wt_fix ugv/drivers/px4.py park_due, 2026-10-13 확인값)와 같은 규칙: 미션 끝 뒤 0.3 m/s 미만 1 s → 시동 끔,
# 속도와 관계없이 미션 끝 뒤 4 s → 시동 끔. 이 rover 는 미션 끝 뒤 마지막 지점 속도로 계속 달린다 (S4 비상 정지 시험: 정지점 뒤 약 1.25 m/s 로 12 s).
PARK_MODE = os.getenv("UGV_DRIVE_PARK", "")
PARK_AFTER_S, RUNAWAY_PARK_S, PARK_SLOW_MPS = 1.0, 4.0, 0.3    # 명령 조건: 출발에서 이만큼 + 목표 속도 95 % 이상 3 회 연속


async def run(name: str, address: str, track_s: float, world="adair_flat", detail=False, tag="", model="adair_ugv_1", profile="adair_ugv"):
    from mavsdk.asyncio.plugins.action import ActionAsync
    from mavsdk.asyncio.plugins.mission import MissionAsync
    from mavsdk.asyncio.plugins.telemetry import TelemetryAsync
    from mavsdk.plugins.mission.mission import MissionPlan

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_csv = LOG_DIR / f"{tag or name}_{stamp}.csv"
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
    out_csv.with_suffix(".plan.json").write_text(json.dumps(
        {"scenario": name, "speed_mps": speed, "plan_xy": plan_xy, "corners": corners,
         "start_latlon": [pos.latitude_deg, pos.longitude_deg]}, ensure_ascii=False, indent=1), encoding="utf-8")

    health = await first(tel.subscribe_health)
    print(f"[{name}] 시작 위치 gz≈({e0:.1f},{n0:.1f}) health local_ok={health.is_local_position_ok}")
    tracker = start_tracker(out_csv, track_s, world=world, detail=detail, model=model)
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
    if speed < 16.7:
        # 미션 항목의 speed_m_s 만으로는 rover 가 RO_SPEED_LIM 까지 가속한다 (2026-10-12 bump 5 m/s 지정 → 실측 12.6 m/s)
        # → MAV_CMD_DO_CHANGE_SPEED(set_current_speed) 를 몇 번 보낸다
        for _ in range(5):
            try:
                await act.set_current_speed(speed)
            except Exception as e:  # noqa: BLE001
                print(f"[{name}] set_current_speed 실패: {e}")
            await asyncio.sleep(0.2)
    t_start = time.time()
    braked = False
    events = [{"ev": "mission_start", "wall": round(t_start, 3), "park_mode": PARK_MODE or "none (예전: 미션 끝 뒤 기록 대기 후 시동 끔)"}]
    cmd_kind = "cancel" if name.startswith("cancel") else "estop" if name.startswith("estop") else None
    above = 0
    if cmd_kind:
        from ugv.profiles import PROFILES
        prof = PROFILES[profile]
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
        elif cmd_kind and not braked:
            v = await first(tel.subscribe_velocity_ned)
            p = await first(tel.subscribe_position)
            e, n = latlon_to_enu(p.latitude_deg, p.longitude_deg)
            vh = math.hypot(v.north_m_s, v.east_m_s)
            above = above + 1 if vh >= 0.95 * speed else 0
            if above >= 3 and math.hypot(e - e0, n - n0) >= STOP_TRIGGER[name[-2:]]:
                tw = time.time()
                if cmd_kind == "cancel":
                    dec, d, isp = prof.decel_mps2, vh * vh / (2 * prof.decel_mps2) + STOP_MARGIN_M, vh
                else:
                    dec, isp = prof.emergency_decel_mps2, 1.0
                    d = max(vh * vh / (2 * dec) + 1.0, 4.0)
                se, sn = e + d * v.east_m_s / vh, n + d * v.north_m_s / vh
                lat, lon = enu_to_latlon(se, sn)
                await mis.upload_mission(MissionPlan([mission_item(lat, lon, isp, through=False)]))
                await start_mission(mis)
                braked = True
                events.append({"ev": "stop_command", "kind": cmd_kind, "wall": round(tw, 3), "wall_started": round(time.time(), 3),
                               "px4_en": [round(e, 3), round(n, 3)], "px4_speed_mps": round(vh, 3), "stop_en": [round(se, 3), round(sn, 3)],
                               "dist_m": round(d, 2), "decel_mps2": dec, "item_speed_mps": round(isp, 2), "profile": profile})
                print(f"[{name}] 정지 명령({cmd_kind}): {d:.1f} m 앞 정지점, 감속 가정 {dec} m/s², PX4 속도 {vh:.2f} m/s")
        else:
            if await mis.is_mission_finished():
                t_fin = time.time()
                events.append({"ev": "mission_finished_seen", "wall": round(t_fin, 3)})
                if PARK_MODE == "immediate":
                    # 개선 후보 (S4 → S5 검토용): 정지 임무가 끝났다고 본 즉시 시동을 끈다 (서버 4 s 상한 대기 동안의 저속 진행을 없앰)
                    v = await first(tel.subscribe_velocity_ned)
                    vh = math.hypot(v.north_m_s, v.east_m_s)
                    try:
                        await act.disarm()
                        why = "immediate"
                    except Exception as e:  # noqa: BLE001
                        why = f"immediate (disarm 실패: {e})"
                    events.append({"ev": "park_disarm", "why": why, "wall": round(time.time(), 3), "px4_speed_mps": round(vh, 3)})
                    print(f"[{name}] 정차 유지(즉시): PX4 속도 {vh:.2f} m/s")
                elif PARK_MODE == "server":
                    slow_since = None
                    while True:
                        v = await first(tel.subscribe_velocity_ned)
                        vh = math.hypot(v.north_m_s, v.east_m_s)
                        now = time.time()
                        slow_since = (slow_since or now) if vh < PARK_SLOW_MPS else None
                        why = ("runaway_4s" if now - t_fin >= RUNAWAY_PARK_S else
                               "still_1s" if slow_since is not None and now - slow_since >= PARK_AFTER_S else None)
                        if why:
                            try:
                                await act.disarm()
                            except Exception as e:  # noqa: BLE001
                                why += f" (disarm 실패: {e})"
                            events.append({"ev": "park_disarm", "why": why, "wall": round(now, 3), "px4_speed_mps": round(vh, 3)})
                            print(f"[{name}] 정차 유지(서버 규칙): {why}, PX4 속도 {vh:.2f} m/s")
                            break
                        await asyncio.sleep(0.1)
                await asyncio.sleep(12.0 if cmd_kind else 6.0)          # 정지 후 기록 (명령 정지는 재이동 관측 10 s 이상)
                break
        await asyncio.sleep(0.2)
    out_csv.with_suffix(".events.json").write_text(json.dumps(events, ensure_ascii=False, indent=1), encoding="utf-8")
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
    ev_path = csv_path.with_suffix(".events.json")
    events = json.loads(ev_path.read_text(encoding="utf-8")) if ev_path.exists() else None
    res = analyze(load_track(csv_path), plan["plan_xy"], [(c[0], tuple(c[1]), c[2]) for c in plan["corners"]], events=events)
    res = {"scenario": plan["scenario"], "commanded_speed_mps": plan["speed_mps"], "measured_from": "gazebo_pose+sim_time",
           "track_csv": str(csv_path.resolve().relative_to(ROOT) if csv_path.resolve().is_relative_to(ROOT) else csv_path), **res}
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
    ap.add_argument("--world", default="adair_flat", help="Gazebo 월드 이름 (px4-start 의 WORLD 와 같게)")
    ap.add_argument("--detail", action="store_true", help="gz_detail 상세 기록 (접촉 센서 있는 후보 모델)")
    ap.add_argument("--tag", default="", help="결과 파일 이름 앞부분 (기본 = 시나리오)")
    ap.add_argument("--model", default="adair_ugv_1", help="Gazebo 차량 이름 (소방차 adair_firetruck_1)")
    ap.add_argument("--profile", default=None, help="정지 명령 감속 가정 차종 (ugv/profiles.py, 기본 = --model 에서)")
    a = ap.parse_args()
    if a.analyze:
        report(a.analyze, a.plan or a.analyze.with_suffix(".plan.json"))
        return
    if not a.scenario:
        ap.error("scenario 또는 --analyze 필요")
    prof = a.profile or ("adair_firetruck" if a.model.startswith("adair_firetruck") else "adair_ugv")
    out = asyncio.run(run(a.scenario, a.address, a.track_s, world=a.world, detail=a.detail, tag=a.tag, model=a.model, profile=prof))
    report(out, out.with_suffix(".plan.json"))


if __name__ == "__main__":
    sys.exit(main())
