# -*- coding: utf-8 -*-
"""ugv/tools/vehicle_drive_report.py — 평탄 월드 주행 시험 판정 (drive_test --detail 의 gz_detail 기록). 기준은 결과를 보기 전에 정한 것.

    python -m ugv.tools.vehicle_drive_report <폴더> [--out report.json]

파일 이름 <후보>_<월드>_<시나리오>_<회차>_<시각>.csv 로 시나리오를 안다. 속도 = Gazebo 위치의 시뮬레이션 시간 미분(0.2 s 창).
 S3 요철 (bump, 계획 4.4): ① 서스펜션 |q| 최대 < 0.08 m (행정 끝 ±0.10 접촉 없음) ② 띠 통과 뒤 1 s 안에 q 변동 진폭이 최대 변동의 10 % 이하
    ③ 바퀴별 접촉 상실(NO_CONTACT·STALE 메시지 끊김) 연속 ≤ 0.2 s ④ 관절 값 발산 없음(nan·|q| ≥ 0.099). base(서스펜션 없음)는 ③ 만 판정.
 T1 직선 60 (straight, 계획 6.2): 58~62 km/h 를 연속 ≥ 10 s, 그 구간 속도 표준편차 ≤ 0.3 m/s, 평균 조향 |δ| ≤ 1.5°·표준편차 ≤ 0.5°,
    yaw rate ≤ 2°/s, 계획 직선 횡편차 ≤ 0.3 m, roll·pitch ≤ 2°, 62 km/h 초과 없음.
 T2 가속 (straight 앞부분): 출발(0.2 m/s 넘음) → 58 km/h 도달 6~10 s, ≤ 90 m.
 T3 제동 60→0 (straight 끝): 감속 시작(안정 구간 끝 이후 속도가 최대보다 0.5 m/s 떨어진 첫 시각 — 명령 속도 기록이 없어 실측 기준) → 0.2 m/s 미만
    거리 30~60 m, 정지 확인(1 s 연속 < 0.2 m/s) 뒤 10 s 재이동(≥ 0.3 m/s) 없음, 바퀴 잠김(바퀴 각속도·반지름 < 0.3 m/s 인데 차속 > 1 m/s) 연속 ≤ 1 s.
 T4 제동 30→0 (stop30): 같은 방식, 거리 ≤ 20 m.
 S4 명령 정지 (cancel60/50/30·estop60/50/30, 2026-10-13 추가 — S4 결과 보기 전에 정함): drive_test 의 정지 명령 기록(<기록>.events.json)부터
    정지(0.1 m/s 미만)까지 실제 거리 ≤ 명령 때 계획한 정지 거리(서버와 같은 감속 가정), 정지 확인 뒤 ≥ 10 s 재이동 없음. 명령 시각 기록이 없으면 미확인.
 회전 curve (S4 7.1·7.3, 2026-10-13 chain_s4b 결과 보기 전에 정함): 90°·45°·135° 꺾임 평지 경로. 판정 = 바퀴별 접촉 상실(NO_CONTACT·한 바퀴 STALE) 연속
    ≤ 0.2 s, (서스펜션) |q| 최대 < 0.08 m·발산 없음, 최종 정지 확인 뒤 ≥ 10 s 재이동 없음. 기록만: roll 최대, 바퀴 fn 최소/축 평균(하중 이동),
    꺾임별 계획선 최대 이탈, 정적 전복 추정 g·윤거/(2h) (동적 한계 보장 아님). 경로 추종 오차·속도 기준은 이 평지 시험에서 판정하지 않는다.
 횡편차 (v2): .plan.json 의 계획(전송) 직선 기준. 첫 구현(v1)은 실제 첫 위치→실제 마지막 위치 선이라 종점이 벗어나면 기준선이 돌았다 — 참고 열로 남김.
 판정 상태 PASS/FAIL/UNVERIFIED/INVALID 와 기록 품질·실행 유효성 = ugv/tools/vehicle_report_common.py. --baseline 으로 이전 report.json 을 주면
 회차마다 이전 판정을 함께 남긴다 (원본 판정 보존).
 소방차 (태그 앞 ft-, 계획 8.1, 별도 합격표): straight50 — 48~52 km/h ≥ 10 s·52 초과 없음, 가속 0→48 km/h 8~13 s·≤ 110 m, 임무 끝 정지 50→0 ≤ 55 m.
    바퀴 반지름·앞축 위치·축간·서스펜션 유무는 후보 SDF 에서 읽는다. 나머지 항목(표준편차·조향·yaw rate·횡편차·roll/pitch·잠김·재이동)은 UGV 와 같은 기준.
"""
import argparse
import csv
import json
import math
import re
import statistics
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from ugv.tools import vehicle_report_common as vc
from ugv.tools.drive_test import command_brake

ROOT = Path(__file__).resolve().parents[2]
JUDGE_VERSION = "vdr-2 (2026-10-13: SDF 제원·품질·상태·계획선 횡편차·명령 정지)"
EXPECTED = {
    "bump": ["no_contact_le_0.2s"], "bump_susp": ["no_contact_le_0.2s", "q_abs_max_lt_0.08", "no_divergence", "settle_1s_le_10pct"],
    "straight": ["stable_ge_10s", "no_over_limit", "speed_std_le_0.3", "steer_mean_le_1.5_std_le_0.5", "yaw_rate_le_2dps", "lateral_le_0.3m",
                 "roll_pitch_le_2deg", "accel_time_dist_ok", "brake_dist_ok", "stop_confirmed_no_remotion_10s", "wheel_lock_le_1s"],
    "stop30": ["brake_dist_ok", "stop_confirmed_no_remotion_10s", "wheel_lock_le_1s"],
    "command": ["cmd_stop_within_planned", "stop_confirmed_no_remotion_10s", "wheel_lock_le_1s"],
    "curve": ["no_contact_le_0.2s", "stop_confirmed_no_remotion_10s"], "curve_susp": ["no_contact_le_0.2s", "stop_confirmed_no_remotion_10s", "q_abs_max_lt_0.08", "no_divergence"],
}

W = ["wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"]
SUSP = ["susp_front_left", "susp_front_right", "susp_rear_left", "susp_rear_right"]
WHEEL_R = 0.35
BUMP_X = 30.0


def fnum(v):
    try:
        x = float(v)
        return None if math.isnan(x) else x
    except (TypeError, ValueError):
        return None


def load(p):
    rows = list(csv.DictReader(open(p, encoding="utf-8")))
    T = [float(r["sim_t"]) for r in rows]
    X = [float(r["x"]) for r in rows]
    Y = [float(r["y"]) for r in rows]
    spd = []
    for i in range(len(rows)):
        j0, j1 = i, i
        while j0 > 0 and T[i] - T[j0] < 0.1:
            j0 -= 1
        while j1 < len(rows) - 1 and T[j1] - T[i] < 0.1:
            j1 += 1
        dt = T[j1] - T[j0]
        spd.append(math.hypot(X[j1] - X[j0], Y[j1] - Y[j0]) / dt if dt > 0 else 0.0)
    return rows, T, X, Y, spd


def runs_of(flags, T):
    """참인 연속 구간의 최대 길이(s)."""
    best = cur0 = None
    best = 0.0
    for i, f in enumerate(flags):
        if f and cur0 is None:
            cur0 = T[i]
        if (not f or i == len(flags) - 1) and cur0 is not None:
            best = max(best, (T[i] if f else T[i - 1]) - cur0)
            cur0 = None
    return round(best, 3)


def stop_after(T, spd, k0):
    """k0 이후 정지 확인(1 s 연속 < 0.2) 시각과 그 뒤 재이동 여부·관측 시간."""
    i = k0
    while i < len(T):
        if spd[i] < 0.2:
            j = i
            while j < len(T) and spd[j] < 0.2:
                j += 1
            if T[j - 1] - T[i] >= 1.0:
                conf = T[i] + 1.0
                after = [k for k in range(len(T)) if T[k] >= conf]
                rem = next((T[k] for k in after if spd[k] >= 0.3), None)
                return {"stop_confirm_t": round(conf, 2), "observed_after_s": round(T[-1] - conf, 2), "re_motion_t": rem}
            i = j
        else:
            i += 1
    return {"stop_confirm_t": None}


def brake_metrics(rows, T, X, Y, spd, vmax_ref):
    # 감속 시작 = 정지 전에 속도가 (최대 − 0.5 m/s) 이상이던 마지막 표본의 다음 시각. 최대·문턱은 첫 구현과 같다.
    # 첫 구현은 그 문턱 아래로 '처음' 내려간 시각이라, 가속 초과(spike) 뒤 순항 요동을 감속 시작으로 잡았다 (ft-susp_a straight50_a 597 m).
    kmax = max(range(len(spd)), key=lambda i: spd[i])
    k1 = next((i for i in range(kmax, len(spd)) if spd[i] < 0.2), None)
    kend = k1 if k1 is not None else len(spd)
    win = range(kend)
    vloc = spd[kmax]
    hi = [i for i in win if spd[i] >= vloc - 0.5]
    k0 = (hi[-1] + 1) if hi and hi[-1] + 1 < len(spd) else None
    if k0 is None:
        return {"brake": "감속 없음"}
    out = {"brake_start_t": round(T[k0], 2), "brake_start_speed_mps": round(spd[k0], 2)}
    if k1 is None:
        out["brake"] = "정지 전 기록 끝"
        return out
    out["brake_dist_m"] = round(sum(math.hypot(X[i + 1] - X[i], Y[i + 1] - Y[i]) for i in range(k0, k1)), 2)
    out["brake_time_s"] = round(T[k1] - T[k0], 2)
    lock, each = [], {w: [] for w in W}
    for i in range(k0, len(rows)):
        wv = {w: fnum(rows[i].get(f"{w}_v")) for w in W}
        lin = [abs(v) * WHEEL_R for v in wv.values() if v is not None]
        lock.append(bool(lin) and max(lin) < 0.3 and spd[i] > 1.0)          # 전체 바퀴 잠김 (모든 바퀴 동시)
        for w in W:
            each[w].append(wv[w] is not None and abs(wv[w]) * WHEEL_R < 0.3 and spd[i] > 1.0)
    out["wheel_lock_max_s"] = runs_of(lock, T[k0:])
    out["wheel_lock_each_max_s"] = {w: runs_of(each[w], T[k0:]) for w in W}   # 개별 바퀴 (판정은 전체 기준 유지, 개별은 기록)
    out.update(stop_after(T, spd, k1))
    return out


def judge(p):
    ft = p.stem.startswith("ft-")
    m = re.match(r"(.+?)_(adair_flat\w*?)_(bump|straight50|straight|stop30|brake|curve|cancel\d\d|estop\d\d)_(\w+?)_(\d{8}_\d{6})$", p.stem.removeprefix("ft-"))
    if not m:
        return None
    variant, world, sc, rep, stamp = m.groups()
    global WHEEL_R
    model = "adair_firetruck" if ft else "adair_ugv"
    geo = vc.sdf_info(variant, model)
    WHEEL_R = geo["wheel_r"]
    tag = p.stem[:-len(stamp) - 1]
    lo_kmh, hi_kmh, acc_kmh, acc_s, acc_m, brake_lim = ((48, 52, 48, (8.0, 13.0), 110.0, (0.0, 55.0)) if sc == "straight50"
                                                        else (58, 62, 58, (6.0, 10.0), 90.0, (30.0, 60.0)))
    rows, T, X, Y, spd = load(p)
    out = {"key": f"{model}|{variant}|{world}|{sc}|{rep}|{stamp}", "file": p.name, "csv_sha1": vc.sha1_file(p), "model": model, "variant": variant,
           "world": world, "scenario": sc, "rep": rep, "rows": len(rows), "geometry": geo, "judge_version": JUDGE_VERSION}
    markers, evidence = vc.run_markers(p.parent, tag)
    out["run_markers"], out["run_evidence"] = markers, evidence
    checks = {}
    q_i0, q_i1 = 0, len(rows)
    if sc == "bump":
        front, wb = geo["front_x"], geo["wheelbase"]        # 앞축 위치·축간 (후보 SDF)
        k_cross = next((i for i in range(len(X)) if X[i] >= BUMP_X - front + wb + 0.6), None)   # 뒷바퀴까지 띠를 지난 시각
        k_pre = next((i for i in range(len(X)) if X[i] >= BUMP_X - front - 0.6), None)
        if k_pre is not None and k_cross is not None:
            q_i0 = next((i for i in range(len(T)) if T[i] >= T[k_pre] - 1.0), 0)
            q_i1 = next((i for i in range(len(T)) if T[i] > T[k_cross] + 2.0), len(T))
        # 접촉 상실 = 메시지 끊김(STALE)·NO_CONTACT 모두 (gz 접촉 센서는 접촉이 없으면 발행하지 않음 — 2026-10-13 확인: base 요철에서
        # 0.36~0.40 s 끊김이 STALE(≤ 0.5 s)로만 남아 첫 구현의 NO_CONTACT 연속 검사를 통과했다). 첫 구현 값은 참고 열
        nc = {w: runs_of([r[f"{w}_c_state"] in ("NO_CONTACT", "STALE") for r in rows], T) for w in W}
        out["no_contact_max_s_v1"] = {w: runs_of([r[f"{w}_c_state"] == "NO_CONTACT" for r in rows], T) for w in W}
        out["no_contact_max_s"] = nc
        checks["no_contact_le_0.2s"] = all(v <= 0.2 for v in nc.values())
        if geo["has_susp"]:
            qs = {s: [fnum(r.get(f"{s}_q")) for r in rows] for s in SUSP}
            qmax = max((abs(v) for s in SUSP for v in qs[s] if v is not None), default=None)
            out["susp_q_abs_max_m"] = None if qmax is None else round(qmax, 4)
            out["susp_q_abs_max_raw_m"] = qmax                          # 반올림 전 원시값 (판정은 원시값으로)
            if qmax is not None:
                kq = max((i for i in range(len(T)) for s_ in SUSP if qs[s_][i] is not None and abs(qs[s_][i]) == qmax), default=None)
                out["susp_q_abs_max_at"] = None if kq is None else {"sim_t": T[kq], "x": X[kq]}
            # q 가 하나도 없으면 (SDF 에 서스펜션이 있는데 관절 값 결측) 계산하지 않음 → 항목 미계산 = UNVERIFIED (발산 없음으로 보지 않음)
            checks["q_abs_max_lt_0.08"] = None if qmax is None else qmax < 0.08
            checks["no_divergence"] = None if qmax is None else qmax < 0.099
            k_in = next((i for i in range(len(X)) if X[i] >= BUMP_X - front - 0.6), None)   # 앞바퀴가 띠에 닿기 직전
            if qmax is None:
                checks["settle_1s_le_10pct"] = None
                out["note"] = "서스펜션 관절 q 결측"
            elif k_cross is not None and k_in is not None:
                dev, dev_static = {}, {}
                for s in SUSP:
                    vals = [v for v in qs[s] if v is not None]
                    pre = [qs[s][i] for i in range(len(T)) if T[k_in] - 1.0 <= T[i] < T[k_in] and qs[s][i] is not None]
                    base_pre = statistics.median(pre) if pre else statistics.median(vals)
                    peak = max(abs(qs[s][i] - base_pre) for i in range(k_in, k_cross + 1) if qs[s][i] is not None)
                    win = [qs[s][i] for i in range(len(T)) if T[k_cross] + 1.0 <= T[i] <= T[k_cross] + 2.0 and qs[s][i] is not None]
                    # 진폭 = 통과 1 s 뒤 1 s 창 안의 변동 (그 창 중앙값 기준) — 가감속에 따른 앞뒤 하중 이동(준정적 평행 이동)은 진동이 아니다
                    base_after = statistics.median(win) if win else None
                    dev[s] = round(max(abs(v - base_after) for v in win) / peak, 3) if win and peak > 1e-6 else None
                    q_eq = statistics.median(vals[-200:])
                    dev_static[s] = round(max(abs(v - q_eq) for v in win) / peak, 3) if win and peak > 1e-6 else None
                out["q_bump_peak_dev_m"] = {s: None for s in SUSP}
                out["q_residual_ratio_1s_after"] = dev
                out["q_offset_vs_static_eq_ratio"] = dev_static   # 참고: 정지 평형 기준 (첫 구현, 가감속 하중 이동 포함)
                checks["settle_1s_le_10pct"] = all(v is not None and v <= 0.10 for v in dev.values())
            else:
                checks["settle_1s_le_10pct"] = False
                out["note"] = "띠 통과 기록 없음"
            out.pop("q_bump_peak_dev_m", None)
            kb = next((i for i in range(len(X)) if X[i] >= BUMP_X - front - 0.6), None)
            out["speed_at_bump_mps"] = None if kb is None else round(spd[kb], 2)
    elif sc in ("straight", "straight50", "stop30"):
        kmh = [v * 3.6 for v in spd]
        if sc in ("straight", "straight50"):
            band = [lo_kmh <= v <= hi_kmh for v in kmh]
            best = (0.0, None, None)
            i = 0
            while i < len(band):
                if band[i]:
                    j = i
                    while j < len(band) and band[j]:
                        j += 1
                    if T[j - 1] - T[i] > best[0]:
                        best = (T[j - 1] - T[i], i, j)
                    i = j
                else:
                    i += 1
            dur, a, b = best
            out["stable_58_62_s"] = round(dur, 2)          # 소방차는 48~52 km/h 구간
            out["max_kmh"] = round(max(kmh), 2)
            checks["stable_ge_10s"] = dur >= 10.0
            checks["no_over_limit"] = max(kmh) <= hi_kmh
            if a is not None:
                seg = rows[a:b]
                sp = spd[a:b]
                steer = [math.degrees((float(r["steer_l"]) + float(r["steer_r"])) / 2) for r in seg]
                yaw = [float(r["yaw"]) for r in seg]
                tt = T[a:b]
                yr = [abs(math.degrees((yaw[k + 1] - yaw[k] + math.pi) % (2 * math.pi) - math.pi) / (tt[k + 1] - tt[k]))
                      for k in range(len(seg) - 1) if tt[k + 1] > tt[k]]
                # v1 (참고): 실제 첫 위치 → 실제 마지막 위치 선
                x0, y0, x1, y1 = X[0], Y[0], X[-1], Y[-1]
                L = math.hypot(x1 - x0, y1 - y0) or 1.0
                lat_v1 = [abs((X[k] - x0) * (y1 - y0) - (Y[k] - y0) * (x1 - x0)) / L for k in range(a, b)]
                out["lateral_dev_max_m_v1"] = round(max(lat_v1), 3)
                # v2 (판정): .plan.json 의 계획(전송) 직선
                plan_p = p.with_suffix(".plan.json")
                if plan_p.exists():
                    (px0, py0), (px1, py1) = json.loads(plan_p.read_text(encoding="utf-8"))["plan_xy"][:2]
                    PL = math.hypot(px1 - px0, py1 - py0) or 1.0
                    lat = [abs((X[k] - px0) * (py1 - py0) - (Y[k] - py0) * (px1 - px0)) / PL for k in range(a, b)]
                    out["end_error_m"] = round(math.hypot(X[-1] - px1, Y[-1] - py1), 3)
                    out["lateral_basis"] = "plan.json 계획 직선"
                else:
                    lat = None
                    out["lateral_basis"] = "plan.json 없음 — 미확인"
                out.update({"speed_std_mps": round(statistics.pstdev(sp), 3), "steer_mean_abs_deg": round(abs(statistics.mean(steer)), 3),
                            "steer_std_deg": round(statistics.pstdev(steer), 3), "yaw_rate_max_dps": round(max(yr), 2) if yr else None,
                            "lateral_dev_max_m": None if lat is None else round(max(lat), 3),
                            "roll_pitch_max_deg": round(max(max(abs(float(r["roll"])), abs(float(r["pitch"]))) for r in seg) * 180 / math.pi, 3)})
                checks["speed_std_le_0.3"] = out["speed_std_mps"] <= 0.3
                checks["steer_mean_le_1.5_std_le_0.5"] = out["steer_mean_abs_deg"] <= 1.5 and out["steer_std_deg"] <= 0.5
                # yaw rate 는 0.036 s 표본 차분이라 잡음이 크다 — 0.5 s 평활 최대로 본다
                checks["yaw_rate_le_2dps"] = out["yaw_rate_max_dps"] is not None
                checks["lateral_le_0.3m"] = None if lat is None else out["lateral_dev_max_m"] <= 0.3
                checks["roll_pitch_le_2deg"] = out["roll_pitch_max_deg"] <= 2.0
                # 평활 yaw rate
                if len(tt) > 20:
                    sm = []
                    for k in range(len(tt)):
                        k2 = next((q for q in range(k, len(tt)) if tt[q] - tt[k] >= 0.5), None)
                        if k2 is None:
                            break
                        sm.append(abs(math.degrees((yaw[k2] - yaw[k] + math.pi) % (2 * math.pi) - math.pi) / (tt[k2] - tt[k])))
                    out["yaw_rate_max_0.5s_dps"] = round(max(sm), 3) if sm else None
                    checks["yaw_rate_le_2dps"] = out["yaw_rate_max_0.5s_dps"] is not None and out["yaw_rate_max_0.5s_dps"] <= 2.0
                q_i0, q_i1 = a, b
            k_go = next((i for i in range(len(spd)) if spd[i] > 0.2), None)
            k_58 = next((i for i in range(len(kmh)) if kmh[i] >= acc_kmh), None)
            if k_go is not None and k_58 is not None:
                out["accel_0_58_s"] = round(T[k_58] - T[k_go], 2)
                out["accel_0_58_m"] = round(sum(math.hypot(X[i + 1] - X[i], Y[i + 1] - Y[i]) for i in range(k_go, k_58)), 1)
                checks["accel_time_dist_ok"] = acc_s[0] <= out["accel_0_58_s"] <= acc_s[1] and out["accel_0_58_m"] <= acc_m
            else:
                checks["accel_time_dist_ok"] = False
        br = brake_metrics(rows, T, X, Y, spd, None)
        out.update(br)
        lim = brake_lim if sc in ("straight", "straight50") else (0.0, 20.0)
        d = br.get("brake_dist_m")
        checks["brake_dist_ok"] = d is not None and lim[0] <= d <= lim[1]
        checks["stop_confirmed_no_remotion_10s"] = (br.get("stop_confirm_t") is not None and br.get("observed_after_s", 0) >= 10.0
                                                   and br.get("re_motion_t") is None)
        checks["wheel_lock_le_1s"] = br.get("wheel_lock_max_s") is not None and br["wheel_lock_max_s"] <= 1.0
    elif sc.startswith(("cancel", "estop")):
        ev_p = p.with_suffix(".events.json")
        events = json.loads(ev_p.read_text(encoding="utf-8")) if ev_p.exists() else None
        rows_f = [{k: fnum(v) for k, v in r.items()} for r in rows]
        cb = command_brake(rows_f, T, spd, events).get("command_brake") if events else {"error": "events.json 없음 — 명령 시각 미확인"}
        if cb and events and "error" not in cb:
            # 계획 정지점 통과(진행 방향 투영 ≥ 0) 시각·속도 — 제동이 계획 감속을 따랐는지. 정차 유지 정책 유무 표시
            cmd = next(e for e in events if e["ev"] == "stop_command")
            k_c = next((i for i, r in enumerate(rows_f) if (r.get("wall_t") or 0) >= cmd["wall"]), 0)
            ux, uy = cmd["stop_en"][0] - cmd["px4_en"][0], cmd["stop_en"][1] - cmd["px4_en"][1]
            L = math.hypot(ux, uy) or 1.0
            kp = next((i for i in range(k_c, len(X)) if ((X[i] - cmd["stop_en"][0]) * ux + (Y[i] - cmd["stop_en"][1]) * uy) / L >= 0), None)
            cb["speed_at_planned_point_mps"] = None if kp is None else round(spd[kp], 2)
            # 기록용 (판정 아님 — 2026-10-13 S4 결과를 본 뒤 발견한 정의 문제라 기준 변경은 별도 검토):
            # 실제 거리(명령 때 Gazebo 위치부터)와 계획 거리(명령 때 PX4 보고 위치부터)는 기준점이 달라 px4_vs_gz 만큼 어긋난다.
            # 정지 위치를 계획 정지점 기준 진행 방향 부호 거리로 (음수 = 정지점 앞에서 섬)
            ks_ = next((i for i in range(k_c, len(spd)) if spd[i] < 0.1), None)
            cb["stop_along_track_vs_planned_point_m"] = (None if ks_ is None else
                                                         round(((X[ks_] - cmd["stop_en"][0]) * ux + (Y[ks_] - cmd["stop_en"][1]) * uy) / L, 2))
            cb["note_basis"] = "cmd_to_stop_m 은 Gazebo 위치 기준, planned_stop_dist_m 은 PX4 보고 위치 기준 — 비교 시 px4_vs_gz 오차 포함"
            cb["cmd_to_planned_point_s"] = None if kp is None else round(T[kp] - T[k_c], 2)
            pk = next((e for e in events if e["ev"] == "park_disarm"), None)
            cb["park_policy"] = (f"server ({pk['why']})" if pk else
                                 "없음 — PX4 단독: 미션 끝 뒤 시험 도구가 기록 대기 후 시동 끔 (시스템 정지거리 아님, 서버는 미션 끝 1 s 정차·4 s 상한에 시동 끔)")
        out["command_brake"] = cb
        if cb and cb.get("cmd_to_stop_m") is not None and cb.get("planned_stop_dist_m") is not None:
            checks["cmd_stop_within_planned"] = cb["cmd_to_stop_m"] <= cb["planned_stop_dist_m"]
            k_cmd = next((i for i, r in enumerate(rows_f) if (r.get("wall_t") or 0) >= next(e for e in events if e["ev"] == "stop_command")["wall"]), 0)
            q_i0 = k_cmd
        br = brake_metrics(rows, T, X, Y, spd, None)
        out["final_decel_heuristic"] = {k: br.get(k) for k in ("brake_start_t", "brake_start_speed_mps", "brake_dist_m", "brake_time_s")}
        out.update({k: br.get(k) for k in ("wheel_lock_max_s", "wheel_lock_each_max_s", "stop_confirm_t", "observed_after_s", "re_motion_t")})
        checks["stop_confirmed_no_remotion_10s"] = (br.get("stop_confirm_t") is not None and br.get("observed_after_s", 0) >= 10.0
                                                   and br.get("re_motion_t") is None)
        checks["wheel_lock_le_1s"] = br.get("wheel_lock_max_s") is not None and br["wheel_lock_max_s"] <= 1.0
    elif sc == "curve":
        nc = {w: runs_of([r[f"{w}_c_state"] in ("NO_CONTACT", "STALE") for r in rows], T) for w in W}
        out["no_contact_max_s"] = nc
        checks["no_contact_le_0.2s"] = all(v <= 0.2 for v in nc.values())
        out["roll_max_deg"] = round(max(abs(float(r["roll"])) for r in rows) * 180 / math.pi, 3)
        mv = [i for i in range(len(rows)) if spd[i] > 1.0]
        fnr = []
        for i in mv:
            fn = [fnum(rows[i].get(f"{w}_fn")) for w in W]
            if None not in fn and all(rows[i][f"{w}_c_state"] == "VALID" for w in W):
                fnr.append(min(fn[0] / ((fn[0] + fn[1]) / 2), fn[1] / ((fn[0] + fn[1]) / 2), fn[2] / ((fn[2] + fn[3]) / 2), fn[3] / ((fn[2] + fn[3]) / 2)))
        out["min_fn_over_axle_mean"] = round(min(fnr), 3) if fnr else None
        plan_p = p.with_suffix(".plan.json")
        if plan_p.exists():
            pl = json.loads(plan_p.read_text(encoding="utf-8"))
            pts = pl["plan_xy"]
            from ugv.tools.drive_test import _dist_to_polyline
            out["corner_max_dev_m"] = {c[0]: round(max((_dist_to_polyline(X[i], Y[i], pts) for i in range(len(X))
                                                        if math.hypot(X[i] - c[1][0], Y[i] - c[1][1]) <= 40.0), default=float("nan")), 2) for c in pl["corners"]}
        geom = vc.sdf_info(variant, model)
        m_sdf = ET.fromstring((ROOT / geom["model_sdf"]).read_text(encoding="utf-8")).find("model")
        bl = [l for l in m_sdf.findall("link") if l.get("name") == "base_link"][0]
        h_cg = float(bl.find("pose").text.split()[2]) + float(bl.find("inertial/pose").text.split()[2])
        ys = [float(l.find("pose").text.split()[1]) for l in m_sdf.findall("link") if l.get("name", "").startswith("wheel_") and l.get("name").endswith("_link") and "steering" not in l.get("name")]
        track = 2 * min(abs(y) for y in ys) if ys else None
        out["static_rollover_g_est"] = round(track / (2 * h_cg), 3) if track and h_cg else None
        out["body_cg_height_m"] = round(h_cg, 3)
        if geo["has_susp"]:
            qmax = max((abs(fnum(r.get(f"{s_}_q"))) for r in rows for s_ in SUSP if fnum(r.get(f"{s_}_q")) is not None), default=None)
            out["susp_q_abs_max_raw_m"] = qmax
            checks["q_abs_max_lt_0.08"] = None if qmax is None else qmax < 0.08
            checks["no_divergence"] = None if qmax is None else qmax < 0.099
        k_go = next((i for i in range(len(spd)) if spd[i] > 0.3), 0)
        k_stop = next((i for i in range(len(spd) - 1, -1, -1) if spd[i] > 0.3), len(spd) - 1)
        q_i0, q_i1 = k_go, k_stop + 1
        st = stop_after(T, spd, k_stop)
        out.update(st)
        checks["stop_confirmed_no_remotion_10s"] = st.get("stop_confirm_t") is not None and st.get("observed_after_s", 0) >= 10.0 and st.get("re_motion_t") is None
    if sc in ("straight", "straight50", "stop30"):
        out["brake_definition"] = "임무 도착 감속 — 실측 휴리스틱(정지 전 마지막 '최대 − 0.5 m/s' 이상 다음 표본), 명령 시각 아님"
    qual = vc.quality(rows, T, geo["has_susp"] and sc in ("bump", "curve"), q_i0, q_i1)
    out["data_quality"] = qual
    exp = (EXPECTED["bump_susp"] if geo["has_susp"] else EXPECTED["bump"]) if sc == "bump" else \
        EXPECTED["straight"] if sc in ("straight", "straight50") else EXPECTED["stop30"] if sc == "stop30" else \
        EXPECTED["command"] if sc.startswith(("cancel", "estop")) else         (EXPECTED["curve_susp"] if geo["has_susp"] else EXPECTED["curve"]) if sc == "curve" else []
    out["checks"] = checks
    out["expected_checks"] = exp
    out["status"], out["missing_checks"] = vc.status(markers, qual["ok"], checks, exp)
    out["pass"] = out["status"] == "PASS" if out["status"] in ("PASS", "FAIL") else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--out", default=None)
    ap.add_argument("--baseline", default=None, help="이전 report.json — 회차별 이전 판정을 함께 남긴다")
    a = ap.parse_args()
    res = [r for r in (judge(p) for p in sorted(Path(a.folder).glob("*.csv"))) if r]
    try:
        commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    except OSError:
        commit = None
    base = {}
    if a.baseline and Path(a.baseline).exists():
        base = {b["file"]: b for b in json.loads(Path(a.baseline).read_text(encoding="utf-8")) if isinstance(b, dict) and "file" in b}
    for r in res:
        r["judge_commit"] = commit
        b = base.get(r["file"])
        if b:
            r["previous"] = {"pass": b.get("pass"), "failed_checks": [k for k, v in (b.get("checks") or {}).items() if not v],
                             "judge_version": b.get("judge_version", "vdr-1")}
    for r in res:
        bad = [k for k, v in r["checks"].items() if v is False] + [f"{k}(미계산)" for k in r.get("missing_checks", [])]
        keys = [k for k in ("stable_58_62_s", "max_kmh", "accel_0_58_s", "accel_0_58_m", "brake_dist_m", "brake_start_speed_mps", "wheel_lock_max_s",
                            "susp_q_abs_max_raw_m", "q_residual_ratio_1s_after", "no_contact_max_s", "lateral_dev_max_m", "lateral_dev_max_m_v1", "end_error_m",
                            "yaw_rate_max_0.5s_dps", "roll_max_deg", "min_fn_over_axle_mean", "corner_max_dev_m", "static_rollover_g_est") if k in r]
        prev = f" [이전 {'합격' if r['previous']['pass'] else '불합격'}]" if r.get("previous") else ""
        print(f"{'ft-' if r['model'] == 'adair_firetruck' else ''}{r['variant']}/{r['scenario']}/{r['rep']}: {r['status']} {bad if bad else ''}{prev} | "
              + ", ".join(f"{k}={r[k]}" for k in keys) + (f" | 품질 {r['data_quality']['why']}" if r["data_quality"].get("why") else "")
              + (f" | 무효표시 {list(r['run_markers'])}" if r["run_markers"] else "") + (f" | 명령정지 {r['command_brake']}" if r.get("command_brake") else ""))
    if a.out:
        Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
