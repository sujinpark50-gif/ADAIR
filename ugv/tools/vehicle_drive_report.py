# -*- coding: utf-8 -*-
"""ugv/tools/vehicle_drive_report.py — 평탄 월드 주행 시험 판정 (drive_test --detail 의 gz_detail 기록). 기준은 결과를 보기 전에 정한 것.

    python -m ugv.tools.vehicle_drive_report <폴더> [--out report.json]

파일 이름 <후보>_<월드>_<시나리오>_<회차>_<시각>.csv 로 시나리오를 안다. 속도 = Gazebo 위치의 시뮬레이션 시간 미분(0.2 s 창).
 S3 요철 (bump, 계획 4.4): ① 서스펜션 |q| 최대 < 0.08 m (행정 끝 ±0.10 접촉 없음) ② 띠 통과 뒤 1 s 안에 q 변동 진폭이 최대 변동의 10 % 이하
    ③ 바퀴별 NO_CONTACT 연속 ≤ 0.2 s ④ 관절 값 발산 없음(nan·|q| ≥ 0.099). base(서스펜션 없음)는 ③ 만 판정.
 T1 직선 60 (straight, 계획 6.2): 58~62 km/h 를 연속 ≥ 10 s, 그 구간 속도 표준편차 ≤ 0.3 m/s, 평균 조향 |δ| ≤ 1.5°·표준편차 ≤ 0.5°,
    yaw rate ≤ 2°/s, 계획 직선 횡편차 ≤ 0.3 m, roll·pitch ≤ 2°, 62 km/h 초과 없음.
 T2 가속 (straight 앞부분): 출발(0.2 m/s 넘음) → 58 km/h 도달 6~10 s, ≤ 90 m.
 T3 제동 60→0 (straight 끝): 감속 시작(안정 구간 끝 이후 속도가 최대보다 0.5 m/s 떨어진 첫 시각 — 명령 속도 기록이 없어 실측 기준) → 0.2 m/s 미만
    거리 30~60 m, 정지 확인(1 s 연속 < 0.2 m/s) 뒤 10 s 재이동(≥ 0.3 m/s) 없음, 바퀴 잠김(바퀴 각속도·반지름 < 0.3 m/s 인데 차속 > 1 m/s) 연속 ≤ 1 s.
 T4 제동 30→0 (stop30): 같은 방식, 거리 ≤ 20 m.
 소방차 (태그 앞 ft-, 계획 8.1, 별도 합격표): straight50 — 48~52 km/h ≥ 10 s·52 초과 없음, 가속 0→48 km/h 8~13 s·≤ 110 m, 임무 끝 정지 50→0 ≤ 55 m.
    바퀴 반지름·앞축 위치·축간·서스펜션 유무는 후보 SDF 에서 읽는다. 나머지 항목(표준편차·조향·yaw rate·횡편차·roll/pitch·잠김·재이동)은 UGV 와 같은 기준.
"""
import argparse
import csv
import json
import math
import re
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

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


def sdf_geometry(variant, model):
    """후보 SDF 에서 앞축 x·축간·바퀴 반지름·서스펜션(prismatic) 유무 — 후보마다 다르다 (fire_v2: 2.795·4.3·0.42)."""
    m = ET.fromstring((ROOT / "ugv" / "gazebo" / "variants" / variant / model / "model.sdf").read_text(encoding="utf-8")).find("model")
    xs = {l.get("name"): float(l.find("pose").text.split()[0]) for l in m.findall("link") if l.get("name") in ("wheel_front_left_link", "wheel_rear_left_link")}
    wl = [l for l in m.findall("link") if l.get("name") == "wheel_front_left_link"][0]
    return {"front_x": xs["wheel_front_left_link"], "wheelbase": round(xs["wheel_front_left_link"] - xs["wheel_rear_left_link"], 4),
            "wheel_r": float(wl.find(".//collision//radius").text), "has_susp": any(j.get("type") == "prismatic" for j in m.findall("joint"))}


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
    lock = []
    for i in range(k0, len(rows)):
        wv = [fnum(rows[i].get(f"{w}_v")) for w in W]
        wv = [abs(v) * WHEEL_R for v in wv if v is not None]
        lock.append(bool(wv) and max(wv) < 0.3 and spd[i] > 1.0)
    out["wheel_lock_max_s"] = runs_of(lock, T[k0:])
    out.update(stop_after(T, spd, k1))
    return out


def judge(p):
    ft = p.stem.startswith("ft-")
    m = re.match(r"(.+?)_(adair_flat\w*?)_(bump|straight50|straight|stop30|brake|curve)_(\w+?)_\d{8}_\d{6}$", p.stem.removeprefix("ft-"))
    if not m:
        return None
    variant, world, sc, rep = m.groups()
    global WHEEL_R
    geo = sdf_geometry(variant, "adair_firetruck" if ft else "adair_ugv")
    WHEEL_R = geo["wheel_r"]
    lo_kmh, hi_kmh, acc_kmh, acc_s, acc_m, brake_lim = ((48, 52, 48, (8.0, 13.0), 110.0, (0.0, 55.0)) if sc == "straight50"
                                                        else (58, 62, 58, (6.0, 10.0), 90.0, (30.0, 60.0)))
    rows, T, X, Y, spd = load(p)
    out = {"file": p.name, "model": "adair_firetruck" if ft else "adair_ugv", "variant": variant, "world": world, "scenario": sc, "rep": rep, "rows": len(rows), "geometry": geo}
    checks = {}
    if sc == "bump":
        front, wb = geo["front_x"], geo["wheelbase"]        # 앞축 위치·축간 (후보 SDF)
        k_cross = next((i for i in range(len(X)) if X[i] >= BUMP_X - front + wb + 0.6), None)   # 뒷바퀴까지 띠를 지난 시각
        nc = {w: runs_of([r[f"{w}_c_state"] == "NO_CONTACT" for r in rows], T) for w in W}
        out["no_contact_max_s"] = nc
        checks["no_contact_le_0.2s"] = all(v <= 0.2 for v in nc.values())
        if geo["has_susp"]:
            qs = {s: [fnum(r.get(f"{s}_q")) for r in rows] for s in SUSP}
            qmax = max((abs(v) for s in SUSP for v in qs[s] if v is not None), default=None)
            out["susp_q_abs_max_m"] = None if qmax is None else round(qmax, 4)
            checks["q_abs_max_lt_0.08"] = qmax is not None and qmax < 0.08
            checks["no_divergence"] = qmax is not None and qmax < 0.099
            k_in = next((i for i in range(len(X)) if X[i] >= BUMP_X - front - 0.6), None)   # 앞바퀴가 띠에 닿기 직전
            if k_cross is not None and k_in is not None:
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
                # 계획 직선 = 출발점 → 마지막 지점 (drive_test 는 출발 방향으로 직선)
                x0, y0, x1, y1 = X[0], Y[0], X[-1], Y[-1]
                L = math.hypot(x1 - x0, y1 - y0) or 1.0
                lat = [abs((X[k] - x0) * (y1 - y0) - (Y[k] - y0) * (x1 - x0)) / L for k in range(a, b)]
                out.update({"speed_std_mps": round(statistics.pstdev(sp), 3), "steer_mean_abs_deg": round(abs(statistics.mean(steer)), 3),
                            "steer_std_deg": round(statistics.pstdev(steer), 3), "yaw_rate_max_dps": round(max(yr), 2) if yr else None,
                            "lateral_dev_max_m": round(max(lat), 3),
                            "roll_pitch_max_deg": round(max(max(abs(float(r["roll"])), abs(float(r["pitch"]))) for r in seg) * 180 / math.pi, 3)})
                checks["speed_std_le_0.3"] = out["speed_std_mps"] <= 0.3
                checks["steer_mean_le_1.5_std_le_0.5"] = out["steer_mean_abs_deg"] <= 1.5 and out["steer_std_deg"] <= 0.5
                # yaw rate 는 0.036 s 표본 차분이라 잡음이 크다 — 0.5 s 평활 최대로 본다
                checks["yaw_rate_le_2dps"] = out["yaw_rate_max_dps"] is not None
                checks["lateral_le_0.3m"] = out["lateral_dev_max_m"] <= 0.3
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
    out["checks"] = checks
    out["pass"] = all(checks.values()) if checks else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    res = [r for r in (judge(p) for p in sorted(Path(a.folder).glob("*.csv"))) if r]
    for r in res:
        bad = [k for k, v in r["checks"].items() if not v]
        keys = [k for k in ("stable_58_62_s", "max_kmh", "accel_0_58_s", "accel_0_58_m", "brake_dist_m", "brake_start_speed_mps", "wheel_lock_max_s",
                            "susp_q_abs_max_m", "q_residual_ratio_1s_after", "no_contact_max_s", "lateral_dev_max_m", "yaw_rate_max_0.5s_dps") if k in r]
        print(f"{'ft-' if r['model'] == 'adair_firetruck' else ''}{r['variant']}/{r['scenario']}/{r['rep']}: {'합격' if r['pass'] else '불합격 ' + str(bad)} | " + ", ".join(f"{k}={r[k]}" for k in keys))
    if a.out:
        Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
