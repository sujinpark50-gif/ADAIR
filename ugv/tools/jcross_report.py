# -*- coding: utf-8 -*-
"""ugv/tools/jcross_report.py — 교차 비교(chain58: j494943 base/susp × D5/D8) 분석 (후속 계획 S1 둘째 항목).

    python -m ugv.tools.jcross_report --detail-dir <logs/ugv/stage6/jcross> [--prefix c58_j494943] [--out report.json]

회차마다 (road_test 결과 JSON 의 마지막 전송 미션 runs[0].route.last_sent_mission.items + gz_detail 상세 기록):
 - 전송 위치 항목별 Gazebo 실제 궤적 최근접 거리 (특히 놓친 항목) — NAV_ACC_RAD 3 m 대비
 - 회전 구간(조향 |δ| > 10°) 의 조향각 평균·최대, 실측 yaw rate(0.2 s 창) 평균·최대, 유효 회전 반경 v/ω 중앙값
 - 바퀴 접촉: 회전 구간에서 네 바퀴 모두 VALID 인 표본 비율, 바퀴별 NO_CONTACT 비율, 바퀴 법선력 최소/평균, 대각 하중 지수 (FL+RR−FR−RL)/Σ 범위
 - PX4 요구 yaw rate 는 이 기록에 없다 (ULog 필요) — '미수집' 으로 둔다.
서스펜션 후보(susp_a)와 원본(base)의 계측 플러그인 차이: base 는 서스펜션 관절이 없고, 둘 다 접촉 센서 100 Hz·관절 발행(susp_a 는 서스펜션 관절 4개 추가).
"""
import argparse
import csv
import glob
import json
import math
import re
import statistics
from pathlib import Path

G = Path(__file__).resolve().parents[3]
W = ["wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"]


def f(v):
    try:
        x = float(v)
        return None if math.isnan(x) else x
    except (TypeError, ValueError):
        return None


def analyze(detail_csv, result_json):
    rows = list(csv.DictReader(open(detail_csv, encoding="utf-8")))
    d = json.loads(Path(result_json).read_text(encoding="utf-8"))
    items = (((d.get("runs") or [{}])[0].get("route") or {}).get("last_sent_mission") or {}).get("items") or []
    task = list((d.get("tasks") or {}).values())[-1] if d.get("tasks") else {}
    T = [float(r["sim_t"]) for r in rows]
    X = [float(r["x"]) for r in rows]
    Y = [float(r["y"]) for r in rows]
    yaw = [float(r["yaw"]) for r in rows]
    near = [round(min(math.hypot(x - it[0], y - it[1]) for x, y in zip(X, Y)), 2) for it in items]
    spd, yr = [], []
    for i in range(len(rows)):
        j0, j1 = i, i
        while j0 > 0 and T[i] - T[j0] < 0.1:
            j0 -= 1
        while j1 < len(rows) - 1 and T[j1] - T[i] < 0.1:
            j1 += 1
        dt = T[j1] - T[j0]
        spd.append(math.hypot(X[j1] - X[j0], Y[j1] - Y[j0]) / dt if dt > 0 else 0.0)
        yr.append(((yaw[j1] - yaw[j0] + math.pi) % (2 * math.pi) - math.pi) / dt if dt > 0 else 0.0)
    steer = [math.degrees((f(r["steer_l"]) or 0) + (f(r["steer_r"]) or 0)) / 2 for r in rows]
    turn = [i for i in range(len(rows)) if abs(steer[i]) > 10 and spd[i] > 0.5]
    out = {"detail": Path(detail_csv).name, "status": task.get("status"), "mission_result": task.get("mission_result"),
           "item_nearest_m": near, "items_missed_over_3m": [k for k, v in enumerate(near) if v > 3.0], "turn_samples": len(turn)}
    if turn:
        out.update({"steer_abs_mean_deg": round(statistics.mean(abs(steer[i]) for i in turn), 2), "steer_abs_max_deg": round(max(abs(steer[i]) for i in turn), 2),
                    "yaw_rate_abs_mean_dps": round(math.degrees(statistics.mean(abs(yr[i]) for i in turn)), 2),
                    "yaw_rate_abs_max_dps": round(math.degrees(max(abs(yr[i]) for i in turn)), 2),
                    "turn_radius_median_m": round(statistics.median(spd[i] / abs(yr[i]) for i in turn if abs(yr[i]) > 1e-3), 2)})
        all4 = [i for i in turn if all(rows[i][f"{w}_c_state"] == "VALID" for w in W)]
        out["turn_all4_valid_ratio"] = round(len(all4) / len(turn), 3)
        out["turn_no_contact_ratio"] = {w: round(sum(rows[i][f"{w}_c_state"] == "NO_CONTACT" for i in turn) / len(turn), 3) for w in W}
        fns = {w: [f(rows[i][f"{w}_fn"]) for i in turn if rows[i][f"{w}_c_state"] == "VALID"] for w in W}
        out["turn_fn_min_N"] = {w: (round(min(v), 0) if v else None) for w, v in fns.items()}
        out["turn_fn_mean_N"] = {w: (round(statistics.mean(v), 0) if v else None) for w, v in fns.items()}
        diag = [((f(rows[i]["wheel_front_left_fn"]) + f(rows[i]["wheel_rear_right_fn"]) - f(rows[i]["wheel_front_right_fn"]) - f(rows[i]["wheel_rear_left_fn"]))
                 / sum(f(rows[i][f"{w}_fn"]) for w in W)) for i in all4]
        out["turn_diag_index_range"] = [round(min(diag), 3), round(max(diag), 3)] if diag else None
    out["px4_yaw_rate_setpoint"] = "미수집 (ULog 필요)"
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detail-dir", default=str(G / "ADAIR/logs/ugv/stage6/jcross"))
    ap.add_argument("--prefix", default="c58_j494943")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    res = []
    for dc in sorted(glob.glob(str(Path(a.detail_dir) / f"{a.prefix}*_detail.csv"))):
        tag = Path(dc).name[:-len("_detail.csv")]
        js = sorted(glob.glob(str(G / "ADAIR_wt_fix/logs/ugv" / f"{tag}_2*.json")))
        if not js:
            res.append({"tag": tag, "error": "결과 JSON 없음"})
            continue
        r = analyze(dc, js[-1])
        m = re.search(r"_(D5|D8)_(base|susp_a)_([abc])$", tag)
        r.update({"tag": tag, "surface": m.group(1), "variant": m.group(2), "rep": m.group(3)})
        res.append(r)
    for r in res:
        if "error" in r:
            print(r["tag"], r["error"])
            continue
        print(f"{r['variant']:6s} {r['surface']} {r['rep']}: {r['mission_result']:11s} 항목최근접 {r['item_nearest_m']} 놓침 {r['items_missed_over_3m']} | "
              f"조향 평균/최대 {r.get('steer_abs_mean_deg')}/{r.get('steer_abs_max_deg')}° yaw {r.get('yaw_rate_abs_mean_dps')}/{r.get('yaw_rate_abs_max_dps')}°/s "
              f"R중앙 {r.get('turn_radius_median_m')} m | 4륜접촉 {r.get('turn_all4_valid_ratio')} 대각지수 {r.get('turn_diag_index_range')}")
    if a.out:
        Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
