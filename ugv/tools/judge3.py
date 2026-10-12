# -*- coding: utf-8 -*-
"""교차로 시험 3분할 판정 (실행 지시서 2~3단계 보완 2항): 임무 도착 / 도로 경계 준수 / 정지 확인 → 최종 합격 = 셋 모두.
- 도착: 임무 COMPLETED 이고 mission_result ARRIVED
- 경계: road_test 분석의 차체 둘레 vs 추정 도로 띠 합집합 이탈 이벤트 0 이고 body_outside_est_road_max_m ≤ 0 (기준 그대로)
- 정지: 임무 끝(서버 TASK_COMPLETED/FAILED 이벤트 벽시계 → Gazebo 기록 wall_t) 뒤 Gazebo 위치 속도 0.2 m/s 미만 1 s 연속을 확인하고,
        확인 뒤 10 s 이상 관측·재이동(0.3 m/s 이상) 없음. 기록이 그 전에 끝나면 '미확인'(관측 부족)
사용: python -m ugv.tools.judge3 <태그 접두어>... [--logs 폴더...] [--out 파일] → 표 + JSON
원본: logs/ugv/stage5/jturn/judge3.py (2026-10-12) 이식. 정지 단계 stop_phases 는 wt_fix ugv/tools/failure_timeline.py (5ffb41a) 와 같은 코드를 여기에 둔다
(이 브랜치에는 failure_timeline 이 없어서). 기준 변경 없음 — 회귀: judge3_c53.json 32건 일치.
"""
import argparse
import bisect
import csv
import glob
import json
import math
import os
from datetime import datetime
from pathlib import Path


G = Path(__file__).resolve().parents[3]   # 저장소 형제 작업 트리들이 있는 폴더




STOP_V = 0.2          # 정지 속도 기준 (서버 정지 감독과 같은 0.2 m/s)
STOP_HOLD_S = 1.0     # 이 시간 동안 모든 표본이 STOP_V 미만이어야 정지 확인 (서버 OFFROAD_CONFIRM_S/정차 확인 2 s 와 다를 수 있음 — 기록)
REMOVE_V = 0.3        # 정지 확인 뒤 이 속도 이상이면 재이동
OBSERVE_AFTER_S = 10.0  # 정지 확인 뒤 최소 관측 시간 (부족하면 '관측 부족')


def stop_phases(T, spd, rows, t_stop):
    """정지 명령 뒤 단계: 저속 시작 / 지속 확인 / 재이동 / 확인 뒤 관측 시간. 경계 표본도 연속 조건으로 본다
    (예전: 1 s 뒤 표본에서 속도가 다시 올라도 시각 차만 보고 정지로 인정 — 반례 [0, 0.5, 1.0] s / [0.1, 0.1, 1.0] m/s)."""
    k0 = next((i for i in range(len(T)) if T[i] >= t_stop), len(T))
    low_start = confirm = None
    i = k0
    while i < len(T):
        if spd[i] >= STOP_V:
            i += 1
            continue
        j = i
        while j < len(T) and spd[j] < STOP_V:
            j += 1
        # [i, j) 가 연속 저속 구간
        if low_start is None:
            low_start = T[i]
        if T[j - 1] - T[i] >= STOP_HOLD_S:
            confirm = T[i] + STOP_HOLD_S
            break
        i = j
    out = {"stop_low_start_sim_t": None if low_start is None else round(low_start, 2),
           "stop_confirm_sim_t": None if confirm is None else round(confirm, 2), "stop_done": confirm is not None,
           "stop_criteria": f"{STOP_V} m/s 미만 {STOP_HOLD_S} s 연속 (Gazebo 위치 차분)", "end_speed_mps": round(spd[-1], 2)}
    k1 = next((i for i in range(len(T)) if confirm is not None and T[i] >= confirm), len(T) - 1)
    out["stop_travel_m"] = round(sum(math.hypot(rows[i + 1]["x"] - rows[i]["x"], rows[i + 1]["y"] - rows[i]["y"])
                                     for i in range(k0, min(k1, len(rows) - 1))), 2)
    if confirm is not None:
        after = [i for i in range(len(T)) if T[i] >= confirm]
        out["observed_after_confirm_s"] = round(T[-1] - confirm, 2)
        out["observation_short"] = T[-1] - confirm < OBSERVE_AFTER_S
        rem = next((T[i] for i in after if spd[i] >= REMOVE_V), None)
        out["re_motion_sim_t"] = rem
        out["drift_after_confirm_m"] = round(math.hypot(rows[-1]["x"] - rows[k1]["x"], rows[-1]["y"] - rows[k1]["y"]), 2)
    return out




def judge(f):
    d = json.loads(Path(f).read_text(encoding="utf-8"))
    tasks = list((d.get("tasks") or {}).values())
    tk = tasks[-1] if tasks else {}
    rd = d["analysis"]["overall"]["road_dev"]
    arrived = tk.get("status") == "COMPLETED" and tk.get("mission_result") == "ARRIVED"
    ev = rd["events"]["count"]
    ex = rd.get("body_outside_est_road_max_m")
    inside = ev == 0 and (ex is not None and ex <= 0)
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(Path(f).with_suffix(".csv"), encoding="utf-8"))]
    T = [r["sim_t"] for r in rows]
    walls = [r["wall_t"] for r in rows]
    spd = []
    for i in range(len(rows)):
        j0, j1 = max(i - 5, 0), min(i + 5, len(rows) - 1)
        dt = T[j1] - T[j0]
        spd.append(math.hypot(rows[j1]["x"] - rows[j0]["x"], rows[j1]["y"] - rows[j0]["y"]) / dt if dt > 0 else 0.0)
    end_ev = [e for e in d.get("events", []) if e.get("event") in ("TASK_COMPLETED", "TASK_FAILED", "TASK_CANCELLED") and e.get("timestamp")]
    stop = {"stop_done": None}
    if end_ev:
        w = datetime.fromisoformat(end_ev[-1]["timestamp"]).timestamp()
        k = min(max(bisect.bisect_left(walls, w), 0), len(rows) - 1)
        stop = stop_phases(T, spd, rows, T[k])
    if not stop.get("stop_done"):
        st = "미확인(정지 확인 전 기록 끝)" if end_ev else "미확인(임무 끝 이벤트 없음)"
    elif stop.get("observation_short"):
        st = f"미확인(확인 뒤 {stop.get('observed_after_confirm_s')} s 관측 < 10 s)"
    elif stop.get("re_motion_sim_t") is not None:
        st = "불합격(재이동)"
    else:
        st = "확인"
    worst = None
    if rd["events"]["count"]:
        e0 = max(rd["events"]["list"], key=lambda e: e["max_excess_m"])
        worst = {k2: e0.get(k2) for k2 in ("start_sim_t", "duration_s", "max_excess_m", "x", "y", "speed_mps", "road", "cause")}
    return {"log": os.path.basename(f), "arrived": arrived, "status": tk.get("status"), "error": tk.get("error"),
            "inside": inside, "boundary_events": ev, "outside_max_m": ex, "worst_event": worst,
            "stop": st, "stop_detail": {k2: stop.get(k2) for k2 in ("stop_confirm_sim_t", "observed_after_confirm_s", "re_motion_sim_t")},
            "final_pass": arrived and inside and st == "확인",
            "final_pass_except_stop_unverified": arrived and inside and st.startswith("미확인")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix", nargs="+")
    ap.add_argument("--logs", nargs="+", default=[str(G / "ADAIR_wt_turn/logs/ugv"), str(G / "ADAIR_wt_fix/logs/ugv")])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    res = []
    for p in a.prefix:
        for L in a.logs:
            for f in sorted(glob.glob(os.path.join(L, p + "*.json"))):
                res.append(judge(f))
    for r in res:
        print(f"{r['log'][:44]:44s} 도착 {'O' if r['arrived'] else 'X'} | 경계 {'O' if r['inside'] else 'X'} ({r['outside_max_m']}) | 정지 {r['stop']:28s} | 최종 {'합격' if r['final_pass'] else ('정지 미확인' if r['final_pass_except_stop_unverified'] else '불합격')}")
    if a.out:
        Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
