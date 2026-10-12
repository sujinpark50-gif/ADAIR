# -*- coding: utf-8 -*-
"""ugv/tools/failure_timeline.py — 주행 실패를 시간순 단계로 분리: 최초 이탈 → 미션 진행 정체 → 고장(FAULT) → 정지 명령 → 정지 완료.

    python -m ugv.tools.failure_timeline <road_test 기록 .json> [--ulog-json <ulog 덤프>] [--out timeline.json]

- 서버 이벤트(timestamp, 벽시계)와 Gazebo 기록(CSV wall_t)을 벽시계로 맞춘다. 서버 sim_time_s 와 CSV sim_t 는 기준이 달라 빼지 않는다.
- 차체 이탈량 = road_test.footprint_dev 와 같은 식 (차체 둘레 vs 추정 도로 띠 합집합, 0 이하면 도로 안). 기준 완화 없음.
- 최대 이탈을 FAULT 전·후로 따로 낸다 (정지 과정의 추가 이탈 구분).
- 정지 완료 = 정지 명령 뒤 속도(Gazebo 위치 차분) 0.2 m/s 미만이 1 s 이어진 첫 시각. 기록 끝까지 없으면 '정지 완료 미확인'.
- ulog 덤프(est·mres)가 있으면 PX4 현재 미션 순번(raw seq — 속도 명령 포함)별 활성 시간·목표 최소 거리 → 3 s 넘게 머문 순번 = 진행 정체 후보.
"""
import argparse
import bisect
import csv
import json
import math
from datetime import datetime
from pathlib import Path

from ugv.roads import RoadNetwork
from ugv.tools import road_test as RT


def _epoch(iso):
    return datetime.fromisoformat(iso).timestamp()


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    ap.add_argument("--ulog-json", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    d = json.loads(a.log.read_text(encoding="utf-8"))
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(a.log.with_suffix(".csv"), encoding="utf-8"))]
    net = RoadNetwork()
    walls = [r["wall_t"] for r in rows]

    def sim_at_wall(w):
        i = min(max(bisect.bisect_left(walls, w), 0), len(rows) - 1)
        return rows[i]["sim_t"]
    ex = [RT.footprint_dev(net, r)[1] for r in rows]
    spd = [0.0] * len(rows)
    for i in range(len(rows)):
        j0, j1 = max(i - 5, 0), min(i + 5, len(rows) - 1)
        dt = rows[j1]["sim_t"] - rows[j0]["sim_t"]
        spd[i] = math.hypot(rows[j1]["x"] - rows[j0]["x"], rows[j1]["y"] - rows[j0]["y"]) / dt if dt > 0 else 0.0
    evs = [e for e in d.get("events", []) if e.get("timestamp")]
    t_start = next((sim_at_wall(_epoch(e["timestamp"])) for e in evs if e["event"] == "TASK_STARTED"), rows[0]["sim_t"])
    t_fault = next((sim_at_wall(_epoch(e["timestamp"])) for e in evs if e["event"] == "FAULT"), None)
    stop_ev = next((e for e in evs if e["event"] == "STOP_PATH"), None)
    t_stop = sim_at_wall(_epoch(stop_ev["timestamp"])) if stop_ev else None
    T = [r["sim_t"] for r in rows]

    def first(cond, t0=-math.inf):
        return next((T[i] for i in range(len(rows)) if T[i] >= t0 and cond(i)), None)
    out = {"log": a.log.name, "task_started_sim_t": round(t_start, 2),
           "first_outside_sim_t": first(lambda i: ex[i] > 0.0, t_start),
           "first_outside_0p5_sim_t": first(lambda i: ex[i] > 0.5, t_start),
           "fault_sim_t": t_fault, "fault": next((e.get("detail") for e in evs if e["event"] == "FAULT"), None),
           "stop_cmd_sim_t": t_stop, "stop_basis": stop_ev.get("basis") if stop_ev else None,
           "stop_needed_m": stop_ev.get("needed_m") if stop_ev else None}
    pre = [(ex[i], T[i]) for i in range(len(rows)) if T[i] >= t_start and (t_fault is None or T[i] < t_fault)]
    post = [(ex[i], T[i], spd[i]) for i in range(len(rows)) if t_fault is not None and T[i] >= t_fault]
    if pre:
        m = max(pre)
        out["max_outside_before_fault"] = {"m": round(m[0], 2), "sim_t": round(m[1], 2)}
    if post:
        m = max(post)
        out["max_outside_after_fault"] = {"m": round(m[0], 2), "sim_t": round(m[1], 2), "speed_mps": round(m[2], 2),
                                          "after_stop_cmd_s": round(m[1] - t_stop, 2) if t_stop is not None else None}
    if t_stop is not None:
        out.update(stop_phases(T, spd, rows, t_stop))
    if a.ulog_json:
        U = json.loads(a.ulog_json.read_text(encoding="utf-8"))
        m = U["mres"]
        ts = [t / 1e6 for t in m["timestamp"]]
        # ulog 시각 ↔ sim_t: Gazebo 실제 위치(gt)와 CSV 위치를 맞추는 시각차 (road_test 기록과 같은 시뮬레이션 시계 — 보통 0 근처)
        from ugv.geo import FRAME
        gt = [(t,) + tuple(FRAME.to_xy(la, lo)) for t, la, lo in zip(U["gt"]["t"], U["gt"]["lat"], U["gt"]["lon"])]
        gts = [g[0] for g in gt]
        best = None
        for off10 in range(-100, 101):
            off = off10 / 10
            e = n = 0
            for r in rows[::25]:
                j = bisect.bisect_left(gts, r["sim_t"] + off)
                if 0 < j < len(gt):
                    e += (gt[j][1] - r["x"]) ** 2 + (gt[j][2] - r["y"]) ** 2
                    n += 1
            if n and (best is None or e / n < best[0]):
                best = (e / n, off)
        off = best[1]
        segs = []
        for k in range(len(ts)):
            t0 = ts[k] - off
            t1 = (ts[k + 1] - off) if k + 1 < len(ts) else T[-1]
            if t1 - t0 >= 3.0 and m["seq_total"][k] > 0 and t0 >= t_start - 1:
                segs.append({"raw_seq": int(m["seq_current"][k]), "from_sim_t": round(t0, 2), "to_sim_t": round(t1, 2),
                             "active_s": round(t1 - t0, 2)})
        out["ulog_time_offset_s"] = off
        out["mission_stall_candidates(>=3s)"] = segs
    js = json.dumps(out, ensure_ascii=False, indent=1)
    if a.out:
        a.out.write_text(js, encoding="utf-8")
    print(js)


if __name__ == "__main__":
    main()
