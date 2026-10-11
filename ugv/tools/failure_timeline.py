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
        done = None
        for i in range(len(rows)):
            if T[i] < t_stop or spd[i] >= 0.2:
                continue
            k = i
            while k < len(rows) and spd[k] < 0.2 and T[k] - T[i] < 1.0:
                k += 1
            if k < len(rows) and T[k] - T[i] >= 1.0:
                done = T[i]
                break
        k_stop = next(i for i in range(len(rows)) if T[i] >= t_stop)
        out["stop_done_sim_t"] = done
        out["stop_done"] = done is not None
        out["stop_travel_m"] = (round(sum(math.hypot(rows[i + 1]["x"] - rows[i]["x"], rows[i + 1]["y"] - rows[i]["y"])
                                          for i in range(k_stop, (next(j for j in range(len(rows)) if T[j] >= done) if done else len(rows) - 1))), 1))
        out["end_speed_mps"] = round(spd[-1], 2)
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
