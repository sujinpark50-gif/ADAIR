# -*- coding: utf-8 -*-
"""ugv/tools/offroad_cause.py — Gazebo 궤적의 도로 이탈 구간을 계획 경로와 비교해 원인을 나눈다.

    python -m ugv.tools.offroad_cause --track logs/ugv/xxx.csv --profile adair_firetruck --plan plan.json [--window t0 t1]

입력: 궤적 CSV (gz_track: sim_t, x, y, yaw, … — 모델 원점 = 차체 중심), 계획 선형 JSON {"pts": [[x, y], …]} (뒤축 경로, 지역 평면).
판정 표본마다:
  body_excess_actual  실제 자세의 차체 둘레가 추정 도로 띠 밖으로 나간 거리 (도로 이탈 판정과 같은 기준)
  cross_track_m       실제 뒤축 위치 ~ 계획 선형 거리 (PX4 추종 오차)
  plan_excess_m       계획 선형 위 가장 가까운 점에 계획 방향으로 섰을 때의 차체 초과 (계획 형상 여유)
분류: PLAN_OUTSIDE (plan_excess > 0) / PX4_TRACKING (계획은 안, 추종 오차 > 0.3 m) / MARGINAL (계획 여유 0.3 m 미만 + 추종 오차 0.3 m 이하
— 둘이 겹친 경계) / OTHER. 도로 폭·판정 기준은 drive_graph 와 같다 (바꾸지 않는다).
"""

import argparse
import csv
import json
import math
from pathlib import Path

from ugv.profiles import get as get_profile
from ugv.roads import RoadNetwork


def load_rows(p):
    with open(p, newline="") as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def nearest_on(pts, x, y, lo=0, hi=None):
    best = (math.inf, 0, 0.0)
    hi = len(pts) - 1 if hi is None else hi
    for i in range(max(0, lo), min(hi, len(pts) - 1)):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        d = math.hypot(x - ax - t * dx, y - ay - t * dy)
        if d < best[0]:
            best = (d, i, t)
    return best


def analyze(rows, plan, profile, window=None, step_s=0.2):
    net = RoadNetwork()
    prof = get_profile(profile)
    g = net.graph_for(prof)
    rac = prof.rear_axle_from_center_m
    out, last = [], -1e9
    for r in rows:
        if window and not (window[0] <= r["sim_t"] <= window[1]):
            continue
        if r["sim_t"] - last < step_s:
            continue
        last = r["sim_t"]
        c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
        rx, ry = r["x"] - rac * c, r["y"] - rac * s                    # 뒤축 위치
        ex = g.body_excess(rx, ry, r["yaw"])
        rec = {"sim_t": round(r["sim_t"], 2), "x": round(rx, 2), "y": round(ry, 2), "body_excess_actual": round(ex, 3)}
        if plan:
            d, i, t = nearest_on(plan, rx, ry)
            (ax, ay), (bx, by) = plan[i], plan[i + 1]
            qx, qy = ax + t * (bx - ax), ay + t * (by - ay)
            pyaw = math.atan2(by - ay, bx - ax)
            rec.update({"cross_track_m": round(d, 3), "plan_excess_m": round(g.body_excess(qx, qy, pyaw), 3),
                        "heading_err_deg": round(math.degrees((r["yaw"] - pyaw + math.pi) % (2 * math.pi) - math.pi), 1)})
        out.append(rec)
    return out


def events(samples, plan_given=True):
    evs, cur = [], None
    for s in samples:
        if s["body_excess_actual"] > 0:
            if cur and s["sim_t"] - cur["last"] <= 0.3:
                cur["last"] = s["sim_t"]
                if s["body_excess_actual"] > cur["worst"]["body_excess_actual"]:
                    cur["worst"] = s
            else:
                cur = {"first": s["sim_t"], "last": s["sim_t"], "worst": s}
                evs.append(cur)
    res = []
    for e in evs:
        w = e["worst"]
        cause = "OTHER"
        if plan_given:
            if w["plan_excess_m"] > 0:
                cause = "PLAN_OUTSIDE"
            elif w["cross_track_m"] > 0.3:
                cause = "PX4_TRACKING"
            elif w["plan_excess_m"] > -0.3:
                cause = "MARGINAL"
        res.append({"first": e["first"], "duration_s": round(e["last"] - e["first"] + 0.2, 1), "cause": cause, **w})
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", type=Path, required=True)
    ap.add_argument("--profile", default="adair_ugv")
    ap.add_argument("--plan", type=Path)
    ap.add_argument("--window", nargs=2, type=float)
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    plan = json.loads(a.plan.read_text(encoding="utf-8"))["pts"] if a.plan else None
    smp = analyze(load_rows(a.track), plan, a.profile, a.window)
    ev = events(smp, plan is not None)
    res = {"track": str(a.track), "profile": a.profile, "samples": len(smp), "events": ev,
           "max_cross_track_m": max((s.get("cross_track_m", 0) for s in smp), default=None),
           "min_plan_margin_m": min((s.get("plan_excess_m", 0) for s in smp), default=None)}
    if a.out:
        a.out.write_text(json.dumps({**res, "samples_list": smp}, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
