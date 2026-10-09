# -*- coding: utf-8 -*-
"""ugv/tools/road_test.py — UGV 서버(PX4 드라이버) 도로 주행 시험 (3단계).

총괄과 같은 순서로 부른다: POST evaluate → (평가의 target_node 로) POST execute → GET task 조회.
  --second LAT LON  주행 중 새 목표 → 이전 임무 CANCELLED(SUPERSEDED), 새 경로로 바뀌는지 (취소·재계획)
  --abort-after S   출발 S 초 뒤 임무 중단(abort) → 앞쪽에서 멈추는지, 되돌아가지 않는지 (정지)
결과는 Gazebo 자세(gz_track.py)로만 판정한다:
  - 도착: 마지막 미션 지점까지 거리
  - 곡선 감속: 계획 속도가 움푹 낮아지는 미션 지점(곡선 감속 지점)을 지날 때 실제 속도 vs 계획 구간 속도,
    그리고 실측 횡가속(속도 × 회전율)이 계획 횡가속 config.LAT_ACCEL_MPS2 를 얼마나 넘는지
  - 도로 이탈: 차체 네 모서리(4.2 × 1.8 m)가 근처 도로 띠(중심선 ± 추정 반폭, 차로 수 기준)의 합집합 밖으로
    나갔는지. 차량 중심만 보지 않는다. 참고로 월드 도로 면 폭 10 m(반폭 5 m) 기준도 같이 센다

    python -m ugv.tools.road_test --target 38.0275 128.13328
    python -m ugv.tools.road_test --target 38.0275 128.13328 --second 38.045 128.19 --second-after 60
    python -m ugv.tools.road_test --target 38.0275 128.13328 --abort-after 40
    python -m ugv.tools.road_test --analyze logs/ugv/road_xxx.json      # 기록만 다시 분석
"""

import argparse
import csv
import json
import math
import time
from pathlib import Path

import httpx

from ugv import config
from ugv.geo import FRAME
from ugv.roads import RoadNetwork
from ugv.tools.drive_test import LOG_DIR, analyze, load_track, start_tracker

ROAD_HALF_WIDTH_M = 5.0          # 월드 도로 면 (road_network.json world.road_width_m = 10)
# 주 판정: 도로별 추정 반폭 (drive_graph.est_half_width: 원자료 차로 수 × 3.25 m / 2 + 길어깨 0.5 m — 추정치)
BODY_HALF_LEN_M, BODY_HALF_WID_M = 2.1, 0.9      # ugv/gazebo/models/adair_ugv 차체 상자 4.2 × 1.8 m
CURVE_NEAR_M = 20.0              # 곡선 감속 지점 앞뒤 이 거리 안의 최대 횡가속을 본다
CURVE_TOL_MPS = 1.0              # 꼭짓점 통과 속도가 계획보다 이만큼 넘게 빠르면 곡선 감속 실패


def last_sim_t(path: Path):
    """기록기가 쓰는 중인 CSV 의 마지막 시뮬레이션 시각 (명령 시점 표시용)."""
    try:
        with open(path, newline="") as f:
            last = None
            for r in csv.DictReader(f):
                last = r
        return float(last["sim_t"]) if last else None
    except (OSError, ValueError, KeyError, TypeError):
        return None


def speeds(rows, half_s=0.25):
    t = [r["sim_t"] for r in rows]
    out, j0, j1 = [], 0, 0
    for i in range(len(rows)):
        while t[i] - t[j0] > half_s:
            j0 += 1
        j1 = max(j1, i)
        while j1 + 1 < len(rows) and t[j1 + 1] - t[i] <= half_s:
            j1 += 1
        dt = t[j1] - t[j0]
        out.append(math.hypot(rows[j1]["x"] - rows[j0]["x"], rows[j1]["y"] - rows[j0]["y"]) / dt if dt > 0 else 0.0)
    return out


def lateral_accel(rows, spd, half_s=0.25):
    """실측 횡가속 = 속도 × 회전율 (Gazebo 자세 yaw 차분)."""
    t = [r["sim_t"] for r in rows]
    out, j0, j1 = [], 0, 0
    for i in range(len(rows)):
        while t[i] - t[j0] > half_s:
            j0 += 1
        j1 = max(j1, i)
        while j1 + 1 < len(rows) and t[j1 + 1] - t[i] <= half_s:
            j1 += 1
        dt = t[j1] - t[j0]
        dyaw = (rows[j1]["yaw"] - rows[j0]["yaw"] + math.pi) % (2 * math.pi) - math.pi
        out.append(spd[i] * dyaw / dt if dt > 0 else 0.0)
    return out


def footprint_dev(net, r):
    """차체 네 모서리: (가장 가까운 도로 중심선 거리 최댓값, 추정 도로 띠 밖으로 나간 거리 최댓값).
    도로 띠 = 근처 모든 도로의 중심선 ± 추정 반폭의 합집합 (교차로에서는 어느 도로 띠 안이든 안). 0 이하면 도로 안."""
    c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
    worst, excess = 0.0, -math.inf
    g = net.graph
    for fx, fy in ((BODY_HALF_LEN_M, BODY_HALF_WID_M), (BODY_HALF_LEN_M, -BODY_HALF_WID_M),
                   (-BODY_HALF_LEN_M, BODY_HALF_WID_M), (-BODY_HALF_LEN_M, -BODY_HALF_WID_M)):
        x, y = r["x"] + fx * c - fy * s, r["y"] + fx * s + fy * c
        p = net.snap(x, y)
        worst = max(worst, p.dist if p is not None else math.inf)
        excess = max(excess, g.outside_m(x, y))
    return worst, excess


def _poly_dist(x, y, poly):
    best, bi = math.inf, 0
    for i, ((ax, ay), (bx, by)) in enumerate(zip(poly, poly[1:])):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        d = math.hypot(x - ax - t * dx, y - ay - t * dy)
        if d < best:
            best, bi = d, i
    return best, bi


def _body_on_path(net, p, yaw):
    """뒤축이 경로 점 p 에 있고 경로 방향 yaw 로 서 있는 차체가 추정 도로 띠 밖으로 나간 거리 (0 이하면 안)."""
    g = net.graph
    cx, cy = p[0] + 1.35 * math.cos(yaw), p[1] + 1.35 * math.sin(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    return max(g.outside_m(cx + fx * c - fy * s, cy + fx * s + fy * c)
               for fx, fy in ((BODY_HALF_LEN_M, BODY_HALF_WID_M), (BODY_HALF_LEN_M, -BODY_HALF_WID_M),
                              (-BODY_HALF_LEN_M, BODY_HALF_WID_M), (-BODY_HALF_LEN_M, -BODY_HALF_WID_M)))


def plan_body_check(net, poly, step=1.0):
    """계획 경로(PX4 에 준 선: 명령 시점 위치 + 미션 지점) 자체가 차체를 포함해 도로 안인지. 1 m 간격."""
    out = []
    for (ax, ay), (bx, by) in zip(poly, poly[1:]):
        L = math.hypot(bx - ax, by - ay)
        if L < 1e-6:
            continue
        yaw = math.atan2(by - ay, bx - ax)
        n = max(1, int(L / step))
        for k in range(n):
            q = (ax + (bx - ax) * k / n, ay + (by - ay) * k / n)
            out.append((q, _body_on_path(net, q, yaw)))
    bad = [(q, e) for q, e in out if e > 0]
    worst = max(out, key=lambda o: o[1]) if out else None
    return {"samples": len(out), "outside_samples": len(bad),
            "max_excess_m": round(worst[1], 2) if worst else None,
            "worst_xy": [round(worst[0][0], 1), round(worst[0][1], 1)] if worst else None}


def offroad_events(net, rows, idx, excess, spd, plans):
    """연속 이탈 사건 (0.2 s 표본이 이어서 도로 밖. 0.3 s 넘게 끊기면 다른 사건). 사건마다 원인을 가른다:
    PLAN_OUTSIDE   계획 경로(차체 포함)가 그 자리에서 이미 도로 밖
    PX4_TRACKING   계획은 도로 안인데 차량 중심이 계획선에서 0.5 m 넘게 벗어남
    JUDGEMENT      계획 안·추종 0.5 m 안인데도 밖 — 도로 폭 추정(차로 수)·판정 근사 문제 후보"""
    evs, cur = [], None
    for k, i in enumerate(idx):
        if excess[k] > 0:
            if cur and rows[i]["sim_t"] - rows[cur["last"]]["sim_t"] <= 0.3:
                cur["last"] = i
                if excess[k] > cur["max"]:
                    cur["max"], cur["at"] = excess[k], i
            else:
                cur = {"first": i, "last": i, "max": excess[k], "at": i}
                evs.append(cur)
    out = []
    for e in evs:
        r = rows[e["at"]]
        t = r["sim_t"]
        poly = None
        for p in plans:
            if p["t0"] <= t and (p["t1"] is None or t < p["t1"]):
                poly = p["poly"]
        cross, plan_ex = None, None
        if poly:
            cross, j = _poly_dist(r["x"], r["y"], poly)
            (ax, ay), (bx, by) = poly[j], poly[j + 1]
            yaw = math.atan2(by - ay, bx - ax)
            L2 = (bx - ax) ** 2 + (by - ay) ** 2
            tt = 0.0 if L2 == 0 else max(0.0, min(1.0, ((r["x"] - ax) * (bx - ax) + (r["y"] - ay) * (by - ay)) / L2))
            q = (ax + tt * (bx - ax), ay + tt * (by - ay))
            # 차량 중심의 가장 가까운 계획점 → 뒤축 위치는 1.35 m 뒤
            plan_ex = max(_body_on_path(net, (q[0] - 1.35 * math.cos(yaw) + dd * math.cos(yaw),
                                              q[1] - 1.35 * math.sin(yaw) + dd * math.sin(yaw)), yaw) for dd in (-1.0, 0.0, 1.0))
        if plan_ex is not None and plan_ex > 0:
            cause = "PLAN_OUTSIDE"
        elif cross is not None and cross > 0.5:
            cause = "PX4_TRACKING"
        else:
            cause = "JUDGEMENT"
        rd = net.snap(r["x"], r["y"])
        out.append({"start_sim_t": rows[e["first"]]["sim_t"],
                    "duration_s": round(rows[e["last"]]["sim_t"] - rows[e["first"]]["sim_t"] + 0.2, 1),
                    "max_excess_m": round(e["max"], 2), "x": round(r["x"], 1), "y": round(r["y"], 1),
                    "lat_lon": [round(v, 6) for v in FRAME.to_ll(r["x"], r["y"])],
                    "speed_mps": round(spd[e["at"]], 2), "cross_track_m": None if cross is None else round(cross, 2),
                    "plan_body_excess_m": None if plan_ex is None else round(plan_ex, 2), "cause": cause,
                    "road": rd.road_id, "lanes": net.roads[rd.road_id].attrs.get("lanes"),
                    "est_half_width_m": round(net.graph.half_w[rd.road_id], 2)})
    return out


def evaluate_log(log: dict, rows) -> dict:
    net = RoadNetwork()
    spd = speeds(rows)
    alat = lateral_accel(rows, spd)
    t = [r["sim_t"] for r in rows]
    overall = analyze(rows, None)
    overall.pop("final_decel", None)          # 직선 시험용 (최고속도 → 정지). 도로 경로에서는 뜻이 없다
    # 도로 이탈: 0.2 s 간격 표본
    idx, last = [], -1e9
    for i, ti in enumerate(t):
        if ti - last >= 0.2:
            idx.append(i); last = ti
    center = [net.snap(rows[i]["x"], rows[i]["y"]).dist for i in idx]
    fp = [footprint_dev(net, rows[i]) for i in idx]
    body = [f[0] for f in fp]
    excess = [f[1] for f in fp]
    moving = [k for k, i in enumerate(idx) if spd[i] > 1.0]
    def stat(v):
        s = sorted(v)
        return {"mean": round(sum(s) / len(s), 2), "p95": round(s[int(0.95 * (len(s) - 1))], 2), "max": round(s[-1], 2)} if s else None
    worst_k = max(range(len(idx)), key=lambda k: excess[k]) if idx else None
    overall["road_dev"] = {
        "samples": len(idx), "moving_samples": len(moving),
        "center_to_centerline_m": stat([center[k] for k in moving]),
        "body_corner_to_centerline_m": stat([body[k] for k in moving]),
        "body_outside_est_road(samples)": sum(1 for k in range(len(idx)) if excess[k] > 0),
        "body_outside_est_road_max_m": round(max(excess), 2) if idx else None,
        f"body_outside_world_surface(>{ROAD_HALF_WIDTH_M:g}m)": sum(1 for k in range(len(idx)) if body[k] > ROAD_HALF_WIDTH_M),
        "basis": "차체 네 모서리 vs 근처 도로 띠 합집합 (중심선 ± 추정 반폭: 차로 수 × 3.25 m / 2 + 0.5 m)",
        "worst": None if worst_k is None else {"sim_t": t[idx[worst_k]], "x": round(rows[idx[worst_k]]["x"], 1),
                                               "y": round(rows[idx[worst_k]]["y"], 1), "body_m": round(body[worst_k], 2),
                                               "excess_m": round(excess[worst_k], 2), "speed_mps": round(spd[idx[worst_k]], 2)},
    }
    out = {"overall": overall, "runs": {}}
    runs = log["runs"]
    # PX4 에 준 계획선 = 명령 시점 차량 위치 + 미션 지점
    plans = []
    for n, run in enumerate(runs):
        items = (run.get("route") or {}).get("items") or []
        if not items:
            continue
        t0 = run.get("sim_t_cmd") or t[0]
        i0 = next((i for i in range(len(rows)) if t[i] >= t0), 0)
        poly = [(rows[i0]["x"], rows[i0]["y"])] + [FRAME.to_xy(it["lat"], it["lon"]) for it in items]
        plans.append({"key": run["key"], "t0": t0, "t1": runs[n + 1].get("sim_t_cmd") if n + 1 < len(runs) else None,
                      "poly": poly})
    evs = offroad_events(net, rows, idx, excess, spd, plans)
    overall["road_dev"]["events"] = {
        "count": len(evs), "longest_s": max((e["duration_s"] for e in evs), default=0.0),
        "max_excess_m": max((e["max_excess_m"] for e in evs), default=None),
        "by_cause": {c: sum(1 for e in evs if e["cause"] == c) for c in ("PLAN_OUTSIDE", "PX4_TRACKING", "JUDGEMENT")},
        "list": evs}
    for n, run in enumerate(runs):
        items = (run.get("route") or {}).get("items") or []
        if not items:
            continue
        t0 = run.get("sim_t_cmd") or t[0]
        t1 = runs[n + 1].get("sim_t_cmd") if n + 1 < len(runs) else None
        seg = [i for i in range(len(rows)) if t[i] >= t0 and (t1 is None or t[i] < t1)]
        xy = [FRAME.to_xy(i["lat"], i["lon"]) for i in items]
        res = {"items": len(items), "eta_sec": run["route"].get("eta_sec"), "length_m": run["route"].get("length_m"),
               "sim_t_cmd": t0, "samples": len(seg)}
        pl = next((p for p in plans if p["key"] == run["key"]), None)
        if pl:
            res["plan_body_check"] = plan_body_check(net, pl["poly"])
        if seg:
            res["duration_s"] = round(t[seg[-1]] - t[seg[0]], 1)
        # 곡선 감속: 계획 속도가 움푹 낮아지는 지점(앞뒤 지점보다 느리고 최고속도보다 낮음)을 지날 때의 속도와
        # 그 앞뒤 최대 횡가속. 출발 가속 구간(첫 지점)과 도착 감속 구간(끝까지 계속 느려지는 지점)은 뺀다
        v_it = [i["speed_mps"] for i in items]
        curves = []
        for k in range(1, len(xy) - 1):
            if not (v_it[k] < config.MAX_SPEED_MPS - 0.5 and v_it[k] <= v_it[k - 1] and v_it[k] <= v_it[k + 1]):
                continue
            if all(v_it[j] <= v_it[k] for j in range(k + 1, len(v_it))):
                continue
            near = [i for i in seg if math.dist((rows[i]["x"], rows[i]["y"]), xy[k]) <= CURVE_NEAR_M]
            if not near:
                curves.append({"item": k, "reached": False})
                continue
            apex = min(near, key=lambda i: math.dist((rows[i]["x"], rows[i]["y"]), xy[k]))
            plan_v = v_it[k]
            curves.append({"item": k, "planned_mps": plan_v, "apex_mps": round(spd[apex], 2),
                           "over_mps": round(spd[apex] - plan_v, 2),
                           "max_lat_accel_mps2": round(max(abs(alat[i]) for i in near), 2),
                           "x": round(xy[k][0], 1), "y": round(xy[k][1], 1)})
        reached = [c for c in curves if c.get("reached", True)]
        la = sorted(abs(alat[i]) for i in seg if spd[i] > 1.0)
        res["curves"] = {"checked": len(reached), "not_reached": len(curves) - len(reached),
                         f"apex_over_plan(>{CURVE_TOL_MPS:g}m/s)": [c for c in reached if c["over_mps"] > CURVE_TOL_MPS],
                         "worst_over_mps": max((c["over_mps"] for c in reached), default=None),
                         "max_lat_accel_at_curves_mps2": max((c["max_lat_accel_mps2"] for c in reached), default=None),
                         "all": reached}
        res["lat_accel_mps2"] = {"plan": config.LAT_ACCEL_MPS2,
                                 "p95": round(la[int(0.95 * (len(la) - 1))], 2) if la else None,
                                 "p99": round(la[int(0.99 * (len(la) - 1))], 2) if la else None,
                                 "max": round(la[-1], 2) if la else None,
                                 "samples_over_plan+1": sum(1 for v in la if v > config.LAT_ACCEL_MPS2 + 1.0)}
        if seg:
            end = rows[seg[-1]]
            res["end_to_last_item_m"] = round(math.hypot(end["x"] - xy[-1][0], end["y"] - xy[-1][1]), 2)
            res["end_speed_mps"] = round(spd[seg[-1]], 2)
            res["max_speed_mps"] = round(max(spd[i] for i in seg), 2)
            res["max_abs_roll_deg"] = round(max(abs(math.degrees(rows[i]["roll"])) for i in seg), 2)
        out["runs"][run["key"]] = res
    ab = log.get("abort")
    if ab and ab.get("sim_t_cmd") is not None:
        i0 = next((i for i in range(len(rows)) if t[i] >= ab["sim_t_cmd"]), None)
        if i0 is not None:
            stop = next((i for i in range(i0, len(rows)) if spd[i] < 0.2), None)
            h0 = (math.cos(rows[i0]["yaw"]), math.sin(rows[i0]["yaw"]))
            after = rows[i0:stop + 1] if stop is not None else rows[i0:]
            back = min(((r["x"] - rows[i0]["x"]) * h0[0] + (r["y"] - rows[i0]["y"]) * h0[1] for r in after), default=0.0)
            ab_res = {"speed_at_cmd_mps": round(spd[i0], 2), "stopped": stop is not None,
                      "stop_time_s": round(t[stop] - t[i0], 2) if stop is not None else None,
                      "stop_distance_m": round(sum(math.hypot(rows[q + 1]["x"] - rows[q]["x"], rows[q + 1]["y"] - rows[q]["y"])
                                                   for q in range(i0, stop)), 1) if stop is not None else None,
                      "min_forward_m": round(back, 2),
                      "moved_after_stop_m": round(math.hypot(rows[-1]["x"] - rows[stop]["x"], rows[-1]["y"] - rows[stop]["y"]), 2)
                      if stop is not None else None}
            ab_res["went_backward_or_uturn"] = back < -1.0
            out["abort"] = ab_res
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="http://127.0.0.1:8100")
    ap.add_argument("--rid", default="A-ugv1")
    ap.add_argument("--target", nargs=2, type=float)
    ap.add_argument("--second", nargs=2, type=float)
    ap.add_argument("--second-after", type=float, default=60.0, help="첫 출발 뒤 몇 초(벽시계)에 새 목표")
    ap.add_argument("--abort-after", type=float, help="첫 출발 뒤 몇 초(벽시계)에 임무 중단")
    ap.add_argument("--track-s", type=float, default=900.0)
    ap.add_argument("--world", default=config.DEFAULT_VEHICLES[0]["px4"]["gz_world"])
    ap.add_argument("--analyze", type=Path, help="저장된 road_*.json 을 다시 분석")
    a = ap.parse_args()

    if a.analyze:
        log = json.loads(a.analyze.read_text(encoding="utf-8"))
        log["analysis"] = evaluate_log(log, load_track(a.analyze.with_suffix(".csv")))
        a.analyze.write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
        print(json.dumps(log["analysis"], ensure_ascii=False, indent=1))
        return
    if not a.target:
        ap.error("--target 또는 --analyze 필요")

    c = httpx.Client(base_url=a.server, timeout=30)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_csv = LOG_DIR / f"road_{stamp}.csv"
    tracker = start_tracker(out_csv, a.track_s, world=a.world)
    time.sleep(3)
    log = {"stamp": stamp, "world": a.world, "target": a.target, "second": a.second, "abort_after": a.abort_after,
           "runs": []}

    def run(key, lat, lon):
        ev = c.post(f"/ugv/{a.rid}/evaluate", json={"task_id": key, "decision_id": "D-" + key,
                                                   "target": {"lat": lat, "lon": lon}}).json()
        print(f"[{key}] evaluate {ev['verdict']} eta {ev.get('eta_sec')} 접근 {json.dumps(ev.get('approach', {}).get('mode'))} "
              f"목표까지 {ev.get('approach', {}).get('dist_to_target_m')} m 화재이격 {ev.get('approach', {}).get('fire_clearance_m')}")
        body = {"task_id": key, "decision_id": "D-" + key, "target": {"lat": lat, "lon": lon},
                "target_node": (ev.get("target_node") or {}).get("node_id")}
        sim_t = last_sim_t(out_csv)
        r = c.post(f"/ugv/{a.rid}/execute", json=body)
        print(f"[{key}] execute {r.status_code} {r.json().get('status')} superseded={r.json().get('superseded_task_id')}")
        dup = c.post(f"/ugv/{a.rid}/execute", json=body).json()
        print(f"[{key}] 같은 요청 재전송 → duplicate={dup.get('duplicate')}")
        route = c.get(f"/ugv/{a.rid}/route").json()
        log["runs"].append({"key": key, "evaluate": ev, "execute": r.json(), "route": route, "t_wall": time.time(),
                            "sim_t_cmd": sim_t})
        return key

    k1 = run(f"RT1-{stamp}", *a.target)
    t0 = time.time()
    active, second_sent, aborted, t_abort = k1, False, False, None
    while time.time() - t0 < a.track_s - 10:
        t = c.get(f"/ugv/{a.rid}/task/{active}").json()
        pr = t.get("progress") or {}
        print(f"  {time.time() - t0:6.1f}s {active[:6]} {t['status']:<11} {pr.get('phase')} 남은 {pr.get('remaining_m')} m "
              f"속도 {pr.get('speed_mps')} m/s 경로편차 {pr.get('off_route_m')} m", flush=True)
        if a.second and not second_sent and time.time() - t0 >= a.second_after:
            active, second_sent = run(f"RT2-{stamp}", *a.second), True
            continue
        if a.abort_after is not None and not aborted and time.time() - t0 >= a.abort_after:
            sim_t = last_sim_t(out_csv)
            r = c.post(f"/ugv/{a.rid}/task/{active}/abort", json={"reason": "ROAD_TEST_ABORT"})
            print(f"[{active}] abort {r.status_code} {r.json().get('status')} {r.json().get('error')}")
            log["abort"] = {"key": active, "sim_t_cmd": sim_t, "response": r.json()}
            aborted, t_abort = True, time.time()
            continue
        if aborted:
            st = c.get(f"/ugv/{a.rid}/state").json()
            if st.get("mission_state") == "IDLE" and (st.get("speed_mps") or 0) < 0.1 and time.time() - t_abort > 15:
                time.sleep(10)          # 정지 뒤 다시 움직이지 않는지 더 본다
                log["final"] = c.get(f"/ugv/{a.rid}/task/{active}").json()
                break
        elif t["status"] in ("COMPLETED", "FAILED", "CANCELLED") and (second_sent or not a.second):
            log["final"] = t
            break
        time.sleep(3)          # uvicorn 연결 유지 5 s 와 겹치지 않게
    time.sleep(5)
    log["state"] = c.get(f"/ugv/{a.rid}/state").json()
    log["events"] = c.get(f"/ugv/{a.rid}/events").json()
    log["tasks"] = {r["key"]: c.get(f"/ugv/{a.rid}/task/{r['key']}").json() for r in log["runs"]}
    print("기록기 종료 대기...")
    tracker.terminate()
    time.sleep(2)
    log["analysis"] = evaluate_log(log, load_track(out_csv))
    (LOG_DIR / f"road_{stamp}.json").write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(log["analysis"], ensure_ascii=False, indent=1))
    print("임무:", json.dumps({k: {f: v.get(f) for f in ("status", "mission_result", "physical_state", "error")}
                              for k, v in log["tasks"].items()}, ensure_ascii=False))


if __name__ == "__main__":
    main()
