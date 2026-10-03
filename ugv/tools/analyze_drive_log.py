# ugv/tools/analyze_drive_log.py — 주행 정밀 기록(ugv/drive_log.py) 요약
#
#   python3 -m ugv.tools.analyze_drive_log ugv/.state/drive_logs/T20.csv
#   python3 -m ugv.tools.analyze_drive_log ugv/.state/drive_logs/*.csv     # 여러 개면 각각 + 합계
#   python3 -m ugv.tools.analyze_drive_log --roads ugv/.state/drive_logs/T20.csv  # 도로별 치우침까지
#
# 보는 것
#   시간  : 벽시계 vs PX4 시각 → 실제 배속(RTF). 서버 시뮬 시각과 PX4 시각의 어긋남.
#           ETA(출발 시 계산) vs 실제(PX4 초)
#   횡오차: 경로선 기준 + = 왼쪽, - = 오른쪽. 평균이 한쪽으로 쏠리면 '중심에서 치우침'.
#           |오차| 이 3 m (10 m 도로의 차선 반 폭 근처)를 넘는 시간 비율
#   속도  : 실제 / 명령 (움직이는 구간만)

import argparse
import csv
import json
import os
import statistics as st
import sys


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * (len(xs) - 1) + 0.5))] if xs else None


def task_record(path):
    """같은 .state 의 ugv_tasks.json 에서 timing·eta 를 찾는다 (있으면)."""
    task_id = os.path.splitext(os.path.basename(path))[0]
    store = os.path.join(os.path.dirname(os.path.dirname(path)), "ugv_tasks.json")
    try:
        data = json.load(open(store, encoding="utf-8"))
    except (OSError, ValueError):
        return None
    tasks = data.get("tasks", data) if isinstance(data, dict) else {}
    t = tasks.get(task_id) if isinstance(tasks, dict) else None
    return t if isinstance(t, dict) else None


def summarize(path, by_road=False):
    rows = load(path)
    if len(rows) < 2:
        return {"file": path, "error": "기록이 2줄 미만"}
    col = lambda k: [_f(r[k]) for r in rows]                    # noqa: E731
    wall, sim, px4 = col("wall_s"), col("server_sim_s"), col("px4_time_s")
    out = {"file": path, "samples": len(rows)}

    # --- 시간 ---
    w = wall[-1] - wall[0]
    out["wall_s"] = round(w, 1)
    out["server_sim_s"] = round(sim[-1] - sim[0], 1)
    px = [(a, b) for a, b in zip(wall, px4) if b is not None]
    if len(px) >= 2:
        p = px[-1][1] - px[0][1]
        out["px4_s"] = round(p, 1)
        out["real_time_factor"] = round(p / (px[-1][0] - px[0][0]), 2) if px[-1][0] > px[0][0] else None
        out["server_clock_vs_px4"] = round(out["server_sim_s"] / p, 2) if p > 0 else None
        # 구간별 배속 (30초 창) — 흔들림 확인
        rtf, j = [], 0
        for i in range(len(px)):
            while px[i][0] - px[j][0] > 30:
                j += 1
            if px[i][0] - px[j][0] >= 20:
                rtf.append((px[i][1] - px[j][1]) / (px[i][0] - px[j][0]))
        if rtf:
            out["rtf_30s_min_max"] = [round(min(rtf), 2), round(max(rtf), 2)]
    rec = task_record(path)
    eta = (rec or {}).get("timing", {}).get("eta_sec") if rec else None
    if eta is None:
        e0 = _f(rows[0]["eta_remaining_sec"])
        eta = e0
    if eta:
        out["eta_sec"] = round(eta)
        actual = out.get("px4_s", out["server_sim_s"])
        out["actual_over_eta"] = round(actual / eta, 2)
        out["actual_basis"] = "px4" if "px4_s" in out else "server_sim"
    # 달린 구간만의 ETA (중간에 /stop 해도 비교 가능): 남은 ETA 가 줄어든 양의 합.
    # 재탐색으로 남은 ETA 가 늘어난 순간은 건너뛴다 (늘어난 만큼을 빼면 우회한 구간이 ETA 에서 빠진다)
    e = [x for x in col("eta_remaining_sec") if x is not None]
    used = sum(max(0.0, a - b) for a, b in zip(e, e[1:]))
    if used > 0:
        out["leg_eta_sec"] = round(used)
        actual = out.get("px4_s", out["server_sim_s"])
        out["leg_actual_over_eta"] = round(actual / out["leg_eta_sec"], 2)
    if rec:
        out["status"] = rec.get("status")
        out["reroutes"] = len(rec.get("reroutes") or [])

    # --- 횡오차 ---
    offs = [o for o in col("offset_m") if o is not None]
    if offs:
        a = [abs(o) for o in offs]
        out["offset"] = {
            "mean_m": round(st.mean(offs), 2),                  # + 왼쪽 / - 오른쪽 치우침
            "median_m": round(st.median(offs), 2),
            "std_m": round(st.pstdev(offs), 2),
            "abs_p95_m": round(pct(a, 0.95), 2),
            "abs_max_m": round(max(a), 2),
            "share_left": round(sum(o > 0.5 for o in offs) / len(offs), 2),
            "share_right": round(sum(o < -0.5 for o in offs) / len(offs), 2),
            "share_over_3m": round(sum(x > 3 for x in a) / len(a), 3),
        }
        out["offset"]["bias"] = ("왼쪽" if out["offset"]["median_m"] > 0.5 else
                                 "오른쪽" if out["offset"]["median_m"] < -0.5 else "치우침 없음")

    # --- 속도 (움직이는 구간만: 명령 > 0, 실제 > 0.2) ---
    pairs = [(_f(r["speed_mps"]), _f(r["cmd_speed_mps"])) for r in rows]
    pairs = [(s, c) for s, c in pairs if s is not None and c and s > 0.2]
    if pairs:
        out["speed"] = {"actual_mean_mps": round(st.mean(s for s, _ in pairs), 2),
                        "cmd_mean_mps": round(st.mean(c for _, c in pairs), 2),
                        "actual_over_cmd": round(st.mean(s / c for s, c in pairs), 2),
                        "actual_max_mps": round(max(s for s, _ in pairs), 2)}
    stopped = [r for r in rows if (_f(r["speed_mps"]) or 0) < 0.2]
    out["stopped_share"] = round(len(stopped) / len(rows), 2)

    # --- 도로별 치우침 ---
    if by_road:
        per = {}
        for r in rows:
            o = _f(r["offset_m"])
            if o is not None and r["road_id"]:
                per.setdefault(r["road_id"], []).append(o)
        out["by_road"] = {rid: {"n": len(v), "mean_m": round(st.mean(v), 2),
                                "abs_max_m": round(max(abs(x) for x in v), 2)}
                          for rid, v in per.items() if len(v) >= 4}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--roads", action="store_true", help="도로별 치우침")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    res = [summarize(f, a.roads) for f in a.files]
    if a.json:
        print(json.dumps(res, ensure_ascii=False, indent=2))
        return
    for r in res:
        print(f"== {r['file']}")
        if "error" in r:
            print("  ", r["error"])
            continue
        print(f"  시간   벽시계 {r['wall_s']}s | PX4 {r.get('px4_s', '-')}s | 서버시뮬 {r['server_sim_s']}s"
              f" | 실제배속 {r.get('real_time_factor', '-')} (30초창 {r.get('rtf_30s_min_max', '-')})"
              f" | 서버시계/PX4 {r.get('server_clock_vs_px4', '-')}")
        if "eta_sec" in r:
            print(f"  ETA    {r['eta_sec']}s → 실제 {r.get('px4_s', r['server_sim_s'])}s ({r['actual_basis']})"
                  f" = ETA x{r['actual_over_eta']}   상태 {r.get('status', '-')}  재탐색 {r.get('reroutes', '-')}")
        if "leg_eta_sec" in r:
            print(f"  달린구간 ETA {r['leg_eta_sec']}s → 실제 {r.get('px4_s', r['server_sim_s'])}s = x{r['leg_actual_over_eta']}")
        if "offset" in r:
            o = r["offset"]
            print(f"  횡오차 중앙값 {o['median_m']:+}m 평균 {o['mean_m']:+}m (+왼/-오) → {o['bias']}"
                  f" | std {o['std_m']} | |p95| {o['abs_p95_m']} | max {o['abs_max_m']}"
                  f" | 왼 {o['share_left']:.0%} 오 {o['share_right']:.0%} | 3m 초과 {o['share_over_3m']:.1%}")
        if "speed" in r:
            s = r["speed"]
            print(f"  속도   실제 {s['actual_mean_mps']} / 명령 {s['cmd_mean_mps']} m/s (x{s['actual_over_cmd']})"
                  f" | 최대 {s['actual_max_mps']} | 정지 비율 {r['stopped_share']:.0%}")
        for rid, v in (r.get("by_road") or {}).items():
            print(f"    {rid:>12}  n={v['n']:<4} 평균 {v['mean_m']:+.2f}m  max {v['abs_max_m']}")


if __name__ == "__main__":
    sys.exit(main())
