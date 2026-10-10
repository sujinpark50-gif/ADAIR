# -*- coding: utf-8 -*-
"""ugv/tools/fleet_test.py — 다중 차량(UGV·소방차 4대) Gazebo 동시 운용 시험. 총괄처럼 HTTP 로 평가·실행한다.

    python -m ugv.tools.fleet_test separate      # F1·F2·F3: 4대 서로 다른 목표 동시 출동 → 각자 소속 거점 복귀
    python -m ugv.tools.fleet_test follow        # F4: 같은 거점 UGV → 소방차 같은 길 앞뒤
    python -m ugv.tools.fleet_test junction      # F5: 다른 거점 차량이 같은 교차로로
    python -m ugv.tools.fleet_test fault_block   # F7·F8: 한 차량 링크 단절(고장 정지) → 다른 차량이 그 차를 점유 차량으로 기다림
    python -m ugv.tools.fleet_test --analyze logs/ugv/fleet_xxx.json

판정 (Gazebo 자세만 사용): 차량별 차체(차종 치수) 둘레 vs 추정 도로 띠, 차량 쌍 차체 직사각형 겹침·최소 간격, RTF.
"""

import argparse
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import httpx

from ugv.drive_graph import access_point
from ugv.geo import FRAME
from ugv.profiles import get as get_profile
from ugv.roads import RoadNetwork
from ugv.tools.drive_test import LOG_DIR, load_track, start_tracker, to_wsl_path

ROOT = Path(__file__).resolve().parents[2]
SERVER = "http://127.0.0.1:8100"
WORLD = "kangwon_flat"
FLEET = json.loads((ROOT / "ugv" / "fleet_ab.json").read_text(encoding="utf-8"))
MODEL = {c["resource_id"]: c["px4"]["gz_model"] for c in FLEET}
PROFILE = {c["resource_id"]: c["profile"] for c in FLEET}
INSTANCE = {c["resource_id"]: c["px4"]["instance"] for c in FLEET}


class Run:
    def __init__(self, tag, track_s):
        self.c = httpx.Client(base_url=SERVER, timeout=90)
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.tag = tag
        self.trackers, self.csv = {}, {}
        for rid, model in MODEL.items():
            p = LOG_DIR / f"fleet_{tag}_{self.stamp}_{rid}.csv"
            self.csv[rid] = p
            self.trackers[rid] = start_tracker(p, track_s, world=WORLD, model=model)
        time.sleep(3)
        self.log = {"tag": tag, "stamp": self.stamp, "start_wall": time.time(), "start_local": time.strftime("%F %T"),
                    "health": self.c.get("/health").json(), "commands": [], "polls": [], "csv": {k: str(v) for k, v in self.csv.items()}}
        self.t0 = time.time()

    def t(self):
        return round(time.time() - self.t0, 1)

    def go(self, rid, key, lat=None, lon=None, node=None):
        tgt = {"target": {"lat": lat, "lon": lon}} if node is None else {"target_node": node}
        ev = self.c.post(f"/ugv/{rid}/evaluate", json={"task_id": key, "decision_id": "D-" + key, **tgt}).json()
        body = {"task_id": key, "decision_id": "D-" + key, **tgt}
        if ev.get("target_node"):
            body["target_node"] = ev["target_node"]["node_id"]
        w0 = time.time()
        r = self.c.post(f"/ugv/{rid}/execute", json=body)
        rec = {"t": self.t(), "rid": rid, "key": key, "verdict": ev.get("verdict"), "eta": ev.get("eta_sec"),
               "status_code": r.status_code, "rtt_s": round(time.time() - w0, 3),
               "response": r.json() if r.headers.get("content-type", "").startswith("application/json") else r.text[:300]}
        self.log["commands"].append(rec)
        print(f"[{self.t():6.1f}] {rid} {key} {ev.get('verdict')} eta {ev.get('eta_sec')} → {r.status_code} "
              f"{(rec['response'] or {}).get('status') if isinstance(rec['response'], dict) else rec['response']} rtt {rec['rtt_s']}",
              flush=True)
        return rec

    def home(self, rid, key):
        st = self.c.get(f"/ugv/{rid}/state").json()
        acc = st["station"]["access_point"]
        return self.go(rid, key, node=f"RP:{acc['road_id']}:{acc['s_m']:.1f}")

    def poll(self):
        sts = self.c.get("/ugv").json()
        tr = self.c.get("/traffic").json()
        now = time.time()
        rec = {"t": self.t(), "vehicles": {s["resource_id"]: {"state": s["state"], "mission": s["mission_state"],
                                                              "speed": s["speed_mps"], "task": s["current_task_id"],
                                                              "tel_age_s": round(now - s["updated_at"], 2) if s["updated_at"] else None,
                                                              "fault": s["fault"]} for s in sts},
               "traffic_waits": {k: {x: v.get(x) for x in ("other", "kind", "why")} for k, v in tr["waits"].items()},
               "deadlocks": tr["deadlocks"], "unavoidable": tr["unavoidable"], "tick_ms": tr["tick_ms"]}
        self.log["polls"].append(rec)
        return rec

    def task(self, rid, key):
        return self.c.get(f"/ugv/{rid}/task/{key}").json()

    def wait(self, keys, timeout_s, every=3.0, extra=None):
        """keys [(rid, key)] 이 모두 끝날 때까지 상태를 기록한다."""
        end = time.time() + timeout_s
        while time.time() < end:
            p = self.poll()
            if extra:
                extra(p)
            line = " ".join(f"{r}:{v['mission'][:5]}/{v['speed']}" + (f"/W{p['traffic_waits'][r]['kind']}" if r in p["traffic_waits"] else "")
                            for r, v in p["vehicles"].items())
            print(f"[{self.t():6.1f}] {line}", flush=True)
            if keys and all(self.task(r, k).get("status") in ("COMPLETED", "FAILED", "CANCELLED") for r, k in keys):
                return True
            time.sleep(every)
        return False

    def finish(self):
        self.log["end_local"] = time.strftime("%F %T")
        self.log["events"] = {rid: self.c.get(f"/ugv/{rid}/events?limit=200").json() for rid in MODEL}
        self.log["tasks"] = {c["key"]: self.task(c["rid"], c["key"]) for c in self.log["commands"]}
        self.log["traffic_final"] = self.c.get("/traffic").json()
        for p in self.trackers.values():
            p.terminate()
        time.sleep(2)
        out = LOG_DIR / f"fleet_{self.tag}_{self.stamp}.json"
        self.log["analysis"] = analyze(self.log)
        out.write_text(json.dumps(self.log, ensure_ascii=False, indent=1), encoding="utf-8")
        print(json.dumps(self.log["analysis"], ensure_ascii=False, indent=1))
        print("기록:", out)
        return out


def link(rid, act):
    sh = to_wsl_path(ROOT / "ugv" / "tools" / "wsl_link.sh")
    subprocess.run(["wsl.exe", "-d", "Ubuntu-24.04", "--", "bash", sh, str(INSTANCE[rid]), act], check=False)


# ---------------------------------------------------------------------------- 분석
def _rect(x, y, yaw, L, W):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + dx * c - dy * s, y + dx * s + dy * c) for dx, dy in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2))]


def _overlap(r1, r2):
    for poly in (r1, r2):
        for i in range(4):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % 4]
            nx, ny = y2 - y1, x1 - x2
            p1 = [nx * x + ny * y for x, y in r1]
            p2 = [nx * x + ny * y for x, y in r2]
            if max(p1) < min(p2) or max(p2) < min(p1):
                return False
    return True


def analyze(log):
    net = RoadNetwork()
    tracks = {rid: load_track(Path(p)) for rid, p in log["csv"].items() if Path(p).exists()}
    out = {"vehicles": {}, "pairs": {}}
    for rid, rows in tracks.items():
        if len(rows) < 3:
            out["vehicles"][rid] = {"error": "기록 부족"}
            continue
        prof = get_profile(PROFILE[rid])
        g = net.graph_for(prof)
        t = [r["sim_t"] for r in rows]
        dist = sum(math.hypot(rows[i + 1]["x"] - rows[i]["x"], rows[i + 1]["y"] - rows[i]["y"]) for i in range(len(rows) - 1))
        spd = []
        for i in range(len(rows)):
            j, k = max(0, i - 5), min(len(rows) - 1, i + 5)
            dt = t[k] - t[j]
            spd.append(math.hypot(rows[k]["x"] - rows[j]["x"], rows[k]["y"] - rows[j]["y"]) / dt if dt > 0 else 0.0)
        out_cnt, worst, last = 0, -math.inf, -1e9
        events, cur = [], None
        for i, r in enumerate(rows):
            if t[i] - last < 0.2:
                continue
            last = t[i]
            c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
            # 모델 원점 = 차체 중심. 차체 둘레 표본(뒤축 기준)을 차체 중심 기준으로
            ex = max(g.outside_m(r["x"] + (fx - prof.rear_axle_from_center_m) * c - fy * s,
                                 r["y"] + (fx - prof.rear_axle_from_center_m) * s + fy * c) for fx, fy in prof.body_samples)
            worst = max(worst, ex)
            if ex > 0:
                out_cnt += 1
                if cur and t[i] - cur["last"] <= 0.3:
                    cur["last"], cur["max"] = t[i], max(cur["max"], ex)
                else:
                    cur = {"first": t[i], "last": t[i], "max": ex, "x": round(r["x"], 1), "y": round(r["y"], 1),
                           "speed": round(spd[i], 2)}
                    events.append(cur)
        wall = [r.get("wall_t") for r in rows]
        rtf = (t[-1] - t[0]) / (wall[-1] - wall[0]) if wall[0] and wall[-1] and wall[-1] > wall[0] else None
        out["vehicles"][rid] = {
            "profile": prof.name, "distance_m": round(dist, 1), "sim_s": round(t[-1] - t[0], 1),
            "max_speed_kmh": round(max(spd) * 3.6, 1), "final_speed_mps": round(spd[-1], 2),
            "max_abs_roll_deg": round(max(abs(math.degrees(r["roll"])) for r in rows), 2),
            "max_abs_pitch_deg": round(max(abs(math.degrees(r["pitch"])) for r in rows), 2),
            "rollover": any(abs(r["roll"]) > math.radians(45) or abs(r["pitch"]) > math.radians(45) for r in rows),
            "offroad_samples": out_cnt, "offroad_events": len(events),
            "offroad_max_m": round(max((e["max"] for e in events), default=0.0), 2),
            "offroad_longest_s": round(max((e["last"] - e["first"] + 0.2 for e in events), default=0.0), 1),
            "offroad_list": [{**e, "max": round(e["max"], 2)} for e in events[:10]],
            "rtf": None if rtf is None else round(rtf, 3), "pose_gap_max_s": round(max(t[i + 1] - t[i] for i in range(len(t) - 1)), 3),
            "basis": f"차체 {prof.body_length_m}×{prof.body_width_m} m 둘레 표본 vs 추정 도로 띠 (0.2 s)"}
    rids = list(tracks)
    for i in range(len(rids)):
        for j in range(i + 1, len(rids)):
            a, b = rids[i], rids[j]
            pa, pb = get_profile(PROFILE[a]), get_profile(PROFILE[b])
            ra, rb = tracks[a], tracks[b]
            if not ra or not rb:
                continue
            tb = [r["sim_t"] for r in rb]
            import bisect
            min_gap, overl = math.inf, 0
            for r in ra[::4]:
                k = min(len(rb) - 1, bisect.bisect_left(tb, r["sim_t"]))
                q = rb[k]
                if abs(q["sim_t"] - r["sim_t"]) > 0.2:
                    continue
                d = math.hypot(r["x"] - q["x"], r["y"] - q["y"])
                if d > 60:
                    min_gap = min(min_gap, d - (pa.body_length_m + pb.body_length_m) / 2)
                    continue
                gap = d - (pa.body_length_m + pb.body_length_m) / 2
                min_gap = min(min_gap, gap)
                if _overlap(_rect(r["x"], r["y"], r["yaw"], pa.body_length_m, pa.body_width_m),
                            _rect(q["x"], q["y"], q["yaw"], pb.body_length_m, pb.body_width_m)):
                    overl += 1
            out["pairs"][f"{a}~{b}"] = {"min_center_gap_minus_half_lengths_m": None if math.isinf(min_gap) else round(min_gap, 2),
                                        "body_overlap_samples": overl}
    cmds = log.get("commands", [])
    out["command_rtt_s"] = {"max": max((c["rtt_s"] for c in cmds), default=None),
                            "mean": round(sum(c["rtt_s"] for c in cmds) / len(cmds), 3) if cmds else None}
    ages = [v["tel_age_s"] for p in log.get("polls", []) for v in p["vehicles"].values() if v["tel_age_s"] is not None]
    out["telemetry_age_s"] = {"max": max(ages, default=None), "p95": sorted(ages)[int(0.95 * (len(ages) - 1))] if ages else None,
                              "polls": len(log.get("polls", []))}
    ev = {rid: [e for e in evs if e["event"] in ("YIELD", "YIELD_END", "TRAFFIC_DEADLOCK", "TRAFFIC_BLOCKED_BY_VEHICLE",
                                                 "TRAFFIC_CONFLICT_UNAVOIDABLE", "FAULT", "STOP_PATH", "DANGER")]
          for rid, evs in log.get("events", {}).items()}
    out["traffic_events"] = {rid: [{k: e.get(k) for k in ("event", "sim_time_s", "other", "kind", "why", "waited_s", "code")}
                                   for e in evs][:30] for rid, evs in ev.items()}
    out["tasks"] = {k: (v.get("resource_id"), v.get("status"), v.get("mission_result"), v.get("error")) for k, v in log.get("tasks", {}).items()}
    tick = [p["tick_ms"] for p in log.get("polls", []) if p.get("tick_ms") is not None]
    out["traffic_tick_ms_max"] = max(tick, default=None)
    return out


# ---------------------------------------------------------------------------- 시나리오
def targets():
    net = RoadNetwork()
    acc = {b: access_point(net, b)[0] for b in "AB"}
    ll = lambda b, dx, dy: FRAME.to_ll(acc[b].x + dx, acc[b].y + dy)
    return net, acc, ll


def sc_separate(track_s):
    net, acc, ll = targets()
    run = Run("separate", track_s)
    # 같은 거점 두 차는 출입 도로가 같다 → 앞(출입 지점) 차부터 출발. 목표는 서로 다른 방향
    # B 주변은 도로가 적어 오프셋 목표가 같은 도로 지점으로 모인다 → 평가로 확인한 서로 다른 목표 (2026-10-10)
    jobs = [("A-ugv1", "S-A-U", ll("A", -900, -700)), ("B-ugv1", "S-B-U", (37.951004, 128.337462)),
            ("A-fire1", "S-A-F", ll("A", 900, 600)), ("B-fire1", "S-B-F", (37.955735, 128.317915))]
    started = []
    for rid, key, (la, lo) in jobs:
        if run.go(rid, key, la, lo)["status_code"] == 200:
            started.append((rid, key, None))
    jobs = started
    run.wait([(r, k) for r, k, _ in jobs], track_s * 0.45)
    homes = [(r, "H-" + k) for r, k, _ in jobs]
    for r, k in homes:
        run.home(r, k)
    run.wait(homes, track_s * 0.45)
    return run.finish()


def sc_follow(track_s):
    """같은 거점 UGV 와 소방차가 같은 길을 앞뒤로: UGV 먼저 출발, 소방차 목표 = UGV 계획 경로 60 % 지점 (같은 길로 뒤따른다)."""
    net, acc, ll = targets()
    run = Run("follow", track_s)
    la, lo = ll("A", -900, -700)
    run.go("A-ugv1", "FO-U", la, lo)
    items = run.c.get("/ugv/A-ugv1/route").json()["items"]
    mid = items[int(len(items) * 0.6)]
    time.sleep(2)
    run.go("A-fire1", "FO-F", mid["lat"], mid["lon"])
    run.wait([("A-ugv1", "FO-U"), ("A-fire1", "FO-F")], track_s * 0.5)
    run.home("A-ugv1", "FO-HU")
    time.sleep(2)
    run.home("A-fire1", "FO-HF")
    run.wait([("A-ugv1", "FO-HU"), ("A-fire1", "FO-HF")], track_s * 0.45)
    return run.finish()


def sc_fault_block(track_s):
    """B-ugv1 을 출발시킨 뒤 링크를 끊어 길 위에서 고장 정지(시동 끔)시킨다. 그 뒤 B-fire1 에 B-ugv1 경로 너머 지점을 준다
    (고장 차가 막은 길) → 계획이 그 도로를 피하거나(우회) 그 앞에서 기다려야 한다. 그동안 A-ugv1 은 정상 운용 (한 차량 고장이
    다른 차량에 번지지 않음). 마지막에 링크 복구·운영자 해제."""
    net, acc, ll = targets()
    run = Run("fault_block", track_s)
    run.go("B-ugv1", "FB-BU", 37.951004, 128.337462)
    items = run.c.get("/ugv/B-ugv1/route").json()["items"]
    beyond = items[int(len(items) * 0.85)]
    run.go("A-ugv1", "FB-AU", *ll("A", -900, -700))
    st = {"cut": None}

    def cut_when_moving(p):
        v = p["vehicles"]["B-ugv1"]
        if st["cut"] is None and (v["speed"] or 0) > 8 and run.t() > 15:
            link("B-ugv1", "stop")
            st["cut"] = run.t()
            run.log["link_cut_t"] = st["cut"]
            print(f"[{run.t():6.1f}] B-ugv1 링크 끊음", flush=True)
        if st["cut"] is not None and "fire" not in st and run.t() > st["cut"] + 25:
            st["fire"] = run.go("B-fire1", "FB-BF", beyond["lat"], beyond["lon"])
    run.wait([], 120, extra=cut_when_moving)
    keys = [("A-ugv1", "FB-AU")] + ([("B-fire1", "FB-BF")] if st.get("fire", {}).get("status_code") == 200 else [])
    run.wait(keys, track_s * 0.5)
    link("B-ugv1", "start")
    time.sleep(8)
    run.log["release"] = [run.c.post("/ugv/B-ugv1/stop", json={"reason": "FLEET_TEST_RELEASE"}).json()]
    time.sleep(10)
    run.log["release"].append(run.c.post("/ugv/B-ugv1/stop", json={"reason": "FLEET_TEST_RELEASE2"}).json())
    run.poll()
    return run.finish()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", nargs="?", choices=("separate", "follow", "fault_block"))
    ap.add_argument("--track-s", type=float, default=1800)
    ap.add_argument("--analyze", type=Path)
    a = ap.parse_args()
    if a.analyze:
        log = json.loads(a.analyze.read_text(encoding="utf-8"))
        log["analysis"] = analyze(log)
        a.analyze.write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
        print(json.dumps(log["analysis"], ensure_ascii=False, indent=1))
        return
    {"separate": sc_separate, "follow": sc_follow, "fault_block": sc_fault_block}[a.scenario](a.track_s)


if __name__ == "__main__":
    sys.exit(main())
