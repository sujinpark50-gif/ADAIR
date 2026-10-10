# -*- coding: utf-8 -*-
"""ugv/tools/summarize_runs.py — road_test 기록(JSON)들의 핵심 수치 한 줄 요약.  python -m ugv.tools.summarize_runs logs/ugv/G*_*.json"""
import json
import sys


def summary(path):
    d = json.load(open(path, encoding="utf-8"))
    a = d.get("analysis") or {}
    o = a.get("overall") or {}
    rd = o.get("road_dev") or {}
    ev = rd.get("events") or {}
    out = {"file": path, "tag": d.get("tag"), "start": d.get("start_local"), "end": d.get("end_local"),
           "distance_m": o.get("distance_m"), "sim_s": o.get("sim_duration_s"),
           "rtf": (o.get("rtf") or {}).get("overall"), "rtf_min10s": (o.get("rtf") or {}).get("min_10s"),
           "vmax_kmh": o.get("max_speed_kmh_0p5s"), "v3s_kmh": o.get("max_sustained_speed_kmh_3s"),
           "roll_max": o.get("max_abs_roll_deg"), "pitch_max": o.get("max_abs_pitch_deg"), "rollover": o.get("rollover"),
           "offroad_events": ev.get("count"), "offroad_max_m": ev.get("max_excess_m"), "offroad_longest_s": ev.get("longest_s"),
           "offroad_by_cause": ev.get("by_cause"), "pose_gap_max_s": o.get("pose_gap_max_sim_s"),
           "tel_age_max_s": d.get("tel_age_max_s"),
           "cmd_rtt_s": [r.get("execute_rtt_s") for r in d.get("runs", [])],
           "cmd_timing": [(r.get("execute") or {}).get("command_timing") for r in d.get("runs", [])],
           "at_command": [r.get("at_command") for r in d.get("runs", [])],
           "second_trigger": d.get("second_trigger"), "fire": d.get("fire"),
           "tasks": {k: (v.get("status"), v.get("mission_result"), v.get("error")) for k, v in (d.get("tasks") or {}).items()},
           "runs": {}}
    for k, r in (a.get("runs") or {}).items():
        c = r.get("curves") or {}
        over_key = next((x for x in c if x.startswith("apex_over")), None)
        out["runs"][k] = {"length_m": r.get("length_m"), "eta_s": r.get("eta_sec"), "dur_s": r.get("duration_s"),
                          "end_err_m": r.get("end_to_last_item_m"), "end_v": r.get("end_speed_mps"),
                          "vmax": r.get("max_speed_mps"), "curves": c.get("checked"),
                          "curves_over_1mps": len(c.get(over_key) or []) if over_key else None,
                          "worst_over_mps": c.get("worst_over_mps"), "lat_accel": r.get("lat_accel_mps2"),
                          "plan_body_check": r.get("plan_body_check")}
    if a.get("abort"):
        out["abort"] = a["abort"]
    evs = [d_ for d_ in (d.get("events") or []) if d_.get("event") in ("STOP_PATH", "DANGER", "EVADING", "EVADED", "YIELD",
                                                                       "TRAFFIC_DEADLOCK", "COMM_LOSS_STOP", "REPLANNED")]
    out["key_events"] = [{k: e.get(k) for k in ("event", "sim_time_s", "basis", "length_m", "needed_m", "short", "drivable",
                                                "clearance_m", "failure_reasons", "operator_action_required")} for e in evs][-8:]
    return out


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(json.dumps(summary(p), ensure_ascii=False, indent=1))
