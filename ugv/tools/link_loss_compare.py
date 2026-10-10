# -*- coding: utf-8 -*-
"""ugv/tools/link_loss_compare.py — 자동조종기 링크 단절 때 PX4 기체 쪽 대응을 같은 속도·같은 곡선에서 비교한다.

    (WSL) bash ugv/tools/px4-stop.sh --params && bash ugv/tools/wsl_start_fleet.sh logs/ugv/linkloss/spawn_bugv1.txt
    (WSL) bash ugv/tools/wsl_param_set.sh 3 NAV_DLL_ACT 1          # 비교할 대응 (기본 airframe 은 6 = 시동 끔)
    UGV_VEHICLES_JSON=logs/ugv/linkloss/fleet_bugv1.json UGV_DRIVER=px4 uvicorn ugv.server:app --port 8100
    python -m ugv.tools.link_loss_compare run --tag hold [--target-node RP:...] [--observe 90]
    python -m ugv.tools.link_loss_compare analyze logs/ugv/linkloss/<tag>_<stamp>.json

조건 (F7 재현과 같음): A 평탄 월드, B-ugv1 단독, B 출입 지점 → 목표 37.951004, 128.337462. 차가 (17087.8, -5214.6) 에 오면
(그때 약 9.8 m/s) 링크를 끊는다. 끊는 방법 (--cut):
  suspend (기본) — Windows 의 UGV 서버 프로세스를 통째로 일시 정지(NtSuspendProcess). PX4 는 지상국 하트비트를 못 받고, 서버는 아무것도
                   보내지 못한다. PX4 안의 MAVLink 모듈은 그대로 — 실제 무선 단절과 같다.
  hook           — 서버의 그 차량 MAVSDK 연결만 끊는다 (UGV_TEST_HOOKS=1, 다중 차량에서 한 대만 끊을 때)
  mavlink        — PX4 안의 서버용 MAVLink 인스턴스를 멈춘다(wsl_link.sh, F7 방식). 멈출 때 PX4 dataman 읽기가 시간 초과
                   ("Waypoint could not be read") 나 미션 추종이 흔들릴 수 있어 시험 방법 자체가 결과를 바꾼다 (2026-10-10 확인). PX4 COM_DL_LOSS_T 10 s 뒤 대응이 시작되는 곳은 13.8 m/s 로 곡선에
들어가는 곳 (F7 에서 바퀴가 잠긴 곳 17179.9, -5295.9). 링크를 끊은 뒤 서버는 차에 아무 명령도 보낼 수 없다 (가정하지 않는다).

기록: 끊은 위치·속도, 대응 시작(끊고 10 s) 위치·속도, 정지까지 시간·거리, 끊은 뒤 총 이동, 차체 도로 이탈 최대(차종 치수 vs 추정
도로 띠), 계획 경로에서 벗어난 최대 거리, 정지 뒤 움직임, 다른 차가 비워 두는 구간(ugv/traffic.py: 마지막 위치부터 계획 경로를 따라
속도 × 10 s + 30 m) 밖으로 나간 최대 거리 — 다른 차량과의 간격은 이 구간 밖에 있는 차 기준으로 이만큼 줄어든다.
"""

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import httpx

from ugv.geo import FRAME
from ugv.profiles import get as get_profile
from ugv.roads import RoadNetwork
from ugv.tools.drive_test import LOG_DIR, ROOT, load_track, start_tracker, to_wsl_path

OUT = LOG_DIR / "linkloss"
SERVER = "http://127.0.0.1:8100"
RID, INST, MODEL, WORLD = "B-ugv1", 3, "adair_ugv_3", "kangwon_flat"
TARGET = (37.951004, 128.337462)
CUT_XY = (17087.82, -5214.56)
DL_LOSS_T = 10.0                     # PX4 COM_DL_LOSS_T (airframe 기본값). 다른 값으로 시험하면 --params 에서 읽는다
DRIVE_S, SKID_M = 10.0, 30.0         # 예전 ugv/traffic.py 점유 구간 (마지막 속도 × 10 s + 30 m) — 비교용


def _server_pid():
    out = subprocess.run(["netstat", "-ano", "-p", "TCP"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        f = line.split()
        if len(f) >= 5 and f[1].endswith(":8100") and f[3] == "LISTENING":
            return int(f[4])
    raise RuntimeError("포트 8100 서버 프로세스를 찾지 못함")


def suspend(pid, on=True):
    import ctypes
    k32, nt = ctypes.windll.kernel32, ctypes.windll.ntdll
    h = k32.OpenProcess(0x0800, False, pid)                      # PROCESS_SUSPEND_RESUME
    if not h:
        raise OSError(f"OpenProcess {pid} 실패")
    try:
        (nt.NtSuspendProcess if on else nt.NtResumeProcess)(h)
    finally:
        k32.CloseHandle(h)


def link(act):
    subprocess.run(["wsl.exe", "-d", "Ubuntu-24.04", "--", "bash", to_wsl_path(ROOT / "ugv" / "tools" / "wsl_link.sh"),
                    str(INST), act], check=False, capture_output=True)


def run(a):
    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv = OUT / f"{a.tag}_{stamp}.csv"
    c = httpx.Client(base_url=SERVER, timeout=60)
    tr = start_tracker(csv, a.observe + 200, world=WORLD, model=MODEL)
    time.sleep(3)
    body = {"task_id": f"LL-{a.tag}-{stamp}", "decision_id": "D-LL"}
    body.update({"target_node": a.target_node} if a.target_node else {"target": {"lat": TARGET[0], "lon": TARGET[1]}})
    ev = c.post(f"/ugv/{RID}/evaluate", json=body).json()
    if ev.get("target_node"):
        body["target_node"] = ev["target_node"]["node_id"]
    ex = c.post(f"/ugv/{RID}/execute", json=body).json()
    route = c.get(f"/ugv/{RID}/route").json()
    log = {"tag": a.tag, "stamp": stamp, "csv": str(csv), "evaluate": ev, "execute": ex, "route": route,
           "cut_xy_target": a.cut_xy, "params": a.params, "start_wall": time.time()}
    print(f"{a.tag}: {ex.get('status')} eta {ex.get('eta_sec')}", flush=True)
    cut = None
    t_end = time.time() + 200
    while time.time() < t_end and cut is None:
        st = c.get(f"/ugv/{RID}/state").json()
        x, y = FRAME.to_xy(st["position"]["lat"], st["position"]["lon"])
        if math.hypot(x - a.cut_xy[0], y - a.cut_xy[1]) < 6.0:
            if a.cut == "suspend":
                pid = _server_pid()
                suspend(pid, True)
                log["server_pid"] = pid
            elif a.cut == "hook":
                c.post(f"/ugv/{RID}/_test/link", json={"cut": True})
            else:
                link("stop")
            cut = {"wall": time.time(), "server_xy": [round(x, 2), round(y, 2)], "server_speed": st.get("speed_mps")}
            print(f"링크 끊음 @ {cut['server_xy']} {cut['server_speed']} m/s", flush=True)
        time.sleep(0.1)
    log["cut"] = cut
    log["cut_method"] = a.cut
    time.sleep(a.observe)
    log["end_wall"] = time.time()
    tr.terminate()
    time.sleep(2)
    p = OUT / f"{a.tag}_{stamp}.json"
    p.write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
    if a.cut == "suspend":                 # 기록이 끝난 뒤에만 잇는다
        suspend(log["server_pid"], False)
    elif a.cut == "hook":
        c.post(f"/ugv/{RID}/_test/link", json={"cut": False})
    else:
        link("start")
    log["analysis"] = analyze(log)
    p.write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(log["analysis"], ensure_ascii=False, indent=1))
    print("기록:", p)


def _near(pts, x, y):
    best = (math.inf, 0, 0.0)
    for i in range(len(pts) - 1):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        d = math.hypot(x - ax - t * dx, y - ay - t * dy)
        if d < best[0]:
            best = (d, i, t)
    return best


def _plan_dir(pts, x, y):
    _, i, _ = _near(pts, x, y)
    return math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])


def analyze(log):
    rows = load_track(Path(log["csv"]))
    if not rows or not log.get("cut"):
        return {"error": "기록 없음 또는 링크를 끊지 못함"}
    prof = get_profile("adair_ugv")
    net = RoadNetwork()
    g = net.graph_for(prof)
    rac = prof.rear_axle_from_center_m
    pts = log["route"]["plan_xy"]
    cum = [0.0]
    for p, q in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist(p, q))
    k0 = min(range(len(rows)), key=lambda i: abs(rows[i]["wall_t"] - log["cut"]["wall"]))
    t_cut = rows[k0]["sim_t"]

    def spd(i):
        a, b = rows[max(0, i - 5)], rows[min(len(rows) - 1, i + 5)]
        dt = b["sim_t"] - a["sim_t"]
        return math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / dt if dt > 0 else 0.0
    import re
    m = re.search(r"COM_DL_LOSS_T (\d+(?:\.\d+)?)", log.get("params") or "")
    dl = float(m.group(1)) if m else DL_LOSS_T
    kt = next((i for i in range(k0, len(rows)) if rows[i]["sim_t"] >= t_cut + dl - 1.0), len(rows) - 1)
    # 대응 시작: 끊고 (COM_DL_LOSS_T - 1) s 이후 바퀴 속도가 20 % 넘게 떨어지거나 조향이 5° 넘게 움직인 첫 표본 (PX4 시각 기록이 없어 자세로 찾는다)
    w0, s0 = abs(rows[kt]["wheel_rad_s"]), rows[kt]["steer_rad"]
    kt = next((i for i in range(kt, len(rows)) if abs(rows[i]["wheel_rad_s"]) < 0.8 * w0
               or abs(rows[i]["steer_rad"] - s0) > math.radians(5)), kt)
    # 정지: 대응 시작 뒤 0.2 m/s 미만이 3 s 이어진 첫 시각
    kr, still = None, None
    for i in range(kt, len(rows)):
        if spd(i) < 0.2:
            still = i if still is None else still
            if rows[i]["sim_t"] - rows[still]["sim_t"] >= 3.0:
                kr = still
                break
        else:
            still = None
    path = lambda i, j: sum(math.hypot(rows[k + 1]["x"] - rows[k]["x"], rows[k + 1]["y"] - rows[k]["y"]) for k in range(i, j))
    # 다른 차가 비워 두는 구간: 끊은 위치(서버가 안 마지막 위치)부터 계획 경로를 따라 속도 × 10 s + 30 m
    cx, cy = log["cut"]["server_xy"]
    _, ic, tc = _near(pts, cx, cy)
    s_c = cum[ic] + tc * (cum[ic + 1] - cum[ic])
    from types import SimpleNamespace
    from ugv.traffic import link_loss_reach
    vplan = log["route"]["plan_v"]
    reach_old = (log["cut"]["server_speed"] or 0) * DRIVE_S + SKID_M
    reach = link_loss_reach(SimpleNamespace(pts=pts, cum=cum, v=vplan), ic, log["cut"]["server_speed"] or 0)
    corridor = [(cx, cy)] + [tuple(pts[k]) for k in range(ic + 1, len(pts)) if cum[k] - s_c <= reach]
    corridor_old = [(cx, cy)] + [tuple(pts[k]) for k in range(ic + 1, len(pts)) if cum[k] - s_c <= reach_old]
    worst_out_old = 0.0
    worst_ex, worst_xt, worst_out, worst_after_rest = -math.inf, 0.0, 0.0, -math.inf
    last = -1e9
    for i in range(k0, len(rows)):
        r = rows[i]
        if r["sim_t"] - last < 0.2:
            continue
        last = r["sim_t"]
        c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
        rx, ry = r["x"] - rac * c, r["y"] - rac * s
        ex = g.body_excess(rx, ry, r["yaw"])
        worst_ex = max(worst_ex, ex)
        if kr is not None and i >= kr:
            worst_after_rest = max(worst_after_rest, ex)
        worst_xt = max(worst_xt, _near(pts, rx, ry)[0])
        for cor, key in ((corridor, "new"), (corridor_old, "old")):
            o = max(0.0, (_near(cor, r["x"], r["y"])[0] if len(cor) > 1 else math.hypot(r["x"] - cx, r["y"] - cy))
                    - prof.body_length_m / 2)
            if key == "new":
                worst_out = max(worst_out, o)
            else:
                worst_out_old = max(worst_out_old, o)
    rp = lambda i: [round(rows[i]["x"], 1), round(rows[i]["y"], 1)]
    res = {
        "params": log.get("params"), "cut_method": log.get("cut_method", "mavlink"),
        "response_after_cut_s": round(rows[kt]["sim_t"] - t_cut, 2), "cut_sim_t": round(t_cut, 2), "cut_xy": rp(k0), "speed_at_cut_mps": round(spd(k0), 2),
        "response_start_xy": rp(kt), "speed_at_response_mps": round(spd(kt), 2),
        "stopped": kr is not None,
        "stop_time_from_response_s": None if kr is None else round(rows[kr]["sim_t"] - rows[kt]["sim_t"], 1),
        "stop_dist_from_response_m": None if kr is None else round(path(kt, kr), 1),
        "travel_after_cut_until_stop_m": round(path(k0, kr if kr is not None else len(rows) - 1), 1),
        "rest_xy": rp(kr) if kr is not None else None,
        "moved_after_rest_m": None if kr is None else round(path(kr, len(rows) - 1), 1),
        "observed_after_cut_s": round(rows[-1]["sim_t"] - t_cut, 1),
        "max_body_offroad_m": round(worst_ex, 2), "offroad_at_or_after_rest_m": None if kr is None else round(worst_after_rest, 2),
        "max_cross_track_from_plan_m": round(worst_xt, 2),
        "occupancy_reach_m": round(reach, 1), "max_outside_occupied_corridor_m": round(worst_out, 2),
        "occupancy_reach_old_m": round(reach_old, 1), "max_outside_old_corridor_m": round(worst_out_old, 2),
        "max_yaw_from_plan_after_response_deg": round(max(abs(math.degrees((rows[i]["yaw"] - _plan_dir(pts, rows[i]["x"], rows[i]["y"]) + math.pi) % (2 * math.pi) - math.pi)) for i in range(kt, (kr or len(rows) - 1) + 1)), 1),
        "max_abs_steer_after_response_deg": round(max(abs(math.degrees(rows[i]["steer_rad"])) for i in range(kt, (kr or len(rows) - 1) + 1)), 1),
        "basis": "Gazebo 자세. 차체 4.2x1.8 m 둘레 표본 vs 추정 도로 띠 (판정 기준 그대로). 대응 시작 = 끊고 9 s 이후 바퀴 속도 20 % 감소 또는 조향 5° 변화"}
    return res


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--tag", required=True)
    r.add_argument("--target-node")
    r.add_argument("--observe", type=float, default=90)
    r.add_argument("--params", default="")
    r.add_argument("--cut", choices=("suspend", "hook", "mavlink"), default="suspend")
    r.add_argument("--cut-xy", nargs=2, type=float, default=CUT_XY, help="링크를 끊을 위치 (지역 평면 m)")
    z = sub.add_parser("analyze")
    z.add_argument("log", type=Path)
    a = ap.parse_args()
    if a.cmd == "run":
        run(a)
    else:
        log = json.loads(a.log.read_text(encoding="utf-8"))
        log["analysis"] = analyze(log)
        a.log.write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
        print(json.dumps(log["analysis"], ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
