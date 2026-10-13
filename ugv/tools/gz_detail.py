# -*- coding: utf-8 -*-
"""ugv/tools/gz_detail.py — (WSL 안에서) Gazebo 차량 상세 기록: 차체 자세, 바퀴별 관절·조향·서스펜션, 바퀴별 접촉 상태·힘.

    GZ_PARTITION=adair_ugv GZ_IP=127.0.0.1 python3 -m ugv.tools.gz_detail --world <월드> --model adair_ugv_1 --out <csv> [--duration 170]
    → <csv> + <csv 이름>_meta.json (기록 주기·누락률·바퀴별 상태 비율·접촉 상대 물체 이름)

접촉 센서가 있는 후보 모델(ugv/gazebo/variants/base·susp_a 등, make_vehicle_variant.py)에서 쓴다. 원본: logs/ugv/stage5/jturn/fable_gz_track.py (2026-10-12).
바뀐 점 (차량 개조 검증 지시서 2절 1~4항):
 - 바퀴별 접촉 상태 <바퀴>_c_state: VALID (마지막 메시지 나이 ≤ 2/센서 주기 이고 접촉·wrench 있음) / STALE (그보다 오래됨, ≤ 0.5 s)
   / NO_CONTACT (받은 적 있고 0.5 s 넘게 안 옴 — 접촉 센서는 접촉이 없으면 발행하지 않는다는 가정, 빈 메시지 수를 meta 에 기록해 확인)
   / NO_MSG (한 번도 못 받음). 힘 열은 VALID 일 때만 값, 그 밖은 빈칸 — 0 으로 채우지 않는다.
 - 힘: wrench body_1 힘(월드 좌표 가정, 평탄 정지에서 Σfz ≈ m·g 로 확인)과 접촉 법선 평균 n → 법선력 fn = F·n, 접선력 ft = |F − fn·n|, 세계 Z 힘 fz 따로.
   첫 메시지의 body_1/body_2(collision) 이름을 meta 에 남긴다.
 - 관절: 바퀴 회전 속도, 좌우 조향 위치, 서스펜션 susp_*_joint 위치·속도 (모델이 발행할 때만 — susp_a 는 발행, susp_a0 는 안 함).
 - 바퀴 높이: 링크 자세가 모델 기준이라는 가정 (원본과 같음). 서스펜션 모델의 중첩 링크도 같은 가정 — 검증은 meta 의 link_pose_basis 에 남김.
"""
import argparse
import csv
import json
import math
import signal
import statistics
import threading
import time

W = ["wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"]
SUSP = ["susp_front_left", "susp_front_right", "susp_rear_left", "susp_rear_right"]
SENSOR_HZ = 100.0
STALE_MAX_S = 0.5
VALID, STALE, NO_CONTACT, NO_MSG = "VALID", "STALE", "NO_CONTACT", "NO_MSG"


def contact_state(age_s, ever, has_wrench, sensor_hz=SENSOR_HZ):
    """바퀴 접촉 상태 분류 (Gazebo 없이 시험 가능). age_s: 시뮬레이션 시각 − 마지막 접촉 메시지 시각."""
    if not ever:
        return NO_MSG
    if age_s <= 2.0 / sensor_hz:
        return VALID if has_wrench else STALE
    if age_s <= STALE_MAX_S:
        return STALE
    return NO_CONTACT


def q_rot(q, v):
    w, x, y, z = q.w, q.x, q.y, q.z
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def ypr(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return (math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)), math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1, min(1, 2 * (w * y - z * x)))))


def split_force(F, n):
    """F 를 법선 n(단위화) 방향 fn 과 접선 크기 ft 로."""
    nn = math.sqrt(sum(c * c for c in n))
    if nn < 1e-9:
        return float("nan"), float("nan")
    n = [c / nn for c in n]
    fn = sum(F[i] * n[i] for i in range(3))
    ft = math.sqrt(max(sum((F[i] - fn * n[i]) ** 2 for i in range(3)), 0.0))
    return fn, ft


def main():
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--model", default="adair_ugv_1")
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=170)
    ap.add_argument("--hz", type=float, default=50)
    a = ap.parse_args()
    lock = threading.Lock()
    step = 1 / a.hz
    last = [-1.0]
    stamps = []
    joints = {k: float("nan") for k in [f"{w}_v" for w in W] + ["steer_l", "steer_r", "steer_l_v", "steer_r_v"]
              + [f"{s}_q" for s in SUSP] + [f"{s}_qd" for s in SUSP]}
    seen_joints = set()
    nan = float("nan")
    contact = {w: {"t": -1.0, "ever": False, "n": 0, "wr": False, "F": (nan, nan, nan), "nrm": (nan, nan, nan)} for w in W}
    meta_c = {w: {"msgs": 0, "empty_msgs": 0, "body_1": None, "body_2": None} for w in W}
    link_rel = {}
    states = {w: {VALID: 0, STALE: 0, NO_CONTACT: 0, NO_MSG: 0} for w in W}
    f = open(a.out, "w", newline="")
    wr = csv.writer(f)
    cols = (["sim_t", "x", "y", "z", "yaw", "roll", "pitch", "steer_l", "steer_r"] + [f"{w}_v" for w in W] + [f"{w}_z" for w in W]
            + [f"{s}_q" for s in SUSP] + [f"{s}_qd" for s in SUSP]
            + [f"{w}_c_state" for w in W] + [f"{w}_c_age" for w in W] + [f"{w}_cn" for w in W]
            + [f"{w}_fz" for w in W] + [f"{w}_fn" for w in W] + [f"{w}_ft" for w in W] + [f"{w}_fx" for w in W] + [f"{w}_fy" for w in W] + ["wall_t"])
    wr.writerow(cols)

    def on_joints(m):
        with lock:
            for j in m.joint:
                seen_joints.add(j.name)
                if j.name == "wheel_front_left_steering_joint":
                    joints["steer_l"], joints["steer_l_v"] = j.axis1.position, j.axis1.velocity
                elif j.name == "wheel_front_right_steering_joint":
                    joints["steer_r"], joints["steer_r_v"] = j.axis1.position, j.axis1.velocity
                elif j.name.startswith("susp_"):
                    s = j.name[:-len("_joint")]
                    joints[f"{s}_q"], joints[f"{s}_qd"] = j.axis1.position, j.axis1.velocity
                else:
                    for w in W:
                        if j.name == f"{w}_joint":
                            joints[f"{w}_v"] = j.axis1.velocity

    def mk_contact(w):
        def cb(m):
            t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
            F = [0.0, 0.0, 0.0]
            N = [0.0, 0.0, 0.0]
            has = False
            for c in m.contact:
                for n_ in c.normal:
                    N[0] += n_.x; N[1] += n_.y; N[2] += n_.z
                for wr_ in c.wrench:
                    has = True
                    F[0] += wr_.body_1_wrench.force.x; F[1] += wr_.body_1_wrench.force.y; F[2] += wr_.body_1_wrench.force.z
            with lock:
                mc = meta_c[w]
                mc["msgs"] += 1
                if not m.contact:
                    mc["empty_msgs"] += 1
                elif mc["body_1"] is None:
                    c0 = m.contact[0]
                    mc["body_1"], mc["body_2"] = c0.collision1.name, c0.collision2.name
                contact[w] = {"t": t, "ever": True, "n": len(m.contact), "wr": has, "F": tuple(F) if has else (nan, nan, nan),
                              "nrm": tuple(N)}
        return cb

    def on_pose(m):
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        base = None
        for p in m.pose:
            if p.name == a.model:
                base = p
            elif any(p.name == f"{w}_link" for w in W):
                link_rel[p.name] = (p.position.x, p.position.y, p.position.z)
        if base is None or t - last[0] < step - 1e-9:
            return
        last[0] = t
        yaw, roll, pitch = ypr(base.orientation)
        zs = []
        for w in W:
            r = link_rel.get(f"{w}_link")
            zs.append(nan if r is None else base.position.z + q_rot(base.orientation, r)[2])
        with lock:
            st, ag, cn, fz, fn, ft, fx, fy = [], [], [], [], [], [], [], []
            for w in W:
                c = contact[w]
                age = t - c["t"] if c["ever"] else nan
                s = contact_state(age if c["ever"] else 1e9, c["ever"], c["wr"])
                states[w][s] += 1
                st.append(s)
                ag.append("" if not c["ever"] else round(age, 3))
                cn.append(c["n"] if s == VALID else "")
                if s == VALID:
                    n_, t_ = split_force(c["F"], c["nrm"])
                    fz.append(round(c["F"][2], 1)); fn.append(round(n_, 1)); ft.append(round(t_, 1))
                    fx.append(round(c["F"][0], 1)); fy.append(round(c["F"][1], 1))
                else:
                    fz.append(""); fn.append(""); ft.append(""); fx.append(""); fy.append("")
            row = ([round(t, 4), round(base.position.x, 4), round(base.position.y, 4), round(base.position.z, 4), round(yaw, 5), round(roll, 5),
                    round(pitch, 5), round(joints["steer_l"], 5), round(joints["steer_r"], 5)] + [round(joints[f"{w}_v"], 4) for w in W]
                   + [round(z, 4) for z in zs] + [round(joints[f"{s}_q"], 5) for s in SUSP] + [round(joints[f"{s}_qd"], 4) for s in SUSP]
                   + st + ag + cn + fz + fn + ft + fx + fy + [round(time.time(), 3)])
            wr.writerow(row)
            stamps.append(t)

    node = Node()
    subs = {"pose": node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose),
            "joint_state": node.subscribe(Model, f"/world/{a.world}/model/{a.model}/joint_state", on_joints)}
    for w in W:
        subs[f"contact_{w}"] = node.subscribe(Contacts, f"/world/{a.world}/model/{a.model}/link/{w}_link/sensor/{w}_link_contact/contact", mk_contact(w))
    if not subs["pose"]:
        raise SystemExit("pose 구독 실패")
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    t0 = time.time()
    interrupted = False
    try:
        while time.time() - t0 < a.duration:
            time.sleep(0.5)
            with lock:
                f.flush()
    except KeyboardInterrupt:
        interrupted = True
    elapsed = time.time() - t0
    with lock:
        f.close()
        dts = [b - c for b, c in zip(stamps[1:], stamps[:-1])]
        tot = {w: max(sum(states[w].values()), 1) for w in W}
        meta = {"world": a.world, "model": a.model, "requested_hz": a.hz, "rows": len(stamps), "subscribed": subs,
                "dt_median_s": round(statistics.median(dts), 4) if dts else None,
                "dt_p99_s": round(sorted(dts)[int(0.99 * (len(dts) - 1))], 4) if dts else None,
                "dt_max_s": round(max(dts), 4) if dts else None, "gaps_over_0_1s": sum(1 for d in dts if d > 0.1),
                "contact_state_ratio": {w: {k: round(v / tot[w], 4) for k, v in states[w].items()} for w in W},
                "contact_messages": meta_c, "joints_seen": sorted(seen_joints),
                "susp_joints_published": any(j.startswith("susp_") for j in seen_joints),
                "force_frame": "body_1_wrench.force (월드 좌표 가정 — 평탄 정지 Σfz≈m·g 로 확인할 것)",
                "link_pose_basis": "dynamic_pose/info 링크 자세 = 모델 기준 가정 (원본 fable_gz_track 과 같음)",
                "completed": (not interrupted) and elapsed >= a.duration * 0.99, "interrupted": interrupted, "elapsed_wall_s": round(elapsed, 2),
                "states": {VALID: f"나이 ≤ {2 / SENSOR_HZ:.3f} s 이고 wrench 있음", STALE: f"≤ {STALE_MAX_S} s", NO_CONTACT: f"> {STALE_MAX_S} s (받은 적 있음)",
                           NO_MSG: "한 번도 못 받음"}}
    import os as _os
    import sys as _sys
    write_ok = True
    try:
        with open(a.out[:-4] + "_meta.json" if a.out.endswith(".csv") else a.out + "_meta.json", "w") as mf:
            json.dump(meta, mf, ensure_ascii=False, indent=1)
    except Exception as e:  # noqa: BLE001
        write_ok = False
        print("메타 쓰기 실패:", e)
    print(len(stamps), "rows ->", a.out, "" if meta["completed"] else f"(미완료: interrupted={interrupted}, elapsed {elapsed:.1f} s)")
    # 2026-10-13 우회책 (근본 원인 해결 아님): 기록·메타를 모두 쓴 뒤 인터프리터·gz-transport 종료 정리에서 'terminate called without an active
    # exception' (rc 134) 가 난 적이 있다 (chain_ft4 정적 1회 원본 로그 확인. chain_ft3 경고가 같은 원인인지는 미확정). 구독을 해제하고 출력을
    # 비운 뒤 정리 과정을 거치지 않고 끝낸다. 종료 코드: 0 = 정상 완료(예정 시간 기록·CSV 닫힘·메타 작성), 3 = 중단(SIGTERM 등), 4 = 메타 쓰기 실패
    for topic in (f"/world/{a.world}/dynamic_pose/info", f"/world/{a.world}/model/{a.model}/joint_state") + tuple(
            f"/world/{a.world}/model/{a.model}/link/{w}_link/sensor/{w}_link_contact/contact" for w in W):
        try:
            node.unsubscribe(topic)
        except Exception:  # noqa: BLE001
            pass
    _sys.stdout.flush()
    _sys.stderr.flush()
    _os._exit(0 if (write_ok and meta["completed"]) else (4 if not write_ok else 3))


if __name__ == "__main__":
    main()
