# -*- coding: utf-8 -*-
"""ugv/tools/vehicle_consistency.py — 차량 제원 자동 대조 (후속 계획 S5 2항): 후보 SDF ↔ PX4 airframe ↔ 서버 profiles(경로 계획·정지 계산).

    python -m ugv.tools.vehicle_consistency --server-tree ../ADAIR_wt_fix [--ugv susp_a] [--fire fire_v2] [--out consistency.json]

항목 (허용 오차): 축간 ±0.01 m, 최대 조향 ±0.5°, 차체 길이·폭 ±0.05 m, 뒤축↔차체 중심 ±0.05 m, 질량 ±2 %, 최고속도 ±0.1 m/s.
감속은 숫자가 같을 필요가 없다 (계획 감속 ≤ PX4 RO_DECEL_LIM 인지, 비상 감속이 실측 제동으로 검증됐는지만 표시 — 후속 계획 7.2-5).
최소 회전반경(뒤축 중심) = 축간 / tan(최대 조향). 서버 profiles 는 그 작업 트리에서 별도 프로세스로 읽는다 (패키지 충돌 방지).
판정: MATCH / MISMATCH / INFO. MISMATCH 가 있으면 그 조합으로 실제 도로 시험을 하지 않는다 (계획기와 차량이 다름).
"""
import argparse
import json
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from ugv.tools import vehicle_report_common as vc

ROOT = Path(__file__).resolve().parents[2]
PX4_ID = {"adair_ugv": "51100_gz_adair_ugv", "adair_firetruck": "51101_gz_adair_firetruck"}


def airframe(variant, model):
    p = ROOT / "ugv" / "gazebo" / "variants" / variant / "airframes" / PX4_ID[model]
    if not p.exists():
        p = ROOT / "ugv" / "px4" / "airframes" / PX4_ID[model]
    prm = {m.group(1): float(m.group(2)) for m in re.finditer(r"param set-default (\w+)\s+([-\d.]+)", p.read_text(encoding="utf-8"))}
    return str(p.relative_to(ROOT)).replace("\\", "/"), vc.sha1_file(p), prm


def sdf_extra(variant, model):
    f = ROOT / "ugv" / "gazebo" / "variants" / variant / model / "model.sdf"
    m = ET.fromstring(f.read_text(encoding="utf-8")).find("model")
    steer = [float(j.find("axis/limit/upper").text) for j in m.findall("joint") if "steering" in j.get("name", "") and j.find("axis/limit/upper") is not None]
    bl = [l for l in m.findall("link") if l.get("name") == "base_link"][0]
    xs, ys = [], []
    for c in bl.findall("collision"):
        box = c.find("geometry/box/size")
        if box is None:
            continue
        sx, sy, _ = map(float, box.text.split())
        px, py = (map(float, c.find("pose").text.split()[:2]) if c.find("pose") is not None else (0.0, 0.0))
        xs += [px - sx / 2, px + sx / 2]
        ys += [py - sy / 2, py + sy / 2]
    return {"steer_max_deg": round(math.degrees(max(steer)), 3) if steer else None,
            "body_length_m": round(max(xs) - min(xs), 3) if xs else None, "body_width_m": round(max(ys) - min(ys), 3) if ys else None,
            "body_center_x": round((max(xs) + min(xs)) / 2, 3) if xs else None}


def server_profiles(tree):
    code = ("import json,dataclasses,subprocess\nfrom ugv.profiles import PROFILES\n"
            "out={k:{f.name:getattr(v,f.name) for f in dataclasses.fields(v) if f.init and isinstance(getattr(v,f.name),(int,float,str))} for k,v in PROFILES.items()}\n"
            "out['_commit']=subprocess.run(['git','rev-parse','--short','HEAD'],capture_output=True,text=True).stdout.strip()\n"
            "print(json.dumps(out))")
    r = subprocess.run([sys.executable, "-c", code], cwd=tree, capture_output=True, text=True, encoding="utf-8")
    if r.returncode:
        raise SystemExit(f"profiles 읽기 실패 ({tree}): {r.stderr[-400:]}")
    return json.loads(r.stdout.strip().splitlines()[-1])


def row(item, sdf, af, prof, tol, unit=""):
    vals = [v for v in (sdf, af, prof) if v is not None]
    ok = None if len(vals) < 2 else (max(vals) - min(vals) <= tol)
    return {"item": item, "sdf": sdf, "airframe": af, "profile": prof, "tol": tol, "unit": unit,
            "status": "INFO" if ok is None else ("MATCH" if ok else "MISMATCH")}


def compare(model, variant, prof, label):
    g = vc.sdf_info(variant, model)
    x = sdf_extra(variant, model)
    af_path, af_sha, prm = airframe(variant, model)
    rows = [
        row("축간 wheelbase", g["wheelbase"], prm.get("RA_WHEEL_BASE"), prof["wheelbase_m"], 0.01, "m"),
        row("최대 조향", x["steer_max_deg"], None if prm.get("RA_MAX_STR_ANG") is None else round(math.degrees(prm["RA_MAX_STR_ANG"]), 3), prof["steer_max_deg"], 0.5, "°"),
        row("차체 길이", x["body_length_m"], None, prof["body_length_m"], 0.05, "m"),
        row("차체 폭", x["body_width_m"], None, prof["body_width_m"], 0.05, "m"),
        row("뒤축 ↔ 차체 중심", None if x["body_center_x"] is None else round(x["body_center_x"] - g["rear_x"], 3), None, prof["rear_axle_from_center_m"], 0.05, "m"),
        row("최고속도", None, prm.get("RO_SPEED_LIM"), round(prof["max_speed_mps"], 3), 0.1, "m/s"),
    ]
    mass = {"item": "질량", "sdf": g["mass_kg"], "airframe": None, "profile": prof["mass_kg"], "tol": "2 %", "unit": "kg"}
    mass["status"] = "MATCH" if abs(g["mass_kg"] - prof["mass_kg"]) / g["mass_kg"] <= 0.02 else "MISMATCH"
    rows.append(mass)
    lim = prm.get("RO_DECEL_LIM")
    rows.append({"item": "계획 감속 ≤ PX4 RO_DECEL_LIM", "sdf": None, "airframe": lim, "profile": prof["decel_mps2"], "unit": "m/s²",
                 "status": "INFO" if lim is None else ("MATCH" if prof["decel_mps2"] <= lim else "MISMATCH")})
    rows.append({"item": "비상 감속(정지 계산)", "sdf": None, "airframe": lim, "profile": prof["emergency_decel_mps2"], "unit": "m/s²", "status": "INFO",
                 "note": "숫자 일치 불필요 — 실측 명령 정지(S4 chain_s4p)로 보수성 검증. profiles 주석 'PX4 제한보다 낮게'는 소방차(2.2 vs 2.5)에는 맞고 UGV(3.5 vs airframe 값) 확인 필요"})
    rmin = {k: (round(w / math.tan(math.radians(d)), 2) if w and d else None) for k, w, d in
            (("sdf", g["wheelbase"], x["steer_max_deg"]), ("profile", prof["wheelbase_m"], prof["steer_max_deg"]))}
    rows.append({"item": "최소 회전반경(뒤축 중심)", "sdf": rmin["sdf"], "airframe": None, "profile": rmin["profile"], "tol": 0.1, "unit": "m",
                 "status": "MATCH" if None not in rmin.values() and abs(rmin["sdf"] - rmin["profile"]) <= 0.1 else "MISMATCH"})
    return {"label": label, "model": model, "variant": variant, "model_sdf": g["model_sdf"], "model_sha1": g["model_sha1"],
            "airframe": af_path, "airframe_sha1": af_sha, "rows": rows, "mismatch": [r["item"] for r in rows if r["status"] == "MISMATCH"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server-tree", default=str(ROOT.parent / "ADAIR_wt_fix"))
    ap.add_argument("--ugv", default="susp_a")
    ap.add_argument("--fire", default="fire_v2")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    profs = server_profiles(a.server_tree)
    res = {"server_tree": a.server_tree, "server_commit": profs.pop("_commit"),
           "vehicles": [compare("adair_ugv", a.ugv, profs["adair_ugv"], "UGV"), compare("adair_firetruck", a.fire, profs["adair_firetruck"], "소방차")]}
    for v in res["vehicles"]:
        print(f"## {v['label']} {v['variant']} (SDF {v['model_sha1'][:12]}, {v['airframe']}) — 불일치 {v['mismatch'] or '없음'}")
        for r in v["rows"]:
            print(f"  {r['status']:8s} {r['item']}: SDF {r['sdf']} | airframe {r['airframe']} | profile {r['profile']} {r.get('unit', '')}")
    if a.out:
        Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    sys.exit(1 if any(v["mismatch"] for v in res["vehicles"]) else 0)


if __name__ == "__main__":
    main()
