# -*- coding: utf-8 -*-
"""ugv/tools/susp_static_report.py — 정지·대각 시험(S1·S2) 판정 (susp_static_test.sh 의 상세 기록 CSV).

    python -m ugv.tools.susp_static_report <폴더> [--out report.json]

기준은 결과를 보기 전에 정한 것 (차량 개조 구현 계획 4.4절), 정착 10 s 뒤 구간만 본다:
 S1 평탄(adair_flat_susp): ① Σfn 과 m·g 오차 ≤ 1 % ② (서스펜션 후보) 네 바퀴 fn 이 평균 ±10 % 안, 서스펜션 관절 q 네 값 차 ≤ 3 mm
    ③ 차체 z 잔류 진동 진폭(마지막 20 s 의 (최대−최소)/2) < 1 mm ④ q_eq 기록
 소방차(태그 ft-): 평탄은 축별 좌우 ±10 %·앞축 하중 비율이 설계값(SDF 바퀴 위치) ±3 %p·축별 좌우 q 차 ≤ 3 mm, 대각은 축별 평균 대비 최소 fn.
 S2 대각(adair_flat_diag2/5): 최소 바퀴 fn ≥ 평균 × 0.7 (2 cm) / × 0.5 (5 cm), 네 바퀴 VALID 비율 ≥ 99 %.
    base(서스펜션 없음)는 대조 — 판정하지 않고 기록만.
m = 후보 model.sdf 질량 합, g = 월드 중력 9.8 m/s² (adair_flat 계열 <gravity>0 0 -9.8>).
힘은 VALID 표본만 쓴다 (미수신·오래됨은 빈칸 — 0 으로 세지 않음).
v2 (2026-10-13 S1·S3 독립 검증 R1): 서스펜션 유무·질량·축 위치·무게중심은 후보 SDF (vehicle_report_common.sdf_info). 소방차 설계 앞축 비율은
링크·관성 pose 로 구한 총무게중심 기준 (첫 구현의 x_cg=0 가정값은 참고 열 front_share_design_v1). 판정 상태 PASS/FAIL/UNVERIFIED/INVALID/CONTROL —
기대 검사 항목이 모두 계산되지 않았거나 기록 품질이 부족하면 UNVERIFIED (빈 checks 를 합격으로 세지 않음). 서스펜션 없는 대각 판 = CONTROL (판정 안 함).
"""
import argparse
import csv
import subprocess
import json
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

from ugv.tools import vehicle_report_common as vc

JUDGE_VERSION = "ssr-2 (2026-10-13: SDF 제원·무게중심 축하중·품질·상태)"
ROOT = Path(__file__).resolve().parents[2]
W = ["wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"]
SUSP = ["susp_front_left", "susp_front_right", "susp_rear_left", "susp_rear_right"]
G = 9.8
SETTLE_S = 10.0


def model_mass(variant, model="adair_ugv"):
    p = ROOT / "ugv" / "gazebo" / "variants" / variant / model / "model.sdf"
    m = ET.fromstring(p.read_text(encoding="utf-8")).find("model")
    return sum(float(l.find("inertial/mass").text) for l in m.findall("link") if l.find("inertial/mass") is not None)


def design_front_share(variant, model):
    """후보 SDF 의 바퀴 x 위치와 차체 무게중심(x=0 가정 확인)으로 정적 앞축 하중 비율 (바퀴·링크 질량 포함 근사)."""
    p = ROOT / "ugv" / "gazebo" / "variants" / variant / model / "model.sdf"
    m = ET.fromstring(p.read_text(encoding="utf-8")).find("model")
    xs = {}
    for l in m.findall("link"):
        if l.get("name") in ("wheel_front_left_link", "wheel_rear_left_link"):
            xs[l.get("name")] = float(l.find("pose").text.split()[0])
    xf, xr = xs["wheel_front_left_link"], xs["wheel_rear_left_link"]
    return -xr / (xf - xr)


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def judge(csv_path):
    tag = csv_path.stem
    model = "adair_firetruck" if tag.startswith("ft-") else "adair_ugv"
    variant, world = tag.removeprefix("ft-").split("_adair_", 1)
    world = "adair_" + world
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
    if not rows:
        return {"tag": tag, "invalid": "기록 없음"}
    t0 = float(rows[0]["sim_t"])
    rows = [r for r in rows if float(r["sim_t"]) - t0 >= SETTLE_S]
    if len(rows) < 100:
        return {"tag": tag, "invalid": f"정착 뒤 표본 {len(rows)} < 100"}
    geo = vc.sdf_info(variant, model)
    mass = geo["mass_kg"]
    mg = mass * G
    valid_ratio = {w: sum(r[f"{w}_c_state"] == "VALID" for r in rows) / len(rows) for w in W}
    all4 = [r for r in rows if all(r[f"{w}_c_state"] == "VALID" for w in W)]
    fn_med = {w: (statistics.median([fnum(r[f"{w}_fn"]) for r in rows if r[f"{w}_c_state"] == "VALID"])
                  if any(r[f"{w}_c_state"] == "VALID" for r in rows) else None) for w in W}
    sum_fn = statistics.median([sum(fnum(r[f"{w}_fn"]) for w in W) for r in all4]) if all4 else None
    sum_fz = statistics.median([sum(fnum(r[f"{w}_fz"]) for w in W) for r in all4]) if all4 else None
    z = [float(r["z"]) for r in rows]
    tl = float(rows[-1]["sim_t"])
    zl = [float(r["z"]) for r in rows if tl - float(r["sim_t"]) <= 20.0]
    q = {s: (statistics.median([fnum(r[f"{s}_q"]) for r in rows if fnum(r[f"{s}_q"]) is not None])
             if any(fnum(r[f"{s}_q"]) is not None for r in rows) else None) for s in SUSP}
    has_susp = geo["has_susp"]   # 이름(susp*)이 아니라 SDF prismatic 관절 (fire_v2 도 서스펜션)
    out = {"tag": tag, "key": f"{model}|{variant}|{world}", "csv_sha1": vc.sha1_file(csv_path), "geometry": geo, "judge_version": JUDGE_VERSION, "model": model, "variant": variant, "world": world, "mass_kg": mass, "mg_N": round(mg, 1), "samples": len(rows),
           "all4_valid_samples": len(all4), "valid_ratio": {w: round(v, 4) for w, v in valid_ratio.items()},
           "fn_median_N": {w: (None if v is None else round(v, 1)) for w, v in fn_med.items()},
           "sum_fn_median_N": None if sum_fn is None else round(sum_fn, 1), "sum_fz_median_N": None if sum_fz is None else round(sum_fz, 1),
           "body_z_mean_m": round(statistics.mean(z), 4), "body_roll_mean_rad": round(statistics.mean(float(r["roll"]) for r in rows), 5),
           "body_pitch_mean_rad": round(statistics.mean(float(r["pitch"]) for r in rows), 5),
           "residual_z_amp_mm": round((max(zl) - min(zl)) / 2 * 1000, 3), "susp_q_eq_m": q}
    checks = {}
    fns = [v for v in fn_med.values() if v is not None]
    mean_fn = sum(fns) / len(fns) if len(fns) == 4 else None
    if world == "adair_flat_susp" and model == "adair_firetruck":
        # 소방차 (후속 계획 S3-1): 앞뒤 축하중이 달라 네 바퀴 동일 기준을 쓰지 않는다 — 축별 좌우 균형·설계 축하중 비율·축별 좌우 q 차
        checks["sum_fn_vs_mg_le_1pct"] = sum_fn is not None and abs(sum_fn - mg) / mg <= 0.01
        checks["residual_z_lt_1mm"] = out["residual_z_amp_mm"] < 1.0
        if len(fns) == 4:
            fl, fr, rl, rr = (fn_med[w] for w in W)
            out["front_share"] = round((fl + fr) / sum(fns), 4)
            out["front_share_design_v1"] = round(design_front_share(variant, model), 4)   # 첫 구현 (x_cg = 0 가정)
            out["front_share_design"] = geo["front_share_cg"]                                # 총무게중심 기준 (강체·평지 4점 지지 단순화)
            checks["axle_lr_within_10pct"] = abs(fl - fr) / ((fl + fr) / 2) <= 0.10 and abs(rl - rr) / ((rl + rr) / 2) <= 0.10
            checks["front_share_within_3pp"] = abs(out["front_share"] - out["front_share_design"]) <= 0.03
        else:
            checks["axle_lr_within_10pct"] = False
        if has_susp:
            qs = [q[s_] for s_ in SUSP]
            checks["q_lr_diff_le_3mm"] = (None not in qs and abs(qs[0] - qs[1]) <= 0.003 and abs(qs[2] - qs[3]) <= 0.003)
    elif world == "adair_flat_susp":
        checks["sum_fn_vs_mg_le_1pct"] = sum_fn is not None and abs(sum_fn - mg) / mg <= 0.01
        checks["residual_z_lt_1mm"] = out["residual_z_amp_mm"] < 1.0
        if has_susp:
            checks["fn_within_10pct"] = mean_fn is not None and all(abs(v - mean_fn) / mean_fn <= 0.10 for v in fns)
            qs = [v for v in q.values() if v is not None]
            checks["q_spread_le_3mm"] = (max(qs) - min(qs) <= 0.003) if len(qs) == 4 else None   # 관절 미발행(susp_a0)이면 판정 불가
    elif world.startswith("adair_flat_diag"):
        lim = 0.7 if "diag2" in world else 0.5
        out["min_fn_over_mean"] = None if mean_fn is None else round(min(fns) / mean_fn, 3)
        if model == "adair_firetruck" and len(fns) == 4:
            fl, fr, rl, rr = (fn_med[w] for w in W)
            out["min_fn_over_axle_mean"] = round(min(fl / ((fl + fr) / 2), fr / ((fl + fr) / 2), rl / ((rl + rr) / 2), rr / ((rl + rr) / 2)), 3)
        if has_susp:
            if model == "adair_firetruck":   # 축별 평균 기준 (앞뒤 하중이 다름)
                checks[f"min_fn_ge_{lim}_axle_mean"] = out.get("min_fn_over_axle_mean") is not None and out["min_fn_over_axle_mean"] >= lim
            else:
                checks[f"min_fn_ge_{lim}_mean"] = mean_fn is not None and min(fns) / mean_fn >= lim
            checks["all_valid_ge_99pct"] = all(v >= 0.99 for v in valid_ratio.values())
        else:
            out["note"] = "base(서스펜션 없음) 대조 — 판정하지 않음"
    out["checks"] = checks
    T = [float(r["sim_t"]) for r in rows]
    qual = vc.quality(rows, T, has_susp)
    out["data_quality"] = qual
    if world.startswith("adair_flat_diag"):
        exp = ([f"min_fn_ge_{lim}_axle_mean" if model == "adair_firetruck" else f"min_fn_ge_{lim}_mean", "all_valid_ge_99pct"] if has_susp else [])
    elif model == "adair_firetruck":
        exp = ["sum_fn_vs_mg_le_1pct", "residual_z_lt_1mm", "axle_lr_within_10pct", "front_share_within_3pp"] + (["q_lr_diff_le_3mm"] if has_susp else [])
    else:
        exp = ["sum_fn_vs_mg_le_1pct", "residual_z_lt_1mm"] + (["fn_within_10pct", "q_spread_le_3mm"] if has_susp else [])
    out["expected_checks"] = exp
    markers, evidence = vc.run_markers(csv_path.parent, tag)
    out["run_markers"], out["run_evidence"] = markers, evidence
    if world.startswith("adair_flat_diag") and not has_susp and not markers:
        out["status"], out["missing_checks"] = "CONTROL", []
    else:
        out["status"], out["missing_checks"] = vc.status(markers, qual["ok"], checks, exp)
    out["pass"] = out["status"] == "PASS" if out["status"] in ("PASS", "FAIL") else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--out", default=None)
    ap.add_argument("--baseline", default=None, help="이전 report.json — 회차별 이전 판정을 함께 남긴다")
    a = ap.parse_args()
    res = [judge(p) for p in sorted(Path(a.folder).glob("*.csv"))]
    commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    base = {}
    if a.baseline and Path(a.baseline).exists():
        base = {b["tag"]: b for b in json.loads(Path(a.baseline).read_text(encoding="utf-8")) if isinstance(b, dict) and "tag" in b}
    for r in res:
        r["judge_commit"] = commit
        b = base.get(r["tag"])
        if b:
            r["previous"] = {"pass": b.get("pass"), "checks": b.get("checks"), "judge_version": b.get("judge_version", "ssr-1")}
    for r in res:
        if "invalid" in r:
            print(f"{r['tag']:34s} 무효: {r['invalid']}")
            continue
        print(f"{r['tag']:34s} m {r['mass_kg']:.0f} Σfn {r['sum_fn_median_N']} (mg {r['mg_N']}) fn {list(r['fn_median_N'].values())} "
              f"z진동 {r['residual_z_amp_mm']} mm q {[None if v is None else round(v, 4) for v in r['susp_q_eq_m'].values()]} "
              f"→ {r['checks']} {r['status']}" + (f" 누락 {r['missing_checks']}" if r.get("missing_checks") else "")
              + (f" 품질 {r['data_quality']['why']}" if r["data_quality"].get("why") else "")
              + (f" [이전 {r['previous']['pass']}]" if r.get("previous") else "")
              + (f" 앞축비 실측 {r['front_share']} 설계(CG) {r['front_share_design']} 설계(v1) {r['front_share_design_v1']}" if "front_share" in r else ""))
    if a.out:
        Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
