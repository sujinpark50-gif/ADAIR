# -*- coding: utf-8 -*-
"""ugv/tools/susp_static_report.py — 정지·대각 시험(S1·S2) 판정 (susp_static_test.sh 의 상세 기록 CSV).

    python -m ugv.tools.susp_static_report <폴더> [--out report.json]

기준은 결과를 보기 전에 정한 것 (차량 개조 구현 계획 4.4절), 정착 10 s 뒤 구간만 본다:
 S1 평탄(adair_flat_susp): ① Σfn 과 m·g 오차 ≤ 1 % ② (서스펜션 후보) 네 바퀴 fn 이 평균 ±10 % 안, 서스펜션 관절 q 네 값 차 ≤ 3 mm
    ③ 차체 z 잔류 진동 진폭(마지막 20 s 의 (최대−최소)/2) < 1 mm ④ q_eq 기록
 S2 대각(adair_flat_diag2/5): 최소 바퀴 fn ≥ 평균 × 0.7 (2 cm) / × 0.5 (5 cm), 네 바퀴 VALID 비율 ≥ 99 %.
    base(서스펜션 없음)는 대조 — 판정하지 않고 기록만.
m = 후보 model.sdf 질량 합, g = 월드 중력 9.8 m/s² (adair_flat 계열 <gravity>0 0 -9.8>).
힘은 VALID 표본만 쓴다 (미수신·오래됨은 빈칸 — 0 으로 세지 않음).
"""
import argparse
import csv
import json
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
W = ["wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"]
SUSP = ["susp_front_left", "susp_front_right", "susp_rear_left", "susp_rear_right"]
G = 9.8
SETTLE_S = 10.0


def model_mass(variant, model="adair_ugv"):
    p = ROOT / "ugv" / "gazebo" / "variants" / variant / model / "model.sdf"
    m = ET.fromstring(p.read_text(encoding="utf-8")).find("model")
    return sum(float(l.find("inertial/mass").text) for l in m.findall("link") if l.find("inertial/mass") is not None)


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def judge(csv_path):
    tag = csv_path.stem
    variant, world = tag.split("_adair_", 1)
    world = "adair_" + world
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
    if not rows:
        return {"tag": tag, "invalid": "기록 없음"}
    t0 = float(rows[0]["sim_t"])
    rows = [r for r in rows if float(r["sim_t"]) - t0 >= SETTLE_S]
    if len(rows) < 100:
        return {"tag": tag, "invalid": f"정착 뒤 표본 {len(rows)} < 100"}
    mass = model_mass(variant)
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
    has_susp = variant.startswith("susp")
    out = {"tag": tag, "variant": variant, "world": world, "mass_kg": mass, "mg_N": round(mg, 1), "samples": len(rows),
           "all4_valid_samples": len(all4), "valid_ratio": {w: round(v, 4) for w, v in valid_ratio.items()},
           "fn_median_N": {w: (None if v is None else round(v, 1)) for w, v in fn_med.items()},
           "sum_fn_median_N": None if sum_fn is None else round(sum_fn, 1), "sum_fz_median_N": None if sum_fz is None else round(sum_fz, 1),
           "body_z_mean_m": round(statistics.mean(z), 4), "body_roll_mean_rad": round(statistics.mean(float(r["roll"]) for r in rows), 5),
           "body_pitch_mean_rad": round(statistics.mean(float(r["pitch"]) for r in rows), 5),
           "residual_z_amp_mm": round((max(zl) - min(zl)) / 2 * 1000, 3), "susp_q_eq_m": q}
    checks = {}
    fns = [v for v in fn_med.values() if v is not None]
    mean_fn = sum(fns) / len(fns) if len(fns) == 4 else None
    if world == "adair_flat_susp":
        checks["sum_fn_vs_mg_le_1pct"] = sum_fn is not None and abs(sum_fn - mg) / mg <= 0.01
        checks["residual_z_lt_1mm"] = out["residual_z_amp_mm"] < 1.0
        if has_susp:
            checks["fn_within_10pct"] = mean_fn is not None and all(abs(v - mean_fn) / mean_fn <= 0.10 for v in fns)
            qs = [v for v in q.values() if v is not None]
            checks["q_spread_le_3mm"] = (max(qs) - min(qs) <= 0.003) if len(qs) == 4 else None   # 관절 미발행(susp_a0)이면 판정 불가
    elif world.startswith("adair_flat_diag"):
        lim = 0.7 if world.endswith("diag2") else 0.5
        out["min_fn_over_mean"] = None if mean_fn is None else round(min(fns) / mean_fn, 3)
        if has_susp:
            checks[f"min_fn_ge_{lim}_mean"] = mean_fn is not None and min(fns) / mean_fn >= lim
            checks["all_valid_ge_99pct"] = all(v >= 0.99 for v in valid_ratio.values())
        else:
            out["note"] = "base(서스펜션 없음) 대조 — 판정하지 않음"
    out["checks"] = checks
    decided = [v for v in checks.values() if v is not None]
    out["pass"] = (all(decided) and len(decided) == len(checks)) if checks else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    res = [judge(p) for p in sorted(Path(a.folder).glob("*.csv"))]
    for r in res:
        if "invalid" in r:
            print(f"{r['tag']:34s} 무효: {r['invalid']}")
            continue
        print(f"{r['tag']:34s} m {r['mass_kg']:.0f} Σfn {r['sum_fn_median_N']} (mg {r['mg_N']}) fn {list(r['fn_median_N'].values())} "
              f"z진동 {r['residual_z_amp_mm']} mm q {[None if v is None else round(v, 4) for v in r['susp_q_eq_m'].values()]} "
              f"→ {r['checks']} {'합격' if r['pass'] else ('대조' if r['pass'] is None else '불합격')}")
    if a.out:
        Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
