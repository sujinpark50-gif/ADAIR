# -*- coding: utf-8 -*-
"""ugv/tools/valid_tally.py — 교차로 매트릭스 최종 유효 집계. judge3 판정에 무효 표시·증거 검사를 더한다 (차량 개조 독립 검증 지적 반영).

    python -m ugv.tools.valid_tally --run c56_=legacy56 --run c57_=legacy57 [--run c60_=v5] --stage4 <logs/ugv/stage4> [--out tally.json]

회차 분류:
  INVALID_NOT_RUN       <TAG>.out 에 '시험 안 함' (실행 전 검사·생성 위치·서버 준비 실패, 오염 거부 등) 또는 결과 JSON 없음
  INVALID_CONTAMINATED  <TAG>.contaminated 있음
  INVALID_STOP          <TAG>.invalid_stop 있음 (사후 종료 확인 실패)
  INVALID_EVIDENCE      <TAG>.invalid_evidence 있음, 또는 증거 규칙(아래)을 못 채움
  VALID_PASS / VALID_FAIL   judge3 최종 합격 / 불합격 (도착·차체 도로 경계·정지 확인 ≥10 s)
증거 규칙 (--run 접두어=규칙):
  v5        manifest_<TAG>.json·verify_<TAG>.txt 가 EVIDENCE_OK 이고 stop_post JSON 이 still_alive 0·residual_own 0
  legacy57  run_sc_v3 회차 (manifest 의 모델 해석 기록 결함): manifest variant == susp_a0 이고 stop_pre/post JSON 잔존 0.
            prov/<TAG>.json (진행 중 읽기 전용 감시) 이 있으면 그 모델 해시 == 35fb49b6… 이어야 함. 없으면 '보완 증거: 후보 이름·파일 이력' 표시
  legacy56  chain56 (run_sc_v2, 시험 사본 fable_models/susp 직접 지정): chain56.sh 의 GZ_MODEL_OVERRIDE 경로 고정 — 표시만
칸 = (교차로, 지면). 칸마다 유효 회차 수·합격 수. 매트릭스 완료 = 모든 칸 유효 ≥ 3.
"""
import argparse
import glob
import json
import os
import re
from pathlib import Path

from ugv.tools import judge3

SUSP_A0_SHA = "35fb49b6be916e78c1e9da9ac5d3ce8d61d3caa8"
G = Path(__file__).resolve().parents[3]
LOGS = [G / "ADAIR_wt_fix/logs/ugv", G / "ADAIR_wt_turn/logs/ugv"]


def jread(p):
    try:
        return json.loads(Path(p).read_text(encoding="utf-8").strip().splitlines()[-1] if Path(p).suffix == ".txt" else Path(p).read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return None


def clean_stop(p):
    d = jread(p)
    return bool(d) and d.get("still_alive") == 0 and d.get("residual_own") == 0


def cell_of(tag):
    m = re.match(r"c\d+[A-Z]?_(j\d+_\d+_\d+)_(D8|D5|v2)_", tag)
    return (m.group(1), m.group(2)) if m else (None, None)


def classify(tag, rule, s4):
    out_txt = (s4 / f"{tag}.out").read_text(encoding="utf-8", errors="replace") if (s4 / f"{tag}.out").exists() else ""
    if "시험 안 함" in out_txt:
        return "INVALID_NOT_RUN", out_txt.strip().splitlines()[0][:160], None
    for mk, cls in ((".contaminated", "INVALID_CONTAMINATED"), (".invalid_stop", "INVALID_STOP"), (".invalid_evidence", "INVALID_EVIDENCE")):
        if (s4 / f"{tag}{mk}").exists():
            return cls, (s4 / f"{tag}{mk}").read_text(encoding="utf-8", errors="replace").strip()[:160], None
    js = [f for L in LOGS for f in glob.glob(str(L / f"{tag}_2*.json"))]
    if not js:
        return "INVALID_NOT_RUN", "결과 JSON 없음", None
    note = ""
    if rule == "v5":
        v = (s4 / f"verify_{tag}.txt")
        if not (v.exists() and "EVIDENCE_OK" in v.read_text(encoding="utf-8", errors="replace")):
            return "INVALID_EVIDENCE", "verify 없음/실패", None
        if not clean_stop(s4 / f"stop_post_{tag}.json"):
            return "INVALID_STOP", "stop_post 잔존 또는 없음", None
    elif rule == "legacy57":
        man = jread(s4 / f"manifest_{tag}.json") or {}
        if man.get("variant") != "susp_a0":
            return "INVALID_EVIDENCE", f"manifest variant {man.get('variant')}", None
        if not (clean_stop(s4 / f"stop_pre_{tag}.json") and clean_stop(s4 / f"stop_post_{tag}.json")):
            return "INVALID_STOP", "stop_pre/post 잔존 또는 없음", None
        pv = jread(s4 / "prov" / f"{tag}.json")
        if pv:
            shas = {v.get("model_content_sha1") for v in pv.get("vehicles", []) if v.get("model") != "x500"}
            pris = {v.get("gz_prismatic_count") for v in pv.get("vehicles", []) if v.get("model") != "x500"}
            if shas != {SUSP_A0_SHA}:
                return "INVALID_EVIDENCE", f"실행 중 모델 해시 {shas}", None
            note = "실행 중 출처 확인" + (" (서스펜션 관절 4)" if pris == {4} else "")
        else:
            note = "보완 증거: manifest 후보 이름 susp_a0·후보 파일 이력 변경 없음 (실행 중 출처 기록 없음)"
    elif rule == "legacy56":
        note = "chain56: GZ_MODEL_OVERRIDE=fable_models/susp 고정 (내용 = susp_a0)"
    r = judge3.judge(sorted(js)[-1])
    return ("VALID_PASS" if r["final_pass"] else "VALID_FAIL"), note, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="append", required=True, help="접두어=규칙 (v5 | legacy57 | legacy56)")
    ap.add_argument("--stage4", default=str(G / "ADAIR/logs/ugv/stage4"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--need", type=int, default=3)
    a = ap.parse_args()
    s4 = Path(a.stage4)
    rows = []
    for spec in a.run:
        pre, rule = spec.split("=")
        tags = sorted({Path(f).stem for f in glob.glob(str(s4 / f"{pre}*.out"))})
        for t in tags:
            cls, note, r = classify(t, rule, s4)
            j, surf = cell_of(t)
            rows.append({"tag": t, "rule": rule, "junction": j, "surface": surf, "class": cls, "note": note,
                         "judge": None if r is None else {k: r[k] for k in ("arrived", "inside", "outside_max_m", "stop", "final_pass")}})
    cells = {}
    for x in rows:
        c = cells.setdefault(f"{x['junction']}|{x['surface']}", {"valid": 0, "pass": 0, "invalid": 0, "tags": []})
        c["tags"].append(f"{x['tag']}:{x['class']}")
        if x["class"].startswith("VALID"):
            c["valid"] += 1
            c["pass"] += x["class"] == "VALID_PASS"
        else:
            c["invalid"] += 1
    short = {k: a.need - v["valid"] for k, v in cells.items() if v["valid"] < a.need}
    from collections import Counter
    summ = {"runs": len(rows), "classes": dict(Counter(x["class"] for x in rows)), "cells": len(cells),
            "cells_complete": sum(v["valid"] >= a.need for v in cells.values()), "need_per_cell": a.need, "short": short,
            "valid_runs": sum(v["valid"] for v in cells.values()), "valid_pass": sum(v["pass"] for v in cells.values())}
    print(json.dumps(summ, ensure_ascii=False, indent=1))
    for x in rows:
        if not x["class"].startswith("VALID_PASS"):
            print(f"  {x['tag']}: {x['class']} {x['note']}")
    if a.out:
        Path(a.out).write_text(json.dumps({"summary": summ, "cells": cells, "runs": rows}, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
