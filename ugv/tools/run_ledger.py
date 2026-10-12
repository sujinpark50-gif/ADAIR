# -*- coding: utf-8 -*-
"""ugv/tools/run_ledger.py — 시험 회차 장부 (소방차·A↔B·통합 후속 계획 S0-2). 한 회차 = 한 줄, 같은 run_id 는 한 번만.

    python -m ugv.tools.run_ledger [--stage4 <logs/ugv/stage4>] [--out-prefix <경로 앞부분>]  → <앞부분>.csv · .json

필드: run_id, stage, vehicle_kind, resource_id, direction, world, variant, code_hash, model_hash, tile_hash, flags,
      valid, invalid_reason, arrived, inside, stopped, evidence_paths, evidence_note
- 유효성은 valid_tally.classify (무효 표시·증거 규칙·judge3) 를 그대로 쓴다. 무효(INVALID_*)·기능 불합격(VALID_FAIL)·합격(VALID_PASS) 구분.
- model_hash / tile_hash: 실행 중 출처 기록(prov_<TAG>.json 또는 prov/<TAG>.json)이 있으면 그 값. 없으면
  model_hash = 후보 파일 내용 해시(근거: 후보 이름·파일 이력)로, tile_hash = 현재 디스크의 월드·타일로 다시 계산하고 evidence_note 에 '복구(현재 디스크)' 로 표시.
- code_hash: manifest 의 서버·도구 커밋. 없으면 그 체인 스크립트의 고정값(wt_turn 30457e7, wt_fix 5ffb41a)을 '체인 고정' 으로.
단계 표시(stage): c56 S1-선별, c57/c57r S1-매트릭스, c58 S1-원인비교, c59A/B/C S1-안전(A 정상·B 대조·C 고장 유도).
"""
import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

from ugv.tools import valid_tally as VT

G = Path(__file__).resolve().parents[3]
TILES_ROOT = G / "ADAIR" / "logs" / "ugv" / "stage5"
RULES = [("c56_", "legacy56", "S1-선별"), ("c57r_", "v5", "S1-매트릭스"), ("c57_", "legacy57", "S1-매트릭스"), ("c58_", "v5", "S1-원인비교"),
         ("c59A_", "v5", "S1-안전 A 정상"), ("c59B_", "v5", "S1-안전 B 대조"), ("c59C_", "v5", "S1-안전 C 고장유도")]
CHAIN_FIXED = {"legacy56": "server wt_turn 30457e7 / tool wt_fix 5ffb41a (체인 고정)", "legacy57": "server wt_turn 30457e7 / tool wt_fix 5ffb41a (체인 고정)"}


def sha(p):
    return hashlib.sha1(Path(p).read_bytes().replace(b"\r", b"")).hexdigest()


def tile_digest_now(world, tiles):
    wf = TILES_ROOT / tiles / f"{world}.sdf"
    if not wf.exists():
        return None
    parts = []
    for t in sorted(set(re.findall(r"model://(T\d+_\d+)", wf.read_text(encoding="utf-8")))):
        d = TILES_ROOT / tiles / t
        if not (d / "heightmap.png").exists():
            return None
        parts.append(f"{t}{sha(d / 'heightmap.png')}{sha(d / 'model.sdf')}")
    return hashlib.sha1("".join(parts).encode()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage4", default=str(G / "ADAIR/logs/ugv/stage4"))
    ap.add_argument("--out-prefix", default=str(G / "ADAIR/logs/ugv/ledger/run_ledger"))
    a = ap.parse_args()
    s4 = Path(a.stage4)
    rows, seen = [], set()
    for pre, rule, stage in RULES:
        for f in sorted(s4.glob(f"{pre}*.out")):
            tag = f.stem
            if tag in seen:
                continue
            seen.add(tag)
            cls, note, r = VT.classify(tag, rule, s4)
            man = VT.jread(s4 / f"manifest_{tag}.json") or {}
            pv = VT.jread(s4 / f"prov_{tag}.json") or VT.jread(s4 / "prov" / f"{tag}.json")
            pre_txt = (s4 / f"preflight_{tag}.json")
            world = man.get("world")
            tiles = man.get("tiles")
            if not world:
                m = re.search(r"(j\d+_\d+_\d+)_(D8|D5|v2)", tag)
                if m:
                    world = "w" + m.group(1)
                    tiles = {"D8": "tiles_v2_D8all", "D5": "tiles_v2_D5all", "v2": "tiles_v2"}[m.group(2)]
            variant = man.get("variant") or ("susp(fable 사본=susp_a0)" if rule == "legacy56" else None)
            veh = (pv or {}).get("vehicles") or []
            v0 = veh[0] if veh else {}
            ev_note = [note] if note else []
            model_hash = v0.get("model_content_sha1")
            if not model_hash and variant:
                vf = G / "ADAIR_wt_vehicle/ugv/gazebo/variants" / ("susp_a0" if rule.startswith("legacy") else variant) / "adair_ugv/model.sdf"
                if vf.exists():
                    model_hash = sha(vf)
                    ev_note.append("model_hash 복구(후보 파일)")
            tile_hash = ((pv or {}).get("world") or {}).get("tiles_digest")
            if not tile_hash and world and tiles:
                tile_hash = tile_digest_now(world, tiles)
                if tile_hash:
                    ev_note.append("tile_hash 복구(현재 디스크)")
            code = (f"server {man.get('server_tree', 'ADAIR_wt_turn')} {man.get('server')} / tool wt_fix {man.get('tool')}" if man.get("server")
                    else CHAIN_FIXED.get(rule, ""))
            j = re.search(r"(j\d+_\d+_\d+)", tag)
            ev = [p.name for p in [s4 / f"{tag}.out", s4 / f"manifest_{tag}.json", s4 / f"prov_{tag}.json", s4 / "prov" / f"{tag}.json",
                                   s4 / f"server_evidence_{tag}.json", s4 / f"verify_{tag}.txt", s4 / f"stop_pre_{tag}.json", s4 / f"stop_post_{tag}.json"] if p.exists()]
            rows.append({"run_id": tag, "stage": stage, "vehicle_kind": v0.get("model", "adair_ugv"), "resource_id": v0.get("resource_id", ""),
                         "direction": f"교차로 회전 {j.group(1)}" if j else "", "world": world, "variant": variant, "code_hash": code,
                         "model_hash": model_hash, "tile_hash": tile_hash, "flags": man.get("env", ""),
                         "valid": cls.startswith("VALID"), "invalid_reason": "" if cls.startswith("VALID") else f"{cls}: {note}",
                         "result_class": cls, "arrived": None if r is None else r["arrived"], "inside": None if r is None else r["inside"],
                         "stopped": None if r is None else r["stop"], "evidence_paths": ";".join(ev), "evidence_note": "; ".join(ev_note)})
    out = Path(a.out_prefix)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out.with_suffix(".csv"), "w", encoding="utf-8-sig", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    out.with_suffix(".json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    from collections import Counter
    print(len(rows), "회차", dict(Counter((x["stage"], x["result_class"]) for x in rows)))


if __name__ == "__main__":
    main()
