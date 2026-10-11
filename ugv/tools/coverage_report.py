"""ugv/tools/coverage_report.py — 차량별 정적 검사·실제 주행 커버리지 (트윈 지형). logs/ugv/stage5/coverage.py 를 버전 관리로 옮김
(2026-10-11 트윈 지도 정제 1단계: --tiles/--out, 지속·국부 경사 구분·가정 표시·시험 전용 시설 따로 — 상태·기준은 그대로).
결과: <out>/coverage.json, coverage_summary.json, coverage_ugv.svg
상태 (링크·차량별, 우선순위 순):
  EXCLUDED_DATA     자료 부족·미확인으로 도로면을 만들지 않음 (교량·터널 코드, 자동차전용, DEM 밖)
  STATIC_FAIL       생성된 충돌면 정적 검사 기준 밖 (종단 > 10 %(검증 범위), 횡단 > 6 %, 2 m 2차 차분 > 0.10 m) 또는 회전 불가(차량 핵심 망 밖)
  DRIVEN_PASS       Gazebo 타일 월드에서 그 링크를 끝까지 지나감 (차체 이탈 0, 전복 없음)
  DRIVEN_FAIL       지나갔지만 이탈·전복 (원시 궤적 모든 표본 기준)
  INCOMPLETE        지나간 관측이 불완전 기록뿐 (JSON·분석 없음) — 통과로 세지 않음
  TEST_TARGET       정적 통과, 아직 주행 안 함 (다음 시험 목록)
  TEST_ONLY_FACILITY 시험 전용 덧붙임 도로 (회전 공터 등 — 트윈 km 에 넣지 않음)
  python -m ugv.tools.coverage_report --tiles <타일 폴더> --out <폴더> [--logs <logs 폴더>] [--runs 'logs/ugv/c5_tz_*.json' ...]
"""
import argparse
import glob
import re
import json
import math
import pathlib
import sys

from ugv.roads import RoadNetwork  # noqa: E402
from ugv.profiles import get  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
# 기록·자료가 있는 logs (작업트리에서 돌리면 메인 저장소 logs — logs/ 는 git 밖)
_MAIN_LOGS = ROOT.parent / "ADAIR" / "logs"
LOGS = _MAIN_LOGS if _MAIN_LOGS.exists() else ROOT / "logs"
GRADE_MAX, CROSS_MAX, JUMP_MAX = 0.10, 0.06, 0.10
VEH = {"adair_ugv": ("A-ugv1", "B-ugv1"), "adair_firetruck": ("A-fire1", "B-fire1")}


def _runs(patterns):
    """기록 → [(csv, 차량 rid, 프로필, 완결 여부, 출처)]. JSON 에 묶인 CSV 와, 같은 무늬인데 JSON 이 없는 CSV (불완전 — 통과에서 제외)."""
    out, seen = [], set()
    for pat in patterns:
        for f in sorted(glob.glob(pat)):
            if f.endswith(".partial.json") and pathlib.Path(f.replace(".partial.json", ".json")).exists():
                continue
            d = json.loads(pathlib.Path(f).read_text(encoding="utf-8"))
            ana = d.get("analysis") or {}
            # 중단·중간 기록(ABORTED·PARTIAL)은 완결 기록이 아니다 (통과로 세지 않음, 이탈은 셈)
            aborted = d.get("status") in ("ABORTED", "PARTIAL") or bool(d.get("aborted"))
            if "csv" in d and isinstance(d["csv"], dict):           # fleet_test
                for rid, csvp in d["csv"].items():
                    seen.add(str(pathlib.Path(csvp)))
                    v = (ana.get("vehicles") or {}).get(rid)
                    ok = bool(v) and "error" not in v and not aborted
                    out.append((csvp, rid, "adair_ugv" if "ugv" in rid else "adair_firetruck", ok, f))
            elif d.get("rid") and d.get("stamp"):                       # road_test
                csvp = str(pathlib.Path(str(f).replace(".json", ".csv")))
                seen.add(csvp)
                ok = bool((ana.get("overall") or {}).get("road_dev")) and not aborted
                out.append((csvp, d["rid"], "adair_ugv" if "ugv" in d["rid"] else "adair_firetruck", ok, f))
        for c in sorted(glob.glob(pat[:-5] + ".csv")):
            if str(pathlib.Path(c)) not in seen:
                m = re.search(r"_([AB]-(?:ugv|fire)1)\.csv$", c)
                rid = m.group(1) if m else ("X-fire" if "fire" in c else "X-ugv")
                out.append((c, rid, "adair_ugv" if "ugv" in rid else "adair_firetruck", False, "NO_JSON"))
                seen.add(str(pathlib.Path(c)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tiles", default=str(LOGS / "ugv" / "stage5" / "tiles"))
    ap.add_argument("--out", default=str(LOGS / "ugv" / "stage5"))
    ap.add_argument("--logs", default=str(LOGS), help="--runs 상대 무늬의 기준 폴더 (logs 의 부모 기준)")
    ap.add_argument("--runs", nargs="*", default=[
        # 지형 타일 월드 실제 주행만 (9af4505 세로 축척 수정 뒤). c5_tz_*·d1·d2 는 깨진 타일이라 뺀다. 평탄 월드 기록은 넣지 않는다
        "logs/ugv/d3_tz_*.json", "logs/ugv/d6_tz_*.json", "logs/ugv/d8_tz_*.json", "logs/ugv/c7_*.json", "logs/ugv/c8_*.json",
        "logs/ugv/c11_*.json", "logs/ugv/fleet_tz_four_20261011_0[2-9]*.json", "logs/ugv/fleet_sweep_*.json",
        "logs/ugv/fleet_follow_20261011_0[3-9]*.json"])
    a = ap.parse_args()
    base = pathlib.Path(a.logs).parent
    a.runs = [p if pathlib.Path(p).is_absolute() else str(base / p) for p in a.runs]
    OUT = pathlib.Path(a.out)
    OUT.mkdir(parents=True, exist_ok=True)
    TILES = pathlib.Path(a.tiles)
    net = RoadNetwork()
    meta = json.loads((TILES / "tiles.json").read_text(encoding="utf-8"))
    ck_all = json.loads((TILES / "static_checks.json").read_text(encoding="utf-8"))
    ck = ck_all["per_road"]
    assumed = meta.get("roads_assumed") or {}
    inv = {r["road"]: r for r in json.loads((LOGS / "ugv" / "stage5" / "road_inventory.json").read_text(encoding="utf-8"))["roads"]}
    from ugv.tools.drive_test import load_track
    from ugv.tools import coverage_judge as CJ
    driven = {p: {} for p in VEH}            # prof -> rid -> [관측 ...]
    runs_used = []
    for csvp, rid, prof, ok, src in _runs(a.runs):
        if not pathlib.Path(csvp).exists():
            runs_used.append({"csv": csvp, "rid": rid, "source": src, "status": "CSV_MISSING"})
            continue
        P = get(prof)
        obs, m = CJ.observations(net, net.graph_for(P), P, load_track(pathlib.Path(csvp)), ok, csvp)
        for link, o in obs.items():
            driven[prof].setdefault(link, []).append(o)
        runs_used.append({"csv": csvp, "rid": rid, "source": src, "complete": ok, **m})
    out = {"criteria": {"grade_max": GRADE_MAX, "cross_max": CROSS_MAX, "second_diff_max_m": JUMP_MAX,
                        "driven": "원시 궤적 연속 표본(간격 ≤ 1 s, 중심선 3 m 안)이 링크 칸(2 m, 최소 4칸)의 90 % 이상을 덮고 "
                                  "모든 표본 차체 이탈 0·전복 없음, 완결 기록(JSON·분석 있음)이면 PASS. 불완전 기록만이면 INCOMPLETE"},
           "tiles": str(TILES), "tiles_params": meta.get("params"), "runs": runs_used, "vehicles": {}}
    for prof in VEH:
        rows, tally = {}, {}
        for rid, r in net.roads.items():
            fails, jst = [], None
            if r.attrs.get("test_only"):
                st, why = "TEST_ONLY_FACILITY", "시험 전용 덧붙임 도로 (원자료 아님)"
            elif rid in meta["roads_excluded"]:
                st, why = "EXCLUDED_DATA", meta["roads_excluded"][rid]
            else:
                c = ck.get(rid) or {}
                core = (inv.get(rid) or {}).get("ugv_core" if prof == "adair_ugv" else "fire_core")
                fails = []
                if "error" in c or not c:
                    fails.append("NO_TILE")
                else:
                    if c["max_long_grade"] > GRADE_MAX:
                        fails.append(f"GRADE {c['max_long_grade'] * 100:.1f} %")
                    if c["max_cross_slope"] > CROSS_MAX:
                        fails.append(f"CROSS {c['max_cross_slope'] * 100:.1f} %")
                    # 교차로 구역 횡경사도 같은 기준 (2026-10-11 검토: 교차로 안 19 % 를 판정에서 빠뜨림). 교차로만 빼서 해결하지 않는다
                    jz = c.get("max_cross_slope_junction_zone") or 0.0
                    if jz > CROSS_MAX:
                        fails.append(f"JUNCTION_CROSS {jz * 100:.1f} %")
                    if c["max_second_diff_m"] > JUMP_MAX:
                        fails.append(f"JUMP {c['max_second_diff_m']:.2f} m")
                if not core:
                    fails.append("NOT_IN_VEHICLE_CORE (회전·차체 조건으로 방향 망 밖)")
                dv = driven[prof].get(rid)
                jst, jwhy = CJ.judge(dv or [])
                if jst:
                    st, why = jst, jwhy
                    if fails:
                        why += " (정적 기준 밖: " + ", ".join(fails) + ")"
                elif fails:
                    st, why = "STATIC_FAIL", ", ".join(fails)
                else:
                    st, why = "TEST_TARGET", ""
            static_fit = None if st in ("EXCLUDED_DATA", "TEST_ONLY_FACILITY") else not fails
            rows[rid] = {"status": st, "why": why, "len_m": round(r.length, 1),
                         # 정적 적합과 주행 결과를 따로 둔다 — 한 번 주행 성공했다고 정적 위험이 사라지지 않는다
                         "static_fit": static_fit, "static_fails": [] if st == "EXCLUDED_DATA" else fails,
                         "driven": jst, "observations": driven[prof].get(rid, []),
                         "grade_fail_kind": (ck.get(rid) or {}).get("grade_fail_kind"),
                         "grade_300m": (ck.get(rid) or {}).get("max_long_grade_300m"),
                         "assumption": (assumed.get(rid) or {}).get("kind")}
            if st == "STATIC_FAIL" and fails and all(f.startswith("GRADE") for f in fails)                     and (ck.get(rid) or {}).get("grade_fail_kind") == "SUSTAINED":
                t3 = tally.setdefault("STATIC_FAIL_GRADE_SUSTAINED_ONLY", {"links": 0, "km": 0.0})   # 보조 집계 (상태값 아님)
                t3["links"] += 1
                t3["km"] = round(t3["km"] + r.length / 1000, 2)
            if st == "DRIVEN_PASS" and static_fit is False:
                t2 = tally.setdefault("DRIVEN_PASS_BUT_STATIC_FAIL", {"links": 0, "km": 0.0})
                t2["links"] += 1
                t2["km"] = round(t2["km"] + r.length / 1000, 2)
            t = tally.setdefault(st, {"links": 0, "km": 0.0})
            t["links"] += 1
            t["km"] = round(t["km"] + r.length / 1000, 2)
        out["vehicles"][prof] = {"tally": tally, "links": rows}
    (OUT / "coverage.json").write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")   # 기계 판정 전체
    summ = {"criteria": out["criteria"], "runs": len(runs_used),
            "incomplete_runs": [r for r in runs_used if not r.get("complete")],
            "vehicles": {p: v["tally"] for p, v in out["vehicles"].items()}}
    (OUT / "coverage_summary.json").write_text(json.dumps(summ, ensure_ascii=False, indent=1), encoding="utf-8")
    # SVG 지도 (UGV)
    col = {"EXCLUDED_DATA": "#999999", "STATIC_FAIL": "#d62728", "DRIVEN_PASS": "#2ca02c", "DRIVEN_FAIL": "#ff7f0e",
           "INCOMPLETE": "#9467bd", "TEST_TARGET": "#1f77b4", "TEST_ONLY_FACILITY": "#e377c2"}
    xs = [x for r in net.roads.values() for x, _ in r.xy]
    ys = [y for r in net.roads.values() for _, y in r.xy]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    W = 1400
    sc = W / (x1 - x0)
    H = int((y1 - y0) * sc)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H + 40}" viewBox="0 0 {W} {H + 40}">',
             '<rect width="100%" height="100%" fill="white"/>']
    for prof, dy in (("adair_ugv", 0),):
        for rid, r in net.roads.items():
            st = out["vehicles"][prof]["links"][rid]["status"]
            pts = " ".join(f"{(x - x0) * sc:.1f},{(y1 - y) * sc:.1f}" for x, y in r.xy)
            parts.append(f'<polyline points="{pts}" fill="none" stroke="{col[st]}" stroke-width="{3 if st.startswith("DRIVEN") else 1.4}"/>')
    lx = 10
    for st, c in col.items():
        parts.append(f'<rect x="{lx}" y="{H + 12}" width="14" height="14" fill="{c}"/><text x="{lx + 18}" y="{H + 24}" '
                     f'font-size="13" font-family="sans-serif">{st}</text>')
        lx += 200
    parts.append("</svg>")
    (OUT / "coverage_ugv.svg").write_text("\n".join(parts), encoding="utf-8")
    for prof in VEH:
        print(prof, out["vehicles"][prof]["tally"])
    return out


if __name__ == "__main__":
    main()
