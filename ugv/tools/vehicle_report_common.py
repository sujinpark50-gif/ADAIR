# -*- coding: utf-8 -*-
"""ugv/tools/vehicle_report_common.py — 차량 시험 판정기(정지·주행) 공통: 후보 SDF 제원, 기록 품질, 실행 유효성 표시 (2026-10-13 S1·S3 독립 검증 R1).

판정 상태 (모든 회차):
  INVALID     실행 래퍼 표시(.contaminated·.invalid·.invalid_evidence·.invalid_record·.invalid_stop) — 시험 무효, 같은 조건으로 보충
  UNVERIFIED  기록 품질 부족(센서 미수신·기록 공백·서스펜션 관절 값 결측) 또는 기대 검사 항목 누락 — 합격으로 세지 않음
  PASS / FAIL 기대 검사 항목이 모두 계산됐을 때만
기록 품질 기준 (이 파일에서 처음 정함, 결과 보기 전 고정 — 판정 구간 안):
  바퀴별 (NO_MSG 또는 네 바퀴 동시 끊김) 표본 비율 ≤ 2 % — 한 바퀴만 끊긴 STALE·NO_CONTACT 는 접촉 없음 후보 (센서가 접촉 없을 때 발행 안 함), 시뮬레이션 시각 최대 공백 ≤ 0.5 s,
  서스펜션 후보의 관절 q 결측(NaN·빈칸) 비율 ≤ 2 %.
"""
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
W = ["wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"]
SUSP = ["susp_front_left", "susp_front_right", "susp_rear_left", "susp_rear_right"]
SENSOR_BAD_MAX = 0.02
GAP_MAX_S = 0.5
Q_NAN_MAX = 0.02
MARKERS = (".contaminated", ".invalid", ".invalid_evidence", ".invalid_record", ".invalid_stop")


def sha1_file(p):
    return hashlib.sha1(Path(p).read_bytes().replace(b"\r", b"")).hexdigest()


def _x(pose_el):
    return float(pose_el.text.split()[0]) if pose_el is not None and pose_el.text else 0.0


def sdf_info(variant, model):
    """후보 SDF: 앞·뒤축 x, 축간, 바퀴 반지름, 서스펜션(prismatic) 관절·행정, 총질량·무게중심 x, 정적 앞축 하중 비율.
    무게중심 = 링크 pose(relative_to 따라 모델 기준으로) + 관성 pose 의 x 만 합성 (바퀴 링크 회전은 x 축 회전이라 x 성분 불변 — 단순화).
    앞축 비율 = (x_cg − x_r)/(x_f − x_r): 강체·평지·바퀴 4점 지지 단순화 (서스펜션·타이어 변형에 따른 하중 이동 무시)."""
    f = ROOT / "ugv" / "gazebo" / "variants" / variant / model / "model.sdf"
    m = ET.fromstring(f.read_text(encoding="utf-8")).find("model")
    links = {l.get("name"): l for l in m.findall("link")}
    joints = {j.get("name"): j for j in m.findall("joint")}
    memo = {}

    def frame_x(name, depth=0):
        if name in (None, "", "__model__") or depth > 20:
            return 0.0
        if name in memo:
            return memo[name]
        el = links.get(name) if name in links else joints.get(name)
        if el is None:
            return 0.0
        pe = el.find("pose")
        rel = pe.get("relative_to") if pe is not None else None
        if rel is None and el.tag == "joint":
            rel = el.find("child").text           # 관절 기본 기준 = 자식 링크
        memo[name] = _x(pe) + frame_x(rel, depth + 1)
        return memo[name]

    mass = mx = 0.0
    for n, l in links.items():
        ine = l.find("inertial")
        if ine is None or ine.find("mass") is None:
            continue
        mk = float(ine.find("mass").text)
        mass += mk
        mx += mk * (frame_x(n) + _x(ine.find("pose")))
    xf, xr = frame_x("wheel_front_left_link"), frame_x("wheel_rear_left_link")
    wl = links["wheel_front_left_link"]
    pris = [j for j in joints.values() if j.get("type") == "prismatic"]
    lim = {j.get("name"): (float(j.find("axis/limit/lower").text), float(j.find("axis/limit/upper").text)) for j in pris}
    x_cg = mx / mass if mass else None
    return {"model_sdf": str(f.relative_to(ROOT)).replace("\\", "/"), "model_sha1": sha1_file(f),
            "front_x": round(xf, 4), "rear_x": round(xr, 4), "wheelbase": round(xf - xr, 4),
            "wheel_r": float(wl.find(".//collision//radius").text), "has_susp": bool(pris), "susp_joints": sorted(lim), "joint_limits": lim,
            "mass_kg": round(mass, 3), "x_cg": None if x_cg is None else round(x_cg, 4),
            "front_share_cg": None if x_cg is None else round((x_cg - xr) / (xf - xr), 4)}


def quality(rows, T, has_susp, i0=0, i1=None):
    """판정 구간 rows[i0:i1] 의 기록 품질. ok=False 면 UNVERIFIED."""
    i1 = len(rows) if i1 is None else i1
    seg = rows[i0:i1]
    n = len(seg)
    out = {"samples": n}
    if n < 10:
        return {**out, "ok": False, "why": ["표본 < 10"]}
    why = []
    # Gazebo 접촉 센서는 접촉이 없으면 메시지를 보내지 않는다 → 한 바퀴만 끊긴 STALE 은 '접촉 없음 후보'(판정에서 접촉 상실로 셈)이고
    # 품질 결함이 아니다. 품질 결함 = 그 바퀴 메시지가 한 번도 없음(NO_MSG) 또는 네 바퀴가 동시에 끊김(센서·기록 전체 중단)
    allbad = [all(r.get(f"{w}_c_state") != "VALID" for w in W) for r in seg]
    bad = {w: round(sum(r.get(f"{w}_c_state") in ("NO_MSG", "", None) or allbad[k] for k, r in enumerate(seg)) / n, 4) for w in W}
    out["contact_nomsg_or_all_dropped_ratio"] = bad
    out["contact_single_wheel_gap_ratio"] = {w: round(sum(r.get(f"{w}_c_state") in ("STALE", "NO_CONTACT") and not allbad[k] for k, r in enumerate(seg)) / n, 4) for w in W}
    if any(v > SENSOR_BAD_MAX for v in bad.values()):
        why.append(f"접촉 센서 미수신·네 바퀴 동시 끊김 비율 > {SENSOR_BAD_MAX}")
    gap = max((T[k + 1] - T[k] for k in range(i0, i1 - 1)), default=0.0)
    out["max_gap_s"] = round(gap, 3)
    if gap > GAP_MAX_S:
        why.append(f"기록 공백 {gap:.2f} s > {GAP_MAX_S}")
    if has_susp:
        def missing(v):
            try:
                return math.isnan(float(v))
            except (TypeError, ValueError):
                return True
        qn = {s: round(sum(missing(r.get(f"{s}_q")) for r in seg) / n, 4) for s in SUSP}
        out["susp_q_missing_ratio"] = qn
        if any(v > Q_NAN_MAX for v in qn.values()):
            why.append(f"서스펜션 관절 q 결측 비율 > {Q_NAN_MAX} (SDF 에 서스펜션 있음)")
    out["ok"] = not why
    out["why"] = why
    return out


def run_markers(folder, tag):
    """실행 래퍼 무효 표시와 증거 (run_drive_v2: verify_<TAG>.txt EVIDENCE_OK / v1 run_drive: 사후 정지 JSON 만)."""
    folder = Path(folder)
    found = {mk: (folder / f"{tag}{mk}").read_text(encoding="utf-8", errors="replace").strip()[:200]
             for mk in MARKERS if (folder / f"{tag}{mk}").exists()}
    v = folder / f"verify_{tag}.txt"
    if v.exists():
        ev = "EVIDENCE_OK (run_drive_v2: 모델 해시·Gazebo 관절)" if "EVIDENCE_OK" in v.read_text(encoding="utf-8", errors="replace") else "EVIDENCE_FAIL"
    else:
        ev = "출처 미확인 (run_drive v1·susp_static_test: 실행 중 모델 해시·사전 정지 결과 기록 없음)"
    return found, ev


def status(markers, quality_ok, checks, expected):
    missing = [k for k in expected if k not in checks or checks[k] is None]
    if markers:
        return "INVALID", missing
    if not quality_ok or missing or not checks:
        return "UNVERIFIED", missing
    return ("PASS" if all(checks.values()) else "FAIL"), missing
