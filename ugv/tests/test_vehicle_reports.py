# -*- coding: utf-8 -*-
"""차량 시험 판정기 회귀 (2026-10-13 S1·S3 독립 검증 R3) — 합성 기록으로 경계 사례를 만든다.
판정기 내부 식을 복제하지 않고, 입력(속도·변위·센서 상태·표시 파일)과 기대 결과(상태·항목)만 고정한다."""
import csv
import json
import math

import pytest

from ugv.tools import valid_tally
from ugv.tools import vehicle_drive_report as vdr
from ugv.tools import vehicle_report_common as vc

W = vc.W
SUSP = vc.SUSP
DT = 0.036


def write_track(path, xs, q=None, states=None, wheel_v=None, t0=29.0, drop=()):
    """xs: 시각마다 x 위치. q(i)->q 값 또는 None(결측), states(i,w)->접촉 상태, drop: 기록에서 뺄 표본 번호(공백)."""
    cols = (["sim_t", "x", "y", "z", "yaw", "roll", "pitch", "steer_l", "steer_r"] + [f"{w}_v" for w in W] + [f"{w}_c_state" for w in W]
            + [f"{s}_q" for s in SUSP] + [f"{w}_fn" for w in W] + ["wall_t"])
    with open(path, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(cols)
        for i, x in enumerate(xs):
            if i in drop:
                continue
            qq = q(i) if q else None
            wv = wheel_v(i) if wheel_v else 10.0
            wr.writerow([round(t0 + i * DT, 3), x, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0] + [wv] * 4
                        + [(states(i, w) if states else "VALID") for w in W]
                        + [("nan" if qq is None else qq) for _ in SUSP] + [20000.0] * 4 + [1000.0 + i * DT])


def bump_xs(v=5.0, x_end=80.0):
    n = int(x_end / (v * DT))
    xs = [min(i * v * DT, x_end) for i in range(n)]
    return xs + [x_end] * int(15 / DT)


def stamp_name(tmp_path, tag):
    return tmp_path / f"{tag}_20261013_000000.csv"


# ---- 1. 같은 후보 이름의 UGV·소방차는 다른 집계 키
def test_same_variant_name_ugv_and_firetruck_have_different_keys(tmp_path):
    xs = bump_xs()
    write_track(stamp_name(tmp_path, "base_adair_flat_susp_bump_a"), xs)
    write_track(stamp_name(tmp_path, "ft-base_adair_flat_susp_bump_a"), xs)
    r = [vdr.judge(p) for p in sorted(tmp_path.glob("*.csv"))]
    assert {x["model"] for x in r} == {"adair_ugv", "adair_firetruck"}
    assert len({x["key"] for x in r}) == 2


# ---- 2. 서스펜션 유무는 SDF 로 — fire_v2 는 있고 fire_v2_base 는 없다. 있는데 q 결측이면 합격 아님
def test_fire_v2_has_susp_and_base_does_not():
    assert vc.sdf_info("fire_v2", "adair_firetruck")["has_susp"] is True
    assert vc.sdf_info("fire_v2_base", "adair_firetruck")["has_susp"] is False


def test_susp_bump_with_missing_q_is_not_pass(tmp_path):
    write_track(stamp_name(tmp_path, "ft-fire_v2_adair_flat_susp_bump_a"), bump_xs(), q=lambda i: None)
    r = vdr.judge(next(tmp_path.glob("*.csv")))
    assert r["status"] == "UNVERIFIED" and r["pass"] is None


# ---- 3. 최고속도 뒤 일시 감속 → 순항 → 최종 정지: 제동 시작은 최종 감속
def test_brake_start_skips_temporary_dip():
    T, spd = [], []
    t = 0.0
    prof = [(10.0, lambda s: 14.4 * s / 10)] + [(2.0, lambda s: 14.4 - 0.6 * s / 2)] + [(2.0, lambda s: 13.8 + 0.1 * s / 2)] \
        + [(40.0, lambda s: 13.9)] + [(14.0, lambda s: max(13.9 - s, 0.0))] + [(15.0, lambda s: 0.0)]
    for dur, f in prof:
        n = int(dur / DT)
        for k in range(n):
            T.append(t)
            spd.append(f(k * DT))
            t += DT
    X = [0.0]
    for k in range(1, len(T)):
        X.append(X[-1] + spd[k] * DT)
    rows = [{f"{w}_v": str(spd[k] / 0.42) for w in W} for k in range(len(T))]
    vdr.WHEEL_R = 0.42
    br = vdr.brake_metrics(rows, T, X, [0.0] * len(T), spd, None)
    final_start_t = 10 + 2 + 2 + 40
    assert br["brake_start_t"] > final_start_t - 1.0          # 일시 감속(10~12 s)이 아니라 최종 감속(54 s 근처)
    assert br["brake_dist_m"] < 120                           # 순항 거리를 합산하지 않음


# ---- 4. 후보 치수가 다르면 요철 관측 창·바퀴 반지름이 달라진다
def test_geometry_differs_between_candidates(tmp_path):
    a, b = vc.sdf_info("fire_v2", "adair_firetruck"), vc.sdf_info("susp_a", "adair_firetruck")
    assert (a["front_x"], a["wheelbase"], a["wheel_r"]) == (2.795, 4.3, 0.42)
    assert (b["front_x"], b["wheelbase"], b["wheel_r"]) == (2.2, 3.8, 0.5)
    xs = bump_xs()
    write_track(stamp_name(tmp_path, "ft-fire_v2_adair_flat_susp_bump_a"), xs, q=lambda i: -0.03)
    r = vdr.judge(next(tmp_path.glob("*.csv")))
    assert r["geometry"]["front_x"] == 2.795 and r["geometry"]["wheel_r"] == 0.42


# ---- 5. 센서 미수신·기록 공백·빈 checks 는 합격으로 세지 않는다
def test_sensor_nomsg_is_unverified(tmp_path):
    write_track(stamp_name(tmp_path, "ft-fire_v2_adair_flat_susp_bump_a"), bump_xs(), q=lambda i: -0.03,
                states=lambda i, w: "NO_MSG" if w == "wheel_front_left" else "VALID")
    assert vdr.judge(next(tmp_path.glob("*.csv")))["status"] == "UNVERIFIED"


def test_record_gap_is_unverified(tmp_path):
    xs = bump_xs()
    k = next(i for i, x in enumerate(xs) if x >= 29.5)
    write_track(stamp_name(tmp_path, "ft-fire_v2_adair_flat_susp_bump_a"), xs, q=lambda i: -0.03, drop=set(range(k, k + 30)))
    r = vdr.judge(next(tmp_path.glob("*.csv")))
    assert r["status"] == "UNVERIFIED" and r["data_quality"]["max_gap_s"] > 0.5


def test_empty_checks_not_pass():
    assert vc.status({}, True, {}, [])[0] == "UNVERIFIED"
    assert vc.status({}, True, {"a": True}, ["a", "b"])[0] == "UNVERIFIED"


def test_single_wheel_message_gap_counts_as_contact_loss(tmp_path):
    # gz 접촉 센서는 접촉이 없으면 발행하지 않는다 → 한 바퀴 0.3 s STALE = 접촉 상실 0.3 s > 0.2 s → 불합격
    xs = bump_xs()
    k = next(i for i, x in enumerate(xs) if x >= 30.0)
    write_track(stamp_name(tmp_path, "ft-fire_v2_base_adair_flat_susp_bump_a"), xs,
                states=lambda i, w: "STALE" if (w == "wheel_front_left" and k <= i < k + 9) else "VALID")
    r = vdr.judge(next(tmp_path.glob("*.csv")))
    assert r["status"] == "FAIL" and r["checks"]["no_contact_le_0.2s"] is False


# ---- 6. 0.08013 m 변위는 기준(< 0.08) 완화 없이 불합격
def test_q_just_over_limit_fails(tmp_path):
    xs = bump_xs()
    write_track(stamp_name(tmp_path, "ft-fire_v2_adair_flat_susp_bump_a"), xs,
                q=lambda i: (0.08013 if 28.0 <= xs[i] <= 29.0 else -0.03))
    r = vdr.judge(next(tmp_path.glob("*.csv")))
    assert r["susp_q_abs_max_raw_m"] == pytest.approx(0.08013)
    assert r["checks"]["q_abs_max_lt_0.08"] is False and r["status"] == "FAIL"


def test_mission_brake_over_limit_stays_fail(tmp_path):
    # 60 km/h 직선, 임무 끝 감속 거리 약 70 m (> 60 m) → brake_dist_ok False
    v, xs, x = [], [], 0.0
    for k in range(int(80 / DT)):
        s = min(16.67, 16.67 * k * DT / 7.0)
        x += s * DT
        xs.append(x)
    for k in range(int(8.4 / DT)):
        s = max(16.67 - 2.0 * k * DT, 0.0)
        x += s * DT
        xs.append(x)
    xs += [x] * int(15 / DT)
    write_track(stamp_name(tmp_path, "susp_a_adair_flat_straight_a"), xs, q=lambda i: -0.03)
    r = vdr.judge(next(tmp_path.glob("*.csv")))
    assert r["brake_dist_m"] > 60 and r["checks"]["brake_dist_ok"] is False and r["status"] != "PASS"


# ---- 7. 실행 무효 표시·정지 실패·기록 실패는 유효 합격에서 빠진다
@pytest.mark.parametrize("mk", [".invalid_stop", ".contaminated", ".invalid_evidence", ".invalid_record"])
def test_run_marker_makes_invalid(tmp_path, mk):
    write_track(stamp_name(tmp_path, "susp_a_adair_flat_susp_bump_a"), bump_xs(), q=lambda i: -0.03)
    (tmp_path / f"susp_a_adair_flat_susp_bump_a{mk}").write_text("x", encoding="utf-8")
    r = vdr.judge(next(tmp_path.glob("*.csv")))
    assert r["status"] == "INVALID" and r["pass"] is None


@pytest.mark.parametrize("mk,cls", [(".invalid_stop", "INVALID_STOP"), (".contaminated", "INVALID_CONTAMINATED"), (".invalid_evidence", "INVALID_EVIDENCE")])
def test_valid_tally_excludes_marked_runs(tmp_path, mk, cls):
    tag = "c59A_j494873_0793_1375_D8_susp0_stall1_ctla"
    (tmp_path / f"{tag}.out").write_text("주행\n", encoding="utf-8")
    (tmp_path / f"{tag}{mk}").write_text("x", encoding="utf-8")
    assert valid_tally.classify(tag, "v5", tmp_path)[0] == cls


def test_valid_tally_record_failure(tmp_path):
    tag = "c99A_j000001_0001_0002_D8_x_r"
    (tmp_path / f"{tag}.out").write_text("임무 끝 뒤 관측\nTraceback\n", encoding="utf-8")
    assert valid_tally.classify(tag, "v5", tmp_path)[0] == "INVALID_RECORD"


# ---- 회전 시험 판정이 돌아가고 기대 항목이 모두 계산된다 (서스펜션 후보)
def test_curve_judge_computes_expected(tmp_path):
    xs = bump_xs(v=10.0, x_end=300.0)
    f = stamp_name(tmp_path, "ft-fire_v2_adair_flat_curve_a")
    write_track(f, xs, q=lambda i: -0.03)
    f.with_suffix(".plan.json").write_text(json.dumps({"plan_xy": [[0, 0], [150, 0], [300, 0]], "corners": [["C1", [150, 0], 0]]}), encoding="utf-8")
    r = vdr.judge(f)
    assert r["missing_checks"] == [] and r["status"] == "PASS"
    assert r["body_cg_height_m"] == pytest.approx(1.2) and r["static_rollover_g_est"] == pytest.approx(1.8 / 2.4, rel=1e-3)
