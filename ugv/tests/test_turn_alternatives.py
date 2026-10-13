"""S6 짧은 도로 연속 회전 — 대안 원호 (2026-10-13, Codex 답변 1626·1705).
실제 도로망의 B→A 단절 지점 683500209 → 683500210(34.9 m) → 683500214: 기본 원호 조합(진입 접선 13.16 + 진출 25.58 > 34.9)은 겹치고,
같은 차체 검사를 통과하는 더 작은 반경(93.53 m, 접선 17.05)으로는 이어진다."""
import math

from ugv.approach import verify_route
from ugv.drive_graph import DirPos
from ugv.roads import RoadNetwork

NET = RoadNetwork()
G = NET.graph
A, B, C, D = "683500209", "683500210", "683500214", "683500211"
N_AB, N_BC = "495818", "495804"


def _turn(frm, node, to):
    return next(t for t in G.turns[(frm, node)] if t.to[0] == to)


def _legs():
    to_c = G.other(C, N_BC)
    return [(A, G.other(A, N_AB), N_AB), (B, N_AB, N_BC), (C, N_BC, to_c)]


def _start():
    r = NET.roads[A]
    s = r.length - 200.0 if N_AB == r.b else 200.0       # 첫 교차로 200 m 앞 (감속 여유 충분)
    return DirPos(NET.at(A, s), N_AB)


def _dest():
    r = NET.roads[C]
    return NET.at(C, 30.0 if G.other(C, N_BC) == r.b else r.length - 30.0)


def test_alternatives_pass_the_same_body_check_and_are_shorter():
    t = _turn(B, N_BC, C)
    assert t.radius_m == 140.3 and t.alts                 # 기본은 그대로 (가장 큰 반경)
    u, w = G._dir(B, N_BC, out=False), G._dir(C, N_BC, out=True)
    fit = dict(G._fit_candidates(u, w, G._strips(N_BC), NET.roads[B].length, NET.roads[C].length))
    for R, T, m, v in t.alts:
        assert R < t.radius_m and T < t.tangent_m
        assert m >= G.p.track_margin_m and abs(fit[R] - m) < 1e-9      # 같은 검사·원래 정밀도
        assert abs(T - R * math.tan(math.radians(t.deflect_deg) / 2)) < 0.05
        assert v <= t.v_turn + 1e-9                                     # 작은 반경 = 낮은 회전 속도


def test_primary_combination_overlaps_but_alternative_fits():
    tin = _turn(A, N_AB, B).tangent_m
    t = _turn(B, N_BC, C)
    L = NET.roads[B].length
    assert not G._lab_ok(tin, t.tangent_m, L)
    assert any(G._lab_ok(tin, a[1], L) for a in t.alts)


def test_chain_picks_largest_feasible_alternative_and_keeps_primary_elsewhere():
    sel = G.choose_turn_variants(_legs(), _start(), 0.0, _dest())
    assert sel is not None and len(sel) == 2
    assert not sel[0].alt_used and sel[0].radius_m == _turn(A, N_AB, B).radius_m
    assert sel[1].alt_used and abs(sel[1].radius_m - 93.53) < 0.01


def test_chain_keeps_primary_when_primary_fits():
    to_d = G.other(D, N_BC)
    legs = [(A, G.other(A, N_AB), N_AB), (B, N_AB, N_BC), (D, N_BC, to_d)]
    r = NET.roads[D]
    dest = NET.at(D, 20.0 if to_d == r.b else r.length - 20.0)
    sel = G.choose_turn_variants(legs, _start(), 0.0, dest)
    assert sel is not None and not any(t.alt_used for t in sel)


def test_entry_choice_decides_next_turn():
    """진출 회전(210→214)에 대안이 없어도 진입 회전(209→210)의 대안(접선 8.77)으로 34.9 m 안에 들어가면 이어진다."""
    t = _turn(B, N_BC, C)
    saved = t.alts
    try:
        t.alts = ()
        sel = G.choose_turn_variants(_legs(), _start(), 0.0, _dest())
        assert sel is not None and sel[0].alt_used and not sel[1].alt_used
        assert G._lab_ok(sel[0].tangent_m, sel[1].tangent_m, NET.roads[B].length)
    finally:
        t.alts = saved


def test_infeasible_chain_is_rejected_not_drawn_with_primary():
    t, t0 = _turn(B, N_BC, C), _turn(A, N_AB, B)
    saved, saved0 = t.alts, t0.alts
    try:
        t.alts = ()                                       # 두 회전 모두 대안이 없으면 이 연속 회전은 불가
        t0.alts = ()
        assert G.choose_turn_variants(_legs(), _start(), 0.0, _dest()) is None
        assert G._chain_fail and G._chain_fail["node"] in (N_AB, N_BC)   # 끝까지 이을 수 없는 첫 회전에서 실패 표시

        class R:
            turn_chain_fail = dict(G._chain_fail)
        v = verify_route(NET, R(), G)
        assert v["ok"] is False and v["issue"]["kind"] == "TURN_CHAIN_INFEASIBLE"
    finally:
        t.alts, t0.alts = saved, saved0


def test_built_route_carries_selected_radius_into_smoothing(monkeypatch):
    st = _start()
    tree = G.search(st, 0.0)
    arr = [a for a in G.arrivals(tree, _dest()) if a.kind == "VIA"]
    assert arr                                            # 대안 덕분에 도달 (전에는 꼬리표 없음)
    seen = {}
    orig = G._smooth

    def spy(pts, vlim, legs, relax=frozenset(), sel=None):
        seen["sel"] = sel
        return orig(pts, vlim, legs, relax, sel)
    monkeypatch.setattr(G, "_smooth", spy)
    route = G.build_route(tree, _dest(), arr[0])
    assert route.turn_chain_fail is None
    used = [t for t in seen["sel"] if t.alt_used]
    assert any(abs(t.radius_m - 93.53) < 0.01 and t.to[0] == C for t in used)
    assert any(abs(r_ - 93.53) < 0.01 for _, r_, _ in route.turn_variants)


def test_arc_omit_eligibility_boundary_and_record():
    """이음매 완화 3단계 자격 = 방향선 어긋남 ≥ JUNCTION_ARC_OMIT_MIN_DEG (경계 포함) — 19.9 아니오 / 20.0 예 / 20.1 예, 판단 기록 (Codex 2100)."""
    from types import SimpleNamespace
    from ugv.approach import arc_omit_eligible
    from ugv.drive_graph import JUNCTION_ARC_OMIT_MIN_DEG
    assert JUNCTION_ARC_OMIT_MIN_DEG == 20.0
    pair = (("r1", "n"), ("r2", "m"))
    log = []
    for deg, want in ((19.9, False), (20.0, True), (20.1, True)):
        g = SimpleNamespace(junction_dirline_mismatch_deg=lambda p, d=deg: d)
        assert arc_omit_eligible(g, pair, log) is want
    assert [x["eligible"] for x in log] == [False, True, True]
    assert [x["mismatch_deg"] for x in log] == [19.9, 20.0, 20.1] and log[0]["pair"] == [["r1", "n"], ["r2", "m"]]


def test_arc_omit_mismatch_known_turns():
    """실제 도로망: 확인 사례(굽은 진출 도로) 32° → 자격, 회귀 사례(직선 도로 87.7° 회전) 0° → 자격 없음."""
    from ugv.approach import arc_omit_eligible
    assert arc_omit_eligible(G, (("682600487", "495178"), ("682501903", "495053")))
    assert not arc_omit_eligible(G, (("683400306", "495666"), ("683400364", "x")))
    assert abs(G.junction_dirline_mismatch_deg((("682600487", "495178"), ("682501903", "495053"))) - 32.0) < 0.5
