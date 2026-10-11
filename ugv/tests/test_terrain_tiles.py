"""트윈 지형 타일 생성기 — 설정 기록·시험 전용 도로 가드 (2026-10-11 트윈 지도 정제 1단계)."""
import pytest

from ugv.tools import build_terrain_tiles as BT


def test_tiles_meta_records_params():
    p = BT.build_params()
    for k in ("junction_mode", "junc_level_sigma_m", "road_surface_sigma_m", "blend_m", "git_head", "extra_roads"):
        assert k in p
    assert p["junction_mode"] in ("terrain", "level")


def test_test_only_roads_rejected_in_twin_build(monkeypatch):
    class R:
        def __init__(self, test):
            self.attrs = {"test_only": True} if test else {}

    class Net:
        roads = {"1": R(False), "T1": R(True)}

    monkeypatch.setattr(BT, "RoadNetwork", lambda: Net())
    monkeypatch.delenv("ADAIR_ALLOW_TEST_ROADS", raising=False)
    with pytest.raises(SystemExit, match="시험 전용"):
        BT.build_all(only={"NONE"})


# ---- 2단계: 자료 교량(FACIL_KIND=1) 직선 상판 가정 ----
import numpy as np  # noqa: E402

_NET_F = {}


def _net_f():
    if not _NET_F:
        from ugv.roads import RoadNetwork
        net = RoadNetwork()
        F = BT.Fields()
        idx = {}
        for rid, r in net.roads.items():
            if str(r.attrs.get("facil_kind")) in ("1", "2") or r.attrs.get("motorway"):
                continue
            q = BT._dense_xy(r.xy, 1.0)
            idx[rid] = (q, q.min(0), q.max(0))
        _NET_F.update(net=net, F=F, idx=idx)
    return _NET_F["net"], _NET_F["F"], _NET_F["idx"]


def test_bridge_rule_examples():
    net, F, idx = _net_f()
    why, rec = BT.bridge_rule(net, F, "683400007", None, idx)          # A–B 경로 교량 149 m
    assert why is None and rec["kind"] == "BRIDGE_DECK_STRAIGHT" and 4.0 < rec["deck_above_dem_max_m"] <= BT.BRIDGE_MAX_DECK_ABOVE_DEM_M
    why, rec = BT.bridge_rule(net, F, "682501943", None, idx)          # 평행 도로 6.6 m — 높이 같아 포함
    assert why is None and len(rec["near_roads"]) == 2 and rec["deck_minus_roadz_max_m"] <= 0.5
    why, _ = BT.bridge_rule(net, F, "683402043", None, idx)            # 자동차전용
    assert why.startswith("FACIL_KIND_1 MOTORWAY")
    big = [r for r, x in net.roads.items() if str(x.attrs.get("facil_kind")) == "1" and not x.attrs.get("motorway")]
    whys = [BT.bridge_rule(net, F, r, None, idx)[0] or "" for r in big]
    assert any(w.startswith("BRIDGE_DECK_ABOVE_DEM") for w in whys)    # 계곡 고가는 성토로 만들지 않는다


def test_bridge_deck_endpoints_equal_node_roadz(monkeypatch):
    net, F, _ = _net_f()
    monkeypatch.setattr(BT, "_BRIDGE_INC", {"683400007"})
    BT._DENSE.pop("683400007", None)
    P, z = BT.road_dense(net, F, "683400007")
    BT._DENSE.pop("683400007", None)
    EN = np.array([BT.to5186_exact(*P[0]), BT.to5186_exact(*P[-1])])
    rz = F.road_z(EN[:, 0], EN[:, 1])
    assert abs(z[0] - rz[0]) < 1e-9 and abs(z[-1] - rz[1]) < 1e-9          # 이웃 도로와 노드 높이가 같다 (교차로 혼합 불변량)
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    assert np.allclose(z, z[0] + (z[-1] - z[0]) * s / s[-1])               # 누적 거리 비례 직선
