"""시험 전용 회전 공터(덧붙임 도로)와 같은 자리 재통과 경로의 진행 위치 (2026-10-11 회전 공터 시험)."""
import json
import math

from ugv import config
from ugv import vehicle as VM


def _out_and_back(L=20.0, loop=100.0, step=2.0, off=0.3):
    """출발 → 앞 L m → (회전 공터 loop m, 옆으로 비켜 돎) → 같은 도로로 출발점을 지나 더 내려감. 되돌아오는 선은 off m 옆."""
    pts = [(x, 0.0) for x in [i * step for i in range(int(L / step) + 1)]]
    n = int(loop / step)
    for k in range(1, n + 1):                      # 공터: 반원 비슷하게 돌아 옆 차로로
        a = math.pi * k / n
        pts.append((L + 5.0 * math.sin(a), off - off * math.cos(a) + 0.0))
    pts += [(x, off) for x in [L - i * step for i in range(1, int((L + 40) / step) + 1)]]
    cum = [0.0]
    for p, q in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist(p, q))
    return pts, cum


def test_progress_seg_prefers_first_pass_when_route_revisits_start():
    pts, _ = _out_and_back()
    # 차는 출발점 근처, 되돌아오는 선(옆 0.3 m)이 오히려 더 가깝게 놓인 경우
    x, y = 0.5, 0.2
    d_near, i_near = VM._dist_to_polyline(x, y, pts)
    d, i = VM._progress_seg(x, y, pts)
    assert i_near > 10                             # 가장 가까운 구간은 나중 통과 (예전 판정 → 공터 지점 모두 빠짐)
    assert i == 0 and d <= d_near + 1.0            # 진행 위치는 첫 통과


def test_progress_seg_takes_later_pass_only_when_clearly_closer():
    pts, _ = _out_and_back(off=3.0)
    x, y = 4.0, 3.0                                # 되돌아오는 선 위 (첫 통과에서 3 m) — 1 m 동률 밖
    d, i = VM._progress_seg(x, y, pts)
    assert i > 10


def test_extra_roads_loaded_only_when_configured(tmp_path, monkeypatch):
    from ugv.road_source import load
    base = load()
    n0 = len(base["roads"])
    nid = base["roads"][0]["node_a"]
    lat, lon = base["nodes"][nid]
    extra = {"nodes": {"TESTM": [lat + 0.0003, lon]},
             "roads": [{"road_id": "TX1", "node_a": nid, "node_b": "TESTM", "name": "시험용 회전 공터", "speed_kmh": 20.0,
                        "geometry": [[lat, lon], [lat + 0.0003, lon]], "attrs": {"oneway": "A_TO_B", "lanes": 2}}]}
    p = tmp_path / "extra.json"
    p.write_text(json.dumps(extra), encoding="utf-8")
    monkeypatch.setattr(config, "EXTRA_ROADS", str(p))
    doc = load()
    assert len(doc["roads"]) == n0 + 1
    tx = [r for r in doc["roads"] if r["road_id"] == "TX1"][0]
    assert tx["attrs"]["test_only"] is True and doc["counts"]["extra_test_roads"] == 1
    monkeypatch.setattr(config, "EXTRA_ROADS", "")
    assert len(load()["roads"]) == n0
