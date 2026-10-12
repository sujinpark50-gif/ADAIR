# -*- coding: utf-8 -*-
"""ugv/tools/ab_connect.py — A↔B 거점 연결을 차종별 세 단계로 따로 판정한다 (트윈 지도 정제 5단계).

    python -m ugv.tools.ab_connect --tiles <타일 폴더> [--out ab_connect.json]

1) 지도상 연결   : 타일 세트에 도로면이 있는 도로(교량 가정 포함)만으로 방향 그래프(일방통행만, 회전 조건 없음)에서 이어지는가
2) 기하학적 통과 : 실제 계획기 평가(회전반경·차체·경로 검사·복귀 조건, 차종별) ACCEPT 이고, 그 경로의 모든 도로가 정적 기준
                   (종단 10 %·횡단 6 %·교차로 횡단 6 %·2차 차분 0.10 m) 안인가 — 기준 밖이면 사유별로 막는 도로를 보고
3) Gazebo 완주   : 이 도구는 판정하지 않는다 (주행 기록으로 따로)
지도상 연결이 곧 통과 가능이 아니고, 계획기 ACCEPT 도 정적 기준 밖 도로를 지나면 '기하학적 통과' 가 아니다.
"""
import argparse
import asyncio
import heapq
import json
import os
import pathlib
import sys
import tempfile

LIM = {"max_long_grade": 0.10, "max_cross_slope": 0.06, "max_cross_slope_junction_zone": 0.06, "max_second_diff_m": 0.10}
NAME = {"max_long_grade": "GRADE", "max_cross_slope": "CROSS", "max_cross_slope_junction_zone": "JUNCTION_CROSS", "max_second_diff_m": "JUMP"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tiles", required=True)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    tiles = pathlib.Path(a.tiles)
    meta = json.loads((tiles / "tiles.json").read_text(encoding="utf-8"))
    ck = json.loads((tiles / "static_checks.json").read_text(encoding="utf-8"))["per_road"]
    roads = sorted({r for t in meta["tiles"].values() for r in t["roads"]})
    sub = tiles / "road_subset_all_included.json"
    sub.write_text(json.dumps({"tiles_dir": str(tiles), "roads": roads}), encoding="utf-8")
    os.environ["UGV_ROAD_SUBSET"] = str(sub)
    os.environ.setdefault("UGV_ALLOW_NO_FIRE_DATA", "1")
    root = pathlib.Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    from ugv.roads import RoadNetwork
    import ugv.tests.test_multi_vehicle as tm
    from ugv.tests.test_multi_vehicle import FLEET_CFG, make_fleet
    from ugv.approach import node_id_of
    net = RoadNetwork()
    tm.NET = net
    assumed = meta.get("roads_assumed") or {}
    cap = (meta.get("design_cap") or {}).get("roads") or {}
    cfg = {c["resource_id"]: c for c in FLEET_CFG}

    def fleet(rid):
        f, _ = make_fleet(pathlib.Path(tempfile.mkdtemp()), [cfg[rid]])
        return f.vehicles[rid]

    # 1) 지도상 연결 (방향 그래프, 일방통행만)
    g = {}
    for rid, r in net.roads.items():
        ow = r.attrs.get("oneway")
        if ow in (None, "BOTH", "A_TO_B"):
            g.setdefault(r.a, []).append((r.b, rid, r.length))
        if ow in (None, "BOTH", "B_TO_A"):
            g.setdefault(r.b, []).append((r.a, rid, r.length))

    def shortest(src_road, dst_road):
        best = None
        for s0 in (net.roads[src_road].a, net.roads[src_road].b):
            dist, prev, pq = {s0: 0.0}, {}, [(0.0, s0)]
            while pq:
                d, u = heapq.heappop(pq)
                if d != dist[u]:
                    continue
                for v, rid, L in g.get(u, []):
                    if v not in dist or d + L < dist[v]:
                        dist[v], prev[v] = d + L, (u, rid)
                        heapq.heappush(pq, (d + L, v))
            for t in (net.roads[dst_road].a, net.roads[dst_road].b):
                if t in dist and (best is None or dist[t] < best[0]):
                    path, u = [], t
                    while u != s0:
                        u, r_ = prev[u]
                        path.append(r_)
                    best = (dist[t], path[::-1])
        return best

    def blockers(route_roads):
        out = {}
        for rid in route_roads:
            c = ck.get(rid) or {}
            if not c or "error" in c:
                out.setdefault("NO_SURFACE", []).append(rid)
                continue
            for k, lim in LIM.items():
                if c[k] > lim:
                    key = NAME[k]
                    if k == "max_long_grade":
                        key += "_" + (c.get("grade_fail_kind") or "?")
                        if cap.get(rid, {}).get("status") == "NOT_RESTORED":
                            key += "(설계기준 미복구)"
                    out.setdefault(key, []).append([rid, round(c[k], 4)])
        return out

    res = {"tiles": str(tiles), "params": meta.get("params"), "stages": "1 지도상 연결 / 2 기하학적 통과 / 3 Gazebo 완주(별도)",
           "pairs": {}}
    pairs = [("A-ugv1", "B-ugv1"), ("B-ugv1", "A-ugv1"), ("A-fire1", "B-fire1"), ("B-fire1", "A-fire1")]
    for src, dst in pairs:
        Vs, Vd = fleet(src), fleet(dst)
        key = f"{src}->{dst[0]}"
        row = {"vehicle": Vs.profile.name}
        sp = shortest(Vs.access.road_id, Vd.access.road_id)
        row["1_map"] = None if sp is None else {"connected": True, "km": round(sp[0] / 1000, 1), "roads": len(sp[1]),
                                                "bridges_assumed": sum(1 for r in sp[1] if r in assumed)}
        if sp is None:
            row["1_map"] = {"connected": False}
        body = {"task_id": "AB", "decision_id": "AB", "target_node": node_id_of(Vd.access)}
        ev = asyncio.run(Vs.evaluate(body))
        apd = ev.get("approach") or {}
        st2 = {"planner": ev["verdict"], "reason": ev.get("detail") or ev.get("reason"), "eta_s": ev.get("eta_sec"),
               "no_directed_route": apd.get("candidates_no_directed_route"), "no_return_path": apd.get("candidates_no_return_path")}
        if ev["verdict"] == "ACCEPT":
            asyncio.run(Vs.execute(body))
            legs = list(dict.fromkeys(str(l[0]) for l in Vs.plan.route.legs))
            bl = blockers(legs)
            st2.update(route_km=round(Vs.plan.route.length / 1000, 1) if hasattr(Vs.plan.route, "length") else None,
                       route_roads=len(legs), bridges_assumed=sum(1 for r in legs if r in assumed),
                       blockers={k: len(v) for k, v in bl.items()}, blocker_roads=bl,
                       geometric_pass=not bl)
        else:
            st2["geometric_pass"] = False
        row["2_geometry"] = st2
        row["3_gazebo"] = "별도 (주행 기록)"
        res["pairs"][key] = row
        print(key, json.dumps({k: (v if k != "2_geometry" else {x: y for x, y in v.items() if x != "blocker_roads"}) for k, v in row.items()},
                              ensure_ascii=False)[:700], flush=True)
    p = pathlib.Path(a.out) if a.out else tiles / "ab_connect.json"
    p.write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
