# -*- coding: utf-8 -*-
"""ugv/tools/route_audit.py — 진행 방향 규칙(ugv/drive_graph.py)을 적용한 도로망 통계. 재현용.

같은 도로망·같은 코드·같은 난수 시드면 같은 결과가 나온다. 결과: ugv/data/route_audit.json (조건·수치·목록)

    python -m ugv.tools.route_audit             # 기본 시드 1, 무작위 300쌍
    python -m ugv.tools.route_audit --seed 2 --pairs 1000

수치
  - 도로 수·길이·일방통행 수 (원자료 gpkg 를 직접 읽음, ugv/road_source.py. 자동차전용 도로 제외)
  - 막다른 노드: 원자료 범위(= 환경 격자 경계) 5 m 안 → CLIP_BOUNDARY (지도를 자른 자리), 나머지 → OTHER
    (실제 막다른 길인지, 자료에서 끊긴 길인지는 이 자료만으로 구분할 수 없다 — 미확인)
  - 회전: 일방통행을 지키고 같은 도로 되짚기를 뺀 회전 수, 그중 기하 검사(차체·추정 도로 폭)로 막힌 수
  - 급커브로 쓰지 않는 도로
  - 거점 출입 지점으로 돌아올 수 있는 도로 길이 (방향별): 그 방향으로 서 있는 차가 출입 지점까지 갈 수 있나
  - 서로 오갈 수 있는 가장 큰 묶음 (방향 상태의 강연결 성분)
  - 무작위 (출발 방향 상태, 목표 도로 지점) 쌍: 방향 규칙으로 도달 불가 비율, 도달 가능하면 이전 무방향 경로
    (U턴·일방통행 역주행 허용, 노드 그래프 최단 거리) 대비 거리 비율
모든 기하 판정은 근사다 (drive_graph.py 머리말). 도로 폭은 차로 수 추정치다.
"""

import argparse
import collections
import heapq
import json
import math
import random
import sys
import time
from pathlib import Path

from ugv import config
from ugv.drive_graph import R_MIN, DirPos, access_point
from ugv.roads import RoadNetwork

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "ugv" / "data" / "route_audit.json"


def clip_class(net, nodes):
    from rasterio.warp import transform
    ext = net.clip_extent
    lats = [FRAME_LL[n][0] for n in nodes]
    lons = [FRAME_LL[n][1] for n in nodes]
    xs, ys = transform("EPSG:4326", f"EPSG:{ext['srs_id']}", lons, lats)
    out = {}
    for n, x, y in zip(nodes, xs, ys):
        d = min(x - ext["min_x"], ext["max_x"] - x, y - ext["min_y"], ext["max_y"] - y)
        out[n] = ("CLIP_BOUNDARY" if d <= 5.0 else "OTHER", round(d, 1))
    return out


def km_by_direction(net, ok_state):
    both = one = none = 0.0
    for rid, r in net.roads.items():
        k = sum(1 for to in (r.a, r.b) if ok_state((rid, to)))
        if k == 2:
            both += r.length
        elif k == 1:
            one += r.length
        else:
            none += r.length
    return {"both_directions_km": round(both / 1000, 2), "one_direction_km": round(one / 1000, 2),
            "neither_km": round(none / 1000, 2)}


def undirected_len(net, src_pos, dst_pos):
    """이전 방식: U턴·일방통행 역주행 허용, 노드 그래프 거리."""
    r0 = net.roads[src_pos.road_id]
    best = {r0.a: src_pos.s, r0.b: r0.length - src_pos.s}
    pq = [(d, n) for n, d in best.items()]
    heapq.heapify(pq)
    while pq:
        d, n = heapq.heappop(pq)
        if d > best[n]:
            continue
        for rid, to in net.adj.get(n, ()):
            nd = d + net.roads[rid].length
            if nd < best.get(to, math.inf):
                best[to] = nd
                heapq.heappush(pq, (nd, to))
    r1 = net.roads[dst_pos.road_id]
    opts = [best.get(r1.a, math.inf) + dst_pos.s, best.get(r1.b, math.inf) + r1.length - dst_pos.s]
    if src_pos.road_id == dst_pos.road_id:
        opts.append(abs(src_pos.s - dst_pos.s))
    return min(opts)


FRAME_LL = {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--pairs", type=int, default=300)
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    t0 = time.time()
    net = RoadNetwork()
    g = net.graph
    from ugv.geo import FRAME
    for n, (x, y) in net.nodes.items():
        FRAME_LL[n] = FRAME.to_ll(x, y)
    res = {"conditions": {
        "network": str(config.ROAD_GPKG.relative_to(ROOT)).replace("\\", "/"),
        "excluded": net.excluded,
        "rules": "ugv/drive_graph.py: 같은 도로 즉시 되짚기 금지, 일방통행, 차체가 도로 안에 들어오는 회전만, 급커브 도로 제외",
        "vehicle": {"wheelbase_m": config.WHEELBASE_M, "steer_max_deg": config.STEER_MAX_DEG,
                    "body_m": [config.BODY_LENGTH_M, config.BODY_WIDTH_M], "r_min_rear_axle_m": round(R_MIN, 2)},
        "road_width": f"추정치: 원자료 차로 수 × {config.LANE_WIDTH_EST_M} m + 길어깨 {config.SHOULDER_EST_M} m × 2 "
                      "(원자료 WIDTH 는 1~4 코드, 의미·단위 미확인이라 쓰지 않음)",
        "turn_check": "근사: 뒤축이 두 중심선에 접하는 원호, 차체 네 모서리가 그 노드 도로 띠 안. 교차로 확장면·우측 통행 미반영",
        "random_pairs": {"seed": a.seed, "n": a.pairs, "start": "방향 상태(도로, 향하는 노드) 균등, 도로 가운데, 정차(v0=0)",
                         "target": "도로 표본점(25 m 간격) 균등", "baseline": "이전 무방향 경로 (U턴·역주행 허용) 거리"}}}
    # 도로·일방통행
    res["roads"] = {"count": len(net.roads), "total_km": round(sum(r.length for r in net.roads.values()) / 1000, 2),
                    "oneway": sum(1 for r in net.roads.values() if r.attrs.get("oneway", "BOTH") != "BOTH")}
    # 막다른 노드
    dead = sorted(n for n, adj in net.adj.items() if len(adj) == 1)
    cls = clip_class(net, dead)
    res["dead_end_nodes"] = {"count": len(dead),
                             "by_class": dict(collections.Counter(c for c, _ in cls.values())),
                             "note": "OTHER = 실제 막다른 길인지 자료에서 끊긴 길인지 미확인",
                             "list": [{"node": n, "class": cls[n][0], "dist_to_clip_edge_m": cls[n][1]} for n in dead]}
    # 회전
    legal = sum(len(ts) for (rin, n), ts in g.turns.items() if g.allowed(rin, g.other(rin, n)))
    blocked = [b for b in g.blocked_turns if b["entry_legal"]]
    res["turns"] = {"legal_entry_total": legal + len(blocked), "allowed": legal, "blocked_by_geometry": len(blocked),
                    "blocked_by_deflection_deg": dict(sorted(collections.Counter(
                        f"{int(b['deflect_deg'] // 30) * 30}-{int(b['deflect_deg'] // 30) * 30 + 30}" for b in blocked).items())),
                    "blocked_list": blocked}
    res["sharp_bend_roads"] = [{"road_id": r, "min_radius_m": round(g.min_radius[r], 2),
                                "length_m": round(net.roads[r].length, 1)} for r in sorted(g.sharp_roads)]
    # 방향 상태
    states = g.states()
    comps = g.components()
    big = set(comps[0])
    res["strongly_connected"] = {"states_total": len(states), "largest_states": len(big),
                                 "largest_pct": round(100 * len(big) / len(states), 1),
                                 **km_by_direction(net, lambda s: s in big)}
    acc = access_point(net, "A")
    start = g.dirpos(acc[0], acc[1])
    rs = g.return_set(acc[0])
    res["return_to_station_A"] = {
        "access_point": {"road_id": acc[0].road_id, "s_m": round(acc[0].s, 1), "heading_deg": round(acc[1], 1)},
        "start_state_in_largest_component": (start.road_id, start.toward) in big,
        "meaning": "그 방향으로 서 있는 차가 출입 지점 A 까지 갈 수 있는 도로 길이 (화재 없음)",
        **km_by_direction(net, lambda s: any(t.to in rs["states"] for t in g.turns.get(s, ())) or s in rs["states"])}
    # 무작위 쌍
    rnd = random.Random(a.seed)
    samples = net.samples()
    unreachable, ratios, examples = 0, [], []
    for _ in range(a.pairs):
        rid, to = rnd.choice(states)
        r = net.roads[rid]
        sp = DirPos(net.at(rid, r.length / 2), to)
        p = rnd.choice(samples)
        tree = g.search(sp, 0.0)
        arrs = g.arrivals(tree, p)
        if not arrs:
            unreachable += 1
            continue
        route = g.build_route(tree, p, min(arrs, key=lambda x: x.time_s))
        base = undirected_len(net, sp.pos, p)
        if base > 200:
            ratios.append(route.length / base)
            if route.length / base > 5 and len(examples) < 5:
                examples.append({"start": [rid, to], "target": [p.road_id, round(p.s, 1)],
                                 "directed_m": round(route.length), "undirected_m": round(base)})
    ratios.sort()
    q = lambda f: round(ratios[min(len(ratios) - 1, int(f * len(ratios)))], 2) if ratios else None
    res["random_pairs"] = {"unreachable": unreachable, "unreachable_pct": round(100 * unreachable / a.pairs, 1),
                           "detour_ratio": {"n": len(ratios), "median": q(0.5), "p90": q(0.9), "max": q(1.0) if ratios else None},
                           "large_detour_examples": examples}
    res["runtime_s"] = round(time.time() - t0, 1)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(res, ensure_ascii=False, indent=1) + "\n")
    short = {k: v for k, v in res.items() if k not in ("conditions",)}
    short["dead_end_nodes"] = {k: v for k, v in res["dead_end_nodes"].items() if k != "list"}
    short["turns"] = {k: v for k, v in res["turns"].items() if k != "blocked_list"}
    print(json.dumps(short, ensure_ascii=False, indent=1))
    print("썼음:", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
