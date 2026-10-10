# -*- coding: utf-8 -*-
"""ugv/tools/pair_test.py — 두 차량이 같은 교차로·같은 좁은 도로로 동시에 들어오는 Gazebo 시험 (모의 시험과 같은 배치).

    python -m ugv.tools.pair_test make junction     # 차량 설정 JSON·스폰 자세 파일을 logs/ugv/ 에 만든다
    (WSL) bash ugv/tools/wsl_start_fleet.sh logs/ugv/pair_junction_spawn.txt
    UGV_VEHICLES_JSON=logs/ugv/pair_junction_fleet.json UGV_DRIVER=px4 uvicorn ugv.server:app --port 8100
    python -m ugv.tools.pair_test run junction

junction: 다른 거점 차량(A-ugv1·B-fire1)이 교차로 494928 로 각자 다른 도로에서 110 m 떨어져 출발 → 같은 도로 682501929 로 합류
          (X 목적지 170 m, Y 60 m). 기대: 늦게 오는 차가 교차로 앞에서 기다린다(YIELD), 차체 겹침 0, 간격 > 1 m.
head_on:  A-ugv1·A-fire1 이 공유 도로 683400826 양 끝에서 70 m 떨어져 출발, 서로 반대 방향으로 그 도로를 지나야 함.
          기대: UGV 가 그 도로에 들어가기 전에 기다린다(YIELD HEAD_ON, 소방차 우선), 겹침 0.
배치는 ugv/tests/test_multi_vehicle.py 의 JUNCTION·HEAD_ON (진행 방향·복귀 조건으로 두 차 모두 계획되는 곳, 2026-10-10).
판정은 fleet_test.analyze (Gazebo 자세만, 차종 치수 vs 추정 도로 띠, 차량 쌍 겹침).
"""

import argparse
import json
import math
import time
from pathlib import Path

from ugv.approach import node_id_of
from ugv.geo import FRAME, bearing_deg
from ugv.roads import RoadNetwork
from ugv.tools import fleet_test
from ugv.tools.drive_test import LOG_DIR
from ugv.tools.spawn_pose import Z

JUNCTION = ("494928", "682501307", "682501539", "682501929")        # 노드, X 진입, Y 진입, 합류 도로
HEAD_ON = ("683400826", "683400838", "683400828", "683401696", "683400719")   # 공유 도로, X 진입, Y 진입, X 진출, Y 진출
CFG = {c["resource_id"]: c for c in fleet_test.FLEET}


def _pose(net, rid, node, dist):
    r = net.roads[rid]
    s = r.length - dist if node == r.b else dist
    p = net.at(rid, s)
    q = net.at(rid, s + 1 if node == r.b else s - 1)
    return p, q


def layout(sc):
    """[(차량 ID, 진입 도로, 진입 노드, 출발 거리 m, 목표 도로, 목표 s m)]"""
    net = RoadNetwork()
    if sc == "junction":
        n, r1, r2, r3 = JUNCTION
        q3 = net.roads[r3]
        s = lambda d: d if q3.a == n else q3.length - d
        return net, [("A-ugv1", r1, n, 110, r3, s(170.0)), ("B-fire1", r2, n, 110, r3, s(60.0))]
    rid, ra, rb, rc, rd = HEAD_ON
    r, qc, qd = net.roads[rid], net.roads[rc], net.roads[rd]
    return net, [("A-ugv1", ra, r.a, 70, rc, 60.0 if qc.a == r.b else qc.length - 60.0),
                 ("A-fire1", rb, r.b, 70, rd, 60.0 if qd.a == r.a else qd.length - 60.0)]


def make(sc):
    net, lay = layout(sc)
    fleet, spawn = [], []
    for rid, road, node, dist, *_ in lay:
        p, q = _pose(net, road, node, dist)
        lat, lon = FRAME.to_ll(p.x, p.y)
        fleet.append({**CFG[rid], "initial": {"lat": lat, "lon": lon, "heading_deg": round(bearing_deg(q.x - p.x, q.y - p.y), 2)}})
        yaw = math.atan2(q.y - p.y, q.x - p.x)
        spawn.append(f'{CFG[rid]["px4"]["instance"]} {CFG[rid]["profile"]} {p.x:.2f},{p.y:.2f},{Z[CFG[rid]["profile"]]},{yaw:.4f} {rid}')
    fp, sp = LOG_DIR / f"pair_{sc}_fleet.json", LOG_DIR / f"pair_{sc}_spawn.txt"
    fp.write_text(json.dumps(fleet, ensure_ascii=False, indent=1), encoding="utf-8")
    sp.write_text("\n".join(spawn) + "\n", encoding="utf-8")
    print(fp, sp, *spawn, sep="\n")


def run(sc, track_s):
    net, lay = layout(sc)
    r = fleet_test.Run(f"pair_{sc}", track_s, rids=[x[0] for x in lay])
    r.log["layout"] = [{"rid": a, "entry_road": b, "node": c, "start_m": d, "dest_road": e, "dest_s_m": round(f, 1)}
                       for a, b, c, d, e, f in lay]
    keys = []
    for rid, _, _, _, road, s in lay:                  # 두 차를 거의 동시에 출발
        key = f"P-{sc[:2].upper()}-{rid}"
        rec = r.go(rid, key, node=node_id_of(net.at(road, s)))
        r.log.setdefault("routes", {})[rid] = r.c.get(f"/ugv/{rid}/route").json()
        if rec["status_code"] == 200:
            keys.append((rid, key))
    r.wait(keys, track_s * 0.8, every=1.0)
    return r.finish()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=("make", "run"))
    ap.add_argument("scenario", choices=("junction", "head_on"))
    ap.add_argument("--track-s", type=float, default=600)
    a = ap.parse_args()
    make(a.scenario) if a.action == "make" else run(a.scenario, a.track_s)


if __name__ == "__main__":
    main()
