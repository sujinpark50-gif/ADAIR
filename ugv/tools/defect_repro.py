# -*- coding: utf-8 -*-
"""ugv/tools/defect_repro.py — 2026-10-10 검토 두 결함의 재현 (수정 전 코드에서도 돌도록 공통 API 만 쓴다).

    python -m ugv.tools.defect_repro            # 결과 JSON 을 표준 출력으로

결함 1: 출발 도로의 급커브(684500365, 최소 반경 3.51 m)를 지나 같은 도로 앞쪽 목표로 — 승인되면 안 된다.
결함 2: 화재 이탈 후보의 최종 검사를 모두 실패로 주입 — 이탈 계획이 OK 를 돌려주면 안 된다.
"""
import json
import math
import sys

from ugv.approach import plan_approach, plan_evasion
from ugv.drive_graph import DirPos, DriveGraph, access_point
from ugv.fire import FireArea
from ugv.geo import FRAME
from ugv.roads import RoadNetwork


def main():
    net = RoadNetwork()
    g = net.graph
    acc = access_point(net, "A")
    out = {"defect1": [], "defect2": None}
    rid, toward = "684500365", "496432"
    r = net.roads[rid]
    for s in (5.0, 36.0, 72.0):
        st = DirPos(net.at(rid, s if toward == r.b else r.length - s), toward)
        tgt = net.at(rid, r.length * 0.6 if toward == r.b else r.length * 0.4)
        plan = plan_approach(net, FireArea([], source="T"), st, *FRAME.to_ll(tgt.x, tgt.y), access=acc[0])
        rc = (plan.detail or {}).get("route_check") or {}
        indep = g.check_route(plan.route) if plan.route is not None else None
        out["defect1"].append({"start_s_m": s, "status": plan.status, "reason": plan.reason,
                               "reported_check_ok": rc.get("ok"), "min_radius_m": rc.get("min_radius_m"),
                               "radius_violations_recorded": rc.get("radius_violations"),
                               "independent_full_check_radius_violations":
                                   None if indep is None else sum(1 for i in _all_issues(g, plan.route) if "RADIUS" in i["kind"]
                                                                  or "STEERING" in i["kind"]),
                               "approved_with_violation": plan.status == "OK" and bool(rc.get("radius_violations"))})
    # 결함 2: 화재 옆, 모든 후보 검사 실패 주입
    here = g.dirpos(acc[0], acc[1])
    from ugv.tests.test_stage3_drive import fire_block
    offs = [(rr * math.cos(math.radians(a)), rr * math.sin(math.radians(a))) for rr in (60, 75, 90) for a in range(0, 360, 30)]
    cells = next(c for c in (fire_block(*FRAME.to_ll(here.pos.x + dx, here.pos.y + dy), n=0) for dx, dy in offs) if c)
    orig = DriveGraph.check_route

    def fail(self, route, *a, **k):
        res = orig(self, route, *a, **k)
        pts = route.pts if hasattr(route, "pts") else route
        return {**res, "ok": False, "issues": [{"kind": "BODY", "s_m": 1.0, "x": pts[1][0], "y": pts[1][1], "value": 0.5,
                                                "start_leg": False}] + list(res.get("issues", []))}
    DriveGraph.check_route = fail
    try:
        ev = plan_evasion(net, FireArea(cells, source="T"), here, access=acc[0])
    finally:
        DriveGraph.check_route = orig
    out["defect2"] = {"injected": "check_route 항상 실패", "status": ev.status, "reason": ev.reason,
                      "would_execute": ev.status == "OK",
                      "reported_check_ok": ((ev.detail or {}).get("route_check") or {}).get("ok")}
    json.dump(out, sys.stdout, ensure_ascii=False, indent=1)


def _all_issues(g, route):
    """수정 전 코드는 출발 구간 issue 를 결과에서 뺀다 → 다시 모아 센다 (start_leg 포함)."""
    import ugv.drive_graph as dg
    res = g.check_route(route)
    issues = list(res.get("issues", []))
    if res.get("start_leg_issues"):
        issues += [{"kind": "RADIUS_START_LEG"}] * res["start_leg_issues"]
    return issues


if __name__ == "__main__":
    main()
