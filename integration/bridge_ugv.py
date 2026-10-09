# -*- coding: utf-8 -*-
"""
integration/bridge_ugv.py
=========================
지상자원 커넥터의 "integrated" 구현 — UGV 서버 없이 같은 프로세스에서 도로 경로만으로 판단한다 (주행 없음).

도로망·경로 규칙은 UGV 서버와 같다: 원자료 도로망(gpkg)을 직접 읽고, 진행 방향을 지키는 경로
(즉시 U턴 없음, 일방통행, 차체가 들어오는 회전만)로 도달 가능 여부와 도로 시간 ETA 를 낸다 (ugv/route_preview.py).
화재 안전거리·접근 지점 선택·주행 감시는 UGV 서버(real 모드)에서만 한다.
차량 목록은 ugv/config.py 의 차량 설정, 위치는 소속 거점 출입 지점이다.
"""

from __future__ import annotations

import config
from interfaces.schema import LocalResponse, Observation, ResourceStatus

from ugv import config as ugv_config
from ugv import route_preview as rp

_blocked: set = set()          # demo() 가 잠시 막는 도로


def _vehicles(base: str | None = None) -> list:
    return [v for v in ugv_config.load_vehicles() if base is None or v["station_base_id"] == base]


def _home(resource_id: str) -> dict | None:
    v = next((v for v in _vehicles() if v["resource_id"] == resource_id), None)
    return rp.station(v["station_base_id"]) if v else None


def get_ugv_status(base: str) -> list:
    out = []
    for v in _vehicles(base):
        home = rp.station(base)
        if home is None:
            continue
        out.append(ResourceStatus(
            resource_id=v["resource_id"], resource_type=v.get("resource_type", "UGV"), base=base,
            location_lat=home["lat"], location_lon=home["lon"], state="READY",
            # 목적지별 값은 판단(send_ugv_command) 때 정해진다
            capability={"eta_sec": None, "road_blocked": None, "reachable": None, "node_id": home["node_id"]}))
    return out


def send_ugv_command(task_id: str, assignment, force_response=None, force_reason=None) -> LocalResponse:
    rid = assignment.resource_id
    if force_response is not None:
        return LocalResponse(resource_id=rid, response=force_response, reason=force_reason)
    home = _home(rid)
    if home is None:
        return LocalResponse(resource_id=rid, response="REJECT", reason="UNKNOWN_RESOURCE")
    limit = getattr(config, "UGV_TARGET_SNAP_M", ugv_config.TARGET_SNAP_M)
    plan = rp.plan(home["lat"], home["lon"], assignment.target_lat, assignment.target_lon,
                   heading_deg=home["heading_deg"], road_ok=lambda r: r not in _blocked, snap_limit_m=limit)
    evidence = {"start_node": plan["start_node"], "target_node": plan["target_node"], "snap_m": plan["snap_m"],
                "basis": plan["basis"]}
    if plan["reachable"]:
        return LocalResponse(resource_id=rid, response="ACCEPT",
                             evidence={**evidence, "eta_sec": plan["eta_s"], "length_m": plan["length_m"]})
    # 목표 근처에 도로가 없거나 진행 방향을 지키는 경로가 없다 → 임의 지점으로 바꾸지 않고 거절
    return LocalResponse(resource_id=rid, response="REJECT", reason="TARGET_UNREACHABLE",
                         evidence={**evidence, "detail": plan["reason"], "snap_limit_m": limit})


def get_ugv_observation(resource_id: str, sim_time_s: float, task_id: str = None, decision_id: str = None) -> Observation:
    home = _home(resource_id) or {}
    return Observation(resource_id=resource_id, location_lat=home.get("lat"), location_lon=home.get("lon"),
                       simulation_time_s=sim_time_s, observation_type="ROAD_STATUS",
                       value={"node_id": home.get("node_id"), "blocked_roads": len(_blocked),
                              "note": "integrated 모드는 주행하지 않는다 (거점 출입 지점의 도로 상태)"},
                       task_id=task_id, decision_id=decision_id)


def demo() -> list[str]:
    """거점 A → B 경로를 구하고, 그 경로의 도로를 하나씩 막아 ① 우회되는 도로 ② 막히면 갈 수 없는 도로를 보여 준다."""
    a, b = rp.station("A"), rp.station("B")
    lines = []
    if not a or not b:
        return ["  거점 출입 지점을 찾지 못함"]

    def run():
        return rp.plan(a["lat"], a["lon"], b["lat"], b["lon"], heading_deg=a["heading_deg"],
                       road_ok=lambda r: r not in _blocked)

    def fmt(r):
        return f"ETA={r['eta_s'] / 60:.1f}분  {r['length_m'] / 1000:.1f} km  도로 {len(r['road_ids'])}개"

    base = run()
    lines.append(f"  A→B 정상:  도달={base['reachable']}  {fmt(base) if base['reachable'] else base['reason']}")
    if not base["reachable"]:
        return lines
    detour = cut = False
    for road in base["road_ids"][1:-1]:
        _blocked.add(road)
        r = run()
        _blocked.discard(road)
        if r["reachable"] and r["eta_s"] - base["eta_s"] >= 60 and not detour:
            lines.append(f"  도로 {road} 차단: 우회 성공  {fmt(r)}  (+{(r['eta_s'] - base['eta_s']) / 60:.1f}분)")
            detour = True
        elif not r["reachable"] and not cut:
            lines.append(f"  도로 {road} 차단: 도달 불가  ({r['reason']}, 우회로 없음)")
            cut = True
        if detour and cut:
            break
    return lines


def reset():
    _blocked.clear()
