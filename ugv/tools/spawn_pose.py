# -*- coding: utf-8 -*-
"""ugv/tools/spawn_pose.py — 거점 출입 지점(대기 위치)의 Gazebo 스폰 자세 "x,y,z,yaw" (px4-start.sh 의 POSE).

출입 지점은 drive_graph.access_point (소방서 좌표에서 가장 가까운, 서로 오갈 수 있는 도로 지점과 그 도로 방향,
차량 특성별 급커브 제외, 같은 거점 k 번째 차량은 대기 위치 k). 소방서 좌표는 environment/config/fire_stations.json.
평탄 월드(kangwon_flat) 기준 z = 바퀴 반지름 + 0.05 정도.

    POSE=$(python -m ugv.tools.spawn_pose A) WORLD=kangwon_flat ugv/tools/px4-start.sh
    python -m ugv.tools.spawn_pose --fleet ugv/fleet_ab.json      # 차량마다 "INSTANCE MODEL POSE 차량ID 출입지점설명"
"""

import json
import math
import sys

from ugv.drive_graph import access_point
from ugv.profiles import get
from ugv.roads import RoadNetwork

Z = {"adair_ugv": 0.4, "adair_firetruck": 0.6}


def pose(net, base, profile, slot=0, slot_lengths=None, taken_roads=None):
    acc = access_point(net, base, graph=net.graph_for(get(profile)), slot=slot, slot_lengths=slot_lengths,
                       taken_roads=taken_roads)
    if acc is None:
        return None
    pos, heading, desc = acc
    yaw = math.radians(90.0 - heading)                 # 방위(북=0, 시계) → ENU yaw(동=0, 반시계)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return f"{pos.x:.2f},{pos.y:.2f},{Z.get(profile, 0.5)},{yaw:.4f}", desc, pos.road_id


def main():
    net = RoadNetwork()
    if len(sys.argv) > 2 and sys.argv[1] == "--fleet":
        cfgs = json.loads(open(sys.argv[2], encoding="utf-8").read())
        rc, taken, lines = 0, {}, {}
        # 서버(Fleet)와 같은 순서·규칙: 같은 거점은 대기 위치 순서대로, 앞 차량들이 쓴 대기 도로는 빼고
        for c in sorted(cfgs, key=lambda c: (c["station_base_id"], int(c.get("station_slot", 0)))):
            prof = c.get("profile", "adair_ugv")
            r = pose(net, c["station_base_id"], prof, int(c.get("station_slot", 0)), c.get("slot_lengths"),
                     set(taken.get(c["station_base_id"], ())))
            if r is None:
                print(f"# {c['resource_id']}: 배치 불가 (출입 지점·대기 위치 없음)", file=sys.stderr)
                rc = 1
                continue
            taken.setdefault(c["station_base_id"], set()).add(r[2])
            lines[c["resource_id"]] = f'{c["px4"]["instance"]} {get(prof).gz_model} {r[0]} {c["resource_id"]}'
        for c in cfgs:
            if c["resource_id"] in lines:
                print(lines[c["resource_id"]])
        return rc
    base = sys.argv[1] if len(sys.argv) > 1 else "A"
    r = pose(net, base, "adair_ugv")
    if r is None:
        print(f"거점 {base} 출입 지점을 찾지 못함", file=sys.stderr)
        return 1
    print(r[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
