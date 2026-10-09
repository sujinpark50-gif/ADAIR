# -*- coding: utf-8 -*-
"""ugv/tools/spawn_pose.py — 거점 출입 지점의 Gazebo 스폰 자세 "x,y,z,yaw" (px4-start.sh 의 POSE).

출입 지점은 drive_graph.access_point (소방서 좌표에서 가장 가까운, 서로 오갈 수 있는 도로 지점과 그 도로 방향).
평탄 월드(kangwon_flat) 기준 z = 0.4.

    POSE=$(python -m ugv.tools.spawn_pose A) WORLD=kangwon_flat ugv/tools/px4-start.sh
"""

import math
import sys

from ugv.drive_graph import access_point
from ugv.roads import RoadNetwork


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "A"
    acc = access_point(RoadNetwork(), base)
    if acc is None:
        print(f"거점 {base} 출입 지점을 찾지 못함", file=sys.stderr)
        return 1
    pos, heading, _ = acc
    yaw = math.radians(90.0 - heading)                 # 방위(북=0, 시계) → ENU yaw(동=0, 반시계)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    print(f"{pos.x:.2f},{pos.y:.2f},0.4,{yaw:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
