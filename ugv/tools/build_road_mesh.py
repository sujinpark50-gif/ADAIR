# -*- coding: utf-8 -*-
"""ugv/tools/build_road_mesh.py — 평탄 월드(kangwon_flat)에 그릴 도로 표시 메시를 원자료 도로망에서 만든다.

시각 표시만 한다 (충돌 없음). 도로마다 중심선 양옆으로 추정 반폭(drive_graph.est_half_width: 차로 수 기준)만큼
띠를 만들고 높이 0 에 둔다 (월드가 0.03 m 위에 놓는다). 좌표는 지역 평면(ugv/geo.py FRAME)과 같다.

    python -m ugv.tools.build_road_mesh        # → ugv/gazebo/models/adair_roads/roads.obj
"""

import math
import sys
from pathlib import Path

from ugv.roads import RoadNetwork

OUT = Path(__file__).resolve().parents[1] / "gazebo" / "models" / "adair_roads" / "roads.obj"


def main():
    net = RoadNetwork()
    hw = net.graph.half_w
    verts, faces = [], []
    for rid in sorted(net.roads):
        pts = net.roads[rid].xy
        h = hw[rid]
        base = len(verts)
        for i, p in enumerate(pts):
            a, b = pts[max(i - 1, 0)], pts[min(i + 1, len(pts) - 1)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            n = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / n, dx / n
            verts.append((p[0] + nx * h, p[1] + ny * h))
            verts.append((p[0] - nx * h, p[1] - ny * h))
        for i in range(len(pts) - 1):
            v = base + 2 * i + 1                      # OBJ 정점 번호는 1부터
            faces.append((v, v + 1, v + 3, v + 2))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"# ADAIR 도로 표시 메시 — ugv/tools/build_road_mesh.py 로 생성 ({net.source})\n")
        f.write(f"# 도로 {len(net.roads)}, 정점 {len(verts)}, 면 {len(faces)}. 좌표 = UGV 지역 평면 (m), z = 0\n")
        f.write("o roads\n")
        for x, y in verts:
            f.write(f"v {x:.2f} {y:.2f} 0\n")
        f.write("vn 0 0 1\n")
        for q in faces:
            f.write("f " + " ".join(f"{i}//1" for i in q) + "\n")
    print(f"썼음: {OUT} (정점 {len(verts)}, 면 {len(faces)}, {OUT.stat().st_size / 1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
