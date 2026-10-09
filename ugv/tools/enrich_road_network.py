# -*- coding: utf-8 -*-
"""ugv/tools/enrich_road_network.py — 원자료 도로 속성을 ugv/data/road_network.json 에 싣는다.

원자료: data/inje2019/roads/roads_clipped_2019.gpkg (표준 노드·링크, 환경 격자 범위로 잘림). road_id = LINK_ID.
기존 JSON 은 일방통행을 버려서 일방통행 109개를 양방향으로 계획했다 (2026-10-09 확인) → 방향 속성을 싣는다.

싣는 값 (도로마다 "attrs")
  oneway      BOTH / A_TO_B / B_TO_A. ONEWAY=1 이면 UP_FROM_NO → UP_TO_NODE 방향만. JSON 끝점과 맞춰 확인한다
  lanes, up_lanes, down_lanes   원자료 차로 수 (LANES 는 일방통행 도로에서 그 방향 차로 수)
  barrier_code, width_code       원자료 코드 그대로. 의미·단위 미확인 → 코드에서 쓰지 않는다
                                 (WIDTH 는 1~4 값이라 미터 단위 폭이라고 볼 근거가 없다)
그리고 최상위 "clip_extent_epsg5186" (원자료 범위 = 환경 격자 범위, 막다른 길 중 '지도 경계' 분류에 쓴다)

    python -m ugv.tools.enrich_road_network            # 덮어쓴다 (다시 돌려도 같은 결과)
    python -m ugv.tools.enrich_road_network --check    # 쓰지 않고 차이만
"""

import argparse
import json
import sqlite3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GPKG = ROOT / "data" / "inje2019" / "roads" / "roads_clipped_2019.gpkg"
NET_JSON = ROOT / "ugv" / "data" / "road_network.json"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    db = sqlite3.connect(GPKG)
    cols = "LINK_ID,UP_FROM_NO,UP_TO_NODE,ONEWAY,LANES,UP_LANES,DOWN_LANES,BARRIER,WIDTH"
    src = {str(r[0]): r for r in db.execute(f"select {cols} from roads_clipped_2019")}
    ext = db.execute("select min_x,min_y,max_x,max_y,srs_id from gpkg_contents where table_name='roads_clipped_2019'").fetchone()
    doc = json.loads(NET_JSON.read_text(encoding="utf-8"))
    problems, oneway = [], 0
    for r in doc["roads"]:
        s = src.get(r["road_id"])
        if s is None:
            problems.append(f"{r['road_id']}: 원자료에 없음")
            continue
        _, up_from, up_to, ow, lanes, upl, dnl, barrier, width = s
        d = "BOTH"
        if ow == 1:
            oneway += 1
            if (str(up_from), str(up_to)) == (r["node_a"], r["node_b"]):
                d = "A_TO_B"
            elif (str(up_from), str(up_to)) == (r["node_b"], r["node_a"]):
                d = "B_TO_A"
            else:
                d = "A_TO_B" if str(up_from) == r["node_a"] else "B_TO_A" if str(up_from) == r["node_b"] else None
                problems.append(f"{r['road_id']}: 일방통행 방향 노드({up_from}→{up_to})가 JSON 끝점({r['node_a']},{r['node_b']})과 다름 → {d}")
                if d is None:
                    d = "UNRESOLVED"
        r["attrs"] = {"oneway": d, "lanes": lanes, "up_lanes": upl, "down_lanes": dnl,
                      "barrier_code": barrier, "width_code": width}
    doc["attrs_source"] = {
        "file": str(GPKG.relative_to(ROOT)).replace("\\", "/"), "key": "road_id = LINK_ID",
        "oneway": "ONEWAY=1 → UP_FROM_NO→UP_TO_NODE 방향만 (JSON 끝점과 대조)",
        "lanes": "LANES/UP_LANES/DOWN_LANES 원자료 값",
        "unverified": "barrier_code(BARRIER), width_code(WIDTH): 코드 의미·단위 미확인, 사용하지 않음",
        "tool": "ugv/tools/enrich_road_network.py"}
    doc["clip_extent_epsg5186"] = {"min_x": ext[0], "min_y": ext[1], "max_x": ext[2], "max_y": ext[3], "srs_id": ext[4]}
    print(f"도로 {len(doc['roads'])}, 일방통행 {oneway}, 문제 {len(problems)}")
    for p in problems[:20]:
        print("  ", p)
    if a.check:
        return 1 if problems else 0
    with open(NET_JSON, "w", encoding="utf-8", newline="\n") as f:      # 원래 LF 그대로
        f.write(json.dumps(doc, ensure_ascii=False, indent=1) + "\n")
    print("썼음:", NET_JSON.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
