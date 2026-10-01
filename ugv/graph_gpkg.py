# ugv/graph_gpkg.py — 실제 도로망 (roads_clipped.gpkg 기반)
# graph_data_demo.py 와 같은 형식(NODES, ROADS)을 내보내므로 RoadGraph(NODES, ROADS) 로 그대로 쓴다.
#
# 데이터는 ugv/tools/build_road_network.py 가 gpkg 에서 미리 변환한 JSON 이다.
# 여기서는 JSON 만 읽으므로 추가 패키지가 필요 없다.
#
# 거점 노드 id 는 "A"(인제119), "B"(기린119) 로 바뀌어 있다 → ugv/config.py 의 home_node 그대로 사용.
# 거점 좌표는 주소로 추정한 값이고, 가장 가까운 도로 노드에 붙였다 (BASES[*]["snap_m"] 참고).
# ROADS 의 geometry 는 도로 선형 [[lat, lon], ...] — 차량이 도로를 따라 달리는 데 쓴다.
# SPAWN 은 자원별 Gazebo 스폰 자세 (ugv/tools/px4-start.sh 가 같은 JSON 을 읽는다).

import json
from pathlib import Path

_DATA = Path(__file__).resolve().parent / "data" / "road_network.json"

with _DATA.open(encoding="utf-8") as f:
    _raw = json.load(f)

NODES: list[dict] = _raw["nodes"]
# Road 모델에는 cells 필드가 없으므로 분리한다 (geometry 는 Road 가 갖는다)
ROADS: list[dict] = [{k: v for k, v in r.items() if k != "cells"} for r in _raw["roads"]]
ROAD_CELLS: dict[str, list[list[int]]] = {r["road_id"]: r["cells"] for r in _raw["roads"]}  # 도로 → 지나는 격자 칸 [x, y]
BASES: dict = _raw["bases"]
SPAWN: dict = _raw.get("spawn", {})
WORLD: dict = _raw.get("world", {})
