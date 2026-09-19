# ugv/graph_data.py — 순수 데이터, 로직 없음
# 가상 도로망. 실제 인제 도로가 아니라 인제 좌표계 위의 단순화 모델.
# 나중에 국가표준노드링크로 교체 예정 (이 파일만 갈아끼우면 됨)

NODES = [
    {"node_id": "A",  "lat": 38.1155, "lon": 128.1566, "name": "원통119"},
    {"node_id": "B",  "lat": 37.9430, "lon": 128.2760, "name": "기린119"},
    {"node_id": "J1", "lat": 38.0700, "lon": 128.1800, "name": "북부분기"},
    {"node_id": "J2", "lat": 38.0500, "lon": 128.2600, "name": "동부우회"},
    {"node_id": "J3", "lat": 37.9800, "lon": 128.2400, "name": "남부합류"},
    {"node_id": "F1", "lat": 38.0300, "lon": 128.1900, "name": "화재지점"},
]

# 무향 간선. node_a/node_b 순서는 의미 없음.
ROADS = [
    {"road_id": "E1", "node_a": "A",  "node_b": "J1", "distance_m":  8200, "base_time_s": 480},
    {"road_id": "E2", "node_a": "J1", "node_b": "J2", "distance_m": 12300, "base_time_s": 720},
    {"road_id": "E3", "node_a": "J1", "node_b": "F1", "distance_m":  5400, "base_time_s": 360},
    {"road_id": "E4", "node_a": "J2", "node_b": "F1", "distance_m": 15375, "base_time_s": 900},
    {"road_id": "E5", "node_a": "F1", "node_b": "J3", "distance_m":  9225, "base_time_s": 540},
    {"road_id": "E6", "node_a": "J3", "node_b": "B",  "distance_m": 11275, "base_time_s": 660},
]