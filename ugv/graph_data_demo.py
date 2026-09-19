# ugv/graph_data_demo.py — PX4 시연용 축소 도로망.
# graph_data.py 와 구조·비율 동일, 거리만 1/100.
# 실거리로는 한 구간에 45분 걸려 시연이 불가능하다.

BASE = (38.1155, 128.1566)   # A 노드 = SITL 인스턴스 0 의 홈

NODES = [
    {"node_id": "A",  "lat": 38.11550, "lon": 128.15660, "name": "원통119"},
    {"node_id": "J1", "lat": 38.11505, "lon": 128.15683, "name": "북부분기"},
    {"node_id": "J2", "lat": 38.11485, "lon": 128.15765, "name": "동부우회"},
    {"node_id": "F1", "lat": 38.11465, "lon": 128.15694, "name": "화재지점"},
    {"node_id": "J3", "lat": 38.11415, "lon": 128.15745, "name": "남부합류"},
    {"node_id": "B",  "lat": 38.11375, "lon": 128.15785, "name": "기린119"},
]

ROADS = [
    {"road_id": "E1", "node_a": "A",  "node_b": "J1", "distance_m":  82, "base_time_s": 28},
    {"road_id": "E2", "node_a": "J1", "node_b": "J2", "distance_m": 123, "base_time_s": 42},
    {"road_id": "E3", "node_a": "J1", "node_b": "F1", "distance_m":  54, "base_time_s": 18},
    {"road_id": "E4", "node_a": "J2", "node_b": "F1", "distance_m": 154, "base_time_s": 52},
    {"road_id": "E5", "node_a": "F1", "node_b": "J3", "distance_m":  92, "base_time_s": 31},
    {"road_id": "E6", "node_a": "J3", "node_b": "B",  "distance_m": 113, "base_time_s": 38},
]