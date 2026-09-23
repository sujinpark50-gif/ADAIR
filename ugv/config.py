# ugv/config.py — 전부 잠정값, 팀 합의 필요

# 자원 배치
RESOURCES = [
    {"resource_id": "A-ugv1",   "resource_type": "UGV",
     "base": "A", "home_node": "A", "px4_port": 14540},
    {"resource_id": "A-fire1", "resource_type": "FIRE_ENGINE",
     "base": "A", "home_node": "A", "px4_port": None},
    {"resource_id": "B-ugv1",   "resource_type": "UGV",
     "base": "B", "home_node": "B", "px4_port": 14541},
]

# PX4 연결
PX4_HOST = "0.0.0.0"
PX4_BASE_PORT = 14540

# 주행 파라미터 — 실측 보정 필요
CRUISE_SPEED_MPS = 3.0
MISSION_ALT_M = 0.0      # 지상차량. 홈 고도 0 기준 상대 0m
ARM_SETTLE_S = 2.0

# 판단 임계값 — 잠정값
FUEL_RETURN_MARGIN_PCT = 20.0
ROAD_SNAP_M = 50.0          # 이 거리 안이면 해당 도로 위로 간주
NODE_ARRIVE_M = 20.0        # 이 거리 안이면 노드 도착으로 간주
STALE_AFTER_S = 10.0        # 이 시간 넘으면 STALE

# 시연용 가속. 그래프 거리가 실제 수 km 라 실속도로는 시연이 불가능하다.
# PX4 연동 시에는 CRUISE_SPEED_MPS 를 쓴다.
DEMO_SPEED_MPS = 2000.0