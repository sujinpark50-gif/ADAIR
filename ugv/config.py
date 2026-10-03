# ugv/config.py — 전부 잠정값, 팀 합의 필요

import os

# 자원 배치 — Gazebo 월드 1개(kangwon)에 PX4 인스턴스 3개 (차량당 PX4 1개, 제어 프로그램은 이 서버 1개)
# px4_instance i → PX4 가 MAVLink 를 14540 + i 로 보낸다. 0 은 UAV 몫이라 UGV 는 1 부터.
# px4_model: PX4 Gazebo 모델 이름. 에어프레임 번호는 ugv/tools/px4-start.sh 가 모델 이름으로 찾는다.
# max_speed_mps: 차량 최고속도(시뮬레이션 초당 m). ETA = 도로별 min(도로 제한속도, 이 값) 으로 계산한다.
#   r1_rover 2.1 (RO_MAX_THR_SPEED), rover_ackermann 3.1, lawnmower 2.7 — PX4 v1.16 기본 파라미터 기준.
#   실제 소방차(수십 km/h)가 아니라 Gazebo 차량 속도에 맞춘 값이다. 실제 시간감은 UGV_TIME_SCALE 로 맞춘다.
# 스폰 위치는 ugv/data/road_network.json 의 spawn[resource_id] (build_road_network.py 가 계산).
RESOURCES = [
    {"resource_id": "A-ugv1",  "resource_type": "UGV",
     "base": "A", "home_node": "A", "px4_instance": 1, "px4_model": "r1_rover", "max_speed_mps": 2.0},
    {"resource_id": "A-fire1", "resource_type": "FIRE_ENGINE",
     "base": "A", "home_node": "A", "px4_instance": 3, "px4_model": "rover_ackermann", "max_speed_mps": 2.5},
    {"resource_id": "B-ugv1",  "resource_type": "UGV",
     "base": "B", "home_node": "B", "px4_instance": 2, "px4_model": "r1_rover", "max_speed_mps": 2.0},
]
for _r in RESOURCES:
    _r["px4_port"] = 14540 + _r["px4_instance"]

# 장비 (ugv/equipment.py) — 잠정값
#   소방차: 물탱크·방수량. 도착하면 자동으로 진압(SUPPRESSING) — 물이 바닥나거나 /suppress stop 까지.
#     3,000 L 탱크 + 분당 1,800 L(30 L/s) 방수는 중형 펌프차 수준 → 100 초. 거점 노드에 도착하면 다시 채운다.
#   UGV: 적재 한도. execute 의 cargo(이름, kg) + via_node(싣는 곳) → 싣기(LOADING)·내리기(UNLOADING) 자동.
#   경광등(siren)은 소방차가 출동·진압 중일 때 켠다 (Gazebo 표시: ugv/gz_fx.py).
EQUIPMENT = {
    "FIRE_ENGINE": {"water_capacity_l": 3000.0, "pump_lps": 30.0, "payload_kg": 0.0, "siren": True},
    "UGV":         {"water_capacity_l": 0.0,    "pump_lps": 0.0,  "payload_kg": 100.0, "siren": False},
}
LOAD_S = float(os.getenv("UGV_LOAD_S", "60"))       # 짐 싣기 (시뮬레이션 초)
UNLOAD_S = float(os.getenv("UGV_UNLOAD_S", "60"))   # 짐 내리기
AUTO_SUPPRESS = os.getenv("UGV_AUTO_SUPPRESS", "1") != "0"   # 소방차 도착 즉시 진압 시작

# PX4 연결
# PX4 SITL 은 MAVLink 를 자기 호스트의 127.0.0.1 로만 보낸다. Gazebo/PX4 가 원격(WSL)이면
# 그쪽에서 ugv/tools/mavlink_relay.py 가 이 서버 호스트의 같은 포트로 넘겨준다 → 여기서는 0.0.0.0 으로 받는다.
PX4_HOST = "0.0.0.0"
PX4_BASE_PORT = 14540   # 인스턴스 0 = UAV. UGV 는 위 RESOURCES 의 px4_port 를 쓴다.
# UGV_DRIVER=px4 일 때 실제로 PX4 에 연결할 자원. 나머지는 sim.
# 띄우지 않은 PX4 를 기다리면 서버가 시작되지 않으므로, 띄운 인스턴스만 적는다.
PX4_RESOURCES = [r for r in os.getenv("UGV_PX4_RESOURCES", "A-ugv1").split(",") if r]

# 시간 — 전부 시뮬레이션 초 기준 (ugv/sim_clock.py)
#   UGV_TIME_SCALE: 시뮬레이션 초 / 벽시계 초. PX4 쪽 PX4_SIM_SPEED_FACTOR 와 반드시 같은 값.
#     sim 드라이버도 이 배율로 움직이므로, PX4 없이 빨리 돌려 보려면 이 값만 키운다 (예: 200).
#   UGV_SECONDS_PER_ENV_STEP: 환경 CA 1 스텝(sim_step)이 몇 시뮬레이션 초인가.
#     환경·총괄 어디에도 정의가 없다(INT-05 미정). 잠정값 — 합의되면 이 값만 바꾼다.
TIME_SCALE = float(os.getenv("UGV_TIME_SCALE", "1"))
SECONDS_PER_ENV_STEP = float(os.getenv("UGV_SECONDS_PER_ENV_STEP", "60"))
# 총괄 보고 (ugv/reporter.py) — 평가·출발·진행·재탐색·도착·실패·중지·자원 상태 변화를 POST {URL}/events 로.
# 기본 꺼짐: 총괄이 1초마다 조회(polling)하므로 필요 없다. 켜려면 UGV_REPORT_URL=http://127.0.0.1:8200
# 꺼져 있어도 GET /reports 에 기록은 남는다.
REPORT_URL = os.getenv("UGV_REPORT_URL", "")

# 도로 환경 시나리오(차단 도로·경유 불가 노드·혼잡의 시간대, CSV 또는 JSON). 비우면 시나리오 없음
SCENARIO_FILE = os.getenv("UGV_SCENARIO", "ugv/scenarios/inje_girin.csv")

# 주행 파라미터 — 실측 보정 필요
CRUISE_SPEED_MPS = 2.0
MISSION_ALT_M = 0.0      # 지상차량. 홈 고도 0 기준 상대 0m
ARM_SETTLE_S = 2.0

# 판단 임계값 — 잠정값
FUEL_RETURN_MARGIN_PCT = 20.0
ROAD_SNAP_M = 50.0          # 이 거리 안이면 해당 도로 위로 간주
NODE_ARRIVE_M = 20.0        # 이 거리 안이면 노드 도착으로 간주
STALE_AFTER_S = 10.0        # 이 시간 넘으면 STALE (주행 중이면 TELEMETRY_LOST 로 task 실패)

# 주행 감시 (server._watch_task) — 잠정값, PX4 rover(2 m/s) 기준
STALL_TIMEOUT_S = float(os.getenv("UGV_STALL_TIMEOUT_S", "60"))   # 이 시뮬레이션 초 동안 (2026-10-03 부터 벽시계 아님 — 4배속이면 벽시계 15초)
STALL_MOVE_M = 10.0          # 이만큼도 못 움직이고 웨이포인트도 안 넘어가면 STALLED
OFF_ROUTE_M = 100.0          # 경로(노드를 이은 선)에서 이보다 벗어나면 OFF_ROUTE
# 추락 판정은 고도 하강 속도로 한다. 강원 지형 월드는 도로를 따라 고도가 수백 m 바뀌므로
# '홈 대비 상대고도 < -5 m' 같은 절대 기준은 정상 주행도 추락으로 잘못 본다.
# 도로 주행 하강은 1 m/s 를 넘기 어렵고(경사 15%, 3 m/s 기준 0.45), 지형을 뚫고 떨어지면 수십 m/s 다.
FALL_RATE_MPS = 8.0          # 시뮬레이션 초당 이보다 빨리 내려가면 추락 (VEHICLE_FAULT)
FAULT_GRACE_S = 5.0          # 출발 직후 이 시간은 차량 이상 판정 안 함 (arm·모드 전환 대기)
FAULT_CONFIRM_S = 3.0        # 차량 이상이 이 시간 이상 계속돼야 확정 (순간 신호 무시)

# 화재 접근 규칙 — 환경변수로 바꿀 수 있다 (코드 수정 없이)
#   기본: 화재에서 가장 가까운 도로 노드 하나만 본다. 거기로 못 가면 거절.
#   UGV_APPROACH_FALLBACK=1: 가장 가까운 노드로 못 가면 APPROACH_MAX_M 안의 다음 노드로 접근
TARGET_SNAP_M = float(os.getenv("UGV_TARGET_SNAP_M", "2000"))      # 가장 가까운 노드가 이보다 멀면 TARGET_UNREACHABLE (루트 config.UGV_TARGET_SNAP_M 과 같은 값)
APPROACH_FALLBACK = os.getenv("UGV_APPROACH_FALLBACK", "0") == "1"
APPROACH_MAX_M = float(os.getenv("UGV_APPROACH_MAX_M", "500"))     # fallback 접근 노드의 화재 거리 상한

# 시연용 가속 — 6노드 시연 도로망(graph_data_demo, 총괄 orchestrator_v012 데모)에서만 쓴다.
# 실제 도로망(server.py)은 차량 max_speed_mps × UGV_TIME_SCALE 로 움직인다.
DEMO_SPEED_MPS = 2000.0