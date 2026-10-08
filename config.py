# -*- coding: utf-8 -*-
"""
config.py
=========
Integration Contract 문서 12번("Config / TBD")에 명시된 항목을
그대로 반영한 설정 파일입니다.

문서가 "TBD 또는 Config 값으로 둔다"고 명시한 항목은 아직 실제 값으로
확정하지 않고 TBD 표시를 유지합니다 (최종 대상지역, Raster 해상도,
실제 PX4 연결 방식, 최종 웹 배포 방식, Safety 세부 임계값, 운용 임계값).
"""

import os

# ---------------------------------------------------------------------------
# 1. 좌표·Grid 설정 (Contract 8, 12번)
# ---------------------------------------------------------------------------

# 후보지 1(인제 원통-기린) 기준 대응거점 근사 좌표
# 주의: 정확한 좌표는 카카오맵/네이버맵에서 재확인 필요
BASE_A_NAME = "원통119안전센터"
BASE_A_LAT = 38.1155
BASE_A_LON = 128.1566

BASE_B_NAME = "기린119안전센터"
BASE_B_LAT = 37.9430
BASE_B_LON = 128.2760

# INT-04 거점 좌표 단일 출처: environment/config/fire_stations.json (멘토 패치 0001)
# 이 파일이 있으면 위 값 대신 파일 값을 쓴다. 팀이 좌표를 확정하면 그 파일만 고친다.
FIRE_STATIONS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "environment", "config", "fire_stations.json")


def _load_stations(path=FIRE_STATIONS_PATH):
    import json
    try:
        with open(path, encoding="utf-8") as f:
            return {s["base_id"]: s for s in json.load(f).get("stations", [])}
    except Exception:
        return {}


_STATIONS = _load_stations()
if "A" in _STATIONS:
    BASE_A_NAME, BASE_A_LAT, BASE_A_LON = _STATIONS["A"]["name"], float(_STATIONS["A"]["lat"]), float(_STATIONS["A"]["lon"])
if "B" in _STATIONS:
    BASE_B_NAME, BASE_B_LAT, BASE_B_LON = _STATIONS["B"]["name"], float(_STATIONS["B"]["lat"]), float(_STATIONS["B"]["lon"])
if "C" in _STATIONS:   # 원통119안전센터 (2026-10-08 추가, 좌표 미확인). 원점 계산에는 넣지 않는다
    BASE_C_NAME, BASE_C_LAT, BASE_C_LON = _STATIONS["C"]["name"], float(_STATIONS["C"]["lat"]), float(_STATIONS["C"]["lon"])
STATIONS_SOURCE = "fire_stations.json" if _STATIONS else "config.py 기본값"

COORDINATE_ORIGIN_LAT = (BASE_A_LAT + BASE_B_LAT) / 2
COORDINATE_ORIGIN_LON = (BASE_A_LON + BASE_B_LON) / 2

# TBD (Contract 12번 "현재 미확정 상태로 둘 항목"):
FINAL_COORDINATE_SYSTEM = "TBD"   # 최종 대상지역 좌표계
RASTER_RESOLUTION_M = "TBD"       # Raster 해상도 / Cell Size — 확정 전까지 GRID_CELL_SIZE_M 임시값 사용
GRID_CELL_SIZE_M = 10             # 임시값 — 최종 확정 전까지만 사용
GRID_WIDTH = 100
GRID_HEIGHT = 100

RISK_SCORE_MIN = 0.0
RISK_SCORE_MAX = 1.0

# ---------------------------------------------------------------------------
# 2. 모듈 연결 방식 (Contract 11, 12번)
# ---------------------------------------------------------------------------

FIRE_CONNECTION_MODE = "mock"
UAV_CONNECTION_MODE = "mock"
UGV_CONNECTION_MODE = "mock"
ORCHESTRATOR_CONNECTION_MODE = "mock"

UAV_SERVER_URL = "TBD"   # 구버전 — 아래 UAV_ENDPOINTS로 대체 예정, 하위호환용으로만 유지
UGV_SERVER_URL = "http://localhost:8100"   # 박유홍님 UGV 서버(ugv/server.py) 기본 포트. UGV_CONNECTION_MODE="real" 일 때만 사용
ORCHESTRATOR_SERVER_URL = "TBD"

# UAV는 "1대 = 프로세스 1개 = 포트 1개" 구조 (uav/uav-agent/README.md "UAV 여러 대 운용" 참고)
# 한 URL로 통일할 수 없고, 드론별로 주소를 따로 관리해야 함
# 기본은 로컬 mock. 클라우드(김동현님 배포) 사용 시 A-uav1 주소를 안내받은 host:port로 교체
UAV_ENDPOINTS = {
    "A-uav1": "http://127.0.0.1:8000",   # 로컬 mock (클라우드 서버는 김동현님께 기동 요청 시 주소 교체)
    "A-uav2": "http://127.0.0.1:8001",   # 로컬 mock
    # 원통119(C) 예비 드론 — 트윈은 run_servers 가 8002·8003 에 띄우고 ORCH_UAV_ENDPOINTS 로 주소를 준다.
    # 기린(B) 드론은 발화점 약 18 km 로 항속 반경 밖이라 뺌 (2026-10-08)
    "C-uav1": "TBD",
    "C-uav2": "TBD",
}

# UAV 판단에 필요한 풍속의 출처 — PX4/Gazebo가 풍속을 제공하지 않아(uav/NOTE.md, docs/uav/R01_R05.md 1-4 확인됨),
# 환경모델(EnvironmentState.wind_speed) 값을 요청에 실어서 보내야 함
WIND_SOURCE = "environment_state"

WEB_DEPLOYMENT_TARGET = "TBD"   # 최종 웹 배포 방식

# 관제판의 "도착 즉시 진화" (총괄 팀별 요청서 INT-02)
# 차량 도착 애니메이션만으로 화재 칸을 UNBURNED 로 바꾸던 시연용 동작이다. 실제 진화 효과는
# 환경 모델 몫(ENV-07, F2)이므로 기본은 끈다. 시연 때만 환경변수 WEB_DEMO_EXTINGUISH=1 로 켠다.
WEB_DEMO_EXTINGUISH = os.environ.get("WEB_DEMO_EXTINGUISH", "0") == "1"

# 총괄 오케스트레이터 서버 (박수진, orchestrator/api.py, 기본 8200) — 관제판 우측 "총괄 판단" 패널이 읽는 곳 (INT-03)
ORCH_URL = os.environ.get("ORCH_URL", "http://127.0.0.1:8200")

# 관제판 "출동" 버튼의 경로 (총괄 팀별 요청서 INT-01)
# 꺼짐(기본): 기존처럼 관제판이 UAV 서버에 직접 evaluate→execute.
# 켜짐(WEB_DISPATCH_VIA_ORCH=1): 총괄 POST /tasks 로만 보낸다. 출동 여부·Safety·실행은 총괄이 결정하고
#   관제판은 UAV 를 직접 부르지 않는다. 총괄 시나리오(기상·환경) 준비 후 켠다.
WEB_DISPATCH_VIA_ORCH = os.environ.get("WEB_DISPATCH_VIA_ORCH", "0") == "1"

# 관제판 산불 시계 (INT-05)
# 비어 있음(기본): 기존처럼 관제판이 산불 모델을 직접 진행시킨다 (단독 시연용).
# 환경 계약 서버 주소(예: http://127.0.0.1:8300): 관제판은 /snapshot 을 읽기만 하고 진행하지 않는다.
#   시계는 트윈 시계(tools/twin_clock.py) 하나만 /advance 를 부른다. 이 모드에서는 관제판 자체 자동 루프를 끈다
#   (자동 판단·배정은 총괄 담당).
WEB_ENV_URL = os.environ.get("WEB_ENV_URL", "").rstrip("/")

# ---------------------------------------------------------------------------
# 3. 시뮬레이션 실행 설정
# ---------------------------------------------------------------------------

SIMULATION_TIMESTEP_SEC = 1.0
DEMO_TOTAL_STEPS = 10
MAX_REEVALUATION_RETRIES = 2   # Safety 세부 임계값 확정 전까지 임시값 (Contract 12번 TBD 항목)

# 화재 목표 좌표 → 도로 노드 스냅 허용 거리(m). 이보다 멀면 임의 노드로 대체하지 않고
# UGV 가 TARGET_UNREACHABLE 로 거절한다. 임시 시뮬레이션 가정 (팀 합의 필요)
UGV_TARGET_SNAP_M = 2000.0

# ---------------------------------------------------------------------------
# 4. 메시지 규격 (Contract 4, 12번)
# ---------------------------------------------------------------------------

SCHEMA_VERSION = "0.1.0"   # schema_version — 인터페이스 변경 시 여기서 올림
SCENARIO_ID = "IT-01"      # 현재 실행 중인 시나리오 식별자 (Integration Test Cases 문서 기준)

# ---------------------------------------------------------------------------
# 5. 로그 설정
# ---------------------------------------------------------------------------

LOG_DIR = "logs"
LOG_FILE_NAME = "simulation_log.jsonl"