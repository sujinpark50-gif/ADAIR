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
UGV_SERVER_URL = "TBD"
ORCHESTRATOR_SERVER_URL = "TBD"

# UAV는 "1대 = 프로세스 1개 = 포트 1개" 구조 (px4/uav-agent/README.md "UAV 여러 대 운용" 참고)
# 한 URL로 통일할 수 없고, 드론별로 주소를 따로 관리해야 함
# TODO: 김동현님이 실제 서버에 배포하면 아래 TBD를 실제 host:port로 교체
UAV_ENDPOINTS = {
    "A-uav1": "TBD",   # 예: "http://<host>:8000"
    "A-uav2": "TBD",   # 예: "http://<host>:8001"
    "B-uav1": "TBD",
    "B-uav2": "TBD",
}

# UAV 판단에 필요한 풍속의 출처 — PX4/Gazebo가 풍속을 제공하지 않아(SHARE.md 4-3 확인됨),
# 환경모델(EnvironmentState.wind_speed) 값을 요청에 실어서 보내야 함
WIND_SOURCE = "environment_state"

WEB_DEPLOYMENT_TARGET = "TBD"   # 최종 웹 배포 방식

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