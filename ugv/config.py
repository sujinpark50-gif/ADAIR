# -*- coding: utf-8 -*-
"""ugv/config.py — 차량별 설정과 운영 기준값.

차량별 설정 (VEHICLES): 식별자, 소속 소방서(복귀 위치), 초기 위치, 드라이버(sim/px4)와 PX4 연결 정보.
  - 소속 소방서 좌표는 환경 모듈의 단일 출처 environment/config/fire_stations.json (INT-04) 에서 base_id 로 읽는다.
    코드에 좌표를 두지 않는다. 사용자 결정 2026-10-09: 첫 차량은 인제119안전센터(A).
  - 차량을 늘리려면 UGV_VEHICLES_JSON 에 같은 형식의 목록을 준다 (기본은 아래 DEFAULT_VEHICLES).
운영 기준값은 프롬프트에서 확정된 값이다. 시험용 값과 구분해 둔다.
"""

import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UGV_DIR = Path(__file__).resolve().parent
STATIONS_JSON = Path(os.getenv("UGV_STATIONS_JSON", ROOT / "environment" / "config" / "fire_stations.json"))
# 도로망: 트윈 환경과 같은 도로 자료를 읽는다 (ugv/road_source.py 가 직접 읽는다. 변환본 파일 없음).
# 트윈 환경 설정 scenario_whatif_2019_legacy.yaml 의 도로 격자(data/processed/roads_raster.tif)를 만든 벡터 원본.
# 링크 901개를 모두 도로망에 넣는다 (환경과 같은 도로 수). 자동차전용 도로는 넣되 UGV 는 다니지 않는다
ROAD_GPKG = Path(os.getenv("UGV_ROAD_GPKG", ROOT / "environment" / "data" / "processed" / "roads_clipped.gpkg"))
UGV_ON_MOTORWAY = os.getenv("UGV_ON_MOTORWAY", "0") == "1"   # 자동차전용 도로(AUTO_EXCLU=1) 통행 (기본 안 함)
# 제한속도가 원자료에 없을 때 도로 등급(ROAD_RANK)별 값 (km/h). 실제 경로 속도는 MIN_ROAD_SPEED_KMH~MAX_SPEED 로 묶인다
ROAD_RANK_SPEED_KMH = {101: 100, 102: 80, 103: 80, 104: 60, 105: 60, 106: 60, 107: 50, 108: 30}
STATE_DIR = Path(os.getenv("UGV_STATE_DIR", UGV_DIR / ".state"))

# ---------------------------------------------------------------------------- 차량
DEFAULT_VEHICLES = [
    {
        "resource_id": "A-ugv1", "resource_type": "UGV",
        "station_base_id": "A",                 # 소속 소방서 = 복귀 위치 (fire_stations.json)
        "initial": "STATION",                   # 초기 위치: "STATION" 또는 {"lat":..,"lon":..}
        "driver": os.getenv("UGV_DRIVER", "sim"),
        "px4": {"address": "udpin://0.0.0.0:14541", "instance": 1,
                # 기본 월드 = 실제 도로 선형 + 평탄 주행면 (A안, 2026-10-09). 지형을 반영한 도로 월드(B안)는 도로망·DEM 기준 확정 뒤
                "gz_world": os.getenv("UGV_GZ_WORLD", "kangwon_flat"), "gz_model": "adair_ugv_1"},
    },
]


def load_vehicles() -> list:
    p = os.getenv("UGV_VEHICLES_JSON")
    if p:
        return json.loads(Path(p).read_text(encoding="utf-8"))
    return [dict(v) for v in DEFAULT_VEHICLES]


def load_station(base_id: str) -> dict:
    doc = json.loads(STATIONS_JSON.read_text(encoding="utf-8"))
    for s in doc.get("stations", []):
        if s.get("base_id") == base_id:
            return {**s, "coordinate_status": doc.get("coordinate_status"), "source_file": str(STATIONS_JSON)}
    raise KeyError(f"소방서 {base_id} 가 {STATIONS_JSON} 에 없음")


# ---------------------------------------------------------------------------- 주행 (확정)
MAX_SPEED_MPS = 60.0 / 3.6                  # 최고속도 60 km/h
# 도로 등급 제한속도가 이보다 낮으면 이 값으로 본다 (총괄 브랜치 사용자 결정 2026-10-07, 이전 UGV 설정 승계).
# 도로망의 시군도 30 km/h 를 그대로 쓰면 도로망 대부분(583/820)이 30 km/h 가 된다. 0 이면 도로 등급 무시
MIN_ROAD_SPEED_KMH = float(os.getenv("UGV_MIN_ROAD_SPEED_KMH", "50"))

# 속도 계획 (구간별 감속). Gazebo 평지 실측(2단계) 기준 잠정값 — 도로 월드 실측으로 다시 맞춘다
LAT_ACCEL_MPS2 = float(os.getenv("UGV_LAT_ACCEL", "1.5"))     # 곡선 속도 = sqrt(횡가속 × 곡률반경)
ACCEL_MPS2 = 2.0                            # 가속 계획 (PX4 RO_ACCEL_LIM 2.5 보다 낮게)
DECEL_MPS2 = 1.5                            # 감속 계획 (PX4 도착 감속 실측 약 2.1 보다 낮게 → 미리 줄인다)
CURVE_WINDOW_M = 15.0                       # 곡률을 볼 앞뒤 거리
ITEM_SPACING_M = 40.0                       # PX4 미션 지점 간격 상한
ITEM_TURN_DEG = 8.0                         # 이보다 꺾이는 곳은 반드시 미션 지점으로
ITEM_CHORD_TOL_M = 0.15                     # 미션 지점을 이은 직선이 계획 선형에서 이보다 벗어나지 않게 지점을 넣는다
STOP_MARGIN_M = 5.0                         # 정지점 = 제동거리 + 여유

# ---------------------------------------------------------------------------- 진행 방향·회전 (2026-10-09 사용자 결정)
# 새 목표를 받아도 지금 진행 방향으로 경로를 시작한다. 같은 도로를 즉시 되짚는 U턴·후진·3점 회전은 이번 범위에서 금지.
# 정차 중에도 방향 제약을 유지한다. 가능한 경로가 없으면 도로를 따라 감속 정지하고 사유를 보고한다 (ugv/drive_graph.py)
WHEELBASE_M = 2.7                           # adair_ugv (ugv/gazebo/models/adair_ugv/model.sdf)
STEER_MAX_DEG = 30.0                        # 최대 조향각 (2단계 실측 30°)
BODY_LENGTH_M, BODY_WIDTH_M = 4.2, 1.8      # 차체 상자. 앞뒤 축은 차체 중심에서 ±1.35 m
REAR_AXLE_FROM_CENTER_M = 1.35
# 도로 폭: 원자료에 미터 단위 폭이 없다 (WIDTH 는 1~4 코드, 의미 미확인) → 차로 수로 추정한 값. 추정치로 표시한다
LANE_WIDTH_EST_M = 3.25
SHOULDER_EST_M = 0.5                        # 한쪽 길어깨
CMD_LATENCY_S = float(os.getenv("UGV_CMD_LATENCY_S", "1.0"))   # 명령 교체 지연 (2026-10-09 PX4 실측 약 1 s)
# 실측 명령 교체 지연이 하한(CMD_LATENCY_S)보다 길면 실측(최근 20회 상위 80 %)을 쓴다 (vehicle.latency_s). 상한
CMD_LATENCY_MAX_S = float(os.getenv("UGV_CMD_LATENCY_MAX_S", "6.0"))
TURN_MARGIN_M = 5.0                         # 회전·정지 전 감속 여유 거리

# ---------------------------------------------------------------------------- 화재 접근 (확정)
FIRE_STANDOFF_M = 100.0                     # 알려진 화재 영역 경계에서 최소 거리
FIRE_STATES_FOR_SAFETY = tuple(os.getenv("UGV_FIRE_STATES", "BURNING").split(","))
APPROACH_SEARCH_M = 2000.0                  # 목표 주변 이 거리 안의 도로에서 접근 지점을 찾는다 (= 관측 최대 거리)
# 접근 지점 ETA 여유 규칙 — 잠정 정책 (우회를 줄이려고 2026-10-09 추가. 확정된 '관측 우선' 기준이 아니다).
# 지형 가시성을 연결할 때(5단계) 다시 평가한다. 비교에 쓰는 ETA 는 도로 제한속도만 본 값이고 (곡선 감속·가감속 미반영),
# 보고하는 최종 도착 예정 eta_sec(속도 계획 기준)와 다르다 → 평가 응답 approach.selection_rule 에 둘을 따로 싣는다
APPROACH_NEAR_M = float(os.getenv("UGV_APPROACH_NEAR_M", "1000"))            # 기준 도착 시간을 정하는 '목표 근처' 범위
APPROACH_ETA_SLACK_RATIO = float(os.getenv("UGV_APPROACH_ETA_SLACK_RATIO", "0.25"))  # 기준보다 이 비율까지 늦어도 더 가까운 지점
APPROACH_ETA_SLACK_S = float(os.getenv("UGV_APPROACH_ETA_SLACK_S", "60"))   #   (최소 여유 초)
TARGET_SNAP_M = float(os.getenv("UGV_TARGET_SNAP_M", "2000"))   # 목표에서 가장 가까운 도로가 이보다 멀면 도달 불가
ROAD_SAMPLE_M = 25.0                        # 접근 후보·안전 검사용 도로 표본 간격
EVADE_SEARCH_M = 3000.0                     # 안전거리 위반 시 이탈 지점을 찾는 범위

# ---------------------------------------------------------------------------- 주행 감시 (잠정)
ARRIVE_M = 8.0                              # 목적지 도착 판정 거리
OFF_ROUTE_M = 30.0                          # 계획 경로에서 이보다 벗어나면
OFF_ROUTE_CONFIRM_S = 5.0                   #   이 시간 이상 계속될 때 경로 이탈 (시뮬레이션 초)
STALL_TIMEOUT_S = 60.0                      # 주행 중 이 시간 동안
STALL_MOVE_M = 5.0                          #   이만큼도 못 움직이면 정지 고장
TELEMETRY_STALE_S = 5.0                     # PX4 위치가 이 시간(벽시계) 넘게 안 오면 텔레메트리 끊김
# 총괄 통신 단절 (사용자 결정 2026-10-10). 벽시계(단조) 기준. 총괄 연락(명령·조회·heartbeat)을 한 번 받은 뒤부터 센다
COMM_LOSS_STOP_S = 600.0                    # 단절이 10분을 넘으면 진행 중 임무를 취소하고 도로를 따라 감속 정지
COMM_LOSS_RETURN_S = 1800.0                 # 정지 확인 뒤 30분 더 단절이면 소속 거점 출입 지점으로 복귀 시도 (경로 없으면 운영자 개입)
SUPERVISE_PERIOD_S = 0.5                    # 주행 감시 주기 (시뮬레이션 초)
SAFETY_CHECK_PERIOD_S = 1.0                 # 화재 안전거리 검사 주기 (시뮬레이션 초)

# ---------------------------------------------------------------------------- 시간
# 시뮬레이션 초 / 벽시계 초. sim 드라이버 주행 배율. PX4 는 Gazebo 실시간(1.0)
TIME_SCALE = float(os.getenv("UGV_TIME_SCALE", "1"))

# ---------------------------------------------------------------------------- 환경
ENV_URL = os.getenv("UGV_ENV_URL", "")      # 환경 계약 서버 (예: http://127.0.0.1:8300). 비우면 화재 정보 없음
ENV_POLL_S = 2.0                            # 환경 스냅샷을 다시 읽는 간격 (벽시계)
