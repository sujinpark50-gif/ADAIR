# -*- coding: utf-8 -*-
"""
orchestrator/config.py
======================
총괄 오케스트레이터 설정. 코드 안에 수치를 흩어두지 않고 여기서만 관리한다.

값마다 상태를 표시한다 (요청서 §13: 운영 정책으로 확정하지 않은 값 구분).
  PROVISIONAL  사용자 결정 또는 잠정 합의. 팀 합의 후 바뀔 수 있음
  UAV_CODE     UAV 담당 코드(uav/uav-agent/config.py)의 현재 값을 그대로 따름
  TEST_ONLY    시험 fixture 전용. 운영 판단·Safety 기준으로 쓰지 않음
  TBD          아직 결정되지 않음. None 이면 해당 기능은 "준비 안 됨"으로 동작

환경변수로 덮어쓸 수 있는 값은 ORCH_ 접두어를 쓴다.
"""

import os
from pathlib import Path

import config as team_config

ROOT = Path(__file__).resolve().parents[1]


def _load_dotenv(path: Path) -> None:
    """저장소 루트 .env 의 KEY=VALUE 를 환경변수로 읽는다 (.gitignore 대상).
    이미 설정된 환경변수는 덮어쓰지 않고, 빈 값은 설정하지 않는다."""
    if not path.is_file():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key, value = key.strip(), value.strip().strip('"').strip("'")
        if key and value and key not in os.environ:
            os.environ[key] = value


if os.getenv("ORCH_SKIP_DOTENV") != "1":
    _load_dotenv(ROOT / ".env")

# ---------------------------------------------------------------------------
# 서버 (PROVISIONAL — 통합 담당과 합의 후 교체. 8100 은 UGV 서버가 사용 중)
# ---------------------------------------------------------------------------
ORCH_HOST = os.getenv("ORCH_HOST", "127.0.0.1")
ORCH_PORT = int(os.getenv("ORCH_PORT", "8200"))
SCHEMA_VERSION = "orch-0.1.0"
# 진행 중 실행 추적 주기(실제 초). PROVISIONAL — UAV 서버의 감시 주기(MONITOR_INTERVAL_S=1)에 맞춤
POLL_INTERVAL_S = float(os.getenv("ORCH_POLL_INTERVAL_S", "1"))

# ---------------------------------------------------------------------------
# 장부 (SQLite). logs/ 는 .gitignore 대상
# ---------------------------------------------------------------------------
DB_PATH = os.getenv("ORCH_DB_PATH", str(ROOT / "logs" / "orchestrator" / "state.sqlite3"))
# 시험용 가짜 환경(FixtureEnv)의 run_id. 비워 두면 서버를 켤 때마다 새 run_id 를 만든다 —
# 가짜 환경은 켤 때마다 0초부터 다시 시작하므로 이전 실행의 신고·관측·임무를 이어받으면 안 된다 (B05).
FIXTURE_RUN_ID = os.getenv("ORCH_RUN_ID") or None

# 진짜 세계(환경) 연결 방식. fixture(기본, 시험용 가짜) / team_inproc / team_http (2026-10 멘토 패치, ENV-01~06)
ENV_MODE = os.getenv("ORCH_ENV_MODE", "fixture")
ENV_URL = os.getenv("ORCH_ENV_URL", "http://127.0.0.1:8300")
ENV_STATE_DIR = os.getenv("ORCH_ENV_STATE_DIR", str(Path(__file__).resolve().parents[1] / "environment" / ".state"))

# ---------------------------------------------------------------------------
# 외부 자원 주소 — 통합 config 를 그대로 따른다 (단일 출처)
# ---------------------------------------------------------------------------
UAV_ENDPOINTS = dict(team_config.UAV_ENDPOINTS)
UGV_SERVER_URL = team_config.UGV_SERVER_URL
HTTP_TIMEOUT_S = float(os.getenv("ORCH_HTTP_TIMEOUT_S", "10"))   # PROVISIONAL (실제 시계)

# ---------------------------------------------------------------------------
# 자원 상태 신선도 (PROVISIONAL, 사용자 결정 2026-09-29, 실제 시계 기준)
# ---------------------------------------------------------------------------
RESOURCE_STALE_S = 10.0
RESOURCE_OFFLINE_S = 30.0

# ---------------------------------------------------------------------------
# 협상 (PROVISIONAL, 사용자 결정 2026-09-29)
# ---------------------------------------------------------------------------
COUNTER_ROUNDS_PER_RESOURCE = 2
# COUNTER 수정안 중 총괄이 재평가 요청에 실어 보낼 수 있는 필드.
# 현재 UAV EvaluateRequest 가 받을 수 있는 수정 필드는 target_agl_m 뿐이다
# (observe_duration_s 는 요청에 넣을 칸이 없어 재평가 불가 — UAV팀 계약 대기).
COUNTER_SUPPORTED_FIELDS = {"UAV": {"target_agl_m"}, "UGV": set()}

# ---------------------------------------------------------------------------
# 우선순위 (사용자 결정 2026-09-30, 기본값은 같은 날 자동으로 변경)
#   AUTO_HIGHER_RISK  (기본) 애매한 묶음도 검증을 통과한 AI 추천 순서로 바로 보냄.
#                     AI 실패·검증 실패 시 인명 → 위험도가 조금이라도 높은 곳 순 (규칙)
#   HUMAN_CHOICE      수동 버전. 애매한 후보를 나란히 보여주고 자원이 모자라면 사람이 고를 때까지 대기
# 두 모드 모두 화면의 '지금 보내기'로 사람이 직접 보낼 수 있다 (Local·Safety 관문은 그대로).
# ---------------------------------------------------------------------------
PRIORITY_MODE = os.getenv("ORCH_PRIORITY_MODE", "AUTO_HIGHER_RISK")
# 상황이 바뀔 때(새 임무·신고·자원 반납 등) 대기 임무 배정을 자동 실행. 주기 실행은 하지 않는다 (부하)
AUTO_DISPATCH_ON_CHANGE = os.getenv("ORCH_AUTO_DISPATCH", "1") == "1"
PRIORITY_MODES = ("HUMAN_CHOICE", "AUTO_HIGHER_RISK")
PRIORITY_SIMILAR_RISK_DELTA = 0.1     # 위험도 차이가 이 값 이내면 '비슷함' (risk_score 0~1 기준)

# ---------------------------------------------------------------------------
# UAV 고도 (UAV_CODE) — 비행고도 = 지면 AMSL + target_agl_m + 10m 는 UAV 가 계산.
# 총괄은 alt_m_amsl 에 지면고도만 넣고 80m·10m 를 더하지 않는다 (요청서 §4.1).
# ---------------------------------------------------------------------------
UAV_DEFAULT_TARGET_AGL_M = 80.0

# ---------------------------------------------------------------------------
# 모의 센서 (TEST_ONLY, 요청서 §8). 실제 탑재 장비·운영 관측범위가 아니다.
# ---------------------------------------------------------------------------
SENSOR_PROFILES = {
    # 한 프레임(정지·수직 촬영) 기준. 사용자 결정 2026-09-30: 기본 비행 높이가 지면 위 90m
    # (임무 80m + UAV 여유 10m)로 바뀌어 기준 높이를 50m(30×25m) → 90m 로 교체했다.
    # 계산: H20T 급 열화상 대각 시야각 40.6°, 영상 5:4 → 대각 2×90×tan(20.3°) ≈ 66.6m → 약 52.0 × 41.6m.
    # 52×42 는 보기 쉽게 반올림한 시험값이다 (실제 탑재 카메라 미확인). 실제 높이가 다르면 비례해 다시 계산한다.
    # "50m 안의 불을 탐지한다"는 뜻이 아니라 한 번 수직으로 찍을 때 영상에 들어오는 지상 넓이다.
    "TEST_THERMAL_90M_V1": {
        "source": "SIMULATED",
        "status": "TEST_ONLY",
        "sensor_type": "THERMAL",
        "agl_m": 90.0,
        "view_direction": "NADIR",
        "footprint_shape": "RECTANGLE",
        "footprint_width_m": 52.0,
        "footprint_height_m": 42.0,
        "reference_payload": "H20T_CLASS_EXAMPLE",
        "basis": "IDEALIZED_DIAGONAL_FOV_GEOMETRY",
    },
    # 지상(UGV) 열화상 — 순찰 차량 마스트의 열화상 카메라가 차 주변을 둘러본다고 가정 (2026-10-05).
    # 차 위치 중심 정사각형(기본 450 m = 90 m 칸 5×5). 수평 시야라 지형 가림은 반영하지 않는다 (가정임을 결과에 남긴다).
    # 바꾸기: ORCH_GROUND_THERMAL_VIEW_M
    "ASSUMED_GROUND_VIEW_V1": {
        "source": "SIMULATED",
        "status": "TEST_ONLY",
        "sensor_type": "THERMAL",
        "platform": "GROUND",
        "agl_m": None,
        "scale_with_agl": False,
        "view_direction": "HORIZONTAL_360",
        "footprint_shape": "RECTANGLE",
        "footprint_width_m": float(os.getenv("ORCH_GROUND_THERMAL_VIEW_M", "450")),
        "footprint_height_m": float(os.getenv("ORCH_GROUND_THERMAL_VIEW_M", "450")),
        "reference_payload": "VEHICLE_MAST_THERMAL_EXAMPLE",
        "basis": "ASSUMED_GROUND_VIEW_SQUARE_AROUND_VEHICLE (지형 가림 미반영)",
    },
    # 임시 가정 (사용자 결정 2026-09-30): 드론이 목표에 도착해 그 칸 위를 원으로 한 바퀴 돌았다고 보고
    # 관측 범위를 90m × 90m 로 둔다. 실제 드론은 아직 제자리 비행만 한다 (원 비행은 UAV-08 계약 대기) —
    # 이 범위는 실제로 본 범위가 아니라 가정이다. 높이에 따라 늘리거나 줄이지 않는다.
    # 드론팀의 원 비행·경로 보고가 생기면 위 TEST_THERMAL_90M_V1 (한 프레임 기준)로 되돌린다.
    "ASSUMED_ORBIT_90M_V1": {
        "source": "SIMULATED",
        "status": "TEST_ONLY",
        "sensor_type": "THERMAL",
        "agl_m": None,
        "scale_with_agl": False,
        "view_direction": "NADIR",
        "footprint_shape": "RECTANGLE",
        "footprint_width_m": 90.0,
        "footprint_height_m": 90.0,
        "reference_payload": "H20T_CLASS_EXAMPLE",
        "basis": "ASSUMED_ONE_ORBIT_OVER_TARGET_CELL (실제 비행 경로 증거 없음)",
    },
}
# 원래 관측 범위(한 프레임)는 TEST_THERMAL_90M_V1 로 남겨 둔다. 되돌리기: ORCH_SENSOR_PROFILE=TEST_THERMAL_90M_V1
DEFAULT_SENSOR_PROFILE_ID = os.getenv("ORCH_SENSOR_PROFILE", "ASSUMED_ORBIT_90M_V1")
GROUND_SENSOR_PROFILE_ID = os.getenv("ORCH_GROUND_SENSOR_PROFILE", "ASSUMED_GROUND_VIEW_V1")   # UGV 열화상

# ---------------------------------------------------------------------------
# 현장 환경 측정 (사용자 결정 2026-09-30: 2019 재현 필수 기능)
# 드론·UGV·소방차에 기상 센서가 아직 없어 총괄 쪽 모의 센서로 측정한다 (TEST_ONLY).
# 모의 센서는 도착 위치의 환경 값을 읽을 뿐이며, 환경이 주지 않은 항목은 만들지 않는다.
# ---------------------------------------------------------------------------
# UGV 지상 열화상 (모의, 2026-10-05): ORCH_UGV_THERMAL=1 일 때만 UGV 에 THERMAL 을 붙인다. 기본은 꺼짐 —
# 켜면 UAV·UGV 를 함께 허용한 열화상 임무에서 직선거리가 가까운 UGV 가 먼저 평가된다.
UGV_THERMAL_ENABLED = os.getenv("ORCH_UGV_THERMAL", "1") == "1"   # 사용자 결정 2026-10-07: 관측 계획 시연용으로 켬
SIMULATED_CAPABILITIES = {"UAV": ("WEATHER",), "UGV": ("WEATHER",) + (("THERMAL",) if UGV_THERMAL_ENABLED else ()),
                          "FIRE_ENGINE": ("WEATHER",)}
WEATHER_SENSOR_PROFILE = {
    "sensor_profile_id": "TEST_WEATHER_POINT_V1", "source": "SIMULATED", "status": "TEST_ONLY",
    "sensor_type": "WEATHER", "measures": ("wind_ms", "wind_dir_deg", "temperature_c", "humidity_pct"),
    "method": "READ_ENV_VALUE_AT_POSITION",
}
# 불 발견 시 측정 임무 자동 생성: 불난 칸 + 바람이 향하는 쪽 위험 칸 (사용자 결정)
AUTO_ENV_SENSE_ENABLED = True
# Local 에 보낼 observation_type. 기상 센서는 Local 에 없어 이동 가능성만 평가받는다 (값 None = 보내지 않음)
LOCAL_OBSERVATION_TYPE = {"THERMAL": "THERMAL", "RGB": "RGB", "WEATHER": None}

# ---------------------------------------------------------------------------
# 기상 항목별 최신성 (사용자 결정 2026-09-30 D02)
# - 항목마다 값·단위·출처·관측시각·유효성을 따로 본다. 유효한 현장 측정 우선, 만료되면 관측소 값으로 바꾸고
#   재측정을 요청한다. 쓸 값이 없으면 UNKNOWN (임의 값으로 채우지 않음).
# - 관측소 유효시간: 그 자료의 제공 간격 (기상청 ASOS 시간자료 = 3600초, knowledge.KmaAsosReplay).
# - 현장 측정 유효시간: 사용자 결정 2026-10-07 = 35분 (시뮬레이션 2,100초, 모든 항목. 트윈 환경 한 단계와 같음).
#   값을 비우면(ORCH_FIELD_WEATHER_MAX_AGE_S="") POLICY_NOT_SET 으로 표시되고 판단에 쓰이지 않는다 (관측소 값 사용).
#   ORCH_FIELD_WEATHER_MAX_AGE_S = "600"  (모든 항목, 시뮬레이션 초)  또는
#                                  '{"wind_ms": 600, "wind_dir_deg": 600}'  (항목별 JSON)
# ---------------------------------------------------------------------------
WEATHER_ITEMS = {
    "wind_ms": {"unit": "m/s", "min": 0.0, "max": None},
    "wind_dir_deg": {"unit": "deg (불어오는 방향, 북 기준 시계방향)", "min": 0.0, "max": 360.0},
    "temperature_c": {"unit": "degC", "min": None, "max": None},
    "humidity_pct": {"unit": "%", "min": 0.0, "max": 100.0},
}


def parse_field_weather_max_age(raw) -> dict:
    """환경변수 값 → {항목: 유효시간(초) 또는 None}. 비어 있으면 모든 항목 None(정책 미설정)."""
    out = {k: None for k in WEATHER_ITEMS}
    raw = (raw or "").strip()
    if not raw:
        return out
    if raw.startswith("{"):
        import json
        given = json.loads(raw)
        unknown = set(given) - set(out)
        if unknown:
            raise ValueError(f"ORCH_FIELD_WEATHER_MAX_AGE_S: 알 수 없는 항목 {sorted(unknown)}")
        out.update({k: (None if v is None else float(v)) for k, v in given.items()})
    else:
        out = {k: float(raw) for k in out}
    if any(v is not None and v < 0 for v in out.values()):
        raise ValueError("ORCH_FIELD_WEATHER_MAX_AGE_S: 음수는 쓸 수 없다")
    return out


# 사용자 결정 2026-10-07: 현장 측정은 관측 시각부터 35분(시뮬레이션 2,100초) 유효 — 모든 항목
FIELD_WEATHER_MAX_AGE_S = parse_field_weather_max_age(os.getenv("ORCH_FIELD_WEATHER_MAX_AGE_S", "2100"))
# 현장 측정이 만료되면 같은 칸의 재측정을 요청한다 (모의 측정 계약 사용, 같은 만료 측정에 대해 한 번)
WEATHER_REMEASURE_ENABLED = True

# ---------------------------------------------------------------------------
# 환경 반영(APPLY) ACK 가 계약으로 있는 관측 종류 (B07). 열화상·기상만 있다 (ENV-05).
# 그 밖의 관측(도로 상황 등)에 needs_env_ack=True 를 요구하면 접수에서 거절한다.
# ---------------------------------------------------------------------------
ENV_ACK_SUPPORTED_SENSORS = ("THERMAL", "WEATHER")

# ---------------------------------------------------------------------------
# 신고 접수 시 최초 정찰 자동 생성 (사용자 결정 2026-09-30 D01)
# ---------------------------------------------------------------------------
AUTO_RECON_ON_REPORT = True
# 신고 접수 시 자동 초기 정찰. ORCH_AUTO_RECON_GROUND=1 이면 UGV 도 후보 (기본은 드론만, 2026-10-05).
# UGV 가 실제로 뽑히려면 ORCH_UGV_THERMAL=1 도 켜야 한다 (아니면 능력 부족으로 후보에서 빠진다)
AUTO_RECON_REQUIREMENTS = {"resource_types": ["UAV", "UGV"] if os.getenv("ORCH_AUTO_RECON_GROUND", "0") == "1" else ["UAV"],
                           "sensor": "THERMAL", "needs_env_ack": True}

# ---------------------------------------------------------------------------
# 확산 예측 활용 (사용자 결정 2026-09-30)
# - 화면 표시 + 우선순위 반영(예측이 주거지·사람 있는 보호대상에 닿으면 인명 위험)은 켜짐
# - 사전 감시 임무 생성: 구현 완료, 부하 문제 해결 전까지 꺼 둠 (후보만 계산·기록·표시)
# ---------------------------------------------------------------------------
PREEMPTIVE_MONITOR_ENABLED = os.getenv("ORCH_PREEMPTIVE_MONITOR", "0") == "1"

# ---------------------------------------------------------------------------
# 기상청 관측 재생 (사용자 결정 2026-09-30). 시나리오 시계(scenario_start_kst)는 환경이 준다.
# ---------------------------------------------------------------------------
KMA_ASOS_CSV = Path(os.getenv("ORCH_KMA_ASOS_CSV", str(ROOT / "data/weather/kma_asos_hourly_20190404_20190405.csv")))
KMA_ASOS_STATIONS = Path(os.getenv("ORCH_KMA_ASOS_STATIONS", str(ROOT / "data/weather/kma_asos_stations.json")))

# ---------------------------------------------------------------------------
# 관측 계획 (사용자 결정 2026-10-07, docs/common/총괄_관측계획_개편_계획서_완성본.md)
#   신고 → 드론이 불 테두리를 따라 날며 불 좌표 보고 → LLM 이 다음 예상 지역(구역)과 보낼 자원을 고름 → 반복.
#   켜져 있으면 신고가 와도 최초 정찰 Task 를 바로 만들지 않고 관측 계획이 자원을 배정한다.
# ---------------------------------------------------------------------------
OBS_PLANNER_ENABLED = os.getenv("ORCH_OBS_PLANNER", "1") == "1"
OBS_REPORT_DELAY_S = 120.0            # 점화 후 신고 시각 (시뮬레이션 초)
# 시연 종료: 2019-04-05 06:00 KST = 시작(04-04 14:45) 후 15시간 15분. 이후 새 관측 계획을 세우지 않는다
OBS_DEMO_END_SIM_S = float(os.getenv("ORCH_OBS_DEMO_END_SIM_S", str(15 * 3600 + 15 * 60)))
OBS_CANDIDATE_RADIUS_M = 1000.0       # 알려진 불에서 이 거리 안의 칸만 후보 (LLM 입력)
OBS_BLOCK_CELLS = 3                   # 후보 칸을 3×3 칸 구역으로 묶어 LLM 에 준다
# LLM 에 주는 후보 구역 상한 (사용자 결정 2026-10-07: 토큰 절약, 30 → 10). 진행 방향 앞쪽 → 테두리 바깥 미관측 → 가까운 순.
# 빠진 구역은 버리지 않고 다음 계획에서 다시 후보가 된다
OBS_MAX_CANDIDATE_BLOCKS = 10        # 2026-10-07: 30 → 10 (실제 배정은 계획당 2~4개, 예상 구역 포함 10개 이하)
# 환경 한 단계(트윈 35분) 안에서는 관측 계획을 한 번만 세운다 (사용자 결정 2026-10-07). 그 뒤 돌아온 자원은 다음 단계에
# 모아서 계획한다. 같은 계획 중 자원 거절로 다시 짜는 것은 허용. 관측 반영·첫 출동·앞질러 보내기는 기다리지 않는다
OBS_ONE_PLAN_PER_ENV_STEP = True
OBS_STALE_S = None                    # 오래된 관측 기준 — 미정. 관측 시각만 넘기고 오래됨 표시는 하지 않는다
OBS_RATIONALE_MAX_CHARS = 400
# 첫 출동 (사용자 결정 2026-10-07): 신고가 오면 LLM 을 기다리지 않고 신고 칸에서 가장 가까운 드론을 바로 보낸다.
# 그 드론의 첫 관측(불 좌표)이 들어온 뒤에 LLM 이 나머지 자원을 계획한다 (첫 관측이 LLM 입력에 들어가도록).
OBS_INITIAL_NEAREST_UAV = True
# 첫 드론은 테두리를 따라가지 않고 신고 지점 둘레를 원으로 돌며 불 전체를 본다 (사용자 결정 2026-10-07, 120초).
# 7 m/s × 120 s = 840 m 를 '중심 → 원 위로 나가기 + 한 바퀴'에 쓴다 → 반지름 = 840 / (1 + 2π) ≈ 116 m
OBS_INITIAL_ORBIT_DURATION_S = 120.0
# 다 타고 꺼짐(추정) (사용자 결정 2026-10-07): 화재 모델에서 한 칸은 3단계 동안 타고 BURNED 가 된다
# (environment/config/environment_config.yaml fire_ca.burn_duration_steps = 3). 그래서 '불타는 중'으로 마지막 확인한
# 시각부터 3 × 환경 단계 길이(tick_s)가 지나면 '다 타고 꺼짐(추정)'으로 표시한다. 트윈(2,100초)이면 1시간 45분.
# 관측으로 확인한 '탄 곳'(BURNED)과 구분한다. 환경 단계 길이를 모르면 추정하지 않는다.
OBS_BURN_OUT_STEPS = 3
# 예비 드론 (사용자 결정 2026-10-07): B 기지(기린119) 드론 3·4번은 대기하다가, 불이 B 쪽으로 다가온다고 판단되면 투입한다.
#   물리 판단: 관측으로 본 불의 이동 방향(fire_progress)이 '불 → B 기지' 방향과 ±45° 이내이고 한 칸(90 m) 이상 움직였을 때
#   LLM 판단: 계획 답변의 request_reserve_uavs=true (이유 필수)
#   한 번 투입하면 그 run 동안 계속 쓴다. B 기지 좌표는 팀 공통 config (fire_stations.json)
OBS_RESERVE_UAVS = ("B-uav1", "B-uav2")
OBS_RESERVE_BASE = "B"
OBS_RESERVE_TRIGGER_DEG = 45.0
OBS_RESERVE_MIN_MOVE_M = 90.0
# 앞질러 보내기 (사용자 결정 2026-10-07): 드론이 도착했는데 테두리·불타는 곳·안 본 꺼진 자리가 모두 없으면
# (전에는 신고 지점 쪽으로 날았다 — 없앰) 처음 발화 지점(첫 신고 칸) → 가장 마지막 불 위치로 진행 방향·속도를 구해
# 1시간·2시간 뒤 예상 지점에 드론을 새로 출동시킨다 (각 지점에 가장 가까운 가용 드론). 같은 '마지막 불 위치'로는 한 번만.
OBS_ADVANCE_HORIZONS_S = (3600.0, 7200.0)
# UGV 가장자리 따라가기 (사용자 결정 2026-10-07): UGV 관측에 다 탄 곳만 있고 불타는 곳이 없으면, 관측으로 본 불의
# 진행 방향 쪽으로 450 m 떨어진 지점에 같은 UGV 를 바로 다시 보낸다. 불을 찾거나 다 탄 곳도 안 보이면 멈춘다
OBS_UGV_FOLLOW_STEP_M = 450.0
# 드론 테두리 추적 관측 (모의, TEST_ONLY). 드론팀 순찰 기능(UAV-08) 전까지 총괄 모의 센서로 만든다.
#   도착 지점에서 불 테두리를 찾아 신고 지점에서 멀어지는 쪽으로 따라 난다. 테두리가 없으면 신고 지점 쪽으로 직선.
#   카메라 한 장 52×42 m(지면 위 90 m) 중 진행 방향에 수직인 52 m 를 띠 폭으로 쓴다.
EDGE_SWEEP_PROFILE = {
    "sensor_profile_id": "ASSUMED_EDGE_SWEEP_V1", "source": "SIMULATED", "status": "TEST_ONLY",
    # duration 60 s (사용자 결정 2026-10-07: 20 → 60, 한 번 가면 60초 둘러봄) → 7 m/s × 60 s = 420 m
    "sensor_type": "THERMAL", "speed_ms": 7.0, "duration_s": 60.0, "swath_m": 52.0,
    "basis": "ASSUMED_EDGE_FOLLOWING_SWEEP (드론 순찰 코드 대기, UAV-08)",
}
# 드론은 불 바로 위를 날지 않는다 (사용자 결정 2026-10-08): 불 테두리(불타는 칸과 안 탄 칸의 경계)에서
# 다 탄 쪽으로 20 m 들어간 선을 따라 난다. 카메라 띠 폭 52 m 의 절반(26 m)이라 테두리와 그 바깥 6 m 가 보인다.
# 불이 보이면 불 머리(바람이 불어 가는 쪽 맨 앞) 쪽으로 테두리를 따라간다.
OBS_EDGE_OFFSET_M = 20.0

# ---------------------------------------------------------------------------
# 작업 지시서 (사용자 결정 2026-10-08) — LLM 호출을 줄이기 위해 회의(계획) 한 번에 자원마다 구역 순서 목록을 준다.
#   - 회의는 환경 단계(35분)마다 한 번 (OBS_ONE_PLAN_PER_ENV_STEP). 지시서도 그 길이. 새 회의는 덜 끝난 지시서를 바꾼다.
#   - 자원은 지시서대로 돌며 발견은 장부에만 적는다 (LLM 을 부르지 않는다).
#   - 지시서를 일찍 끝내면(또는 드론 배터리 30% 미만): 지시서 동안 새로 찾은 불 칸이 있으면 불 머리를 계속 따라가고
#     (배터리는 드론의 복귀 안전선 15% 까지 — 드론이 거절하면 멈춤), 없으면 드론은 기지로 복귀해 충전, UGV 는 대기.
#   - 마지막으로 하늘에 남은 드론은 새 불이 없어도 배터리 30% 이상이면 복귀하지 않고 불 머리를 본다 (한꺼번에 충전 방지).
#   - 인명피해 예상(환경 확산 예측이 주거지·사람 있는 보호대상에 닿음)이 새로 생기면 회의를 바로 연다 (장소마다 한 번).
# ---------------------------------------------------------------------------
OBS_ORDER_MAX_STOPS = 3
OBS_ORDER_BATTERY_MIN_PCT = 30.0
# 새 불이 없어도 배터리가 이만큼 남았으면 충전하러 가지 않고 불 머리를 계속 본다 (사용자 결정 2026-10-08, run2 뒤)
OBS_KEEP_WATCH_BATTERY_PCT = 50.0
# LLM 계획 한 번을 기다리는 최대 시간 (벽시계). 넘으면 규칙 계획으로 (2026-10-08 run2: 연결이 357초 멈춤)
OBS_LLM_WALL_MAX_S = 120.0
# 기지 충전 30분 — 시험용 가정 (TEST_ONLY, 사용자 결정 2026-10-08). 드론팀에 실제 방식(교체·도킹 충전) 확인 필요.
# 드론 mock 은 착륙하면 배터리를 바로 100% 로 바꾸므로 총괄이 착륙 뒤 이 시간 동안 일을 주지 않는다.
OBS_DRONE_CHARGE_S = float(os.getenv("ORCH_DRONE_CHARGE_S", "1800"))
OBS_DRONE_CHARGE_BASIS = "TEST_ONLY_ASSUMED_DOCK_CHARGE_30MIN"
OBS_MIN_AIRBORNE_UAVS = 1
OBS_EMERGENCY_ON_HUMAN_RISK = True
# 돌아가는 중에 새 임무 받기 (드론팀 UAV-06, 사용자 결정 2026-10-08: 이 브랜치에서 켬). 드론 서버도
# UAV_RETASK_WHILE_RETURNING=1 로 띄워야 한다 (run_servers.py). 끄면 다음 지점은 착륙·반납 뒤에 보낸다.
OBS_RETASK_WHILE_RETURNING = os.getenv("ORCH_RETASK_WHILE_RETURNING", "1") == "1"
# 지시서를 배터리에 맞게 자르기 위한 드론 값 — 드론팀 uav/uav-agent/config.py 값을 옮겨 적음 (드론팀 '실측 보정 필요')
UAV_ASSUMED_CRUISE_MS = 10.0
UAV_ASSUMED_DRAIN_PCT_S = 0.033
UAV_ASSUMED_RETURN_RESERVE_PCT = 15.0
UAV_ASSUMED_OBSERVE_S = 60.0

# ---------------------------------------------------------------------------
# LLM (요청서 §9: 모델·timeout·호출/비용 한도는 설정값. 임의 기본값 금지)
# ---------------------------------------------------------------------------
LLM_PROVIDER = "openai"
LLM_MODEL = os.getenv("ORCH_LLM_MODEL", "gpt-5.5")   # 사용자 결정 2026-10-07 (copa 게이트웨이에 gpt-5.6-luna 없음)
LLM_API_KEY_ENV = "OPENAI_API_KEY"                         # 키 값은 코드·로그에 남기지 않는다
# 예비 키 (사용자 결정 2026-10-07): .env 에 OPENAI_API_KEY_2 … _9 를 더 넣으면, 쓰던 키의 예산이 떨어졌을 때
# (게이트웨이 429 code=budget_exceeded) 다음 키로 넘어간다. 기록에는 키 순번만 남긴다
LLM_EXTRA_KEY_ENVS = tuple(f"OPENAI_API_KEY_{i}" for i in range(2, 10))
LLM_BASE_URL = os.getenv("OPENAI_BASE_URL") or "https://copa.codyssey.kr/v1"  # 사용자 결정 2026-10-07
LLM_TIMEOUT_S = float(os.getenv("ORCH_LLM_TIMEOUT_S", "60"))     # 사용자 결정 2026-10-07: 30→60 (gpt-5.5 관측 계획 응답 20~42초 실측)
LLM_MAX_CALLS_PER_RUN = int(os.getenv("ORCH_LLM_MAX_CALLS", "50"))  # 사용자 결정 2026-09-30 (서버 1회 실행당)
LLM_TEMPERATURE = None           # TBD — 모델이 지원하는 경우에만 전달

# ---------------------------------------------------------------------------
# F1 공식 출동 기준 / F2 진화 효과 (요청서 §10: 자료 확보 전 미완료)
# ---------------------------------------------------------------------------
OFFICIAL_RULE_SOURCE = None          # TBD — 2026 시행 원문 확보 후 기록
OFFICIAL_RULE_VERSION = None
OFFICIAL_RULE_EFFECTIVE_DATE = None
SUPPRESSION_CONTRACT_READY = False   # 환경 진화 효과 계약 전까지 False
