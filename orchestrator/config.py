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
    "TEST_THERMAL_50M_V1": {
        "source": "SIMULATED",
        "status": "TEST_ONLY",
        "sensor_type": "THERMAL",
        "agl_m": 50.0,
        "view_direction": "NADIR",
        "footprint_shape": "RECTANGLE",
        "footprint_width_m": 30.0,
        "footprint_height_m": 25.0,
        "reference_payload": "H20T_CLASS_EXAMPLE",
        "basis": "IDEALIZED_DIAGONAL_FOV_GEOMETRY",
    },
}
DEFAULT_SENSOR_PROFILE_ID = "TEST_THERMAL_50M_V1"

# ---------------------------------------------------------------------------
# 현장 환경 측정 (사용자 결정 2026-09-30: 2019 재현 필수 기능)
# 드론·UGV·소방차에 기상 센서가 아직 없어 총괄 쪽 모의 센서로 측정한다 (TEST_ONLY).
# 모의 센서는 도착 위치의 환경 값을 읽을 뿐이며, 환경이 주지 않은 항목은 만들지 않는다.
# ---------------------------------------------------------------------------
SIMULATED_CAPABILITIES = {"UAV": ("WEATHER",), "UGV": ("WEATHER",), "FIRE_ENGINE": ("WEATHER",)}
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
# - 현장 측정 유효시간: 근거가 되는 자료 계약이 아직 없다 → 기본값을 두지 않는다 (TBD).
#   미설정이면 현장 측정은 POLICY_NOT_SET 으로 표시되고 판단에 쓰이지 않는다 (관측소 값 사용).
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


FIELD_WEATHER_MAX_AGE_S = parse_field_weather_max_age(os.getenv("ORCH_FIELD_WEATHER_MAX_AGE_S"))
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
AUTO_RECON_REQUIREMENTS = {"resource_types": ["UAV"], "sensor": "THERMAL", "needs_env_ack": True}

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
# LLM (요청서 §9: 모델·timeout·호출/비용 한도는 설정값. 임의 기본값 금지)
# ---------------------------------------------------------------------------
LLM_PROVIDER = "openai"
LLM_MODEL = os.getenv("ORCH_LLM_MODEL", "gpt-5.6-luna")   # 사용자 결정 2026-09-30
LLM_API_KEY_ENV = "OPENAI_API_KEY"                         # 키 값은 코드·로그에 남기지 않는다
LLM_TIMEOUT_S = float(os.getenv("ORCH_LLM_TIMEOUT_S", "30"))     # 사용자 결정 2026-09-30 (실제 초)
LLM_MAX_CALLS_PER_RUN = int(os.getenv("ORCH_LLM_MAX_CALLS", "50"))  # 사용자 결정 2026-09-30 (서버 1회 실행당)
LLM_TEMPERATURE = None           # TBD — 모델이 지원하는 경우에만 전달

# ---------------------------------------------------------------------------
# F1 공식 출동 기준 / F2 진화 효과 (요청서 §10: 자료 확보 전 미완료)
# ---------------------------------------------------------------------------
OFFICIAL_RULE_SOURCE = None          # TBD — 2026 시행 원문 확보 후 기록
OFFICIAL_RULE_VERSION = None
OFFICIAL_RULE_EFFECTIVE_DATE = None
SUPPRESSION_CONTRACT_READY = False   # 환경 진화 효과 계약 전까지 False
