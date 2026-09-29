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
# 우선순위 (사용자 결정 2026-09-30)
#   HUMAN_CHOICE      비슷하거나 엇갈리는 후보를 화면에 나란히 보여주고, 자원이 모자라면 사람이 고를 때까지 대기
#   AUTO_HIGHER_RISK  인명 → 위험도가 조금이라도 높은 곳 순으로 바로 보내고 '자동 결정'으로 표시
# ---------------------------------------------------------------------------
PRIORITY_MODE = os.getenv("ORCH_PRIORITY_MODE", "HUMAN_CHOICE")
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
