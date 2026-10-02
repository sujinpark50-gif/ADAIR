#!/usr/bin/env bash
# 2019 인제 디지털 트윈 — 서버 6개를 한 번에 띄운다. 끄기: Ctrl+C (전부 함께 종료)
#
#   bash tools/run_twin.sh                 # UAV mock (PX4 없이), 100배속
#   UAV_MODE=real bash tools/run_twin.sh   # UAV PX4 real (PX4 SITL 이 먼저 떠 있어야 함), 시계 1배속
#   TWIN_LAUNCH=base bash tools/run_twin.sh  # 드론을 원통 기지에서 띄움 → 12 km 라 복귀 여유 부족(COUNTER),
#                                            #  총괄이 그 제안을 실을 칸이 없어(UAV-02) 출동 못 함 — 현재 계약 그대로의 결과
#   기본 TWIN_LAUNCH=forward : 소방차가 닿는 현장 근처 도로 노드에서 띄움 (시나리오 B)
#
#   환경 계약 서버 :8300 (진짜 세계 CA = 재현과 같은 설정) · UAV Agent A-uav1 :8000, A-uav2 :8001
#   UGV 서버 :8100 · 총괄 :8200 (환경 서버에 연결) · 관제판 :8080 · 트윈 시계(환경 진행)
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
PY="${PY:-${VENV:-$HOME/uav-venv}/bin/python}"; [[ -x "$PY" ]] || PY=python3
MODE="${UAV_MODE:-mock}"
LAUNCH="${TWIN_LAUNCH:-forward}"
SPEED="${TWIN_SPEED:-$([[ $MODE == real ]] && echo 1 || echo 100)}"
LOG="$ROOT/logs/twin/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOG"
# 재현과 같은 진짜 세계 설정 (web/static/inje2019/scenario.json 의 twin_env)
eval "$("$PY" - <<'PYX'
import json
d = json.load(open("web/static/inje2019/scenario.json", encoding="utf-8"))["twin_env"]
for k, v in d.items(): print(f'export {k}="{v}"')
PYX
)"
export ENV_STATE_DIR="$LOG/env" ENV_RESUME=0
UAV_HOME_ENV=""
if [[ $LAUNCH == forward && $MODE == mock ]]; then
  UAV_HOME_ENV="$("$PY" - <<'PYX'
import json, gz_bridge as gb
import math
d = json.load(open("web/static/inje2019/scenario.json", encoding="utf-8"))
path, sp = d["vehicles"][0]["path"], d["spring"]
# 집결지 = 소방차 경로에서 발화점 700 m 안으로 처음 들어가는 지점 (불 바로 위에서 이착륙하지 않는다)
p = next((q for q in path if math.hypot(q[0] - sp["gx"], q[1] - sp["gy"]) * 90 <= 700), path[-1])
lat, lon = gb.grid_cell_to_latlon(p[0], p[1]); print(f"{lat:.6f},{lon:.6f},{gb.terrain_elev(round(p[0]), round(p[1])):.1f}")
PYX
)"
fi
export ORCH_ENV_MODE=team_http ORCH_DB_PATH="$LOG/orchestrator.sqlite3"
PIDS=()
start(){ local name=$1; shift; ( "$@" >"$LOG/$name.log" 2>&1 ) & PIDS+=($!); echo "  $name → $LOG/$name.log"; }
wait_http(){ for _ in $(seq 1 60); do curl -s -o /dev/null "$1" && return 0; sleep 1; done; echo "  ✖ $1 응답 없음 (로그 확인)"; return 1; }
cleanup(){ echo; echo "종료 중…"; kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; echo "로그: $LOG"; }
trap cleanup EXIT INT TERM

echo "▶ 디지털 트윈 기동 (UAV $MODE · 이륙 $LAUNCH${UAV_HOME_ENV:+ [$UAV_HOME_ENV]} · 시계 ${SPEED}배속 · 로그 $LOG)"
start env      "$PY" -m uvicorn environment.server:app --host 127.0.0.1 --port 8300 --log-level warning
for spec in "A-uav1 8000" "A-uav2 8001"; do set -- $spec
  ( cd uav/uav-agent && UAV_ID=$1 UAV_MODE=$MODE UAV_STATE_DIR="$LOG/uav" UAV_HOME="$UAV_HOME_ENV" exec "$PY" -m uvicorn main:app --host 127.0.0.1 --port $2 --log-level warning ) >"$LOG/uav_$1.log" 2>&1 &
  PIDS+=($!); echo "  uav $1 → $LOG/uav_$1.log"; done
start ugv      env UGV_DRIVER=sim UGV_STATE_DIR="$LOG/ugv" "$PY" -m uvicorn ugv.server:app --host 127.0.0.1 --port 8100 --log-level warning
wait_http http://127.0.0.1:8300/health && wait_http http://127.0.0.1:8000/health && wait_http http://127.0.0.1:8100/ugv
start orch     "$PY" -m orchestrator.api
start web      "$PY" -m uvicorn web.app:app --host 0.0.0.0 --port 8080 --log-level warning
wait_http http://127.0.0.1:8200/health && wait_http http://127.0.0.1:8080/inje3d
start clock    "$PY" tools/twin_clock.py --speed "$SPEED"
[[ "${TWIN_OPERATOR:-1}" == 1 ]] && start operator "$PY" tools/twin_operator.py     # 일몰 뒤 순찰 요청 (끄기: TWIN_OPERATOR=0)
echo
echo "✔ 브라우저:  http://localhost:8080/inje3d  → '실시간 연결 LIVE'"
echo "  총괄 화면:  http://localhost:8200/board"
echo "  끄기: Ctrl+C"
wait
