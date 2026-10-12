#!/usr/bin/env bash
# ugv/tools/px4-stop.sh — px4-start.sh 로 띄운 PX4·Gazebo 를 끈다.
#   ./ugv/tools/px4-stop.sh            # PX4 전부 + Gazebo
#   ./ugv/tools/px4-stop.sh --params   # + PX4 파라미터 저장 파일 삭제 (airframe 기본값으로 다시 시작)
#
# 2026-10-12 신뢰성 수정 (차량 개조 검증 지시서 2절 5~6항):
#  1) px4-start.sh 가 남긴 $STATE_DIR/procs.tsv (역할·PID·시작 시각·파티션·명령) 의 프로세스만 대상으로
#     PID 재사용 검사(/proc/<pid>/stat 시작 시각 일치) → TERM(프로세스 그룹) → STOP_GRACE_S 대기 → KILL → 종료 확인.
#     종료가 확인된 뒤에만 기록을 지운다. 예전 *.pid 파일도 같은 절차 (명령에 gz sim/px4 가 있을 때만).
#  2) 무차별 pkill 대신, 같은 사용자의 gz sim·px4 중 환경 GZ_PARTITION 이 우리 파티션(기본 adair_ugv)인 것만 잔존으로 보고 끈다.
#     다른 파티션(통합 시연 adair_integ, 영상 촬영 등)은 끄지 않고 보고만 한다.
#  3) 결과를 $STATE_DIR/stop_last.json 한 줄로 남긴다 (killed, forced, residual_own, still_alive, other_partition).
#     종료 코드: 0. STOP_STRICT=1 이면 2 = 기록 밖 잔존(앞 실행 오염)이 있었음, 3 = 끝까지 안 죽은 프로세스.
STATE_DIR="${STATE_DIR:-$HOME/.adair-ugv}"
PART="${GZ_PARTITION:-adair_ugv}"
GRACE="${STOP_GRACE_S:-10}"
PROCS="$STATE_DIR/procs.tsv"
killed=0; forced=0; residual=0; still=0; other=""

starttime() { awk '{print $22}' "/proc/$1/stat" 2>/dev/null; }
alive() { [ -d "/proc/$1" ] && [ "$(awk '{print $3}' "/proc/$1/stat" 2>/dev/null)" != "Z" ]; }
part_of() { tr '\0' '\n' < "/proc/$1/environ" 2>/dev/null | sed -n 's/^GZ_PARTITION=//p' | head -1; }
is_sim() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -qE '(^|/| )gz sim|/bin/px4 '; }

# 프로세스(그룹) 하나를 TERM → 대기 → KILL → 확인. $1 pid, $2 그룹으로 보낼지(1/0)
stop_one() {
  local pid=$1 grp=$2 t
  alive "$pid" || return 0
  if [ "$grp" = 1 ]; then kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null; else kill -TERM "$pid" 2>/dev/null; fi
  t=0
  while alive "$pid" && [ "$t" -lt $((GRACE * 5)) ]; do sleep 0.2; t=$((t + 1)); done
  if alive "$pid"; then
    forced=$((forced + 1))
    if [ "$grp" = 1 ]; then kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null; else kill -KILL "$pid" 2>/dev/null; fi
    t=0; while alive "$pid" && [ "$t" -lt 15 ]; do sleep 0.2; t=$((t + 1)); done
  fi
  if alive "$pid"; then still=$((still + 1)); return 1; fi
  killed=$((killed + 1)); return 0
}

# 1) 기록된 프로세스
if [ -f "$PROCS" ]; then
  keep=""
  while IFS=$'\t' read -r role pid st part cmd; do
    [ -z "$pid" ] && continue
    alive "$pid" || continue
    [ "$(starttime "$pid")" = "$st" ] || continue                  # PID 재사용 — 다른 프로세스, 신호 보내지 않음
    stop_one "$pid" 1 || keep="$keep$role	$pid	$st	$part	$cmd"$'\n'
  done < "$PROCS"
  if [ -z "$keep" ]; then rm -f "$PROCS"; else printf '%s' "$keep" > "$PROCS"; fi
fi
# 예전 형식 pid 파일 (procs.tsv 이전에 띄운 것)
for f in "$STATE_DIR"/px4-*.pid "$STATE_DIR"/gz-server.pid; do
  [ -f "$f" ] || continue
  pid=$(cat "$f" 2>/dev/null)
  if [ -n "$pid" ] && alive "$pid" && is_sim "$pid"; then
    stop_one "$pid" 0 && rm -f "$f"
  else
    rm -f "$f"
  fi
done

# 2) 기록 밖 잔존: 같은 사용자 gz sim·px4 중 우리 파티션만
for pid in $(pgrep -u "$(id -u)" -f 'gz sim|/bin/px4 ' 2>/dev/null); do
  [ "$pid" = "$$" ] && continue
  alive "$pid" && is_sim "$pid" || continue
  p=$(part_of "$pid")
  if [ "$p" = "$PART" ]; then
    residual=$((residual + 1)); stop_one "$pid" 0
  else
    other="$other${other:+,}$pid:${p:-none}"
  fi
done

if [ "${1:-}" = "--params" ]; then
  rm -f "$STATE_DIR"/px4-*/parameters*.bson "$STATE_DIR"/px4-*/dataman 2>/dev/null || true
fi
mkdir -p "$STATE_DIR"
printf '{"time":"%s","partition":"%s","killed":%d,"forced":%d,"residual_own":%d,"still_alive":%d,"other_partition":"%s"}\n' \
  "$(date -Is)" "$PART" "$killed" "$forced" "$residual" "$still" "$other" > "$STATE_DIR/stop_last.json"
echo "[ugv] 종료 (끔 $killed, 강제 $forced, 기록 밖 잔존 $residual, 남음 $still${other:+, 다른 파티션 $other 은 그대로})"
if [ "${STOP_STRICT:-0}" = 1 ]; then
  [ "$still" -gt 0 ] && exit 3
  [ "$residual" -gt 0 ] && exit 2
fi
exit 0
