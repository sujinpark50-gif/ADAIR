#!/usr/bin/env bash
# ugv/tools/px4-stop.sh — px4-start.sh 로 띄운 PX4·Gazebo 를 끈다.
#   ./ugv/tools/px4-stop.sh            # PX4 전부 + Gazebo
#   ./ugv/tools/px4-stop.sh --params   # + PX4 파라미터 저장 파일 삭제 (airframe 기본값으로 다시 시작)
STATE_DIR="${STATE_DIR:-$HOME/.adair-ugv}"
for f in "$STATE_DIR"/px4-*.pid "$STATE_DIR"/gz-server.pid; do
  [ -f "$f" ] || continue
  kill "$(cat "$f")" 2>/dev/null || true
  rm -f "$f"
done
pkill -f "gz sim --verbose=[0-9] -r -s" 2>/dev/null || true
pkill -f "gz sim -g" 2>/dev/null || true
if [ "${1:-}" = "--params" ]; then
  rm -f "$STATE_DIR"/px4-*/parameters*.bson "$STATE_DIR"/px4-*/dataman 2>/dev/null || true
fi
echo "[ugv] 종료"
