#!/usr/bin/env bash
# ugv/tools/susp_static_test.sh — (WSL 안에서) 차량 후보 정지·대각 시험 S1·S2 (차량 개조 계획 4.4절). PX4 없이 Gazebo 월드·차량만 띄워 상세 기록.
#   bash ugv/tools/susp_static_test.sh <출력 폴더> <후보>:<월드>[:<기록 s>] ...
#   예) bash ugv/tools/susp_static_test.sh logs/ugv/stage6/susp_static base:adair_flat_susp susp_a:adair_flat_diag2 susp_a_m:adair_flat_susp
# 월드 = ugv/gazebo/worlds/test/<월드>.sdf. 스폰 원점 yaw 0, z = 0.40 m (+ 대각 판 두께). 시험 전용 파티션·상태 폴더 (실제 시험·다른 작업과 섞이지 않음).
# 바퀴 구동 명령이 없으므로 JointController 속도 0 = 바퀴 고정 상태에서 정지 하중을 잰다.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/$1"; shift
mkdir -p "$OUT"
export STATE_DIR="$HOME/.adair-susptest" GZ_PARTITION=adair_susptest GZ_IP=127.0.0.1
for spec in "$@"; do
  IFS=: read -r V WNAME DUR <<< "$spec"
  DUR=${DUR:-60}
  case "$WNAME" in *diag2) Z=0.42 ;; *diag5) Z=0.45 ;; *) Z=0.40 ;; esac
  TAG="${V}_${WNAME}"
  echo "== $TAG (기록 ${DUR} s)"
  STOP_STRICT=1 bash "$REPO/ugv/tools/px4-stop.sh" >/dev/null; rc=$?
  [ "$rc" = 0 ] || echo "  [경고] 시작 전 잔존 정리 (종료 코드 $rc) — $(cat "$STATE_DIR/stop_last.json")"
  ADAIR_RUN_ID="$TAG" NO_PX4=1 UGV_MODEL_VARIANT="$V" WORLD="$WNAME" WORLD_FILE="$REPO/ugv/gazebo/worlds/test/$WNAME.sdf" \
    MODEL=adair_ugv INSTANCE=1 POSE="0,0,$Z,0" GZ_VERBOSE=2 bash "$REPO/ugv/tools/px4-start.sh" | tail -2
  rc=${PIPESTATUS[0]}
  if [ "$rc" != 0 ]; then echo "  기동 실패 (종료 코드 $rc) — 무효"; echo "{\"tag\":\"$TAG\",\"invalid\":\"start rc $rc\"}" > "$OUT/$TAG.invalid.json"; continue; fi
  ( cd "$REPO" && python3 -m ugv.tools.gz_detail --world "$WNAME" --model adair_ugv_1 --out "$OUT/$TAG.csv" --duration "$DUR" ) 2>&1 | tail -1
  cp "$STATE_DIR/gz-server.log" "$OUT/${TAG}_gz-server.log" 2>/dev/null
  STOP_STRICT=1 bash "$REPO/ugv/tools/px4-stop.sh"; rc=$?
  cp "$STATE_DIR/stop_last.json" "$OUT/${TAG}_stop.json"
  [ "$rc" = 0 ] || echo "  [경고] 종료 확인 이상 (종료 코드 $rc)"
done
echo "SUSP_STATIC_DONE $(date)"
