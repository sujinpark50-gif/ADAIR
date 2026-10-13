#!/usr/bin/env bash
# ugv/tools/susp_static_test.sh — (WSL 안에서) 차량 후보 정지·대각 시험 S1·S2 (차량 개조 계획 4.4절). PX4 없이 Gazebo 월드·차량만 띄워 상세 기록.
#   bash ugv/tools/susp_static_test.sh <출력 폴더> <후보>:<월드>[:<기록 s>[:<모델>]] ...   (모델 기본 adair_ugv, 소방차 adair_firetruck → 태그 앞 ft-)
#   예) bash ugv/tools/susp_static_test.sh logs/ugv/stage6/susp_static base:adair_flat_susp susp_a:adair_flat_diag2 susp_a_m:adair_flat_susp
# 월드 = ugv/gazebo/worlds/test/<월드>.sdf. 스폰 원점 yaw 0, z = 0.40 m (+ 대각 판 두께). 시험 전용 파티션·상태 폴더 (실제 시험·다른 작업과 섞이지 않음).
# 바퀴 구동 명령이 없으므로 JointController 속도 0 = 바퀴 고정 상태에서 정지 하중을 잰다.
# 2026-10-13 (S1·S3 독립 검증 P2): 사전 정지 실패·잔존 → 시험 안 함 (<TAG>.contaminated), 기동 실패 → <TAG>.invalid,
#   실행 출처(run_provenance)·증거 검사(logs/ugv/stage6/verify_drive_evidence.py: 후보 모델 해시·Gazebo prismatic 수) 실패 → <TAG>.invalid_evidence,
#   사후 정지 실패·잔존 → <TAG>.invalid_stop. 코드 커밋 = VEHICLE_TREE_COMMIT (Windows 쪽에서 넘김 — WSL git 은 worktree gitdir C:/… 를 못 읽음).
#   기록기(gz_detail) 출력·종료 코드는 <TAG>_gz_detail.log. 무효 표시 회차는 판정기(susp_static_report)가 INVALID 로 뺀다.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/$1"; shift
mkdir -p "$OUT"
export STATE_DIR="$HOME/.adair-susptest" GZ_PARTITION=adair_susptest GZ_IP=127.0.0.1
for spec in "$@"; do
  IFS=: read -r V WNAME DUR MDL <<< "$spec"
  DUR=${DUR:-60}; MDL=${MDL:-adair_ugv}
  case "$WNAME" in *diag2*) Z=0.42 ;; *diag5*) Z=0.45 ;; *) Z=0.40 ;; esac
  PFX=""; if [ "$MDL" = adair_firetruck ]; then PFX="ft-"; Z=$(echo "$Z - 0.35" | bc); fi   # 소방차 base_link 가 모델 원점 위 0.5 m (바퀴 반지름 0.5)
  TAG="${PFX}${V}_${WNAME}"
  echo "== $TAG (기록 ${DUR} s)"
  rm -f "$OUT/$TAG.contaminated" "$OUT/$TAG.invalid" "$OUT/$TAG.invalid_evidence" "$OUT/$TAG.invalid_stop"
  STOP_STRICT=1 bash "$REPO/ugv/tools/px4-stop.sh" >/dev/null; rc=$?
  cp "$STATE_DIR/stop_last.json" "$OUT/stop_pre_$TAG.json" 2>/dev/null
  if [ "$rc" != 0 ] || ! grep -q '"residual_own":0,"still_alive":0' "$OUT/stop_pre_$TAG.json" 2>/dev/null; then
    echo "  앞 실행 잔존 — 시험 안 함 (종료 코드 $rc): $(cat "$STATE_DIR/stop_last.json" 2>/dev/null)" | tee "$OUT/$TAG.contaminated"; continue
  fi
  ADAIR_RUN_ID="$TAG" NO_PX4=1 UGV_MODEL_VARIANT="$V" WORLD="$WNAME" WORLD_FILE="$REPO/ugv/gazebo/worlds/test/$WNAME.sdf" \
    MODEL=$MDL INSTANCE=1 POSE="0,0,$Z,0" GZ_VERBOSE=2 bash "$REPO/ugv/tools/px4-start.sh" | tail -2
  rc=${PIPESTATUS[0]}
  if [ "$rc" != 0 ]; then echo "  기동 실패 (종료 코드 $rc) — 무효" | tee "$OUT/$TAG.invalid"; bash "$REPO/ugv/tools/px4-stop.sh" >/dev/null 2>&1; continue; fi
  printf '1 %s 0,0,%s,0 %s_1
' "$MDL" "$Z" "$MDL" > "$OUT/spawn_$TAG.txt"
  ( cd "$REPO" && python3 -m ugv.tools.run_provenance --run-id "$TAG" --spawn "$OUT/spawn_$TAG.txt" --out "$OUT/prov_$TAG.json" ) > "$OUT/prov_$TAG.txt" 2>&1
  if ! python3 "$REPO/../ADAIR/logs/ugv/stage6/verify_drive_evidence.py" "$OUT/prov_$TAG.json" "$V" "$MDL" "${VEHICLE_TREE_COMMIT:-$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo 'unknown(WSL 에서 Windows worktree git 해석 불가 — VEHICLE_TREE_COMMIT 로 넘길 것)')}" > "$OUT/verify_$TAG.txt" 2>&1; then
    echo "  증거 검사 실패 — 시험 안 함: $(tail -1 "$OUT/verify_$TAG.txt")" | tee "$OUT/$TAG.invalid_evidence"; bash "$REPO/ugv/tools/px4-stop.sh" >/dev/null 2>&1; continue
  fi
  ( cd "$REPO" && python3 -m ugv.tools.gz_detail --world "$WNAME" --model ${MDL}_1 --out "$OUT/$TAG.csv" --duration "$DUR" ) > "$OUT/${TAG}_gz_detail.log" 2>&1
  echo "gz_detail_rc=$?" >> "$OUT/${TAG}_gz_detail.log"; tail -2 "$OUT/${TAG}_gz_detail.log"
  cp "$STATE_DIR/gz-server.log" "$OUT/${TAG}_gz-server.log" 2>/dev/null
  STOP_STRICT=1 bash "$REPO/ugv/tools/px4-stop.sh"; rc=$?
  cp "$STATE_DIR/stop_last.json" "$OUT/${TAG}_stop.json"
  if [ "$rc" != 0 ] || ! grep -q '"still_alive":0' "$OUT/${TAG}_stop.json" || ! grep -q '"residual_own":0' "$OUT/${TAG}_stop.json"; then
    echo "  사후 종료 확인 실패 (종료 코드 $rc): $(cat "$OUT/${TAG}_stop.json")" | tee "$OUT/$TAG.invalid_stop"
  fi
done
echo "SUSP_STATIC_DONE $(date)"
