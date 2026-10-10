#!/usr/bin/env bash
# 4대 (fleet_ab.json) 를 A 평탄 월드에 띄운다. 스폰 자세는 Windows 에서 만든 ugv/fleet_ab_spawn.txt
#   (.venv\Scripts\python -m ugv.tools.spawn_pose --fleet ugv/fleet_ab.json > ugv/fleet_ab_spawn.txt)
# 인자로 다른 스폰 파일(같은 형식 "INSTANCE MODEL POSE 차량ID")을 주면 그 차량만 (pair_test: logs/ugv/pair_<시나리오>_spawn.txt)
set -e
cd "$(dirname "$0")/../.."
SPAWN="${1:-ugv/fleet_ab_spawn.txt}"
while read -r INST MODEL POSE RID; do
  [ -z "$INST" ] && continue
  echo "== $RID (instance $INST, $MODEL)"
  WORLD=kangwon_flat INSTANCE="$INST" MODEL="$MODEL" POSE="$POSE" GZ_VERBOSE=${GZ_VERBOSE:-3} ./ugv/tools/px4-start.sh | tail -2
done < <(sed 's/\r$//' "$SPAWN")
