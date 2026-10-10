#!/usr/bin/env bash
# 4대 (fleet_ab.json) 를 A 평탄 월드에 띄운다. 스폰 자세는 Windows 에서 만든 ugv/fleet_ab_spawn.txt
#   (.venv\Scripts\python -m ugv.tools.spawn_pose --fleet ugv/fleet_ab.json > ugv/fleet_ab_spawn.txt)
set -e
cd "$(dirname "$0")/../.."
while read -r INST MODEL POSE RID; do
  [ -z "$INST" ] && continue
  echo "== $RID (instance $INST, $MODEL)"
  WORLD=kangwon_flat INSTANCE="$INST" MODEL="$MODEL" POSE="$POSE" ./ugv/tools/px4-start.sh | tail -2
done < <(sed 's/\r$//' ugv/fleet_ab_spawn.txt)
