#!/usr/bin/env bash
# B 월드 제한 구역 + UGV 1대. 기본: kangwon_terrain_a (거점 A 중심 1.2 km), 스폰 ugv/terrain_a_spawn.txt.
#   WORLD=kangwon_terrain_b1 SPAWN=ugv/terrain_b1_spawn.txt bash ugv/tools/wsl_start_b.sh   # 반폭 1 km 구역 (10 % 경사 도로 왕복)
# 스폰 자세 = 도로 부분집합 기준 거점 A 출입 지점, 높이장 도로면 + 0.45 m (Windows 에서 만든다 — 보고서 재현 명령 참고)
set -e
cd "$(dirname "$0")/../.."
WORLD=${WORLD:-kangwon_terrain_a}
SPAWN=${SPAWN:-ugv/terrain_a_spawn.txt}
POSE=$(sed 's/\r$//' "$SPAWN")
WORLD=$WORLD INSTANCE=1 MODEL=adair_ugv POSE="$POSE" GZ_VERBOSE=${GZ_VERBOSE:-3} ./ugv/tools/px4-start.sh
