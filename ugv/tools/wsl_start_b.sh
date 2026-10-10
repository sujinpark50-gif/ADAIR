#!/usr/bin/env bash
# B 월드 제한 구역(kangwon_terrain_a) + UGV 1대. 스폰 자세 = ugv/terrain_a_spawn.txt (도로 부분집합 기준 거점 A 출입 지점,
# 높이장 도로면 + 0.45 m. Windows 에서 만든다 — 보고서 재현 명령 참고)
set -e
cd "$(dirname "$0")/../.."
POSE=$(sed 's/\r$//' ugv/terrain_a_spawn.txt)
WORLD=kangwon_terrain_a INSTANCE=1 MODEL=adair_ugv POSE="$POSE" GZ_VERBOSE=${GZ_VERBOSE:-3} ./ugv/tools/px4-start.sh
