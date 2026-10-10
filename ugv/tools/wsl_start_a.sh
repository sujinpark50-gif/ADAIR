#!/usr/bin/env bash
# A 평탄 월드(kangwon_flat) + UGV 1대(인스턴스 1, 거점 A 출입 지점). Windows 에서:
#   wsl.exe -d Ubuntu-24.04 -- bash /mnt/c/.../ugv/tools/wsl_start_a.sh
set -e
cd "$(dirname "$0")/../.."
WORLD=kangwon_flat INSTANCE=1 POSE="${POSE_A:-3992.08,5606.28,0.4,-2.3835}" ./ugv/tools/px4-start.sh
