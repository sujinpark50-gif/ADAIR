#!/usr/bin/env bash
# PX4 인스턴스의 실제 적용 파라미터 (주행 관련) 출력: bash wsl_params.sh <instance>
I=${1:-1}
B=$HOME/PX4-Autopilot/build/px4_sitl_default/bin
for p in NAV_ACC_RAD RA_ACC_RAD_MAX RA_ACC_RAD_GAIN RA_WHEEL_BASE RA_MAX_STR_ANG RA_STR_RATE_LIM RO_SPEED_LIM RO_MAX_THR_SPEED RO_ACCEL_LIM RO_DECEL_LIM RO_JERK_LIM RO_SPEED_RED RO_YAW_RATE_LIM PP_LOOKAHD_GAIN PP_LOOKAHD_MIN PP_LOOKAHD_MAX MIS_DIST_1WP SYS_AUTOSTART; do
  "$B/px4-param" --instance "$I" show "$p" 2>/dev/null | grep -E "^\s*[x\*+ ]*\s*$p" | head -1
done
"$B/px4-ver" --instance "$I" all 2>/dev/null | grep -iE "PX4 git|FW version|Build" | head -3
