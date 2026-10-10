#!/usr/bin/env bash
# PX4 인스턴스 파라미터 설정: bash wsl_param_set.sh <instance> NAME VALUE [NAME VALUE ...]
I=$1; shift
B=$HOME/PX4-Autopilot/build/px4_sitl_default/bin
while [ $# -ge 2 ]; do "$B/px4-param" --instance "$I" set "$1" "$2" | tail -1; shift 2; done
