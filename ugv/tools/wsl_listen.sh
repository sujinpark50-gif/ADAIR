#!/usr/bin/env bash
# PX4 uORB 토픽 한 번 읽기: bash wsl_listen.sh <instance> <topic> [-n N]
I=$1; shift
"$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4-listener" --instance "$I" "$@"
