#!/usr/bin/env bash
# bash wsl_param_show.sh <instance> NAME [NAME ...]
I=$1; shift
B=$HOME/PX4-Autopilot/build/px4_sitl_default/bin
for p in "$@"; do "$B/px4-param" --instance "$I" show "$p" 2>/dev/null | grep -E "^\s*[x\*+ ]*\s*$p" | head -1; done
