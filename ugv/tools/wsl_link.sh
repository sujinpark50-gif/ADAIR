#!/usr/bin/env bash
# PX4 인스턴스의 Windows(UGV 서버) 쪽 MAVLink 링크 끊기/잇기: bash wsl_link.sh <instance> stop|start
I=${1:-1}; ACT=${2:-stop}
B=$HOME/PX4-Autopilot/build/px4_sitl_default/bin
LOCAL_PORT=$((14590 + I)); REMOTE_PORT=$((14540 + I))
HOST_IP="$(ip route | awk '/^default/ {print $3; exit}')"
if [ "$ACT" = stop ]; then
  "$B/px4-mavlink" --instance "$I" stop -u "$LOCAL_PORT" && echo "link stopped ($LOCAL_PORT)"
else
  "$B/px4-mavlink" --instance "$I" start -x -u "$LOCAL_PORT" -o "$REMOTE_PORT" -t "$HOST_IP" -m onboard -r 4000000 && echo "link started"
fi
