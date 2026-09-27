#!/usr/bin/env bash
# PX4 SITL rover 인스턴스 기동.
#   ./ugv/tools/px4-start.sh 0    A 거점 (포트 14540)
#   ./ugv/tools/px4-start.sh 1    B 거점 (포트 14541)
#
# 확인된 주의사항
#   PX4_HOME_ALT 는 0. Gazebo 월드 지면이 0m 이므로 500 을 주면
#     PX4 가 500m 상공에 있다고 믿고 Takeoff 를 시도한다.
#   SYS_HAS_MAG / FD_FAIL_* 는 건드리지 말 것.
#     SYS_HAS_MAG=0 이면 yaw_align 이 서지 않아 arm 이 거부된다.
#   파라미터는 fs/parameters.bson 에 저장되어 재시작해도 남는다.
#     오염되면 px4-stop.sh --params 로 정리.

set -e

INSTANCE="${1:-0}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

# 홈 = 실제 도로망(ugv/data/road_network.json)의 거점 노드 좌표
# 인스턴스마다 GZ_PARTITION 을 달리해 Gazebo world 를 따로 띄운다 (팀 결정: 차량당 Gazebo 1개, 차량 간 상호작용 없음).
# 그래서 모델 위치는 각 world 의 원점(0,0).
case "$INSTANCE" in
  0)  LAT=38.092790;  LON=128.179569 ;;  # 노드 A (원통119 → 도로망 경계 노드)
  1)  LAT=37.930108;  LON=128.292720 ;;  # 노드 B (기린119 → 도로망 경계 노드)
  *)  echo "지원하지 않는 인스턴스: $INSTANCE"; exit 1 ;;
esac
POSE="0,0,0,0,0,0"

cd "$PX4_DIR"
[ -d .venv ] && source .venv/bin/activate

echo "인스턴스 $INSTANCE  홈=($LAT, $LON, 0m)  포트=$((14540 + INSTANCE))  world 파티션=ugv$INSTANCE"

PX4_SYS_AUTOSTART=4009 \
PX4_SIM_MODEL=gz_r1_rover \
PX4_HOME_LAT="$LAT" \
PX4_HOME_LON="$LON" \
PX4_HOME_ALT=0 \
PX4_GZ_MODEL_POSE="$POSE" \
GZ_IP=127.0.0.1 \
GZ_PARTITION="ugv$INSTANCE" \
HEADLESS="${HEADLESS:-1}" \
./build/px4_sitl_default/bin/px4 -i "$INSTANCE"