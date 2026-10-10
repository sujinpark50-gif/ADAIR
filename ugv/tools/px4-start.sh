#!/usr/bin/env bash
# ugv/tools/px4-start.sh — WSL(Linux)에서 Gazebo 월드 + adair_ugv 차량 + PX4 SITL 1대를 띄운다.
#
#   ./ugv/tools/px4-start.sh                       # 평지 시험 월드, 인스턴스 1, 원점에 스폰, 창 없음
#   GUI=1 ./ugv/tools/px4-start.sh                 # Gazebo 창 함께
#   WORLD=adair_flat INSTANCE=1 POSE="0,0,0.4,0" ./ugv/tools/px4-start.sh
#
# 환경 변수
#   PX4_DIR    PX4-Autopilot 경로 (기본 ~/PX4-Autopilot, 빌드 build/px4_sitl_default 필요)
#   WORLD      ugv/gazebo/worlds/<WORLD>.sdf (기본 adair_flat). 월드 파일 안의 world name 과 같아야 한다
#   WORLD_FILE 월드 파일 경로를 직접 줄 때 (시험용 사본 등). 기본 ugv/gazebo/worlds/<WORLD>.sdf
#   GZ_SIM_RESOURCE_PATH 를 미리 주면 그 경로도 모델 검색에 쓴다 (시험용 모델 사본)
#   INSTANCE   PX4 인스턴스 번호 (기본 1. 0 은 UAV 몫). MAVLink 는 14540+INSTANCE 로 보낸다
#   MODEL      차량 모델 adair_ugv (기본, airframe 51100) / adair_firetruck (airframe 51101). Gazebo 이름 = <MODEL>_<INSTANCE>
#   POSE       스폰 위치 "x,y,z,yaw" (Gazebo 로컬 ENU, m·rad). 기본 "0,0,0.4,0"
#              거점 출입 지점: python -m ugv.tools.spawn_pose A  (도로 월드 kangwon_flat 에서)
#   HOST_IP    MAVLink 를 받을 컴퓨터 (UGV 서버). 기본: WSL 기본 게이트웨이 = Windows 호스트
#   GUI        1 이면 Gazebo 창
#   SPEED      PX4_SIM_SPEED_FACTOR (기본 1). 2단계 측정은 1 로 한다
#   STATE_DIR  로그·PX4 작업 폴더 (기본 ~/.adair-ugv)
#
# PX4 소스는 고치지 않는다. 우리 airframe 파일만 빌드 결과 폴더(etc/init.d-posix/airframes)에 복사한다.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
: "${PX4_DIR:=$HOME/PX4-Autopilot}"        # PX4 소스·빌드 위치
BUILD=$PX4_DIR/build/px4_sitl_default
WORLD="${WORLD:-adair_flat}"
WORLD_FILE="${WORLD_FILE:-$REPO/ugv/gazebo/worlds/$WORLD.sdf}"
INSTANCE="${INSTANCE:-1}"
POSE="${POSE:-0,0,0.4,0}"
HOST_IP="${HOST_IP:-$(ip route | awk '/^default/ {print $3; exit}')}"
STATE_DIR="${STATE_DIR:-$HOME/.adair-ugv}"
MODEL="${MODEL:-adair_ugv}"
MODEL_NAME="${MODEL}_${INSTANCE}"
case "$MODEL" in
  adair_ugv)       AUTOSTART=51100; AIRFRAME="51100_gz_adair_ugv" ;;
  adair_firetruck) AUTOSTART=51101; AIRFRAME="51101_gz_adair_firetruck" ;;
  *) echo "모델 $MODEL 없음 (adair_ugv / adair_firetruck)"; exit 1 ;;
esac

[ -x "$BUILD/bin/px4" ] || { echo "PX4 빌드 없음: $BUILD/bin/px4 (make px4_sitl 먼저)"; exit 1; }
mkdir -p "$STATE_DIR/px4-$INSTANCE"

# 1) airframe 설치 (Windows 에서 편집해 CRLF 가 섞여도 sh 가 읽게 LF 로)
sed 's/\r$//' "$REPO/ugv/px4/airframes/$AIRFRAME" > "$BUILD/etc/init.d-posix/airframes/$AIRFRAME"

# 2) Gazebo 환경: PX4 기본 설정 + 우리 모델·월드 경로. 파티션을 나눠 UAV Gazebo 와 섞이지 않게 한다
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}" GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
# shellcheck disable=SC1091
. "$BUILD/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH="$REPO/ugv/gazebo/models:$REPO/ugv/gazebo/worlds:$GZ_SIM_RESOURCE_PATH"
export GZ_PARTITION="${GZ_PARTITION:-adair_ugv}"
export GZ_IP="${GZ_IP:-127.0.0.1}"

world_up() { gz service -i --service "/world/$WORLD/scene/info" 2>&1 | grep -q "Service providers"; }

# 3) 월드 (이미 떠 있으면 그대로 쓴다)
if ! world_up; then
  echo "[ugv] Gazebo 월드 시작: $WORLD"
  nohup gz sim --verbose="${GZ_VERBOSE:-1}" -r -s "$WORLD_FILE" > "$STATE_DIR/gz-server.log" 2>&1 &
  echo $! > "$STATE_DIR/gz-server.pid"
  for _ in $(seq 60); do world_up && break; sleep 1; done
  world_up || { echo "[ugv] Gazebo 월드가 뜨지 않음 ($STATE_DIR/gz-server.log)"; exit 1; }
fi
if [[ ${GUI:-0} == 1 ]] && ! pgrep -f "gz sim -g" >/dev/null; then       # 창은 하나만
  nohup gz sim -g > "$STATE_DIR/gz-gui.log" 2>&1 &
fi

# 4) 차량 스폰 (같은 이름이 있으면 건너뜀)
if gz model --list 2>/dev/null | grep -qx "    - $MODEL_NAME"; then
  echo "[ugv] 이미 있는 차량: $MODEL_NAME"
else
  IFS=',' read -r PX PY PZ PYAW <<< "$POSE"
  SDF="<sdf version='1.9'><include><uri>model://$MODEL</uri><pose>$PX $PY $PZ 0 0 ${PYAW:-0}</pose></include></sdf>"
  gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "name: \"$MODEL_NAME\", allow_renaming: false, sdf: \"$SDF\"" > /dev/null
  echo "[ugv] 차량 스폰: $MODEL_NAME @ $POSE"
fi

# 5) PX4 (이미 떠 있으면 건너뜀)
if pgrep -f "px4 -i $INSTANCE " > /dev/null; then
  echo "[ugv] PX4 인스턴스 $INSTANCE 이미 실행 중"
else
  (
    cd "$STATE_DIR/px4-$INSTANCE"
    PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=$AUTOSTART PX4_GZ_MODEL_NAME="$MODEL_NAME" PX4_GZ_WORLD="$WORLD" \
    PX4_SIM_SPEED_FACTOR="${SPEED:-1}" HEADLESS=1 \
      nohup "$BUILD/bin/px4" -i "$INSTANCE" -d -w "$STATE_DIR/px4-$INSTANCE" "$BUILD/etc" \
      > "$STATE_DIR/px4-$INSTANCE.log" 2>&1 &
    echo $! > "$STATE_DIR/px4-$INSTANCE.pid"
  )
  for _ in $(seq 60); do grep -q "Ready for takeoff\|home set\|ekf2.*local position" "$STATE_DIR/px4-$INSTANCE.log" 2>/dev/null && break; sleep 1; done
fi

# 6) UGV 서버 쪽으로 MAVLink 링크 추가 (PX4 기본 링크는 WSL 안 127.0.0.1 로만 보낸다)
LOCAL_PORT=$((14590 + INSTANCE)); REMOTE_PORT=$((14540 + INSTANCE))
"$BUILD/bin/px4-mavlink" --instance "$INSTANCE" start -x -u "$LOCAL_PORT" -o "$REMOTE_PORT" -t "$HOST_IP" -m onboard -r 4000000 \
  > /dev/null 2>&1 || true
echo "[ugv] MAVLink → $HOST_IP:$REMOTE_PORT (UGV 서버는 udpin://0.0.0.0:$REMOTE_PORT 로 받는다)"
echo "[ugv] 로그: $STATE_DIR/px4-$INSTANCE.log, $STATE_DIR/gz-server.log  /  GZ_PARTITION=$GZ_PARTITION"
