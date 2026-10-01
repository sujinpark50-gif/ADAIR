#!/usr/bin/env bash
# UGV 3대: Gazebo 월드 1개(kangwon, UAV 와 같은 강원 지형) + PX4 SITL 인스턴스 3개.
# Gazebo/PX4 를 돌리는 기계(현재 Windows WSL)에서 저장소 루트 기준으로 실행한다.
#
#   UGV_HOST=192.168.0.23 ./ugv/tools/px4-start.sh           # 3대 모두 (UGV_HOST = UGV 서버가 도는 Mac 의 IP)
#   UGV_HOST=192.168.0.23 ./ugv/tools/px4-start.sh A-ugv1    # 한 대만
#   GUI=1 SPEED=5 UGV_HOST=... ./ugv/tools/px4-start.sh      # Gazebo 창 + 5배속
#   ./ugv/tools/px4-stop.sh                                  # 전부 종료
#
# 구성 (값의 정본은 ugv/config.py RESOURCES 와 ugv/data/road_network.json spawn — 이 스크립트는 읽기만 한다)
#   자원      PX4 인스턴스  MAVLink 포트  모델
#   A-ugv1    1             14541         r1_rover
#   B-ugv1    2             14542         r1_rover
#   A-fire1   3             14543         rover_ackermann (없으면 lawnmower → r1_rover)
#   인스턴스 0 / 14540 은 UAV 몫이라 비워 둔다.
#
# 동작
#   1) gz sim 서버를 kangwon.sdf 로 한 번 띄운다 (GZ_PARTITION=ugv — UAV 시뮬레이션과 섞이지 않게)
#   2) PX4 를 PX4_GZ_STANDALONE=1 로 인스턴스마다 띄운다 → 이미 떠 있는 월드에 자기 차량을 스폰한다
#      에어프레임 번호는 모델 이름으로 PX4 소스(airframes/*_gz_<모델>)에서 찾는다 (버전마다 번호가 달라서)
#   3) UGV_HOST 가 있으면 인스턴스마다 MAVLink 링크를 하나 더 열어 UGV_HOST:1454i 로 보낸다
#      (PX4 기본 링크는 127.0.0.1 로만 나간다). 이게 안 되면 ugv/tools/mavlink_relay.py 를 쓴다.
#
# 시간: SPEED(=PX4_SIM_SPEED_FACTOR) 는 UGV 서버의 UGV_TIME_SCALE 과 반드시 같게 한다.
#       다르면 ETA·시나리오 시각과 실제 주행이 어긋난다.
#
# 확인된 주의사항 (이전 단독 월드 구성에서)
#   SYS_HAS_MAG / FD_FAIL_* 는 건드리지 말 것 — yaw_align 이 서지 않아 arm 이 거부된다.
#   파라미터는 build/px4_sitl_default/rootfs/N/ 아래에 남는다. 오염되면 px4-stop.sh --params.
#   PX4 는 HEADLESS 변수가 "있기만 하면" GUI 를 끈다 → GUI 는 이 스크립트가 따로 띄운다.

set -eo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD="$PX4_DIR/build/px4_sitl_default"
WORLD_SDF="$REPO/uav/gazebo/kangwon.sdf"
WORLD_NAME="kangwon"
SPEED="${SPEED:-1}"
UGV_HOST="${UGV_HOST:-}"
LOG_DIR="${LOG_DIR:-/tmp/ugv-px4}"
ONLY="${1:-}"

[ -x "$BUILD/bin/px4" ] || { echo "PX4 빌드 없음: $BUILD (make px4_sitl 먼저)"; exit 1; }
[ -f "$WORLD_SDF" ]     || { echo "월드 없음: $WORLD_SDF"; exit 1; }
mkdir -p "$LOG_DIR"
[ -d "$PX4_DIR/.venv" ] && source "$PX4_DIR/.venv/bin/activate"

# PX4 의 모델·월드 경로 (PX4_GZ_MODELS, GZ_SIM_RESOURCE_PATH 등). standalone 이면 PX4 가 안 읽으므로 여기서 읽는다
GZ_ENV=$(find "$BUILD" -maxdepth 2 -name gz_env.sh | head -1)
[ -n "$GZ_ENV" ] || { echo "gz_env.sh 없음 ($BUILD) — PX4 gz 빌드 확인"; exit 1; }
# shellcheck disable=SC1090
source "$GZ_ENV"
export GZ_SIM_RESOURCE_PATH="$REPO/uav/gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"   # model://kangwon
export GZ_PARTITION=ugv
export GZ_IP=127.0.0.1
export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="$WORLD_NAME"
export PX4_SIM_SPEED_FACTOR="$SPEED"
export HEADLESS=1

# 모델 이름 → 에어프레임 번호. 없으면 빈 문자열
airframe_of() {
    local f
    f=$(ls "$PX4_DIR"/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_"$1" 2>/dev/null | head -1)
    [ -n "$f" ] && basename "$f" | cut -d_ -f1
}

# 1) Gazebo 서버 (이미 떠 있으면 재사용)
if gz topic -l 2>/dev/null | grep -q "^/world/$WORLD_NAME/clock"; then
    echo "Gazebo 월드 $WORLD_NAME 이미 실행 중 (파티션 $GZ_PARTITION)"
else
    echo "Gazebo 서버 시작: $WORLD_SDF"
    gz sim --verbose=1 -r -s "$WORLD_SDF" > "$LOG_DIR/gz.log" 2>&1 &
    echo $! >> "$LOG_DIR/pids"                      # px4-stop.sh 가 우리 프로세스만 끄도록 기록
    for _ in $(seq 60); do
        gz service -i --service "/world/$WORLD_NAME/scene/info" 2>&1 | grep -q "Service providers" && break
        sleep 1
    done
fi
if [ "${GUI:-0}" = "1" ] && ! pgrep -f "gz sim -g" > /dev/null; then
    gz sim -g > "$LOG_DIR/gz-gui.log" 2>&1 &
    echo $! >> "$LOG_DIR/pids"
fi

# 2) 자원별 PX4. 자원 목록·스폰 위치는 저장소 파일에서 읽는다
cd "$REPO"
python3 - "$ONLY" > "$LOG_DIR/resources.txt" <<'PY'
import json, sys
from ugv.config import RESOURCES
spawn = json.load(open("ugv/data/road_network.json", encoding="utf-8"))["spawn"]
only = sys.argv[1]
for r in RESOURCES:
    if only and r["resource_id"] != only:
        continue
    s = spawn[r["resource_id"]]
    print(r["resource_id"], r["px4_instance"], r["px4_model"], s["x"], s["y"], s["z"], s["yaw"])
PY
[ -s "$LOG_DIR/resources.txt" ] || { echo "자원 없음: ${ONLY:-전체}"; exit 1; }

while read -r RID INST MODEL X Y Z YAW; do
    for m in "$MODEL" lawnmower r1_rover; do          # 지정 모델이 이 PX4 버전에 없으면 대체
        AF=$(airframe_of "$m" || true)
        if [ -n "$AF" ] && [ -f "$PX4_GZ_MODELS/$m/model.sdf" ]; then MODEL="$m"; break; fi
        AF=""
    done
    [ -n "$AF" ] || { echo "$RID: 쓸 수 있는 rover 모델이 없다"; exit 1; }

    WD="$BUILD/rootfs/$INST"                        # PX4 Tools/simulation 의 다중 인스턴스 스크립트와 같은 위치
    mkdir -p "$WD"
    echo "$RID  인스턴스 $INST  모델 $MODEL(에어프레임 $AF)  스폰 ($X, $Y, $Z)  포트 $((14540 + INST))"
    ( cd "$WD" && \
      PX4_SYS_AUTOSTART="$AF" PX4_SIM_MODEL="gz_$MODEL" PX4_GZ_MODEL_POSE="$X,$Y,$Z,0,0,$YAW" \
      exec "$BUILD/bin/px4" -i "$INST" -d "$BUILD/etc" > "$LOG_DIR/px4-$INST.log" 2>&1 ) &
    echo "$! $INST" >> "$LOG_DIR/pids"

    # 3) Mac(UGV 서버)으로 가는 MAVLink 링크 추가. PX4 가 부팅을 마칠 때까지 재시도
    if [ -n "$UGV_HOST" ]; then
        ok=0
        for _ in $(seq 60); do
            if "$BUILD/bin/px4-mavlink" --instance "$INST" start -x -u $((14590 + INST)) -r 4000000 -f \
                   -m onboard -o $((14540 + INST)) -t "$UGV_HOST" >> "$LOG_DIR/px4-$INST.log" 2>&1; then
                ok=1; break
            fi
            sleep 1
        done
        [ "$ok" = 1 ] && echo "  MAVLink → $UGV_HOST:$((14540 + INST))" \
                      || echo "  MAVLink 링크 추가 실패 — ugv/tools/mavlink_relay.py 사용 ($LOG_DIR/px4-$INST.log)"
    fi
done < "$LOG_DIR/resources.txt"

echo
echo "로그: $LOG_DIR   Mac 쪽 서버:"
echo "  UGV_DRIVER=px4 UGV_TIME_SCALE=$SPEED UGV_PX4_RESOURCES=$(cut -d' ' -f1 "$LOG_DIR/resources.txt" | paste -sd,) \\"
echo "    python3 -m uvicorn ugv.server:app --host 0.0.0.0 --port 8100"
