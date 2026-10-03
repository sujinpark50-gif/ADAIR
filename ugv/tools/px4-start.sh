#!/usr/bin/env bash
# UGV 3대: Gazebo 월드 1개(kangwon_ugv = UAV 와 같은 강원 지형 + 도로 면) + PX4 SITL 인스턴스 3개.
# Gazebo/PX4 를 돌리는 기계(현재 Windows WSL)에서 저장소 루트 기준으로 실행한다.
#
#   UGV_HOST=192.168.0.23 ./ugv/tools/px4-start.sh           # 3대 모두 (UGV_HOST = UGV 서버가 도는 Mac 의 IP)
#   UGV_HOST=192.168.0.23 ./ugv/tools/px4-start.sh A-ugv1    # 한 대만
#   GUI=1 SPEED=5 UGV_HOST=... ./ugv/tools/px4-start.sh      # Gazebo 창 + 5배속
#   ./ugv/tools/px4-stop.sh                                  # 전부 종료
#   WORLD=kangwon_ugv2 ...                                   # 다른 월드 (ugv/gazebo/<이름>.sdf). 기본은 road_network.json world
#   STEP_MS=8 ...                                            # 물리 스텝 (기본 4 ms). 실험용 — 아래 '배속' 참고
#   TERRAIN_COLLISION=0 ...                                  # 지형 충돌 끄기 (배속 원인 찾기용 — 차는 도로 면에만 닿는다)
#   FIRE_TRUCK=0 ...                                         # 소방차(A-fire1)를 경광등·방수포 없이 기반 모델로
#
# 구성 (값의 정본은 ugv/config.py RESOURCES 와 ugv/data/road_network.json spawn — 이 스크립트는 읽기만 한다)
#   자원      PX4 인스턴스  MAVLink 포트  모델
#   A-ugv1    1             14541         r1_rover
#   B-ugv1    2             14542         r1_rover
#   A-fire1   3             14543         r1_rover + 경광등·방수포 (fire_truck, FIRE_TRUCK=0 이면 r1_rover 그대로)
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
# 배속: SPEED 는 '목표 상한'이다. Gazebo 는 lockstep(매 스텝 PX4 를 기다림)이라 CPU 가 버티는 만큼만 빨라진다
#       (PX4 문서: 데스크톱 6~10배, 노트북 3~4배 — 빈 월드·기체 1대 기준). 실제 배속은 시작 끝에 찍는
#       /world/<이름>/stats 의 real_time_factor, 또는 주행 기록(ugv/tools/analyze_drive_log.py)으로 본다.
#       STEP_MS=8 은 같은 시뮬 1초에 스텝 수를 절반으로 줄인다. 대신 IMU 가 125 Hz 로 떨어진다 (PX4 경고 확인).
#
# 확인된 주의사항 (이전 단독 월드 구성에서)
#   SYS_HAS_MAG / FD_FAIL_* 는 건드리지 말 것 — yaw_align 이 서지 않아 arm 이 거부된다.
#   파라미터는 build/px4_sitl_default/rootfs/N/ 아래에 남는다. 오염되면 px4-stop.sh --params.
#   PX4 는 HEADLESS 변수가 "있기만 하면" GUI 를 끈다 → GUI 는 이 스크립트가 따로 띄운다.

set -eo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD="$PX4_DIR/build/px4_sitl_default"
# 월드는 ugv/data/road_network.json 의 world 를 따른다 (기본: ugv/tools/build_road_world.py 가 만든
# ugv/gazebo/kangwon_ugv.sdf — 강원 지형 + 도로 면). 값이 없으면 UAV 월드(uav/gazebo/kangwon.sdf).
if [ -n "${WORLD:-}" ]; then
    WORLD_REL="ugv/gazebo/$WORLD.sdf"; WORLD_NAME="$WORLD"
else
    read -r WORLD_REL WORLD_NAME < <(python3 -c "import json;w=json.load(open('$REPO/ugv/data/road_network.json',encoding='utf-8')).get('world',{});print(w.get('sdf','uav/gazebo/kangwon.sdf'),w.get('name','kangwon'))")
fi
WORLD_SDF="$REPO/$WORLD_REL"
STEP_MS="${STEP_MS:-}"
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
export GZ_SIM_RESOURCE_PATH="$LOG_DIR/models:$REPO/ugv/gazebo/models:$REPO/uav/gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"   # fire_truck, model://kangwon_ugv, model://kangwon
# 월드 사본 (항상): 텍스처 자리표시(@UAV_TEX@ — 산불 파티클) → 절대 경로, 물리 스텝(STEP_MS),
# 지형 충돌 끄기(TERRAIN_COLLISION=0). model:// 경로라 사본 위치가 달라도 된다
NOTERRAIN=""
if [ "${TERRAIN_COLLISION:-1}" = "0" ]; then NOTERRAIN="-noterrain"; fi
# (주의) set -e 에서 OUT="$( [ ... ] && echo ...)" 는 조건이 거짓일 때 스크립트가 조용히 끝난다 — if 로 쓴다
OUT="$LOG_DIR/$WORLD_NAME${STEP_MS:+-step${STEP_MS}ms}$NOTERRAIN.sdf"
python3 - "$WORLD_SDF" "$OUT" "${STEP_MS:-}" "${TERRAIN_COLLISION:-1}" "$REPO/uav/gazebo/models/kangwon/materials/textures" <<'PY'
import re, sys
src, out, step_ms, terrain, uav_tex = sys.argv[1:]
w = open(src, encoding="utf-8").read().replace("@UAV_TEX@", uav_tex)
if step_ms:
    w = re.sub(r"<max_step_size>[^<]*</max_step_size>", f"<max_step_size>{float(step_ms)/1000}</max_step_size>", w)
    w = re.sub(r"<real_time_update_rate>[^<]*</real_time_update_rate>",
               f"<real_time_update_rate>{round(1000/float(step_ms))}</real_time_update_rate>", w)
if terrain == "0":   # 지형 heightmap 충돌 제거 — 차는 도로 면(메시)에만 닿는다. 도로 밖으로 나가면 떨어진다 (실험용)
    w, n = re.subn(r'\s*<model name="kangwon_terrain_collision">.*?</model>', "", w, count=1, flags=re.S)
    assert n == 1, "kangwon_terrain_collision 모델을 못 찾음"
open(out, "w", encoding="utf-8").write(w)
PY
WORLD_SDF="$OUT"
echo "월드 사본: $WORLD_SDF"
STEP_S=$(python3 -c "print(${STEP_MS:-4}/1000)")

# Gazebo 서버 시스템 목록 = PX4 기본(server.config) + ParticleEmitter (산불·경광등·물줄기 파티클).
# 월드에 <plugin> 을 적으면 Gazebo 가 기본 목록을 안 읽어 물리·센서가 빠지므로 목록 파일을 바꿔 준다 (UAV 와 같은 방식)
SRC_CFG="${GZ_SIM_SERVER_CONFIG_PATH:-${PX4_GZ_SERVER_CONFIG:-}}"
if [ -f "$SRC_CFG" ]; then
    python3 - "$SRC_CFG" "$LOG_DIR/server.config" <<'PY'
import sys
src, out = sys.argv[1:]
c = open(src, encoding="utf-8").read()
if "particle-emitter" not in c:
    c = c.replace("</plugins>", '  <plugin entity_name="*" entity_type="world" filename="gz-sim-particle-emitter-system" '
                  'name="gz::sim::systems::ParticleEmitter"/>\n  </plugins>', 1)
open(out, "w", encoding="utf-8").write(c)
PY
    export GZ_SIM_SERVER_CONFIG_PATH="$LOG_DIR/server.config"
else
    echo "PX4 server.config 를 못 찾음 — 파티클(산불·경광등·물줄기)이 안 보일 수 있다"
fi
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
    echo "Gazebo 월드 $WORLD_NAME 이미 실행 중 (파티션 $GZ_PARTITION) — WORLD·STEP_MS 를 바꿨다면 px4-stop.sh 먼저"
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
    print(r["resource_id"], r["px4_instance"], r["px4_model"], s["x"], s["y"], s["z"], s["yaw"], r["resource_type"])
PY
[ -s "$LOG_DIR/resources.txt" ] || { echo "자원 없음: ${ONLY:-전체}"; exit 1; }

while read -r RID INST MODEL X Y Z YAW RTYPE; do
    # 이미 떠 있는 인스턴스는 다시 띄우지 않는다. 같은 -i 를 두 번 띄우면 잠금·포트(1454i)를 두고 싸우고
    # 차량 이름(<모델>_<i>)이 겹쳐 스폰이 실패해, 달리던 차의 텔레메트리까지 끊긴다.
    if pgrep -f "bin/px4 -i $INST " > /dev/null; then
        echo "$RID  인스턴스 $INST 이미 실행 중 — 건너뜀 (다시 띄우려면 ./ugv/tools/px4-stop.sh 먼저)"
        continue
    fi
    for m in "$MODEL" lawnmower r1_rover; do          # 지정 모델이 이 PX4 버전에 없으면 대체
        AF=$(airframe_of "$m" || true)
        if [ -n "$AF" ] && [ -f "$PX4_GZ_MODELS/$m/model.sdf" ]; then MODEL="$m"; break; fi
        AF=""
    done
    [ -n "$AF" ] || { echo "$RID: 쓸 수 있는 rover 모델이 없다"; exit 1; }

    # 소방차: 기반 모델 + 경광등·방수포 (ugv/gazebo/fire_truck/model.sdf.in). FIRE_TRUCK=0 이면 기반 모델 그대로
    SPAWN="$MODEL"; MODELS_DIR="$PX4_GZ_MODELS"
    if [ "$RTYPE" = "FIRE_ENGINE" ] && [ "${FIRE_TRUCK:-1}" = "1" ]; then
        BASE_LINK=$(grep -o "<link name=['\"][^'\"]*" "$PX4_GZ_MODELS/$MODEL/model.sdf" | head -1 | sed "s/<link name=['\"]//" || true)
        mkdir -p "$LOG_DIR/models/fire_truck"
        sed -e "s|@BASE@|$MODEL|g" -e "s|@BASE_LINK@|${BASE_LINK:-base_link}|g" -e "s|@RID@|$RID|g" \
            -e "s|@FX@|$REPO/ugv/gazebo/fire_truck|g" "$REPO/ugv/gazebo/fire_truck/model.sdf.in" \
            > "$LOG_DIR/models/fire_truck/model.sdf"
        printf '<?xml version="1.0"?>\n<model><name>fire_truck</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>\n' \
            > "$LOG_DIR/models/fire_truck/model.config"
        SPAWN="fire_truck"; MODELS_DIR="$LOG_DIR/models"
        echo "  소방차 모델: $MODEL + 경광등·방수포 (차체 링크 ${BASE_LINK:-base_link})"
    fi

    WD="$BUILD/rootfs/$INST"                        # PX4 Tools/simulation 의 다중 인스턴스 스크립트와 같은 위치
    mkdir -p "$WD"
    echo "$RID  인스턴스 $INST  모델 $SPAWN(에어프레임 $AF)  스폰 ($X, $Y, $Z)  포트 $((14540 + INST))"
    ( cd "$WD" && \
      PX4_SYS_AUTOSTART="$AF" PX4_SIM_MODEL="gz_$SPAWN" PX4_GZ_MODELS="$MODELS_DIR" PX4_GZ_MODEL_POSE="$X,$Y,$Z,0,0,$YAW" \
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

# 4) 물리 설정 확인 (PX4 가 배속을 걸면서 스텝을 덮어쓸 수 있어, STEP_MS 를 줬으면 한 번 더 건다)
if [ -n "$STEP_MS" ]; then
    gz service -s "/world/$WORLD_NAME/set_physics" --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean \
        --timeout 3000 --req "max_step_size: $STEP_S, real_time_factor: $SPEED" > /dev/null 2>&1 || true
fi
sleep 3
STATS=$(timeout 5 gz topic -e -t "/world/$WORLD_NAME/stats" -n 1 2>/dev/null | tr '\n' ' ' || true)
echo "물리: $(echo "$STATS" | grep -o 'step_size {[^}]*}' | head -1)  real_time_factor: $(echo "$STATS" | grep -o 'real_time_factor: [0-9.e-]*' | head -1 | cut -d' ' -f2)  (목표 SPEED=$SPEED)"

echo
echo "로그: $LOG_DIR   Mac 쪽 서버:"
echo "  UGV_DRIVER=px4 UGV_TIME_SCALE=$SPEED UGV_PX4_RESOURCES=$(cut -d' ' -f1 "$LOG_DIR/resources.txt" | paste -sd,) \\"
echo "    python3 -m uvicorn ugv.server:app --host 0.0.0.0 --port 8100"
