#!/bin/bash
# PX4 SITL + Gazebo — 강원(원통-기린) 지형 월드
#   ./px4-start-kangwon.sh          headless (EC2)
#   ./px4-start-kangwon.sh --gui    로컬 WSL2 + WSLg GUI
#   ./px4-start-kangwon.sh --gui --fire   산불(파티클 불꽃·연기) 시연 월드
# 월드 재생성: python3 gazebo/build_world.py  (GDAL 필요)
# 산불 월드 재생성: python3 gazebo/add_fire.py
# 월드 원점 = gz_bridge.py datum (38.0112, 128.122643, 172.1m). PX4_HOME_* 은 월드 원점을
# 덮어쓰므로 쓰지 않고, 기체 위치는 PX4_GZ_MODEL_POSE(원통119)로 지정한다.
# 종료: ./px4-stop.sh

W="$(cd "$(dirname "$0")/.." && pwd)/gazebo"   # uav/etc/ → uav/gazebo
source "$W/spawn.env"

GUI=0; WORLD=kangwon; FIRE_OPTS=(); CMD_OPTS=()
for arg in "$@"; do
  case "$arg" in
    --gui) GUI=1 ;;
    --fire) WORLD=kangwon_fire
            # 파티클 시스템은 월드가 아닌 server.config 로 켠다 (add_fire.py 주석 참고)
            FIRE_OPTS=(-v "$W/server_fire.config":/opt/px4-gazebo/share/gz/server.config:ro) ;;
  esac
done

if [ "$GUI" = 1 ]; then
  RUN_OPTS=(-it
    -e DISPLAY="$DISPLAY"
    -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY"
    -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
    -e LIBGL_ALWAYS_SOFTWARE=1
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -v /mnt/wslg:/mnt/wslg
    -v "$W/gui.config":/root/.gz/sim/8/gui.config:ro)
else
  # `-d` = daemon mode. 대화형 pxh 셸을 띄우지 않는다.
  # 없으면 PX4 가 `pxh> ` 프롬프트를 초당 ~830 KB 쏟아내고, 그 전량이
  # dockerd/containerd-shim 을 거치며 CPU 1 코어를 통째로 태운다 (실측 110% → 12%).
  # 로그 크기 제한(--log-opt)은 디스크만 지킬 뿐 이 CPU 낭비는 못 막는다.
  RUN_OPTS=(--restart unless-stopped -e HEADLESS=1)
  CMD_OPTS=(-d)
fi

docker rm -f px4 2>/dev/null   # 유령 컨테이너 레코드 정리

# 로그 크기를 제한한다 — 안 걸면 pxh> 프롬프트가 무한 누적되어 디스크를 채운다(35GB 사례 있음).
docker run -d --name px4 \
  --network host \
  --log-opt max-size=50m \
  --log-opt max-file=2 \
  "${RUN_OPTS[@]}" \
  "${FIRE_OPTS[@]}" \
  -v "$W/$WORLD.sdf":/opt/px4-gazebo/share/gz/worlds/$WORLD.sdf:ro \
  -v "$W/models/kangwon":/opt/px4-gazebo/share/gz/models/kangwon:ro \
  -e PX4_GZ_WORLD=$WORLD \
  -e PX4_GZ_MODEL_POSE="$PX4_GZ_MODEL_POSE" \
  px4io/px4-sitl-gazebo:latest \
  "${CMD_OPTS[@]}"

# 기동 대기 — 고정 sleep 을 쓰지 않는다.
# 강원 지형 월드는 heightmap 로딩이 무거워 2 vCPU 에서 40초를 넘기는 경우가 있고,
# 부팅이 덜 끝난 상태에서 px4-param 을 때리면 MAVLink 가 소켓만 열고 죽는다
# (px4-mavlink status 가 빈 출력, 14540 으로 패킷 0개). 실제로 뜰 때까지 기다린다.
echo "기동 대기..."
for i in $(seq 1 90); do
  # offboard 링크(14580)가 열리고 PX4 셸이 응답하면 부팅 완료로 본다.
  if ss -uln 2>/dev/null | grep -q ':14580' \
     && timeout 5 docker exec px4 /opt/px4-gazebo/bin/px4-commander status >/dev/null 2>&1; then
    echo "PX4 기동 완료 (${i}초)"
    break
  fi
  sleep 1
  [ "$i" = 90 ] && { echo "⚠️  90초 내 기동 실패 — docker logs px4 확인"; exit 1; }
done

# GCS(QGC) 없이 시동을 걸기 위한 안전장치 해제. ⚠️ 시뮬레이션 전용 — 실기체 금지.
PX4_BIN=/opt/px4-gazebo/bin/px4-param
docker exec px4 $PX4_BIN set NAV_RCL_ACT 0      # RC 끊겨도 failsafe 동작 안 함
docker exec px4 $PX4_BIN set NAV_DLL_ACT 0      # datalink(GCS) 끊겨도 failsafe 동작 안 함
docker exec px4 $PX4_BIN set COM_RCL_EXCEPT 4   # RC 상실 예외 처리 — offboard 모드 허용

# MAVLink 가 실제로 14540 으로 송신하는지 확인한다. Agent 는 udpin://0.0.0.0:14540 으로 받는다.
echo "MAVLink 확인..."
python3 - <<'EOF' || echo "⚠️  14540 으로 패킷이 오지 않음 — ./px4-stop.sh 후 재기동 필요"
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 14540)); s.settimeout(15)
try:
    s.recvfrom(2048); print("  → 14540 수신 확인")
except socket.timeout:
    sys.exit(1)
EOF

docker ps --format '{{.Names}} | {{.Status}}'
