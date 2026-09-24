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

GUI=0; WORLD=kangwon; FIRE_OPTS=()
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
  RUN_OPTS=(--restart unless-stopped -e HEADLESS=1)
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
  px4io/px4-sitl-gazebo:latest

echo "기동 대기..."
sleep 40   # 지형 로딩 때문에 기본 월드(30초)보다 길게 둔다

# GCS(QGC) 없이 시동을 걸기 위한 안전장치 해제. ⚠️ 시뮬레이션 전용 — 실기체 금지.
PX4_BIN=/opt/px4-gazebo/bin/px4-param
docker exec px4 $PX4_BIN set NAV_RCL_ACT 0      # RC 끊겨도 failsafe 동작 안 함
docker exec px4 $PX4_BIN set NAV_DLL_ACT 0      # datalink(GCS) 끊겨도 failsafe 동작 안 함
docker exec px4 $PX4_BIN set COM_RCL_EXCEPT 4   # RC 상실 예외 처리 — offboard 모드 허용

docker ps --format '{{.Names}} | {{.Status}}'
