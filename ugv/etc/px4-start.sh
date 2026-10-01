#!/bin/bash
# UGV PX4 SITL + Gazebo (headless) 기동
#   ./px4-start.sh          A 거점만 (기본 — 권장)
#   ./px4-start.sh --all    A + B 두 대
#
# UGV 1대 = PX4 1개 + Gazebo 1개. 구성은 docker-compose.yml 을 보라.
# 안 띄운 자원은 ugv/fleet.py 가 자동으로 SimDriver 로 돌린다(코드 수정 불필요).
# 종료: ./px4-stop.sh
set -e

cd "$(dirname "$0")"

PROFILE=a
[ "$1" = "--all" ] && PROFILE=all

docker compose --profile "$PROFILE" up -d

# 기동 대기 — 고정 sleep 을 쓰지 않는다.
# 부팅이 덜 끝난 상태에서 px4-param 을 때리면 MAVLink 가 소켓만 열고 죽는다
# (UAV px4-start-kangwon.sh 에서 확인된 것과 같은 함정).
# 컨테이너 / PX4 인스턴스 번호 / MAVLink 포트(=14540+i) 는 한 줄로 짝이다.
NAMES=(px4-ugv0); INSTS=(1); PORTS=(14541)
if [ "$PROFILE" = "all" ]; then NAMES+=(px4-ugv1); INSTS+=(2); PORTS+=(14542); fi

for k in "${!NAMES[@]}"; do
  n=${NAMES[$k]}; inst=${INSTS[$k]}
  echo "기동 대기: $n"
  for i in $(seq 1 90); do
    # ⚠️ 인스턴스 지정은 `--instance N` 이다. `-i N` 은 px4 데몬 기동 플래그일 뿐
    #    CLI 툴에서는 무시되어 `PX4 server not running` 이 난다 (실측 확인).
    if timeout 5 docker exec "$n" /opt/px4-gazebo/bin/px4-commander \
         --instance "$inst" status >/dev/null 2>&1; then
      echo "  기동 완료 (${i}초)"; break
    fi
    sleep 1
    [ "$i" = 90 ] && { echo "  ⚠️ 90초 내 기동 실패 — docker logs $n 확인"; exit 1; }
  done
done

# GCS 없이 시동을 걸기 위한 안전장치 해제. ⚠️ 시뮬레이션 전용 — 실기체 금지.
# ⚠️ SYS_HAS_MAG / FD_FAIL_* 는 건드리지 않는다. SYS_HAS_MAG=0 이면 yaw_align 이
#    서지 않아 arm 이 거부된다 (ugv/README.md '환경 설정 함정').
for k in "${!NAMES[@]}"; do
  n=${NAMES[$k]}; inst=${INSTS[$k]}
  P="/opt/px4-gazebo/bin/px4-param --instance $inst"
  docker exec "$n" $P set NAV_RCL_ACT 0      >/dev/null
  docker exec "$n" $P set NAV_DLL_ACT 0      >/dev/null
  docker exec "$n" $P set COM_RCL_EXCEPT 4   >/dev/null
done
echo "안전장치 해제 완료"

# MAVLink 가 실제로 오는지 확인한다. Agent 는 udpin://0.0.0.0:<port> 로 받는다.
echo "MAVLink 확인..."
for p in "${PORTS[@]}"; do
  python3 - "$p" <<'EOF' || echo "  ⚠️ 패킷 없음 — ./px4-stop.sh 후 재기동 필요"
import socket, sys
port = int(sys.argv[1])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", port)); s.settimeout(15)
try:
    s.recvfrom(2048); print(f"  → {port} 수신 확인")
except socket.timeout:
    sys.exit(1)
EOF
done

# ⚠️ 기동 직후 60초간은 EKF 가 수렴하지 않는다 (GPS Horizontal Pos Drift 경고).
#    그 전에 arm 하면 위치 추정이 발산한 상태로 주행한다 (ugv/README.md).
echo "※ arm 전에 EKF 수렴까지 약 60초 기다릴 것"

docker ps --filter name=px4-ugv --format '{{.Names}} | {{.Status}}'
