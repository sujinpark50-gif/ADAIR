#!/usr/bin/env bash
# ugv/tools/px4-start.sh 가 띄운 것(Gazebo 서버·GUI, PX4 인스턴스 1~3, relay)만 끈다.
# --params 를 붙이면 UGV 인스턴스(1~3)의 저장된 파라미터·미션도 지운다(오염 복구용).
#
# pkill -f "gz sim" 처럼 이름으로 훑지 않는다 — 같은 기계의 UAV(인스턴스 0, Docker 포함) Gazebo·PX4 까지
# 죽인다 (ugv/etc/px4-stop.sh 와 같은 이유). 기동 때 기록한 PID 만 끈다.

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-$REPO/ugv/.state/px4}"   # px4-start.sh 와 같게
PIDS="$LOG_DIR/pids"
INSTANCES="1 2 3"

if [ -f "$PIDS" ]; then
    while read -r pid inst; do
        kill "$pid" 2>/dev/null && echo "종료: PID $pid ${inst:+(PX4 인스턴스 $inst)}"
    done < "$PIDS"
    sleep 2
    while read -r pid _; do kill -9 "$pid" 2>/dev/null; done < "$PIDS"
    rm -f "$PIDS"
else
    echo "기록된 PID 없음 ($PIDS) — 이 스크립트로 띄운 프로세스가 없다"
fi
pkill -f "ugv/tools/mavlink_relay.py" 2>/dev/null || true
for i in $INSTANCES; do rm -f "/tmp/px4_lock-$i" "/tmp/px4-sock-$i" 2>/dev/null; done

if [ "$1" = "--params" ]; then
    BUILD="${PX4_DIR:-$HOME/PX4-Autopilot}/build/px4_sitl_default"
    for i in $INSTANCES; do
        rm -f "$BUILD/rootfs/$i"/parameters*.bson "$BUILD/rootfs/$i"/dataman \
              "$BUILD/rootfs/$i"/fs/parameters*.bson "$BUILD/rootfs/$i"/fs/dataman "$BUILD/rootfs/$i"/.ugv_airframe \
              "$BUILD/instance_$i"/fs/parameters*.bson "$BUILD/instance_$i"/fs/dataman 2>/dev/null
    done
    echo "UGV 인스턴스 $INSTANCES 파라미터·미션 초기화 완료"
fi
