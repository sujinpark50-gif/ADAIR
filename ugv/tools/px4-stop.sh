#!/usr/bin/env bash
# PX4/Gazebo 프로세스 및 저장 상태 정리.
# --params 를 붙이면 저장된 파라미터·미션도 지운다(오염 복구용).

pkill -9 -f "bin/px4"  2>/dev/null || true
pkill -9 -f "gz sim"   2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null || true

if [ "$1" = "--params" ]; then
    BUILD="${PX4_DIR:-$HOME/PX4-Autopilot}/build/px4_sitl_default"
    rm -f "$BUILD"/rootfs/fs/parameters*.bson "$BUILD"/rootfs/fs/dataman 2>/dev/null || true
    rm -f "$BUILD"/instance_*/fs/parameters*.bson "$BUILD"/instance_*/fs/dataman 2>/dev/null || true
    echo "파라미터·미션 초기화 완료"
fi

sleep 1
if ps aux | grep -E "bin/px4|gz sim" | grep -qv grep; then
    echo "남은 프로세스:"; ps aux | grep -E "bin/px4|gz sim" | grep -v grep
else
    echo "정리 완료"
fi