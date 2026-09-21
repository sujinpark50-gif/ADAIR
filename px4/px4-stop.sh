#!/bin/bash
# PX4 중지 및 삭제 (로그 파일까지 정리)
docker rm -f px4 2>/dev/null && echo "px4 중지·삭제 완료" || echo "(실행 중인 px4 없음)"
df -h / | tail -1
