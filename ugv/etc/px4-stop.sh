#!/bin/bash
# UGV PX4 중지·삭제
#   ./px4-stop.sh    UGV 컨테이너만 내린다
#
# ⚠️ 여기서 pkill 을 쓰지 않는다.
#    구버전 ugv/tools/px4-stop.sh 는 `pkill -9 -f "gz sim"` 으로 호스트 전체의
#    gz sim 을 죽였다. UGV 가 호스트 네이티브로 돌던 때는 문제가 없었지만,
#    Docker 로 옮긴 지금은 그 한 줄이 UAV 의 Gazebo 까지 같이 죽인다.
#    컨테이너 이름으로만 지정한다.
#
# 파라미터 초기화(오염 복구)는 컨테이너를 지우면 끝난다 — 이미지가 매번
# 새 rootfs 로 뜨므로 parameters.bson 이 남지 않는다.

cd "$(dirname "$0")"

docker compose --profile all down --remove-orphans 2>/dev/null

# compose 가 모르는 유령 컨테이너까지 정리 (이름 명시)
for n in px4-ugv0 px4-ugv1; do
  docker rm -f "$n" >/dev/null 2>&1 && echo "$n 삭제"
done

echo "--- 남은 PX4 컨테이너 ---"
docker ps --filter name=px4 --format '{{.Names}} | {{.Status}}' || true
