#!/bin/bash
# PX4 SITL + Gazebo (headless) 기동
# 로그 크기를 제한한다 — 안 걸면 pxh> 프롬프트가 무한 누적되어 디스크를 채운다(35GB 사례 있음).

docker rm -f px4 2>/dev/null   # 유령 컨테이너 레코드 정리

# 이거 참고로 강원도 지역에서 드론 리스폰되도록 해놨음.
docker run -d --name px4 \
  --network host \
  --restart unless-stopped \
  --log-opt max-size=50m \
  --log-opt max-file=2 \
  -e HEADLESS=1 \
  -e PX4_HOME_LAT=38.1205 \
  -e PX4_HOME_LON=128.2018 \
  -e PX4_HOME_ALT=235 \
  px4io/px4-sitl-gazebo:latest

echo "기동 대기..."
sleep 30

# GCS(QGC) 없이 시동을 걸기 위한 안전장치 해제. 안 걸면 arm() 이 거부된다
# (`Preflight Fail: No connection to the GCS`). ⚠️ 시뮬레이션 전용 — 실기체 금지.
# 부팅 후에만 적용 가능해 sleep 뒤에 둔다. 재기동 시 기본값으로 돌아가므로 매번 실행한다.
PX4_BIN=/opt/px4-gazebo/bin/px4-param
docker exec px4 $PX4_BIN set NAV_RCL_ACT 0      # RC 끊겨도 failsafe 동작 안 함
docker exec px4 $PX4_BIN set NAV_DLL_ACT 0      # datalink(GCS) 끊겨도 failsafe 동작 안 함
docker exec px4 $PX4_BIN set COM_RCL_EXCEPT 4   # RC 상실 예외 처리 — offboard 모드 허용

docker ps --format '{{.Names}} | {{.Status}}'
