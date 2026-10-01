"""임계값 설정. 전부 잠정값 — 팀 합의 후 확정할 것 (NOTE.md 6-2)."""

# 풍속
WIND_LIMIT_MS = 10.0        # 이상이면 REJECT / HIGH_WIND
WIND_COUNTER_MS = 8.0       # 이상이면 COUNTER 검토

# 배터리
BATTERY_RESERVE_PCT = 15.0  # 복귀 후 남아야 할 최소 배터리 (PX4 저전압 경고 BAT_LOW_THR 기준)
BATTERY_DRAIN_PCT_S = 0.033  # 초당 소모율 (100% / 약 50분) — **실측 보정 필요**
                             # 0.05 은 x500 실제 항속(~30분)보다 과도해 기본 시나리오가
                             # REJECT 로 떨어졌음. 아래는 추정치이며 측정값이 아님.

# 비행
CRUISE_SPEED_MS = 10.0     # PX4 MPC_XY_CRUISE 에도 같은 값을 설정한다 (drone.Px4Drone.goto)
CLIMB_SPEED_MS = 3.0
OBSERVE_DURATION_S = 60     # 관측 체류시간
DEFAULT_TARGET_AGL_M = 80   # 요청에 target_agl_m 이 없을 때 쓰는 목표 높이 (시나리오 기본)
AGL_MARGIN_M = 10           # target_agl_m 위에 더하는 여유고도

# 기지 (시나리오: A·B 소방서에 UAV 2대씩. UAV_ID 앞글자로 소속을 정한다 — "A-uav1" → A)
# (lat, lon, 지면 해발고도 m). Mock 의 출발·복귀 위치다. real 은 PX4 home 을 쓴다.
HOME_BASES = {
    "A": (38.1205, 128.2018, 235.0),   # 원통119 — Gazebo 강원 월드 스폰 위치와 같음
    "B": (37.9645, 128.3062, 279.0),   # 기린119 — 팀 DEM 지면고도
}

# 실행
ARRIVAL_TOLERANCE_M = 10.0  # 목표 수평거리 이내면 도착으로 본다
MONITOR_INTERVAL_S = 1.0    # 비행 중 배터리 복귀 여유 확인 주기 (실제 초)
MOCK_TIME_SCALE = 100.0     # Mock 비행 배속. 1200 s 비행이 12 s 에 끝난다
                            # (통합 커넥터가 Task 완료를 30 s 까지 기다린다)

# 센서
AVAILABLE_SENSORS = ["THERMAL", "RGB"]
