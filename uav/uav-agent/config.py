"""임계값 설정. 전부 잠정값 — 팀 합의 후 확정할 것 (NOTE.md 6-2)."""

# 풍속
WIND_LIMIT_MS = 10.0        # 이상이면 REJECT / HIGH_WIND
WIND_COUNTER_MS = 8.0       # 이상이면 COUNTER 검토

# 배터리
SAFETY_MARGIN_PCT = 20.0    # 복귀 후 남아야 할 최소 배터리
BATTERY_DRAIN_PCT_S = 0.033  # 초당 소모율 (100% / 약 50분) — **실측 보정 필요**
                             # 0.05 은 x500 실제 항속(~30분)보다 과도해 기본 시나리오가
                             # REJECT 로 떨어졌음. 아래는 추정치이며 측정값이 아님.

# 비행
CRUISE_SPEED_MS = 15.0
CLIMB_SPEED_MS = 3.0
OBSERVE_DURATION_S = 60     # 관측 체류시간
MIN_CLEARANCE_M = 50        # 지형 위 최소 여유고도

# 센서
AVAILABLE_SENSORS = ["THERMAL", "RGB"]
