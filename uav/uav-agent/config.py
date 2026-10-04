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
OBSERVE_DURATION_S = 60     # 관측 체류시간 기본값 (요청에 observe_duration_s 가 없을 때)
OBSERVE_DURATION_MIN_S = 10  # 요청·COUNTER 로 줄일 수 있는 최소 체류 (UAV-02)
OBSERVE_DURATION_MAX_S = 600
DEFAULT_TARGET_AGL_M = 80   # 요청에 target_agl_m 이 없을 때 쓰는 목표 높이 (시나리오 기본)
AGL_MARGIN_M = 10           # target_agl_m 위에 더하는 여유고도

# 기지 (시나리오: A·B 소방서에 UAV 2대씩. UAV_ID 앞글자로 소속을 정한다 — "A-uav1" → A)
# (lat, lon, 지면 해발고도 m). Mock 의 출발·복귀 위치다. real 은 PX4 home 을 쓴다.
HOME_BASES = {
    "A": (38.1205, 128.2018, 235.0),   # 원통119 — Gazebo 강원 월드 스폰 위치와 같음
    "B": (37.9645, 128.3062, 279.0),   # 기린119 — 팀 DEM 지면고도
}

# 전진 이착륙 (mock 전용). UAV_HOME="lat,lon,지면해발" 을 주면 소속 기지 대신 그 지점에서 뜨고 내린다.
# 소방차에 싣고 현장 근처로 가서 띄우는 운용을 흉내 낸다 (2019 인제 트윈 시나리오 B). real 은 PX4 home 을 쓴다.
import os as _os
if _os.getenv("UAV_HOME"):
    _lat, _lon, _alt = (float(v) for v in _os.environ["UAV_HOME"].split(","))
    HOME_BASES = {k: (_lat, _lon, _alt) for k in HOME_BASES}

# 실행
ARRIVAL_TOLERANCE_M = 10.0  # 목표 수평거리 이내면 도착으로 본다
MONITOR_INTERVAL_S = 1.0    # 비행 중 배터리 복귀 여유 확인 주기 (실제 초)
MOCK_TIME_SCALE = 100.0     # Mock 비행 배속. 1200 s 비행이 12 s 에 끝난다
                            # (통합 커넥터가 Task 완료를 30 s 까지 기다린다)

# 센서. WEATHER 는 mock 에서만 모의값으로 지원한다 (실기체에는 기상 센서가 없다 — UAV-05·07)
AVAILABLE_SENSORS = ["THERMAL", "RGB"]
MOCK_EXTRA_SENSORS = ["WEATHER"]


def available_sensors() -> list:
    mode = _os.getenv("UAV_MODE", "mock").lower()
    return AVAILABLE_SENSORS + ([] if mode == "real" else MOCK_EXTRA_SENSORS)


# 열화상 자리표시 값 (UAV-05). 실제 열화상 처리가 없으므로 측정값이 아니다.
#   mock: 기존 연동(connectors/fire_connector)이 hotspot_detected 로 화재를 표시하므로 남기되 simulated 로 표시한다
#   real: 기본은 값을 비운다 (만든 값을 실측처럼 보내지 않는다). UAV_PLACEHOLDER_THERMAL=1 이면 mock 과 같이 채운다
MOCK_THERMAL_VALUES = {"max_temp_c": 312.5, "hotspot_detected": True}


def placeholder_thermal() -> bool:
    if _os.getenv("UAV_PLACEHOLDER_THERMAL") is not None:
        return _os.getenv("UAV_PLACEHOLDER_THERMAL") == "1"
    return _os.getenv("UAV_MODE", "mock").lower() != "real"


# 탑재 카메라 (UAV-08). 시뮬레이션 기체는 PX4 Gazebo 기본 x500 (uav/etc/px4-start-kangwon.sh 가 모델을
# 지정하지 않음) 이라 카메라가 없다. THERMAL·RGB 관측값은 그래서 모의값이거나 비어 있다 (UAV-05).
# 카메라 달린 모델(x500_mono_cam 등)로 바꾸면 여기 시야각·해상도를 채운다 — 총괄이 프레임 지상 범위를 다시 계산한다.
CAMERA = {"model": None, "dfov_deg": None, "resolution_px": None, "status": "NONE",
          "airframe": "PX4 Gazebo x500 (기본, 카메라 없음)"}

# 복귀 중 재배정 (UAV-06). 기본 0 = 착륙(phase=DONE)까지 새 임무를 BUSY 로 거절한다 — 총괄 방침
# (착륙·READY 확인 전까지 점유 유지, 재배정 안 함)과 같다.
# 1 이면 복귀 중인 기체가 새 임무를 받는다(복귀를 멈추고 출발, 기존 Task phase=RETASKED. 예전 시나리오 4·7 재배치용).
RETASK_WHILE_RETURNING = _os.getenv("UAV_RETASK_WHILE_RETURNING", "0") == "1"

# 경로 (UAV-03). 출발→목표, 목표→기지 직선 경로를 DEM 으로 샘플해 순항고도를 정한다.
ROUTE_VERSION = "R1-STRAIGHT-DEM-CRUISE"
ROUTE_USE_DEM = _os.getenv("UAV_ROUTE_DEM", "1") == "1"
ROUTE_SAMPLE_M = 45.0              # 경로 샘플 간격 (DEM 격자 90 m 의 절반)
ROUTE_TERRAIN_CLEARANCE_M = 50.0   # 경로 최고 지형 위 여유 (ridge_avoid.py 와 같은 값)
ROUTE_START_TOLERANCE_M = 50.0     # 평가 때 출발점과 실행 때 위치가 이보다 멀면 경로를 다시 평가해야 한다
ROUTE_KEEP = 200                   # 저장해 둘 최근 평가 경로 수
# 1 이면 실행 요청에 route_hash 가 반드시 있어야 한다. 기본 0 — 해시 없는 기존 호출(웹·커넥터)은 실행 때 경로를 새로 잡는다
ROUTE_REQUIRED = _os.getenv("UAV_REQUIRE_ROUTE_HASH", "0") == "1"
