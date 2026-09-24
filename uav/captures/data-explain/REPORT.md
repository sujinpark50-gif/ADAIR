# UAV 데이터 캡처 — 원본(raw) vs 가공본

PX4 가 내보내는 원본 데이터 전체와, UAV Agent 가 이를 가공해 팀에 제공하는 응답을 같은 시점에 캡처했다.

| 항목 | 값 |
|---|---|
| 캡처 시각 (UTC) | 2026-09-24T08:03:25Z |
| 원본 캡처 (position) | 2026-09-24T08:03:26.101Z |
| 가공본 캡처 (/state) | 2026-09-24T08:03:38.241Z |
| MAVSDK 연결 | `udpin://0.0.0.0:14550` (PX4 GCS 링크) |
| UAV Agent | `http://localhost:8000` — mode `real` |
| PX4 버전 | 1.18.0 BETA (git 61907962c) |

파일: `raw_mavsdk.json` (원본 전체), `raw_px4_uorb.txt` (PX4 내부값·파라미터), `processed_agent.json` (가공본)

## 1. 원본 → 가공 매핑 (`GET /state`)

UAV Agent 가 원본 중 무엇을 어떻게 바꿔 `/state` 로 내보내는지 보여준다.

| 가공 필드 | 의미 | 가공 값 | 원본 (MAVSDK) | 원본 값 | 처리 |
|---|---|---|---|---|---|
| `position.lat` | 위도 (°, WGS84) | 38.121499899999996 | `position.latitude_deg` | 38.1215 | 그대로 |
| `position.lon` | 경도 (°, WGS84) | 128.2029997 | `position.longitude_deg` | 128.20299989999998 | 그대로 |
| `position.alt_m_amsl` | 해발고도 (m) | 280.9930114746094 | `position.absolute_altitude_m` | 281.00201416015625 | 그대로 |
| `position.alt_m_agl` | 이륙 지점 기준 상대고도 (m). 지면 기준 아님 | 50.038002014160156 | `position.relative_altitude_m` | 50.048004150390625 | 그대로 (이륙지점 기준 상대고도, 실제 AGL 아님) |
| `battery.percent` | 배터리 잔량 (%, 0~100) | 50.0 | `battery.remaining_percent` | 50.0 | 그대로 (MAVSDK 가 이미 0~100) |
| `battery.voltage` | 배터리 전압 (V) | 15.30000114440918 | `battery.voltage_v` | 15.30000114440918 | 그대로 |
| `velocity_ms` | 수평 속도 (m/s) | 0.0 | `velocity_ned.north_m_s / east_m_s` | 0.0, 0.0 | hypot(N, E) 소수 2자리 — 수직속도 버림 |
| `flight_mode` | PX4 비행 모드 이름 | "HOLD" | `flight_mode` | "HOLD" | enum 이름 문자열 |
| `armed` | 시동 여부 | true | `armed` | true | bool |
| `health.gps_ok` | GPS 기반 위치 추정 가능 | true | `health.is_global_position_ok` | true | 그대로 |
| `health.sensors_ok` | 자이로·가속도계·지자기계 보정 모두 완료 | true | `health.is_gyrometer/accelerometer/magnetometer_calibration_ok` | True, True, True | 세 값 AND |
| `health.failsafe` | PX4 failsafe 작동 중 (real 에서는 항상 false, 고정값) | false | `(없음)` | — | 고정 false |
| `wind_ms` | 풍속 (m/s). real 에서는 항상 null (PX4 미발행) | null | `wind (미발행)` | — | 고정 null |
| `link_quality` | 통신 상태 (real 에서는 항상 "OK", 고정값) | "OK" | `(없음)` | — | 고정 "OK" |
| `current_task_id` | 수행 중인 Task (현재 항상 null, 미구현) | null | `(없음)` | — | 고정 null |
| `timestamp` | UAV 서버가 응답을 만든 시각 (ISO8601 UTC). PX4 측정 시각 아님 | "2026-09-24T08:03:34Z" | `(없음 — 서버 시각)` | — | 응답 생성 시각 ISO8601 UTC |

## 2. 배터리 단위 대조 (R05)

| 단계 | 값 | 척도 |
|---|---|---|
| PX4 uORB `battery_status.remaining` | 0.50003 | 0~1 |
| MAVSDK `battery.remaining_percent` | 50.0 | 0~100 |
| UAV API `battery.percent` | 50.0 | 0~100 |

MAVSDK 가 ×100 해서 넘기므로 UAV·통합 모두 추가 변환이 필요 없다.

## 3. 가공본 — `POST /evaluate`

요청: `{"task_id": "capture-001", "decision_id": "capture-dec-001", "target": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 761.0}, "observation_type": "THERMAL", "wind_ms": 3.1}`

| 응답 필드 | 의미 | 값 |
|---|---|---|
| `task_id` | 요청의 Task ID (그대로 반환) | "capture-001" |
| `decision_id` | 요청의 판단 ID (그대로 반환) | "capture-dec-001" |
| `uav_id` | 판단한 UAV | "A-uav1" |
| `verdict` | ACCEPT(수행 가능) / REJECT(불가) / COUNTER(조건부 가능) | "REJECT" |
| `eta_sec` | 목표 상공 도착 예상 시간 (s). 상승 시간 포함. REJECT 는 null | null |
| `reason` | REJECT·COUNTER 사유 코드 (interfaces/schema.py RejectReason) | "LOW_BATTERY" |
| `detail` | 사람이 읽는 설명 (일부 REJECT 만) | "need 57.0% but have 50.0%" |
| `counter_offer` | COUNTER 일 때 제안 조건 | null |
| `constraints.distance_km` | 현재 위치 → 목표 수평 거리 (km) | 9.86 |
| `constraints.battery_now_pct` | 현재 배터리 (%) | 50.0 |
| `constraints.battery_after_pct` | 왕복 + 관측 후 예상 배터리 (%) | -7.0 |
| `constraints.return_margin_pct` | 복귀 후 남는 배터리 (%). 20 미만이면 COUNTER | -7.0 |
| `constraints.wind_ms` | 판단에 쓴 풍속 (m/s, 요청값) | 3.1 |
| `constraints.required_climb_m` | 목표 비행고도(목표 지면 + 50 m)까지 상승량 (m) | 530 |
| `constraints.eta_sec` | ETA (s). REJECT 여도 계산된 경우 여기 남음 | 834 |

## 4. 원본 전체 필드 사전 — MAVSDK Telemetry

스트림 33종 중 **22종 수신**. ✅ 는 UAV Agent 가 사용하는 스트림이다.

### `altitude` — 여러 기준의 고도

| 필드 | 의미 | 값 |
|---|---|---|
| `altitude_amsl_m` | 해발고도 (m) | 281.0104675292969 |
| `altitude_local_m` | 로컬 원점(EKF 원점) 기준 고도 (m) | 50.12434387207031 |
| `altitude_monotonic_m` | 추정기 리셋 시에도 튀지 않도록 보정한 연속 고도 (m) | 281.5786437988281 |
| `altitude_relative_m` | 홈(이륙 지점) 기준 상대고도 (m) | 50.056243896484375 |
| `altitude_terrain_m` | 지면 기준 고도 (m). 거리센서가 없으면 NaN | "nan" |
| `bottom_clearance_m` | 기체 아래 지면·장애물까지 거리 (m). 거리센서가 없으면 NaN | "nan" |
| `timestamp_us` | PX4 부팅 후 경과 시간 (µs) | 4669360000 |

### `armed` ✅ — 시동 여부 (true = 모터 회전 가능)

값: `true`

### `attitude_angular_velocity_body` — 기체 좌표계 각속도

| 필드 | 의미 | 값 |
|---|---|---|
| `pitch_rad_s` | 피치 축 회전 속도 (rad/s) | 0.0012650967109948397 |
| `roll_rad_s` | 롤 축 회전 속도 (rad/s) | 0.00838270504027605 |
| `yaw_rad_s` | 요 축 회전 속도 (rad/s) | 0.0007266038446687162 |

### `attitude_euler` — 자세 (오일러각)

| 필드 | 의미 | 값 |
|---|---|---|
| `pitch_deg` | 앞뒤 기울기 (°, 기수 위가 +) | -0.3952074348926544 |
| `roll_deg` | 좌우 기울기 (°, 오른쪽 날개 아래가 +) | -0.2906687557697296 |
| `timestamp_us` | PX4 부팅 후 경과 시간 (µs) | 4669380000 |
| `yaw_deg` | 기수 방향 (°, 북 기준 시계방향) | -0.01887333393096924 |

### `attitude_quaternion` — 자세 (쿼터니언). attitude_euler 와 같은 정보의 다른 표현

| 필드 | 의미 | 값 |
|---|---|---|
| `timestamp_us` | PX4 부팅 후 경과 시간 (µs) | 4669400000 |
| `w` | 쿼터니언 스칼라 성분 | 0.9999910593032837 |
| `x` | 쿼터니언 x 성분 | -0.0024653340224176645 |
| `y` | 쿼터니언 y 성분 | -0.0034447996877133846 |
| `z` | 쿼터니언 z 성분 | -0.0001721866283332929 |

### `battery` ✅ — 배터리 상태

| 필드 | 의미 | 값 |
|---|---|---|
| `battery_function` | 배터리 용도 (ALL = 전체 계통 전원) | "ALL" |
| `capacity_consumed_ah` | 소모한 용량 (Ah). 용량 설정(BAT1_CAPACITY)이 없으면 0 | 0.0 |
| `current_battery_a` | 현재 전류 (A). 전류 측정이 없으면 NaN | "nan" |
| `id` | 배터리 번호 | 0 |
| `remaining_percent` | 잔량 (%, 0~100). PX4 내부 0~1 값을 MAVSDK 가 ×100 | 50.0 |
| `temperature_degc` | 배터리 온도 (°C). 측정 없으면 NaN | "nan" |
| `time_remaining_s` | 예상 잔여 시간 (s). 추정 불가 시 NaN | "nan" |
| `voltage_v` | 전압 (V) | 15.30000114440918 |

### `fixedwing_metrics` — 비행 계기값 (MAVLink VFR_HUD). 이름과 달리 멀티콥터도 발행

| 필드 | 의미 | 값 |
|---|---|---|
| `absolute_altitude_m` | 해발고도 (m) | 280.99322509765625 |
| `airspeed_m_s` | 대기속도 (m/s). 대기속도 센서가 없으면 NaN | "nan" |
| `climb_rate_m_s` | 상승률 (m/s, 위가 +) | 0.006864469964057207 |
| `groundspeed_m_s` | 대지속도 (m/s) | 0.015125281177461147 |
| `heading_deg` | 기수 방향 (°, 0~360) | 359.0 |
| `throttle_percentage` | 스로틀 출력 (MAVSDK 명칭은 %, 값 척도 미확인) | 0.7199999690055847 |

### `flight_mode` ✅ — PX4 비행 모드 (HOLD, TAKEOFF, MISSION, RETURN_TO_LAUNCH, LAND 등)

값: `"HOLD"`

### `gps_info` — GPS 수신 상태 요약

| 필드 | 의미 | 값 |
|---|---|---|
| `fix_type` | 측위 종류 (NO_FIX, FIX_2D, FIX_3D, RTK_FLOAT, RTK_FIXED 등) | "FIX_3D" |
| `num_satellites` | 사용 중인 위성 수 | 10 |

### `heading` — 기수 방향

| 필드 | 의미 | 값 |
|---|---|---|
| `heading_deg` | 기수 방향 (°, 북 기준 시계방향) | 0.0 |

### `health` ✅ — 비행 전 점검 항목

| 필드 | 의미 | 값 |
|---|---|---|
| `is_accelerometer_calibration_ok` | 가속도계 보정 완료 | true |
| `is_armable` | 지금 시동을 걸 수 있는 상태 | true |
| `is_global_position_ok` | GPS 기반 전역 위치 추정 가능 | true |
| `is_gyrometer_calibration_ok` | 자이로 보정 완료 | true |
| `is_home_position_ok` | 홈 위치 설정됨 | true |
| `is_local_position_ok` | 로컬 위치 추정 가능 | true |
| `is_magnetometer_calibration_ok` | 지자기계 보정 완료 | true |

### `health_all_ok` — health 전 항목이 모두 true 인지

값: `true`

### `home` — 홈(이륙·복귀) 위치

| 필드 | 의미 | 값 |
|---|---|---|
| `absolute_altitude_m` | 해발고도 (m) | 230.95401000976562 |
| `approach_down_m` | 착륙 접근 방향 벡터 하 성분 (m) | 0.0 |
| `approach_east_m` | 착륙 접근 방향 벡터 동 성분 (m) | 0.0 |
| `approach_north_m` | 착륙 접근 방향 벡터 북 성분 (m) | 0.0 |
| `latitude_deg` | 위도 (°) | 38.1204933 |
| `local_down_m` | 로컬 원점 기준 아래쪽 거리 (m, 위가 −) | -0.0680999755859375 |
| `local_east_m` | 로컬 원점 기준 동쪽 거리 (m) | 0.9362789988517761 |
| `local_north_m` | 로컬 원점 기준 북쪽 거리 (m) | -0.295594185590744 |
| `longitude_deg` | 경도 (°) | 128.2018085 |
| `q.timestamp_us` | 홈 위치에서의 기체 방향 (쿼터니언) | 0 |
| `q.w` | 홈 위치에서의 기체 방향 (쿼터니언) | 0.46027636528015137 |
| `q.x` | 홈 위치에서의 기체 방향 (쿼터니언) | 0.0033701681531965733 |
| `q.y` | 홈 위치에서의 기체 방향 (쿼터니언) | -0.009738176129758358 |
| `q.z` | 홈 위치에서의 기체 방향 (쿼터니언) | 0.8877158761024475 |
| `relative_altitude_m` | 홈 기준 상대고도 (항상 0) | 0.0 |
| `timestamp_us` | 홈이 설정된 시각 (PX4 부팅 후 µs) | 79488000 |

### `in_air` — 비행 중 여부 (true = 공중)

값: `true`

### `landed_state` — 착륙 상태 (ON_GROUND, TAKING_OFF, IN_AIR, LANDING, UNKNOWN)

값: `"IN_AIR"`

### `position` ✅ — 전역 위치 (EKF 추정)

| 필드 | 의미 | 값 |
|---|---|---|
| `absolute_altitude_m` | 해발고도 (m) | 281.00201416015625 |
| `latitude_deg` | 위도 (°) | 38.1215 |
| `longitude_deg` | 경도 (°) | 128.20299989999998 |
| `relative_altitude_m` | 홈(이륙 지점) 기준 상대고도 (m). 지면 기준 AGL 아님 | 50.048004150390625 |

### `position_velocity_ned` — 로컬 원점 기준 위치·속도 (NED: 북·동·아래)

| 필드 | 의미 | 값 |
|---|---|---|
| `position.down_m` | 아래쪽 거리 (m, 위가 −) | -50.11894226074219 |
| `position.east_m` | 동쪽 거리 (m) | 105.15902709960938 |
| `position.north_m` | 북쪽 거리 (m) | 111.64556884765625 |
| `velocity.down_m_s` | 하강 속도 (m/s, 상승이 −) | 0.005685833282768726 |
| `velocity.east_m_s` | 동쪽 속도 (m/s) | -0.015294735319912434 |
| `velocity.north_m_s` | 북쪽 속도 (m/s) | 0.0030483563896268606 |

### `raw_gps` — GPS 수신기 원시값

| 필드 | 의미 | 값 |
|---|---|---|
| `absolute_altitude_m` | 해발고도 (m) | 280.9690246582031 |
| `altitude_ellipsoid_m` | WGS84 타원체 기준 고도 (m) | 280.9690246582031 |
| `cog_deg` | 진행 방향 (Course Over Ground, °). 정지 중에는 의미 없음 | 271.7099914550781 |
| `hdop` | 수평 정밀도 저하율 (작을수록 좋음, 1 이하 양호) | 0.699999988079071 |
| `heading_uncertainty_deg` | 방향 오차 추정 (°) | 0.0 |
| `horizontal_uncertainty_m` | 수평 위치 오차 추정 (m) | 0.9000000357627869 |
| `latitude_deg` | 위도 (°) | 38.1215001 |
| `longitude_deg` | 경도 (°) | 128.2030002 |
| `timestamp_us` | PX4 부팅 후 경과 시간 (µs) | 4685540000 |
| `vdop` | 수직 정밀도 저하율 (작을수록 좋음) | 1.100000023841858 |
| `velocity_m_s` | 대지속도 (m/s) | 0.03999999910593033 |
| `velocity_uncertainty_m_s` | 속도 오차 추정 (m/s) | 0.4000000059604645 |
| `vertical_uncertainty_m` | 수직 위치 오차 추정 (m) | 1.78000009059906 |
| `yaw_deg` | GPS 기반 방향 (°). 듀얼 안테나가 없으면 0 | 0.0 |

### `rc_status` — RC 조종기 연결 상태

| 필드 | 의미 | 값 |
|---|---|---|
| `is_available` | 현재 조종기 연결됨 | false |
| `signal_strength_percent` | 신호 세기 (%). 연결 없으면 NaN | "nan" |
| `was_available_once` | 한 번이라도 연결된 적 있음 | false |

### `scaled_pressure` — 기압계

| 필드 | 의미 | 값 |
|---|---|---|
| `absolute_pressure_hpa` | 대기압 (hPa). 기압고도 계산에 사용 | 1002.0852661132812 |
| `differential_pressure_hpa` | 차압 (hPa). 대기속도 센서용, 멀티콥터는 0 | 0.0 |
| `differential_pressure_temperature_deg` | 차압 센서 온도 (°C) | 0.0 |
| `temperature_deg` | 기압계 온도 (°C) | 15.0 |
| `timestamp_us` | PX4 부팅 후 경과 시간 (µs) | 4693280000 |

### `velocity_ned` ✅ — 속도 (NED)

| 필드 | 의미 | 값 |
|---|---|---|
| `down_m_s` | 하강 속도 (m/s, 상승이 −) | 0.0 |
| `east_m_s` | 동쪽 속도 (m/s) | 0.0 |
| `north_m_s` | 북쪽 속도 (m/s) | 0.0 |

### `vtol_state` — VTOL 전환 상태. 멀티콥터(x500)는 UNDEFINED

값: `"UNDEFINED"`

### 이 링크에서 수신되지 않은 스트림

| 스트림 | 의미 | 상태 |
|---|---|---|
| `actuator_control_target` | 제어기가 액추에이터에 내리는 목표값 (정규화된 제어 입력) | timeout |
| `actuator_output_status` | 실제 모터·서보 출력값 (PWM 등) | timeout |
| `distance_sensor` | 하방 거리센서 (라이다·초음파) 측정값. 기체에 센서 없음 | timeout |
| `ground_truth` | 시뮬레이터 참값 위치 (실기체에는 없음) | timeout |
| `imu` | IMU 측정값 (가속도·각속도·자기장·온도) | timeout |
| `odometry` | 위치·속도·자세 통합 추정값 | timeout |
| `raw_imu` | IMU 원시 측정값 | timeout |
| `scaled_imu` | 단위 변환된 IMU 측정값 | timeout |
| `status_text` | PX4 경고·안내 메시지. 발생할 때만 전송 | timeout |
| `unix_epoch_time` | PX4 시스템 시각 (Unix epoch µs) | timeout |
| `wind` | 풍속·풍향 추정값 | timeout |

`timeout` 은 캡처 시간(스트림당 수 초) 안에 값이 오지 않았다는 뜻이다. PX4 SITL 이 이 링크(GCS)에
해당 메시지를 보내지 않거나(`imu`, `odometry` 등), 센서가 없거나(`distance_sensor`),
이벤트가 있을 때만 보내는(`status_text`) 경우다. `wind` 는 Agent 링크(14540)에서도 미발행이었다(NOTE.md 6-4 실측).

### 기체 정보 (MAVSDK Info)

| 항목 | 의미 | 값 |
|---|---|---|
| `version` | PX4 펌웨어·OS 버전 (flight_sw_* = PX4, os_sw_* = 호스트 OS) | {"flight_sw_git_hash": "61907962c", "flight_sw_major": 1,… |
| `identification` | 기체 고유 ID (SITL 은 0) | {"hardware_uid": "000000000000000000000000000000000000", … |
| `product` | 제조사·제품 정보 (SITL 은 undefined) | {"product_id": 0, "product_name": "undefined", "vendor_id… |
| `speed_factor` | 시뮬레이션 속도 배율 (1.0 = 실시간) | 1.000227746856821 |
| `gps_global_origin` | EKF 로컬 원점의 위경도·고도. 로컬 NED 좌표의 기준점 | {"altitude_m": 230.88601684570312, "latitude_deg": 38.120… |

## 5. ⚠️ 알려진 이슈 — SITL 배터리가 50% 에서 멈춤

**현상**

- 캡처 시점 배터리: uORB `0.50003` → API `50.0%`
- PX4 배터리 시뮬레이터 파라미터: `SIM_BAT_DRAIN = 60.0000` (초), `SIM_BAT_MIN_PCT = 50.0000` (%)
- 시동 후 비행하면 배터리가 줄어들다가 **`SIM_BAT_MIN_PCT` 에서 멈추고 더 내려가지 않는다.**
  한 번 비행한 기체는 PX4 를 재기동할 때까지 이 값에 머문다.

**영향**

- 이번 캡처의 evaluate 가 바로 이 경우다: `REJECT / LOW_BATTERY` — need 57.0% but have 50.0%.
- 원통119 → 관측 목표(약 9.8 km, 왕복 + 관측)에는 약 57% 가 필요하다고 계산되므로,
  배터리가 50% 에 걸린 기체는 **이 정도 거리 이상의 목표를 계속 `LOW_BATTERY` 로 거절한다.**
  복귀 여유 20% 까지 따지면 50% 기체가 ACCEPT 할 수 있는 목표는 매우 가까운 곳뿐이다.
- 반대로 방전이 50% 에서 멈추므로, 실제 비행이 길어져도 배터리 부족 상황이 재현되지 않는다.
- 즉 real 모드의 배터리 값은 **실제 소모를 반영하지 않으며**, 판단 근거로 쓰기에 부적절하다.

**대응 (결정 필요)**

1. 시연 직전에 PX4 를 재기동해 100% 에서 시작 (`px4-stop.sh` → `px4-start*.sh`)
2. 기동 스크립트에서 `SIM_BAT_MIN_PCT` 를 낮춰 실제에 가까운 방전 허용 (예: 5)
3. `SIM_BAT_DRAIN` 을 조정해 방전 속도를 기체 사양에 맞춤 — 배터리 소모율(`BATTERY_DRAIN_PCT_S`)
   실측 보정과 함께 진행
