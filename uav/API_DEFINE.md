# UAV Local Agent API 정의서

| 항목 | 값 |
|---|---|
| 버전 | 0.2.0 (2026-10-04, 총괄 팀별 요청서 UAV-01~10 반영) |
| Base URL | 서버 `http://3.37.210.229:8000` (평소 꺼져 있음, 필요 시 UAV 담당에게 요청) / 로컬 `http://localhost:8000` |
| 형식 | JSON, UTF-8 |
| 단위 | 거리·고도 m, 속도·풍속 m/s, 시간 s, 배터리 %(0~100), 좌표 WGS84 |
| 코드 | `uav/uav-agent/main.py`, `models.py`, `evaluator.py`, `route.py` |

0.1.0 과의 호환: 필드를 **추가만** 했습니다. 기존 필드의 이름·값·의미는 그대로입니다
(`status` 네 값, `progress.phase`, `observation.sensor_type`·`values`). 바뀐 동작은 [0.1.0 대비 변경](#010-대비-변경)에 모았습니다.

## 엔드포인트 요약

| 메서드 | 경로 | 용도 |
|---|---|---|
| GET | `/health` | 서버 모드·담당 UAV 확인 |
| GET | `/uav/{uav_id}/state` | 상태 조회 |
| GET | `/uav/{uav_id}/capabilities` | 지원 기능 (센서·측정·체류 범위·중단·경로·카메라) |
| POST | `/uav/{uav_id}/evaluate` | 수행 가능성 판단 (**실행하지 않음**). ACCEPT 면 경로 ID·해시 |
| POST | `/uav/{uav_id}/execute` | 실행 시작 (**Safety ALLOW 이후에만 호출**) |
| GET | `/uav/{uav_id}/task/{task_id}` | 실행 진행·결과 조회 |
| POST | `/uav/{uav_id}/task/{task_id}/abort` | 중단 요청 |
| POST | `/mock/set` | 테스트용 상태 주입 (mock 전용) |

호출 순서: `state` → `evaluate` → (총괄·Safety 판단) → `execute`(평가의 `route_id`·`route_hash` 포함) → `task` 폴링 → (필요 시) `abort`

---

## GET /health

```json
{"status": "ok", "mode": "mock", "uav_id": "A-uav1",
 "exec_store": {"path": ".../tasks_A-uav1.json", "restarts": 0, "closed_after_restart": [], "tasks": 0},
 "routes": 0}
```

## GET /uav/{uav_id}/state

**응답 200**

| 필드 | 타입 | 설명 |
|---|---|---|
| `uav_id` | string | 예: `A-uav1` |
| `timestamp` | string | 응답 생성 시각 (ISO8601 UTC). PX4 텔레메트리 시각이 아님 |
| `position.lat` / `position.lon` | float | WGS84 |
| `position.alt_m_amsl` | float | 해발고도 |
| `position.alt_m_agl` | float | 이륙 지점 기준 상대고도 (실제 지면 기준 아님) |
| `battery.percent` | float | 0~100 |
| `battery.voltage` | float | V |
| `velocity_ms` | float | 수평 속도 |
| `flight_mode` | string | 예: `HOLD`, `TAKEOFF` |
| `armed` | bool | 시동 여부 |
| `health.gps_ok` | bool | 전역 위치 확보 여부 |
| `health.sensors_ok` | bool | 자이로·가속도계·지자기계 캘리브레이션 AND |
| `health.failsafe` | bool | ⚠️ real에서 항상 `false` (고정값) |
| `wind_ms` | float \| null | ⚠️ real에서 항상 `null` (PX4 SITL 이 wind telemetry 를 발행하지 않음, 실측) |
| `link_quality` | string | ⚠️ real에서 항상 `"OK"` (고정값) |
| `current_task_id` | string \| null | 이동·관측 중인 Task id. 대기·복귀 중에는 `null` |
| `returning_task_id` | string \| null | 관측을 마치고(또는 끊겨) 기지로 복귀 중인 Task id |

⚠️ 표시 필드는 가용성 판단 근거로 쓰지 마세요.

## GET /uav/{uav_id}/capabilities

```json
{"uav_id":"A-uav1","mode":"mock","sensors":["THERMAL","RGB","WEATHER"],
 "measurements_per_visit":true,
 "observe_duration_s":{"default":60,"min":10,"max":600},
 "counter_fields":["observe_duration_s"],
 "deadline_field":"remaining_time_s",
 "abort":true,"retask_while_returning":true,
 "route":{"version":"R1-STRAIGHT-DEM-CRUISE","dem":true,"required_on_execute":false},
 "observation_values":{"THERMAL":"MOCK_FIXED_PLACEHOLDER","WEATHER":"MOCK_PROVIDER_VALUE","RGB":"NO_SENSOR_PIPELINE"},
 "camera":{"model":null,"dfov_deg":null,"resolution_px":null,"status":"NONE","airframe":"PX4 Gazebo x500 (기본, 카메라 없음)"},
 "patrol":false}
```

- `sensors`: real 모드에는 `WEATHER` 가 없습니다 (기체에 기상 센서 없음).
- `camera`: 시뮬레이션 기체가 PX4 Gazebo 기본 x500 이라 **카메라가 없습니다** (`NONE`). 그래서 THERMAL·RGB 관측값은 실측이 아닙니다.

## POST /uav/{uav_id}/evaluate

판단만 하며 기체를 움직이지 않습니다.

**요청**

| 필드 | 타입 | 필수 | 설명 |
|---|---|---|---|
| `task_id` | string | ✅ | |
| `decision_id` | string | ✅ | 재평가마다 새 값 |
| `target.lat` / `target.lon` | float | ✅ | 관측 목표 |
| `target.alt_m_amsl` | float | ✅ | 목표 지점 **지면** 해발고도(AMSL). 없으면 `REJECT/TARGET_ALTITUDE_UNKNOWN` |
| `target.target_agl_m` | float | | 지면 위 목표 높이(AGL). 없으면 80 |
| `observation_type` | string | | `THERMAL` / `RGB` / `WEATHER`(mock). **없으면 이동 가능성만 평가** |
| `measurements` | string[] | | 한 방문에서 할 측정 목록. 예) `["THERMAL","WEATHER"]`. `observation_type` 과 합칩니다 |
| `observe_duration_s` | int | | 관측 체류시간 10~600 s. 없으면 60. COUNTER 가 제안한 값을 그대로 싣습니다 |
| `remaining_time_s` | float | | 기한까지 남은 **시뮬레이션** 시간(s). 실제 시계와 비교하지 않습니다 |
| `wind_ms` | float | | 환경모델 풍속. **real에서 없으면 `REJECT/WIND_UNKNOWN`** (mock은 내부값 3.1 사용) |

관측고도(AMSL) = `alt_m_amsl` + `target_agl_m` + 10 m(여유). 예: 804 + 80 + 10 = 894 m.
순항고도(AMSL) = max(관측고도, 경로 최고 지형 + 50 m). 경로는 출발→목표, 목표→기지 직선이며 DEM 으로 샘플합니다 ([경로](#경로-uav-03)).

```json
{"task_id":"task-101","decision_id":"dec-001",
 "target":{"lat":38.0425,"lon":128.2541,"alt_m_amsl":804,"target_agl_m":80},
 "measurements":["THERMAL","WEATHER"],"observe_duration_s":60,"remaining_time_s":1800,"wind_ms":3.1}
```

**응답 200**

| 필드 | 타입 | 설명 |
|---|---|---|
| `task_id`, `decision_id`, `uav_id` | string | 요청값 그대로 |
| `verdict` | string | `ACCEPT` / `REJECT` / `COUNTER` |
| `eta_sec` | int \| null | ACCEPT·COUNTER는 정수, REJECT는 `null` |
| `reason` | string \| null | 아래 표 |
| `detail` | string \| null | 사람이 읽는 설명 |
| `counter_offer` | object \| null | COUNTER일 때. **재평가 요청에 그대로 실을 수 있는 필드만** (`observe_duration_s`) |
| `constraints` | object | 판단 근거 (`distance_km`, `battery_now_pct`, `battery_after_pct`, `return_margin_pct`, `wind_ms`, `flight_alt_m_amsl`, `cruise_alt_m_amsl`, `terrain_check`, `required_climb_m`, `eta_sec`, `observe_duration_s`, `measurements`, 해당 시 `deadline_slack_sec`·`max_observe_duration_s`·`warnings`) |
| `route_id`, `route_version`, `route_hash` | string \| null | ACCEPT일 때만. 실행 요청에 그대로 싣습니다 |
| `route` | object \| null | ACCEPT일 때만. 순항·관측고도, 경유점, 지형 확인 결과 |

**판정 순서와 reason 코드** (위에서부터 검사하고 처음 걸린 것을 반환)

| 순서 | verdict | reason | 조건 |
|---|---|---|---|
| 1 | REJECT | `WIND_UNKNOWN` | 풍속 없음 |
| 2 | REJECT | `TARGET_ALTITUDE_UNKNOWN` | `target.alt_m_amsl` 없음 |
| 3 | REJECT | `FAILSAFE_ACTIVE` | failsafe 작동 |
| 4 | REJECT | `SENSOR_FAILURE` | GPS 또는 센서 이상 |
| 5 | REJECT | `COMMUNICATION_FAILURE` | `link_quality == "LOST"` |
| 6 | REJECT | `BUSY` | 다른 Task 이동·관측 중, 또는 복귀 중 (착륙 전. `UAV_RETASK_WHILE_RETURNING=1` 이면 복귀 중은 허용) |
| 7 | REJECT | `REQUIRED_CAPABILITY_UNAVAILABLE` | 요청한 측정을 지원하지 않음 |
| 8 | REJECT | `HIGH_WIND` | 풍속 ≥ 10 m/s |
| 9 | COUNTER | `RETURN_MARGIN_INSUFFICIENT` | 복귀 후 배터리 < 15% 이지만 체류를 줄이면 가능 → `{"observe_duration_s": n}` |
| 10 | REJECT | `LOW_BATTERY` | 체류를 줄여도 안 되고, 복귀 후 배터리 < 0 |
| 11 | REJECT | `RETURN_MARGIN_INSUFFICIENT` | 최소 체류(10 s)로도 복귀 여유 15% 를 못 지킴 |
| 12 | REJECT | `TIMEOUT` | `remaining_time_s` ≤ 0 |
| 13 | REJECT | `DEADLINE_TIGHT` | ETA > `remaining_time_s` |
| — | ACCEPT | — | 모두 통과. 8 ≤ 풍속 < 10 m/s 이면 `constraints.warnings=["MODERATE_WIND"]` |

COUNTER 는 이제 9번 하나뿐입니다. 제안값 `n` 을 `observe_duration_s` 로 실어 다시 평가하면 ACCEPT 가 나옵니다(상태가 같다면).
임계값(풍속 8/10 m/s, 복귀 여유 15%)은 모두 프로젝트 잠정값입니다(`uav-agent/config.py`).

## POST /uav/{uav_id}/execute

Safety ALLOW 이후에만 호출합니다. 비동기로 시작하고 즉시 반환합니다.

**요청**

| 필드 | 필수 | 설명 |
|---|---|---|
| `task_id` | ✅ | **실행 키**. 총괄은 실행시도 ID 를 넣습니다 |
| `decision_id`, `target` | ✅ | evaluate 와 같은 형식 |
| `observation_type`, `measurements`, `observe_duration_s` | | evaluate 와 같은 값이어야 경로가 맞습니다 |
| `execution_attempt_id` | | 주면 `task_id` 와 같아야 합니다 (다르면 422) |
| `route_id`, `route_hash` | | 평가 응답의 값. 주면 그 경로로만 실행합니다 |

**실행 키 규칙 (UAV-01)**: 같은 키·같은 내용 재요청 → 새로 출동하지 않고 기존 실행을 돌려줍니다(`duplicate: true`).
같은 키·다른 내용 → 409 `EXECUTION_ID_CONFLICT` (기존 실행 유지). 기록은 파일(`UAV_STATE_DIR`)에 남아 재시작 뒤에도 같습니다.
거절(400·409·422)은 실행하지 않았음이 확실한 경우만이며 키를 쓰지 않습니다 — 고쳐서 같은 키로 다시 보낼 수 있습니다.

**경로 규칙 (UAV-03)**

| 상황 | 결과 |
|---|---|
| `route_id`·`route_hash` 없음 | 실행 시점에 경로를 새로 잡습니다 (`UAV_REQUIRE_ROUTE_HASH=1` 이면 409 `ROUTE_REQUIRED`) |
| 평가 기록에 없는 경로 | 409 `ROUTE_NOT_FOUND` |
| 목표·체류·측정이 평가와 다름 | 409 `ROUTE_MISMATCH` |
| 평가 뒤 기체가 50 m 넘게 움직임 | 409 `ROUTE_STALE` (다시 평가) |

**응답 200**

```json
{"task_id":"ATT-1","status":"STARTED","tracking_url":"/uav/A-uav1/task/ATT-1","uav_id":"A-uav1",
 "duplicate":false,"route_id":"RT-5c6bd4aa6ee90084","route_hash":"5c6bd4aa..."}
```

## GET /uav/{uav_id}/task/{task_id}

| 필드 | 설명 |
|---|---|
| `task_id`, `uav_id` | 실행 키, 기체 |
| `status` | `STARTED` / `IN_PROGRESS` / `COMPLETED` / `FAILED` |
| `mission_result` | `PENDING` / `OBSERVED` / `NOT_OBSERVED` — **관측(도착+체류)을 마쳤는지** |
| `physical_state` | `PREPARING` → `ENROUTE` → `OBSERVING` → `RETURNING` → `LANDED`. 그 밖에 `RETURN_FAILED`, `RETASKED`, `UNKNOWN` — **기체가 물리적으로 어디에 있는지** |
| `physical_failure_reason` | `RETURN_FAILED` / `FLIGHT_ERROR` / `AGENT_RESTARTED` / null |
| `progress.phase` | 0.1.0 과 같음: `ENROUTE` → `OBSERVING` → `RETURNING` → `DONE`(= LANDED), `RETASKED`, `RETURN_FAILED`, `AGENT_RESTARTED` |
| `observation` | 관측을 마쳤을 때만. 아래 |
| `reason` | FAILED 사유 코드: `RETURN_MARGIN_INSUFFICIENT`, `ABORTED`, `AGENT_RESTARTED_BEFORE_OBSERVATION` |
| `error` | FAILED·복귀 실패 상세 |
| `abort` | 중단 요청을 받았으면 `{requested_at, reason, result}` |
| `route_id`, `route_hash` | 이 실행이 난 경로 |
| `agent_restart` | 재시작으로 끊긴 실행이면 근거 |

**관측 결과와 물리 상태 구분 (UAV-10)**

| status | mission_result | observation |
|---|---|---|
| `STARTED` / `IN_PROGRESS` | `PENDING` | 없음 |
| `COMPLETED` | `OBSERVED` | 있음. **이후 복귀 실패·재시작에도 ID·내용·시각이 바뀌지 않습니다** |
| `FAILED` | `NOT_OBSERVED` | 없음 (관측 전에 끝남) |

관측 뒤 복귀에 실패하면 `status=COMPLETED` 그대로, `physical_state=RETURN_FAILED`, `physical_failure_reason=RETURN_FAILED` 입니다.
관측 뒤에 `FAILED` 를 보내는 경로는 없습니다.

**observation**

```json
{"observation_id":"A-uav1:ATT-1:OBS","observed_at":"2026-10-04T05:00:00Z",
 "position":{"lat":38.0425,"lon":128.2541,"alt_m_amsl":894.0},
 "sensor_type":"THERMAL","values":{"max_temp_c":312.5,"hotspot_detected":true},"value_status":"SIMULATED",
 "measurements":[
   {"measurement_id":"A-uav1:ATT-1:THERMAL","sensor_type":"THERMAL","values":{"max_temp_c":312.5,"hotspot_detected":true},
    "value_status":"SIMULATED","basis":"MOCK_FIXED_PLACEHOLDER","observed_at":"2026-10-04T05:00:00Z"},
   {"measurement_id":"A-uav1:ATT-1:WEATHER","sensor_type":"WEATHER",
    "values":{"wind_ms":3.1,"wind_dir_deg":null,"temperature_c":null,"humidity_pct":null},
    "value_status":"SIMULATED","basis":"MOCK_PROVIDER_VALUE","observed_at":"2026-10-04T05:00:00Z"}]}
```

- `position` 은 관측 시점 기체의 실제 위치입니다 (lat·lon·alt_m_amsl 항상 포함).
- `sensor_type`·`values` 는 첫 측정입니다 (0.1.0 호환). 측정이 없으면(이동만) `null`·`{}`.
- **값은 실측이 아닙니다 (UAV-05)**. `value_status`: `SIMULATED`(모의값) / `NO_DATA`(값 없음).
  열화상 `312.5℃/hotspot` 은 mock 의 고정 자리표시입니다(`basis=MOCK_FIXED_PLACEHOLDER`). real 은 기본으로 값을 비웁니다(`NO_THERMAL_PIPELINE`).

흐름: 순항고도로 목표 상공 → 관측고도로 하강 → 체류(`observe_duration_s`) → `COMPLETED` → 순항고도로 기지 복귀(`RETURNING`) → 착륙(`DONE`).

- 비행·관측 중 1초마다 "지금 돌아가면 남을 배터리"를 확인합니다. 15% 미만이면 임무를 끊고 `FAILED / RETURN_MARGIN_INSUFFICIENT` 로 바꾼 뒤 복귀합니다.
- 복귀 중 재배정(UAV-06): 기본은 **받지 않습니다** — 착륙(`phase=DONE`)까지 evaluate 는 `REJECT/BUSY`, execute 는 409 `BUSY`. 총괄 방침(READY 확인 전 재배정 안 함)과 같습니다. `UAV_RETASK_WHILE_RETURNING=1` 이면 복귀를 멈추고 새 목표로 가며 이전 Task 는 `phase=RETASKED`.
- mock은 `MOCK_TIME_SCALE`(기본 100배속)으로 비행합니다.

## POST /uav/{uav_id}/task/{task_id}/abort

본문(선택): `{"reason": "CANCELLED_BY_ORCH"}`. 응답은 Task 상태(위와 같은 형식)입니다. 같은 요청을 다시 보내면 처음 결과를 그대로 돌려줍니다.

| `abort.result` | 상황 | 이후 상태 |
|---|---|---|
| `ABORTED_BEFORE_OBSERVATION` | 이동·관측 중 | `status=FAILED`, `reason=ABORTED`, 관측 없음, `physical_state` `RETURNING` → `LANDED` |
| `ALREADY_RETURNING` | 관측을 마쳤거나 이미 끊겨 복귀 중 | 바뀌지 않음 (멈출 임무가 없음) |
| `ALREADY_FINISHED` | 착륙·복귀 실패·재시작 정리 뒤 | 바뀌지 않음 |

없는 Task 는 404. 점유 해제는 지금처럼 `phase=DONE`(착륙) 확인으로 합니다.

## 경로 (UAV-03)

- 경로 = 출발점 → 순항고도로 목표 상공 → 관측고도로 하강·체류 → 순항고도로 기지 상공 → 착륙. 고도는 제자리에서 맞춘 뒤 수평 이동합니다 (mock·real 동일).
- 지형: 저장소 DEM(`environment/data/processed/dem_clipped.tif`, 90 m 격자)을 45 m 간격으로 샘플하고, 샘플마다 주변 8칸까지 최고값을 봅니다.
- `route.terrain.status`: `OK`(전 구간 확인) / `PARTIAL`(일부 구간이 DEM 격자 밖 — 확인한 구간만 반영) / `UNVERIFIED`(DEM 없음·전부 격자 밖 — 관측고도로만 비행, 0.1.0 과 같음).
- `route_hash` = 경로 내용(출발·목표·고도·체류·측정·경유점·지형 결과)의 SHA-256. 같은 조건이면 같은 해시입니다. 평가한 경로는 파일에 저장해 재시작 뒤에도 실행할 수 있습니다.

## POST /mock/set (mock 전용)

쿼리 파라미터 `battery_pct`, `wind_ms`, `failsafe`, `gps_ok`, `link_quality` 중 원하는 것만 넣습니다. 응답은 바뀐 상태입니다.

```bash
curl -X POST 'localhost:8000/mock/set?battery_pct=78.5&wind_ms=9'
```

## 오류 응답

| 코드 | 상황 |
|---|---|
| 404 | 경로의 `uav_id`가 이 서버 담당이 아님, 또는 없는 `task_id` |
| 409 | 다른 Task 이동·관측 중 / `EXECUTION_ID_CONFLICT` / `BUSY`(복귀 중 재배정 끔) / `ROUTE_*` |
| 400 | 목표 지면고도 없음 / 지원하지 않는 측정 / real 모드에서 `/mock/set` |
| 422 | 요청 본문 형식 오류, `execution_attempt_id ≠ task_id`, 체류시간 범위 밖 |
| 500 | PX4 연결 실패 또는 텔레메트리 타임아웃 (다음 호출에서 재연결 시도) |

## 0.1.0 대비 변경

| 항목 | 0.1.0 | 0.2.0 | 요청서 |
|---|---|---|---|
| `observation_type` 기본값 | `THERMAL` | 없음 = 이동만 (기상 전용 임무) | UAV-07 |
| 기한 | `deadline`(ISO 시각, 실제 UTC 와 비교) | `remaining_time_s`(시뮬레이션 초). `deadline` 은 무시 | UAV-04 |
| `DEADLINE_TIGHT` | COUNTER (실을 필드 없음) | REJECT | UAV-02 |
| `MODERATE_WIND` | COUNTER (note 만) | ACCEPT + `constraints.warnings` | UAV-02 |
| 복귀 중 새 임무 | 받음 (복귀 중단, `RETASKED`) | 착륙까지 `BUSY` (설정으로 예전 동작 가능) | UAV-06 |
| `RETURN_MARGIN_INSUFFICIENT` COUNTER | 체류 30 s 고정 제안 (요청에 칸 없음) | 복귀 여유를 지키는 최대 체류를 계산해 제안, 요청에 `observe_duration_s` 칸 추가. 불가능하면 REJECT | UAV-02 |
| 비행고도 | 관측고도로 직선 비행 (경로 중간 지형 무시) | 경로 지형을 넘는 순항고도 → 목표에서 관측고도로 하강 | UAV-03 |
| 관측값 | 고정값을 실측처럼 보고 | `value_status`·`basis` 로 모의임을 표시. real 은 기본으로 비움 | UAV-05 |

## 팀 합의안과 현재 코드의 차이

팀 합의안은 [docs/uav/uav_final](../docs/uav/uav_final.md)입니다. 아래는 아직 코드에 반영되지 않은 부분입니다.

| 항목 | 합의안 | 현재 코드 |
|---|---|---|
| 목표 고도 | AGL(`target_agl_m`)로 전달, 경로 전체 terrain-following | `target_agl_m` 수신(+10 m 여유). 경로는 지형 최고점 위 단일 순항고도 (구간별 지형 추종은 아님) |
| 풍속 전달 | `environment.wind_speed_mps` 등 객체 | 최상위 `wind_ms` 하나 |
| 배터리 부족 코드 | `INSUFFICIENT_BATTERY` | `LOW_BATTERY` (팀 `RejectReason` 기준) |
| 기한 불가 코드 | `DEADLINE_INFEASIBLE` | `TIMEOUT` / `DEADLINE_TIGHT` |
| BUSY | 수행 중인 동일 Task의 재평가는 허용 | 이동·관측 중이면 어떤 Task든 `BUSY` |
| 상태 필드명 | `resource_id`, `battery_pct`, `communication_state` | `uav_id`, `battery.percent`, `link_quality` |
| 실행 중 수행 곤란 보고·복귀 | 사유·수집값 보고 후 복귀 | 배터리 부족·중단 요청 구현. 수집값 보고는 없음 |
| 순찰·스캔 비행 | — | 미구현 (UAV-08 향후). 도착 후 체류만 |
