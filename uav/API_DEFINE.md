# UAV Local Agent API 정의서

| 항목 | 값 |
|---|---|
| 버전 | 0.1.0 |
| Base URL | 서버 `http://3.37.210.229:8000` (평소 꺼져 있음, 필요 시 UAV 담당에게 요청) / 로컬 `http://localhost:8000` |
| 형식 | JSON, UTF-8 |
| 단위 | 거리·고도 m, 속도·풍속 m/s, 시간 s, 배터리 %(0~100), 좌표 WGS84 |
| 코드 | `uav/uav-agent/main.py`, `models.py`, `evaluator.py` |

## 엔드포인트 요약

| 메서드 | 경로 | 용도 |
|---|---|---|
| GET | `/health` | 서버 모드·담당 UAV 확인 |
| GET | `/uav/{uav_id}/state` | 상태 조회 |
| POST | `/uav/{uav_id}/evaluate` | 수행 가능성 판단 (**실행하지 않음**) |
| POST | `/uav/{uav_id}/execute` | 실행 시작 (**Safety ALLOW 이후에만 호출**) |
| GET | `/uav/{uav_id}/task/{task_id}` | 실행 진행·결과 조회 |
| POST | `/mock/set` | 테스트용 상태 주입 (mock 전용) |

호출 순서: `state` → `evaluate` → (총괄·Safety 판단) → `execute` → `task` 폴링

---

## GET /health

```json
{"status": "ok", "mode": "real", "uav_id": "A-uav1"}
```

`mode`는 `mock` 또는 `real`입니다.

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
| `wind_ms` | float \| null | ⚠️ real에서 항상 `null` (현재 PX4 SITL 구성에서 wind telemetry가 발행되지 않음, 실측) |
| `link_quality` | string | ⚠️ real에서 항상 `"OK"` (고정값) |
| `current_task_id` | string \| null | 이동·관측 중인 Task id. 대기·복귀 중에는 `null` |

⚠️ 표시 필드는 가용성 판단 근거로 쓰지 마세요.

```json
{"uav_id":"A-uav1","timestamp":"2026-09-24T07:01:05Z",
 "position":{"lat":38.1205,"lon":128.2018,"alt_m_amsl":235.0,"alt_m_agl":0.0},
 "battery":{"percent":95.0,"voltage":15.8},"velocity_ms":0.0,"flight_mode":"HOLD","armed":false,
 "health":{"gps_ok":true,"sensors_ok":true,"failsafe":false},
 "wind_ms":3.1,"link_quality":"OK","current_task_id":null}
```

## POST /uav/{uav_id}/evaluate

판단만 하며 기체를 움직이지 않습니다.

**요청**

| 필드 | 타입 | 필수 | 설명 |
|---|---|---|---|
| `task_id` | string | ✅ | |
| `decision_id` | string | ✅ | 재평가마다 새 값 |
| `target.lat` / `target.lon` | float | ✅ | 관측 목표 |
| `target.alt_m_amsl` | float | ✅ | 목표 지점 **지면** 해발고도(AMSL). 없으면 `REJECT/TARGET_ALTITUDE_UNKNOWN` |
| `target.target_agl_m` | float | | 지면 위 목표 높이(AGL). Orchestrator 가 정한다. 없으면 80 |

비행고도(AMSL) = `alt_m_amsl` + `target_agl_m` + 10 m(여유, `config.AGL_MARGIN_M`). 예: 804 + 80 + 10 = 894 m. 목표 지점의 지면만 고려하며 경로 중간 지형은 아직 반영하지 않습니다.
| `observation_type` | string | | `THERMAL`(기본) 또는 `RGB` |
| `deadline` | string | | ISO8601 UTC |
| `wind_ms` | float | | 환경모델 풍속. **real에서 없으면 `REJECT/WIND_UNKNOWN`** (mock은 내부값 3.1 사용) |

```json
{"task_id":"task-101","decision_id":"dec-001",
 "target":{"lat":38.0425,"lon":128.2541,"alt_m_amsl":804,"target_agl_m":80},
 "observation_type":"THERMAL","wind_ms":3.1}
```

**응답 200**

| 필드 | 타입 | 설명 |
|---|---|---|
| `task_id`, `decision_id`, `uav_id` | string | 요청값 그대로 |
| `verdict` | string | `ACCEPT` / `REJECT` / `COUNTER` |
| `eta_sec` | int \| null | ACCEPT·COUNTER는 정수(0 가능), REJECT는 `null` |
| `reason` | string \| null | 아래 표 |
| `detail` | string \| null | 사람이 읽는 설명 (일부 REJECT만) |
| `counter_offer` | object \| null | COUNTER일 때 대안 |
| `constraints` | object | 판단 근거 (`distance_km`, `battery_now_pct`, `battery_after_pct`, `return_margin_pct`, `wind_ms`, `flight_alt_m_amsl`, `required_climb_m`, `eta_sec`, `deadline_slack_sec`) |

```json
{"task_id":"task-101","decision_id":"dec-001","uav_id":"A-uav1","verdict":"ACCEPT","eta_sec":860,
 "reason":null,"detail":null,"counter_offer":null,
 "constraints":{"distance_km":9.81,"battery_now_pct":95,"battery_after_pct":36.3,"return_margin_pct":36.3,
                "wind_ms":3.1,"flight_alt_m_amsl":894.0,"required_climb_m":659,"eta_sec":860}}
```

**판정 순서와 reason 코드** (위에서부터 검사하고 처음 걸린 것을 반환)

| 순서 | verdict | reason | 조건 |
|---|---|---|---|
| 1 | REJECT | `WIND_UNKNOWN` | 풍속 없음 |
| 2 | REJECT | `TARGET_ALTITUDE_UNKNOWN` | `target.alt_m_amsl` 없음 |
| 3 | REJECT | `FAILSAFE_ACTIVE` | failsafe 작동 |
| 4 | REJECT | `SENSOR_FAILURE` | GPS 또는 센서 이상 |
| 5 | REJECT | `COMMUNICATION_FAILURE` | `link_quality == "LOST"` |
| 6 | REJECT | `BUSY` | 다른 Task 수행 중 (현재 동작 안 함) |
| 7 | REJECT | `REQUIRED_CAPABILITY_UNAVAILABLE` | 센서 미탑재 |
| 8 | REJECT | `HIGH_WIND` | 풍속 ≥ 10 m/s |
| 9 | REJECT | `LOW_BATTERY` | 왕복+관측 후 배터리 < 0 |
| 10 | COUNTER | `RETURN_MARGIN_INSUFFICIENT` | 복귀 후 배터리 < 15% |
| 11 | REJECT | `TIMEOUT` | deadline 이미 지남 |
| 12 | COUNTER | `DEADLINE_TIGHT` | ETA > deadline |
| 13 | COUNTER | `MODERATE_WIND` | 8 ≤ 풍속 < 10 m/s |
| — | ACCEPT | — | 모두 통과 |

풍속이 8~10 m/s라도 10·12번에 먼저 걸리면 `reason`은 그쪽으로 나옵니다. 풍속은 `constraints.wind_ms`에서 확인하세요. 임계값(풍속 8/10 m/s, 복귀 여유 15%)은 모두 프로젝트 잠정값입니다(`uav-agent/config.py`). reason 코드는 `TARGET_ALTITUDE_UNKNOWN`을 빼고 팀 `interfaces/schema.py`의 `RejectReason`에 등록되어 있습니다(`TARGET_ALTITUDE_UNKNOWN`은 등록 협의 필요).

## POST /uav/{uav_id}/execute

Safety ALLOW 이후에만 호출합니다. 비동기로 시작하고 즉시 반환합니다.

**요청**: `task_id`, `decision_id`, `target`, `observation_type` (evaluate와 같은 형식, `wind_ms`·`deadline` 없음)

**응답 200**

```json
{"task_id":"task-101","status":"STARTED","tracking_url":"/uav/A-uav1/task/task-101"}
```

## GET /uav/{uav_id}/task/{task_id}

| 필드 | 설명 |
|---|---|
| `status` | `STARTED` / `IN_PROGRESS` / `COMPLETED` / `FAILED` |
| `progress.phase` | `ENROUTE` → `OBSERVING` → `RETURNING` → `DONE`. 복귀 중 새 임무를 받으면 `RETASKED`, 복귀 실패는 `RETURN_FAILED` |
| `observation` | COMPLETED일 때 `observed_at`, `position`(관측 시점 기체 실제 위치), `sensor_type`, `values` |
| `reason` | FAILED 사유 코드. 비행 중 배터리 부족으로 중단하면 `RETURN_MARGIN_INSUFFICIENT` |
| `error` | FAILED일 때 상세 |

흐름: 목표 도착까지 대기 → 관측 체류 60초 → `COMPLETED`(관측값 반환) → 기지 복귀(`RETURNING`) → 착륙(`DONE`).

- 비행·관측 중 1초마다 "지금 돌아가면 남을 배터리"를 확인합니다. 15% 미만이면 임무를 끊고 `FAILED / RETURN_MARGIN_INSUFFICIENT`로 바꾼 뒤 복귀합니다(시나리오 5). 총괄은 이 Task를 다른 기체에 인계하면 됩니다.
- 복귀 중인 기체는 BUSY가 아닙니다. evaluate가 현재 위치 → 목표 → 기지로 배터리를 계산하고, execute하면 복귀를 멈추고 새 목표로 갑니다(시나리오 4·7 재배치).
- mock은 `MOCK_TIME_SCALE`(기본 100배속)으로 비행합니다. 10 km 편도 임무도 관측 완료까지 약 13초입니다. real은 실제 시간이 걸립니다.
- ⚠️ `values`(`max_temp_c: 312.5`)는 고정값입니다. Task 상태는 메모리에만 있어 서버를 다시 시작하면 사라집니다.

## POST /mock/set (mock 전용)

쿼리 파라미터 `battery_pct`, `wind_ms`, `failsafe`, `gps_ok`, `link_quality` 중 원하는 것만 넣습니다. 응답은 바뀐 상태입니다.

```bash
curl -X POST 'localhost:8000/mock/set?battery_pct=78.5&wind_ms=9'
```

## 오류 응답

| 코드 | 상황 |
|---|---|
| 404 | 경로의 `uav_id`가 이 서버 담당이 아님, 또는 없는 `task_id` |
| 409 | 다른 Task가 이동·관측 중 |
| 400 | real 모드에서 `/mock/set` 호출 |
| 422 | 요청 본문 형식 오류 |
| 500 | PX4 연결 실패 또는 텔레메트리 타임아웃 (다음 호출에서 재연결 시도) |

## 팀 합의안과 현재 코드의 차이

팀 합의안은 [docs/uav/uav_final](../docs/uav/uav_final)입니다. 아래는 아직 코드에 반영되지 않은 부분입니다.

| 항목 | 합의안 | 현재 코드 |
|---|---|---|
| 목표 고도 | AGL(`target_agl_m`)로 전달, 경로 전체 terrain-following | `target_agl_m` 수신(+10 m 여유), 목표 지점 지면만 고려 — 경로 terrain-following 미구현 |
| 풍속 전달 | `environment.wind_speed_mps` 등 객체 | 최상위 `wind_ms` 하나 |
| 배터리 부족 코드 | `INSUFFICIENT_BATTERY` | `LOW_BATTERY` (팀 `RejectReason` 기준) |
| 기한 불가 코드 | `DEADLINE_INFEASIBLE` | `TIMEOUT` (팀 `RejectReason` 기준) |
| BUSY | 수행 중인 동일 Task의 재평가는 허용 | 이동·관측 중이면 어떤 Task든 `BUSY` (동일 Task 재평가 예외 미구현) |
| 상태 필드명 | `resource_id`, `battery_pct`, `communication_state` | `uav_id`, `battery.percent`, `link_quality` |
| 실행 중 수행 곤란 보고·복귀 | 사유·수집값 보고 후 복귀 | 배터리 부족만 구현 (`FAILED / RETURN_MARGIN_INSUFFICIENT` 후 복귀). 수집값 보고는 없음 |
