# UAV Local Agent

산불 관측 시뮬레이터 — UAV 담당(김동현) 모듈.

이 모듈은 **별도 서버로 실행**되고, 팀 코드는 `connectors/uav_connector.py` 에서
HTTP 로 호출합니다. PX4/Gazebo 도커가 필요해 저장소에 코드만 두고 실행은 UAV 서버에서 합니다.

> **두 가지 모드로 돕니다.**
> - `mock` — PX4 없이 고정값 응답. 다른 팀원이 UAV 서버 없이 개발할 때 사용
> - `real` — PX4/Gazebo 실연동. 위치·배터리·health 를 실제로 읽음
>
> 응답 **형식은 두 모드가 동일**합니다.

## 팀 스키마와 다른 점 (합의 필요)

| 항목 | 이 모듈 | 팀 `interfaces/schema.py` |
|---|---|---|
| 고도 | `alt_m_amsl` / `alt_m_agl` | **필드 없음** — 목표 고도를 받을 수 없음 |
| 시간 | ISO8601 UTC 문자열 | `timestamp: float` (경과 초) |
| 풍속 | 요청의 `wind_ms` 로 주입받음 | `EnvironmentState.wind_speed` — 전달 경로 없음 |
| 좌표 | 위경도만 | 화재 위치가 격자 `cell_id` (`"50_50"`) |

`reason` 코드는 팀 `RejectReason` 에 맞췄습니다. 다만 `FAILSAFE_ACTIVE`,
`BUSY`, `WIND_UNKNOWN` 3개는 대응물이 없어 **추가 합의가 필요**합니다.

자세한 배경은 [SHARE.md](../SHARE.md) 4번 참조.

## 실행

```bash
source ~/uav-venv/bin/activate
cd px4/uav-agent

# Mock — PX4 불필요
UAV_MODE=mock uvicorn main:app --host 0.0.0.0 --port 8000

# Real — PX4/Gazebo 를 먼저 띄워야 한다
../px4-start.sh                                             # 기동에 약 30초
UAV_MODE=real uvicorn main:app --host 0.0.0.0 --port 8000
```

현재 모드 확인: `curl localhost:8000/health` → `mode`, `uav_id` 필드

## UAV 여러 대 운용

**UAV 1대 = 프로세스 1개 = 포트 1개**입니다. 한 프로세스가 여러 대를 관리하지 않습니다.

```bash
UAV_ID=A-uav1 UAV_MODE=mock uvicorn main:app --host 0.0.0.0 --port 8000
UAV_ID=A-uav2 UAV_MODE=mock uvicorn main:app --host 0.0.0.0 --port 8001
```

호출측은 **UAV 별로 주소를 다르게** 잡습니다. 경로의 `uav_id` 는 그대로 두고 포트(또는 호스트)로 구분합니다.

```python
UAV_ENDPOINTS = {
    "A-uav1": "http://<host>:8000",
    "A-uav2": "http://<host>:8001",
}
base = UAV_ENDPOINTS[resource_id]
requests.post(f"{base}/uav/{resource_id}/evaluate", json=payload)
```

담당이 아닌 UAV 를 호출하면 **404** 로 끊습니다. 조용히 다른 기체의 상태를 돌려주지 않습니다.

```
GET http://<host>:8000/uav/A-uav2/state
→ 404 {"detail":"이 서버는 A-uav1 담당이다. 요청된 A-uav2 는 다른 주소를 확인할 것"}
```

> ⚠️ **`real` 모드는 서버당 1대만** 올립니다. PX4 1개가 약 1.1 코어를 쓰고, 두 프로세스가
> 같은 UDP 포트(14540)를 물면 텔레메트리 값이 튑니다. 실기체 여러 대는 **서버를 분리**하고
> 각자 `PX4_ADDRESS` 로 자기 PX4 를 가리킵니다. Mock 은 한 서버에 여러 대 올려도 됩니다.

API 문서: `http://<host>:8000/docs` (Swagger UI에서 직접 호출 가능)

## 엔드포인트

| 메서드 | 경로 | 용도 |
|---|---|---|
| GET | `/uav/{uav_id}/state` | UAV 상태 조회 |
| POST | `/uav/{uav_id}/evaluate` | **수행 가능성 판단** (실행 안 함) |
| POST | `/uav/{uav_id}/execute` | 실행 (Safety ALLOW 이후) |
| GET | `/uav/{uav_id}/task/{task_id}` | 진행/결과 조회 |
| POST | `/mock/set` | 테스트용 상태 주입 (Mock 전용) |

## 판단 결과

`verdict` 는 `ACCEPT` / `REJECT` / `COUNTER` 중 하나입니다.

**`evaluate` 는 판단만 합니다.** ACCEPT 를 받아도 자동으로 비행하지 않습니다.
Safety 검증 후 `execute` 를 호출해 주세요.

### reason 코드

| 코드 | 의미 |
|---|---|
| `HIGH_WIND` | 풍속 한계 초과 |
| `FAILSAFE_ACTIVE` | PX4 failsafe 작동 중 |
| `SENSOR_FAILURE` | GPS·센서 이상 |
| `COMMUNICATION_FAILURE` | 통신 두절 |
| `BUSY` | 다른 Task 수행 중 |
| `REQUIRED_CAPABILITY_UNAVAILABLE` | 요청한 관측 센서 미탑재 |
| `LOW_BATTERY` | 배터리 부족 |
| `TIMEOUT` | 기한 내 불가 |
| `RETURN_MARGIN_INSUFFICIENT` | (COUNTER) 복귀 여유 부족 |
| `DEADLINE_TIGHT` | (COUNTER) 기한 초과하나 수행 가능 |
| `MODERATE_WIND` | (COUNTER) 풍속 주의 |
| `WIND_UNKNOWN` | 풍속 정보 없음 — 요청에 `wind_ms` 를 포함해야 함 |

모든 응답에 `constraints` 가 포함되어 판단 근거를 확인할 수 있습니다.

## 호출 예시

### 상태 조회

```bash
curl localhost:8000/uav/uav-01/state
```

### 판단 요청

```bash
curl -X POST localhost:8000/uav/uav-01/evaluate \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "task-101",
    "decision_id": "dec-001",
    "target": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 804},
    "observation_type": "THERMAL"
  }'
```

### IT-02 (강풍 REJECT) 재현

요청에 `wind_ms` 를 실으면 그 값으로 판단합니다.

```bash
curl -X POST localhost:8000/uav/uav-01/evaluate \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "task-101",
    "decision_id": "dec-001",
    "target": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 804},
    "observation_type": "THERMAL",
    "wind_ms": 12.4
  }'
```

→ `REJECT / HIGH_WIND` 반환. 이 경우 **Safety 를 호출하지 마세요.**
총괄이 새 `decision_id` 로 다른 UAV 를 평가하면 됩니다.

또는 UAV 자체 상태를 바꿔도 됩니다:

```bash
curl -X POST 'localhost:8000/mock/set?wind_ms=12.4'
```

### 실행 → 결과 조회

```bash
curl -X POST localhost:8000/uav/uav-01/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "task-101",
    "decision_id": "dec-001",
    "target": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 804},
    "observation_type": "THERMAL"
  }'

curl localhost:8000/uav/uav-01/task/task-101
```

## 좌표 참고

| 지점 | 좌표 | 고도 |
|---|---|---|
| 원통119 (홈) | 38.1205, 128.2018 | ~235 m |
| 관측 목표 (중간) | 38.0425, 128.2541 | **~804 m** |
| 기린119 | 37.9645, 128.3062 | ~285 m |

⚠️ 중간 지점은 평지가 아니라 **능선(약 800~1000 m)** 입니다. 좌표는 ±0.5~1 km 근사입니다.

## 알려진 제약

- 상태값은 Mock 고정값입니다 (PX4 미연동)
- 임계값(풍속 한계 10 m/s, 안전마진 20%)은 **잠정값**으로 팀 합의 필요
- **풍속은 PX4/Gazebo 가 제공하지 않습니다** (실측 확인). `telemetry.wind` 스트림은
  존재하나 PX4 SITL 이 아무것도 발행하지 않습니다. 따라서 풍속은 **요청에 실어 보내거나**
  환경모델에서 받아야 하며, 없으면 `REJECT / WIND_UNKNOWN` 을 반환합니다
- 배터리 소모율(`BATTERY_DRAIN_PCT_S = 0.033`)은 **추정치이며 실측 보정 전**입니다
- Mock 기본 배터리는 95%입니다. 기본 시나리오(원통→중간지점) 판정 분포:
  95% → ACCEPT, 78.5% → COUNTER, 40% → REJECT
  (`/mock/set?battery_pct=` 로 바꿔 세 분기를 모두 재현할 수 있습니다)
- Task 상태는 메모리에 있어 프로세스 재시작 시 사라집니다
