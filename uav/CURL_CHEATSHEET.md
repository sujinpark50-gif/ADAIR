# UAV Agent curl 치트시트

`A-uav1` / `localhost:8000` 기준. **그대로 복붙하면 동작합니다.**
서버가 다르면 맨 아래 "호스트·ID 바꿔 쓰기"를 보세요.
전체 스펙은 [API_DEFINE.md](API_DEFINE.md), Swagger 는 <http://localhost:8000/docs>.

> 아래 응답 예시는 2026-09-28 real 모드(PX4 SITL, 강원 월드)에서 실제로 받은 값입니다.

## 0. 준비

```bash
# PX4 (real 모드일 때만. mock 이면 건너뜀)
cd uav/etc && ./px4-start-kangwon.sh

# Agent
source .venv/bin/activate
cd uav/uav-agent
UAV_ID=A-uav1 UAV_MODE=real uvicorn main:app --host 0.0.0.0 --port 8000
```

## 1. GET /health

```bash
curl -s localhost:8000/health
```
```json
{"status":"ok","mode":"real","uav_id":"A-uav1"}
```

## 2. GET /state

```bash
curl -s localhost:8000/uav/A-uav1/state
```
```json
{"uav_id":"A-uav1","position":{"lat":38.1204959,"lon":128.2017977,"alt_m_amsl":230.92,"alt_m_agl":0.026},
 "battery":{"percent":100.0,"voltage":16.2},"flight_mode":"HOLD","armed":false,
 "health":{"gps_ok":true,"sensors_ok":true,"failsafe":false},"wind_ms":null,"link_quality":"OK"}
```

> real 에서 `wind_ms` 는 항상 `null`, `failsafe`·`link_quality`·`current_task_id` 는 고정값입니다.
> 가용성 판단 근거로 쓰지 마세요.

## 3. POST /evaluate — 판단만, 기체는 안 움직임

```bash
curl -s -X POST localhost:8000/uav/A-uav1/evaluate \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "task-101",
    "decision_id": "dec-001",
    "target": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 804},
    "observation_type": "THERMAL",
    "wind_ms": 3.1
  }'
```
```json
{"verdict":"ACCEPT","eta_sec":861,"reason":null,
 "constraints":{"distance_km":9.81,"battery_now_pct":100.0,"battery_after_pct":41.2,
                "return_margin_pct":41.2,"wind_ms":3.1,"required_climb_m":623,"eta_sec":861}}
```

**`wind_ms` 를 빼먹으면 무조건 거절됩니다** — real 에서 가장 흔한 실수입니다.

```bash
curl -s -X POST localhost:8000/uav/A-uav1/evaluate \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"task-102","decision_id":"dec-002",
       "target":{"lat":38.0425,"lon":128.2541,"alt_m_amsl":804}}'
```
```json
{"verdict":"REJECT","reason":"WIND_UNKNOWN","constraints":{}}
```

`target.alt_m_amsl` 은 **목표 지점 지면의 해발고도**입니다. 비행고도는 여기에 +50 m.
빼면 기본 0 이 들어가 지형을 뚫는 경로가 나옵니다 — 반드시 넣으세요.

### 판정별로 찔러보기

| 보고 싶은 결과 | 바꿀 값 |
|---|---|
| `REJECT` / `HIGH_WIND` | `"wind_ms": 12` |
| `COUNTER` / `MODERATE_WIND` | `"wind_ms": 9` |
| `REJECT` / `TIMEOUT` | `"deadline": "2020-01-01T00:00:00Z"` 추가 |
| `COUNTER` / `DEADLINE_TIGHT` | `"deadline"` 을 지금부터 3분 뒤로 |

> **배터리가 먼저 걸립니다.** 판정은 위에서부터 검사해 처음 걸린 것을 반환하는데,
> `LOW_BATTERY`(8번)가 `MODERATE_WIND`(12번)보다 앞입니다. real 에서 execute 를 한 번
> 돌리면 배터리가 50% 대로 떨어져, 풍속·deadline 을 뭘 넣어도 `LOW_BATTERY` 만 나옵니다.
> 위 표대로 재현하려면 PX4 를 재기동해 배터리를 100% 로 되돌리세요:
> `cd uav/etc && ./px4-stop.sh && ./px4-start-kangwon.sh`
> (`constraints.battery_now_pct` 로 현재 배터리를 확인할 수 있습니다.)

```bash
# HIGH_WIND
curl -s -X POST localhost:8000/uav/A-uav1/evaluate -H 'Content-Type: application/json' \
  -d '{"task_id":"t-wind","decision_id":"d1",
       "target":{"lat":38.0425,"lon":128.2541,"alt_m_amsl":804},"wind_ms":12}'

# DEADLINE_TIGHT (지금부터 3분 뒤)
curl -s -X POST localhost:8000/uav/A-uav1/evaluate -H 'Content-Type: application/json' \
  -d "{\"task_id\":\"t-dl\",\"decision_id\":\"d1\",
       \"target\":{\"lat\":38.0425,\"lon\":128.2541,\"alt_m_amsl\":804},\"wind_ms\":3.1,
       \"deadline\":\"$(date -u -d '+3 minutes' +%Y-%m-%dT%H:%M:%SZ)\"}"
```

## 4. POST /execute — 실제로 움직임

`evaluate` 가 ACCEPT 고 Safety ALLOW 를 받은 뒤에만 호출합니다.
`wind_ms`·`deadline` 은 받지 않습니다. 즉시 반환하고 비행은 백그라운드로 진행됩니다.

```bash
curl -s -X POST localhost:8000/uav/A-uav1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "task-101",
    "decision_id": "dec-001",
    "target": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 804},
    "observation_type": "THERMAL"
  }'
```
```json
{"task_id":"task-101","status":"STARTED","tracking_url":"/uav/A-uav1/task/task-101"}
```

## 5. GET /task — 진행 조회

```bash
curl -s localhost:8000/uav/A-uav1/task/task-101
```
```json
{"status":"COMPLETED","progress":{"phase":"DONE"},
 "observation":{"observed_at":"2026-09-28T07:14:13Z","sensor_type":"THERMAL",
                "values":{"max_temp_c":312.5,"hotspot_detected":true}}}
```

> ⚠️ `COMPLETED` 는 **도착·관측 완료가 아닙니다.** 이동 명령 접수 2초 뒤 바뀌고
> `values` 는 고정값입니다. Task 는 메모리에만 있어 Agent 재시작 시 사라집니다.

2초 간격 폴링:

```bash
while :; do curl -s localhost:8000/uav/A-uav1/task/task-101; echo; sleep 2; done
```

## 6. 한 번에: evaluate → execute → 폴링

ACCEPT 일 때만 execute 로 넘어갑니다.

```bash
TARGET='{"lat":38.0425,"lon":128.2541,"alt_m_amsl":804}'
TASK="task-$(date +%s)"

V=$(curl -s -X POST localhost:8000/uav/A-uav1/evaluate -H 'Content-Type: application/json' \
  -d "{\"task_id\":\"$TASK\",\"decision_id\":\"dec-1\",\"target\":$TARGET,\"wind_ms\":3.1}")
echo "$V"

if echo "$V" | grep -q '"verdict":"ACCEPT"'; then
  curl -s -X POST localhost:8000/uav/A-uav1/execute -H 'Content-Type: application/json' \
    -d "{\"task_id\":\"$TASK\",\"decision_id\":\"dec-1\",\"target\":$TARGET,\"observation_type\":\"THERMAL\"}"
  echo
  for i in 1 2 3 4 5; do sleep 2; curl -s "localhost:8000/uav/A-uav1/task/$TASK"; echo; done
else
  echo "ACCEPT 아님 — execute 생략"
fi
```

## 7. POST /mock/set — mock 전용 상태 주입

`UAV_MODE=mock` 으로 띄웠을 때만 됩니다. real 이면 400.

```bash
curl -s -X POST 'localhost:8000/mock/set?battery_pct=15&wind_ms=9'
curl -s -X POST 'localhost:8000/mock/set?failsafe=true'
curl -s -X POST 'localhost:8000/mock/set?gps_ok=false'
curl -s -X POST 'localhost:8000/mock/set?link_quality=LOST'
```

배터리를 15% 로 낮춘 뒤 evaluate 하면 `LOW_BATTERY` 나 `RETURN_MARGIN_INSUFFICIENT` 가 나옵니다.

## 오류 확인

```bash
# 404 — 이 서버 담당이 아닌 uav_id
curl -s -X POST localhost:8000/uav/A-uav9/evaluate -H 'Content-Type: application/json' \
  -d '{"task_id":"t","decision_id":"d","target":{"lat":38.0,"lon":128.2},"wind_ms":3}'
# → {"detail":"이 서버는 A-uav1 담당이다. 요청된 A-uav9 는 다른 주소를 확인할 것"}

# 409 — 진행 중인 task_id 로 재실행 (끝난 task 는 200 으로 재실행됨)
# 422 — 필수 필드 누락
curl -s -X POST localhost:8000/uav/A-uav1/evaluate -H 'Content-Type: application/json' -d '{"task_id":"x"}'

# 500 — PX4 연결 실패. 컨테이너가 Up 이어도 MAVLink 가 죽어 있을 수 있다:
#   docker exec px4 /opt/px4-gazebo/bin/px4-mavlink status   # 빈 출력이면 죽은 것
#   cd uav/etc && ./px4-stop.sh && ./px4-start-kangwon.sh
```

## 호스트·ID 바꿔 쓰기

```bash
H=localhost:8000 ; U=A-uav1          # 로컬
# H=3.37.210.229:8000 ; U=A-uav1     # 서버 (평소 꺼져 있음)

curl -s "$H/health"
curl -s "$H/uav/$U/state"
curl -s -X POST "$H/uav/$U/evaluate" -H 'Content-Type: application/json' \
  -d "{\"task_id\":\"t1\",\"decision_id\":\"d1\",
       \"target\":{\"lat\":38.0425,\"lon\":128.2541,\"alt_m_amsl\":804},\"wind_ms\":3.1}"
```

`jq` 가 있으면 `| jq` 를 붙여 보기 좋게 출력할 수 있습니다.

## 좌표 참고

| 지점 | lat | lon | 지면 AMSL |
|---|---|---|---|
| 이륙 지점 (원통119) | 38.1205 | 128.2018 | 약 231 |
| 관측 목표 예시 | 38.0425 | 128.2541 | 804 |
