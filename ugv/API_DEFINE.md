# UGV Local Agent API 정의서

| 항목 | 값 |
|---|---|
| 버전 | 0.1.0 |
| Base URL | 로컬 `http://localhost:8100` |
| 형식 | JSON, UTF-8 |
| 단위 | 거리 m, 시간 s, 연료 %(0~100), 좌표 WGS84 |
| 코드 | `ugv/server.py`, `ugv/api_models.py` |
| 문서 (자동) | `http://localhost:8100/docs` |

## 실행

```bash
pip install fastapi uvicorn
UGV_DRIVER=sim uvicorn ugv.server:app --port 8100     # PX4 없이 (좌표 계산 주행)
UGV_DRIVER=px4 uvicorn ugv.server:app --port 8100     # UGV_PX4_RESOURCES 자원만 PX4, 나머지 sim
GUI=1 ./ugv/tools/px4-start.sh 0                       # PX4 SITL rover (gz_r1_rover) + Gazebo 창
```

| 환경변수 | 기본값 | 의미 |
|---|---|---|
| `UGV_DRIVER` | `sim` | `sim` 또는 `px4` |
| `UGV_PX4_RESOURCES` | `A-ugv1` | px4 모드에서 PX4 에 연결할 자원 (쉼표 구분) |
| `UGV_TARGET_SNAP_M` | `2000` | 화재에서 가장 가까운 도로 노드가 이보다 멀면 `TARGET_UNREACHABLE` |
| `UGV_APPROACH_FALLBACK` | `0` | `1` 이면 가장 가까운 노드로 못 갈 때 다음 노드로 접근 |
| `UGV_APPROACH_MAX_M` | `500` | fallback 접근 노드의 화재 거리 상한 |

## 구조

- 서버 1개가 지상자원 전부(`ugv/config.py` 의 `RESOURCES`)를 맡는다. UAV 는 기체 1대당 서버 1개지만, UGV 는 자원들이 도로망 그래프 하나를 공유해야 도로 차단이 모든 자원의 판단에 동시에 반영된다.
- 도로망은 환경 데이터(`environment/data/processed/roads_clipped.gpkg`)를 변환한 `ugv/data/road_network.json` (노드 534, 도로 672, 184 km). 거점 노드 id 는 `A`(원통119), `B`(기린119).
- 도로 노드 id 는 UGV 내부 값이다. 호출측은 위경도로 요청하고, 응답의 `target_node` 로 실제 목적지를 확인한다.

## 엔드포인트 요약

| 메서드 | 경로 | 용도 |
|---|---|---|
| GET | `/health` | 드라이버 모드·자원·도로망·접근 규칙 확인 |
| GET | `/ugv?base=A` | 거점별 자원 상태 목록 |
| GET | `/ugv/{resource_id}/state` | 자원 1대 상태 |
| POST | `/ugv/{resource_id}/evaluate` | 수행 가능성 판단 (**움직이지 않음**) |
| POST | `/ugv/{resource_id}/execute` | 주행 시작 (**Safety ALLOW 이후에만 호출**) |
| POST | `/ugv/{resource_id}/stop` | 주행 중지 |
| GET | `/ugv/{resource_id}/task/{task_id}` | 주행 진행·결과 |
| GET | `/ugv/{resource_id}/observation` | 현재 위치의 도로 상태 관측 |
| GET | `/graph/nodes/{node_id}` | 도로 노드 좌표 |
| POST | `/env/fire_cells` | 화재 셀 반영 → 도로 차단 재계산 |
| POST | `/env/congestion` | 시나리오용 혼잡 설정 |
| GET | `/env/roads` | 현재 차단·혼잡 도로 |

호출 순서: `state` → `evaluate` → (총괄·Safety 판단) → `execute` → `task` 폴링

---

## GET /ugv/{resource_id}/state

| 필드 | 타입 | 설명 |
|---|---|---|
| `resource_id` | string | `A-ugv1`, `A-fire1`, `B-ugv1` |
| `resource_type` | string | `UGV` / `FIRE_ENGINE` |
| `base` | string | `A` / `B` |
| `state` | string | `READY` / `RUNNING` |
| `position.lat` / `position.lon` | float | 현재 위치 |
| `fuel_pct` | float | 0~100. sim: 1 km 당 1% 감소, px4: 배터리 값 |
| `current_node` | string \| null | 노드에 정지 중일 때만 값. 주행 중 `null` |
| `current_task_id` | string \| null | 수행 중 task |
| `driver` | string | `sim` / `px4` |
| `driver_status` | string | sim: `IDLE`/`MISSION`/`ARRIVED`, px4: flight mode |
| `updated_at` | float | 마지막 위치 반영 시각 (epoch s) |

없는 자원이면 `404`.

## POST /ugv/{resource_id}/evaluate

판단만 하며 차를 움직이지 않는다.

**요청** — `target` 과 `target_node` 중 하나

| 필드 | 타입 | 필수 | 설명 |
|---|---|---|---|
| `task_id` | string | ✅ | |
| `decision_id` | string | ✅ | 재평가마다 새 값 |
| `target.lat` / `target.lon` | float | | 화재 좌표 (권장) |
| `target_node` | string | | 도로 노드 id 를 이미 알 때 |

```json
{"task_id": "task-101", "decision_id": "dec-001", "target": {"lat": 38.0444, "lon": 128.2028}}
```

**응답 200**

| 필드 | 타입 | 설명 |
|---|---|---|
| `task_id`, `decision_id`, `resource_id` | string | 요청값 그대로 |
| `verdict` | string | `ACCEPT` / `REJECT` (COUNTER 없음) |
| `eta_sec` | int \| null | ACCEPT 일 때 도로망 기준 도착 예상 시간. REJECT 는 `null` |
| `reason` | string \| null | 아래 표 |
| `detail` | string \| null | 사람이 읽는 설명 |
| `target_node` | object \| null | 실제 목적지 도로 노드 `{node_id, lat, lon, snap_m}`. `snap_m` = 요청 좌표와의 거리 |
| `path` | string[] \| null | ACCEPT 일 때 경유 노드 id (출발 노드 포함) |

```json
{"task_id": "task-101", "decision_id": "dec-001", "resource_id": "A-ugv1",
 "verdict": "ACCEPT", "eta_sec": 436, "reason": null, "detail": null,
 "target_node": {"node_id": "494852", "lat": 38.043899, "lon": 128.202801, "snap_m": 55.6},
 "path": ["A", "495227", "...", "494852"]}
```

**판정 순서와 reason 코드**

| 순서 | reason | 조건 |
|---|---|---|
| 1 | `TARGET_UNREACHABLE` | 화재에서 가장 가까운 도로 노드가 `UGV_TARGET_SNAP_M` 보다 멂 |
| 2 | `BUSY` | 다른 task 주행 중 |
| 3 | `ROAD_BLOCKED` | 경로가 차단 도로 때문에 끊김. `detail` 에 원인 도로 id |
| 4 | `TARGET_UNREACHABLE` | 차단과 무관하게 도로망이 끊겨 있음 |

`UGV_APPROACH_FALLBACK=1` 이면 3·4 에서 바로 거절하지 않고, 화재에서 `UGV_APPROACH_MAX_M` 안의 다음 노드를 가까운 순으로 시도한다. 성공하면 `detail` 에 접근 거리를 적는다.

ETA 는 도로 길이 ÷ 도로등급별 속도(잠정: 고속국도 80, 일반국도 60, 지방도 50, 시군도 30 km/h) × 혼잡 배율이다. 거점 → 도로망 경계 구간(원통 3.2 km, 기린 2.1 km)은 포함하지 않는다.

## POST /ugv/{resource_id}/execute

Safety ALLOW 이후 호출한다. 주행을 시작하고 즉시 반환한다.
UAV 와 같이 **화재 좌표(`target`)** 를 받는다. 목적지 도로 노드는 evaluate 와 같은 규칙으로 다시 고르므로, evaluate 이후 도로가 막혔으면 여기서 거절된다.

**요청** — `target` 과 `target_node` 중 하나

```json
{"task_id": "task-101", "decision_id": "dec-001", "target": {"lat": 38.0444, "lon": 128.2028}}
```

`target_node` 는 노드 id 를 직접 지정할 때만 쓴다 (시험·시연용).

**응답 200**

```json
{"task_id": "task-101", "resource_id": "A-ugv1", "status": "STARTED",
 "tracking_url": "/ugv/A-ugv1/task/task-101",
 "target_node": {"node_id": "494852", "lat": 38.043899, "lon": 128.202801, "snap_m": 55.6},
 "eta_sec": 436}
```

| 코드 | 의미 |
|---|---|
| 404 | 자원 또는 도로 노드 없음 |
| 409 | 다른 task 수행 중, 같은 task 진행 중, 또는 판단 결과 REJECT (`detail` 에 reason) |
| 422 | `target`·`target_node` 둘 다 없음 |
| 500 | 드라이버가 주행을 시작하지 못함 (PX4 arm·미션 거부 등) |

## POST /ugv/{resource_id}/stop

주행을 멈춘다. 진행 중 task 는 `CANCELLED`. 멈춘 위치에서 가장 가까운 도로 노드에 선 것으로 보고 `READY` 로 돌린다. PX4: 미션 일시정지 → HOLD → disarm.

## GET /ugv/{resource_id}/task/{task_id}

| 필드 | 설명 |
|---|---|
| `status` | `STARTED` → `IN_PROGRESS` → `COMPLETED` / `FAILED` |
| `progress` | `{"phase": "ENROUTE" \| "ARRIVED", "waypoint": i, "total": n}` |
| `observation` | COMPLETED 일 때 `{"observation_type": "ROAD_STATUS", "arrived_node", "position", "fuel_pct"}` |
| `error` | FAILED 일 때 |

도착은 도로 노드 도착이다. **화재를 관측했다는 뜻이 아니다.**

## GET /ugv/{resource_id}/observation

주행하지 않고 현재 도로 상태를 돌려준다. 필드 이름은 공통 `Observation` 을 따른다 (`simulation_time_s` 는 호출측이 붙인다).

```json
{"resource_id": "A-ugv1", "location_lat": 38.09279, "location_lon": 128.179569,
 "observation_type": "ROAD_STATUS",
 "value": {"state": "READY", "current_node": "A", "current_task_id": null,
           "blocked_roads": 5, "burning_cells": 2}}
```

## POST /env/fire_cells

현재 BURNING 셀 **전체**(스냅샷)를 보낸다. 이전 화재 차단은 해제하고 다시 계산한다. 환경의 `get_fire_locations()` 결과를 그대로 넣어도 된다 (`x`, `y` 외 필드는 무시).

```json
{"cells": [{"x": 134, "y": 123}, {"x": 135, "y": 123}]}
```

차단 규칙: 도로가 지나는 격자 칸 또는 그 8방향 이웃이 BURNING 이면 그 도로는 차단. 환경 CA 에서 도로 칸은 대부분 연료 0 이라 직접 타지 않으므로 이웃까지 본다.

**응답** `{"burning_cells": 2, "blocked_roads": 5, "newly_blocked": [...], "cleared": [...]}`

이미 주행 중인 차량의 경로는 바꾸지 않는다.

## POST /env/congestion

환경에 혼잡 정보가 없으므로 시나리오용으로 직접 준다.

| 필드 | 기본 | 설명 |
|---|---|---|
| `preset` | `none` | `none`: 전부 1.0 / `random`: 시드 고정 랜덤 |
| `seed` | 0 | 같은 seed 면 같은 결과 |
| `ratio` | 0.2 | random: 혼잡 도로 비율 |
| `min_factor` / `max_factor` | 1.5 / 3.0 | random: 통과시간 배율 범위 |
| `roads` | null | `{road_id: 배율}` 직접 지정 (preset 적용 후 덮어씀) |

## 공통 커넥터 대응 (`connectors/ugv_connector.py`, real 모드)

루트 `config.py` 에서 `UGV_CONNECTION_MODE = "real"`, `UGV_SERVER_URL = "http://localhost:8100"`.

| 커넥터 함수 | 서버 호출 | 반환 |
|---|---|---|
| `get_ugv_status(base)` | `GET /ugv?base=` | `ResourceStatus` 목록. 서버 연결 실패 시 빈 목록 |
| `send_ugv_command(task_id, assignment)` | `POST /evaluate` (좌표) | `LocalResponse`. ETA·목적지 노드는 `counter_constraint` 에 임시로 실음 (공통 형식에 칸이 생기면 이동) |
| `get_ugv_observation(resource_id, ts, task_id, ...)` | ACCEPT 된 task: `POST /execute`(evaluate 때의 좌표) → `GET /task` 폴링 / 그 외: `GET /observation` | `Observation` (`ROAD_STATUS`) |

## 알려진 제한

- sim 주행 속도는 시연용 가속값(2,000 m/s). ETA 는 실제 도로 속도 기준.
- 주행 중 도로가 막혀도 경로를 다시 짜지 않는다.
- task 기록은 메모리에만 있다. 서버를 재시작하면 사라진다.
- 도로등급별 속도, 2,000 m 스냅 거리, 거점 위치는 잠정값이다 (팀 합의 필요).
