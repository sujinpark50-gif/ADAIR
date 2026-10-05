# UAV Local Agent

UAV 상태 제공과 수행 가능성 판단(ACCEPT / REJECT / COUNTER)을 하는 FastAPI 서버입니다.
팀 코드는 `connectors/uav_connector.py`에서 HTTP로 호출합니다.

- 실행 방법: [../README.md](../README.md)
- API 사양: [../API_DEFINE.md](../API_DEFINE.md)
- 팀 합의 정책: [docs/uav/uav_final](../../docs/uav/uav_final)

## 파일 구성

| 파일 | 역할 |
|---|---|
| `main.py` | REST 엔드포인트, execute 비동기 실행 |
| `models.py` | 요청·응답 스키마 (pydantic) |
| `evaluator.py` | 판단 로직. 판단만 하고 실행하지 않음 |
| `route.py` | 경로 계획(DEM 순항고도)·경로 해시·평가 경로 저장 (UAV-03) |
| `drone.py` | 상태 제공자. `MockDrone`(고정값), `Px4Drone`(MAVSDK 4.x) |
| `config.py` | 임계값. 전부 잠정값 |

## 환경변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `UAV_ID` | `A-uav1` | 이 프로세스가 담당하는 UAV. 경로의 `uav_id`가 다르면 404 |
| `UAV_MODE` | `mock` | `mock`: PX4 없이 고정값 / `real`: PX4 연동 |
| `PX4_ADDRESS` | `udpin://0.0.0.0:14540` | real 모드의 MAVSDK 연결 주소 (14550 아님) |
| `UAV_STATE_DIR` | `uav-agent/.state` | 실행 기록·평가 경로 파일 위치 (재시작 뒤 조회용) |
| `UAV_RETASK_WHILE_RETURNING` | `0` | 기본은 복귀 중 새 임무를 착륙까지 `BUSY` 로 거절 (총괄 방침, UAV-06). `1` 이면 복귀를 멈추고 새 임무로 출발 |
| `UAV_ROUTE_DEM` | `1` | `0` 이면 DEM 을 보지 않고 관측고도로만 비행 (경로 `terrain.status=UNVERIFIED`) |
| `UAV_REQUIRE_ROUTE_HASH` | `0` | `1` 이면 실행 요청에 평가의 `route_id`·`route_hash` 가 반드시 있어야 함 |
| `UAV_PLACEHOLDER_THERMAL` | mock `1` / real `0` | 열화상 자리표시값(312.5℃/hotspot)을 채울지. 채워도 `value_status=SIMULATED` |

UAV 1대당 프로세스 1개를 띄우고, 호출측은 UAV별로 다른 주소(포트)를 씁니다.
mock은 한 서버에 여러 개 띄워도 됩니다. real은 현재 서버(2 vCPU)에서 1기만 안정성을 확인했습니다.

## 알려진 제약

- real 모드의 `failsafe`, `link_quality`, `wind_ms`는 고정값입니다.
- 실제 열화상·기상 센서 처리가 없습니다. 관측값은 모의값(`value_status=SIMULATED`)이거나 비어 있습니다(`NO_DATA`).
- 경로는 지형 최고점 위 단일 순항고도입니다(구간별 지형 추종 아님). 원통 기지 주변은 DEM 격자 밖이라 그 구간은 확인하지 못합니다(`terrain.status=PARTIAL`).
- real 모드의 "고도 먼저 맞추고 수평 이동"(`Px4Drone.goto`)과 복귀 순항고도는 PX4/Gazebo 에서 아직 비행 확인하지 않았습니다 (mock 으로만 시험).
- real 모드 임무는 실제 시간이 걸립니다(편도 10 km ≈ 20분). 통합 커넥터는 30초만 기다립니다.
- 배터리 소모율 `BATTERY_DRAIN_PCT_S = 0.033`은 추정치이며 실측 보정 전입니다.
- 시뮬레이션 기체(PX4 Gazebo 기본 x500)에는 카메라가 없습니다 (`config.CAMERA`, UAV-08). THERMAL·RGB 관측값은 실측이 아닙니다.

## 시험

```bash
.venv/bin/python -m pytest tests/test_exec_idempotency.py tests/test_uav_requests.py -q   # 저장소 루트 .venv
```
