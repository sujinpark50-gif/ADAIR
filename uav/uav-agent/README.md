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
| `drone.py` | 상태 제공자. `MockDrone`(고정값), `Px4Drone`(MAVSDK 4.x) |
| `config.py` | 임계값. 전부 잠정값 |

## 환경변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `UAV_ID` | `A-uav1` | 이 프로세스가 담당하는 UAV. 경로의 `uav_id`가 다르면 404 |
| `UAV_MODE` | `mock` | `mock`: PX4 없이 고정값 / `real`: PX4 연동 |
| `PX4_ADDRESS` | `udpin://0.0.0.0:14540` | real 모드의 MAVSDK 연결 주소 (14550 아님) |

UAV 1대당 프로세스 1개를 띄우고, 호출측은 UAV별로 다른 주소(포트)를 씁니다.
mock은 한 서버에 여러 개 띄워도 됩니다. real은 현재 서버(2 vCPU)에서 1기만 안정성을 확인했습니다.

## 알려진 제약

- real 모드의 `failsafe`, `link_quality`, `current_task_id`, `wind_ms`는 고정값입니다.
- `current_task_id`가 execute 중에도 갱신되지 않아 `BUSY` 판정이 동작하지 않습니다.
- execute의 `COMPLETED`는 도착·관측 완료가 아닙니다. 관측값은 고정값입니다.
- 비행고도는 목표 지면 해발고도 + 50 m만 반영합니다. 경로 중간 지형은 고려하지 않습니다.
- 배터리 소모율 `BATTERY_DRAIN_PCT_S = 0.033`은 추정치이며 실측 보정 전입니다.
- Task 상태는 메모리에만 있어 재시작하면 사라집니다.
