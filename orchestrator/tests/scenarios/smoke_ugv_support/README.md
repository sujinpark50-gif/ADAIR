# 시연 시나리오: 연기로 드론 관측 불가 → 지상 로봇(UGV) 출동

연기 때문에 드론으로 관측할 수 없는 상황을 가정해, 총괄에 **UGV 만, 관측 없음** 임무를 보낸다.
총괄은 UGV 를 골라 목표(2019 인제 발화점) 옆 도로까지 보내고, 도착하면 임무를 완료로 닫는다.

| 파일 | 내용 |
|---|---|
| `fixture.json` | 시험 환경. 2019 인제 발화점(남면 남전리, 38.02845, 128.1309) 한 칸이 타는 중 |
| `task.json` | 총괄에 보낼 임무. `GROUND_SUPPORT`, `resource_types: ["UGV"]`, `sensor: null` |

## 실행 (저장소 루트, Git Bash)

```bash
# 1) UGV 서버 — 시뮬레이션 주행, 50배속
UGV_DRIVER=sim UGV_TIME_SCALE=50 python -m uvicorn ugv.server:app --host 127.0.0.1 --port 8100

# 2) 다른 창: 총괄 — 시험 환경 지정, 기록 DB 는 따로
ORCH_ENV_FIXTURE=orchestrator/tests/scenarios/smoke_ugv_support/fixture.json \
ORCH_DB_PATH=logs/orchestrator/smoke_ugv_support.sqlite3 python -m orchestrator.api

# 3) 또 다른 창: 임무 보내기
curl -X POST http://127.0.0.1:8200/tasks -H "Content-Type: application/json" \
     --data-binary @orchestrator/tests/scenarios/smoke_ugv_support/task.json
```

PowerShell 에서는 환경변수를 먼저 지정한다: `$env:UGV_DRIVER="sim"; $env:UGV_TIME_SCALE="50"` 후 실행.

총괄 판단 화면 http://127.0.0.1:8200/board 에서 지켜본다.

## 기대 결과 (2026-10-05, main 0a8ef68 에서 실제 UGV sim 서버로 확인)

1. 후보: A-ugv1(직선 약 5.0 km), B-ugv1(약 17.9 km). 소방차 A-fire1 은 `TYPE_NOT_ALLOWED` 로 제외
2. A-ugv1 이 ACCEPT (ETA 약 2758 시뮬레이션 초), Safety ALLOW, 출동
3. 50배속에서 실제 약 50초 뒤 발화점에서 41 m 떨어진 도로 노드 494959 에 도착
4. 임무 `COMPLETED` (근거 ARRIVED / UGV_ROAD_STATUS / NO_ENV_ACK_REQUIRED, 수준 SIMULATION_TEST)

## 주의

- **main 에서 돌린다.** 2019 인제 도로망(UGV)이 main 에 있다.
- **"연기 때문"은 시스템에 기록되지 않는다.** 총괄은 "UGV 만, 관측 없음" 지정만 보고 움직인다.
- **목표는 도로에서 2 km 이내여야 한다.** 더 멀면 UGV 가 `TARGET_UNREACHABLE` 로 거절하고 임무는 대기(HOLD)한다.
- **짐(소방물품)은 싣지 않는다.** 총괄은 아직 UGV 에 cargo 를 보내지 않는다 — 목표까지 차량을 보내는 시연이다.
- 같은 `request_id` 로 다시 보내면 같은 임무로 처리된다. 다시 시연하려면 총괄을 재시작하거나 `request_id` 를 바꾼다.
