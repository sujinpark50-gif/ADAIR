# ADAIR 멘토 패치 설명

| 묶음 | 요청 ID | 브랜치 커밋 |
|---|---|---|
| 1. 환경 계약 계층 | ENV-01~06 | `feat(env)` |
| 2. 실행 키·실행 기록 영속 | UAV-01 · UGV-02(영속) · UGV-04 · J3 | `feat(exec)` |

각 묶음은 기존 판단 로직을 갈아엎지 않고 계약 칸만 채운다. 시험은 저장소 루트에서:

```bash
python -m pytest orchestrator/tests environment/tests tests/test_exec_idempotency.py -q
# 356 passed, 8 skipped (기존 329 + 묶음1 20 + 묶음2 7)
```

---

# 1. 환경 계약 계층 (ENV-01~06)

총괄 요청서(`docs/common/ADAIR_총괄_팀별_요청서.md`)의 환경팀 P0 요청을 **김은주님 CA 코드를 고치지 않고** 감싸서 구현했다.
`grid.py`·`fire_model.py`·`api.py`·`risk.py`는 한 줄도 바뀌지 않았다. 환경팀이 검토 후 자기 코드로 흡수하거나 그대로 써도 된다.

## 무엇이 생겼나

| 파일 | 역할 |
|---|---|
| `environment/src/contract_env.py` | 계약 환경. READ/ADVANCE/APPLY 분리, run_id·버전·시각, 상태 영속·복구, 기상청 바람 구동, APPLY 중복·충돌 처리, 신고 |
| `environment/src/belief_analysis.py` | ENV-04 분석. 총괄의 belief 만 입력 → 위험 칸·확산 예측 |
| `environment/server.py` | 위 둘의 HTTP 서버 (포트 8300, 임시) |
| `environment/config/fire_stations.json` | 소방서 좌표 단일 출처 **후보** (INT-04) |
| `orchestrator/team_env.py` | 총괄 쪽 어댑터 (in-process / HTTP / 분석) |
| `orchestrator/api.py`, `config.py` | `ORCH_ENV_MODE` 추가. **기본값은 그대로 fixture** — 기존 동작·시험 불변 |
| `environment/tests/test_contract_env.py` | 요청서 '수락 시험' 칸을 옮긴 시험 17개 |
| `orchestrator/tests/test_team_env.py` | 총괄 ↔ 실제 CA 연동 시험 3개 (HTTP 포함) |

## 실행

```bash
# 환경 서버 (저장소 루트)
python -m uvicorn environment.server:app --port 8300
# 총괄을 그 환경에 붙이기
ORCH_ENV_MODE=team_http python -m orchestrator.api          # Windows: set ORCH_ENV_MODE=team_http
# 시험
python -m pytest environment/tests orchestrator/tests -q     # 349 passed, 8 skipped (기존 329 + 신규 20)
```

## 요청별 상태 (요청서 표기 L/M/T/C 기준)

| ID | 이전 | 지금 | 근거 |
|---|---|---|---|
| ENV-01 | C | **L·M** | READ 반복 시 버전 불변, ADVANCE 만 진행. 재시작 → 같은 run_id·시각·버전, RNG 까지 복구해 이어 돌린 결과가 재시작 안 한 경우와 동일. reset → 새 run_id |
| ENV-02 | C | **L** | `map_cells` 72,688칸(위치·크기·지면고도·건물유형·`human_exposure`). 주거 칸 41개를 보호대상으로. 소방서 2곳 |
| ENV-03 | C | **L** (지도 전체 값) | 진짜 세계 바람 = 기상청 인제(211) 시간자료. 14:45 → 5.7 m/s·160°, 15:01 → 6.3 m/s. CA 가 실제로 그 바람으로 돈다 |
| ENV-04 | C | **L** | `analyze(belief)` — 진짜 세계를 바꿔도 결과 불변(①), 신고 전엔 예측 없음(②), 현장 측정 유무로 예측 달라짐(③) |
| ENV-05 | C | **L·M** (J1·J2) | 같은 ID·같은 내용 → `duplicate=true`, 재시작 후에도 유지(sqlite). 같은 ID·다른 내용 → `OBSERVATION_ID_CONFLICT`. 다른 run → `RUN_MISMATCH`. NACK 도 멱등 |
| ENV-06 | C | **L** | `reports_until(t)` 에 `event_id`(내용 해시로 고정)·`run_id` |
| 총괄 연동 | M (FixtureEnv) | **M → T 준비** | 신고 → 최초 정찰 → 가짜 UAV → 모의 열화상 → 실제 환경 ACK → 목적 완료. 실제 uvicorn 서버로도 `team_http` 연결 확인 |

T(팀 서버 연동) 판정은 환경팀이 이 코드를 확인하고 공동 수락시험을 돌린 뒤에 한다 — 멘토가 만든 것만으로 T 로 적지 않는다.

## 가정값 (모두 `/health` 의 `assumptions` 에 노출)

- **tick_s = 60초** — CA 한 스텝이 시뮬레이션 몇 초인지 환경 모델에 정의가 없다. 환경팀이 정해야 한다.
- **scenario_start_kst = 2019-04-04 14:45 KST**, 점화 칸 `19_142`, 신고 = 점화 칸·0초 — 요청서도 "시험값, 백서 대조 필요"로 적은 값.
- **vertical_datum = `DEM_AMSL_UNVERIFIED`** — DEM 수직 기준을 확인하지 못했다.
- 진짜 세계 snapshot 의 `risk_cells` = 화재 칸 주변 3칸의 UNBURNED 칸. **모의 센서가 '불 없음'을 볼 수 있게 하는 용도**이고, 총괄 판단 화면(`view()`)에는 들어가지 않는다 (시험으로 확인).
- APPLY 는 **기록만** 한다. 관측으로 진짜 화재 상태를 바꾸지 않는다 (요청서 금지 사항). 기존 `ObservationUpdateHandler` 는 BURNING 을 직접 쓰므로 계약 경로에서 부르지 않았다.

## 솔직한 한계

- **ENV-04 예측은 과소 예측이다.** 같은 CA 확산식을 쓰되 "절반의 경우 번지는 대기 시간"으로 도달 시각을 낸다.
  30분 시험에서 실제 CA 화재 칸 대비 재현율 0.45 / 정밀도 0.99 (평균 대기식을 쓰면 재현율 0.16). 여러 불 칸이
  동시에 번지는 효과를 넣지 않았기 때문이다. 보정은 환경팀 몫 — `environment/src/benchmark.py` 와 엮으면 된다.
- **칸별 바람장(양간지풍)은 없다.** 지도 전체 값 하나 (`weather.scope = GLOBAL_FIELD`).
- **INT-04 는 끝나지 않았다.** `fire_stations.json` 은 루트 `config.py` 값을 옮긴 후보이고, `uav/NOTE.md` 의 다른 좌표를
  `alternatives` 로 남겼다. 팀이 하나를 확정해야 한다.
- **INT-05(시계 소유자)** 는 합의 사항이다. 서버는 `/advance` 를 누가 부르든 받는다. 웹(`/api/env`)은 아직 자기 CA 를 따로 돌린다.
- `map_cells` 는 약 10 MB 로 처음 한 번만 받고 캐시한다 (`map_version` 이 같으면 재요청 없음).

---

# 2. 실행 키·실행 기록 영속 (UAV-01 · UGV-02 · UGV-04 · J3)

## 왜

UAV Agent·UGV 서버는 실행 기록을 메모리(`_tasks`)에만 들고 있었다. 서버가 한 번 재시작하면

- 총괄이 `/task/{attempt_id}` 를 조회 → **404** → 총괄은 `UNKNOWN`(점유 유지·수동 해소 대기)로 멈춘다.
- 응답을 못 받은 출동 요청을 누가 재전송하면 **두 번째 물리 출동**이 나간다 (UGV 는 끝난 task_id 를 덮어썼다).

원본 main 에 같은 시험을 돌려 확인했다: 재시작 뒤 조회 `404`, 총괄 실행시도 `UNKNOWN`.

## 무엇을 바꿨나

| 파일 | 변경 |
|---|---|
| `interfaces/exec_store.py` (신규) | UAV·UGV 공용 실행 기록. JSON 원자 저장, 실행 키 검사(NEW/DUPLICATE/CONFLICT), 재시작 정리 |
| `uav/uav-agent/main.py`, `models.py` | 실행 키 규칙·영속·재시작 정리. `TaskStatus.uav_id`(UAV-01 ④), `ExecuteResponse.duplicate`, `agent_restart` |
| `ugv/server.py`, `api_models.py` | 같은 규칙. 수행 중 같은 키 재요청은 처음 응답 그대로(`target_node` 포함) |
| `integration/uav_launcher.py` | 통합 실행마다 새 상태 폴더 (`logs/uav_state/<시각>`) |
| `connectors/uav_connector.py`, `ugv_connector.py` | 구 커넥터 실행 키 = `프로세스표식:task_id:decision_id` (TASK_001 이 실행마다 반복되므로) |

## 규칙 (UAV·UGV 동일)

| 요청 | 응답 |
|---|---|
| 새 키 | 200 `duplicate=false`, 기록을 **출동 전에** 저장 |
| 같은 키·같은 내용 (수행 중이든 끝났든, 재시작 뒤든) | 200 `duplicate=true` — 새로 출동하지 않음 |
| 같은 키·다른 내용 | 409 `EXECUTION_ID_CONFLICT` — 기존 실행 유지 |
| 출동 전 거절 (400 고도 없음, 409 도로 불가 등) | 키를 쓰지 않는다 → 고쳐서 같은 키로 재전송 가능 |

실행 키 = 요청의 `task_id`. 총괄은 이미 이 칸에 실행시도 ID 를 넣고 있어 **총괄 코드는 바꿀 필요가 없다**.
UAV 는 `execution_attempt_id` 칸도 받지만 `task_id` 와 같아야 한다 (키는 하나).

## 재시작으로 끊긴 실행의 정리

| 끊긴 시점 | UAV mock | UAV real | UGV |
|---|---|---|---|
| 관측(도착) 전 | `FAILED`, `AGENT_RESTARTED_BEFORE_OBSERVATION`, 관측 비움, phase `DONE` | 같지만 phase `AGENT_RESTARTED` | `FAILED`, phase `FAILED` |
| 관측 뒤 복귀 중 | `COMPLETED` 유지, **관측 그대로**, phase `DONE` | 관측 그대로, phase `AGENT_RESTARTED` | (도착 = 완료라 해당 없음) |

- mock 은 재시작하면 MockDrone 이 기지에 서 있으므로 물리 상태를 안다 → `DONE`. real 은 기체가 아직 떠 있을 수 있어
  **모른다고 보고한다** (`physical_basis=PHYSICAL_STATE_UNKNOWN_AFTER_RESTART`). 총괄은 이때 점유를 유지하고 사람이 해소한다.
- 총괄 쪽 결과 (J3 시험): 관측 전 끊김 → 기체 READY 확인 후 반납, 목적은 `PENDING`(인계 대기), 자동 재전송 0회.

## 한계

- **물리 명령 '최대 1회'는 기록 저장과 출동 시작 사이의 아주 짧은 틈까지는 보장하지 못한다.** 기록을 먼저 저장하므로
  그 틈에 죽으면 '출동 안 했는데 끊김으로 정리'되는 쪽으로 틀린다 (중복 출동 쪽이 아님). 의도한 방향이다.
- UGV px4 드라이버 재시작 뒤 실제 차량 위치는 텔레메트리로 다시 읽는다. 끊긴 주행을 이어서 감시하지는 않는다.
- UAV-09(중단 API)·UAV-10(관측 완료와 물리 실패 분리 칸)은 이번 범위가 아니다. 재시작 정리에서 관측 보존 규칙만 먼저 맞췄다.
- 발견 사항 (이번 변경과 무관, main 에서도 같음): `python run_integrated.py` 의 폐루프 4스텝이 모두 `FAILED` 다.
  점화 칸(19,142)이 A 기지에서 약 15 km 라 UAV 가 `LOW_BATTERY` 로 거절한다. 시연용 점화 위치나 기지 배치를 다시 볼 필요가 있다.
- INT-04 추가 근거: UGV 서버의 A 거점 위치(38.0928, 128.1796)는 위 세 곳과도 또 다르다.
