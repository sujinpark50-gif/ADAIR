# ADAIR 총괄 오케스트레이터 — 구현 준비 기록

- 작성: 2026-09-30
- 기준 요청서: `docs/orchestration/ADAIR_오케스트레이터_구현요청_코드대조_수정본 (1).md` (이하 "요청서")
- 성격: 요청서 §0 시작 절차의 기록과 구현 설계안(§0~§6), 그리고 구현 결과 현황(아래 "구현 현황").
- 팀별 요청: `docs/common/ADAIR_총괄_팀별_요청서.md`

## 구현 현황 (2026-09-30 기준, 피드백 반영 재분류)

분류: **A** 코드와 시험으로 확인 · **B** fixture 에서만 확인 · **C** 타 팀 계약·연동 대기 · **D** 최종 발표 미완료.
건너뛴 시험(실제 서버·실제 API 호출)은 통과로 세지 않는다. 모의 센서·가짜 환경 ACK 로 끝난 Task 완료는 **시뮬레이션 시험 완료**이며 실제 센서 관측·실제 환경 연동 완료가 아니다.

| 기능 | 분류 | 근거·한계 |
| --- | --- | --- |
| SQLite 장부: 멱등 접수, 원자 예약, 송신 전 선저장 | A | `test_ledger`, `test_flow` |
| PREPARED 재시작 복구 (송신 증명 불가 → UNKNOWN·점유 유지, 제공자가 같은 시도 보고 시 재개) | A | `test_recovery` 5건. **물리 명령 최대 1회는 보장하지 않음** (C, UAV-01) |
| 물리 사전필터, 규칙 Safety | A | `test_flow`. 기체 제원·datum 출처는 C |
| 출동·불명·고장·수동 해소, 복귀·READY 확인 후 반납 | A | `test_flow`, `test_live_*`(실행 시에만) |
| UAV·UGV HTTP 계약 | A (mock/sim) | UAV Agent `UAV_MODE=mock`, UGV 서버 `UGV_DRIVER=sim` 대상 시험. PX4/Gazebo 실비행은 D |
| COUNTER 3A 규칙 | A / C | 실제 UAV COUNTER 는 실을 칸이 없어 실행 불가 (UAV-02) |
| 정보 경계: 판단 화면은 신고·정찰·관측·현장 모의 측정만 | A | `test_info_boundary`, `test_knowledge`. 2026-09-30 이전 `FixtureAnalysis` 가 진짜 위험·확산을 복사하던 누출을 수정함 |
| 위험 칸·확산 예측 값 | B | `InjectedAnalysis` 시험 주입값 (`TEST_INJECTED`, 관측으로 계산한 예측 아님). 산출은 ENV-04 (C) |
| 우선순위 규칙 (인명 우선·비슷함 0.1·엇갈림·두 모드) | A (규칙) / B (데이터) | 위험 칸·보호대상은 주입·fixture |
| LLM 제안 + 결정론적 검증·fallback | A | `test_llm`(가짜 모델). 실제 gpt-5.6-luna 호출은 `ORCH_LIVE_LLM=1` 로 1회 확인(기본 시험에서는 건너뜀) |
| 기상청 ASOS 시간자료 재생 (그 시각까지만) | A | `test_knowledge`, `test_info_boundary` |
| 현장 환경 측정 + 모의 기상 센서 | B | `SIMULATED / TEST_ONLY`. 환경 기상이 지도 전체 한 값이면 `GLOBAL_FIELD` 로 표시, 공간 차이를 만들지 않음. 실제 센서 C |
| Task 완료 | B | `evidence_level=SIMULATION_TEST` (모의 센서 + FixtureEnv ACK) |
| 판단 화면 `/board` | A (총괄 내장) | 기존 웹관제 연결은 C (INT-01~03) |
| 사전 감시 임무 생성 | A (꺼 둠) | `ORCH_PREEMPTIVE_MONITOR=1`. 입력 예측이 주입값이라 실효는 B |
| 환경 연결 | B | `build_default()` 는 `FixtureEnv`. 팀 공유 환경 실행 아님 (`/health` `truth_env.shared_team_env=false`). ENV-01·05 는 C |
| 웹관제 경유·도착 즉시 소화 제거 | C | 통합 코드 수정 필요 (INT-01, INT-02 요청서에 현 코드·이유·계약·영향·수락 시험 기재) |
| F1 2026 공식 출동 기준, F2 진화 효과 | D | 원문·팀 계약·연동 시험 없음 |

### 실행 방법

```powershell
# 저장소 루트, 가상환경 .venv (fastapi, uvicorn, httpx, openai, pytest)
.\.venv\Scripts\python -m pytest orchestrator/tests -q            # 단위 시험 (실제 서버·API 시험은 건너뜀)
.\.venv\Scripts\python -m orchestrator.api                          # 총괄 서버 127.0.0.1:8200 → /board
# 선택: ORCH_ENV_FIXTURE=시나리오.json, ORCH_PREEMPTIVE_MONITOR=1, ORCH_PRIORITY_MODE=AUTO_HIGHER_RISK
# 실제 서버 시험: ORCH_LIVE_UAV=1 / ORCH_LIVE_UGV=1 / ORCH_LIVE_LLM=1 (각 서버·키 필요)
```

비밀값은 `.env`(git 제외): `OPENAI_API_KEY`, `DATA_GO_KR_SERVICE_KEY`. 형식은 `.env.example`.

---

아래 §0~§6 은 구현 시작 전(2026-09-30 오전) 기록이다.

## 0. 시작 절차 기록 (요청서 §0)

| 항목 | 값 |
| --- | --- |
| 작업 브랜치 | `feat/orchestrator-final` |
| 브랜치 구성 | `origin/feat/uav-agent`(`6b052a0`) + `origin/main`(`979d8b2`, 문서) + `origin/fix/integration-restore`(`1ac2ca9`, 환경 `src` 평탄화) 병합 |
| 이유 | `origin/main`은 문서만 있고 통합 코드보다 29커밋 뒤처짐. 요청서가 기준으로 삼은 UAV 코드 `6b052a0`은 `feat/uav-agent`에만 있음 |
| 미커밋 파일 | 요청서 1건(untracked). 원격 내용 덮어쓰기·삭제 없음 |
| 기존 총괄 코드 | `orchestrator_v012/`(PR #7로 병합, 지난주 기본형). **수정하지 않고 보존**, 신규 패키지에서 개념만 재사용 |
| 로컬 Python | 3.14 (orchestrator_v012는 3.12에서만 검증됨). fastapi 미설치 → 가상환경 필요 |

### 요청서 대비 코드에서 재확인한 사실

- UAV `Target`은 `alt_m_amsl`(지면 해발, 없으면 거절)과 `target_agl_m`(기본 80)을 받고, 비행고도 = 지면 + AGL + 10 (`uav/uav-agent/models.py`, `config.py`). **요청서 §4.1과 일치.**
- UAV 실행 상태는 프로세스 메모리(`_tasks`)에만 있음. 재시작 시 조회 404. 실행 중 중복 요청은 409. idempotency key 없음 (`uav/uav-agent/main.py`).
- UAV 관측값은 고정 `312.5℃ / hotspot_detected=True` (`main.py` `_run_task`). `COMPLETED` 시점에 `progress.phase=RETURNING`.
- 환경 `EnvironmentModelAPI`는 `state_version`을 갖지만 `step()`과 `apply_observation()` 모두 버전을 올리고, `simulation_time`·`run_id`·`map_version`이 없음 (`environment/src/api.py`). HTTP API 없음.
- **포트 충돌:** UGV 서버가 이미 8100을 씀 (`ugv/server.py`, `config.UGV_SERVER_URL`). 총괄 포트는 설정값으로 둔다.
- UGV 서버 API: `/ugv/{id}/state|evaluate|execute`, 응답 `verdict/eta_sec/reason/target_node/path` (`ugv/api_models.py`). COUNTER 없음.

## 1. 패키지 구성 (신규 `orchestrator/`)

```
orchestrator/
  config.py        모든 수치·주소·모드. 값마다 출처와 상태(PROVISIONAL/TEST_ONLY/TBD) 표기
  models.py        Task(목적 상태) / ExecutionAttempt(실행 substatus) / Reservation / HoldReason / Snapshot
  ledger.py        SQLite 장부: task, decision, attempt, reservation, event, idempotency. 원자 예약
  env_adapter.py   READ/ADVANCE/APPLY 경계. FixtureEnv(시험) / InProcessEnv(현 환경코드, 읽기 전용 조회)
  resources.py     UAV/UGV 어댑터: state·evaluate·execute·status 조회, 원문 보존, 404→UNKNOWN
  prefilter.py     Local 평가 전 물리 필터 (capability, READY/미점유, 신선도, 필수 입력)
  priority.py      위험 셀 + 보호대상 거리 계산, 확인된 입력만으로 결정론적 순서, 충돌 시 HOLD
  negotiation.py   ACCEPT/REJECT/COUNTER 처리, 구체 수정필드만 재평가, 예산 2회
  safety.py        규칙 Safety (요청서 §5-3 항목), 정보 누락 시 ALLOW 금지
  executor.py      attempt 발급→Safety→예약·선저장→1회 송신→상태 대조, UNKNOWN 점유 유지
  observation.py   도착 확인 후 모의 관측(TEST_THERMAL_50M_V1, SIMULATED), 환경 APPLY·ACK
  llm.py           OpenAI 계획 제안 + 결정론적 validator + fallback/HOLD
  engine.py        폐루프: snapshot→Task 갱신→우선순위→후보→협상→Safety→실행→관측→재판단
  api.py           FastAPI: health, tasks, events, state, decisions, attempts, resolve
  tests/           총괄 단독 시험 (요청서 §12 목록)
```

기존 `connectors/`, `interfaces/schema.py`, `uav/`, `ugv/`, `environment/`, `web/`은 수정하지 않는다. 공통 `TaskState`는 재정의하지 않고 내부 substatus를 매핑한다(§3).

## 2. 기능별 계획과 상태 (요청서 §0 형식, §12 분류)

분류: **A** 구현 완료 · **B** fixture로만 시험 · **C** 팀 계약 대기 · **D** 최종 발표 미완료

| 기능 | 문서 목표 | 현재 코드 | 이번 총괄 작업 | 타 팀 계약 | 목표 상태 |
| --- | --- | --- | --- | --- | --- |
| 영속 장부·상태 전이 | Task/decision/attempt/예약 분리 저장, 재시작 복구 | 없음(v012는 메모리) | SQLite 장부, 원자 예약, 재시작 시 UNKNOWN 재조회 | — | A |
| 내부 중복 억제 | 같은 접수 1회 수렴 | 없음 | 요청 ID + 내용 해시, 다른 내용이면 409 | 물리 1회 보장은 UAV idempotency 필요 | A(내부) / C(물리) |
| 환경 READ/ADVANCE/APPLY | 단일 run 원자적 snapshot | 조회가 step 유발 | 어댑터 경계 + FixtureEnv, InProcessEnv는 step 호출 금지 | 환경·통합: run_id, simulation_time, map_version, 풍속 유효시각 | B / C |
| 물리 사전필터 | Local 전 불가능 기체 제외 | v012 일부 | capability·READY·점유·신선도(10s/30s 잠정)·필수입력 | 자원 registry·제원 출처 | A |
| UAV 단일목표 요청 | `alt_m_amsl`=지면, `target_agl_m` 명시, `wind_ms` snapshot | v012는 AGL 미전달 | 어댑터가 AGL 명시, 80/10 가산 금지 시험 | — | A (mock UAV 서버로 시험) |
| UAV route 추적 | route id/version/hash 관리 | UAV route 미구현 | route 필드 자리·hash 일치 검사, 없으면 단일목표 경로로만 | UAV: route schema, DEM 계약 | C |
| COUNTER | 구체 수정필드만 재평가 | v012는 MODERATE_WIND 수용 | 수정필드 화이트리스트, note-only 차단, 예산 2 | UAV: 수정 필드 정의(현재 `observe_duration_s`는 요청에 넣을 칸 없음) | A(규칙) / C(실효) |
| Safety | 요청서 §5-3 | 무조건 ALLOW mock | 규칙 Safety 신규, 누락 시 REJECT | datum·물리한계 출처 | A(규칙) |
| 실행·UNKNOWN | 선저장·1회 송신·불명 시 점유 유지 | v012 일부 | executor + 재전송 금지 + 수동 해소 | UAV 상태 영속 조회 | A |
| 도착·관측·ACK 분리 | 이벤트 분리, mock 관측 불인정 | 고정값을 근거로 사용 | 도착 확인 후 TEST_ONLY 센서로 모의 관측, APPLY ACK | UAV 도착 근거, 센서 profile | B |
| 복귀·READY 반납 | COMPLETED≠반납 | 없음 | phase=DONE/READY 확인 후 해제 | — | A |
| 우선순위 | 위험 셀 + 보호대상 경계 거리, 임의 가중치 금지 | 없음 | 거리 계산·근거 기록, 비교규칙 없으면 HOLD | 환경: 위험 셀 ~10칸, 보호대상 registry | B / C |
| 두 시계 | 실제 timeout vs simulation deadline | UAV가 UTC로 deadline 비교 | deadline 미전송, ETA는 소요시간으로 기록 | UAV: remaining_time_s 등 | A / C |
| LLM | 계획 제안 + validator + fallback | 없음 | OpenAI 어댑터, 모델·timeout·한도 설정값, 키 없으면 규칙 | — | B(키 확보 전) |
| 총괄 API | health/접수/event/조회/해소 | 없음 | FastAPI, 포트 설정값 | 통합: 웹 호출 전환 | A(API) / C(웹) |
| F1 공식 기준 | 2026 시행 기준 매핑 | 없음 | `official_rule_source/version/effective_date` 자리만 | 자료 확보 | D |
| F2 진화 효과 | 진화 행동→환경 효과→재관측 | 웹이 즉시 UNBURNED | 계약 자리만, 웹 즉시 소화 경로 호출 금지 | 환경·통합 | D |

## 3. 상태 매핑 (공통 TaskState 유지)

| 공통 TaskState | 총괄 내부 Task 목적 상태 | 실행 substatus |
| --- | --- | --- |
| READY | PENDING, HOLD(사유·재개조건), EVALUATING | — |
| ASSIGNED | APPROVED(Safety ALLOW, 예약) | PREPARED(미전송) |
| RUNNING | IN_EXECUTION | REQUESTED, STARTED, ARRIVED, OBSERVED, APPLIED, UNKNOWN |
| COMPLETED | 목적조건 충족(+필요 시 환경 ACK) | — (자원 해제는 READY/반납 확인 후 별도) |
| FAILED | 실패 확인 | FAILED(확인된 경우만) |
| CANCELLED | 사용자 취소 | CANCELLED |

예약 해제: Local REJECT(예약 전), Safety 비ALLOW(송신 전), 송신 전 저장 실패, 자원 READY/반납 확인, 수동 해소. **timeout·UNKNOWN·OFFLINE은 해제하지 않는다.**

## 4. 설정값 (전부 `orchestrator/config.py`, 운영 정책 아님)

| 키 | 값 | 상태 | 출처 |
| --- | --- | --- | --- |
| `COUNTER_ROUNDS_PER_RESOURCE` | 2 | PROVISIONAL | 사용자 결정, 기존 `MAX_REEVALUATION_RETRIES` |
| `RESOURCE_STALE_S` / `RESOURCE_OFFLINE_S` | 10 / 30 (실제 시계) | PROVISIONAL | 사용자 결정 |
| `DEFAULT_TARGET_AGL_M` | 명시 필수, 미지정 시 80 | UAV 코드 기본 | `uav/uav-agent/config.py` |
| `SENSOR_PROFILE` | TEST_THERMAL_50M_V1 (30×25m) | TEST_ONLY | 요청서 §8 |
| `LLM_PROVIDER` / `LLM_MODEL` / `LLM_TIMEOUT_S` / 호출·비용 한도 | openai / TBD / TBD / TBD | TBD | 요청서 §9: 임의 결정 금지 |
| `ORCH_HOST` / `ORCH_PORT` | TBD (8100은 UGV 사용 중) | TBD | 통합과 협의 |
| 소방서·지도 | 환경 map metadata에서 읽음 | 새 맵 대기 | 요청서 §3 |

## 5. 구현 순서

1. 가상환경·의존성, `config`/`models`/`ledger` + 장부 시험
2. `env_adapter`(Fixture), `resources`(UAV/UGV mock 서버 대상), `prefilter`
3. `negotiation`, `safety`, `executor` → 시나리오 1(정상 정찰)·3(강풍 실패 후 대체/보류) 폐루프
4. `observation`(도착 후 모의 관측, APPLY ACK), 복귀·READY 반납
5. `priority`(fixture 지도: 위험 셀 10칸 + 보호대상)
6. `api`(FastAPI)
7. `llm`(키 확보 시 실연결, 아니면 validator·fallback만)
8. 요청서 §12 시험 목록 실행 후 A/B/C/D 보고

## 6. 사용자·팀 확인 필요

- 총괄 서버 포트 (8100은 UGV 사용 중)
- OpenAI 모델명·timeout·호출/비용 한도 (요청서 §9: 기본값 임의 결정 금지)
- 팀 요청서 발송 (요청서 §11): 환경 READ/ADVANCE/APPLY·위험 셀·보호대상 registry, UAV idempotency·route·COUNTER 필드·simulation deadline, 통합 웹 호출 전환·즉시 소화 경로
