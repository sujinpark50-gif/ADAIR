# ADAIR 총괄 오케스트레이터 — 구현 준비 기록

- 작성: 2026-09-30
- 기준 요청서: `docs/orchestration/ADAIR_오케스트레이터_구현요청_코드대조_수정본 (1).md` (이하 "요청서")
- 성격: 구현 결과 현황(아래 "구현 현황", 최신)과 요청서 §0 시작 절차·초기 설계안(§0~§6, 구현 전 기록).
- 팀별 요청: `docs/common/ADAIR_총괄_팀별_요청서.md`

## 검토 반영 (2026-09-30, 기준 `747d74a` 위 작업본)

검토에서 확인된 결함 B01~B08 과 승인된 설계 D01~D03 을 코드·회귀시험에 반영했다. 모든 시험은 fixture·가짜 자원 기준이며 실제 비행·센서 검증이 아니다.

**시험:** `python -m pytest orchestrator/tests -q` → **329 통과, 8 건너뜀** (건너뜀: 실제 서버·API 가 필요한 `test_live_uav` 3, `test_live_ugv` 4, `test_live_llm` 1 — 이번에는 실행하지 않음). `d2e1c80` 시점은 213 통과.

### 추가 수정 R01~R04 (검증 기준 `d2e1c80`)

| 항목 | 반영 내용 | 위치 | 시험 |
| --- | --- | --- | --- |
| R01 인계 뒤 이전 시도 | Task 에 목적 담당 시도(`owner_attempt_id`)를 장부에 기록. 실행시도에서 온 사건은 담당 시도일 때만 목적 상태를 바꾼다. 인계는 "임무 종료 근거"(`detail.mission_end`: 제공자의 실패 확정 또는 도착·관측 완료)가 있고 불명이 아닐 때만 허용 — RETURNING 이라는 이름만으로는 허용하지 않는다 | `engine._set_purpose`·`_owns`·`_track`, `ledger.open_mission_attempts` | `test_followup_r01_r04` |
| R02 부분 음성 관측 | `fire_states` 가 "현재 믿음"을 계산: 칸 일부만 보고 불을 못 본 관측은 기존 신고·확인을 해소하지 않고 `last_partial_observation`(본 비율·시각·출처)로 덧붙는다. 칸 전체 음성(`OBSERVED_CLEAR`)만 해소. 화면·API·분석 입력이 같은 함수 사용 | `knowledge.fire_states` | 같음 |
| R03 비정상 응답 | `status` 가 문자열인지 먼저 검사, `progress.phase`·`observation.position`·`reason`/`error` 자료형 검사 → UNKNOWN 격리. poll 은 한 시도의 내부 오류를 기록(`TRACK_ERROR`)하고 나머지를 다 추적한 뒤 다시 던진다 | `engine._response_problem`·`poll` | 같음, `test_state_transitions` |
| R04 환경 반영 대기 | 응답 없는 APPLY 는 `pending_applies` 표에 영속 보관. 기체는 정상 반납, 목적은 `IN_EXECUTION`(사유 `ENV_APPLY_PENDING`)으로 남아 재출동하지 않음. poll 마다 같은 관측 ID 로 재반영. 거절(NACK)은 대기로 남기지 않고 재관측 대기 | `engine._conclude`·`retry_pending_applies`, `ledger.*pending_apply*` | 같음 |

### 총괄 내부 과제 1~3 (2026-10-01)

재검증(`7712e8a`) 보완: 자동 배정 전용 작업자 스레드(추적 루프는 부탁만), `_set_purpose` 저장 경합을 거절로 처리하고 송신 전 실패 시 예약 해제, 후속 측정 계획을 처음 한 번 확정해 저장, run 전환 시 이전 run 후속 작업 종료·수행 직전 run 재확인. 검토자 재현 4개는 `test_followup_7712_edges.py`.

| 과제 | 반영 | 위치 |
| --- | --- | --- |
| 1 자동 후속 측정 임무 복구 | 관측 저장 트랜잭션에서 `followups` 기록, 관측 직후·poll·recover 에서 수행, 계획 기록·종료 표시 동시 저장 | `engine.run_followups`·`_auto_sense`, `ledger.*followup*` |
| 2 느린 요청 중 추적 지연 | API 잠금 분리: `lock`(추적·접수·취소) / `dispatch_gate`(배정·LLM). 배정은 `lock` 없이 실행. 송신 중 시도는 추적 제외(`_sending`), 평가 도중 취소되면 송신 안 함, 담당 지정은 `prepare_attempt` 트랜잭션에서. 배정 중 상태가 바뀌면 `DISPATCH_ABORTED`. 추가 측정 붙이기는 상태를 건드리지 않는 부분 갱신(`ledger.modify_task`) | `api.py`, `engine._approve_and_send`·`_reserve_and_send`·`dispatch`, `ledger.prepare_attempt` |
| 3 UGV 목적지 고정 | 평가 응답 `target_node.node_id` 를 실행 요청·평가 조건에 넣음 | `engine._approve_and_send` |

### 추가 수정 F01~F07 (검토 기준 `92a3153`, 2026-10-01)

| 항목 | 반영 내용 | 위치 |
| --- | --- | --- |
| F01 관측 저장 | 관측 원본 표(`observations`, 실행시도·역할별 1건). 원본·관측 사건·아는 세계·OBSERVED·반영 기록(·반영이 필요 없으면 판정까지)을 한 트랜잭션으로 저장, 그 뒤 트랜잭션 밖에서 APPLY. 중단되면 전부 되돌아가 ARRIVED 로 남고, 저장 뒤 중단이면 원본을 그대로 다시 쓴다. 메모리 상태는 저장 성공 뒤 갱신 | `engine._observe_and_apply`·`_remember`, `ledger.save_observation`·`get_observation` |
| F02 관측 뒤 실패 | FAILED 전에 유효 관측 확보 여부 확인. 확보했으면 물리 실패만 기록(`physical_failure_after_observation`)하고 목적은 재대기로 돌리지 않는다. 위치 불명 실패도 임무를 보류시키지 않음 | `engine._track`·`_unknown(hold_task)` |
| F03 부가 측정 | 같은 방문 기상 측정도 원본·반영 기록을 먼저 저장하고 같은 반영 경로 사용 | `engine._observe_and_apply` |
| F04 취소 | 취소와 끝나지 않은 주·부가 기록 종료를 한 트랜잭션. 복구 시 취소된 임무의 부가 측정도 적용하지 않음 (완료된 임무의 부가 측정은 계속 재시도) | `engine.resolve`·`_process_apply` |
| F05 이전 판 이관 | 장부를 열 때 ACKED/NACKED 기록을 한 트랜잭션으로 이관: 판정 안 된 것은 RESPONDED(응답 근거 = 이전 상태, `restored_from` 표시), 이미 판정·종료된 것은 DONE, 담당 변경은 CLOSED, Task·시도 없음은 MANUAL_REVIEW(재출동 차단). 반복해도 같은 결과. (재검증 반영) 끝나지 않은 목적(불명 때문에 보류 중 포함)은 응답을 보존해 RESPONDED 로 이어 간다 | `ledger._migrate_legacy_apply_rows` |
| F06 위치 수치 | 초대형 정수 등 변환 실패를 `POSITION_INVALID` 로 (그 구간만 예외 처리) | `engine._finite_number` |
| F07 run 전환 | run 활성화와 이전 run 기록 종료를 한 트랜잭션, 메모리 활성 run 은 저장 뒤 갱신. recover 에서 멱등 정리 | `ledger.activate_run`, `engine.recover` |

남은 제약: 관측 저장 뒤·자동 측정 임무 생성(불 발견 시 바람 방향 칸) 전에 멈추면 그 자동 생성은 다시 하지 않는다 (관측·판정에는 영향 없음). 검토에 첨부됐다는 독립 95개 시험은 받지 못해 같은 조건으로 직접 작성했다.

반영 기록 정책 (피드백 반영, `094f965` 이후): 관측을 환경에 보내기 **전에** 기록을 만들고 두 단계를 따로 적는다 — `PENDING`(환경 응답 모름 → 같은 관측 ID 로 APPLY 재시도) → `RESPONDED`(ACK/NACK 저장, 임무 판정 전 → 저장한 응답으로 판정만 재개) → `DONE`(판정 끝) / `CLOSED`(사유). 임무 판정(Task 상태·구역 누적·완료/재대기 사건·판정 종료 표시)은 한 DB 트랜잭션이고 환경 호출은 그 밖이다. 실행시도가 불명이라 임무가 보류 중이면 응답을 `RESPONDED` 로 보관했다가 재접촉 뒤 판정한다. `recover()` 와 `poll()` 이 두 단계를 모두 이어서 처리한다. 확인된 재현(`d74f053`): 불명 중 ACK → 임무가 영영 `IN_EXECUTION`, ACK 뒤 종료 → 재시작 후에도 미완료, 위치 없는 완료 응답 → 관측 실패로 재출동 대기 — 모두 수정. R03 보강: UAV 완료 응답은 관측 처리 전 단계에서 `observation.position`(열화상 lat·lon·alt, 기상 lat·lon) 필수, 없으면 `POSITION_MISSING` 으로 격리.

이전 설명(참고): 상태 `PENDING → ACKED / NACKED / CLOSED(사유)`, 지우지 않는다. 다른 run 의 환경에는 적용하지 않고 run 이 바뀌면 `CLOSED: RUN_ENDED`. 취소된 목적의 대기 관측은 환경에 반영하지 않고 `CLOSED: TASK_CANCELLED` (취소 뒤에 온 관측을 반영하지 않는 기존 규칙과 같음). 늦게 온 ACK 는 그 시도가 아직 목적 담당일 때만 완료로 이어진다. 재시도 횟수 제한·관측 유효시간은 근거 값이 없어 두지 않았다.

| 항목 | 반영 내용 | 시험 | 구분 |
| --- | --- | --- | --- |
| B01 취소 부활 | 목적 상태 전이를 한 곳(`engine._set_purpose`)에서만 하고 장부(`ledger.save_task`)가 다시 검증. 취소 뒤에 온 도착·관측·완료 응답은 목적을 바꾸지 못함. 관측은 `received_after_purpose` 표시와 함께 보관만 함. 실행시도 추적·점유는 READY 확인까지 유지 | `test_state_transitions` | 코드 반영 |
| B02 완료 Task RETRY | 완료·취소 Task 의 RETRY 거절. 임무가 열린 시도가 있으면 새 시도 금지(장부 `prepare_attempt`). 재관측은 새 Task `RECHECK` (`parent_task_id`) | `test_state_transitions` | 코드 반영 |
| B03 응답 식별 | 상태 조회 응답의 실행 식별자(=총괄 attempt_id)·자원 ID·상태 값·형식을 검증. 실패 시 UNKNOWN(점유 유지), 이후 정상 응답으로 회복. 이미 지난 단계(관측·반영)는 다시 밟지 않음 | `test_state_transitions` | 코드 반영 (UAV 응답에 자원 ID 칸 없음 → UAV-01) |
| B04 이벤트 접수키 | 검증 → 접수키 확정 → 반영. 거절(422)은 키를 쓰지 않음. 같은 내용 200, 다른 내용 409. `RECEIVED→APPLIED`, 재시작 시 미반영 이벤트 재처리 | `test_event_intake` (HTTP) | 코드 반영 |
| B05 run 격리 | 단일 활성 run. Task·지식·요청/이벤트 키를 run 범위로. 점유가 남아 있으면 전환 차단, 닫힌 run 재활성 금지, 이전 장부 행 격리 | `test_run_isolation` | 코드 반영 |
| B06 다각형 거리 | 변끼리 교차·접촉·포함이면 0. 유효하지 않은 도형은 거리 계산 안 함 | `test_validation_fixes` | 코드 반영 |
| B07 환경 ACK | ACK 계약이 있는 관측(열화상·기상)만 `needs_env_ack`. 도로 관측에 True 요청은 접수 거절. ACK 전 완료 금지, 응답 없음(시간 초과)은 같은 관측 ID 로 재시도 | `test_validation_fixes` | 코드 반영 |
| B08 LLM 검증 | 값 사용 전 자료형 검사(bool 을 정수로 받지 않음). 잘못된 출력은 INVALID → 규칙 순서 | `test_validation_fixes` | 코드 반영 |
| D01 신고 → 최초 정찰 | 유효 신고마다 처분을 한 번 정해 기록, 새 사건이면 RECON 멱등 생성 후 기존 배정 흐름으로 | `test_event_intake` | 코드 반영 |
| D02 기상 최신성 | 항목별 값·단위·출처·관측시각·유효성. 현장 측정 만료 시 관측소 값으로 전환 + 재측정 요청 | `test_weather_freshness` | 코드 반영 / **현장 측정 유효시간은 설정 필요** |
| D03 완료조건 분리 | RECON = 목표 지점, MONITOR = 필수 셀 전체 누적 관측. 부분 겹침은 부분으로만 기록 | `test_area_completion` | 코드 반영 (임시 모의 범위 90×90m 기준으로 완료됨. 실제 센서·경로 기준은 UAV-08) |
| LLM 추천 재사용 | 키를 "LLM 에 실제로 주는 입력의 해시"로 변경. 확인 결과 이전 키는 위험점수만 바뀌면 옛 추천을 재사용했음 | `test_llm_cache_lock` | 코드 반영 |
| 공통 잠금 | 확인 결과 LLM 호출 동안 추적·취소가 막혔음 → LLM 호출만 잠금 밖으로 분리, 적용 전 입력 재검증. **기체 평가·출동 HTTP 는 여전히 잠금 안** (호출당 최대 `ORCH_HTTP_TIMEOUT_S`) | `test_llm_cache_lock` | LLM 코드 반영 / HTTP 미해결 |

### 상태 전이 (Task 목적 상태)

`models.PURPOSE_TRANSITIONS`. 표에 없는 전이는 거절하고 `PURPOSE_TRANSITION_REJECTED` 로 사유를 남긴다. 모든 전이는 `PURPOSE_TRANSITION` 사건에 이전→이후·근거를 남긴다.

| 현재 | 갈 수 있는 상태 | 대표 근거 |
| --- | --- | --- |
| PENDING | EVALUATING, HOLD, CANCELLED | 배정 평가 시작, 사람 선택 대기, 취소 |
| HOLD | PENDING, EVALUATING, IN_EXECUTION, CANCELLED | 재평가, 불명 시도가 다시 보고됨(IN_EXECUTION), 취소 |
| EVALUATING | APPROVED, HOLD, PENDING, CANCELLED | Safety ALLOW + 예약, 후보 없음 |
| APPROVED | IN_EXECUTION, EVALUATING, HOLD, CANCELLED | 송신, 실행 거절 확정(다음 후보), 응답 유실 |
| IN_EXECUTION | COMPLETED, PENDING, HOLD, CANCELLED | 목적 충족, 관측 미충족·인계, 불명 |
| COMPLETED · CANCELLED · FAILED | (없음) | 종료. 재관측은 새 Task `RECHECK` |

실행시도(attempt)는 목적과 별도로 `PREPARED → STARTED → ARRIVED → OBSERVED/APPLIED → RETURNING → RELEASED` 를 끝까지 간다. "임무가 열린" 상태(`PREPARED, REQUESTED, STARTED, ARRIVED, UNKNOWN`)의 시도가 있으면 같은 Task 에 새 시도를 만들지 않는다. 임무 부분이 끝난 상태(`OBSERVED, APPLIED, RETURNING, FAULTED` — 실패 확정 후 복귀 중 포함)는 기체 점유만 유지하며, 이때는 다른 자원으로 인계 배정할 수 있다 (기존 인계 동작 유지).

수동 해소 `POST /tasks/{id}/resolve`: `RETRY`(실패·보류 뒤 재평가만) · `CANCEL`(중단 명령은 보내지 않음, 추적·점유 유지) · `CONFIRM_RESOURCE_IDLE`(불명 시도 점유 해제, 종료된 목적은 되살리지 않음) · `RECHECK`(종료된 Task 의 재관측 Task 생성, `request_id` 로 멱등).

### run 격리·이관 정책

- 활성 run 은 하나 (`runs` 표). 환경 snapshot 의 `run_id` 가 활성 run 과 다르면: 다른 run 의 점유가 없을 때만 새 run 을 활성화하고 이전 run 을 닫는다. 점유가 남아 있으면 `RUN_SWITCH_BLOCKED` — 판단·접수를 보류하고 이전 실행의 추적만 계속한다. 닫힌 run 의 snapshot 은 `RUN_CLOSED` 로 거절한다.
- 같은 run 안에서 `state_version` 이 역행하면 `SNAPSHOT_STALE` 로 판단을 보류한다 (마지막 버전은 장부에 저장, 재시작 후에도 유지).
- 점유(`reservations`)와 실행시도 추적은 run 과 무관하게 전역이다. run 이 바뀌어도 점유를 지우거나 반납 처리하지 않는다. 다른 run 의 세계를 읽어 이전 실행의 관측을 만들지 않는다 (`OBSERVATION_SKIPPED: RUN_MISMATCH`).
- 이관: run 범위가 없던 이전 장부(스키마 v1)를 열면 Task·지식·실행시도·사건을 `LEGACY-UNSCOPED` 로 표시해 격리한다. 새 run 에 귀속시키지 않는다. **시작 조건:** 이전 장부에 열린 점유가 있으면 그것이 해소(READY 확인 또는 수동 idle 확인)될 때까지 새 run 을 시작하지 않는다.
- 시험용 가짜 환경은 서버를 켤 때마다 0초부터 시작하므로 `build_default()` 가 매번 새 `run_id`(`RUN-FIXTURE-<UTC시각>`)를 만든다. 고정하려면 `ORCH_RUN_ID`.

### 기상 정책 (항목별)

| 출처 | 유효시간 | 근거 |
| --- | --- | --- |
| 관측소 (기상청 ASOS 시간자료) | 3600초 | 자료 제공 간격 (`provision_interval_s`) |
| 시험 관측소 (`EnvStationFeed`, TEST_ONLY) | 환경 한 틱 | 조회할 때마다 그 시각 값을 새로 냄 |
| 현장 측정 (모의 센서) | **미설정** (`ORCH_FIELD_WEATHER_MAX_AGE_S`) | 근거가 되는 자료 계약 없음 → 기본값을 두지 않음 |

- 나이는 시뮬레이션 **관측시각** 기준. 수신시각(`received_sim_s`)은 기록만. 미래 관측·관측시각 불명·값 이상은 쓰지 않는다.
- 항목마다: 목표 칸의 유효한 현장 측정 → 가장 가까운 관측소의 유효한 값 → 없음(`unknown`). 풍속이 없으면 UAV 배정은 기존대로 보류(`WIND_MISSING`).
- 현장 측정이 만료되면 같은 칸 재측정을 한 번 요청한다 (그 칸에 가는 열린 정찰이 있으면 거기에 붙이고, 없으면 `ENV_SENSE` 생성). 측정은 모의 센서다.
- 분석 입력(`belief.field_weather`)과 화면(`known.field_weather`)은 같은 함수로 만든 "지금 유효한 항목"만 담는다. 쓰지 않은 항목은 사유와 함께 `field_weather_excluded` 에. 원본 이력은 `kb.field_history`.
- **현장 측정 유효시간이 미설정이면 현장 측정은 `POLICY_NOT_SET` 으로 표시되고 판단에 쓰이지 않는다** (관측소 값 사용, `/health` `weather_policy.field_policy_unset_items` 와 화면에 표시). 값을 정하면 환경변수로 설정한다.

### 완료조건 (RECON / MONITOR)

| 규칙 (`completion_rule`) | 적용 | 완료 조건 |
| --- | --- | --- |
| `TARGET_POINT` | RECON, ENV_SENSE, 지상 임무, (정찰의) RECHECK | 목표 지점이 유효 관측 범위 안 + (필요 시) 환경 ACK. `area_cell_ids` 는 우선순위 근거용 메타데이터 |
| `AREA_ALL_REQUIRED_CELLS` | MONITOR, (감시의) RECHECK | `required_cell_ids` 전체가 누적 관측 사각형의 합집합으로 완전히 덮임 + 환경 ACK |

- 관측 결과: `covered_cells`(칸 전체가 프레임 안) / `partial_cells`(일부만) / `cell_coverage`(겹친 사각형·넓이 비율). 비율은 기하 계산값이며 임계값을 두지 않는다.
- 아는 세계: 불을 본 칸은 `CONFIRMED`, 칸 전체를 봤고 불 없음은 `OBSERVED_CLEAR`, 일부만 봤고 불 없음은 `NOT_DETECTED_PARTIAL`.
- 구역 임무는 방문마다 누적하고(같은 관측 ID 는 한 번), 남은 필수 셀 중 안 가 본 칸을 다음 방문 지점으로 둔다.
- **관측 범위 두 가지 (구분할 것):** ① 한 프레임 계산값 52×42m (`TEST_THERMAL_90M_V1`, 지면 위 90m, 시험용·보존). ② **임시 모의 관측범위 90×90m** (`ASSUMED_ORBIT_90M_V1`, 사용자 결정, 현재 기본값) — 도착한 칸을 둘러봤다고 가정한 값이며 물리 센서의 실제 범위가 아니다. 드론의 60초 대기는 합의된 동작으로 유지하고, 구역 순찰·스캔은 향후 드론 담당이 구현한다 (UAV-08). 그때 ②를 걷어낸다.
- 임시 범위에서는 90m 칸이 한 번 방문으로 전체 관측이 되어 구역 감시가 정상 완료된다. 실제 입력이 부분 관측일 때(프레임이 칸보다 작거나 기체가 칸 중심에서 벗어남)는 부분으로만 기록하고, 남은 칸을 모두 한 번씩 갔는데도 덮이지 않으면 `AREA_EVIDENCE_UNSUPPORTED` 로 보류한다. 출동 전에 미리 보류하지는 않는다.
- 아는 세계의 화재 믿음: 부분 음성 관측은 기존 신고·확인을 해소하지 않는다 (R02).
- 주기 감시 계약은 아직 없다 (주기·관측 유효성 조건 없음).

### 신고 → 최초 정찰 (사건 대체키)

사건 ID 가 계약에 없어 `INC-AUTO:{run}:{cell}:{n}` 을 쓴다 (그 칸의 n번째 사건). 신고 한 건의 처분: 그 칸을 볼 열화상 임무가 열려 있으면 중복(`DUPLICATE_OF_OPEN_TASK`), 최신 관측이 불 확인이면 `ALREADY_CONFIRMED`(재관측은 RECHECK), 그 밖에는 새 사건(`NEW_INCIDENT`) → RECON. 지도에 칸 위치가 없으면 좌표를 만들지 않고 `TARGET_LOCATION_UNKNOWN` 으로 보류한다.

### 이번에 검증하지 않은 것

PX4/Gazebo 실비행, 실제 열화상·기상 센서, 팀 공유 환경 API, 실제 UAV Agent·UGV 서버·OpenAI 호출(건너뛴 8건), 기존 웹관제 연동.

---

## 구현 현황 (2026-09-30, 커밋 `01b2b33` 기준 — 위 "검토 반영"으로 바뀐 부분은 위 내용이 우선)

분류: **A** 코드와 시험으로 확인 · **B** fixture 에서만 확인 · **C** 타 팀 계약·연동 대기 · **D** 최종 발표 미완료.
건너뛴 시험(실제 서버·실제 API 호출)은 통과로 세지 않는다. 모의 센서·가짜 환경 ACK 로 끝난 Task 완료는 **시뮬레이션 시험 완료**이며 실제 센서 관측·실제 환경 연동 완료가 아니다.

**시험 (`01b2b33` 시점):** `pytest orchestrator/tests -q` → **80 통과, 8 건너뜀**(실제 서버·API 시험: `test_live_uav` 3, `test_live_ugv` 4, `test_live_llm` 1 — 환경변수를 켜고 서버를 띄웠을 때만 실행. 2026-09-30 에 각각 실행해 통과를 확인했으나 기본 집계에는 넣지 않음).

### 기능별 상태

| 기능 | 분류 | 근거·한계 |
| --- | --- | --- |
| SQLite 장부: 멱등 접수, 원자 예약, 송신 전 선저장 | A | `test_ledger`, `test_flow` |
| PREPARED 재시작 복구 (송신 증명 불가 → UNKNOWN·점유 유지, 제공자가 같은 시도 보고 시 재개), 서버 시작 시 자동 실행 | A | `test_recovery` 5건. **물리 명령 최대 1회는 보장하지 않음** (C, UAV-01) |
| 물리 사전필터, 규칙 Safety (정보 누락 시 ALLOW 금지, 80/10m 중복 가산 차단) | A | `test_flow`. 기체 제원·datum 출처는 C |
| 출동·불명(UNKNOWN)·고장(FAULTED)·수동 해소, 복귀·READY 확인 후 반납 | A | `test_flow`, `test_live_*`(실행 시에만) |
| UAV·UGV HTTP 계약 | A (mock/sim) | UAV Agent `UAV_MODE=mock`, UGV 서버 `UGV_DRIVER=sim` 대상. PX4/Gazebo 실비행은 D |
| COUNTER 3A 규칙 (구체 수정필드만 재평가, 드론당 2회) | A / C | 실제 UAV COUNTER 는 실을 칸이 없어 실행 불가 (UAV-02) |
| 정보 경계: 판단 화면은 신고·정찰·관측소·현장 모의 측정 + 정적 지도만 | A | `test_info_boundary`, `test_knowledge`. 이전 `FixtureAnalysis` 의 진짜 위험·확산 복사 누출을 수정함 |
| 분석 입력(belief): 아는 불·불 이력·관측소 최신값·현장 측정 | A | `test_knowledge` (`field_weather` 포함 확인) |
| 위험 칸·확산 예측 값 | B | `InjectedAnalysis` 시험 주입값 (`TEST_INJECTED`, `derived_from_observation=false`). 산출은 ENV-04 (C) |
| 우선순위 규칙: 인명 항상 먼저 → 위험도 → 보호대상 거리, 비슷함(0.1)·엇갈림 묶음 | A (규칙) / B (데이터) | `test_priority`. 위험 칸·보호대상은 주입·fixture |
| **기본 자동 출발**: 애매한 묶음도 검증 통과한 AI 추천 순서, 실패 시 규칙 | A | `test_priority::test_default_mode_is_auto_*`, `test_llm` |
| 수동 버전(`HUMAN_CHOICE`) 전환, 사람 선택 대기 | A | `test_priority` |
| "지금 보내기"(`manual_dispatch`): 두 모드 모두, 이유 기록, Local·Safety 유지, 불명 실행 거절 | A | `test_priority::test_manual_dispatch_*` |
| 상황 변화 시 자동 배정 (새 임무·신고·추적 변화, 주기 실행 없음) | A (API) | `ORCH_AUTO_DISPATCH` 기본 켜짐. 서버 폴링 루프 동작은 시연으로 확인 (단위 시험은 엔진 단위) |
| LLM(gpt-5.6-luna) 제안 + 결정론적 검증·fallback, 같은 상황 재사용 | A | `test_llm`(가짜 모델), 실제 호출 `ORCH_LIVE_LLM=1`. 대기 30초, 서버 1회 실행당 50회 |
| 기상청 ASOS 시간자료 재생 (그 시각까지만, 가까운 관측소) | A | `test_knowledge`, `test_info_boundary` |
| 불 발견 시 현장 측정: 같은 방문에서 측정 + 바람 방향 쪽 칸 | A (로직) / B (측정값) | `test_env_sense`. 측정은 `SIMULATED / TEST_ONLY`, 지도 전체 한 값이면 `GLOBAL_FIELD` |
| 같은 칸 중복 방문 방지 (정찰에 기상 측정 붙이기, 출발 전 측정 임무 흡수) | A | `test_env_sense::test_sense_point_*`, `test_new_recon_absorbs_*` |
| Task 완료 | B | `evidence_level=SIMULATION_TEST` (모의 센서 + FixtureEnv ACK) |
| 판단 화면 `/board` (한글: 아는 상황·확산 예측·정찰/측정 결과·선택 묶음·AI 추천·지금 보내기) | A (총괄 내장) | 기존 웹관제 연결은 C (INT-01~03) |
| 사전 감시 임무 생성 (보호대상별 1건) | A (꺼 둠) | `ORCH_PREEMPTIVE_MONITOR=1`. 입력 예측이 주입값이라 실효는 B |
| 환경 연결 | B | `build_default()` 는 `FixtureEnv`. 팀 공유 환경 아님 (`/health` `truth_env.shared_team_env=false`). ENV-01·05, INT-05 는 C |
| 웹관제 경유·도착 즉시 소화 제거 | C | 통합 코드 수정 필요 (INT-01, INT-02) |
| F1 2026 공식 출동 기준, F2 진화 효과 | D | 원문·팀 계약·연동 시험 없음 (ENV-07) |

### 실제 패키지 구성 (`orchestrator/`)

| 파일 | 역할 |
| --- | --- |
| `config.py` | 모든 설정값과 출처·상태 표시. `.env` 읽기 |
| `models.py` | Task(목적 상태)·실행 substatus·Snapshot·ResourceView |
| `ledger.py` | SQLite 장부: runs·tasks·decisions·attempts·reservations·events·knowledge·external_events. run 범위, 목적 전이 검증, 이전 장부 격리 이관 |
| `knowledge.py` | 총괄이 아는 세계, 기상청 재생(`KmaAsosReplay`), 시험 관측소(`EnvStationFeed`, TEST_ONLY), 분석 주입값(`InjectedAnalysis`) |
| `env_adapter.py` | 환경 경계(READ/ADVANCE/APPLY/map_cells/reports_until), `FixtureEnv` |
| `resources.py` | UAV·UGV HTTP 연결, 응답 끊김 = 불명 |
| `prefilter.py` · `negotiation.py` · `safety.py` | 사전 걸러내기 · COUNTER · 규칙 Safety |
| `observation.py` | 모의 열화상(한 프레임 `TEST_THERMAL_90M_V1` 52×42m / 기본값 원 비행 가정 `ASSUMED_ORBIT_90M_V1` 90×90m)·모의 기상 센서 |
| `priority.py` | 위험 칸↔보호대상 거리, 인명 우선, 선택 묶음 |
| `llm.py` | OpenAI 제안 + 결정론적 검증 |
| `engine.py` | 판단 화면(view), 배정·협상·Safety·출동, 추적·복구, 측정 합치기, 자동/수동, 이벤트 접수·최초 정찰, 목적 전이, 구역 완료 판정 |
| `api.py` · `board.py` | FastAPI 서버, 판단 화면 |

### 실행 방법

```powershell
# 저장소 루트, 가상환경 .venv (fastapi, uvicorn, httpx, openai, pytest)
.\.venv\Scripts\python -m pytest orchestrator/tests -q            # 단위 시험 (실제 서버·API 시험은 건너뜀)
.\.venv\Scripts\python -m orchestrator.api                          # 총괄 서버 127.0.0.1:8200 → /board
# 선택 환경변수
#   ORCH_ENV_FIXTURE=시나리오.json   (진짜 세계 fixture + injected_analysis 분리 키)
#   ORCH_PRIORITY_MODE=HUMAN_CHOICE  (기본 AUTO_HIGHER_RISK = AI 추천으로 자동 출발)
#   ORCH_AUTO_DISPATCH=0             (상황 변화 시 자동 배정 끄기, 기본 켜짐)
#   ORCH_PREEMPTIVE_MONITOR=1        (사전 감시 임무 생성 켜기, 기본 꺼짐)
#   ORCH_FIELD_WEATHER_MAX_AGE_S=600 (현장 측정 유효시간, 시뮬레이션 초. 기본 미설정 = 현장 측정을 판단에 쓰지 않음.
#                                     항목별: '{"wind_ms": 600, "wind_dir_deg": 600}')
#   ORCH_RUN_ID=RUN-X                (가짜 환경의 run_id 고정. 기본은 켤 때마다 새 run_id)
# 실제 서버 시험: ORCH_LIVE_UAV=1 / ORCH_LIVE_UGV=1 / ORCH_LIVE_LLM=1 (각 서버·키 필요)
```

비밀값은 `.env`(git 제외): `OPENAI_API_KEY`, `DATA_GO_KR_SERVICE_KEY`. 형식은 `.env.example`.

### 2026-09-30 작업 이력 (주요 커밋)

| 커밋 | 내용 |
| --- | --- |
| `7eb6537` | 총괄 서버 1차 (장부·판단·출동·추적) |
| `754d998` | UGV 연동, 주행 고장(FAULTED) |
| `224ef9a` · `2b0dc46` | 위험 칸·보호대상 거리 우선순위, 인명 우선·두 모드·판단 화면 |
| `9e4952d` | LLM 제안 + 결정론적 검증 |
| `33e69b0` | 현장 환경 측정·확산 예측 활용 (사전 감시는 꺼 둠) |
| `55ce7bd` · `bd6de64` | 기상청 ASOS 자료, 진짜 세계 / 아는 세계 분리 |
| `38bb633` | 피드백 반영: PREPARED 복구, 분석 누출 차단, 완료 수준 표시 |
| `c2c2adb` | 같은 칸 중복 방문 방지, LLM order 에 task_id 강제 |
| `888b29a` | 기본 자동 출발(AI 추천), 지금 보내기, 상황 변화 시 자동 배정 |
| `01b2b33` | 팀별 요청서 최신화, 분석 입력에 현장 측정 |

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
