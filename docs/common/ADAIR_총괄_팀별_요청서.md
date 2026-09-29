# ADAIR 총괄 오케스트레이터 — 팀별 요청서

- 갱신: 2026-09-30 (최신) · 기준: 브랜치 `feat/orchestrator-final` (`888b29a` 이후)
- 근거: `docs/orchestration/ADAIR_오케스트레이터_구현요청_코드대조_수정본 (1).md` §11 형식, `docs/orchestration/ADAIR_총괄_구현준비.md`
- 총괄 쪽은 아래 계약을 **가짜(fixture)로 먼저 구현·시험**해 두었다. 각 팀이 제공하면 가짜만 교체한다.
- 상태 표기: **A** 코드와 시험으로 확인 · **B** fixture 에서만 확인 · **C** 타 팀 계약·연동 대기 · **D** 최종 발표 미완료

## 0. 먼저 합의할 원칙 — 정답지 방지

환경 시뮬레이터는 **진짜 세계**다. 총괄은 진짜 상태(불 전체, 위험점수, 확산, 기상)를 판단 입력으로 받지 않는다.
총괄의 판단 입력은 **그 시뮬레이션 시각까지** 들어온 다음 네 가지와 공개 가능한 정적 지도 정보뿐이다 (`orchestrator/knowledge.py`, `engine.view()`).

| 판단 입력 | 출처 | 표시 |
|---|---|---|
| 신고 | 시나리오 신고 이벤트, `/events FIRE_REPORT` | `REPORTED` (확인 전) |
| 정찰 결과 | 모의 열화상 (관측 범위 안만) | `CONFIRMED` / `OBSERVED_CLEAR`, `SIMULATED` |
| 관측소 실측 | 기상청 ASOS 시간자료 재생 (미래 행 금지) | `KMA_ASOS_HOURLY` |
| 현장 측정 | 총괄 쪽 모의 기상 센서 (실제 센서 없음) | `SIMULATED / TEST_ONLY`. 환경 기상이 지도 전체 한 값이면 `GLOBAL_FIELD` — 위치별 차이를 만들지 않음 |

- 센서만 진짜 세계를 **관측 범위 안에서** 읽는다. 진짜 상태는 사후 평가에만 쓴다.
- 위험 칸·확산 예측은 **총괄이 아는 세계를 입력**으로 계산해야 한다 (ENV-04). 그래야 예측이 틀릴 수 있고, 현장 측정의 가치와 "총괄이 있었다면" 비교가 성립한다.
- 2026-09-30 수정: 이전 총괄 시험 코드(`FixtureAnalysis`)가 환경의 진짜 위험 칸·확산을 그대로 복사하던 누출을 제거했다. 지금은 환경과 분리된 **시험 주입값**(`InjectedAnalysis`, `TEST_INJECTED`)이며, 관측으로 계산한 예측으로 표시하지 않는다.
- 백서 재현: 그 시각까지 알려진 정보만 준다. 나중에 밝혀진 피해·결과를 판단 입력으로 주지 않는다.

## 요청 요약

| ID | 팀 | 요청 | 막히는 것 | 우선 |
|---|---|---|---|---|
| ENV-01 | 환경 | 조회(READ)·진행(ADVANCE)·반영(APPLY) 분리, snapshot 필수 필드 | 조회만 해도 시뮬레이션이 진행됨 | P0 |
| ENV-02 | 환경 | 정적 지도: 칸 위치·크기·지면고도·건물유형(1=주거), 보호대상, 소방서 위치 | 인명 우선·보호대상 거리·출발점 | P0 |
| ENV-03 | 환경 | 진짜 세계 바람을 **같은 기상청 ASOS 자료**로, 가능하면 칸별 바람장 | 두 세계의 기상 불일치, 양간지풍 재현 | P0 |
| ENV-04 | 환경 | 위험 칸·확산 예측을 **"아는 세계 입력 → 결과" 기능**으로 | 예측이 정답지가 되거나 시험 주입값에 머묾 | P0 |
| ENV-05 | 환경 | 관측 반영 APPLY + ACK (관측 ID 중복 방지, 열화상·기상) | 환경 반영 완료 판정 | P0 |
| ENV-06 | 환경·통합 | 시나리오 신고 이벤트 (백서 시각표) | 총괄이 불을 처음 알게 되는 경로 | P1 |
| ENV-07 | 환경 | (F2) 진화 행동 → 효과 계산 → 새 상태 | 최종 발표 진화 효과 | D |
| UAV-01 | 드론 | 실행 중복방지 키·실행시도 ID, **상태 영속과 재시작 후 조회** | 물리 출동 1회 보장, 재시작 복구 | P0 |
| UAV-02 | 드론 | COUNTER 수정값은 재평가 요청에 실을 수 있는 필드로만 | COUNTER 가 사실상 실행 불가 | P0 |
| UAV-03 | 드론 | DEM 경로 ID·버전·해시 반환 | 평가·승인·실행 경로 동일성 | P1 |
| UAV-04 | 드론 | 기한을 시뮬레이션 시간(remaining_time_s)으로 | 2019 시각 비교 오류 | P1 |
| UAV-05 | 드론 | 관측 고정값(312.5℃) 제거, 기상 센서 형식 | 관측 근거 | P1 |
| UAV-06 | 드론 | 복귀 중 기체의 재배정 가능 여부 합의 | 점유 해제 시점 | P1 |
| UAV-07 | 드론 | **한 방문에 여러 측정**(정찰+기상) 요청 형식, 기상 전용 이동 평가 | 같은 칸 중복 비행 | P1 |
| UGV-01 | UGV | 평가 때 고른 목적지로 실행 (target_node 고정) | 평가·실행 목적지 불일치 | P1 |
| UGV-02 | UGV | 실행 기록 영속·재시작 후 조회, 기상 센서 형식 | 불명 처리, 현장 측정 | P2 |
| INT-01 | 통합 | 웹 출동 경로를 새 총괄 경유로 | 총괄·Safety·정보 경계 우회 | P0 |
| INT-02 | 통합 | 도착 즉시 불 끄기(`/api/extinguish`, `/api/auto/done`) 제거 | 도착=진화로 보임 | P0 |
| INT-03 | 통합 | 총괄 포트(임시 8200)·배포, 판단 화면·API 연결 | 관제 연결 | P1 |
| INT-04 | 통합 | 거점(소방서) 좌표 단일 출처 | 좌표가 파일마다 4종 | P0 |
| INT-05 | 통합·환경 | 시뮬레이션 시계 소유자(ADVANCE 주체)·진행 주기 합의 | 웹·총괄이 각자 시계를 돌림 | P0 |

---

## 환경팀

### ENV-01 조회·진행·반영 분리
| 항목 | 내용 |
|---|---|
| 현재 코드 | `connectors/fire_connector.py` 는 조회 때마다 `api.step()`, `web/app.py` `/api/env` 도 진행. `environment/src/api.py` 에 `run_id`·`simulation_time`·`map_version` 없음, `state_version` 은 재시작 시 1로 돌아감 |
| 차단 문제 | 조회만 해도 불이 번짐. 웹과 총괄이 서로 다른 환경 인스턴스를 가짐 |
| 계약 요청 | 한 실행(run)에 환경 인스턴스 하나. `READ` 는 상태 불변, `ADVANCE(steps)`, `APPLY(observation)` 분리 |
| 응답 예시 | `{"run_id":"RUN-INJE-01","state_version":42,"simulation_time_s":1800.0,"scenario_start_kst":"2019-04-04T14:45:00+09:00","map_version":"INJE-2019-v1","crs":"EPSG:5186","vertical_datum":"(확인 필요)"}` |
| 단위·시계 | 시뮬레이션 초(float). `scenario_start_kst` 는 시뮬레이션 0초의 실제 시각 (기상청 재생·화면 시각 기준) |
| 오류 | 같은 run 안에서 버전 역행 금지. run 이 바뀌면 새 `run_id` (총괄은 `RUN_CHANGED` 로 처리) |
| 수락 시험 | READ 를 여러 번 불러도 `state_version` 불변 |
| 총괄 상태 | B — 기본 서버(`build_default`)도 `FixtureEnv`. `/health` 에 `truth_env.shared_team_env=false` 로 표시 |

### ENV-02 정적 지도
| 항목 | 내용 |
|---|---|
| 현재 코드 | 환경은 칸마다 `building_type`(0 없음·1 주거·2 중요·3 핵심)을 쓰지만 밖으로 내보내지 않음. 보호대상·소방서 목록 없음 |
| 계약 요청 | `map_cells`: `cell_id, lat, lon, cell_size_m, ground_amsl_m, building_type` — 불·위험·기상 같은 **동적 값 제외**. `protected_sites`: `site_id, site_type, human_occupied, boundary[[lat,lon]...] 또는 location, 좌표 출처`. `stations`: 소방서 `base_id, lat, lon` |
| 총괄 사용 | 칸 ID 로 위치·지면고도·건물유형을 찾는 데만 씀 (목록 자체를 판단에 쓰지 않음). 주거 칸·사람 있는 보호대상과 겹치는 칸은 **인명피해 예상**으로 항상 먼저 |
| 비고 | 주거지 정보 제공은 환경팀과 구두 합의 (2026-09-30). 백서 141쪽 시설(남전리 마을회관·주택, 인제휴게소, 위생환경처리장)은 **후보**이며 좌표는 별도 근거 필요 |
| 수락 시험 | 주거 칸 과제가 위험도가 낮아도 먼저 배정 (`test_residential_*`) |
| 총괄 상태 | A (계산) / B (데이터) |

### ENV-03 진짜 세계 바람 = 기상청 관측
| 항목 | 내용 |
|---|---|
| 현재 코드 | 바람이 지도 전체에 값 하나(`environment_config.yaml`), 칸별 바람은 "향후 확장" |
| 자료 | `data/weather/kma_asos_hourly_20190404_20190405.csv` (인제 211·속초 90·홍천 212·대관령 100, 결측 없음), 좌표 `kma_asos_stations.json` (기상청 지점정보, 현재 기준) |
| 계약 요청 | 진짜 세계 바람을 이 자료로 시각별로 구동. 가능하면 지형을 반영한 칸별 바람장(양간지풍). 칸별 값은 `cell.weather = {wind_ms, wind_dir_deg, temperature_c, humidity_pct}` |
| 이유 | 총괄은 같은 자료를 **관측값**으로 받는다 (그 시각까지만). 시연에서 현장 모의 측정(3.0 m/s 서풍)과 인제 관측(5.7 m/s 남남동풍)이 어긋난 원인 |
| 총괄 상태 | A (`KmaAsosReplay`, 모의 기상 센서는 `cell.weather` 가 있으면 그 칸 값, 없으면 `GLOBAL_FIELD`) |

### ENV-04 분석 기능 (정답지 방지 핵심)
| 항목 | 내용 |
|---|---|
| 계약 요청 | `analyze(belief) → {risk_cells, spread_forecast, forecast_ref, source}` |
| 입력 `belief` (총괄이 넘김) | `known_fires`(칸·상태·시각·출처), `fire_states`(신고·확인·불 없음 이력), `station_weather`(관측소 최신값), `field_weather`(현장 측정, 출처 포함), `run_id`, `simulation_time_s` |
| 출력 | `risk_cells`: 화선 주변 UNBURNED 약 10칸 `cell_id, risk_score, 선정 기준`. `spread_forecast`: `cell_id, expected_arrival_s, probability(있으면)`. `forecast_ref`: `model_version, issued_sim_s, horizon_s` |
| 금지 | 진짜 불 상태·진짜 바람·진짜 확산을 입력으로 쓰지 않음 |
| 총괄 활용 | 화면 표시, 예측이 주거지·사람 있는 보호대상에 닿으면 인명 우선(근거 `FORECAST`), 불 발견 시 바람 방향 쪽 측정 지점 선택, 보호대상별 사전 감시(구현, 부하 문제로 꺼 둠) |
| 수락 시험 | ① 진짜 세계의 미래 화재·확산만 바꿔도 결과 불변 (총괄 `test_truth_only_changes_do_not_leak_*` 와 같은 조건). ② 새 신고·정찰 후에만 결과 갱신. ③ 같은 진짜 상태에서 측정을 더 한 경우와 안 한 경우 예측이 달라짐 |
| 총괄 상태 | B — `InjectedAnalysis`: 환경과 분리된 시험 주입값, `requires_known_fire` 로 아는 불에 따라서만 노출, `TEST_INJECTED`·`derived_from_observation=false` 표시 |

### ENV-05 관측 반영
| 항목 | 내용 |
|---|---|
| 계약 요청 | `APPLY(observation)` 는 `observation_id` 로 한 번만 반영하고 ACK `{accepted, state_version, duplicate}`. 열화상(`detections`, `covered_cells`)과 기상(`values`) 모두 |
| 금지 | 관측만으로 UNBURNED→BURNING 임의 변경, 도착만으로 진화 |
| 총괄 상태 | B — 임무 완료 근거에 `evidence_level=SIMULATION_TEST` (모의 센서 + 가짜 환경 ACK) 로 표시. 실제 환경 연동 완료 아님 |

### ENV-06 신고 이벤트
| 항목 | 내용 |
|---|---|
| 계약 요청 | 시나리오 시각표의 신고를 이벤트로: `{cell_id, sim_time_s, source:"119_CALL", note}`. 총괄 `/events` `FIRE_REPORT` 로 보내도 됨 |
| 근거 | 백서 140~146쪽 (발화·신고·대피·헬기 불가·대응 2단계 시각). 시연의 14:45 신고는 시험값이며 백서 대조 필요 |
| 총괄 상태 | A (`FIRE_REPORT`, `FixtureEnv.reports_until`) |

### ENV-07 (F2) 진화 효과
| 항목 | 내용 |
|---|---|
| 계약 요청 | `도착 → 진화 행동(위치·시간·자원량) → 환경 효과 계산 → 새 state_version·ACK → 재관측 → 종료 판단`. 효과 계수는 환경 모델 몫 |
| 금지 | 소방차 1대당 진압 면적 등 임의 수치, 도착만으로 소화 |
| 총괄 상태 | D |

---

## 드론(UAV)팀

| ID | 현재 코드 | 요청 | 총괄 쪽 현재 처리 |
|---|---|---|---|
| UAV-01 | 실행 상태가 메모리(`_tasks`)에만 있어 재시작하면 404. 중복 키 없음 | `ExecuteRequest` 에 `execution_attempt_id`(또는 idempotency key), 같은 키 재요청 시 같은 결과. **상태 영속**, 재시작 후 `/task/{id}` 조회 | `task_id` 칸에 총괄 `attempt_id` 를 넣어 보냄. 총괄 재시작 시 송신 직전·직후 시도(`PREPARED`)를 `/task/{attempt_id}` 로 대조: 같은 시도를 보고하면 추적 재개, **404·조회 불가는 UNKNOWN**(점유 유지, 재송신·재출동 없음, 수동 해소). 제공자 멱등성 없이는 **물리 명령 최대 1회를 보장하지 않음** |
| UAV-02 | `RETURN_MARGIN_INSUFFICIENT` COUNTER 가 `observe_duration_s` 를 제안하지만 `EvaluateRequest` 에 그 칸이 없음. `MODERATE_WIND` 는 note 만 | COUNTER 는 재평가 요청에 실을 수 있는 필드만 제안하거나, `observe_duration_s` 를 요청 필드로 추가 | 구체 수정값만 재평가(드론당 2회). 실제 서버 COUNTER 는 모두 "실행 불가"로 다음 후보 (능선 목표 시험에서 확인) |
| UAV-03 | DEM 경로 미구현 (책임은 드론팀 확정) | 평가 응답에 `route_id, route_version, route_hash`, 실행은 같은 해시만 | Safety 에 평가·실행 조건 동일성 검사 있음. 경로 해시 칸은 계약 후 추가 |
| UAV-04 | `deadline` 을 현재 UTC 와 비교 | `remaining_time_s`(시뮬레이션 초) | 기한을 보내지 않음 |
| UAV-05 | 관측값이 항상 `312.5℃ / hotspot` | 모의임을 명시하거나 제거. 기상 센서(`WEATHER`) 지원 여부·형식 | 고정값 무시(`uav_reported_values_ignored` 로 원문만 보관), 총괄 모의 센서 사용 |
| UAV-06 | 복귀 중(RETURNING)은 BUSY 아님 | 복귀 중 재배정 허용 여부 합의 | 착륙·READY 확인 전까지 점유 유지 |
| UAV-07 | `observation_type` 하나만 받음, `WEATHER` 는 미지원(`REQUIRED_CAPABILITY_UNAVAILABLE`) | ① 한 방문에서 여러 측정: 예) `measurements: ["THERMAL","WEATHER"]`, 결과도 측정별로. ② 기상 측정만 하는 임무의 이동 가능성 평가 | 총괄은 같은 칸 중복 방문을 막으려 정찰에 기상 측정을 붙이고(`extra_sensors`), 불 발견 방문에서 바로 측정. 기상 전용 임무는 `observation_type` 없이 보내 **이동 가능성만** 평가받음 (측정은 총괄 모의 센서) |

## UGV팀

| ID | 현재 코드 | 요청 | 총괄 쪽 현재 처리 |
|---|---|---|---|
| UGV-01 | `execute` 가 목적지 노드를 다시 고름 (우회 옵션 켜면 평가와 달라질 수 있음) | 평가에서 받은 `target_node` 로 실행하면 그대로 따름 확인 | 현재는 같은 `target` 좌표로 실행 요청. `target_node` 전달은 계약 후 추가 |
| UGV-02 | 실행 기록 메모리, A거점 좌표가 다른 파일과 다름 | 상태 영속·재시작 후 조회, 거점 좌표는 INT-04 단일 출처, 기상 센서 형식 | 주행 고장은 `FAULTED`(운영자 해제로 READY 될 때까지 점유 유지), 도로 관측은 화재 관측으로 쓰지 않음. 실제 UGV 서버(sim) 연동 시험 4개 통과 |

---

## 통합팀

총괄 패키지만 고쳐서 웹 통합을 끝낼 수는 없다. INT-01·02 는 **통합 담당 코드(`web/app.py`, `web/static/auto.js`) 수정**이 필요하며 수정 전 합의가 필요하다. 총괄은 웹 통합을 완료로 표시하지 않는다 (상태 C).

### INT-01 웹 출동 경로를 새 총괄 경유로
| 항목 | 내용 |
|---|---|
| 현 코드 (2026-09-30 재확인) | ① `/api/fly` (`web/app.py:201`): 화면에서 고른 칸으로 UAV `/evaluate`→`/execute` 를 **직접** 호출 (`:217`, `:220`). ② `/api/auto/tick` (`:153`): 옛 연결 코드 `main.process_task` + `connectors/safety_connector` 사용 — 이 Safety 는 **항상 ALLOW 하는 mock** 이고, 목표는 환경의 **진짜 불 목록**(`es.fire_cells`)에서 위험점수 최대 칸 |
| 수정 이유 | 새 총괄의 Local·규칙 Safety·장부(중복 방지·불명 처리)·정보 경계를 우회. 자동 모드가 총괄이 알 수 없는 진짜 불 위치로 출동 (정답지) |
| 계약 변화 | 웹 → 총괄 `POST /tasks` (`request_id` 멱등). 진행은 `GET /tasks/{id}`·`/priority/board`. 불을 처음 아는 경로는 신고(`POST /events FIRE_REPORT`) 또는 정찰. 자동 출발은 총괄이 상황 변화 때 스스로 배정 |
| 기존 시연 영향 | 칸을 눌러 바로 비행하던 동작이 총괄 판단(걸러내기·Safety·보류)을 거침. 자동 모드는 신고가 있어야 시작 |
| 수락 시험 | 웹 요청 1건 → 총괄 장부에 `TASK_RECEIVED`→`LOCAL_RESPONSE`→`SAFETY_JUDGEMENT`→`EXECUTION_SENT`, UAV `/execute` 는 총괄에서만 1회. 같은 `request_id` 재전송 시 execute 추가 없음. 신고 없이 자동 출동 없음 |

### INT-02 도착 즉시 불 끄기 제거
| 항목 | 내용 |
|---|---|
| 현 코드 | `/api/extinguish` (`web/app.py:127`) 와 `/api/auto/done` (`:189`) 이 환경에 `fire_state=UNBURNED` 적용 (`:135`, `:195`). 화면은 **이동 애니메이션 진행률**(`_driveT ≥ 0.999`, `:366`)만 보고 호출 — 실제 차량 도착 아님 |
| 수정 이유 | 도착(화면 애니메이션)만으로 불이 꺼져 진화 효과처럼 보임. 진화는 ENV-07 |
| 계약 변화 | 두 경로 제거 또는 "시연 전용" 스위치로 분리해 기본 꺼짐 |
| 기존 시연 영향 | 도착해도 불이 즉시 사라지지 않음. F2 전까지 불은 환경 모델대로 진행 |
| 수락 시험 | 도착 후에도 해당 칸이 `UNBURNED` 로 바뀌지 않음. 애니메이션 완료가 환경 상태를 바꾸지 않음 |

### INT-03 총괄 서버·판단 화면 연결
| 항목 | 내용 |
|---|---|
| 포트·배포 | 총괄 8200 (임시, UGV 가 8100 사용). 배포 위치 합의 필요 |
| 총괄 API | `GET /health` (환경 종류·분석 출처·LLM 상태), `POST /tasks`, `GET /tasks[/{id}]`, `/tasks/{id}/decisions`·`/events`, `POST /tasks/{id}/manual_dispatch` (사람이 지금 보내기, 이유 필수), `POST /tasks/{id}/resolve` (불명·취소 수동 해소), `POST /dispatch_pending`, `POST /events` (`FIRE_REPORT`·`ENV_UPDATED`·`RESOURCE_CHANGED`), `GET /state`, `GET /priority/board`, `POST /priority/mode`, `POST /priority/choose`, `GET /board`(총괄 내장 한글 화면) |
| 동작 기본값 | 우선순위 모드 **자동**(애매한 묶음도 검증 통과한 AI 추천 순서로 출발, 실패 시 규칙). 수동 모드(`HUMAN_CHOICE`)는 화면 전환. 상황 변화(새 임무·신고·자원 반납) 때 자동 배정 — 주기 실행 없음 |
| 화면 표시 요청 | 기존 관제에 붙일 때 다음을 구분 표시: 확인 전 신고 / 정찰 확인, `시험 주입값` 분석, `SIMULATED` 측정, 완료 근거 `SIMULATION_TEST` |
| 총괄 상태 | 총괄 내장 화면 A, 기존 관제 연결 C |

### INT-04 거점 좌표 단일 출처
원통 좌표가 `config.py`·`px4/NOTE.md`·UGV 서버·환경에 서로 다름. 새 인제 맵의 소방서 좌표 하나로 통일 (ENV-02 `stations`). 총괄은 좌표를 코드에 두지 않는다.

### INT-05 시뮬레이션 시계 소유자
| 항목 | 내용 |
|---|---|
| 현 코드 | 웹 `/api/env` 조회가 환경을 진행시키고, 총괄은 FixtureEnv 의 시계를 씀 — 시계가 둘 |
| 요청 | ADVANCE 를 누가 얼마 간격으로 부를지 하나로 정함 (ENV-01). 총괄은 READ 만 하고, 시계가 바뀌면 `/events ENV_UPDATED` 로 알림 받는 방식 권장 |
| 영향 | 기상청 재생·신고 시각·화면 시각이 모두 이 시계를 따름 |

---

## 총괄이 이미 준비해 둔 것 (각 팀이 붙이면 되는 곳)

| 기능 | 위치 | 상태 |
|---|---|---|
| 아는 세계 장부 (신고·정찰·관측소·현장 측정) | `orchestrator/knowledge.py`, `ledger.py` | A |
| 정보 경계 (판단 화면에 진짜 상태 없음) | `engine.view()`, `view_for()` | A (`test_info_boundary`) |
| 기상청 관측 재생 | `KmaAsosReplay` | A |
| 분석 교체 지점 | `Orchestrator(analysis=...)` — `analyze(belief)` 만 구현 | B (`InjectedAnalysis`) |
| 환경 교체 지점 | `read / advance / apply / map_cells / reports_until` | B (`FixtureEnv`) |
| 드론·UGV 연결 | `resources.py` — UAV Agent(mock)·UGV 서버(sim) 연동 시험 | A (mock/sim) |
| 재시작 복구 (`PREPARED` 대조, 불명 시 점유 유지) | `engine.recover()` | A (`test_recovery`), 물리 1회 보장은 C |
| 같은 칸 중복 방문 방지 (측정 합치기) | `engine._attach_or_create_sense`, `_merge_pending_sense_into` | A |
| 우선순위 (인명 우선·비슷함 0.1·엇갈림) + 자동/수동 모드 + 지금 보내기 | `priority.py`, `engine.dispatch_pending`, `manual_dispatch` | A (규칙) / B (데이터) |
| LLM 추천 + 결정론적 검증·재사용 | `llm.py` | A |
| 판단 화면·API | `/board`, `/priority/*`, `/tasks`, `/events` | A (총괄 내장) |
| F1 공식 출동 기준 · F2 진화 효과 | — | D |
