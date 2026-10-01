# 환경 계약 계층 (ENV-01~06) — 멘토 패치 설명

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
