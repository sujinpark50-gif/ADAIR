# wildfire-disaster-response — D파트(시스템 통합·관제·로그) 뼈대

박수진님의 **"01. Integration Contract"**, **"02. Integration Test Cases"** 문서를
기준으로 전면 재작성한 뼈대입니다.

## 폴더 구조

```
├── main.py                          # 전체 흐름 실행 (process_task가 핵심 로직)
├── orchestration_demo.py            # IT-01/IT-02 Mock 오케스트레이션 폐루프 시연
├── config.py                        # 좌표·Grid·연결방식·schema_version 등 설정값
├── ids.py                           # run_id/task_id/decision_id 생성 (Decision ID 원칙)
├── logger.py                        # 이벤트 단위 JSONL 로거
├── interfaces/
│   └── schema.py                    # Task, ResourceStatus, LocalResponse, SafetyVerdict 등
├── connectors/
│   ├── fire_connector.py            # 환경 Connector
│   ├── uav_connector.py             # UAV Connector
│   ├── ugv_connector.py             # Ground Resource Connector (UGV·소방차 공통)
│   ├── orchestrator_connector.py    # Task 생성·제안·재평가 (Mock)
│   └── safety_connector.py          # Safety Layer (ALLOW/MODIFY/REJECT)
├── tests/
│   └── it_cases.py                  # IT-01, IT-02 실행 가능한 테스트
└── logs/                            # 실행 시 생성 (1 Run = 1 Log File)
```

## 실행 방법

```bash
python3 orchestration_demo.py   # IT-01 정상 흐름 + IT-02 REJECT 재할당 시연
python3 main.py                 # 일반 시뮬레이션 (10 스텝)
python3 -m tests.it_cases       # IT-01, IT-02 통합 테스트
```

`orchestration_demo.py`는 Environment, UAV, Safety를 Mock으로 두고 가운데 Orchestrator의
선택 → Local 응답 → Safety → 실행 → 관측 → 환경 갱신 → 완료 흐름을 출력합니다.
IT-02에서는 첫 UAV의 `HIGH_WIND` 거절 후 새 `decision_id`로 다른 UAV를
재할당하는 과정도 확인할 수 있습니다. 상세 이벤트는 `logs/`의 JSONL에도 저장됩니다.

## Integration Contract/Test Cases 반영 사항

- **Task 개념**: 환경 상태 → Task 생성 → Orchestrator 후보 평가 → 자원 선택 흐름을
  `Task` dataclass와 `orchestrator_connector.create_task()`로 구현
- **Decision ID 원칙**: REJECT/COUNTER/Safety REJECT로 재평가가 발생하면
  `ids.new_decision_id()`로 반드시 새 ID 부여 (`tests/it_cases.py`에서 검증)
- **ACCEPT 후에만 Safety 호출**: `main.py`의 `process_task()`에서 순서 강제
- **Safety 3단계 응답**: `ALLOW`/`MODIFY`/`REJECT` (`SafetyVerdict`)
- **REJECT 사유 고정 Enum**: `LOW_BATTERY`, `HIGH_WIND`, `ROAD_BLOCKED` 등 9종
- **이벤트 단위 로그(JSONL)**: 스텝 하나에 여러 이벤트 줄이 남는 구조로 전환
- **Resource 상태값**: `READY/ASSIGNED/RUNNING/RETURNING/STALE/OFFLINE/UNAVAILABLE`
- **resource_type**: `UAV`/`UGV`/`FIRE_ENGINE`으로 공통 Resource 관리

## 초기 범위에서 제외된 것 (Integration Test Cases 4번 기준)

아래는 지금 구현하지 않았고, 대표 시나리오 구현 단계에서 필요할 때 추가합니다.

- COUNTER 세부 처리 (지금은 REJECT와 동일하게 처리)
- 배터리 저하 및 임무 인계
- Heartbeat / STALE / OFFLINE 전이
- 통신 복구, Retry
- 중복/오래된 메시지 처리
- 이기종 자원 대체
- 환경 변화에 따른 부분/전체 재계획
- Safety MODIFY의 세부 수정 로직 (지금은 REJECT와 동일하게 재평가로 처리)

## 아직 TBD인 것 (config.py에 표시됨)

- 최종 대상지역 좌표계, Raster 해상도/Cell Size
- 실제 PX4 연결 방식, 최종 웹 배포 방식
- Safety 세부 임계값, 풍속·배터리 등 최종 운용 임계값

## Mock → 실제 모듈로 교체하는 방법

1. `config.py`에서 해당 `_CONNECTION_MODE`를 `"mock"` → `"real"`로 변경
2. 각 `connectors/xxx_connector.py`의 `raise NotImplementedError(...)` 부분에 실제 연동 코드 작성
3. `main.py`는 그대로 — connector가 알아서 real 로직을 타게 됨
