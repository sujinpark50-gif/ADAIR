# ADAIR 총괄 오케스트레이터 — 멘토링 준비

## 1. 담당 작업

전체 시스템의 중앙 Orchestrator를 설계하고 구현한다. 환경 상태와 자원 상태를 받아 실행 가능한 후보를 구성하고, 적절한 자원에 Task를 제안한 뒤 `ACCEPT / REJECT / COUNTER` 응답에 따라 재평가·재할당한다.

Safety Layer와 각 자원 Connector 사이의 의사결정 흐름과 주요 판단 로그의 의미를 정의한다. 총괄은 재평가 시 새로운 `decision_id`가 필요한 시점 등 판단 규칙을 정하고, 통합 담당자는 이를 공통 Schema/Enum, ID 생성·전달, 로그 구조, Connector/Adapter 및 규격 버전 관리로 구현한다.

### 현재 설계 원칙

아래는 목표 설계 원칙이며, 각 항목의 구현 완료를 뜻하지 않는다. 현재 Mock의 구현 범위는 2절에 구분한다.

- 명확한 불가능 조건은 AI가 아닌 하드 제약 규칙으로 필터링한다.
- 실행 가능한 후보가 여러 개일 때의 전략적 선택은 AI Orchestrator 적용 대상으로 검토한다.
- 중앙 Orchestrator가 모든 현장 정보를 알고 있다고 가정하지 않는다.
- 각 Local Agent가 자신의 상태와 현장 조건을 다시 판단한다.
- 실패 또는 환경 변화가 발생하면 폐루프로 재평가한다.
- Mock과 실제 모듈이 같은 Connector 인터페이스를 사용하도록 분리한다.

## 2. 현재 진행 상황

### 완료 및 정리

- 전체 계층형 아키텍처 설계
- Orchestrator ↔ Local Agent ↔ Safety 실행 흐름 정의
- `ACCEPT / REJECT / COUNTER` 프로토콜 정의
- 통합계약 및 IT-01/IT-02 테스트 기준 작성
- UAV 고도·풍속·좌표 책임 경계 협의
- `cell_id ↔ lat/lon` 공통 변환을 총괄 계층에서 담당하는 방향 정리
- 통합 담당자의 기존 Mock Environment/UAV/Safety/Orchestrator Connector와 통합 테스트를 활용
- 기존 Connector 규격과 `main.process_task()`를 호출하는 `orchestration_demo.py` 시연 실행기 추가
- IT-01 정상 폐루프의 Mock 실행 확인
- IT-02 `HIGH_WIND REJECT → 재평가 → 다른 UAV 재할당`의 Mock 실행 확인
- REJECT 이후 새로운 `decision_id` 발급 확인
- 기존 JSONL 로거에 콘솔 이벤트 출력과 실행 자원 식별정보 보강
- 이전 로그 누적으로 테스트가 실패하던 문제 수정 및 기존 IT-01/IT-02 테스트의 반복 실행 확인

현재 Mock Orchestrator는 AI가 아닌 결정론적 규칙을 사용한다. `READY` 상태인 UAV 중 첫 번째 후보를 선택하고, REJECT한 자원을 후보에서 제외한 뒤 남은 후보를 다시 선택한다.

### 현재 Mock의 검증 범위와 한계

- 데모의 `ACCEPT`와 `REJECT(HIGH_WIND)`는 시나리오에서 강제한 응답이다. 배터리·풍속·통신·센서 상태를 근거로 Local Agent가 판단한 결과는 아니다.
- Mock Safety는 배정이 있으면 `ALLOW`한다. 실제 안전 조건이나 임계값은 검증하지 않는다.
- 실행 시작은 로그와 Task 상태 전이로 모사하며, 이어서 Mock Observation을 반환한다. 실제 비행이나 목표 도달을 확인하지 않는다.
- 환경 갱신은 관측 수신 후 `env_updated=True`로 표시한다. 실제 화재 모델에 관측값을 반영하는 계산은 미구현이다.
- 확인한 것은 `ACCEPT → Safety ALLOW → 실행 단계 → 관측 → 환경 갱신 → 완료`의 연결과, REJECT 시 기존 Task를 유지하면서 새 `decision_id`로 다른 UAV를 제안하는 흐름이다.
- `COUNTER`와 Safety `MODIFY`는 현재 REJECT와 같은 재평가 경로로 처리한다. 대안·수정안 검토는 후속 구현 대상이다.
- `cell_id ↔ lat/lon` 변환은 책임 방향을 정리한 상태이며, 현재 Mock은 목표 위치에 Config의 원점 좌표를 사용한다.

### 진행 필요

- 하드 제약 후보 필터 구체화
- 후보별 평가 점수와 선택 기준 구현
- `COUNTER` 세부 처리
- Safety `MODIFY` 세부 처리
- 상태 정보의 유효시간 및 `STALE / OFFLINE` 처리
- 실제 Environment/UAV Connector 연동
- AI Orchestrator 적용 범위 결정
- 자원 간 충돌 및 동시 Task 처리 검증

## 3. 모듈 연결 내용

### 입력받아야 할 정보

- Environment → 화재 위치, `cell_id`, 풍속 등 환경 상태
- UAV/UGV/소방차 → 위치, 배터리, 상태, capability, Local Response
- Safety → `ALLOW / MODIFY / REJECT`

### 공통 메시지 식별 규칙

통합계약 4절에 따라 주요 통합 메시지는 다음 식별정보를 갖추도록 한다.

- 공통 필수: `message_id`, `sender`, `message_type`, `schema_version`, `timestamp`, `run_id`
- 상황별 추가: `task_id`, `resource_id`, `decision_id`, `correlation_id`

이는 실제 연동 시 준수할 계약이다. 현재 Mock의 함수 호출과 이벤트 로그가 모든 공통 메시지 필드를 전달하는지까지 검증한 것은 아니다.

### 제공해야 할 결과

- 선택된 자원에 전달할 Task
- 자원 선택 및 재선택 결과
- `decision_id`
- 재평가·재할당 이벤트
- 통합 로그 및 대시보드용 상태
- 최종 Task 상태(`COMPLETED / FAILED / CANCELLED`)

## 4. 현재 Mock 실행 흐름

다음 명령으로 IT-01과 IT-02를 연속 실행한다.

```bash
python orchestration_demo.py
```

### IT-01 — 정상 폐루프

```text
Mock Environment에서 화재 발생
→ Orchestrator가 UAV-A 선택
→ UAV-A ACCEPT
→ Mock Safety ALLOW
→ 실행 단계 모사
→ Mock Observation 반환
→ Environment Update 플래그 설정
→ Task COMPLETED
```

### IT-02 — REJECT 이후 재할당

```text
Orchestrator가 UAV-A 선택
→ UAV-A REJECT(HIGH_WIND)
→ 기존 UAV-A를 후보에서 제외
→ 기존 Task를 유지하고 새로운 decision_id로 재평가
→ UAV-B 선택
→ UAV-B ACCEPT
→ Mock Safety ALLOW
→ 실행 단계 모사
→ Mock Observation 반환
→ Environment Update 플래그 설정
→ Task COMPLETED
```

현재 단계의 목적은 AI 판단 품질이나 실제 PX4 연동이 아니라, 총괄 오케스트레이션 폐루프와 모듈 경계가 코드로 정상 연결되는지 검증하는 것이다.

## 5. 멘토 질문

### 질문 1 — 중앙 Orchestrator와 Local Agent의 책임 경계

저희는 중앙 Orchestrator가 전역 환경 상태와 전체 자원 상태를 바탕으로 후보 자원을 평가하고 Task를 제안하며, 각 Local Agent는 배터리·풍속·통신·센서 상태 등 자신의 최신 로컬 정보를 바탕으로 `ACCEPT / REJECT / COUNTER`를 판단하도록 설계했습니다.

즉, **전략적 판단과 자원 배분은 중앙에서 담당하고, 실제 실행 가능성과 현장 조건 판단은 Local Agent에서 담당하는 구조**입니다.

이러한 책임 분리가 실제 멀티에이전트 시스템에서도 적절한지 궁금합니다. 중앙과 Local Agent가 동일한 조건을 중복해서 판단하거나, 서로 다른 시점의 상태 정보를 바탕으로 상반된 결론을 내리는 상황을 고려했을 때 다음 책임을 어디에 두는 것이 적절한지도 조언받고 싶습니다.

- 중앙이 반드시 판단해야 하는 조건
- Local Agent가 최종 결정권을 가져야 하는 조건
- 중앙과 Local Agent가 함께 검증해야 하는 공통 조건
- Local Agent의 `REJECT / COUNTER`를 중앙이 재평가할 때 필요한 정보
- 통신이 지연되거나 단절됐을 때 Local Agent가 독립적으로 실행·중단할 수 있는 범위

목표 설계는 중앙이 명확한 전역 하드 제약으로 후보를 먼저 필터링하고, Local Agent가 최신 로컬 상태를 이용해 실행 가능성을 확인하며, ACCEPT된 제안을 별도의 Safety Layer가 검증해 ALLOW한 경우에만 실행하는 것입니다. 현재 Mock에서는 후보 필터와 Local/Safety 판단을 단순화해 흐름만 확인했습니다. 이 목표 구조에서 책임이 중복되거나 빠진 부분이 있는지도 검토받고 싶습니다.

### 질문 2 — 실제 통합 시 발생하는 실행 문제

현재 PX4/Gazebo를 실제로 실행하면서 예상하지 못했던 CPU 자원 문제가 먼저 확인됐습니다. 아직 전체 모듈을 연결한 통합 시뮬레이션은 실행하기 전인데, 멘토님 경험상 멀티에이전트와 시뮬레이터를 실제로 통합했을 때 설계 단계에서는 잘 드러나지 않다가 실행 과정에서 주로 발생하는 문제에는 어떤 것들이 있는지 궁금합니다.

특히 다음 위험을 예상하고 있습니다.

- 상태 동기화
- 통신 지연 및 메시지 유실
- 모듈별 시간 기준 차이
- 동일 자원에 대한 중복 할당
- 반복적인 재계획
- 오래된 상태를 기반으로 한 의사결정
- PX4/Gazebo 실행에 따른 CPU 병목

이 중 초기 MVP 단계에서 반드시 먼저 검증해야 할 위험과 권장 검증 방법에 대해 조언받고 싶습니다.

### 질문 3 — 병렬 개발 이후의 통합 위험

환경모델, Orchestrator, UAV, UGV, Safety 등 여러 파트를 병렬로 개발한 뒤 통합해야 합니다.

이러한 멀티모듈 프로젝트에서 통합 직전에 문제가 커지는 주요 원인은 무엇이며, 팀장이 초기에 어떤 협업 규칙과 기준을 마련하면 이를 줄일 수 있는지 궁금합니다.

현재 다음 사항은 정해둔 상태입니다.

- 통합계약
- 공통 Schema와 Enum
- Mock과 실제 모듈 구분
- Connector 경계
- 변경사항 공유 기준
- 공통 ID 및 로그 기준

이 외에 실무에서 놓치기 쉬운 버전 호환성, 타임아웃, 재시도, 메시지 중복 처리, 장애 책임 범위 등에 대해서도 조언받고 싶습니다.

## 6. 다음 일주일 계획

1. 하드 제약 기반 후보 필터 구현
2. 배터리·ETA·거리·상태 기반 후보 평가 기준 작성
3. IT-01/IT-02 테스트 및 로그 보강
4. 후보 없음·재평가 한도 초과 등 실패 시나리오 추가
5. 실제 Environment/UAV Connector와 교체 가능한 경계 검증
6. 상태 유효시간과 통신 지연 처리 방식 정리
7. 멘토 피드백을 바탕으로 AI 적용 범위와 MVP 범위 확정

## 7. 멘토링 시 설명할 핵심

> 통합 담당자가 만든 기존 Mock Connector와 통합 흐름을 활용해 오케스트레이션 시연 실행기와 로그 출력을 보강하고, IT-01과 IT-02의 반복 실행을 확인했습니다. 현재는 READY 상태이며 이번 Task 처리에서 제외되지 않은 첫 번째 UAV를 선택합니다. Local 응답과 Safety 판단, 실행·환경 갱신은 단순화한 Mock이며, 다음 단계에서 같은 Connector 인터페이스 안에 하드 제약 필터와 후보 평가 로직을 추가할 예정입니다.
