## Integration

- `01_integration_contract.md`
  - 환경·UAV·UGV·소방차·Orchestrator·Safety Layer를 연결할 때 사용하는 공통 인터페이스 규약
  - ID, 상태값, 좌표·단위·시간, Connector 경계, 로그 기준 등을 정의

- `02_integration_test_cases.md`
  - 초기 통합 단계의 최소 필수 테스트
  - 정상 폐루프와 `REJECT → 재평가 → 재할당` 흐름을 검증

- `03. 통합 작업 기록 (2026-10-01~02).md`
  - 팀원 PR 통합과 main 반영 과정, 총괄 팀별 요청서 INT-01~05 진행 상황
  - 서버 일괄 실행(`run_servers.py`), 관제 로그 보강, GitHub 자동 시험
