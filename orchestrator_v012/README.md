# Orchestrator v0.1.2 연결 시연 코드

기존 저장소 Xn0w/Codyssey_M2-1에 추가하는 패키지입니다. 독립 실행 프로젝트가 아닙니다.
검증 기준 커밋: `8bfe9716a776d762e60f2561a5b07a878f962502`.
기존 팀 파일은 변경하지 않았습니다. 기존 main.py 또는 관제 UI에 자동 연결되지 않으며 별도 실행 명령을 사용합니다.

## 로컬 적용 및 PR

1. 기존 저장소를 VS Code로 열고, 작업 중인 변경 사항을 먼저 확인합니다.
2. 저장소 루트의 터미널에서 `git switch -c feat/orchestrator-v012`로 브랜치를 만듭니다.
3. ZIP의 `orchestrator_v012` 폴더를 저장소 루트(main.py와 같은 위치)에 복사합니다.
   같은 이름의 폴더가 이미 있으면 덮어쓰지 말고 먼저 비교하세요.
4. 아래 설치·테스트 명령을 실행합니다.
5. 통과 후 다음 명령으로 이 폴더만 커밋하고 푸시합니다.

```powershell
git status --short
git add orchestrator_v012
git diff --cached --stat
git commit -m "feat: add orchestrator v0.1.2 simulation integration"
git push -u origin feat/orchestrator-v012
```

GitHub에서 해당 브랜치의 Compare & pull request를 선택합니다. 저장소의 실제 통합 대상 브랜치를 확인한 뒤 PR을 생성하세요.

## 설치·실행

검증 환경은 Linux / Python 3.12.14입니다. Windows 및 Python 3.14에서의 실행은 미검증입니다.
기존 환경 Raster 원본 파일과 environment, ugv, px4/uav-agent, interfaces, connectors, logger.py, config.py가 필요합니다.
ZIP에는 기존 팀 코드와 대용량 Raster가 들어 있지 않습니다. 기존 저장소에 추가해서 사용하세요.

Windows PowerShell 예시(저장소 루트):

```powershell
py -3.12 -m venv .venv-orchestrator
.\.venv-orchestrator\Scripts\python.exe -m pip install -r orchestrator_v012/requirements.txt
.\.venv-orchestrator\Scripts\python.exe -m unittest discover -s orchestrator_v012/tests -v
.\.venv-orchestrator\Scripts\python.exe -m tests.it_cases
.\.venv-orchestrator\Scripts\python.exe -m orchestrator_v012.demo --scenario all
```

Python 3.12가 설치되어 있어야 합니다. Linux에서는 가상환경 Python으로 동일한 `-m` 명령을 실행합니다.
시연은 localhost에 기존 UAV API 서버 2개를 Mock 모드로 실행하고 종료합니다.
결과 로그는 `logs/orchestrator_v012/`, 연결 테스트 로그는 `logs/orchestrator_v012_verified/`에 생성됩니다.
로그나 가상환경은 PR에 추가하지 마세요.

## 파일 책임

| 파일 | 역할 |
|---|---|
| models.py | 기존 공통 모델을 상속한 원본 데이터 보존 모델 |
| uav_adapter.py | 기존 UAV HTTP API 조회·평가·실행과 데이터 보존 |
| runtime.py | 기존 환경 모델 및 GroundFleet 호출 경계 |
| selector.py | 최근접 UAV, 풍속 전환, 제한된 COUNTER 수용, UGV 최소 ETA·2km 판단 |
| runner.py | 선택→기존 Safety→실행→관측 반환, 이벤트 기록 |
| demo.py | 기존 모듈을 연결하는 7가지 시연 |
| tests/ | 정책·데이터 보존·연결 테스트 |

## 확인된 결과

기존 테스트 기록은 TEST_RESULTS.txt에 포함했습니다. 17개 테스트 메서드 통과(그중 연결 테스트 1개에 7개 시나리오 포함), 기존 IT-01/IT-02 통과입니다.

| 시나리오 | 확인 결과 |
|---|---|
| normal | UAV 실행·모의 열화상 반환·환경 반영 |
| moderate | MODERATE_WIND COUNTER 원본 유지 + 정책 수용 로그 |
| strong | UGV 도착, 화재 관측 미완료로 수동 검토 |
| uav_rejected | UAV 2대 거절 후 UGV 도착, 수동 검토 |
| blocked | 접근 가능한 자원 없음 → 수동 검토 |
| distant | 관측거리 2km 초과 → 수동 검토 |
| unknown_target | UGV 목적지 정보 없음 → 수동 검토 |

## 검증 범위와 남은 연동

- 실제 환경 CA 코드와 Raster, 기존 UAV HTTP API(MockDrone), GroundFleet(SimDriver), 기존 Mock Safety를 연결했습니다. 실제 PX4·센서·Safety 검증이 아닙니다.
- UGV target_node는 시연 입력으로 F1을 명시합니다. 화재 좌표에서 도로 노드를 자동 선정하는 기능은 없습니다.
- UGV 관측 반환은 ROAD_STATUS이므로 도착을 화재 관측 완료로 처리하지 않습니다. ARRIVED_OBSERVATION_PENDING 및 수동 검토 이벤트를 남깁니다.
- 대부분의 시연은 연료량 0인 셀(37,2)을 강제 발화시킨 연결·경로 검증용 입력입니다. 실제 산불 확산 재현 검증이 아닙니다. distant는 (19,142)를 사용합니다.
- 2,000m는 검증된 센서 성능이 아닌 시뮬레이션 가정입니다.
- 환경 어댑터는 선택에 필요한 최소 입력만 연결하며 risk_zone은 빈 목록입니다.
- 이벤트는 로그·전달 경계까지입니다. 관제 승인/ACK UI 연결은 구현하지 않았습니다.
- AI API는 호출하지 않습니다. 자원 선택부를 나중에 교체할 수 있도록 경계를 둡니다.
- 팀 담당자 후속 확인: 자동 target_node 변환, UGV 화재 관측 데이터, 실제 Safety, 관제 이벤트 연결 및 공통 Connector 데이터 보존.

PR에는 위 검증 범위를 그대로 기재하고, 전체 시스템의 실제 운용 완료로 표현하지 마세요.
