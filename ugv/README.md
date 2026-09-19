# UGV Local Agent

지상자원(UGV·소방차)의 접근성 판단과 경로 계획을 담당한다.
도로망 그래프 위에서 ETA를 계산하고, Task 수행 가능 여부를
ACCEPT / REJECT / COUNTER 로 응답한다.

UAV가 직선거리로 이동하는 것과 달리 지상자원은 도로를 따라간다.
이 비대칭성(직선거리는 가깝지만 도로 소요는 더 긴 경우)이
본 모듈이 별도로 존재하는 이유다.

## 구성

| 파일 | 설명 |
|---|---|
| `models.py` | Node, Road, RouteResult 자료형 |
| `graph_data.py` | 샘플 도로망. 교체 가능하도록 로직과 분리 |
| `road_graph.py` | 경로 탐색(Dijkstra), 도로 상태 갱신 |
| `resource.py` | Ground Resource 공통 상태 (UGV/FIRE_ENGINE) |
| `evaluator.py` | 수행 가능성 판단 |
| `agent.py` | Orchestrator 인터페이스 |
| `geo.py` | 위경도 <-> 로컬 NED 미터 변환 (기준점 방식) |
| `drivers/base.py` | `MotionDriver` 추상 인터페이스 |
| `drivers/px4.py` | `PX4Driver` — PX4/Gazebo MAVSDK 연동 |
| `drivers/sim.py` | `SimDriver` — PX4 없이 선형 보간으로 이동 (테스트·시연) |
| `tools/` | 연결 확인·이동 시험 스크립트 (`check_px4.py`, `try_move.py`) |

## 현재 상태

| 구분 | 내용 |
|---|---|
| 실제 구현 | 그래프 자료구조, Dijkstra, 도로 상태 갱신 API, PX4 MAVSDK 드라이버(`goto`) |
| 단순화(Mock) | 도로망 데이터 — 8노드 가상 그래프 |
| 미구현 | 환경모듈 연동 |

## 설계 결정

**알고리즘 교체 가능** — `RouteAlgorithm` 프로토콜로 분리.
현재 Dijkstra, 이후 A* 또는 LLM 기반 대안 경로 탐색 검토.

**데이터 주입** — `RoadGraph(nodes, roads)` 생성자로 받는다.
`graph_data.py`만 교체하면 실제 도로 데이터로 전환된다.

**구조와 상태 분리** — 도로망 구조(노드·간선·거리)는 본 모듈이 소유하고,
도로 상태(차단·혼잡)는 환경모듈에서 받는다.

## 추후 과제

- 국가표준노드링크 또는 지리정보 API로 실제 도로망 확보
- 환경모듈 Raster ↔ Road Graph 좌표 변환 (통합계약 §8 합의 필요)
- PX4 rover 경로 실행 — 공식 API가 ROS 2 기반이라 MAVSDK Mission 지원 여부 확인 필요