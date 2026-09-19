# UGV Local Agent — 지상자원(UGV/소방차) 모듈

지상자원의 접근성 판단과 경로 계획을 담당한다.
도로망 그래프 위에서 ETA를 계산하고, Task 수행 가능 여부를
`ACCEPT` / `REJECT` / `COUNTER` 로 응답한다.

UAV가 직선거리로 이동하는 것과 달리 지상자원은 도로를 따라간다.
직선거리는 가깝지만 도로 소요시간은 더 긴 비대칭이 발생하고,
이 판단이 오케스트레이터의 자원 선택 근거가 된다.

## 구성

| 파일 | 설명 |
|---|---|
| `models.py` | Node, Road, RouteResult 자료형 |
| `graph_data.py` | 도로망 데이터. 로직과 분리되어 교체 가능 |
| `graph_data_demo.py` | PX4 시연용 축소 도로망 (거리 1/100) |
| `road_graph.py` | 경로 탐색(Dijkstra), 도로 상태 갱신 |
| `geo.py` | 위경도 ↔ 로컬 NED 미터 변환 |
| `resource.py` | GroundResource — 자원 상태 |
| `agent.py` | GroundResourceAgent — 판단·상태·실행 |
| `fleet.py` | GroundFleet — 자원 전체 관리, 커넥터 진입점 |
| `config.py` | 자원 배치, 임계값 (전부 잠정값) |
| `drivers/base.py` | MotionDriver 인터페이스 |
| `drivers/px4.py` | PX4Driver — MAVSDK 연동 |
| `drivers/sim.py` | SimDriver — PX4 없이 동작 |
| `tools/px4-start.sh` | SITL 인스턴스 기동 |
| `tools/px4-stop.sh` | 프로세스·파라미터 정리 |
| `tools/check_px4.py` | 연결 진단 |
| `demo_ugv.py` | 재할당 시나리오 시연 |

## 실행

```bash
# 판단 로직만 (PX4 불필요)
python3 -m ugv.demo_ugv

# PX4 연동
./ugv/tools/px4-start.sh 0        # 터미널 1
./ugv/tools/px4-start.sh 1        # 터미널 2
python3 -m ugv.demo_ugv --px4     # 터미널 3

# 연결 진단
python3 ugv/tools/check_px4.py

# 정리 (파라미터 오염 시 --params)
./ugv/tools/px4-stop.sh --params
```

## 시연 시나리오

```
[평상시]
  A-ugv1  ACCEPT  eta=46s   경로=A -> J1 -> F1
  B-ugv1  ACCEPT  eta=69s   경로=B -> J3 -> F1

[E3 차단 — 화재 확산으로 북부 진입로 통제]
  A-ugv1  ACCEPT  eta=122s  경로=A -> J1 -> J2 -> F1   ← ETA 역전
  B-ugv1  ACCEPT  eta=69s

[E2 추가 차단 — A 거점 도달 불가]
  A-ugv1  REJECT  ROAD_BLOCKED
  B-ugv1  ACCEPT  eta=69s                              ← 재할당
```

도로 차단 → ETA 역전 → 재할당이라는 폐루프를 검증한다.
IT-03(지상자원 재할당) 초안에 해당한다.

## 현재 상태

| 구분 | 내용 |
|---|---|
| 실제 구현 | 도로 그래프·Dijkstra, 자원 상태 관리, 접근성·ETA 판단, 재할당 시나리오, 드라이버 추상화 |
| 실제 확인 | PX4 rover SITL — 연결·텔레메트리·arm·미션 업로드·실행·주행, 다중 인스턴스 동시 제어 |
| 단순화(Mock) | 도로망 데이터, 연료 소모율 1%/km 가정 |
| **미해결** | **PX4 rover가 웨이포인트에 기착하지 못함 (아래 참조)** |

## 미해결 — PX4 rover 웨이포인트 기착 실패

**증상**: 경로 계획 결과가 PX4에 정확히 전달되고 로버가 주행을 시작하지만,
웨이포인트 반경 안으로 진입하지 못하고 옆을 지나쳐 계속 주행한다.
`mission_result.seq_reached` 가 `-1` 에서 갱신되지 않는다.

**확인된 정상 동작**

```
position_setpoint_triplet
  current:  lat 38.114150  lon 128.157450  valid: True  type: 0   ← J3 좌표 정확
  next:     lat 38.114650  lon 128.156940  valid: True  type: 0   ← F1 좌표 정확
  acceptance_radius: 10.00000                                     ← 설정값 반영됨

estimator_status_flags
  cs_yaw_align: True   cs_mag_hdg: True   cs_heading_observable: True
  fs_bad_hdg: False    filter_fault_flags: 0

estimator_status
  hdg_test_ratio: 0.00068    pre_flt_fail_innov_heading: False
```

경로 전달, 자세 추정, 자력계 융합 모두 정상이다.

**문제 지점**

```
pure_pursuit_status
  target_bearing:      2.05965    ← 조향 목표
  bearing_to_waypoint: 2.05965    ← 웨이포인트 방향 (일치)
  crosstrack_error:    -129.6     ← 경로에서 이탈, 감소하지 않음
```

pure pursuit 이 계산한 조향 목표는 웨이포인트 방향과 일치하는데,
로버의 실제 주행 방향이 그와 다르다. 결과적으로 경로선과 평행하게 주행하며
`crosstrack_error` 가 수렴하지 않는다.

**시도한 것**

| 파라미터 | 시도 | 결과 |
|---|---|---|
| `PP_LOOKAHD_GAIN` / `MIN` / `MAX` | 1/1/10 → 5/15/60 → 2/5/60 | `target_bearing` 이 웨이포인트 방향과 일치하게 됨. 주행 방향은 여전히 불일치 |
| `RD_TRANS_DRV_TRN` | 3.1416(기본) ↔ 0.5 | 값을 낮추면 출발 시 제자리 회전이 기대되나 발생하지 않음 |
| `NAV_ACC_RAD` / MissionItem `acceptance_radius_m` | 2 → 10 → 15 | 값은 반영되나 도달 판정 변화 없음 |
| `MISSION_ALT_M` / `PX4_HOME_ALT` | 2.0/500 → 0.0/0 | `Takeoff detected` 는 여전히 발생 |

PX4 rover 지원은 공식 문서에 **experimental** 로 명시되어 있으며,
유사 증상이 이슈 [#21353](https://github.com/PX4/PX4-Autopilot/issues/21353) 로 보고되어 있다.

**다음 확인 대상**: `rover_steering_setpoint` 와 `vehicle_local_position.heading` /
`vx` / `vy` 의 부호 일치 여부.

## 환경 설정 함정 (확인된 것)

다중 인스턴스 기동과 파라미터 설정에서 다음을 확인했다.

**`SYS_HAS_MAG = 0` 으로 설정하지 말 것**
자력계를 끄면 `yaw_align` 이 서지 않아 `no heading reference` 로 arm 이 거부된다.
`EKF2_MAG_TYPE` 도 기본값(0) 을 유지한다.

**파라미터는 `fs/parameters.bson` 에 저장되어 재시작해도 남는다**
잘못된 값을 넣으면 재시작만으로 복구되지 않는다.
`px4-stop.sh --params` 로 파일을 지우거나 `param reset_all` 후 `param save`.

**`RD_TRANS_DRV_TRN` 의 단위는 라디안**
공식 문서에는 `deg` 로 표기되어 있으나 실제 기본값이 `3.1416`(π)이다.
문서대로 3.14를 "도" 로 알고 넣으면 heading 오차 3.14도에서 제자리 회전이 걸려
출발하자마자 회전만 반복한다.

**`PX4_HOME_LAT` / `LON` / `ALT` 는 셋 다 설정해야 한다**
하나라도 빠지면 `PX4_HOME_LAT, PX4_HOME_LON and PX4_HOME_ALT must all be set` 으로 기동 실패.
`ALT` 는 Gazebo 월드 지면 기준 0.

**macOS 에서 `GZ_IP=127.0.0.1` 필요**
없으면 `Exception sending a multicast message: No route to host` 로 Gazebo 연결 실패.

**다중 인스턴스는 `bin/px4 -i N` 으로 직접 실행**
`make px4_sitl` 은 항상 인스턴스 0 으로 뜨므로 두 번째가 `already running` 으로 막힌다.
포트는 `14540 + N`.

**`-s` 옵션으로 파라미터 스크립트를 넘기지 말 것**
시스템 셸 스크립트로 실행되어 `param: command not found` 가 나고,
기본 `rcS` 를 대체해 Gazebo 가 기동하지 않는다. 파라미터는 MAVLink 로 설정한다.

**기동 후 EKF 수렴까지 대기 필요**
`GPS Horizontal Pos Drift too high` 경고가 멈출 때까지 약 60초.
그 전에 arm 하면 위치 추정이 발산한 상태로 주행한다.

**`mavsdk` 패키지가 `mavsdk-grpc` 로 이름 변경됨**
기존 `mavsdk` 이름은 이제 다른 API(네이티브 바인딩)를 가리킨다.
`from mavsdk import System` 은 동작하지 않으며 `from mavsdk_grpc import System` 을 사용한다.
경고문이 안내하는 `import mavsdk_grpc as mavsdk` 는 `from` 절에는 적용되지 않는다.

**v1.18-beta 에서 `RD_` 파라미터 대부분이 이름 변경됨**
`param show RD_*` 결과 `RD_TRANS_DRV_TRN`, `RD_TRANS_TRN_DRV`,
`RD_WHEEL_TRACK`, `RD_YAW_STK_GAIN` 4개만 존재한다.
공식 문서(v1.16 기준)의 `RD_MISS_SPD_DEF`, `RD_MAX_ACCEL` 등은 없다.

## 설계 결정

**알고리즘 교체 가능** — `RouteAlgorithm` 프로토콜로 분리.
현재 Dijkstra, 이후 A* 또는 LLM 기반 대안 경로 탐색 검토.

**데이터 주입** — `RoadGraph(nodes, roads)` 생성자로 받는다.
`graph_data.py` 만 교체하면 실제 도로 데이터로 전환된다.

**구조와 상태 분리** — 도로망 구조(노드·간선·거리)는 본 모듈이 소유하고,
도로 상태(차단·혼잡)는 환경모듈에서 받는다.
`Node` 는 불변, `Road` 의 `blocked` / `congestion` 은 가변이다.

**드라이버 추상화** — `MotionDriver` 인터페이스 뒤에 PX4 와 시뮬레이션을 둔다.
판단 로직은 PX4 없이 테스트된다.

**RoadGraph 단일 인스턴스** — `GroundFleet` 이 하나만 만들어 전 자원이 공유한다.
도로 차단이 모든 자원의 판단에 즉시 반영된다.

**노드·간선 id 는 내부 표현** — 외부 스키마에는 노출하지 않는다.
오케스트레이터와는 위경도로만 주고받는다.

## 추후 과제

- 국가표준노드링크 또는 지리정보 API 로 실제 도로망 확보
- 환경모듈 Raster ↔ Road Graph 좌표 변환 (통합계약 §8 합의 필요)
- 주행 중 현재 위치의 간선 판정 (`_locate_on_graph`)
- 위경도 목표 → 그래프 노드 변환
- 연료 왕복 여유 판단 (`RETURN_MARGIN_INSUFFICIENT`)
- 실제 센서 연동 후 `observation()` 의 `value` 채우기