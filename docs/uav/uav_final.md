# UAV 담당 협의사항 정리본 — 초기 시뮬레이션 정책 수정본

작성일: 2026-09-17  
대상: UAV Local Agent / Orchestrator / Environment / Safety / Integration

---

## 1. 목적

UAV 담당의 `NOTE.md`, `SHARE.md`에서 나온 질문을 기준으로, **현재 확정된 공통 규칙과 아직 실측이 필요한 항목만 정리**한다.

현재는 통합 시뮬레이션 전 단계다. 선행연구와 공식 사양은 설계 방향과 시작값을 정하는 참고 근거로 사용하며, 본 프로젝트에서 성능을 검증했다는 의미는 아니다. 우선 아래 정책으로 구현·시뮬레이션을 시작하고, 결과와 기종 선정에 따라 수치 및 모델을 점차 조정한다. 확정은 현재의 설계 합의를 뜻하며 구현 완료나 실측 검증을 뜻하지 않는다.

구분:

- **확정**: 팀 공통 설계로 사용
- **잠정**: 프로젝트 기준으로 우선 사용하되 실측 후 조정 가능
- **TBD**: UAV 담당의 기체 설정·실측 결과가 필요
- **프로젝트 고유**: 선행연구와 모순되지는 않지만 표준이라고 표현하지 않음

---

# 2. 한눈에 보는 최종 결정

| 항목 | 결정 | 상태 |
|---|---|---|
| 고도 기준 | Task는 AGL로 전달하고 이동 경로에서도 지형 대비 일정 고도 유지 | 확정 |
| 초기 운용 고도 | **70~120m AGL 범위** 안에서 시나리오별 목표고도 지정 | 잠정 |
| 좌표 변환 | `cell_id ↔ lat/lon`은 **총괄 계층 Spatial Mapper** 담당 | 확정 |
| 풍속 출처 | 총괄 판단 시 Environment 값을 기준으로 사용, 수행 곤란 보고 시 기체 수집값 반영 | 확정 |
| 임무 중 수행 곤란 | 기체 보고 → 총괄 환경값 반영·재계산 → 임무 재부여; 실행 중 REJECT 기체는 복귀 | 확정 |
| Gazebo 물리풍 | MVP 필수 아님 | 확정 |
| 풍속 기준 | 8m/s 이상 경계, 10m/s 이상 `REJECT` | 잠정 |
| UAV 판단 | health / capability / 통신 / busy / 풍속 / 배터리 / ETA | 확정 |
| 응답 | `ACCEPT / REJECT / COUNTER` | 프로젝트 고유 |
| 배터리 reserve | 실측·기체 근거 후 결정 | TBD |
| 실제 PX4 동시 대수 | 1대 안정화 후 CPU/RAM 실측으로 확대 여부 결정 | TBD |

---

# 3. 데이터 흐름

```text
Environment
  ├─ fire / risk / wind / terrain
  └─ grid(cell_id)
        ↓
Orchestrator 계층
  ├─ cell_id ↔ lat/lon 변환
  ├─ Task 생성
  └─ 현재 환경값 포함
        ↓
UAV Local Agent
  └─ 수행 가능성 평가
        ↓
ACCEPT ─────────────→ Safety Layer → ALLOW → UAV Connector / PX4 실행
REJECT / COUNTER ──→ Orchestrator 재평가
        ↓
Observation / Result
        ↓
Orchestrator / Environment 갱신
```

핵심 원칙:

1. **총괄은 임무와 공통 좌표·환경정보를 정한다.**
2. **UAV Local Agent는 현재 자기 자원이 그 임무를 수행 가능한지 판단한다.**
3. `ACCEPT`만으로 비행하지 않고 **Safety `ALLOW` 이후 실행**한다.
4. PX4는 저수준 비행제어와 자체 failsafe를 담당한다.

---

# 4. UAV 담당 질문별 결정

## 4.1 고도는 어떻게 전달하는가?

산악지형에서는 절대 해발고도만 고정하면 지면과의 실제 거리가 크게 달라질 수 있으므로, **Task의 고도 의미는 AGL로 통일**한다.

예:

```json
{
  "target_lat": 38.0425,
  "target_lon": 128.2541,
  "target_agl_m": 80
}
```

- 70~120m는 **프로젝트 초기 운용 범위**다.
- 실제 Task에는 범위가 아니라 `target_agl_m = 80`처럼 **구체적인 목표값 하나**를 전달한다.
- 선행연구에는 50m, 100m, 122m, 150m AGL 등 다양한 사례가 있으므로, 70~120m를 **보편적 표준**이라고 표현하지 않는다.

책임:

- Environment: 지형고도/지형 데이터 제공
- Orchestrator: 임무의 목표 AGL 결정
- UAV Connector: 경로상의 지형고도와 목표 AGL을 이용해 PX4가 사용할 고도 표현으로 변환한다. AMSL 사용 시 동일한 수직 기준의 지형고도에 목표 AGL을 더한다.
- PX4: 실제 목표고도 추종

> 이동 중에도 지형을 따라 목표 AGL을 일정하게 유지하는 terrain-following을 요구사항으로 한다. 목표 지점에서만 AGL을 맞추는 의미가 아니다. 초기에는 시뮬레이션 지형 데이터를 이용해 구현하며, 경로 샘플링 간격·고도 추종 허용오차는 시뮬레이션으로 조정한다. 이륙 지점 기준 상대고도를 AGL로 간주하지 않는다.

---

## 4.2 `cell_id`를 위·경도로 누가 바꾸는가?

**총괄 계층의 공통 Spatial Mapper가 담당한다.**

```text
Environment: cell_id
→ Spatial Mapper
→ lat/lon
→ UAV Connector
→ PX4/MAVSDK
```

UAV가 grid 원점, cell size, numbering 규칙까지 알 필요는 없다.

반대로 UAV가 화재를 먼저 발견하면:

```text
UAV GPS observation
→ lat/lon
→ Spatial Mapper
→ grid/cell 역매핑
→ Environment 갱신
→ 새 Task 또는 재평가
```

즉:

- **공간 의미 변환:** 총괄 계층
- **위치 측정:** 각 자원
- **PX4 명령 변환:** UAV Connector

이 책임 배분은 중앙 mission-planning 구조와 양립하지만, 문헌의 표준 모듈 배치라고 주장하지 않는다.

---

## 4.3 풍속은 어디서 오는가?

**총괄이 임무를 판단하는 시점에는 Environment의 환경값을 기준으로 한다.** 이후 기체가 수행 곤란을 판단하면 기체에서 수집한 환경변수를 총괄에 보고하고, 총괄이 해당 관측을 반영해 다시 계산한다.

```text
Environment wind
→ Orchestrator가 Task 평가 요청에 포함
→ UAV Local Agent가 판단
```

예:

```json
{
  "task_id": "TASK_101",
  "target_lat": 38.0425,
  "target_lon": 128.2541,
  "target_agl_m": 80,
  "environment": {
    "wind_speed_mps": 6.4,
    "wind_direction_deg": 250,
    "observed_at": "2026-09-17T01:00:00Z"
  }
}
```

UAV Local Agent가 Environment API를 직접 조회하지 않는다. PX4 SITL + MAVSDK의 wind telemetry 가용성은 선택 버전과 구성에서 확인한다. 초기에는 시나리오 입력 또는 모의 관측을 사용할 수 있으며, 실제 수집값과 모의 값을 구분한다. “PX4는 풍속을 제공할 수 없다”고 일반화하지 않는다.

---

## 4.4 임무 중 수행이 어려워지면?

초기 임무는 총괄 판단 시점의 환경값을 기준으로 진행한다. 이후 UAV Local Agent가 임무 수행이 어렵다고 판단하면 사유와 기체에서 수집한 환경변수·관측시각·위치를 총괄에 보고한다. 총괄은 보고값을 Environment에 반영하고 수행 가능성을 다시 계산해 임무를 부여한다.

```text
총괄 판단 시점의 환경값 → 평가 → ACCEPT → Safety ALLOW → 수행
→ 기체가 수행 곤란 판단
→ 사유 + 수집 환경변수 + 현재 상태 + 임무 진행상황 보고
→ 총괄이 환경·자원 상태 반영 후 재계산
→ 원인별 대체 자원 평가 → Safety ALLOW → 임무 부여
```

실행 중 `REJECT`한 UAV는 현재 관측 임무를 중단하고 복귀한다. 신규 임무를 출발 전에 거절한 경우에는 해당 임무를 시작하지 않는다. 복귀 명령 전달과 복귀 상태 확인은 UAV Connector가 담당하며, PX4 자체 failsafe가 활성화된 경우 해당 동작을 우선한다.

| 실행 중 거절 사유 | 기존 UAV | 총괄의 후속 처리 |
|---|---|---|
| 풍속으로 UAV 관측 불가 | 임무 중단·복귀 | 자율주행차(UGV)의 도로 접근·관측 가능성을 평가하고 가능하면 관측 임무 부여 |
| 풍속으로 UAV 불가 + 화재가 도로에서 너무 멀어 UGV 관측도 불가 | 임무 중단·복귀 | 해당 화재의 자동 관측 임무와 대체 할당을 모두 취소하고 사람에게 “드론과 자율주행차로 관측이 어렵다”고 경고 |
| 배터리 부족 | 임무 중단·복귀 | 배터리가 충분하고 나머지 조건도 만족하는 다른 UAV에 남은 관측 임무를 인계 |

취소 범위는 해당 화재의 관측 임무이며 다른 화재의 임무까지 취소하지 않는다. 배터리 문제로 인계할 때는 완료한 관측과 남은 관측을 전달한다. 대체 UAV가 없으면 관측 중단 상태를 총괄에 남기고 사람에게 알린다. 복귀와 재할당은 각각 상태를 추적하며, 다른 기체의 수락만으로 기존 기체가 복귀 완료했다고 간주하지 않는다.

UGV가 관측 가능한 도로 이격거리의 시작값은 도로·센서 시나리오 구성 시 정하고 시뮬레이션 후 조정한다.

Gazebo 물리풍 동기화는 후속 선택사항이다. 물리풍을 적용하지 않는 단계에서는 환경값에 따른 판단·복귀 명령·재할당 흐름을 검증한다. 이 결과를 실제 기체의 내풍 성능 검증으로 해석하지 않는다.

---
# 5. UAV Local Agent 판단 기준

## 5.1 즉시 수행 불가 조건

| 조건 | 기본 처리 |
|---|---|
| failsafe 활성 | `REJECT` |
| GPS/필수 센서 이상 | `REJECT` |
| 필요한 센서 미탑재 | `REJECT` |
| 다른 Task 수행 중에 신규 Task 요청 | `REJECT / BUSY` |
| 풍속 한계 초과 | `REJECT / HIGH_WIND` |
| 통신 불가 | 응답 불가 가능성 있으므로 Orchestrator timeout/heartbeat와 함께 처리 |

`한 UAV가 동시에 Task 하나만 수행`은 **MVP 단순화 규칙**이다. 현재 수행 중인 동일 Task의 재평가는 허용하며 BUSY로 거절하지 않는다. 다른 Task의 신규 요청만 BUSY로 거절한다. 임무 교체는 기존 임무 중단·취소 상태를 확인한 뒤 수행한다. 신규 요청에 대한 BUSY 응답은 현재 임무를 중단하거나 복귀하라는 의미가 아니다.

---

## 5.2 배터리/에너지

평가 개념:

```text
목표까지 이동
+ 상승/하강
+ 관측 체류
+ 복귀
+ 안전 여유
```

- 필요한 에너지가 부족 → `REJECT / INSUFFICIENT_BATTERY`
- 수행 가능하지만 여유가 적음 → `COUNTER` 후보
- 충분함 → 다음 평가

배터리/에너지를 task feasibility에 사용하는 것은 UAV MRTA 연구와 정합적이다.

다만 **“복귀 후 20% 미만이면 COUNTER”는 보편 기준이 아니다.** `BATTERY_RESERVE_PCT`는 기체 설정과 실측 근거를 받은 뒤 확정한다.

---

## 5.3 ETA / deadline

- deadline 내 도착 가능 → 계속 평가
- 늦지만 수행 가능 → `COUNTER`로 가능한 도착시각 제시
- 늦으면 임무 가치가 사라짐 → `REJECT / DEADLINE_INFEASIBLE`

시간 제약을 task feasibility에 포함하는 것은 문헌과 양립한다. `COUNTER` 방식은 프로젝트 고유 설계다.

---

## 5.4 풍속

프로젝트 초기 기준:

```text
wind < 8.0 m/s       → 정상 평가
8.0 ≤ wind < 10.0    → COUNTER 또는 제한 운용 후보
wind ≥ 10.0 m/s      → REJECT / HIGH_WIND
```

근거:

- DJI Matrice 30 Series, Matrice 350 RTK 등 산업용 기체의 공식 최대 풍속 저항 **12m/s** 사례가 있다.
- PX4는 고정된 10m/s 기준을 강제하지 않고 `COM_WIND_MAX`와 `COM_WIND_MAX_ACT`로 사용자가 high-wind failsafe 기준과 동작을 설정한다.

8/10m/s는 **선행연구와 상용 기체의 공개 사양을 참고해 초기 시뮬레이션을 시작하기 위해 정한 잠정 시나리오 값**이다. 해당 수치가 대중적으로 통용되는 표준이거나 미선정 기체의 안전 한계로 검증되었다고 주장하지 않는다.

추후 기종을 정하면 해당 기체 사양과 시뮬레이션 결과를 바탕으로 변경할 수 있다. 물리풍 없는 시뮬레이션은 판단 로직 검증으로 한정한다. PX4 파라미터는 실제 사용하는 버전에서 지원 여부를 확인한다.

---

# 6. `ACCEPT / REJECT / COUNTER`

선행연구에는 feasibility evaluation, bidding/utility, dynamic reassignment가 존재한다. 다만 `ACCEPT / REJECT / COUNTER`라는 3단계 메시지는 **본 프로젝트가 정의한 인터페이스**다.

```text
ACCEPT  = 현재 조건으로 수행 가능
REJECT  = 현재 평가 요청을 수락할 수 없음
COUNTER = 특정 조건을 바꾸면 수행 가능
```

`COUNTER` 예:

- 관측시간 60초 → 30초로 줄이면 가능
- deadline에는 못 맞추지만 특정 시각 도착 가능
- 요구 조건 일부를 바꾸면 가능

Safety 조건을 Local Agent가 임의로 완화하는 용도로 사용하지 않는다. 실행 중 현재 Task에 대한 REJECT는 중단·복귀와 총괄 재계산으로 연결한다. 다른 신규 Task에 대한 REJECT/BUSY는 현재 Task의 수행에 영향을 주지 않는다.

---

# 7. UAV가 화재를 먼저 발견한 경우

UAV는 **관측 결과와 자기 위치를 보고**하고, 자원 재할당 판단은 총괄이 한다. 초기 시뮬레이션에서는 관측 시 UAV의 위·경도를 화재 위치로 간주한다. 이는 시작 단계의 명시적 단순화 가정이며, 실제 화재 위치를 계산한 결과는 아니다. 추후 카메라 자세·시선·지형고도를 이용한 화재 위치 추정으로 고도화한다.

```text
UAV Observation
- resource_id
- timestamp
- lat/lon
- altitude
- sensor_type
- detection_type
- confidence (사용 시)
- sensor result
        ↓
Spatial Mapper: lat/lon → grid/cell
        ↓
Environment update
        ↓
새 Task 또는 기존 Task 재평가
```

UAV가 “어느 자원을 투입할지”까지 판단하지 않는다.

---

# 8. PX4/Gazebo 실행 범위

PX4는 하나의 Linux/Gazebo 환경에서 **여러 PX4 instance를 실행하는 multi-vehicle simulation을 공식 지원**한다. 따라서 “UAV마다 서버 1대가 반드시 필요하다”는 표현은 사용하지 않는다.

현재 통합 시뮬레이션 전 단계이므로 **지원 가능 여부와 팀 서버에서 안정적으로 운용 가능한 대수는 구분**한다. 실제 수용 능력은 아래 실측으로 확인한다.

MVP:

```text
실제 PX4/Gazebo UAV: 1대 우선 안정화
Mock UAV: 여러 대
```

이후 실측:

- 1대 CPU/RAM
- 2대 CPU/RAM
- headless 장시간 안정성
- instance/port 분리

결과를 보고 실제 PX4 대수를 늘린다.

---

# 9. 최소 인터페이스 의미

필드명은 통합 담당이 최종 확정한다.

## UAV 상태

```json
{
  "resource_id": "UAV_A1",
  "timestamp": "...",
  "position": {
    "lat": 0.0,
    "lon": 0.0,
    "alt_amsl_m": 0.0,
    "alt_agl_m": 0.0
  },
  "battery_pct": 0.0,
  "flight_mode": "...",
  "health": {
    "gps_ok": true,
    "sensors_ok": true,
    "failsafe": false
  },
  "communication_state": "OK",
  "current_task_id": null
}
```

## 수행 가능성 평가 요청

```json
{
  "task_id": "TASK_101",
  "decision_id": "DEC_001",
  "target": {
    "lat": 0.0,
    "lon": 0.0,
    "target_agl_m": 80
  },
  "requirements": {
    "sensor_type": "THERMAL",
    "observe_duration_s": 60,
    "deadline": "..."
  },
  "environment": {
    "wind_speed_mps": 0.0,
    "wind_direction_deg": 0.0,
    "timestamp": "..."
  }
}
```

## Local Response

```json
{
  "task_id": "TASK_101",
  "decision_id": "DEC_001",
  "resource_id": "UAV_A1",
  "verdict": "ACCEPT | REJECT | COUNTER",
  "reason": "...",
  "counter_offer": null
}
```

원칙:

- `REJECT/COUNTER`는 `reason` 필수
- `ACCEPT`만 Safety로 전달
- `REJECT/COUNTER`는 Orchestrator 재평가
- 동일 Task 재평가와 재할당 시 새 `decision_id`를 발급하고 현재 결정과 일치하는 응답만 적용
- 요청은 현재 Task 재평가인지 신규 Task 평가인지 구분
- 실행 중 수행 곤란 보고에는 사유, 수집 환경변수, 관측시각·위치, 현재 배터리, 임무 진행상황 포함
- 초기 모의 관측과 실제 기체 수집값의 출처 구분
- 실행 중 현재 Task의 REJECT는 복귀 명령·상태 확인으로 연결; 다른 신규 Task의 BUSY 응답은 현재 임무 유지
- 복귀 명령과 완료 상태를 구분하고, 인계 시 남은 관측 범위를 전달

---

# 10. 아직 정해야 할 값

다음 항목은 관련 담당이 시나리오 시작값을 정하고, 시뮬레이션과 기종 선정 이후 근거·실측으로 조정한다.

- [ ] 기본 `target_agl_m` (70~120m 범위 안에서 시나리오 기본값)
- [ ] UAV airframe / 기체 모델
- [x] `WIND_COUNTER_MS = 8.0 m/s` — 잠정
- [x] `WIND_LIMIT_MS = 10.0 m/s` — 잠정
- [ ] `BATTERY_RESERVE_PCT` 또는 에너지 reserve 정책
- [ ] 순항속도 / 상승속도 / ETA 계산 기준
- [ ] 관측 체류시간 기본값
- [ ] 실제 PX4/Gazebo 동시 실행 대수
- [ ] 지형 추종 경로 샘플링 간격과 AGL 허용오차
- [ ] UGV 관측 가능 도로 이격거리 — 총괄/UGV 담당
- [ ] 모의 환경변수 입력과 실제 기체 수집값의 연동 방식

---

# 11. 선행연구·공식문서와의 관계

| 내용 | 판단 |
|---|---|
| UAV가 자기 상태·capability로 임무 가능 여부 평가 | 문헌과 정합 |
| 배터리·에너지 제약 | 문헌과 정합 |
| failure 후 재할당 | 문헌과 정합 |
| 환경 변화 후 재할당 | 문헌과 정합 |
| 통신 제약 고려 | 문헌과 정합 |
| `ACCEPT/REJECT/COUNTER` | 프로젝트 고유 |
| 8/10m/s 풍속 기준 | 프로젝트 잠정 기준 |
| 70~120m AGL | 프로젝트 운용 범위 |
| `cell_id ↔ lat/lon`을 총괄 계층에서 변환 | 문헌과 양립하는 자체 설계 |
| UAV마다 서버 1대 필수 | 근거 없음 / 사용하지 않음 |
| Gazebo 물리풍 동기화 필수 | 아님 |

---

# 12. 참고 근거

1. Skaltsis, Shin, Tsourdos (2023), **A Review of Task Allocation Methods for UAVs**, *Journal of Intelligent & Robotic Systems*.
2. Chakraa et al. (2023), **Optimization techniques for Multi-Robot Task Allocation problems**, *Robotics and Autonomous Systems*.
3. **Multi-UAV Task Assignment in Dynamic Environments: Current Trends and Future Directions**, *Drones* 9(1), 75.
4. Hu et al. (2022), **Fault-tolerant cooperative navigation of networked UAV swarms for forest fire monitoring**, *Aerospace Science and Technology*.
5. **A distributed task allocation approach for multi-UAV persistent monitoring in dynamic environments**, *Scientific Reports* 15, 6437 (2025).
6. **A Lightweight Remote Sensing Payload for Wildfire Detection and Fire Radiative Power Measurements**, *Sensors* 23(7), 3514 (2023).
7. **Thermal-Drones as a Safe and Reliable Method for Detecting Subterranean Peat Fires**, *Drones* 3(1), 23 (2019).
8. González-Jorge et al. (2017), **Low-altitude long-endurance solar unmanned plane for forest fire prevention**.
9. **Multi-Target Firefighting Task Planning Strategy for Multiple UAVs Under Dynamic Forest Fire Environment**, *Fire* 8(2), 61.
10. PX4 User Guide — **Multi-Vehicle Simulation with Gazebo**.
11. PX4 User Guide — **Safety / High Wind Failsafe**.
12. DJI Enterprise — **Matrice 30 Series / Matrice 350 RTK Specifications**.

---

## 문서 작성 시 피할 표현

다음처럼 단정하지 않는다.

- “산불 드론의 표준 고도는 70~120m다.”
- “10m/s 이상이면 모든 드론은 비행할 수 없다.”
- “배터리 20% 미만이면 무조건 복귀해야 한다.”
- “PX4는 풍속을 제공하지 않는다.”
- “드론 한 대마다 서버 한 대가 필요하다.”
- “ACCEPT/REJECT/COUNTER는 기존 연구의 표준 프로토콜이다.”

현재 프로젝트에서는 **문헌의 feasibility·energy constraint·dynamic reassignment 개념을 참고하고, 구체적인 응답 구조와 임계값은 프로젝트 정책으로 명시**한다.

