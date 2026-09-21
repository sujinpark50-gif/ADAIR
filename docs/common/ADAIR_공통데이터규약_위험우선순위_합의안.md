# ADAIR 공통 데이터 규약 및 위험 우선순위 합의안

> 팀 공통 공유용 · 현재 합의사항과 추가 합의 예정 항목

## 목적

각 파트가 동일한 의미와 형식을 사용하도록 하여, 실제 모듈 연결 시 변환 코드와 해석 차이를 줄인다.

---

## 1. 시간 기준 — 확정

| 필드 | 의미 | 형식/단위 | 사용처 |
|---|---|---|---|
| `simulation_time_s` | 시뮬레이션 시작 후 경과시간 | `float`, second | ETA, deadline, timeout, 환경변화, 재계획 등 **시뮬레이션 내부 판단** |
| `timestamp` | 실제 메시지·이벤트 생성 시각 | ISO 8601 | 로그, 디버깅, 모듈 간 추적, 외부연동 |

### 원칙

- 시뮬레이션 내부 의사결정은 `simulation_time_s`를 기준으로 한다.
- `timestamp`는 실행 기록, 디버깅, 외부 시스템 연동 및 로그 추적에 사용한다.

예시:

```json
{
  "simulation_time_s": 23.4,
  "timestamp": "2026-09-21T16:32:41+09:00"
}
```

---

## 2. `resource_id` — 확정

자원 ID는 **거점 + 자원종류 + 번호**로 통일한다.

형식:

```text
<station>-<resource><number>
```

예시:

```text
A-uav1
A-uav2
A-ugv1
A-ugv2
A-fire1

B-uav1
B-uav2
B-ugv1
B-ugv2
B-fire1
```

### 규칙

- 거점: `A`, `B`
- 자원종류: `uav`, `ugv`, `fire`
- 번호: `1`부터 시작
- 하이픈 위치와 대소문자를 임의로 변경하지 않는다.
- 필요 시 자원 종류는 별도 `resource_type` 필드로도 기록할 수 있다.

예시:

```json
{
  "resource_id": "A-uav1",
  "resource_type": "UAV"
}
```

---

## 3. `risk_score` — 의미 확정 / 범위·산식은 추후 합의

### 의미

`risk_score`는 특정 `cell` 또는 지역의 **현재 환경 위험도**를 나타낸다.

- 생성 주체: **Environment**
- 사용 주체: **Orchestrator, Dashboard**
- 목적: 여러 위험지역 또는 Task의 상대적 위험 수준을 비교하기 위한 입력값
- 주의: `risk_score` 자체가 최종 대응 우선순위를 의미하지는 않는다.

```text
Environment
risk_score
= "어디가 얼마나 위험한가?"

        ↓

Orchestrator
task_priority
= "그래서 어느 Task를 먼저 대응할 것인가?"
```

### 위험도 구성 후보

- `human_risk` — 주민·민가 등 인명위험
- `spread_risk` — 급속·대규모 확산 가능성
- `critical_infrastructure_risk` — 국가기간시설 등 중요시설 위험
- `secondary_hazard_risk` — 위험물·폭발 등 2차 재난 가능성
- 필요 시 `property_risk` 등 추가

### 현재 미확정 사항

다음 항목은 **현재 확정하지 않고, 환경 담당 코드와 실제 사용처를 확인한 뒤 팀 합의로 결정한다.**

- `risk_score` 값 범위  
  예: `0.0 ~ 1.0`, `0 ~ 100` 등
- 세부 계산식
- 각 위험요소의 가중치
- 상위 정책조건과 점수 계산의 결합 방식

> 즉, `risk_score = 0~1`은 아직 확정하지 않는다.  
> 범위와 산식은 **추후 팀 합의 후 Integration Contract에 반영**한다.

---

## 4. 대응 우선순위 정책 — 소방청 SOP 참고

소방청 「재난현장 표준작전절차」의 지휘통제 및 산림화재 대응 원칙을 상위 정책 근거로 참고한다.

### 일반 지휘 우선순위

`SOP 106 전략 설정, 대응활동계획 수립 및 수정`

```text
대원안전
→ 인명구조
→ 재산보호
→ 재난 안정화
→ 환경보호
```

### 산림화재 진화 우선순위

`SOP 224 산림화재 대응절차`

```text
1. 인명의 보호
2. 국가기간산업시설·군사시설·국가유산 보호
3. 가옥 등 재산 보호
4. 중요 산림자원 보호
5. 그 밖의 산림지역 산림화재 확산 방지
```

### ADAIR 적용 원칙

```text
[상위 정책]
대원·자원 안전
        ↓
인명 보호
        ↓
중요시설·재산 보호
        ↓
화재 안정화·확산 방지
        ↓
환경·산림 보호
```

- 인명위험은 단순 가중치 하나로 다른 위험요소와 동일하게 처리하지 않고, **상위 정책조건으로 우선 검토**한다.
- 소방청 SOP는 **운영 우선순위의 근거**로 사용한다.
- 숫자형 `risk_score` 산식 자체는 소방청 공식 기준이 아니라 **ADAIR 프로젝트에서 별도 정의**한다.

---

## 5. 책임 경계

### Environment

- 화재 및 환경상태 계산
- 지역·cell별 `risk_score` 산출
- 관측 결과 반영 및 위험도 갱신

### Orchestrator

- SOP 기반 상위 정책 우선순위 적용
- 여러 Task 간 실제 대응 우선순위 결정
- ETA, 가용자원, capability, 접근성 등 함께 고려
- 자원 선택, 재평가, 재할당

### Local Agent

- 배터리
- 풍속
- 통신
- 센서 상태
- 접근 가능성
- 현재 임무상태

등을 기준으로 **해당 자원의 실제 수행 가능성**을 판단한다.

### 정리

```text
risk_score
= 환경이 말하는 "위험의 정도"

task_priority
= Orchestrator가 결정하는 "실제 대응 순서"
```

---

## 6. 현재 합의 상태

| 항목 | 상태 |
|---|---|
| `simulation_time_s` | ✅ 경과초 (`float`, second) |
| `timestamp` | ✅ ISO 8601 |
| `resource_id` | ✅ `<station>-<resource><number>` 형식 |
| `risk_score` 의미 | ✅ 지역·cell의 환경 위험도 |
| `risk_score` 생성 주체 | ✅ Environment |
| 최종 대응 우선순위 결정 | ✅ Orchestrator |
| `risk_score` 값 범위 | ⏳ 추후 합의 |
| `risk_score` 산식·가중치 | ⏳ 환경 담당과 추후 합의 |

---

## 참고

- 소방청 「재난현장 표준작전절차」
  - SOP 106 전략 설정, 대응활동계획 수립 및 수정
  - SOP 224 산림화재 대응절차
- 기존 ADAIR Integration Contract 및 파트별 구현 문서
