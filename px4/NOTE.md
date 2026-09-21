# UAV 담당(김동현) 구축·개발 노트

작성일: 2026-09-15
대상: 산불 관측 폐루프 시뮬레이터 — UAV 파트

---

## 0. 내 역할과 산출물

`info/1.md` 기준 내 담당은 **UAV Local Agent** 다.

| 구분 | 내용 |
|---|---|
| 받는 것 | 총괄(Orchestrator)의 Task / 실행 명령 |
| 주는 것 | UAV 상태, Local 응답(ACCEPT/REJECT/COUNTER), 관측·실행 결과 |
| 먼저 연결할 것 | **ACCEPT**, **HIGH_WIND REJECT**, **실행 결과 반환** |

핵심: 나는 "드론을 나는 사람"이 아니라 **"UAV의 상태와 판단을 남에게 제공하는 사람"** 이다.
따라서 설계의 중심은 PX4가 아니라 **인터페이스 계약**이다.

### 판단 권한의 경계 (중요)

```
총괄   : 무슨 일을 누구에게 맡길지
나(Local): 내 자원이 지금 수행할 수 있는지     ← 여기까지만
Safety : 그 명령을 실행해도 되는지
```

**내가 ACCEPT 했다고 바로 비행하면 안 된다.** ACCEPT는 Safety로 넘어가고,
Safety가 ALLOW 한 뒤에야 실행 명령이 내려온다. 즉 **판단 API와 실행 API는 분리**한다.

단, 즉각적인 비행 안정화·비상동작(failsafe)은 PX4가 자체 처리하며 총괄 응답을 기다리지 않는다.

---

## 1. 전체 구성

```
[총괄/Safety/통합]  ──HTTP/JSON──>  [UAV Agent : 8000]   ← 내가 만드는 것
                                        │ MAVSDK (UDP 14540)
                                        ▼
                                   [PX4 SITL]
                                        ↕
                                   [Gazebo headless]   ← 물리 계산
```

- **Gazebo**: 물리 계산 담당. 위치·배터리·자세 같은 "사실"을 만든다. 화면은 부산물이라 headless로 끈다.
- **PX4**: 비행제어. Gazebo가 준 센서값으로 기체를 제어한다.
- **MAVSDK**: PX4의 값을 Python으로 가져온다.
- **UAV Agent**: 그 값으로 **판단**하고 REST로 제공한다. ← 내 산출물
- **QGroundControl**: 사람이 눈으로 보는 용도. **API 파이프라인에 불필요.** 개발 초기 확인용으로만.


##  내 설명
말이 너무 AI스러워서 이해하기 쉽지 않은데, 
Gazebo와 PX4는 서로 양방향으로 계속 데이터를 엄청나게 주고 받고 있음. 
Gaezbo -> PX4는 물리 계산으로 센서값(위치, 자세, 배터리)을 만들어 보냄. 쉽게 말하면, 차가 왼쪽으로 기울었다, 속도는 80km/h다, 연료가 절반 남았다. 이런 정보들을 화면에서 볼 수 있는데 이러한 센서값들을 Gazebo가 전부 계산해서 지금 드론이 어디있고, 얼마나 기울었고, 배터리가 얼마 남았다를 만들어서 px4에게 알려주는 것임.
PX4 -> Gazebo는 그 센서값으로 제어 판단해서 모터 출력을 보냄. 예를 들어, 왼쪽으로 지금 기울었네? 오른쪽으로 좀 꺾어야곘다. 속도가 너무 빠르네? 좀 줄여야겠다. 이러한 명령들을 Gazebo(모터)에 전달해주는 것임.
QGC는 명령어 역할도 일부 수행하긴 하는데, PX4처럼 디테일하게는 못하고 모니터링 용도가 더 큼. 자세, 속도, 배터리가 계기판으로 보이고 경로가 보임.

아 그리고, MAVLink 씀. 이게 뭐냐? 드론과 통신하기 위한 프로토콜임. 드론은 무선으로 통신해야되기 때문에 대역폭이 좁고 배터리도 아껴야되기 때문에 최적화한 프로토콜임. (UDP 14540)

### 개발/운영 환경 구분 (`info/1.md` 7절)

| 환경 | 구성 |
|---|---|
| 개발·발표 시연 | PX4 + Gazebo **GUI** + 필요 시 QGC (WSL2 권장) |
| 외부 서비스 | PX4 + Gazebo **headless** + 웹 관제 (EC2) |

---

## 2. EC2 설치 (headless 백엔드)

### 2-1. 현재 상태 (2026-09-15 확인)

| 항목 | 상태 |
|---|---|
| 공인 IP | `3.37.210.229`  |
| 보안그룹 | `launch-wizard-18` |
| Docker | 29.8.0, 설치됨 |
| PX4 이미지 | `px4io/px4-sitl-gazebo:latest` (3.01GB) 캐시됨 |
| mavlink-router | systemd 등록, TCP 5760, 선택사항(QGC 보는 용도) |
| Python | 3.14.4 |

### 2-2. Docker 설치

```bash
sudo apt-get update
sudo apt-get install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
```

**재로그인 필수** (그룹 반영).

### 2-3. PX4 SITL + Gazebo headless 실행

```bash
docker run -d --name px4 \
  --network host \
  --restart unless-stopped \
  -e HEADLESS=1 \
  px4io/px4-sitl-gazebo:latest
```

> ⚠️ **`--network host` 는 필수. `-p 14550:14550/udp` 는 동작하지 않는다.**
> 실측 확인 결과 패킷 0개:
> 1. PX4는 기본적으로 컨테이너 내부 `127.0.0.1` 로만 MAVLink 를 보낸다.
> 2. `docker-proxy` 가 `0.0.0.0:14550` 을 점유해 mavlink-router 가 바인딩조차 못 한다.

확인:

```bash
docker ps --format '{{.Names}} | {{.Status}}'
```

### 2-4. mavlink-router (QGC 원격 접속용 — 선택)

API만 쓸 거면 **없어도 된다.** QGC로 눈으로 확인하고 싶을 때만 설치.

```bash
sudo curl -fsSL -o /usr/local/bin/mavlink-routerd \
  https://github.com/mavlink-router/mavlink-router/releases/download/v4/mavlink-routerd-glibc-x86_64
sudo chmod +x /usr/local/bin/mavlink-routerd
```

⚠️ apt·snap·공식 Docker 이미지 **없음**. 소스 빌드는 GCC 15 + `gnu++14` 조합이라 깨질 위험이 있으니
   사전 빌드 바이너리를 쓴다 (Ubuntu 26.04 동작 확인됨).

systemd 등록:

```bash
sudo tee /etc/systemd/system/mavlink-router.service > /dev/null << 'EOF'
[Unit]
Description=MAVLink Router
After=network.target docker.service

[Service]
ExecStart=/usr/local/bin/mavlink-routerd -t 5760 0.0.0.0:14550
Restart=always
RestartSec=3
User=ubuntu

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now mavlink-router
```

⚠️ 문법 주의: TCP 서버는 `[General]` 의 `TcpServerPort` **전역 옵션**이다.
   `[TcpServerEndpoint]` 섹션은 **존재하지 않는다.** `[TcpEndpoint]` 는 반대로 나가는 연결이다.

### 2-5. 포트 정리

| 포트 | 프로토콜 | 용도 | 외부 개방 |
|---|---|---|---|
| **14540** | UDP | **MAVSDK ← UAV Agent** | ❌ 내부 |
| 14550 | UDP | PX4 → mavlink-router | ❌ 내부 |
| 5760 | TCP | QGC (선택) | 필요 시 |
| **8000** | TCP | **UAV Agent REST API** | ✅ 팀에 개방 |

MAVSDK 연결 타임아웃의 대부분은 14550 을 잘못 쓴 경우다. **Agent 는 14540.**

---

## 3. Python 환경

```bash
python3 -m venv ~/uav-venv
source ~/uav-venv/bin/activate
pip install --upgrade pip
pip install mavsdk fastapi uvicorn pydantic
```

**Python 3.14 에서 정상 동작 확인됨** (mavsdk 3.17.2, fastapi 0.141.1).
이전에 우려했던 3.12 폴백은 불필요하다.

---

## 4. 시나리오 좌표

산불 지역: **원통119 ↔ 기린119 사이** (강원 인제군)

| 지점 | 좌표 (근사) | 고도 |
|---|---|---|
| 원통119안전센터 | 38.1205, 128.2018 | ~235 m |
| 기린119안전센터 | 37.9645, 128.3062 | ~285 m |
| 중간 지점 | 38.0425, 128.2541 | **~804 m** |

- 직선거리 **약 19~21 km**, 방위 약 152°(SSE)

> ⚠️ **좌표 정확도**: 도로명 주소(북면 원통로 276 / 기린면 대내로 34-14)는 확인됐으나
> 건물 단위 지오코딩은 실패했다. 위 값은 **거리/마을 수준 근사(±0.5~1 km)** 다.
> 정밀값이 필요하면 Kakao/Naver Maps 또는 juso.go.kr API 로 재지오코딩할 것.

> 🔺 **지형 주의 — 판단 로직에 직접 영향**
> 두 지점의 중간은 평지가 아니라 **태백산맥 능선(약 800~1000 m)** 이다.
> 원통(235 m)에서 출발해 중간 지점을 관측하려면 **600~780 m 고도 상승**이 필요하다.
> **기본 100~120 m AGL 순항으로는 능선에 충돌한다.**
> 또한 두 지점 간 **가시선(LOS)이 지형에 막힌다** — 통신 상태 시뮬레이션 시 고려.
> (SRTM 30 m DEM 기준이므로 지형 성격 파악용이며, 장애물 회피용 정밀도는 아니다.)

---

## 5. API 설계

> ⚠️ 아래 스키마는 **잠정안**이다. 실제 필드명·ID·좌표계·단위·시간 형식은
> `docs/통합/01_integration_contract.md` (통합계약)를 따른다. 현재 로컬에 해당 문서가 없어
> 계약 확인 후 맞춰야 한다. **팀 합의 전에 이 스키마로 구현을 확정하지 말 것.**

### 5-1. 상태 조회 — `GET /uav/{uav_id}/state`

```json
{
  "uav_id": "uav-01",
  "timestamp": "2026-09-15T10:23:45Z",
  "position": {"lat": 38.1205, "lon": 128.2018, "alt_m_amsl": 235.0, "alt_m_agl": 0.0},
  "battery": {"percent": 78.5, "voltage": 15.8},
  "velocity_ms": 0.0,
  "flight_mode": "HOLD",
  "armed": false,
  "health": {"gps_ok": true, "sensors_ok": true, "failsafe": false},
  "wind_ms": 3.1,
  "link_quality": "OK",
  "current_task_id": null
}
```

### 5-2. 수행 가능성 판단 — `POST /uav/{uav_id}/evaluate`

**핵심 산출물.** 실행하지 않고 판단만 반환한다.

요청:
```json
{
  "task_id": "task-101",
  "decision_id": "dec-001",
  "target": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 804.0},
  "observation_type": "THERMAL",
  "deadline": "2026-09-15T10:40:00Z"
}
```

응답 (ACCEPT):
```json
{
  "task_id": "task-101",
  "decision_id": "dec-001",
  "uav_id": "uav-01",
  "verdict": "ACCEPT",
  "eta_sec": 420,
  "constraints": {
    "distance_km": 10.2,
    "battery_now_pct": 78.5,
    "battery_after_pct": 41.3,
    "return_margin_pct": 21.3,
    "wind_ms": 3.1,
    "required_climb_m": 569
  }
}
```

응답 (REJECT — IT-02 필수 케이스):
```json
{
  "task_id": "task-101",
  "decision_id": "dec-001",
  "uav_id": "uav-01",
  "verdict": "REJECT",
  "reason": "HIGH_WIND",
  "detail": "wind 12.4 m/s exceeds limit 10.0 m/s",
  "constraints": {"wind_ms": 12.4, "wind_limit_ms": 10.0}
}
```

응답 (COUNTER):
```json
{
  "verdict": "COUNTER",
  "reason": "DEADLINE_TIGHT",
  "counter_offer": {"eta_sec": 480, "arrival": "2026-09-15T10:42:00Z"},
  "constraints": {"battery_after_pct": 31.2, "return_margin_pct": 12.5}
}
```

**`reason` 과 `constraints` 는 반드시 넣는다.** 총괄이 "왜 거절인지" 모르면
재평가 근거가 없고, 통합·관제의 추적 로그도 성립하지 않는다.

**`decision_id`**: 같은 Task 를 다른 자원으로 재평가할 때 새 `decision_id` 가 발급된다.
Task 단위가 아니라 판단 단위 식별자이므로 응답에 그대로 echo 한다.

### 5-3. 실행 — `POST /uav/{uav_id}/execute`

Safety ALLOW 이후 호출된다. **비동기로 만든다** — 비행은 수 분이 걸리므로
동기 방식이면 호출측이 타임아웃 난다.

즉시 응답:
```json
{"task_id": "task-101", "status": "STARTED", "tracking_url": "/uav/uav-01/task/task-101"}
```

### 5-4. 진행/결과 — `GET /uav/{uav_id}/task/{task_id}`

```json
{
  "task_id": "task-101",
  "status": "IN_PROGRESS",
  "progress": {"phase": "ENROUTE", "eta_sec": 180},
  "observation": null
}
```

완료 시 `status: "COMPLETED"` 와 관측 결과. 실패 시 `"FAILED"` 와 사유.

### 5-5. 관측 결과 — 환경모델로 전달

관측값에는 **위치·시각·센서 종류**를 포함해야 한다.

```json
{
  "observed_at": "2026-09-15T10:35:12Z",
  "position": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 804.0},
  "sensor_type": "THERMAL",
  "values": {"max_temp_c": 312.5, "hotspot_detected": true}
}
```

---

## 6. 판단 로직 (ACCEPT / REJECT / COUNTER)

`info/1.md` "주요 고려사항"(풍속·배터리·통신·센서·failsafe)을 그대로 기준으로 삼는다.

### 6-1. 판정 순서

```
1단계: 하드 제약 — 하나라도 걸리면 즉시 REJECT
   - failsafe 활성            → REJECT / FAILSAFE_ACTIVE
   - GPS/센서 이상            → REJECT / SENSOR_FAULT
   - 통신 두절                → REJECT / LINK_LOST
   - 이미 다른 Task 수행 중    → REJECT / BUSY
   - 풍속 > 한계              → REJECT / HIGH_WIND      ← IT-02 필수
   - 센서 미탑재(관측 유형 불일치) → REJECT / SENSOR_UNAVAILABLE

2단계: 에너지 여유
   battery_needed = (편도ETA × 2 × 소모율) + 관측체류 + 상승분
   margin = battery_now - battery_needed
   - margin < 0               → REJECT / INSUFFICIENT_BATTERY
   - margin < SAFETY_MARGIN   → COUNTER (고도↓ 또는 체류시간↓ 제안)

3단계: 시간
   - ETA ≤ deadline           → ACCEPT
   - ETA > deadline 이나 수행 가능 → COUNTER (가능 시각 제시)
   - 수행 불가                 → REJECT / DEADLINE_INFEASIBLE
```

### 6-2. 설정값 (전부 TBD — 팀 합의 필요)

`info/1.md` 6절에 "임계값은 설정값 또는 TBD로 관리"라고 되어 있으므로 **하드코딩하지 말고 설정 파일로 뺀다.**

```python
# config.py — 값은 잠정. 팀 합의 후 확정
WIND_LIMIT_MS        = 10.0   # 이상이면 REJECT/HIGH_WIND
WIND_COUNTER_MS      = 8.0    # 이상이면 COUNTER 검토
SAFETY_MARGIN_PCT    = 20.0   # 복귀 후 남아야 할 배터리
CRUISE_SPEED_MS      = 15.0   # x500 기준 순항속도
CLIMB_SPEED_MS       = 3.0
BATTERY_DRAIN_PCT_S  = 0.033  # 초당 소모율 (100%/약 50분) — 실측 보정 필요
OBSERVE_DURATION_S   = 60     # 관측 체류시간
MIN_CLEARANCE_M      = 50     # 지형 위 최소 여유고도
```

> ⚠️ `BATTERY_DRAIN_PCT_S` 는 반드시 **실측 보정**해야 한다. Gazebo 의 배터리 모델은
> 기본값이 실제 기체와 다르다. 이륙~착륙 로그로 실제 소모율을 측정해 넣을 것.
>
> **경위**: 초기값 0.05(%/s)는 x500 실제 항속(~30분)보다 과도해, 기본 시나리오
> (원통→중간지점 왕복 약 860s)가 배터리 부족으로 REJECT 로 떨어졌다. 0.033 으로
> 조정했으나 **이 값도 추정치이며 측정값이 아니다.**

#### PX4 SITL 배터리 시뮬레이션 실태 (실측 확인)

- `SIM_BAT_DRAIN = 60.0` (드레인 시상수, 초) → **0보다 크므로 방전이 활성화돼 있다.**
  `rcS:253` 의 `if param greater SIM_BAT_DRAIN 0` 가 `battery_simulator` 를 기동한다.
- **지상 대기(disarmed) 상태에서는 100% 에서 움직이지 않는다.** 30초간 관찰해도
  `100.00% / 16.200V` 로 고정이었다.
- 시동(arm) 후 값이 움직이긴 하나 **단조 감소가 아니다.** 1차 관찰에서
  100 → 95 → 88 로 내려갔다가 93 → 100 으로 **되돌아왔다.** 배터리가 충전될 리 없으므로
  이는 방전 곡선이 아니라 시동 과도구간의 전압 변동으로 보인다.
  (해당 측정은 실연동 서버와 14540 포트를 경합한 상태였으므로 신뢰도가 낮다.)
  **재측정 필요 — 아래 '미해결' 참조.**
- `BAT1_CAPACITY = -1.0` → 용량 기반 추정이 꺼져 있고 **전압 기반 모델**을 쓴다.
  따라서 `remaining_percent` 는 실제 mAh 소모가 아니라 전압 곡선에서 역산한 값이다.

**시사점**: 배터리는 고정값이 아니지만, 비행 중에만 변한다. 소모율 실측 보정은
반드시 **실제 이륙~착륙 로그**로 해야 하며, 지상에서 관찰해서는 얻을 수 없다.

### 기본 시나리오 판정 분포 (원통 → 중간지점, ETA 860s, 소요 58.7%)

| 시작 배터리 | 잔량 | 판정 |
|---|---|---|
| 95% | 36.3% | ACCEPT |
| 78.5% | 19.8% | COUNTER (마진 20% 미달) |
| 60% | 1.3% | COUNTER |
| 40% | -18.7% | REJECT |

Mock 기본 배터리를 95% 로 둔 이유다. 78.5% 로는 마진 기준(20%)에 0.2%p 차이로
걸려 COUNTER 가 되는데, 반올림 수준의 차이로 판정이 갈리는 값을 기본 시연에
쓰는 것은 부적절하다. **임계값을 원하는 답에 맞춰 조정한 것이 아니라
시나리오의 시작 조건을 바꾼 것이다.**

### 6-3. 고도/지형 처리 (이 시나리오 특유)

4절 지형 경고 때문에 단순 직선거리 계산으로는 부족하다.

```
required_alt = target_terrain_alt + MIN_CLEARANCE_M
required_climb = required_alt - current_alt
```

- 상승에도 배터리가 들고 시간도 걸리므로 **ETA·배터리 계산에 상승분을 반영**한다.
- 초기 구현은 "목표 지점 지형고도 + 여유"로 단순화하고,
  경로상 최고점까지 고려하는 것은 후속으로 둔다. **단순화했다는 사실을 명시할 것.**

### 6-4. 풍속 출처

**PX4 에서는 못 받는다 (실측 확인).** MAVSDK 에 `telemetry.wind` 스트림이 존재하지만
PX4 SITL 은 여기에 아무것도 발행하지 않는다. 실제로 구독하면 타임아웃한다.
따라서 선택지는 하나로 좁혀진다.

1. ~~Gazebo/PX4 값 사용~~ → **불가**
2. **환경모델(김은주님)이 제공** → 사실상 유일한 경로. Task 요청에 풍속을 실어 보내거나
   환경모델 API 를 직접 조회하는 방식 중 택일. **협의 필요.**

현재 구현은 `EvaluateRequest.wind_ms` 로 주입받는다. 풍속을 알 수 없으면
`REJECT / WIND_UNKNOWN` 을 반환한다 — 모르는 값을 0 으로 간주해 통과시키지 않는다.

IT-02(강풍 REJECT)를 재현하려면 **풍속을 인위적으로 주입할 수 있어야** 한다.
Mock 단계에서는 요청에 풍속을 실어 보내거나 설정값으로 강제하는 경로를 반드시 만들 것.

---

## 7. 구현 순서


### 1순위: Mock 서버 (팀 차단 해제 — 지금 당장)

PX4 연동 없이 고정값만 반환해도 **다른 팀이 즉시 개발을 시작할 수 있다.**


```
uav-agent/
├── main.py         # FastAPI 엔드포인트
├── models.py       # 요청/응답 스키마 (pydantic)
├── evaluator.py    # 판단 로직  ← PX4 없이도 테스트 가능
├── config.py       # 임계값
├── drone.py        # MAVSDK 실제 구현
└── drone_mock.py   # Mock 구현 (같은 인터페이스)
```

`drone.py` 와 `drone_mock.py` 를 **같은 인터페이스**로 만들고 환경변수로 교체한다.
그러면 판단 로직을 드론 없이 검증할 수 있고, 나중에 실물로 바꿔도 호출측 코드는 안 바뀐다.

### 2순위: IT-01 / IT-02 통과

| 테스트 | 내가 보장할 것 |
|---|---|
| IT-01 정상 | evaluate → **ACCEPT** → (Safety) → execute → 관측 결과 반환 |
| IT-02 강풍 | evaluate → **REJECT / HIGH_WIND** + 사유 기록, **실행 안 함** |

IT-02 에서 중요한 것: **REJECT 시 Safety 를 호출하지 않는다.** 내 응답이 그대로 총괄로 돌아가고,
총괄이 새 `decision_id` 로 다른 UAV 를 평가한다. 내 쪽에서 재시도하거나 대체 자원을 찾지 않는다.

### 3순위: MAVSDK 실연동

상태 조회(위치·배터리) → 이륙/이동/착륙 → 관측 시뮬레이션 순.

### 4순위: 실측 보정

배터리 소모율, 순항속도, ETA 오차를 실제 비행 로그로 보정.

---

## 8. MAVSDK 연결 확인

```python
# ~/uav-agent/check.py
import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")   # 14550 아님

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("연결됨")
            break

    async for pos in drone.telemetry.position():
        print("위치:", pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
        break

    async for bat in drone.telemetry.battery():
        print("배터리:", bat.remaining_percent)
        break

asyncio.run(main())
```

```bash
source ~/uav-venv/bin/activate
python ~/uav-agent/check.py
```

**실측 확인됨 (2026-09-15)** — 위 스크립트로 연결·위치·배터리·health 조회 성공.

```
연결됨
위치: 47.3979711 8.5461636 AMSL: 0.172 AGL: -0.023
배터리: 100.0
health: gps_ok= True home_ok= True
```

> ⚠️ `udp://` 는 deprecated 경고를 내지만 **동작에는 문제 없다.** `udpin://0.0.0.0:14540`
> 을 권장한다. 두 방식 모두 정상 연결됨을 실측 확인했다.

### ⚠️ PX4 가 MAVLink 송신을 멈추는 현상 (2 vCPU 환경)

**실측 경위**: 컨테이너를 50분 가동한 뒤 MAVSDK 연결이 전부 타임아웃했다.
처음에는 `udpin://` 스킴 문제로 의심했으나 **오진이었다.** 확인된 사실:

- mavlink-router 로그가 14:31:20 이후 **39분간 완전히 멈춤** (평소 5초마다 기록)
- PX4 온보드 포트로 정상 HEARTBEAT 를 보내도 **응답 0 패킷**
- `mavsdk_server` 프로세스는 정상 기동 → 클라이언트 문제 아님
- `udp://` `udpin://` **둘 다** 실패 → 스킴 문제 아님
- load average **9.04 / 2 vCPU**, PX4 CPU 95%, 여유 메모리 122 MiB, 스왑 921 MiB 사용

> ⚠️ **위 메모리 수치는 과장되었다(사후 정정).** 당시 조사용으로 돌리던
> `docker logs px4 | tr | sed` 파이프라인 2개가 RAM 의 34% + 16% 를 점유하고 있었다
> (21 MB 로그를 sed 가 통째로 버퍼링). 이를 정리하자 여유 메모리가
> 147 MiB → 2.2 GiB, 스왑이 3.9 GiB → 1.2 GiB 로 회복됐다.
> 다만 **PX4 정지 시각(14:31)이 해당 파이프라인 시작(15:06)보다 앞서므로,
> 이 파이프라인이 정지의 원인은 아니다.** CPU 경합은 실재했으나
> 정지 원인은 확정되지 않았다.

즉 **PX4 lockstep 시뮬레이션이 CPU 고갈로 정지**한 것이다. OOM kill 은 아니다.
`docker restart px4` 로 즉시 복구되며 router 카운트·MAVSDK 연결 모두 정상화된다.

### 실제 CPU 소비 내역 (정리 후 측정)

| 프로세스 | CPU |
|---|---|
| `px4` | 73.8% |
| `gz sim` (Gazebo) | 34.8% |
| `dockerd` | 23.2% |
| `containerd-shim` | 22.2% |
| **합계** | **약 154% / 200%** |

즉 시뮬레이터 자체가 **약 1.1 코어**, Docker 오버헤드가 **약 0.45 코어**다.
2 vCPU 에서는 여유가 거의 없다. 메모리는 PX4 컨테이너가 16 MiB 로 매우 가볍다.

> 참고: `ps` 의 `comm` 에 `ruby` 로 표시되는 프로세스는 **Gazebo(`gz sim`)** 다.
> 스레드 이름이 그렇게 잡힐 뿐 루비와 무관하다.

### CPU 사용을 줄이는 방법

| 방법 | 효과 | 부작용 |
|---|---|---|
| **Gazebo 물리 주기 하향** — `default.sdf` 의 `real_time_update_rate` (현재 **250** Hz) 를 100~125 로 | 가장 직접적. Gazebo CPU 비례 감소 | 물리 정밀도 하락 |
| **`PX4_SIM_SPEED_FACTOR` 로 감속** — 예: `0.5` | 시뮬레이션을 느리게 돌려 CPU 절감 | 비행 시간이 실제보다 느리게 흐름 |
| **SIH 이미지로 교체** — `px4io/px4-sitl` (152 MB) | **Gazebo 를 아예 안 씀** → 약 35% 절감 | 3D 물리·센서 시뮬 없음 |
| **Docker CPU 상한** — `--cpus 1.5` | 다른 프로세스 보호 | PX4 lockstep 이 더 쉽게 밀림 |

현재 컨테이너에는 CPU/메모리 제한이 없다 (`CpuQuota=0`).

**권고**: UAV Agent 개발이 목적이고 3D 물리 정밀도가 중요하지 않다면
`real_time_update_rate` 를 낮추는 것이 가장 부작용이 적다. 관측 로직만 검증할
단계에서는 SIH 이미지도 선택지다.

**대응**
- 현 인스턴스(2 vCPU / 3.7 GB)에서 PX4 + Gazebo + uvicorn 동시 구동은 여유가 없다.
  위 튜닝으로 버티거나 **vCPU 4개 이상 증설을 검토할 것.**
- 조사용으로 `docker logs` 를 파이프에 물릴 때는 반드시 `--tail` 을 붙일 것.
  전체 로그(21 MB)를 sed 로 넘기면 그 자체가 부하의 주범이 된다.
- `Px4Drone` 에 조회 타임아웃(8초)과 재연결 로직을 넣었다. PX4 무응답 시 요청이
  무한 대기하지 않고 오류를 반환한다.
- 장시간 운용 시 PX4 헬스체크 + 자동 재시작을 두는 편이 안전하다.

### 홈 위치를 산불 시나리오로 변경

기본 홈 위치는 **스위스 취리히(47.3979, 8.5461)** 다. 4절 좌표로 바꿔야
거리·ETA·지형 계산이 의미를 갖는다. 컨테이너 실행 시 환경변수로 지정한다.

```bash
docker rm -f px4
docker run -d --name px4 --network host --restart unless-stopped \
  -e HEADLESS=1 \
  -e PX4_HOME_LAT=38.1205 \
  -e PX4_HOME_LON=128.2018 \
  -e PX4_HOME_ALT=235 \
  px4io/px4-sitl-gazebo:latest
```

### QGC 없이 이륙하기

PX4 는 기본적으로 GCS 연결이 없으면 시동을 거부한다 (`Preflight Fail: No connection to the GCS`).
headless 자동화에서는 이 안전장치를 꺼야 한다.

```
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set COM_RCL_EXCEPT 4
```

> ⚠️ 이건 **시뮬레이션 전용 설정**이다. 실기체에 그대로 쓰면 안 된다.

---

## 9. 팀 협업 체크리스트

- [ ] `docs/통합/01_integration_contract.md` 확인 → 5절 스키마를 계약에 맞춤
- [ ] 필드명·좌표계·단위·시간형식 합의 (특히 고도: AMSL vs AGL)
- [ ] 풍속 출처 합의 (환경모델 제공 vs Gazebo 값) — 6-4 참조
- [ ] 임계값 확정 (풍속 한계, 안전마진) — 6-2 참조
- [ ] Mock 서버 배포 + 팀에 URL 공유
- [ ] `reason` 코드 목록 합의 (`HIGH_WIND` 등 — 총괄이 파싱해야 함)
- [ ] **Elastic IP 연결** — API 주소를 팀에 알려주기 전에 필수

---



### 10. 재부팅 복구 절차

```bash
docker rm -f px4 2>/dev/null
docker run -d --name px4 --network host --restart unless-stopped \
  -e HEADLESS=1 px4io/px4-sitl-gazebo:latest
docker ps --format '{{.Names}} | {{.Status}}'
```

> 유령 컨테이너 문제가 계속되면 `--restart` 정책 대신 mavlink-router 처럼
> **systemd unit 으로 관리**하는 편이 확실하다.

---

## 11. 미해결 / 확인 필요

| 항목 | 상태 |
|---|---|
| 통합계약 문서 | **로컬에 없음.** GitHub 관리본 확인 후 5절 스키마 수정 필요 |
| 좌표 정밀도 | 건물 단위 지오코딩 실패. ±0.5~1 km 근사 (4절) |
| 경로상 지형 회피 | 초기엔 목표지점만 고려. 경로 최고점 반영은 후속 |
| 배터리 소모율 | Gazebo 기본값 사용 중. **실측 보정 필요** |
| 배터리 방전 거동 | **미해결.** `SIM_BAT_DRAIN=60` 이라 방전이 켜져 있어야 하나, 시동 후 값이 100→95→88→93→100 으로 **되돌아왔다**(포트 경합 상태의 측정이라 신뢰도 낮음). `battery_simulator status` 는 출력이 없어 모듈 기동 여부도 미확인. **부하 낮은 상태에서 단독 클라이언트로 재측정 필요** |
| 풍속 출처 | 환경모델 vs Gazebo — 미합의 (6-4) |
| Elastic IP | `15.164.176.101` 확보했으나 **미연결**. 과금 중 |
| COUNTER 상세 처리 | `info/1.md` 기준 후속 구현 범위 |
| 다중 UAV | IT-02 는 대체 UAV 가 필요. 현재 단일 인스턴스만 구동 중 |
| 홈 위치 | 기본값이 취리히. `PX4_HOME_*` 로 원통 좌표 지정 필요 (8절) |
