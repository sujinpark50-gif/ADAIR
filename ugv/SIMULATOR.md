# UGV 시뮬레이터 실행 — Gazebo 1개 + PX4 3대 (원격 WSL) + UGV 서버 (Mac)

```
 Windows WSL (같은 공유기)                         Mac (총괄·환경과 같은 서버)
 ┌───────────────────────────────┐   MAVLink UDP   ┌──────────────────────────────┐
 │ gz sim  kangwon.sdf (1개)      │  14541~14543    │ ugv/server.py :8100           │
 │ PX4 -i 1  A-ugv1   r1_rover    │ ──────────────▶ │  PX4Driver ×3 (MAVSDK)        │
 │ PX4 -i 2  B-ugv1   r1_rover    │ ◀────────────── │  SimClock · Scenario · Route  │
 │ PX4 -i 3  A-fire1  ackermann   │                 │ ◀── orchestrator UgvClient    │
 └───────────────────────────────┘                 └──────────────────────────────┘
```

- 차량당 PX4 1개, 제어 프로그램(UGV 서버)은 1개. PX4 인스턴스 0/14540 은 UAV 몫이라 비운다.
- 월드는 UAV 와 같은 강원 지형(`uav/gazebo/kangwon.sdf`). Gazebo 파티션은 `ugv` 로 분리.
- 스폰 위치는 `ugv/data/road_network.json` 의 `spawn` (거점 노드 위, 지형 높이 +0.3 m).

## 0. 한 번만

WSL:
```bash
cd ~/Codyssey_M2-1 && git pull            # 이 브랜치
python3 -c "import json; json.load(open('ugv/data/road_network.json'))['spawn']"   # 새 JSON 인지 확인
```

`%UserProfile%\.wslconfig` (Windows) — WSL 을 공유기 네트워크에 직접 붙인다. 바꾼 뒤 `wsl --shutdown`:
```ini
[wsl2]
networkingMode=mirrored
```

Mac: 방화벽이 python 수신을 물으면 허용. Mac IP 확인 `ipconfig getifaddr en0`.

## 1. PX4 오프라인 확인 (Mac, PX4 없이)

```bash
UGV_TIME_SCALE=200 UGV_DRIVER=sim python3 -m uvicorn ugv.server:app --port 8100
curl -s localhost:8100/scenario | python3 -m json.tool
curl -s -X POST localhost:8100/ugv/A-ugv1/execute -H 'content-type: application/json' \
     -d '{"task_id":"T1","decision_id":"D1","target_node":"B"}'
watch -n2 'curl -s localhost:8100/ugv/A-ugv1/task/T1'
```
200배속이면 A→B 약 2분, 도로 환경 시나리오(`ugv/scenarios/inje_girin.csv`)의 통제 때문에 `reroutes` 에 재탐색이 찍힌다.
도로·노드를 직접 막아 보려면 `POST /roads/{road_id}/block`, `POST /roads/nodes/{node_id}/block` (Swagger `/docs`).

## 2. WSL — Gazebo + PX4 3대

```bash
./ugv/tools/px4-stop.sh --params                      # 이전 파라미터·미션 정리 (처음 한 번)
UGV_HOST=<Mac IP> SPEED=5 ./ugv/tools/px4-start.sh    # GUI=1 을 앞에 붙이면 Gazebo 창
tail -f ugv/.state/px4/px4-1.log
```
- `SPEED` = `PX4_SIM_SPEED_FACTOR`. 서버의 `UGV_TIME_SCALE` 과 **같은 값**이어야 한다.
  3대 + 지형이라 WSL 이 실제로 몇 배속까지 버티는지 확인 필요 (Gazebo RTF 가 SPEED 에 못 미치면 시계가 어긋난다).
- 끝에 Mac 에서 칠 서버 명령을 출력한다.
- `MAVLink 링크 추가 실패` 가 나오면: `python3 ugv/tools/mavlink_relay.py --host <Mac IP>` 를 따로 띄운다.

## 3. Mac — UGV 서버

```bash
python3 -m ugv.tools.check_px4 14541                  # 하나씩 텔레메트리 확인 (14542, 14543)
UGV_DRIVER=px4 UGV_TIME_SCALE=5 UGV_PX4_RESOURCES=A-ugv1,B-ugv1,A-fire1 \
  python3 -m uvicorn ugv.server:app --host 0.0.0.0 --port 8100
curl -s localhost:8100/health | python3 -m json.tool  # resources 가 모두 px4, clock.time_scale 5
```
그다음 1번과 같은 execute. `GET /ugv/A-ugv1/route` 로 경로 선형·남은 거리를 본다.

## 시간을 맞추는 규칙

| 값 | 어디 | 의미 |
|---|---|---|
| `SPEED` / `PX4_SIM_SPEED_FACTOR` | WSL | Gazebo·PX4 가 도는 배율 |
| `UGV_TIME_SCALE` | 서버 | sim 드라이버 차량 속도 배율, PX4 시각이 안 들어올 때 서버 시계 배율 — SPEED 와 같게 |
| `UGV_CLOCK_SOURCE` | 서버 | `px4`(기본): PX4 차량이 있으면 서버 시계가 PX4(Gazebo) 시뮬레이션 시간을 따른다. `wall`: 예전처럼 벽시계 × TIME_SCALE |
| `UGV_SECONDS_PER_ENV_STEP` | Mac | 환경 1스텝 = 몇 초 (잠정 60, INT-05 미정) |
| `max_speed_mps` | `ugv/config.py` | 차량 최고속도 (UGV 2.0, 소방차 2.5 m/s) |

Gazebo 배속은 상한이라 CPU 가 밀리면 흔들린다 (실측 30초 평균 3.1~4.0, 목표 4). 벽시계 × 4 로 세면 서버만 6~10% 앞서
시나리오 사건이 일찍 발동했다. 그래서 PX4 차량이 연결돼 있으면 서버 시계의 '흐름'은 PX4 시각을 따른다 — 환경 시계를
바꾸지 않고 읽기만 한다. `GET /clock` 의 `rate_source` 가 `px4:A-ugv1` 이면 PX4 를, `wall` 이면 벽시계를 따르는 중.
멈춤 판정(STALL_TIMEOUT_S)도 시뮬레이션 초다. 주행 기록 분석의 `서버시계/PX4` 가 1.0 이면 맞게 도는 것.

환경 스텝은 `POST /clock/env {"sim_step": n}` 로 알려 주면 시계가 `n × SECONDS_PER_ENV_STEP` 에 맞춰지고,
`last_drift_s`(내 시계 − 환경 시계)로 어긋남을 볼 수 있다. 아직 이 호출을 넣는 쪽(총괄/환경 루프)이 없다.

## 아직 검증 안 된 것 (PX4 가 필요한 부분)

1. `px4-mavlink --instance N start ... -t <Mac IP>` 로 원격 링크 추가 — 안 되면 relay 로 대체
2. `PX4_GZ_STANDALONE=1` + 우리가 띄운 kangwon 월드에 3대 스폰
3. `rover_ackermann` 모델/에어프레임이 이 PX4 버전에 있는지 (없으면 lawnmower → r1_rover 로 자동 대체)
4. 미션 항목별 속도(혼잡 구간 감속)를 rover 가 따르는지
5. 주행 중 미션 교체(재탐색) 시 rover 가 멈칫하지 않고 이어 가는지
6. 지형 경사에서 r1_rover 가 2 m/s 를 내는지, 도로 위에서 뒤집히지 않는지

## 시험할 때 자주 헷갈리는 것

- **task_id 는 매번 새로.** 실행 기록이 `ugv/.state/ugv_tasks.json` 에 남는다 (UGV-04, 서버를 다시 켜도 유지).
  같은 task_id·같은 내용을 다시 보내면 새로 출발하지 않고 예전 결과를 `"duplicate": true` 로 돌려준다.
  깨끗하게 시작하려면 서버를 끄고 `rm ugv/.state/ugv_tasks.json`, 또는 `UGV_STATE_DIR=/tmp/ugv-state` 로 띄운다.
- **px4-start.sh 를 두 번 실행해도** 이미 떠 있는 인스턴스는 건너뛴다. 다시 띄우려면 `px4-stop.sh` 먼저.
- **차가 지면 위에 떠 보이는 것**: Gazebo 지형의 화면 표시(ogre2)와 물리 충돌면(dartsim)이 같은 heightmap 을
  다르게 보간해서 생기는 표시 차이다. 차의 z 가 스폰 높이 근처에서 유지되고 주행하면 물리상으로는 땅 위에 있는 것이다.
  확인: `GZ_PARTITION=ugv GZ_IP=127.0.0.1 gz topic -e -t /stats -n 1` 의 real_time_factor, 서버 `/ugv/{id}/state` 위치 변화.

## UGV 전용 월드 — 강원 지형 + 도로 면 (`ugv/gazebo/kangwon_ugv.sdf`)

UAV 월드의 지형은 90 m DEM 을 56 m heightmap 으로 보간한 것이라 도로를 모른다. 원본 DEM 에서는 같은 칸(경사 0)인 곳에
보간 때문에 62 m 에 8 m 오르는 가짜 오르막이 생겨 r1_rover 가 같은 지점에서 두 번 멈췄다 (2026-10-02).
그래서 도로 데이터로 주행 면을 만든다. `px4-start.sh` 는 `road_network.json` 의 `world` 를 읽어 이 월드를 띄운다.

| 항목 | 값 |
|---|---|
| 도로 면 | 모든 도로를 폭 10 m 띠로, 교차로는 반지름 7 m 패드. 시각·충돌 공용 `models/kangwon_ugv/roads.obj` |
| 도로 높이 | 원본 DEM → 도로 따라 150 m 이동평균 → 교차로 높이 일치 → 경사 상한 8% (최대 8.4%) |
| 지형 | 도로 면보다 0.6 m 이상 높던 곳을 깎은 heightmap 사본 (터널 구간은 100 m 넘게 깎인 곳도 있다) |
| 스폰 | 도로 면 위 + 0.4 m. 같은 거점 두 번째 차량은 도로를 따라 8 m 앞 |

다시 만들기 (도로망을 다시 만들었으면 반드시 뒤이어 실행 — `build_road_network` 가 spawn·world 를 지형 기준으로 되돌린다):
```bash
python3 -m ugv.tools.build_road_network     # 도로망이 바뀐 경우만
python3 -m ugv.tools.build_road_world       # numpy scipy pillow rasterio pyproj 필요
```
UAV 월드로 돌아가려면 `road_network.json` 의 `world` 를 지우거나 `build_road_network` 만 다시 실행한다.

### v2 월드 (`kangwon_ugv2`, 기본) — 갓길 + 교차로 턱 줄임 + 산불 표시

v1 과 같은 도로 높이·폭·스폰에 갓길·턱 줄임을 더했다. 2026-10-03 WSL 주행 비교 뒤 **기본 월드**다
(`road_network.json` 의 world). v1 은 `WORLD=kangwon_ugv`. 산불(UAV 월드와 같은 자리·배치의 파티클 불꽃 8·연기 3)과
도로 차단 벽 위치(`models/kangwon_ugv2/road_marks.json`)도 이 빌드가 만든다.

| 항목 | v1 | v2 |
|---|---|---|
| 갓길 | 없음 — 도로 끝에서 0.6 m 이상 아래 지형으로 떨어진다 | 양옆 4 m, 바깥으로 0.25 m 내려가는 비탈 |
| 띠 폭 방향 꼭짓점 | 양 끝 2개 | 가운데 1줄 추가 (교차로에서 높이장과 덜 어긋남) |
| 길이 방향 꼭짓점 | 5 m | 10 m |
| 주 경로(A↔B) 10 m 안 턱 0.1 m 이상 | 16 칸 (최대 0.15 m) | 3 칸 (최대 0.14 m) |
| 삼각형 / 파일 | 82k / 4.5 MB | 159k / 7.3 MB (갓길이 절반) |

교차로 넓히기(패드 12 m, 끝 폭 16 m 테이퍼)는 해 봤지만 버렸다: 넓힌 면이 교차로 높이장과 어긋나
주 경로 턱이 54~107 칸(최대 0.37 m), 갓길이 도로 위로 1 m 솟는 곳까지 생겼다. 코너에서 떨어지는 문제는
넓히기 대신 갓길로 받는다. 다시 해 보려면 `UGV_V2_PAD_R=12 UGV_V2_TAPER=1 python3 -m ugv.tools.build_road_world --v2`.

```bash
python3 -m ugv.tools.build_road_world --v2      # 다시 만들기 (road_network.json 은 건드리지 않는다)
```

## 주행 정밀 기록 — 시간·중심 치우침·속도 (`ugv/drive_log.py`)

task 마다 `ugv/.state/drive_logs/<task_id>.csv` 에 0.5 초 간격으로 남는다 (`UGV_DRIVE_LOG=0` 이면 끔).
열: 벽시계 / 서버 시뮬 시각 / PX4 시각(imu), 위치, 경로선 기준 횡오차(+ 왼쪽, − 오른쪽), 실제·명령 속도, 방위, 도로.
task 조회(`GET /ugv/{id}/task/{task_id}`)의 `timing` 에 시작·끝 시각, ETA, 실제 배속, 실제/ETA 요약이 붙는다.

```bash
python3 -m ugv.tools.analyze_drive_log --roads ugv/.state/drive_logs/LOG1.csv
```
읽는 법: 횡오차 중앙값이 한쪽(+1.5 m 이상 등)으로 쏠리면 '중심 치우침', 평균은 0 근처인데 |p95| 만 크면 '코너 자르기'.
`실제배속` 이 SPEED 보다 낮으면 CPU 한계. `서버시계/PX4` 가 1 에서 멀면 서버 ETA 시계가 Gazebo 와 어긋난 것.
중간에 `/stop` 해도 '달린구간 ETA' 로 ETA 대비 실제를 본다.

### 측정 순서 (각 10분 정도, 매번 새 task_id)

| # | WSL | 보는 것 |
|---|---|---|
| 1 | `SPEED=2 UGV_HOST=<Mac IP> ./ugv/tools/px4-start.sh A-ugv1` | 기준: v1 월드, 4 ms |
| 2 | `WORLD=kangwon_ugv2 SPEED=2 ...` | 갓길·턱 — 치우침·멈춤 비교 |
| 3 | `WORLD=kangwon_ugv2 STEP_MS=8 SPEED=4 ...` | 배속 — `실제배속` 이 1·2 보다 오르는지, PX4 경고(accel/gyro timeout) |

사이마다 `./ugv/tools/px4-stop.sh`. Mac 서버의 `UGV_TIME_SCALE` 은 그때의 SPEED 와 같게.
GUI 는 배속을 깎으므로 측정할 때는 끈다 (`GUI=1` 은 눈으로 볼 때만).

## 장비·작업 — 진압(소방차), 짐(UGV), 경광등 (`ugv/equipment.py`)

task 는 지금 계약 그대로 **도착하면 COMPLETED** 다. 그 뒤 작업 동안 차는 `state: "WORKING"` 이라 READY 가 아니고,
총괄은 READY 를 확인한 뒤에만 반납하므로(`orchestrator/engine.py _maybe_release`) 점유가 유지된다. 총괄 코드는 그대로.

| 무엇 | 언제 | 끝 | 보이는 곳 |
|---|---|---|---|
| 진압 SUPPRESSING (소방차) | 도착하면 자동 (`UGV_AUTO_SUPPRESS=1`, 거점 노드 제외) | 물이 바닥(EMPTY) · `POST /ugv/{id}/suppress {"action":"stop"}`(STOPPED) · `/stop` | task `work`, 상태 `equipment.water_l` |
| 짐 싣기 LOADING (UGV) | execute 에 `cargo` — `via_node` 가 있으면 거기 가서, 없으면 출발 전 | `UGV_LOAD_S` (60 시뮬레이션 초) | task `progress.phase=LOADING`, `stage` |
| 짐 내리기 UNLOADING | 짐을 싣고 목적지 도착 | `UGV_UNLOAD_S` (60초) | task `work`, `cargo.unloaded_sim_s` |
| 물 채우기 | 거점(home_node) 도착 | 즉시 | 보고 `UGV_REFILLED` |
| 경광등 | 소방차 출동·진압 중 | 작업 끝 · `/stop` | 상태 `equipment.siren_on` |

- 물탱크 3,000 L, 30 L/s(분당 1,800 L) → 100초 진압 (`ugv/config.py EQUIPMENT`, 잠정값). 진압이 화재를 끄는 효과는
  환경 담당이다 — 총괄도 `SUPPRESSION_CONTRACT_READY = False`. UGV 는 상태만 보고한다.
- 적재 한도(UGV 100 kg)를 넘거나 소방차에 짐을 실으면 evaluate·execute 가 `REQUIRED_CAPABILITY_UNAVAILABLE` 로 거절.
- ETA 에는 싣기 시간이 들어가고, 내리기·진압은 도착 뒤라 들어가지 않는다.
- 수동 진압: `POST /ugv/A-fire1/suppress {"action":"start"}` (READY, task 없음, 물 있음일 때만).

```bash
# 짐 + 경유지
curl -XPOST localhost:8100/ugv/A-ugv1/execute -H 'content-type: application/json' \
  -d '{"task_id":"K1","decision_id":"d1","target_node":"495068","cargo":{"name":"구호물자","kg":40},"via_node":"495070"}'
# 소방차 출동 → 도착하면 자동 진압
curl -XPOST localhost:8100/ugv/A-fire1/execute -H 'content-type: application/json' \
  -d '{"task_id":"F1","decision_id":"d1","target_node":"622559"}'
curl -s localhost:8100/ugv/A-fire1/state | python3 -m json.tool      # activity, equipment.water_l
curl -XPOST localhost:8100/ugv/A-fire1/suppress -H 'content-type: application/json' -d '{"action":"stop"}'
```

## Gazebo 표시 — 산불·경광등·물줄기·도로 차단 벽 (`ugv/gz_fx.py`, `UGV_GZ_FX=1`)

서버가 **Gazebo 와 같은 기계(WSL)** 에서 돌 때만 켠다. gz CLI 로 보내고 결과를 기다리지 않는다 (실패해도 주행 무관).
파티클은 Gazebo 화면(GUI)에서만 보이고 물리 계산에 들지 않는다.

| 표시 | 방식 | 준비 |
|---|---|---|
| 산불 | 월드에 고정 파티클 (UAV 와 같은 자리) | v2 월드 |
| 경광등·물줄기 | 소방차 모델의 particle emitter 를 `/ugv/fx/<자원>/siren·water` 로 켜고 끔 | `px4-start.sh` 가 소방차 모델을 만든다 — 기반 rover(`r1_rover`) model.sdf 복사본 + 경광등·방수포 (`ugv/tools/make_fire_truck.py`). `<include merge>` 로 감싸면 바퀴 토픽이 어긋나 한쪽만 돌았다 |
| 도로 차단 벽 | 막히면 빨간 벽 모델 생성, 풀리면 삭제 (충돌 없음 — 차는 재탐색으로 피한다) | `road_marks.json` |
| 짐 | 표시 없음 (상태만) | |

`px4-start.sh` 는 Gazebo 서버 시스템 목록에 ParticleEmitter 를 더한다 (PX4 server.config 사본 → `$LOG_DIR/server.config`).
월드에 `<plugin>` 을 직접 적으면 기본 목록이 빠져 물리·센서가 없어지므로 이 방식을 쓴다 (UAV 와 같음).

```bash
# WSL — 소방차까지 2대, Gazebo 창
GUI=1 SPEED=4 ./ugv/tools/px4-start.sh A-ugv1
GUI=1 SPEED=4 ./ugv/tools/px4-start.sh A-fire1
# WSL — 서버도 여기서 (Gazebo 표시는 같은 기계에서만)
UGV_GZ_FX=1 UGV_TIME_SCALE=4 UGV_DRIVER=px4 UGV_PX4_RESOURCES=A-ugv1,A-fire1 \
  python3 -m uvicorn ugv.server:app --port 8100
```
`WORLD=` 로 다른 월드를 띄웠으면 서버에도 `UGV_GZ_WORLD=<같은 이름>`.

아직 WSL 에서 확인 안 된 것 (Gazebo 가 여기 없어 실행 검증 못 함):
1. 소방차 모델 — 바퀴 토픽이 `/model/fire_truck_3/...` 만 있는지 (`gz topic -l | grep fire_truck`) (`ugv/.state/px4/px4-3.log`).
   안 되면 `FIRE_TRUCK=0` 으로 기반 모델만.
2. 파티클 켜고 끄기 — `gz topic -t /ugv/fx/A-fire1/water -m gz.msgs.ParticleEmitter -p 'emitting: {data: true}'`
3. 벽 생성·삭제 — 시나리오 시각(5·10·20분)에 인제 쪽 도로에 빨간 벽
4. 2대일 때 배속 (1대 3.8 → ?)
