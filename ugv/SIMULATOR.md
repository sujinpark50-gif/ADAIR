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
tail -f /tmp/ugv-px4/px4-1.log
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
| `UGV_TIME_SCALE` | Mac | 서버 시계 배율 — 위와 같게 |
| `UGV_SECONDS_PER_ENV_STEP` | Mac | 환경 1스텝 = 몇 초 (잠정 60, INT-05 미정) |
| `max_speed_mps` | `ugv/config.py` | 차량 최고속도 (UGV 2.0, 소방차 2.5 m/s) |

환경 스텝은 `POST /clock/env {"sim_step": n}` 로 알려 주면 시계가 `n × SECONDS_PER_ENV_STEP` 에 맞춰지고,
`last_drift_s`(내 시계 − 환경 시계)로 어긋남을 볼 수 있다. 아직 이 호출을 넣는 쪽(총괄/환경 루프)이 없다.

## 아직 검증 안 된 것 (PX4 가 필요한 부분)

1. `px4-mavlink --instance N start ... -t <Mac IP>` 로 원격 링크 추가 — 안 되면 relay 로 대체
2. `PX4_GZ_STANDALONE=1` + 우리가 띄운 kangwon 월드에 3대 스폰
3. `rover_ackermann` 모델/에어프레임이 이 PX4 버전에 있는지 (없으면 lawnmower → r1_rover 로 자동 대체)
4. 미션 항목별 속도(혼잡 구간 감속)를 rover 가 따르는지
5. 주행 중 미션 교체(재탐색) 시 rover 가 멈칫하지 않고 이어 가는지
6. 지형 경사에서 r1_rover 가 2 m/s 를 내는지, 도로 위에서 뒤집히지 않는지
