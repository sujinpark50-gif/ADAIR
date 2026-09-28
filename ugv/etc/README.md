# UGV PX4 + Gazebo 기동

UGV 1대 = **PX4 인스턴스 1개 + Gazebo 1개**. 공유하지 않는다.

## 왜 개별인가

| | 이유 |
|---|---|
| **PX4** | 인스턴스 하나가 기체 하나다. 에어프레임(`4009`=rover vs `x500`=멀티콥터)·EKF·파라미터가 프로세스 단위라 한 PX4 로 드론과 차량을 같이 몰 수 없다. **선택의 여지가 없다.** |
| **Gazebo** | 합칠 수는 있으나 하지 않는다. 팀 결정이 "차량당 Gazebo 1개, 차량 간 상호작용 없음"이고, 월드가 서로 안 맞으며(`kangwon.sdf` 29km 산악 250Hz vs `rover.sdf` 평지 500Hz), 하나가 죽으면 같이 죽는다. 아끼는 건 400% 중 20%p 남짓이다. |

공유해서 얻는 건 **드론이 차량을 보거나 서로 충돌 판정하는 것** 하나뿐인데, 이 시나리오엔 필요 없다.

## 실행

```bash
cd ugv/etc
./px4-start.sh          # A 거점만 (기본 — 권장)
./px4-start.sh --all    # A + B 두 대
./px4-stop.sh           # 내리기
```

띄우지 않은 자원은 `ugv/fleet.py` 가 자동으로 `SimDriver` 로 돌린다. 코드를 고칠 필요 없다.
시연에서 3D 로 보여줄 자원만 PX4 로 띄우면 된다.

`docker compose up -d` 를 직접 쓰지 않는 이유: 기동 폴링, 안전장치 파라미터,
MAVLink 수신 확인이 빠진다. compose 는 구성 정의이고 기동은 스크립트가 한다.

## 포트

| 자원 | 컨테이너 | 인스턴스 | MAVLink |
|---|---|---|---|
| UAV | `px4` | 0 | 14540 |
| A-ugv1 | `px4-ugv0` | 1 | **14541** |
| B-ugv1 | `px4-ugv1` | 2 | **14542** |

`14540 + 인스턴스번호` 규칙이다. **14540 은 UAV 가 쓰므로 UGV 는 1번부터 시작한다.**
`ugv/config.py` 의 `px4_port` 가 이 표와 짝이다 — 한쪽만 고치면 안 된다.

> 예전 `ugv/config.py` 는 UGV 를 14540/14541 로 잡고 있었다. UGV 가 호스트
> 네이티브로 따로 돌던 때의 값이라 UAV 와 겹쳤고, 통합하면서 14541/14542 로 옮겼다.

## 실측 (4 vCPU, 30초 평균, headless)

| | PX4 | Gazebo | 소계 |
|---|---|---|---|
| UAV (kangwon 지형) | 12% | 45% | 57% |
| UGV × 1 (rover 평지) | 12% | 38% | 50% |

**Gazebo 가 비싸고 PX4 는 싸다.** 최적화는 "Gazebo 를 몇 개 띄우냐" 하나로 정해진다.
대수가 아니라.

## 확인된 함정

**CLI 인스턴스 지정은 `--instance N` 이다**
`-i N` 은 px4 데몬 기동 플래그일 뿐, `px4-commander` / `px4-param` 같은 CLI 에서는
무시되어 `ERROR [px4] PX4 server not running` 이 난다. 소켓(`/tmp/px4-sock-1`)이
멀쩡히 있어도 그렇다.

**`-d` 를 반드시 준다**
없으면 PX4 가 `pxh> ` 프롬프트를 초당 ~830 KB 쏟아내고, 그 전량이
dockerd/containerd-shim 을 거치며 CPU 1 코어를 태운다. UAV 에서 실측한 값은
px4 프로세스 **110% → 13%**. `logging.max-size` 는 디스크만 지킬 뿐 못 막는다.

**`PX4_HOME_ALT` 는 0 이다**
`rover.sdf` 지면이 0m 라 235 를 주면 PX4 가 235m 상공에 있다고 믿고 Takeoff 를 시도한다.

**`SYS_HAS_MAG` / `FD_FAIL_*` 는 건드리지 않는다**
`SYS_HAS_MAG=0` 이면 yaw_align 이 서지 않아 `no heading reference` 로 arm 이 거부된다.

**arm 전에 EKF 수렴까지 약 60초**
`GPS Horizontal Pos Drift too high` 경고가 멈출 때까지 기다린다. 그 전에 arm 하면
위치 추정이 발산한 상태로 주행한다.

**`px4-stop.sh` 에서 `pkill` 을 쓰지 않는다**
구버전 `ugv/tools/px4-stop.sh` 의 `pkill -9 -f "gz sim"` 은 호스트 전체를 훑어
**UAV 의 Gazebo 까지 죽인다.** UGV 가 호스트 네이티브로 돌던 때는 문제가 없었지만
Docker 로 옮긴 지금은 사고가 난다. 컨테이너 이름으로만 지정할 것.

**무해한 경고들**
`gz_frame_id ... not defined in SDF`, `LED: open /dev/led0 failed`,
`Parameter HIWONDER_EMM_EN not found`, `Preflight Fail: system power unavailable`
— 전부 SITL 에서 정상적으로 나온다.

## 미해결

**PX4 rover 가 웨이포인트에 기착하지 못한다** ([ugv/README.md](../README.md) 참조)
이번 Docker 전환과 무관한 별개 문제다. 구축 방식을 바꿔도 해결되지 않는다.

## 구버전

`ugv/tools/px4-start.sh` / `px4-stop.sh` 는 호스트 네이티브 PX4(`~/PX4-Autopilot`)
전제다. 이 서버에는 그 빌드가 없다. 컨테이너 이미지에 `r1_rover` 모델과
`4009_gz_r1_rover` 에어프레임이 이미 들어 있어 빌드(~30분) 없이 Docker 로 띄운다.
