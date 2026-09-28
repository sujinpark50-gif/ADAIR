# UAV 실행 방법

> **서버 안내**
> UAV Agent는 `http://3.37.210.229:8000` 에 배포합니다.
> 다만 상시 운영하면 클라우드 비용이 많이 나와서 **평소에는 꺼둡니다.**
> 테스트는 각자 로컬에서 띄워서 해 주세요. 띄우는 방법은 아래에 순서대로 적어 두었습니다.
> 서버가 꼭 필요할 때만 UAV 담당(김동현)에게 요청해 주세요.

UAV 1대 = PX4 컨테이너 1개 + UAV Agent 프로세스 1개입니다. API 사양은 [API_DEFINE.md](API_DEFINE.md)를 보세요.

로컬 테스트는 PX4 없이 **mock 모드**로 충분합니다. 0번의 pip 설치 후 2번(mock)만 하면 됩니다.

## 0. 최초 1회 설치

```bash
docker pull px4io/px4-sitl-gazebo:latest

python3 -m venv ~/uav-venv4
source ~/uav-venv4/bin/activate
pip install "mavsdk>=4,<5" fastapi uvicorn pydantic
```

> `mavsdk`는 반드시 4.x여야 합니다. 3.x에서는 real 모드가 동작하지 않습니다.

## 1. PX4 + Gazebo 기동 (real 모드일 때만, mock이면 건너뜀)

```bash
cd uav/etc
./px4-start-kangwon.sh          # 강원 지형 월드, headless (서버)
./px4-start-kangwon.sh --gui    # 강원 지형 월드, 화면 표시 (로컬 WSL2)
./px4-start.sh                  # 평지 월드, headless (지형 없이 가볍게)
```

기동까지 30~40초 걸립니다. `docker ps`에 `px4`가 보이면 준비된 것입니다.

## 2. UAV Agent 기동

```bash
source ~/uav-venv4/bin/activate
cd uav/uav-agent

UAV_ID=A-uav1 UAV_MODE=real uvicorn main:app --host 0.0.0.0 --port 8000   # PX4 연동
UAV_ID=A-uav1 UAV_MODE=mock uvicorn main:app --host 0.0.0.0 --port 8000   # PX4 없이 고정값
```

UAV를 여러 대 띄울 때는 `UAV_ID`와 `--port`를 바꿔 프로세스를 따로 띄웁니다. real 모드는 현재 서버(2 vCPU)에서 1기만 안정성을 확인했습니다.

## 3. 동작 확인

```bash
curl localhost:8000/health              # {"status":"ok","mode":"real","uav_id":"A-uav1"}
curl localhost:8000/uav/A-uav1/state    # 위치·배터리 등 상태
```

Swagger UI: `http://<host>:8000/docs`

## 4. 종료

```bash
# Agent: 실행 중인 터미널에서 Ctrl+C
cd uav/etc && ./px4-stop.sh
```

## 지형 월드를 다시 만들 때

DEM이 바뀌었을 때만 실행합니다. GDAL이 필요합니다.

```bash
python3 uav/gazebo/build_world.py
```
