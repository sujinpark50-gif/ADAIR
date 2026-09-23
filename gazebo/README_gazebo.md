# 강원 지형 Gazebo 연동 패키지 (실전 검증판)

환경 모델(PR #3)의 실측 강원 DEM(인제 원통·기린)을 **Gazebo(gz sim, Harmonic)** 3D 지형으로
올리고, PX4 SITL 드론이 그 지형 위에서 **화재 좌표로 실제 비행**하도록 잇는 패키지.
2026-09-21 WSL2 + `px4io/px4-sitl-gazebo:latest` 환경에서 **비행까지 실측 성공**.

## 파일
| 파일 | 역할 |
|---|---|
| `kangwon.sdf` | gz 월드 — **PX4 default.sdf 구조 그대로**(플러그인 0개, ode) + ground_plane 을 강원 heightmap 으로 교체 + datum 만 강원 |
| `kangwon_heightmap.png` | 513×513 16-bit heightmap (DEM 172~1449 m) |
| `gz_bridge.py` | WGS84 ↔ EPSG:5186 ↔ gz 로컬 ENU + **CA 격자(col,row)→위경도/ENU/지형고도** |
| `demo_fly_to_fire.py` | 화재 격자셀 → 위경도+지형고도 → MAVSDK goto (실제 비행 데모) |
| `kangwon_terrain_preview.png` | 음영기복 미리보기(참고) |

## ★ 핵심 교훈 (GPS 정합)
초기엔 월드에 `gz-sim-navsat/sensors/...` **플러그인을 직접 넣었더니 `gps_ok=False`**(로컬만 OK)로
막혔다. PX4 gz 는 **월드에 플러그인을 넣지 않고 실행 시 자동 주입**한다(default.sdf 에 plugin 0개).
그래서 `kangwon.sdf` 를 **default.sdf 구조 그대로** 두고 지형·datum 만 바꾸니 **`gps_ok=True` 즉시 성공**.
→ 커스텀 gz 월드를 PX4 에 쓸 때는 default.sdf 를 베이스로 하고 최소만 바꾼다.

## ★ 단일 datum (세 곳 일치 필수)
lat **38.011200**, lon **128.122643**, alt **172.1** (격자 최저셀=계곡).
`kangwon.sdf <spherical_coordinates>` · `PX4_HOME_*` · `gz_bridge.DATUM_*` 세 곳이 같아야 정합.

## 실행 (검증된 순서)

### 1) 시뮬레이터 확인 (엔트리포인트 우회)
```bash
docker run --rm --entrypoint bash px4io/px4-sitl-gazebo:latest -lc 'gz sim --version; ls /opt/px4-gazebo/share/gz/worlds/'
# Gazebo Sim 8.x (Harmonic) + worlds 폴더 확인
```

### 2) 강원 월드로 기동 (월드+heightmap 마운트, datum=PX4_HOME)
```bash
docker rm -f px4 2>/dev/null
docker run -d --name px4 --network host --restart unless-stopped \
  --cpus 1.5 -e HEADLESS=1 \
  -e PX4_HOME_LAT=38.011200 -e PX4_HOME_LON=128.122643 -e PX4_HOME_ALT=172.1 \
  -e PX4_GZ_WORLD=kangwon \
  -v $HOME/Codyssey_M2-1/gazebo/kangwon.sdf:/opt/px4-gazebo/share/gz/worlds/kangwon.sdf:ro \
  -v $HOME/Codyssey_M2-1/gazebo/kangwon_heightmap.png:/opt/px4-gazebo/share/gz/worlds/kangwon_heightmap.png:ro \
  px4io/px4-sitl-gazebo:latest
sleep 25
docker exec px4 gz model --list          # kangwon_terrain, x500_0 확인
```

### 3) GPS 정합 확인 (gps_ok=True 대기)
```bash
python3 -W ignore - << 'PY' 2>/dev/null
import asyncio
from mavsdk import System
async def main():
    d=System(); await d.connect(system_address="udpin://0.0.0.0:14540")
    async for s in d.core.connection_state():
        if s.is_connected: break
    for i in range(60):
        h=await anext(d.telemetry.health())
        print(f"[{i:2d}s] gps_ok={h.is_global_position_ok} local_ok={h.is_local_position_ok}")
        if h.is_global_position_ok: print("✅ 비행 가능"); return
        await asyncio.sleep(1)
asyncio.run(main())
PY
```

### 4) 드론을 화재 셀로 비행
```bash
cd ~/Codyssey_M2-1
python3 -W ignore demo_fly_to_fire.py --col 15 --row 115
# 거리(m)가 줄며 alt 가 지형+50m 로 상승 → ✅ 화재 타깃 도착 → RTL
```
`--col/--row` 로 화재 격자셀 지정, 또는 인자 없이 실 환경 최고위험셀 자동 선택.

## 주의
- **2 vCPU 는 빠듯**(px4/NOTE.md). load 5+ 면 `-e PX4_SIM_SPEED_FACTOR=0.5` 추가하거나 EC2(4 vCPU↑) 권장.
- datum(홈)을 옮기려면 heightmap 모델 `pose.z = 172.1 − 새홈고도` 로 바꾸고 세 곳 datum 갱신.
- 경로 중간 능선 회피는 미구현(직선 goto). 타깃 고도만 지형+여유 반영.
