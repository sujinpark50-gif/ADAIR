# ADAIR 통합 실행 안내 (run_integrated.py)

5개 파트(환경·UAV·UGV·통합·총괄)를 **하나의 명령으로 함께 실행**하기 위한 통합 계층입니다.
통합 계층은 `integration/` 과 `run_integrated.py` 에 있습니다. 기존 파일의 변경 내역은 PR 설명의 "변경 파일" 표를 참고하세요.

---

## 빠른 실행 (3줄)

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements-integrated.txt
python run_integrated.py            # 기본 8스텝 (python run_integrated.py 15 처럼 스텝 지정 가능)
```

- **Python 3.10 이상**, **저장소 루트에서 실행**하세요.
- **PX4·Gazebo·GPU·ROS 전부 필요 없습니다.**

---

## 무엇이 '진짜'로 도는가

| 파트 | 담당 | 통합 실행에서 | 
|---|---|---|
| 환경·산불 모델 | 김은주 | `environment/`의 **실제 CA 산불 확산 모델**을 매 스텝 진행.<br>`rasterio`+지형 데이터가 없으면 **내장 stateful 산불 모델로 자동 전환**(그래도 불이 실제로 번짐). |
| UAV | 김동현 | `px4/uav-agent` **FastAPI 서버(MockDrone)를 서브프로세스로 기동**(포트 8000·8001).<br>이원규님 실제 HTTP 커넥터가 `/evaluate`·`/execute`를 **진짜 HTTP로 호출**. |
| UGV | 박유홍 | **실제 도로 그래프 + Dijkstra**로 도달성·ETA·도로차단 재탐색. |
| 통합·재평가 | 이원규 | `main.process_task`의 폐루프·재평가·Decision ID를 그대로 사용. |
| 총괄·Safety | 박수진 | 계약 기반 Mock 후보선택·Safety 분기(현재 설계 산출물). |

실행하면 다음이 순서대로 보입니다:
1. UAV 서버 2대 기동 → 준비 완료 로그
2. 폐루프 8스텝: 화재 셀 수가 **매 스텝 늘어나고**(stateful), 각 Task가 UAV 판단→Safety→실행→환경갱신을 거침
3. **재평가 시연**: step 2에서 A-uav1 배터리를 낮춰 → A-uav1 거절 → **A-uav2가 인계**(실제 HTTP 재평가)
4. UGV 경로탐색 시연: `A→F1` 정상 → E3 차단 시 우회 → 더 차단 시 도달 불가 + 원인 도로 역추적
5. 이벤트 로그 위치 안내(`logs/simulation_log.jsonl`)

---

## 이 통합이 한 일 (변경 요약)

원본은 5개의 '섬'이라 그냥 묶으면 함께 돌지 않습니다. 다음만 **추가**했습니다:

- `integration/bridge_fire.py` — 환경 브리지(김은주 실모델 시도 → 실패 시 내장 산불 fallback)
- `integration/bridge_ugv.py` — UGV 브리지(박유홍 RoadGraph를 팀 스키마로 연결)
- `integration/uav_launcher.py` — UAV FastAPI 서버 기동·정리
- `run_integrated.py` — 전체를 묶는 실행 엔트리
- `connectors/fire_connector.py`, `connectors/ugv_connector.py` — `"integrated"` 모드 분기만 **추가**
  (기본값은 여전히 `mock`이라 기존 `python -m tests.it_cases`(IT-01/02)는 그대로 통과합니다.)

즉 각 파트의 **판단 로직 자체는 학생 원본 그대로**이고, 커넥터가 그 실제 모듈을 불러 연결할 뿐입니다.

---

## 아직 mock/미연동인 부분 (다음 통합 과제)

- **총괄·Safety(박수진)**: 실제 자원 평가정책·Safety 임계값은 아직 계약 Mock. 그래서 폐루프의 자원 선택은 UAV 우선입니다.
- **계약 확정 필요**: `timestamp`(경과 초 vs ISO8601), `resource_id` vs `uav_id`, `risk_score` 범위 — 3종 합의가 되면 실연동이 더 매끄러워집니다.
- **PX4/Gazebo 실비행**: 이 zip 범위 밖(별도 시뮬레이터 스택·CPU 필요). 통합 실행은 그 없이 도는 것이 목적입니다.

---

## 자주 나는 문제

- `ModuleNotFoundError: fastapi/uvicorn/requests` → `pip install -r requirements-integrated.txt`
- UAV 서버가 안 뜸 → 포트 8000·8001 사용 중일 수 있음. `logs/uav_A-uav1.log` 확인.
- 환경이 "내장 산불 모델로 전환"으로 뜸 → 정상입니다. `rasterio` 설치 시 김은주님 실모델로 자동 전환됩니다.
- `python: command not found` → `python3 run_integrated.py`

---

## Gazebo 3D 지형 연동 (gazebo/, gz_bridge.py) — 추가됨

환경 모델의 실측 강원 DEM(인제 원통·기린)을 **Gazebo(gz sim) 3D 지형**으로 올리고,
CA 격자·환경모델·PX4/gz 좌표를 **하나의 datum**으로 정렬하는 계층입니다. 통합 실행
(`run_integrated.py`)은 PX4 없이 돌지만, 이 파일들은 **실제 PX4-gz 시연** 단계에서 씁니다.

| 파일 | 역할 |
|---|---|
| `gazebo/kangwon.sdf` | gz sim 월드 — 강원 heightmap 지형 + spherical_coordinates(datum) + PX4 플러그인 |
| `gazebo/kangwon_heightmap.png` | 513×513 16-bit heightmap (DEM 172~1449 m, 태백산맥 능선 포함) |
| `gazebo/kangwon_terrain_preview.png` | 음영기복 미리보기(사람 확인용) |
| `gazebo/README_gazebo.md` | PX4-gz 실행법·단일 datum 규칙·좌표 사용 예 |
| `gz_bridge.py` (루트) | WGS84 ↔ EPSG:5186 ↔ gz 로컬 ENU + **CA 격자(col,row) 변환** |

**단일 datum**(= PX4_HOME = gz 원점): lat 38.011200, lon 128.122643, alt 172.1 m.
`kangwon.sdf`·`PX4_HOME_*`·`gz_bridge.DATUM_*` 세 곳이 모두 같아야 좌표가 맞물립니다.

**bridge_fire 화재셀 → 드론 명령** (격자 x,y 를 위경도/gz 로컬로):
```python
import gz_bridge as gb
c = gb.fire_cell_grid_to_gz(env_state.fire_cells[0])   # {x,y,...} → +lat,lon,east,north
# MAVSDK:  drone.goto(c['lat'], c['lon'], alt)
```
격자→위경도 변환은 실 DEM 과 오차 0 으로 검증됨. PX4-gz 로 월드를 띄우는 실행 명령과
주의사항은 `gazebo/README_gazebo.md` 참고. (월드 로드·드론 스폰은 팀 PX4-gz 환경에서 테스트)

---

## 웹 멀티에이전트 자동 관제판 (web/, 최종)

브라우저에서 강원 지형·산불(CA)·드론·지상자원(UGV/소방차)을 실시간으로 보고, 버튼 하나로
감지→출동→진화→재확산이 도는 완전 자동 폐루프. FastAPI 게이트웨이(:8080)가 gz_bridge(좌표)·
fire_connector(환경 CA)·ugv/road_graph(도로 경로)를 감싸고, uav-agent(:8000, real)로 실제 PX4 제어.

실행 (repo 루트에서, PX4+gz+uav-agent real 기동 후):
    source ~/uav-venv/bin/activate
    pip install fastapi uvicorn httpx
    uvicorn web.app:app --host 0.0.0.0 --port 8080
    # 브라우저 http://localhost:8080 → "자동 시작"

파일: web/app.py(게이트웨이+UI+자동 폐루프), web/static/auto.js(자동 프론트),
     gz_bridge.py(좌표), ugv/graph_data_demo.py(도로망), connectors/fire_connector.py(강원 CA real),
     gazebo/kangwon.sdf(default 기반, GPS 정합 성공버전) + kangwon_heightmap.png.
