# -*- coding: utf-8 -*-
"""
run_integrated.py
=================
ADAIR 산불 대응 멀티에이전트 — 5개 파트를 하나로 묶어 실행하는 통합 엔트리.

    python run_integrated.py            # 기본 8스텝
    python run_integrated.py 15         # 스텝 수 지정

무엇이 '진짜'로 도는가 (PX4·Gazebo·GPU·ROS 전부 불필요):
  · 환경(김은주)  : environment/의 실제 산불 CA 모델을 매 스텝 확산.
                    rasterio/데이터가 없으면 내장 stateful 산불 모델로 자동 전환.
  · UAV(김동현)   : px4/uav-agent FastAPI 서버(MockDrone)를 서브프로세스로 띄우고,
                    이원규님 실제 HTTP 커넥터가 evaluate/execute를 HTTP로 호출.
  · UGV(박유홍)   : 실제 도로 그래프 + Dijkstra로 도달성·ETA·도로차단 재탐색.
  · 통합/재평가(이원규) : main.process_task의 폐루프·재평가·Decision ID를 그대로 사용.
  · 총괄/Safety(박수진) : 계약 기반 Mock 후보선택·Safety 분기(설계 산출물).

핵심: connectors 의 'integrated' 모드로 각 파트의 실제 모듈을 지연 로딩해 연결한다.
      (환경 커넥터는 integrated 를 real 과 동일하게 처리한다.)
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
os.chdir(ROOT)
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

os.makedirs("logs", exist_ok=True)

import config
import ids
import main
from logger import EventLogger
from connectors import fire_connector, ugv_connector, orchestrator_connector
from integration import bridge_fire, bridge_ugv, uav_launcher


def banner(t):
    print("\n" + "=" * 64)
    print(t)
    print("=" * 64)


def run(total_steps: int = 8):
    bridge_fire.reset()
    bridge_ugv.reset()

    # 1) 커넥터 모드 전환 (원본 config 기본값은 mock 유지 — 테스트 안 깨짐)
    config.FIRE_CONNECTION_MODE = "integrated"
    config.UGV_CONNECTION_MODE = "integrated"
    # ORCHESTRATOR/Safety 는 계약 Mock 그대로 (박수진님 산출물)

    banner("ADAIR 통합 실행 시작")

    # 2) UAV 서버(김동현) 기동 → 이원규 실제 HTTP 커넥터 활성화
    started_uavs = uav_launcher.start()

    run_id = ids.new_run_id()
    logger = EventLogger(run_id=run_id)
    print(f"\nrun_id={run_id}  ·  총 {total_steps} 스텝\n")

    try:
        banner("폐루프 (환경→Task→UAV 판단→Safety→실행→환경갱신)")
        for step in range(total_steps):
            ts = step * config.SIMULATION_TIMESTEP_SEC
            env = fire_connector.get_environment_state(ts)
            logger.log_event("ENVIRONMENT_CHANGE", ts, result="OK")

            pool = main.get_all_resources(ts)
            task = orchestrator_connector.create_task(ts, env)

            # 재평가 시연: step 2에서 A-uav1 배터리를 낮춰 LOW_BATTERY REJECT → A-uav2 인계
            if step == 2 and "A-uav1" in started_uavs:
                uav_launcher.force_reject("A-uav1", reason="battery")
                print("  * [시나리오] A-uav1 배터리 5% 주입 → 재평가 유발 예정")

            task, env = main.process_task(task, env, pool, logger, ts)

            fire_n = len(env.fire_cells)
            print(f"[step {step}] fire_cells={fire_n:3d}  wind={env.wind_speed:>4} m/s "
                  f"({env.spread_direction:>2})  →  task={task.task_id} {task.state}")

        # 3) UGV 실제 경로탐색(박유홍) 시연 — 폐루프와 별개로 Dijkstra·도로차단 재탐색을 보여줌
        banner("UGV 경로탐색 (박유홍 · 실제 Dijkstra + 도로차단 재탐색)")
        for line in bridge_ugv.demo():
            print(line)

        # 4) UGV 판단이 팀 스키마로 연결되는지 확인 (커넥터 경유)
        sample = ugv_connector.get_ugv_status("A")
        if sample:
            s = sample[0]
            print(f"\n  커넥터 경유 UGV 상태: {s.resource_id}  reachable={s.capability['reachable']} "
                  f"ETA={s.capability['eta_sec']}s  blocked={s.capability['road_blocked']}")

        banner("완료")
        print(f"이벤트 로그: {logger.log_path}")
        print("UAV 서버 로그: logs/uav_A-uav1.log, logs/uav_A-uav2.log")
        if not started_uavs:
            print("\n(참고) UAV 서버가 안 떠서 UAV는 mock으로 돌았습니다. "
                  "`pip install fastapi uvicorn requests` 후 다시 실행하면 실제 HTTP 연동이 켜집니다.")
    finally:
        uav_launcher.stop()


if __name__ == "__main__":
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    run(steps)
