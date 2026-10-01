# -*- coding: utf-8 -*-
"""
UGV 서버(ugv/server.py) ↔ 공통 커넥터 real 모드 연결 확인 (통합 점검용)

사전: 다른 터미널에서  python -m uvicorn ugv.server:app --port 8100   (UGV_DRIVER 기본값 sim, PX4 불필요)
실행: python -m tests.ugv_server_check
"""

import config
config.UGV_CONNECTION_MODE = "real"

from connectors import ugv_connector
from interfaces.schema import Assignment, is_registered_reason

ok = True


def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({info})" if info else ""))


print(f"UGV 서버: {config.UGV_SERVER_URL}\n")

# 1) 상태 조회 — 공통 ResourceStatus 로 변환되는가
statuses = ugv_connector.get_ugv_status("A")
ids = [s.resource_id for s in statuses]
check("상태 조회 (A 거점)", len(statuses) > 0, f"{ids}")
check("resource_id 규약 §2 (truck 없음)", all("truck" not in i for i in ids))

# 2) 수행가능성 판단 — 도로망 영역 안(자원 위치 근처)은 ACCEPT, 밖은 REJECT
home = statuses[0]
near = Assignment("CHK_1", home.resource_id, home.location_lat + 0.01, home.location_lon + 0.01)
r = ugv_connector.send_ugv_command("CHK_1", near, decision_id="DEC_CHK_1")
eta = (r.evidence or {}).get("eta_sec")
check("근처 목표 → ACCEPT", r.response == "ACCEPT", f"{r.response} {r.reason or ''} eta={eta}")
check("판정 근거(evidence) 전달", bool(r.evidence), f"target_node={(r.evidence or {}).get('target_node')}")

far = Assignment("CHK_2", home.resource_id, 37.50, 127.50)
r2 = ugv_connector.send_ugv_command("CHK_2", far, decision_id="DEC_CHK_2")
check("영역 밖 목표 → REJECT", r2.response == "REJECT", f"{r2.reason}")
check("REJECT 사유가 공통 목록에 등록된 코드", is_registered_reason(r2.reason), f"{r2.reason}")

# 3) 관측 — simulation_time_s 로 채워지는가
obs = ugv_connector.get_ugv_observation(home.resource_id, 12.5, "CHK_1", "DEC_CHK_1")
check("관측 조회 (simulation_time_s 규약 §1)", obs.simulation_time_s == 12.5,
      f"type={obs.observation_type}")

print("\n모두 통과" if ok else "\n실패 항목 있음")
