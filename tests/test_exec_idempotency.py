# -*- coding: utf-8 -*-
"""UAV-01 · UGV-02 · UGV-04 · 공동 수락시험 J3 — 실제 서버 프로세스를 띄우고 강제 종료(SIGKILL)·재시작한다.

실행 (저장소 루트):  python -m pytest tests/test_exec_idempotency.py -q
- UAV Agent: uav/uav-agent (UAV_MODE=mock, MockDrone 100배속)
- UGV 서버 : ugv.server (UGV_DRIVER=sim, 실제 도로망)
"""

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import httpx
import pytest

ROOT = Path(__file__).resolve().parents[1]
UAV_DIR = ROOT / "uav" / "uav-agent"
UAV = "A-uav1"
TARGET = {"lat": 38.1250, "lon": 128.2060, "alt_m_amsl": 240.0}     # 원통 기지 근처 (짧은 비행)


def _port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p


class Server:
    def __init__(self, args, cwd, env, health="/health"):
        self.args, self.cwd, self.env, self.health = args, cwd, env, health
        self.port = _port()
        self.url = f"http://127.0.0.1:{self.port}"
        self.p = None

    def start(self, timeout=40):
        self.p = subprocess.Popen([sys.executable, "-m", "uvicorn", *self.args, "--port", str(self.port),
                                   "--log-level", "warning"], cwd=str(self.cwd),
                                  env={**os.environ, **self.env}, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        end = time.time() + timeout
        while time.time() < end:
            try:
                if httpx.get(self.url + self.health, timeout=1).status_code == 200:
                    return self
            except httpx.HTTPError:
                pass
            if self.p.poll() is not None:
                raise RuntimeError(self.p.stderr.read().decode()[-2000:])
            time.sleep(0.3)
        raise TimeoutError("server did not start")

    def crash(self):
        self.p.kill(); self.p.wait()          # 정상 종료 훅 없이 끊는다

    def restart(self):
        self.crash(); return self.start()


@pytest.fixture
def uav(tmp_path):
    s = Server(["main:app"], UAV_DIR, {"UAV_ID": UAV, "UAV_MODE": "mock", "UAV_STATE_DIR": str(tmp_path)}).start()
    yield s
    if s.p.poll() is None:
        s.crash()


def body(tid="ATT-1", **over):
    b = {"task_id": tid, "decision_id": "DEC-1", "target": dict(TARGET), "observation_type": "THERMAL"}
    b.update(over)
    return b


def ex(s, b):
    return httpx.post(f"{s.url}/uav/{UAV}/execute", json=b, timeout=10)


def task(s, tid):
    return httpx.get(f"{s.url}/uav/{UAV}/task/{tid}", timeout=5)


def wait(s, tid, pred, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        r = task(s, tid)
        if r.status_code == 200 and pred(r.json()):
            return r.json()
        time.sleep(0.05)
    raise TimeoutError(task(s, tid).text)


# ---------------------------------------------------------------- UAV-01
def test_uav_same_key_same_body_is_not_a_second_dispatch(uav):
    r1 = ex(uav, body())
    assert r1.status_code == 200 and r1.json()["duplicate"] is False
    r2 = ex(uav, body())                                  # 응답 유실 후 재전송
    assert r2.status_code == 200 and r2.json()["duplicate"] is True and r2.json()["uav_id"] == UAV
    h = httpx.get(uav.url + "/health").json()["exec_store"]
    assert h["tasks"] == 1                                # 실행은 하나
    st = task(uav, "ATT-1").json()
    assert st["uav_id"] == UAV                            # UAV-01 ④


def test_uav_same_key_other_body_conflicts_and_keeps_original(uav):
    ex(uav, body())
    r = ex(uav, body(target={**TARGET, "lat": 38.13}))
    assert r.status_code == 409 and r.json()["detail"]["reason"] == "EXECUTION_ID_CONFLICT"
    assert task(uav, "ATT-1").json()["status"] in ("STARTED", "IN_PROGRESS")   # 기존 실행 유지


def test_uav_refused_request_does_not_consume_key(uav):
    bad = body(target={"lat": 38.125, "lon": 128.206})    # 지면고도 없음 → 400, 실행 안 함
    assert ex(uav, bad).status_code == 400
    assert ex(uav, body()).status_code == 200             # 고쳐서 같은 키로 다시 보낼 수 있다


def test_uav_restart_before_observation(uav):
    """J3: 비행 중 서버가 죽었다 살아남 → 같은 키 조회 가능, 관측 전 끊김으로 정리, 재요청해도 재출동 없음"""
    ex(uav, body())
    wait(uav, "ATT-1", lambda d: (d.get("progress") or {}).get("phase") == "ENROUTE")
    uav.restart()
    st = task(uav, "ATT-1")
    assert st.status_code == 200                          # 예전에는 404 (메모리 기록 소실)
    d = st.json()
    assert d["status"] == "FAILED" and d["reason"] == "AGENT_RESTARTED_BEFORE_OBSERVATION"
    assert d["observation"] is None and d["progress"]["phase"] == "DONE"
    assert d["agent_restart"]["physical_basis"] == "MOCK_PROVIDER_RESET_TO_HOME"
    again = ex(uav, body())
    assert again.json()["duplicate"] is True
    state = httpx.get(f"{uav.url}/uav/{UAV}/state").json()
    assert state["current_task_id"] is None               # 새 비행이 시작되지 않았다


def test_uav_restart_after_observation_keeps_observation(uav):
    """UAV-10 ②: 관측 뒤(복귀 중) 끊겨도 관측 ID·내용·시각은 그대로"""
    ex(uav, body())
    d1 = wait(uav, "ATT-1", lambda d: d["status"] == "COMPLETED" and d["progress"]["phase"] == "RETURNING")
    uav.restart()
    d2 = task(uav, "ATT-1").json()
    assert d2["status"] == "COMPLETED" and d2["observation"] == d1["observation"]
    assert d2["progress"]["phase"] == "DONE"


# ---------------------------------------------------------------- UGV-02 · UGV-04
@pytest.fixture(scope="module")
def ugv(tmp_path_factory):
    s = Server(["ugv.server:app"], ROOT, {"UGV_DRIVER": "sim",
                                         "UGV_STATE_DIR": str(tmp_path_factory.mktemp("ugv"))}).start(60)
    yield s
    if s.p.poll() is None:
        s.crash()


def ugv_body(tid, lat=37.97, lon=128.28):      # 원통→기린 쪽 먼 목표 (DEMO_SPEED 로도 수 초 걸림)
    return {"task_id": tid, "decision_id": "DEC-U1", "target": {"lat": lat, "lon": lon}}


def test_ugv_duplicate_conflict_and_restart(ugv):
    url = f"{ugv.url}/ugv/A-ugv1"
    r1 = httpx.post(url + "/execute", json=ugv_body("U-1"), timeout=10)
    assert r1.status_code == 200 and r1.json()["duplicate"] is False
    r2 = httpx.post(url + "/execute", json=ugv_body("U-1"), timeout=10)          # 수행 중 같은 키
    assert r2.status_code == 200 and r2.json()["duplicate"] is True
    assert r2.json()["target_node"] == r1.json()["target_node"]                  # 처음 응답 그대로
    r3 = httpx.post(url + "/execute", json=ugv_body("U-1", lat=38.0, lon=128.25), timeout=10)
    assert r3.status_code == 409 and r3.json()["detail"]["reason"] == "EXECUTION_ID_CONFLICT"
    time.sleep(0.5)
    assert httpx.get(url + "/task/U-1").json()["status"] in ("STARTED", "IN_PROGRESS")
    ugv.restart()
    st = httpx.get(url + "/task/U-1", timeout=5)
    assert st.status_code == 200
    d = st.json()
    assert d["status"] == "FAILED" and d["error"].startswith("AGENT_RESTARTED") and d["progress"]["phase"] == "FAILED"
    assert d["agent_restart"]["physical_basis"] == "SIM_DRIVER_RESET"
    r4 = httpx.post(url + "/execute", json=ugv_body("U-1"), timeout=10)          # 재시작 뒤 재전송
    assert r4.json()["duplicate"] is True and r4.json()["current_status"] == "FAILED"
    units = {u["resource_id"]: u for u in httpx.get(f"{ugv.url}/ugv").json()}
    assert units["A-ugv1"]["current_task_id"] is None                            # 새 주행 없음


# ---------------------------------------------------------------- J3 (총괄 연결)
def test_orchestrator_handover_after_uav_restart(uav, tmp_path):
    """총괄이 보낸 실행 뒤 UAV 서버가 재시작 → 예전에는 404 로 UNKNOWN(수동 해소 대기),
    지금은 '관측 전 끊김'을 확인하고 기체가 READY 일 때 반납·목적은 재배정 대기."""
    sys.path.insert(0, str(ROOT))
    os.environ["ORCH_SKIP_DOTENV"] = "1"
    from orchestrator import config as oc
    from orchestrator.engine import Orchestrator
    from orchestrator.knowledge import EnvStationFeed
    from orchestrator.env_adapter import FixtureEnv
    from orchestrator.ledger import Ledger
    from orchestrator.resources import UavClient, UgvClient

    cell = {"cell_id": "C1", "lat": TARGET["lat"], "lon": TARGET["lon"], "cell_size_m": 90.0,
            "ground_amsl_m": TARGET["alt_m_amsl"], "fire_state": "BURNING"}
    env = FixtureEnv(fire_cells=[cell], wind_ms=3.0)
    orch = Orchestrator(Ledger(str(tmp_path / "o.sqlite3")), env, UavClient({UAV: uav.url}),
                        UgvClient("TBD"), weather_feeds=[EnvStationFeed(env, TARGET["lat"], TARGET["lon"])])
    task_, _ = orch.submit_task({"request_id": "R1", "kind": "RECON",
                                 "target": {k: cell[k] for k in ("lat", "lon", "ground_amsl_m", "cell_id")},
                                 "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL"}})
    out = orch.dispatch(task_.task_id)
    assert out["status"] == "STARTED", out
    wait(uav, out["attempt_id"], lambda d: (d.get("progress") or {}).get("phase") == "ENROUTE")
    uav.restart()
    orch.poll()
    att = orch.ledger.get_attempt(out["attempt_id"])
    t = orch.ledger.get_task(task_.task_id)
    assert att["substatus"] == "RELEASED"                 # 기체 READY 확인 후 반납 (점유 해제)
    assert t.purpose_status == "PENDING" and t.hold_reason.startswith("HANDOVER")
    execs = [e for e in orch.ledger.events() if e["event_type"] == "EXECUTION_SENT"]
    assert len(execs) == 1                                # 자동 재전송 없음
