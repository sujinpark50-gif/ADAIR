# -*- coding: utf-8 -*-
"""
integration/uav_launcher.py
===========================
김동현님의 UAV Local Agent(px4/uav-agent, FastAPI)를 UAV 1대 = 프로세스 1개 = 포트 1개
구조 그대로 서브프로세스로 띄운다. 각 서버는 UAV_MODE=mock(MockDrone)이라 PX4/Gazebo가
필요 없다. 서버가 준비되면 config.UAV_ENDPOINTS를 채우고 UAV_CONNECTION_MODE='real'로 바꾼다.

→ 이렇게 하면 이원규님의 실제 HTTP 커넥터(_send_uav_command_real 등)와
  김동현님의 실제 evaluate/execute 로직이 진짜 HTTP로 붙는다(인프로세스 재구현 아님).
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

try:
    import requests
except ImportError:
    requests = None

import config

_REPO_ROOT = Path(__file__).resolve().parent.parent
_UAV_DIR = _REPO_ROOT / "px4" / "uav-agent"

# 띄울 UAV: 재평가(REJECT→다른 후보) 시연을 위해 base A에 2대
_UAV_PLAN = [
    ("A-uav1", 8000),
    ("A-uav2", 8001),
]

_procs: list[subprocess.Popen] = []


def _wait_health(url: str, timeout_s: float = 25.0) -> bool:
    if requests is None:
        return False
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            r = requests.get(f"{url}/health", timeout=2)
            if r.status_code == 200:
                return True
        except Exception:
            pass
        time.sleep(0.4)
    return False


def start() -> list[str]:
    """UAV 서버들을 띄우고 준비된 resource_id 목록을 반환. config를 real 모드로 전환."""
    if requests is None:
        print("[UAV] requests 미설치 → UAV 서버 연동 생략(UAV는 mock 유지). "
              "pip install requests 후 다시 실행하면 실제 HTTP 연동이 켜집니다.")
        return []

    started = []
    for uav_id, port in _UAV_PLAN:
        env = dict(os.environ, UAV_ID=uav_id, UAV_MODE="mock")
        log = open(_REPO_ROOT / "logs" / f"uav_{uav_id}.log", "w")
        p = subprocess.Popen(
            [sys.executable, "-m", "uvicorn", "main:app", "--host", "127.0.0.1",
             "--port", str(port), "--log-level", "warning"],
            cwd=str(_UAV_DIR), env=env, stdout=log, stderr=subprocess.STDOUT,
        )
        _procs.append(p)
        url = f"http://127.0.0.1:{port}"
        if _wait_health(url):
            config.UAV_ENDPOINTS[uav_id] = url
            started.append(uav_id)
            print(f"[UAV] {uav_id} 준비됨 → {url} (MockDrone)")
        else:
            print(f"[UAV] {uav_id} 기동 실패(포트 {port}). logs/uav_{uav_id}.log 확인")

    if started:
        config.UAV_CONNECTION_MODE = "real"   # 이원규님 실제 HTTP 커넥터 경로 사용
    return started


def force_reject(uav_id: str, reason: str = "failsafe") -> None:
    """시나리오 재현용 — 특정 UAV를 REJECT 상태로 만든다(/mock/set)."""
    if requests is None:
        return
    url = config.UAV_ENDPOINTS.get(uav_id)
    if not url or url == "TBD":
        return
    params = {"failsafe": True} if reason == "failsafe" else {"battery_pct": 5.0}
    try:
        requests.post(f"{url}/mock/set", params=params, timeout=3)
    except Exception:
        pass


def stop() -> None:
    for p in _procs:
        try:
            p.terminate()
        except Exception:
            pass
    for p in _procs:
        try:
            p.wait(timeout=5)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    _procs.clear()
