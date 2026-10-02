# -*- coding: utf-8 -*-
"""
ADAIR 서버 일괄 실행·중지·상태 확인 (통합 담당)

관제판을 띄우려면 터미널을 여러 개 열어 서버마다 명령을 따로 쳐야 했다.
이 스크립트 하나로 켜고, 끄고, 상태를 보고, 하나만 다시 켤 수 있다.
공동 수락시험(J1~J6: 서버 끄기·재시작)에서도 같은 명령을 쓴다.

    python run_servers.py start                 # 기본: UAV(8000) + 총괄(8200) + 관제판(8080)
    python run_servers.py start --all           # + UAV2(8001), UGV(8100)
    python run_servers.py start --ugv           # 기본 + UGV(8100)
    python run_servers.py start --via-orch      # 관제판 출동을 총괄 경유로 (INT-01 스위치)
    python run_servers.py start --demo-extinguish   # 도착 시 진화 반영 (INT-02 시연 전용)
    python run_servers.py start --uav-url http://<EC2 IP>:8000   # 원격(real) UAV 에 연결, 로컬 UAV 는 띄우지 않음
    python run_servers.py status                # 켜진 서버와 응답 여부
    python run_servers.py stop                  # 이 스크립트로 켠 서버 모두 끄기
    python run_servers.py stop orch             # 하나만 끄기
    python run_servers.py restart orch          # 하나만 다시 켜기 (처음 켤 때의 설정 그대로)

서버 이름: uav1, uav2, ugv, orch, web
각 서버의 출력은 logs/servers/<이름>.log 에 쌓인다 (logs/ 는 git 에 올라가지 않음).
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RUN_DIR = ROOT / "logs" / "servers"
STATE_FILE = RUN_DIR / "running.json"
PY = sys.executable
IS_WIN = os.name == "nt"


def server_defs(opts: dict) -> dict:
    """서버별 실행 방법. opts 는 start 할 때 받은 설정 (restart 때 그대로 재사용)."""
    uav_url = opts.get("uav_url") or "http://127.0.0.1:8000"
    web_env = {"UAV_AGENT_URL": uav_url, "UAV_ID": "A-uav1"}
    if opts.get("via_orch"):
        web_env["WEB_DISPATCH_VIA_ORCH"] = "1"
    if opts.get("demo_extinguish"):
        web_env["WEB_DEMO_EXTINGUISH"] = "1"
    uv = [PY, "-m", "uvicorn"]
    return {
        "uav1": {"cwd": ROOT / "uav" / "uav-agent", "port": 8000, "health": "/health",
                 "cmd": uv + ["main:app", "--host", "127.0.0.1", "--port", "8000"],
                 "env": {"UAV_ID": "A-uav1", "UAV_MODE": "mock"}, "desc": "UAV A-uav1 (mock)"},
        "uav2": {"cwd": ROOT / "uav" / "uav-agent", "port": 8001, "health": "/health",
                 "cmd": uv + ["main:app", "--host", "127.0.0.1", "--port", "8001"],
                 "env": {"UAV_ID": "A-uav2", "UAV_MODE": "mock"}, "desc": "UAV A-uav2 (mock)"},
        "ugv":  {"cwd": ROOT, "port": 8100, "health": "/health",
                 "cmd": uv + ["ugv.server:app", "--host", "127.0.0.1", "--port", "8100"],
                 "env": {"UGV_DRIVER": "sim"}, "desc": "UGV 서버 (sim)"},
        "orch": {"cwd": ROOT, "port": 8200, "health": "/health",
                 "cmd": [PY, "-m", "orchestrator.api"], "env": {}, "desc": "총괄 오케스트레이터"},
        "web":  {"cwd": ROOT, "port": 8080, "health": "/",
                 "cmd": uv + ["web.app:app", "--host", "127.0.0.1", "--port", "8080"],
                 "env": web_env, "desc": "웹 관제판"},
    }


ORDER = ["uav1", "uav2", "ugv", "orch", "web"]   # 켜는 순서 (관제판은 마지막)


# ---------------------------------------------------------------- 상태 파일
def load_state() -> dict:
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except Exception:
        return {"opts": {}, "servers": {}}


def save_state(st: dict) -> None:
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    STATE_FILE.write_text(json.dumps(st, ensure_ascii=False, indent=2), encoding="utf-8")


# ---------------------------------------------------------------- 프로세스
def alive(pid: int) -> bool:
    if not pid:
        return False
    if IS_WIN:
        out = subprocess.run(["tasklist", "/FI", f"PID eq {pid}", "/NH"], capture_output=True, text=True).stdout
        return str(pid) in out
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def kill(pid: int) -> None:
    if not alive(pid):
        return
    if IS_WIN:
        subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True)
        return
    try:
        os.killpg(pid, signal.SIGTERM)          # 새 세션으로 띄웠으므로 그룹째 종료
    except OSError:
        os.kill(pid, signal.SIGTERM)
    for _ in range(30):
        if not alive(pid):
            return
        time.sleep(0.1)
    try:
        os.killpg(pid, signal.SIGKILL)
    except OSError:
        pass


def responds(port: int, path: str, timeout: float = 1.5) -> bool:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout) as r:
            return 200 <= r.status < 500
    except Exception:
        return False


def launch(name: str, d: dict) -> int:
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    log = open(RUN_DIR / f"{name}.log", "a", encoding="utf-8")
    log.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} 시작: {' '.join(map(str, d['cmd']))}\n")
    log.flush()
    env = {**os.environ, **d["env"], "PYTHONIOENCODING": "utf-8"}
    kw = {"cwd": str(d["cwd"]), "env": env, "stdout": log, "stderr": subprocess.STDOUT,
          "stdin": subprocess.DEVNULL}
    if IS_WIN:
        kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
    else:
        kw["start_new_session"] = True
    return subprocess.Popen(d["cmd"], **kw).pid


def wait_ready(d: dict, pid: int, limit: float = 40.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < limit:
        if responds(d["port"], d["health"]):
            return True
        if not alive(pid):
            return False
        time.sleep(0.5)
    return False


# ---------------------------------------------------------------- 명령
def start_one(name: str, defs: dict, st: dict) -> None:
    d = defs[name]
    old = st["servers"].get(name, {}).get("pid")
    if old and alive(old):
        print(f"  {name:5} 이미 실행 중 (PID {old}) — 건너뜀")
        return
    if responds(d["port"], d["health"], timeout=0.8):
        print(f"  {name:5} ⚠ 포트 {d['port']}을 다른 프로그램이 이미 쓰는 중 — 건너뜀 "
              f"(직접 켜 둔 터미널이 있으면 그 창에서 Ctrl+C)")
        return
    pid = launch(name, d)
    st["servers"][name] = {"pid": pid, "port": d["port"]}
    save_state(st)
    ok = wait_ready(d, pid)
    mark = "✅ 응답" if ok else "❌ 응답 없음"
    print(f"  {name:5} {mark}  :{d['port']}  {d['desc']}  (PID {pid})"
          + ("" if ok else f"  → logs/servers/{name}.log 확인"))


def cmd_start(a) -> None:
    st = load_state()
    opts = {"uav_url": a.uav_url, "via_orch": a.via_orch, "demo_extinguish": a.demo_extinguish}
    # 전체 시작(이름 생략)은 이번에 준 설정으로 새로 정하고,
    # 일부만 켤 때(start orch 등)는 옵션을 주지 않으면 지난 설정을 그대로 쓴다.
    if not a.names or any(v for v in opts.values()):
        st["opts"] = opts
    defs = server_defs(st["opts"])
    names = a.names or (["uav1", "uav2", "ugv", "orch", "web"] if a.all
                        else ["uav1", "orch", "web"] + (["ugv"] if a.ugv else []))
    if a.uav_url:                                   # 원격 UAV 를 쓰면 로컬 UAV 는 띄우지 않는다
        names = [n for n in names if n not in ("uav1", "uav2")]
    print("서버 시작:")
    for n in [x for x in ORDER if x in names]:
        start_one(n, defs, st)
    print("\n관제판: http://127.0.0.1:8080   총괄 판단 화면: http://127.0.0.1:8200/board")
    print("끄기: python run_servers.py stop")


def cmd_stop(a) -> None:
    st = load_state()
    names = a.names or list(st["servers"])
    if not names:
        print("이 스크립트로 켠 서버가 없습니다.")
        return
    for n in reversed([x for x in ORDER if x in names]):
        info = st["servers"].pop(n, None)
        if not info:
            print(f"  {n:5} 기록 없음")
            continue
        kill(info["pid"])
        print(f"  {n:5} 종료 (PID {info['pid']})")
    save_state(st)


def cmd_restart(a) -> None:
    st = load_state()
    defs = server_defs(st.get("opts") or {})
    for n in [x for x in ORDER if x in a.names]:
        info = st["servers"].pop(n, None)
        if info:
            kill(info["pid"])
            print(f"  {n:5} 종료 (PID {info['pid']})")
        time.sleep(0.5)
        start_one(n, defs, st)


def cmd_status(a) -> None:
    st = load_state()
    defs = server_defs(st.get("opts") or {})
    print(f"{'이름':5} {'포트':>5}  {'응답':6} {'PID':>7}  설명")
    for n in ORDER:
        d = defs[n]
        pid = st["servers"].get(n, {}).get("pid")
        on = responds(d["port"], d["health"], timeout=0.8)
        who = (str(pid) if pid and alive(pid) else ("직접 실행" if on else "-"))
        print(f"{n:5} {d['port']:>5}  {'✅' if on else '·':6} {who:>7}  {d['desc']}")
    o = st.get("opts") or {}
    print(f"\n설정: UAV={o.get('uav_url') or '로컬 mock'} · 출동 총괄 경유={'켜짐' if o.get('via_orch') else '꺼짐'}"
          f" · 진화 시연={'켜짐' if o.get('demo_extinguish') else '꺼짐'}")


def main() -> None:
    for stream in (sys.stdout, sys.stderr):          # Windows 콘솔·리디렉션에서도 한글·기호가 깨지지 않게
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
    p = argparse.ArgumentParser(description="ADAIR 서버 일괄 실행·중지·상태 확인")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("start", help="서버 켜기")
    s.add_argument("names", nargs="*", default=[], metavar="이름")
    s.add_argument("--all", action="store_true", help="UAV2·UGV 포함 전부")
    s.add_argument("--ugv", action="store_true", help="UGV 서버(8100) 포함")
    s.add_argument("--uav-url", help="원격 UAV 서버 주소 (예: EC2 real). 지정하면 로컬 UAV 를 띄우지 않음")
    s.add_argument("--via-orch", action="store_true", help="관제판 출동을 총괄 경유로 (INT-01)")
    s.add_argument("--demo-extinguish", action="store_true", help="도착 시 진화 반영 (INT-02 시연 전용)")
    s.set_defaults(fn=cmd_start)
    for name, fn, h in (("stop", cmd_stop, "서버 끄기 (이름 생략 시 전부)"),
                        ("restart", cmd_restart, "서버 다시 켜기")):
        x = sub.add_parser(name, help=h)
        x.add_argument("names", nargs="*" if name == "stop" else "+", metavar="이름")
        x.set_defaults(fn=fn)
    sub.add_parser("status", help="상태 보기").set_defaults(fn=cmd_status)
    a = p.parse_args()
    bad = [n for n in (getattr(a, "names", None) or []) if n not in ORDER]
    if bad:
        p.error(f"알 수 없는 서버 이름: {', '.join(bad)} (가능: {', '.join(ORDER)})")
    a.fn(a)


if __name__ == "__main__":
    main()
