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
    python run_servers.py start --twin          # 2019 인제 디지털 트윈 (멘토 패치, tools/run_twin.sh 와 같은 구성)
                                                #   환경 계약 서버(8300) + UAV 2대(인제) + 예비 UAV 2대(원통 8002·8003) + UGV + 총괄(팀 환경) + 관제판 + 트윈 시계 + 야간 순찰
    python run_servers.py start --twin --extra-fleet   # + 원통 UGV 2대 (C-ugv1·2)
                                                #   + 지상 출동 요청(불 확인 → 소방차 마을회관 방어, TWIN_DISPATCH_ARGS 로 옵션)
                                                #   관제판은 환경 서버를 읽기만 하고 출동은 총괄 경유. 3D 화면: /inje3d
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


def twin_env(opts: dict) -> dict:
    """트윈 모드 공통 설정 (tools/run_twin.sh 와 같은 값). 시나리오 파일의 twin_env 를 그대로 쓴다."""
    if not opts.get("twin"):
        return {}
    d = json.loads((ROOT / "web" / "static" / "inje2019" / "scenario.json").read_text(encoding="utf-8"))
    env = {k: str(v) for k, v in d["twin_env"].items()}
    run = Path(opts.get("twin_dir") or RUN_DIR)
    env.update({"ENV_STATE_DIR": str(run / "env"), "ENV_RESUME": "0",
                # 총괄 브랜치 2026-10-07: 신고 2분 뒤, 불 지도 1분 단위 (CA 35분 단계 사이 채우기). 밖에서 주면 그 값
                "ENV_REPORT_DELAY_S": os.environ.get("ENV_REPORT_DELAY_S", "120"),
                "ENV_SUBTICK_S": os.environ.get("ENV_SUBTICK_S", "60")})
    return env


def twin_uav_home(opts: dict) -> str:
    """mock 드론을 발화점 700 m 밖 차량 집결지에서 띄운다 (run_twin.sh 의 forward 이륙과 같음)."""
    if not opts.get("twin") or opts.get("uav_url"):
        return ""
    import math
    sys.path.insert(0, str(ROOT))
    import gz_bridge as gb
    d = json.loads((ROOT / "web" / "static" / "inje2019" / "scenario.json").read_text(encoding="utf-8"))
    path, sp = d["vehicles"][0]["path"], d["spring"]
    p = next((q for q in path if math.hypot(q[0] - sp["gx"], q[1] - sp["gy"]) * 90 <= 700), path[-1])
    lat, lon = gb.grid_cell_to_latlon(p[0], p[1])
    return f"{lat:.6f},{lon:.6f},{gb.terrain_elev(round(p[0]), round(p[1])):.1f}"


# 지상 출동 운용자 옵션 (예: TWIN_DISPATCH_ARGS="--trigger sunset --site NAMJEON1_HALL,INJE_REST_AREA")
DISPATCH_ARGS = os.environ.get("TWIN_DISPATCH_ARGS", "").split()


def server_defs(opts: dict) -> dict:
    """서버별 실행 방법. opts 는 start 할 때 받은 설정 (restart 때 그대로 재사용)."""
    uav_url = opts.get("uav_url") or "http://127.0.0.1:8000"
    tw = twin_env(opts)
    run = Path(opts["twin_dir"]) if opts.get("twin") and opts.get("twin_dir") else RUN_DIR
    web_env = {"UAV_AGENT_URL": uav_url, "UAV_ID": "A-uav1"}
    if opts.get("via_orch") or opts.get("twin"):
        web_env["WEB_DISPATCH_VIA_ORCH"] = "1"
    if opts.get("demo_extinguish"):
        web_env["WEB_DEMO_EXTINGUISH"] = "1"
    if opts.get("twin"):
        web_env["WEB_ENV_URL"] = "http://127.0.0.1:8300"
    speed = str(opts.get("twin_speed") or 100)
    # 트윈: 드론 mock 배속 = 트윈 시계 배속 (UAV_MOCK_TIME_SCALE, 2026-10-07)
    # 돌아가는 중에 새 임무 받기 (드론팀 UAV-06, 총괄 작업 지시서 — 사용자 결정 2026-10-08, 드론팀 공유 필요)
    uav_extra = {**tw, "UAV_STATE_DIR": str(run / "uav"), "UAV_HOME": twin_uav_home(opts),
                 "UAV_MOCK_TIME_SCALE": speed,
                 "UAV_RETASK_WHILE_RETURNING": os.environ.get("UAV_RETASK_WHILE_RETURNING", "1")} if tw else {}
    orch_env = {**tw, "ORCH_ENV_MODE": "team_http", "ORCH_DB_PATH": str(run / "orchestrator.sqlite3")} if tw else {}
    # 드론 (2026-10-08): A 인제119 2대(트윈은 차량 집결지에서 이륙) + C 원통119 예비 2대(8002·8003, 기지에서 이륙).
    #   기린(B)은 발화점 18 km 로 드론 항속 반경(약 12.5 km) 밖이라 드론을 두지 않는다.
    # --extra-fleet: 원통에 UGV C-ugv1·2 추가. 원통은 UGV 도로망 밖이라 가장 가까운 도로 노드
    #   495165(광치령로, 원통 남쪽 1.3 km)에서 출발한다.
    extra = opts.get("extra_fleet")
    reserve_uav_env = {k: v for k, v in uav_extra.items() if k != "UAV_HOME"}   # 전진 이륙 지점 말고 소속 기지(C)
    if tw:
        orch_env["ORCH_UAV_ENDPOINTS"] = "C-uav1=http://127.0.0.1:8002,C-uav2=http://127.0.0.1:8003"
    uv = [PY, "-m", "uvicorn"]
    return {
        "env":  {"cwd": ROOT, "port": 8300, "health": "/health",
                 "cmd": uv + ["environment.server:app", "--host", "127.0.0.1", "--port", "8300", "--log-level", "warning"],
                 "env": tw, "desc": "환경 계약 서버 (멘토 패치)"},
        "uav1": {"cwd": ROOT / "uav" / "uav-agent", "port": 8000, "health": "/health",
                 "cmd": uv + ["main:app", "--host", "127.0.0.1", "--port", "8000"],
                 "env": {"UAV_ID": "A-uav1", "UAV_MODE": "mock", **uav_extra}, "desc": "UAV A-uav1 (mock)"},
        "uav2": {"cwd": ROOT / "uav" / "uav-agent", "port": 8001, "health": "/health",
                 "cmd": uv + ["main:app", "--host", "127.0.0.1", "--port", "8001"],
                 "env": {"UAV_ID": "A-uav2", "UAV_MODE": "mock", **uav_extra}, "desc": "UAV A-uav2 (mock)"},
        "cuav1": {"cwd": ROOT / "uav" / "uav-agent", "port": 8002, "health": "/health",
                  "cmd": uv + ["main:app", "--host", "127.0.0.1", "--port", "8002"],
                  "env": {"UAV_ID": "C-uav1", "UAV_MODE": "mock", **reserve_uav_env}, "desc": "예비 UAV C-uav1 원통 (mock)"},
        "cuav2": {"cwd": ROOT / "uav" / "uav-agent", "port": 8003, "health": "/health",
                  "cmd": uv + ["main:app", "--host", "127.0.0.1", "--port", "8003"],
                  "env": {"UAV_ID": "C-uav2", "UAV_MODE": "mock", **reserve_uav_env}, "desc": "예비 UAV C-uav2 원통 (mock)"},
        "ugv":  {"cwd": ROOT, "port": 8100, "health": "/health",
                 "cmd": uv + ["ugv.server:app", "--host", "127.0.0.1", "--port", "8100"],
                 "env": {"UGV_DRIVER": "sim", **({"UGV_STATE_DIR": str(run / "ugv")} if tw else {}),
                         **({"UGV_EXTRA": "C-ugv1:C:495165,C-ugv2:C:495165"} if extra else {}),
                         # 트윈: 차량 배속 = 트윈 시계 배속 (밖에서 UGV_TIME_SCALE 을 주면 그 값을 쓴다)
                         **({"UGV_TIME_SCALE": os.environ.get("UGV_TIME_SCALE", speed)} if tw else {})},
                 "desc": "UGV 서버 (sim)"},
        "orch": {"cwd": ROOT, "port": 8200, "health": "/health",
                 "cmd": [PY, "-m", "orchestrator.api"], "env": orch_env,
                 "desc": "총괄 오케스트레이터" + (" (팀 환경)" if tw else "")},
        "web":  {"cwd": ROOT, "port": 8080, "health": "/",
                 "cmd": uv + ["web.app:app", "--host", "127.0.0.1", "--port", "8080"],
                 "env": web_env, "desc": "웹 관제판"},
        "clock": {"cwd": ROOT, "port": None, "health": None,
                  "cmd": [PY, "tools/twin_clock.py", "--speed", speed], "env": tw,
                  "desc": f"트윈 시계 ({speed}배속, /advance 유일 호출)"},
        "operator": {"cwd": ROOT, "port": None, "health": None,
                     "cmd": [PY, "tools/twin_operator.py"], "env": tw, "desc": "야간 순찰 요청"},
        "dispatch": {"cwd": ROOT, "port": None, "health": None,
                     "cmd": [PY, "tools/twin_ground_dispatch.py"] + DISPATCH_ARGS, "env": tw,
                     "desc": "지상 출동 요청 (불 확인 → 소방차 마을회관 방어)"},
    }


ORDER = ["env", "uav1", "uav2", "cuav1", "cuav2", "ugv", "orch", "web", "clock", "operator", "dispatch"]   # 켜는 순서
TWIN_SET = ORDER                                   # --twin 일 때 전부 (원통 예비 드론 포함)
BASIC = ["env", "uav1", "uav2", "ugv", "orch", "web"]   # 포트가 있는 서버


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
    if d["port"] is None:                       # 포트 없는 프로세스(시계·순찰)는 살아 있는지만 본다
        time.sleep(3)
        return alive(pid)
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
    if d["port"] and responds(d["port"], d["health"], timeout=0.8):
        print(f"  {name:5} ⚠ 포트 {d['port']}을 다른 프로그램이 이미 쓰는 중 — 건너뜀 "
              f"(직접 켜 둔 터미널이 있으면 그 창에서 Ctrl+C)")
        return
    pid = launch(name, d)
    st["servers"][name] = {"pid": pid, "port": d["port"]}
    save_state(st)
    ok = wait_ready(d, pid)
    mark = ("✅ 응답" if d["port"] else "✅ 실행") if ok else "❌ 응답 없음"
    port = f":{d['port']}" if d["port"] else "  -  "
    print(f"  {name:8} {mark}  {port}  {d['desc']}  (PID {pid})"
          + ("" if ok else f"  → logs/servers/{name}.log 확인"))


def cmd_start(a) -> None:
    st = load_state()
    opts = {"uav_url": a.uav_url, "via_orch": a.via_orch, "demo_extinguish": a.demo_extinguish,
            "twin": a.twin, "twin_speed": a.twin_speed, "extra_fleet": a.extra_fleet}
    if a.twin:
        opts["twin_dir"] = str(ROOT / "logs" / "twin" / time.strftime("%Y%m%d_%H%M%S"))
    # 전체 시작(이름 생략)은 이번에 준 설정으로 새로 정하고,
    # 일부만 켤 때(start orch 등)는 옵션을 주지 않으면 지난 설정을 그대로 쓴다.
    if not a.names or any([a.uav_url, a.via_orch, a.demo_extinguish, a.twin, a.extra_fleet]):
        st["opts"] = opts
    defs = server_defs(st["opts"])
    if st["opts"].get("twin") and not a.names:
        names = TWIN_SET
    else:
        names = a.names or (["uav1", "uav2", "ugv", "orch", "web"] if a.all
                            else ["uav1", "orch", "web"] + (["ugv"] if a.ugv else []))
    if a.uav_url:                                   # 원격 UAV 를 쓰면 로컬 UAV 는 띄우지 않는다
        names = [n for n in names if n not in ("uav1", "uav2")]
    print("서버 시작:")
    for n in [x for x in ORDER if x in names]:
        start_one(n, defs, st)
    print("\n관제판: http://127.0.0.1:8080   총괄 판단 화면: http://127.0.0.1:8200/board")
    if st["opts"].get("twin"):
        print("3D 트윈: http://127.0.0.1:8080/inje3d  → '실시간 연결 LIVE'   기록: " + st["opts"]["twin_dir"])
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
    print(f"{'이름':8} {'포트':>5}  {'응답':6} {'PID':>7}  설명")
    for n in ORDER:
        d = defs[n]
        pid = st["servers"].get(n, {}).get("pid")
        if d["port"]:
            on = responds(d["port"], d["health"], timeout=0.8)
        else:
            on = bool(pid and alive(pid))
        who = (str(pid) if pid and alive(pid) else ("직접 실행" if on else "-"))
        print(f"{n:8} {str(d['port'] or '-'):>5}  {'✅' if on else '·':6} {who:>7}  {d['desc']}")
    o = st.get("opts") or {}
    print(f"\n설정: {'트윈 · ' if o.get('twin') else ''}UAV={o.get('uav_url') or '로컬 mock'} · 출동 총괄 경유={'켜짐' if (o.get('via_orch') or o.get('twin')) else '꺼짐'}"
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
    s.add_argument("--twin", action="store_true", help="2019 인제 디지털 트윈 (멘토 패치) 구성으로 전부 켜기")
    s.add_argument("--twin-speed", type=int, default=100, help="트윈 시계 배속 (기본 100, real 드론이면 1)")
    s.add_argument("--extra-fleet", action="store_true",
                   help="원통119(C)에 UGV C-ugv1·2 추가 (--twin 과 함께)")
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
