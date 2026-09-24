#!/usr/bin/env python3
"""UAV 데이터 캡처 — PX4 원본(raw)과 UAV Agent 가공본(/state, /evaluate)을 같은 시점에 저장한다.

  raw 1) MAVSDK 4.x Telemetry 전 스트림(33종) 첫 값 + Info(버전·기체 정보)
  raw 2) PX4 내부 uORB battery_status, 배터리 시뮬레이터 파라미터 (docker exec)
  가공  UAV Agent GET /state, POST /evaluate (evaluate 는 판단만 하며 기체를 움직이지 않는다)

실행 (PX4 + UAV Agent real 모드가 떠 있는 상태):
  ~/uav-venv4/bin/python uav/tools/capture_telemetry.py
  ~/uav-venv4/bin/python uav/tools/capture_telemetry.py --agent http://3.37.210.229:8000

MAVSDK 는 기본으로 PX4 의 GCS 링크(14550)에 붙는다. UAV Agent 가 쓰는 14540 에 같이 붙으면
포트 경합으로 Agent 텔레메트리가 튄다(NOTE.md 6-2).

출력: uav/captures/<UTC 시각>/ 아래 raw_mavsdk.json, raw_px4_uorb.txt, processed_agent.json, REPORT.md
보고서만 다시: capture_telemetry.py --from uav/captures/<폴더>   (필드 의미는 field_docs.py)
"""
import argparse
import asyncio
import enum
import json
import math
import os
import subprocess
import urllib.request
from datetime import datetime, timezone

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.info import InfoAsync
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

from field_docs import INFO, PROCESSED_EVALUATE, PROCESSED_STATE, RAW

HERE = os.path.dirname(os.path.abspath(__file__))
PX4_BIN = "/opt/px4-gazebo/bin"
BAT_PARAMS = ["SIM_BAT_DRAIN", "SIM_BAT_MIN_PCT", "BAT1_CAPACITY", "BAT1_N_CELLS", "COM_WIND_MAX"]

# UAV Agent 가 쓰는 raw 필드 → 가공 필드 (drone.py Px4Drone.get_state 기준)
MAPPING = [
    ("position.latitude_deg", "position.lat", "그대로"),
    ("position.longitude_deg", "position.lon", "그대로"),
    ("position.absolute_altitude_m", "position.alt_m_amsl", "그대로"),
    ("position.relative_altitude_m", "position.alt_m_agl", "그대로 (이륙지점 기준 상대고도, 실제 AGL 아님)"),
    ("battery.remaining_percent", "battery.percent", "그대로 (MAVSDK 가 이미 0~100)"),
    ("battery.voltage_v", "battery.voltage", "그대로"),
    ("velocity_ned.north_m_s / east_m_s", "velocity_ms", "hypot(N, E) 소수 2자리 — 수직속도 버림"),
    ("flight_mode", "flight_mode", "enum 이름 문자열"),
    ("armed", "armed", "bool"),
    ("health.is_global_position_ok", "health.gps_ok", "그대로"),
    ("health.is_gyrometer/accelerometer/magnetometer_calibration_ok", "health.sensors_ok", "세 값 AND"),
    ("(없음)", "health.failsafe", "고정 false"),
    ("wind (미발행)", "wind_ms", "고정 null"),
    ("(없음)", "link_quality", "고정 \"OK\""),
    ("(없음)", "current_task_id", "고정 null"),
    ("(없음 — 서버 시각)", "timestamp", "응답 생성 시각 ISO8601 UTC"),
]


def to_jsonable(v, depth=0):
    """MAVSDK 4.x 객체(일반 클래스·IntEnum·리스트)를 JSON 으로 바꾼다."""
    if depth > 6:
        return repr(v)
    if isinstance(v, enum.Enum):
        return v.name
    if isinstance(v, bool) or v is None or isinstance(v, (int, str)):
        return v
    if isinstance(v, float):
        return v if math.isfinite(v) else str(v)   # NaN/inf 는 JSON 표준이 아님
    if isinstance(v, (list, tuple)):
        return [to_jsonable(x, depth + 1) for x in v]
    if isinstance(v, bytes):
        return v.hex()
    out = {}
    for a in dir(v):
        if a.startswith("_") or a in ("from_c_struct", "to_c_struct"):
            continue
        try:
            x = getattr(v, a)
        except Exception as e:  # noqa: BLE001
            out[a] = f"<error {e}>"
            continue
        if callable(x):
            continue
        out[a] = to_jsonable(x, depth + 1)
    return out


async def first(stream, timeout_s):
    gen = stream()
    try:
        return "ok", to_jsonable(await asyncio.wait_for(anext(gen), timeout_s))
    except asyncio.TimeoutError:
        return "timeout", None
    except Exception as e:  # noqa: BLE001
        return "error", repr(e)
    finally:
        await gen.aclose()


USED_STREAMS = ["position", "battery", "velocity_ned", "flight_mode", "armed", "health"]


def now_iso():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-4] + "Z"


async def capture_mavsdk(address, timeout_s, between):
    """Agent 가 쓰는 6종을 먼저 읽고, between()(Agent 호출)을 바로 실행한 뒤 나머지를 읽는다.
    원본과 가공본의 시각 차이를 줄이기 위해서다."""
    sdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    try:
        await sdk.add_any_connection(address)
        system = await sdk.first_autopilot(timeout_s=20)
        if system is None:
            raise SystemExit(f"{address} 에서 PX4 를 찾지 못함")
        tel, info = TelemetryAsync(system), InfoAsync(system)

        streams = {}

        async def grab(name):
            status, value = await first(getattr(tel, f"subscribe_{name}"), timeout_s)
            streams[name] = {"status": status, "captured_at": now_iso(), "value": value}

        for name in USED_STREAMS:
            await grab(name)
        between_result = between()
        rest = sorted(m[len("subscribe_"):] for m in dir(tel) if m.startswith("subscribe_"))
        for name in rest:
            if name not in streams:
                await grab(name)
        streams = dict(sorted(streams.items()))

        info_out = {}
        for name in ("get_version", "get_identification", "get_product", "get_speed_factor"):
            try:
                info_out[name[4:]] = to_jsonable(await getattr(info, name)())
            except Exception as e:  # noqa: BLE001
                info_out[name[4:]] = f"<error {e!r}>"
        try:
            info_out["gps_global_origin"] = to_jsonable(await tel.get_gps_global_origin())
        except Exception as e:  # noqa: BLE001
            info_out["gps_global_origin"] = f"<error {e!r}>"
        return {"streams": streams, "info": info_out}, between_result
    finally:
        sdk.destroy()


def capture_uorb(container):
    def sh(*args):
        r = subprocess.run(["docker", "exec", container, *args], capture_output=True, text=True, timeout=20)
        return (r.stdout + r.stderr).strip()
    parts = ["## uORB battery_status (PX4 내부 원값, remaining 은 0~1)",
             sh(f"{PX4_BIN}/px4-listener", "battery_status", "-n", "1"),
             "", "## 배터리·풍속 관련 파라미터"]
    for p in BAT_PARAMS:
        lines = [ln.strip() for ln in sh(f"{PX4_BIN}/px4-param", "show", p).splitlines() if f" {p} " in ln]
        parts.append(lines[0] if lines else f"{p}: (이 펌웨어에 없음)")
    parts += ["", "## PX4 버전", sh(f"{PX4_BIN}/px4-ver", "all")]
    return "\n".join(parts)


def http(url, body=None):
    req = urllib.request.Request(url, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"},
                                 method="GET" if body is None else "POST")
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read())
    except Exception as e:  # noqa: BLE001
        return {"<error>": repr(e)}


def capture_agent(agent, uav_id):
    evaluate_req = {
        "task_id": "capture-001", "decision_id": "capture-dec-001",
        "target": {"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 761.0},   # 팀 DEM 기준 지면고도
        "observation_type": "THERMAL", "wind_ms": 3.1,
    }
    return {
        "health": http(f"{agent}/health"),
        "state": http(f"{agent}/uav/{uav_id}/state"),
        "evaluate_request": evaluate_req,
        "evaluate_response": http(f"{agent}/uav/{uav_id}/evaluate", evaluate_req),
    }


def dig(d, path):
    for k in path.split("."):
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def raw_value(streams, path):
    if path.startswith("(") or " " in path.split(".")[0]:
        return "—"
    head, _, rest = path.partition(".")
    s = streams.get(head.split(" ")[0], {})
    if s.get("status") != "ok":
        return f"({s.get('status', '없음')})"
    if rest.startswith("is_gyrometer"):
        v = s["value"]
        return ", ".join(str(v.get(f"is_{k}_calibration_ok")) for k in ("gyrometer", "accelerometer", "magnetometer"))
    if "/" in rest:   # velocity_ned.north_m_s / east_m_s
        return ", ".join(str(s["value"].get(k.strip())) for k in rest.split("/"))
    return json.dumps(dig(s["value"], rest) if rest else s["value"], ensure_ascii=False)


def uorb_field(uorb, key):
    for ln in uorb.splitlines():
        ln = ln.strip()
        if ln.startswith(f"{key}:"):
            return ln.split(":", 1)[1].strip()
    return None


def param_value(uorb, name):
    for ln in uorb.splitlines():
        if f" {name} " in ln and ":" in ln:
            return ln.rsplit(":", 1)[1].strip()
    return "?"


def fmt(v):
    s = json.dumps(v, ensure_ascii=False)
    return s if len(s) <= 60 else s[:57] + "…"


def flatten(v, prefix=""):
    """중첩 dict 를 'a.b' 키로 펼친다. 쿼터니언처럼 작은 하위 객체도 펼친다."""
    if not isinstance(v, dict):
        return {prefix: v}
    out = {}
    for k, x in v.items():
        out.update(flatten(x, f"{prefix}.{k}" if prefix else k))
    return out


def md_table(head, rows):
    L = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    L += ["| " + " | ".join(str(c).replace("|", "\\|") for c in r) + " |" for r in rows]
    return L


def report(meta, raw, uorb, agent):
    streams = raw["streams"]
    state = agent.get("state", {})
    ev = agent.get("evaluate_response", {})
    ok = [k for k, v in streams.items() if v["status"] == "ok"]
    ng = [k for k, v in streams.items() if v["status"] != "ok"]
    used = set(USED_STREAMS)
    L = [
        "# UAV 데이터 캡처 — 원본(raw) vs 가공본",
        "",
        "PX4 가 내보내는 원본 데이터 전체와, UAV Agent 가 이를 가공해 팀에 제공하는 응답을 같은 시점에 캡처했다.",
        "",
        *md_table(["항목", "값"], [
            ["캡처 시각 (UTC)", meta["captured_at"]],
            ["원본 캡처 (position)", dig(streams, "position.captured_at")],
            ["가공본 캡처 (/state)", agent.get("captured_at")],
            ["MAVSDK 연결", f"`{meta['mavsdk_address']}` (PX4 GCS 링크)"],
            ["UAV Agent", f"`{meta['agent']}` — mode `{dig(agent, 'health.mode')}`"],
            ["PX4 버전", "{flight_sw_major}.{flight_sw_minor}.{flight_sw_patch} {flight_sw_version_type} "
                        "(git {flight_sw_git_hash})".format(**raw["info"]["version"])
             if isinstance(raw["info"].get("version"), dict) else raw["info"].get("version")],
        ]),
        "",
        "파일: `raw_mavsdk.json` (원본 전체), `raw_px4_uorb.txt` (PX4 내부값·파라미터), "
        "`processed_agent.json` (가공본)",
        "",
        "## 1. 원본 → 가공 매핑 (`GET /state`)",
        "",
        "UAV Agent 가 원본 중 무엇을 어떻게 바꿔 `/state` 로 내보내는지 보여준다.",
        "",
    ]
    rows = []
    for src, dst, how in MAPPING:
        rows.append([f"`{dst}`", PROCESSED_STATE.get(dst, ""), fmt(dig(state, dst)),
                     f"`{src}`", raw_value(streams, src), how])
    L += md_table(["가공 필드", "의미", "가공 값", "원본 (MAVSDK)", "원본 값", "처리"], rows)

    bat = streams.get("battery", {}).get("value") or {}
    L += [
        "",
        "## 2. 배터리 단위 대조 (R05)",
        "",
        *md_table(["단계", "값", "척도"], [
            ["PX4 uORB `battery_status.remaining`", uorb_field(uorb, "remaining"), "0~1"],
            ["MAVSDK `battery.remaining_percent`", bat.get("remaining_percent"), "0~100"],
            ["UAV API `battery.percent`", dig(state, "battery.percent"), "0~100"],
        ]),
        "",
        "MAVSDK 가 ×100 해서 넘기므로 UAV·통합 모두 추가 변환이 필요 없다.",
        "",
        "## 3. 가공본 — `POST /evaluate`",
        "",
        "요청: `" + json.dumps(agent.get("evaluate_request"), ensure_ascii=False) + "`",
        "",
    ]
    L += md_table(["응답 필드", "의미", "값"],
                  [[f"`{k}`", PROCESSED_EVALUATE.get(k, ""), fmt(v)] for k, v in flatten(ev).items()])

    L += [
        "",
        "## 4. 원본 전체 필드 사전 — MAVSDK Telemetry",
        "",
        f"스트림 {len(streams)}종 중 **{len(ok)}종 수신**. ✅ 는 UAV Agent 가 사용하는 스트림이다.",
        "",
    ]
    for name in ok:
        desc, fields = RAW.get(name, ("", {}))
        mark = " ✅" if name in used else ""
        L += [f"### `{name}`{mark} — {desc}", ""]
        v = streams[name]["value"]
        if not isinstance(v, dict):
            L += [f"값: `{fmt(v)}`", ""]
            continue
        flat = flatten(v)
        rows = []
        for k, x in flat.items():
            meaning = fields.get(k) or fields.get(k.split(".")[0], "")
            rows.append([f"`{k}`", meaning, fmt(x)])
        L += md_table(["필드", "의미", "값"], rows) + [""]

    L += ["### 이 링크에서 수신되지 않은 스트림", ""]
    L += md_table(["스트림", "의미", "상태"],
                  [[f"`{n}`", RAW.get(n, ("", {}))[0], streams[n]["status"]] for n in ng])
    L += [
        "",
        "`timeout` 은 캡처 시간(스트림당 수 초) 안에 값이 오지 않았다는 뜻이다. PX4 SITL 이 이 링크(GCS)에",
        "해당 메시지를 보내지 않거나(`imu`, `odometry` 등), 센서가 없거나(`distance_sensor`),",
        "이벤트가 있을 때만 보내는(`status_text`) 경우다. `wind` 는 Agent 링크(14540)에서도 미발행이었다(NOTE.md 6-4 실측).",
        "",
        "### 기체 정보 (MAVSDK Info)",
        "",
    ]
    L += md_table(["항목", "의미", "값"],
                  [[f"`{k}`", INFO.get(k, ""), fmt(v)] for k, v in raw["info"].items()])

    # ── 알려진 이슈 ──
    drain, minpct = param_value(uorb, "SIM_BAT_DRAIN"), param_value(uorb, "SIM_BAT_MIN_PCT")
    L += [
        "",
        "## 5. ⚠️ 알려진 이슈 — SITL 배터리가 50% 에서 멈춤",
        "",
        "**현상**",
        "",
        f"- 캡처 시점 배터리: uORB `{uorb_field(uorb, 'remaining')}` → API `{dig(state, 'battery.percent')}%`",
        f"- PX4 배터리 시뮬레이터 파라미터: `SIM_BAT_DRAIN = {drain}` (초), `SIM_BAT_MIN_PCT = {minpct}` (%)",
        "- 시동 후 비행하면 배터리가 줄어들다가 **`SIM_BAT_MIN_PCT` 에서 멈추고 더 내려가지 않는다.**",
        "  한 번 비행한 기체는 PX4 를 재기동할 때까지 이 값에 머문다.",
        "",
        "**영향**",
        "",
    ]
    if ev.get("reason") == "LOW_BATTERY":
        L.append(f"- 이번 캡처의 evaluate 가 바로 이 경우다: `REJECT / LOW_BATTERY` — {ev.get('detail')}.")
    L += [
        "- 원통119 → 관측 목표(약 9.8 km, 왕복 + 관측)에는 약 57% 가 필요하다고 계산되므로,",
        "  배터리가 50% 에 걸린 기체는 **이 정도 거리 이상의 목표를 계속 `LOW_BATTERY` 로 거절한다.**",
        "  복귀 여유 20% 까지 따지면 50% 기체가 ACCEPT 할 수 있는 목표는 매우 가까운 곳뿐이다.",
        "- 반대로 방전이 50% 에서 멈추므로, 실제 비행이 길어져도 배터리 부족 상황이 재현되지 않는다.",
        "- 즉 real 모드의 배터리 값은 **실제 소모를 반영하지 않으며**, 판단 근거로 쓰기에 부적절하다.",
        "",
        "**대응 (결정 필요)**",
        "",
        "1. 시연 직전에 PX4 를 재기동해 100% 에서 시작 (`px4-stop.sh` → `px4-start*.sh`)",
        "2. 기동 스크립트에서 `SIM_BAT_MIN_PCT` 를 낮춰 실제에 가까운 방전 허용 (예: 5)",
        "3. `SIM_BAT_DRAIN` 을 조정해 방전 속도를 기체 사양에 맞춤 — 배터리 소모율(`BATTERY_DRAIN_PCT_S`)",
        "   실측 보정과 함께 진행",
    ]
    return "\n".join(L) + "\n"


def rebuild(d):
    """저장된 캡처 폴더에서 REPORT.md 만 다시 만든다."""
    raw = json.load(open(f"{d}/raw_mavsdk.json", encoding="utf-8"))
    agent = json.load(open(f"{d}/processed_agent.json", encoding="utf-8"))
    uorb = open(f"{d}/raw_px4_uorb.txt", encoding="utf-8").read()
    meta = raw.pop("meta")
    agent.pop("meta", None)
    with open(f"{d}/REPORT.md", "w", encoding="utf-8") as f:
        f.write(report(meta, raw, uorb, agent))
    print(os.path.abspath(f"{d}/REPORT.md"))


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mavsdk", default="udpin://0.0.0.0:14550", help="PX4 GCS 링크 (Agent 는 14540)")
    ap.add_argument("--agent", default="http://localhost:8000")
    ap.add_argument("--uav-id", default="A-uav1")
    ap.add_argument("--container", default="px4")
    ap.add_argument("--timeout", type=float, default=3.0, help="스트림당 대기 초")
    ap.add_argument("--out", default=os.path.join(HERE, "..", "captures"))
    ap.add_argument("--from", dest="rebuild_dir", help="캡처 없이 이 폴더의 REPORT.md 만 다시 생성")
    a = ap.parse_args()
    if a.rebuild_dir:
        return rebuild(a.rebuild_dir)

    ts = datetime.now(timezone.utc)
    out = os.path.join(a.out, ts.strftime("%Y%m%dT%H%M%SZ"))
    os.makedirs(out, exist_ok=True)
    meta = {"captured_at": ts.strftime("%Y-%m-%dT%H:%M:%SZ"), "mavsdk_address": a.mavsdk, "agent": a.agent}

    def between():
        d = capture_agent(a.agent, a.uav_id)
        d["captured_at"] = now_iso()
        d["uorb"] = capture_uorb(a.container)
        return d

    raw, agent = await capture_mavsdk(a.mavsdk, a.timeout, between)
    uorb = agent.pop("uorb")

    with open(f"{out}/raw_mavsdk.json", "w", encoding="utf-8") as f:
        json.dump({"meta": meta, **raw}, f, ensure_ascii=False, indent=1)
    with open(f"{out}/raw_px4_uorb.txt", "w", encoding="utf-8") as f:
        f.write(uorb + "\n")
    with open(f"{out}/processed_agent.json", "w", encoding="utf-8") as f:
        json.dump({"meta": meta, **agent}, f, ensure_ascii=False, indent=1)
    with open(f"{out}/REPORT.md", "w", encoding="utf-8") as f:
        f.write(report(meta, raw, uorb, agent))
    print(os.path.abspath(out))


if __name__ == "__main__":
    asyncio.run(main())
