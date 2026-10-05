"""수행 가능성 판단 — ACCEPT / REJECT / COUNTER (NOTE.md 6절).

판단만 하고 실행하지 않는다. ACCEPT 는 Safety 검증을 거쳐야 실행된다.

reason 코드는 interfaces/schema.py 의 RejectReason 과 같다. UAV 연동 과정에서
아래 코드가 팀 Enum 에 추가됐다:
  FAILSAFE_ACTIVE  PX4 failsafe 작동 중 (드론 고유 상태)
  BUSY             다른 Task 수행 중
  WIND_UNKNOWN     풍속 미제공 — 요청에 wind_ms 가 없을 때
  TARGET_ALTITUDE_UNKNOWN  target 의 alt_m_amsl(지면 고도) 누락 (팀 등록 전)
  DEADLINE_TIGHT   기한(remaining_time_s) 안에 도착하지 못함 (REJECT)

COUNTER 는 재평가 요청에 그대로 실을 수 있는 필드만 제안한다 (UAV-02):
  RETURN_MARGIN_INSUFFICIENT → {"observe_duration_s": n}  — 그 값이면 복귀 여유를 지킨다
풍속 주의(WIND_COUNTER_MS 이상, 한계 미만)는 바꿀 필드가 없어 COUNTER 가 아니라
ACCEPT + constraints.warnings=["MODERATE_WIND"] 로 알린다.
"""
import math
from typing import Optional

import config
import route as route_mod

EARTH_R = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def measurements_of(req) -> list[str]:
    """요청한 측정 목록 (순서 유지, 중복 제거). observation_type 이 있으면 맨 앞. 비면 이동만 한다."""
    out = []
    for m in ([req.observation_type] if req.observation_type else []) + list(req.measurements or []):
        if m not in out:
            out.append(m)
    return out


def observe_seconds(req) -> int:
    return req.observe_duration_s if req.observe_duration_s is not None else config.OBSERVE_DURATION_S


def flight_times(state_pos: dict, target, home, obs_alt: float, cruise_alt: float) -> tuple[float, float]:
    """(가는 시간, 복귀 시간) 초. 고도는 제자리에서 맞춘 뒤 수평 이동한다 (MockDrone·route 와 같은 방식).

    가는 길: 순항고도로 상승 → 수평 → 관측고도로 하강
    복귀   : 순항고도로 상승 → 수평 → 기지 지면까지 하강 (하강도 상승과 같은 속도로 잡는다)
    """
    out_s = (abs(cruise_alt - state_pos["alt_m_amsl"]) / config.CLIMB_SPEED_MS
             + haversine_m(state_pos["lat"], state_pos["lon"], target.lat, target.lon) / config.CRUISE_SPEED_MS
             + (cruise_alt - obs_alt) / config.CLIMB_SPEED_MS)
    back_s = ((cruise_alt - obs_alt) / config.CLIMB_SPEED_MS
              + haversine_m(target.lat, target.lon, home[0], home[1]) / config.CRUISE_SPEED_MS
              + max(0.0, cruise_alt - home[2]) / config.CLIMB_SPEED_MS)
    return out_s, back_s


def flight_alt_amsl(target) -> Optional[float]:
    """목표 비행고도(AMSL). 지면 고도가 없으면 None — 추측하지 않는다."""
    if target.alt_m_amsl is None:
        return None
    return target.alt_m_amsl + target.target_agl_m + config.AGL_MARGIN_M


def return_margin_now(state: dict, home) -> float:
    """지금 바로 기지로 돌아가면 남을 배터리(%). 비행 중 감시용 (시나리오 5).

    복귀는 하강이라 상승 시간이 없다. 수평 이동만 계산한다.
    """
    pos = state["position"]
    dist_m = haversine_m(pos["lat"], pos["lon"], home[0], home[1])
    need = dist_m / config.CRUISE_SPEED_MS * config.BATTERY_DRAIN_PCT_S
    return state["battery"]["percent"] - need


def evaluate(state: dict, req, home=None) -> tuple[dict, Optional[dict]]:
    """state: drone.get_state() 결과, req: EvaluateRequest, home: 기지 (lat, lon, 지면고도).

    반환: (평가 응답, ACCEPT 면 경로 / 아니면 None). 경로는 main 이 저장해 두고 실행 때 맞춰 본다.

    배터리는 현재 위치 → 목표 → 관측 → 기지 로 계산한다. 기지에 있으면 왕복과 같다.
    복귀 중인 기체를 다시 보낼 때(시나리오 4·7)는 현재 위치가 기지가 아니다.
    home 이 없으면 현재 위치를 기지로 본다.
    """
    uav_id = state["uav_id"]
    base = {
        "task_id": req.task_id,
        "decision_id": req.decision_id,
        "uav_id": uav_id,
    }

    def reject(reason, detail=None, constraints=None):
        out = {**base, "verdict": "REJECT", "reason": reason, "constraints": constraints or {}}
        if detail:
            out["detail"] = detail
        return out, None

    pos = state["position"]
    health = state["health"]
    battery_now = state["battery"]["percent"]
    # 풍속: 요청값 우선. PX4 SITL 은 풍속을 발행하지 않으므로 실연동에서는
    # state["wind_ms"] 가 None 이다. 이때는 '모름'이므로 판단을 거절한다 (NOTE.md 6-4).
    wind = req.wind_ms if req.wind_ms is not None else state.get("wind_ms")
    if wind is None:
        return reject("WIND_UNKNOWN", "풍속 정보 없음. 요청에 wind_ms 를 포함하거나 "
                                      "환경모델에서 제공받아야 한다.")

    required_alt = flight_alt_amsl(req.target)
    if required_alt is None:
        return reject("TARGET_ALTITUDE_UNKNOWN", "target 에 alt_m_amsl(지면 해발고도)이 있어야 "
                                                 "비행고도를 정할 수 있다.")

    # ── 1단계: 하드 제약 ──────────────────────────────────
    if health["failsafe"]:
        return reject("FAILSAFE_ACTIVE")

    if not health["gps_ok"] or not health["sensors_ok"]:
        return reject("SENSOR_FAILURE", f"gps_ok={health['gps_ok']} sensors_ok={health['sensors_ok']}")

    if state["link_quality"] == "LOST":
        return reject("COMMUNICATION_FAILURE")

    if state.get("current_task_id"):
        return reject("BUSY", f"executing {state['current_task_id']}")

    if state.get("returning_task_id") and not config.RETASK_WHILE_RETURNING:
        return reject("BUSY", f"returning after {state['returning_task_id']} (복귀 중 재배정 안 함, UAV-06)")

    measurements = measurements_of(req)
    available = config.available_sensors()
    missing = [m for m in measurements if m not in available]
    if missing:
        return reject("REQUIRED_CAPABILITY_UNAVAILABLE", f"{missing} not in {available}")

    if wind >= config.WIND_LIMIT_MS:
        return reject("HIGH_WIND", f"wind {wind} m/s exceeds limit {config.WIND_LIMIT_MS} m/s",
                      {"wind_ms": wind, "wind_limit_ms": config.WIND_LIMIT_MS})

    # ── 경로·소요 계산 ────────────────────────────────────
    if home is None:
        home = (pos["lat"], pos["lon"], pos["alt_m_amsl"])
    observe_s = observe_seconds(req)
    route = route_mod.plan(uav_id, pos, req.target, home, required_alt, observe_s, measurements)
    cruise_alt = route["cruise_alt_m_amsl"]
    eta_s, back_s = flight_times(pos, req.target, home, required_alt, cruise_alt)
    dist_m = haversine_m(pos["lat"], pos["lon"], req.target.lat, req.target.lon)
    climb_m = max(0.0, cruise_alt - pos["alt_m_amsl"])

    drain = config.BATTERY_DRAIN_PCT_S
    battery_needed = (eta_s + observe_s + back_s) * drain   # 가는 길 + 체류 + 복귀
    battery_after = battery_now - battery_needed
    margin = battery_after

    constraints = {
        "distance_km": round(dist_m / 1000, 2),
        "battery_now_pct": round(battery_now, 1),
        "battery_after_pct": round(battery_after, 1),
        "return_margin_pct": round(margin, 1),
        "wind_ms": wind,
        "flight_alt_m_amsl": round(required_alt, 1),   # 관측고도
        "cruise_alt_m_amsl": cruise_alt,               # 경로 지형을 넘는 순항고도 (UAV-03)
        "terrain_check": route["terrain"]["status"],
        "required_climb_m": round(climb_m),
        "eta_sec": round(eta_s),
        "observe_duration_s": observe_s,
        "measurements": measurements,
    }

    # ── 2단계: 에너지 ────────────────────────────────────
    if margin < config.BATTERY_RESERVE_PCT:
        # 복귀 여유를 지키는 최대 체류시간. 최소 체류 이상이면 그 값을 제안한다 (요청에 그대로 실을 수 있는 필드)
        max_observe = math.floor((battery_now - config.BATTERY_RESERVE_PCT) / drain - eta_s - back_s)
        if config.OBSERVE_DURATION_MIN_S <= max_observe < observe_s:
            constraints["max_observe_duration_s"] = max_observe
            return ({**base, "verdict": "COUNTER", "reason": "RETURN_MARGIN_INSUFFICIENT",
                     "eta_sec": round(eta_s),
                     "counter_offer": {"observe_duration_s": max_observe},
                     "detail": f"관측 체류 {max_observe}s 이하이면 복귀 후 "
                               f"{config.BATTERY_RESERVE_PCT:.0f}% 이상 남는다",
                     "constraints": constraints}, None)
        if battery_after < 0:
            return reject("LOW_BATTERY", f"need {battery_needed:.1f}% but have {battery_now:.1f}%", constraints)
        return reject("RETURN_MARGIN_INSUFFICIENT",
                      f"최소 체류 {config.OBSERVE_DURATION_MIN_S}s 로도 복귀 여유 "
                      f"{config.BATTERY_RESERVE_PCT:.0f}% 를 지킬 수 없다", constraints)

    # ── 3단계: 시간 (시뮬레이션 초, UAV-04) ─────────────────
    if req.remaining_time_s is not None:
        constraints["deadline_slack_sec"] = round(req.remaining_time_s - eta_s)
        if req.remaining_time_s <= 0:
            return reject("TIMEOUT", "deadline already passed", constraints)
        if eta_s > req.remaining_time_s:
            # 바꿀 수 있는 필드가 없어 COUNTER 가 아니다. 늦게라도 갈지는 총괄이 기한을 늘려 다시 묻는다
            return reject("DEADLINE_TIGHT", f"eta {eta_s:.0f}s > remaining {req.remaining_time_s:.0f}s",
                          constraints)

    # 풍속이 한계 미만이나 주의 기준 이상이면 수행은 하되 경고를 단다
    if wind >= config.WIND_COUNTER_MS:
        constraints["warnings"] = ["MODERATE_WIND"]

    return ({**base, "verdict": "ACCEPT", "eta_sec": round(eta_s), "constraints": constraints,
             "route_id": route["route_id"], "route_version": route["version"],
             "route_hash": route["route_hash"], "route": _route_summary(route)}, route)


def _route_summary(route: dict) -> dict:
    return {k: route[k] for k in ("cruise_alt_m_amsl", "obs_alt_m_amsl", "waypoints", "terrain",
                                  "terrain_clearance_m", "profile_assumption")}
