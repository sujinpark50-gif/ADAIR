"""수행 가능성 판단 — ACCEPT / REJECT / COUNTER (NOTE.md 6절).

판단만 하고 실행하지 않는다. ACCEPT 는 Safety 검증을 거쳐야 실행된다.

reason 코드는 interfaces/schema.py 의 RejectReason 과 같다. UAV 연동 과정에서
아래 코드가 팀 Enum 에 추가됐다:
  FAILSAFE_ACTIVE  PX4 failsafe 작동 중 (드론 고유 상태)
  BUSY             다른 Task 수행 중
  WIND_UNKNOWN     풍속 미제공 — 요청에 wind_ms 가 없을 때
  TARGET_ALTITUDE_UNKNOWN  target 의 alt_m_amsl(지면 고도) 누락 (팀 등록 전)
  MODERATE_WIND    (COUNTER) 풍속 주의
  DEADLINE_TIGHT   (COUNTER) 기한 초과하나 수행 가능
"""
import math
from datetime import datetime, timezone
from typing import Optional

import config

EARTH_R = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def _parse_deadline(s: Optional[str]) -> Optional[datetime]:
    if not s:
        return None
    try:
        return datetime.fromisoformat(s.replace("Z", "+00:00"))
    except ValueError:
        return None


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


def evaluate(state: dict, req, home=None) -> dict:
    """state: drone.get_state() 결과, req: EvaluateRequest, home: 기지 (lat, lon, 지면고도).

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

    pos = state["position"]
    health = state["health"]
    battery_now = state["battery"]["percent"]
    # 풍속: 요청값 우선. PX4 SITL 은 풍속을 발행하지 않으므로 실연동에서는
    # state["wind_ms"] 가 None 이다. 이때는 '모름'이므로 판단을 거절한다 (NOTE.md 6-4).
    wind = req.wind_ms if req.wind_ms is not None else state.get("wind_ms")
    if wind is None:
        return {**base, "verdict": "REJECT", "reason": "WIND_UNKNOWN",
                "detail": "풍속 정보 없음. 요청에 wind_ms 를 포함하거나 "
                          "환경모델에서 제공받아야 한다.",
                "constraints": {}}

    required_alt = flight_alt_amsl(req.target)
    if required_alt is None:
        return {**base, "verdict": "REJECT", "reason": "TARGET_ALTITUDE_UNKNOWN",
                "detail": "target 에 alt_m_amsl(지면 해발고도)이 있어야 "
                          "비행고도를 정할 수 있다.",
                "constraints": {}}

    # ── 1단계: 하드 제약 ──────────────────────────────────
    if health["failsafe"]:
        return {**base, "verdict": "REJECT", "reason": "FAILSAFE_ACTIVE",
                "constraints": {}}

    if not health["gps_ok"] or not health["sensors_ok"]:
        return {**base, "verdict": "REJECT", "reason": "SENSOR_FAILURE",
                "detail": f"gps_ok={health['gps_ok']} sensors_ok={health['sensors_ok']}",
                "constraints": {}}

    if state["link_quality"] == "LOST":
        return {**base, "verdict": "REJECT", "reason": "COMMUNICATION_FAILURE",
                "constraints": {}}

    if state.get("current_task_id"):
        return {**base, "verdict": "REJECT", "reason": "BUSY",
                "detail": f"executing {state['current_task_id']}",
                "constraints": {}}

    if req.observation_type not in config.AVAILABLE_SENSORS:
        return {**base, "verdict": "REJECT", "reason": "REQUIRED_CAPABILITY_UNAVAILABLE",
                "detail": f"{req.observation_type} not in {config.AVAILABLE_SENSORS}",
                "constraints": {}}

    if wind >= config.WIND_LIMIT_MS:
        return {**base, "verdict": "REJECT", "reason": "HIGH_WIND",
                "detail": f"wind {wind} m/s exceeds limit {config.WIND_LIMIT_MS} m/s",
                "constraints": {"wind_ms": wind,
                                "wind_limit_ms": config.WIND_LIMIT_MS}}

    # ── 소요 계산 ────────────────────────────────────────
    dist_m = haversine_m(pos["lat"], pos["lon"], req.target.lat, req.target.lon)

    # 지형: 목표 지점 기준. 경로 최고점은 미반영 (NOTE.md 6-3 단순화)
    climb_m = max(0.0, required_alt - pos["alt_m_amsl"])

    cruise_s = dist_m / config.CRUISE_SPEED_MS
    climb_s = climb_m / config.CLIMB_SPEED_MS
    eta_s = cruise_s + climb_s

    if home is None:
        home = (pos["lat"], pos["lon"], pos["alt_m_amsl"])
    # 복귀: 수평 + 기지 지면까지 하강(상승과 같은 속도로 잡는다)
    back_s = (haversine_m(req.target.lat, req.target.lon, home[0], home[1])
              / config.CRUISE_SPEED_MS
              + max(0.0, required_alt - home[2]) / config.CLIMB_SPEED_MS)

    total_s = eta_s + config.OBSERVE_DURATION_S + back_s  # 가는 길 + 체류 + 복귀
    battery_needed = total_s * config.BATTERY_DRAIN_PCT_S
    battery_after = battery_now - battery_needed
    margin = battery_after

    constraints = {
        "distance_km": round(dist_m / 1000, 2),
        "battery_now_pct": round(battery_now, 1),
        "battery_after_pct": round(battery_after, 1),
        "return_margin_pct": round(margin, 1),
        "wind_ms": wind,
        "flight_alt_m_amsl": round(required_alt, 1),
        "required_climb_m": round(climb_m),
        "eta_sec": round(eta_s),
    }

    # ── 2단계: 에너지 ────────────────────────────────────
    if battery_after < 0:
        return {**base, "verdict": "REJECT", "reason": "LOW_BATTERY",
                "detail": f"need {battery_needed:.1f}% but have {battery_now:.1f}%",
                "constraints": constraints}

    if margin < config.BATTERY_RESERVE_PCT:
        return {**base, "verdict": "COUNTER", "reason": "RETURN_MARGIN_INSUFFICIENT",
                "eta_sec": round(eta_s),
                "counter_offer": {
                    "observe_duration_s": max(10, config.OBSERVE_DURATION_S // 2),
                    "note": "관측 체류시간 단축 시 수행 가능",
                },
                "constraints": constraints}

    # ── 3단계: 시간 ──────────────────────────────────────
    deadline = _parse_deadline(req.deadline)
    if deadline:
        now = datetime.now(timezone.utc)
        available_s = (deadline - now).total_seconds()
        constraints["deadline_slack_sec"] = round(available_s - eta_s)
        if available_s <= 0:
            return {**base, "verdict": "REJECT", "reason": "TIMEOUT",
                    "detail": "deadline already passed",
                    "constraints": constraints}
        if eta_s > available_s:
            arrival = now.timestamp() + eta_s
            return {**base, "verdict": "COUNTER", "reason": "DEADLINE_TIGHT",
                    "eta_sec": round(eta_s),
                    "counter_offer": {
                        "eta_sec": round(eta_s),
                        "arrival": datetime.fromtimestamp(
                            arrival, timezone.utc
                        ).strftime("%Y-%m-%dT%H:%M:%SZ"),
                    },
                    "constraints": constraints}

    # 풍속이 한계 미만이나 COUNTER 기준 이상이면 주의 표시
    if wind >= config.WIND_COUNTER_MS:
        return {**base, "verdict": "COUNTER", "reason": "MODERATE_WIND",
                "eta_sec": round(eta_s),
                "counter_offer": {"note": "풍속 주의. 고도 하향 권장"},
                "constraints": constraints}

    return {**base, "verdict": "ACCEPT", "eta_sec": round(eta_s),
            "constraints": constraints}
