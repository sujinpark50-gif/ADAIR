# -*- coding: utf-8 -*-
"""총괄 팀별 요청서의 드론 항목 (UAV-02~10) — 실제 UAV Agent 프로세스(UAV_MODE=mock)를 띄워 HTTP 로 확인한다.

실행 (저장소 루트):  python -m pytest tests/test_uav_requests.py -q
UAV-01 · J3 (실행 키·재시작)은 tests/test_exec_idempotency.py 에 있다.
"""

import httpx
import pytest

from tests.test_exec_idempotency import UAV, UAV_DIR, Server, TARGET, wait

# 원통 기지에서 약 3 km. 복귀가 몇 초(실제 시간) 걸려 복귀 중 상태를 볼 수 있다
NEAR = {"lat": 38.1000, "lon": 128.2150, "alt_m_amsl": 400.0}


def start(tmp_path, **env):
    return Server(["main:app"], UAV_DIR, {"UAV_ID": UAV, "UAV_MODE": "mock",
                                          "UAV_STATE_DIR": str(tmp_path), **env}).start()


@pytest.fixture
def uav(tmp_path):
    s = start(tmp_path)
    yield s
    if s.p.poll() is None:
        s.crash()


def ev(s, **over):
    b = {"task_id": "T-1", "decision_id": "D-1", "target": dict(TARGET), "observation_type": "THERMAL",
         "wind_ms": 3.0}
    b.update(over)
    return httpx.post(f"{s.url}/uav/{UAV}/evaluate", json=b, timeout=10).json()


def ex(s, tid="ATT-1", **over):
    b = {"task_id": tid, "decision_id": "D-1", "target": dict(TARGET), "observation_type": "THERMAL"}
    b.update(over)
    return httpx.post(f"{s.url}/uav/{UAV}/execute", json=b, timeout=10)


def task(s, tid="ATT-1"):
    return httpx.get(f"{s.url}/uav/{UAV}/task/{tid}", timeout=5).json()


def abort(s, tid="ATT-1", **body):
    return httpx.post(f"{s.url}/uav/{UAV}/task/{tid}/abort", json=body or None, timeout=10)


def mock_set(s, **params):
    httpx.post(f"{s.url}/mock/set", params=params, timeout=5)


def landed(d):
    return (d.get("progress") or {}).get("phase") == "DONE"


# ---------------------------------------------------------------- 기능 목록
def test_capabilities(uav):
    c = httpx.get(f"{uav.url}/uav/{UAV}/capabilities").json()
    assert c["counter_fields"] == ["observe_duration_s"] and c["deadline_field"] == "remaining_time_s"
    assert c["abort"] is True and "WEATHER" in c["sensors"] and c["camera"]["status"] == "NONE"


# ---------------------------------------------------------------- UAV-02
def test_counter_offers_carriable_observe_duration(uav):
    base = ev(uav, target=dict(NEAR))
    assert base["verdict"] == "ACCEPT"
    need = 95.0 - base["constraints"]["return_margin_pct"]          # 기본 체류 60 s 기준 소모량
    mock_set(uav, battery_pct=round(need + 15.0 - 1.0, 2))           # 복귀 여유 14% → 체류를 줄이면 가능
    c = ev(uav, target=dict(NEAR))
    assert c["verdict"] == "COUNTER" and c["reason"] == "RETURN_MARGIN_INSUFFICIENT"
    assert set(c["counter_offer"]) == {"observe_duration_s"}         # 요청에 실을 수 있는 필드만
    n = c["counter_offer"]["observe_duration_s"]
    assert 10 <= n < 60
    again = ev(uav, target=dict(NEAR), observe_duration_s=n)         # 제안을 그대로 실어 재평가
    assert again["verdict"] == "ACCEPT" and again["constraints"]["observe_duration_s"] == n
    assert again["constraints"]["return_margin_pct"] >= 15.0


def test_counter_impossible_even_with_min_observe_is_reject(uav):
    mock_set(uav, battery_pct=5)
    r = ev(uav, target=dict(NEAR))
    assert r["verdict"] == "REJECT" and r["reason"] in ("LOW_BATTERY", "RETURN_MARGIN_INSUFFICIENT")


def test_moderate_wind_is_accept_with_warning(uav):
    r = ev(uav, wind_ms=8.5)
    assert r["verdict"] == "ACCEPT" and r["constraints"]["warnings"] == ["MODERATE_WIND"]
    assert r.get("counter_offer") is None


# ---------------------------------------------------------------- UAV-04
def test_deadline_is_simulation_seconds(uav):
    ok = ev(uav, remaining_time_s=10_000)
    assert ok["verdict"] == "ACCEPT" and ok["constraints"]["deadline_slack_sec"] > 0
    tight = ev(uav, remaining_time_s=1)
    assert tight["verdict"] == "REJECT" and tight["reason"] == "DEADLINE_TIGHT"
    assert ev(uav, remaining_time_s=0)["reason"] == "TIMEOUT"
    # 이전의 ISO 시각 deadline 은 더 이상 보지 않는다 (2019 시각과 실제 시계를 비교하던 오류)
    assert ev(uav, deadline="2019-04-04T10:00:00Z")["verdict"] == "ACCEPT"


# ---------------------------------------------------------------- UAV-05 · 07 · 10
def test_multi_measurement_visit_and_split_report(uav):
    r = ev(uav, measurements=["THERMAL", "WEATHER"], observation_type=None)
    assert r["verdict"] == "ACCEPT" and r["constraints"]["measurements"] == ["THERMAL", "WEATHER"]
    assert ex(uav, measurements=["WEATHER"]).status_code == 200      # observation_type THERMAL + WEATHER
    d = wait(uav, "ATT-1", lambda d: d["status"] == "COMPLETED")
    assert d["mission_result"] == "OBSERVED" and d["physical_state"] in ("RETURNING", "LANDED")
    obs = d["observation"]
    kinds = [m["sensor_type"] for m in obs["measurements"]]
    assert kinds == ["THERMAL", "WEATHER"]
    thermal, weather = obs["measurements"]
    assert thermal["value_status"] == "SIMULATED" and thermal["basis"] == "MOCK_FIXED_PLACEHOLDER"
    assert weather["values"]["wind_ms"] is not None and weather["value_status"] == "SIMULATED"
    assert obs["sensor_type"] == "THERMAL" and obs["values"] == thermal["values"]   # 기존 형식 호환
    end = wait(uav, "ATT-1", landed)
    assert end["physical_state"] == "LANDED" and end["observation"] == obs           # 관측은 바뀌지 않는다


def test_weather_only_visit_is_movement_only(uav):
    r = ev(uav, observation_type=None)
    assert r["verdict"] == "ACCEPT" and r["constraints"]["measurements"] == []
    assert ex(uav, observation_type=None).status_code == 200
    d = wait(uav, "ATT-1", lambda d: d["status"] == "COMPLETED")
    assert d["observation"]["position"]["lat"] is not None and d["observation"]["measurements"] == []


def test_unknown_measurement_rejected(uav):
    r = ev(uav, measurements=["LIDAR"])
    assert r["verdict"] == "REJECT" and r["reason"] == "REQUIRED_CAPABILITY_UNAVAILABLE"
    assert ex(uav, measurements=["LIDAR"]).status_code == 400
    assert ex(uav).status_code == 200                                 # 거절은 키를 쓰지 않는다


def test_real_style_thermal_has_no_fabricated_values(tmp_path):
    s = start(tmp_path, UAV_PLACEHOLDER_THERMAL="0")
    try:
        ex(s)
        d = wait(s, "ATT-1", lambda d: d["status"] == "COMPLETED")
        assert d["observation"]["values"] == {} and d["observation"]["value_status"] == "NO_DATA"
    finally:
        s.crash()


# ---------------------------------------------------------------- UAV-03
def test_route_from_evaluate_is_executed(uav):
    r = ev(uav)
    assert r["route_version"] and len(r["route_hash"]) == 64 and r["route_id"].startswith("RT-")
    assert ev(uav)["route_hash"] == r["route_hash"]                   # 같은 조건이면 같은 경로
    resp = ex(uav, route_id=r["route_id"], route_hash=r["route_hash"])
    assert resp.status_code == 200 and resp.json()["route_hash"] == r["route_hash"]
    assert task(uav)["route_hash"] == r["route_hash"]


def test_route_mismatch_refused_without_consuming_key(uav):
    r = ev(uav)
    bad = ex(uav, route_hash="0" * 64)
    assert bad.status_code == 409 and bad.json()["detail"]["reason"] == "ROUTE_NOT_FOUND"
    other = ex(uav, route_id=r["route_id"], target={**TARGET, "lat": 38.126})
    assert other.status_code == 409 and other.json()["detail"]["reason"] == "ROUTE_MISMATCH"
    longer = ex(uav, route_id=r["route_id"], observe_duration_s=30)
    assert longer.status_code == 409 and longer.json()["detail"]["reason"] == "ROUTE_MISMATCH"
    assert ex(uav, route_id=r["route_id"], route_hash=r["route_hash"]).status_code == 200


def test_route_survives_restart(uav):
    r = ev(uav)
    uav.restart()
    assert ex(uav, route_id=r["route_id"], route_hash=r["route_hash"]).status_code == 200


def test_route_required_mode(tmp_path):
    s = start(tmp_path, UAV_REQUIRE_ROUTE_HASH="1")
    try:
        no = ex(s)
        assert no.status_code == 409 and no.json()["detail"]["reason"] == "ROUTE_REQUIRED"
        r = ev(s)
        assert ex(s, route_id=r["route_id"], route_hash=r["route_hash"]).status_code == 200
    finally:
        s.crash()


def test_route_cruise_clears_dem_terrain(uav):
    """능선 목표: 경로 중간 지형이 관측고도보다 높아 순항고도를 올린다 (DEM 이 있을 때)"""
    caps = httpx.get(f"{uav.url}/uav/{UAV}/capabilities").json()
    if not caps["route"]["dem"]:
        pytest.skip("DEM(rasterio) 없음")
    mock_set(uav, battery_pct=100)
    r = ev(uav, target={"lat": 38.0425, "lon": 128.2541, "alt_m_amsl": 804.0})
    c = r["constraints"]
    assert c["cruise_alt_m_amsl"] > c["flight_alt_m_amsl"] and c["terrain_check"] in ("OK", "PARTIAL")


# ---------------------------------------------------------------- UAV-09
def test_abort_before_observation(uav):
    ex(uav, observe_duration_s=600)                                   # 체류 600 s = 실제 약 6 s
    wait(uav, "ATT-1", lambda d: (d.get("progress") or {}).get("phase") == "OBSERVING")
    r = abort(uav, reason="CANCELLED_BY_ORCH")
    assert r.status_code == 200
    d = r.json()
    assert d["status"] == "FAILED" and d["reason"] == "ABORTED" and d["observation"] is None
    assert d["mission_result"] == "NOT_OBSERVED" and d["physical_state"] in ("RETURNING", "LANDED")
    assert d["abort"]["result"] == "ABORTED_BEFORE_OBSERVATION" and d["abort"]["reason"] == "CANCELLED_BY_ORCH"
    end = wait(uav, "ATT-1", landed)
    assert end["status"] == "FAILED" and end["physical_state"] == "LANDED"
    assert abort(uav).json()["abort"] == d["abort"]                   # 다시 보내도 처음 결과
    st = httpx.get(f"{uav.url}/uav/{UAV}/state").json()
    assert st["current_task_id"] is None and st["returning_task_id"] is None


def test_abort_after_observation_and_unknown(uav):
    ex(uav)
    wait(uav, "ATT-1", landed)
    d = abort(uav).json()
    assert d["abort"]["result"] == "ALREADY_FINISHED" and d["status"] == "COMPLETED"
    assert abort(uav, tid="NOPE").status_code == 404


# ---------------------------------------------------------------- UAV-06
def test_no_retask_while_returning_by_default(tmp_path):
    s = start(tmp_path)                                               # 기본값 = 착륙까지 거절 (총괄 방침)
    try:
        ex(s, target=dict(NEAR), observe_duration_s=10)
        wait(s, "ATT-1", lambda d: d["status"] == "COMPLETED")
        st = httpx.get(f"{s.url}/uav/{UAV}/state").json()
        assert st["returning_task_id"] == "ATT-1" and st["current_task_id"] is None
        r = ev(s, task_id="T-2")
        assert r["verdict"] == "REJECT" and r["reason"] == "BUSY"
        refused = ex(s, tid="ATT-2")
        assert refused.status_code == 409 and refused.json()["detail"]["reason"] == "BUSY"
        wait(s, "ATT-1", landed)
        assert ex(s, tid="ATT-2").status_code == 200                  # 착륙 뒤에는 받는다
    finally:
        s.crash()


def test_retask_while_returning_opt_in(tmp_path):
    """UAV_RETASK_WHILE_RETURNING=1: 예전 동작 — 복귀 중 새 임무를 받으면 이전 Task 는 RETASKED"""
    s = start(tmp_path, UAV_RETASK_WHILE_RETURNING="1")
    try:
        ex(s, target=dict(NEAR), observe_duration_s=10)
        wait(s, "ATT-1", lambda d: d["status"] == "COMPLETED")
        assert ev(s, task_id="T-2")["verdict"] == "ACCEPT"
        assert ex(s, tid="ATT-2").status_code == 200
        prev = wait(s, "ATT-1", lambda d: (d.get("progress") or {}).get("phase") == "RETASKED")
        assert prev["status"] == "COMPLETED" and prev["physical_state"] == "RETASKED"
    finally:
        s.crash()
