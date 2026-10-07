# -*- coding: utf-8 -*-
"""관측 계획 (사용자 결정 2026-10-07, docs/common/총괄_관측계획_개편_계획서_완성본.md).

신고 → 계획(LLM 또는 규칙 대체)이 예상 구역과 자원을 고름 → 고른 자원으로만 배정 → 드론 테두리 추적 / UGV 지상 관측
→ 인지 지도 갱신 → 다시 계획. 정답(진짜 불)은 계획 입력에 들어가지 않는다.
"""
import json
import math

import pytest

from orchestrator import config, observation
from orchestrator.engine import Orchestrator
from orchestrator.env_adapter import FixtureEnv
from orchestrator.knowledge import EnvStationFeed, InjectedAnalysis
from orchestrator.ledger import Ledger
from orchestrator.llm import LlmPlanner
from orchestrator.models import Target, Task
from orchestrator.observation_planner import MapIndex, rule_plan, validate
from orchestrator.resources import UavClient, UgvClient

from .fakes import FakeUav, FakeUgv, normal_flight
from .test_llm import FakeOpenAI

LAT0, LON0, SIZE, N = 38.05, 128.25, 90.0, 30
DLAT = SIZE / 111_320.0
DLON = SIZE / (111_320.0 * math.cos(math.radians(LAT0)))
REPORT_CELL = "12_12"
# 진짜 불: 5×5 덩어리(10~14) + 동쪽으로 뻗은 혀(15~17, 12행)
BURNING = {f"{c}_{r}" for c in range(10, 15) for r in range(10, 15)} | {f"{c}_12" for c in (15, 16, 17)}


def cell(c, r, state="UNBURNED"):
    return {"cell_id": f"{c}_{r}", "lat": LAT0 - r * DLAT, "lon": LON0 + c * DLON, "cell_size_m": SIZE,
            "ground_amsl_m": 600.0, "fire_state": state}


def grid(burning=BURNING):
    fire = [cell(c, r, "BURNING") for c in range(N) for r in range(N) if f"{c}_{r}" in burning]
    rest = [cell(c, r) for c in range(N) for r in range(N) if f"{c}_{r}" not in burning]
    return fire, rest


@pytest.fixture
def obs_planner(monkeypatch):
    monkeypatch.setattr(config, "OBS_PLANNER_ENABLED", True)
    monkeypatch.setattr(config, "OBS_INITIAL_NEAREST_UAV", False)     # 첫 출동은 test_initial_* 에서 따로 켠다
    caps = dict(config.SIMULATED_CAPABILITIES)
    caps["UGV"] = tuple(caps["UGV"]) + ("THERMAL",)
    monkeypatch.setattr(config, "SIMULATED_CAPABILITIES", caps)


def _ugv_arrives_at_target(ugv):
    def script(rid):
        body = next(b for m, p, b in reversed(ugv.calls) if p.endswith("/execute"))
        t = body["target"]
        return [{"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
                {"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
                 "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1",
                                 "position": {"lat": t["lat"], "lon": t["lon"]}},
                 "_unit": {"state": "READY", "current_task_id": None}}]
    ugv.default_script = script


@pytest.fixture
def pw(tmp_path, obs_planner):
    """관측 계획 시험 세계: 30×30 칸 지도, 신고 120초, UAV 2 + UGV 1"""
    fire, rest = grid()
    env = FixtureEnv(fire_cells=fire, risk_cells=rest, wind_dir_deg=270.0,
                     reports=[{"cell_id": REPORT_CELL, "sim_time_s": 120.0, "source": "119_CALL"}])
    uav = FakeUav(positions={"A-uav1": (LAT0 + 0.02, LON0), "A-uav2": (LAT0 + 0.02, LON0 + 0.01)})
    uav.default_script = normal_flight()
    ugv = FakeUgv()
    _ugv_arrives_at_target(ugv)
    ledger = Ledger(str(tmp_path / "state.sqlite3"))
    orch = Orchestrator(ledger, env, UavClient(uav.endpoints(), uav.client()), UgvClient("http://ugv", ugv.client()),
                        analysis=InjectedAnalysis(), weather_feeds=[EnvStationFeed(env, LAT0, LON0)])
    return {"env": env, "uav": uav, "ugv": ugv, "ledger": ledger, "orch": orch}


def _events(lg, kind):
    return [e for e in lg.events() if e["event_type"] == kind]


def _to_report_time(env):
    env.advance(2)                    # 0 → 120 초


def _poll(orch, n=6):
    for _ in range(n):
        orch.poll()


# ---------------------------------------------------------------------------
# 신고·시작 조건
# ---------------------------------------------------------------------------

def test_no_plan_before_report_and_report_creates_no_task(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    env.advance(1)                                                      # 60 초: 아직 신고 전
    assert orch.obs_plan_request() is None
    _to_report_time(env)
    sync = orch.sync()
    assert sync["observation_plan_wanted"] and sync["created"] == []    # 신고가 바로 출동 Task 를 만들지 않음
    assert lg.list_tasks() == []
    assert _events(lg, "INITIAL_RECON")[-1]["result"] == "OBSERVATION_PLAN_REQUESTED"


def test_rule_fallback_assigns_each_resource_its_own_block_and_pins_it(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    out = orch.obs_plan_cycle()
    assert out[0]["source"] == "RULE_FALLBACK"                          # LLM 미연결
    rows = out[0]["assignments"]
    assert sorted(r["resource_id"] for r in rows) == ["A-uav1", "A-uav2", "A-ugv1"]
    assert len({r["block_id"] for r in rows}) == 3                      # 서로 다른 구역
    assert all(r["dispatch"] == "STARTED" for r in rows)
    for r in rows:
        t = lg.get_task(r["task_id"])
        assert t.kind == "OBSERVE" and t.assigned_resource_id == r["resource_id"]
        filt = [e for e in lg.events(t.task_id) if e["event_type"] == "CANDIDATES_FILTERED"][-1]["detail"]
        assert [c[0] for c in filt["candidates"]] == [r["resource_id"]]   # 고른 자원만 평가
    assert orch.obs_plan_request() is None                              # 가용 자원 없음 → 다시 계획 안 함


def test_plan_input_does_not_change_when_only_truth_changes(pw):
    orch, env = pw["orch"], pw["env"]
    _to_report_time(env)
    h1 = orch.obs_plan_request()["inp"]
    fire, rest = grid(burning={f"{c}_{r}" for c in range(0, 5) for r in range(0, 5)})   # 진짜 불을 통째로 바꿈
    env.fire_cells, env.risk_cells = fire, rest
    h2 = orch.obs_plan_request()["inp"]
    assert h1["input_hash"] == h2["input_hash"]
    text = json.dumps(h2, ensure_ascii=False)
    assert "fire_state" not in text and "risk_score" not in text


def test_plan_stops_after_demo_end(pw, monkeypatch):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    monkeypatch.setattr(config, "OBS_DEMO_END_SIM_S", 150.0)
    env.advance(3)                                                      # 180 초 > 150
    assert orch.obs_plan_request() is None
    assert _events(lg, "OBS_PLAN_STOPPED")[0]["result"] == "DEMO_TIME_END"


# ---------------------------------------------------------------------------
# LLM 계획
# ---------------------------------------------------------------------------

def _llm_plan(pick):
    """pick(inp) → assignments. 입력 해시를 그대로 돌려준다."""
    def reply(kw):
        inp = json.loads(kw["messages"][1]["content"])
        return {"input_hash": inp["input_hash"], "predicted_blocks": [b["block_id"] for b in inp["blocks"]][:2],
                "prediction_rationale": "풍하 쪽 미관측 구역에 불이 이어졌을 가능성", "assignments": pick(inp)}
    return reply


def _far_blocks(inp):
    """일부러 규칙과 다르게: 신고에서 가장 먼 선택 가능 구역들"""
    return sorted(inp["blocks"],
                  key=lambda b: (-b["dist_to_known_fire_m"], b["block_id"]))


def test_llm_choice_of_block_and_resource_is_executed(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]

    def pick(inp):
        far = _far_blocks(inp)
        res = sorted(r["resource_id"] for r in inp["resources_available"])
        return [{"resource_id": rid, "block_id": far[i]["block_id"], "purpose": "PREDICTED_SPREAD",
                 "evidence_refs": [far[i]["block_id"]], "rationale": "먼 미관측 구역 확인"} for i, rid in enumerate(res)]
    fake = FakeOpenAI(_llm_plan(pick))
    orch.llm = LlmPlanner(client=fake, timeout_s=30, max_calls=5)
    _to_report_time(env)
    out = orch.obs_plan_cycle()[0]
    assert out["source"] == "LLM"
    want = {a["resource_id"]: a["block_id"] for a in _events(lg, "OBS_PLAN")[-1]["detail"]["plan"]["assignments"]}
    assert {r["resource_id"]: r["block_id"] for r in out["assignments"]} == want
    for r in out["assignments"]:
        t = lg.get_task(r["task_id"])
        assert MapIndex.block_id(t.target.cell_id) == r["block_id"]       # 코드가 구역 가운데 칸으로 목표를 정함
    hyp = lg.knowledge("HYPOTHESIS")[-1]["body"]
    assert hyp["predicted_blocks"] and hyp["kind"] == "CURRENT_FIRE_HYPOTHESIS"
    assert lg.knowledge("FIRE") and all(k["body"]["status"] != "CONFIRMED" for k in lg.knowledge("FIRE"))  # 가설 ≠ 사실


def test_invalid_llm_output_is_repaired_once(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    calls = []

    def reply(kw):
        calls.append(kw)
        inp = json.loads(kw["messages"][1]["content"])
        if len(calls) == 1:                               # 없는 구역 + 자원 중복
            return {"input_hash": inp["input_hash"], "predicted_blocks": [], "prediction_rationale": "x",
                    "assignments": [{"resource_id": "A-uav1", "block_id": "B99_99", "purpose": "RECHECK",
                                     "evidence_refs": [], "rationale": "x"}]}
        return _llm_plan(lambda i: [{"resource_id": r["resource_id"], "block_id": _far_blocks(i)[n]["block_id"],
                                     "purpose": "BOUNDARY_CHECK", "evidence_refs": [], "rationale": "수정"}
                                    for n, r in enumerate(i["resources_available"])])(kw)
    orch.llm = LlmPlanner(client=FakeOpenAI(reply), timeout_s=30, max_calls=5)
    _to_report_time(env)
    out = orch.obs_plan_cycle()[0]
    assert out["source"] == "LLM" and len(calls) == 2
    assert "BLOCK_NOT_SELECTABLE:B99_99" in calls[1]["messages"][-1]["content"]    # 오류 목록을 주고 다시 요청
    rec = _events(lg, "OBS_PLAN")[-1]["detail"]["llm"]
    assert rec["repaired"] is True and rec["attempts"][0]["status"] == "INVALID"


def test_llm_wrong_twice_falls_back_to_rules(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    orch.llm = LlmPlanner(client=FakeOpenAI(lambda kw: "불이 동쪽으로 갑니다"), timeout_s=30, max_calls=5)
    _to_report_time(env)
    out = orch.obs_plan_cycle()[0]
    assert out["source"] == "RULE_FALLBACK"
    ev = _events(lg, "OBS_PLAN")[-1]["detail"]
    assert ev["llm"]["status"] == "INVALID" and ev["llm"]["violations"] == ["NOT_JSON"]
    assert orch.llm.calls == 2


def test_validator_rejects_invented_ids_and_busy_blocks(pw):
    orch, env = pw["orch"], pw["env"]
    _to_report_time(env)
    inp = orch.obs_plan_request()["inp"]
    good = rule_plan(inp)
    assert validate(good, inp) == []
    bad = {**good, "assignments": [dict(good["assignments"][0], resource_id="Z-uav9")] + good["assignments"][1:]}
    assert any(v.startswith("RESOURCE_NOT_AVAILABLE") for v in validate(bad, inp))
    two = {**good, "assignments": [good["assignments"][0], dict(good["assignments"][1],
                                                                block_id=good["assignments"][0]["block_id"])]}
    assert any(v.startswith("BLOCK_DUPLICATE") for v in validate(two, inp))
    assert "INPUT_HASH_MISMATCH" in validate({**good, "input_hash": "x"}, inp)


def test_rejected_resource_is_not_silently_replaced_and_replanned(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    pw["uav"].verdicts["A-uav1"] = {"verdict": "REJECT", "reason": "LOW_BATTERY", "eta_sec": None}
    _to_report_time(env)
    outs = orch.obs_plan_cycle()
    first = outs[0]
    row = next(r for r in first["assignments"] if r["resource_id"] == "A-uav1")
    t = lg.get_task(row["task_id"])
    assert t.purpose_status == "CANCELLED" and t.hold_reason.startswith("PLANNED_RESOURCE_REJECTED")
    assert not lg.list_attempts(t.task_id)                              # 다른 드론으로 바꿔 보내지 않음
    assert first["replan"] and len(outs) >= 2
    second = outs[1]
    assert all(not (r["resource_id"] == "A-uav1" and r["block_id"] == row["block_id"]) for r in second["assignments"])
    assert _events(lg, "OBS_PLAN")[1]["detail"]["recent_rejections"][0]["resource_id"] == "A-uav1"


def test_plan_discarded_when_input_changes_during_call(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    req = orch.obs_plan_request()
    pw["uav"].state["A-uav2"]["current_task_id"] = "SOMETHING-ELSE"     # 호출 도중 자원 상태가 바뀜
    out = orch.obs_plan_apply(req, {"status": "DISABLED", "attempts": []})
    assert out["status"] == "STALE" and lg.list_tasks() == []
    assert _events(lg, "OBS_PLAN_DISCARDED")


# ---------------------------------------------------------------------------
# 모의 센서: 드론 테두리 추적
# ---------------------------------------------------------------------------

def _sweep(target_cell, burning=BURNING):
    fire, rest = grid(burning)
    env = FixtureEnv(fire_cells=fire, risk_cells=rest)
    c = next(x for x in fire + rest if x["cell_id"] == target_cell)
    rep = next(x for x in fire + rest if x["cell_id"] == REPORT_CELL)
    task = Task(task_id="T", incident_id="I", kind="OBSERVE",
                target=Target(lat=c["lat"], lon=c["lon"], ground_amsl_m=600.0, cell_id=target_cell))
    return observation.simulate_edge_sweep(snapshot=env.read(), map_cells=env.map_cells(), task=task,
                                           attempt_id="ATT", resource_id="A-uav1",
                                           position={"lat": c["lat"], "lon": c["lon"], "alt_m_amsl": 690.0},
                                           report_point={"lat": rep["lat"], "lon": rep["lon"]})


def test_edge_sweep_follows_edge_away_from_report():
    o = _sweep("14_12")                                  # 덩어리 동쪽 가장자리 → 혀를 따라 동쪽(신고에서 먼 쪽)
    fp = o["footprint"]
    assert fp["mode"] == "EDGE_FOLLOW" and fp["length_m"] == 140.0 and fp["swath_m"] == 52.0
    assert [p["cell_id"] for p in o["fire_points"]] == ["14_12", "15_12", "16_12"]
    assert all(set(p) == {"lat", "lon", "cell_id"} for p in o["fire_points"])   # 좌표만 (같이 주는 정보 없음)
    assert o["sensor_status"] == "TEST_ONLY" and o["source"] == "SIMULATED"
    assert "13_12" not in o["covered_cells"] + o["partial_cells"]          # 지나가지 않은 칸은 보지 않음
    assert "17_12" not in o["covered_cells"] + o["partial_cells"]          # 140 m 밖
    assert o["target_covered"] and o["result"] == "DETECTED"


def test_edge_sweep_without_edge_flies_toward_report():
    o = _sweep("20_12")                                  # 가장 가까운 테두리(17_12)가 270 m 밖
    assert o["footprint"]["mode"] == "STRAIGHT_TO_REPORT"
    seen = o["covered_cells"] + o["partial_cells"]
    assert {"20_12", "19_12", "18_12"} <= set(seen) and "21_12" not in seen   # 신고 지점(서쪽) 쪽으로만
    assert o["result"] == "NOT_DETECTED" and o["fire_points"] == []


# ---------------------------------------------------------------------------
# 한 바퀴: 계획 → 관측 → 인지 지도 → 다시 계획
# ---------------------------------------------------------------------------

def test_observation_updates_belief_and_triggers_new_plan(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    first = orch.obs_plan_cycle()[0]
    h1 = _events(lg, "OBS_PLAN")[-1]["detail"]["input_hash"]
    _poll(orch)
    obs = [e["detail"] for e in _events(lg, "OBSERVATION") if (e["detail"] or {}).get("sensor_type") == "THERMAL"]
    assert {o["resource_id"] for o in obs} == {"A-uav1", "A-uav2", "A-ugv1"}
    profiles = {o["resource_id"]: o["sensor_profile_id"] for o in obs}
    assert profiles["A-uav1"] == "ASSUMED_EDGE_SWEEP_V1" and profiles["A-ugv1"] == "ASSUMED_GROUND_VIEW_V1"
    states = orch.kb.fire_states(env.simulation_time_s)
    assert any(v["status"] == "CONFIRMED" for v in states.values())        # 관측으로 불 확인
    assert not _events(lg, "ENV_SENSE_PLANNED")                             # 따로 측정 출동을 만들지 않음
    for r in first["assignments"]:
        assert lg.get_task(r["task_id"]).purpose_status == "COMPLETED"
    nxt = orch.obs_plan_cycle()
    assert nxt and _events(lg, "OBS_PLAN")[-1]["detail"]["input_hash"] != h1   # 새 관측 → 새 계획


def test_observation_plan_view_shows_layers_without_truth(pw):
    orch, env = pw["orch"], pw["env"]
    _to_report_time(env)
    orch.obs_plan_cycle()
    _poll(orch)
    v = orch.observation_plan_view()
    assert v["enabled"] and v["last_plan"]["source"] == "RULE_FALLBACK"
    states = {b["state"] for b in v["belief"]}
    assert "REPORTED" in states or "BURNING" in states
    assert "fire_state" not in json.dumps(v["belief"])


def test_unselectable_blocks_are_removed_before_llm(pw):
    """고를 수 없는 구역은 LLM 입력의 blocks 에서 미리 빠진다 (사용자 결정 2026-10-07)"""
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    pw["uav"].verdicts["A-uav1"] = {"verdict": "REJECT", "reason": "LOW_BATTERY", "eta_sec": None}
    _to_report_time(env)
    first = orch.obs_plan_cycle(max_rounds=1)[0]
    rejected = next(r for r in first["assignments"] if r["resource_id"] == "A-uav1")
    going = {r["block_id"] for r in first["assignments"] if r["dispatch"] == "STARTED"}
    inp = orch._obs_build()["inp"]
    ids = {b["block_id"] for b in inp["blocks"]}
    assert not (ids & going)                                             # 다른 자원이 보러 가는 구역은 후보 아님
    ctx = {b["block_id"]: b for b in inp["context_blocks"]}
    assert all(ctx[g]["in_progress_by"] for g in going)                  # 참고용으로만 보인다
    assert all("selectable" not in b and "in_progress_by" not in b for b in inp["blocks"])
    blk = next((b for b in inp["blocks"] if b["block_id"] == rejected["block_id"]), None)
    assert blk is None or "A-uav1" not in blk["allowed_resources"]       # 방금 거절한 조합은 미리 뺀다
    assert "recent_rejections" not in inp


# ---------------------------------------------------------------------------
# 평가: 테두리 재현율 (사용자 결정 2026-10-07)
# ---------------------------------------------------------------------------

def test_truth_edge_definition():
    from orchestrator.map_evaluation import truth_edge
    fire, _ = grid()
    edge = truth_edge(fire)
    assert "12_12" not in edge                                   # 덩어리 안쪽은 테두리 아님
    assert {"10_10", "14_14", "17_12", "15_12"} <= edge
    assert len(edge) == 16 + 3                                   # 5×5 둘레 16칸 + 혀 3칸


def test_edge_recall_counts_only_confirmed_and_is_recorded(pw):
    from orchestrator.map_evaluation import truth_edge
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    orch.sync()
    before = orch.evaluate_map()
    assert before["truth_edge_cells"] == 19 and before["edge_cells_found"] == 0   # 신고(REPORTED)는 세지 않음
    assert before["edge_recall_pct"] == 0.0
    orch.obs_plan_cycle()
    _poll(orch)
    rec = [e["detail"] for e in _events(lg, "MAP_EVALUATION")]
    assert rec and rec[-1]["used_for_planning"] is False
    after = orch.evaluate_map()
    known = {c for c, f in orch.kb.fire_states(env.simulation_time_s).items() if f["status"] == "CONFIRMED"}
    assert after["edge_cells_found"] == len(known & truth_edge(env.fire_cells))
    assert "edge_recall" not in json.dumps(orch._obs_build()["inp"])          # 평가 결과는 계획 입력에 없음


def test_edge_recall_not_applicable_without_fire(tmp_path):
    from orchestrator.map_evaluation import edge_recall
    _, rest = grid(burning=set())
    snap = FixtureEnv(fire_cells=[], risk_cells=rest).read()
    assert edge_recall(snap, {})["edge_recall_pct"] is None


def test_known_edge_and_frontier_use_belief_only():
    """총괄이 아는 테두리: 불 확인 칸 중 옆이 불 확인·탄 곳이 아닌 칸 / 그 바로 바깥의 미관측 칸 (정답 미사용)"""
    from orchestrator.observation_planner import belief_layers, known_edge
    fire, rest = grid()
    idx = MapIndex(fire + rest)
    belief = belief_layers({"12_12": {"status": "CONFIRMED"}, "11_12": {"status": "CONFIRMED"},
                            "13_12": {"status": "OBSERVED_CLEAR"}})
    edge, frontier = known_edge(idx, belief)
    assert edge == {"11_12", "12_12"}
    assert "13_12" not in frontier and "11_12" not in frontier          # 본 칸은 미관측 바깥이 아님
    assert {"10_11", "10_12", "10_13", "13_11", "13_13", "12_11"} <= frontier
    assert "15_12" not in frontier                                      # 정답상 불이어도 이웃이 아니면 아님


def test_plan_input_carries_edge_fields(pw):
    orch, env = pw["orch"], pw["env"]
    _to_report_time(env)
    orch.obs_plan_cycle()
    _poll(orch)
    inp = orch._obs_build()["inp"]
    assert inp["fire_summary"]["known_edge_cells"] > 0
    assert all({"known_edge_cells", "unobserved_next_to_fire"} <= set(b) for b in inp["blocks"])
    assert any(b["unobserved_next_to_fire"] for b in inp["blocks"])


def test_field_weather_replaces_station_in_plan_input(pw, monkeypatch):
    """드론·UGV 현장 측정이 유효(기본 35분)하면 계획 입력의 기상을 대체한다 (사용자 결정 2026-10-07)"""
    orch, env = pw["orch"], pw["env"]
    monkeypatch.setattr(config, "FIELD_WEATHER_MAX_AGE_S", {k: 2100.0 for k in config.WEATHER_ITEMS})
    _to_report_time(env)
    w0 = orch._obs_build()["inp"]["weather"]["items"]
    assert w0["wind_ms"]["basis"] == "NEAREST_STATION"
    orch.obs_plan_cycle()
    _poll(orch)                                                  # 불 발견 → 같은 방문에서 기상 측정
    w1 = orch._obs_build()["inp"]["weather"]["items"]
    assert w1["wind_ms"]["basis"] == "FIELD_LATEST_VALID" and w1["wind_ms"]["cell_id"]
    env.advance(36)                                              # 35분 넘게 지나면 다시 관측소 값
    w2 = orch._obs_build()["inp"]["weather"]["items"]
    assert w2["wind_ms"]["basis"] == "NEAREST_STATION"


# ---------------------------------------------------------------------------
# 첫 출동: 가장 가까운 드론을 LLM 없이 바로 → 그 관측이 들어온 뒤 LLM 계획 (사용자 결정 2026-10-07)
# ---------------------------------------------------------------------------

def test_initial_nearest_uav_goes_first_then_llm_plans_with_its_observation(pw, monkeypatch):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    monkeypatch.setattr(config, "OBS_INITIAL_NEAREST_UAV", True)
    seen = []

    def pick(inp):
        seen.append(inp)
        return [{"resource_id": r["resource_id"], "block_id": inp["blocks"][n]["block_id"], "purpose": "BOUNDARY_CHECK",
                 "evidence_refs": [], "rationale": "첫 관측 뒤 배정"} for n, r in enumerate(inp["resources_available"])]
    fake = FakeOpenAI(_llm_plan(pick))
    orch.llm = LlmPlanner(client=fake, timeout_s=30, max_calls=5)
    _to_report_time(env)
    assert orch.obs_plan_cycle() == []                                   # LLM 을 부르지 않고 첫 드론만 보냄
    first = _events(lg, "OBS_INITIAL_DISPATCH")[0]
    rep = cell(12, 12)
    nearest = min(("A-uav1", "A-uav2"), key=lambda r: math.hypot(pw["uav"].state[r]["lat"] - rep["lat"],
                                                                 (pw["uav"].state[r]["lon"] - rep["lon"])
                                                                 * math.cos(math.radians(LAT0))))
    assert nearest == "A-uav2"
    assert first["result"] == "STARTED" and first["resource_id"] == nearest   # 신고 칸에 더 가까운 드론
    t = lg.get_task(first["task_id"])
    assert t.target.cell_id == REPORT_CELL and t.kind == "OBSERVE"
    assert fake.requests == []
    assert orch.obs_plan_cycle() == [] and fake.requests == []           # 관측 전에는 계속 기다림
    _poll(orch)                                                          # 첫 드론 관측 도착
    out = orch.obs_plan_cycle()
    assert out and out[0]["source"] == "LLM"
    inp = seen[0]
    assert inp["fire_summary"]["burning_cells"] > 0                      # 첫 관측의 불 좌표가 LLM 입력에 들어감
    assert len(_events(lg, "OBS_INITIAL_DISPATCH")) == 1                 # 첫 출동은 한 번만


def test_initial_dispatch_rejected_does_not_block_planning(pw, monkeypatch):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    monkeypatch.setattr(config, "OBS_INITIAL_NEAREST_UAV", True)
    for rid in ("A-uav1", "A-uav2"):
        pw["uav"].verdicts[rid] = {"verdict": "REJECT", "reason": "HIGH_WIND", "eta_sec": None}
    _to_report_time(env)
    out = orch.obs_plan_cycle()
    assert _events(lg, "OBS_INITIAL_DISPATCH")[0]["result"] == "HOLD"     # 드론 모두 거절 → 첫 출동 보류
    assert out and out[0]["source"] == "RULE_FALLBACK"                    # 기다리지 않고 바로 계획으로 넘어감
