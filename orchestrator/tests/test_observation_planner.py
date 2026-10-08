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
from orchestrator.models import Requirements, Target, Task
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

def _seen(kw):
    """가짜 LLM 이 받은 짧은 입력(llm_view)을 시험에서 쓰기 쉬운 이름으로 펼친다"""
    v = json.loads(kw["messages"][1]["content"])
    return {"input_hash": v["input_hash"], "fire_summary": v["fire"],
            "blocks": [{"block_id": b["id"], "dist_to_known_fire_m": b.get("d", 0)} for b in v["blocks"]],
            "resources_available": [{"resource_id": r[0], "resource_type": r[1]} for r in v["res"]], "raw": v}


def _llm_plan(pick):
    """pick(inp) → assignments. 입력 해시를 그대로 돌려준다."""
    def reply(kw):
        inp = _seen(kw)
        return {"input_hash": inp["input_hash"], "predicted_blocks": [b["block_id"] for b in inp["blocks"]][:2],
                "prediction_rationale": "풍하 쪽 미관측 구역에 불이 이어졌을 가능성", "assignments": pick(inp),
                "request_reserve_uavs": False, "reserve_reason": ""}
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
        return [{"resource_id": rid, "block_ids": [far[i]["block_id"]], "purpose": "PREDICTED_SPREAD",
                 "evidence_refs": [far[i]["block_id"]], "rationale": "먼 미관측 구역 확인"} for i, rid in enumerate(res)]
    fake = FakeOpenAI(_llm_plan(pick))
    orch.llm = LlmPlanner(client=fake, timeout_s=30, max_calls=5)
    _to_report_time(env)
    out = orch.obs_plan_cycle()[0]
    assert out["source"] == "LLM"
    want = {a["resource_id"]: a["block_ids"][0] for a in _events(lg, "OBS_PLAN")[-1]["detail"]["plan"]["assignments"]}
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
        inp = _seen(kw)
        if len(calls) == 1:                               # 없는 구역 + 자원 중복
            return {"input_hash": inp["input_hash"], "predicted_blocks": [], "prediction_rationale": "x",
                    "assignments": [{"resource_id": "A-uav1", "block_ids": ["B99_99"], "purpose": "RECHECK",
                                     "evidence_refs": [], "rationale": "x"}],
                    "request_reserve_uavs": False, "reserve_reason": ""}
        return _llm_plan(lambda i: [{"resource_id": r["resource_id"], "block_ids": [_far_blocks(i)[n]["block_id"]],
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
                                                                block_ids=good["assignments"][0]["block_ids"])]}
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


def _ok_plan(inp):
    return {"status": "OK", "attempts": [], "plan": rule_plan(inp)}


def test_input_change_during_call_keeps_still_valid_assignments(pw):
    """호출 동안 입력이 바뀌면 버리지 않고 지금 상태로 배정마다 다시 검증한다 (계획서 §7)"""
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    req = orch.obs_plan_request()
    pw["uav"].state["A-uav2"]["current_task_id"] = "SOMETHING-ELSE"     # 호출 도중 A-uav2 가 다른 일을 맡음
    out = orch.obs_plan_apply(req, _ok_plan(req["inp"]))
    assert out["status"] == "APPLIED" and out["source"] == "LLM"
    assert sorted(r["resource_id"] for r in out["assignments"]) == ["A-uav1", "A-ugv1"]
    d = _events(lg, "OBS_PLAN")[-1]["detail"]
    assert d["input_changed_during_call"] is True
    assert [(x["resource_id"], x["dropped_reason"]) for x in d["dropped_assignments"]] == [
        ("A-uav2", "RESOURCE_NO_LONGER_AVAILABLE")]
    assert not [t for t in lg.list_tasks() if t.assigned_resource_id == "A-uav2"]


def test_plan_discarded_only_when_no_assignment_is_valid_now(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    req = orch.obs_plan_request()
    for rid in ("A-uav1", "A-uav2"):
        pw["uav"].state[rid]["current_task_id"] = "SOMETHING-ELSE"
    pw["ugv"].units["A-ugv1"]["current_task_id"] = "SOMETHING-ELSE"
    out = orch.obs_plan_apply(req, _ok_plan(req["inp"]))
    assert out["status"] == "STALE" and lg.list_tasks() == []
    assert _events(lg, "OBS_PLAN_DISCARDED")[-1]["reason"] == "NO_ASSIGNMENT_VALID_NOW"


def test_resource_freed_during_call_gets_planned_next(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    pw["uav"].state["A-uav2"]["current_task_id"] = "SOMETHING-ELSE"     # 호출 시작 때는 사용 중
    req = orch.obs_plan_request()
    assert "A-uav2" not in [r["resource_id"] for r in req["inp"]["resources_available"]]
    pw["uav"].state["A-uav2"]["current_task_id"] = None                 # 호출 도중 돌아옴
    out = orch.obs_plan_apply(req, _ok_plan(req["inp"]))
    assert out["replan"] is True                                         # 남은 자원은 다음 계획에서
    assert orch.obs_plan_cycle() == []                                   # 같은 환경 단계에서는 다시 계획하지 않음
    env.advance(1)                                                       # 다음 단계에 모아서
    nxt = orch.obs_plan_cycle()
    assert any(r["resource_id"] == "A-uav2" for o in nxt for r in o["assignments"])


# ---------------------------------------------------------------------------
# 모의 센서: 드론 테두리 추적
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def _sweep_20s(monkeypatch, request):
    """테두리 추적 기하 시험은 20초(140 m) 기준으로 쓴다. 기본값(60초)은 test_edge_sweep_default_duration_is_60s"""
    if request.node.name != "test_edge_sweep_default_duration_is_60s":
        monkeypatch.setattr(config, "EDGE_SWEEP_PROFILE", {**config.EDGE_SWEEP_PROFILE, "duration_s": 20.0})


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


def test_edge_sweep_without_any_reference_holds_and_never_heads_to_report():
    """테두리·아는 불·꺼진 자리·앞 예상 지점이 모두 없으면 제자리 (신고 지점 쪽으로 가지 않는다, 2026-10-07)"""
    o = _sweep("20_12")                                  # 가장 가까운 테두리(17_12)가 270 m 밖
    assert o["footprint"]["mode"] == "HOLD_AT_ARRIVAL"
    seen = set(o["covered_cells"] + o["partial_cells"])
    assert "20_12" in seen and "18_12" not in seen
    assert o["result"] == "NOT_DETECTED" and o["fire_points"] == []


def test_edge_sweep_default_duration_is_60s():
    assert config.EDGE_SWEEP_PROFILE["duration_s"] == 60.0
    assert config.EDGE_SWEEP_PROFILE["speed_ms"] * config.EDGE_SWEEP_PROFILE["duration_s"] == 420.0


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
    env.advance(1)                                                       # 한 단계 안에서는 한 번만 계획
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


def test_plan_input_carries_edge_fields(pw, monkeypatch):
    monkeypatch.setattr(config, "OBS_MAX_CANDIDATE_BLOCKS", 0)            # 상한과 무관한 시험 (전체 후보)
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
        return [{"resource_id": r["resource_id"], "block_ids": [inp["blocks"][n]["block_id"]], "purpose": "BOUNDARY_CHECK",
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
    assert inp["raw"]["progress"]["basis"] in ("IGNITION_TO_BURNING", "PAST_FIRE_TO_BURNING")   # 첫 비행으로 진행 방향
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


def test_terrain_fields_uphill_road_fuel(tmp_path, monkeypatch):
    """오르막·도로·연료는 정적 지도 정보라 계획 입력에 넣는다 (정답 아님). 지도에 없으면 넣지 않는다"""
    monkeypatch.setattr(config, "OBS_MAX_CANDIDATE_BLOCKS", 0)            # 상한과 무관한 시험 (전체 후보)
    from orchestrator.observation_planner import build_input
    fire, rest = grid()
    cells = []
    for c in fire + rest:
        col, row = map(int, c["cell_id"].split("_"))
        c = {k: v for k, v in c.items() if k != "fire_state"}
        c["ground_amsl_m"] = 600.0 + 10.0 * col                       # 동쪽으로 갈수록 오르막
        if col <= 14:
            c["is_road"], c["fuel_amount"] = (row == 9), (0.0 if row == 9 else 0.6)
        cells.append(c)
    idx = MapIndex(cells)
    inp = build_input(idx=idx, fire_states={"12_12": {"status": "CONFIRMED"}}, reports=[], weather={},
                      available=[{"resource_id": "A-uav1", "resource_type": "UAV", "lat": LAT0, "lon": LON0}],
                      busy=[], rejections=[], sim_time_s=0.0, run_id="R")
    rows = {b["block_id"]: b for b in inp["blocks"] + inp["context_blocks"]}
    assert rows["B5_4"]["elev_above_known_fire_m"] > 0 > rows["B2_4"]["elev_above_known_fire_m"]
    assert rows["B4_3"]["road_cells"] == 3 and rows["B4_3"]["mean_fuel"] == 0.4   # 9행 도로 3칸
    assert "road_cells" not in rows["B6_4"] and "mean_fuel" not in rows["B6_4"]   # 지도에 없으면 모름


def test_prompt_mentions_terrain():
    from orchestrator.observation_planner import SYSTEM
    assert "오르막" in SYSTEM and "도로" in SYSTEM and "연료" in SYSTEM


# ---------------------------------------------------------------------------
# 다 타고 꺼짐(추정)과 테두리를 못 찾았을 때의 비행 (사용자 결정 2026-10-07)
# ---------------------------------------------------------------------------

def test_burning_becomes_presumed_burned_after_burn_out():
    from orchestrator.observation_planner import belief_layers, burn_out_s
    assert burn_out_s(2100.0) == 6300.0 and burn_out_s(None) is None      # 3단계 × 단계 길이 (트윈 1시간 45분)
    fs = {"1_1": {"status": "CONFIRMED", "sim_time_s": 0.0}, "2_2": {"status": "CONFIRMED", "sim_time_s": 5000.0},
          "3_3": {"status": "CONFIRMED_BURNED", "sim_time_s": 0.0}}
    lay = belief_layers(fs, now_s=6400.0, burn_out=6300.0)
    assert lay["1_1"]["state"] == "PRESUMED_BURNED" and lay["1_1"]["basis"] == "BURN_OUT_RULE"
    assert lay["2_2"]["state"] == "BURNING"                                  # 다시 본 지 얼마 안 됨
    assert lay["3_3"]["state"] == "BURNED"                                   # 관측으로 확인한 탄 곳은 그대로
    assert belief_layers(fs, now_s=6400.0, burn_out=None)["1_1"]["state"] == "BURNING"   # 단계 길이 모르면 추정 안 함


def _sweep2(target_cell, fallback=None, trail=None, burning=frozenset()):
    fire, rest = grid(set(burning))
    env = FixtureEnv(fire_cells=fire, risk_cells=rest)
    c = next(x for x in fire + rest if x["cell_id"] == target_cell)
    rep = cell(12, 12)
    task = Task(task_id="T", incident_id="I", kind="OBSERVE",
                target=Target(lat=c["lat"], lon=c["lon"], ground_amsl_m=600.0, cell_id=target_cell))
    pt = lambda cid: {"lat": cell(*map(int, cid.split("_")))["lat"], "lon": cell(*map(int, cid.split("_")))["lon"],
                      "cell_id": cid}
    return observation.simulate_edge_sweep(
        snapshot=env.read(), map_cells=env.map_cells(), task=task, attempt_id="A", resource_id="A-uav1",
        position={"lat": c["lat"], "lon": c["lon"], "alt_m_amsl": 690.0},
        report_point={"lat": rep["lat"], "lon": rep["lon"]},
        fallback_point=pt(fallback) if fallback else None, burned_trail=[pt(x) for x in trail or []])


def test_no_edge_flies_toward_nearest_known_fire_not_report():
    o = _sweep2("20_12", fallback="20_16")                                   # 아는 불은 남쪽, 신고는 서쪽
    assert o["footprint"]["mode"] == "STRAIGHT_TO_KNOWN_FIRE"
    seen = set(o["covered_cells"] + o["partial_cells"])
    assert {"20_13", "20_14"} <= seen and "19_12" not in seen


def test_no_known_fire_follows_presumed_burned_trail():
    o = _sweep2("20_12", trail=["20_13", "21_14", "22_15"])
    assert o["footprint"]["mode"] == "FOLLOW_PRESUMED_BURNED"
    seen = set(o["covered_cells"] + o["partial_cells"])
    assert {"20_13", "21_14"} <= seen and "19_12" not in seen
    assert o["burned_trail"] == ["20_13", "21_14", "22_15"]


def test_burned_trail_starts_at_latest_and_skips_observed(pw, monkeypatch):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    orch.sync()                                                              # run 활성화 + 신고 반영
    for cid, t in (("5_5", 0.0), ("6_5", 60.0), ("7_5", 100.0), ("7_6", 30.0)):
        lg.add_knowledge("FIRE", f"T:{cid}", cid, t, "TEST", {"cell_id": cid, "status": "CONFIRMED", "observation_id": f"O{cid}"})
    lg.add_knowledge("FIRE", "T:8_5", "8_5", 50.0, "TEST", {"cell_id": "8_5", "status": "CONFIRMED_BURNED", "observation_id": "O8"})
    monkeypatch.setattr(env, "tick_s", 5.0)                                  # 꺼짐 추정 15초 → 120초 시점엔 모두 추정
    trail = [p["cell_id"] for p in orch._burned_trail(env.simulation_time_s)]
    assert trail[0] == "7_5"                                                 # 가장 마지막에 타던 곳에서 시작
    assert "8_5" not in trail                                                # 관측으로 확인한 탄 곳은 이미 본 곳
    assert trail == ["7_5", "6_5", "7_6"]                                    # 더 최근까지 타던 이웃 칸 순, 끊기면 멈춤
    assert orch._nearest_known_fire({"lat": LAT0, "lon": LON0}, env.simulation_time_s) is None


def test_planning_continues_when_all_known_fire_is_presumed_burned():
    """아는 불이 모두 '다 타고 꺼짐(추정)'이 돼도 그 주변 후보가 남아 계획이 멈추지 않는다"""
    from orchestrator.observation_planner import build_input
    fire, rest = grid()
    idx = MapIndex(fire + rest)
    inp = build_input(idx=idx, fire_states={"12_12": {"status": "CONFIRMED", "sim_time_s": 0.0}}, reports=[],
                      weather={}, available=[{"resource_id": "A-uav1", "resource_type": "UAV", "lat": LAT0, "lon": LON0}],
                      busy=[], rejections=[], sim_time_s=10_000.0, run_id="R", burn_out=6300.0)
    assert inp["fire_summary"]["presumed_burned_out_cells"] == 1 and inp["fire_summary"]["burning_cells"] == 0
    assert inp["blocks"]                                                     # 후보가 남아 있다


def test_fire_progress_from_burned_to_burning():
    """관측으로 본 불의 이동 방향: 지난 불(탄 곳·꺼짐 추정) 중심 → 지금 불타는 중 중심 (정답 미사용)"""
    from orchestrator.observation_planner import belief_layers, fire_progress
    fire, rest = grid()
    idx = MapIndex(fire + rest)
    lay = belief_layers({"10_12": {"status": "CONFIRMED_BURNED", "sim_time_s": 0.0},
                         "11_12": {"status": "CONFIRMED", "sim_time_s": 0.0},          # 오래돼 꺼짐 추정
                         "15_12": {"status": "CONFIRMED", "sim_time_s": 9000.0}}, now_s=10_000.0, burn_out=6300.0)
    p = fire_progress(idx, lay)
    assert p["basis"] == "PAST_FIRE_TO_BURNING" and 80 <= p["moved_toward_deg"] <= 100     # 동쪽으로 이동
    assert p["moved_m"] > 300 and p["newest_burning_blocks"][0]["block_id"] == "B5_4"


def test_fire_progress_without_past_uses_observation_times():
    from orchestrator.observation_planner import belief_layers, fire_progress
    fire, rest = grid()
    idx = MapIndex(fire + rest)
    lay = belief_layers({"12_10": {"status": "CONFIRMED", "sim_time_s": 100.0},
                         "12_14": {"status": "CONFIRMED", "sim_time_s": 900.0}})
    p = fire_progress(idx, lay)
    assert p["basis"] == "EARLIEST_TO_LATEST_BURNING" and 170 <= p["moved_toward_deg"] <= 190   # 남쪽
    assert fire_progress(idx, belief_layers({"12_10": {"status": "CONFIRMED", "sim_time_s": 100.0}})) is None



# ---------------------------------------------------------------------------
# 예비 드론 (B 기지 3·4번): 불이 B 쪽으로 오면 투입 (사용자 결정 2026-10-07, ±45°)
# ---------------------------------------------------------------------------

@pytest.fixture
def pw4(tmp_path, obs_planner, monkeypatch):
    """B 기지를 시험 지도 동쪽(오른쪽)에 둔 세계. UAV 4대 (B-uav1·2 는 예비)"""
    import config as team_config
    monkeypatch.setattr(team_config, "BASE_B_LAT", LAT0 - 12 * DLAT, raising=False)
    monkeypatch.setattr(team_config, "BASE_B_LON", LON0 + 40 * DLON, raising=False)
    fire, rest = grid()
    env = FixtureEnv(fire_cells=fire, risk_cells=rest, wind_dir_deg=270.0,
                     reports=[{"cell_id": REPORT_CELL, "sim_time_s": 120.0, "source": "119_CALL"}])
    ids = ("A-uav1", "A-uav2", "B-uav1", "B-uav2")
    uav = FakeUav(ids=ids, positions={"A-uav1": (LAT0 + 0.02, LON0), "A-uav2": (LAT0 + 0.02, LON0 + 0.01),
                                      "B-uav1": (LAT0 - 0.01, LON0 + 0.05), "B-uav2": (LAT0 - 0.01, LON0 + 0.05)})
    uav.default_script = normal_flight()
    ugv = FakeUgv()
    _ugv_arrives_at_target(ugv)
    lg = Ledger(str(tmp_path / "s.sqlite3"))
    orch = Orchestrator(lg, env, UavClient(uav.endpoints(), uav.client()), UgvClient("http://ugv", ugv.client()),
                        analysis=InjectedAnalysis(), weather_feeds=[EnvStationFeed(env, LAT0, LON0)])
    return {"env": env, "uav": uav, "ledger": lg, "orch": orch}


def _known(lg, cid, status, t):
    lg.add_knowledge("FIRE", f"T:{cid}:{t}", cid, t, "TEST", {"cell_id": cid, "status": status, "observation_id": f"O{cid}"})


def test_reserve_uavs_stand_by_until_fire_moves_toward_base(pw4):
    orch, env, lg = pw4["orch"], pw4["env"], pw4["ledger"]
    _to_report_time(env)
    orch.sync()
    b = orch._obs_build()
    assert {"B-uav1", "B-uav2"}.isdisjoint(r["resource_id"] for r in b["inp"]["resources_available"])
    assert b["unavailable"]["B-uav1"] == "RESERVE_STANDBY" and b["inp"]["reserve_uavs"]["active"] is False
    _known(lg, "12_12", "CONFIRMED_BURNED", 100.0)                       # 지난 불 (서쪽)
    _known(lg, "16_12", "CONFIRMED", 120.0)                              # 지금 불 (동쪽 = B 쪽으로 이동)
    b = orch._obs_build()
    ev = [e for e in lg.events() if e["event_type"] == "RESERVE_UAVS_ACTIVATED"]
    assert ev and ev[0]["result"] == "PHYSICAL_FIRE_PROGRESS" and ev[0]["detail"]["angle_diff_deg"] <= 45
    assert {"B-uav1", "B-uav2"} <= {r["resource_id"] for r in b["inp"]["resources_available"]}
    orch._obs_build()
    assert len([e for e in lg.events() if e["event_type"] == "RESERVE_UAVS_ACTIVATED"]) == 1   # 한 번만


def test_reserve_not_activated_when_fire_moves_away(pw4):
    orch, env, lg = pw4["orch"], pw4["env"], pw4["ledger"]
    _to_report_time(env)
    orch.sync()
    _known(lg, "16_12", "CONFIRMED_BURNED", 100.0)
    _known(lg, "12_12", "CONFIRMED", 120.0)                              # 서쪽으로 이동 (B 반대)
    b = orch._obs_build()
    assert b["inp"]["reserve_uavs"]["active"] is False
    assert not [e for e in lg.events() if e["event_type"] == "RESERVE_UAVS_ACTIVATED"]


def test_llm_can_request_reserve_uavs(pw4):
    orch, env, lg = pw4["orch"], pw4["env"], pw4["ledger"]

    def reply(kw):
        inp = _seen(kw)
        return {"input_hash": inp["input_hash"], "predicted_blocks": [], "prediction_rationale": "B 쪽으로 번질 위험",
                "assignments": [{"resource_id": r["resource_id"], "block_ids": [inp["blocks"][n]["block_id"]],
                                 "purpose": "BOUNDARY_CHECK", "evidence_refs": [], "rationale": "확인"}
                                for n, r in enumerate(inp["resources_available"])],
                "request_reserve_uavs": True, "reserve_reason": "불이 동쪽 오르막으로 번져 B 기지 쪽으로 다가옴"}
    orch.llm = LlmPlanner(client=FakeOpenAI(reply), timeout_s=30, max_calls=6)
    _to_report_time(env)
    orch.obs_plan_cycle()
    ev = [e for e in lg.events() if e["event_type"] == "RESERVE_UAVS_ACTIVATED"]
    assert ev and ev[0]["result"] == "LLM_REQUEST" and "B 기지" in ev[0]["detail"]["reason"]
    assigned = {a["resource_id"] for e in lg.events() if e["event_type"] == "OBS_PLAN_EXECUTED"
                for a in e["detail"]["assignments"]}
    assert assigned & {"B-uav1", "B-uav2"}                                # 투입 뒤 다음 계획에서 배정됨


def test_reserve_request_needs_reason():
    from orchestrator.observation_planner import validate
    inp = {"input_hash": "h", "blocks": [], "context_blocks": [], "reports": [], "resources_available": [],
           "resources_busy": [], "weather": {}}
    out = {"input_hash": "h", "predicted_blocks": [], "prediction_rationale": "x", "assignments": [],
           "request_reserve_uavs": True, "reserve_reason": ""}
    assert "RESERVE_REASON_INVALID" in validate(out, inp)



def test_planning_continues_when_all_known_fire_is_observed_burned():
    """드론이 다시 가서 '다 탄 곳'으로 본 칸만 남아도 후보가 남는다 (실제 환경 실행 7번째에서 멈춘 경우)"""
    from orchestrator.observation_planner import build_input
    fire, rest = grid()
    idx = MapIndex(fire + rest)
    inp = build_input(idx=idx, fire_states={"12_12": {"status": "CONFIRMED_BURNED", "sim_time_s": 0.0},
                                            "13_12": {"status": "OBSERVED_CLEAR", "sim_time_s": 0.0}},
                      reports=[], weather={},
                      available=[{"resource_id": "A-uav1", "resource_type": "UAV", "lat": LAT0, "lon": LON0}],
                      busy=[], rejections=[], sim_time_s=10_000.0, run_id="R", burn_out=6300.0)
    assert inp["blocks"]


# ---------------------------------------------------------------------------
# 앞질러 보내기: 처음 발화(첫 신고) → 마지막 불로 진행 방향·속도 → 1·2시간 뒤 예상 지점 새 출동 (2026-10-07)
# ---------------------------------------------------------------------------

def test_advance_targets_one_and_two_hours_ahead(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    orch.sync()                                                          # 신고 12_12 @120 s
    _known(lg, "16_12", "CONFIRMED", 3720.0)                             # 1시간 뒤 동쪽 4칸(360 m)에서 불 확인
    t = orch._advance_targets(3720.0)
    assert [x["horizon_s"] for x in t] == [3600.0, 7200.0]
    assert t[0]["speed_m_per_h"] == 360 and 80 <= t[0]["bearing_deg"] <= 100      # 시간당 360 m, 동쪽
    assert t[0]["cell_id"] == "20_12" and t[1]["cell_id"] == "24_12"     # 1시간 앞 4칸, 2시간 앞 8칸
    assert t[0]["origin_cell"] == REPORT_CELL


def test_advance_scouts_are_dispatched_once_to_nearest_drones(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    orch.sync()
    _known(lg, "16_12", "CONFIRMED", 120.0 + 3600.0)
    orch._request_advance_scouts(3720.0)
    orch._request_advance_scouts(3720.0)                                 # 같은 '마지막 불'로는 한 번만
    adv = [t for t in lg.list_tasks() if t.kind == "ADVANCE_SCOUT"]
    assert len(adv) == 2 and {t.target.cell_id for t in adv} == {"20_12", "24_12"}
    orch.obs_plan_cycle()
    sent = [e for e in lg.events() if e["event_type"] == "OBS_ADVANCE_SCOUT_DISPATCH"]
    assert {e["result"] for e in sent} == {"STARTED"}
    assert {e["resource_id"] for e in sent} == {"A-uav1", "A-uav2"}       # 두 지점에 드론 두 대
    for t in adv:                                                        # 다음 계획이 앞질러 보내기를 취소하지 않음
        assert lg.get_task(t.task_id).purpose_status != "CANCELLED"



# ---------------------------------------------------------------------------
# API 키 예산 초과 → 다음 키 (사용자 결정 2026-10-07)
# ---------------------------------------------------------------------------

def test_budget_exceeded_switches_to_next_key(monkeypatch):
    import openai
    from orchestrator import llm as llm_mod
    monkeypatch.setenv("OPENAI_API_KEY", "key-one")
    monkeypatch.setenv("OPENAI_API_KEY_2", "key-two")
    used = []

    class Budget(Exception):
        body = {"error": {"code": "budget_exceeded", "message": "Budget limit exceeded."}}

    class Fake:
        def __init__(self, api_key=None, **kw):
            self.key = api_key
            self.chat = type("C", (), {"completions": self})()

        def create(self, **kw):
            used.append(self.key)
            if self.key == "key-one":
                raise Budget("Error code: 429 budget_exceeded")
            msg = type("M", (), {"content": "OK"})()
            return type("R", (), {"choices": [type("Ch", (), {"message": msg})()]})()
    monkeypatch.setattr(openai, "OpenAI", Fake)
    p = llm_mod.LlmPlanner(timeout_s=30, max_calls=5)
    assert p._chat([{"role": "user", "content": "x"}]) == "OK"
    assert used == ["key-one", "key-two"] and p.key_index == 1 and p.exhausted_keys == [0]
    assert p.status() is None


def test_all_keys_exhausted_disables_llm(monkeypatch):
    import openai
    from orchestrator import llm as llm_mod
    monkeypatch.setenv("OPENAI_API_KEY", "key-one")
    monkeypatch.delenv("OPENAI_API_KEY_2", raising=False)

    class Budget(Exception):
        body = {"error": {"code": "budget_exceeded"}}

    class Fake:
        def __init__(self, api_key=None, **kw):
            self.chat = type("C", (), {"completions": self})()

        def create(self, **kw):
            raise Budget("budget_exceeded")
    monkeypatch.setattr(openai, "OpenAI", Fake)
    p = llm_mod.LlmPlanner(timeout_s=30, max_calls=5)
    with pytest.raises(Budget):
        p._chat([{"role": "user", "content": "x"}])
    assert p.status() == "ALL_KEYS_BUDGET_EXCEEDED"



def _orbit(burning):
    fire, rest = grid(burning)
    env = FixtureEnv(fire_cells=fire, risk_cells=rest)
    c = cell(12, 12)
    task = Task(task_id="T", incident_id="I", kind="OBSERVE",
                target=Target(lat=c["lat"], lon=c["lon"], ground_amsl_m=600.0, cell_id="12_12"))
    return observation.simulate_edge_sweep(snapshot=env.read(), map_cells=env.map_cells(), task=task, attempt_id="A",
                                           resource_id="A-uav1", position={"lat": c["lat"], "lon": c["lon"]},
                                           report_point={"lat": c["lat"], "lon": c["lon"]}, orbit_duration_s=120.0)


def test_initial_drone_orbits_20m_inside_fire_line():
    """첫 드론: 신고 지점 둘레 원 120초. 원은 불 테두리선에서 다 탄 쪽 20 m 안 (2026-10-08).
    십자 모양 불(가운데 + 사방 한 칸): 가장 먼 테두리선 = 90 + 45 = 135 m → 반지름 115 m"""
    plus = {"12_12", "11_12", "13_12", "12_11", "12_13"}
    o = _orbit(plus)
    fp = o["footprint"]
    assert fp["mode"] == "ORBIT_FIRE_EDGE" and fp["duration_s"] == 120.0 and fp["edge_offset_m"] == 20.0
    assert 114.5 <= fp["orbit_radius_m"] <= 115.5
    seen = set(o["covered_cells"] + o["partial_cells"])
    assert plus <= seen and "15_12" not in seen
    assert o["result"] == "DETECTED"
    assert _orbit({"12_12"})["footprint"]["orbit_radius_m"] == 25.0      # 한 칸 불: 45 - 20 m


def test_initial_drone_follows_edge_when_fire_too_big_to_orbit():
    """불이 이미 커서 테두리 안쪽 원이 120초에 안 들어가면 원 대신 테두리를 따라간다"""
    o = _orbit(BURNING)
    assert o["footprint"]["mode"] == "EDGE_FOLLOW" and o["footprint"]["orbit_radius_m"] is None
    assert 0 < o["footprint"]["length_m"] <= 840.0 and o["result"] == "DETECTED"


def test_initial_drone_payload_requests_120s(pw, monkeypatch):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    monkeypatch.setattr(config, "OBS_INITIAL_NEAREST_UAV", True)
    _to_report_time(env)
    orch.obs_plan_cycle()
    ex = [b for m, rid, path, b in pw["uav"].calls if path == "/execute"]
    assert ex and ex[0]["observe_duration_s"] == 120
    _poll(orch)
    o = [e["detail"] for e in lg.events() if e["event_type"] == "OBSERVATION"
         and (e["detail"] or {}).get("sensor_type") == "THERMAL"][0]
    assert o["footprint"]["mode"] == "EDGE_FOLLOW"          # 시험 불이 커서 원 대신 테두리 (2026-10-08)



def test_candidate_blocks_capped_and_ahead_of_progress_first():
    """후보 구역은 상한(기본 10개)까지, 진행 방향 앞쪽 구역이 먼저 (사용자 결정 2026-10-07)"""
    from orchestrator.observation_planner import build_input, llm_view
    fire, rest = grid()
    idx = MapIndex(fire + rest)
    fs = {f"{c}_{r}": {"status": "CONFIRMED", "sim_time_s": 3000.0} for c in (12, 13) for r in (12, 13)}
    inp = build_input(idx=idx, fire_states=fs, reports=[{"report_id": "R1", "cell_id": "9_12", "sim_time_s": 120.0}],
                      weather={}, available=[{"resource_id": "A-uav1", "resource_type": "UAV", "lat": LAT0, "lon": LON0}],
                      busy=[], rejections=[], sim_time_s=3000.0, run_id="R", burn_out=6300.0)
    assert inp["fire_progress"]["basis"] == "IGNITION_TO_BURNING" and 80 <= inp["fire_progress"]["moved_toward_deg"] <= 100
    assert len(inp["blocks"]) == config.OBS_MAX_CANDIDATE_BLOCKS == 10 and inp["deferred_blocks"] > 0
    first = inp["blocks"][:5]
    assert all(b["ahead_of_progress"] for b in first)                    # 동쪽(진행 방향) 구역부터
    v = llm_view(inp)
    assert len(json.dumps(v, ensure_ascii=False)) < len(json.dumps(inp, ensure_ascii=False)) * 0.6   # 짧은 형태
    assert {b["id"] for b in v["blocks"]} == {b["block_id"] for b in inp["blocks"]}



def test_one_plan_per_env_step_then_gathered_next_step(pw):
    """환경 한 단계 안에서는 관측 계획을 한 번만, 그 뒤 돌아온 자원은 다음 단계에 모아서 (사용자 결정 2026-10-07)"""
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    first = orch.obs_plan_cycle()
    assert first
    _poll(orch)                                                          # 자원들이 관측하고 돌아옴 (같은 단계)
    assert orch.obs_plan_cycle() == []
    idle = [e for e in lg.events() if e["event_type"] == "OBS_PLAN_IDLE"]
    assert idle[-1]["result"] == "ALREADY_PLANNED_THIS_ENV_STEP"
    assert any("OBSERVATION" == e["event_type"] for e in lg.events())   # 관측 반영은 기다리지 않음
    env.advance(1)
    nxt = orch.obs_plan_cycle()
    # 드론은 다음 지점을 받기 전에 내려앉았지만 충전 결정이 아니라 충전하지 않는다 (2026-10-08) → 셋 모두 다음 회의에
    assert {e["resource_id"] for e in _events(lg, "OBS_LANDED_BEFORE_NEXT_STOP")} == {"A-uav1", "A-uav2"}
    assert nxt and len(nxt[0]["assignments"]) == 3


def test_frontier_includes_unobserved_next_to_passed_fire():
    """불이 지나간 칸(다 탄 곳·꺼짐 추정) 바로 옆 미관측도 '확인할 바깥'이다 (2026-10-07)"""
    from orchestrator.observation_planner import belief_layers, known_edge
    fire, rest = grid()
    idx = MapIndex(fire + rest)
    lay = belief_layers({"12_12": {"status": "CONFIRMED", "sim_time_s": 0.0},          # 오래돼 꺼짐 추정
                         "20_20": {"status": "CONFIRMED_BURNED", "sim_time_s": 9000.0},  # 관측으로 본 탄 곳
                         "13_12": {"status": "OBSERVED_CLEAR", "sim_time_s": 9000.0}}, now_s=10_000.0, burn_out=6300.0)
    edge, frontier = known_edge(idx, lay)
    assert not edge                                                    # 불타는 중인 칸이 없으니 테두리는 없다
    assert {"11_12", "12_11", "21_21", "19_20"} <= frontier and "13_12" not in frontier


def test_ugv_seeing_only_burned_follows_progress_450m(pw):
    """UGV 가 다 탄 곳만 보고 불을 못 찾으면 진행 방향으로 450 m 떨어진 곳에 같은 UGV 를 바로 다시 보낸다"""
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    orch.sync()                                                          # 신고 12_12
    _known(lg, "16_12", "CONFIRMED", 120.0)                              # 진행 방향: 동쪽
    here = cell(14, 12)
    task = Task(task_id="T-UGV", incident_id="I", kind="OBSERVE", plan_id="PLAN-X",
                target=Target(lat=here["lat"], lon=here["lon"], ground_amsl_m=600.0, cell_id="14_12"),
                requirements=Requirements(resource_types=("UGV",), sensor="THERMAL"))
    obs = {"observation_id": "OBS-UGV-1", "footprint_center": {"lat": here["lat"], "lon": here["lon"]}}
    orch._request_ugv_follow(task, obs, "A-ugv1", 120.0)
    orch._request_ugv_follow(task, obs, "A-ugv1", 120.0)                 # 같은 관측으로는 한 번만
    fol = [t for t in lg.list_tasks() if t.kind == "UGV_EDGE_FOLLOW"]
    assert len(fol) == 1 and fol[0].assigned_resource_id == "A-ugv1"
    assert fol[0].target.cell_id == "19_12"                              # 동쪽 450 m = 5칸
    orch.obs_plan_cycle()
    sent = [e for e in lg.events() if e["event_type"] == "OBS_UGV_FOLLOW_DISPATCH"]
    assert sent and sent[0]["result"] == "STARTED" and sent[0]["resource_id"] == "A-ugv1"


def test_ugv_follow_skipped_without_direction(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    orch.sync()
    here = cell(14, 12)
    task = Task(task_id="T-UGV", incident_id="I", kind="OBSERVE", plan_id="PLAN-X",
                target=Target(lat=here["lat"], lon=here["lon"], ground_amsl_m=600.0, cell_id="14_12"),
                requirements=Requirements(resource_types=("UGV",), sensor="THERMAL"))
    orch._request_ugv_follow(task, {"observation_id": "O2", "footprint_center": {"lat": here["lat"], "lon": here["lon"]}},
                             "A-ugv1", 120.0)
    assert not [t for t in lg.list_tasks() if t.kind == "UGV_EDGE_FOLLOW"]
    assert [e for e in lg.events() if e["event_type"] == "OBS_UGV_FOLLOW_SKIPPED"][0]["reason"] == "NO_PROGRESS_DIRECTION"


def test_ugv_follow_stops_when_stuck_or_same_step(pw):
    """UGV 가 같은 도로 지점에 다시 서면(길 막힘) 멈추고, 한 환경 단계에 한 번만 따라간다"""
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    orch.sync()
    _known(lg, "16_12", "CONFIRMED", 120.0)
    here = cell(14, 12)
    task = Task(task_id="T-UGV", incident_id="I", kind="OBSERVE", plan_id="PLAN-X",
                target=Target(lat=here["lat"], lon=here["lon"], ground_amsl_m=600.0, cell_id="14_12"),
                requirements=Requirements(resource_types=("UGV",), sensor="THERMAL"))
    pos = {"lat": here["lat"], "lon": here["lon"]}
    orch._request_ugv_follow(task, {"observation_id": "O1", "footprint_center": pos}, "A-ugv1", 120.0)
    orch._request_ugv_follow(task, {"observation_id": "O2", "footprint_center": pos}, "A-ugv1", 120.0)   # 같은 단계
    assert len([t for t in lg.list_tasks() if t.kind == "UGV_EDGE_FOLLOW"]) == 1
    orch._request_ugv_follow(task, {"observation_id": "O3", "footprint_center": pos}, "A-ugv1", 2220.0)  # 같은 자리
    assert len([t for t in lg.list_tasks() if t.kind == "UGV_EDGE_FOLLOW"]) == 1
    sk = [e for e in lg.events() if e["event_type"] == "OBS_UGV_FOLLOW_SKIPPED"]
    assert sk[-1]["reason"] == "UGV_DID_NOT_MOVE_ROAD_LIMIT"


def test_three_discovery_metrics():
    """테두리 재현율 + 불타는 곳 발견율 + 화재 영역 발견율(누적, 관측만)"""
    from orchestrator.map_evaluation import edge_recall
    fire = [cell(c, 12, "BURNING") for c in (10, 11, 12)] + [cell(c, 12, "BURNED") for c in (7, 8, 9)]
    _, rest = grid(burning=set())
    rest = [c for c in rest if c["cell_id"] not in {f["cell_id"] for f in fire}]
    snap = FixtureEnv(fire_cells=fire, risk_cells=rest).read()
    layers = {"12_12": {"state": "BURNING"}, "11_12": {"state": "PRESUMED_BURNED"}}
    ever = {"12_12", "11_12", "8_12"}                                     # 관측으로 본 적 있는 불·탄 곳
    m = edge_recall(snap, layers, None, ever)
    assert m["burning_found"] == 1 and m["burning_found_pct"] == 33.3    # 불타는 3칸 중 지금 아는 1칸
    assert m["area_found"] == 3 and m["fire_area_found_pct"] == 50.0     # 불·탄 6칸 중 본 적 있는 3칸
    assert m["edge_recall_pct"] is not None and m["used_for_planning"] is False


# ---------------------------------------------------------------------------
# 작업 지시서·충전·불 머리·긴급 회의 (사용자 결정 2026-10-08)
# ---------------------------------------------------------------------------

def _returning_flight(margin=10.0):
    """ENROUTE → OBSERVING → COMPLETED(RETURNING) 에서 멈춤 — 복귀 중에 다음 지점을 받는 드론"""
    def script(rid, body):
        t = body["target"]
        pos = {"lat": t["lat"], "lon": t["lon"], "alt_m_amsl": t["alt_m_amsl"] + t["target_agl_m"] + margin}
        return [{"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
                {"status": "IN_PROGRESS", "progress": {"phase": "OBSERVING"}},
                {"status": "COMPLETED", "progress": {"phase": "RETURNING"},
                 "observation": {"position": pos, "sensor_type": "THERMAL"}}]
    return script


def _hook_task(cell_id="12_12", plan_id="PLAN-T"):
    c = cell(*map(int, cell_id.split("_")))
    return Task(task_id="T-HOOK", incident_id="I", kind="OBSERVE", plan_id=plan_id,
                target=Target(lat=c["lat"], lon=c["lon"], ground_amsl_m=600.0, cell_id=cell_id))


def _with_known_fire(orch, env, lg):
    _to_report_time(env)
    orch.sync()                                          # 기상(바람 270° = 서풍) 을 아는 세계에
    for cid in ("14_12", "15_12", "16_12", "17_12", "12_12"):
        _known(lg, cid, "CONFIRMED", 120.0)


def test_rule_plan_gives_each_resource_up_to_three_stops_and_validator_checks_count(pw):
    orch, env = pw["orch"], pw["env"]
    _to_report_time(env)
    inp = orch.obs_plan_request()["inp"]
    plan = rule_plan(inp)
    assert validate(plan, inp) == []
    assert all(1 <= len(a["block_ids"]) <= 3 for a in plan["assignments"])
    assert len({b for a in plan["assignments"] for b in a["block_ids"]}) == sum(len(a["block_ids"])
                                                                                for a in plan["assignments"])
    four = {**plan, "assignments": [dict(plan["assignments"][0], block_ids=[b["block_id"] for b in inp["blocks"][:4]])]
            + plan["assignments"][1:]}
    assert any(v.startswith("BLOCK_IDS_INVALID") for v in validate(four, inp))


def test_prompt_is_short_and_view_carries_battery_and_charging(pw):
    from orchestrator.observation_planner import LEGEND, SYSTEM, llm_view
    assert len(SYSTEM) < 800 and len(LEGEND) < 600                     # 장황한 설명은 뺀다 (2026-10-08)
    orch, env = pw["orch"], pw["env"]
    _to_report_time(env)
    orch._charging["A-uav2"] = {"since_s": 0.0, "until_s": 2000.0}
    v = llm_view(orch.obs_plan_request()["inp"])
    assert ["A-uav1", "UAV", 95, "IDLE"] in v["res"]
    assert v["chg"] == [["A-uav2", 1880]] and v["order_max_stops"] == 3
    assert "note" not in json.dumps(v.get("progress") or {})


def test_order_next_stop_is_sent_while_returning_without_landing(pw):
    orch, env, lg, uav = pw["orch"], pw["env"], pw["ledger"], pw["uav"]
    uav.default_script = _returning_flight()
    _to_report_time(env)
    row = next(r for r in orch.obs_plan_cycle()[0]["assignments"] if r["resource_id"] == "A-uav1")
    assert len(row["block_ids"]) == 3 and row["trimmed_for_battery"] == []
    _poll(orch)                                          # 첫 지점 관측 → 복귀 중
    for rid in ("A-uav1", "A-uav2"):
        uav.state[rid]["current_task_id"] = None         # 실제 드론: 복귀 중은 '실행 중' 이 아니다
    orch.obs_plan_cycle()
    assert "A-uav1" in {e["resource_id"] for e in _events(lg, "OBS_RETASK_WHILE_RETURNING")}
    q = [e for e in _events(lg, "OBS_ORDER_STOP_QUEUED") if e["resource_id"] == "A-uav1"]
    assert [e["detail"]["stop"] for e in q] == [1, 2]
    second = lg.get_task(q[1]["task_id"])
    assert MapIndex.block_id(second.target.cell_id) == row["block_ids"][1]
    assert lg.reservations()["A-uav1"]["task_id"] == second.task_id    # 기지에 내리지 않고 바로 다음 지점
    assert not _events(lg, "OBS_CHARGING_START")


def test_battery_below_30_ends_order_early(pw, monkeypatch):
    orch, env, lg, uav = pw["orch"], pw["env"], pw["ledger"], pw["uav"]
    _with_known_fire(orch, env, lg)
    orch._orders["A-uav1"] = {"plan_id": "P", "source": "LLM", "stops": [{"block_id": "B6_6", "cell_id": "19_19"}],
                              "next": 0, "new_fire": 0, "kind": "ORDER", "resource_type": "UAV", "since_s": 0}
    orch._orders["A-uav2"] = dict(orch._orders["A-uav1"], stops=[])          # 다른 드론도 일하는 중
    uav.state["A-uav1"]["battery"] = 25.0
    orch._order_after_observation(_hook_task(), "A-uav1", 0, 180.0, ground=False)
    end = _events(lg, "OBS_ORDER_END")[-1]
    assert end["result"] == "RETURN_TO_CHARGE" and end["detail"]["stops_total"] == 1 and end["detail"]["stops_done"] == 0
    assert "A-uav1" in orch._to_charge


def test_new_fire_in_order_follows_fire_head_downwind(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _with_known_fire(orch, env, lg)
    orch._order_after_observation(_hook_task(), "A-uav1", 2, 180.0, ground=False)
    end = _events(lg, "OBS_ORDER_END")[-1]
    assert end["result"] == "FOLLOW_HEAD"
    assert end["detail"]["head"] == {"cell_id": "17_12", "bearing_deg": 90, "basis": "DOWNWIND"}   # 서풍 → 동쪽 끝
    t = lg.get_task(_events(lg, "OBS_ORDER_STOP_QUEUED")[-1]["task_id"])
    assert t.kind == "HEAD_FOLLOW" and t.target.cell_id == "17_12" and t.assigned_resource_id == "A-uav1"


def test_no_new_fire_returns_to_charge_unless_last_drone_in_air(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _with_known_fire(orch, env, lg)
    for rid in ("A-uav1", "A-uav2"):
        pw["uav"].state[rid]["battery"] = 40.0           # 30% 이상 50% 미만
    orch._order_after_observation(_hook_task(), "A-uav1", 0, 180.0, ground=False)
    assert _events(lg, "OBS_ORDER_END")[-1]["result"] == "SEEK_UNSEEN_EDGE_LAST_AIRBORNE"   # 하늘에 혼자 → 남음
    orch._order_after_observation(_hook_task(), "A-uav2", 0, 180.0, ground=False)
    assert _events(lg, "OBS_ORDER_END")[-1]["result"] == "RETURN_TO_CHARGE"           # A-uav1 이 하늘에 있음
    orch._order_after_observation(_hook_task(), "A-ugv1", 0, 180.0, ground=True)
    assert _events(lg, "OBS_ORDER_END")[-1]["result"] == "WAIT"                       # UGV 는 충전 없음 → 대기


def test_landed_drone_charges_30min_then_gets_rule_order(pw):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    orch.obs_plan_cycle()
    orch._to_charge |= {"A-uav1", "A-uav2"}              # 충전 복귀로 정한 드론 (규칙 판단은 다른 시험)
    _poll(orch)                                          # normal_flight: 관측 → 복귀 → 착륙(DONE)
    start = [e for e in _events(lg, "OBS_CHARGING_START") if e["resource_id"] == "A-uav1"][0]
    assert start["result"] == "LANDED_AT_BASE" and start["detail"]["charge_s"] == 1800.0
    assert start["detail"]["basis"] == "TEST_ONLY_ASSUMED_DOCK_CHARGE_30MIN"
    b = orch._obs_build()
    assert b["unavailable"]["A-uav1"] == "CHARGING"
    assert {r["resource_id"] for r in b["inp"]["resources_charging"]} == {"A-uav1", "A-uav2"}
    t, _ = orch.submit_task({"request_id": "ANY", "incident_id": "I", "kind": "OBSERVE",
                             "target": {"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0, "cell_id": "0_0"},
                             "requirements": {"resource_types": ["UAV"], "sensor": "THERMAL"}})
    assert orch.dispatch(t.task_id)["excluded"]["A-uav1"] == "CHARGING"     # 어떤 출동에도 쓰지 않음
    env.advance(31)                                      # 31분 뒤
    orch._obs_planned_sim = env.simulation_time_s        # 정기 회의는 이미 한 것으로 (규칙 지시서만 보기)
    orch._obs_last_hash = None
    outs = orch.obs_plan_cycle()
    assert {e["resource_id"] for e in _events(lg, "OBS_CHARGING_DONE")} == {"A-uav1", "A-uav2"}
    after = [o for o in outs if o["source"] == "RULE_AFTER_CHARGE"]
    assert {r["resource_id"] for o in after for r in o["assignments"]} == {"A-uav1", "A-uav2"}
    assert not orch.llm                                  # LLM 없이


def test_order_trimmed_to_stops_the_battery_can_return_from(pw):
    orch = pw["orch"]
    idx = orch._map_index()
    r = {"resource_id": "A-uav1", "resource_type": "UAV", "lat": LAT0 + 0.02, "lon": LON0}
    cells = ["12_12", "20_20", "28_28"]
    assert orch._trim_order({**r, "battery_pct": 95}, cells, idx) == 3
    assert orch._trim_order({**r, "battery_pct": 16}, cells, idx) == 0
    full = orch._trim_order({**r, "battery_pct": 95}, cells, idx)
    mid = [orch._trim_order({**r, "battery_pct": p}, cells, idx) for p in range(16, 96, 5)]
    assert mid == sorted(mid) and mid[0] < full                          # 배터리가 적을수록 짧게
    assert orch._trim_order({**r, "resource_type": "UGV", "battery_pct": 5}, cells, idx) == 3


def test_human_risk_opens_meeting_now_once_per_site(pw, monkeypatch):
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _to_report_time(env)
    assert orch.obs_plan_cycle()                         # 정기 회의
    assert orch.obs_plan_cycle() == []                   # 같은 단계 — 회의 없음
    site = {"key": "SITE-1", "site_id": "SITE-1", "cells": ["17_12"], "basis": ["FORECAST_REACHES_OCCUPIED_SITE"],
            "earliest_arrival_s": 900.0, "target_cell": "17_12"}
    monkeypatch.setattr(orch, "preemptive_candidates", lambda snap: [site])
    env.state_version += 1
    out = orch.obs_plan_cycle()
    assert out and _events(lg, "OBS_EMERGENCY_CALL")[0]["result"] == "SITE-1"
    env.state_version += 1
    assert orch.obs_plan_cycle() == []                   # 같은 장소로는 다시 부르지 않음
    assert len(_events(lg, "OBS_EMERGENCY_CALL")) == 1


def test_edge_sweep_goes_toward_fire_head_20m_inside_burned_side():
    fire, rest = grid()
    env = FixtureEnv(fire_cells=fire, risk_cells=rest)
    c = next(x for x in fire if x["cell_id"] == "14_12")
    task = Task(task_id="T", incident_id="I", kind="OBSERVE",
                target=Target(lat=c["lat"], lon=c["lon"], ground_amsl_m=600.0, cell_id="14_12"))
    o = observation.simulate_edge_sweep(snapshot=env.read(), map_cells=env.map_cells(), task=task, attempt_id="A",
                                        resource_id="A-uav1", position={"lat": c["lat"], "lon": c["lon"]},
                                        report_point=None, head_bearing_deg=90.0)
    fp = o["footprint"]
    assert fp["mode"] == "EDGE_FOLLOW" and fp["edge_offset_m"] == 20.0 and fp["head_bearing_deg"] == 90.0
    assert [p["cell_id"] for p in o["fire_points"]][:3] == ["14_12", "15_12", "16_12"]   # 동쪽(불 머리)으로
    # 14_12 의 안 탄 이웃(15_11·15_13)은 동쪽 → 테두리선(칸 가운데 + 45 m)에서 20 m 안쪽 = 가운데에서 동쪽 25 m
    east_m = (fp["path"][1]["lon"] - c["lon"]) * 111_320.0 * math.cos(math.radians(c["lat"]))
    assert abs(east_m - 25.0) < 1.0


def test_queued_stop_is_dispatched_while_waiting_for_slow_llm(pw):
    """LLM 을 기다리는 동안 들어온 지시서 다음 지점도 바로 보낸다 (기다리다 드론이 내려앉지 않게, 2026-10-08)"""
    import time as _t
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    seen = {}

    def reply(kw):
        inp = _seen(kw)
        orch._orders["A-uav1"] = {"plan_id": "P-SLOW", "source": "LLM", "stops": [{"block_id": "B6_6", "cell_id": "19_19"}],
                                  "next": 0, "new_fire": 0, "kind": "ORDER", "resource_type": "UAV", "since_s": 0}
        tid = orch._queue_stop("A-uav1", 120.0).task_id          # 추적 루프가 관측 뒤에 넣은 것처럼
        _t.sleep(0.8)
        seen.setdefault("status", lg.get_task(tid).purpose_status)   # 첫 호출의 답이 나가기 전 상태
        return {"input_hash": inp["input_hash"], "predicted_blocks": [], "prediction_rationale": "x",
                "assignments": [], "request_reserve_uavs": False, "reserve_reason": ""}
    orch.llm = LlmPlanner(client=FakeOpenAI(reply), timeout_s=30, max_calls=5)
    _to_report_time(env)
    orch.obs_plan_cycle(max_rounds=1)
    assert seen["status"] != "PENDING"


def test_no_new_fire_but_battery_50_keeps_watching_head(pw):
    """새 불이 없어도 배터리 50% 이상이면 충전하러 가지 않고 불 머리를 계속 본다 (2026-10-08)"""
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    _with_known_fire(orch, env, lg)
    orch._orders["A-uav2"] = {"plan_id": "P", "source": "LLM", "stops": [], "next": 0, "new_fire": 0, "kind": "ORDER",
                              "resource_type": "UAV", "since_s": 0}          # 다른 드론도 하늘에
    pw["uav"].state["A-uav1"]["battery"] = 60.0
    orch._order_after_observation(_hook_task(), "A-uav1", 0, 180.0, ground=False)
    end = _events(lg, "OBS_ORDER_END")[-1]
    # 그 근처는 다 봄 → 방금 본 곳(12_12)에서 가장 가까운, 아직 안 본 테두리 바깥 칸으로
    assert end["result"] == "SEEK_UNSEEN_EDGE_BATTERY_OK" and end["detail"]["head"]["basis"] == "NEAREST_UNSEEN_EDGE"
    target = end["detail"]["head"]["cell_id"]
    assert target not in orch.kb.fire_states(180.0)                     # 아직 아무도 안 본 칸
    c, r = map(int, target.split("_"))
    assert any(f"{c + dc}_{r + dr}" in {"12_12", "14_12", "15_12", "16_12", "17_12"}
               for dc in (-1, 0, 1) for dr in (-1, 0, 1))                # 아는 불 바로 바깥
    assert end["detail"]["head"]["distance_m"] <= 130                    # 12_12 바로 옆
    assert "A-uav1" not in orch._to_charge


def test_llm_stuck_past_wall_limit_falls_back_to_rules(pw, monkeypatch):
    import time as _t
    orch, env, lg = pw["orch"], pw["env"], pw["ledger"]
    monkeypatch.setattr(config, "OBS_LLM_WALL_MAX_S", 0.5)

    def reply(kw):
        _t.sleep(2.0)                                    # 시간 제한이 안 걸리는 연결 멈춤
        return "{}"
    orch.llm = LlmPlanner(client=FakeOpenAI(reply), timeout_s=30, max_calls=5)
    _to_report_time(env)
    t0 = _t.monotonic()
    out = orch.obs_plan_cycle(max_rounds=1)[0]
    assert _t.monotonic() - t0 < 1.8
    assert out["source"] == "RULE_FALLBACK" and _events(lg, "OBS_LLM_WALL_TIMEOUT")
    assert _events(lg, "OBS_PLAN")[-1]["detail"]["llm"]["reason"] == "WALL_CLOCK_TIMEOUT"


def test_unknown_evidence_ref_is_dropped_not_reasked(pw):
    orch, env = pw["orch"], pw["env"]
    _to_report_time(env)
    inp = orch.obs_plan_request()["inp"]
    plan = rule_plan(inp)
    plan["assignments"][0]["evidence_refs"] = plan["assignments"][0]["evidence_refs"] + ["B99_99"]
    assert validate(plan, inp) == []
    assert "B99_99" not in plan["assignments"][0]["evidence_refs"]


def test_ugv_target_out_of_road_view_is_not_resent_forever(pw):
    """UGV 는 도로에서만 본다 — 목표 칸이 시야 밖이어도 그 지점 관측으로 끝내고 같은 자리로 다시 보내지 않는다
    (2026-10-08 실제 환경 실행: 다시 열고 다시 보내기가 끝없이 반복됨)"""
    orch, env, lg, ugv = pw["orch"], pw["env"], pw["ledger"], pw["ugv"]

    def far_road(rid):
        body = next(b for m, p, b in reversed(ugv.calls) if p.endswith("/execute"))
        t = body["target"]
        return [{"status": "COMPLETED", "progress": {"phase": "ARRIVED"},
                 "observation": {"observation_type": "ROAD_STATUS", "arrived_node": "N1",
                                 "position": {"lat": t["lat"] + 0.004, "lon": t["lon"]}},   # 목표에서 약 450 m
                 "_unit": {"state": "READY", "current_task_id": None}}]
    ugv.default_script = far_road
    _to_report_time(env)
    row = next(r for r in orch.obs_plan_cycle()[0]["assignments"] if r["resource_id"] == "A-ugv1")
    _poll(orch)
    t = lg.get_task(row["task_id"])
    assert t.purpose_status == "COMPLETED"
    done = [e for e in lg.events(t.task_id) if e["event_type"] == "TASK_COMPLETE"][-1]["detail"]
    assert done["basis"] == ["ARRIVED", "PLANNED_VISIT_DONE"] and done["target_covered"] is False
    orch.obs_plan_cycle()
    _poll(orch)
    assert len([a for a in lg.list_attempts(t.task_id)]) == 1           # 같은 지점으로 다시 보내지 않음
