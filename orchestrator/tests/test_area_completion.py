# -*- coding: utf-8 -*-
"""정찰(RECON)과 구역 감시(MONITOR)의 완료조건 분리 (사용자 결정 2026-09-30 D03).

- RECON    목표 지점이 유효 관측 범위 안이면 완료. area_cell_ids 는 관련 영역 메타데이터일 뿐이다.
- MONITOR  required_cell_ids 전체가 누적 관측으로 완전히 덮여야 완료. 한 점 결과를 구역 완료로 올리지 않는다.
- 프레임이 칸 일부만 덮으면 그만큼만 쌓인다 (비율은 기하 계산값, 임계값 없음).

시험 프레임: TEST_THERMAL_50M_V1 을 지면 위 90m 에서 → 54m × 45m.
"""

import pytest

from orchestrator import observation
from orchestrator.engine import TaskRejected

from .conftest import submit
from .test_priority import LAT0, LON0, cell

M1, M2 = cell("M1", 0, 0, 0.5, size=30.0), cell("M2", 0, 30, 0.5, size=30.0)     # 30m 칸 둘이 동서로 붙어 있다
BIG = cell("BIG", 0, 0, 0.5, size=90.0)
THERMAL = {"resource_types": ["UAV"], "sensor": "THERMAL"}


def _map(world, *cells):
    world["env"].fire_cells = []
    world["env"].risk_cells = [dict(c) for c in cells]


def _target(c):
    return {"lat": c["lat"], "lon": c["lon"], "ground_amsl_m": 600.0, "cell_id": c["cell_id"]}


def _monitor(orch, required, c=M1, rid="MON-1"):
    return submit(orch, rid, kind="MONITOR", target=_target(c), required_cell_ids=required, requirements=THERMAL)


def _visit(orch, task_id):
    out = orch.dispatch(task_id)
    assert out["status"] == "STARTED", out
    for _ in range(3):                                            # 이동 → 도착 → 관측·복귀 시작
        orch.poll()
    return out


def _ev(lg, task_id, etype):
    return [e for e in lg.events(task_id) if e["event_type"] == etype]


def test_monitor_is_not_complete_after_one_of_two_cells_and_completes_when_all_covered(world):
    orch, lg, uav = world["orch"], world["ledger"], world["uav"]
    _map(world, M1, M2)
    task = _monitor(orch, ["M1", "M2"])
    assert task.completion_rule == "AREA_ALL_REQUIRED_CELLS"
    first = _visit(orch, task.task_id)

    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "AREA_COVERAGE_INCOMPLETE"   # M1 만 봤다 → 미완료
    assert t.coverage["M1"]["fraction"] == 1.0 and t.coverage["M2"]["fraction"] == pytest.approx(0.4, abs=2e-3)
    assert t.coverage["M2"]["sources"] == ["SIMULATED"] and t.coverage["M2"]["last_observed_sim_s"] == 0.0
    assert t.visited_cell_ids == ["M1"] and t.visit_target.cell_id == "M2"
    cov = _ev(lg, task.task_id, "AREA_COVERAGE")[0]
    assert cov["result"] == "INCOMPLETE" and cov["detail"]["remaining"] == ["M2"]
    assert _ev(lg, task.task_id, "TASK_COMPLETE") == []

    orch.poll()                                                   # 기체 반납 뒤에도 남은 범위가 있으면 미완료
    assert lg.get_attempt(first["attempt_id"])["substatus"] == "RELEASED"
    p = orch.area_progress(lg.get_task(task.task_id))
    assert p["complete"] is False and p["remaining"] == ["M2"] and p["unvisited"] == ["M2"]

    _visit(orch, task.task_id)                                    # 남은 칸으로 다시 보낸다
    sent = uav.exec_calls()[1][3]["target"]
    assert sent["lon"] == pytest.approx(M2["lon"]) and sent["lat"] == pytest.approx(M2["lat"])
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "COMPLETED" and t.coverage["M2"]["fraction"] == 1.0
    done = _ev(lg, task.task_id, "TASK_COMPLETE")[0]["detail"]
    assert done["completion_rule"] == "AREA_ALL_REQUIRED_CELLS"
    assert "ALL_REQUIRED_CELLS_FULLY_OBSERVED" in done["basis"] and done["coverage"]["remaining"] == []


def test_recon_completes_on_target_point_and_area_cells_are_only_metadata(world):
    orch, lg = world["orch"], world["ledger"]
    _map(world, M1, M2)
    task = submit(orch, "RECON-1", target=_target(M1), area_cell_ids=["M1", "M2"], requirements=THERMAL)
    assert task.completion_rule == "TARGET_POINT" and task.required_cell_ids == []
    assert task.area_cell_ids == ["M1", "M2"]
    _visit(orch, task.task_id)
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"
    assert _ev(lg, task.task_id, "TASK_COMPLETE")[0]["detail"]["completion_rule"] == "TARGET_POINT"
    assert _ev(lg, task.task_id, "AREA_COVERAGE") == []


def test_recon_target_condition_not_met_when_target_outside_frame(world):
    orch, lg, uav = world["orch"], world["ledger"], world["uav"]
    _map(world, M1)

    def far(rid, body):                                           # 목표에서 1km 남짓 떨어진 곳에서 관측
        pos = {"lat": body["target"]["lat"] + 0.01, "lon": body["target"]["lon"], "alt_m_amsl": 690.0}
        return [{"status": "COMPLETED", "progress": {"phase": "RETURNING"},
                 "observation": {"position": pos, "sensor_type": "THERMAL", "values": {}}}]
    uav.default_script = far
    task = submit(orch, "RECON-2", target=_target(M1), requirements=THERMAL)
    orch.dispatch(task.task_id)
    orch.poll()
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "PENDING" and t.hold_reason == "OBSERVATION_REQUIREMENT_NOT_MET"


def test_scope_fields_are_validated_at_intake(world):
    orch = world["orch"]
    _map(world, M1, M2)
    with pytest.raises(TaskRejected, match="REQUIRED_CELLS_ONLY_FOR_AREA_TASKS"):
        submit(orch, "R", target=_target(M1), required_cell_ids=["M1", "M2"], requirements=THERMAL)
    with pytest.raises(TaskRejected, match="AREA_EVIDENCE_UNSUPPORTED_FOR_SENSOR:WEATHER"):
        submit(orch, "M", kind="MONITOR", target=_target(M1), required_cell_ids=["M1"],
               requirements={"resource_types": ["UAV"], "sensor": "WEATHER"})
    with pytest.raises(TaskRejected, match="AREA_REQUIRED_CELLS_MISSING"):
        submit(orch, "M2", kind="MONITOR", target={"lat": LAT0, "lon": LON0, "ground_amsl_m": 600.0},
               requirements=THERMAL)
    # 필수 셀을 따로 주지 않으면 area_cell_ids → 목표 칸 순서로 정한다
    assert submit(orch, "M3", kind="MONITOR", target=_target(M1), area_cell_ids=["M1", "M2"],
                  requirements=THERMAL).required_cell_ids == ["M1", "M2"]
    assert submit(orch, "M4", kind="MONITOR", target=_target(M2), requirements=THERMAL).required_cell_ids == ["M2"]


def test_partial_overlap_is_recorded_as_partial_not_as_whole_cell(world):
    orch, lg = world["orch"], world["ledger"]
    _map(world, BIG)                                               # 90m 칸 > 54×45m 프레임
    task = submit(orch, "RECON-BIG", target=_target(BIG), requirements=THERMAL)
    _visit(orch, task.task_id)
    obs = _ev(lg, task.task_id, "OBSERVATION")[0]["detail"]
    assert obs["covered_cells"] == [] and obs["partial_cells"] == ["BIG"] and obs["target_covered"] is True
    cc = obs["cell_coverage"][0]
    assert cc["full"] is False and cc["fraction"] == pytest.approx(54 * 45 / 8100) and cc["rect"] == [-27, -22.5, 27, 22.5]
    assert orch.kb.fire_states(1e9)["BIG"]["status"] == "NOT_DETECTED_PARTIAL"
    assert lg.get_task(task.task_id).purpose_status == "COMPLETED"   # 정찰은 목표 지점 기준


def test_area_task_is_held_when_single_frames_cannot_cover_the_cell(world):
    orch, lg, uav = world["orch"], world["ledger"], world["uav"]
    _map(world, BIG)
    task = _monitor(orch, ["BIG"], c=BIG)
    first = _visit(orch, task.task_id)
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "HOLD" and t.hold_reason == "AREA_EVIDENCE_UNSUPPORTED"
    assert t.resume_condition == "MANUAL_RESOLUTION"              # 같은 지점으로 계속 다시 보내지 않는다
    hold = _ev(lg, task.task_id, "TASK_HOLD")[-1]["detail"]
    assert hold["remaining"] == ["BIG"] and hold["fraction"]["BIG"] == pytest.approx(0.3) and hold["unvisited"] == []
    orch.poll()
    assert lg.get_attempt(first["attempt_id"])["substatus"] == "RELEASED"
    orch.dispatch_pending()
    assert lg.get_task(task.task_id).purpose_status == "HOLD" and len(uav.exec_calls()) == 1
    assert _ev(lg, task.task_id, "TASK_COMPLETE") == []


def test_required_cell_missing_from_map_is_never_counted_as_observed(world):
    orch, lg = world["orch"], world["ledger"]
    _map(world, M1)
    task = _monitor(orch, ["M1", "GHOST"])
    _visit(orch, task.task_id)
    t = lg.get_task(task.task_id)
    assert t.purpose_status == "HOLD" and t.hold_reason == "AREA_EVIDENCE_UNSUPPORTED"
    hold = _ev(lg, task.task_id, "TASK_HOLD")[-1]["detail"]
    assert hold["remaining"] == ["GHOST"] and hold["geometry_missing"] == ["GHOST"]
    assert t.coverage["M1"]["fraction"] == 1.0 and "GHOST" not in t.coverage


def test_same_observation_is_not_accumulated_twice_and_union_does_not_double_count(world):
    orch, lg = world["orch"], world["ledger"]
    _map(world, M1, M2)
    task = _monitor(orch, ["M1", "M2"])
    out = _visit(orch, task.task_id)
    att = lg.get_attempt(out["attempt_id"])
    obs = _ev(lg, task.task_id, "OBSERVATION")[0]["detail"]
    t = lg.get_task(task.task_id)
    before = {c: dict(v) for c, v in t.coverage.items()}
    orch._conclude_area(att, t, obs, {"accepted": True}, True)      # 같은 관측이 다시 전달됨
    after = lg.get_task(task.task_id).coverage
    assert after["M2"]["fraction"] == before["M2"]["fraction"] and len(after["M2"]["rects"]) == 1
    assert after["M1"]["fraction"] == before["M1"]["fraction"] == 1.0
    # 겹친 부분은 한 번만 센다
    assert observation.union_fraction([[-15, -15, 0, 15, "a"], [-5, -15, 15, 15, "b"]], 30.0) == 1.0
    assert observation.union_fraction([[-15, -15, 0, 15, "a"], [-15, -15, 0, 15, "b"]], 30.0) == pytest.approx(0.5)
    assert observation.union_fraction([[-15, -15, -3, 15, "a"], [3, -15, 15, 15, "b"]], 30.0) == pytest.approx(0.8)
    assert observation.union_fraction([], 30.0) == 0.0


def test_recheck_of_area_task_inherits_area_rule(world):
    orch, lg = world["orch"], world["ledger"]
    _map(world, M1, M2)
    task = _monitor(orch, ["M1", "M2"])
    orch.resolve(task.task_id, "CANCEL", "취소")
    r = orch.resolve(task.task_id, "RECHECK", "다시 감시")
    child = lg.get_task(r["task_id"])
    assert child.kind == "RECHECK" and child.completion_rule == "AREA_ALL_REQUIRED_CELLS"
    assert child.required_cell_ids == ["M1", "M2"] and child.coverage == {}
