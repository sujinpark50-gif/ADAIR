# -*- coding: utf-8 -*-
"""총괄 정보 경계 (피드백 2026-09-30 §2).

환경모델(진짜 세계)의 미래 화재·위험·확산만 바꾸고, 총괄이 아는 신고·관측·기상을 그대로 두면
총괄의 판단용 위험도·확산 예상은 바뀌지 않아야 한다. 새 신고·유효 관측이 들어온 뒤에만 갱신된다.
"""

from orchestrator.knowledge import InjectedAnalysis

from .test_priority import cell


def _view_analysis(orch):
    v = orch.view()
    return ([c["cell_id"] for c in v.risk_cells], [c["cell_id"] for c in v.spread_forecast], v.analysis_source)


def test_truth_only_changes_do_not_leak_into_orchestrator_analysis(world):
    orch, env = world["orch"], world["env"]
    before = _view_analysis(orch)
    # 진짜 세계에서만 불이 번지고 위험·확산이 바뀜 (총괄은 아직 아무 신고·관측 없음)
    env.fire_cells.append({"cell_id": "NEW-FIRE", "lat": 38.06, "lon": 128.26, "cell_size_m": 90.0,
                           "fire_state": "BURNING", "risk_score": 1.0})
    env.risk_cells = [cell("TRUE-RISK", 0, 0, 0.99)]
    env.spread_forecast = [dict(cell("TRUE-SPREAD", 0, 90, 0.9), expected_arrival_s=60.0)]
    env.advance()
    after = _view_analysis(orch)
    assert after[:2] == before[:2]
    assert "TRUE-RISK" not in after[0] and "TRUE-SPREAD" not in after[1]
    assert "NEW-FIRE" not in [c["cell_id"] for c in orch.view().fire_cells]


def test_analysis_is_labeled_test_injected(world):
    assert orch_source(world) == "TEST_INJECTED"


def orch_source(world):
    return world["orch"].view().analysis_source


def test_injected_analysis_updates_only_after_new_report(world):
    orch, env = world["orch"], world["env"]
    world["analysis"].risk_cells = [dict(cell("R-NEAR-C1", 0, 180, 0.6), requires_known_fire="C1")]
    assert _view_analysis(orch)[0] == []                     # 아직 C1 을 모름
    env.reports = [{"cell_id": "C1", "sim_time_s": env.simulation_time_s, "source": "119_CALL"}]
    assert _view_analysis(orch)[0] == ["R-NEAR-C1"]          # 신고 후부터 갱신


def test_injected_analysis_updates_after_valid_observation(world):
    from .conftest import submit
    orch = world["orch"]
    world["analysis"].risk_cells = [dict(cell("R-NEAR-C1", 0, 180, 0.6), requires_known_fire="C1")]
    assert _view_analysis(orch)[0] == []
    orch.dispatch(submit(orch).task_id)                      # 불타는 C1 정찰
    for _ in range(3):
        orch.poll()
    assert _view_analysis(orch)[0] == ["R-NEAR-C1"]


def test_injected_values_are_marked_not_derived_from_observation(world):
    world["analysis"].risk_cells = [cell("R1", 0, 0, 0.5)]
    v = world["orch"].view()
    assert v.risk_cells[0]["derived_from_observation"] is False
    assert v.risk_cells[0]["analysis_source"] == "TEST_INJECTED"


def test_kma_future_rows_never_enter(tmp_path):
    from .test_knowledge import _world
    orch, env, uav, lg = _world(tmp_path)
    env.simulation_time_s = 0.0                              # 14:00
    orch.view()
    times = {k["body"]["observed_kst"] for k in lg.knowledge("WEATHER") if k["body"].get("kind") == "STATION"}
    assert max(times) == "2019-04-04 14:00"


def test_completion_is_labeled_simulation_test_and_health_shows_fixture_env(world):
    from fastapi.testclient import TestClient
    from orchestrator.api import create_app
    from .conftest import submit
    orch = world["orch"]
    with TestClient(create_app(orch)) as c:                  # startup → recover() 실행
        h = c.get("/health").json()
        assert h["truth_env"]["source"] == "FIXTURE" and h["truth_env"]["shared_team_env"] is False
        t = submit(orch)
        orch.dispatch(t.task_id)
        for _ in range(3):
            orch.poll()
        comp = c.get(f"/tasks/{t.task_id}").json()["observation"]["completion"]
        assert comp["evidence_level"] == "SIMULATION_TEST"
        assert comp["evidence"] == {"observation_source": "SIMULATED", "env_source": "FIXTURE", "env_ack": True}
    assert [e for e in world["ledger"].events() if e["event_type"] == "RECOVERY_SCAN"]
