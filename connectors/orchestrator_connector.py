# -*- coding: utf-8 -*-
"""
connectors/orchestrator_connector.py
======================================
오케스트레이터(박수진님)의 Task 생성·판단 로직을 대신하는 Mock입니다.
실제 판단(Task Requirement, Resource Capability/State 비교, 자원 후보
평가와 선택)은 박수진님 담당이며, 여기서는 그 자리를 흉내만 냅니다.

Decision ID 원칙 (Contract):
  REJECT/COUNTER/Safety REJECT 이후 다시 자원을 선택하는 재평가가
  발생하면 새로운 decision_id를 부여한다.
"""

import random
import config
import ids
from interfaces.schema import Task, Decision, Assignment


def create_task(timestamp: float, env_state) -> Task:
    """환경 상태/사건으로부터 Task를 생성한다 (기본 실행 흐름 1단계)

    타깃 확정 규칙 (fail-closed):
      - 화재셀이 있으면 위험도 최고 셀을 gz_bridge 로 위경도 + 지면고도(DEM)로 변환한다.
      - 변환에 실패하면 real 모드에서는 임의 원점으로 대체하지 않고
        target_resolved=False(TARGET_UNRESOLVED)로 돌려준다. main 이 배정 없이 FAILED 처리한다.
      - mock 모드에서만 설정 원점을 명시적 placeholder(target_source=MOCK_PLACEHOLDER)로 허용한다.
      - 화재셀이 없으면 target_resolved=False(NO_FIRE_TARGET) → CANCELLED.
    """
    task_id = ids.new_task_id()
    kw = dict(task_id=task_id, created_at=timestamp, state="READY")

    if not env_state.fire_cells:
        return Task(**kw, description="관측 대상 없음",
                    target_resolved=False, unresolved_reason="NO_FIRE_TARGET")

    cell = max(env_state.fire_cells, key=lambda c: c.get("risk_score", 0.0))
    description = f"화재 셀({cell['x']},{cell['y']}) 관측"
    try:
        import gz_bridge
        # clearance_m=0 → 지면 고도(AMSL). 여유고도는 UAV Local Agent 가 한 번만 더한다.
        lat, lon, ground_alt = gz_bridge.fire_cell_to_target(cell, clearance_m=0.0)
        return Task(**kw, description=description, target_lat=lat, target_lon=lon,
                    target_alt_m=ground_alt, target_source="DEM")
    except Exception as e:
        if config.FIRE_CONNECTION_MODE == "mock":
            return Task(**kw, description=description + " [mock placeholder]",
                        target_lat=config.COORDINATE_ORIGIN_LAT,
                        target_lon=config.COORDINATE_ORIGIN_LON,
                        target_source="MOCK_PLACEHOLDER")
        print(f"[orchestrator] 화재셀 좌표 확정 실패 → 배정 중단: {e}")
        return Task(**kw, description=description,
                    target_resolved=False, unresolved_reason="TARGET_UNRESOLVED")


def propose(timestamp: float, task: Task, env_state, resource_pool, exclude_ids: set = None) -> Decision:
    """오케스트레이터의 후보 평가와 자원 선택 (1차 제안)"""
    if config.ORCHESTRATOR_CONNECTION_MODE == "mock":
        return _propose_mock(timestamp, task, resource_pool, exclude_ids or set())
    raise NotImplementedError("오케스트레이터 실제 연동이 아직 구현되지 않았습니다.")


def reevaluate(timestamp: float, task: Task, env_state, resource_pool, exclude_ids: set) -> Decision:
    """REJECT/COUNTER/Safety REJECT 이후 재평가 — 반드시 새 decision_id 부여"""
    if config.ORCHESTRATOR_CONNECTION_MODE == "mock":
        return _propose_mock(timestamp, task, resource_pool, exclude_ids)
    raise NotImplementedError("오케스트레이터 재평가 로직이 아직 구현되지 않았습니다.")


def _propose_mock(timestamp: float, task: Task, resource_pool, exclude_ids: set) -> Decision:
    decision_id = ids.new_decision_id()

    candidates = [
        r for r in resource_pool.by_type("UAV")
        if r.state == "READY" and r.resource_id not in exclude_ids
    ]

    assignments = []
    if candidates:
        chosen = candidates[0]  # Mock: 첫 번째 후보 선택 (결정론적 — 실제 평가 로직은 박수진님 모듈 완성 후 대체)
        assignments.append(
            Assignment(
                task_id=task.task_id,
                resource_id=chosen.resource_id,
                target_lat=task.target_lat,
                target_lon=task.target_lon,
                task_type="OBSERVE",
                target_alt_m=getattr(task, "target_alt_m", 0.0),
            )
        )

    return Decision(decision_id=decision_id, task_id=task.task_id, simulation_time_s=timestamp, assignments=assignments)