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
    """환경 상태/사건으로부터 Task를 생성한다 (기본 실행 흐름 1단계)"""
    task_id = ids.new_task_id()
    if env_state.fire_cells:
        cell = env_state.fire_cells[0]
        description = f"화재 셀({cell['x']},{cell['y']}) 관측"
        target_lat, target_lon = config.COORDINATE_ORIGIN_LAT, config.COORDINATE_ORIGIN_LON
    else:
        description = "관측 대상 없음"
        target_lat, target_lon = config.COORDINATE_ORIGIN_LAT, config.COORDINATE_ORIGIN_LON

    return Task(
        task_id=task_id,
        created_at=timestamp,
        state="READY",
        description=description,
        target_lat=target_lat,
        target_lon=target_lon,
    )


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
            )
        )

    return Decision(decision_id=decision_id, task_id=task.task_id, timestamp=timestamp, assignments=assignments)