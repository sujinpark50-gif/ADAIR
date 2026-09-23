# -*- coding: utf-8 -*-
"""
main.py
=======
D(시스템 통합·관제·로그) 파트의 핵심 실행 파일.
Integration Contract 3번 "기본 실행 흐름"을 그대로 코드로 옮겼습니다.

    환경 상태/사건
       ↓
    Task 생성 또는 기존 Task 재평가
       ↓
    Orchestrator 후보 자원 평가
       ↓
    자원 선택 및 제안
       ↓
    Local Agent 수행 가능성 확인 ── ACCEPT ──→ Safety Layer ── ALLOW ──→ 실행
       │                                          │├─ MODIFY → (초기 범위 제외, REJECT처럼 처리)
       │                                          └─ REJECT → 총괄 재평가
       ├── REJECT ──→ 총괄 재평가
       └── COUNTER ─→ 총괄 재평가 (초기 범위 제외 — 대안 검토 로직 미구현, REJECT처럼 처리)
       ↓
    실행 → 상태·관측·결과 반환 → 환경 모델 갱신 → 필요 시 재평가·재할당

process_task()가 한 Task의 전체 흐름을 처리하며, IT-01/IT-02 테스트에서도
이 함수를 그대로 재사용합니다 (tests/it_cases.py 참고).
"""

import config
import ids
from interfaces.schema import ResourcePool
from connectors import fire_connector, uav_connector, ugv_connector, orchestrator_connector, safety_connector
from logger import EventLogger


def get_all_resources(timestamp: float) -> ResourcePool:
    return ResourcePool(
        base_a=uav_connector.get_uav_status("A") + ugv_connector.get_ugv_status("A"),
        base_b=uav_connector.get_uav_status("B") + ugv_connector.get_ugv_status("B"),
    )


def _send_command(task_id, decision_id, assignment, env_state, force_response=None, force_reason=None):
    if "uav" in assignment.resource_id:
        # PX4/Gazebo가 풍속을 안 주므로, 환경모델 값을 UAV 판단 요청에 실어 보낸다
        wind_ms = getattr(env_state, "wind_speed", None)
        return uav_connector.send_uav_command(task_id, decision_id, assignment, wind_ms, force_response, force_reason)
    return ugv_connector.send_ugv_command(task_id, assignment, force_response, force_reason)


def _get_observation(resource_id, timestamp, task_id, decision_id, assignment):
    if "uav" in resource_id:
        return uav_connector.get_uav_observation(resource_id, timestamp, task_id, decision_id, assignment)
    return ugv_connector.get_ugv_observation(resource_id, timestamp, task_id, decision_id)


def _refresh_for_reevaluation(timestamp, env_state, resource_pool):
    """재평가 직전에 환경·자원 상태를 다시 읽는다 (R04).

    환경은 advance=False 로 읽어 재조회가 불 확산을 추가로 일으키지 않게 한다.
    조회에 실패하면 이전 상태를 그대로 쓴다(재평가 자체는 계속).
    """
    try:
        env_state = fire_connector.get_environment_state(timestamp, advance=False)
    except Exception as e:
        print(f"[main] 재평가용 환경 재조회 실패, 이전 상태 사용: {e}")
    try:
        resource_pool = get_all_resources(timestamp)
    except Exception as e:
        print(f"[main] 재평가용 자원 재조회 실패, 이전 상태 사용: {e}")
    return env_state, resource_pool


def process_task(task, env_state, resource_pool, logger: EventLogger, timestamp: float, forced_responses: dict = None):
    """
    Task 하나를 '자원 선택 → Local 확인 → Safety → 실행 → 관측 → 환경 갱신'까지
    끝까지 처리한다. forced_responses는 테스트에서 특정 시나리오를 강제할 때 쓴다.
    형식: {resource_id: (response, reason)}
    """
    forced_responses = forced_responses or {}
    logger.log_event("TASK_CREATED", timestamp, task_id=task.task_id, result=task.state,
                     detail={"target_source": getattr(task, "target_source", "")})

    # fail-closed: 타깃 좌표를 확정하지 못한 Task 는 자원에 배정하지 않는다
    if not getattr(task, "target_resolved", True):
        reason = getattr(task, "unresolved_reason", None) or "TARGET_UNRESOLVED"
        task.state = "CANCELLED" if reason == "NO_FIRE_TARGET" else "FAILED"
        logger.log_event("TASK_COMPLETE", timestamp, task_id=task.task_id, result=task.state, reason=reason)
        return task, env_state

    exclude_ids = set()
    decision = orchestrator_connector.propose(timestamp, task, env_state, resource_pool, exclude_ids)
    logger.log_event(
        "CANDIDATE_EVALUATED", timestamp,
        decision_id=decision.decision_id, task_id=task.task_id,
        result="PROPOSED", detail={"assignments": [a.resource_id for a in decision.assignments]},
    )

    reevaluation_count = 0

    while True:
        if not decision.assignments:
            task.state = "FAILED"
            logger.log_event("TASK_COMPLETE", timestamp, task_id=task.task_id, result="FAILED", reason="NO_CANDIDATE")
            return task, env_state

        assignment = decision.assignments[0]
        forced = forced_responses.get(assignment.resource_id)
        force_response, force_reason = forced if forced else (None, None)

        local_response = _send_command(task.task_id, decision.decision_id, assignment, env_state, force_response, force_reason)
        logger.log_event(
            "LOCAL_RESPONSE", timestamp,
            decision_id=decision.decision_id, task_id=task.task_id, resource_id=assignment.resource_id,
            result=local_response.response, reason=local_response.reason,
        )

        if local_response.response != "ACCEPT":
            # REJECT 또는 COUNTER(초기 범위 제외 — REJECT와 동일하게 처리) → 총괄 재평가
            exclude_ids.add(assignment.resource_id)
            reevaluation_count += 1
            if reevaluation_count > config.MAX_REEVALUATION_RETRIES:
                task.state = "FAILED"
                logger.log_event("TASK_COMPLETE", timestamp, task_id=task.task_id, result="FAILED", reason="MAX_REEVALUATION_EXCEEDED")
                return task, env_state
            env_state, resource_pool = _refresh_for_reevaluation(timestamp, env_state, resource_pool)
            decision = orchestrator_connector.reevaluate(timestamp, task, env_state, resource_pool, exclude_ids)
            logger.log_event(
                "REEVALUATION", timestamp,
                decision_id=decision.decision_id, task_id=task.task_id,
                result="NEW_CANDIDATE", reason=local_response.reason,
            )
            continue

        # ACCEPT된 것만 Safety Layer로 전달
        verdict = safety_connector.verify(decision)
        decision.safety_verdict = verdict
        logger.log_event(
            "SAFETY_JUDGEMENT", timestamp,
            decision_id=decision.decision_id, task_id=task.task_id,
            result=verdict.response, reason=verdict.reason,
        )

        if verdict.response == "ALLOW":
            task.state = "RUNNING"
            logger.log_event(
                "EXECUTION", timestamp,
                decision_id=decision.decision_id, task_id=task.task_id,
                resource_id=assignment.resource_id, result="STARTED",
            )

            observation = _get_observation(assignment.resource_id, timestamp, task.task_id, decision.decision_id, assignment)
            logger.log_event(
                "OBSERVATION", timestamp,
                decision_id=decision.decision_id, task_id=task.task_id, resource_id=assignment.resource_id,
                result="RECEIVED",
            )

            env_state = fire_connector.update_environment(env_state, [observation])
            logger.log_event("ENVIRONMENT_UPDATE", timestamp, task_id=task.task_id, result="UPDATED")

            task.state = "COMPLETED"
            logger.log_event("TASK_COMPLETE", timestamp, task_id=task.task_id, result="COMPLETED")
            return task, env_state

        # Safety REJECT 또는 MODIFY(초기 범위 제외 — REJECT와 동일 처리) → 총괄 재평가
        exclude_ids.add(assignment.resource_id)
        reevaluation_count += 1
        if reevaluation_count > config.MAX_REEVALUATION_RETRIES:
            task.state = "FAILED"
            logger.log_event("TASK_COMPLETE", timestamp, task_id=task.task_id, result="FAILED", reason="SAFETY_REJECT_MAX_RETRY")
            return task, env_state
        env_state, resource_pool = _refresh_for_reevaluation(timestamp, env_state, resource_pool)
        decision = orchestrator_connector.reevaluate(timestamp, task, env_state, resource_pool, exclude_ids)
        logger.log_event(
            "REEVALUATION", timestamp,
            decision_id=decision.decision_id, task_id=task.task_id,
            result="NEW_CANDIDATE", reason="SAFETY_" + verdict.response,
        )


def run_simulation(total_steps: int = None):
    total_steps = total_steps or config.DEMO_TOTAL_STEPS
    run_id = ids.new_run_id()
    logger = EventLogger(run_id=run_id)

    print(f"=== 시뮬레이션 시작 (run_id={run_id}, 총 {total_steps} 스텝, Mock 모드) ===\n")

    for step in range(total_steps):
        timestamp = step * config.SIMULATION_TIMESTEP_SEC

        env_state = fire_connector.get_environment_state(timestamp)
        logger.log_event("ENVIRONMENT_CHANGE", timestamp, result="OK")

        resource_pool = get_all_resources(timestamp)
        task = orchestrator_connector.create_task(timestamp, env_state)

        task, env_state = process_task(task, env_state, resource_pool, logger, timestamp)

        print(f"[Step {step}] task={task.task_id} state={task.state}")

    print(f"\n=== 시뮬레이션 종료. 로그 저장 위치: {logger.log_path} ===")


if __name__ == "__main__":
    run_simulation()
