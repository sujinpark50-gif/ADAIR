# -*- coding: utf-8 -*-
"""
tests/it_cases.py
==================
"02. Integration Test Cases" 문서의 IT-01, IT-02를 그대로 실행 가능한
코드로 옮긴 것입니다. 화면이 아니라 로그(및 반환된 task 상태)로
성공 여부를 판정합니다 (문서 1번 원칙).

실행: python3 -m tests.it_cases   (프로젝트 루트에서)
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
import ids
from interfaces.schema import ResourcePool
from connectors import fire_connector, uav_connector, orchestrator_connector
from logger import EventLogger
from main import process_task


def _make_resource_pool_uav_only():
    """테스트용: A거점 UAV 2대만 있는 자원풀 (get_uav_status의 결정론적 ID: A-uav1, A-uav2)"""
    return ResourcePool(base_a=uav_connector.get_uav_status("A"), base_b=[])


def test_it_01_normal_execution():
    """
    IT-01. 정상 임무 수행
    입력: 화재 발생, 사용 가능한 UAV 존재, UAV 상태 정상, Safety 조건 정상
    기대: Environment→Task→Orchestrator후보평가→UAV선택→ACCEPT→ALLOW→실행→관측→환경갱신→Task완료
    """
    run_id = ids.new_run_id()
    logger = EventLogger(run_id=run_id, scenario_id="IT-01", file_name="it_01.jsonl")

    timestamp = 0.0
    env_state = fire_connector.get_environment_state(timestamp)
    resource_pool = _make_resource_pool_uav_only()
    task = orchestrator_connector.create_task(timestamp, env_state)

    # UAV 상태 정상, Safety 조건 정상 → 두 후보 모두 ACCEPT 강제
    forced = {"A-uav1": ("ACCEPT", None), "A-uav2": ("ACCEPT", None)}

    task, env_state = process_task(task, env_state, resource_pool, logger, timestamp, forced_responses=forced)

    records = logger.read_all()
    event_types = [r["event_type"] for r in records]

    assert task.state == "COMPLETED", f"IT-01 실패: task.state={task.state}"
    assert "LOCAL_RESPONSE" in event_types
    assert any(r["event_type"] == "LOCAL_RESPONSE" and r["result"] == "ACCEPT" for r in records)
    assert any(r["event_type"] == "SAFETY_JUDGEMENT" and r["result"] == "ALLOW" for r in records)
    assert "EXECUTION" in event_types
    assert "OBSERVATION" in event_types
    assert "ENVIRONMENT_UPDATE" in event_types
    assert any(r["event_type"] == "TASK_COMPLETE" and r["result"] == "COMPLETED" for r in records)

    print("[PASS] IT-01. 정상 임무 수행")


def test_it_02_reject_then_reevaluate():
    """
    IT-02. UAV 강풍 REJECT 후 재평가
    입력: UAV-A가 최초 후보, UAV-A가 HIGH_WIND로 임무 수행 불가, 다른 가용 자원 존재
    기대: DEC_001(UAV-A 제안→REJECT/HIGH_WIND→Safety 호출 안 함)
          → DEC_002(재평가→다른 후보 선택→ACCEPT시 Safety→실행)
    """
    run_id = ids.new_run_id()
    logger = EventLogger(run_id=run_id, scenario_id="IT-02", file_name="it_02.jsonl")

    timestamp = 0.0
    env_state = fire_connector.get_environment_state(timestamp)
    resource_pool = _make_resource_pool_uav_only()
    task = orchestrator_connector.create_task(timestamp, env_state)

    # A-uav1(=UAV-A, 첫 후보)은 강풍으로 REJECT, A-uav2는 ACCEPT
    forced = {
        "A-uav1": ("REJECT", "HIGH_WIND"),
        "A-uav2": ("ACCEPT", None),
    }

    task, env_state = process_task(task, env_state, resource_pool, logger, timestamp, forced_responses=forced)

    records = logger.read_all()

    reject_records = [r for r in records if r["event_type"] == "LOCAL_RESPONSE" and r["result"] == "REJECT"]
    accept_records = [r for r in records if r["event_type"] == "LOCAL_RESPONSE" and r["result"] == "ACCEPT"]
    safety_records = [r for r in records if r["event_type"] == "SAFETY_JUDGEMENT"]
    reeval_records = [r for r in records if r["event_type"] == "REEVALUATION"]

    # decision_id가 첫 제안과 재평가에서 달라야 한다 (Decision ID 원칙)
    decision_ids_used = [r["decision_id"] for r in records if r.get("decision_id")]
    first_decision_id = reject_records[0]["decision_id"] if reject_records else None
    second_decision_id = accept_records[0]["decision_id"] if accept_records else None

    assert task.state == "COMPLETED", f"IT-02 실패: task.state={task.state}"
    assert len(reject_records) == 1 and reject_records[0]["resource_id"] == "A-uav1"
    assert reject_records[0]["reason"] == "HIGH_WIND"
    # REJECT 이후 그 decision_id로는 Safety가 호출되지 않아야 함
    assert not any(s["decision_id"] == first_decision_id for s in safety_records), "REJECT 이후 Safety가 호출됨 (계약 위반)"
    assert len(reeval_records) >= 1
    assert first_decision_id != second_decision_id, "재평가 시 새 decision_id가 부여되지 않음"
    assert any(r["result"] == "ALLOW" and r["decision_id"] == second_decision_id for r in safety_records), \
        "재평가된 decision에서 Safety ALLOW가 확인되지 않음"

    print("[PASS] IT-02. UAV 강풍 REJECT 후 재평가")


if __name__ == "__main__":
    test_it_01_normal_execution()
    test_it_02_reject_then_reevaluate()
    print("\n두 테스트 모두 통과했습니다.")