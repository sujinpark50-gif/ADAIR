# -*- coding: utf-8 -*-
"""
connectors/safety_connector.py
================================
Safety Connector (Contract 7번 "Safety 연결"):
- Local Agent가 ACCEPT한 명령만 검증
- ALLOW / MODIFY / REJECT 반환

주의: Safety REJECT가 나오면 main.py에서 총괄 재평가로 돌아간다
(Contract Connector 최소 책임 7번: "REJECT/COUNTER/Safety REJECT 후 재평가").
MODIFY의 세부 수정 로직은 Integration Test Cases 문서상 초기 필수 범위가
아니므로, 여기서는 자리만 만들고 REJECT와 동일하게 재평가로 넘긴다.
"""

import config
from interfaces.schema import Decision, SafetyVerdict


def verify(decision: Decision) -> SafetyVerdict:
    if config.ORCHESTRATOR_CONNECTION_MODE == "mock":
        return _verify_mock(decision)
    raise NotImplementedError("Safety Layer 실제 검증 로직이 아직 구현되지 않았습니다.")


def _verify_mock(decision: Decision) -> SafetyVerdict:
    """Mock: 배정이 있으면 ALLOW. (세부 임계값은 Contract 12번 TBD 항목)"""
    if decision.assignments:
        return SafetyVerdict(response="ALLOW")
    return SafetyVerdict(response="REJECT", reason="NO_ASSIGNMENT")