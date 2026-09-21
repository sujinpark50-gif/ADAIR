# -*- coding: utf-8 -*-
"""
ids.py
======
Integration Contract 4번(공통 메시지 식별 정보)과 "Decision ID 원칙"에서
요구하는 식별자들을 일관된 형식으로 생성하는 유틸입니다.

형식은 문서의 예시(DEC_001, DEC_002)를 그대로 따릅니다.
"""

import itertools
import time

_task_counter = itertools.count(1)
_decision_counter = itertools.count(1)
_message_counter = itertools.count(1)


def new_run_id() -> str:
    """시뮬레이션 실행(run) 하나를 식별하는 ID. 1 Run = 1 Log File 원칙과 연결됨"""
    return f"RUN_{int(time.time())}"


def new_task_id() -> str:
    return f"TASK_{next(_task_counter):03d}"


def new_decision_id() -> str:
    """
    REJECT 이후 재평가가 발생하면 반드시 새 decision_id를 부여해야 한다.
    (Contract "Decision ID 원칙" — 예: DEC_001 REJECT 후 재평가는 DEC_002)
    """
    return f"DEC_{next(_decision_counter):03d}"


def new_message_id() -> str:
    return f"MSG_{next(_message_counter):06d}"