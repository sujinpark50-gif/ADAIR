# -*- coding: utf-8 -*-
"""
logger.py
=========
Contract 10번(로그 기준)과 Integration Test Cases 2번(공통 기록 항목)을
그대로 반영한 이벤트 단위 로거입니다.

이전 버전과의 가장 큰 차이:
- 이전: "스텝 하나 = 로그 한 줄"에 모든 정보를 욱여넣음
- 지금: "이벤트 하나 = 로그 한 줄". 한 스텝 안에서도 여러 줄이 남는다.
  (환경 변화 / Task 생성 / 후보 평가 / 자원 선택 / Local 응답 / Safety 판단 /
   실행 / 관측·결과 / 재평가 또는 재할당 / 완료·실패)

"테스트 성공 여부는 화면이 아니라 상태·메시지·로그로 확인할 수 있어야 한다"
(Integration Test Cases 1번)는 원칙에 따라, 최소 필드만으로도
IT-01/IT-02 성공 조건을 로그만 보고 판정할 수 있게 만든다.
"""

import json
import os
from dataclasses import asdict, is_dataclass
from datetime import datetime

import config


def _to_jsonable(value):
    """dataclass는 dict로, 나머지는 그대로 — JSON 직렬화 가능하게 변환"""
    if is_dataclass(value):
        return asdict(value)
    return value


class EventLogger:
    """1 Run = 1 Log File 원칙에 따라, 실행(run_id)마다 파일을 새로 만든다."""

    def __init__(
        self,
        run_id: str,
        scenario_id: str = None,
        log_dir: str = None,
        file_name: str = None,
        console: bool = False,
    ):
        self.run_id = run_id
        self.scenario_id = scenario_id or config.SCENARIO_ID
        self.log_dir = log_dir or config.LOG_DIR
        self.file_name = file_name or config.LOG_FILE_NAME
        self.console = console
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, self.file_name)
        # 1 Run = 1 Log File: 같은 이름으로 데모를 다시 실행해도
        # 이전 run의 레코드가 섞이지 않도록 새 로그로 시작한다.
        with open(self.log_path, "w", encoding="utf-8"):
            pass

    def log_event(
        self,
        event_type: str,
        timestamp: float,
        result: str = "",
        reason: str = None,
        decision_id: str = None,
        task_id: str = None,
        resource_id: str = None,
        detail: dict = None,
    ):
        """
        Integration Test Cases 2번 "공통 기록 항목" 최소 필드:
        scenario_id, run_id, decision_id, task_id, resource_id, timestamp
        + event_type, result, reason(해당 시)
        """
        record = {
            "scenario_id": self.scenario_id,
            "run_id": self.run_id,
            "decision_id": decision_id,
            "task_id": task_id,
            "resource_id": resource_id,
            "timestamp": timestamp,
            "logged_at": datetime.now().isoformat(),
            "event_type": event_type,
            "result": result,
            "reason": reason,
        }
        if detail:
            record["detail"] = {k: _to_jsonable(v) for k, v in detail.items()}

        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

        if self.console:
            print(self._format_console(record))

    @staticmethod
    def _format_console(record: dict) -> str:
        """시연에서 폐루프를 한눈에 볼 수 있는 짧은 이벤트 로그."""
        event = record["event_type"]
        result = record["result"]
        resource = record.get("resource_id")
        decision = record.get("decision_id")
        reason = record.get("reason")
        detail = record.get("detail", {})

        if event == "ENVIRONMENT_CHANGE":
            return f"[ENV] FIRE_DETECTED cell={detail.get('cell_id', 'unknown')}"
        if event == "TASK_CREATED":
            return f"[TASK] {record['task_id']} CREATED"
        if event == "CANDIDATE_EVALUATED":
            selected = detail.get("assignments", ["NONE"])[0]
            return f"[ORCH] {decision} -> {selected} assigned"
        if event == "LOCAL_RESPONSE":
            suffix = f" reason={reason}" if reason else ""
            return f"[{resource}] {result}{suffix}"
        if event == "REEVALUATION":
            return f"[ORCH] reevaluate -> {decision} reason={reason}"
        if event == "SAFETY_JUDGEMENT":
            return f"[SAFETY] {result}"
        if event == "EXECUTION":
            return f"[EXEC] {resource or 'mission'} started"
        if event == "OBSERVATION":
            return f"[OBS] {resource} target reached"
        if event == "ENVIRONMENT_UPDATE":
            return "[ENV] observation applied"
        if event == "TASK_COMPLETE":
            return f"[TASK] {result}"
        return f"[{event}] {result}"

    def read_all(self):
        if not os.path.exists(self.log_path):
            return []
        records = []
        with open(self.log_path, "r", encoding="utf-8") as f:
            for line in f:
                if line.strip():
                    records.append(json.loads(line))
        return records
