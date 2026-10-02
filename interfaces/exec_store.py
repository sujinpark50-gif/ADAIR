# -*- coding: utf-8 -*-
"""실행 기록 저장소 — UAV-01 · UGV-02 · UGV-04 (총괄 요청서) 공용.

UAV Agent(uav/uav-agent/main.py)와 UGV 서버(ugv/server.py)가 같은 규칙을 쓰도록 한 곳에 둔다.

규칙
- 실행 키 = 요청의 task_id (총괄은 이 칸에 실행시도 ID(attempt_id)를 넣는다).
- 같은 키·같은 내용   → 새로 출동하지 않고 기존 실행을 돌려준다 (DUPLICATE).
- 같은 키·다른 내용   → 충돌 (CONFLICT). 호출 측이 409 로 거절하고 기존 실행은 그대로 둔다.
- 실행이 실제로 시작된 요청만 기록한다. 시작 전에 거절한 요청(400·409·422)은 키를 쓰지 않는다
  → 고쳐서 같은 키로 다시 보낼 수 있다.
- 기록은 JSON 파일에 원자적으로 저장한다 (임시 파일 → os.replace). 프로세스를 다시 켜도 같은 키로
  조회·재요청하면 같은 결과가 나온다.
- 재시작으로 끊긴 실행은 reconcile_after_restart() 로 '끝난 것'으로 정리한다. 이때 물리 상태를 아는 척하지 않는다:
  끊긴 사실과 근거(physical_basis)를 기록에 남기고, 무엇으로 바꿀지는 호출 측(UAV/UGV)이 정한다.
"""

from __future__ import annotations

import hashlib
import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional, Tuple


def body_hash(body: dict) -> str:
    return hashlib.sha256(json.dumps(body, sort_keys=True, ensure_ascii=False, default=str).encode()).hexdigest()


class ExecStore:
    VERSION = 1

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self.tasks: dict[str, dict] = {}       # 실행 키 → 상태 dict (TaskStatus 내용, 호출 측이 직접 고친다)
        self.meta: dict[str, dict] = {}        # 실행 키 → {hash, response, created_wall}
        self.restarts = 0
        if self.path.is_file():
            doc = json.loads(self.path.read_text(encoding="utf-8"))
            self.tasks = doc.get("tasks", {})
            self.meta = doc.get("meta", {})
            self.restarts = int(doc.get("restarts", 0)) + 1
            self.loaded_from_disk = True
        else:
            self.loaded_from_disk = False

    # ------------------------------------------------------------------
    def check(self, key: str, body: dict) -> Tuple[str, Optional[dict]]:
        """("NEW"|"DUPLICATE"|"CONFLICT", 기존 meta)"""
        with self._lock:
            m = self.meta.get(key)
            if m is None:
                return "NEW", None
            return ("DUPLICATE" if m["hash"] == body_hash(body) else "CONFLICT"), m

    def register(self, key: str, body: dict, task: dict, response: dict) -> None:
        with self._lock:
            self.tasks[key] = task
            self.meta[key] = {"hash": body_hash(body), "response": response,
                              "created_wall": datetime.now(timezone.utc).isoformat()}
            self.save()

    def save(self) -> None:
        with self._lock:
            doc = {"version": self.VERSION, "tasks": self.tasks, "meta": self.meta, "restarts": self.restarts,
                   "saved_wall": datetime.now(timezone.utc).isoformat()}
            tmp = self.path.with_suffix(self.path.suffix + ".tmp")
            tmp.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
            os.replace(tmp, self.path)

    def reconcile_after_restart(self, is_active: Callable[[dict], bool],
                                close: Callable[[dict], None], basis: str) -> list:
        """이전 프로세스에서 진행 중이던 실행을 정리한다. close(task) 가 상태를 바꾼다."""
        if not self.loaded_from_disk:
            return []
        closed = []
        with self._lock:
            for key, t in self.tasks.items():
                if is_active(t):
                    close(t)
                    t["agent_restart"] = {"restart_no": self.restarts, "physical_basis": basis,
                                          "at_wall": datetime.now(timezone.utc).isoformat()}
                    closed.append(key)
            self.save()
        return closed
