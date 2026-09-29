# -*- coding: utf-8 -*-
"""
orchestrator/ledger.py
======================
SQLite 장부. 서버를 껐다 켜도 Task·판단·실행시도·자원 점유를 잊지 않는다.

원칙 (요청서 §5, §6):
- 같은 요청 키로 다시 접수해도 Task 는 하나로 수렴한다. 내용이 다르면 거절한다.
- 자원 예약은 원자적이다. 두 Task 가 같은 자원을 동시에 잡을 수 없다.
- 실행 명령은 송신 "전에" 저장한다.
- UNKNOWN 인 실행시도의 자원은 해제하지 않는다.
- 모든 사건은 events 표에 순서대로 남긴다 (JSONL 로그와 별개).
"""

import hashlib
import json
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Iterable, List, Optional

from .models import Task

_SCHEMA = """
CREATE TABLE IF NOT EXISTS tasks (
    task_id TEXT PRIMARY KEY,
    incident_id TEXT NOT NULL,
    request_key TEXT UNIQUE,
    request_hash TEXT,
    purpose_status TEXT NOT NULL,
    body TEXT NOT NULL,
    created_wall REAL NOT NULL,
    updated_wall REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS decisions (
    decision_id TEXT PRIMARY KEY,
    task_id TEXT NOT NULL,
    parent_decision_id TEXT,
    created_wall REAL NOT NULL,
    body TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS attempts (
    attempt_id TEXT PRIMARY KEY,
    task_id TEXT NOT NULL,
    decision_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    command TEXT NOT NULL,
    command_hash TEXT NOT NULL,
    substatus TEXT NOT NULL,
    detail TEXT,
    created_wall REAL NOT NULL,
    updated_wall REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS reservations (
    resource_id TEXT PRIMARY KEY,
    task_id TEXT NOT NULL,
    attempt_id TEXT NOT NULL,
    created_wall REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS events (
    seq INTEGER PRIMARY KEY AUTOINCREMENT,
    wall_time REAL NOT NULL,
    sim_time_s REAL,
    event_type TEXT NOT NULL,
    task_id TEXT,
    decision_id TEXT,
    attempt_id TEXT,
    resource_id TEXT,
    result TEXT,
    reason TEXT,
    detail TEXT
);
CREATE TABLE IF NOT EXISTS knowledge (
    seq INTEGER PRIMARY KEY AUTOINCREMENT,
    kind TEXT NOT NULL,
    key TEXT NOT NULL UNIQUE,
    subject TEXT,
    sim_time_s REAL,
    source TEXT,
    body TEXT NOT NULL,
    wall_time REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS ix_events_task ON events(task_id);
CREATE INDEX IF NOT EXISTS ix_attempts_task ON attempts(task_id);
"""

# 이 상태의 실행시도는 자원을 계속 점유한다
OCCUPYING_SUBSTATUSES = {"PREPARED", "REQUESTED", "STARTED", "ARRIVED", "OBSERVED",
                         "APPLIED", "RETURNING", "UNKNOWN", "FAULTED"}


class RequestConflict(Exception):
    """같은 요청 키에 다른 내용이 들어옴"""


class ReservationConflict(Exception):
    """이미 다른 실행시도가 점유한 자원"""


def new_id(prefix: str) -> str:
    return f"{prefix}-{uuid.uuid4().hex[:12]}"


def content_hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, ensure_ascii=False).encode("utf-8")).hexdigest()


def _dumps(obj) -> Optional[str]:
    return None if obj is None else json.dumps(obj, ensure_ascii=False, default=str)


def _loads(s):
    return None if s is None else json.loads(s)


class Ledger:
    def __init__(self, db_path: str):
        if db_path != ":memory:":
            Path(db_path).parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False, isolation_level=None)
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA journal_mode=WAL" if db_path != ":memory:" else "PRAGMA journal_mode=MEMORY")
        self._conn.executescript(_SCHEMA)
        self._lock = threading.RLock()

    def close(self):
        self._conn.close()

    def _tx(self):
        return _Tx(self)

    # ------------------------------------------------------------------
    # Tasks
    # ------------------------------------------------------------------
    def create_task(self, task: Task, request_body: dict) -> (Task, bool):
        """(task, created). 같은 request_key 가 있으면 기존 Task 반환(created=False)."""
        req_hash = content_hash(request_body)
        now = time.time()
        with self._tx() as c:
            if task.request_key:
                row = c.execute("SELECT body, request_hash FROM tasks WHERE request_key=?",
                                (task.request_key,)).fetchone()
                if row:
                    if row["request_hash"] != req_hash:
                        raise RequestConflict(task.request_key)
                    return Task.from_dict(json.loads(row["body"])), False
            task.created_wall = task.created_wall or now
            c.execute("INSERT INTO tasks VALUES (?,?,?,?,?,?,?,?)",
                      (task.task_id, task.incident_id, task.request_key, req_hash,
                       task.purpose_status, _dumps(task.to_dict()), task.created_wall, now))
        return task, True

    def save_task(self, task: Task):
        with self._tx() as c:
            c.execute("UPDATE tasks SET purpose_status=?, body=?, updated_wall=? WHERE task_id=?",
                      (task.purpose_status, _dumps(task.to_dict()), time.time(), task.task_id))

    def get_task(self, task_id: str) -> Optional[Task]:
        row = self._conn.execute("SELECT body FROM tasks WHERE task_id=?", (task_id,)).fetchone()
        return Task.from_dict(json.loads(row["body"])) if row else None

    def list_tasks(self, statuses: Optional[Iterable[str]] = None) -> List[Task]:
        rows = self._conn.execute("SELECT body FROM tasks ORDER BY created_wall, task_id").fetchall()
        tasks = [Task.from_dict(json.loads(r["body"])) for r in rows]
        if statuses is not None:
            statuses = set(statuses)
            tasks = [t for t in tasks if t.purpose_status in statuses]
        return tasks

    # ------------------------------------------------------------------
    # Decisions (재평가마다 새 decision, 같은 Task 유지)
    # ------------------------------------------------------------------
    def add_decision(self, task_id: str, body: dict, parent_decision_id: Optional[str] = None) -> str:
        did = new_id("DEC")
        with self._tx() as c:
            c.execute("INSERT INTO decisions VALUES (?,?,?,?,?)",
                      (did, task_id, parent_decision_id, time.time(), _dumps(body)))
        return did

    def update_decision(self, decision_id: str, **fields):
        with self._tx() as c:
            row = c.execute("SELECT body FROM decisions WHERE decision_id=?", (decision_id,)).fetchone()
            body = json.loads(row["body"])
            body.update(fields)
            c.execute("UPDATE decisions SET body=? WHERE decision_id=?", (_dumps(body), decision_id))

    def list_decisions(self, task_id: str) -> List[dict]:
        rows = self._conn.execute(
            "SELECT * FROM decisions WHERE task_id=? ORDER BY created_wall, rowid", (task_id,)).fetchall()
        return [{"decision_id": r["decision_id"], "parent_decision_id": r["parent_decision_id"],
                 "created_wall": r["created_wall"], **json.loads(r["body"])} for r in rows]

    # ------------------------------------------------------------------
    # 실행시도 + 원자 예약 (요청서 §5 순서 4: ALLOW 후 예약·선저장, 실패 시 송신 금지)
    # ------------------------------------------------------------------
    def prepare_attempt(self, attempt_id: str, task_id: str, decision_id: str,
                        resource_id: str, command: dict) -> dict:
        now = time.time()
        with self._tx() as c:
            held = c.execute("SELECT attempt_id, task_id FROM reservations WHERE resource_id=?",
                             (resource_id,)).fetchone()
            if held:
                raise ReservationConflict(f"{resource_id} held by {held['attempt_id']} ({held['task_id']})")
            c.execute("INSERT INTO reservations VALUES (?,?,?,?)", (resource_id, task_id, attempt_id, now))
            c.execute("INSERT INTO attempts VALUES (?,?,?,?,?,?,?,?,?,?)",
                      (attempt_id, task_id, decision_id, resource_id, _dumps(command),
                       content_hash(command), "PREPARED", None, now, now))
        return self.get_attempt(attempt_id)

    def set_attempt_status(self, attempt_id: str, substatus: str, detail: Optional[dict] = None,
                           release: bool = False):
        """substatus 갱신. release=True 는 예약 해제까지 같은 트랜잭션에서 수행."""
        with self._tx() as c:
            c.execute("UPDATE attempts SET substatus=?, detail=COALESCE(?, detail), updated_wall=? "
                      "WHERE attempt_id=?", (substatus, _dumps(detail), time.time(), attempt_id))
            if release:
                if substatus in OCCUPYING_SUBSTATUSES:
                    raise ValueError(f"{substatus} 상태로는 예약을 해제할 수 없다")
                c.execute("DELETE FROM reservations WHERE attempt_id=?", (attempt_id,))

    def get_attempt(self, attempt_id: str) -> Optional[dict]:
        r = self._conn.execute("SELECT * FROM attempts WHERE attempt_id=?", (attempt_id,)).fetchone()
        return self._attempt_row(r) if r else None

    def list_attempts(self, task_id: Optional[str] = None, substatuses: Optional[Iterable[str]] = None) -> List[dict]:
        q, args = "SELECT * FROM attempts", []
        if task_id:
            q, args = q + " WHERE task_id=?", [task_id]
        rows = [self._attempt_row(r) for r in self._conn.execute(q + " ORDER BY created_wall, rowid", args)]
        if substatuses is not None:
            s = set(substatuses)
            rows = [a for a in rows if a["substatus"] in s]
        return rows

    @staticmethod
    def _attempt_row(r) -> dict:
        return {"attempt_id": r["attempt_id"], "task_id": r["task_id"], "decision_id": r["decision_id"],
                "resource_id": r["resource_id"], "command": _loads(r["command"]),
                "command_hash": r["command_hash"], "substatus": r["substatus"],
                "detail": _loads(r["detail"]), "created_wall": r["created_wall"],
                "updated_wall": r["updated_wall"]}

    def reservations(self) -> dict:
        """resource_id → {task_id, attempt_id}"""
        return {r["resource_id"]: {"task_id": r["task_id"], "attempt_id": r["attempt_id"]}
                for r in self._conn.execute("SELECT * FROM reservations")}

    # ------------------------------------------------------------------
    # 사건 기록
    # ------------------------------------------------------------------
    def log(self, event_type: str, *, task_id=None, decision_id=None, attempt_id=None, resource_id=None,
            result=None, reason=None, detail=None, sim_time_s=None) -> int:
        with self._tx() as c:
            cur = c.execute(
                "INSERT INTO events (wall_time, sim_time_s, event_type, task_id, decision_id, attempt_id, "
                "resource_id, result, reason, detail) VALUES (?,?,?,?,?,?,?,?,?,?)",
                (time.time(), sim_time_s, event_type, task_id, decision_id, attempt_id, resource_id,
                 result, reason, _dumps(detail)))
            return cur.lastrowid

    def record_external_event(self, event_id: str, body: dict) -> bool:
        """외부 이벤트를 event_id 기준 한 번만 기록. 처음이면 True, 중복이면 False."""
        with self._tx() as c:
            dup = c.execute("SELECT 1 FROM events WHERE event_type='EXTERNAL_EVENT' AND reason=?",
                            (event_id,)).fetchone()
            if dup:
                return False
            c.execute("INSERT INTO events (wall_time, sim_time_s, event_type, result, reason, detail) "
                      "VALUES (?,?,?,?,?,?)", (time.time(), body.get("simulation_time_s"), "EXTERNAL_EVENT",
                                               body.get("type"), event_id, _dumps(body)))
            return True

    # ------------------------------------------------------------------
    # 총괄이 아는 세계 (knowledge.py)
    # ------------------------------------------------------------------
    def add_knowledge(self, kind: str, key: str, subject: Optional[str], sim_time_s: Optional[float],
                      source: Optional[str], body: dict) -> bool:
        """같은 key 는 한 번만. 새로 들어가면 True"""
        with self._tx() as c:
            cur = c.execute("INSERT OR IGNORE INTO knowledge (kind, key, subject, sim_time_s, source, body, wall_time) "
                            "VALUES (?,?,?,?,?,?,?)", (kind, key, subject, sim_time_s, source, _dumps(body), time.time()))
            return cur.rowcount == 1

    def knowledge(self, kind: str) -> List[dict]:
        rows = self._conn.execute("SELECT * FROM knowledge WHERE kind=? ORDER BY sim_time_s, seq", (kind,))
        return [{"seq": r["seq"], "key": r["key"], "subject": r["subject"], "sim_time_s": r["sim_time_s"],
                 "source": r["source"], "body": _loads(r["body"])} for r in rows]

    def events(self, task_id: Optional[str] = None, after_seq: int = 0) -> List[dict]:
        q, args = "SELECT * FROM events WHERE seq>?", [after_seq]
        if task_id:
            q, args = q + " AND task_id=?", args + [task_id]
        return [{**dict(r), "detail": _loads(r["detail"])}
                for r in self._conn.execute(q + " ORDER BY seq", args)]


class _Tx:
    """BEGIN IMMEDIATE 트랜잭션 + 프로세스 내 lock. 예외 시 롤백."""

    def __init__(self, ledger: Ledger):
        self.l = ledger

    def __enter__(self):
        self.l._lock.acquire()
        self.l._conn.execute("BEGIN IMMEDIATE")
        return self.l._conn

    def __exit__(self, exc_type, exc, tb):
        try:
            self.l._conn.execute("ROLLBACK" if exc_type else "COMMIT")
        finally:
            self.l._lock.release()
        return False
