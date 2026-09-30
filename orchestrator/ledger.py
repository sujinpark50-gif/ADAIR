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

실행(run) 범위 (2026-09-30 B05):
- 활성 run 은 한 번에 하나다 (runs 표). Task·지식·외부 이벤트·요청 키는 run 안에서만 유효하다.
- 점유(reservations)는 물리 자원이라 run 과 무관하게 전역이다. 다른 run 의 점유가 남아 있으면
  새 run 을 활성화하지 않는다 (RunSwitchBlocked). 닫힌 run 은 다시 활성화하지 않는다 (RunClosed).
- run 범위가 없는 이전 장부(스키마 v1)의 행은 LEGACY-UNSCOPED 로 격리한다. 새 run 에 귀속시키지 않는다.

상태 전이 (B01/B02): save_task 가 models.PURPOSE_TRANSITIONS 로 검증한다. 같은 Task 에 임무가 열린
실행시도가 있으면 prepare_attempt 가 새 시도를 거절한다.
"""

import hashlib
import json
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

from .models import MISSION_OPEN_SUBSTATUSES, TERMINAL_PURPOSES, Task, can_transition

SCHEMA_USER_VERSION = 2
LEGACY_RUN = "LEGACY-UNSCOPED"

_SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id TEXT PRIMARY KEY,
    status TEXT NOT NULL,
    activated_wall REAL,
    closed_wall REAL,
    last_state_version INTEGER
);
CREATE TABLE IF NOT EXISTS tasks (
    task_id TEXT PRIMARY KEY,
    run_id TEXT NOT NULL,
    incident_id TEXT NOT NULL,
    request_key TEXT,
    request_hash TEXT,
    purpose_status TEXT NOT NULL,
    body TEXT NOT NULL,
    created_wall REAL NOT NULL,
    updated_wall REAL NOT NULL,
    UNIQUE (run_id, request_key)
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
    updated_wall REAL NOT NULL,
    run_id TEXT
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
    detail TEXT,
    run_id TEXT
);
CREATE TABLE IF NOT EXISTS knowledge (
    seq INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id TEXT NOT NULL,
    kind TEXT NOT NULL,
    key TEXT NOT NULL,
    subject TEXT,
    sim_time_s REAL,
    source TEXT,
    body TEXT NOT NULL,
    wall_time REAL NOT NULL,
    UNIQUE (run_id, key)
);
CREATE TABLE IF NOT EXISTS external_events (
    run_id TEXT NOT NULL,
    event_id TEXT NOT NULL,
    content_hash TEXT NOT NULL,
    status TEXT NOT NULL,
    body TEXT NOT NULL,
    result TEXT,
    received_wall REAL NOT NULL,
    applied_wall REAL,
    PRIMARY KEY (run_id, event_id)
);
CREATE TABLE IF NOT EXISTS pending_applies (
    observation_id TEXT PRIMARY KEY,
    run_id TEXT NOT NULL,
    task_id TEXT NOT NULL,
    attempt_id TEXT NOT NULL,
    body TEXT NOT NULL,
    purpose_bound INTEGER NOT NULL,
    status TEXT NOT NULL,
    reason TEXT,
    tries INTEGER NOT NULL,
    created_wall REAL NOT NULL,
    resolved_wall REAL,
    ack TEXT
);
CREATE TABLE IF NOT EXISTS observations (
    observation_id TEXT PRIMARY KEY,
    attempt_id TEXT NOT NULL,
    role TEXT NOT NULL,
    run_id TEXT,
    task_id TEXT NOT NULL,
    sim_time_s REAL,
    body TEXT NOT NULL,
    created_wall REAL NOT NULL,
    UNIQUE (attempt_id, role)
);
CREATE INDEX IF NOT EXISTS ix_events_task ON events(task_id);
CREATE INDEX IF NOT EXISTS ix_attempts_task ON attempts(task_id);
CREATE INDEX IF NOT EXISTS ix_knowledge_run_kind ON knowledge(run_id, kind);
"""

# 스키마 v1(run 범위 없음) → v2. 기존 행은 어느 run 인지 알 수 없으므로 LEGACY_RUN 으로 격리한다.
_MIGRATE_V1 = f"""
BEGIN IMMEDIATE;
ALTER TABLE tasks RENAME TO tasks_v1;
ALTER TABLE knowledge RENAME TO knowledge_v1;
CREATE TABLE tasks (
    task_id TEXT PRIMARY KEY, run_id TEXT NOT NULL, incident_id TEXT NOT NULL, request_key TEXT,
    request_hash TEXT, purpose_status TEXT NOT NULL, body TEXT NOT NULL, created_wall REAL NOT NULL,
    updated_wall REAL NOT NULL, UNIQUE (run_id, request_key));
INSERT INTO tasks SELECT task_id, '{LEGACY_RUN}', incident_id, request_key, request_hash, purpose_status, body,
    created_wall, updated_wall FROM tasks_v1;
CREATE TABLE knowledge (
    seq INTEGER PRIMARY KEY AUTOINCREMENT, run_id TEXT NOT NULL, kind TEXT NOT NULL, key TEXT NOT NULL,
    subject TEXT, sim_time_s REAL, source TEXT, body TEXT NOT NULL, wall_time REAL NOT NULL,
    UNIQUE (run_id, key));
INSERT INTO knowledge (seq, run_id, kind, key, subject, sim_time_s, source, body, wall_time)
    SELECT seq, '{LEGACY_RUN}', kind, key, subject, sim_time_s, source, body, wall_time FROM knowledge_v1;
DROP TABLE tasks_v1;
DROP TABLE knowledge_v1;
ALTER TABLE attempts ADD COLUMN run_id TEXT;
UPDATE attempts SET run_id = '{LEGACY_RUN}';
ALTER TABLE events ADD COLUMN run_id TEXT;
UPDATE events SET run_id = '{LEGACY_RUN}';
COMMIT;
"""

# 이 상태의 실행시도는 자원을 계속 점유한다
OCCUPYING_SUBSTATUSES = {"PREPARED", "REQUESTED", "STARTED", "ARRIVED", "OBSERVED",
                         "APPLIED", "RETURNING", "UNKNOWN", "FAULTED"}

ACTIVE = object()      # 조회 범위 기본값: 지금 활성 run
ALL_RUNS = object()    # 모든 run (감사·추적용)


class RequestConflict(Exception):
    """같은 요청 키에 다른 내용이 들어옴"""


class ReservationConflict(Exception):
    """이미 다른 실행시도가 점유한 자원"""


class AttemptConflict(Exception):
    """같은 Task 에 임무가 열린 실행시도가 이미 있거나, Task 가 이미 종료됨"""


class IllegalTransition(Exception):
    """허용되지 않은 Task 목적 상태 전이"""


class RunNotActive(Exception):
    """활성 run 이 없거나 요청한 run 이 활성 run 이 아님"""


class RunSwitchBlocked(Exception):
    """다른 run 의 점유·열린 실행이 남아 있어 run 을 바꿀 수 없음"""


class RunClosed(Exception):
    """이미 닫힌 run 을 다시 활성화하려 함 (늦게 온 이전 run)"""


def new_id(prefix: str) -> str:
    return f"{prefix}-{uuid.uuid4().hex[:12]}"


def content_hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, ensure_ascii=False, default=str).encode("utf-8")).hexdigest()


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
        self._lock = threading.RLock()
        self.migrated_from_legacy = self._init_schema()
        row = self._conn.execute("SELECT run_id FROM runs WHERE status='ACTIVE'").fetchone()
        self._active_run = row["run_id"] if row else None

    def _init_schema(self) -> bool:
        """스키마 생성·이관. 이전 장부를 격리 이관했으면 True."""
        version = self._conn.execute("PRAGMA user_version").fetchone()[0]
        has_tasks = self._conn.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='tasks'").fetchone()
        legacy = False
        if version < SCHEMA_USER_VERSION and has_tasks:
            cols = {r["name"] for r in self._conn.execute("PRAGMA table_info(tasks)")}
            if "run_id" not in cols:
                self._conn.executescript(_MIGRATE_V1)
                legacy = True
        self._conn.executescript(_SCHEMA)
        if "ack" not in {r["name"] for r in self._conn.execute("PRAGMA table_info(pending_applies)")}:
            self._conn.execute("ALTER TABLE pending_applies ADD COLUMN ack TEXT")     # 094f965 장부에서 올라온 경우
        self.legacy_apply_migration = self._migrate_legacy_apply_rows()
        if legacy:
            self._conn.execute("INSERT OR IGNORE INTO runs VALUES (?,?,?,?,?)",
                               (LEGACY_RUN, "QUARANTINED", None, time.time(), None))
        self._conn.execute(f"PRAGMA user_version = {SCHEMA_USER_VERSION}")
        return legacy

    def close(self):
        self._conn.close()

    def _migrate_legacy_apply_rows(self) -> dict:
        """이전 판(094f965)의 ACKED/NACKED 반영 기록 이관 (F05). 그 판은 환경 응답을 받으면 기록을 먼저 ACKED/NACKED 로
        닫고 나서 임무 판정을 했으므로, 그 사이에 멈춘 임무는 새 흐름의 '끝나지 않은 기록' 조회에서 빠진다.
          - 목적이 이미 끝났거나(완료·취소 등) 그 시도의 완료 사건이 있으면 → DONE (다시 적용하지 않음)
          - 담당이 다른 시도로 바뀌었으면 → CLOSED OWNER_CHANGED
          - 판정이 안 된 것 → RESPONDED 로 되돌려 새 흐름이 판정한다. 응답 근거는 장부에 남은 이전 상태
            (ACKED = 환경이 수용, NACKED = 거절)와 사유이며, 그 밖의 값을 지어내지 않는다 (restored_from 표시)
          - Task·실행시도를 찾을 수 없으면 → MANUAL_REVIEW (사람 확인, 재출동도 막는다)
        한 트랜잭션이라 도중에 멈추면 전부 되돌아가고, 다시 열 때 같은 결과로 반복된다 (대상은 ACKED/NACKED 뿐)."""
        rows = self._conn.execute("SELECT * FROM pending_applies WHERE status IN ('ACKED','NACKED')").fetchall()
        out = {}
        if not rows:
            return out
        with self._tx() as c:
            for r in rows:
                trow = c.execute("SELECT body FROM tasks WHERE task_id=?", (r["task_id"],)).fetchone()
                arow = c.execute("SELECT attempt_id FROM attempts WHERE attempt_id=?", (r["attempt_id"],)).fetchone()
                legacy = r["status"]
                if not r["purpose_bound"]:
                    new, reason = "DONE", f"LEGACY_{legacy}_AUXILIARY"
                elif trow is None or arow is None:
                    new, reason = "MANUAL_REVIEW", f"LEGACY_{legacy}_TASK_OR_ATTEMPT_MISSING"
                else:
                    task = json.loads(trow["body"])
                    judged = c.execute("SELECT 1 FROM events WHERE event_type='TASK_COMPLETE' AND attempt_id=?",
                                       (r["attempt_id"],)).fetchone()
                    owner = task.get("owner_attempt_id")
                    if task["purpose_status"] != "IN_EXECUTION" or judged:
                        new, reason = "DONE", f"LEGACY_{legacy}_ALREADY_JUDGED:{task['purpose_status']}"
                    elif owner not in (None, r["attempt_id"]):
                        new, reason = "CLOSED", f"LEGACY_{legacy}_OWNER_CHANGED"
                    else:
                        new, reason = "RESPONDED", None
                ack = {"accepted": legacy == "ACKED", "reason": r["reason"], "restored_from": f"LEGACY_STATUS_{legacy}"}
                c.execute("UPDATE pending_applies SET status=?, reason=?, ack=COALESCE(ack, ?), "
                          "resolved_wall=CASE WHEN ?='RESPONDED' THEN NULL ELSE COALESCE(resolved_wall, ?) END "
                          "WHERE observation_id=?",
                          (new, reason, _dumps(ack), new, time.time(), r["observation_id"]))
                out[r["observation_id"]] = new
        return out

    def _tx(self):
        return _Tx(self)

    # ------------------------------------------------------------------
    # 실행(run) 범위
    # ------------------------------------------------------------------
    def active_run(self) -> Optional[str]:
        return self._active_run

    def run_status(self, run_id: str) -> Optional[str]:
        row = self._conn.execute("SELECT status FROM runs WHERE run_id=?", (run_id,)).fetchone()
        return row["status"] if row else None

    def foreign_reservations(self, run_id: str) -> dict:
        """run_id 가 아닌 run 의 실행시도가 잡고 있는 점유 (run 전환을 막는 것)"""
        rows = self._conn.execute(
            "SELECT r.resource_id, r.task_id, r.attempt_id, a.run_id, a.substatus FROM reservations r "
            "LEFT JOIN attempts a ON a.attempt_id = r.attempt_id").fetchall()
        return {r["resource_id"]: {"task_id": r["task_id"], "attempt_id": r["attempt_id"],
                                   "run_id": r["run_id"], "substatus": r["substatus"]}
                for r in rows if r["run_id"] != run_id}

    def activate_run(self, run_id: str) -> str:
        """run 을 활성화한다. "ALREADY_ACTIVE" / "ACTIVATED".
        다른 run 의 점유가 남아 있으면 RunSwitchBlocked, 닫힌 run 이면 RunClosed."""
        with self._tx() as c:
            if self._active_run == run_id:
                return "ALREADY_ACTIVE"
            row = c.execute("SELECT status FROM runs WHERE run_id=?", (run_id,)).fetchone()
            if row and row["status"] != "ACTIVE":
                raise RunClosed(run_id)
            blocking = self.foreign_reservations(run_id)
            if blocking:
                raise RunSwitchBlocked(json.dumps(blocking, ensure_ascii=False))
            now = time.time()
            c.execute("UPDATE runs SET status='CLOSED', closed_wall=? WHERE status='ACTIVE'", (now,))
            c.execute("INSERT INTO runs VALUES (?,?,?,?,?)", (run_id, "ACTIVE", now, None, None))
            # 같은 트랜잭션에서 이전 run 의 끝나지 않은 반영 기록을 닫는다 (F07) — 둘 중 하나만 저장되는 일이 없다
            self.last_activation_closed = self.close_pending_applies_of_other_runs(run_id, "RUN_ENDED")
        self._active_run = run_id                     # 저장이 끝난 뒤에 메모리를 바꾼다
        return "ACTIVATED"

    def note_state_version(self, run_id: str, version: int) -> str:
        """같은 run 안의 환경 버전 순서: "NEW" / "DUPLICATE" / "STALE"(역행). 재시작 후에도 이어진다."""
        with self._tx() as c:
            row = c.execute("SELECT last_state_version FROM runs WHERE run_id=?", (run_id,)).fetchone()
            last = row["last_state_version"] if row else None
            if last is not None and version == last:
                return "DUPLICATE"
            if last is not None and version < last:
                return "STALE"
            c.execute("UPDATE runs SET last_state_version=? WHERE run_id=?", (version, run_id))
            return "NEW"

    def _scope(self, run_id):
        """조회 범위 → (WHERE 절 조각, 인자). 활성 run 이 없으면 아무 행도 고르지 않는다."""
        if run_id is ALL_RUNS:
            return "1=1", []
        rid = self._active_run if run_id is ACTIVE else run_id
        if rid is None:
            return "1=0", []
        return "run_id=?", [rid]

    def _require_run(self, run_id=None) -> str:
        rid = run_id or self._active_run
        if rid is None:
            raise RunNotActive("활성 run 이 없다")
        return rid

    # ------------------------------------------------------------------
    # Tasks
    # ------------------------------------------------------------------
    def create_task(self, task: Task, request_body: dict) -> (Task, bool):
        """(task, created). 같은 run 에 같은 request_key 가 있으면 기존 Task 반환(created=False)."""
        req_hash = content_hash(request_body)
        now = time.time()
        with self._tx() as c:
            run_id = self._require_run(task.run_id)
            if task.request_key:
                row = c.execute("SELECT body, request_hash FROM tasks WHERE run_id=? AND request_key=?",
                                (run_id, task.request_key)).fetchone()
                if row:
                    if row["request_hash"] != req_hash:
                        raise RequestConflict(task.request_key)
                    return Task.from_dict(json.loads(row["body"])), False
            task.run_id = run_id
            task.created_wall = task.created_wall or now
            c.execute("INSERT INTO tasks (task_id, run_id, incident_id, request_key, request_hash, purpose_status, "
                      "body, created_wall, updated_wall) VALUES (?,?,?,?,?,?,?,?,?)",
                      (task.task_id, run_id, task.incident_id, task.request_key, req_hash,
                       task.purpose_status, _dumps(task.to_dict()), task.created_wall, now))
        return task, True

    def save_task(self, task: Task):
        """Task 저장. 목적 상태가 바뀌면 허용 전이인지 검증한다 (종료된 목적은 되살아나지 않는다)."""
        with self._tx() as c:
            row = c.execute("SELECT purpose_status FROM tasks WHERE task_id=?", (task.task_id,)).fetchone()
            if row and not can_transition(row["purpose_status"], task.purpose_status):
                raise IllegalTransition(f"{task.task_id}: {row['purpose_status']} -> {task.purpose_status}")
            c.execute("UPDATE tasks SET purpose_status=?, body=?, updated_wall=? WHERE task_id=?",
                      (task.purpose_status, _dumps(task.to_dict()), time.time(), task.task_id))

    def get_task(self, task_id: str) -> Optional[Task]:
        row = self._conn.execute("SELECT body FROM tasks WHERE task_id=?", (task_id,)).fetchone()
        return Task.from_dict(json.loads(row["body"])) if row else None

    def task_by_request_key(self, request_key: str, run_id=ACTIVE) -> Optional[Task]:
        where, args = self._scope(run_id)
        row = self._conn.execute(f"SELECT body FROM tasks WHERE {where} AND request_key=?",
                                 args + [request_key]).fetchone()
        return Task.from_dict(json.loads(row["body"])) if row else None

    def list_tasks(self, statuses: Optional[Iterable[str]] = None, run_id=ACTIVE) -> List[Task]:
        where, args = self._scope(run_id)
        rows = self._conn.execute(f"SELECT body FROM tasks WHERE {where} ORDER BY created_wall, task_id",
                                  args).fetchall()
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
            trow = c.execute("SELECT run_id, purpose_status FROM tasks WHERE task_id=?", (task_id,)).fetchone()
            if trow and trow["purpose_status"] in TERMINAL_PURPOSES:
                raise AttemptConflict(f"{task_id} is {trow['purpose_status']}")
            open_ = self.open_mission_attempts(task_id)
            if open_:
                raise AttemptConflict(f"{task_id} has open attempt {open_[0]['attempt_id']} ({open_[0]['substatus']})")
            if trow and c.execute("SELECT 1 FROM pending_applies WHERE task_id=? AND purpose_bound=1 "
                                  "AND status IN ('PENDING','RESPONDED','MANUAL_REVIEW')", (task_id,)).fetchone():
                raise AttemptConflict(f"{task_id} has an observation waiting for environment apply")
            c.execute("INSERT INTO reservations VALUES (?,?,?,?)", (resource_id, task_id, attempt_id, now))
            c.execute("INSERT INTO attempts (attempt_id, task_id, decision_id, resource_id, command, command_hash, "
                      "substatus, detail, created_wall, updated_wall, run_id) VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                      (attempt_id, task_id, decision_id, resource_id, _dumps(command),
                       content_hash(command), "PREPARED", None, now, now, trow["run_id"] if trow else None))
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

    def open_mission_attempts(self, task_id: str) -> List[dict]:
        """임무가 아직 열린 실행시도 (R01). 점유 중인 시도는 '임무 종료 근거'(detail.mission_end)가 있고
        상태가 불명(UNKNOWN)이 아닐 때만 닫힌 것으로 본다. RETURNING 같은 상태 이름만으로는 닫혔다고 보지 않는다."""
        return [a for a in self.list_attempts(task_id)
                if a["substatus"] in OCCUPYING_SUBSTATUSES
                and (a["substatus"] in MISSION_OPEN_SUBSTATUSES or not (a["detail"] or {}).get("mission_end"))]

    # ------------------------------------------------------------------
    # 환경 반영 기록 (R04). 기체 반납과 별개로 남는다. 두 단계를 따로 기록한다:
    #   PENDING    환경이 받았는지 모름            → 같은 observation_id 로 APPLY 재시도 (멱등)
    #   RESPONDED  환경 응답(ACK/NACK)을 저장했지만 임무 판정은 아직 → 저장한 응답으로 판정 재개 (환경에 다시 보내지 않음)
    #   DONE       임무 판정까지 끝남 / CLOSED 판정 없이 종료 (취소·담당 변경·run 종료 등, reason 에 사유)
    # 지우지 않는다. OPEN = PENDING + RESPONDED.
    # ------------------------------------------------------------------
    OPEN_APPLY = ("PENDING", "RESPONDED")

    # ------------------------------------------------------------------
    # 관측 원본 (F01). 실행시도마다 역할(MAIN 주 관측 / AUX 함께 한 측정)별로 한 번 확정한다.
    # 복구할 때는 이 원본을 다시 쓴다 — 재시작 시점의 환경을 읽어 새 관측으로 바꾸지 않는다.
    # ------------------------------------------------------------------
    def save_observation(self, obs: dict, attempt_id: str, role: str, run_id: Optional[str], task_id: str) -> bool:
        with self._tx() as c:
            cur = c.execute("INSERT OR IGNORE INTO observations VALUES (?,?,?,?,?,?,?,?)",
                            (obs["observation_id"], attempt_id, role, run_id, task_id, obs.get("simulation_time_s"),
                             _dumps(obs), time.time()))
            return cur.rowcount == 1

    def get_observation(self, attempt_id: str, role: str) -> Optional[dict]:
        r = self._conn.execute("SELECT body FROM observations WHERE attempt_id=? AND role=?",
                               (attempt_id, role)).fetchone()
        return _loads(r["body"]) if r else None

    def add_pending_apply(self, observation: dict, run_id: str, task_id: str, attempt_id: str,
                          purpose_bound: bool) -> bool:
        with self._tx() as c:
            cur = c.execute("INSERT OR IGNORE INTO pending_applies (observation_id, run_id, task_id, attempt_id, "
                            "body, purpose_bound, status, reason, tries, created_wall, resolved_wall, ack) "
                            "VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
                            (observation["observation_id"], run_id, task_id, attempt_id, _dumps(observation),
                             1 if purpose_bound else 0, "PENDING", None, 0, time.time(), None, None))
            return cur.rowcount == 1

    def pending_applies(self, task_id: Optional[str] = None, run_id=ALL_RUNS, status: Optional[str] = "OPEN") -> List[dict]:
        """status: "OPEN"(기본, 아직 끝나지 않은 것) / 특정 상태 / None(전부)"""
        where, args = self._scope(run_id)
        q = f"SELECT * FROM pending_applies WHERE {where}"
        if status == "OPEN":
            q += " AND status IN ('PENDING','RESPONDED')"
        elif status:
            q, args = q + " AND status=?", args + [status]
        if task_id:
            q, args = q + " AND task_id=?", args + [task_id]
        out = []
        for r in self._conn.execute(q + " ORDER BY created_wall, rowid", args):
            ack = _loads(r["ack"])
            out.append({"observation_id": r["observation_id"], "run_id": r["run_id"], "task_id": r["task_id"],
                        "attempt_id": r["attempt_id"], "observation": _loads(r["body"]),
                        "purpose_bound": bool(r["purpose_bound"]), "status": r["status"], "reason": r["reason"],
                        "tries": r["tries"], "ack": ack,
                        "env_result": None if ack is None else ("ACK" if ack.get("accepted") else "NACK")})
        return out

    def note_pending_apply_try(self, observation_id: str):
        with self._tx() as c:
            c.execute("UPDATE pending_applies SET tries=tries+1 WHERE observation_id=?", (observation_id,))

    def store_apply_response(self, observation_id: str, ack: dict):
        """환경 응답을 저장한다 (PENDING → RESPONDED). 임무 판정은 별도 단계다."""
        with self._tx() as c:
            c.execute("UPDATE pending_applies SET status='RESPONDED', ack=?, tries=tries+1 "
                      "WHERE observation_id=? AND status='PENDING'", (_dumps(ack), observation_id))

    def finish_pending_apply(self, observation_id: str, status: str, reason: Optional[str] = None):
        """끝난 상태(DONE/CLOSED)로 표시. 저장한 환경 응답은 그대로 남는다."""
        with self._tx() as c:
            c.execute("UPDATE pending_applies SET status=?, reason=?, resolved_wall=? "
                      "WHERE observation_id=? AND status IN ('PENDING','RESPONDED')",
                      (status, reason, time.time(), observation_id))

    def close_pending_applies_of_other_runs(self, run_id: str, reason: str) -> int:
        with self._tx() as c:
            cur = c.execute("UPDATE pending_applies SET status='CLOSED', reason=?, resolved_wall=? "
                            "WHERE status IN ('PENDING','RESPONDED') AND run_id<>?", (reason, time.time(), run_id))
            return cur.rowcount

    def get_attempt(self, attempt_id: str) -> Optional[dict]:
        r = self._conn.execute("SELECT * FROM attempts WHERE attempt_id=?", (attempt_id,)).fetchone()
        return self._attempt_row(r) if r else None

    def list_attempts(self, task_id: Optional[str] = None, substatuses: Optional[Iterable[str]] = None) -> List[dict]:
        """실행시도는 run 과 무관하게 모두 본다 (이전 run 의 열린 실행도 계속 추적해야 한다)."""
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
                "updated_wall": r["updated_wall"], "run_id": r["run_id"]}

    def reservations(self) -> dict:
        """resource_id → {task_id, attempt_id}. 물리 자원 점유라 run 과 무관하게 전역이다."""
        return {r["resource_id"]: {"task_id": r["task_id"], "attempt_id": r["attempt_id"]}
                for r in self._conn.execute("SELECT * FROM reservations")}

    # ------------------------------------------------------------------
    # 사건 기록
    # ------------------------------------------------------------------
    def log(self, event_type: str, *, task_id=None, decision_id=None, attempt_id=None, resource_id=None,
            result=None, reason=None, detail=None, sim_time_s=None, run_id=None) -> int:
        with self._tx() as c:
            cur = c.execute(
                "INSERT INTO events (wall_time, sim_time_s, event_type, task_id, decision_id, attempt_id, "
                "resource_id, result, reason, detail, run_id) VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                (time.time(), sim_time_s, event_type, task_id, decision_id, attempt_id, resource_id,
                 result, reason, _dumps(detail), run_id or self._active_run))
            return cur.lastrowid

    def events(self, task_id: Optional[str] = None, after_seq: int = 0, run_id=ALL_RUNS) -> List[dict]:
        where, scope_args = self._scope(run_id)
        q, args = f"SELECT * FROM events WHERE seq>? AND {where}", [after_seq] + scope_args
        if task_id:
            q, args = q + " AND task_id=?", args + [task_id]
        return [{**dict(r), "detail": _loads(r["detail"])}
                for r in self._conn.execute(q + " ORDER BY seq", args)]

    # ------------------------------------------------------------------
    # 외부 이벤트 접수 (B04). 검증을 통과한 이벤트만 여기 들어온다.
    #   RECEIVED → (지식·Task 반영) → APPLIED. RECEIVED 로 남은 것은 재시작 때 다시 처리한다.
    # ------------------------------------------------------------------
    def receive_external_event(self, event_id: str, chash: str, body: dict) -> Tuple[str, dict]:
        """("NEW" | "DUPLICATE" | "CONFLICT", 저장된 행). 같은 run·event_id 의 같은 내용은 DUPLICATE,
        다른 내용은 CONFLICT (기존 행을 바꾸지 않는다)."""
        with self._tx() as c:
            run_id = self._require_run()
            row = c.execute("SELECT * FROM external_events WHERE run_id=? AND event_id=?",
                            (run_id, event_id)).fetchone()
            if row:
                return ("DUPLICATE" if row["content_hash"] == chash else "CONFLICT"), self._event_row(row)
            c.execute("INSERT INTO external_events VALUES (?,?,?,?,?,?,?,?)",
                      (run_id, event_id, chash, "RECEIVED", _dumps(body), None, time.time(), None))
            row = c.execute("SELECT * FROM external_events WHERE run_id=? AND event_id=?",
                            (run_id, event_id)).fetchone()
            return "NEW", self._event_row(row)

    def mark_external_event_applied(self, event_id: str, result: dict, run_id: Optional[str] = None):
        with self._tx() as c:
            c.execute("UPDATE external_events SET status='APPLIED', result=?, applied_wall=? "
                      "WHERE run_id=? AND event_id=?", (_dumps(result), time.time(),
                                                        self._require_run(run_id), event_id))

    def pending_external_events(self, run_id=ACTIVE) -> List[dict]:
        where, args = self._scope(run_id)
        return [self._event_row(r) for r in self._conn.execute(
            f"SELECT * FROM external_events WHERE {where} AND status='RECEIVED' ORDER BY received_wall, rowid", args)]

    @staticmethod
    def _event_row(r) -> dict:
        return {"run_id": r["run_id"], "event_id": r["event_id"], "content_hash": r["content_hash"],
                "status": r["status"], "body": _loads(r["body"]), "result": _loads(r["result"])}

    # ------------------------------------------------------------------
    # 총괄이 아는 세계 (knowledge.py)
    # ------------------------------------------------------------------
    def add_knowledge(self, kind: str, key: str, subject: Optional[str], sim_time_s: Optional[float],
                      source: Optional[str], body: dict) -> bool:
        """활성 run 안에서 같은 key 는 한 번만. 새로 들어가면 True"""
        with self._tx() as c:
            cur = c.execute("INSERT OR IGNORE INTO knowledge (run_id, kind, key, subject, sim_time_s, source, body, "
                            "wall_time) VALUES (?,?,?,?,?,?,?,?)",
                            (self._require_run(), kind, key, subject, sim_time_s, source, _dumps(body), time.time()))
            return cur.rowcount == 1

    def knowledge(self, kind: str, run_id=ACTIVE) -> List[dict]:
        where, args = self._scope(run_id)
        rows = self._conn.execute(f"SELECT * FROM knowledge WHERE {where} AND kind=? ORDER BY sim_time_s, seq",
                                  args + [kind])
        return [{"seq": r["seq"], "key": r["key"], "subject": r["subject"], "sim_time_s": r["sim_time_s"],
                 "source": r["source"], "body": _loads(r["body"])} for r in rows]

    def get_knowledge(self, key: str, run_id=ACTIVE) -> Optional[dict]:
        where, args = self._scope(run_id)
        r = self._conn.execute(f"SELECT * FROM knowledge WHERE {where} AND key=?", args + [key]).fetchone()
        return None if r is None else {"seq": r["seq"], "key": r["key"], "subject": r["subject"], "kind": r["kind"],
                                       "sim_time_s": r["sim_time_s"], "source": r["source"], "body": _loads(r["body"])}


class _Tx:
    """BEGIN IMMEDIATE 트랜잭션 + 프로세스 내 lock. 예외 시 롤백. 같은 스레드에서 겹쳐 쓰면 안쪽은 바깥 트랜잭션에 합류한다."""

    def __init__(self, ledger: Ledger):
        self.l = ledger
        self.outer = False

    def __enter__(self):
        self.l._lock.acquire()
        self.outer = not self.l._conn.in_transaction
        if self.outer:
            self.l._conn.execute("BEGIN IMMEDIATE")
        return self.l._conn

    def __exit__(self, exc_type, exc, tb):
        try:
            if self.outer:
                self.l._conn.execute("ROLLBACK" if exc_type else "COMMIT")
        finally:
            self.l._lock.release()
        return False
