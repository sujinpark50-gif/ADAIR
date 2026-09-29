# -*- coding: utf-8 -*-
import pytest

from orchestrator.ledger import Ledger, RequestConflict, ReservationConflict
from orchestrator.models import Target, Task


def _task(tid, key):
    return Task(task_id=tid, incident_id="INC", kind="RECON", target=Target(38.0, 128.0, 500.0), request_key=key)


def test_same_request_key_converges_to_one_task(tmp_path):
    lg = Ledger(str(tmp_path / "s.db"))
    t1, c1 = lg.create_task(_task("T1", "K"), {"a": 1})
    t2, c2 = lg.create_task(_task("T2", "K"), {"a": 1})
    assert c1 and not c2 and t2.task_id == "T1"
    with pytest.raises(RequestConflict):
        lg.create_task(_task("T3", "K"), {"a": 2})


def test_reservation_is_exclusive_and_unknown_cannot_release(tmp_path):
    lg = Ledger(str(tmp_path / "s.db"))
    lg.prepare_attempt("ATT-1", "T1", "D1", "A-uav1", {"x": 1})
    with pytest.raises(ReservationConflict):
        lg.prepare_attempt("ATT-2", "T2", "D2", "A-uav1", {"x": 2})
    with pytest.raises(ValueError):
        lg.set_attempt_status("ATT-1", "UNKNOWN", release=True)
    assert "A-uav1" in lg.reservations()
    lg.set_attempt_status("ATT-1", "RELEASED", release=True)
    assert lg.reservations() == {}


def test_state_survives_restart(tmp_path):
    p = str(tmp_path / "s.db")
    lg = Ledger(p)
    lg.create_task(_task("T1", "K"), {"a": 1})
    lg.prepare_attempt("ATT-1", "T1", "D1", "A-uav1", {"x": 1})
    lg.set_attempt_status("ATT-1", "UNKNOWN")
    lg.close()
    lg2 = Ledger(p)
    assert lg2.get_task("T1") is not None
    assert lg2.reservations()["A-uav1"]["attempt_id"] == "ATT-1"
    assert lg2.get_attempt("ATT-1")["substatus"] == "UNKNOWN"
