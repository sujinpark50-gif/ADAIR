# -*- coding: utf-8 -*-
"""재검증(7712e8a) 재현시험 — 검토자 제공, 그대로 회귀시험으로 둔다.
1) 자동 배정 중(평가가 느림)에도 주기적 추적은 계속된다 (배정은 별도 작업자).
2) 예약 뒤 승인 저장 직전에 취소되면 출동하지 않고 예약을 푼다.
3) 후속 측정 계획은 한 번 정해 저장하고, 복구 때 풍향이 바뀌어도 그대로 쓴다.
4) 이전 run 의 후속 작업은 새 run 에서 수행하지 않는다.
"""
import threading
import pytest
from fastapi.testclient import TestClient
from orchestrator import config
from orchestrator.api import create_app
from orchestrator.tests.conftest import submit
from orchestrator.tests.test_internal_tasks import _restart
from orchestrator.tests.test_env_sense import _setup
class Stop(BaseException): pass

def test_background_poller_continues_during_automatic_slow_dispatch(world,monkeypatch):
    o,u,lg=world['orch'],world['uav'],world['ledger']
    world['env'].fire_cells[0]['fire_state']='UNBURNED'
    first=o.dispatch(submit(o,'FIRST').task_id)
    submit(o,'SECOND')
    entered,release,tracked=threading.Event(),threading.Event(),threading.Event()
    def slow(body):
        entered.set()
        assert release.wait(5)
        return {'verdict':'ACCEPT','eta_sec':600}
    u.verdicts['A-uav2']=slow
    real=o.uav.task_status
    def status(rid,aid):
        if aid==first['attempt_id'] and entered.is_set(): tracked.set()
        return real(rid,aid)
    o.uav.task_status=status
    monkeypatch.setattr(config,'AUTO_DISPATCH_ON_CHANGE',True)
    monkeypatch.setattr(o,'llm_request',lambda:None)
    app=create_app(o,poll_interval_s=.01)
    with TestClient(app):
        try:
            assert entered.wait(3)
            continued=tracked.wait(.5)
        finally:
            release.set()
            tracked.wait(2)
    assert continued, 'background poller blocked inside dispatch_all'

def test_cancel_between_approval_check_and_save_releases_unsent_reservation(world):
    o,lg,u=world['orch'],world['ledger'],world['uav']
    t=submit(o)
    reached,resume=threading.Event(),threading.Event()
    real=lg.save_task
    def save(task):
        if task.task_id==t.task_id and task.purpose_status=='APPROVED':
            reached.set(); assert resume.wait(5)
        return real(task)
    lg.save_task=save
    app=create_app(o); result={}
    def dispatch():
        result['response']=TestClient(app).post(f'/tasks/{t.task_id}/dispatch').json()
    th=threading.Thread(target=dispatch,daemon=True); th.start()
    try:
        assert reached.wait(3)
        r=TestClient(app).post(f'/tasks/{t.task_id}/resolve',json={'action':'CANCEL','reason':'review'})
        assert r.json()['status']=='CANCELLED'
    finally:
        resume.set(); th.join(3)
    assert not th.is_alive()
    assert len(u.exec_calls())==0
    assert lg.get_task(t.task_id).purpose_status=='CANCELLED'
    assert lg.reservations()=={}, result

def pending_followup(world,after_creation=False):
    o,lg=world['orch'],world['ledger']; _setup(world)
    t=submit(o); a=o.dispatch(t.task_id); o.poll(); o.poll()
    if after_creation:
        def stop(*args,**kwargs): raise Stop()
        lg.finish_followup=stop
    else:
        def stop(*args,**kwargs): raise Stop()
        o.run_followups=stop
    with pytest.raises(Stop): o.poll()
    return t,a

def test_followup_restart_after_task_created_does_not_plan_a_second_target(world):
    t,a=pending_followup(world,True)
    lg,env=world['ledger'],world['env']
    assert [t.target.cell_id for t in lg.list_tasks() if t.kind=='ENV_SENSE']==['EAST']
    env.wind_dir_deg=180.0; env.advance()
    o,lg=_restart(world); o.recover()
    assert [t.target.cell_id for t in lg.list_tasks() if t.kind=='ENV_SENSE']==['EAST']

def test_old_followup_is_not_executed_in_new_run(world):
    t,a=pending_followup(world)
    o,lg=_restart(world)
    # Physically finish returning without consuming the saved followup.
    o._track(lg.get_attempt(a['attempt_id']),o.uav)
    assert lg.reservations()=={}
    env=world['env']; env.run_id='RUN-NEW'
    env.reports=[{'cell_id':'C1','sim_time_s':0.0,'source':'119_CALL'}]
    o.run_followups()
    assert lg.active_run()=='RUN-NEW'
    assert [x for x in lg.list_tasks() if x.kind=='ENV_SENSE']==[]
