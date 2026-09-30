# -*- coding: utf-8 -*-
"""재검증(41bdbb0) 남은 문제 재현시험 — 검토자 제공, 그대로 회귀시험으로 둔다.
1) 이전 판에서 실행시도 불명 때문에 HOLD 였던 임무의 ACKED/NACKED 는 이관 뒤에도 보존되어 재접촉 후 판정된다.
2) 부가 기상 측정의 반영 결과 사건과 DONE 표시는 함께 저장되어 재시작 뒤 이력이 중복되지 않는다.
"""
import pytest
from orchestrator.tests.conftest import submit
from orchestrator.tests.test_followup_r01_r04 import _restart, _env_down, _ev

@pytest.mark.parametrize('legacy_status',['ACKED','NACKED'])
def test_legacy_response_during_unknown_survives_upgrade(world,legacy_status):
    o,lg,u,env=world['orch'],world['ledger'],world['uav'],world['env']
    calls,recover=_env_down(world)
    t=submit(o); a=o.dispatch(t.task_id)
    for _ in range(3): o.poll()
    u.scripts[a['attempt_id']]=[{'status':[]}]
    o.poll()
    assert lg.get_task(t.task_id).purpose_status=='HOLD'
    # d74f053 stored ACKED/NACKED before checking HOLD and lost judgement.
    lg._conn.execute('UPDATE pending_applies SET status=?, ack=NULL',(legacy_status,))
    o,lg=_restart(world); o.recover()
    u.scripts[a['attempt_id']]=[{'status':'COMPLETED','progress':{'phase':'DONE'}}]
    for _ in range(3): o.poll()
    assert lg.get_attempt(a['attempt_id'])['substatus']=='RELEASED'
    expected='COMPLETED' if legacy_status=='ACKED' else 'PENDING'
    assert lg.get_task(t.task_id).purpose_status==expected

@pytest.mark.parametrize('stop_point',['before_aux_finish','after_aux_log'])
def test_auxiliary_apply_event_is_not_duplicated_after_restart(world,stop_point):
    o,lg,env=world['orch'],world['ledger'],world['env']
    env.fire_cells[0]['fire_state']='UNBURNED'
    t=submit(o,requirements={'resource_types':['UAV'],'sensor':'THERMAL','extra_sensors':['WEATHER']})
    a=o.dispatch(t.task_id)
    o.poll(); o.poll()
    class Stop(BaseException): pass
    if stop_point=='before_aux_finish':
        real=lg.finish_pending_apply
        def f(oid,status,reason=None):
            if reason=='AUXILIARY_MEASUREMENT': raise Stop()
            return real(oid,status,reason)
        lg.finish_pending_apply=f
    else:
        real=lg.log
        def f(event,*args,**kw):
            result=real(event,*args,**kw)
            if event=='ENVIRONMENT_APPLY' and kw.get('detail',{}).get('attributed_to_purpose') is False:
                raise Stop()
            return result
        lg.log=f
    with pytest.raises(Stop): o.poll()
    aux=lg.get_observation(a['attempt_id'],'AUX')
    o,lg=_restart(world); o.recover()
    events=[e for e in _ev(lg,t.task_id,'ENVIRONMENT_APPLY')
            if e['detail'].get('observation_id')==aux['observation_id'] and e['result']=='ACK']
    assert len(env.applied)==2  # one main + one auxiliary; no duplicated physical application
    assert len(_ev(lg,t.task_id,'TASK_COMPLETE'))==1
    assert len(events)==1
