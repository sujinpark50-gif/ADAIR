"""총괄 전용 실행 흐름. 기존 Safety를 주입받고, 실행의 시작과 완료를 구분한다."""
import time
from .selector import DeterministicSelector


class Orchestrator:
    def __init__(self, environment, uav, ground, safety, logger, selector_factory=DeterministicSelector):
        self.environment, self.uav, self.ground = environment, uav, ground
        self.safety, self.logger = safety, logger
        self.selector = selector_factory(environment, uav, ground, self.emit)

    def emit(self, event, task, did=None, rid=None, result='', reason=None, detail=None):
        self.logger.log_event(event, time.time(), task_id=task.task_id, decision_id=did,
                              resource_id=rid, result=result, reason=reason, detail=detail)

    async def run(self, task):
        self.emit('TASK_CREATED', task, result='READY')
        excluded = set()
        while True:
            choice = self.selector.select(task, excluded)
            if choice is None:
                return {'status': 'MANUAL_REVIEW_REQUIRED', 'task_id': task.task_id}
            decision = choice.decision
            assignment = decision.assignments[0]
            rid, did = assignment.resource_id, decision.decision_id
            try:
                verdict = self.safety(decision)
                self.emit('SAFETY_JUDGEMENT', task, did, rid, result=verdict.response,
                          reason=verdict.reason)
                if verdict.response != 'ALLOW':
                    excluded.add(rid)
                    continue
                task.state = 'RUNNING'
                self.emit('EXECUTION', task, did, rid, result='REQUESTED')
                if choice.resource_type == 'UAV':
                    result = await self.uav.execute_and_wait(task.task_id, did, assignment)
                    self.emit('EXECUTION_RESULT', task, did, rid, result=result['status'])
                    if result['status'] == 'COMPLETED':
                        observation = result.get('observation') or {}
                        self.emit('OBSERVATION', task, did, rid, result='RECEIVED', detail={'raw': observation})
                        update = self.environment.apply_uav_observation(rid, observation)
                        self.emit('ENVIRONMENT_UPDATE', task, did, rid,
                                  result='UPDATED' if update.get('success') else 'FAILED', detail=update)
                        if update.get('success'):
                            task.state = 'COMPLETED'
                            self.emit('TASK_COMPLETE', task, did, rid, result='COMPLETED')
                            return {'status': 'COMPLETED', 'resource_id': rid, 'task_id': task.task_id}
                    self.selector.manual(task, did, 'OBSERVATION_FEASIBILITY_UNKNOWN', rid,
                                         detail={'execution_status': result['status']})
                else:
                    result = await self.ground.execute_and_wait(rid, task.target_node)
                    self.emit('EXECUTION_RESULT', task, did, rid, result=result['status'], detail=result)
                    # 도착과 ROAD_STATUS를 THERMAL/FIRE 관측 완료로 바꾸지 않는다.
                    self.selector.manual(task, did, 'OBSERVATION_FEASIBILITY_UNKNOWN', rid,
                        detail={'execution_status': result['status'],
                                'message': '현재 UGV 관측은 ROAD_STATUS이며 화재 검출을 제공하지 않음'})
                    return {'status': 'ARRIVED_OBSERVATION_PENDING' if result['status'] == 'ARRIVED' else 'MANUAL_REVIEW_REQUIRED',
                            'resource_id': rid, 'task_id': task.task_id}
            except Exception as exc:
                # 실행 요청 이후 실패는 실행 중단을 뜻하지 않으므로 재출동하지 않는다.
                self.selector.manual(task, did, 'OBSERVATION_FEASIBILITY_UNKNOWN', rid,
                                     detail={'error': str(exc), 'automatic_redispatch': False})
            return {'status': 'MANUAL_REVIEW_REQUIRED', 'task_id': task.task_id}
