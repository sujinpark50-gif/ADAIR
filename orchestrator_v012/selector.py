"""단일 화재 Task 결정론적 fallback. 향후 선택기는 select 경계로 교체한다."""
import math
from dataclasses import asdict, dataclass

import ids
from interfaces.schema import Assignment, Decision

UGV_OBSERVATION_MAX_DISTANCE_M = 2000


def haversine_m(lat1, lon1, lat2, lon2):
    # px4/uav-agent/evaluator.py의 기존 공식 재사용. config import 충돌을 피한다.
    for lat, lon in ((lat1, lon1), (lat2, lon2)):
        if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError('유효하지 않은 위경도')
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin(math.radians(lat2-lat1)/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(math.radians(lon2-lon1)/2)**2
    return 2 * 6371000.0 * math.asin(math.sqrt(min(1.0, max(0.0, a))))


@dataclass
class Selection:
    decision: Decision
    resource_type: str
    local_response: object


class DeterministicSelector:
    def __init__(self, environment, uav, ground, emit):
        self.environment, self.uav, self.ground, self.emit = environment, uav, ground, emit

    def manual(self, task, decision_id, reason, resource_id=None, distance=None, detail=None):
        self.emit('MANUAL_REVIEW_REQUIRED', task, decision_id, resource_id,
                  reason=reason, detail={'fire_target': {'lat': task.target_lat, 'lon': task.target_lon},
                  **({'observation_distance_m': distance} if distance is not None else {}),
                  **(detail or {})})
        return None

    def select(self, task, excluded):
        # 실패 후 상태를 다시 조회한다. 제외 목록은 같은 Task 동안 유지한다.
        for _ in range(100):
            did = ids.new_decision_id()
            try:
                haversine_m(task.target_lat, task.target_lon, task.target_lat, task.target_lon)
                env = self.environment.snapshot()
                wind = env.wind_speed
                if not isinstance(wind, (int, float)) or not math.isfinite(wind) or wind < 0:
                    raise ValueError('Environment 풍속 판단 불가')
                aerial = self.uav.get_status('A') + self.uav.get_status('B') if wind < 10 else []
                self.emit('STATE_REFRESHED', task, did, detail={
                    'wind_ms': wind, 'excluded_ids': sorted(excluded),
                    'uav_ids': [r.resource_id for r in aerial],
                })
                if wind < 10:
                    ranked = sorted((
                        (haversine_m(task.target_lat, task.target_lon, r.location_lat, r.location_lon), r.resource_id)
                        for r in aerial if r.resource_id not in excluded
                    ))
                    if ranked:
                        distance, rid = ranked[0]
                        assignment = Assignment(task.task_id, rid, task.target_lat, task.target_lon, target_alt_m=task.target_alt_m)
                        response = self.uav.evaluate(task.task_id, did, assignment, wind_ms=wind)
                        self.emit('LOCAL_RESPONSE', task, did, rid, result=response.response,
                                  reason=response.reason, detail={'source': 'UAV_API', 'raw': asdict(response)})
                        if (response.resource_id, response.task_id, response.decision_id) != (rid, task.task_id, did):
                            raise ValueError('UAV Local 응답 식별자 불일치')
                        accepted = response.response == 'ACCEPT'
                        policy_counter = response.response == 'COUNTER' and response.reason == 'MODERATE_WIND' and 8 <= wind < 10
                        if accepted or policy_counter:
                            self.emit('DECISION', task, did, rid, result='SELECTED',
                                      reason='V012_MODERATE_WIND_ACCEPTED' if policy_counter else 'NEAREST_ACCEPTED_UAV',
                                      detail={'distance_m': distance, 'wind_ms': wind, 'policy_version': '0.1.2',
                                              'local_verdict': response.response})
                            return Selection(Decision(did, task.task_id, env.timestamp, [assignment]), 'UAV', response)
                        excluded.add(rid)
                        self.emit('REEVALUATION', task, did, rid, reason=response.reason,
                                  detail={'next_decision_will_refresh': True})
                        continue
                # GroundFleet는 목적지 노드가 필요하다. 임의의 새 변환 규칙은 만들지 않는다.
                ground = self.ground.statuses()
                self.emit('GROUND_STATE_REFRESHED', task, did,
                          detail={'ugv_ids': [r.resource_id for r in ground]})
                if task.target_node is None:
                    return self.manual(task, did, 'OBSERVATION_FEASIBILITY_UNKNOWN', detail={'message': 'target_node 미제공'})
                node_lat, node_lon = self.ground.position_of_node(task.target_node)
                distance = haversine_m(task.target_lat, task.target_lon, node_lat, node_lon)
                candidates = []
                for resource in sorted(ground, key=lambda r: r.resource_id):
                    rid = resource.resource_id
                    if resource.resource_type != 'UGV' or resource.state != 'READY' or rid in excluded:
                        continue
                    response = self.ground.evaluate(rid, task.target_node)
                    self.emit('LOCAL_RESPONSE', task, did, rid, result=response['response'], reason=response.get('reason'),
                              detail={'source': 'GroundFleet.evaluate', 'tracking_ids_source': 'call_context', 'raw': response})
                    if response['resource_id'] != rid:
                        raise ValueError('UGV 응답 식별자 불일치')
                    if response['response'] == 'ACCEPT':
                        eta = response.get('eta_s')
                        if not isinstance(eta, (int, float)) or not math.isfinite(eta) or eta < 0:
                            raise ValueError('UGV ETA 판단 불가')
                        candidates.append((eta, rid, response))
                    else:
                        excluded.add(rid)
                if not candidates:
                    return self.manual(task, did, 'NO_AUTONOMOUS_CANDIDATE')
                eta, rid, response = min(candidates, key=lambda c: (c[0], c[1]))
                if distance > UGV_OBSERVATION_MAX_DISTANCE_M:
                    return self.manual(task, did, 'OBSERVATION_DISTANCE_EXCEEDED', rid, distance)
                assignment = Assignment(task.task_id, rid, node_lat, node_lon)
                self.emit('DECISION', task, did, rid, result='SELECTED', reason='MINIMUM_ROAD_ETA', detail={
                    'eta_s': eta, 'eta_source': 'RoadGraph', 'target_node': task.target_node,
                    'observation_distance_m': distance, 'observation_max_m': 2000,
                    'distance_limit_is_simulation_assumption': True,
                })
                return Selection(Decision(did, task.task_id, env.timestamp, [assignment]), 'UGV', response)
            except (ValueError, TypeError, KeyError, OSError, RuntimeError) as exc:
                return self.manual(task, did, 'OBSERVATION_FEASIBILITY_UNKNOWN', detail={'error': str(exc)})
        return self.manual(task, did, 'NO_AUTONOMOUS_CANDIDATE', detail={'message': '재평가 보호 상한 도달'})
