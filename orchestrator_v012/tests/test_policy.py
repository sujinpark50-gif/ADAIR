import asyncio
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from interfaces.schema import ResourceStatus, SafetyVerdict
from orchestrator_v012.models import FireTask, UavEvaluation
from orchestrator_v012.selector import DeterministicSelector, haversine_m
from orchestrator_v012.runner import Orchestrator


def resource(rid, lat=38.1, lon=128.1, kind='UAV'):
    return ResourceStatus(rid, kind, rid[0], lat, lon)


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.task = FireTask('TASK_ONE', 0, target_lat=38.1, target_lon=128.1, target_node='F1')
        self.env = Mock(); self.env.snapshot.return_value = SimpleNamespace(wind_speed=5, timestamp=0)
        self.uav = Mock(); self.uav.get_status.return_value = []
        self.ground = Mock(); self.ground.statuses.return_value = []
        self.ground.position_of_node.return_value = (38.1, 128.1)
        self.events = []
        self.selector = DeterministicSelector(self.env, self.uav, self.ground, lambda *a, **kw: self.events.append((a, kw)))

    def aerial(self, resources, responses):
        self.uav.get_status.side_effect = lambda base: [r for r in resources if r.base == base]
        def evaluate(tid, did, assignment, wind_ms):
            verdict, reason = responses.get(assignment.resource_id, ('ACCEPT', None))
            return UavEvaluation(assignment.resource_id, verdict, reason, task_id=tid, decision_id=did, eta_sec=12)
        self.uav.evaluate.side_effect = evaluate

    def ground_candidates(self, etas):
        self.ground.statuses.return_value = [resource(rid, kind='UGV') for rid in etas]
        self.ground.evaluate.side_effect = lambda rid, node: {'resource_id': rid, 'response': 'ACCEPT', 'eta_s': etas[rid], 'path': ['A', node]}

    def test_nearest_uav_not_input_order_or_derived_state(self):
        far = resource('A-uav1', lat=38.2)
        near = resource('B-uav1'); near.state = 'OFFLINE'  # 파생 상태를 feasibility로 사용하지 않음
        self.aerial([far, near], {})
        selected = self.selector.select(self.task, set())
        self.assertEqual(selected.decision.assignments[0].resource_id, 'B-uav1')
        self.assertEqual(self.uav.evaluate.call_count, 1)

    def test_moderate_wind_exact_boundaries_preserve_counter(self):
        for wind, expected in [(7.999, False), (8.0, True), (9.999, True), (10.0, False)]:
            with self.subTest(wind=wind):
                self.setUp(); self.env.snapshot.return_value.wind_speed = wind
                self.aerial([resource('A-uav1')], {'A-uav1': ('COUNTER', 'MODERATE_WIND')})
                choice = self.selector.select(self.task, set())
                self.assertEqual(choice is not None, expected)
                if choice:
                    self.assertEqual(choice.local_response.response, 'COUNTER')
                    self.assertTrue(any(k.get('reason') == 'V012_MODERATE_WIND_ACCEPTED' for a,k in self.events))
                if wind == 10:
                    self.uav.get_status.assert_not_called(); self.uav.evaluate.assert_not_called()

    def test_other_counters_are_not_accepted(self):
        for reason in ('RETURN_MARGIN_INSUFFICIENT', 'DEADLINE_TIGHT'):
            with self.subTest(reason=reason):
                self.setUp(); self.env.snapshot.return_value.wind_speed = 9
                self.aerial([resource('A-uav1'), resource('B-uav1', lat=38.11)], {'A-uav1': ('COUNTER', reason)})
                excluded = set(); choice = self.selector.select(self.task, excluded)
                self.assertEqual(choice.decision.assignments[0].resource_id, 'B-uav1')
                self.assertEqual(excluded, {'A-uav1'})

    def test_four_rejected_uavs_are_exhausted_before_ground(self):
        resources = [resource(rid) for rid in ['A-uav1','A-uav2','B-uav1','B-uav2']]
        self.aerial(resources, {r.resource_id: ('REJECT', 'LOW_BATTERY') for r in resources})
        self.ground_candidates({'A-ugv1': 46})
        excluded = set(); choice = self.selector.select(self.task, excluded)
        self.assertEqual(choice.resource_type, 'UGV')
        self.assertEqual(self.uav.evaluate.call_count, 4)
        self.assertEqual(self.env.snapshot.call_count, 5)
        calls = self.uav.evaluate.call_args_list
        self.assertEqual({c.args[0] for c in calls}, {'TASK_ONE'})
        self.assertEqual(len({c.args[1] for c in calls}), 4)
        self.assertEqual(len(excluded), 4)

    def test_refresh_can_switch_to_ground_after_reject(self):
        self.env.snapshot.side_effect = [SimpleNamespace(wind_speed=5, timestamp=1), SimpleNamespace(wind_speed=10, timestamp=2)]
        self.aerial([resource('A-uav1'), resource('B-uav1')], {'A-uav1': ('REJECT', 'HIGH_WIND')})
        self.ground_candidates({'B-ugv1': 20})
        choice = self.selector.select(self.task, set())
        self.assertEqual(choice.resource_type, 'UGV')
        self.assertEqual(self.uav.evaluate.call_count, 1)

    def test_ground_minimum_eta_and_id_tie(self):
        for etas, expected in [({'B-ugv1': 10, 'A-ugv1': 46}, 'B-ugv1'), ({'B-ugv1': 10, 'A-ugv1': 10}, 'A-ugv1')]:
            with self.subTest(etas=etas):
                self.setUp(); self.ground_candidates(etas)
                for r in self.ground.statuses.return_value:
                    r.capability = {'reachable': False, 'road_blocked': True}
                choice = self.selector.select(self.task, set())
                self.assertEqual(choice.decision.assignments[0].resource_id, expected)

    def test_observation_distance_boundary(self):
        for distance, accepted in [(1999.9, True), (2000, True), (2000.1, False)]:
            with self.subTest(distance=distance):
                self.setUp(); self.ground_candidates({'A-ugv1': 46})
                with patch('orchestrator_v012.selector.haversine_m', return_value=distance):
                    result = self.selector.select(self.task, set())
                self.assertEqual(result is not None, accepted)
                if not accepted:
                    self.assertEqual(self.events[-1][1]['reason'], 'OBSERVATION_DISTANCE_EXCEEDED')

    def test_missing_target_and_invalid_wind_request_manual_review(self):
        self.task.target_node = None
        self.assertIsNone(self.selector.select(self.task, set()))
        self.assertEqual(self.events[-1][1]['reason'], 'OBSERVATION_FEASIBILITY_UNKNOWN')
        self.env.snapshot.return_value.wind_speed = float('nan')
        self.assertIsNone(self.selector.select(self.task, set()))
        self.uav.evaluate.assert_not_called()

    def test_response_id_mismatch_cannot_be_selected(self):
        self.aerial([resource('A-uav1')], {})
        self.uav.evaluate.side_effect = None
        self.uav.evaluate.return_value = UavEvaluation('WRONG', 'ACCEPT', task_id='WRONG', decision_id='WRONG')
        self.assertIsNone(self.selector.select(self.task, set()))

    def test_safety_reject_never_executes_and_new_decision_is_created(self):
        self.aerial([resource('A-uav1')], {})
        logger = Mock()
        runner = Orchestrator(self.env, self.uav, self.ground, lambda d: SafetyVerdict('REJECT'), logger)
        result = asyncio.run(runner.run(self.task))
        self.assertEqual(result['status'], 'MANUAL_REVIEW_REQUIRED')
        self.uav.execute_and_wait.assert_not_called()
        self.ground.execute_and_wait.assert_not_called()
        self.assertEqual(self.env.snapshot.call_count, 2)

    def test_haversine_known_distance_and_invalid_position(self):
        self.assertAlmostEqual(haversine_m(0, 0, 0, 1), 111194.9266, places=3)
        self.assertEqual(haversine_m(38,128,38,128), 0)
        with self.assertRaises(ValueError): haversine_m(91,0,0,0)


if __name__ == '__main__': unittest.main()
