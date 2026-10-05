"""기존 HTTP 서버와 CA/Raster/GroundFleet를 실제 호출하는 연결 검증."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest

from orchestrator_v012.demo import mock_servers, scenario, SCENARIOS
from orchestrator_v012.runtime import EnvironmentAdapter


class ConnectedTests(unittest.TestCase):
    def test_seven_connected_scenarios(self):
        output = Path('logs/orchestrator_v012_verified').resolve()
        output.mkdir(parents=True, exist_ok=True)
        results = []
        with mock_servers(output) as uav:
            for name in SCENARIOS:
                with self.subTest(scenario=name):
                    environment = EnvironmentAdapter()
                    version = environment.api.state_version
                    environment.snapshot(); environment.snapshot()
                    self.assertEqual(environment.api.state_version, version)
                    result, records = asyncio.run(scenario(name, uav, environment, output))
                    events = [r['event_type'] for r in records]
                    local = [r for r in records if r['event_type']=='LOCAL_RESPONSE']
                    if name in ('normal','moderate'):
                        self.assertEqual(result['status'], 'COMPLETED')
                        self.assertEqual(result['environment_version_after'], result['environment_version_before']+1)
                        self.assertLess(events.index('SAFETY_JUDGEMENT'), events.index('EXECUTION'))
                        self.assertLess(events.index('EXECUTION'), events.index('OBSERVATION'))
                        self.assertIn('ENVIRONMENT_UPDATE', events)
                        if name == 'moderate':
                            self.assertEqual(local[0]['result'], 'COUNTER')
                            self.assertEqual(local[0]['reason'], 'MODERATE_WIND')
                            self.assertTrue(any(r['reason']=='V012_MODERATE_WIND_ACCEPTED' for r in records))
                    elif name in ('strong','uav_rejected'):
                        self.assertEqual(result['status'], 'ARRIVED_OBSERVATION_PENDING')
                        self.assertEqual(result['environment_version_after'], result['environment_version_before'])
                        self.assertTrue(any(r['event_type']=='EXECUTION_RESULT' and r['result']=='ARRIVED' for r in records))
                        self.assertNotIn('TASK_COMPLETE', events)
                        self.assertIn('MANUAL_REVIEW_REQUIRED', events)
                        if name == 'uav_rejected':
                            rejects=[r for r in local if r['result']=='REJECT']
                            self.assertEqual(len(rejects),2)
                            self.assertEqual(len({r['resource_id'] for r in rejects}),2)
                            self.assertEqual(len({r['decision_id'] for r in rejects}),2)
                    else:
                        self.assertEqual(result['status'], 'MANUAL_REVIEW_REQUIRED')
                        expected={'blocked':'NO_AUTONOMOUS_CANDIDATE','distant':'OBSERVATION_DISTANCE_EXCEEDED','unknown_target':'OBSERVATION_FEASIBILITY_UNKNOWN'}[name]
                        self.assertTrue(any(r['event_type']=='MANUAL_REVIEW_REQUIRED' and r['reason']==expected for r in records))
                        self.assertNotIn('EXECUTION',events)
                    results.append({**result, 'verified': True})
        (output / 'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__': unittest.main()
