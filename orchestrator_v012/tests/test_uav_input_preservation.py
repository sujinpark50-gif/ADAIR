"""UAV HTTP 응답 보존 계약. 네트워크/PX4 호출 없이 Connector 경계를 검증한다."""

import copy
import unittest
from unittest.mock import Mock, patch

import config
from connectors import ugv_connector
from interfaces.schema import Assignment, LocalResponse, ResourceStatus
from orchestrator_v012.uav_adapter import UavHttpAdapter


class UavInputPreservationTests(unittest.TestCase):
    def setUp(self):
        self.assignment = Assignment("TASK_REQUEST", "A-uav1", 38.1, 128.1)
        self.http = Mock()
        self.adapter = UavHttpAdapter(
            endpoints={"A-uav1": "http://uav.test"}, http=self.http,
        )

    def test_state_preserves_source_values_and_marks_derived_state(self):
        payload = {
            "uav_id": "A-uav1",
            "timestamp": "2026-09-23T00:00:00.123Z",
            "position": {"lat": 38.1, "lon": 128.1, "alt_m_amsl": 172.1},
            "battery": {"percent": 0.95},
            "health": {"gps_ok": False, "sensors_ok": False, "failsafe": False},
            "velocity_ms": 0.0,
            "flight_mode": "HOLD",
            "link_quality": "WEAK",
            "current_task_id": "TASK_ACTIVE",
        }
        original = copy.deepcopy(payload)
        self.http.get.return_value.json.return_value = payload

        status, = self.adapter.get_status("A")

        # 원본 API에 없는 READY는 수행 가능성 ACCEPT를 뜻하지 않는다.
        self.assertNotIn("state", payload)
        self.assertEqual(status.state, "READY")
        self.assertIsInstance(status, ResourceStatus)
        self.assertEqual(status.state_source, "uav_adapter:health.failsafe")
        self.assertEqual(status.source_timestamp, payload["timestamp"])
        self.assertEqual(status.current_task_id, "TASK_ACTIVE")
        self.assertEqual(status.capability["battery_pct"], 0.95)
        self.assertIs(status.capability["gps_ok"], False)
        self.assertIs(status.capability["sensors_ok"], False)
        self.assertEqual(status.capability["velocity_ms"], 0.0)
        self.assertEqual(status.capability["flight_mode"], "HOLD")
        self.assertEqual(status.comm_status, "WEAK")
        self.assertEqual(payload, original)
        self.http.get.assert_called_once_with(
            "http://uav.test/uav/A-uav1/state", timeout=5,
        )
        self.http.post.assert_not_called()

    def test_evaluate_preserves_verdict_ids_eta_and_counter_payload(self):
        cases = [
            ("ACCEPT", None, 0),
            ("REJECT", "HIGH_WIND", None),
            ("COUNTER", "MODERATE_WIND", 90),
            ("COUNTER", "RETURN_MARGIN_INSUFFICIENT", 120),
            ("COUNTER", "DEADLINE_TIGHT", 180),
        ]
        for verdict, reason, eta in cases:
            with self.subTest(verdict=verdict, reason=reason):
                payload = {
                    "uav_id": "A-uav1",
                    # 요청 ID로 덮어쓰면 응답 불일치를 추적할 수 없으므로 원문 보존.
                    "task_id": "TASK_RETURNED",
                    "decision_id": "DEC_RETURNED",
                    "verdict": verdict,
                    "reason": reason,
                    "eta_sec": eta,
                    "detail": "Local 판단 원문",
                    "counter_offer": {"observation_sec": 30} if verdict == "COUNTER" else None,
                    "constraints": {"wind_ms": 9.0, "eta_sec": 999},
                }
                original = copy.deepcopy(payload)
                self.http.post.reset_mock()
                self.http.post.return_value.json.return_value = payload

                response = self.adapter.evaluate(
                    "TASK_REQUEST", "DEC_REQUEST", self.assignment, wind_ms=9.0,
                )

                self.assertEqual(response.resource_id, payload["uav_id"])
                self.assertIsInstance(response, LocalResponse)
                self.assertEqual(response.task_id, "TASK_RETURNED")
                self.assertEqual(response.decision_id, "DEC_RETURNED")
                self.assertEqual(response.response, verdict)
                self.assertEqual(response.reason, reason)
                self.assertEqual(response.eta_sec, eta)
                self.assertEqual(response.detail, payload["detail"])
                self.assertEqual(response.counter_alternative, payload["counter_offer"])
                self.assertEqual(response.counter_constraint, payload["constraints"])
                self.assertEqual(payload, original)
                self.http.post.assert_called_once()
                args, kwargs = self.http.post.call_args
                self.assertEqual(args[0], "http://uav.test/uav/A-uav1/evaluate")
                self.assertEqual(kwargs["json"]["wind_ms"], 9.0)
                self.assertEqual(kwargs["json"]["task_id"], "TASK_REQUEST")
                self.assertEqual(kwargs["json"]["decision_id"], "DEC_REQUEST")

    def test_missing_optional_values_are_not_fabricated(self):
        self.http.post.return_value.json.return_value = {
            "uav_id": "A-uav1", "task_id": "TASK_1", "decision_id": "DEC_1",
            "verdict": "REJECT", "reason": "WIND_UNKNOWN", "constraints": {},
        }
        response = self.adapter.evaluate(
            "TASK_1", "DEC_1", self.assignment, wind_ms=None,
        )
        self.assertIsNone(response.eta_sec)
        self.assertIsNone(response.detail)
        self.assertIsNone(response.counter_alternative)
        self.assertEqual(response.counter_constraint, {})
        self.assertIsNone(self.http.post.call_args.kwargs["json"]["wind_ms"])

    def test_unknown_endpoint_does_not_make_a_request(self):
        adapter = UavHttpAdapter(endpoints={"A-uav1": "TBD"}, http=self.http)
        self.assertEqual(adapter.get_status("A"), [])
        with self.assertRaises(RuntimeError):
            adapter.evaluate("TASK_1", "DEC_1", self.assignment)
        self.http.get.assert_not_called()
        self.http.post.assert_not_called()

    def test_existing_local_response_and_ugv_calls_remain_compatible(self):
        legacy = LocalResponse("A-ugv1", "REJECT", "ROAD_BLOCKED", None, {})
        self.assertEqual(legacy.counter_constraint, {})
        self.assertFalse(hasattr(legacy, "task_id"))
        self.assertFalse(hasattr(legacy, "decision_id"))
        with patch.object(config, "UGV_CONNECTION_MODE", "mock"):
            response = ugv_connector.send_ugv_command(
                "TASK_1", Assignment("TASK_1", "A-ugv1", 38.1, 128.1),
                force_response="ACCEPT",
            )
        self.assertEqual(response.response, "ACCEPT")
        self.assertFalse(hasattr(response, "eta_sec"))


if __name__ == "__main__":
    unittest.main()
