# -*- coding: utf-8 -*-
"""시험용 가짜 UAV/UGV 서버. 요청·응답 형식은 uav/uav-agent/models.py, ugv/api_models.py 를 따른다.

httpx.MockTransport 로 붙여서, 총괄이 실제 HTTP 형식으로 주고받는지까지 검사한다.
"""

import json
import re
from datetime import datetime, timezone

import httpx


def iso_now():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class FakeUav:
    """uav_id → 상태. verdicts[rid] 는 payload→응답 dict 함수 또는 고정 dict."""

    def __init__(self, ids=("A-uav1", "A-uav2"), positions=None):
        self.ids = list(ids)
        self.state = {}
        for i, rid in enumerate(self.ids):
            lat, lon = (positions or {}).get(rid, (38.12 + 0.01 * i, 128.20))
            self.state[rid] = {"lat": lat, "lon": lon, "alt": 235.0, "battery": 95.0, "failsafe": False,
                               "link": "OK", "current_task_id": None, "timestamp": None}
        self.verdicts = {rid: {"verdict": "ACCEPT", "eta_sec": 600} for rid in self.ids}
        self.execute_mode = {rid: "ok" for rid in self.ids}   # ok / timeout / 409 / 500
        self.scripts = {}          # exec task_id → [TaskStatus dict ...] (GET 할 때마다 하나씩 소비)
        self.default_script = None
        self.calls = []            # (method, rid, path, body)
        self.unreachable = set()

    def endpoints(self):
        return {rid: f"http://uav-{rid}" for rid in self.ids}

    def _state_json(self, rid):
        s = self.state[rid]
        return {"uav_id": rid, "timestamp": s["timestamp"] or iso_now(),
                "position": {"lat": s["lat"], "lon": s["lon"], "alt_m_amsl": s["alt"], "alt_m_agl": 0.0},
                "battery": {"percent": s["battery"], "voltage": 16.0}, "velocity_ms": 0.0,
                "flight_mode": "HOLD", "armed": False,
                "health": {"gps_ok": True, "sensors_ok": True, "failsafe": s["failsafe"]},
                "wind_ms": None, "link_quality": s["link"], "current_task_id": s["current_task_id"]}

    def handler(self, request: httpx.Request) -> httpx.Response:
        m = re.match(r"/uav/([^/]+)(/.*)", request.url.path)
        rid, path = m.group(1), m.group(2)
        body = json.loads(request.content) if request.content else None
        self.calls.append((request.method, rid, path, body))
        if rid in self.unreachable:
            raise httpx.ConnectError("unreachable", request=request)
        if path == "/state":
            return httpx.Response(200, json=self._state_json(rid))
        if path == "/evaluate":
            v = self.verdicts[rid]
            v = v(body) if callable(v) else dict(v)
            return httpx.Response(200, json={"task_id": body["task_id"], "decision_id": body["decision_id"],
                                             "uav_id": rid, "constraints": {}, **v})
        if path == "/execute":
            mode = self.execute_mode[rid]
            if mode == "timeout":
                raise httpx.ReadTimeout("timeout", request=request)
            if mode == "409":
                return httpx.Response(409, json={"detail": "task already running"})
            if mode == "500":
                return httpx.Response(500, text="boom")
            tid = body["task_id"]
            if tid not in self.scripts and self.default_script:
                self.scripts[tid] = [dict(x) for x in self.default_script(rid, body)]
            self.state[rid]["current_task_id"] = tid
            return httpx.Response(200, json={"task_id": tid, "status": "STARTED",
                                             "tracking_url": f"/uav/{rid}/task/{tid}"})
        if path.startswith("/task/") and path.endswith("/abort"):
            # 드론팀 UAV-09: 관측 전이면 FAILED/ABORTED + 복귀 중, 아니면 ALREADY_RETURNING (같은 요청 반복은 처음 결과)
            tid = path.split("/")[2]
            seq = self.scripts.get(tid)
            if not seq:
                return httpx.Response(404, json={"detail": "not found"})
            if not seq[0].get("abort"):
                before_obs = seq[0]["status"] in ("STARTED", "IN_PROGRESS")
                rec = {"reason": (body or {}).get("reason"),
                       "result": "ABORTED_BEFORE_OBSERVATION" if before_obs else "ALREADY_RETURNING"}
                if before_obs:
                    self.scripts[tid] = [{"status": "FAILED", "reason": "ABORTED", "observation": None,
                                          "progress": {"phase": "RETURNING"}, "abort": rec}]
                    self.state[rid]["current_task_id"] = None
                else:
                    for x in seq:
                        x["abort"] = rec
            cur = self.scripts[tid][0]
            return httpx.Response(200, json={"task_id": tid, "observation": None, "progress": None, **cur})
        if path.startswith("/task/"):
            tid = path.split("/")[-1]
            seq = self.scripts.get(tid)
            if not seq:
                return httpx.Response(404, json={"detail": "not found"})
            cur = seq.pop(0) if len(seq) > 1 else seq[0]
            if isinstance(cur.get("progress"), dict) and cur["progress"].get("phase") == "DONE":
                self.state[rid]["current_task_id"] = None
            return httpx.Response(200, json={"task_id": tid, "observation": None, "progress": None, **cur})
        return httpx.Response(404)

    def client(self):
        return httpx.Client(transport=httpx.MockTransport(self.handler))

    def exec_calls(self):
        return [c for c in self.calls if c[2] == "/execute"]

    def eval_calls(self):
        return [c for c in self.calls if c[2] == "/evaluate"]


def normal_flight(target_ground=600.0, agl=80.0, margin=10.0):
    """ENROUTE → OBSERVING → COMPLETED(RETURNING) → DONE"""
    def script(rid, body):
        t = body["target"]
        pos = {"lat": t["lat"], "lon": t["lon"], "alt_m_amsl": t["alt_m_amsl"] + t["target_agl_m"] + margin}
        return [
            {"status": "IN_PROGRESS", "progress": {"phase": "ENROUTE"}},
            {"status": "IN_PROGRESS", "progress": {"phase": "OBSERVING"}},
            {"status": "COMPLETED", "progress": {"phase": "RETURNING"},
             "observation": {"position": pos, "sensor_type": "THERMAL",
                             "values": {"max_temp_c": 312.5, "hotspot_detected": True}}},
            {"status": "COMPLETED", "progress": {"phase": "DONE"},
             "observation": {"position": pos, "sensor_type": "THERMAL",
                             "values": {"max_temp_c": 312.5, "hotspot_detected": True}}},
        ]
    return script


class FakeUgv:
    """ugv/server.py 형식. scripts[exec task_id] 로 진행을 흉내 낸다."""

    def __init__(self, ids=("A-ugv1",)):
        self.units = {rid: {"state": "READY", "lat": 38.11, "lon": 128.19, "current_task_id": None}
                      for rid in ids}
        self.verdicts = {rid: {"verdict": "ACCEPT", "eta_sec": 300} for rid in ids}
        self.default_script = None
        self.scripts = {}
        self.calls = []

    def handler(self, request):
        import time
        path = request.url.path
        body = json.loads(request.content) if request.content else None
        self.calls.append((request.method, path, body))
        if path == "/ugv":
            return httpx.Response(200, json=[{
                "resource_id": rid, "resource_type": "UGV", "base": "A", "state": u["state"],
                "position": {"lat": u["lat"], "lon": u["lon"]}, "fuel_pct": 90.0, "current_node": "A",
                "current_task_id": u["current_task_id"], "driver": "sim", "driver_status": "IDLE",
                "updated_at": time.time()} for rid, u in self.units.items()])
        m = re.match(r"/ugv/([^/]+)/(evaluate|execute|task)(?:/(.+))?", path)
        rid, op, tid = m.group(1), m.group(2), m.group(3)
        node = {"node_id": "N1", "lat": 38.07, "lon": 128.20, "snap_m": 10.0}
        if op == "evaluate":
            return httpx.Response(200, json={"task_id": body["task_id"], "decision_id": body["decision_id"],
                                             "resource_id": rid, "detail": None, "target_node": node,
                                             "path": ["A", "N1"], "reason": None, "eta_sec": None,
                                             **self.verdicts[rid]})
        if op == "execute":
            self.units[rid]["current_task_id"] = body["task_id"]
            self.units[rid]["state"] = "RUNNING"
            self.scripts[body["task_id"]] = list(self.default_script(rid)) if self.default_script else []
            return httpx.Response(200, json={"task_id": body["task_id"], "resource_id": rid, "status": "STARTED",
                                             "tracking_url": "", "target_node": node, "eta_sec": 300})
        seq = self.scripts.get(tid)
        if not seq:
            return httpx.Response(404, json={"detail": "not found"})
        cur = seq.pop(0) if len(seq) > 1 else seq[0]
        eff = cur.pop("_unit", None)
        if eff:
            self.units[rid].update(eff)
        return httpx.Response(200, json={"task_id": tid, "resource_id": rid, "target_node": "N1",
                                         "progress": None, "observation": None, "error": None, **cur})

    def client(self):
        return httpx.Client(transport=httpx.MockTransport(self.handler))
