# ugv/drive_log.py — 주행 정밀 기록 (task 1개 = CSV 1개)
#
# 목적: Gazebo 에서 실제로 어떻게 달렸는지 수치로 본다.
#   - 시간: 벽시계 / 서버 시뮬레이션 시각 / PX4 시각(imu timestamp) → 실제 배속, ETA 대비 실제
#   - 위치: 경로선에서 좌(+)/우(-) 부호 있는 횡오차 → 도로 중심에서 한쪽으로 치우치는지
#   - 속도: 실제 속도 vs 명령 속도
# 파일: ugv/.state/drive_logs/<task_id>.csv  (UGV_DRIVE_LOG=0 이면 끔)
# 분석: python3 -m ugv.tools.analyze_drive_log ugv/.state/drive_logs/<task_id>.csv

import csv
import math
import os
import time

from . import route_plan

ENABLED = os.getenv("UGV_DRIVE_LOG", "1") != "0"
INTERVAL_S = float(os.getenv("UGV_DRIVE_LOG_INTERVAL_S", "0.5"))   # 벽시계 기준 기록 간격

COLUMNS = ["wall_s", "server_sim_s", "px4_time_s", "lat", "lon", "north_m", "east_m",
           "offset_m", "segment", "waypoint", "total", "remaining_m", "eta_remaining_sec",
           "speed_mps", "cmd_speed_mps", "heading_deg", "road_id", "state"]


def _r(v, nd):
    return "" if v is None or (isinstance(v, float) and math.isnan(v)) else round(v, nd)


class DriveLog:
    def __init__(self, state_dir: str, task_id: str, agent, clock):
        self.agent, self.clock = agent, clock
        self.path = os.path.join(state_dir, "drive_logs", f"{task_id}.csv")
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self._f = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(COLUMNS)
        r = agent.resource
        self._origin = (r.lat, r.lon)
        self._t0 = time.monotonic()
        self._last = -1e9
        self.start = self._stamp()
        self.end: dict | None = None

    def _stamp(self) -> dict:
        tel = self.agent.driver.telemetry()
        px4 = tel.get("px4_time_s")
        return {"wall_s": round(time.monotonic() - self._t0, 2),
                "wall_epoch": round(time.time(), 2),
                "server_sim_s": round(self.clock.now(), 2),
                "px4_time_s": None if px4 is None else round(px4, 3)}

    def _ne(self, lat, lon):
        lat0, lon0 = self._origin
        north = (lat - lat0) * 111_320.0
        east = (lon - lon0) * 111_320.0 * math.cos(math.radians(lat0))
        return north, east

    def sample(self, progress: dict, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last < INTERVAL_S:
            return
        self._last = now
        a, r = self.agent, self.agent.resource
        tel = a.driver.telemetry()
        here = (r.lat, r.lon)
        off, seg = (None, None)
        if getattr(a, "route", None) and len(a.route) >= 2:
            off, seg = route_plan.signed_offset_m(here, a.route)
        cur = progress.get("waypoint")
        cmd = None
        plan = a.plan
        if plan is not None and plan.speeds and cur is not None:
            cmd = plan.speeds[min(cur, len(plan.speeds) - 1)]
        n, e = self._ne(*here)
        self._w.writerow([
            round(now - self._t0, 2), round(self.clock.now(), 2), _r(tel.get("px4_time_s"), 3),
            f"{r.lat:.7f}", f"{r.lon:.7f}", round(n, 2), round(e, 2),
            _r(off, 2), "" if seg is None else seg, cur if cur is not None else "", progress.get("total", ""),
            progress.get("remaining_m", ""), progress.get("eta_remaining_sec", ""),
            _r(tel.get("speed_mps"), 2), _r(cmd, 2), _r(tel.get("heading_deg"), 1),
            progress.get("current_road_id") or "", r.state])
        self._f.flush()

    def close(self, eta_sec: float | None) -> dict:
        """종료 시각을 찍고 task 기록에 넣을 timing 요약을 돌려준다."""
        if self.end is None:
            self.end = self._stamp()
            self._f.close()
        s, e = self.start, self.end
        out = {"start": s, "end": e, "eta_sec": eta_sec,
               "wall_elapsed_s": round(e["wall_s"] - s["wall_s"], 1),
               "server_sim_elapsed_s": round(e["server_sim_s"] - s["server_sim_s"], 1)}
        if s["px4_time_s"] is not None and e["px4_time_s"] is not None:
            px4 = e["px4_time_s"] - s["px4_time_s"]
            out["px4_elapsed_s"] = round(px4, 1)
            if out["wall_elapsed_s"] > 0:
                out["real_time_factor"] = round(px4 / out["wall_elapsed_s"], 2)
            if eta_sec:
                out["actual_over_eta"] = round(px4 / eta_sec, 2)
        elif eta_sec:
            out["actual_over_eta"] = round(out["server_sim_elapsed_s"] / eta_sec, 2)
        return out
