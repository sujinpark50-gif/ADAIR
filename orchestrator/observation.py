# -*- coding: utf-8 -*-
"""
orchestrator/observation.py
===========================
모의 관측 어댑터 (요청서 §8, 사용자 결정 2B).

- 물리 도착이 따로 확인된 뒤에만 부른다.
- 결과는 항상 source=SIMULATED, 센서 profile 은 TEST_ONLY 다.
- 관측 범위(footprint) 밖의 환경 정답은 쓰지 않는다. 범위 안에서 본 칸과 못 본 칸을 모두 남긴다.
- footprint 는 profile 기준 높이(50m)의 사각형을 실제 지면 위 높이에 비례해 다시 계산한다
  (같은 시야각의 기하 계산). 실제 높이를 모르면 관측을 만들지 않는다.
- UAV 가 돌려주는 고정 관측값(312.5℃ / hotspot)은 근거로 쓰지 않고 원문만 보관한다.

공간 계산: 관측 위치 기준 국지 평면 근사(등장방형). 수 km 이내 판단용이며 방법을 결과에 기록한다.
격자 칸은 중심 위경도와 한 변 길이(cell_size_m)로 정사각형으로 본다.
"""

import math
import time
import uuid
from datetime import datetime, timezone
from typing import Optional

from . import config

_M_PER_DEG_LAT = 111_320.0


def _to_local_m(lat0, lon0, lat, lon):
    return ((lon - lon0) * _M_PER_DEG_LAT * math.cos(math.radians(lat0)), (lat - lat0) * _M_PER_DEG_LAT)


def footprint_for_agl(profile: dict, agl_m: float) -> dict:
    scale = agl_m / profile["agl_m"]
    return {"shape": profile["footprint_shape"], "width_m": profile["footprint_width_m"] * scale,
            "height_m": profile["footprint_height_m"] * scale, "agl_m": agl_m,
            "scaled_from_profile_agl_m": profile["agl_m"], "heading_deg": 0.0,
            "method": "LOCAL_EQUIRECTANGULAR_AXIS_ALIGNED"}


def _overlaps(fp: dict, cx: float, cy: float, half_cell: float) -> bool:
    return abs(cx) <= fp["width_m"] / 2 + half_cell and abs(cy) <= fp["height_m"] / 2 + half_cell


def simulate(*, snapshot, task, attempt_id: str, resource_id: str, position: dict,
             uav_raw_observation: Optional[dict] = None,
             profile_id: str = config.DEFAULT_SENSOR_PROFILE_ID) -> dict:
    profile = config.SENSOR_PROFILES[profile_id]
    base = {
        "observation_id": f"OBS-{uuid.uuid4().hex[:12]}", "run_id": snapshot.run_id,
        "state_version_seen": snapshot.state_version, "simulation_time_s": snapshot.simulation_time_s,
        "observed_wall": datetime.now(timezone.utc).isoformat(), "task_id": task.task_id,
        "attempt_id": attempt_id, "resource_id": resource_id, "source": "SIMULATED",
        "sensor_profile_id": profile_id, "sensor_status": profile["status"],
        "sensor_type": profile["sensor_type"], "position": position,
        "uav_reported_values_ignored": (uav_raw_observation or {}).get("values"),
    }
    lat, lon, alt = position.get("lat"), position.get("lon"), position.get("alt_m_amsl")
    ground = task.target.ground_amsl_m
    if None in (lat, lon, alt) or ground is None:
        return {**base, "result": "FAILED", "failure_reason": "POSITION_OR_GROUND_UNKNOWN",
                "footprint": None, "covered_cells": [], "detections": [], "target_covered": False}
    agl = alt - ground
    if agl <= 0:
        return {**base, "result": "FAILED", "failure_reason": "NON_POSITIVE_AGL",
                "footprint": None, "covered_cells": [], "detections": [], "target_covered": False}
    fp = footprint_for_agl(profile, agl)
    covered, detections = [], []
    for cell in snapshot.fire_cells + snapshot.risk_cells:
        if cell.get("lat") is None or cell.get("lon") is None or not cell.get("cell_size_m"):
            continue
        cx, cy = _to_local_m(lat, lon, cell["lat"], cell["lon"])
        if _overlaps(fp, cx, cy, cell["cell_size_m"] / 2):
            covered.append(cell["cell_id"])
            if cell.get("fire_state") in ("BURNING", "BURNED"):
                detections.append({"cell_id": cell["cell_id"], "fire_state": cell["fire_state"]})
    tx, ty = _to_local_m(lat, lon, task.target.lat, task.target.lon)
    target_covered = abs(tx) <= fp["width_m"] / 2 and abs(ty) <= fp["height_m"] / 2
    result = "DETECTED" if detections else ("NOT_DETECTED" if covered or target_covered else "NO_COVERAGE")
    return {**base, "result": result, "footprint": fp, "covered_cells": covered,
            "detections": detections, "target_covered": target_covered,
            "footprint_center": {"lat": lat, "lon": lon}}
