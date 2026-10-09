# -*- coding: utf-8 -*-
"""
orchestrator/observation.py
===========================
모의 관측 어댑터 (요청서 §8, 사용자 결정 2B).

- 물리 도착이 따로 확인된 뒤에만 부른다.
- 결과는 항상 source=SIMULATED, 센서 profile 은 TEST_ONLY 다.
- 관측 범위(footprint) 밖의 환경 정답은 쓰지 않는다. 범위 안에서 본 칸과 못 본 칸을 모두 남긴다.
- 프레임이 칸 일부에만 겹치면 그 칸을 '전체 관측'으로 세지 않는다 (covered_cells 는 전체가 들어온 칸만,
  partial_cells 는 일부만 본 칸, cell_coverage 에 겹친 사각형과 넓이 비율). 비율은 기하 계산값이며 임계값을 두지 않는다.
- footprint 는 profile 기준 높이(90m, 52×42m)의 사각형을 실제 지면 위 높이에 비례해 다시 계산한다
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
    if profile.get("scale_with_agl") is False:
        # 가정 범위 (예: 원으로 한 바퀴 돌았다고 본 90×90m). 높이로 다시 계산하지 않고 가정임을 남긴다
        return {"shape": profile["footprint_shape"], "width_m": profile["footprint_width_m"],
                "height_m": profile["footprint_height_m"], "agl_m": agl_m, "scaled_from_profile_agl_m": None,
                "heading_deg": 0.0, "method": "ASSUMED_FIXED_FOOTPRINT", "assumption": profile["basis"]}
    scale = agl_m / profile["agl_m"]
    return {"shape": profile["footprint_shape"], "width_m": profile["footprint_width_m"] * scale,
            "height_m": profile["footprint_height_m"] * scale, "agl_m": agl_m,
            "scaled_from_profile_agl_m": profile["agl_m"], "heading_deg": 0.0,
            "method": "LOCAL_EQUIRECTANGULAR_AXIS_ALIGNED"}


_FULL_EPS_M = 1e-6


def cell_overlap(fp: dict, cx: float, cy: float, half_cell: float) -> Optional[dict]:
    """관측 프레임과 칸(정사각형)이 겹치는 부분. 겹치지 않으면 None.
    rect 는 칸 중심 기준 좌표(m) [x0, y0, x1, y1], fraction 은 칸 넓이 대비 겹친 넓이 (기하 계산값)."""
    hw, hh = fp["width_m"] / 2, fp["height_m"] / 2
    x0, x1 = max(-hw, cx - half_cell), min(hw, cx + half_cell)
    y0, y1 = max(-hh, cy - half_cell), min(hh, cy + half_cell)
    if x1 <= x0 or y1 <= y0:
        return None
    full = (x0 <= cx - half_cell + _FULL_EPS_M and x1 >= cx + half_cell - _FULL_EPS_M
            and y0 <= cy - half_cell + _FULL_EPS_M and y1 >= cy + half_cell - _FULL_EPS_M)
    side = 2 * half_cell
    return {"rect": [x0 - cx, y0 - cy, x1 - cx, y1 - cy], "full": full,
            "fraction": 1.0 if full else (x1 - x0) * (y1 - y0) / (side * side)}


def union_fraction(rects, cell_size_m: float) -> float:
    """칸 안에 쌓인 관측 사각형들의 합집합 넓이 / 칸 넓이. 겹친 부분을 두 번 세지 않는다 (좌표 압축)."""
    h = cell_size_m / 2
    clipped = [(max(-h, r[0]), max(-h, r[1]), min(h, r[2]), min(h, r[3])) for r in rects]
    clipped = [r for r in clipped if r[2] > r[0] and r[3] > r[1]]
    if not clipped:
        return 0.0
    xs = sorted({v for r in clipped for v in (r[0], r[2])})
    ys = sorted({v for r in clipped for v in (r[1], r[3])})
    area = 0.0
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            mx, my = (xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2
            if any(r[0] <= mx <= r[2] and r[1] <= my <= r[3] for r in clipped):
                area += (xs[i + 1] - xs[i]) * (ys[j + 1] - ys[j])
    frac = area / (cell_size_m * cell_size_m)
    return 1.0 if frac >= 1.0 - 1e-9 else frac


def simulate(*, snapshot, task, attempt_id: str, resource_id: str, position: dict,
             uav_raw_observation: Optional[dict] = None,
             profile_id: Optional[str] = None, cells: Optional[list] = None) -> dict:
    """cells 를 주면 (정답 상태를 붙인 정적 지도 칸, with_truth_state) 그 칸들로 범위를 계산한다.
    없으면 기존처럼 환경이 준 불·위험·예측 칸만 본다."""
    profile_id = profile_id or config.DEFAULT_SENSOR_PROFILE_ID
    profile = config.SENSOR_PROFILES[profile_id]
    base = {
        "observation_id": f"OBS-{uuid.uuid4().hex[:12]}", "run_id": snapshot.run_id,
        "state_version_seen": snapshot.state_version, "simulation_time_s": snapshot.simulation_time_s,
        "observed_wall": datetime.now(timezone.utc).isoformat(), "task_id": task.task_id,
        "attempt_id": attempt_id, "resource_id": resource_id, "source": "SIMULATED",
        "sensor_profile_id": profile_id, "sensor_status": profile["status"],
        "sensor_type": profile["sensor_type"], "position": position, "footprint_basis": profile["basis"],
        "uav_reported_values_ignored": (uav_raw_observation or {}).get("values"),
    }
    lat, lon, alt = position.get("lat"), position.get("lon"), position.get("alt_m_amsl")
    goal = task.nav_target                       # 이번 실행시도가 간 지점 (구역 임무는 방문 칸마다 다름)
    ground = goal.ground_amsl_m
    empty = {"footprint": None, "covered_cells": [], "partial_cells": [], "cell_coverage": [],
             "detections": [], "target_covered": False}
    if None in (lat, lon, alt) or ground is None:
        return {**base, "result": "FAILED", "failure_reason": "POSITION_OR_GROUND_UNKNOWN", **empty}
    agl = alt - ground
    if agl <= 0:
        return {**base, "result": "FAILED", "failure_reason": "NON_POSITIVE_AGL", **empty}
    fp = footprint_for_agl(profile, agl)
    # covered_cells  칸 전체가 프레임 안에 들어온 칸 (그 칸에 대해 '불 없음'을 말할 수 있다)
    # partial_cells  일부만 겹친 칸 (본 부분의 사실만 남긴다. 칸 전체 관측으로 세지 않는다)
    covered, partial, coverage, detections, seen = [], [], [], [], set()
    for cell in (cells if cells is not None else snapshot.fire_cells + snapshot.risk_cells + snapshot.spread_forecast):
        if cell.get("lat") is None or cell.get("lon") is None or not cell.get("cell_size_m"):
            continue
        if cell["cell_id"] in seen:
            continue
        cx, cy = _to_local_m(lat, lon, cell["lat"], cell["lon"])
        ov = cell_overlap(fp, cx, cy, cell["cell_size_m"] / 2)
        if ov is None:
            continue
        seen.add(cell["cell_id"])
        (covered if ov["full"] else partial).append(cell["cell_id"])
        coverage.append({"cell_id": cell["cell_id"], "cell_size_m": cell["cell_size_m"], "full": ov["full"],
                         "fraction": round(ov["fraction"], 6), "rect": [round(v, 3) for v in ov["rect"]]})
        if cell.get("fire_state") in ("BURNING", "BURNED"):
            detections.append({"cell_id": cell["cell_id"], "fire_state": cell["fire_state"]})
    tx, ty = _to_local_m(lat, lon, goal.lat, goal.lon)
    target_covered = abs(tx) <= fp["width_m"] / 2 and abs(ty) <= fp["height_m"] / 2
    result = "DETECTED" if detections else ("NOT_DETECTED" if covered or partial or target_covered else "NO_COVERAGE")
    return {**base, "result": result, "footprint": fp, "covered_cells": covered, "partial_cells": partial,
            "cell_coverage": coverage, "coverage_method": "AXIS_ALIGNED_FRAME_CELL_INTERSECTION",
            "detections": detections, "target_covered": target_covered,
            "target_point": {"lat": goal.lat, "lon": goal.lon, "cell_id": goal.cell_id},
            "footprint_center": {"lat": lat, "lon": lon},
            "fire_points": [{"lat": c["lat"], "lon": c["lon"], "cell_id": c["cell_id"]}
                            for c in (cells if cells is not None else snapshot.fire_cells)
                            if c.get("fire_state") == "BURNING" and c["cell_id"] in seen]}


# ---------------------------------------------------------------------------
# 관측 계획용: 지도 전체 칸으로 범위 계산 (사용자 결정 2026-10-07)
#   기존 simulate 는 환경이 준 불·위험 칸만 훑어 그 밖의 칸은 '불 없음'으로도 남지 않는다.
#   관측 계획은 미관측과 '불 없음'을 구분해야 하므로 정적 지도 칸(위치·크기)에 범위 안의 정답 상태만 붙인다.
# ---------------------------------------------------------------------------

def parse_cell_key(cid):
    """'{col}_{row}' → (col, row). 형식이 다르면 None (시험 지도 등)"""
    if not isinstance(cid, str) or "_" not in cid:
        return None
    a, _, b = cid.partition("_")
    try:
        return int(a), int(b)
    except ValueError:
        return None


def cells_near(map_cells, lat: float, lon: float, radius_m: float) -> list:
    """정적 지도 칸 중 (lat, lon) 에서 radius_m 사각형 안 (위경도 상자로 먼저 거른다)"""
    dlat = radius_m / _M_PER_DEG_LAT
    dlon = radius_m / (_M_PER_DEG_LAT * max(0.01, math.cos(math.radians(lat))))
    return [c for c in map_cells if c.get("lat") is not None and c.get("lon") is not None
            and abs(c["lat"] - lat) <= dlat and abs(c["lon"] - lon) <= dlon]


def with_truth_state(cells, snapshot) -> list:
    """정적 칸에 정답 화재 상태를 붙인다 (모의 센서 내부 전용 — 범위 밖 칸에는 쓰지 않는다)"""
    truth = {c["cell_id"]: c.get("fire_state") for c in snapshot.fire_cells + snapshot.risk_cells}
    return [{**c, "fire_state": truth.get(c["cell_id"]) or "UNBURNED"} for c in cells]


def _seg_dist(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _path_dist(px, py, path) -> float:
    if len(path) == 1:
        return math.hypot(px - path[0][0], py - path[0][1])
    return min(_seg_dist(px, py, *path[i], *path[i + 1]) for i in range(len(path) - 1))


def _truncate(path, length):
    out, used = [path[0]], 0.0
    for a, b in zip(path, path[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if used + d >= length:
            t = 0.0 if d == 0 else (length - used) / d
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
            return out, length
        out.append(b)
        used += d
    return out, used


_SAMPLES = 8          # 칸 한 변을 8등분한 64 점으로 띠가 덮은 비율을 잰다 (기하 근사, 방법을 결과에 남김)


def simulate_edge_sweep(*, snapshot, map_cells, task, attempt_id: str, resource_id: str, position: dict,
                        report_point: Optional[dict], fallback_point: Optional[dict] = None,
                        burned_trail: Optional[list] = None, advance_point: Optional[dict] = None,
                        orbit_duration_s: Optional[float] = None, head_bearing_deg: Optional[float] = None) -> dict:
    """드론 테두리 추적 관측 (모의, TEST_ONLY — 드론 순찰 기능 UAV-08 대기).

    도착 위치의 카메라 범위(띠 폭의 절반 반경) 안에 불 테두리 칸(불타는 칸 중 옆에 안 탄 칸이 있는 칸)이 있으면
    그 테두리를 따라 신고 지점에서 멀어지는 쪽으로 speed×duration 만큼 난다. 테두리가 없으면 fallback_point
    (총괄이 지금 불타는 중으로 아는 가장 가까운 칸, 사용자 결정 2026-10-07) 쪽으로 직선으로 난다.
    그것도 없으면 burned_trail(가장 마지막에 불타던 곳에서 시작해 아직 안 본 '다 타고 꺼짐(추정)' 칸을 이은 순서)을
    따라 난다. 그것도 없으면 advance_point(진행 방향으로 1시간 앞 예상 지점) 쪽으로 직선으로 난다 — 신고 지점 쪽으로
    가지 않는다 (사용자 결정 2026-10-07). 그것도 없으면 제자리. 모두 같은 길이(speed×duration)다.
    orbit_duration_s 를 주면(첫 드론) 테두리를 따라가지 않고 도착 지점을 중심으로 원을 돈다: speed×orbit_duration_s 를
    '중심 → 원 위(북쪽)로 나가기 + 한 바퀴'에 쓰므로 반지름 = 길이 / (1 + 2π). 불 전체 모양을 보기 위함 (2026-10-07).
    head_bearing_deg(불 머리 쪽 = 바람이 불어 가는 방향)를 주면 테두리를 그 방향으로 따라간다 (사용자 결정 2026-10-08).
    테두리를 따라갈 때는 칸 가운데가 아니라 테두리선(불타는 칸과 안 탄 칸의 경계)에서 다 탄 쪽으로
    config.OBS_EDGE_OFFSET_M 들어간 선 위를 난다 — 불 바로 위를 날지 않는다 (사용자 결정 2026-10-08).
    결과 footprint.mode 가 NO_FIRE_REFERENCE_NEARBY 계열이면 총괄이 앞질러 보내기 출동을 만든다. 지나간 경로 양옆 swath/2 안이 관측 범위다.
    정답(불 상태)은 이 범위 안의 칸에만 쓴다. 결과의 fire_points 는 불이 보인 칸 중심 좌표뿐이다 (드론 보고 계약)."""
    prof = config.EDGE_SWEEP_PROFILE
    base = {
        "observation_id": f"OBS-{uuid.uuid4().hex[:12]}", "run_id": snapshot.run_id,
        "state_version_seen": snapshot.state_version, "simulation_time_s": snapshot.simulation_time_s,
        "observed_wall": datetime.now(timezone.utc).isoformat(), "task_id": task.task_id,
        "attempt_id": attempt_id, "resource_id": resource_id, "source": prof["source"],
        "sensor_profile_id": prof["sensor_profile_id"], "sensor_status": prof["status"],
        "sensor_type": "THERMAL", "position": position, "footprint_basis": prof["basis"],
    }
    lat0, lon0 = position.get("lat"), position.get("lon")
    empty = {"footprint": None, "covered_cells": [], "partial_cells": [], "cell_coverage": [], "detections": [],
             "fire_points": [], "target_covered": False}
    if lat0 is None or lon0 is None:
        return {**base, "result": "FAILED", "failure_reason": "POSITION_UNKNOWN", **empty}
    length = prof["speed_ms"] * (orbit_duration_s or prof["duration_s"])
    half = prof["swath_m"] / 2
    cell_size = max((c.get("cell_size_m") or 0) for c in map_cells) if map_cells else 0
    near = with_truth_state(cells_near(map_cells, lat0, lon0, length + half + cell_size), snapshot)
    by_id = {c["cell_id"]: c for c in near}
    by_cr = {parse_cell_key(c["cell_id"]): c for c in near if parse_cell_key(c["cell_id"])}
    xy = {cid: _to_local_m(lat0, lon0, c["lat"], c["lon"]) for cid, c in by_id.items()}

    def nbrs(cid):
        p = parse_cell_key(cid)
        if p is None:
            return []
        return [by_cr[(p[0] + dc, p[1] + dr)]["cell_id"] for dc in (-1, 0, 1) for dr in (-1, 0, 1)
                if (dc or dr) and (p[0] + dc, p[1] + dr) in by_cr]

    edge = {cid for cid, c in by_id.items() if c["fire_state"] == "BURNING"
            and any(by_id[n]["fire_state"] == "UNBURNED" for n in nbrs(cid))}
    rep = None if not report_point else _to_local_m(lat0, lon0, report_point["lat"], report_point["lon"])

    def to_square(cid):                      # 도착점에서 칸(정사각형)까지 거리
        x, y = xy[cid]
        h = (by_id[cid].get("cell_size_m") or 0) / 2
        return math.hypot(max(abs(x) - h, 0.0), max(abs(y) - h, 0.0))
    start = sorted((to_square(c), c) for c in edge if to_square(c) <= half)
    path = [(0.0, 0.0)]
    # 첫 드론의 원 (2026-10-08): 불 위가 아니라 불 테두리선에서 다 탄 쪽으로 OBS_EDGE_OFFSET_M 들어간 원을 돈다.
    # 반지름 = (도착점에서 가장 먼 불타는 칸의 바깥 변까지) - OBS_EDGE_OFFSET_M. 그 원이 120초 길이에 안 들어가면
    # (불이 이미 크면) 원 대신 가장 가까운 테두리부터 다른 드론처럼 테두리를 따라간다.
    orbit_r, r_fit = None, length / (1 + 2 * math.pi)
    if orbit_duration_s:
        burn = [cid for cid, c in by_id.items() if c["fire_state"] == "BURNING"]
        ext = max((math.hypot(*xy[cid]) + (by_id[cid].get("cell_size_m") or 0) / 2 for cid in burn), default=None)
        if ext is None:
            orbit_r = r_fit
        elif ext - config.OBS_EDGE_OFFSET_M <= r_fit:
            orbit_r = max(ext - config.OBS_EDGE_OFFSET_M, 1.0)
        else:
            start = sorted((to_square(c), c) for c in edge)
    if orbit_r is not None:
        mode = "ORBIT_SURVEY" if orbit_r == r_fit else "ORBIT_FIRE_EDGE"
        r = orbit_r
        path.extend([(0.0, r)] + [(r * math.sin(math.radians(a)), r * math.cos(math.radians(a)))
                                  for a in range(10, 361, 10)])
    elif start:
        mode, cur, seen = "EDGE_FOLLOW", start[0][1], {start[0][1]}
        path.append(xy[cur])
        walked = math.hypot(*xy[cur])
        while walked < length:
            cand = [n for n in nbrs(cur) if n in edge and n not in seen]
            if not cand:
                break
            if head_bearing_deg is not None:     # 불 머리 쪽으로 (같으면 칸 ID 순 — 결정론)
                hx, hy = math.sin(math.radians(head_bearing_deg)), math.cos(math.radians(head_bearing_deg))
                far = lambda n: xy[n][0] * hx + xy[n][1] * hy          # noqa: E731
            else:                                # 신고 지점에서 더 먼 쪽으로
                far = (lambda n: math.hypot(xy[n][0] - rep[0], xy[n][1] - rep[1])) if rep else (lambda n: 0.0)
            nxt = sorted(cand, key=lambda n: (-far(n), n))[0]
            walked += math.hypot(xy[nxt][0] - xy[cur][0], xy[nxt][1] - xy[cur][1])
            path.append(xy[nxt])
            seen.add(nxt)
            cur = nxt
        # 칸 가운데 → 테두리선에서 다 탄 쪽으로 OBS_EDGE_OFFSET_M 들어간 점 (바깥쪽 = 안 탄 이웃 칸들의 평균 방향)
        for i in range(1, len(path)):
            cid = next((c for c in seen if xy[c] == path[i]), None)
            out_n = [n for n in nbrs(cid) if by_id[n]["fire_state"] == "UNBURNED"] if cid else []
            if not out_n:
                continue
            ox = sum(xy[n][0] - xy[cid][0] for n in out_n)
            oy = sum(xy[n][1] - xy[cid][1] for n in out_n)
            d = math.hypot(ox, oy)
            if d < 1e-6:
                continue
            shift = max(0.0, (by_id[cid].get("cell_size_m") or 0) / 2 - config.OBS_EDGE_OFFSET_M)
            path[i] = (path[i][0] + ox / d * shift, path[i][1] + oy / d * shift)
    else:
        fb = None if not fallback_point else _to_local_m(lat0, lon0, fallback_point["lat"], fallback_point["lon"])
        trail = [_to_local_m(lat0, lon0, p["lat"], p["lon"]) for p in (burned_trail or [])]
        if fb is None and trail:
            mode = "FOLLOW_PRESUMED_BURNED"
            path.extend(p for p in trail if math.hypot(p[0] - path[-1][0], p[1] - path[-1][1]) > 1e-6)
        else:
            adv = None if not advance_point else _to_local_m(lat0, lon0, advance_point["lat"], advance_point["lon"])
            goal_pt, mode = ((fb, "STRAIGHT_TO_KNOWN_FIRE") if fb is not None and math.hypot(*fb) > 1e-6 else
                             (adv, "STRAIGHT_TO_PREDICTED_FRONT") if adv is not None and math.hypot(*adv) > 1e-6 else
                             (None, "HOLD_AT_ARRIVAL"))
            if goal_pt is not None:
                d = math.hypot(*goal_pt)
                path.append((goal_pt[0] / d * length, goal_pt[1] / d * length))
    path, flown = _truncate(path, length) if len(path) > 1 else (path, 0.0)

    covered, partial, coverage, detections, points = [], [], [], [], []
    for cid in sorted(by_id):
        c, (cx, cy) = by_id[cid], xy[cid]
        size = c.get("cell_size_m")
        if not size:
            continue
        step, hit = size / _SAMPLES, 0
        for i in range(_SAMPLES):
            for j in range(_SAMPLES):
                px, py = cx - size / 2 + (i + 0.5) * step, cy - size / 2 + (j + 0.5) * step
                if _path_dist(px, py, path) <= half:
                    hit += 1
        if not hit:
            continue
        full = hit == _SAMPLES * _SAMPLES
        (covered if full else partial).append(cid)
        coverage.append({"cell_id": cid, "cell_size_m": size, "full": full,
                         "fraction": round(hit / (_SAMPLES * _SAMPLES), 6), "rect": None})
        if c["fire_state"] in ("BURNING", "BURNED"):
            detections.append({"cell_id": cid, "fire_state": c["fire_state"]})
            if c["fire_state"] == "BURNING":
                points.append({"lat": c["lat"], "lon": c["lon"], "cell_id": cid})
    goal = task.nav_target
    target_covered = (goal.lat is not None and goal.lon is not None
                      and _path_dist(*_to_local_m(lat0, lon0, goal.lat, goal.lon), path) <= half)

    def back(p):
        return {"lat": round(lat0 + p[1] / _M_PER_DEG_LAT, 7),
                "lon": round(lon0 + p[0] / (_M_PER_DEG_LAT * math.cos(math.radians(lat0))), 7)}
    fp = {"shape": "SWATH_ALONG_PATH", "mode": mode, "path": [back(p) for p in path], "length_m": round(flown, 1),
          "orbit_radius_m": round(orbit_r, 1) if orbit_r is not None else None,
          "swath_m": prof["swath_m"], "speed_ms": prof["speed_ms"],
          "duration_s": orbit_duration_s or prof["duration_s"],
          "width_m": prof["swath_m"], "height_m": round(flown, 1), "agl_m": None,
          "method": "CAPSULE_BUFFER_8x8_CELL_SAMPLING",
          "edge_offset_m": config.OBS_EDGE_OFFSET_M if mode in ("EDGE_FOLLOW", "ORBIT_FIRE_EDGE") else None,
          "head_bearing_deg": head_bearing_deg}
    result = "DETECTED" if detections else ("NOT_DETECTED" if coverage else "NO_COVERAGE")
    return {**base, "result": result, "footprint": fp, "covered_cells": covered, "partial_cells": partial,
            "cell_coverage": coverage, "coverage_method": fp["method"], "detections": detections,
            "fire_points": points, "target_covered": target_covered,
            "target_point": {"lat": goal.lat, "lon": goal.lon, "cell_id": goal.cell_id},
            "footprint_center": {"lat": lat0, "lon": lon0}, "report_point": report_point,
            "fallback_point": fallback_point, "burned_trail": [p.get("cell_id") for p in (burned_trail or [])],
            "advance_point": advance_point}


def simulate_ground_los(*, snapshot, map_cells, task, attempt_id: str, resource_id: str, position: dict) -> dict:
    """UGV 지상 열화상 (모의, TEST_ONLY — 사용자 결정 2026-10-08): 반지름 config.UGV_VIEW 의 min_m 안은 항상 보이고,
    min_m ~ max_m 는 카메라(UGV 칸 지면 + mast_m)에서 그 칸 지면까지의 시선이 사이 지형에 막히지 않으면 보인다.
    지형은 정적 지도 칸의 지면 높이(ground_amsl_m)만 쓴다. 정답(불 상태)은 보인 칸에만 쓴다."""
    prof = config.UGV_VIEW
    base = {
        "observation_id": f"OBS-{uuid.uuid4().hex[:12]}", "run_id": snapshot.run_id,
        "state_version_seen": snapshot.state_version, "simulation_time_s": snapshot.simulation_time_s,
        "observed_wall": datetime.now(timezone.utc).isoformat(), "task_id": task.task_id,
        "attempt_id": attempt_id, "resource_id": resource_id, "source": prof["source"],
        "sensor_profile_id": prof["sensor_profile_id"], "sensor_status": prof["status"],
        "sensor_type": "THERMAL", "position": position, "footprint_basis": prof["basis"],
    }
    lat0, lon0 = position.get("lat"), position.get("lon")
    empty = {"footprint": None, "covered_cells": [], "partial_cells": [], "cell_coverage": [], "detections": [],
             "fire_points": [], "target_covered": False}
    if lat0 is None or lon0 is None:
        return {**base, "result": "FAILED", "failure_reason": "POSITION_UNKNOWN", **empty}
    near = [c for c in cells_near(map_cells, lat0, lon0, prof["max_m"])
            if math.hypot(*_to_local_m(lat0, lon0, c["lat"], c["lon"])) <= prof["max_m"]]
    near = with_truth_state(near, snapshot)
    by_cr = {parse_cell_key(c["cell_id"]): c for c in near if parse_cell_key(c["cell_id"])}
    here = min(near, key=lambda c: (math.hypot(*_to_local_m(lat0, lon0, c["lat"], c["lon"])), c["cell_id"]),
               default=None)
    if here is None or here.get("ground_amsl_m") is None or parse_cell_key(here["cell_id"]) is None:
        return {**base, "result": "FAILED", "failure_reason": "GROUND_UNKNOWN_AT_POSITION", **empty}
    c0, r0 = parse_cell_key(here["cell_id"])
    eye = here["ground_amsl_m"] + prof["mast_m"]

    def visible(c):
        d = math.hypot(*_to_local_m(lat0, lon0, c["lat"], c["lon"]))
        if d <= prof["min_m"]:
            return True
        tgt = c.get("ground_amsl_m")
        p = parse_cell_key(c["cell_id"])
        if tgt is None or p is None:
            return False
        steps = max(abs(p[0] - c0), abs(p[1] - r0)) * 2          # 칸 반 개 간격으로 사이 지형을 본다
        for i in range(1, steps):
            t = i / steps
            m = by_cr.get((round(c0 + t * (p[0] - c0)), round(r0 + t * (p[1] - r0))))
            if m is None or m.get("ground_amsl_m") is None or m is c:
                continue
            if m["ground_amsl_m"] > eye + t * (tgt - eye):        # 사이 지면이 시선보다 높다 → 가림
                return False
        return True
    seen, hidden = [], 0
    for c in sorted(near, key=lambda c: c["cell_id"]):
        if visible(c):
            seen.append(c)
        else:
            hidden += 1
    covered = [c["cell_id"] for c in seen]
    goal = task.nav_target
    fp = {"shape": "RADIUS_TERRAIN_LOS", "min_m": prof["min_m"], "max_m": prof["max_m"], "mast_m": prof["mast_m"],
          "eye_amsl_m": round(eye, 1), "visible_cells": len(seen), "hidden_cells": hidden,
          "method": "GRID_LINE_OF_SIGHT_HALF_CELL_STEP"}
    detections = [{"cell_id": c["cell_id"], "fire_state": c["fire_state"]} for c in seen
                  if c["fire_state"] in ("BURNING", "BURNED")]
    result = "DETECTED" if detections else ("NOT_DETECTED" if seen else "NO_COVERAGE")
    return {**base, "result": result, "footprint": fp, "covered_cells": covered, "partial_cells": [],
            "cell_coverage": [{"cell_id": c["cell_id"], "cell_size_m": c.get("cell_size_m"), "full": True,
                               "fraction": 1.0, "rect": None} for c in seen],
            "coverage_method": fp["method"], "detections": detections,
            "fire_points": [{"lat": c["lat"], "lon": c["lon"], "cell_id": c["cell_id"]} for c in seen
                            if c["fire_state"] == "BURNING"],
            "target_covered": goal.cell_id in covered,
            "target_point": {"lat": goal.lat, "lon": goal.lon, "cell_id": goal.cell_id},
            "footprint_center": {"lat": lat0, "lon": lon0}}


def simulate_weather(*, snapshot, task, attempt_id: str, resource_id: str, position: dict,
                     provider_raw: Optional[dict] = None) -> dict:
    """모의 기상 측정 (TEST_ONLY). 도착 위치가 속한 셀에 환경 기상값(cell['weather'])이 있으면 그 값,
    없으면 지도 전체 값(snapshot.weather + wind_ms/wind_dir_deg)을 읽는다. 환경이 준 항목만 보고한다."""
    prof = config.WEATHER_SENSOR_PROFILE
    base = {
        "observation_id": f"OBS-{uuid.uuid4().hex[:12]}", "run_id": snapshot.run_id,
        "state_version_seen": snapshot.state_version, "simulation_time_s": snapshot.simulation_time_s,
        "observed_wall": datetime.now(timezone.utc).isoformat(), "task_id": task.task_id,
        "attempt_id": attempt_id, "resource_id": resource_id, "source": prof["source"],
        "sensor_profile_id": prof["sensor_profile_id"], "sensor_status": prof["status"],
        "sensor_type": "WEATHER", "position": position, "method": prof["method"],
        "provider_raw_ignored": provider_raw, "detections": [],
    }
    lat, lon = position.get("lat"), position.get("lon")
    if lat is None or lon is None:
        return {**base, "result": "FAILED", "failure_reason": "POSITION_UNKNOWN", "values": {},
                "covered_cells": [], "target_covered": False, "footprint": None}
    cells = snapshot.fire_cells + snapshot.risk_cells + snapshot.spread_forecast
    here = None
    for c in cells:
        if c.get("lat") is None or not c.get("cell_size_m"):
            continue
        dx, dy = _to_local_m(lat, lon, c["lat"], c["lon"])
        if abs(dx) <= c["cell_size_m"] / 2 and abs(dy) <= c["cell_size_m"] / 2:
            here = c
            break
    if here is not None and here.get("weather"):
        raw, scope = here["weather"], "CELL"
    else:
        raw = {"wind_ms": snapshot.wind_ms, "wind_dir_deg": snapshot.wind_dir_deg, **(snapshot.weather or {})}
        scope = "GLOBAL_FIELD"          # 환경 기상이 지도 전체 값 하나뿐 — 지점 측정의 의미가 제한됨
    values = {k: raw.get(k) for k in prof["measures"] if raw.get(k) is not None}
    tgt = next((c for c in cells if c.get("cell_id") == task.nav_target.cell_id and c.get("cell_size_m")), None)
    if tgt is not None:
        dx, dy = _to_local_m(lat, lon, tgt["lat"], tgt["lon"])
        covered = abs(dx) <= tgt["cell_size_m"] / 2 and abs(dy) <= tgt["cell_size_m"] / 2
    else:
        covered = False
    return {**base, "result": "MEASURED" if values else "FAILED",
            "failure_reason": None if values else "WEATHER_NOT_PROVIDED_BY_ENV",
            "values": values, "scope": scope, "covered_cells": [here["cell_id"]] if here else [],
            "target_covered": covered and bool(values), "footprint": None}
