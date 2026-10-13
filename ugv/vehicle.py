# -*- coding: utf-8 -*-
"""ugv/vehicle.py — 차량 1대의 임무 상태 전이, 취소 규칙, 주행 감시, 화재 안전거리 처리.

임무 상태 (mission_state)
  IDLE        정차 대기 (도착 후 정차 관측 포함 — 다음 명령을 기다린다)
  DRIVING     목표 접근 지점으로 주행
  EVADING     화재 안전거리 위반 → 안전한 도로 지점으로 이탈 주행
  STOPPING    정지 명령 → 감속 중
  FAULT       주행 이상 확정 (경로 이탈·정지 고장·텔레메트리 끊김·차량 이상). 운영자 /stop 으로만 해제
  DANGER      자동 이탈 불가 상태 — 안전 확보 완료가 아니다. 화재 안전거리 위반인데 진행 방향을 지키는 이탈 경로가 없다.
              도로를 따라 제동해 멈추고 운영자 개입을 요청한다. 자동 재출발 없음. 원격 운전 기능은 아직 없다
FAULT·DANGER 해제: 운영자 /stop. 차량이 멈췄고(0.2 m/s 미만) 텔레메트리가 살아 있고 정지 감독이 정차를 확인했을 때만 해제한다
정지 감독 (상태와 별개, _stop_sup): 정지 명령(고장·취소·막힘·비상·감독 없는 움직임)을 보낸 순간부터 실제 속도·이동 거리·경과 시간을
  텔레메트리로만 본다 (미션 완료 신호에 기대지 않는다). 허용 거리(정지 경로·제동거리 중 긴 것 + 지연 거리 + 여유)나 허용 시간을 넘거나
  감속하지 않으면 드라이버 구동 차단(cut_drive — PX4 시동 끄기, 거부되면 kill)으로 넘어가 고장(STOP_NOT_CONFIRMED)으로 묶고,
  정차가 확인될 때까지 STOP_RETRY_S 마다 다시 차단한다. 정차 확인 전에는 READY 가 아니고 점유는 넓게 (ugv/traffic.py) 유지한다.
  (2026-10-11 Gazebo 지속 운용: 정지 미션 끝 2.9 m/s 로 FAULT 차가 직진 2.6 km)
총괄에 보이는 자원 상태 (계약 값): READY / RUNNING / UNAVAILABLE

진행 방향 (ugv/drive_graph.py, 2026-10-09 사용자 결정)
  - 계획 출발점 = 지금 위치를 진행 방향으로 명령 교체 지연 동안 갈 거리만큼 앞으로 옮긴 곳. 주행 중이면 계획 경로의
    방향, 아니면 텔레메트리 진행 방위를 쓴다. 정차 중에도 방향 제약을 유지한다
  - 새 목표에 진행 방향을 지키는 경로가 없으면: 실행을 거절(409)하고, 주행 중이던 임무가 있으면 취소한 뒤 도로를 따라
    감속 정지한다 (도로 밖으로 돌아서라도 수행하지 않는다)
  - 정지는 도로를 따른다: 남은 계획 경로(없으면 도로 앞쪽)를 따라 제동거리만큼 가는 감속 미션

취소 규칙
  - 차량당 진행 중 임무는 하나. 새 execute 는 잠금 안에서: 계획(부작용 없음) → 기존 임무 CANCELLED(SUPERSEDED) →
    세대 번호 증가 → 드라이버 미션 교체(follow 는 기존 미션을 통째로 바꾼다). 주행 명령이 둘 남지 않는다.
  - 같은 실행 키·같은 내용 = 기존 실행 반환, 같은 키·다른 내용 = 409 (interfaces/exec_store.py, UGV-04).
  - 감시 루프는 세대 번호가 바뀐 이전 임무의 진행·도착을 무시한다.
"""

import asyncio
import json
import logging
import math
import os
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from interfaces.exec_store import ExecStore

from . import config
from .approach import ApproachPlan, node_id_of, parse_node_id, plan_approach, plan_evasion
from .drive_graph import DirPos, access_point
from .drivers import DriverError, build_driver
from .geo import FRAME, bearing_deg
from .profiles import get as get_profile
from .roads import RoadNetwork, RoadPos, Route, plan_speeds

ACTIVE = ("STARTED", "IN_PROGRESS")
DANGER_MEANING = "자동 이탈 불가 상태 (안전 확보 완료 아님). 도로를 따라 제동·정지, 자동 재출발 없음, 운영자 개입 필요"


class PlacementError(Exception):
    """소속 거점에 차량을 둘 도로 출입 지점·대기 위치가 없다."""


def config_station_m():
    from .drive_graph import STATION_SEARCH_M
    return STATION_SEARCH_M


class HttpError(Exception):
    def __init__(self, code: int, detail):
        super().__init__(detail)
        self.code, self.detail = code, detail


def _now_iso():
    return datetime.now(timezone.utc).isoformat()


def _dist_to_polyline(x, y, pts, lo=0, hi=None):
    best, bi = math.inf, lo
    hi = len(pts) - 1 if hi is None else hi
    for i in range(max(lo, 0), max(min(hi, len(pts) - 1), 0)):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        if d < best:
            best, bi = d, i
    return best, bi


def _progress_seg(x, y, pts, lo=0, hi=None, tie_m=1.0):
    """경로 진행 위치: (x, y) 에서 가장 가까운 거리 + tie_m 안에 드는 구간 중 경로 순서상 가장 앞 구간 (거리, 구간 번호).
    같은 자리를 경로가 다시 지나면(회전 공터로 돌아 같은 도로로 내려옴 등) 가장 가까운 구간이 나중 구간일 수 있다 —
    그러면 그 사이 미션 지점을 '이미 지남' 으로 빼 버린다 (2026-10-11 회전 공터 시험: 복귀 미션에서 오르막 끝·공터 지점이 모두 빠져
    차 뒤 지점으로 출발 → 도로 이탈 10.9 m). 38aa911 first_item_ahead 와 같은 기준 (START_TIE_M 1 m)."""
    hi = len(pts) - 1 if hi is None else hi
    ds = []
    for i in range(max(lo, 0), max(min(hi, len(pts) - 1), 0)):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        ds.append((math.hypot(x - (ax + t * dx), y - (ay + t * dy)), i))
    if not ds:
        return math.inf, lo
    dmin = min(d for d, _ in ds)
    return next((d, i) for d, i in ds if d <= dmin + tie_m)


_log = logging.getLogger("ugv.vehicle")
BLOCK_REPLAN_MAX = 3      # 한 임무에서 서 있는 차에 막혀 다시 계획하는 최대 횟수
# 막힘 우회 때 피할 도로: origin = 상대 텔레메트리 원점 snap 도로 하나 (기본, 예전 동작) / footprint = 상대 차체(프로필 길이·폭)가
# 도로 띠(drive_graph 반폭 추정)에 걸친 도로 전부. 2026-10-13 간헐 실패 비교 시험용 선택 — 기본값을 바꾸지 않는다.
BLOCK_AVOID_MODE = os.getenv("UGV_BLOCK_AVOID", "origin")
# 정지 후보 (2026-10-13 S4 경사 중단 시험: 5~6 % 하강 정지가 계획의 1.50~1.57 배 — 명령 뒤 감속 시작까지 2.4~3.0 s·26~34 m, 그 사이 PX4 가속).
# 기본 off (예전 동작). 검증 전 운영 가능 선언 금지.
#   UGV_STOP_SPEED_CMD=1 : 정지 경로 전송 직후 지금 속도를 속도 상한으로 (PX4 DO_CHANGE_SPEED) — 미션 항목 속도 무시로 인한 가속 방지
#   UGV_STOP_GRADE=1     : 정지 필요거리 = v·τ + v²/(2·max(a − g·하강경사, 0.5)) + 여유. τ = latency_s(), 하강 경사 = 지형 타일
#                          (UGV_TERRAIN_TILES) 에서 진행 방향 필요거리 구간 고도차 + 불확실성 0.01. 타일이 없으면 경사 0 (지연 항만)
STOP_SPEED_CMD = os.getenv("UGV_STOP_SPEED_CMD", "0") == "1"
STOP_GRADE_MODE = os.getenv("UGV_STOP_GRADE", "0") == "1"
# 개선 2 전달 보완 (2026-10-13 Codex 답변 1423: 계산한 속도 계획을 PX4 가 수행하도록 잇는 최소 연결 — ESC·새 자율 기능 아님). 기본 off.
#   UGV_SPEED_TERRAIN_CMD=1 : 주행 감독 중 '지금 위치 ~ v·τ 앞' 계획 속도(sp.v, 감속 경사 포함)의 최소를 DO_CHANGE_SPEED 로 보낸다.
#   진입 전 적용 = τ(TS.TAU_S) 만큼 앞을 본다. 값이 SPEED_CMD_DELTA 넘게 바뀔 때만, SPEED_CMD_MIN_S 간격 이상으로 보낸다 (매 틱 재전송 금지).
#   양보·정지 감독·취소·재계획 중에는 보내지 않는다 (정지 속도 명령과 경쟁 금지). 실패해도 감독 유지 (문제 기록만).
#   DO_CHANGE_SPEED 는 목표 속도 명령이지 실제 속도 상한 보장이 아니다 — 보낸 시각·순번·값·ACK·그때 실제 속도를 SPEED_CMD 이벤트로 남긴다.
SPEED_TERRAIN_CMD = os.getenv("UGV_SPEED_TERRAIN_CMD", "0") == "1"
SPEED_CMD_DELTA = 0.5
SPEED_CMD_MIN_S = 1.0
# 지형 표면은 terrain_speed.surface() 로 일원화 (UGV_TERRAIN_TILES)
HOLD_FAIL_MAX = 2         # 양보 정지 명령이 연속 이만큼 실패하면 고장(STOP_COMMAND_FAILED)으로 처리
STOP_RETRY_S = 2.0        # 정지 명령이 실패한 차(명령 불확실)는 이 간격으로 정지를 다시 보낸다
STANDSTILL_ARMED_S = 5.0  # PX4 가 정차 유지(시동 끔)를 못 했을 때 정지로 보는 0.2 m/s 미만 지속 시간
STOP_CMD_TIMEOUT_S = float(os.getenv("UGV_STOP_CMD_TIMEOUT_S", "8.0"))
STOP_SUP_MARGIN_M = 5.0   # 정지 감독 허용 거리 = max(정지 경로, 제동거리) + 속도 × 명령 지연 + 이 여유
STOP_SUP_LAT_PAD_S = 1.0  # 허용 거리의 명령 지연 몫에 더하는 시간 — 정지 명령 실제 지연이 추정(latency_s)보다 길다 (2026-10-11 Gazebo
                          # 소방차 8.5 m/s 고장·취소: 44~46 m 로 허용 43 m 를 1~3 m 넘겨 서기 직전 차단 → 취소마다 고장)
STOP_SUP_SLACK_S = 4.0    # 정지 감독 허용 시간 = 허용 거리를 평균 속도(처음 속도의 절반)로 가는 시간 + 명령 지연 + 이 여유
STOP_SUP_ACCEL_MPS = 0.8  # 정지 명령 뒤 지연이 지나서도 처음 속도보다 이만큼 빨라지면 감속이 진행되지 않는 것
STOP_SUP_ACCEL_HOLD_S = 2.0  # 그 가속이 이만큼 이어져야 '감속 안 함' (명령 교체 순간의 짧은 흔들림과 구분 — 거리·시간 상한은 따로)
STOP_SLOW_FINISH_MPS = 1.2  # 정지 미션 끝 구간 항목 최소 속도(1 m/s) + 여유 — 이 미만·허용 거리 안이면 시간 초과를 고장으로 묶지 않음
STOP_CONFIRM_S = 1.0
START_END_M = 15.0          # 출발 도로에서 진행 방향으로 남은 거리가 이보다 짧으면 (교차로 회전 계획 어려움 — 약 14 m 안) 거의 같은 거리의 다른 도로를 본다
START_ALIGN_MIN = 0.5       # 그 다른 도로의 방향과 진행 방향 차이 60° 안 (회전 도중 정차는 두 도로 모두 수십 도 어긋난다)
START_STILL_MPS = 0.5       # 이보다 느릴 때만 (정차) 출발 도로 다른 해석을 본다 — 주행 중은 계획 경로 방위·투영이 맡는다
START_TIE_M = 1.0           # 가장 가까운 도로와 이 거리 차 안의 도로만 (교차로 근처 동률)
COAST_DECEL_MPS2 = 0.5      # 시동 꺼진 뒤 관성 감속 가정 (Gazebo c23 B-ugv1: 13.5 m/s → 정차 약 22 s, 0.6 m/s²)
COAST_PAD_S = 2.0
COAST_MIN_MPS = 0.5         # 시동 꺼지기 직전 속도가 이보다 낮으면 보류하지 않는다
STOP_ROAD_SNAP_MAX_M = 5.0  # 정지 경로를 '진행 방향 도로' 로 만들 수 있는 도로까지 최대 거리 (밖이면 진행 방향 직선 제동)
# 도로를 따라가는 정지 경로를 쓸 수 없을 때(계획선에서 멀고, 도로 방향도 진행 방향과 맞지 않음 — 이미 도로 밖)의 정지:
#   straight  = 드라이버 기본 (진행 방향 직선, 일반 감속 + 여유 5 m — 3.94 m/s 에서 10.2 m)
#   emergency = 진행 방향 직선, 비상 감속(차종 emergency_decel) 제동거리 + 1 m, 최소 EMERG_STOP_MIN_M, 지점 속도 1 m/s
# 2026-10-12 c42 j494846 v2: 직선 정지 10 m 동안 도로 밖 3.67 → 5.5 m, 기록 끝까지 정지 미확인. 시동 끄기(구동 차단)는 관성 주행이라
# 대안이 아니다 (2026-10-11 c23: 13.5 m/s 에서 약 75 m). 도로로 급히 되돌아가는 경로는 만들지 않는다
OFFROAD_STOP_MODE = os.getenv("UGV_OFFROAD_STOP", "straight")
# 고장 정지에서 남은 계획 경로를 따라 정지할 계획선 거리 상한. 기본 0 = 끔(예전 동작: 고장 정지는 진행 방향 도로 정지 경로).
# 켜면 고장 차가 계획 경로 위에 서서 다른 차의 대체 경로를 막는 경우가 생겼다 (2026-10-12 단위 시험
# test_closed_loop_fault_redecision_other_resource_return_ready: 수정 전 4/4 통과, 켠 상태 2/4 실패 — BLOCKED_BY_VEHICLE)
FAULT_STOP_PLAN_M = float(os.getenv("UGV_FAULT_STOP_PLAN_M", "0"))
EMERG_STOP_MIN_M = 4.0      # PX4 도착 반경(3 m)보다 멀게 — 가까우면 바로 도착 처리돼 미션 끝 대기(선회)로 간다
PASS_STALL_S = 1.0          # 정체(드라이버 pass_blocked_since: 현재 항목을 놓치고 멀어지는데 건너뛸 수 없음)가 이만큼 → 고장·정지
                            # (기록 재생: 1 s 에서도 정상 주행 오작동 0, 실패 주행은 항목에서 6.6~7.3 m 일 때 감지 — 2 s 는 7.5~11 m)
OFFROAD_STOP_M = 2.0        # 주행 중 실제 차체가 도로 띠 밖으로 이만큼 넘게 나가면 (OFFROAD_CONFIRM_S 지속) 고장·정지
OFFROAD_CONFIRM_S = 2.0      # 정지 감독: 0.2 m/s 미만(최신 텔레메트리)이 이만큼 이어지면 정차 확인
# 정지·양보 명령이 이 시간 안에 끝나지 않으면 '전달 여부 불명' 실패로 본다 (실측 명령 교체 지연 최대 CMD_LATENCY_MAX_S 6 s + 여유)


def _progress_index(x, y, pts, cum, near_i, back=20, ahead=200, tol_m=3.0, gap_m=30.0):
    """계획 선형 위 진행 위치 (가장 가까운 구간). 선형이 같은 곳을 두 번 지나면(거점을 지나 한 바퀴 돌아 같은 도로를 되짚어 오는 복귀
    등) 가장 가까운 구간이 나중 통과일 수 있다 → 경로상 gap_m 넘게 앞선, 거리 차 tol_m 안의 더 이른 통과가 있으면 그쪽을 고른다.
    (2026-10-10 Gazebo 4대: 진행 위치가 되짚어 오는 구간으로 건너뛴 상태에서 대기가 풀려 남은 경로를 다시 보내자, 첫 지점이 차 뒤라
    PX4 가 도로 밖에서 돌아 10 m 이탈)"""
    lo, hi = max(near_i - back, 0), min(near_i + ahead, len(pts) - 1)
    ds = []
    for i in range(lo, hi):
        d, _ = _dist_to_polyline(x, y, pts, i, i + 1)
        ds.append(d)
    if not ds:
        return _dist_to_polyline(x, y, pts, lo, hi)
    b = min(range(len(ds)), key=ds.__getitem__)
    for j in range(max(near_i - 2 - lo, 0), b):          # 더 이른 통과 — 지금 진행 위치보다 뒤(이미 지난 통과)는 보지 않는다
        if ds[j] <= ds[b] + tol_m and cum[lo + b] - cum[lo + j] > gap_m:
            k = j
            while k + 1 < b and cum[lo + k + 1] - cum[lo + j] <= gap_m and ds[k + 1] <= ds[k]:
                k += 1                                   # 그 통과 안에서 가장 가까운 구간
            return ds[k], lo + k
    return ds[b], lo + b


def choose_start_dirpos(net, graph, x: float, y: float, heading_deg: float, speed_mps: float,
                        route: Optional[Route] = None, pos: Optional[RoadPos] = None) -> Optional[DirPos]:
    """계획 출발 도로·방향.

    기본: 가장 가까운 도로 + 진행 방위로 향하는 노드 (예전 그대로). 주행 중(START_STILL_MPS 이상)이거나 앞 교차로까지 START_END_M 이상이면
    여기서 끝난다. 정차 중 교차로 START_END_M 안 (정차 상태 교차로 회전 계획이 안 되는 거리)에서는 같은 자리의 다른 해석을 본다 —
    가장 가까운 도로와 거리 차 START_TIE_M 안의 도로만:
      1) 직전 주행 경로(도로 연결 순서): 차가 그 경로 선 위(OFF_ROUTE_M/3 안)에 있으면 후보 중 경로에서 가장 진행한 구간을
         경로의 방향으로 (정차 중 진행 방위는 PX4 추정이 흘러 믿기 어렵다 — 2026-10-11 B3).
      2) 경로 정보가 없거나 맞지 않을 때만: 진행 방위와 60° 안이고 앞으로 갈 거리가 더 긴 도로.
    (2026-10-11 Gazebo c23: 소방차가 교차로 495011 에서 2.5 m 떨어져 회전 도중 정차 — 두 도로 2.52·2.74 m, 방향 차 39°·47°.
    위치 오차 0.2 m 로 지나온 도로 끝(교차로까지 2.8 m)이 출발로 잡혀 복귀·출동이 모두 거절돼 갇혔다)"""
    pos = pos if pos is not None else net.snap(x, y)
    dp = graph.dirpos(pos, heading_deg)
    if speed_mps >= START_STILL_MPS or (dp is not None and graph.ahead_m(dp) >= START_END_M):
        return dp
    base = graph.ahead_m(dp) if dp is not None else -1.0
    cands = [(pos.road_id, pos)]
    for rid, dist in net.near(x, y, pos.dist + START_TIE_M).items():
        if rid != pos.road_id and dist <= pos.dist + START_TIE_M:
            cands.append((rid, net.on_road(rid, x, y)))
    # 1) 직전 주행 경로
    if route is not None and route.legs and route.pts and _dist_to_polyline(x, y, route.pts)[0] <= config.OFF_ROUTE_M / 3:
        idx = {}
        for k, (rid, _frm, to) in enumerate(route.legs):
            idx.setdefault(rid, []).append((k, to))
        near_k = [k for k, _ in idx.get(pos.road_id, [])]
        on_route = []
        for rid, q in cands:
            for k, to in idx.get(rid, []):
                if not near_k or any(k0 - 1 <= k <= k0 + 2 for k0 in near_k):   # 가장 가까운 도로 구간 주변만 (같은 도로 재방문 제외)
                    on_route.append((k, DirPos(q, to)))
        if on_route:
            k, d2 = max(on_route, key=lambda t: t[0])
            if graph.oneway_ok(d2.road_id, graph.other(d2.road_id, d2.toward)):
                return d2
    # 2) 진행 방위
    if dp is None:
        return None
    hx, hy = math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg))
    best = None
    for rid, q in cands[1:]:
        r = net.roads[rid]
        p0 = net.at(rid, max(0.0, q.s - 2.0))
        p1 = net.at(rid, min(r.length, q.s + 2.0))
        tx, ty = p1.x - p0.x, p1.y - p0.y
        n = math.hypot(tx, ty) or 1.0
        if abs(tx * hx + ty * hy) / n < START_ALIGN_MIN:
            continue
        d2 = graph.dirpos(q, heading_deg)
        if d2 is None or graph.ahead_m(d2) <= base:
            continue
        if best is None or graph.ahead_m(d2) > graph.ahead_m(best):
            best = d2
    return best if best is not None else dp


class Vehicle:
    def __init__(self, cfg: dict, net: RoadNetwork, fire_source, clock, store: ExecStore, wall=time.monotonic,
                 taken_roads=None):
        self.cfg, self.net, self.fire_source, self.clock, self.store = cfg, net, fire_source, clock, store
        self.wall = wall                        # 통신 단절 판단용 벽시계 (단조). 시험은 가짜 시계를 넣는다
        self.resource_id = cfg["resource_id"]
        self.resource_type = cfg.get("resource_type", "UGV")
        # 차량 특성 (차체·회전반경·속도·가감속). 경로 탐색·회전 가능성·차체 검사·속도 계획을 이 값으로 한다
        self.profile = get_profile(cfg.get("profile") or ("adair_firetruck" if self.resource_type == "FIRE_ENGINE"
                                                          else "adair_ugv"))
        self.graph = net.graph_for(self.profile)
        self.cmd_latency_s = float(cfg.get("cmd_latency_s", config.CMD_LATENCY_S))   # 하한 (설정값)
        self._lat_samples = deque(maxlen=20)     # 실측 명령 교체 지연 (계획 시작 → 드라이버 미션 시작 완료, 벽시계 s)
        st = config.load_station(cfg["station_base_id"])
        sx, sy = FRAME.to_xy(st["lat"], st["lon"])
        nearest = net.snap(sx, sy)
        # 소방서 좌표와 차량 출입 지점을 구분한다. 좌표에서 가장 가까운 도로가 막다른 길일 수 있다 (drive_graph.access_point)
        acc = access_point(net, st["base_id"], graph=self.graph, slot=int(cfg.get("station_slot", 0)),
                           slot_lengths=cfg.get("slot_lengths"), taken_roads=taken_roads)
        if acc is None:
            # 임의의 먼 도로에 두지 않는다 — 배치 불가로 보고 (Fleet 가 이 차량을 빼고 사유를 남긴다)
            raise PlacementError(f"{self.resource_id}: 소방서 {st['base_id']} 좌표 {config_station_m():g} m 안에 출입 지점 없음 "
                                 f"(양방향·서로 오갈 수 있는 도로·차량 {self.profile.name} 급커브 아님 조건) 또는 대기 위치 "
                                 f"{cfg.get('station_slot', 0)} 를 겹치지 않게 둘 도로 길이 없음")
        self.access: RoadPos = acc[0]
        # 대기 방향: 출입 지점 도로의 a → b (access_point). 복귀할 때 이 방향으로 도착해야 같은 거점 차들의 대기 순서가 유지된다
        self.access_toward = net.roads[acc[0].road_id].b
        self.access_heading = acc[1] if acc[1] is not None else 0.0
        self.station = {"base_id": st["base_id"], "name": st.get("name"), "lat": st["lat"], "lon": st["lon"],
                        "coordinate_status": st.get("coordinate_status"), "source_file": st.get("source_file"),
                        "road_point": dict(zip(("lat", "lon"), nearest.ll())), "road_offset_m": round(nearest.dist, 1),
                        "access_point": {**dict(zip(("lat", "lon"), self.access.ll())), "road_id": self.access.road_id,
                                         "s_m": round(self.access.s, 1), "heading_deg": round(self.access_heading, 1),
                                         "from_station_m": round(math.hypot(sx - self.access.x, sy - self.access.y), 1),
                                         "source": acc[2]}}
        init = cfg.get("initial", "STATION")
        start = self.access if init == "STATION" else net.snap_ll(init["lat"], init["lon"])
        heading = self.access_heading if init == "STATION" else float(init.get("heading_deg", 0.0))
        self.driver = build_driver(cfg, (start.x, start.y), clock, heading, self.profile)
        if hasattr(self.driver, "pass_guard"):
            # 지나침 감시가 항목을 건너뛰는 조건: 실제 차체가 추정 도로 띠 안 (밖이면 보류 → 정체 고장·정지)
            self.driver.pass_guard = lambda tel: self._body_outside(tel) <= 0.0
        self.lock = asyncio.Lock()
        self.state = "IDLE"
        self.fault: Optional[str] = None
        self.gen = 0
        self.task_key: Optional[str] = None
        self.plan: Optional[ApproachPlan] = None
        self.speed = None
        self.seq: Optional[int] = None
        self.target_ll = None
        self.events = deque(maxlen=200)
        self._off_since = None
        self._move_ref = None
        self._last_safety = -math.inf
        self._near_i = 0
        self._drive_started_sim = None
        # 정지 명령 상태 (ugv/traffic.py 점유 예측이 쓴다). stop_path_pts: 실제로 보낸 정지·대기 경로 선형 (모르면 None).
        # command_uncertain: 정지·양보 명령이 실패해 차가 무엇을 따라가는지 확신할 수 없음 → 남은 계획 경로 전체를 점유로 보고
        # STOP_RETRY_S 마다 정지를 다시 보낸다 (2026-10-10 검토: 명령 실패 뒤에도 정지 중·대기로 보던 결함)
        self.stop_path_pts = None
        self.command_uncertain = False
        self._stop_retry_at = -1e9
        self._last_armed = None          # (시각, 속도) 시동 걸린 상태의 마지막 텔레메트리 — 시동 꺼진 뒤 관성 주행 추정용
        self._stop_sup = None            # 정지 감독 (위 머리말). 정차가 확인되면 None, 마지막 결과는 last_stop_result
        self.last_stop_result = None
        self._hold_failures = 0
        self._task: Optional[asyncio.Task] = None
        self.last_clearance_m = None
        self.last_stop: dict = {}
        self._last_contact: Optional[float] = None   # 총괄 마지막 연락 (벽시계)
        self.hold: Optional[dict] = None             # 차량 간 충돌 방지 대기 (ugv/traffic.py): {"s": 계획 선형 위치, ...}
        self.peers = lambda: []                      # 같은 서버의 다른 차량 (Fleet 가 넣는다)
        self.comm = {"phase": "OK"}                  # OK / STOPPING / STOPPED / RETURNING / RETURNED / FAILED
        self.danger: Optional[dict] = None

    # ------------------------------------------------------------------ 수명
    async def start(self):
        await self.driver.start()
        self._reconcile_restart()
        self._task = asyncio.create_task(self._run())

    async def close(self):
        if self._task:
            self._task.cancel()
        await self.driver.close()

    async def _run(self):
        while True:
            await asyncio.sleep(max(0.05, self.clock.to_wall(config.SUPERVISE_PERIOD_S)))
            try:
                await self.tick()
            except Exception as e:  # noqa: BLE001 — 감시 루프는 멈추지 않는다
                self._event("SUPERVISOR_ERROR", detail=f"{type(e).__name__}: {e}")

    def _reconcile_restart(self):
        def close(t):
            # 실행 키 계약 (UGV-04): 재시작 전 진행 중이던 임무는 FAILED(AGENT_RESTARTED). 같은 키 재전송은 이 기록을 돌려준다
            t["status"] = "FAILED"
            t["error"] = "AGENT_RESTARTED: 서버 재시작으로 주행 감시가 끊김 (차량 실제 상태는 텔레메트리로 다시 읽음)"
            t["physical_state"] = "UNKNOWN_AFTER_RESTART"
            t["progress"] = {**(t.get("progress") or {}), "phase": "FAILED"}
        basis = "SIM_DRIVER_RESET" if self.driver.kind == "sim" else "PX4_TELEMETRY_REREAD"
        self.store.reconcile_after_restart(
            lambda t: t.get("resource_id") == self.resource_id and t.get("status") in ACTIVE, close, basis=basis)

    # ------------------------------------------------------------------ 보조
    def _event(self, event_name, /, **kw):
        e = {"sim_time_s": round(self.clock.now(), 2), "timestamp": _now_iso(), "event": event_name,
             "resource_id": self.resource_id, "task_id": self.task_key, **kw}
        self.events.append(e)
        t = self.store.tasks.get(self.task_key) if self.task_key else None
        if t is not None:
            t.setdefault("events", []).append(e)
            t["events"] = t["events"][-50:]
        return e

    def _record_rejection(self, kind: str, body: dict, reason: str, detail: str, approach: Optional[dict],
                          keep_event: bool = True):
        """거절(평가·실행)을 남긴다 — 응답에만 싣고 버리면 개발·분석 때 사유를 다시 계산해야 했다 (2026-10-10).
        사건 기록(/ugv/{id}/events), 서버 로그, 상태 폴더의 ugv_rejections.jsonl (재시작해도 남음, 한 줄 한 건)."""
        ap = approach or {}
        rec = {"timestamp": _now_iso(), "sim_time_s": round(self.clock.now(), 2), "kind": kind, "resource_id": self.resource_id,
               "task_id": body.get("task_id"), "decision_id": body.get("decision_id"),
               "target": body.get("target"), "target_node": body.get("target_node"), "reason": reason, "detail": detail,
               "start": (ap.get("direction") or {}).get("start_road"),
               "counts": {k: ap.get(k) for k in ("candidates_checked", "candidates_no_directed_route",
                                                  "candidates_no_return_path", "unsafe_roads") if k in ap},
               "route_check_rejected": ap.get("route_check_rejected"),
               "avoided_roads_occupied_by_vehicles": ap.get("avoided_roads_occupied_by_vehicles")}
        if keep_event:
            self._event(kind, rejected_task_id=body.get("task_id"), reason=reason, detail=detail,
                        route_check_rejected=rec["route_check_rejected"])
        _log.info("%s %s task=%s: %s", kind, self.resource_id, body.get("task_id"), detail)
        try:
            path = Path(self.store.path).parent / "ugv_rejections.jsonl"
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
        except OSError as e:                     # 기록 실패가 응답을 막지 않는다
            _log.warning("거절 기록 저장 실패: %s", e)

    def here(self) -> Optional[RoadPos]:
        t = self.driver.telemetry()
        return None if t is None else self.net.snap(t.x, t.y)

    def _plan_heading(self, tel) -> Optional[float]:
        """주행 중이고 계획 경로 위에 있으면 계획 경로의 진행 방위 (교차로에서 차체 방향보다 믿을 만하다)."""
        if self.speed is None or self.state not in ("DRIVING", "EVADING", "STOPPING"):
            return None
        pts = self.speed.pts
        d, i = _progress_index(tel.x, tel.y, pts, self.speed.cum, self._near_i)   # 같은 자리 재통과 경로에서 나중 통과로 건너뛰지 않게
        if d > config.OFF_ROUTE_M / 3 or i + 1 >= len(pts):
            return None
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        return bearing_deg(bx - ax, by - ay) if math.hypot(bx - ax, by - ay) > 0 else None

    def _stop_need_grade(self, v: float, tel, path=None):
        """UGV_STOP_GRADE 후보: 필요거리 = v·τ + v²/(2·a_eff) + 여유, a_eff = max(a − g·하강, A_EFF_MIN) (terrain_speed 와 같은 모델).
        τ = max(서버 실측 명령 지연, TS.TAU_S — 2026-10-13 경사 중단 실측 명령→감속 시작 상한 3.0 s). 하강 = path(정지 경로 점)가 있으면 그 경로를
        따라 10 m 창 국부 하강의 최대, 없으면 진행 방향 직선 근사. 지형 없음·NaN = TS.GRADE_UNKNOWN (보수값, 표시). 불확실성 TS.GRADE_UNC 가산.
        raw 감속(a − g·하강)이 하한 이하면 insufficient 표시 — 정지 명령·감독은 그대로 (경로 거절로 끝내지 않음)."""
        from . import terrain_speed as TS
        tau = max(self.latency_s(), TS.TAU_S)
        a = self.profile.decel_mps2
        surf = TS.surface()
        grade, basis = None, None
        if surf is not None and path and len(path) >= 2:
            cum = [0.0]
            for (x0, y0), (x1, y1) in zip(path, path[1:]):
                cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))
            zs = []
            for x, y in path:
                try:
                    zs.append(surf.z(x, y))
                except Exception:  # noqa: BLE001
                    zs.append(float("nan"))
            if all(z == z for z in zs):
                worst = 0.0
                for i in range(len(path)):
                    j = next((k for k in range(i + 1, len(path)) if cum[k] - cum[i] >= 10.0), None)
                    if j is not None:
                        worst = max(worst, (zs[i] - zs[j]) / (cum[j] - cum[i]))
                grade, basis = max(worst, 0.0) + TS.GRADE_UNC, "STOP_PATH 10 m 창 국부 하강 최대"
        if grade is None and surf is not None:
            hd = math.radians(tel.heading_deg)
            d0 = v * tau + v * v / (2 * a) + config.STOP_MARGIN_M
            try:
                z0, z1 = surf.z(tel.x, tel.y), surf.z(tel.x + d0 * math.sin(hd), tel.y + d0 * math.cos(hd))
            except Exception:  # noqa: BLE001
                z0 = z1 = float("nan")
            if z0 == z0 and z1 == z1:
                grade, basis = max(0.0, (z0 - z1) / max(d0, 1.0)) + TS.GRADE_UNC, "진행 방향 직선 평균 (경로 전 근사)"
        if grade is None:
            grade, basis = TS.GRADE_UNKNOWN, "지형 없음·값 없음 — 보수값 (TS.GRADE_UNKNOWN)"
        raw = a - TS.G * grade
        a_eff = TS.a_eff(a, grade)
        need = v * tau + v * v / (2 * a_eff) + config.STOP_MARGIN_M
        return need, {"model": "v*tau + v^2/(2*max(a-g*grade_down,A_EFF_MIN)) + margin", "tau_s": round(tau, 2), "tau_basis": "max(latency_s, TS.TAU_S)",
                      "decel_mps2": a, "grade_down": round(grade, 4), "grade_basis": basis, "raw_decel_mps2": round(raw, 3),
                      "insufficient": raw <= TS.A_EFF_MIN, "base_needed_m": round(self.profile.stop_distance(v), 1)}

    def latency_s(self) -> float:
        """계획 출발점을 앞으로 옮길 명령 교체 지연: max(설정 하한, 최근 실측 상위 80 %). 상한 LATENCY_MAX_S.
        2026-10-10 Gazebo 실측 왕복 2.5 s 가 설정 1.0 s 보다 길었다 → 실측을 따른다 (지점을 지나쳐 되돌아가는 미션 방지)."""
        if not self._lat_samples:
            return self.cmd_latency_s
        srt = sorted(self._lat_samples)
        return min(max(self.cmd_latency_s, srt[int(0.8 * (len(srt) - 1))]), config.CMD_LATENCY_MAX_S)

    def _start_dirpos(self, tel, pos, heading_deg: float) -> Optional[DirPos]:
        return choose_start_dirpos(self.net, self.graph, tel.x, tel.y, heading_deg, tel.speed_mps,
                                   self.plan.route if self.plan is not None else None, pos)

    def _footprint_roads(self, other, tel) -> list:
        """상대 차체(프로필 길이·폭, 텔레메트리 원점 = 뒤축 중심)가 도로 띠(drive_graph 반폭 추정)에 걸친 도로 — 1 m 격자점·공간 색인.
        UGV_BLOCK_AVOID=footprint 비교 시험용 (근사: 반폭 = 차로 수 기반 추정, 원점 가정)."""
        from .drive_graph import est_half_width
        pr = other.profile
        hd = math.radians(tel.heading_deg)
        fx, fy = math.sin(hd), math.cos(hd)
        rx, ry = fy, -fx
        cx, cy = tel.x + fx * pr.rear_axle_from_center_m, tel.y + fy * pr.rear_axle_from_center_m
        hl, hw = pr.body_length_m / 2, pr.body_width_m / 2
        na, nb = max(2, int(math.ceil(2 * hl))), max(2, int(math.ceil(2 * hw)))
        hwmap = getattr(self.graph, "half_w", None) or {}
        found = set()
        for ia in range(na + 1):
            a = -hl + 2 * hl * ia / na
            for ib in range(nb + 1):
                b = -hw + 2 * hw * ib / nb
                x, y = cx + fx * a + rx * b, cy + fy * a + ry * b
                for rid, d in self.net.near(x, y, 12.0).items():
                    if rid not in found and d <= hwmap.get(rid, est_half_width(self.net.roads[rid])):
                        found.add(rid)
        return sorted(found)

    def here_state(self):
        """계획 출발 상태: (DirPos — 진행 방향, 명령 교체 지연 동안 갈 거리만큼 앞, 속도 m/s, 오류 사유)."""
        tel = self.driver.telemetry()
        if tel is None:
            return None, 0.0, "COMMUNICATION_FAILURE: 차량 위치 없음"
        pos = self.net.snap(tel.x, tel.y)
        hd = self._plan_heading(tel)
        dp = self._start_dirpos(tel, pos, hd if hd is not None else tel.heading_deg)
        if dp is None:
            return None, tel.speed_mps, f"WRONG_WAY_ON_ONEWAY: 일방통행 도로 {pos.road_id} 를 거꾸로 향함"
        v = max(tel.speed_mps, 0.0)
        # 지연 동안 갈 거리: 주행 중이면 지금 계획 경로를 따라, 아니면 진행 방향 도로를 따라 교차로 너머까지 예측한다
        prefer = None
        if hd is not None:
            _, i = _progress_index(tel.x, tel.y, self.speed.pts, self.speed.cum, self._near_i)
            prefer = [(tel.x, tel.y)] + self.speed.pts[i + 1:]
        return self.graph.project(dp, v * self.latency_s(), prefer), v, None

    def current_task(self) -> Optional[dict]:
        t = self.store.tasks.get(self.task_key) if self.task_key else None
        return t if t and t.get("status") in ACTIVE else None

    def resource_state(self) -> str:
        if self.state in ("FAULT", "DANGER"):
            return "UNAVAILABLE"
        if self.current_task() or self.state in ("DRIVING", "EVADING", "STOPPING") or self._stop_sup is not None:
            return "RUNNING"                        # 정지 감독이 정차를 확인하기 전에는 READY 가 아니다
        return "READY"

    def _approach(self, body: dict, start: DirPos, v0: float, extra_avoid: Optional[dict] = None) -> ApproachPlan:
        fire = self.fire_source.area()
        tgt = body.get("target") or {}
        fixed = None
        if body.get("target_node"):
            nid = body["target_node"]["node_id"] if isinstance(body["target_node"], dict) else body["target_node"]
            fixed = parse_node_id(self.net, nid)
            if fixed is None:
                raise HttpError(404, f"target_node 없음: {nid}")
        if tgt.get("lat") is None or tgt.get("lon") is None:
            if fixed is None:
                raise HttpError(422, "target(lat, lon) 또는 target_node 필요")
            tgt = dict(zip(("lat", "lon"), fixed.ll()))
        home = fixed is not None and fixed.road_id == self.access.road_id and abs(fixed.s - self.access.s) < 1.0
        avoid = {**self.occupied_roads(home=home), **(extra_avoid or {})}
        # 같은 조건(목표·출발 상태·피할 도로·화재 판)의 직전 계획을 다시 쓴다 — 총괄은 평가 직후 같은 목표로 실행을 보내는데,
        # 장거리(B→A)는 계획이 수 초라 실행 응답이 총괄 제한(10 s)을 넘어 '응답 불명'이 됐다 (2026-10-10 Gazebo 총괄 고장 인계).
        # 계획은 상태가 없는 함수라 같은 입력이면 같은 결과. 60 s 안에서만, 정지 상태(v0 = 0)에서만
        nid = body["target_node"]["node_id"] if isinstance(body.get("target_node"), dict) else body.get("target_node")
        key = (json.dumps(body.get("target") or {"n": nid}, sort_keys=True, default=str),
               start.road_id, round(start.pos.s, 1), start.toward, tuple(sorted(avoid.items())),
               getattr(fire, "version", None), getattr(fire, "source", None), len(getattr(fire, "cells", ()) or ()))
        cached = getattr(self, "_plan_cache", None)
        # 실행 요청은 평가가 고른 목적지 지점(target_node)을 함께 보낸다 — 그 지점이 직전 계획의 목적지와 같으면 같은 계획
        same_dest = nid is None or (cached is not None and cached[2].dest is not None and node_id_of(cached[2].dest) == nid)
        if v0 < 0.05 and cached is not None and cached[0] == key and same_dest and self.clock.now() - cached[1] <= 60.0:
            plan = cached[2]
        else:
            plan = plan_approach(self.net, fire, start, float(tgt["lat"]), float(tgt["lon"]), v0=v0, access=self.access,
                                 fixed_dest=fixed, graph=self.graph, avoid_roads=avoid,
                                 arrive_toward=self.access_toward if home else None)
            spare = {k: v for k, v in avoid.items() if str(v).endswith("(대기 도로)")}
            if home and plan.status != "OK" and spare:
                # 같은 거점 다른 차의 대기 도로(지금 그 차가 서 있지 않은 도로)를 피하면 복귀 경로가 없다 → 그 도로를 넣어 다시 계획하고 보고.
                # (2026-10-11 Gazebo c23: B 거점 대기 도로 683400513 이 회전 모형상 거점 주변과 도로망을 잇는 유일한 연결로 — B-fire1 이
                # 거점 12 m 옆에서 복귀가 계속 거절돼 갇혔다. 원인 검증 Fable. 대기 도로 선택 자체의 수정은 범위 밖 — 보고서 한계)
                plan2 = plan_approach(self.net, fire, start, float(tgt["lat"]), float(tgt["lon"]), v0=v0, access=self.access,
                                      fixed_dest=fixed, graph=self.graph,
                                      avoid_roads={k: v for k, v in avoid.items() if k not in spare},
                                      arrive_toward=self.access_toward)
                if plan2.status == "OK":
                    plan2.detail["spare_roads_released"] = spare
                    plan2.detail["spare_release_reason"] = ("같은 거점 다른 차 대기 도로를 피하면 복귀 경로 없음 — 그 차는 지금 그 도로에 없다. "
                                                            "마주침은 교통 관리가 조정")
                    plan = plan2
            if v0 < 0.05:
                self._plan_cache = (key, self.clock.now(), plan)
        plan = self._check_start_road_blocker(plan, start, home)
        return plan if home else self._check_return_from_dest(plan, fire)

    RETURN_CHECK_BEND_FACTOR = 1.5

    def _check_return_from_dest(self, plan: ApproachPlan, fire) -> ApproachPlan:
        """모든 목표(config.RETURN_CHECK_ALL, 기본) — 예전에는 도착할 도로에 급커브에 가까운 굽음(도로 최소 반경 < 최소 회전반경 ×
        RETURN_CHECK_BEND_FACTOR)이 있을 때만 — 도착 자세(도착 방향)에서
        소속 거점까지 복귀 계획을 실제로 세워 최종 검사를 통과하는지 본다. 나가는 경로는 도로의 들어온 쪽 절반만 지나므로, 남은 쪽 굽음이
        최종 검사(±3 m 창)에서 최소 회전반경에 걸려 도착한 뒤 갇힐 수 있다 (2026-10-11 Gazebo: 소방차가 682502608 가운데 목표에서
        모든 경로 반경 5.21 m 로 거절 — 도로 급커브 판정은 ±7.5 m 창이라 놓침). 2026-10-11 소방차 무작위 목표 12개: 급커브 후보만 보면
        놓치는 '도착하면 못 돌아옴' 2개 → 모든 목표로 (평가 평균 0.63 → 1.25 s). 결과는 도착 자세·그래프·화재 버전별로 재사용."""
        if plan.status != "OK" or plan.route is None or plan.dest is None:
            return plan
        rid = plan.dest.road_id
        if (not config.RETURN_CHECK_ALL
                and self.graph.min_radius.get(rid, math.inf) >= self.profile.r_min * self.RETURN_CHECK_BEND_FACTOR):
            return plan
        toward = plan.route.arrive_toward or (plan.route.legs[-1][2] if plan.route.legs else None)
        if toward is None:
            return plan
        # 결과 재사용: 같은 도착 자세(도로·5 m 칸·방향)·차종 그래프·화재 자료 버전·출입 지점
        key = (rid, int(plan.dest.s // 5.0), toward, id(self.graph), getattr(fire, "version", None), self.access.road_id)
        cache = self.__dict__.setdefault("_ret_cache", {})
        if key in cache:
            ok, info = cache[key]
            plan.detail["return_check_from_dest"] = {**info, "cached": True}
            if ok:
                return plan
            import copy
            bad = copy.copy(plan)
            bad.status, bad.reason = "TARGET_UNREACHABLE", info["reason_full"]
            bad.detail = {**plan.detail}
            return bad
        from .drive_graph import DirPos as _DP
        start = _DP(plan.dest, toward)
        acc = self.access
        back = plan_approach(self.net, fire, start, *acc.ll(), v0=0.0, access=acc, fixed_dest=acc, graph=self.graph,
                             arrive_toward=self.access_toward)
        if len(cache) > 500:
            cache.clear()
        if back.status == "OK":
            plan.detail["return_check_from_dest"] = {"ok": True, "road": rid, "length_m": round(back.route.length, 1)}
            cache[key] = (True, dict(plan.detail["return_check_from_dest"]))
            return plan
        import copy
        bad = copy.copy(plan)
        bad.status = "TARGET_UNREACHABLE"
        bad.reason = (f"NO_RETURN_FROM_DEST_AFTER_CHECK: 도착 도로 {rid} (최소 반경 {self.graph.min_radius[rid]:.1f} m) 에 서면 "
                      f"거점 복귀 경로가 최종 검사를 통과하지 못함 ({back.status}: {back.reason})")
        bad.detail = {**plan.detail, "return_check_from_dest": {"ok": False, "road": rid, "reason": back.reason}}
        cache[key] = (False, {"ok": False, "road": rid, "reason": back.reason, "reason_full": bad.reason})
        return bad

    def _check_start_road_blocker(self, plan: ApproachPlan, start: DirPos, home: bool) -> ApproachPlan:
        """출발 도로 앞쪽에 스스로 움직이지 않을 차(occupied_roads 와 같은 기준)가 서 있으면, 그 차 앞에서 끝나는 경로 말고는
        갈 수 없다 (즉시 유턴·후진 없음, 중심선 모형이라 추월 없음). plan_approach 는 출발 도로를 계속 달리는 것을 막지 않으므로
        여기서 거른다 → ROAD_BLOCKED. (2026-10-10 총괄 폐루프: 막힘으로 실패해 선 UGV 를 총괄이 같은 임무에 다시 배정했고
        평가가 ACCEPT 해 막힘·실패·재배정이 되풀이됨)"""
        if plan.status != "OK" or plan.route is None:
            return plan
        from .traffic import TRAFFIC_GAP_M
        road = self.net.roads[start.road_id]
        fwd = start.toward == road.b
        legs = plan.route.legs or []
        leaves = len(legs) > 1 or (legs and legs[0][0] != start.road_id)
        for p in self.peers():
            t = p.driver.telemetry()
            if t is None or t.speed_mps >= 0.3 or p.state in ("DRIVING", "EVADING"):
                continue
            if p.current_task() is not None and p.state not in ("FAULT", "DANGER"):
                continue
            q = self.net.snap(t.x, t.y)
            if q is None or q.road_id != start.road_id or (home and q.road_id == self.access.road_id):
                continue
            ahead = (q.s - start.pos.s) if fwd else (start.pos.s - q.s)
            if ahead <= 0:
                continue
            need = (self.profile.body_length_m + p.profile.body_length_m) / 2 + TRAFFIC_GAP_M
            dest_ahead = None
            if not leaves and plan.dest is not None and plan.dest.road_id == start.road_id:
                dest_ahead = (plan.dest.s - start.pos.s) if fwd else (start.pos.s - plan.dest.s)
            if dest_ahead is not None and dest_ahead <= ahead - need:
                continue                         # 막은 차 앞(간격 확보)에서 끝나는 경로
            import copy
            blocked = copy.copy(plan)                       # 원본(계획 재사용 캐시)은 그대로 둔다
            blocked.status = "ROAD_BLOCKED"
            blocked.reason = (f"BLOCKED_BY_VEHICLE: {p.resource_id} 가 출발 도로 {start.road_id} 앞 {ahead:.0f} m 에 서 있음 "
                              f"({p.state}), 유턴·추월 없이 지날 수 없음")
            blocked.detail = {**plan.detail, "start_road_blocker": {"resource_id": p.resource_id, "road_id": start.road_id,
                                                                    "ahead_m": round(ahead, 1), "state": p.state}}
            return blocked
        return plan

    def occupied_roads(self, home: bool = False) -> dict:
        """스스로 움직이지 않을 다른 차량이 차지한 도로 {도로: 차량}: 고장·위험(운영자 해제 전 정지) 또는 임무 없이 자기 거점 대기
        위치에 서 있는 차. 그 밖에 잠깐 서 있는 차는 피하지 않는다 (움직일 수 있다 — 주행 중 충돌은 ugv/traffic.py 가 막는다).
        자기 출입 도로는 복귀 임무일 때 뺀다 (같은 거점 대기 위치 순서는 traffic 이 기다리게 한다).
        복귀 임무면 같은 거점 다른 차의 대기 도로도 뺀다 — 지금 비어 있어도 그 차가 돌아와 서면 길이 막힌다. 대기 도로는 막아도 도로망이
        끊기지 않는 도로로 골랐다 (drive_graph.access_point). (2026-10-10 Gazebo 4대: 대기 방향 도착을 위해 블록을 도는 A-ugv1 복귀 경로가
        A-fire1 대기 도로를 지났고, 먼저 돌아온 A-fire1 앞에서 시험 끝까지 기다림)"""
        out = {}
        if home:
            for p in self.peers():
                if p.station.get("base_id") == self.station.get("base_id") and p.access.road_id != self.access.road_id:
                    out[p.access.road_id] = f"{p.resource_id} (대기 도로)"
        for p in self.peers():
            t = p.driver.telemetry()
            if t is not None and p.fault and p.fault.startswith("TELEMETRY_LOST") and p.speed is not None:
                # 위치를 모르는 차: 마지막 위치부터 남은 경로를 따라 갈 수 있었던 구간의 도로 전부 (ugv/traffic.py 와 같은 범위)
                from .traffic import link_loss_reach
                sp = p.speed
                i0 = max(0, min(p._near_i, len(sp.pts) - 1))
                reach = link_loss_reach(sp, i0, t.speed_mps)
                for k in range(i0, len(sp.pts)):
                    if sp.cum[k] - sp.cum[i0] > reach:
                        break
                    q = self.net.snap(*sp.pts[k])
                    rid_k = q.road_id if q is not None else None
                    if rid_k is not None and not (home and rid_k == self.access.road_id):
                        out[rid_k] = p.resource_id
                continue
            if t is None or t.speed_mps >= 0.3 or p.state in ("DRIVING", "EVADING"):
                continue
            # 임무 없이 서 있는 차(도착 뒤 대기·거점 대기)와 고장·위험 차는 지시 없이는 움직이지 않는다 → 그 도로로 계획하지 않는다.
            # (2026-10-10 Gazebo: 목적지에 서 있던 B-fire1 을 지나야 하는 B-ugv1 복귀 경로를 골라 길이 막혔다)
            if p.current_task() is not None and p.state not in ("FAULT", "DANGER"):
                continue
            q = self.net.snap(t.x, t.y)
            if q is None:
                # 도로망에 붙일 수 없는 곳(도로 밖·월드 밖)에 있는 차 — 막는 도로가 없다 (2026-10-11 Gazebo 지속 운용: 바닥 밖으로 떨어진
                # 차 때문에 이 함수가 예외로 죽어 /state 가 500)
                continue
            rid = q.road_id
            if home and rid == self.access.road_id:
                continue
            out[rid] = p.resource_id
        return out

    def _target_node(self, plan: ApproachPlan) -> Optional[dict]:
        if plan.dest is None:
            return None
        lat, lon = plan.dest.ll()
        return {"node_id": node_id_of(plan.dest), "lat": round(lat, 7), "lon": round(lon, 7),
                "snap_m": round(plan.dist_to_target_m or 0.0, 1)}

    # ------------------------------------------------------------------ 판단 (움직이지 않음)
    async def evaluate(self, body: dict) -> dict:
        base = {"task_id": body.get("task_id"), "decision_id": body.get("decision_id"), "resource_id": self.resource_id}
        if self.state in ("FAULT", "DANGER"):
            return {**base, "verdict": "REJECT", "eta_sec": None, "reason": "FAILSAFE_ACTIVE",
                    "detail": f"{self.state}: {self.fault}", "target_node": None, "path": None}
        fd = self._fire_data_problem()
        if fd:
            return {**base, "verdict": "REJECT", "eta_sec": None, "reason": "FIRE_DATA_UNAVAILABLE", "detail": fd,
                    "target_node": None, "path": None}
        start, v0, err = self.here_state()
        if start is None:
            return {**base, "verdict": "REJECT", "eta_sec": None, "reason": err.split(":")[0],
                    "detail": err, "target_node": None, "path": None}
        plan = await asyncio.to_thread(self._approach, body, start, v0)       # 계획 계산은 이벤트 루프 밖에서
        if plan.status != "OK":
            out = {**base, "verdict": "REJECT", "eta_sec": None,
                   "reason": "ROAD_BLOCKED" if plan.status == "ROAD_BLOCKED" else "TARGET_UNREACHABLE",
                   "detail": f"{plan.status}: {plan.reason}", "target_node": None, "path": None,
                   "approach": plan.report()}
            self._record_rejection("EVALUATE_REJECTED", body, out["reason"], out["detail"], plan.report())
            return out
        sp = plan_speeds(plan.route, v0, profile=self.profile)
        out = {**base, "verdict": "ACCEPT", "eta_sec": int(round(sp.eta_s)), "reason": None, "detail": None,
               "target_node": self._target_node(plan), "path": plan.route.road_ids, "approach": plan.report()}
        if self.current_task():
            out["detail"] = f"실행하면 진행 중 임무 {self.task_key} 를 취소하고 새로 계획한다"
            out["supersedes_task_id"] = self.task_key
        return out

    # ------------------------------------------------------------------ 실행
    async def execute(self, body: dict) -> dict:
        key = body.get("task_id")
        if not key:
            raise HttpError(422, "task_id 필요")
        if not body.get("target") and not body.get("target_node"):
            raise HttpError(422, "target(lat, lon) 또는 target_node 필요")
        async with self.lock:
            kind, meta = self.store.check(key, body)
            if kind == "DUPLICATE":
                t = self.store.tasks.get(key, {})
                return {**meta["response"], "duplicate": True, "current_status": t.get("status")}
            if kind == "CONFLICT":
                raise HttpError(409, {"reason": "EXECUTION_ID_CONFLICT",
                                      "detail": f"같은 task_id {key} 로 다른 내용이 이미 실행됨"})
            if self.state in ("FAULT", "DANGER"):
                raise HttpError(409, f"FAILSAFE_ACTIVE: {self.state} {self.fault} (/stop 으로 운영자 해제 필요)")
            fd = self._fire_data_problem()
            if fd:
                raise HttpError(409, {"reason": "FIRE_DATA_UNAVAILABLE", "detail": fd})
            w0 = time.monotonic()
            lat_used = self.latency_s()
            start, v0, err = self.here_state()
            plan = await asyncio.to_thread(self._approach, body, start, v0) if start is not None else None
            if plan is None or plan.status != "OK":
                # 진행 방향을 지키는 경로가 없다 → 새 목표를 수행하지 않는다. 주행 중이던 임무는 취소하고 도로를 따라 멈춘다
                cancelled, action = None, None
                if self.current_task():
                    cancelled = self.task_key
                    self.gen += 1
                    reason = err or f"{plan.status}: {plan.reason}"
                    self._finish(cancelled, "CANCELLED", error=f"NEW_TARGET_NO_FEASIBLE_ROUTE:{key}: {reason}",
                                 physical_state="STOPPING")
                    await self._safe_stop()
                    action = "STOPPING_ALONG_ROAD"
                why = "ROAD_BLOCKED" if plan is not None and plan.status == "ROAD_BLOCKED" else "TARGET_UNREACHABLE"
                self._record_rejection("EXECUTE_REJECTED", body, why, err or f"{plan.status}: {plan.reason}",
                                       plan.report() if plan else None, keep_event=False)
                self._event("NEW_TARGET_REJECTED", rejected_task_id=key, detail=err or plan.reason,
                            cancelled_task_id=cancelled, vehicle_action=action)
                raise HttpError(409, {"reason": why,
                                      "detail": err or f"{plan.status}: {plan.reason}",
                                      "approach": plan.report() if plan else None,
                                      "cancelled_task_id": cancelled, "vehicle_action": action})
            prev = self.task_key if self.current_task() else None
            if prev:
                self._finish(prev, "CANCELLED", error=f"SUPERSEDED_BY:{key}", physical_state="REPLANNED")
            tel = self.driver.telemetry()
            here_d = math.hypot(tel.x - plan.dest.x, tel.y - plan.dest.y) if tel is not None else math.inf
            already = tel is not None and tel.speed_mps < 0.2 and here_d <= config.ARRIVE_M
            self.gen += 1
            if already:
                # 이미 목적지 도착 반경 안에 서 있다 → 주행하지 않고 바로 도착 처리
                # (길이 0 경로로 출발시키면 차가 움직이지 않아 정지 고장(STALLED)이 된다, 2026-10-10 실서버 시험)
                self.plan, self.speed = plan, plan_speeds(plan.route, 0.0, profile=self.profile)
                seq = self.driver.progress().mission_seq
            else:
                w1 = time.monotonic()
                try:
                    seq = await self._drive(plan)
                except DriverError as e:
                    await self._safe_stop()
                    self.task_key = None
                    raise HttpError(500, f"DRIVER_START_FAILED: {e}")
                w2 = time.monotonic()
                self._lat_samples.append(w2 - w0)
                self.last_cmd_timing = {"plan_s": round(w1 - w0, 3), "driver_start_s": round(w2 - w1, 3),
                                        "total_s": round(w2 - w0, 3), "latency_assumed_s": round(lat_used, 2),
                                        "v0_mps": round(v0, 2), "projected_m": round(v0 * lat_used, 1),
                                        "trimmed_items_behind": getattr(self, "_trimmed", 0)}
            self.task_key = key
            self.target_ll = (body.get("target") or {}) or dict(zip(("lat", "lon"), plan.dest.ll()))
            tn = self._target_node(plan)
            resp = {"task_id": key, "resource_id": self.resource_id, "status": "STARTED",
                    "command_timing": None if already else getattr(self, "last_cmd_timing", None),
                    "tracking_url": f"/ugv/{self.resource_id}/task/{key}", "target_node": tn,
                    "eta_sec": int(round(self.speed.eta_s)), "approach": plan.report(),
                    "superseded_task_id": prev, "duplicate": False}
            task = {"task_id": key, "decision_id": body.get("decision_id"), "resource_id": self.resource_id,
                    "status": "STARTED", "target": body.get("target"), "target_node": tn["node_id"] if tn else None,
                    "approach": plan.report(), "mission_result": "NOT_ARRIVED", "physical_state": "DRIVING",
                    "progress": {"phase": "ENROUTE", "waypoint": 0, "total": len(self.speed.items)},
                    "observation": None, "error": None, "replans": [], "events": [],
                    "timing": {"start_sim_s": round(self.clock.now(), 2), "start_wall": _now_iso(),
                               "eta_sec": resp["eta_sec"]},
                    "superseded_task_id": prev, "mission_seq": seq}
            self.store.register(key, body, task, resp)
            self._event("TASK_STARTED", approach=plan.report(), eta_sec=resp["eta_sec"], superseded=prev)
            if already:
                self.state = "IDLE"
                await self._arrived(tel, here_d)
            return resp

    async def _drive(self, plan: ApproachPlan, state="DRIVING") -> int:
        # 실행 경계의 최종 확인: 최종 검증(approach.verify_route)을 통과한 경로만 드라이버로 보낸다.
        # 검사 결과를 보고에만 싣고 실행은 하는 우회를 막는다 (2026-10-10 검토 — 예전 화재 이탈이 그랬다)
        chk = (plan.detail or {}).get("route_check") or {}
        if chk.get("ok") is not True:
            raise DriverError(f"UNVERIFIED_ROUTE: 최종 경로 검사 통과 기록 없음 ({plan.mode}, route_check={chk.get('ok')})")
        t = self.driver.telemetry()
        self.speed = plan_speeds(plan.route, t.speed_mps if t else 0.0, profile=self.profile)
        self.plan = plan
        self.hold = None
        items = self.speed.items
        self._trimmed = 0
        t = self.driver.telemetry()
        if t is not None and len(items) > 1:
            # 계획하는 동안 차가 이미 지나친 앞쪽 미션 지점은 빼고 보낸다 (뒤에 남은 지점으로 되돌아가지 않게).
            # 지난 판정 = 계획 선형 위 차량 위치보다 앞(경로를 따라 2 m 이상 뒤)에 있는 지점
            sp = self.speed
            hi = next((k for k, c in enumerate(sp.cum) if c > 300.0), len(sp.pts) - 1)   # 경로 앞부분만 (되도는 경로 오인 방지)
            _, iv = _progress_seg(t.x, t.y, sp.pts, 0, hi)
            s_veh = sp.cum[iv] + math.dist(sp.pts[iv], (t.x, t.y)) if iv > 0 else 0.0
            keep, j = [], 0
            # 미션 지점(보정 전 좌표)은 속도 계획 점의 부분열 — 차례로 맞춘다
            for it, on in zip(items, sp.items_on_path or items):
                while j < len(sp.pts) - 1 and math.dist(sp.pts[j], on[:2]) > 1e-6:
                    j += 1
                if sp.cum[j] >= s_veh + 2.0 or it is items[-1]:
                    keep.append(it)
            self._trimmed = len(items) - len(keep)
            items = keep
        self.seq = await self.driver.follow(items)
        self.state = state
        self.stop_path_pts, self.command_uncertain, self._hold_failures = None, False, 0
        self._stop_sup = None                       # 새 주행 미션 — 주행 감독이 맡는다
        self._spd_cmd = None                        # UGV_SPEED_TERRAIN_CMD: 새 미션 — 마지막 보낸 값 초기화
        self._off_since, self._near_i, self._offroad_since = None, 0, None
        self._move_ref = (self.clock.now(), t.x, t.y) if t else None
        self._drive_started_sim = self.clock.now()
        return self.seq

    async def _safe_stop(self) -> dict:
        """도로를 따라 감속 정지. 남은 계획 경로가 있으면 그것을, 없으면 진행 방향 도로를 따라 제동거리만큼 간다.
        즉시 멈춘다고 가정하지 않는다. 도로 위치·방향을 모르면 드라이버 기본 정지(진행 방향 직선)로 물러선다.
        정지 경로가 제동거리보다 짧거나(short) 회전반경·차체 도로 포함 검사를 통과하지 못하면(drivable=false) 그 사실을
        돌려주고 기록한다 — 정지 명령은 그래도 보낸다 (다른 선택지가 없다). 정지가 안전 확보를 뜻하지 않는다."""
        info = {}
        self.hold = None
        if self.stop_cut_active:
            # 구동 차단 중: 정지 미션을 다시 보내면 시동이 다시 걸린다 → 보내지 않고 차단을 유지한다
            await self._cut_drive("STOP_REQUEST_WHILE_CUT")
            info = {"basis": "CUT_DRIVE_ACTIVE", "problems": ["정지 감독 구동 차단 중"]}
            self.state = "STOPPING" if self.state not in ("FAULT", "DANGER") else self.state
            self.last_stop = info
            return info
        try:
            tel = self.driver.telemetry()
            items = None
            if tel is not None and tel.speed_mps >= 0.3:
                v = tel.speed_mps
                need = self.profile.stop_distance(v)
                need_info = None
                if STOP_GRADE_MODE:
                    need, need_info = self._stop_need_grade(v, tel)
                g = self.graph
                prefer = None
                if self.speed is not None and self._plan_heading(tel) is not None:
                    # 예전과 같은 조건 (주행 상태·계획선 OFF_ROUTE_M/3 안)
                    _, i = _progress_index(tel.x, tel.y, self.speed.pts, self.speed.cum, self._near_i)
                    prefer = [(tel.x, tel.y)] + self.speed.pts[i + 1:]
                elif self.speed is not None and self.state == "FAULT" and FAULT_STOP_PLAN_M > 0:
                    # 선택(UGV_FAULT_STOP_PLAN_M>0): 고장 정지도 계획선 가까이면 남은 계획 경로로. _fail 이 상태를 먼저 FAULT 로 바꿔
                    # _plan_heading 이 None 이 되는 문제 보완 — 기본 끔 (다른 차의 대체 경로를 막는 회귀, 위 상수 설명)
                    d_pl, i = _progress_index(tel.x, tel.y, self.speed.pts, self.speed.cum, self._near_i)
                    if d_pl <= FAULT_STOP_PLAN_M and i + 1 < len(self.speed.pts):
                        prefer = [(tel.x, tel.y)] + self.speed.pts[i + 1:]
                pos = self.net.snap(tel.x, tel.y)
                dp = g.dirpos(pos, tel.heading_deg) if prefer is None else None
                if prefer is None and dp is not None and not self._road_stop_fits(pos, tel):
                    # 도로에서 멀거나 도로 방향이 진행 방향과 크게 다르면 '진행 방향 도로' 정지 경로가 차 옆·뒤를 가리킨다
                    # (2026-10-11 Gazebo: 도로 밖 46 m 에서 옆 도로로 향한 정지 경로 → 급회전·맴돌기). 진행 방향 직선 제동으로
                    dp = None
                info = {"basis": "NO_ROAD_DIRECTION", "needed_m": round(need, 1), "speed_mps": round(v, 2)}
                if need_info:
                    info["stop_need_model"] = need_info
                if prefer is not None or dp is not None:
                    pts, got = g.stop_path(dp, need, prefer)
                    if STOP_GRADE_MODE and len(pts) >= 2:
                        need2, need_info = self._stop_need_grade(v, tel, path=pts)
                        if need2 > need + 0.5:
                            need = need2
                            pts, got = g.stop_path(dp, need, prefer)
                    if len(pts) >= 2 and got > 1.0:
                        # 속도 상한 = 지금 속도 (정지 경로는 감속만). 예전에는 최고 속도라 정지 거리 여유(STOP_MARGIN_M) 때문에 시작 구간에서
                        # 지금보다 빠른 속도를 허용해, 2026-10-11 Gazebo 재검증에서 취소·고장 정지 직후 1.3~2 m/s 가속했다
                        route = Route(pts, [max(v, 0.5)] * len(pts), [], pos, pos, 0.0)
                        items = plan_speeds(route, v, profile=self.profile).items
                        chk = g.check_route(pts, label="STOP_PATH")
                        info = {"basis": "PLANNED_ROUTE" if prefer else "ROAD_AHEAD", "length_m": round(got, 1),
                                "needed_m": round(need, 1), "short": got < need - 0.5, "speed_mps": round(v, 2),
                                **({"stop_need_model": need_info} if need_info else {}),
                                "drivable": chk["ok"], "steering_violations": chk["steering_violations"],
                                "body_violations": chk["body_violations"], "worst_excess_m": chk["worst_excess_m"]}
                        if not chk["ok"] and OFFROAD_STOP_MODE == "emergency":
                            # 검사를 통과하지 못한 정지 경로는 보내지 않는다 → 아래 비상 감속 직선 (거절한 경로를 기록)
                            items = None
                            info = {"basis": "NO_ROAD_DIRECTION", "rejected_stop_path": info}
            if not items and tel is not None and tel.speed_mps >= 0.3 and OFFROAD_STOP_MODE == "emergency":
                v = tel.speed_mps
                d = max(v * v / (2 * self.profile.emergency_decel_mps2) + 1.0, EMERG_STOP_MIN_M)
                hd = math.radians(tel.heading_deg)
                pts = [(tel.x, tel.y), (tel.x + d * math.sin(hd), tel.y + d * math.cos(hd))]
                items = [(pts[1][0], pts[1][1], 1.0)]
                info = {**info, "basis": "EMERGENCY_STRAIGHT (도로를 따르는 정지 경로 없음 — 진행 방향 직선, 비상 감속)",
                        "length_m": round(d, 1), "needed_m": round(d, 1), "speed_mps": round(v, 2), "drivable": None,
                        "decel_mps2": self.profile.emergency_decel_mps2}
            if items:
                self.seq = await self._send_stop(self.driver.follow(items))
                self.stop_path_pts = [tuple(q) for q in pts]
                if STOP_SPEED_CMD and tel is not None:
                    info["speed_cap_cmd"] = {"v": round(tel.speed_mps, 2), "ok": await self.driver.set_speed_cap(tel.speed_mps)}
            else:
                self.seq = await self._send_stop(self.driver.stop())
                if tel is not None and tel.speed_mps >= 0.3:       # 드라이버 기본 정지 = 진행 방향 직선
                    hd = math.radians(tel.heading_deg)
                    d = self.profile.stop_distance(tel.speed_mps)
                    self.stop_path_pts = [(tel.x, tel.y), (tel.x + d * math.sin(hd), tel.y + d * math.cos(hd))]
                else:
                    self.stop_path_pts = None
                info = {**info, "basis": info.get("basis", "STANDSTILL") if tel is not None and tel.speed_mps < 0.3
                        else "DRIVER_DEFAULT (진행 방향 직선 — 도로를 따르지 않을 수 있음)"}
                if tel is not None and tel.speed_mps >= 0.3:
                    info["drivable"] = None
            self.command_uncertain = False
            self.state = "STOPPING"
            self._sup_begin(tel, info.get("length_m"), info.get("basis"))
            problems = []
            if info.get("short"):
                problems.append(f"정지 경로 {info['length_m']} m < 필요 {info['needed_m']} m")
            if info.get("drivable") is False:
                problems.append("정지 경로가 회전반경·차체 도로 포함 검사를 통과하지 못함")
            if (info.get("stop_need_model") or {}).get("insufficient"):
                problems.append("경사로 유효 감속 부족 (후보 정지 모델) — 정지 감독 유지")
            if info.get("drivable") is None and ("DRIVER_DEFAULT" in str(info.get("basis")) or "EMERGENCY" in str(info.get("basis"))):
                problems.append("도로 방향을 몰라 직선 정지")
            info["problems"] = problems
            self._event("STOP_PATH", **info)
        except DriverError as e:
            info = {"basis": "FAILED", "problems": [f"정지 명령 실패: {e}"]}
            self.command_uncertain, self.stop_path_pts = True, None
            self._stop_retry_at = self.clock.now()
            self._event("STOP_FAILED", detail=str(e))
            self._sup_begin(self.driver.telemetry(), None, "STOP_COMMAND_FAILED")
        self.last_stop = info
        return info

    def _finish(self, key, status, *, error=None, physical_state=None, mission_result=None):
        t = self.store.tasks.get(key)
        if not t or t.get("status") not in ACTIVE:
            return
        t["status"] = status
        if error:
            t["error"] = error
        if physical_state:
            t["physical_state"] = physical_state
        if mission_result:
            t["mission_result"] = mission_result
        if status in ("FAILED", "CANCELLED"):
            # 총괄 계약 (orchestrator/engine.py 제공자 FAILED 처리): progress.phase 가 FAILED 면 고장으로 묶고(운영자 /stop 해제 뒤 반납),
            # DONE 이면 READY 확인 뒤 반납한다. 그 밖(ENROUTE 등)이면 위치 미확인으로 점유를 계속 잡아 둔다 — 막혀 멈춘 멀쩡한 차도
            # 다시 배정되지 못했다 (2026-10-10 계약 대조). 고장·위험(fault)이 있으면 FAILED, 아니면 DONE (차는 서 있거나 서는 중)
            t.setdefault("progress", {})["phase"] = "FAILED" if (status == "FAILED" and self.fault) else "DONE"
        t.setdefault("timing", {})["end_sim_s"] = round(self.clock.now(), 2)
        t["timing"]["end_wall"] = _now_iso()
        self._event(f"TASK_{status}", detail=error)
        self.store.save()

    # ------------------------------------------------------------------ 중단·정지
    async def abort(self, key: str, reason: str = "ABORT_REQUESTED") -> dict:
        async with self.lock:
            t = self.store.tasks.get(key)
            if t is None or t.get("resource_id") != self.resource_id:
                raise HttpError(404, f"task {key} 없음")
            if key == self.task_key and t.get("status") in ACTIVE:
                self.gen += 1
                await self._safe_stop()
                self._finish(key, "CANCELLED", error=reason, physical_state="STOPPING")
            return t

    async def stop(self, reason: str = "STOP_REQUESTED") -> dict:
        """주행 중지 + 운영자 해제. FAULT·DANGER 는 차량이 멈췄고(0.2 m/s 미만) 텔레메트리가 살아 있을 때만 해제한다.
        아직 움직이면 도로를 따라 멈추게 하고 해제는 보류한다 (다시 /stop 으로 해제)."""
        async with self.lock:
            self.gen += 1
            if self.current_task():
                self._finish(self.task_key, "CANCELLED", error=reason, physical_state="STOPPING")
            tel = self.driver.telemetry()
            stopped = (tel is not None and tel.speed_mps < 0.2 and tel.age_s() <= config.TELEMETRY_STALE_S
                       and self._stop_sup is None)
            released, pending = None, None
            if self.fault is not None and not stopped:
                pending = "NOT_STOPPED_OR_NO_TELEMETRY: 정지·텔레메트리 확인 뒤 다시 /stop"
                held = self.state
                if tel is not None and tel.speed_mps >= 0.2:
                    await self._safe_stop()
                self.state = held                      # 고장·위험 상태 유지 (자동 재출발 없음)
            else:
                released = self.fault
                self.fault = None
                self.danger = None
                if self.state in ("FAULT", "DANGER"):
                    self.state = "IDLE"
                await self._safe_stop()                # 멈춰 있으면 드라이버가 제자리 정지로 처리한다
            self._event("STOP", detail=reason, released_fault=released, release_pending=pending)
            return {"resource_id": self.resource_id, "state": self.resource_state(), "released_fault": released,
                    "release_pending": pending}

    # ------------------------------------------------------------------ 감시
    def _fail(self, code: str, detail: str):
        self.fault = f"{code}: {detail}"
        if self.current_task():
            self._finish(self.task_key, "FAILED", error=self.fault, physical_state=code)
        self.state = "FAULT"
        self._event("FAULT", code=code, detail=detail)

    async def tick(self):
        now = self.clock.now()
        tel = self.driver.telemetry()
        if tel is None:
            return
        moving = self.state in ("DRIVING", "EVADING")
        if moving and tel.age_s() > config.TELEMETRY_STALE_S:
            async with self.lock:
                self._fail("TELEMETRY_LOST", f"위치·속도 수신 {tel.age_s():.1f} s 없음")
                await self._safe_stop()
                self.state = "FAULT"
            return
        if tel.armed:
            self._last_armed = (now, max(tel.speed_mps, 0.0))
        if self.state == "STOPPING" and self._standstill(tel):
            self.state = "IDLE"
            self.stop_path_pts = None
            for t in self.store.tasks.values():           # 정지 중으로 끝낸 임무의 물리 상태를 정차로 (임무 실패와 정차를 따로)
                if t.get("resource_id") == self.resource_id and str(t.get("physical_state", "")).startswith("STOPPING"):
                    t["physical_state"] = "STOPPED" + t["physical_state"][len("STOPPING"):]
            self.store.save()
            self._event("STOPPED", position={"lat": tel.lat, "lon": tel.lon})
        if self._stop_sup is not None:
            await self._sup_check(now, tel)
        if (self.command_uncertain and not self.stop_cut_active and now - self._stop_retry_at >= STOP_RETRY_S
                and tel.age_s() <= config.TELEMETRY_STALE_S):
            # 정지 명령이 실패했던 차: 다시 보낸다 (고장·위험 상태는 그대로 둔다)
            self._stop_retry_at = now
            async with self.lock:
                keep = self.state
                await self._safe_stop()
                if keep in ("FAULT", "DANGER"):
                    self.state = keep
                self._event("STOP_RETRY", ok=not self.command_uncertain)
        if (self.state in ("IDLE", "FAULT", "DANGER", "STOPPING") and self._stop_sup is None and tel.speed_mps > 0.5
                and tel.age_s() <= config.TELEMETRY_STALE_S and self.current_task() is None):
            # 감독 없는 움직임: 임무도 정지 감독도 없는데 움직인다 — 서버 재시작으로 진행 중 임무 감시가 끊겼는데 PX4 는 남은 미션을
            # 계속 달리는 경우 (재시작 정리는 임무 기록만 FAILED 로 바꾼다), 정차 확인 뒤 다시 움직이는 경우 (고장·위험 차 포함).
            # READY 로 두지 않고 도로를 따라 세우고 정지 감독을 건다. 시동이 꺼진 채 구르면 정지 미션(시동을 건다) 대신 감독만
            async with self.lock:
                if self._stop_sup is None and self.current_task() is None:
                    keep = self.state
                    self._event("UNSUPERVISED_MOTION_STOP", speed_mps=round(tel.speed_mps, 2), mode=tel.mode,
                                armed=tel.armed, state=keep)
                    if tel.armed:
                        await self._safe_stop()
                    else:
                        self._sup_begin(tel, None, "ROLLING_DISARMED")
                    self.state = keep if keep in ("FAULT", "DANGER") else "STOPPING"
        tw = getattr(self, "traffic_watch", None)
        if (moving and tw is not None and tw.last_tick_sim is not None and now - tw.last_tick_sim > config.TRAFFIC_STALE_S
                and self.hold is None):
            # 차량 간 충돌 판단이 멈췄다 (판단 루프 예외·정지) — 감독 없이 달리지 않게 세운다
            self._event("TRAFFIC_SUPERVISION_STALE", age_s=round(now - tw.last_tick_sim, 1))
            await self.traffic_emergency_stop({"reason": "TRAFFIC_SUPERVISION_STALE"})
            return
        if moving:
            await self._supervise_drive(now, tel)
        await self._comm_watch(tel)
        if now - self._last_safety >= config.SAFETY_CHECK_PERIOD_S:
            self._last_safety = now
            src = self.fire_source
            st = src.data_status() if hasattr(src, "data_status") else "STATIC"
            if st != getattr(self, "_fire_data_seen", st):
                # 주행 중 화재 자료가 끊기면 마지막으로 받은 화재로 계속 판단하고 보고한다 (그 자리 정지가 더 위험할 수 있다)
                self._event("FIRE_DATA_STATUS", status=st, previous=self._fire_data_seen, moving=moving,
                            note="마지막 수신 화재로 안전거리 판단 계속" if st not in ("FRESH", "STATIC") else None)
            self._fire_data_seen = st
            await self._safety(now, tel)

    async def _supervise_drive(self, now, tel):
        sp = self.speed
        prog = self.driver.progress()
        # 진행: 계획 선형 위 가장 가까운 점 (지난 위치 근처만 본다)
        d, i = _progress_index(tel.x, tel.y, sp.pts, sp.cum, self._near_i)
        if d > config.OFF_ROUTE_M:
            d, i = _dist_to_polyline(tel.x, tel.y, sp.pts)
        self._near_i = i
        remaining = sp.cum[-1] - sp.cum[min(i + 1, len(sp.cum) - 1)] + math.dist((tel.x, tel.y), sp.pts[min(i + 1, len(sp.pts) - 1)])
        t = self.current_task()
        if t is not None:
            t["status"] = "IN_PROGRESS"
            eta_rem = sum((sp.cum[k] - sp.cum[k - 1]) / max((sp.v[k] + sp.v[k - 1]) / 2, 0.5)
                          for k in range(max(i + 1, 1), len(sp.v)))
            t["progress"] = {"phase": "EVADING" if self.state == "EVADING" else "ENROUTE",
                             "waypoint": prog.current, "total": prog.total, "remaining_m": round(remaining, 1),
                             "eta_remaining_sec": int(round(eta_rem)), "speed_mps": round(tel.speed_mps, 2),
                             "sim_time_s": round(now, 2), "off_route_m": round(d, 1)}
        # 도착
        dest = self.plan.dest
        to_dest = math.hypot(tel.x - dest.x, tel.y - dest.y)
        if self.hold is not None:
            # 차량 간 충돌 방지 대기 중: 대기 지점 정지는 도착이 아니다. 정지 고장 판단도 하지 않는다
            if t is not None:
                t["progress"]["phase"] = "YIELDING"
                t["progress"]["yield"] = {k: v for k, v in self.hold.items() if k != "s"}
            self._move_ref = (now, tel.x, tel.y)
            return
        # 도착 = 미션 끝 + 정지 확인 (STOPPING→IDLE 과 같은 기준: 0.2 m/s 미만·최신 텔레메트리). 예전 0.5 m/s 미만이면
        # 0.3~0.5 m/s 로 아직 움직이는데 IDLE·READY 가 됐다 (2026-10-10 Gazebo 전수 2832 표본 중 15)
        if prog.mission_seq == self.seq and prog.finished and self._standstill(tel):
            async with self.lock:
                if prog.mission_seq != self.seq:
                    return
                if to_dest <= config.ARRIVE_M * 3:
                    await self._arrived(tel, to_dest)
                else:
                    self._fail("ARRIVAL_MISMATCH", f"미션 끝, 목적지까지 {to_dest:.1f} m")
            return
        # 도로 경계 이탈 (계획 경로 이탈과 따로): 실제 차체가 도로 띠 밖으로 OFFROAD_STOP_M 넘게 OFFROAD_CONFIRM_S 이어지면 고장·정지.
        # 계획 경로 30 m·5 s 기준은 도로 밖 수십 m 를 늦게 잡았다 (2026-10-11 Gazebo 지속 운용: 경로 이탈 확정 전 33 s·260 m, 도로 밖 43.9 m)
        blk = getattr(self.driver, "pass_blocked_since", None)
        if blk is not None and time.monotonic() - blk >= PASS_STALL_S:
            async with self.lock:
                self._fail("MISSION_STALL", f"PX4 현재 미션 항목을 놓치고 멀어짐 — 건너뛸 안전 조건 없음 ({PASS_STALL_S:g} s)")
                await self._safe_stop()
                self.state = "FAULT"
            self.driver.pass_blocked_since = None
            return
        if tel.speed_mps >= 1.0:
            ex = self._body_outside(tel)
            if ex > OFFROAD_STOP_M:
                self._offroad_since = self._offroad_since if getattr(self, "_offroad_since", None) is not None else now
                if now - self._offroad_since >= OFFROAD_CONFIRM_S:
                    async with self.lock:
                        self._fail("OFF_ROAD", f"차체가 추정 도로 띠 밖 {ex:.1f} m ({OFFROAD_CONFIRM_S:g} s 넘게)")
                        await self._safe_stop()
                        self.state = "FAULT"
                    return
            else:
                self._offroad_since = None
        # 경로 이탈
        if d > config.OFF_ROUTE_M:
            self._off_since = self._off_since if self._off_since is not None else now
            if now - self._off_since >= config.OFF_ROUTE_CONFIRM_S:
                async with self.lock:
                    self._fail("OFF_ROUTE", f"계획 경로에서 {d:.1f} m")
                    await self._safe_stop()
                    self.state = "FAULT"
                return
        else:
            self._off_since = None
        # 정지 고장
        if self._move_ref is None or math.hypot(tel.x - self._move_ref[1], tel.y - self._move_ref[2]) >= config.STALL_MOVE_M:
            self._move_ref = (now, tel.x, tel.y)
        elif now - self._move_ref[0] >= config.STALL_TIMEOUT_S:
            async with self.lock:
                self._fail("STALLED", f"{config.STALL_TIMEOUT_S:g} s 동안 {config.STALL_MOVE_M:g} m 미만 이동")
                await self._safe_stop()
                self.state = "FAULT"
            return
        # 차량 이상 (PX4): 주행 중 시동 꺼짐
        # (출발 시각 0.0 을 '없음' 으로 보지 않는다 — 전에는 `or now` 라 시각 0 에 출발한 차는 이 판정이 꺼져 있었다)
        started = self._drive_started_sim if self._drive_started_sim is not None else now
        parked = prog.finished and getattr(self.driver, "parked_seq", None) == prog.mission_seq == self.seq
        if self.driver.kind == "px4" and not tel.armed and now - started > 5.0 and not parked:
            async with self.lock:
                self._fail("VEHICLE_FAULT", "주행 중 시동 꺼짐 (disarm)")
                # 다른 고장처럼 미션을 정지로 바꾼다. 안 바꾸면 시동이 다시 걸릴 때 PX4 가 남은 미션을 감독 없이 달린다
                # (2026-10-10 Gazebo: 오판된 VEHICLE_FAULT 뒤 FAULT 상태로 8 m/s 주행)
                await self._safe_stop()
                self.state = "FAULT"
            return
        if SPEED_TERRAIN_CMD and not prog.finished:
            await self._speed_cmd_tick(now, tel, i)

    async def _speed_cmd_tick(self, now, tel, i):
        """UGV_SPEED_TERRAIN_CMD (머리말). 목표 = min(sp.v[j] : s_now ≤ cum[j] ≤ s_now + v·τ + 여유). 잠금이 잡혀 있으면 이번 틱은 건너뛴다."""
        from . import terrain_speed as TS
        sp = self.speed
        if self.hold is not None or self._stop_sup is not None or self.state not in ("DRIVING", "EVADING") or self.lock.locked():
            return
        s0 = sp.cum[i]
        look = tel.speed_mps * TS.TAU_S + config.STOP_MARGIN_M
        ahead = [sp.v[j] for j in range(i, len(sp.v)) if sp.cum[j] <= s0 + look] or [sp.v[min(i, len(sp.v) - 1)]]
        target = round(max(min(ahead), 0.5), 2)
        last = getattr(self, "_spd_cmd", None)
        if last is not None and (abs(target - last["v"]) < SPEED_CMD_DELTA or now - last["t"] < SPEED_CMD_MIN_S):
            return
        seq = self.seq
        async with self.lock:
            if self.seq != seq or self.hold is not None or self._stop_sup is not None:
                return                               # 그 사이 재계획·양보·정지 — 지난 미션 값을 보내지 않는다
            try:
                ok = await asyncio.wait_for(self.driver.set_speed_cap(target), min(STOP_CMD_TIMEOUT_S, 2.0))
            except asyncio.TimeoutError:
                ok = None
        self._spd_cmd = {"v": target, "t": now, "seq": seq, "ok": ok}
        self._event("SPEED_CMD", sim_t=round(now, 2), s_m=round(s0, 1), lookahead_m=round(look, 1), target_mps=target,
                    actual_mps=round(tel.speed_mps, 2), mission_seq=seq, ack=ok,
                    prev_mps=None if last is None else last["v"], basis="UGV_SPEED_TERRAIN_CMD 계획 속도 v·τ 앞 최소")
        if not ok:
            self.last_error = f"SPEED_CMD 실패 (ack={ok}) — 감독 유지"

    # ------------------------------------------------------------------ 차량 간 충돌 방지 (ugv/traffic.py 가 부른다)
    def _s_now(self) -> float:
        return self.speed.cum[max(0, min(self._near_i, len(self.speed.cum) - 1))] if self.speed else 0.0

    def _fire_data_problem(self) -> Optional[str]:
        """새 화재 접근 임무에 쓸 화재 자료가 없으면 그 사유. 받지 못함·오래됨·미설정을 '화재 없음' 으로 보지 않는다
        (2026-10-11 검토: 첫 수신 실패면 빈 화재 영역으로 계획, 단절 중에도 나이 0 s). 복귀·통신 단절 복귀는 막지 않는다."""
        from .fire import USABLE
        src = self.fire_source
        st = src.data_status() if hasattr(src, "data_status") else "STATIC"
        if st in USABLE:
            return None
        info = src.status() if hasattr(src, "status") else {}
        return (f"{st}: 화재 자료 나이 {info.get('age_s')} s (허용 {config.FIRE_DATA_MAX_AGE_S:g} s), "
                f"오류 {info.get('error')}" if st != "NOT_CONFIGURED" else
                "NOT_CONFIGURED: 환경 서버(UGV_ENV_URL) 미설정 — 무화재 시험은 UGV_ALLOW_NO_FIRE_DATA=1 로 명시")

    def _road_stop_fits(self, pos, tel) -> bool:
        """진행 방향 도로 정지 경로를 쓸 수 있는가: 도로까지 STOP_ROAD_SNAP_MAX_M 안, 도로 방향과 진행 방향 차이 60° 안."""
        if pos is None or pos.dist > STOP_ROAD_SNAP_MAX_M:
            return False
        r = self.net.roads[pos.road_id]
        a = self.net.at(pos.road_id, max(0.0, pos.s - 2.0))
        b = self.net.at(pos.road_id, min(r.length, pos.s + 2.0))
        hd = bearing_deg(b.x - a.x, b.y - a.y)
        dh = abs((tel.heading_deg - hd + 180.0) % 360.0 - 180.0)
        return min(dh, 180.0 - dh) <= 60.0

    def _body_outside(self, tel) -> float:
        """실제 차체(차종 치수·진행 방향)가 추정 도로 띠 밖으로 나간 거리 (m, 안이면 ≤ 0). 텔레메트리 위치 기준(차체 중심·뒤축)을 맞춘다."""
        yaw = math.radians(90.0 - tel.heading_deg)
        x, y = tel.x, tel.y
        if getattr(self.driver, "position_ref", "rear_axle") == "center":
            x -= self.profile.rear_axle_from_center_m * math.cos(yaw)
            y -= self.profile.rear_axle_from_center_m * math.sin(yaw)
        return self.graph.body_excess(x, y, yaw)

    # ------------------------------------------------------------------ 정지 감독
    @property
    def stop_cut_active(self) -> bool:
        """정지 감독이 구동 차단으로 넘어갔고 아직 정차가 확인되지 않음 (ugv/traffic.py 는 이때 점유를 넓게 본다)."""
        return self._stop_sup is not None and (self._stop_sup["escalations"] > 0 or bool(self._stop_sup.get("coast_hold_until")))

    def _disarm_coast_until(self, tel) -> Optional[float]:
        """시동이 꺼진 차의 정차 확인을 미룰 시각 (없으면 None). PX4 로버는 시동이 꺼지면 위치·속도 추정을 정지로 보고한다 —
        2026-10-11 Gazebo c23: B-ugv1 시동 꺼진 뒤 실제 13.5 m/s 로 약 75 m 관성 주행하는 동안 텔레메트리는 0.2 m 이동·속도 0.
        시동 걸린 마지막 속도 v 로 관성 감속 COAST_DECEL_MPS2 를 가정해 v / a + COAST_PAD_S 동안은 텔레메트리 정차를 믿지 않는다."""
        if tel.armed or self._last_armed is None or self._last_armed[1] < COAST_MIN_MPS:
            return None
        t, v = self._last_armed
        return t + v / COAST_DECEL_MPS2 + COAST_PAD_S

    def _sup_begin(self, tel, path_len, basis):
        """정지 감독 시작 (정지 명령을 보낸 직후). 이미 감독 중이면(정지 재시도·운영자 /stop 반복) 처음 시각·허용 거리를 그대로 둔다 —
        다시 시작하면 재시도마다 기준이 늦춰져 끝없이 미뤄진다. 새 주행 미션(_start)에서만 지운다."""
        if self._stop_sup is not None:
            return
        now = self.clock.now()
        v0 = max(tel.speed_mps, 0.0) if tel is not None else 0.0
        lat = self.latency_s()
        reach = max(path_len or 0.0, self.profile.stop_distance(v0) if v0 >= 0.3 else 0.0)
        allow = reach + v0 * (lat + STOP_SUP_LAT_PAD_S) + STOP_SUP_MARGIN_M
        t_allow = (2.0 * reach / v0 if v0 >= 0.3 else 0.0) + lat + STOP_SUP_SLACK_S
        self._stop_sup = {"basis": basis, "t0": now, "x": tel.x if tel else None, "y": tel.y if tel else None,
                          "v0": round(v0, 2), "vmax": v0, "travel_m": 0.0, "allow_m": round(allow, 1),
                          "deadline": now + t_allow, "escalations": 0, "next_cut": now, "still_since": None,
                          "cut_ok": None}

    async def _cut_drive(self, why) -> bool:
        s = self._stop_sup
        try:
            await asyncio.wait_for(self.driver.cut_drive(), STOP_CMD_TIMEOUT_S)
            ok, detail = True, None
        except (DriverError, asyncio.TimeoutError, NotImplementedError) as e:
            ok, detail = False, f"{type(e).__name__}: {e}"
        if s is not None:
            s["cut_ok"] = ok
        self._event("STOP_CUT_DRIVE", why=why, ok=ok, detail=detail)
        return ok

    async def _sup_check(self, now, tel):
        s = self._stop_sup
        if tel.age_s() > config.TELEMETRY_STALE_S:
            return                                   # 확인할 수 없다 — 감독·점유 유지 (텔레메트리 끊김 감시는 따로)
        if s["x"] is not None:
            s["travel_m"] += math.dist((tel.x, tel.y), (s["x"], s["y"]))
        s["x"], s["y"] = tel.x, tel.y
        s["vmax"] = max(s["vmax"], tel.speed_mps)
        if tel.speed_mps < 0.2:
            s["still_since"] = s["still_since"] if s["still_since"] is not None else now
            coast = self._disarm_coast_until(tel)
            if coast is not None and now < coast:
                if not s.get("coast_hold_until"):
                    v = self._last_armed[1]
                    s["coast_hold_until"] = round(coast, 1)
                    s["coast_est_m"] = round(v * v / (2.0 * COAST_DECEL_MPS2), 1)
                    self._event("STOP_COAST_HOLD", v_armed_mps=round(v, 2), until_s=round(coast, 1), coast_est_m=s["coast_est_m"],
                                why="시동 꺼짐 — 텔레메트리 정차를 믿지 않고 관성 주행 추정 시간까지 정차 확인 보류")
                return
            if now - s["still_since"] >= STOP_CONFIRM_S:
                self.last_stop_result = {"basis": s["basis"], "v0_mps": s["v0"], "travel_m": round(s["travel_m"], 1),
                                         "allow_m": s["allow_m"], "time_s": round(now - s["t0"], 1),
                                         "cut_drive": s["escalations"] > 0, "cut_attempts": s["escalations"],
                                         # 허용 거리를 넘어 섰다 (예: 링크 단절 중 PX4 자체 정지 — 그동안 감독은 거리를 볼 수 없다)
                                         "exceeded_allow": s["travel_m"] > s["allow_m"]}
                if s.get("coast_hold_until"):
                    # 시동 꺼진 뒤 정차 — 실제 이동은 관측할 수 없었다 (추정 관성 거리만)
                    self.last_stop_result.update({"basis_note": "DISARMED_COAST_ASSUMED", "coast_est_m": s["coast_est_m"],
                                                  "exceeded_allow": None})
                self._stop_sup = None
                self._event("STOP_CONFIRMED", **self.last_stop_result)
            return
        s["still_since"] = None
        if tel.speed_mps <= s["v0"] + STOP_SUP_ACCEL_MPS:
            s.pop("fast_since", None)
        why = None
        if s["travel_m"] > s["allow_m"]:
            why = f"DISTANCE {s['travel_m']:.1f} m > {s['allow_m']} m"
        elif now > s["deadline"]:
            why = f"TIME {now - s['t0']:.1f} s"
        elif (s["basis"] != "BLOCKED_HOLD" and now - s["t0"] > self.latency_s() + 1.0
              and tel.speed_mps > s["v0"] + STOP_SUP_ACCEL_MPS
              and now - s.setdefault("fast_since", now) >= STOP_SUP_ACCEL_HOLD_S):
            # 대기 지점 정지(BLOCKED_HOLD)는 대기 지점까지 앞으로 가는 미션이라 저속에서 다시 가속할 수 있다 — 거리·시간 상한만
            why = f"NOT_DECELERATING {tel.speed_mps:.1f} m/s > {s['v0']} m/s"
        if why is None or now < s["next_cut"]:
            return
        async with self.lock:
            first = s["escalations"] == 0
            s["escalations"] += 1
            s["next_cut"] = now + STOP_RETRY_S
            self.stop_path_pts = None                # 정지 경로를 벗어났다 — 점유는 넓게 (traffic)
            self._event("STOP_NOT_CONFIRMED", why=why, speed_mps=round(tel.speed_mps, 2),
                        travel_m=round(s["travel_m"], 1), allow_m=s["allow_m"], attempt=s["escalations"])
            await self._cut_drive(why)
            # 허용 거리 안에서 저속(STOP_SLOW_FINISH_MPS 미만 — 정지 미션 끝 구간은 항목 최소 속도 1 m/s 로 다가간다)으로 마무리하는데
            # 시간만 넘은 경우(2026-10-11 Gazebo UGV 취소 13.7~13.9 s, 그때 0.9 m/s)는 구동 차단(정차 유지)만 하고 고장으로 묶지 않는다.
            # 거리 초과·감속 안 함·고속 시간 초과는 고장
            slow_finish = why.startswith("TIME") and tel.speed_mps < STOP_SLOW_FINISH_MPS and s["travel_m"] <= s["allow_m"]
            if slow_finish:
                self._event("STOP_SLOW_FINISH_CUT", speed_mps=round(tel.speed_mps, 2), travel_m=round(s["travel_m"], 1))
            elif first and self.state not in ("FAULT", "DANGER"):
                self._fail("STOP_NOT_CONFIRMED", f"정지 명령 뒤 {why} (속도 {tel.speed_mps:.1f} m/s) → 구동 차단")

    def _standstill(self, tel) -> bool:
        """정지 확인: 최신 텔레메트리·0.2 m/s 미만. PX4 는 드라이버의 정차 유지(미션 끝 뒤 시동 끔)까지 — 이 rover 는 미션 끝
        대기(loiter)에서 0.2~0.3 m/s 로 다시 굴러가곤 했다 (2026-10-10 Gazebo: 도착 READY 뒤 0.27~0.31 m/s). 시동을 끄지 못하면
        0.2 m/s 미만이 STANDSTILL_ARMED_S 이어질 때."""
        if tel is None or tel.age_s() > config.TELEMETRY_STALE_S or tel.speed_mps >= 0.2:
            self._still_since = None
            return False
        if self.driver.kind != "px4" or not tel.armed:
            return True
        now = self.clock.now()                  # 시뮬레이션 시계 (정지 고장 STALL_TIMEOUT_S 와 같은 시계)
        if getattr(self, "_still_since", None) is None:
            self._still_since = now
        return now - self._still_since >= STANDSTILL_ARMED_S

    async def _send_stop(self, coro):
        """정지·양보 명령 (드라이버 follow/stop). 시간 안에 끝나지 않으면 DriverError — 응답 유실·성공 여부 불명을 실패로 다룬다
        (명령 불확실 → 보수적 점유·재시도). 예외가 아니어도 끝나지 않는 명령이 이벤트 루프·차량 잠금을 붙잡지 않게 한다."""
        try:
            return await asyncio.wait_for(coro, STOP_CMD_TIMEOUT_S)
        except asyncio.TimeoutError:
            raise DriverError(f"STOP_COMMAND_TIMEOUT: {STOP_CMD_TIMEOUT_S:g} s 안에 응답 없음 (전달 여부 불명)")

    def _path_between(self, s_to: Optional[float]):
        """지금 위치 → 계획 선형 위 s_to(없으면 목적지)까지 선형 (점유 예측용)."""
        sp = self.speed
        tel = self.driver.telemetry()
        i0 = max(0, min(self._near_i, len(sp.pts) - 1))
        out = [(tel.x, tel.y)] if tel is not None else []
        for k in range(i0 + 1, len(sp.pts)):
            if s_to is not None and sp.cum[k] > s_to:
                a, b = sp.pts[k - 1], sp.pts[k]
                seg = sp.cum[k] - sp.cum[k - 1]
                f = 0.0 if seg <= 0 else (s_to - sp.cum[k - 1]) / seg
                out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
                break
            out.append(tuple(sp.pts[k]))
        return out

    def _items_between(self, s_to: Optional[float]):
        """지금 위치 → 계획 선형 위 s_to(없으면 목적지)까지 미션 지점. 검증된 경로의 일부만 쓴다 (형상 그대로)."""
        sp = self.speed
        tel = self.driver.telemetry()
        i0 = max(0, min(self._near_i, len(sp.pts) - 1))
        pts, caps = [(tel.x, tel.y)], [sp.v[i0]]
        for k in range(i0 + 1, len(sp.pts)):
            if s_to is not None and sp.cum[k] >= s_to:
                a, b = sp.pts[k - 1], sp.pts[k]
                seg = sp.cum[k] - sp.cum[k - 1]
                f = 0.0 if seg <= 0 else (s_to - sp.cum[k - 1]) / seg
                pts.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
                caps.append(sp.v[k])
                break
            pts.append(sp.pts[k])
            caps.append(max(sp.v[k], 1.0) if k < len(sp.pts) - 1 else 1.0)
        if len(pts) < 2:
            pts.append(pts[0])
            caps.append(1.0)
        r = Route(pts, [max(c, 1.0) for c in caps], [], self.plan.dest, self.plan.dest, 0.0)
        r.item_comp_l = getattr(self.plan.route, "item_comp_l", None)     # 원래 경로 검사에서 보정을 껐으면 정지 경로도 끈다
        r.item_chord_tol = getattr(self.plan.route, "item_chord_tol", None)  # 촘촘한 미션 지점으로 통과한 경로면 정지 경로도
        return plan_speeds(r, tel.speed_mps, profile=self.profile).items

    async def set_hold(self, s_hold: float, info: dict):
        """s_hold(계획 선형 위치)에서 멈춰 기다린다. 같은 대기 지점이면 다시 보내지 않는다 (명령 교체 지연이 있으므로)."""
        if self.state not in ("DRIVING", "EVADING") or self.speed is None:
            return
        cur = self.hold
        # 더 앞(가까이)에서 서야 하면 바로 다시 보낸다 (0.3 m 넘게). 더 멀리 가도 되는 변화는 8 m 넘을 때만 (명령 교체 지연이 있으므로).
        # 예전에는 1 m 가까워지는 것까지 무시해, 움직이던 앞차 기준(FOLLOW) 대기 지점에서 앞차가 선 뒤(STATIC) 더 앞 지점으로
        # 바꾸지 않고 그대로 가서 간격이 2.62 m 였다 (2026-10-10 Gazebo pair_block)
        if cur is not None and -0.3 <= s_hold - cur["s"] < 8.0:
            self.hold = {**cur, **info}
            return
        async with self.lock:
            if self.state not in ("DRIVING", "EVADING"):
                return
            items = self._items_between(s_hold)
            try:
                seq = await self._send_stop(self.driver.follow(items))
            except DriverError as e:
                # 대기를 확정하지 않는다 (예전에는 먼저 hold 를 기록해, 실패한 뒤 같은 대기 요청을 '이미 명령함' 으로 보고 다시 보내지
                # 않았다 — 차는 원래 미션을 계속 달린다). 다음 판단 주기에 다시 보내고, 연속 HOLD_FAIL_MAX 번이면 고장으로 처리
                self._hold_failures += 1
                self._event("YIELD_FAILED", detail=str(e), failures=self._hold_failures)
                if self._hold_failures >= HOLD_FAIL_MAX:
                    self._fail("STOP_COMMAND_FAILED", f"양보 정지 명령 {self._hold_failures}회 실패: {e}")
                    await self._safe_stop()
                    self.state = "FAULT"
                return
            self.seq, self._hold_failures = seq, 0
            self.hold = {"s": s_hold, **info, "since_sim": round(self.clock.now(), 2)}
            self._event("YIELD", s_m=round(s_hold, 1), ahead_m=round(s_hold - self._s_now(), 1), **info)

    async def clear_hold(self):
        if self.hold is None:
            return
        async with self.lock:
            if self.hold is None or self.state not in ("DRIVING", "EVADING"):
                self.hold = None
                return
            info = self.hold
            items = self._items_between(None)
            self.hold = None
            try:
                # 재출발도 시간 제한 (_send_stop 과 같은 STOP_CMD_TIMEOUT_S). 실패·시간 초과면 미션이 일부 적용됐을 수 있다 →
                # 명령 불확실(보수적 점유·재시도) + 고장 + 정지 (2026-10-11 검토: 재출발 follow 에 시간 제한이 없었다)
                self.seq = await self._send_stop(self.driver.follow(items))
            except DriverError as e:
                self._fail("RESUME_FAILED", str(e))
                self.command_uncertain = True
                await self._safe_stop()
                self.state = "FAULT"
                return
            # 대기 지점에 서면 PX4 는 미션 끝 정차로 시동을 끈다 → 다시 출발할 때 시동 상태가 텔레메트리에 늦게 온다.
            # 출발 시각을 지금으로 해 '주행 중 시동 꺼짐' 판정이 재출발 직후를 오판하지 않게 한다 (2026-10-10 Gazebo 마주 접근 시험)
            self._drive_started_sim = self.clock.now()
            self._move_ref = None
            self._event("YIELD_END", waited_s=round(self.clock.now() - info.get("since_sim", self.clock.now()), 1),
                        other=info.get("other"))

    async def blocked_by_vehicle(self, other_id: str, waited_s: float, immovable: bool):
        """서 있는 차에 막혀 오래 기다렸다 (ugv/traffic.py). 같은 목적지로 그 차가 선 도로를 빼고 다시 계획한다.
        통과 가능한 우회 경로(진행 방향·차체·회전 검사 통과)가 있으면 그 길로 가고, 없으면 계속 기다리지 않고 임무를 실패
        (BLOCKED_BY_VEHICLE)로 끝낸다 — 차는 대기 지점에 선 채(이미 보낸 정지 미션) 총괄의 재판단을 기다린다.
        같은 임무에서 같은 차 때문에 다시 계획하는 것은 한 번, 막힘 재계획은 모두 BLOCK_REPLAN_MAX 번까지
        (되풀이 재계획·두 차가 번갈아 길을 바꾸는 것을 막는다)."""
        async with self.lock:
            t = self.current_task()
            if t is None or self.state not in ("DRIVING", "EVADING") or self.hold is None:
                return
            other = next((p for p in self.peers() if p.resource_id == other_id), None)
            tel_o = other.driver.telemetry() if other is not None else None
            q_o = self.net.snap(tel_o.x, tel_o.y) if tel_o is not None else None
            blocked_road = q_o.road_id if q_o is not None else None
            hist = t.setdefault("blocked", [])
            rec = {"sim_time_s": round(self.clock.now(), 2), "other": other_id, "waited_s": waited_s,
                   "blocker_immovable": immovable, "blocked_road": blocked_road}
            hist.append(rec)
            repeated = sum(1 for h in hist if h["other"] == other_id) > 1 or len(hist) > BLOCK_REPLAN_MAX
            plan, why = None, None
            if repeated:
                why = "REPEATED_BLOCK (같은 차에 다시 막힘 또는 막힘 재계획 횟수 초과)"
            else:
                start, v0, err = self.here_state()
                body = {"target": t["target"]} if t.get("target") else {"target_node": t.get("target_node")}
                avoid = {blocked_road: other_id} if blocked_road else {}
                if BLOCK_AVOID_MODE == "footprint" and other is not None and tel_o is not None:
                    # 비교 시험용 선택 (기본 off): 상대 차체가 걸친 도로 전부 (원점 snap 하나만이 아니라) — 2026-10-13 간헐 실패 진단:
                    # 교차로에 선 고장 차는 도로 3개에 걸쳤고, 원점 snap 도로 하나만 피한 우회가 같은 교차로로 들어가 다시 막혔다
                    for rid_fp in self._footprint_roads(other, tel_o):
                        avoid.setdefault(rid_fp, other_id)
                    rec["avoid_roads"] = sorted(avoid)
                try:
                    plan = None if start is None else await asyncio.to_thread(self._approach, body, start, v0, avoid)
                except HttpError as e:
                    plan, why = None, f"PLAN_ERROR: {e}"
                if plan is not None and plan.status == "OK" and blocked_road in plan.route.road_ids:
                    plan, why = None, "DETOUR_STILL_USES_BLOCKED_ROAD"
                elif plan is not None and plan.status != "OK":
                    plan, why = None, f"{plan.status}: {plan.reason}"
                elif plan is None:
                    why = why or err
            if plan is not None:
                rec["result"] = "REPLANNED"
                try:
                    await self._drive(plan)
                except DriverError as e:
                    await self._safe_stop()
                    self._finish(self.task_key, "FAILED", error=f"DRIVER: {e}", physical_state="STOPPING")
                    return
                tn = self._target_node(plan)
                t["target_node"] = tn["node_id"] if tn else t.get("target_node")
                t["approach"] = plan.report()
                t.setdefault("replans", []).append({"sim_time_s": rec["sim_time_s"], "why": f"BLOCKED_BY_VEHICLE:{other_id}",
                                                    "result": "OK", "approach": plan.report()})
                self._event("REPLANNED_AROUND_VEHICLE", other=other_id, blocked_road=blocked_road, waited_s=waited_s,
                            route_length_m=round(self.speed.cum[-1], 1), eta_sec=int(round(self.speed.eta_s)))
                self.store.save()
                return
            rec["result"] = "FAILED_NO_DETOUR"
            rec["why"] = why
            # 대기 지점 정지 미션은 그대로 둔다 (새 명령을 보내지 않는다 — 그 앞에서 선다). 임무 실패와 차량 정차를 나눈다:
            # 신선한 텔레메트리로 정차를 확인할 때까지 STOPPING (READY 아님 → 총괄이 반납·재배정하지 않음), 확인되면 STOPPED_BLOCKED
            # (예전에는 바로 IDLE·STOPPED_BLOCKED → READY. Gazebo pair_block: 보고 때 2.4 m/s, 그 뒤 40.7 m 더 감)
            self.stop_path_pts = self._path_between(self.hold["s"])
            self._sup_begin(self.driver.telemetry(), max(self.hold["s"] - self._s_now(), 0.0), "BLOCKED_HOLD")
            self.hold = None
            self.state = "STOPPING"
            self._finish(self.task_key, "FAILED",
                         error=f"BLOCKED_BY_VEHICLE: {other_id} 가 선 도로 {blocked_road} 에 막힘, 우회 경로 없음 "
                               f"({why}), {waited_s:.0f} s 대기",
                         physical_state="STOPPING_BLOCKED")
            self._event("TRAFFIC_BLOCKED_NO_DETOUR", other=other_id, blocked_road=blocked_road, waited_s=waited_s,
                        why=why, reported_to="ORCHESTRATOR_TASK_FAILED")

    async def traffic_emergency_stop(self, info: dict):
        """차량 간 충돌을 피할 수 없다고 판단될 때: 계획 경로를 따라 최대 감속으로 가장 짧게 선다.
        평소 정지(_safe_stop, 계획 감속)처럼 길게 가면 상대 쪽으로 더 다가간다 (2026-10-10 Gazebo 4대 시험: 마주 선 두 차가
        각자 경로를 따라 정지하며 13 m → 접촉). 이미 대기 지점이 더 가까우면 그 앞에서 선다."""
        async with self.lock:
            self.gen += 1
            tel = self.driver.telemetry()
            if self.speed is not None and tel is not None and self.state in ("DRIVING", "EVADING") and tel.speed_mps >= 0.3:
                v = tel.speed_mps
                d = v * v / (2 * self.profile.emergency_decel_mps2) + 1.0
                s_stop = self._s_now() + d
                if self.hold is not None:
                    s_stop = min(s_stop, max(self.hold["s"], self._s_now()))
                path = self._path_between(s_stop)
                try:
                    self.seq = await self._send_stop(self.driver.follow(self._items_between(s_stop)))
                    self._event("STOP_PATH", basis="TRAFFIC_EMERGENCY", length_m=round(s_stop - self._s_now(), 1),
                                speed_mps=round(v, 2), decel_mps2=self.profile.emergency_decel_mps2)
                    self.stop_path_pts, self.command_uncertain = path, False
                    self.hold = None
                    self.state = "STOPPING"
                    self._sup_begin(tel, s_stop - self._s_now(), "TRAFFIC_EMERGENCY")
                except DriverError as e:
                    # 정지 명령이 안 갔다 → 원래 미션이 계속될 수 있다. 정지 중으로 보지 않고 고장으로 — 감시·재시도·보수적 점유 유지
                    # (예전에는 이벤트만 남기고 STOPPING 으로 바꿔 주행 감독에서 빠졌다 — 2026-10-10 검토, 실패 주입 10 m/s)
                    self.command_uncertain, self.stop_path_pts = True, None
                    self._stop_retry_at = self.clock.now()
                    self._event("STOP_FAILED", detail=str(e), basis="TRAFFIC_EMERGENCY")
                    self._sup_begin(tel, None, "STOP_COMMAND_FAILED")
                    self.hold = None
                    self._fail("STOP_COMMAND_FAILED", f"비상 정지 명령 실패: {e} (차량 간 충돌 회피 불가 {info})")
            elif self.state in ("DRIVING", "EVADING"):
                self.hold = None
                self.state = "STOPPING"                 # 이미 멈춰 있음 — 더 보내지 않는다
            if self.current_task():
                self._finish(self.task_key, "FAILED", error=f"TRAFFIC_CONFLICT_UNAVOIDABLE: {info}",
                             physical_state="STOPPING")

    # ------------------------------------------------------------------ 총괄 통신 단절 (벽시계)
    def note_contact(self):
        """총괄 연락을 받았다 (서버가 명령·조회·heartbeat 마다 부른다). 단절 상태를 끝낸다 — 복귀 주행 중이면 그대로 둔다."""
        self._last_contact = self.wall()
        if self.comm["phase"] != "OK":
            self._event("COMM_RESTORED", previous=dict(self.comm))
            self.comm = {"phase": "OK"}

    def comm_lost_s(self) -> Optional[float]:
        return None if self._last_contact is None else self.wall() - self._last_contact

    async def _comm_watch(self, tel):
        lost = self.comm_lost_s()
        if lost is None:
            return
        ph = self.comm["phase"]
        if ph == "OK" and lost > config.COMM_LOSS_STOP_S:
            async with self.lock:
                self.comm = {"phase": "STOPPING", "lost_s_at_stop": round(lost, 1)}
                if self.current_task():
                    self.gen += 1
                    self._finish(self.task_key, "CANCELLED", error=f"COMM_LOSS: 총괄 연락 {lost:.0f} s 없음",
                                 physical_state="STOPPING")
                if self.state in ("DRIVING", "EVADING"):    # 이미 정차·대기 중인 차에는 정지 명령을 보내지 않는다
                    self.gen += 1
                    await self._safe_stop()
                self._event("COMM_LOSS_STOP", lost_s=round(lost, 1), threshold_s=config.COMM_LOSS_STOP_S)
            return
        if ph == "STOPPING" and tel.speed_mps < 0.2 and self.state not in ("DRIVING", "EVADING", "STOPPING"):
            self.comm = {**self.comm, "phase": "STOPPED", "stopped_wall": self.wall()}
            self._event("COMM_LOSS_STOPPED", lost_s=round(lost, 1))
            return
        if ph == "STOPPED" and self.wall() - self.comm["stopped_wall"] > config.COMM_LOSS_RETURN_S:
            if self.state in ("FAULT", "DANGER"):
                return                                   # 운영자 해제 전에는 자동으로 움직이지 않는다
            async with self.lock:
                await self._comm_return(lost)

    async def _comm_return(self, lost):
        here, v0, err = self.here_state()
        acc = self.access
        tel = self.driver.telemetry()
        if tel is not None and math.hypot(tel.x - acc.x, tel.y - acc.y) <= config.ARRIVE_M:
            self.comm = {**self.comm, "phase": "RETURNED"}
            self._event("COMM_LOSS_AT_STATION", lost_s=round(lost, 1))
            return
        plan = None if here is None else await asyncio.to_thread(
            plan_approach, self.net, self.fire_source.area(), here, *acc.ll(), v0=v0, access=acc, fixed_dest=acc,
            graph=self.graph, arrive_toward=self.access_toward)
        if plan is None or plan.status != "OK":
            why = err or f"{plan.status}: {plan.reason}"
            self.fault = f"COMM_LOSS_NO_RETURN_ROUTE: 총괄 연락 {lost:.0f} s 없음, 소속 거점 {self.station['base_id']} 복귀 경로 없음 ({why})"
            self.state = "FAULT"
            self.comm = {**self.comm, "phase": "FAILED", "reason": why}
            self._event("COMM_LOSS_RETURN_FAILED", reason=why, operator_action_required=True)
            return
        try:
            self.gen += 1
            self.task_key = None
            await self._drive(plan)
        except DriverError as e:
            self._fail("COMM_LOSS_RETURN_START_FAILED", str(e))
            self.comm = {**self.comm, "phase": "FAILED", "reason": str(e)}
            return
        self.comm = {**self.comm, "phase": "RETURNING"}
        self._event("COMM_LOSS_RETURN", lost_s=round(lost, 1), approach=plan.report())

    async def _arrived(self, tel, to_dest):
        evading = self.state == "EVADING"
        self.state = "IDLE"
        obs = {"observation_type": "ROAD_STATUS", "arrived_node": node_id_of(self.plan.dest),
               "position": {"lat": tel.lat, "lon": tel.lon}, "fuel_pct": None,
               "note": "도로 지점 도착 기록. 화재 관측이 아니다 (화재 관측은 4단계 관측 보고)"}
        t = self.current_task()
        if self.comm["phase"] == "RETURNING":
            self.comm = {**self.comm, "phase": "RETURNED"}
        self._event("ARRIVED" if not evading else "EVADED", to_dest_m=round(to_dest, 1),
                    position={"lat": tel.lat, "lon": tel.lon})
        if t is None:
            return
        if evading:
            # 이탈을 마쳤다 → 원래 목표 접근을 다시 계획한다
            await self._replan_task(t, "AFTER_EVASION")
            return
        t["observation"] = obs
        t["progress"] = {**(t.get("progress") or {}), "phase": "ARRIVED", "remaining_m": round(to_dest, 1),
                         "eta_remaining_sec": 0}
        self._finish(self.task_key, "COMPLETED", physical_state="STOPPED_ON_STATION", mission_result="ARRIVED")

    async def _replan_task(self, t, why):
        here, v0, err = self.here_state()
        plan = None if here is None else await asyncio.to_thread(
            plan_approach, self.net, self.fire_source.area(), here, float(self.target_ll["lat"]),
            float(self.target_ll["lon"]), v0=v0, access=self.access, graph=self.graph)
        rec = {"sim_time_s": round(self.clock.now(), 2), "why": why, "result": plan.status if plan else "NO_START_STATE",
               "approach": plan.report() if plan else {"reason": err}}
        t.setdefault("replans", []).append(rec)
        if plan is None or plan.status != "OK":
            await self._safe_stop()
            self._finish(self.task_key, "FAILED", error=f"NO_SAFE_APPROACH: {plan.reason if plan else err}",
                         physical_state="STOPPING")
            return
        try:
            await self._drive(plan)
        except DriverError as e:
            await self._safe_stop()
            self._finish(self.task_key, "FAILED", error=f"DRIVER: {e}", physical_state="STOPPING")
            return
        tn = self._target_node(plan)
        t["target_node"] = tn["node_id"] if tn else None
        t["approach"] = plan.report()
        self._event("REPLANNED", why=why, approach=plan.report())
        self.store.save()

    async def _safety(self, now, tel):
        fire = self.fire_source.area()
        if not len(fire):
            self.last_clearance_m = None
            return
        here_c = fire.distance(tel.x, tel.y, max_m=2000.0)        # 2 km 넘으면 inf (칸이 많을 때 매 초 전체 탐색 방지)
        self.last_clearance_m = here_c
        if self.state in ("FAULT", "DANGER", "EVADING"):
            return
        async with self.lock:
            if here_c < config.FIRE_STANDOFF_M:
                here, v0, err = self.here_state()
                ev = await asyncio.to_thread(plan_evasion, self.net, fire, here, v0=v0, access=self.access,
                                             graph=self.graph) if here is not None else None
                if ev is None or ev.status != "OK":
                    # 자동 이탈 불가 — 안전 확보가 아니다. 도로를 따라 제동하고 운영자 개입을 요청한다 (자동 재출발 없음)
                    await self._enter_danger(here_c, ev, err)
                    return
                self.gen += 1
                try:
                    await self._drive(ev, state="EVADING")
                except DriverError as e:
                    await self._enter_danger(here_c, ev, f"EVASION_START_FAILED: {e}")
                    return
                self._event("EVADING", clearance_m=round(here_c, 1), evasion=ev.report())
                return
            if self.state == "DRIVING" and self.speed is not None:
                rest = self.speed.pts[self._near_i:]
                c = fire.clearance_along(rest[::4] + rest[-1:], max_m=config.FIRE_STANDOFF_M + 1.0)
                if c < config.FIRE_STANDOFF_M and self.current_task():
                    self.gen += 1
                    await self._replan_task(self.current_task(), f"ROUTE_CLEARANCE_{c:.0f}M")

    async def _enter_danger(self, here_c, ev, err=None):
        """DANGER: 실행 가능한 이탈 경로 없음 → 도로를 따라 가능한 범위에서 감속 정지 + 운영자 개입 요청.
        정지가 탈출·안전 확보를 뜻하지 않는다. 정지 경로의 길이 부족·주행 불가도 함께 보고한다."""
        self.gen += 1
        stop = await self._safe_stop()
        why = (ev.reason if ev is not None and ev.reason else None) or err or "UNKNOWN"
        self.fault = f"NO_SAFE_EVASION_ROUTE: 화재 경계 {here_c:.0f} m (안전거리 {config.FIRE_STANDOFF_M:g} m 안), {why}"
        if stop.get("problems"):
            self.fault += " / 정지: " + "; ".join(stop["problems"])
        if self.current_task():
            self._finish(self.task_key, "FAILED", error=self.fault, physical_state="DANGER")
        self.state = "DANGER"
        self.danger = {"clearance_m": round(here_c, 1), "evasion": ev.report() if ev else {"reason": err},
                       "stop_path": stop, "meaning": DANGER_MEANING, "operator_action_required": True,
                       "failure_reasons": [why] + stop.get("problems", [])}
        self._event("DANGER", **self.danger)

    # ------------------------------------------------------------------ 상태
    def state_view(self) -> dict:
        tel = self.driver.telemetry()
        t = self.current_task()
        pos = {"lat": tel.lat, "lon": tel.lon} if tel else {"lat": None, "lon": None}
        here = self.net.snap(tel.x, tel.y) if tel else None
        stopped = tel is not None and tel.speed_mps < 0.2 and self.state == "IDLE"
        return {
            "resource_id": self.resource_id, "resource_type": self.resource_type, "base": self.station["base_id"],
            "state": self.resource_state(), "position": pos, "fuel_pct": None,
            "current_node": node_id_of(here) if (stopped and here) else None,
            "current_task_id": t["task_id"] if t else None, "driver": self.driver.kind,
            "driver_status": tel.mode if tel else "NO_TELEMETRY",
            "updated_at": (time.time() - tel.age_s()) if tel and math.isfinite(tel.age_s()) else 0.0, "fault": self.fault,
            "sim_time_s": round(self.clock.now(), 2), "mission_state": self.state,
            "speed_mps": round(tel.speed_mps, 2) if tel else None, "heading_deg": round(tel.heading_deg, 1) if tel else None,
            "station": self.station, "profile": self.profile.summary(),
            "comm": {**{k: v for k, v in self.comm.items() if k != "stopped_wall"},
                     "orchestrator_silence_s": None if self.comm_lost_s() is None else round(self.comm_lost_s(), 1),
                     "stop_after_s": config.COMM_LOSS_STOP_S, "return_after_stop_s": config.COMM_LOSS_RETURN_S,
                     "basis": "벽시계(단조). 총괄 연락을 한 번 받은 뒤부터 센다"},
            "operator_intervention": {
                "required": self.state in ("FAULT", "DANGER") or self.comm["phase"] == "FAILED", "reason": self.fault,
                "meaning": DANGER_MEANING if self.state == "DANGER" else None,
                "danger": self.danger if self.state == "DANGER" else None,
                "release": "POST /ugv/{id}/stop — 차량 정지(0.2 m/s 미만)·텔레메트리 확인 뒤에만 해제. 원격 운전 기능 없음"},
            "fire_data": (self.fire_source.status() if hasattr(self.fire_source, "status") else None),
            "safety": {"fire_clearance_m": None if self.last_clearance_m is None or math.isinf(self.last_clearance_m)
                       else round(self.last_clearance_m, 1), "standoff_m": config.FIRE_STANDOFF_M,
                       **(self.fire_source.area().summary())},
        }
