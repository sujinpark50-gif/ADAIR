# -*- coding: utf-8 -*-
"""ugv/drivers/px4.py — PX4 SITL(adair_ugv, Gazebo) 드라이버. mavsdk 4.x (네이티브 바인딩, UAV 와 같은 라이브러리).

주행 = PX4 Mission 모드. 미션 지점마다 구간 속도(speed_m_s)를 싣는다. 경로 추종(pure pursuit)·꺾임 감속은 PX4 가 한다.
- 속도 의미 변환 (mavsdk_plan): 공통 Item 속도는 '그 지점까지 가는 구간' 속도인데, MAVSDK/PX4 는 지점의 speed_m_s 를
  '그 지점을 지난 뒤' 쓴다 (2026-10-09 평탄 도로 월드 실측: 구간 k-1→k 실제 속도가 지점 k-1 속도와 평균 0.06 m/s 차,
  지점 k 속도와 2.9 m/s 차). 그대로 실으면 감속이 한 구간(최대 40 m) 늦어 곡선에 빠르게 들어간다
  (계획 8.7 m/s 곡선을 11.3 m/s, 횡가속 4.75 m/s²) → 한 칸 당겨 싣고, 첫 구간 속도를 정하려고 앞쪽에 선행 지점을 하나 넣는다.
- follow(): 미션을 통째로 교체 → (시동) → 시작. 업로드 직후 PX4 가 검증을 끝내기 전에는 시작을 거부하므로 재시도한다.
- stop(): 진행 방향 앞 제동거리 지점 하나짜리 미션으로 교체 (base.Driver.stop). HOLD 는 명령 시점 위치로 U턴한다.
- 정차 유지 = 시동 끄기(disarm): 미션이 끝나 멈추면(0.3 m/s 미만 PARK_AFTER_S) 시동을 끈다. 이 rover 는 미션 끝 '대기(loiter)'
  에서 조금 지나친 마지막 지점으로 돌아가려다 다시 달리거나 맴돌았다 (2026-10-10 Gazebo: 정차 5 s 뒤 스스로 출발, 26분 6 km).
  시동이 꺼지면 바퀴 속도 명령이 0 이라 그 자리에 선다. 다음 미션(follow)에서 다시 시동을 건다.
  미션이 끝났는데 서지 않고 RUNAWAY_PARK_S 넘게 달리면 그래도 시동을 끈다: 마지막 지점 속도(> 0)를 그대로 유지한 채 대기(HOLD)에서
  직진했다 (2026-10-11 Gazebo 지속 운용: 고장 주입 → 정지 경로 64 m 끝 2.9 m/s → FAULT 상태로 15분 넘게 직진 2.6 km 이상).
- 텔레메트리는 구독을 계속 열어 두고 마지막 값을 보관한다. 위치가 끊기면 Telemetry.age_s() 가 커진다 (감시는 vehicle.py).
검증 (2026-10-09, 평지 월드): 직선 60.5 km/h, 정지 57 m, 90°·45°·135° 꺾임 17.9/36.1/17.7 km/h. ugv/tools/drive_test.py
"""

import asyncio
import math
import os
import time
from typing import List, Optional

from ..geo import FRAME, GZ_FRAME
from .base import Driver, DriverError, Item, Progress, Telemetry

START_RETRIES = 20
PARK_AFTER_S = 1.0          # 미션 끝 + 0.3 m/s 미만이 이만큼 이어지면 시동을 꺼 정차 유지
ARMED_STALE_S = 3.0         # armed 보고가 이만큼 없으면 시동 걸림으로 본다 (heartbeat 1 Hz)
RUNAWAY_PARK_S = 4.0        # 미션 끝 + 0.3 m/s 이상이 이만큼 이어지면 (미션 끝에서 서지 않고 달림) 그래도 시동을 끈다
LEAD_BASE_M = 5.0           # 선행 지점 = 지금 위치에서 경로를 따라 LEAD_BASE_M + 속도 × LEAD_TIME_S 앞
LEAD_TIME_S = 2.0           #   (업로드·시작에 걸리는 동안 지나쳐 뒤에 남지 않게. 뒤에 남으면 rover 가 되돌아간다)
ACCEPT_M = float(os.getenv("UGV_PX4_ACCEPT_M", "3.0"))   # 미션 지점 도착 반경 (airframe NAV_ACC_RAD 와 맞춘다. 환경변수는 시험용)
START_RETRY_S = 0.5
ARM_RETRIES = 15
ARM_RETRY_S = 1.0


def mavsdk_plan(items: List[Item], here, v_now: float):
    """공통 Item[(x, y, 그 지점까지 구간 속도)] → MAVSDK 미션 [(x, y, 그 지점을 지난 뒤 속도, 통과 여부)].

    지점 k 에 지점 k+1 의 구간 속도를 싣는다 (마지막 지점은 정지점이라 지난 뒤 속도가 없다 — 자기 값).
    첫 구간(지금 위치 → 첫 지점)의 속도는 실을 곳이 없으므로, 경로를 따라 조금 앞에 선행 지점을 넣고 거기에
    첫 지점의 구간 속도를 싣는다. 첫 지점이 선행 거리보다 가까우면 넣지 않는다 (짧은 첫 구간은 이전 속도 그대로)."""
    if not items:
        return []
    out = []
    lead = LEAD_BASE_M + max(v_now, 0.0) * LEAD_TIME_S
    d0 = math.dist(here, items[0][:2])
    if d0 > lead + ACCEPT_M:
        t = lead / d0
        out.append((here[0] + (items[0][0] - here[0]) * t, here[1] + (items[0][1] - here[1]) * t, items[0][2], True))
    for k, (x, y, v) in enumerate(items):
        last = k == len(items) - 1
        out.append((x, y, v if last else items[k + 1][2], not last))
    return out


def park_after_policy(end_stop_info: dict) -> dict:
    """E2 기준 ③ 관측용 시험 훅 (Fable 미해결 2.2): 정상 정차 시동 끔 대기(PARK_AFTER_S)를 UGV_TEST_HOOKS=1 일 때만
    UGV_PX4_PARK_AFTER_S 로 늘린다 — [PARK_AFTER_S, RUNAWAY_PARK_S − 1.0] 로 자름. 운영 기본 1.0·RUNAWAY 상한·cut_drive·비상 정지 불변.
    PX4 가 끝 지점에서 스스로 서지 않는 차(RA_ACC_RAD_MAX < NAV_ACC_RAD, 또는 파라미터 미확인)는 무시 — LOITER 에서 기어가므로 긴 대기는 위치 오차만 키운다."""
    req = os.getenv("UGV_PX4_PARK_AFTER_S") if os.getenv("UGV_TEST_HOOKS") == "1" else None
    pol = {"after_s": PARK_AFTER_S, "test_hook": False, "requested": req, "basis": "운영 기본"}
    if req is None:
        return pol
    try:
        v = min(max(float(req), PARK_AFTER_S), RUNAWAY_PARK_S - 1.0)
    except ValueError:
        return {**pol, "basis": "요청값 해석 실패 — 기본 유지"}
    nav, ra = (end_stop_info or {}).get("NAV_ACC_RAD"), (end_stop_info or {}).get("RA_ACC_RAD_MAX")
    if nav is None or ra is None or not (ra >= nav > 0.0):
        return {**pol, "basis": "PX4 끝 지점 자체 정지 없음/미확인 (RA_ACC_RAD_MAX < NAV_ACC_RAD) — 시험 훅 무시"}
    return {**pol, "after_s": v, "test_hook": True, "basis": "시험 훅 (UGV_TEST_HOOKS=1)"}


def park_due(speed_mps: float, now: float, still_since, finished_since, park_after_s: float = None):
    """미션이 끝난 차의 정차 유지(시동 끔) 판단 → (끌지, still_since, finished_since).
    0.3 m/s 미만이 PARK_AFTER_S 이어지면 끈다 (정상 정차). 속도와 관계없이 미션 끝 뒤 RUNAWAY_PARK_S 가 지나도 끈다 (서지 않고 달림).
    finished_since 는 그 미션이 끝난 것을 처음 본 시각 — 속도로 초기화하지 않는다 (호출 쪽이 새 미션에서만 지운다).
    (2026-10-11 검토: 고속·저속 타이머가 서로 지워 0.29↔0.31 m/s 교대면 끝없이 미뤄졌다)"""
    d = park_decide(speed_mps, now, still_since, finished_since, park_after_s)
    return d["due"], d["still_since"], d["finished_since"]


def park_decide(speed_mps: float, now: float, still_since, finished_since, park_after_s: float = None) -> dict:
    """park_due 와 같은 판단·분기 우선순위(상한 먼저)를 그대로 하되, 실제로 선택한 분기와 두 조건을 따로 돌려준다 (Codex 0130 P1).
    reason: 'RUNAWAY_CAP' (미션 끝 뒤 상한 — 속도와 무관, 저속이어도 상한이 먼저 참이면 이것) / 'NORMAL_STILL' (저속 연속 after_s) / None.
    cond_cap·cond_still 은 그 순간의 조건 참/거짓 (선택 사유와 별개). still_reset = 이번 표본이 저속 타이머를 지웠는지."""
    after = PARK_AFTER_S if park_after_s is None else park_after_s
    finished_since = finished_since if finished_since is not None else now
    cond_cap = now - finished_since >= RUNAWAY_PARK_S
    slow = speed_mps < 0.3
    new_still = (still_since if still_since is not None else now) if slow else None
    cond_still = slow and now - new_still >= after
    if cond_cap:                                   # 원래 순서: 상한 분기가 먼저 — still_since 는 건드리지 않는다
        return {"due": True, "reason": "RUNAWAY_CAP", "still_since": still_since, "finished_since": finished_since,
                "cond_cap": True, "cond_still": cond_still, "still_reset": False, "after_s": after}
    return {"due": cond_still, "reason": "NORMAL_STILL" if cond_still else None, "still_since": new_still,
            "finished_since": finished_since, "cond_cap": False, "cond_still": cond_still,
            "still_reset": still_since is not None and not slow, "after_s": after}


START_AHEAD_SEARCH = 60     # 시작 지점을 찾을 때 앞쪽 몇 개 항목까지 보는가 (업로드 동안 지나칠 수 있는 범위)
START_AHEAD_ACCEL_MPS2 = 1.5  # 업로드 동안 거리 상한의 가속 여유 (차종 최대 가속 1.2~1.5)
START_TIE_M = 1.0              # 미션 선 여러 구간이 이 거리 차 안으로 맞으면 경로 순서상 앞 구간 (나중에 다시 지나는 곳 배제)
START_AHEAD_MAX_OFF_M = 30.0   # 미션 선에서 이보다 멀면 맞추지 않는다 (다른 곳 — 서버 감시가 맡는다)


def first_item_ahead(sent, pos, v_mps: float, max_along_m: float = None, here=None) -> int:
    """업로드가 끝나 미션을 시작하기 직전 위치 pos 에서, 보낸 미션 [(x, y, v, through)] 중 차가 아직 지나지 않은 첫 항목 번호. 0 이면 그대로 시작.
    (2026-10-11 Gazebo 지속 운용: 목표 교체 때 긴 미션 업로드 8.9 s 동안 지나쳐 첫 항목들이 차 뒤에 남자 PX4 로버가 되돌아 맴돌다 도로 이탈 —
    원인 검증: Fable 서브에이전트)

    경로 진행 위치로 찾는다 — 같은 도로·같은 자리를 경로가 여러 번 지나도 출발부터 이어지는 첫 진행 구간에서만:
      - 경로선 = 계획 시점 위치 here (미션을 만든 출발점) + 미션 항목들. 정차 출발이면 차는 첫 구간(here → 선행 지점) 위다.
      - max_along_m (업로드 동안 갈 수 있었던 거리) 밖 구간은 보지 않는다.
      - 거의 같은 거리(START_TIE_M 안)로 맞는 구간이 여럿이면 경로 순서상 가장 앞 구간 (뒤 구간 = 나중에 다시 지나는 곳).
    (2026-10-11 Gazebo c23 A-ugv1 SK102: 블록을 돌아 출발 도로를 반대로 다시 지나는 구간이 주차 위치와 0.06 m 로 겹쳐, 정차한 차를
    '이미 476 m 지나왔다' 로 보고 뒤쪽 항목 44 부터 시작 → 제자리 180° 선회·도로 이탈. 원인 검증: Fable 서브에이전트)"""
    head = [tuple(here[:2])] if here is not None else []
    pts = head + [(it[0], it[1]) for it in sent[:START_AHEAD_SEARCH + 1]]
    off = len(head)                                 # pts 번호 - off = sent 번호
    if len(pts) < 2:
        return 0
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    segs = []
    for k, (a, b) in enumerate(zip(pts, pts[1:])):
        if max_along_m is not None and cum[k] > max_along_m:
            break                                  # 업로드 동안 갈 수 없었던 곳
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((pos[0] - a[0]) * dx + (pos[1] - a[1]) * dy) / L2))
        d = math.hypot(pos[0] - a[0] - t * dx, pos[1] - a[1] - t * dy)
        segs.append((d, cum[k] + t * (cum[k + 1] - cum[k])))
    if not segs:
        return 0
    dmin = min(d for d, _ in segs)
    d, along = next((d, s) for d, s in segs if d <= dmin + START_TIE_M)      # 경로 순서상 가장 앞 구간
    lead = cum[off] if off else 0.0                 # 첫 미션 항목(선행 지점)까지 경로 거리
    if d > START_AHEAD_MAX_OFF_M or along <= lead + 0.5:
        return 0                                   # 멀거나, 아직 첫 항목(선행 지점)에 닿지 않았다 — 평소 출발은 그대로
    need = along + ACCEPT_M + max(v_mps, 0.0) * 1.0
    for k in range(off, len(cum)):
        if cum[k] >= need:
            return k - off
    return 0


PASS_WINDOW = 15            # 지나침 감시: 현재 항목 앞뒤로 볼 구간 수
PASS_MARGIN_M = 0.5         # 경로 진행 위치가 현재 항목을 이만큼 넘으면 '지나침'
PASS_MAX_OFF_M = 15.0       # 미션 선에서 이보다 멀면 판단하지 않는다 (서버 이탈 감시가 맡는다)
PASS_EVERY_S = 0.2
PASS_HOLDOFF_S = 1.0        # 건너뛰기 명령 뒤 다음 판단까지
PASS_TIE_M = 0.1            # 가장 가까운 구간이 여럿이면 이 차 안에서 경로 순서상 앞 (옆으로 밀린 차에는 1 m 면 앞 구간 끝점이 뽑힌다)
STALL_AWAY_M = 1.5          # 정체: PX4 현재 항목이 그대로인데 차가 그 항목에서 (도착 반경 + 0.5 m 밖, 지금까지 최소 거리 + 이만큼) 멀어짐
PASS_HEAD_MAX_DEG = 60.0    # 건너뛸 항목이 차 진행 방향 이 각도 안(앞)일 때만 — 아니면 보류(차량이 정체로 판단해 정지)
PASS_SKIP = os.getenv("UGV_PX4_PASS_SKIP", "0") == "1"          # 자동 건너뛰기 (안전 조건 통과 때만)
STALL_DETECT = os.getenv("UGV_PX4_STALL_DETECT", "0") == "1"    # 정체 감지(→ 차량이 MISSION_STALL 고장·정지). 건너뛰기와 따로 켠다


def passed_item(sent, here, cur: int, pos, v_mps: float = 0.0):
    """주행 중 지나침 감시: 차가 PX4 현재 항목 cur 을 도착 반경(ACCEPT_M) 밖으로 지나쳤으면 차 앞의 첫 항목 번호, 아니면 None.
    PX4 미션은 도착 반경 안에 들어와야 다음 항목으로 넘어간다. 급한 곡선에서 차가 바깥으로 3 m 넘게 밀려 항목을 지나치면
    PX4 는 그 항목을 계속 현재 목표로 두고 되돌아가려 최대 조향으로 돌다가, 멈춘 구간의 연장선을 따라 직진해 도로를 벗어났다
    (2026-10-12 Gazebo c33 A 교량 경로 항목 51 최소 3.34 m, c36 B 소방서 출발 항목 9 최소 3.06 m — ulog 미션 진행·궤적 대조).
    경로 진행 위치는 cur 앞뒤 구간에서 가장 가까운 구간(거의 같으면 경로 순서상 앞)으로 잰다 (first_item_ahead 와 같은 방식).
    마지막 항목(정지점)은 건너뛰지 않는다.
    건너뛸 항목 = 경로 진행 위치 + ACCEPT_M + 속도 × 1 s 보다 먼 첫 항목 (first_item_ahead 와 같은 기준). PX4 는 도착 반경 안 항목을
    스스로 넘기므로 실제 현재 항목은 그보다 앞이 아니다 — MAVSDK 진행 보고가 늦어도 뒤로 되돌리지 않는다
    (2026-10-12 c38: '진행 위치 + 1 m' 기준은 늦은 보고 12 를 보고 13 을 설정, PX4 는 이미 15 → 되돌아가 이탈 25.9 m)."""
    n = len(sent)
    if cur is None or cur <= 0 and here is None or cur >= n - 1:
        return None
    if math.dist(pos, sent[cur][:2]) <= ACCEPT_M:
        return None                                 # 곧 PX4 가 도착 처리한다
    lo = max(cur - 1, 0)
    pts = ([tuple(here[:2])] if cur == 0 and here is not None else []) + [tuple(it[:2]) for it in sent[lo:cur + PASS_WINDOW + 1]]
    base = lo - (1 if cur == 0 and here is not None else 0)    # pts 번호 + base = sent 번호
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    segs = []
    for k, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((pos[0] - a[0]) * dx + (pos[1] - a[1]) * dy) / L2))
        segs.append((math.hypot(pos[0] - a[0] - t * dx, pos[1] - a[1] - t * dy), cum[k] + t * (cum[k + 1] - cum[k])))
    if not segs:
        return None
    dmin = min(d for d, _ in segs)
    d, along = next((d, s) for d, s in segs if d <= dmin + PASS_TIE_M)
    if d > PASS_MAX_OFF_M or along < cum[cur - base] + PASS_MARGIN_M:
        return None
    need = along + ACCEPT_M + max(v_mps, 0.0) * 1.0
    for k in range(cur - base + 1, len(cum)):
        if cum[k] >= need:
            return min(k + base, n - 1)
    return None                                     # 창 안에 없으면 다음 판단으로 (멀리 건너뛰지 않는다)


def stall_away(dcur: float, dmin: float) -> bool:
    """PX4 현재 항목에서 멀어지는 중인가: 도착 반경 + 0.5 m 밖이고, 그 항목이 현재가 된 뒤 최소 거리보다 STALL_AWAY_M 넘게 멀다.
    (기록 재생 2026-10-12: 정상 주행 5건 0회, 놓친 항목 실패 5건 모두 — 1 s 지속 기준)"""
    return dcur > ACCEPT_M + 0.5 and dcur > dmin + STALL_AWAY_M


def pass_allowed(target_xy, pos, heading_deg: float, body_inside: bool):
    """지나친 항목을 건너뛰어도 되는가 → (가부, 진행 방향과 목표 방위 차 °). 목표가 차 앞(±PASS_HEAD_MAX_DEG)이고 차체가 도로 띠 안일 때만.
    아니면 건너뛰지 않는다 (무조건 넘기지 않음 — 차량이 정체로 보고 정지한다)."""
    brg = math.degrees(math.atan2(target_xy[0] - pos[0], target_xy[1] - pos[1])) % 360.0
    dh = abs((brg - heading_deg + 180.0) % 360.0 - 180.0)
    return (dh <= PASS_HEAD_MAX_DEG and body_inside), dh


def _mission_item(lat, lon, speed, through):
    from mavsdk.plugins.mission.mission import MissionItem
    return MissionItem(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=0.0, speed_m_s=speed,
                       is_fly_through=through, gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
                       camera_action=MissionItem.CameraAction.NONE, loiter_time_s=0.0,
                       camera_photo_interval_s=0.0, acceptance_radius_m=ACCEPT_M, yaw_deg=float("nan"),
                       camera_photo_distance_m=0.0, vehicle_action=MissionItem.VehicleAction.NONE)


class Px4Driver(Driver):
    kind = "px4"
    position_ref = "center"   # 텔레메트리 위치 기준 (차체 중심 = GPS). ugv/traffic.py 가 차체 중심으로 맞춘다

    def __init__(self, address: str, clock, connect_timeout_s: float = 30.0, decel_mps2=None, gazebo: bool = False):
        self.address, self.clock, self.connect_timeout_s = address, clock, connect_timeout_s
        # Gazebo SITL 이면 Gazebo 와 같은 위경도 변환 (geo.GzEnuFrame — 도로 월드 좌표와 PX4 위치를 맞춘다). 실차는 FRAME
        self.frame = GZ_FRAME if gazebo else FRAME
        self.decel_mps2 = decel_mps2
        self._sdk = self._tel = self._act = self._mis = None
        self._pos = self._vel = None
        self._mono = {}                         # 필드별 마지막 수신 시각 (위치·속도·방향·armed·모드·진행)
        self._heading = 0.0
        self._armed = False
        self._mode = "UNKNOWN"
        self._pos_mono = 0.0
        self._prog = Progress()
        self._expected_total = 0
        self._seen_running = False
        self._tasks: List[asyncio.Task] = []
        self._cmd = asyncio.Lock()
        self.connected = False
        self.last_error: Optional[str] = None
        self.parked_seq = None                  # 정차 유지(시동 끔)한 미션 번호
        self._sent = None                       # (미션 번호, 보낸 항목, 계획 시점 위치) — 지나침 감시용
        self.pass_skips: List[dict] = []        # 지나침 감시가 건너뛴·보류한 기록 (시험 분석용)
        self.pass_guard = None                  # 차량이 넘겨주는 판정: guard(tel) → 차체가 도로 띠 안이면 True
        self.pass_blocked_since: Optional[float] = None   # 정체 시작 시각 (monotonic): 현재 항목을 놓치고 멀어지는데 건너뛸 수 없음
        self._stall_key, self._stall_dmin = None, math.inf

    async def start(self):
        from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
        from mavsdk.asyncio.plugins.action import ActionAsync
        from mavsdk.asyncio.plugins.mission import MissionAsync
        from mavsdk.asyncio.plugins.telemetry import TelemetryAsync
        sdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
        await sdk.add_any_connection(self.address)
        system = await sdk.first_autopilot(timeout_s=self.connect_timeout_s)
        if system is None:
            sdk.destroy()
            raise DriverError(f"PX4_NOT_FOUND: {self.address} (ugv/tools/px4-start.sh, 방화벽 확인)")
        self._sdk, self._tel = sdk, TelemetryAsync(system)
        self._act, self._mis = ActionAsync(system), MissionAsync(system)
        await self._mis.set_return_to_launch_after_mission(False)
        for name, stream in (("position", self._tel.subscribe_position), ("velocity", self._tel.subscribe_velocity_ned),
                             ("heading", self._tel.subscribe_heading), ("armed", self._tel.subscribe_armed),
                             ("mode", self._tel.subscribe_flight_mode),
                             ("progress", self._mis.subscribe_mission_progress)):
            self._tasks.append(asyncio.create_task(self._pump(name, stream)))
        for _ in range(100):
            if self._pos is not None:
                break
            await asyncio.sleep(0.1)
        self.connected = self._pos is not None
        if not self.connected:
            raise DriverError("PX4_NO_POSITION")
        await self._read_end_stop_params(system)
        self.park_policy = park_after_policy(self.end_stop_info)
        self._tasks.append(asyncio.create_task(self._park_watch()))
        if PASS_SKIP or STALL_DETECT:
            self._tasks.append(asyncio.create_task(self._pass_watch()))

    async def _read_end_stop_params(self, system):
        """E4 G2 (Fable 미해결 1.3): PX4 로버는 끝 지점에서 position_controller_status 의 도착 반경 경계에 들어서면 속도 설정 0 으로 선다 —
        그 메시지는 RA_ACC_RAD_MAX ≥ NAV_ACC_RAD 일 때만 발행된다 (AckermannAutoMode). 그런 차만 offset = NAV_ACC_RAD, 아니면 0
        (RA −1 차에 연장하면 점까지 기어가 목적지를 지나친다). 읽기 실패 → 0 + 사유 기록."""
        self.end_stop_offset_m, self.end_stop_info = 0.0, {"basis": "읽기 전"}
        try:
            from mavsdk.asyncio.plugins.param import ParamAsync
            prm = ParamAsync(system)
            nav = float(await asyncio.wait_for(prm.get_param_float("NAV_ACC_RAD"), 5.0))
            ra = float(await asyncio.wait_for(prm.get_param_float("RA_ACC_RAD_MAX"), 5.0))
            on = ra >= nav > 0.0
            self.end_stop_offset_m = nav if on else 0.0
            self.end_stop_info = {"NAV_ACC_RAD": nav, "RA_ACC_RAD_MAX": ra, "offset_m": self.end_stop_offset_m,
                                  "basis": "RA_ACC_RAD_MAX ≥ NAV_ACC_RAD — 끝 지점 반경 경계 정지" if on else "RA_ACC_RAD_MAX < NAV_ACC_RAD — 반경 정지 없음, 연장 안 함"}
        except Exception as e:  # noqa: BLE001
            self.end_stop_offset_m = 0.0
            self.end_stop_info = {"offset_m": 0.0, "basis": "파라미터 읽기 실패 — 연장 안 함", "error": repr(e)[:200]}

    async def test_link(self, cut: bool) -> dict:
        """시험용 (UGV_TEST_HOOKS=1): 이 차량의 MAVLink 연결만 끊거나 잇는다. 끊으면 이 차량의 MAVSDK 인스턴스를 닫아 서버는 이 차에
        아무것도 보내지 못하고(하트비트 포함) PX4 는 지상국 연결 끊김(COM_DL_LOSS_T)으로 판단한다 — PX4 안의 MAVLink 모듈은 그대로라
        실제 무선 단절과 같다. 마지막 텔레메트리는 그대로 남아 낡아 간다 (서버는 TELEMETRY_LOST 로 판단).
        (remove_connection 만으로는 하트비트가 계속 나가 PX4 가 끊김을 몰랐다. PX4 MAVLink 인스턴스를 멈추는 방법은 dataman 읽기
        시간 초과로 미션 추종이 흔들려 결과를 바꿨다, 2026-10-10)"""
        if cut and self._sdk is not None:
            await self.close()
        elif not cut and self._sdk is None:
            await self.start()
        return {"linked": self._sdk is not None}

    async def _park_watch(self):
        """미션이 끝나고 멈춘 차의 시동을 끈다 (정차 유지). 다음 follow 가 다시 시동을 건다."""
        still_since = fin_since = fin_seq = None
        while True:
            await asyncio.sleep(0.5)
            try:
                tel = self.telemetry()
                prog = self._prog
                if fin_seq != prog.mission_seq:          # 새 미션에서만 미션 끝 타이머를 지운다
                    fin_seq, fin_since, still_since = prog.mission_seq, None, None
                if not self._armed or not prog.finished or self.parked_seq == prog.mission_seq:
                    continue
                # 속도를 믿을 수 없으면(없음·오래됨) 정차로 보지 않는다 — 미션 끝 상한(RUNAWAY_PARK_S)만 센다
                fresh = tel is not None and tel.age_s() <= 5.0
                now = time.monotonic()
                if fin_since is None:
                    resets, fin_first = [], now                  # 이 미션 끝을 처음 본 시각 (서버 monotonic)
                dec = park_decide(tel.speed_mps if fresh else 1.0, now, still_since, fin_since, self.park_policy.get("after_s"))
                if dec["still_reset"] and len(resets) < 20:
                    resets.append({"t": round(now, 3), "speed": round(tel.speed_mps, 3) if fresh else None, "fresh": fresh})
                still_since, fin_since = dec["still_since"], dec["finished_since"]
                if not dec["due"]:
                    continue
                # 판단 순간 기록 (Codex 0130 P1): 실제 선택 분기·두 조건·입력 속도와 나이 — 잠금 뒤 다시 계산하지 않는다
                rec = {"mission_seq": prog.mission_seq, "run_id": os.getenv("ADAIR_RUN_ID"), "reason": dec["reason"],
                       "cond_cap": dec["cond_cap"], "cond_still": dec["cond_still"], "after_s": dec["after_s"],
                       "decide_mono": round(now, 3), "decide_wall": round(time.time(), 3), "finished_first_mono": round(fin_first, 3),
                       "still_since_mono": None if still_since is None else round(still_since, 3), "still_resets": resets,
                       "speed_in": round(tel.speed_mps, 3) if fresh else None, "tel_age_s": round(tel.age_s(), 3) if tel else None,
                       "fresh": fresh, "poll_s": 0.5}
                async with self._cmd:
                    rec["lock_mono"] = round(time.monotonic(), 3)
                    if self._prog.mission_seq != prog.mission_seq or not self._prog.finished:
                        rec["outcome"] = "ABANDONED_MISSION_CHANGED"
                        self._park_record(rec)
                        continue
                    if dec["reason"] == "RUNAWAY_CAP":
                        self.last_error = (f"park: 미션 끝 뒤 {RUNAWAY_PARK_S:g} s 상한 → 시동 끔 "
                                           f"(속도 {tel.speed_mps:.1f} m/s)" if fresh else "park: 미션 끝 상한 → 시동 끔 (속도 미확인)")
                    rec["disarm_req_mono"] = round(time.monotonic(), 3)
                    try:
                        await self._act.disarm()
                        rec["disarm_call"] = "OK"
                    except Exception as e:  # noqa: BLE001
                        rec["disarm_call"] = f"ERR {type(e).__name__}: {e}"[:200]
                        self._park_record(rec)
                        raise
                    rec["disarm_ret_mono"] = round(time.monotonic(), 3)
                    self.parked_seq = prog.mission_seq
                self._tasks.append(asyncio.create_task(self._park_observe_disarmed(rec)))
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001
                self.last_error = f"park: {type(e).__name__}: {e}"

    def _park_record(self, rec: dict):
        """시동 끔 판단 기록: park_log(마지막, /state) + UGV_STATE_DIR/park_events.jsonl (있으면)."""
        self.park_log = rec
        d = os.getenv("UGV_STATE_DIR")
        if d:
            try:
                import json
                with open(os.path.join(d, "park_events.jsonl"), "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            except OSError:
                pass

    async def _park_observe_disarmed(self, rec: dict, limit_s: float = 5.0):
        """disarm 뒤 armed 해제 관측 시각 (텔레메트리 armed 흐름). limit_s 안에 못 보면 미관측(None)으로 기록."""
        t0 = time.monotonic()
        rec["disarmed_seen_mono"] = None
        while time.monotonic() - t0 < limit_s:
            if not self._armed:
                rec["disarmed_seen_mono"] = round(time.monotonic(), 3)
                break
            await asyncio.sleep(0.05)
        self._park_record(rec)

    async def _pass_watch(self):
        """주행 중 감시 (PASS_SKIP: 지나친 항목 건너뛰기, STALL_DETECT: 정체 감지). 한 번 분량은 _pass_step."""
        hold_until = 0.0
        while True:
            await asyncio.sleep(PASS_EVERY_S)
            try:
                hold_until = await self._pass_step(hold_until)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001
                self.last_error = f"pass_watch: {type(e).__name__}: {e}"
                hold_until = time.monotonic() + PASS_HOLDOFF_S

    async def _pass_step(self, hold_until: float) -> float:
        sent, prog, tel = self._sent, self._prog, self.telemetry()
        if (sent is None or tel is None or tel.age_s() > 1.0 or not self._armed or prog.finished
                or prog.mission_seq != sent[0] or prog.total != len(sent[1])):
            # 판단할 수 없는 동안(텔레메트리 지연·시동 꺼짐·미션 끝·미션 교체 중)은 정체 타이머를 지운다 —
            # 재개 직후 이전 타이머로 오작동하지 않게
            self.pass_blocked_since, self._stall_key = None, None
            return hold_until
        if time.monotonic() < hold_until:
            return hold_until
        # 정체 판단은 지나침 판정과 따로: 현재 항목에서 계속 멀어지면 (PX4 로버는 항목을 지나쳤다는 판정이 없다 —
        # navigator passed_curr_wp 는 고정익만. 도착 반경 밖으로 지나치면 그 항목에 계속 머문다)
        key = (sent[0], prog.current)
        dcur = math.dist((tel.x, tel.y), sent[1][prog.current][:2])
        if key != self._stall_key:
            self._stall_key, self._stall_dmin, self.pass_blocked_since = key, dcur, None
        self._stall_dmin = min(self._stall_dmin, dcur)
        away = stall_away(dcur, self._stall_dmin)
        k = passed_item(sent[1], sent[2], prog.current, (tel.x, tel.y), tel.speed_mps) if PASS_SKIP else None
        ok, dh, inside = False, None, None
        if k is not None:
            # 안전 조건: 건너뛸 항목이 차 앞(진행 방향 ±PASS_HEAD_MAX_DEG), 차체가 도로 띠 안. 아니면 건너뛰지 않는다
            inside = True if self.pass_guard is None else bool(self.pass_guard(tel))
            ok, dh = pass_allowed(sent[1][k][:2], (tel.x, tel.y), tel.heading_deg, inside)
        if not ok:
            if not STALL_DETECT:
                return hold_until
            if away and self.pass_blocked_since is None:
                self.pass_blocked_since = time.monotonic()
                self.pass_skips.append({"wall": round(time.time(), 2), "mission_seq": sent[0], "from": prog.current,
                                        "to": k, "blocked": True, "dist_m": round(dcur, 2), "dmin_m": round(self._stall_dmin, 2),
                                        "heading_diff_deg": None if dh is None else round(dh, 1), "body_inside": inside,
                                        "xy": [round(tel.x, 1), round(tel.y, 1)]})
            elif not away:
                self.pass_blocked_since = None
            return hold_until
        self.pass_blocked_since = None
        async with self._cmd:
            if self._prog.mission_seq != sent[0] or self._prog.current != prog.current:
                return hold_until
            await self._mis.set_current_mission_item(k)
            rec = {"wall": round(time.time(), 2), "mission_seq": sent[0], "from": prog.current, "to": k,
                   "dist_m": round(dcur, 2), "xy": [round(tel.x, 1), round(tel.y, 1)]}
            self.pass_skips.append(rec)
            if isinstance(getattr(self, "last_sent", None), dict):
                self.last_sent.setdefault("pass_skips", []).append(rec)
        return time.monotonic() + PASS_HOLDOFF_S

    async def _pump(self, name, stream):
        import time
        while True:
            try:
                async for v in stream():
                    self._mono[name] = time.monotonic()
                    if name == "position":
                        self._pos, self._pos_mono = v, time.monotonic()
                    elif name == "velocity":
                        self._vel = v
                    elif name == "heading":
                        self._heading = v.heading_deg
                    elif name == "armed":
                        self._armed = bool(v)
                    elif name == "mode":
                        from mavsdk.plugins.telemetry.telemetry import FlightMode
                        try:
                            self._mode = FlightMode(int(v)).name
                        except (ValueError, TypeError):
                            self._mode = getattr(v, "name", str(v))
                    elif name == "progress":
                        # 새 미션을 보낸 뒤 '진행 중'(current < total)을 한 번 본 다음에만 '끝남'을 받아들인다
                        # (지난 미션의 끝남 보고가 늦게 와도 새 미션 완료로 오해하지 않게)
                        if v.total != self._expected_total or v.total == 0:
                            continue
                        if v.current < v.total:
                            self._seen_running = True
                        fin = self._seen_running and v.current >= v.total
                        self._prog = Progress(v.current, v.total, fin, self._prog.mission_seq)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 — 구독이 끊기면 다시 연다
                self.last_error = f"{name}: {type(e).__name__}: {e}"
                await asyncio.sleep(1.0)

    async def close(self):
        for t in self._tasks:
            t.cancel()
        self._tasks = []
        if self._sdk is not None:
            self._sdk.destroy()
            self._sdk = None

    def telemetry(self) -> Optional[Telemetry]:
        if self._pos is None:
            return None
        import time
        x, y = self.frame.to_xy(self._pos.latitude_deg, self._pos.longitude_deg)
        # 신선도 = 위치·속도 중 오래된 쪽. 속도를 아직 못 받았으면 0 m/s(정지)로 보지 않고 나이 무한대(텔레메트리 없음과 같게)
        # (2026-10-11 검토: 위치만 새로 오면 오래된·없는 속도도 최신 정지로 보였다 — _standstill·정지 감독·출발 판정 공통)
        v = math.hypot(self._vel.north_m_s, self._vel.east_m_s) if self._vel is not None else 0.0
        upd = min(self._pos_mono, self._mono.get("velocity", -math.inf))
        # armed 가 오래되면 '시동 걸림' 으로 본다 (시동 꺼짐 = 정차 유지로 오판하지 않게)
        armed = self._armed or time.monotonic() - self._mono.get("armed", -math.inf) > ARMED_STALE_S
        return Telemetry(x, y, self._pos.latitude_deg, self._pos.longitude_deg, v, self._heading, armed,
                         self._mode, upd, self._pos.absolute_altitude_m)

    def progress(self) -> Progress:
        return self._prog

    async def _ensure_armed(self):
        for k in range(ARM_RETRIES):
            if self._armed:
                return
            try:
                await self._act.arm()
                return
            except Exception as e:  # noqa: BLE001 — EKF 안정 전에는 거절
                self.last_error = f"arm: {e}"
                await asyncio.sleep(ARM_RETRY_S)
        raise DriverError(f"ARM_FAILED: {self.last_error}")

    async def follow(self, items: List[Item]) -> int:
        if not items:
            raise DriverError("EMPTY_MISSION")
        from mavsdk.plugins.mission.mission import MissionPlan
        tel = self.telemetry()
        here = (tel.x, tel.y) if tel is not None else items[0][:2]
        plan = []
        sent = mavsdk_plan(items, here, tel.speed_mps if tel is not None else 0.0)
        t_up, v_up = time.monotonic(), (max(tel.speed_mps, 0.0) if tel is not None else 0.0)
        self.last_sent = {"wall": time.time(), "here": [round(here[0], 2), round(here[1], 2)],
                          "items": [[round(x, 2), round(y, 2), round(v, 2), th] for x, y, v, th in sent]}
        for x, y, v, through in sent:
            lat, lon = self.frame.to_ll(x, y)
            plan.append(_mission_item(lat, lon, float(v), through=through))
        async with self._cmd:
            seq = self._prog.mission_seq + 1
            self._expected_total = len(plan)
            self._seen_running = False
            self._prog = Progress(0, len(plan), False, seq)
            self._sent = (seq, sent, here)
            self.pass_blocked_since = None
            try:
                await self._mis.upload_mission(MissionPlan(plan))
                await self._ensure_armed()
                tel2 = self.telemetry()
                if tel2 is not None:
                    # 업로드 동안 갈 수 있었던 거리 상한: 선행 지점 + 수용 반경 + (더 빠른 속도 + 가속 여유) × 경과 + 여유
                    dt = time.monotonic() - t_up
                    reach = LEAD_BASE_M + ACCEPT_M + (max(v_up, tel2.speed_mps) + START_AHEAD_ACCEL_MPS2 * dt) * dt + 2.0
                    k0 = first_item_ahead(sent, (tel2.x, tel2.y), tel2.speed_mps, max_along_m=reach, here=here)
                    self.last_sent["start_reach_m"] = round(reach, 1)
                else:
                    k0 = 0
                if k0 > 0:
                    # 업로드 동안 지나친 항목은 건너뛰고 차 앞 항목부터 (뒤에 남은 항목으로 되돌아가지 않게)
                    try:
                        await self._mis.set_current_mission_item(k0)
                        self.last_sent["start_item"] = k0
                    except Exception as e:  # noqa: BLE001
                        self.last_error = f"set_current_mission_item({k0}): {e}"
                        self.last_sent["start_item_error"] = str(e)[:120]
                for k in range(START_RETRIES):
                    try:
                        await self._mis.start_mission()
                        break
                    except Exception as e:  # noqa: BLE001
                        if k == START_RETRIES - 1:
                            raise
                        await asyncio.sleep(START_RETRY_S)
            except DriverError:
                raise
            except Exception as e:  # noqa: BLE001
                self.last_error = f"follow: {type(e).__name__}: {e}"
                raise DriverError(f"MISSION_START_FAILED: {e}") from e
            return seq

    async def set_speed_cap(self, v: float) -> bool:
        """MAV_CMD_DO_CHANGE_SPEED (mavsdk set_current_speed). 이 rover 는 미션 항목 speed_m_s 를 따르지 않고 RO_SPEED_LIM 까지 가속한다
        (2026-10-12 drive_test bump 5 m/s 지정 → 12.6 m/s, 2026-10-13 경사 중단: 정지 경로 전송 뒤 9.4 → 13.4 m/s 가속 후 감속).
        UGV_STOP_SPEED_CMD=1 후보에서 정지 경로 전송 직후 지금 속도를 상한으로 보낸다. 실패하면 False (호출 쪽 기록)."""
        if self._act is None:
            return False
        try:
            await self._act.set_current_speed(max(float(v), 0.5))
            return True
        except Exception as e:  # noqa: BLE001
            self.last_error = f"set_speed_cap: {e}"
            return False

    async def cut_drive(self) -> int:
        """구동 차단 (vehicle.py 정지 감독의 마지막 수단): 시동을 끈다. 시동 끄기가 거부되면 kill (강제 차단).
        시동이 꺼지면 바퀴 속도 명령이 0 이다. 다음 follow 가 다시 시동을 건다 — 정지 감독은 차단 중에 정지 미션을 다시 보내지 않는다."""
        if self._act is None:
            raise DriverError("NO_LINK")
        async with self._cmd:
            try:
                await self._act.disarm()
            except Exception as e:  # noqa: BLE001
                try:
                    await self._act.kill()
                except Exception as e2:  # noqa: BLE001
                    raise DriverError(f"CUT_FAILED: disarm {e}; kill {e2}") from e2
            self.parked_seq = self._prog.mission_seq
            return self._prog.mission_seq

    async def halt_in_place(self) -> int:
        """이미 거의 멈춘 차의 '제자리 정지'. MAVLink HOLD(action.hold) 는 보내지 않는다:
        이 rover(ackermann)에서 HOLD 는 정지가 아니라 주행을 일으켰다 — 2026-10-10 Gazebo 에서 정차 중인 차에 HOLD 를 보내자
        3.8 m/s 로 26분 동안 약 6 km 를 달렸다 (통신 단절 정지 처리가 정차 차량에 정지 명령을 보낸 경우).
        미션이 끝나 멈춰 있으면 아무것도 보내지 않는다 (미션 끝 정차 상태 유지). 아직 미션을 따라 움직이는 중이면
        지금 위치 한 점짜리 미션으로 바꿔 그 자리에서 끝나게 한다."""
        async with self._cmd:
            seq = self._prog.mission_seq + 1
            tel = self.telemetry()
            prog = self._prog
            if tel is not None and (prog.finished or prog.total == 0) and tel.speed_mps < 0.3:
                self._prog = Progress(prog.current, prog.total, True, seq)
                return seq
        if tel is None:
            raise DriverError("TELEMETRY_UNAVAILABLE")
        return await self.follow([(tel.x, tel.y, 1.0)])
