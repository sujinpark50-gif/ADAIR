# -*- coding: utf-8 -*-
"""지형(종단·횡경사) 반영 속도 계획 후보 — UGV_SPEED_TERRAIN=1 일 때만 roads.plan_speeds 가 쓴다 (기본 off, 2026-10-13).

보수적 계획 방식이다 (완전한 ESC·능동 제동이 아니다). 정지 후보(vehicle UGV_STOP_GRADE)와 같은 유효 감속 모델을 쓴다.
 - 유효 감속 a_eff = max(a − g·하강경사, A_EFF_MIN). 하강경사 = 진행 방향 고도 감소율(양수) + 불확실성.
 - 하강 속도 상한: 지연 포함 정지거리 v·τ + v²/(2·a_eff) 가 같은 계획 속도의 평지 정지거리 v_p·τ + v_p²/(2·a) 를 넘지 않는 v
   (평지보다 정지거리가 길어지지 않게). 감속 통과(뒤→앞)에 a_eff 를 써서 하강 구간·곡선에 들어가기 전에 줄인다.
 - 횡경사: 회전 방향(부호 곡률 + = 왼쪽) 기준 바깥쪽이 낮으면(불리) 허용 횡가속 = max(lat − g·불리 횡경사, lat·LAT_MIN_RATIO).
   유리한 횡경사로 속도를 올리지 않는다.
 - 지형 정보 없음·값 없음(NaN): 보수 정책 — 하강 GRADE_UNKNOWN, 불리 횡경사 BANK_UNKNOWN 을 가정하고 표시.
 - 유효 감속 부족: a − g·하강경사 ≤ A_EFF_MIN 인 지점이 있으면 approach.verify_route 가 경로 거절 (TERRAIN_DECEL_INSUFFICIENT).
 - 모델(평지 감속 − g·경사)은 확정이 아닌 후보다 — 실제 정지점·경계·정지 유지 측정(chain_slope3·chain_xslope)으로만 채택을 판단한다.
영향: 불확실성 0.01 을 항상 더하므로 평지에서도 a_eff 가 약 0.1 m/s² 낮아져 계획 속도가 약 3 % 낮다 (보수 쪽, 비교 시험에서 확인).
상수는 결과 전 고정. τ 2.5 s = 2026-10-13 경사 중단 실측 명령→감속 시작 2.4~3.0 s (속도 상한 명령 후보 검증 뒤 갱신 대상).
"""
import math
import os

G = 9.8
TAU_S = float(os.getenv("UGV_SPEED_TERRAIN_TAU_S", "2.5"))
GRADE_UNC = 0.01
GRADE_UNKNOWN = 0.06
BANK_UNKNOWN = 0.03
A_EFF_MIN = 0.5
LAT_MIN_RATIO = 0.5
BANK_HALF_M = 3.0
ENABLED = os.getenv("UGV_SPEED_TERRAIN", "0") == "1"
_SURF = None


def surface():
    """UGV_TERRAIN_TILES 지형 (없으면 None)."""
    global _SURF
    if _SURF is None:
        d = os.getenv("UGV_TERRAIN_TILES")
        _SURF = False
        if d:
            try:
                from .tools.terrain_compare import Surf
                _SURF = Surf(d)
            except Exception:  # noqa: BLE001
                _SURF = False
    return _SURF or None


def a_eff(decel, grade_down):
    return max(decel - G * max(grade_down, 0.0), A_EFF_MIN)


def stop_dist(v, decel_eff, tau=TAU_S):
    return v * tau + v * v / (2 * decel_eff)


def v_for_stop_dist(s, decel_eff, tau=TAU_S):
    """v·τ + v²/(2a) = s 의 양의 해."""
    return -tau * decel_eff + math.sqrt((tau * decel_eff) ** 2 + 2 * decel_eff * max(s, 0.0))


def profile_along(pts, cum, signed_curv):
    """점마다 (하강경사, 불리 횡경사, 지형 있음). signed_curv(i) = 부호 곡률 (+ 왼쪽)."""
    surf = surface()
    out = []
    n = len(pts)
    for i in range(n):
        j0, j1 = max(i - 1, 0), min(i + 1, n - 1)
        if surf is None or j1 == j0:
            out.append((GRADE_UNKNOWN, BANK_UNKNOWN, False))
            continue
        (ax, ay), (bx, by) = pts[j0], pts[j1]
        L = math.hypot(bx - ax, by - ay) or 1.0
        tx, ty = (bx - ax) / L, (by - ay) / L
        lx, ly = -ty, tx                                  # 왼쪽 법선
        x, y = pts[i]
        try:
            za, zb = surf.z(ax, ay), surf.z(bx, by)
            zl, zr = surf.z(x + lx * BANK_HALF_M, y + ly * BANK_HALF_M), surf.z(x - lx * BANK_HALF_M, y - ly * BANK_HALF_M)
        except Exception:  # noqa: BLE001
            za = zb = zl = zr = float("nan")
        if any(v != v for v in (za, zb, zl, zr)):
            out.append((GRADE_UNKNOWN, BANK_UNKNOWN, False))
            continue
        grade_down = max(0.0, (za - zb) / L) + GRADE_UNC
        cross = (zl - zr) / (2 * BANK_HALF_M)              # + = 왼쪽이 높음
        k = signed_curv(i)
        # 왼쪽 회전(k>0) 바깥 = 오른쪽: 오른쪽이 낮으면(cross>0) 불리. 오른쪽 회전 바깥 = 왼쪽: cross<0 이면 불리
        adverse = max(0.0, cross if k > 0 else -cross if k < 0 else 0.0)
        out.append((grade_down, adverse, True))
    return out
