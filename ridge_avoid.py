# -*- coding: utf-8 -*-
"""
ridge_avoid.py  ·  경로 중간 능선 회피 (강원 DEM 기반)
=====================================================
직선 goto 는 타깃 지형고도만 보고 순항고도를 잡아, 경로 중간의 능선(태백산맥)에
충돌할 수 있다. 이 모듈은 출발→타깃 직선 경로를 DEM 으로 샘플해
  1) 경로 최고 지형(능선)을 찾고,
  2) 안전 순항고도 = 경로최고 + 여유 를 계산하며,
  3) (선택) 지형 추종 웨이포인트를 만든다.

gz_bridge 의 격자 transform·terrain_elev 를 재사용한다(같은 datum·DEM).
검증 예: 계곡(5,121)→능선 너머(120,40) — 경로 최고 ~809 m,
직선 순항 545 m 는 능선을 ~265 m 관통(충돌), 안전 순항 859 m 로 회피.
"""
from __future__ import annotations
from typing import List, Tuple
import gz_bridge as gb


def path_cells(c0, r0, c1, r1, n: int = 60) -> List[Tuple[float, float]]:
    """출발셀→타깃셀 직선을 n개 격자점으로 샘플."""
    return [(c0 + (c1 - c0) * i / (n - 1),
             r0 + (r1 - r0) * i / (n - 1)) for i in range(n)]


def terrain_profile(c0, r0, c1, r1, n: int = 60) -> List[float]:
    """경로를 따른 지형고도(m) 프로파일."""
    return [gb.terrain_elev(round(c), round(r)) for c, r in path_cells(c0, r0, c1, r1, n)]


def safe_cruise_alt(c0, r0, c1, r1, clearance_m: float = 50.0, n: int = 60):
    """경로 최고 능선 + 여유 = 충돌 없는 최소 순항고도(AMSL).
    반환: dict(ridge_max, safe_alt, naive_alt, collide, worst_margin)."""
    prof = terrain_profile(c0, r0, c1, r1, n)
    ridge = max(prof)
    naive = prof[-1] + clearance_m               # 직선 goto: 타깃 지형+여유
    safe = ridge + clearance_m                    # 능선 회피: 경로 최고+여유
    worst = min(naive - e for e in prof)          # 직선 순항 시 최소 지형이격
    return {
        "ridge_max": ridge, "safe_alt": safe, "naive_alt": naive,
        "collide": worst < clearance_m, "worst_margin": worst, "profile": prof,
    }


def climb_waypoints(c0, r0, c1, r1, clearance_m: float = 50.0, n: int = 8):
    """지형 추종 웨이포인트: 각 지점을 (지형+여유) 높이로 넘는다.
    반환: [(lat, lon, alt_amsl), ...]. 능선 구간에서 자연히 상승."""
    cells = path_cells(c0, r0, c1, r1, n)
    wps = []
    for c, r in cells:
        lat, lon = gb.grid_cell_to_latlon(round(c), round(r))
        alt = gb.terrain_elev(round(c), round(r)) + clearance_m
        wps.append((round(lat, 6), round(lon, 6), round(alt, 1)))
    # 단조 안전화(옵션): 다음 목표가 더 높으면 미리 올려 접근 — 여기선 원값 유지
    return wps


if __name__ == "__main__":
    r = safe_cruise_alt(5, 121, 120, 40, clearance_m=50.0)
    print(f"경로 최고 능선 : {r['ridge_max']:.0f} m")
    print(f"직선 goto 순항 : {r['naive_alt']:.0f} m  (최소 지형이격 {r['worst_margin']:+.0f} m"
          f" → {'충돌' if r['collide'] else 'OK'})")
    print(f"능선회피 안전순항: {r['safe_alt']:.0f} m  (직선 대비 +{r['safe_alt']-r['naive_alt']:.0f} m)")
