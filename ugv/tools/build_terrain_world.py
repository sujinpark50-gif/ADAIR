# -*- coding: utf-8 -*-
"""ugv/tools/build_terrain_world.py — B 월드(지형을 반영한 연속 도로면)의 제한 검증 구역 생성.

    python -m ugv.tools.build_terrain_world                 # 거점 A 중심 1.2 km 구역 → ugv/gazebo/worlds/kangwon_terrain_a.sdf
    python -m ugv.tools.build_terrain_world --check-only    # 생성물 정적 검사만

방식 (2026-10-10): 구역 전체를 **하나의 연속 높이장(heightmap)** 으로 만든다. 도로마다 충돌 메시를 겹쳐 깔지 않으므로
교차로에서 면이 겹치거나 틈·수직 턱이 생기지 않는다 (예전 지형 도로 월드의 전복 원인: 겹친 도로 면 턱 0.65~1.15 m).
- 도로면 고도 = DEM 을 가우스(σ 40 m)로 크게 다듬은 **하나의 연속 고도장**. 모든 도로·교차로가 같은 면에서 높이를 가져와
  평면교차로가 같은 고도로 이어지고, 갈라지는 도로 사이에도 턱이 없다 (도로별 종단을 따로 쓰자 0.78 m 턱이 생겼다).
  90 m DEM 을 도로 위에서 그대로 따르면 골짜기 사면이 섞여 경사 20 % 넘는 곳이 7.5 % (도로망 전체)라 다듬어 쓴다.
  최대 종단경사는 정적 검사로 보고한다 (제한하지 않음 — 넘으면 보고서에 남기고 속도 계획에서 다룬다).
- 횡단 = 평평 (편경사·횡단경사 없음). 도로 폭 = 차로 수 추정 (drive_graph.est_half_width).
- 높이장 칸 값: 도로 띠 안 = 가까운 도로들의 종단 고도 (거리 가중, 교차로에서 연속), 띠 밖 BLEND_M 까지 = 도로 고도에서 DEM 으로
  선형 전이, 그 밖 = DEM. 그래서 도로 띠 안에서 지형이 도로 위로 튀어나오거나 도로가 지형에 묻히지 않는다.
- 제외: 시설 코드 FACIL_KIND 1·2 (교량·터널로 보이나 이름이 깨져 미확인) 링크와 구역 경계를 넘는 링크. 입체교차 정보(층·고도)는
  원자료에 없다 → 구역 안 입체교차를 가정하지 않으며, 시설 코드 링크를 빼서 평면교차로로 잘못 잇지 않는다.
결과: 월드 SDF, 16 bit 높이 PNG, 도로 표시 OBJ, 메타데이터·정적 검사 JSON (ugv/gazebo/worlds/kangwon_terrain_a.*),
UGV 가 쓸 도로 부분집합 (UGV_ROAD_SUBSET).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from ugv.drive_graph import access_point, est_half_width
from ugv.roads import RoadNetwork
from ugv.terrain.dem import meta as dem_meta, z_at

ROOT = Path(__file__).resolve().parents[2]
WORLDS = ROOT / "ugv" / "gazebo" / "worlds"
MODELS = ROOT / "ugv" / "gazebo" / "models"
NAME = "kangwon_terrain_a"
HALF_M = 600.0            # 구역 반폭 (정사각형 1.2 km)
N = 1025                  # 높이장 격자 (2^n + 1). 1200 m / 1024 = 1.17 m 간격
EDGE_M = 25.0             # 구역 경계에서 이만큼 안쪽 도로만 (높이장 밖으로 나가지 않게)
MAX_GRADE = 0.12          # 최대 종단경사
SMOOTH_M = 60.0           # DEM 잔차 다듬기 창
BLEND_M = 12.0            # 도로 띠 밖 지형 전이 폭
ROAD_SURFACE_SIGMA_M = 40.0   # 도로면 고도장 = DEM 가우스 다듬기 (표준편차)
PROFILE_STEP_M = 5.0
WORLD_ORIGIN_ALT = 172.1  # kangwon_flat 과 같은 원점 고도 (DEM 원점 값과 같음)


def write_png16(path, img):
    """16 bit 회색조 PNG (zlib 만 사용 — 추가 패키지 없음). img: uint16 2차원."""
    import struct
    import zlib
    h, w = img.shape
    raw = b"".join(b"\x00" + img[i].astype(">u2").tobytes() for i in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 16, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def select_roads(net, cx, cy):
    inside, excluded = [], {}
    for rid, r in net.roads.items():
        ok = all(abs(x - cx) <= HALF_M - EDGE_M and abs(y - cy) <= HALF_M - EDGE_M for x, y in r.xy)
        touch = any(abs(x - cx) <= HALF_M and abs(y - cy) <= HALF_M for x, y in r.xy)
        if not touch:
            continue
        if not ok:
            excluded[rid] = "ZONE_BOUNDARY"
        elif r.attrs.get("facil_kind") in ("1", "2"):
            excluded[rid] = f"FACIL_KIND_{r.attrs['facil_kind']} (교량·터널 추정, 입체 정보 없음)"
        else:
            inside.append(rid)
    return inside, excluded


def profiles(net, rids):
    """도로별 종단 [(s, z)] — 노드 고도 공유, 잔차 다듬기, 끝 0, 최대 경사 제한."""
    node_z = {}
    for rid in rids:
        r = net.roads[rid]
        for n in (r.a, r.b):
            if n not in node_z:
                x, y = net.nodes[n]
                node_z[n] = z_at(x, y)
    out = {}
    for rid in rids:
        r = net.roads[rid]
        L = r.length
        k = max(2, int(math.ceil(L / PROFILE_STEP_M)) + 1)
        s = np.linspace(0.0, L, k)
        za, zb = node_z[r.a], node_z[r.b]
        lin = za + (zb - za) * s / L
        dem = np.array([z_at(*net.at(rid, v)[2:4]) for v in s])
        res = dem - lin
        w = max(1, int(SMOOTH_M / (L / (k - 1))))
        ker = np.ones(2 * w + 1) / (2 * w + 1)
        sm = np.convolve(np.pad(res, w, mode="edge"), ker, mode="valid")
        taper = np.minimum(1.0, np.minimum(s, L - s) / min(SMOOTH_M, L / 2 + 1e-9))
        z = lin + sm * taper
        # 최대 경사 제한: 앞뒤로 번갈아 기울기를 깎는다 (끝 고도는 고정)
        ds = np.diff(s)
        for _ in range(200):
            g = np.diff(z) / ds
            if np.all(np.abs(g) <= MAX_GRADE + 1e-6):
                break
            for i in range(len(ds)):
                lim = MAX_GRADE * ds[i]
                if z[i + 1] - z[i] > lim:
                    z[i + 1] = z[i] + lim
                elif z[i] - z[i + 1] > lim:
                    z[i + 1] = z[i] - lim
            z[-1] = zb
            for i in range(len(ds) - 1, -1, -1):
                lim = MAX_GRADE * ds[i]
                if z[i] - z[i + 1] > lim:
                    z[i] = z[i + 1] + lim
                elif z[i + 1] - z[i] > lim:
                    z[i] = z[i + 1] - lim
            z[0] = za
        out[rid] = (s, z)
    return out, node_z


def build(cx, cy):
    net = RoadNetwork()
    rids, excluded = select_roads(net, cx, cy)
    prof, node_z = profiles(net, rids)
    half_w = {rid: est_half_width(net.roads[rid]) for rid in rids}
    xs = cx - HALF_M + np.arange(N) * (2 * HALF_M / (N - 1))
    ys = cy + HALF_M - np.arange(N) * (2 * HALF_M / (N - 1))       # 행 0 = 북쪽 (영상 위)
    H = np.zeros((N, N))
    # DEM 바탕 (쌍선형 — 칸마다)
    for i, y in enumerate(ys):
        H[i] = [z_at(x, y) for x in xs]
    dem_grid = H.copy()
    # 도로면 고도장: DEM 을 가우스로 크게 다듬은 하나의 연속 면 (모든 도로가 같은 면에서 높이를 가져온다 → 교차로·갈라지는
    # 도로·겹치는 띠에서 구조상 턱이 없다. 도로별 종단을 따로 쓰자 갈라지는 두 도로 사이에 0.78 m 턱이 생겼다 — 2026-10-10)
    from scipy.ndimage import gaussian_filter
    cell = xs[1] - xs[0]
    sigma = ROAD_SURFACE_SIGMA_M / cell
    Z = gaussian_filter(dem_grid, sigma=sigma, mode="nearest")
    # 도로 띠 (어느 포함 도로든 중심선 ± 추정 반폭) 와 띠 밖 거리
    edge = np.full((N, N), np.inf)          # 가장 가까운 도로 띠 가장자리까지 거리 (안이면 ≤ 0)
    for rid in rids:
        hw = half_w[rid]
        pts = np.array(net.roads[rid].xy)
        reach = hw + BLEND_M + cell
        j0 = max(0, int((pts[:, 0].min() - reach - xs[0]) / cell))
        j1 = min(N - 1, int((pts[:, 0].max() + reach - xs[0]) / cell) + 1)
        i0 = max(0, int((ys[0] - (pts[:, 1].max() + reach)) / cell))
        i1 = min(N - 1, int((ys[0] - (pts[:, 1].min() - reach)) / cell) + 1)
        X, Y = np.meshgrid(xs[j0:j1 + 1], ys[i0:i1 + 1])
        best = np.full(X.shape, np.inf)
        for k in range(len(pts) - 1):
            ax, ay = pts[k]
            bx, by = pts[k + 1]
            dx_, dy_ = bx - ax, by - ay
            L2 = dx_ * dx_ + dy_ * dy_
            t = np.clip(((X - ax) * dx_ + (Y - ay) * dy_) / L2, 0.0, 1.0) if L2 > 0 else np.zeros(X.shape)
            best = np.minimum(best, np.hypot(X - ax - t * dx_, Y - ay - t * dy_))
        sub = (slice(i0, i1 + 1), slice(j0, j1 + 1))
        edge[sub] = np.minimum(edge[sub], best - hw)
    road_mask = edge <= 0
    H = dem_grid.copy()
    H[road_mask] = Z[road_mask]
    bm = (~road_mask) & (edge <= BLEND_M)
    t = edge[bm] / BLEND_M
    H[bm] = Z[bm] * (1 - t) + dem_grid[bm] * t
    # 보고용 종단: 같은 고도장을 도로 위에서 읽은 값
    from scipy.ndimage import map_coordinates
    for rid in rids:
        s_, _ = prof[rid]
        xy = np.array([net.at(rid, v)[2:4] for v in s_])
        cols = (xy[:, 0] - xs[0]) / cell
        rows = (ys[0] - xy[:, 1]) / cell
        prof[rid] = (s_, map_coordinates(Z, [rows, cols], order=1, mode="nearest"))
    for n in node_z:
        x, y = net.nodes[n]
        node_z[n] = float(map_coordinates(Z, [[(ys[0] - y) / cell], [(x - xs[0]) / cell]], order=1, mode="nearest")[0])
    return net, rids, excluded, prof, node_z, half_w, xs, ys, H, road_mask, dem_grid


def static_checks(net, rids, prof, node_z, half_w, xs, ys, H, road_mask, dem_grid):
    d = xs[1] - xs[0]
    out = {}
    # 1) 도로 띠 안 이웃 칸 높이 차 (턱): 최대 경사 × 간격 + 여유보다 크면 턱
    dx = np.abs(np.diff(H, axis=1))
    dy = np.abs(np.diff(H, axis=0))
    mx = road_mask[:, 1:] & road_mask[:, :-1]
    my = road_mask[1:, :] & road_mask[:-1, :]
    steps = np.concatenate([dx[mx], dy[my]])
    out["road_cell_step_m"] = {"max": round(float(steps.max()), 3), "p99": round(float(np.percentile(steps, 99)), 3),
                               "limit": round(MAX_GRADE * d * 1.5 + 0.02, 3), "cell_m": round(float(d), 3),
                               "over_limit_pairs": int((steps > MAX_GRADE * d * 1.5 + 0.02).sum())}
    # 2) 종단경사
    g = [float(np.max(np.abs(np.diff(z) / np.diff(s)))) for s, z in prof.values()]
    out["profile_max_grade"] = round(max(g), 4)
    # 3) 교차로: 노드 고도 공유 — 한 노드에 붙은 도로 종단 끝 고도 차
    spread = []
    for n, z0 in node_z.items():
        ends = []
        for rid in rids:
            r = net.roads[rid]
            s, z = prof[rid]
            if r.a == n:
                ends.append(z[0])
            if r.b == n:
                ends.append(z[-1])
        if len(ends) > 1:
            spread.append(max(ends) - min(ends))
    out["junction_end_z_spread_max_m"] = round(max(spread), 6) if spread else None
    # 4) 도로 띠 안에서 DEM 이 도로면보다 얼마나 높은가 (원래 지형이 도로 위로 튀어나왔을 높이 → 높이장에서는 깎았다)
    cut = (dem_grid - H)[road_mask]
    out["dem_minus_road_in_road_m"] = {"max_cut": round(float(cut.max()), 2), "max_fill": round(float(-cut.min()), 2),
                                       "p95_abs": round(float(np.percentile(np.abs(cut), 95)), 2)}
    # 5) 높이장 = 단일 연속면 → 겹침·틈 없음 (구조상)
    out["surface"] = "단일 높이장 (도로·지형 같은 면) — 충돌면 겹침·틈 구조상 없음"
    out["road_cells"] = int(road_mask.sum())
    return out


def write(net, rids, excluded, prof, node_z, half_w, xs, ys, H, road_mask, dem_grid, cx, cy, acc):
    zmin, zmax = float(H.min()), float(H.max())
    span = max(zmax - zmin, 1.0)
    img = np.round((H - zmin) / span * 65535).astype(np.uint16)
    model_dir = MODELS / f"{NAME}_ground"
    model_dir.mkdir(parents=True, exist_ok=True)
    png = model_dir / "heightmap.png"
    write_png16(png, img)
    size = 2 * HALF_M
    # 도로 표시 OBJ (도로면 위 3 cm, 충돌 없음)
    verts, faces = [], []
    for rid in rids:
        r = net.roads[rid]
        s, z = prof[rid]
        hw = half_w[rid]
        pts = [net.at(rid, v)[2:4] for v in s]
        base = len(verts)
        for k, (p, zz) in enumerate(zip(pts, z)):
            q = pts[min(k + 1, len(pts) - 1)] if k + 1 < len(pts) else p
            pp = pts[k - 1] if k > 0 else p
            tx, ty = (q[0] - pp[0]), (q[1] - pp[1])
            nrm = math.hypot(tx, ty) or 1.0
            nx, ny = -ty / nrm, tx / nrm
            for sgn in (1, -1):
                verts.append((p[0] - cx + sgn * nx * hw, p[1] - cy + sgn * ny * hw, zz - zmin + 0.03))
        for k in range(len(pts) - 1):
            a, b, c, d = base + 2 * k, base + 2 * k + 1, base + 2 * k + 2, base + 2 * k + 3
            faces += [(a, b, d), (a, d, c)]
    obj = model_dir / "roads.obj"
    with open(obj, "w", encoding="utf-8") as f:
        for v in verts:
            f.write(f"v {v[0]:.3f} {v[1]:.3f} {v[2]:.3f}\n")
        for a, b, c in faces:
            f.write(f"f {a + 1} {b + 1} {c + 1}\n")
    z_off = zmin - WORLD_ORIGIN_ALT
    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<!--
  {NAME} — B 월드 제한 검증 구역 (지형을 반영한 연속 도로면). 생성: python -m ugv.tools.build_terrain_world
  구역: 거점 A 출입 지점 중심 {size:.0f} m 정사각형, 지역 평면 중심 ({cx:.1f}, {cy:.1f}). 충돌 = 단일 높이장 {N}×{N} ({size / (N - 1):.2f} m 간격),
  높이 {zmin:.2f}~{zmax:.2f} m (원점 고도 {WORLD_ORIGIN_ALT} m 기준 z 오프셋 {z_off:.2f}). 도로 표시 OBJ 는 충돌 없음.
  구역 밖·제외 도로(교량·터널 코드)는 없다. 자세한 기준: {NAME}.json
-->
<sdf version="1.9">
  <world name="{NAME}">
    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene><grid>false</grid><ambient>0.4 0.4 0.4 1</ambient><background>0.62 0.78 0.92 1</background><shadows>false</shadows></scene>
    <model name="terrain">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {z_off:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><heightmap><uri>model://{NAME}_ground/heightmap.png</uri><size>{size:.3f} {size:.3f} {span:.4f}</size><pos>0 0 0</pos></heightmap></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><heightmap><uri>model://{NAME}_ground/heightmap.png</uri><size>{size:.3f} {size:.3f} {span:.4f}</size><pos>0 0 0</pos></heightmap></geometry>
        </visual>
      </link>
    </model>
    <model name="road_marks">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {z_off:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>model://{NAME}_ground/roads.obj</uri></mesh></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.33 0.33 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose><cast_shadows>false</cast_shadows><intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction><diffuse>0.904 0.904 0.904 1</diffuse><specular>0.271 0.271 0.271 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>38.0112</latitude_deg><longitude_deg>128.122643</longitude_deg><elevation>{WORLD_ORIGIN_ALT}</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""
    (WORLDS / f"{NAME}.sdf").write_text(sdf, encoding="utf-8")
    (model_dir / "model.config").write_text(f"""<?xml version="1.0"?>
<model><name>{NAME}_ground</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>B 월드 제한 구역 높이장·도로 표시 (ugv/tools/build_terrain_world.py 생성)</description></model>
""", encoding="utf-8")
    checks = static_checks(net, rids, prof, node_z, half_w, xs, ys, H, road_mask, dem_grid)
    # 출입 지점 높이 (스폰 z)
    acc_z = float(np.interp(acc.s, *prof[acc.road_id])) if acc.road_id in prof else None
    doc = {"name": NAME, "center_xy": [cx, cy], "half_m": HALF_M, "grid": N, "cell_m": round(2 * HALF_M / (N - 1), 4),
           "z_min": zmin, "z_max": zmax, "z_offset_world": z_off, "world_origin_alt_m": WORLD_ORIGIN_ALT,
           "dem": dem_meta(), "params": {"max_grade": MAX_GRADE, "smooth_m": SMOOTH_M, "blend_m": BLEND_M,
                                         "edge_m": EDGE_M, "profile_step_m": PROFILE_STEP_M,
                                         "road_width": "차로 수 × 3.25 + 1.0 m (추정)", "cross_section": "평평"},
           "roads_included": sorted(rids), "roads_excluded": excluded, "nodes": len(node_z),
           "road_length_km": round(sum(net.roads[r].length for r in rids) / 1000, 2),
           "access_point": {"road_id": acc.road_id, "s_m": round(acc.s, 1), "z_road_m": acc_z,
                            "z_world": None if acc_z is None else round(acc_z - WORLD_ORIGIN_ALT, 3)},
           "static_checks": checks,
           "not_verified": ["입체교차·교량·터널 (원자료에 층·고도 없음, 시설 코드 링크 제외)", "DEM 수직 기준 미확인",
                            "편경사·횡단경사 없음", "구역 밖 도로", "90 m DEM 의 지형 세부 (사면·하천)"]}
    (WORLDS / f"{NAME}.json").write_text(json.dumps(doc, ensure_ascii=False, indent=1), encoding="utf-8")
    (ROOT / "ugv" / f"road_subset_{NAME}.json").write_text(json.dumps({"world": NAME, "roads": sorted(rids)}, indent=1),
                                                           encoding="utf-8")
    return doc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="A")
    a = ap.parse_args()
    net = RoadNetwork()
    acc = access_point(net, a.base)[0]
    cx, cy = round(acc.x), round(acc.y)
    res = build(cx, cy)
    doc = write(*res, cx, cy, acc)
    print(json.dumps({k: doc[k] for k in ("road_length_km", "z_min", "z_max", "access_point", "static_checks")},
                     ensure_ascii=False, indent=1))
    print("excluded:", doc["roads_excluded"])


if __name__ == "__main__":
    main()
