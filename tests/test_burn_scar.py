# -*- coding: utf-8 -*-
"""burn_scar_s2.py 계산부 시험 — 네트워크 없이 합성 Sentinel-2 장면(UTM 52N, 10 m)으로 검증한다.

알려진 크기의 '탄 원'을 남전약수터 근처에 그려 넣고, 격자 재투영 → dNBR → 연결 성분 → 면적이
기하 면적과 맞는지, 구름 칸을 판정 불가로 두는지, 먼 곳의 변화는 버리는지 본다.
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

rasterio = pytest.importorskip("rasterio")
pytest.importorskip("scipy")
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("burn", ROOT / "tools" / "burn_scar_s2.py")
burn = importlib.util.module_from_spec(spec); spec.loader.exec_module(burn)


@pytest.fixture(scope="module")
def G():
    return burn.load_grid()


def scene(tmp, G, name, burned_r_m=0, cloud=False, far_change=False):
    from rasterio.transform import from_origin
    from rasterio.warp import transform as warp
    x0, y0 = warp("EPSG:4326", "EPSG:32652", [128.08], [38.07])
    x1, y1 = warp("EPSG:4326", "EPSG:32652", [128.25], [37.95])
    res = 10.0
    W, H = int((x1[0] - x0[0]) / res), int((y0[0] - y1[0]) / res)
    tr = from_origin(x0[0], y0[0], res, res)
    sx, sy = warp("EPSG:4326", "EPSG:32652", [burn.SPRING[1]], [burn.SPRING[0]])
    cx, cy = (sx[0] - x0[0]) / res + 60, (y0[0] - sy[0]) / res - 40       # 약수터 동북쪽 약 700 m 가 중심
    yy, xx = np.mgrid[0:H, 0:W]
    b08 = np.full((H, W), 3000, np.uint16); b12 = np.full((H, W), 1200, np.uint16)   # 건강한 숲 NBR≈0.43
    if burned_r_m:
        m = np.hypot(xx - cx, yy - cy) * res <= burned_r_m
        b08[m], b12[m] = 1500, 2200                                          # 탄 곳 NBR≈-0.19
        if far_change:
            fx = W - 200
            m2 = np.hypot(xx - fx, yy - 200) * res <= 300
            b08[m2], b12[m2] = 1500, 2200
    scl = np.full((H, W), 4, np.uint8)
    if cloud:
        scl[:, : W // 8] = 9
    out = {}
    for band, arr in (("B08", b08), ("B12", b12), ("SCL", scl)):
        p = tmp / f"{name}_{band}.tif"
        with rasterio.open(p, "w", driver="GTiff", width=W, height=H, count=1, dtype=arr.dtype,
                           crs="EPSG:32652", transform=tr) as d:
            d.write(arr, 1)
        out[band] = str(p)
    return out


def read(paths, G):
    return (burn.warp_to_grid(paths["B08"], G), burn.warp_to_grid(paths["B12"], G),
            burn.warp_to_grid(paths["SCL"], G, resampling="nearest"))


def test_area_matches_geometry(tmp_path, G):
    r = 1000.0                                                  # 반경 1 km 원 = 314 ha (실제 342 ha 와 비슷한 크기)
    pre = read(scene(tmp_path, G, "pre"), G)
    post = read(scene(tmp_path, G, "post", burned_r_m=r, far_change=True), G)
    mask, dnbr, info = burn.burn_mask(pre, post, G)
    expect = math.pi * r * r / 10_000
    assert abs(info["area_ha"] - expect) / expect < 0.12       # 90 m 칸 경계 효과 범위 안
    assert info["components_kept"] == 1                        # 먼 곳(약 10 km)의 변화는 버림


def test_cloud_cells_are_unknown_not_burned(tmp_path, G):
    pre = read(scene(tmp_path, G, "pre"), G)
    post = read(scene(tmp_path, G, "postc", burned_r_m=800, cloud=True), G)
    mask, _, info = burn.burn_mask(pre, post, G)
    assert (mask == 255).any() and info["unknown_cells_near_fire"] > 0


def test_no_change_no_burn(tmp_path, G):
    pre = read(scene(tmp_path, G, "pre"), G)
    mask, _, info = burn.burn_mask(pre, pre, G)
    assert info["area_ha"] == 0.0


def test_wide_window_finds_scar_outside_ca_grid(tmp_path, G):
    """실제 피해지가 CA 격자 밖(발화점 서쪽)에 있으면: CA 격자 판정은 놓치고, 넓은 창 진단은 찾아 '격자 안 0%'로 알린다."""
    from rasterio.transform import from_origin
    from rasterio.warp import transform as warp
    W = burn.wide_grid()
    res = 10.0
    x0, y0 = warp("EPSG:4326", "EPSG:32652", [128.03], [38.10])
    x1, y1 = warp("EPSG:4326", "EPSG:32652", [128.20], [37.96])
    Wd, Hd = int((x1[0] - x0[0]) / res), int((y0[0] - y1[0]) / res)
    tr = from_origin(x0[0], y0[0], res, res)
    bx, by = warp("EPSG:4326", "EPSG:32652", [128.100], [38.040])          # 발화점 서북서 약 3 km
    cx, cy = (bx[0] - x0[0]) / res, (y0[0] - by[0]) / res
    yy, xx = np.mgrid[0:Hd, 0:Wd]
    def write(name, burned):
        b08 = np.full((Hd, Wd), 3000, np.uint16); b12 = np.full((Hd, Wd), 1200, np.uint16)
        if burned:
            m = np.hypot(xx - cx, yy - cy) * res <= 1000
            b08[m], b12[m] = 1500, 2200
        paths = {}
        for band, arr in (("B08", b08), ("B12", b12), ("SCL", np.full((Hd, Wd), 4, np.uint8))):
            p = tmp_path / f"{name}_{band}.tif"
            with rasterio.open(p, "w", driver="GTiff", width=Wd, height=Hd, count=1, dtype=arr.dtype,
                               crs="EPSG:32652", transform=tr) as d:
                d.write(arr, 1)
            paths[band] = str(p)
        return paths
    pre_p, post_p = write("pre", False), write("post", True)
    _, _, info_ca = burn.burn_mask(read(pre_p, G), read(post_p, G), G)
    assert info_ca["area_ha"] < 20                                  # CA 격자에서는 거의 안 보인다
    mask, _, info_w = burn.burn_mask(read(pre_p, W), read(post_p, W), W, radius_km=8.0, min_cells=3)
    comps = burn.describe_components(mask, W, G)
    assert abs(comps[0]["area_ha"] - 314) / 314 < 0.12
    assert comps[0]["inside_ca_grid"] < 0.05 and 250 <= comps[0]["bearing_deg"] <= 320
