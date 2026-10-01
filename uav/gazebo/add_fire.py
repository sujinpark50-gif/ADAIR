#!/usr/bin/env python3
"""강원 월드에 산불(파티클 불꽃·연기)을 넣은 kangwon_fire.sdf 를 만든다. 시연용.

  입력: kangwon.sdf, dem_local.tif (build_world.py 산출물)
  출력: kangwon_fire.sdf, models/kangwon/materials/textures/{fire,smoke,fire_colors,smoke_colors}.png

화재 중심은 관측 목표(38.0425, 128.2541). 불꽃 FIRE_N 개를 반경 FIRE_R 안에, 연기 SMOKE_N 개를 중심 근처에 둔다.
파티클 시스템은 월드가 아니라 server_fire.config 로 켠다 — 월드에 <plugin> 을 적으면 Gazebo 가
PX4 기본 시스템 목록(server.config)을 읽지 않아 물리·센서가 빠진다.

실행: python3 uav/gazebo/add_fire.py   (GDAL, numpy 필요)
"""
import math
import os
import random

import numpy as np
from osgeo import gdal, osr

gdal.UseExceptions()
HERE = os.path.dirname(os.path.abspath(__file__))
TEX_DIR = f"{HERE}/models/kangwon/materials/textures"
TEX_IN_CONTAINER = "/opt/px4-gazebo/share/gz/models/kangwon/materials/textures"

DATUM_LAT, DATUM_LON, DATUM_ALT = 38.011200, 128.122643, 172.1   # build_world.py 와 동일
FIRE_LAT, FIRE_LON = 38.0425, 128.2541
FIRE_N, FIRE_R = 8, 300.0
SMOKE_N = 3


def local_xy(lat, lon):
    s = osr.SpatialReference(); s.ImportFromEPSG(4326); s.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    t = osr.SpatialReference()
    t.ImportFromProj4(f"+proj=tmerc +lat_0={DATUM_LAT} +lon_0={DATUM_LON} +k=1 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs")
    t.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    return osr.CoordinateTransformation(s, t).TransformPoint(lon, lat)[:2]


class Ground:
    """월드 z (지면). build_world.py 의 곡률 보정·datum 오프셋을 그대로 적용한다."""
    def __init__(self):
        ds = gdal.Open(f"{HERE}/dem_local.tif")
        self.z, self.gt = ds.ReadAsArray(), ds.GetGeoTransform()

    def __call__(self, x, y):
        c = int((x - self.gt[0]) / self.gt[1]); r = int((y - self.gt[3]) / self.gt[5])
        return float(self.z[r, c]) - (x * x + y * y) / (2 * 6371000.0) - DATUM_ALT


def write_png(path, rgba):
    h, w, _ = rgba.shape
    m = gdal.GetDriverByName("MEM").Create("", w, h, 4, gdal.GDT_Byte)
    for i in range(4):
        m.GetRasterBand(i + 1).WriteArray(rgba[..., i])
    gdal.GetDriverByName("PNG").CreateCopy(path, m)


def make_textures():
    os.makedirs(TEX_DIR, exist_ok=True)
    n = 128
    yy, xx = np.mgrid[0:n, 0:n] / (n - 1) * 2 - 1
    r = np.hypot(xx, yy)
    rng = np.random.default_rng(0)
    # 불꽃: 가운데가 밝고 가장자리가 투명한 부드러운 덩어리
    a = np.clip(1 - r, 0, 1) ** 1.6
    blob = np.dstack([np.full((n, n), 255)] * 3 + [a * 255]).astype(np.uint8)
    write_png(f"{TEX_DIR}/fire.png", blob)
    # 연기: 가장자리를 불규칙하게
    noise = rng.random((n // 8, n // 8)).repeat(8, 0).repeat(8, 1)
    a = np.clip(1 - r * (0.8 + 0.4 * noise), 0, 1) ** 1.2
    write_png(f"{TEX_DIR}/smoke.png", np.dstack([np.full((n, n), 255)] * 3 + [a * 255]).astype(np.uint8))

    def ramp(stops, w=256):
        t = np.linspace(0, 1, w)
        ch = [np.interp(t, [s[0] for s in stops], [s[i] for s in stops]) for i in (1, 2, 3, 4)]
        return np.dstack(ch).astype(np.uint8).repeat(4, 0)   # (4, w, 4)
    # 수명에 따른 색: 흰노랑 → 주황 → 빨강 → 검붉게 사라짐
    write_png(f"{TEX_DIR}/fire_colors.png", ramp([
        (0.00, 255, 250, 200, 255), (0.20, 255, 200, 60, 255), (0.50, 255, 110, 20, 230),
        (0.80, 200, 40, 10, 150), (1.00, 60, 20, 10, 0)]))
    write_png(f"{TEX_DIR}/smoke_colors.png", ramp([
        (0.00, 70, 60, 55, 200), (0.40, 110, 105, 100, 170), (1.00, 170, 170, 170, 0)]))


def emitter(name, x, y, z, *, size, psize, lifetime, rate, vmin, vmax, scale_rate, tex, colors):
    # 파티클은 이미터 +X 방향으로 나간다 → pitch -90° 로 위를 향하게. box 크기의 x 가 세로가 된다.
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.1f} {y:.1f} {z:.1f} 0 0 0</pose>
      <link name="link">
        <particle_emitter name="{name}_emitter" type="box">
          <pose>0 0 0 0 -1.5708 0</pose>
          <emitting>true</emitting>
          <size>2 {size} {size}</size>
          <particle_size>{psize} {psize} {psize}</particle_size>
          <lifetime>{lifetime}</lifetime>
          <rate>{rate}</rate>
          <min_velocity>{vmin}</min_velocity>
          <max_velocity>{vmax}</max_velocity>
          <scale_rate>{scale_rate}</scale_rate>
          <material>
            <diffuse>1 1 1</diffuse>
            <pbr><metal><albedo_map>{TEX_IN_CONTAINER}/{tex}</albedo_map></metal></pbr>
          </material>
          <color_range_image>{TEX_IN_CONTAINER}/{colors}</color_range_image>
        </particle_emitter>
      </link>
    </model>"""


def main():
    make_textures()
    ground = Ground()
    cx, cy = local_xy(FIRE_LAT, FIRE_LON)
    random.seed(11)
    models = []
    for i in range(FIRE_N):
        rr = FIRE_R * math.sqrt(random.random()) if i else 0.0
        a = random.uniform(0, 2 * math.pi)
        x, y = cx + rr * math.cos(a), cy + rr * math.sin(a)
        models.append(emitter(f"fire_{i + 1}", x, y, ground(x, y), size=45, psize=18, lifetime=2.5, rate=35,
                              vmin=6, vmax=14, scale_rate=4, tex="fire.png", colors="fire_colors.png"))
    for i in range(SMOKE_N):
        x, y = cx + random.uniform(-150, 150), cy + random.uniform(-150, 150)
        models.append(emitter(f"smoke_{i + 1}", x, y, ground(x, y) + 30, size=80, psize=45, lifetime=22, rate=4,
                              vmin=5, vmax=9, scale_rate=5, tex="smoke.png", colors="smoke_colors.png"))

    sdf = open(f"{HERE}/kangwon.sdf", encoding="utf-8").read()
    sdf = sdf.replace('<world name="kangwon">', '<world name="kangwon_fire">')
    i = sdf.index("<light ")
    sdf = sdf[:i] + "\n".join(models).strip() + "\n    " + sdf[i:]
    with open(f"{HERE}/kangwon_fire.sdf", "w", encoding="utf-8") as f:
        f.write(sdf)
    print(f"kangwon_fire.sdf: 불꽃 {FIRE_N}, 연기 {SMOKE_N} — 중심 월드좌표 ({cx:.0f}, {cy:.0f}, {ground(cx, cy):.0f})")


if __name__ == "__main__":
    main()
