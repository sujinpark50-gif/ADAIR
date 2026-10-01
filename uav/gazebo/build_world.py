#!/usr/bin/env python3
"""강원(원통-기린) DEM -> Gazebo(gz sim Harmonic) heightmap 월드 생성.

좌표 기준은 Codyssey_M2-1/gz_bridge.py 의 datum 과 동일:
  월드 원점(0,0,0) = DATUM_LAT/LON, 월드 z=0 = 해발 DATUM_ALT
단, DEM 은 EPSG:5186 격자가 아니라 datum 중심 TM(=gz 의 로컬 ENU 와 거의 동일)으로
재투영한다. 5186 격자북은 이 지역에서 진북과 약 0.7° 어긋나 가장자리에서 100m+ 오차가 난다.

출력: heightmap.png(16bit), texture.png, normal.png, kangwon.sdf, spawn.env
"""
import json
import os
import subprocess

import numpy as np
from osgeo import gdal, osr

gdal.UseExceptions()

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))   # uav/gazebo -> 저장소 루트
SRC_DEM = f"{REPO}/environment/data/processed/gangwon/dem_gangwon.tif"   # 넓은 90m DEM (원통 포함)
SCN_DEM = f"{REPO}/environment/data/processed/dem_clipped.tif"           # 시나리오 영역

# gz_bridge.py 와 동일 datum
DATUM_LAT, DATUM_LON, DATUM_ALT = 38.011200, 128.122643, 172.1
LOCAL = (f"+proj=tmerc +lat_0={DATUM_LAT} +lon_0={DATUM_LON} +k=1 "
         "+x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs")

SPAWN_NAME, SPAWN_LAT, SPAWN_LON = "원통119", 38.1205, 128.2018
RES_PX = 513          # gz heightmap 은 정사각형 2^n+1 이어야 함 (513 -> 약 55 m/px)
MARGIN_M = 500


def to_local(pts_ll):
    s = osr.SpatialReference(); s.ImportFromEPSG(4326)
    s.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    t = osr.SpatialReference(); t.ImportFromProj4(LOCAL)
    ct = osr.CoordinateTransformation(s, t)
    return [ct.TransformPoint(lon, lat)[:2] for lat, lon in pts_ll]


def scenario_corners_ll():
    ds = gdal.Open(SCN_DEM)
    gt = ds.GetGeoTransform()
    w, h = ds.RasterXSize, ds.RasterYSize
    src = osr.SpatialReference(); src.ImportFromWkt(ds.GetProjection())
    src.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    dst = osr.SpatialReference(); dst.ImportFromEPSG(4326)
    dst.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    ct = osr.CoordinateTransformation(src, dst)
    out = []
    for px, py in [(0, 0), (w, 0), (0, h), (w, h)]:
        x = gt[0] + px * gt[1]; y = gt[3] + py * gt[5]
        lon, lat = ct.TransformPoint(x, y)[:2]
        out.append((lat, lon))
    return out


def main():
    # 1) 시나리오 영역 + 스폰지점을 모두 덮는 정사각형 범위(로컬 m)
    pts = to_local(scenario_corners_ll() + [(SPAWN_LAT, SPAWN_LON)])
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    side = max(max(xs) - min(xs), max(ys) - min(ys)) + 2 * MARGIN_M
    te = (cx - side / 2, cy - side / 2, cx + side / 2, cy + side / 2)

    # 2) 재투영 + 리샘플
    # heightmap 은 꼭짓점 격자(간격 side/(N-1))라 DEM 픽셀 중심이 꼭짓점에 오도록 반 픽셀 확장
    d = side / (RES_PX - 1)
    dem_local = f"{HERE}/dem_local.tif"
    gdal.Warp(dem_local, SRC_DEM, dstSRS=LOCAL,
              outputBounds=(te[0] - d / 2, te[1] - d / 2, te[2] + d / 2, te[3] + d / 2),
              width=RES_PX, height=RES_PX, resampleAlg="bilinear",
              srcNodata=-9999, dstNodata=-9999, outputType=gdal.GDT_Float32)
    z = gdal.Open(dem_local).ReadAsArray().astype(np.float64)
    if (z <= -9000).any():
        raise SystemExit("범위 안에 NoData 가 있음 — MARGIN/범위 확인 필요")
    # gz 월드는 원점 접평면이라 NavSat 고도가 원점에서 멀수록 d²/2R 만큼 높게 나온다
    # (원통119 ~14 km 에서 약 15 m). 지형을 그만큼 내려서 GPS 해발고도 = DEM 해발고도가 되게 한다.
    u = np.linspace(te[0], te[2], RES_PX)
    v = np.linspace(te[3], te[1], RES_PX)          # 행0 = 북쪽
    gx_m, gy_m = np.meshgrid(u, v)
    z = z - (gx_m ** 2 + gy_m ** 2) / (2 * 6371000.0)
    zmin, zmax = float(z.min()), float(z.max())

    # 3) 16bit heightmap (행0 = 북쪽 = +Y)
    h16 = np.round((z - zmin) / (zmax - zmin) * 65535).astype(np.uint16)
    drv = gdal.GetDriverByName("MEM")
    m = drv.Create("", RES_PX, RES_PX, 1, gdal.GDT_UInt16); m.GetRasterBand(1).WriteArray(h16)
    gdal.GetDriverByName("PNG").CreateCopy(f"{HERE}/heightmap.png", m)

    # 4) 텍스처: 고도 색 + 음영기복
    gy, gx = np.gradient(z, side / (RES_PX - 1))
    az, alt = np.radians(315), np.radians(45)
    slope = np.arctan(np.hypot(gx, gy)); aspect = np.arctan2(-gx, gy)
    shade = np.clip(np.sin(alt) * np.cos(slope) + np.cos(alt) * np.sin(slope) * np.cos(az - aspect), 0, 1)
    stops = np.array([[0, 70, 120, 60], [0.25, 110, 150, 70], [0.5, 150, 140, 90],
                      [0.75, 130, 110, 90], [1, 235, 235, 235]], dtype=float)
    t = (z - zmin) / (zmax - zmin)
    rgb = np.stack([np.interp(t, stops[:, 0], stops[:, i]) for i in (1, 2, 3)], -1)
    rgb = np.clip(rgb * (0.35 + 0.65 * shade[..., None]), 0, 255).astype(np.uint8)
    TEX = 1024
    m = drv.Create("", RES_PX, RES_PX, 3, gdal.GDT_Byte)
    for i in range(3):
        m.GetRasterBand(i + 1).WriteArray(rgb[..., i])
    gdal.Translate(f"{HERE}/texture.png", m, format="PNG", width=TEX, height=TEX, resampleAlg="bilinear")
    m = drv.Create("", 4, 4, 3, gdal.GDT_Byte)
    for i, v in enumerate((128, 128, 255)):
        m.GetRasterBand(i + 1).Fill(v)
    gdal.GetDriverByName("PNG").CreateCopy(f"{HERE}/normal.png", m)

    # 5) 스폰 지점 고도
    sx, sy = to_local([(SPAWN_LAT, SPAWN_LON)])[0]
    fc, fr = (sx - te[0]) / d, (te[3] - sy) / d
    c0, r0 = int(fc), int(fr); dc, dr = fc - c0, fr - r0
    ground = (z[r0, c0] * (1 - dc) * (1 - dr) + z[r0, c0 + 1] * dc * (1 - dr)
              + z[r0 + 1, c0] * (1 - dc) * dr + z[r0 + 1, c0 + 1] * dc * dr)
    spawn_amsl = float(ground + (sx ** 2 + sy ** 2) / (2 * 6371000.0))   # 곡률 보정 되돌린 실제 해발
    spawn_z = float(ground) - DATUM_ALT + 0.5

    # 6) SDF
    with open(f"{HERE}/default.sdf", encoding="utf-8-sig") as f:
        sdf = f.read()
    start = sdf.index('<model name="ground_plane">'); end = sdf.index("</model>", start) + len("</model>")
    # gz sim 8 에서 heightmap 오프셋 처리가 물리/렌더링이 서로 반대다(둘 다 실측 확인):
    #   - collision(dartsim): <pos> 무시, model <pose> 적용
    #   - visual(ogre2 Terra): model <pose> 무시, <pos> 적용
    # 그래서 모델을 둘로 나눠 각자 먹히는 방식으로 같은 위치에 놓는다.
    offset = f"{cx:.2f} {cy:.2f} {zmin - DATUM_ALT:.2f}"
    hm_size = f"{side:.2f} {side:.2f} {zmax - zmin:.2f}"
    terrain = f"""<model name="kangwon_terrain_collision">
      <static>true</static>
      <pose>{offset} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><heightmap>
            <uri>model://kangwon/heightmap.png</uri>
            <size>{hm_size}</size><pos>0 0 0</pos>
          </heightmap></geometry>
        </collision>
      </link>
    </model>
    <model name="kangwon_terrain_visual">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><heightmap>
            <use_terrain_paging>false</use_terrain_paging>
            <texture>
              <diffuse>model://kangwon/texture.png</diffuse>
              <normal>model://kangwon/normal.png</normal>
              <size>{side:.2f}</size>
            </texture>
            <uri>model://kangwon/heightmap.png</uri>
            <size>{hm_size}</size><pos>{offset}</pos>
          </heightmap></geometry>
        </visual>
      </link>
    </model>"""
    sdf = sdf[:start] + terrain + sdf[end:]
    sdf = sdf.replace('<world name="default">', '<world name="kangwon">')
    sdf = sdf.replace("<latitude_deg>47.397971057728974</latitude_deg>", f"<latitude_deg>{DATUM_LAT}</latitude_deg>")
    sdf = sdf.replace("<longitude_deg> 8.546163739800146</longitude_deg>", f"<longitude_deg>{DATUM_LON}</longitude_deg>")
    sdf = sdf.replace("<elevation>0</elevation>", f"<elevation>{DATUM_ALT}</elevation>")
    sdf = sdf.replace("<shadows>true</shadows>", "<shadows>false</shadows>")   # 소프트웨어 렌더링 부하 절감
    sdf = sdf.replace("<background>0.7 0.7 0.7 1</background>", "<background>0.62 0.78 0.92 1</background>")
    os.makedirs(f"{HERE}/models/kangwon", exist_ok=True)
    for fn in ("heightmap.png", "texture.png", "normal.png"):
        os.replace(f"{HERE}/{fn}", f"{HERE}/models/kangwon/{fn}")
    with open(f"{HERE}/kangwon.sdf", "w", encoding="utf-8") as f:
        f.write(sdf)

    with open(f"{HERE}/spawn.env", "w") as f:
        f.write(f"PX4_GZ_MODEL_POSE={sx:.1f},{sy:.1f},{spawn_z:.1f}\n")

    info = dict(extent_local_m=[round(v, 1) for v in te], side_m=round(side, 1), px=RES_PX,
                m_per_px=round(side / (RES_PX - 1), 1), zmin=round(zmin, 1), zmax=round(zmax, 1),
                spawn=dict(name=SPAWN_NAME, enu=[round(sx, 1), round(sy, 1)], amsl=round(spawn_amsl, 1)))
    print(json.dumps(info, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
