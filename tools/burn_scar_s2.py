# -*- coding: utf-8 -*-
"""2019 인제 산불 — 실제 피해지(burn scar)를 Sentinel-2 위성영상으로 CA 격자에 그대로 얹는다.

인터넷이 되는 PC(WSL 포함)에서 실행한다. 계정·키 없이 Microsoft Planetary Computer 공개 STAC 을 쓴다.

    python tools/burn_scar_s2.py                         # → web/static/inje2019/burn_scar.json
    python tools/burn_scar_s2.py --firms-key <MAP_KEY>   # NASA FIRMS 열점(위성 화점)도 함께 (키 무료 발급)
    python tools/burn_scar_s2.py --wide                  # 진단: 발화점 중심 20 km 창에서 실제 피해지가 어디 있는지
                                                         #       (CA 격자는 발화점 서쪽 1.3 km 에서 끝난다)

방법 (기술문서 8절 '검증' 그대로)
  1. 산불 전·후 Sentinel-2 L2A 장면을 고른다 (기본: 2019-04-01~04-03 중 구름 적은 것 / 04-06~04-12).
     선행연구도 4월 3일(전)·4월 8일(후) 장면을 썼다.
  2. NIR(B08)·SWIR2(B12)를 CA 격자(EPSG:5186, 90 m)에 바로 재투영해 읽는다 (COG 개요 사용 → 수십 MB 안쪽).
     구름·그림자(SCL 3·8·9·10)는 둘 중 하나라도 걸리면 '판정 불가' 칸으로 둔다.
  3. NBR = (B08 − B12)/(B08 + B12),  dNBR = NBR_전 − NBR_후.  dNBR ≥ 0.27(USGS 중-저 피해 기준) 을 피해로 본다.
  4. 남전약수터 반경 안의 연결 성분만 남긴다 (다른 원인 변화 제거). 면적을 아리랑3호 판독 342.2 ha 와 비교해 출력.

결과는 '관측 사실'이지만 한계가 있다: 90 m 칸으로 평균내면 가장자리가 뭉개지고, 4월 초 식생 변화·지형 그림자가
dNBR 에 섞일 수 있다. 임계값은 --threshold 로 바꿔 민감도를 본다.
"""

from __future__ import annotations

import argparse
import base64
import csv
import io
import json
import math
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "environment"))
OUT = ROOT / "web" / "static" / "inje2019" / "burn_scar.json"
SPRING = (38.0277, 128.1303)            # 남전약수터 (지도 장소 검색 좌표)
ARIRANG_HA = 342.2                      # 국립산림과학원 아리랑3호 판독 (보도)
KST = timezone(timedelta(hours=9))
START_KST = datetime(2019, 4, 4, 14, 45, tzinfo=KST)
CLOUD_SCL = (3, 8, 9, 10)               # 구름 그림자, 구름(중·고), 권운


# ── 격자 ──────────────────────────────────────────────────────────────────
def load_grid():
    from src.grid import EnvironmentGrid
    g = EnvironmentGrid(config_path=str(ROOT / "environment" / "config" / "environment_config.yaml"), seed=0)
    return {"crs": g.crs, "transform": g.transform, "cols": g.cols, "rows": g.rows, "res": float(g.cell_resolution)}


def wide_grid(half_km=10.0):
    """발화점 중심 창. 강원 DEM(dem_gangwon.tif)과 같은 90 m 격자에 맞춘다 (CA 격자와도 칸 경계가 같다)."""
    import rasterio
    from rasterio.transform import from_origin
    from rasterio.warp import transform as warp
    with rasterio.open(ROOT / "environment" / "data" / "processed" / "gangwon" / "dem_gangwon.tif") as d:
        crs, t = d.crs, d.transform
    xs, ys = warp("EPSG:4326", crs, [SPRING[1]], [SPRING[0]])
    n = int(round(half_km * 1000 / 90))
    c0 = int((xs[0] - t.c) // t.a) - n
    r0 = int((t.f - ys[0]) // -t.e) - n
    return {"crs": crs, "transform": from_origin(t.c + c0 * t.a, t.f + r0 * t.e, t.a, -t.e),
            "cols": 2 * n, "rows": 2 * n, "res": float(t.a)}


def describe_components(mask, G, ca=None):
    """피해 연결 성분별 면적·중심·발화점에서 방향, CA 격자 안 비율."""
    from scipy import ndimage
    from rasterio.warp import transform as warp
    lab, n = ndimage.label(mask == 1, structure=np.ones((3, 3)))
    t = G["transform"]; cell_ha = G["res"] ** 2 / 10_000
    out = []
    for k in range(1, n + 1):
        rr, cc = np.nonzero(lab == k)
        xs = t.c + (cc + 0.5) * t.a; ys = t.f + (rr + 0.5) * t.e
        lons, lats = warp(G["crs"], "EPSG:4326", [float(xs.mean())], [float(ys.mean())])
        dx = (lons[0] - SPRING[1]) * 111320 * math.cos(math.radians(SPRING[0])); dy = (lats[0] - SPRING[0]) * 111320
        inside = None
        if ca is not None:
            ct = ca["transform"]
            gc = (xs - ct.c) / ct.a; gr = (ys - ct.f) / ct.e
            inside = float(np.mean((gc >= 0) & (gc < ca["cols"]) & (gr >= 0) & (gr < ca["rows"])))
        out.append({"area_ha": round(len(rr) * cell_ha, 1), "lat": round(lats[0], 5), "lon": round(lons[0], 5),
                    "from_spring_km": round(math.hypot(dx, dy) / 1000, 2),
                    "bearing_deg": round((math.degrees(math.atan2(dx, dy)) + 360) % 360),
                    "inside_ca_grid": None if inside is None else round(inside, 2)})
    return sorted(out, key=lambda c: -c["area_ha"])


def grid_bbox_wgs84(G):
    from rasterio.warp import transform_bounds
    t = G["transform"]
    left, top = t.c, t.f
    right, bottom = left + G["cols"] * t.a, top + G["rows"] * t.e
    return transform_bounds(G["crs"], "EPSG:4326", left, bottom, right, top)


def warp_to_grid(href, G, resampling="average"):
    """원격/로컬 래스터 하나를 CA 격자 그대로 읽는다 (WarpedVRT → 필요한 개요만 내려받음)."""
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.vrt import WarpedVRT
    with rasterio.open(href) as src:
        with WarpedVRT(src, crs=G["crs"], transform=G["transform"], width=G["cols"], height=G["rows"],
                       resampling=getattr(Resampling, resampling), nodata=0) as vrt:
            return vrt.read(1).astype(np.float32)


# ── 장면 선택 (Planetary Computer STAC) ───────────────────────────────────
def find_scenes(bbox, pre_range, post_range, max_cloud):
    import planetary_computer
    import pystac_client
    cat = pystac_client.Client.open("https://planetarycomputer.microsoft.com/api/stac/v1",
                                    modifier=planetary_computer.sign_inplace)

    def best(rng):
        items = list(cat.search(collections=["sentinel-2-l2a"], bbox=bbox, datetime=rng,
                                query={"eo:cloud_cover": {"lt": max_cloud}}).items())
        if not items:
            raise SystemExit(f"장면 없음: {rng} (구름 {max_cloud}% 미만). --max-cloud 를 올리거나 기간을 넓힐 것")
        # 격자를 가장 많이 덮는 타일 → 구름 적은 순
        def cover(it):
            from shapely.geometry import box, shape
            return shape(it.geometry).intersection(box(*bbox)).area
        try:
            items.sort(key=lambda it: (-round(cover(it), 4), it.properties.get("eo:cloud_cover", 100)))
        except ImportError:
            items.sort(key=lambda it: it.properties.get("eo:cloud_cover", 100))
        return items
    return best(pre_range), best(post_range)


def read_scene(item, G):
    b08 = warp_to_grid(item.assets["B08"].href, G)
    b12 = warp_to_grid(item.assets["B12"].href, G)
    scl = warp_to_grid(item.assets["SCL"].href, G, resampling="nearest")
    return b08, b12, scl


# ── 계산 (네트워크와 분리 — 시험 가능) ─────────────────────────────────────
def nbr(b08, b12):
    with np.errstate(invalid="ignore", divide="ignore"):
        v = (b08 - b12) / (b08 + b12)
    v[(b08 <= 0) | (b12 <= 0)] = np.nan
    return v


def burn_mask(pre, post, G, threshold=0.27, radius_km=6.0, min_cells=2):
    """pre/post = (b08, b12, scl) 격자 배열. 반환: mask(uint8: 0 아님·1 피해·255 판정불가), 정보"""
    from scipy import ndimage
    n0, n1 = nbr(pre[0], pre[1]), nbr(post[0], post[1])
    bad = np.isin(pre[2], CLOUD_SCL) | np.isin(post[2], CLOUD_SCL) | np.isnan(n0) | np.isnan(n1)
    dnbr = n0 - n1
    burned = (dnbr >= threshold) & ~bad
    lab, nlab = ndimage.label(burned, structure=np.ones((3, 3)))
    # 남전약수터 반경 안에 걸친 성분만
    from rasterio.warp import transform as warp
    xs, ys = warp("EPSG:4326", G["crs"], [SPRING[1]], [SPRING[0]])
    t = G["transform"]
    sc, sr = (xs[0] - t.c) / t.a - 0.5, (ys[0] - t.f) / t.e - 0.5
    rr, cc = np.mgrid[0:G["rows"], 0:G["cols"]]
    near = np.hypot(rr - sr, cc - sc) * G["res"] <= radius_km * 1000
    keep = [k for k in range(1, nlab + 1) if (lab == k).sum() >= min_cells and ((lab == k) & near).any()]
    mask = np.zeros(burned.shape, np.uint8)
    mask[np.isin(lab, keep)] = 1
    mask[bad] = 255
    cell_ha = G["res"] ** 2 / 10_000
    area = float((mask == 1).sum() * cell_ha)
    return mask, dnbr, {"threshold": threshold, "components_kept": len(keep), "area_ha": round(area, 1),
                        "unknown_cells_near_fire": int((bad & near).sum()),
                        "vs_arirang_ratio": round(area / ARIRANG_HA, 3)}


# ── NASA FIRMS (선택, 키 필요) ─────────────────────────────────────────────
def firms(key, bbox, G):
    import urllib.request
    from rasterio.warp import transform as warp
    w, s, e, n = bbox
    out = []
    for src in ("VIIRS_SNPP_SP", "MODIS_SP"):
        url = (f"https://firms.modaps.eosdis.nasa.gov/api/area/csv/{key}/{src}/"
               f"{w:.4f},{s:.4f},{e:.4f},{n:.4f}/3/2019-04-04")
        try:
            txt = urllib.request.urlopen(url, timeout=60).read().decode()
        except Exception as ex:  # noqa: BLE001
            print(f"  FIRMS {src} 실패: {ex}")
            continue
        for r in csv.DictReader(io.StringIO(txt)):
            lat, lon = float(r["latitude"]), float(r["longitude"])
            hhmm = r["acq_time"].zfill(4)
            utc = datetime.strptime(r["acq_date"] + hhmm, "%Y-%m-%d%H%M").replace(tzinfo=timezone.utc)
            xs, ys = warp("EPSG:4326", G["crs"], [lon], [lat])
            t = G["transform"]
            out.append({"src": src, "lat": lat, "lon": lon, "kst": utc.astimezone(KST).strftime("%m-%d %H:%M"),
                        "t": (utc - START_KST).total_seconds(), "confidence": r.get("confidence"),
                        "gx": round((xs[0] - t.c) / t.a - 0.5, 2), "gy": round((ys[0] - t.f) / t.e - 0.5, 2)})
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pre", default="2019-03-28/2019-04-03")
    ap.add_argument("--post", default="2019-04-06/2019-04-15")
    ap.add_argument("--max-cloud", type=float, default=40)
    ap.add_argument("--threshold", type=float, default=0.27)
    ap.add_argument("--firms-key", default=None)
    ap.add_argument("--wide", action="store_true", help="발화점 중심 20 km 창에서 피해지 위치 진단")
    a = ap.parse_args(argv)
    if a.wide:
        return main_wide(a)
    G = load_grid()
    bbox = grid_bbox_wgs84(G)
    print(f"격자 {G['cols']}×{G['rows']} @ {G['res']} m, bbox {tuple(round(v, 4) for v in bbox)}")
    pres, posts = find_scenes(bbox, a.pre, a.post, a.max_cloud)
    pre_item, post_item = pres[0], posts[0]
    print(f"전: {pre_item.id} 구름 {pre_item.properties.get('eo:cloud_cover')}%")
    print(f"후: {post_item.id} 구름 {post_item.properties.get('eo:cloud_cover')}%")
    print("영상 읽는 중 (격자 크기로 재투영, 수십 MB)…")
    pre, post = read_scene(pre_item, G), read_scene(post_item, G)
    mask, dnbr, info = burn_mask(pre, post, G, threshold=a.threshold)
    sens = {str(th): burn_mask(pre, post, G, threshold=th)[2]["area_ha"] for th in (0.1, 0.2, 0.27, 0.44)}
    hot = firms(a.firms_key, bbox, G) if a.firms_key else []
    doc = {"source": "Sentinel-2 L2A (Microsoft Planetary Computer)", "method": "dNBR = NBR(전) − NBR(후), 90 m 격자 평균",
           "pre_scene": pre_item.id, "post_scene": post_item.id,
           "pre_datetime": pre_item.properties.get("datetime"), "post_datetime": post_item.properties.get("datetime"),
           **info, "area_by_threshold_ha": sens, "arirang_ha": ARIRANG_HA,
           "mask_u8": base64.b64encode(mask.tobytes()).decode(),
           "dnbr_i8": base64.b64encode(np.clip(np.nan_to_num(dnbr) * 100, -127, 127).astype(np.int8).tobytes()).decode(),
           "firms": hot, "note": "255 = 구름·그림자로 판정 불가 칸"}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(doc, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    print(json.dumps({k: v for k, v in doc.items() if k not in ("mask_u8", "dnbr_i8", "firms")}, ensure_ascii=False, indent=1))
    print(f"FIRMS 열점 {len(hot)}개" if a.firms_key else "FIRMS 생략 (--firms-key 로 추가)")
    print(f"저장: {OUT}")


def main_wide(a):
    W, CA = wide_grid(), load_grid()
    bbox = grid_bbox_wgs84(W)
    pres, posts = find_scenes(bbox, a.pre, a.post, a.max_cloud)
    print(f"넓은 창 {W['cols']}×{W['rows']} (발화점 중심 ±10 km)\n전: {pres[0].id}\n후: {posts[0].id}")
    pre, post = read_scene(pres[0], W), read_scene(posts[0], W)
    rep = {}
    for th in (0.27, 0.44):
        mask, _, info = burn_mask(pre, post, W, threshold=th, radius_km=8.0, min_cells=3)
        comps = describe_components(mask, W, CA)
        rep[str(th)] = {"area_ha": info["area_ha"], "components": comps[:8]}
        print(f"\n[dNBR ≥ {th}] 발화점 8 km 안 피해 합계 {info['area_ha']} ha (아리랑3호 {ARIRANG_HA} ha)")
        for c in comps[:8]:
            print(f"  {c['area_ha']:7.1f} ha  발화점에서 {c['from_spring_km']:5.2f} km, 방위 {c['bearing_deg']:3d}°"
                  f"  ({c['lat']}, {c['lon']})  CA 격자 안 {int((c['inside_ca_grid'] or 0) * 100)}%")
    out = OUT.with_name("burn_scar_wide.json")
    out.write_text(json.dumps({"pre_scene": pres[0].id, "post_scene": posts[0].id, "window": "spring ±10 km, 90 m",
                               "by_threshold": rep}, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"\n저장: {out}")


if __name__ == "__main__":
    main()
