# -*- coding: utf-8 -*-
"""ugv/tools/world_preflight.py — Gazebo 시험 시작 전 월드·생성 위치 검사 (확정적 설정 오류는 재시도하지 않고 중단).

    python -m ugv.tools.world_preflight --world-sdf <월드.sdf> --tiles <타일 폴더> --spawn <생성 목록>
        [--subset road_subset.json] [--extra extra_roads.json] [--out preflight.json]

검사 (하나라도 실패하면 종료 코드 2, 사유는 JSON 으로):
1) 생성 지점과 차체 둘레(차종 차체 + 여유 5 m)가 월드에 포함된 타일 위에 있는가 — 없으면 차량이 땅 없는 곳에 생겨 떨어진다
   (2026-10-12 c34·c35: 거점 B 타일 T17_04 누락 → PX4_NO_POSITION)
2) 생성 높이가 그 지점 타일 지면보다 0.2–1.0 m 위인가 (지면 아래·공중 생성 방지)
3) 추가 도로(회전 공터 등)와 도로 부분집합의 모든 점이 포함된 타일 위에 있는가, 부분집합의 월드 이름이 같은가
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

from ugv.profiles import PROFILES
from ugv.tools import build_terrain_tiles as BT
from ugv.tools.terrain_compare import Surf

MARGIN_M = 5.0
Z_MIN, Z_MAX = 0.2, 1.0


def tile_of(x, y):
    return BT.tile_name(int((x - BT.X0) // BT.TILE_M), int((y - BT.Y0) // BT.TILE_M))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world-sdf", required=True)
    ap.add_argument("--tiles", required=True)
    ap.add_argument("--spawn", required=True)
    ap.add_argument("--subset", default=None)
    ap.add_argument("--extra", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--flat-ground-z", type=float, default=None, help="평탄 비교 월드: 타일 대신 이 높이의 지면 상자 (윗면 z)")
    ap.add_argument("--subset-any-world", action="store_true", help="다른 월드의 도로 부분집합을 일부러 쓸 때 (같은 계획으로 비교시험)")
    a = ap.parse_args()
    sdf = Path(a.world_sdf).read_text(encoding="utf-8")
    world = re.search(r'<world name="([^"]+)"', sdf).group(1)
    inc = set(re.findall(r"<uri>model://(T\d+_\d+)</uri>", sdf))
    flat = a.flat_ground_z is not None
    if flat:
        inc = {tile_of(x, y) for x in range(int(BT.X0), int(BT.X0 + 40 * BT.TILE_M), 500) for y in range(int(BT.Y0), int(BT.Y0 + 30 * BT.TILE_M), 500)}
    S = None if flat else Surf(a.tiles)
    alt0 = 0.0 if flat else S.meta.get("world_origin_alt_m", 0.0)
    fails, rows = [], []
    for line in Path(a.spawn).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        parts = line.split()
        model, (x, y, z, yaw), rid = parts[1], map(float, parts[2].split(",")), parts[3] if len(parts) > 3 else "?"
        prof = PROFILES.get(model) or PROFILES["adair_ugv"]
        hl, hw = prof.body_length_m / 2 + MARGIN_M, prof.body_width_m / 2 + MARGIN_M
        c, s = math.cos(yaw), math.sin(yaw)
        need = sorted({tile_of(x + fx * c - fy * s, y + fx * s + fy * c) for fx in (-hl, 0, hl) for fy in (-hw, 0, hw)})
        miss = [t for t in need if t not in inc]
        if flat:
            ground = a.flat_ground_z
        else:
            ground = S.z(x, y) - alt0 if tile_of(x, y) in S.meta["tiles"] else math.nan
        clr = z - ground
        row = {"vehicle": rid, "model": model, "xy": [x, y], "tiles_needed": need, "tiles_missing": miss,
               "spawn_z": z, "ground_z": None if math.isnan(ground) else round(ground, 3),
               "clearance_m": None if math.isnan(clr) else round(clr, 3)}
        rows.append(row)
        if miss:
            fails.append(f"{rid}: 생성 지점·차체 둘레 타일 {miss} 가 월드 {world} 에 없음")
        if math.isnan(clr) or not (Z_MIN <= clr <= Z_MAX):
            fails.append(f"{rid}: 생성 높이 - 지면 = {row['clearance_m']} m (허용 {Z_MIN}–{Z_MAX} m)")

    def pts_outside(label, roads):
        bad = sorted({rid for rid, xy in roads for x, y in xy if tile_of(x, y) not in inc})
        if bad:
            fails.append(f"{label} {len(bad)}개가 월드 타일 밖: {bad[:8]}")
        return bad

    res = {"world": world, "tiles_in_world": sorted(inc), "spawn": rows}
    if a.extra:
        # 추가 도로(회전 공터 등, 위경도): 일부만 월드 안이면 공터가 잘린 것 → 실패. 전부 밖(다른 거점 공터)은 기록만
        from ugv.geo import FRAME
        ex = json.loads(Path(a.extra).read_text(encoding="utf-8"))
        part, away = [], []
        for r in ex.get("roads", []):
            ins = [tile_of(*FRAME.to_xy(la, lo)) in inc for la, lo in r["geometry"]]
            if any(ins) and not all(ins):
                part.append(r["road_id"])
            elif not any(ins):
                away.append(r["road_id"])
        res["extra_cut"], res["extra_not_in_world"] = part, away
        if part:
            fails.append(f"추가 도로(회전 공터 등) {part} 가 월드 경계에서 잘림")
    if a.subset:
        sub = json.loads(Path(a.subset).read_text(encoding="utf-8"))
        if sub.get("world") not in (None, world) and not a.subset_any_world:
            fails.append(f"도로 부분집합 월드 {sub.get('world')} ≠ {world}")
        from ugv.roads import RoadNetwork
        net = RoadNetwork()
        res["subset_outside"] = pts_outside("부분집합 도로", [(r, net.roads[r].xy) for r in sub["roads"] if r in net.roads])
    res["ok"] = not fails
    res["fails"] = fails
    out = json.dumps(res, ensure_ascii=False, indent=1)
    if a.out:
        Path(a.out).write_text(out, encoding="utf-8")
    print(("PREFLIGHT_OK " if not fails else "PREFLIGHT_FAIL ") + world + (" — " + " / ".join(fails) if fails else ""))
    sys.exit(0 if not fails else 2)


if __name__ == "__main__":
    main()
