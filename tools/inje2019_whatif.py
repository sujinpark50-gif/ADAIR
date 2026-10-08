# -*- coding: utf-8 -*-
"""2019 인제 산불 — "드론이 있었다면?" 반사실(counterfactual) 시나리오 빌더.

같은 '진짜 세계'(김은주님 CA + 기상청 인제 211 시간자료 + 실제 DEM·도로망)를 한 번 돌리고,
총괄이 아는 세계를 두 갈래로 만든다.

  A. 실제 기록 근사 : 낮에는 헬기 육안으로 화선을 안다. 19:06 일몰 뒤 헬기 진화 중단(백서 142~146쪽) →
                      일출까지 공중 관측 없음 (마지막으로 본 화선에서 멈춘 지식).
  B. 드론이 있었다면 : 같은 낮 + 야간에 열화상 UAV 2대가 교대로 화선을 돈다. 출동 차량과 함께 전진 이착륙.
                      관측소 풍속이 UAV 한계(10 m/s, uav-agent config)를 넘는 시간에는 못 뜬다.

두 갈래 모두 같은 ENV-04 분석기(BeliefSpreadAnalysis)로 2시간 확산 예측을 내고, 진짜 세계의 다음 2시간과 비교한다.
→ "드론이 무엇을 바꾸는가"를 **진압 효과가 아니라 '총괄이 아는 것'과 '예측 정확도'로** 잰다.
   진압(ENV-07)은 모델이 없어 다루지 않는다.

출력: web/static/inje2019/scenario.json (three.js 뷰어 web/static/inje2019/index.html 이 읽는다)

실행 (저장소 루트):  python tools/inje2019_whatif.py
"""

from __future__ import annotations

import base64
import copy
import json
import math
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ENV_DIR = ROOT / "environment"
for p in (str(ENV_DIR), str(ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

import src.fire_model as fire_model                       # noqa: E402
from src.belief_analysis import BeliefSpreadAnalysis      # noqa: E402
from src.cell import FireState                            # noqa: E402
from src.contract_env import KmaWindDriver                # noqa: E402
from src.fire_model import WildfireCAEngine               # noqa: E402
from src.grid import EnvironmentGrid                      # noqa: E402

KST = timezone(timedelta(hours=9))
START_KST = "2019-04-04T14:45:00+09:00"      # 신고 시각 (기사·백서). 실화 추정 14:43 — 2분 차이는 무시
# 재현을 만든 CA 설정 (2026-10-03 환경 기본 설정이 새로 보정되며 legacy 로 격리됨). 트윈 환경 서버도 같은 파일을 쓴다 (twin_env.ENV_CONFIG)
CONFIG = ENV_DIR / "config" / "scenario_whatif_2019_legacy.yaml"
WEATHER_CSV = ROOT / "data" / "weather" / "kma_asos_hourly_20190404_20190405.csv"
STATIONS_JSON = ROOT / "data" / "weather" / "kma_asos_stations.json"
ROADS_JSON = ROOT / "ugv" / "data" / "road_network.json"
OUT = ROOT / "web" / "static" / "inje2019" / "scenario.json"

# ── 근거가 있는 사실 (출처는 events 에 함께 싣는다) ─────────────────────────────
NAMJEON_SPRING = (38.0277, 128.1303)    # 남전약수터 — Google 지도 장소 검색 좌표. 발화점은 '약수터 인근 밭→야산'
SITES = [  # 백서 141쪽 시설 후보 중 좌표를 지도 검색으로 찾은 것만 (백서 그림 픽셀은 쓰지 않음)
    {"site_id": "NAMJEON1_HALL", "name": "남전1리 마을회관", "lat": 38.015369, "lon": 128.142508,
     "human_occupied": True, "coord_source": "Google 지도 장소 검색(정류장 표기)"},
    {"site_id": "INJE_REST_AREA", "name": "인제휴게소", "lat": 38.0352, "lon": 128.1466,
     "human_occupied": True, "coord_source": "Google 지도 장소 검색"},
]
EVENTS = [  # (KST, 내용, 출처)
    ("2019-04-04T14:43", "남전약수터 인근 밭 잡풀 소각 중 발화", "경찰 수사 결과 보도(한국일보 2019-05-14)"),
    ("2019-04-04T14:45", "산불 신고", "백서·보도"),
    ("2019-04-04T15:06", "기상악화로 소방헬기 출동 불가", "강원 산불백서 142~146쪽"),
    ("2019-04-04T15:38", "주민 대피 지시", "강원 산불백서 142~146쪽"),
    ("2019-04-04T16:25", "대응 2단계 발령", "강원 산불백서 142~146쪽"),
    ("2019-04-04T19:00", "산림 약 10 ha 소실 추정", "한국일보 2019-04-04 19시 현재 보도"),
    ("2019-04-04T19:06", "일몰, 헬기 진화 중단 — 야간 민가 방어", "강원 산불백서 142~146쪽"),
    ("2019-04-05T06:10", "일출(근사) — 헬기 재투입 가능", "천문 계산 근사값"),
    ("2019-04-06T12:00", "주불 진화 (약 46시간)", "한국일보 2019-04-06"),
]
# 실제 투입 자원 — 보도 시각 기준 (그 시각까지 알려진 숫자만 보여 준다)
RESOURCES = [
    {"kst": "2019-04-04T14:45", "helicopters": 3, "note": "산림청 등 진화 헬기·진화대 투입 (초기 보도)",
     "src": "서울신문 2019-04-04 포토 기사"},
    {"kst": "2019-04-04T17:52", "helicopters": 6, "equipment": 39, "note": "대응 2단계, 헬기 6대 등 장비 39대",
     "src": "한국일보 2019-04-04 17:52 기사"},
    {"kst": "2019-04-04T19:00", "helicopters": 9, "equipment": 36, "personnel": 543, "evacuated": "17가구 35명",
     "note": "19시 현재. 일몰 후 헬기 철수, 민가 방어선", "src": "한국일보 2019-04-04 기사 (19시 현재)"},
    {"kst": "2019-04-06T12:00", "helicopters": 11, "equipment": 12, "personnel": 720,
     "note": "주불 진화까지 누적 (군 헬기 6·주한미군 4 지원 별도)", "src": "한국일보 2019-04-06 기사"},
]
DAY_AIR_END = "2019-04-04T19:06"     # 헬기 육안 관측 끝 (일몰)
DAY_AIR_RESUME = "2019-04-05T06:10"  # 일출 근사
FINAL_AREA_HA = 345.0                # 산림 피해 최종 (보도). 아리랑3호 판독 342.2 ha
ANCHOR_19H_HA = 10.0                 # 19:00 추정 소실 면적 (보도) — tick_s 보정 기준

# ── UAV 가정 (uav/uav-agent/config.py 값을 그대로 씀, 그 파일도 '실측 보정 필요'로 표시) ─────────
UAV_CRUISE_MS = 10.0
UAV_DRAIN_PCT_S = 0.033
UAV_START_PCT, UAV_RESERVE_PCT = 95.0, 15.0
UAV_WIND_LIMIT_MS = 10.0
UAV_ENDURANCE_S = (UAV_START_PCT - UAV_RESERVE_PCT) / UAV_DRAIN_PCT_S      # ≈ 2424 s

HORIZON_S = 7200.0                   # 예측 비교 구간 2시간
SIM_HOURS = 32.25                    # 기상청 시간자료 끝(4/5 23:00)까지
SEEDS = list(range(12))


def kst(s: str) -> datetime:
    return datetime.fromisoformat(s).replace(tzinfo=KST) if "+" not in s else datetime.fromisoformat(s)


T0 = kst(START_KST)


def sim_s(s: str) -> float:
    return (kst(s) - T0).total_seconds()


def b64(arr: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(arr).tobytes()).decode()


# ─────────────────────────────────────────────────────────────────────────────
class FastCA:
    """김은주님 CA 를 그대로 쓰되, 매 스텝 전 격자 위험도 재계산(시각화용)만 끈다 — 확산식·난수는 동일."""

    def __init__(self, grid, seed, ignition):
        self.grid = grid
        for c in grid.cells:
            c.fire_state = FireState.UNBURNED
        self._orig = fire_model.update_grid_risk_scores
        fire_model.update_grid_risk_scores = lambda *a, **k: None
        try:
            self.e = WildfireCAEngine(grid=grid, config_path=str(CONFIG), seed=seed)
        finally:
            pass
        self.e.predict_spread = lambda *a, **k: {}
        self.e.ignite_at(*ignition)

    def step(self, wind_ms, wind_from_deg):
        self.e._config["wind"] = {"speed_ms": wind_ms, "direction_deg": wind_from_deg}
        self.e.step()

    def close(self):
        fire_model.update_grid_risk_scores = self._orig


def burnt_count(grid):
    return sum(1 for c in grid.cells if c.fire_state != FireState.UNBURNED)


def run_truth(grid, wind, ignition, seed, tick_s, until_s, record=False):
    ca = FastCA(grid, seed, ignition)
    n = len(grid.cells)
    ign = np.full(n, -1, np.int32)
    out = np.full(n, -1, np.int32)
    ign[[c.cell_id for c in grid.cells if c.fire_state == FireState.BURNING]] = 0
    steps = int(round(until_s / tick_s))
    areas = [burnt_count(grid)]
    for k in range(steps):
        w = wind.at(k * tick_s)["values"]
        ca.step(w["wind_ms"], w["wind_dir_deg"])
        if record:
            for c in grid.cells:
                i = c.cell_id
                if c.fire_state == FireState.BURNING and ign[i] < 0:
                    ign[i] = k + 1
                elif c.fire_state == FireState.BURNED:
                    if ign[i] < 0:
                        ign[i] = k + 1
                    if out[i] < 0:
                        out[i] = k + 1
        areas.append(burnt_count(grid))
    ca.close()
    return areas, ign, out


def calibrate(grid, wind, ignition):
    """19:00 추정 10 ha 에 맞는 tick_s(CA 한 스텝 = 몇 초)를 고른다. 여러 seed 평균."""
    cell_ha = grid.cell_resolution ** 2 / 10_000
    target_cells = ANCHOR_19H_HA / cell_ha
    t19 = sim_s("2019-04-04T19:00")
    rows = []
    for tick in (900, 1200, 1500, 1800, 2100, 2400, 2700, 3000, 3600):
        vals = [run_truth(grid, wind, ignition, s, tick, t19)[0][-1] for s in SEEDS]
        rows.append({"tick_s": tick, "mean_cells_19h": float(np.mean(vals)), "min": int(min(vals)),
                     "max": int(max(vals)), "mean_ha_19h": round(float(np.mean(vals)) * cell_ha, 2)})
    best = min(rows, key=lambda r: abs(r["mean_cells_19h"] - target_cells))
    return best, rows, target_cells


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 2 * 6371000 * math.asin(math.sqrt(a))


def graph_gpkg_bases():
    from ugv import graph_gpkg
    return graph_gpkg.BASES


def neighbors8(i, n_cols, n_rows):
    x, y = i % n_cols, i // n_cols
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx or dy:
                xx, yy = x + dx, y + dy
                if 0 <= xx < n_cols and 0 <= yy < n_rows:
                    yield yy * n_cols + xx


def main():
    grid = EnvironmentGrid(config_path=str(CONFIG), seed=0)
    n_cols, n_rows, res = grid.cols, grid.rows, float(grid.cell_resolution)
    n = len(grid.cells)
    wind = KmaWindDriver(WEATHER_CSV, "211", START_KST, STATIONS_JSON)
    from rasterio.warp import transform as warp
    tr = grid.transform

    def to_grid(lat, lon):
        xs, ys = warp("EPSG:4326", grid.crs, [lon], [lat])
        return (xs[0] - tr.c) / tr.a - 0.5, (ys[0] - tr.f) / tr.e - 0.5        # 칸 중심 기준 실수 (col,row)

    # 발화 칸 = 남전약수터에서 가장 가까운 연료 칸
    sc, sr = to_grid(*NAMJEON_SPRING)
    ign_cell = min((math.hypot(c.x - sc, c.y - sr), c.x, c.y) for c in grid.cells if c.fuel_amount > 0
                   and abs(c.x - sc) < 8 and abs(c.y - sr) < 8)
    ignition = (ign_cell[1], ign_cell[2])

    best, table, target_cells = calibrate(grid, wind, ignition)
    tick = float(best["tick_s"])
    until = SIM_HOURS * 3600
    t19 = sim_s("2019-04-04T19:00")
    # 보정 tick 에서 19시 면적이 평균에 가장 가까운 seed 를 '대표 실행'으로 고른다
    reps = [(abs(run_truth(grid, wind, ignition, s, tick, t19)[0][-1] - best["mean_cells_19h"]), s) for s in SEEDS]
    seed = min(reps)[1]
    areas, ign, out = run_truth(grid, wind, ignition, seed, tick, until, record=True)
    steps = len(areas) - 1
    cell_ha = res * res / 10_000

    def burning_at(k):     # 스텝 k 끝 시점에 타고 있는 칸
        return set(np.nonzero((ign >= 0) & (ign <= k) & ((out < 0) | (out > k)))[0].tolist())

    def ever_at(k):
        return set(np.nonzero((ign >= 0) & (ign <= k))[0].tolist())

    # ── 지식 두 갈래 ───────────────────────────────────────────────────────────
    air_end, air_resume = sim_s(DAY_AIR_END), sim_s(DAY_AIR_RESUME)
    base_a = graph_gpkg_bases()["A"]
    transit_s = haversine(base_a["lat"], base_a["lon"], *NAMJEON_SPRING) / UAV_CRUISE_MS
    # 드론 변형: B = 출동 차량과 함께 전진 이착륙(이동 0), Bb = 원통 기지에서 왕복 (같은 2대·같은 배터리)
    ON_STATION = {"B": min(tick, UAV_ENDURANCE_S),
                  "Bb": max(0.0, min(tick, UAV_ENDURANCE_S - 2 * transit_s))}
    known_A = set(ever_at(0))                                 # 14:45 신고 = 발화 칸
    burning_known_A = set(burning_at(0))
    kB = {v: set(ever_at(0)) for v in ON_STATION}
    bB = {v: set(burning_at(0)) for v in ON_STATION}
    sorties = {v: 0 for v in ON_STATION}
    timeline, drone_tracks = [], []
    static = {"n_cols": n_cols, "n_rows": n_rows, "cell_size_m": res,
              "lat": np.zeros(n), "lon": np.zeros(n),          # 분석기는 위경도를 바람 선택 거리에만 씀
              "elevation": np.array([c.elevation for c in grid.cells]),
              "fuel_amount": np.array([c.fuel_amount for c in grid.cells]),
              "moisture": np.array([c.moisture for c in grid.cells]),
              "building_type": np.array([c.building_type for c in grid.cells])}
    cols = np.array([c.x for c in grid.cells]); rows = np.array([c.y for c in grid.cells])
    xs = tr.c + (cols + 0.5) * tr.a; ys = tr.f + (rows + 0.5) * tr.e
    lons, lats = warp(grid.crs, "EPSG:4326", xs.tolist(), ys.tolist())
    static["lat"], static["lon"] = np.array(lats), np.array(lons)
    analyzer = BeliefSpreadAnalysis(static, copy.deepcopy(grid._config), tick_s=tick, horizon_s=HORIZON_S,
                                    max_forecast_cells=10 ** 6, n_risk_cells=10)
    st211 = json.load(open(STATIONS_JSON, encoding="utf-8"))["stations"]["211"]
    sortie = 0
    known_step = {x: np.full(n, -1, np.int16) for x in ("A", "B", "Bb")}
    for k in range(steps + 1):
        t = k * tick
        w = wind.at(t)["values"]
        truth_b, truth_e = burning_at(k), ever_at(k)
        daylight = t <= air_end or t >= air_resume
        drone_ok = w["wind_ms"] < UAV_WIND_LIMIT_MS
        # A: 낮(헬기 육안)에는 그 시각 화선 전체를 안다고 가정. 밤에는 갱신 없음.
        if daylight:
            known_A |= truth_e; burning_known_A = set(truth_b)
        # B·Bb: 낮은 A 와 같다. 밤에는 UAV 한 대가 이 스텝 동안 화선을 돈다 (2대 교대).
        tour = []
        for v, on_station in ON_STATION.items():
            if daylight:
                kB[v] |= truth_e; bB[v] = set(truth_b)
                continue
            if not (drone_ok and k > 0 and on_station > 0):
                continue
            capacity = max(1, int(on_station * UAV_CRUISE_MS / res))  # 한 칸(90m)을 지나는 데 9초
            if truth_b:
                cx = float(np.mean([cols[i] for i in truth_b])); cy = float(np.mean([rows[i] for i in truth_b]))
                ring = sorted(truth_b, key=lambda i: (math.atan2(rows[i] - cy, cols[i] - cx), i))
                start = (sorties[v] * capacity) % len(ring)      # 소티마다 이어서 돈다 (같은 곳만 보지 않게)
                seen = (ring[start:] + ring[:start])[:capacity]
            else:
                seen = []
            seen_set = set(seen)
            # 열화상 프레임 안의 주변 칸: 이미 다 탄 칸(전소)도 확인된다
            seen_burned = {j for i in seen for j in neighbors8(i, n_cols, n_rows) if j in truth_e and j not in truth_b}
            kB[v] |= seen_set | seen_burned
            # 다시 본 칸만 갱신. 안 본 곳은 마지막으로 믿던 대로 둔다 (진짜 세계를 몰래 읽지 않음)
            bB[v] = (bB[v] - seen_burned) | seen_set
            sorties[v] += 1
            if v == "B":
                tour = [[round(float(cols[i]), 1), round(float(rows[i]), 1)] for i in seen[:: max(1, len(seen) // 120)]]
        known_B, burning_known_B = kB["B"], bB["B"]
        sortie = sorties["B"]
        for name, ks in (("A", known_A), ("B", kB["B"]), ("Bb", kB["Bb"])):
            arr = known_step[name]
            new_idx = [i for i in ks if arr[i] < 0]
            arr[new_idx] = k
        # 예측 (ENV-04 분석기, 아는 세계만 입력) — 2시간 뒤 진짜 세계와 비교
        future = ever_at(min(steps, k + int(round(HORIZON_S / tick)))) - truth_e
        res_ab = {}
        for name, kb in (("A", burning_known_A), ("B", bB["B"]), ("Bb", bB["Bb"])):
            belief = {"simulation_time_s": t, "known_fires": [{"cell_id": f"{cols[i]}_{rows[i]}"} for i in sorted(kb)],
                      "station_weather": [{"station_id": "211", "lat": st211["lat"], "lon": st211["lon"],
                                           "sim_time_s": t, "values": {"wind_ms": w["wind_ms"],
                                                                       "wind_dir_deg": w["wind_dir_deg"]}}],
                      "field_weather": []}
            an = analyzer.analyze(belief)
            pred = {int(c["cell_id"].split("_")[1]) * n_cols + int(c["cell_id"].split("_")[0])
                    for c in an["spread_forecast"]}
            kn = known_A if name == "A" else kB[name]
            pred -= kn
            hit = len(pred & future)
            res_ab[name] = {"known_ha": round(len(kn) * cell_ha, 2),
                            "unknown_burning_ha": round(len(truth_b - kb) * cell_ha, 2),
                            "forecast_recall": round(hit / len(future), 3) if future else None,
                            "forecast_precision": round(hit / len(pred), 3) if pred else None,
                            "forecast": sorted(pred)}
        timeline.append({"k": k, "t": t, "kst": (T0 + timedelta(seconds=t)).strftime("%m-%d %H:%M"),
                         "wind_ms": w["wind_ms"], "wind_from_deg": w["wind_dir_deg"], "daylight": daylight,
                         "drone_ok": drone_ok, "truth_ha": round(len(truth_e) * cell_ha, 2),
                         "truth_burning": len(truth_b), "A": res_ab["A"], "B": res_ab["B"], "Bb": res_ab["Bb"],
                         "drone": {"active": f"UAV-{1 + sortie % 2}" if tour else None, "tour": tour}})

    # ── 차량: 실제 도로망(박유홍) Dijkstra ─────────────────────────────────────
    from ugv import graph_gpkg
    from ugv.road_graph import RoadGraph
    rg = RoadGraph(graph_gpkg.NODES, graph_gpkg.ROADS)
    target_node, snap = rg.nearest_node(*NAMJEON_SPRING)
    vehicles = []
    for vid, label, base, speed_note in (("A-fire1", "소방차", "A", "도로 등급별 속도"),
                                         ("A-ugv1", "UGV", "A", "도로 등급별 속도")):
        rr = rg.find_route(base, target_node.node_id)
        if not rr.reachable:
            continue
        pts, t_acc, prev = [], [], None
        base_info = graph_gpkg.BASES[base]
        snap_s = base_info["snap_m"] / (50 / 3.6)          # 거점→경계 노드 (도로망 밖) 50 km/h 가정
        for nid in rr.path:
            nd = rg.node(nid)
            gx, gy = to_grid(nd.lat, nd.lon)
            if prev is None:
                t_acc.append(snap_s)
            else:
                road = next(r for nb, r in rg.neighbors(prev) if nb == nid)
                t_acc.append(t_acc[-1] + road.base_time_s)
            pts.append([round(gx, 2), round(gy, 2)])
            prev = nid
        vehicles.append({"id": vid, "label": label, "path": pts, "t": [round(x, 1) for x in t_acc],
                         "eta_s": round(t_acc[-1], 1), "speed_note": speed_note,
                         "note": "출동 14:45 가정. 거점은 도로망 밖이라 경계 노드까지 50 km/h 가정"})
    bx, by = to_grid(graph_gpkg.BASES["A"]["lat"], graph_gpkg.BASES["A"]["lon"])

    roads = []
    nodes = {nd["node_id"]: nd for nd in graph_gpkg.NODES}
    for r in graph_gpkg.ROADS:
        a, b = nodes[r["node_a"]], nodes[r["node_b"]]
        roads.append([*map(lambda v: round(v, 2), to_grid(a["lat"], a["lon"])),
                      *map(lambda v: round(v, 2), to_grid(b["lat"], b["lon"]))])
    sites = []
    for s in SITES:
        gx, gy = to_grid(s["lat"], s["lon"])
        idx = int(round(gy)) * n_cols + int(round(gx))
        arr = int(ign[idx]) if 0 <= idx < n else -1
        sites.append({**s, "gx": round(gx, 2), "gy": round(gy, 2),
                      "fire_arrival_kst": None if arr < 0 else (T0 + timedelta(seconds=arr * tick)).strftime("%m-%d %H:%M")})

    # 야간 요약 (발표용 숫자)
    night = [r for r in timeline if not r["daylight"]]
    def avg(key, who):
        v = [r[who][key] for r in night if r[who][key] is not None]
        return round(float(np.mean(v)), 3) if v else None
    summary = {
        "tick_s": tick, "seed": seed, "ignition": {"col": ignition[0], "row": ignition[1]},
        "truth_ha_19h": timeline[min(range(len(timeline)), key=lambda i: abs(timeline[i]["t"] - t19))]["truth_ha"],
        "truth_ha_end": timeline[-1]["truth_ha"], "end_kst": timeline[-1]["kst"], "final_reported_ha": FINAL_AREA_HA,
        "night_steps": len(night),
        "night_unknown_burning_ha_mean": {x: avg("unknown_burning_ha", x) for x in ("A", "B", "Bb")},
        "night_forecast_recall_mean": {x: avg("forecast_recall", x) for x in ("A", "B", "Bb")},
        "night_forecast_precision_mean": {x: avg("forecast_precision", x) for x in ("A", "B", "Bb")},
        "uav_transit_from_base_s": round(transit_s), "on_station_s_per_step": {k: round(v) for k, v in ON_STATION.items()},
        "uav_endurance_s": round(UAV_ENDURANCE_S), "drone_grounded_night_steps": sum(1 for r in night if not r["drone_ok"]),
    }
    k_sr = min(steps, int(math.ceil(air_resume / tick)) - 1)
    def known_at(name, k):
        return set(np.nonzero((known_step[name] >= 0) & (known_step[name] <= k))[0].tolist())
    summary["sunrise_unknown_ha"] = {
        **{x: round(len(ever_at(k_sr) - known_at(x, k_sr)) * cell_ha, 2) for x in ("A", "B", "Bb")},
        "at_kst": timeline[k_sr]["kst"]}

    elev = np.array([c.elevation for c in grid.cells], dtype=np.float32).reshape(n_rows, n_cols)
    doc = {
        "title": "2019 인제 산불 — 드론이 있었다면?",
        "start_kst": START_KST, "grid": {"cols": n_cols, "rows": n_rows, "cell_m": res, "crs": grid.crs},
        "elev_dm_u16": b64(np.clip(elev * 10, 0, 65535).astype(np.uint16)),
        "fuel_u8": b64(np.array([c.fuel_type for c in grid.cells], np.uint8)),
        "ign_step_i16": b64(ign.astype(np.int16)), "out_step_i16": b64(out.astype(np.int16)),
        "knownA_step_i16": b64(known_step["A"]), "knownB_step_i16": b64(known_step["B"]),
        "knownBb_step_i16": b64(known_step["Bb"]),
        "steps": steps, "tick_s": tick, "roads": roads, "sites": sites, "vehicles": vehicles,
        "base_A": {"gx": round(bx, 2), "gy": round(by, 2), "name": graph_gpkg.BASES["A"]["name"]},
        "spring": dict(zip(("gx", "gy"), map(lambda v: round(v, 2), (sc, sr)))),
        "timeline": timeline, "events": [{"kst": e[0], "t": sim_s(e[0]), "text": e[1], "src": e[2]} for e in EVENTS],
        "resources": [{**r, "t": sim_s(r["kst"])} for r in RESOURCES],
        "twin_env": {"ENV_IGNITION": f"{ignition[0]},{ignition[1]}", "ENV_TICK_S": str(int(tick)), "ENV_SEED": str(seed),
                     "ENV_SCENARIO_START_KST": START_KST, "ENV_CONFIG": CONFIG.relative_to(ROOT).as_posix()},
        "calibration": {"anchor": f"19:00 추정 {ANCHOR_19H_HA} ha", "target_cells": round(target_cells, 1),
                        "chosen": best, "table": table},
        "summary": summary,
        "assumptions": [
            "진짜 세계 = 팀 CA(문헌 초기값, 인제로 보정 안 됨) + 인제 관측소 바람 전 영역 균일 (양간지풍 미반영)",
            f"CA 한 스텝 = {int(tick)}초 — 19:00 추정 10 ha 하나에 맞춘 값 (보도 추정치라 오차 큼)",
            "진압(헬기·진화대 효과)은 모델 없음 → 시뮬 면적은 '진압 없이 번졌다면'이다. 최종 345 ha 와 직접 비교하지 않는다",
            "낮 헬기 육안 = 화선 전체를 안다고 단순화. 실제로는 연기로 가려진다",
            "UAV: uav-agent 설정값(10 m/s, 0.033%/s, 예비 15%) — 실측 보정 전. 2대 교대, 스텝당 한 소티",
            "B = 출동 차량과 함께 현장 근처에서 이착륙(이동 시간 0). Bb = 원통 기지에서 왕복 (편도 약 10 km)",
            "드론 비행 가능 = 관측소 평균풍속 < 10 m/s. 능선 돌풍은 더 셀 수 있다",
            "발화점 = 남전약수터 최근접 연료 칸 (정밀 발화 좌표 아님)",
        ],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(doc, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    print(json.dumps({"out": str(OUT), "bytes": OUT.stat().st_size, "calibration": best, "summary": summary},
                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
