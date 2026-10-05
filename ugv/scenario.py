# ugv/scenario.py — 도로 환경 시나리오 (차단 도로·경유 불가 노드·혼잡, 시간대별)
#
# 도로 환경은 환경 모듈(화재)과 별개다. 차단·혼잡은 전부 이 파일이 정한다 (API 로 덧붙일 수도 있다).
# 한 줄 = 한 규칙 = "이 대상은 start 부터 end 전까지 이렇다". 시각은 시뮬레이션 시계(ugv/sim_clock.py) 기준.
#
# CSV (ugv/scenarios/*.csv) — 첫 줄은 머리글, '#' 로 시작하는 줄은 주석
#   kind,target,start,end,value,note
#   road,682500248,00:10,01:30,,내린천로 낙석 통제
#   node,495721,00:20,00:50,,대내로 교차로 통제 (닿은 도로 전부)
#   congestion,682501921,0,01:00,2.0,인제읍 시가지 정체 (통과시간 2배)
#
# JSON (ugv/scenarios/*.json) — 같은 필드
#   {"name": "...", "description": "...", "rows": [{"kind": "road", "target": "682500248", "start": "00:10", ...}]}
#
# 필드
#   kind   : road(도로 차단) | node(경유 불가 노드) | congestion(혼잡, 막히게 할 도로만 적는다)
#   target : road_id 또는 node_id (ugv/data/road_network.json). 여러 개는 '|' 로 이어 쓴다
#   start  : 시작. 초(600) | 시:분(00:10) | 시:분:초 | 환경 스텝(step:10). 비우면 0
#   end    : 끝(이 시각부터 해제). 형식은 start 와 같다. 비우면 끝까지
#   value  : congestion 의 통과시간 배율 (>1 이면 느려짐). road/node 는 비운다
#
# 같은 도로에 혼잡이 겹치면 큰 배율을 쓴다. 규칙이 끝나면 차단은 풀리고 혼잡은 1.0 으로 돌아간다.
# 매 tick 마다 "지금 켜져 있어야 할 규칙"을 다시 계산하므로 시계가 되돌아가도(환경 새 실행) 그대로 맞는다.

import csv
import json
import logging
from pathlib import Path

from .road_status import RoadStatus

log = logging.getLogger(__name__)
KINDS = ("road", "node", "congestion")
FIELDS = ("kind", "target", "start", "end", "value", "note")


def parse_time(v, seconds_per_env_step: float, default: float | None) -> float | None:
    """'' → default, '600' → 600, '01:30' → 5400, '00:01:30' → 90, 'step:10' → 10 × 스텝 초."""
    s = "" if v is None else str(v).strip()
    if not s:
        return default
    if s.startswith("step:"):
        return float(s[5:]) * seconds_per_env_step
    if ":" in s:
        parts = [float(p) for p in s.split(":")]
        if len(parts) == 2:
            h, m, sec = parts[0], parts[1], 0.0
        elif len(parts) == 3:
            h, m, sec = parts
        else:
            raise ValueError(f"시각 형식 오류: {s}")
        return h * 3600 + m * 60 + sec
    return float(s)


def fmt_time(t: float | None) -> str:
    if t is None:
        return "끝까지"
    t = int(t)
    return f"{t // 3600:02d}:{t % 3600 // 60:02d}" + (f":{t % 60:02d}" if t % 60 else "")


class Scenario:
    def __init__(self, rows: list[dict], seconds_per_env_step: float,
                 name: str = "", description: str = "", source: str = ""):
        self.name, self.description, self.source = name, description, source
        self.rules = []
        for i, r in enumerate(rows, start=1):
            kind = (r.get("kind") or "").strip()
            if kind not in KINDS:
                raise ValueError(f"{i}번째 규칙: kind 는 {KINDS} 중 하나 ({kind!r})")
            targets = [t.strip() for t in str(r.get("target") or "").split("|") if t.strip()]
            if not targets:
                raise ValueError(f"{i}번째 규칙: target 이 비어 있다")
            start = parse_time(r.get("start"), seconds_per_env_step, 0.0)
            end = parse_time(r.get("end"), seconds_per_env_step, None)
            if end is not None and end <= start:
                raise ValueError(f"{i}번째 규칙: end({r.get('end')}) 가 start({r.get('start')}) 보다 늦어야 한다")
            value = None
            if kind == "congestion":
                value = float(r.get("value") or 0)
                if value <= 0:
                    raise ValueError(f"{i}번째 규칙: congestion 은 value(배율 > 0) 가 필요하다")
            self.rules.append({"no": i, "kind": kind, "targets": targets, "start": start, "end": end,
                               "value": value, "note": (r.get("note") or "").strip()})
        self._on_blocks: set[tuple[str, str]] = set()     # 지금 적용 중인 (kind, id)
        self._on_cong: dict[str, float] = {}              # 지금 적용 중인 혼잡
        self.log: list[dict] = []                         # 켜짐/꺼짐 기록

    # --- 읽기 ----------------------------------------------------------------

    @classmethod
    def load(cls, path: str | Path, seconds_per_env_step: float) -> "Scenario":
        p = Path(path)
        if p.suffix.lower() == ".csv":
            with p.open(encoding="utf-8-sig", newline="") as f:
                lines = [ln for ln in f if ln.strip() and not ln.lstrip().startswith("#")]
            rows = list(csv.DictReader(lines))
            meta = {"name": p.stem}
        else:
            with p.open(encoding="utf-8") as f:
                meta = json.load(f)
            rows = meta.get("rows", [])
        return cls(rows, seconds_per_env_step, name=meta.get("name", p.stem),
                   description=meta.get("description", ""), source=str(p))

    def validate(self, roads: RoadStatus) -> None:
        """없는 도로·노드 id 면 서버 시작 때 바로 실패시킨다."""
        for r in self.rules:
            for t in r["targets"]:
                try:
                    roads.graph.node(t) if r["kind"] == "node" else roads.graph.get_road(t)
                except KeyError:
                    raise ValueError(f"{self.source} {r['no']}번째 규칙: {r['kind']} {t} 가 도로망에 없다") from None

    # --- 적용 ----------------------------------------------------------------

    def active(self, now: float) -> list[dict]:
        return [r for r in self.rules if r["start"] <= now and (r["end"] is None or now < r["end"])]

    def tick(self, now: float, roads: RoadStatus) -> list[dict]:
        """지금 켜져 있어야 할 규칙을 계산해 달라진 것만 적용한다. 바뀐 내역을 돌려준다."""
        want_blocks, want_cong = set(), {}
        for r in self.active(now):
            for t in r["targets"]:
                if r["kind"] == "congestion":
                    want_cong[t] = max(want_cong.get(t, 0.0), r["value"])
                else:
                    want_blocks.add((r["kind"], t))

        changes = []
        for kind, t in self._on_blocks - want_blocks:
            self._set_block(roads, kind, t, False)
            changes.append({"action": "open", "kind": kind, "target": t})
        for kind, t in want_blocks - self._on_blocks:
            self._set_block(roads, kind, t, True)
            changes.append({"action": "close", "kind": kind, "target": t})
        for t in set(self._on_cong) - set(want_cong):
            roads.graph.set_congestion(t, 1.0)
            changes.append({"action": "congestion_end", "kind": "congestion", "target": t})
        for t, v in want_cong.items():
            if self._on_cong.get(t) != v:
                roads.graph.set_congestion(t, v)
                changes.append({"action": "congestion", "kind": "congestion", "target": t, "value": v})
        self._on_blocks, self._on_cong = want_blocks, want_cong

        for c in changes:
            c["sim_time_s"] = round(now, 1)
            self.log.append(c)
        if changes:
            log.info("시나리오 [%s] %s: %s", self.name, fmt_time(now),
                     ", ".join(f"{c['action']} {c['kind']} {c['target']}" for c in changes))
        return changes

    @staticmethod
    def _set_block(roads: RoadStatus, kind: str, target: str, on: bool) -> None:
        if kind == "node":
            roads.set_node_blocked(target, on, "scenario")
        else:
            roads.set_blocked(target, on, "scenario")

    def resync(self) -> None:
        """다른 경로(혼잡 API)가 혼잡 값을 덮어썼을 때, 다음 tick 에 시나리오 혼잡을 다시 걸게 한다."""
        self._on_cong = {t: -1.0 for t in self._on_cong}

    def reset(self, roads: RoadStatus) -> None:
        """시나리오가 건 것을 모두 되돌린다 (다음 tick 이 현재 시각 기준으로 다시 건다)."""
        roads.clear_source("scenario")
        for t in self._on_cong:
            roads.graph.set_congestion(t, 1.0)
        self._on_blocks, self._on_cong, self.log = set(), {}, []

    # --- 조회 ----------------------------------------------------------------

    def info(self, now: float) -> dict:
        def view(r):
            return {"no": r["no"], "kind": r["kind"], "targets": r["targets"],
                    "start": fmt_time(r["start"]), "end": fmt_time(r["end"]),
                    "start_s": r["start"], "end_s": r["end"], "value": r["value"], "note": r["note"]}
        active = {r["no"] for r in self.active(now)}
        upcoming = sorted((r for r in self.rules if r["start"] > now), key=lambda r: r["start"])
        return {"name": self.name, "description": self.description, "source": self.source,
                "sim_time_s": round(now, 1), "sim_time": fmt_time(now),
                "active": [view(r) for r in self.rules if r["no"] in active],
                "upcoming": [view(r) | {"in_s": round(r["start"] - now, 1)} for r in upcoming],
                "rules": [view(r) for r in self.rules], "log": self.log[-50:]}
