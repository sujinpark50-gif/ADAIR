# ugv/gz_fx.py — Gazebo 표시 (경광등·물줄기·도로 차단 벽). 주행·판단과 무관한 '보여 주기'만 한다.
#
# 켜기: UGV_GZ_FX=1 — 서버가 Gazebo 와 같은 기계(WSL)에서 돌 때만. gz CLI 를 비동기로 부르고 결과를 기다리지
# 않는다 (실패해도 주행에 영향 없음, 경고 로그만). Gazebo 는 GZ_PARTITION=ugv, GZ_IP=127.0.0.1 (px4-start.sh 와 같게).
#
#   경광등·물줄기 : 소방차 모델(ugv/gazebo/models/fire_truck — px4-start.sh 가 만든다)의 particle emitter 를
#                   토픽 /ugv/fx/<자원>/<siren|water> 로 켜고 끈다 (gz.msgs.ParticleEmitter emitting)
#   도로 차단 벽  : 막힌 도로 가운데에 빨간 벽 모델을 만들고, 풀리면 지운다 (/world/<월드>/create · remove).
#                   충돌 없음 — 달리던 차가 부딪히는 대신 재탐색으로 피한다. 위치는 build_road_world 의 road_marks.json
#   짐            : Gazebo 표시 없음 (상태만, ugv/equipment.py)
#
# 파티클은 Gazebo 화면(GUI)에서만 보이고 물리 계산에는 들지 않는다.

import asyncio
import json
import logging
import os

log = logging.getLogger(__name__)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENABLED = os.getenv("UGV_GZ_FX", "0") == "1"
WALL_H_M = 4.0
WALL_T_M = 0.6


def _world() -> tuple[str, dict]:
    net = json.load(open(os.path.join(ROOT, "ugv/data/road_network.json"), encoding="utf-8"))
    w = net.get("world", {})
    name = os.getenv("UGV_GZ_WORLD", w.get("name", "kangwon_ugv"))
    marks_path = os.path.join(ROOT, os.path.dirname(w.get("sdf", "ugv/gazebo/kangwon_ugv.sdf")),
                              "models", name, "road_marks.json")
    try:
        marks = json.load(open(marks_path, encoding="utf-8"))
    except OSError:
        marks = {}
    return name, marks


class GzFx:
    def __init__(self, enabled: bool = ENABLED):
        self.enabled = enabled
        self.world, self.marks = _world() if enabled else ("", {})
        self.walls: set[str] = set()
        self._env = {**os.environ, "GZ_PARTITION": os.getenv("GZ_PARTITION", "ugv"),
                     "GZ_IP": os.getenv("GZ_IP", "127.0.0.1")}
        if enabled:
            log.info("Gazebo 표시 켜짐: 월드 %s, 벽 위치 %d개", self.world, len(self.marks))

    def _run(self, *args: str) -> None:
        if not self.enabled:
            return
        try:
            asyncio.get_running_loop().create_task(self._exec(args))
        except RuntimeError:                  # 이벤트 루프 밖 (테스트 등)
            pass

    async def _exec(self, args) -> None:
        try:
            p = await asyncio.create_subprocess_exec("gz", *args, env=self._env, stdout=asyncio.subprocess.DEVNULL,
                                                     stderr=asyncio.subprocess.PIPE)
            _, err = await asyncio.wait_for(p.communicate(), 10)
            if p.returncode:
                log.warning("gz %s 실패: %s", args[:3], err.decode(errors="replace").strip()[:200])
        except Exception as e:   # noqa: BLE001 — 표시 실패는 주행과 무관
            log.warning("gz %s 실패: %s", args[:3], e)

    def _emit(self, rid: str, what: str, on: bool) -> None:
        self._run("topic", "-t", f"/ugv/fx/{rid}/{what}", "-m", "gz.msgs.ParticleEmitter",
                  "-p", f"emitting: {{data: {'true' if on else 'false'}}}")

    # --- 차량 -------------------------------------------------------------------
    def siren(self, rid: str, on: bool) -> None:
        self._emit(rid, "siren", on)

    def spray(self, rid: str, on: bool) -> None:
        self._emit(rid, "water", on)

    def cargo(self, rid: str, on: bool) -> None:
        pass                                   # Gazebo 표시 없음 — 상태만

    # --- 도로 차단 벽 ------------------------------------------------------------
    def sync_walls(self, blocked: list[str]) -> None:
        """막힌 도로 목록에 맞춰 벽을 만들고 지운다."""
        if not self.enabled:
            return
        want = {rid for rid in blocked if rid in self.marks}
        for rid in sorted(want - self.walls):
            x, y, z, yaw = self.marks[rid]
            sdf = (f"<?xml version='1.0'?><sdf version='1.9'><model name='block_{rid}'><static>true</static>"
                   f"<pose>{x:.2f} {y:.2f} {z + WALL_H_M / 2:.2f} 0 0 {yaw:.3f}</pose><link name='l'>"
                   f"<visual name='v'><geometry><box><size>{WALL_T_M} 12 {WALL_H_M}</size></box></geometry>"
                   "<material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse>"
                   "<emissive>0.5 0 0 1</emissive></material></visual></link></model></sdf>")
            self._run("service", "-s", f"/world/{self.world}/create", "--reqtype", "gz.msgs.EntityFactory",
                      "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f"sdf: {json.dumps(sdf)}")
        for rid in sorted(self.walls - want):
            self._run("service", "-s", f"/world/{self.world}/remove", "--reqtype", "gz.msgs.Entity",
                      "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f'name: "block_{rid}" type: MODEL')
        self.walls = want


fx = GzFx()
