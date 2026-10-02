# ugv/reporter.py — 총괄에 이벤트 보고 (push)
#
# 동작 원리: UGV 자원도 UAV 처럼 총괄이 개별로 지휘한다. 현장 지휘 역할은 없다.
# 우리는 평가·수행하고, 일이 생길 때마다(평가, 출발, 진행, 재탐색, 도착, 실패, 중지, 상태 변화) 보고한다.
#
# 보고 경로: 총괄 POST /events (orchestrator/api.py). 형식 {event_id, type, payload}.
#   - event_id 가 같으면 총괄이 중복으로 처리한다 → 재시도해도 두 번 반영되지 않는다.
#   - type 중 RESOURCE_CHANGED 는 총괄이 보류 중인 임무를 다시 배정하는 계기가 된다.
#     나머지 UGV_* 는 총괄 장부에 기록만 된다 (총괄이 처리 규칙을 붙이면 그때부터 쓰인다).
#   - simulation_time_s 는 보내지 않는다. 총괄은 자기 환경 시각보다 미래면 거절(422)하는데,
#     UGV 시계(ugv/sim_clock.py)와 총괄 시계가 아직 묶여 있지 않다. 우리 시각은 payload.sim_time_s 에 싣는다.
#
# 기존 경로(총괄이 1초마다 GET /ugv/{id}/task/{task_id} 로 조회)는 그대로 둔다. 이 보고는 그 위에 더하는 것이다.
# 보고가 실패해도 주행은 멈추지 않는다 (재시도 후 버리고 로그만 남긴다).
#
# 설정: UGV_REPORT_URL (기본 http://127.0.0.1:8200 = 총괄 기본 주소), 빈 값이면 보고 끔.

import asyncio
import itertools
import logging
import time

log = logging.getLogger(__name__)

RETRIES = 3
RETRY_BASE_S = 0.5
QUEUE_MAX = 1000


class Reporter:
    def __init__(self, base_url: str | None, clock=None, timeout_s: float = 2.0):
        self.base_url = (base_url or "").rstrip("/")
        self.clock = clock
        self.timeout_s = timeout_s
        self._q: asyncio.Queue | None = None
        self._task: asyncio.Task | None = None
        self._seq = itertools.count(1)
        self._boot = time.strftime("%Y%m%dT%H%M%S")      # 서버 재시작마다 event_id 가 겹치지 않게
        self.sent = self.failed = self.dropped = 0
        self.recent: list[dict] = []                      # 최근 보고 (조회·시험용)
        self._warned = False

    @property
    def enabled(self) -> bool:
        return bool(self.base_url)

    async def start(self) -> None:
        self._q = asyncio.Queue(QUEUE_MAX)
        if self.enabled:            # 꺼져 있으면 전송 작업자를 띄우지 않는다 (httpx 불필요)
            self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()

    def report(self, type_: str, resource_id: str, task_id: str | None = None, **payload) -> dict:
        """보고 하나를 큐에 넣는다 (기다리지 않음). 넣은 이벤트를 돌려준다."""
        ev = {
            "event_id": f"UGV:{self._boot}:{next(self._seq):06d}:{resource_id}:{type_}",
            "type": type_,
            "payload": {"source": "UGV", "resource_id": resource_id, "task_id": task_id,
                        "sim_time_s": None if self.clock is None else round(self.clock.now(), 1),
                        **payload},
        }
        self.recent = (self.recent + [ev])[-100:]
        if not self.enabled or self._q is None:
            return ev
        try:
            self._q.put_nowait(ev)
        except asyncio.QueueFull:
            self.dropped += 1
            log.warning("보고 큐가 가득 차 버림: %s", ev["event_id"])
        return ev

    async def _run(self) -> None:
        import httpx    # 보고를 켤 때만 필요하다 (기본 꺼짐) — 없어도 서버는 뜬다
        async with httpx.AsyncClient(timeout=self.timeout_s) as client:
            while True:
                ev = await self._q.get()
                await self._send(client, ev)

    async def _send(self, client, ev: dict) -> None:
        import httpx
        for attempt in range(RETRIES):
            try:
                r = await client.post(f"{self.base_url}/events", json=ev)
                if r.status_code in (200, 202):       # 202 반영, 200 같은 내용 재전송(중복)
                    self.sent += 1
                    self._warned = False
                    return
                if r.status_code in (409, 422):      # 내용 문제 — 다시 보내도 같다
                    self.failed += 1
                    log.warning("보고 거절 %s %s: %s", r.status_code, ev["type"], r.text[:200])
                    return
            except httpx.HTTPError as e:
                if not self._warned:
                    log.warning("총괄 보고 실패 (%s) — %s. 주행은 계속한다", self.base_url, e)
                    self._warned = True
            await asyncio.sleep(RETRY_BASE_S * 2 ** attempt)
        self.failed += 1

    def info(self) -> dict:
        return {"enabled": self.enabled, "url": self.base_url, "sent": self.sent, "failed": self.failed,
                "dropped": self.dropped, "queued": self._q.qsize() if self._q else 0}
