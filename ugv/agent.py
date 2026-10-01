import time

from . import route_plan
from .drivers.base import MotionDriver
from .geo import distance_m
from .resource import GroundResource
from .road_graph import RoadGraph
from .route_plan import RoutePlan

# 이상 코드 → evaluate 거절 사유 (공통 계약 RejectReason 에 있는 값만 쓴다)
FAULT_REASON = {"TELEMETRY_LOST": "COMMUNICATION_FAILURE"}   # 나머지(STALLED·OFF_ROUTE·VEHICLE_FAULT)는 FAILSAFE_ACTIVE


class GroundResourceAgent:
    """자원 한 대를 담당한다. RoadGraph 는 전 자원이 같은 인스턴스를 공유한다."""

    def __init__(
        self,
        resource: GroundResource,
        graph: RoadGraph,
        driver: MotionDriver | None = None,
        max_speed_mps: float | None = None,
    ):
        self.resource = resource
        self.graph = graph
        self.driver = driver
        self.max_speed_mps = max_speed_mps           # 차량 최고속도. None 이면 도로 제한속도만 본다 (시연 도로망)
        self._target_node: str | None = None
        self.plan: RoutePlan | None = None           # 주행 중 계획 (도로 선형 웨이포인트)
        self.fault: str | None = None                # 'CODE: 설명'. 있으면 UNAVAILABLE, /stop 으로 해제
        self.reroutes = 0                            # 이번 task 에서 경로를 바꾼 횟수

    @property
    def route(self) -> list[tuple[float, float]]:
        """주행 중 경로 선형 (출발점 포함) — 이탈 판정·시각화용. 주행 중이 아니면 []."""
        return self.plan.polyline if self.plan else []

    def refresh(self) -> None:
        """드라이버 스냅샷을 자원 상태로 옮긴다.

        status() 는 자원만 읽으므로, 최신 값이 필요하면 먼저 호출한다.
        드라이버가 없으면 그래프 좌표가 그대로 유지된다.
        """
        if self.driver is None:
            return
        lat, lon = self.driver.position()
        if lat != lat:      # NaN — 텔레메트리 미수신
            return
        self.resource.lat = lat
        self.resource.lon = lon
        self.resource.fuel_pct = self.driver.battery()
        self.resource.updated_at = time.time()
    # --- get_ugv_status 응답 -------------------------------------------

    def status(self) -> dict:
        """자원 상태 스냅샷. ResourceStatus(**이 dict) 로 변환 가능한 형태."""
        r = self.resource
        return {
            "resource_id": r.resource_id,
            "resource_type": r.resource_type,
            "base": r.base,
            "location_lat": r.lat,
            "location_lon": r.lon,
            "state": r.state,
            "capability": {
                "fuel_pct": r.fuel_pct,
                "eta_sec": None,        # 목적지 미지정 — evaluate 응답에서 산출
                "road_blocked": False,
                "reachable": True,
            },
        }

    # --- send_ugv_command 응답 -----------------------------------------

    def evaluate(self, target_node: str) -> dict:
        """목적지까지 갈 수 있는지 판단한다.

        ACCEPT  -> eta_s, path 포함
        REJECT  -> reason (BUSY / ROAD_BLOCKED / TARGET_UNREACHABLE)
        """
        rid = self.resource.resource_id

        if self.resource.state == "UNAVAILABLE":
            code = (self.fault or "UNKNOWN").split(":")[0]
            return {"resource_id": rid, "response": "REJECT",
                    "reason": FAULT_REASON.get(code, "FAILSAFE_ACTIVE"), "fault": self.fault}

        if self.resource.state != "READY":
            return {"resource_id": rid, "response": "REJECT", "reason": "BUSY"}

        if self.resource.current_node is None:
            # 주행 중이라 어느 노드에도 정지해 있지 않다
            return {"resource_id": rid, "response": "REJECT", "reason": "BUSY"}

        route = self.graph.find_route(self.resource.current_node, target_node, self.max_speed_mps)

        if not route.reachable:
            reason = "ROAD_BLOCKED" if route.blocked_road_id else "TARGET_UNREACHABLE"
            return {"resource_id": rid, "response": "REJECT", "reason": reason,
                    "blocked_road_id": route.blocked_road_id}

        return {
            "resource_id": rid,
            "response": "ACCEPT",
            "eta_s": route.eta_s,
            "path": route.path,
        }

    # --- get_ugv_observation 응답 --------------------------------------

    def observation(self) -> dict:
        """현재 관측 결과. 지상자원의 주 관측은 도로 상태다."""
        r = self.resource
        return {
            "resource_id": r.resource_id,
            "location_lat": r.lat,
            "location_lon": r.lon,
            "observation_type": "ROAD_STATUS",
            "value": {
                "current_road_id": r.current_road_id,
                "passable": True,       # 센서 미연동 — 잠정값
            },
        }

    # --- 실행 -----------------------------------------------------------

    def _plan(self, start_node: str, target_node: str, start: tuple[float, float],
              lead: list[dict] | None = None) -> RoutePlan | None:
        """start_node → target_node 주행 계획. lead: 앞에 붙일 구간(지금 달리는 도로의 남은 부분)."""
        route = self.graph.find_route(start_node, target_node, self.max_speed_mps)
        if not route.reachable:
            return None
        legs = (lead or []) + self.graph.legs(route.path, self.max_speed_mps)
        lead_s = sum(leg["distance_m"] / leg["speed_mps"] for leg in (lead or []))
        return route_plan.build(legs, start, target_node, route.path, route.eta_s + lead_s)

    async def execute(self, target_node: str) -> bool:
        """판단 후 도로 선형을 따라가는 웨이포인트를 드라이버에 넘겨 주행을 시작한다."""
        result = self.evaluate(target_node)
        if result["response"] != "ACCEPT":
            return False
        if self.driver is None:
            return False
        if len(result["path"]) == 1:
            return True     # 이미 목적지 노드에 있다 — 움직일 것 없이 READY 그대로 (감시가 곧바로 도착 처리)

        start = self.graph.node(self.resource.current_node)
        plan = self._plan(self.resource.current_node, target_node, (start.lat, start.lon))
        started = await self.driver.goto(plan.waypoints, plan.speeds)
        if started:
            self.plan = plan
            self.reroutes = 0
            self.resource.state = "RUNNING"
            self.resource.current_node = None   # 주행 중 — 노드에 정지해 있지 않음
            self._target_node = target_node
        return started

    def blocked_ahead(self) -> str | None:
        """주행 중 남은 경로(지금 달리는 도로 제외)에 막힌 도로가 있으면 그 id."""
        if self.plan is None or self.driver is None:
            return None
        passed, _ = self.driver.progress()
        ahead = self.plan.road_ahead(passed)[1:]
        return next((rid for rid in ahead if self.graph.get_road(rid).blocked), None)

    async def reroute(self) -> dict:
        """남은 경로가 막혔을 때 지금 달리는 도로의 끝 노드에서 다시 탐색해 이어 달린다.
        반환: {"result": "REROUTED"|"NO_ROUTE", "blocked_road_id", "eta_s"}"""
        passed, _ = self.driver.progress()
        blocked = self.blocked_ahead()
        leg = self.plan.current_leg(passed)
        # 지금 달리는 도로의 남은 점 (다음 웨이포인트부터 도로 끝까지)
        remaining = [self.plan.waypoints[i] for i in range(passed, len(self.plan.waypoints))
                     if self.plan.leg_of[i] == self.plan.leg_of[min(passed, len(self.plan.leg_of) - 1)]]
        here = (self.resource.lat, self.resource.lon)
        pts = [here] + remaining
        lead = [{**leg, "points": pts,
                 "distance_m": sum(distance_m(a, b) for a, b in zip(pts, pts[1:]))}]
        plan = self._plan(leg["to"], self._target_node, here, lead)
        if plan is None:
            return {"result": "NO_ROUTE", "blocked_road_id": blocked, "eta_s": None}
        if not await self.driver.goto(plan.waypoints, plan.speeds):
            return {"result": "NO_ROUTE", "blocked_road_id": blocked, "eta_s": None}
        self.plan = plan
        self.reroutes += 1
        return {"result": "REROUTED", "blocked_road_id": blocked, "eta_s": plan.eta_s}

    def current_road_id(self) -> str | None:
        if self.plan is None or self.driver is None:
            return None
        leg = self.plan.current_leg(self.driver.progress()[0])
        return leg["road_id"] if leg else None

    async def stop(self) -> None:
        """주행을 멈춘다. 멈춘 위치에서 가장 가까운 도로 노드에 선 것으로 보고 READY 로 돌린다."""
        if self.driver is not None:
            await self.driver.stop()
        self.refresh()
        node, _ = self.graph.nearest_node(self.resource.lat, self.resource.lon)
        self.resource.current_node = node.node_id
        self.resource.state = "READY"
        self._target_node = None
        self.plan, self.fault = None, None

    async def fail(self, fault: str) -> None:
        """주행 중 이상. 차를 세우고 UNAVAILABLE 로 둔다. 운영자가 /stop 으로 확인해야 READY 로 돌아온다."""
        if self.driver is not None:
            await self.driver.stop()
        self.refresh()
        self.resource.state = "UNAVAILABLE"
        self.resource.current_node = None
        self._target_node = None
        self.plan = None
        self.fault = fault

    def off_route_m(self) -> float:
        """현재 위치에서 경로(도로 선형)까지 최단 거리(m). 경로가 없으면 0."""
        return route_plan.distance_to_polyline_m((self.resource.lat, self.resource.lon), self.route)

    def check_arrival(self) -> None:
        """진행률을 보고 도착 여부를 확정한다. 주기적으로 호출한다."""
        if self.resource.state != "RUNNING" or self.driver is None:
            return
        current, total = self.driver.progress()
        if total > 0 and current >= total:
            self.resource.current_node = self._target_node
            self.resource.state = "READY"
            self._target_node = None
            self.plan = None