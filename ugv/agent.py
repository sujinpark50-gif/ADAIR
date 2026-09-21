from .drivers.base import MotionDriver
from .resource import GroundResource
from .road_graph import RoadGraph
import time


class GroundResourceAgent:
    """자원 한 대를 담당한다. RoadGraph 는 전 자원이 같은 인스턴스를 공유한다."""

    def __init__(
        self,
        resource: GroundResource,
        graph: RoadGraph,
        driver: MotionDriver | None = None,
    ):
        self.resource = resource
        self.graph = graph
        self.driver = driver
        self._target_node: str | None = None

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

        if self.resource.state != "READY":
            return {"resource_id": rid, "response": "REJECT", "reason": "BUSY"}

        if self.resource.current_node is None:
            # 주행 중이라 어느 노드에도 정지해 있지 않다
            return {"resource_id": rid, "response": "REJECT", "reason": "BUSY"}

        route = self.graph.find_route(self.resource.current_node, target_node)

        if not route.reachable:
            reason = "ROAD_BLOCKED" if route.blocked_road_id else "TARGET_UNREACHABLE"
            return {"resource_id": rid, "response": "REJECT", "reason": reason}

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

    async def execute(self, target_node: str) -> bool:
        """판단 후 경로를 드라이버에 넘겨 주행을 시작한다."""
        result = self.evaluate(target_node)
        if result["response"] != "ACCEPT":
            return False
        if self.driver is None:
            return False

        coords = [
            (self.graph.node(n).lat, self.graph.node(n).lon)
            for n in result["path"][1:]      # 출발 노드 제외
        ]
        
        started = await self.driver.goto(coords)
        if started:
            self.resource.state = "RUNNING"
            self.resource.current_node = None   # 주행 중 — 노드에 정지해 있지 않음
            self._target_node = target_node
        return started

    def check_arrival(self) -> None:
        """진행률을 보고 도착 여부를 확정한다. 주기적으로 호출한다."""
        if self.resource.state != "RUNNING" or self.driver is None:
            return
        current, total = self.driver.progress()
        if total > 0 and current >= total:
            self.resource.current_node = self._target_node
            self.resource.state = "READY"
            self._target_node = None