from . import config
from .agent import GroundResourceAgent
from .drivers.base import MotionDriver
from .drivers.sim import SimDriver
from .graph_data_demo import NODES, ROADS
from .resource import GroundResource
from .road_graph import RoadGraph


class GroundFleet:
    def __init__(self, use_px4: bool = False):
        self.graph = RoadGraph(NODES, ROADS)
        self.agents: dict[str, GroundResourceAgent] = {}

        for cfg in config.RESOURCES:
            node = self.graph.node(cfg["home_node"])
            resource = GroundResource(
                resource_id=cfg["resource_id"],
                resource_type=cfg["resource_type"],
                base=cfg["base"],
                home_node=cfg["home_node"],
                lat=node.lat,
                lon=node.lon,
                current_node=cfg["home_node"],
            )
            driver = self._make_driver(cfg, node, use_px4)
            self.agents[cfg["resource_id"]] = GroundResourceAgent(
                resource, self.graph, driver
            )

    def _make_driver(self, cfg: dict, node, use_px4: bool) -> MotionDriver:
        if use_px4 and cfg.get("px4_port"):
            from .drivers.px4 import PX4Driver   # mavsdk 없으면 임포트 실패하므로 지연
            addr = f"udpin://{config.PX4_HOST}:{cfg['px4_port']}"
            return PX4Driver(
                addr,
                speed_mps=config.CRUISE_SPEED_MPS,
                alt_m=config.MISSION_ALT_M,
            )
        return SimDriver(
            start=(node.lat, node.lon),
            speed_mps=config.DEMO_SPEED_MPS,
        )

    # --- 생애주기 -------------------------------------------------------

    async def connect_all(self) -> None:
        for agent in self.agents.values():
            if agent.driver:
                await agent.driver.connect()

    def refresh_all(self) -> None:
        for agent in self.agents.values():
            agent.refresh()
            agent.check_arrival()

    # --- 커넥터가 호출하는 것 -------------------------------------------

    def status_by_base(self, base: str) -> list[dict]:
        """get_ugv_status(base) 대응."""
        self.refresh_all()
        return [a.status() for a in self.agents.values()
                if a.resource.base == base]

    def evaluate(self, resource_id: str, target_node: str) -> dict:
        """send_ugv_command 대응."""
        return self.agents[resource_id].evaluate(target_node)

    def observation(self, resource_id: str) -> dict:
        """get_ugv_observation 대응."""
        return self.agents[resource_id].observation()

    async def execute(self, resource_id: str, target_node: str) -> bool:
        return await self.agents[resource_id].execute(target_node)

    # --- 환경모듈이 호출하는 것 -----------------------------------------

    def set_road_blocked(self, road_id: str, blocked: bool) -> None:
        self.graph.set_blocked(road_id, blocked)

    def set_road_congestion(self, road_id: str, factor: float) -> None:
        self.graph.set_congestion(road_id, factor)