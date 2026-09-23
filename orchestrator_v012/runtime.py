"""기존 모듈을 읽기/호출로 연결하는 시연 경계. 팀 파일을 수정하지 않는다."""
import asyncio
import sys
import time
from pathlib import Path

from interfaces.schema import EnvironmentState, ResourceStatus
from .models import FireTask

ROOT = Path(__file__).resolve().parents[1]


class EnvironmentAdapter:
    def __init__(self, config_path=None):
        # 환경 모듈의 기존 src.environment import 경로를 제공한다.
        env_dir = str(ROOT / 'environment')
        if env_dir not in sys.path:
            sys.path.insert(0, env_dir)
        from src.environment.grid import EnvironmentGrid
        from src.environment.fire_model import WildfireCAEngine
        from src.environment.api import EnvironmentModelAPI
        path = str(config_path or ROOT / 'environment/config/environment_config.yaml')
        grid = EnvironmentGrid(path, seed=42)
        engine = WildfireCAEngine(grid, config_path=path, seed=42)
        self.api = EnvironmentModelAPI(grid, engine)

    def snapshot(self):
        # 기존 API 조회는 step()하지 않는다. Connector의 자동 step 경로와 구분.
        spread = self.api.get_spread_direction()
        cells = self.api.get_fire_locations()
        return EnvironmentState(
            time.time(), cells, [], spread['wind_speed_ms'],
            spread['wind_direction_deg'], spread['primary_cardinal'],
        )

    def task_for_cell(self, x, y, task_id, target_node=None):
        from rasterio.warp import transform
        cell = self.api.grid.get_cell(x, y)
        wx, wy = self.api.grid.rowcol_to_xy(y, x)
        lon, lat = transform(self.api.grid.crs, 'EPSG:4326', [wx], [wy])
        return FireTask(
            task_id=task_id, created_at=time.time(), description=f'지정 화재 셀({x},{y}) 관측',
            target_lat=lat[0], target_lon=lon[0], target_alt_m=cell.elevation,
            target_node=target_node, grid_x=x, grid_y=y,
        )

    def apply_uav_observation(self, resource_id, observation):
        # 실제 서버도 현재 Mock 열화상 값을 반환하므로 센서 실증으로 주장하지 않는다.
        from rasterio.warp import transform
        if observation.get('sensor_type') != 'THERMAL' or observation.get('values', {}).get('hotspot_detected') is not True:
            return {'success': False, 'error': '화재 상태로 변환할 열화상 검출 없음'}
        pos = observation['position']
        x, y = transform('EPSG:4326', self.api.grid.crs, [pos['lon']], [pos['lat']])
        return self.api.apply_observation({
            'reporter_id': resource_id, 'world_x': x[0], 'world_y': y[0],
            'fire_state': 'BURNING',
        })


class GroundAdapter:
    def __init__(self, fleet):
        self.fleet = fleet

    def statuses(self):
        return [ResourceStatus(**s) for base in ('A', 'B')
                for s in self.fleet.status_by_base(base) if s['resource_type'] == 'UGV']

    def evaluate(self, resource_id, target_node):
        return self.fleet.evaluate(resource_id, target_node)

    def position_of_node(self, target_node):
        node = self.fleet.graph.node(target_node)
        return node.lat, node.lon

    async def execute_and_wait(self, resource_id, target_node, timeout_s=30):
        started = await self.fleet.execute(resource_id, target_node)
        if not started:
            return {'status': 'NOT_STARTED'}
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.fleet.refresh_all()
            agent = self.fleet.agents[resource_id]
            if agent.resource.state == 'READY' and agent.resource.current_node == target_node:
                return {'status': 'ARRIVED', 'observation': self.fleet.observation(resource_id)}
            await asyncio.sleep(.05)
        # 시간초과는 실제 이동 중단을 뜻하지 않는다. 자동 재할당하지 않는다.
        return {'status': 'UNKNOWN'}
