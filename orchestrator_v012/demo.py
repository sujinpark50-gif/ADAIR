"""python -m orchestrator_v012.demo --scenario normal

기존 UAV 서버(MockDrone), 실제 환경 CA 소스와 Raster, GroundFleet(SimDriver),
기존 Mock Safety를 연결한다. 실제 PX4/센서 안전 검증은 아니다.
"""
import argparse
import asyncio
from contextlib import contextmanager
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

import requests
from connectors import safety_connector
from logger import EventLogger
from ugv.fleet import GroundFleet
from .runtime import EnvironmentAdapter, GroundAdapter, ROOT
from .runner import Orchestrator
from .uav_adapter import UavHttpAdapter

SCENARIOS = ('normal', 'moderate', 'strong', 'uav_rejected', 'blocked', 'distant', 'unknown_target')


@contextmanager
def mock_servers(log_dir):
    """새로 띄운 localhost UAV Mock 프로세스만 사용하고 종료한다."""
    processes, files, endpoints = [], [], {}
    http = requests.Session()
    http.trust_env = False
    try:
        for rid in ('A-uav1', 'B-uav1'):
            with socket.socket() as sock:
                sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
            url = f'http://127.0.0.1:{port}'
            endpoints[rid] = url
            log = open(Path(log_dir) / f'{rid}-server.log', 'w')
            files.append(log)
            env = {**os.environ, 'UAV_ID': rid, 'UAV_MODE': 'mock'}
            proc = subprocess.Popen([
                sys.executable, '-m', 'uvicorn', 'main:app', '--host', '127.0.0.1',
                '--port', str(port), '--log-level', 'warning',
            ], cwd=ROOT / 'px4/uav-agent', env=env, stdout=log, stderr=log)
            processes.append(proc)
            deadline = time.monotonic() + 20
            while True:
                if proc.poll() is not None:
                    raise RuntimeError(f'{rid} 서버 시작 실패: {log.name}')
                try:
                    response = http.get(url + '/health', timeout=.5)
                    response.raise_for_status()
                    if response.json().get('mode') != 'mock':
                        raise RuntimeError('시연은 Mock UAV만 허용')
                    break
                except requests.RequestException:
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f'{rid} 서버 시작 시간초과')
                    time.sleep(.1)
        yield UavHttpAdapter(endpoints, http)
    finally:
        for proc in processes:
            proc.terminate()
        for proc in processes:
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
        for stream in files: stream.close()
        http.close()


async def scenario(name, uav, environment, output_dir):
    # 시연 입력: Raster 내부 지정 셀과 기존 도로망 F1을 명시적으로 제공한다.
    # (37,2)는 fuel_amount=0인 통신/경로 검증용 강제 발화이며 산불 재현 사례가 아니다.
    x, y = (19, 142) if name == 'distant' else (37, 2)
    wind = 9.0 if name == 'moderate' else 10.0 if name in ('strong', 'blocked', 'distant', 'unknown_target') else 5.0
    environment.api.ca_engine.ignite_at(x, y)
    changed = environment.api.apply_observation({'reporter_id': 'scenario', 'x': x, 'y': y, 'wind_speed_ms': wind})
    if not changed.get('success'): raise RuntimeError(changed)
    environment.api.step()
    for url in uav.endpoints.values():
        response = uav.http.post(url + '/mock/set', params={'battery_pct': 0 if name == 'uav_rejected' else 95}, timeout=5)
        response.raise_for_status()
    fleet = GroundFleet(use_px4=False)
    await fleet.connect_all()
    if name == 'blocked':
        fleet.set_road_blocked('E1', True)
        fleet.set_road_blocked('E6', True)
    task = environment.task_for_cell(x, y, 'TASK_' + uuid.uuid4().hex[:12], None if name == 'unknown_target' else 'F1')
    logger = EventLogger('RUN_' + uuid.uuid4().hex[:12], scenario_id=name,
                         log_dir=str(output_dir), file_name=f'{name}.jsonl')
    logger.log_event('RUN_CONTEXT', time.time(), detail={
        'environment': 'existing CA + repository Raster', 'uav': 'existing HTTP API + MockDrone',
        'ugv': 'GroundFleet + SimDriver + graph_data_demo', 'safety': 'existing MOCK',
        'fire_cell': [x,y], 'explicit_target_node': task.target_node,
        'ignition_source': 'explicit scenario via existing ignite_at',
        'fire_cell_fuel_amount': environment.api.grid.get_cell(x,y).fuel_amount,
        'ugv_thermal_sensor': 'NOT_IMPLEMENTED',
    })
    before = environment.api.state_version
    orchestrator = Orchestrator(environment, uav, GroundAdapter(fleet), safety_connector.verify, logger)
    result = await orchestrator.run(task)
    return {**result, 'scenario': name, 'environment_version_before': before,
            'environment_version_after': environment.api.state_version,
            'log': logger.log_path}, logger.read_all()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=(*SCENARIOS, 'all'), default='normal')
    parser.add_argument('--output-dir', default='logs/orchestrator_v012')
    args = parser.parse_args()
    output = Path(args.output_dir).resolve(); output.mkdir(parents=True, exist_ok=True)
    environment = EnvironmentAdapter()
    with mock_servers(output) as uav:
        for name in SCENARIOS if args.scenario == 'all' else (args.scenario,):
            result, _ = asyncio.run(scenario(name, uav, environment, output))
            print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
