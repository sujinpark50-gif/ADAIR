# -*- coding: utf-8 -*-
"""
connectors/fire_connector.py
==============================
환경 Connector (Contract 7번 "환경 Connector"):
- 현재 환경 상태 제공
- 관측 결과 수신
- 환경 모델 갱신
"""

import random
import config
from interfaces.schema import EnvironmentState


def get_environment_state(timestamp: float) -> EnvironmentState:
    if config.FIRE_CONNECTION_MODE == "mock":
        return _get_environment_state_mock(timestamp)
    raise NotImplementedError("환경모델 실제 연동이 아직 구현되지 않았습니다.")


def _get_environment_state_mock(timestamp: float) -> EnvironmentState:
    fire_cells = [
        {
            "cell_id": "50_50",
            "x": 50,
            "y": 50,
            "fire_state": "BURNING",
            "risk_score": round(random.uniform(0.5, 0.9), 2),
        }
    ]
    risk_zone = [
        {"cell_id": "51_50", "x": 51, "y": 50, "risk_score": 0.6},
        {"cell_id": "50_51", "x": 50, "y": 51, "risk_score": 0.55},
    ]
    return EnvironmentState(
        timestamp=timestamp,
        fire_cells=fire_cells,
        risk_zone=risk_zone,
        wind_speed=round(random.uniform(1.0, 8.0), 1),
        wind_direction=round(random.uniform(0, 360), 1),
        spread_direction=random.choice(["N", "NE", "E", "SE", "S", "SW", "W", "NW"]),
        env_updated=False,
    )


def update_environment(env_state: EnvironmentState, observations: list) -> EnvironmentState:
    """실행 후 관측 결과를 환경모델에 반영 (Mock: 갱신 플래그만 표시)"""
    if config.FIRE_CONNECTION_MODE == "mock":
        if observations:
            env_state.env_updated = True
        return env_state
    raise NotImplementedError("환경모델 실제 갱신 로직이 아직 구현되지 않았습니다.")