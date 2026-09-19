# -*- coding: utf-8 -*-
"""Mock 기반 총괄 오케스트레이션 폐루프 시연.

실행:
    python orchestration_demo.py
"""

import ids
from connectors import fire_connector, orchestrator_connector, uav_connector
from interfaces.schema import ResourcePool
from logger import EventLogger
from main import process_task


def _uav_pool() -> ResourcePool:
    return ResourcePool(base_a=uav_connector.get_uav_status("A"), base_b=[])


def run_scenario(scenario_id: str, forced_responses: dict) -> None:
    print(f"\n=== {scenario_id} ===")
    timestamp = 0.0
    logger = EventLogger(
        run_id=ids.new_run_id(),
        scenario_id=scenario_id,
        file_name=f"{scenario_id.lower().replace('-', '_')}_demo.jsonl",
        console=True,
    )
    env_state = fire_connector.get_environment_state(timestamp)
    fire_cell = env_state.fire_cells[0]["cell_id"]
    logger.log_event(
        "ENVIRONMENT_CHANGE", timestamp, result="FIRE_DETECTED",
        detail={"cell_id": fire_cell},
    )
    task = orchestrator_connector.create_task(timestamp, env_state)
    task, env_state = process_task(
        task, env_state, _uav_pool(), logger, timestamp,
        forced_responses=forced_responses,
    )
    assert task.state == "COMPLETED"
    assert env_state.env_updated


def main() -> None:
    run_scenario(
        "IT-01",
        {"A-uav1": ("ACCEPT", None), "A-uav2": ("ACCEPT", None)},
    )
    run_scenario(
        "IT-02",
        {"A-uav1": ("REJECT", "HIGH_WIND"), "A-uav2": ("ACCEPT", None)},
    )
    print("\n[DONE] IT-01, IT-02 mock orchestration completed")


if __name__ == "__main__":
    main()
