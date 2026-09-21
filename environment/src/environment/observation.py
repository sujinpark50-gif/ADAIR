"""Field Observation Update Handler for the Wildfire Simulation Environment.

Processes ground and aerial (UAV/UGV) observation reports, validates coordinates
and authorized fields, updates cell/environment states, and triggers dynamic
recalculation of risk scores and fire spread predictions.

AUTHORITY & SPECIFICATION COMPLIANCE:
------------------------------------
- Authority: `mission/environmental-modeling.txt` §4 Step 7 & §5.
- Authority: `specs/environment_acceptance_criteria.md` §2.3.
- Only authorized environmental attributes (`fire_state`, `moisture`, `wind`) can be updated.
- Invalid or out-of-bounds observations are validated and handled explicitly without corrupting grid state.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Union, TYPE_CHECKING

from src.environment.cell import Cell, FireState
from src.environment.risk import update_grid_risk_scores

if TYPE_CHECKING:
    from src.environment.fire_model import WildfireCAEngine
    from src.environment.grid import EnvironmentGrid

logger = logging.getLogger(__name__)


@dataclass
class ObservationReport:
    """Provisional structured container for field observation reports.

    PROVISIONAL INTEGRATION ASSUMPTION:
    Cross-subsystem payload schema with Orchestrator/UAV/UGV leads is pending.
    This dataclass provides a clean internal domain representation that can be
    adapted when official JSON/protobuf schemas are agreed upon.
    """

    reporter_id: str
    x: Optional[int] = None
    y: Optional[int] = None
    cell_id: Optional[int] = None
    fire_state: Optional[Union[str, FireState]] = None
    moisture: Optional[float] = None
    wind_speed_ms: Optional[float] = None
    wind_direction_deg: Optional[float] = None
    timestamp: Optional[str] = None


class ObservationUpdateHandler:
    """Processes observation reports and updates authorized environment states.

    Parameters
    ----------
    grid : EnvironmentGrid
        The environment grid instance to update.
    ca_engine : WildfireCAEngine
        The active cellular automaton engine instance.
    """

    def __init__(self, grid: EnvironmentGrid, ca_engine: WildfireCAEngine) -> None:
        self._grid: EnvironmentGrid = grid
        self._ca_engine: WildfireCAEngine = ca_engine

    def process_observation(
        self, observation: Union[dict, ObservationReport]
    ) -> Dict[str, Any]:
        """Validate and apply a field observation update to the environment model.

        Parameters
        ----------
        observation : dict or ObservationReport
            Observation data payload containing coordinates and state updates.

        Returns
        -------
        dict
            Processing status result containing ``success``, ``cell_id``,
            ``updated_fields``, or ``error``.
        """
        # Normalize dict or dataclass to standard dict
        if isinstance(observation, ObservationReport):
            data = {
                "reporter_id": observation.reporter_id,
                "x": observation.x,
                "y": observation.y,
                "cell_id": observation.cell_id,
                "fire_state": observation.fire_state,
                "moisture": observation.moisture,
                "wind_speed_ms": observation.wind_speed_ms,
                "wind_direction_deg": observation.wind_direction_deg,
                "timestamp": observation.timestamp,
            }
        elif isinstance(observation, dict):
            data = observation
        else:
            return {"success": False, "error": f"Unsupported observation payload type: {type(observation)}"}

        # 1. Resolve target cell by cell_id, (x, y) grid indices, or world coordinates
        cell: Optional[Cell] = None
        if data.get("cell_id") is not None:
            try:
                cell = self._grid.get_cell_by_id(int(data["cell_id"]))
            except KeyError:
                return {"success": False, "error": f"Cell ID {data['cell_id']} not found on grid."}
        elif data.get("x") is not None and data.get("y") is not None:
            try:
                cell = self._grid.get_cell(int(data["x"]), int(data["y"]))
            except IndexError:
                return {
                    "success": False,
                    "error": f"Coordinates ({data['x']}, {data['y']}) out of grid bounds.",
                }
        elif data.get("world_x") is not None and data.get("world_y") is not None:
            try:
                row, col = self._grid.xy_to_rowcol(float(data["world_x"]), float(data["world_y"]))
                cell = self._grid.get_cell(col, row)
            except IndexError:
                return {
                    "success": False,
                    "error": f"World coordinates ({data['world_x']}, {data['world_y']}) out of grid bounds.",
                }

        updated_fields: List[str] = []

        # 2. Update cell-level attributes if target cell specified
        if cell is not None:
            # Fire state update
            raw_state = data.get("fire_state")
            if raw_state is not None:
                new_state: Optional[FireState] = None
                if isinstance(raw_state, FireState):
                    new_state = raw_state
                elif isinstance(raw_state, str):
                    try:
                        new_state = FireState[raw_state.upper()]
                    except KeyError:
                        try:
                            new_state = FireState(raw_state.upper())
                        except ValueError:
                            pass
                if new_state is not None:
                    cell.fire_state = new_state
                    if new_state == FireState.BURNING:
                        # Register in CA engine burn counters
                        self._ca_engine._burn_counters[cell.cell_id] = 0
                    updated_fields.append("fire_state")

            # Fuel moisture update
            raw_moisture = data.get("moisture")
            if raw_moisture is not None:
                try:
                    m_val = float(raw_moisture)
                    if 0.0 <= m_val <= 1.0:
                        cell.moisture = m_val
                        updated_fields.append("moisture")
                    else:
                        logger.warning(f"Moisture value {m_val} out of range [0.0, 1.0], skipping.")
                except (ValueError, TypeError):
                    pass

        # 3. Update global environmental attributes (wind vector) if reported
        wind_updated = False
        wind_cfg = self._ca_engine._config.setdefault("wind", {})
        if data.get("wind_speed_ms") is not None:
            try:
                wind_cfg["speed_ms"] = float(data["wind_speed_ms"])
                updated_fields.append("wind_speed_ms")
                wind_updated = True
            except (ValueError, TypeError):
                pass
        if data.get("wind_direction_deg") is not None:
            try:
                wind_cfg["direction_deg"] = float(data["wind_direction_deg"]) % 360.0
                updated_fields.append("wind_direction_deg")
                wind_updated = True
            except (ValueError, TypeError):
                pass

        # Require at least one valid cell coordinate or global update
        if cell is None and not wind_updated:
            return {
                "success": False,
                "error": "No valid cell coordinates or global environmental update specified.",
            }

        # 4. Trigger dynamic recalculation of risk scores upon accepted update
        update_grid_risk_scores(self._grid, self._ca_engine._config)

        return {
            "success": True,
            "cell_id": cell.cell_id if cell else None,
            "updated_fields": updated_fields,
            "reporter_id": data.get("reporter_id", "UNKNOWN"),
        }
