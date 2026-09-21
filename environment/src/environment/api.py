"""Public API interface exposed by the Environment & Wildfire Model.

Provides query and command interfaces for the Master Orchestrator, UAV/UGV agents,
and System Integration modules.

AUTHORITY & SPECIFICATION COMPLIANCE:
------------------------------------
- Authority: `mission/environmental-modeling.txt` §5 (Interfacing Information) & §6.
- Authority: `specs/environment_acceptance_criteria.md` §2.3 & §2.4.

PROVISIONAL INTEGRATION ASSUMPTIONS NOTICE:
-------------------------------------------
Cross-subsystem payload schemas, timestamp formats, and message identifiers with
the Master Orchestrator and System Integration Leads are subject to ongoing consensus.
All external schemas in this API are isolated behind adapter wrappers and documented
as PROVISIONAL INTEGRATION ASSUMPTIONS so that final schemas can replace them without
restructuring the core environment engine.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Union, TYPE_CHECKING

from src.environment.cell import Cell, FireState
from src.environment.observation import ObservationReport, ObservationUpdateHandler

if TYPE_CHECKING:
    from src.environment.fire_model import WildfireCAEngine
    from src.environment.grid import EnvironmentGrid


class EnvironmentModelAPI:
    """Public API gateway wrapping the environment grid and fire engine.

    Parameters
    ----------
    grid : EnvironmentGrid
        The simulation environment grid.
    ca_engine : WildfireCAEngine
        The cellular automaton simulation engine.
    """

    def __init__(self, grid: EnvironmentGrid, ca_engine: WildfireCAEngine) -> None:
        self._grid: EnvironmentGrid = grid
        self._ca_engine: WildfireCAEngine = ca_engine
        self._handler: ObservationUpdateHandler = ObservationUpdateHandler(grid, ca_engine)

        self._state_version: int = 1
        self._last_updated: str = datetime.now(timezone.utc).isoformat()

    @property
    def grid(self) -> EnvironmentGrid:
        """Return the underlying environment grid."""
        return self._grid

    @property
    def ca_engine(self) -> WildfireCAEngine:
        """Return the underlying cellular automaton engine."""
        return self._ca_engine

    @property
    def state_version(self) -> int:
        """Monotonically increasing version identifier for environment state updates."""
        return self._state_version

    @property
    def last_updated(self) -> str:
        """ISO 8601 UTC timestamp of the most recent environment state update."""
        return self._last_updated

    def get_fire_locations(self) -> List[Dict[str, Any]]:
        """Return list of active BURNING cell locations and attributes.

        PROVISIONAL INTEGRATION SCHEMA:
        Returns list of dicts with key attributes: cell_id, x, y, elevation, risk_score.
        """
        burning_cells = [c for c in self._grid.cells if c.fire_state == FireState.BURNING]
        return [
            {
                "cell_id": cell.cell_id,
                "x": cell.x,
                "y": cell.y,
                "elevation": cell.elevation,
                "risk_score": cell.risk_score,
            }
            for cell in burning_cells
        ]

    def get_fire_perimeter(self) -> List[Dict[str, Any]]:
        """Return list of fire boundary cells adjacent to UNBURNED terrain.

        Perimeter cells are BURNING or BURNED cells that have at least one valid
        neighbor in the UNBURNED state.
        """
        perimeter: List[Dict[str, Any]] = []
        for cell in self._grid.cells:
            if cell.fire_state in (FireState.BURNING, FireState.BURNED):
                neighbors = self._grid.get_neighbors(cell.x, cell.y, mode="8-neighbor")
                if any(n.fire_state == FireState.UNBURNED for n in neighbors):
                    perimeter.append({
                        "cell_id": cell.cell_id,
                        "x": cell.x,
                        "y": cell.y,
                        "fire_state": cell.fire_state.value,
                    })
        return perimeter

    def get_risk_map(self) -> Dict[int, float]:
        """Return mapping of cell_id to risk_score across all grid cells."""
        return {cell.cell_id: cell.risk_score for cell in self._grid.cells}

    def get_spread_direction(self) -> Dict[str, Any]:
        """Calculate predicted primary fire spread direction vector.

        Combines active burning front orientation with current wind vector settings.

        Returns
        -------
        dict
            Contains dx, dy direction components, wind_speed_ms, wind_direction_deg,
            and primary cardinal direction.
        """
        wind_cfg = self._ca_engine._config.get("wind", {})
        wind_speed = float(wind_cfg.get("speed_ms", 0.0))
        wind_deg = float(wind_cfg.get("direction_deg", 270.0))

        # Wind blowing towards in grid coordinates
        towards_deg = (wind_deg + 180.0) % 360.0
        towards_rad = math.radians(towards_deg)
        wind_dx = math.sin(towards_rad)
        wind_dy = -math.cos(towards_rad)

        # Primary cardinal direction label
        cardinals = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        idx = int(round(towards_deg / 45.0)) % 8
        cardinal_label = cardinals[idx]

        return {
            "dx": wind_dx,
            "dy": wind_dy,
            "wind_speed_ms": wind_speed,
            "wind_direction_deg": wind_deg,
            "primary_cardinal": cardinal_label,
        }

    def get_environment_state(self) -> Dict[str, Any]:
        """Return comprehensive environment model snapshot metadata."""
        burning = [c for c in self._grid.cells if c.fire_state == FireState.BURNING]
        burned = [c for c in self._grid.cells if c.fire_state == FireState.BURNED]
        unburned = [c for c in self._grid.cells if c.fire_state == FireState.UNBURNED]

        return {
            "state_version": self._state_version,
            "timestamp": self._last_updated,
            "crs": self._grid.crs,
            "grid_rows": self._grid.rows,
            "grid_cols": self._grid.cols,
            "total_cells": len(self._grid.cells),
            "unburned_cell_count": len(unburned),
            "burning_cell_count": len(burning),
            "burned_cell_count": len(burned),
        }

    def apply_observation(
        self, observation: Union[dict, ObservationReport]
    ) -> Dict[str, Any]:
        """Process a field observation update report.

        Upon successful state update, increments ``state_version`` and updates ``timestamp``.
        """
        result = self._handler.process_observation(observation)
        if result.get("success", False):
            self._state_version += 1
            self._last_updated = datetime.now(timezone.utc).isoformat()
            result["state_version"] = self._state_version
            result["timestamp"] = self._last_updated
        return result

    def step(self) -> List[Cell]:
        """Execute one discrete cellular automaton simulation step.

        Increments ``state_version`` and updates ``timestamp``.
        """
        newly_ignited = self._ca_engine.step()
        self._state_version += 1
        self._last_updated = datetime.now(timezone.utc).isoformat()
        return newly_ignited
