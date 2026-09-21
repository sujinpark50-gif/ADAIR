"""Wildfire Cellular Automaton (CA) fire spread engine.

Implements discrete time-step simulation of wildfire propagation across an
:class:`EnvironmentGrid`.

AUTHORITY & SPECIFICATION COMPLIANCE:
------------------------------------
- Authority: `mission/environmental-modeling.txt` §4 (Fire Implementation Procedure).
- State transition machine: `UNBURNED -> BURNING -> BURNED`.
- NO scientific constants, spread probability weights, or burn duration steps are
  hardcoded in this source code. All scientific modeling parameters are loaded directly
  from `config/environment_config.yaml`.
- Seeded execution (`seed`) guarantees bitwise reproducible simulation runs.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, Union, TYPE_CHECKING

import numpy as np
import yaml

from src.environment.cell import Cell, FireState
from src.environment.risk import update_grid_risk_scores

if TYPE_CHECKING:
    from src.environment.grid import EnvironmentGrid


def _load_config(config_input: Union[str, Path, dict]) -> dict:
    """Load configuration from file path or return dict directly."""
    if isinstance(config_input, dict):
        return config_input
    path = Path(config_input)
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


class WildfireCAEngine:
    """Cellular Automaton simulation engine for wildfire spread.

    Parameters
    ----------
    grid : EnvironmentGrid
        The underlying 2D simulation terrain grid.
    config_path : str | Path | dict, default="config/environment_config.yaml"
        Configuration file path or dictionary containing CA simulation parameters.
    seed : int, optional
        Random seed for bitwise reproducible simulation execution. If None,
        defaults to the seed attached to *grid*.
    """

    def __init__(
        self,
        grid: EnvironmentGrid,
        config_path: Union[str, Path, dict] = "config/environment_config.yaml",
        seed: Optional[int] = None,
    ) -> None:
        self._grid: EnvironmentGrid = grid
        self._config: dict = _load_config(config_path)

        # Establish random seed from argument or grid property
        effective_seed: Optional[int] = seed if seed is not None else grid.seed
        self._seed: Optional[int] = effective_seed
        self._rng: np.random.Generator = np.random.default_rng(effective_seed)

        # Track steps spent in BURNING state for each burning cell
        self._burn_counters: Dict[int, int] = {}

        # Load CA parameters from config (no hardcoded scientific constants)
        ca_cfg = self._config.get("fire_ca", {})
        self._base_spread_prob: float = float(ca_cfg.get("base_spread_probability", 0.2))
        self._burn_duration: int = int(ca_cfg.get("burn_duration_steps", 3))
        self._slope_weight: float = float(ca_cfg.get("slope_factor_weight", 0.05))
        self._wind_weight: float = float(ca_cfg.get("wind_factor_weight", 0.1))
        self._moisture_weight: float = float(ca_cfg.get("moisture_dampening_weight", 1.0))
        self._neighbor_mode: str = str(ca_cfg.get("neighbor_mode", "8-neighbor"))

        # Initialize risk scores across the grid
        update_grid_risk_scores(self._grid, self._config)

    @property
    def grid(self) -> EnvironmentGrid:
        """Return the environment grid instance."""
        return self._grid

    @property
    def seed(self) -> Optional[int]:
        """Return the simulation random seed."""
        return self._seed

    def ignite_at(self, x: int, y: int) -> Cell:
        """Ignite cell at grid position (col *x*, row *y*).

        Transitions cell state from `UNBURNED` to `BURNING`.
        Raises `IndexError` if coordinates are out of bounds.
        """
        cell = self._grid.get_cell(x, y)
        cell.fire_state = FireState.BURNING
        self._burn_counters[cell.cell_id] = 0
        update_grid_risk_scores(self._grid, self._config)
        return cell

    def ignite_cell(self, cell_id: int) -> Cell:
        """Ignite cell with unique *cell_id*.

        Transitions cell state from `UNBURNED` to `BURNING`.
        Raises `KeyError` if cell_id is not found.
        """
        cell = self._grid.get_cell_by_id(cell_id)
        cell.fire_state = FireState.BURNING
        self._burn_counters[cell.cell_id] = 0
        update_grid_risk_scores(self._grid, self._config)
        return cell

    def _calculate_spread_probability(self, source: Cell, target: Cell) -> float:
        """Calculate fire spread probability from *source* (BURNING) to *target* (UNBURNED).

        Modifiers:
          - Fuel amount: target.fuel_amount (if 0.0, zero spread)
          - Moisture: max(0, 1 - moisture_weight * target.moisture)
          - Slope: exp(slope_weight * gradient), where gradient = dz / dist
          - Wind: exp(wind_weight * speed * alignment), where alignment = dot product of
            direction vector (source -> target) with wind vector
        """
        if target.fuel_amount <= 0.0 or target.fire_state != FireState.UNBURNED:
            return 0.0       # 연료 없거나 이미 탄 셀은 확산 불가

        # Base fuel factor
        fuel_factor = target.fuel_amount     # 연료가 많을수록 잘 붙음

        # Moisture dampening factor     # 습도가 높을수록 확산 확률 감소
        moisture_factor = max(0.0, 1.0 - self._moisture_weight * target.moisture)
        if moisture_factor <= 0.0:
            return 0.0

        # Slope factor (elevation gradient from source to target)
        dz = target.elevation - source.elevation        # 고도 차이
        dx_cell = target.x - source.x
        dy_cell = target.y - source.y
        cell_dist_m = math.hypot(dx_cell, dy_cell) * self._grid.cell_resolution
        gradient = (dz / cell_dist_m) if cell_dist_m > 0 else 0.0       # 경사도
        slope_factor = math.exp(self._slope_weight * gradient)          # 오르막(dz > 0)이면 확산 증가, 내리막이면 감소

        # Wind factor (wind vector direction alignment)
        wind_cfg = self._config.get("wind", {})
        wind_speed = float(wind_cfg.get("speed_ms", 0.0))
        wind_deg = float(wind_cfg.get("direction_deg", 270.0))

        # Wind blowing towards (in grid coordinates: East is +x, South is +y, North is -y, West is -x)
        towards_deg = (wind_deg + 180.0) % 360.0        # 바람이 불어가는 방향 계산
        towards_rad = math.radians(towards_deg)
        wind_dx = math.sin(towards_rad)
        wind_dy = -math.cos(towards_rad)

        dist_cell = math.hypot(dx_cell, dy_cell)
        if dist_cell > 0:
            ux = dx_cell / dist_cell
            uy = dy_cell / dist_cell
            alignment = ux * wind_dx + uy * wind_dy     # source → target 방향과 바람 방향의 일치도 (내적) (1: 확산 최대; 0: 중립; -1: 확산 억제)
        else:
            alignment = 0.0

        wind_factor = math.exp(self._wind_weight * wind_speed * alignment)      # 바람 방향과 일치할수록 확산 증가

        p = self._base_spread_prob * fuel_factor * moisture_factor * slope_factor * wind_factor     # 최종 확률 계산
        return max(0.0, min(1.0, p))        # 0~1 사이로

    def step(self) -> List[Cell]:
        """Execute one discrete simulation step.

        Process:
          1. Age currently BURNING cells; transition cells exceeding `burn_duration_steps` from `BURNING -> BURNED`.
             BURNING 셀 노화 → burn_duration 초과 시 BURNED로 전환
          2. For each active `BURNING` cell, identify `UNBURNED` neighbors.
             활성 BURNING 셀의 UNBURNED 이웃 셀 탐색
          3. Compute combined spread probability for candidate `UNBURNED` cells.
             여러 BURNING 셀에서 동시에 받는 확산 확률 합산
          4. Evaluate stochastic ignition using seeded RNG (`self._rng`).
             난수(RNG)로 발화 여부 결정
          5. Update states of newly ignited cells to `BURNING`.
             새로 발화된 셀 UNBURNED → BURNING 전환
          6. Recalculate risk scores across the grid.
             전체 격자 위험도(risk_score) 재계산

        Returns
        -------
        List[Cell]
            List of cells that newly ignited during this step.
        """
        # Step 1: Age BURNING cells and transition to BURNED if duration reached
        currently_burning = [c for c in self._grid.cells if c.fire_state == FireState.BURNING]
        for cell in currently_burning:
            cell_id = cell.cell_id
            self._burn_counters[cell_id] = self._burn_counters.get(cell_id, 0) + 1
            if self._burn_counters[cell_id] >= self._burn_duration:
                cell.fire_state = FireState.BURNED

        # Refresh list of active BURNING cells after aging
        active_burning = [c for c in self._grid.cells if c.fire_state == FireState.BURNING]

        # Step 2: Find all candidate UNBURNED neighbors of active BURNING cells
        # Group by candidate UNBURNED cell ID to combine probabilities from multiple burning neighbors
        candidate_sources: Dict[int, List[Tuple[Cell, Cell]]] = {}

        for burning_cell in active_burning:
            neighbors = self._grid.get_neighbors(
                burning_cell.x, burning_cell.y, mode=self._neighbor_mode  # type: ignore
            )
            for neighbor in neighbors:
                if neighbor.fire_state == FireState.UNBURNED and neighbor.fuel_amount > 0.0:
                    cid = neighbor.cell_id
                    if cid not in candidate_sources:
                        candidate_sources[cid] = []
                    candidate_sources[cid].append((burning_cell, neighbor))

        # Step 3 & 4: Evaluate spread probability and roll RNG in deterministic (sorted) order
        newly_ignited: List[Cell] = []
        sorted_candidate_ids = sorted(candidate_sources.keys())

        for cid in sorted_candidate_ids:
            pairs = candidate_sources[cid]
            target_cell = pairs[0][1]

            # Calculate combined probability of NOT igniting from any neighbor
            prob_not_igniting = 1.0
            for source_cell, _ in pairs:
                p_spread = self._calculate_spread_probability(source_cell, target_cell)
                prob_not_igniting *= (1.0 - p_spread)

            p_combined = 1.0 - prob_not_igniting

            # Roll RNG (deterministic order guaranteed by sorted cell IDs)
            roll = float(self._rng.random())
            if roll < p_combined:
                newly_ignited.append(target_cell)

        # Step 5: Transition newly ignited cells UNBURNED -> BURNING
        for cell in newly_ignited:
            cell.fire_state = FireState.BURNING
            self._burn_counters[cell.cell_id] = 0

        # Step 6: Recalculate risk scores across the grid
        update_grid_risk_scores(self._grid, self._config)

        return newly_ignited
