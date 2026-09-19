"""Risk scoring engine for grid cells in the wildfire simulation environment.

UNRESOLVED MODELING DECISION & SPECIFICATION COMPLIANCE NOTICE:
----------------------------------------------------------------
Per `mission/environmental-modeling.txt` §3.3 (line 72):
  "risk_score의 계산 기준과 값의 범위는 총괄·통합 담당과 합의한다."
  ("The calculation criteria and value range of risk_score are
  agreed with the Master Orchestrator and System Integration Leads.")

Because final formal agreement across subsystems has not yet been established,
the calculation in this module serves as a **temporary scenario modeling assumption**.
It is implemented in a modular, configuration-driven manner so that when the final
agreed-upon formula and bounds are specified, they can directly replace the contents
of this function without altering the Phase 1 EnvironmentGrid or Phase 2 CA Engine.

All parameters used in this calculation are loaded from `config/environment_config.yaml`
under the `risk_scoring` key — zero scientific constants are hardcoded here.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Set, Tuple, TYPE_CHECKING

from src.environment.cell import Cell, FireState

if TYPE_CHECKING:
    from src.environment.grid import EnvironmentGrid


def _compute_wind_vector(config: dict) -> Tuple[float, float]:
    """Return wind direction unit vector in grid coordinates.

    Notes
    -----
    - ``direction_deg`` is interpreted as the direction the wind comes *from*.
    - For downwind checks, we convert it to the direction the wind blows *towards*.
    - Grid coordinates use:
        * +x : east
        * +y : south  아래로 증가 (위로 증가 아님)
      so y uses negative cosine.
    """
    wind_cfg = config.get("wind", {})
    wind_deg = float(wind_cfg.get("direction_deg", 270.0))

    # Convert "coming from" direction to "blowing towards" direction.
    towards_deg = (wind_deg + 180.0) % 360.0
    towards_rad = math.radians(towards_deg)

    wind_dx = math.sin(towards_rad)
    wind_dy = -math.cos(towards_rad)    # 예: towards_rad = 0 북향 바람: wind_dy = -cos(0°) = -1: (0, -1) 북쪽 방향
    return wind_dx, wind_dy


def _collect_downwind_cells(
    grid: EnvironmentGrid,
    burning_cells: list[Cell],
    wind_dx: float,
    wind_dy: float,
    downwind_buffer: int,
    downwind_dot_thresh: float,
) -> Set[Tuple[int, int]]:
    """Precompute coordinates of cells that are near and downwind of any burning cell.

    Why this helper exists
    ----------------------
    The old approach checked every grid cell against every burning cell, which is
    roughly O(N * B), where:
    - N = total number of cells
    - B = number of burning cells

    However, downwind influence only matters within ``downwind_buffer`` cells.
    So instead of global search, we iterate locally around each burning cell only.
    This reduces the work to roughly O(B * r^2), where r is the buffer radius.

    Implementation detail
    ---------------------
    For each burning cell, only cells inside the square bounding box around radius r
    are visited, and then filtered by:
    1. radial distance <= r
    2. alignment with wind direction

    To reduce repeated sqrt/division cost, the common case
    (``downwind_dot_thresh >= 0``) uses squared-distance and projection comparison.
    """
    if not burning_cells or downwind_buffer <= 0:
        return set()

    rows = grid.rows
    cols = grid.cols
    r = downwind_buffer
    r2 = r * r
    downwind_coords: Set[Tuple[int, int]] = set()

    # For typical thresholds like 0.3, squared comparison avoids sqrt/division.
    use_squared_compare = downwind_dot_thresh >= 0.0
    thresh2 = downwind_dot_thresh * downwind_dot_thresh

    for bc in burning_cells:
        min_x = max(0, bc.x - r)
        max_x = min(cols - 1, bc.x + r)
        min_y = max(0, bc.y - r)
        max_y = min(rows - 1, bc.y + r)

        for ny in range(min_y, max_y + 1):
            dy = ny - bc.y
            for nx in range(min_x, max_x + 1):
                dx = nx - bc.x
                dist2 = dx * dx + dy * dy

                # Skip the source burning cell itself and cells beyond buffer radius.
                if dist2 == 0 or dist2 > r2:
                    continue

                # Projection of (dx, dy) onto the wind direction vector.
                proj = dx * wind_dx + dy * wind_dy

                if use_squared_compare:
                    # Equivalent to:
                    #   (proj / sqrt(dist2)) > downwind_dot_thresh
                    # but avoids sqrt when threshold >= 0.
                    #
                    # Need proj > 0 first; otherwise squaring would lose sign.
                    if proj <= 0.0:
                        continue
                    if (proj * proj) > (thresh2 * dist2):
                        downwind_coords.add((nx, ny))
                else:
                    # Rare fallback path for negative thresholds to preserve semantics.
                    dist = math.sqrt(dist2)
                    dot = proj / dist
                    if dot > downwind_dot_thresh:
                        downwind_coords.add((nx, ny))

    return downwind_coords


def calculate_risk_scores(
    grid: EnvironmentGrid,
    config: Optional[dict] = None,
) -> Dict[int, float]:
    """Calculate and return risk scores for all cells in *grid*.

    Parameters
    ----------
    grid : EnvironmentGrid
        The simulation environment grid.
    config : dict, optional
        Configuration dictionary containing a ``risk_scoring`` section.
        If None, attempts to use configuration attached to *grid*.

    Returns
    -------
    Dict[int, float]
        Dictionary mapping ``cell_id`` to calculated risk score.
    """
    if config is None:
        config = getattr(grid, "_config", {})

    cfg = config.get("risk_scoring", {})
    burning_risk = float(cfg.get("burning_cell_risk", 1.0))
    burned_risk = float(cfg.get("burned_cell_risk", 0.2))
    unburned_base_risk = float(cfg.get("unburned_cell_base_risk", 0.0))
    downwind_buffer = int(cfg.get("downwind_buffer_cells", 3))
    downwind_weight = float(cfg.get("downwind_risk_weight", 0.4))
    slope_weight = float(cfg.get("slope_risk_weight", 0.2))
    max_slope_deg = float(cfg.get("max_slope_degrees", 45.0))
    downwind_dot_thresh = float(cfg.get("downwind_dot_threshold", 0.3))
    fuel_weight = float(cfg.get("fuel_risk_weight", 0.2))
    min_risk = float(cfg.get("min_risk_score", 0.0))
    max_risk = float(cfg.get("max_risk_score", 1.0))

    # grid.cells creates a new flat list each time, so materialize it once.
    cells = grid.cells

    # Collect burning cells once.
    burning_cells = [c for c in cells if c.fire_state == FireState.BURNING]

    # Compute wind vector once.
    wind_dx, wind_dy = _compute_wind_vector(config)

    # Precompute cells that are within radius and aligned with wind from any burning cell.
    downwind_near_coords = _collect_downwind_cells(
        grid=grid,
        burning_cells=burning_cells,
        wind_dx=wind_dx,
        wind_dy=wind_dy,
        downwind_buffer=downwind_buffer,
        downwind_dot_thresh=downwind_dot_thresh,
    )

    risk_scores: Dict[int, float] = {}

    for cell in cells:
        if cell.fire_state == FireState.BURNING:
            score = burning_risk
        elif cell.fire_state == FireState.BURNED:
            score = burned_risk
        else:
            score = unburned_base_risk

            # Add slope component:
            # normalize slope by max_slope_deg and scale by slope_weight.
            norm_slope = (
                min(1.0, max(0.0, cell.slope / max_slope_deg))
                if max_slope_deg > 0
                else 0.0
            )
            score += slope_weight * norm_slope

            # Add fuel amount component:
            # more available fuel -> higher risk.
            score += fuel_weight * min(1.0, max(0.0, cell.fuel_amount))

            # Add downwind risk if this cell is precomputed as near + downwind.
            if (cell.x, cell.y) in downwind_near_coords:
                score += downwind_weight

        # Enforce configurable bounds.
        clamped_score = max(min_risk, min(max_risk, score))
        risk_scores[cell.cell_id] = float(clamped_score)

    return risk_scores


def update_grid_risk_scores(
    grid: EnvironmentGrid,
    config: Optional[dict] = None,
) -> Dict[int, float]:
    """Calculate risk scores and update ``cell.risk_score`` on *grid* cells.

    Returns mapping of ``cell_id`` to updated risk score.
    """
    scores = calculate_risk_scores(grid, config)

    # grid.cells creates a new list, so only materialize once here too.
    cells = grid.cells
    for cell in cells:
        cell.risk_score = scores[cell.cell_id]

    return scores