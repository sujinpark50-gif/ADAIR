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

from datetime import datetime, timedelta
import logging
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple, Union, TYPE_CHECKING

import numpy as np
import yaml

from src.cell import Cell, FireState
from src.risk import update_grid_risk_scores
from src.weather import (
    WeatherProvider,
    WeatherState,
    create_weather_provider,
    to_toward_direction,
)

if TYPE_CHECKING:
    from src.grid import EnvironmentGrid

logger = logging.getLogger(__name__)


def _load_config(config_input: Union[str, Path, dict]) -> dict:
    """Load configuration from file path or return dict directly."""
    if isinstance(config_input, dict):
        return config_input
    path = Path(config_input)
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _compute_wind_vector(
    config: dict, weather: Optional[WeatherState] = None
) -> Tuple[float, float]:
    """Return wind direction unit vector in grid coordinates (pointing TOWARD spread direction)."""
    if weather is not None:
        towards_deg = float(weather.wind_direction_deg)
    else:
        wind_cfg = config.get("wind", {})
        raw_deg = float(wind_cfg.get("direction_deg", 270.0))
        conv = config.get("weather", {}).get("wind_direction_convention", "from")
        towards_deg = to_toward_direction(raw_deg, conv)

    towards_rad = math.radians(towards_deg)
    wind_dx = math.sin(towards_rad)
    wind_dy = -math.cos(towards_rad)
    return wind_dx, wind_dy


def _collect_downwind_cells(
    grid: EnvironmentGrid,
    burning_cells: list[Cell],
    wind_dx: float,
    wind_dy: float,
    downwind_buffer: int,
    downwind_dot_thresh: float,
) -> Set[Tuple[int, int]]:
    """Precompute coordinates of cells that are near and downwind of any burning cell."""
    if not burning_cells or downwind_buffer <= 0:
        return set()

    rows = grid.rows
    cols = grid.cols
    r = downwind_buffer
    r2 = r * r
    downwind_coords: Set[Tuple[int, int]] = set()

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

                if dist2 == 0 or dist2 > r2:
                    continue

                proj = dx * wind_dx + dy * wind_dy

                if use_squared_compare:
                    if proj <= 0.0:
                        continue
                    if (proj * proj) > (thresh2 * dist2):
                        downwind_coords.add((nx, ny))
                else:
                    dist = math.sqrt(dist2)
                    dot = proj / dist
                    if dot > downwind_dot_thresh:
                        downwind_coords.add((nx, ny))

    return downwind_coords


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

        # Establish random seed from argument, grid property, or config
        effective_seed: Optional[int] = seed if seed is not None else grid.seed
        if effective_seed is None:
            cfg_seed = (
                self._config.get("simulation", {}).get("random_seed")
                if isinstance(self._config.get("simulation"), dict)
                else None
            )
            if cfg_seed is None and isinstance(self._config.get("simulation"), dict):
                cfg_seed = self._config.get("simulation", {}).get("seed")
            if cfg_seed is None:
                cfg_seed = self._config.get("random_seed", self._config.get("seed"))
            effective_seed = int(cfg_seed) if cfg_seed is not None else None

        self._seed: Optional[int] = effective_seed
        self._rng: np.random.Generator = np.random.default_rng(effective_seed)

        # Track steps spent in BURNING state for each burning cell
        self._burn_counters: Dict[int, int] = {}

        # Simulation clock configuration
        sim_cfg = self._config.get("simulation", {})
        start_time_str = sim_cfg.get("start_time", "2019-04-04T14:45:00")
        end_time_str = sim_cfg.get("end_time", "2019-04-06T12:05:00")
        timestep_min = float(sim_cfg.get("timestep_minutes", 10.0))

        self._start_time: datetime = datetime.fromisoformat(str(start_time_str))
        self._end_time: datetime = datetime.fromisoformat(str(end_time_str))
        self._timestep: timedelta = timedelta(minutes=timestep_min)
        self._simulation_time: datetime = self._start_time
        self._step_count: int = 0

        # Initialize WeatherProvider and active weather state
        base_dir = getattr(grid, "_project_root", None)
        self._weather_provider: WeatherProvider = create_weather_provider(
            self._config, base_dir=base_dir
        )
        self._current_weather: WeatherState = self._weather_provider.get_weather(
            self._simulation_time, self._timestep
        )
        self._sync_config_weather()

        # Load CA parameters from config (no hardcoded scientific constants)
        ca_cfg = self._config.get("fire_ca", {})
        self._base_spread_prob: float = float(ca_cfg.get("base_spread_probability", 0.2))
        self._burn_duration: int = int(ca_cfg.get("burn_duration_steps", 3))
        self._slope_weight: float = float(ca_cfg.get("slope_factor_weight", 0.05))
        self._wind_weight: float = float(ca_cfg.get("wind_factor_weight", 0.1))
        self._moisture_weight: float = float(ca_cfg.get("moisture_dampening_weight", 1.0))
        self._neighbor_mode: str = str(ca_cfg.get("neighbor_mode", "8-neighbor"))
        # Roads act as a firebreak: when enabled, fire cannot spread into a road
        # cell, so it cannot cross a road. See config fire_ca.roads_block_spread.
        self._roads_block_spread: bool = bool(ca_cfg.get("roads_block_spread", True))

        # Initialize risk scores across the grid using selected spread mode
        spread_scores = self.predict_spread()
        update_grid_risk_scores(self._grid, self._config, spread_scores=spread_scores)

    def _sync_config_weather(self) -> None:
        """Synchronize active weather parameters into self._config['wind'] for backward compatibility."""
        wind_cfg = self._config.setdefault("wind", {})
        wind_cfg["speed_ms"] = self._current_weather.wind_speed_ms
        wind_cfg["direction_deg"] = (
            self._current_weather.raw_wind_direction_deg
            if self._current_weather.raw_wind_direction_deg is not None
            else self._current_weather.wind_direction_deg
        )

    @property
    def grid(self) -> EnvironmentGrid:
        """Return the environment grid instance."""
        return self._grid

    @property
    def seed(self) -> Optional[int]:
        """Return the simulation random seed."""
        return self._seed

    @property
    def start_time(self) -> datetime:
        """Return the configured simulation start time."""
        return self._start_time

    @property
    def end_time(self) -> datetime:
        """Return the configured simulation end time."""
        return self._end_time

    @property
    def current_simulation_time(self) -> datetime:
        """Return current physical simulation datetime."""
        return self._simulation_time

    @property
    def simulation_time(self) -> datetime:
        """Alias for current_simulation_time."""
        return self._simulation_time

    @property
    def timestep(self) -> timedelta:
        """Return duration of a single simulation timestep."""
        return self._timestep

    @property
    def timestep_minutes(self) -> float:
        """Return timestep duration in minutes."""
        return self._timestep.total_seconds() / 60.0

    @property
    def step_count(self) -> int:
        """Return the number of simulation steps completed."""
        return self._step_count

    @property
    def simulation_step(self) -> int:
        """Alias for step_count."""
        return self._step_count

    @property
    def weather_provider(self) -> WeatherProvider:
        """Return active weather provider instance."""
        return self._weather_provider

    @property
    def current_weather(self) -> WeatherState:
        """Return the current active weather state."""
        return self._current_weather

    @property
    def simulation_finished(self) -> bool:
        """True if current simulation start time exceeds configured end time."""
        return self._simulation_time > self._end_time

    def get_weather_diagnostics(self) -> Dict[str, Any]:
        """Return diagnostic metadata regarding current simulation time and active weather state."""
        return {
            "step": self._step_count,
            "simulation_time": self._simulation_time.isoformat(),
            "weather_mode": self._config.get("weather", {}).get("mode", "fixed"),
            "wind_direction_convention": self._current_weather.wind_direction_convention,
            "raw_wind_direction_deg": self._current_weather.raw_wind_direction_deg,
            "effective_toward_direction_deg": self._current_weather.wind_direction_deg,
            "wind_speed_ms": self._current_weather.wind_speed_ms,
            "wind_direction_deg": self._current_weather.wind_direction_deg,
            "humidity_percent": self._current_weather.humidity_percent,
            "weather_source": self._current_weather.source,
        }

    def predict_spread_geometric(self) -> Dict[int, float]:
        """Calculate baseline spatial spread hazard score (S_spread ∈ [0, 1]) for all cells.

        Combines normalized slope, fuel amount, and downwind cone proximity:
            S_spread = w_slope * S_slope + w_fuel * S_fuel + w_downwind * S_downwind
        where w_slope + w_fuel + w_downwind = 1.0.
        """
        cfg = self._config.get("risk_scoring", {})
        downwind_buffer = int(cfg.get("downwind_buffer_cells", 3))
        max_slope_deg = float(cfg.get("max_slope_degrees", 45.0))
        downwind_dot_thresh = float(cfg.get("downwind_dot_threshold", 0.3))

        spread_weights_cfg = cfg.get("spread_component_weights", {})
        raw_w_slope = float(spread_weights_cfg.get("slope", 1.0 / 3.0))
        raw_w_fuel = float(spread_weights_cfg.get("fuel", 1.0 / 3.0))
        raw_w_downwind = float(spread_weights_cfg.get("downwind", 1.0 / 3.0))
        tot_w = raw_w_slope + raw_w_fuel + raw_w_downwind
        if tot_w > 0:
            w_slope = raw_w_slope / tot_w
            w_fuel = raw_w_fuel / tot_w
            w_downwind = raw_w_downwind / tot_w
        else:
            w_slope = w_fuel = w_downwind = 1.0 / 3.0

        cells = self._grid.cells
        burning_cells = [c for c in cells if c.fire_state == FireState.BURNING]
        wind_dx, wind_dy = _compute_wind_vector(self._config, weather=self._current_weather)

        downwind_coords = _collect_downwind_cells(
            grid=self._grid,
            burning_cells=burning_cells,
            wind_dx=wind_dx,
            wind_dy=wind_dy,
            downwind_buffer=downwind_buffer,
            downwind_dot_thresh=downwind_dot_thresh,
        )

        spread_map: Dict[int, float] = {}
        for cell in cells:
            if cell.fire_state == FireState.UNBURNED and not (
                self._roads_block_spread and cell.is_road
            ):
                s_slope = (
                    min(1.0, max(0.0, cell.slope / max_slope_deg))
                    if max_slope_deg > 0
                    else 0.0
                )
                s_fuel = min(1.0, max(0.0, cell.fuel_amount))
                s_downwind = 1.0 if (cell.x, cell.y) in downwind_coords else 0.0
                s_spread = w_slope * s_slope + w_fuel * s_fuel + w_downwind * s_downwind
                spread_map[cell.cell_id] = max(0.0, min(1.0, float(s_spread)))
            else:
                spread_map[cell.cell_id] = 0.0

        return spread_map

    def predict_spread_ensemble(
        self,
        steps: int = 5,
        runs: int = 5,
        seed: Optional[int] = None,
    ) -> Dict[int, float]:
        """Calculate dynamic spread hazard probability (S_spread ∈ [0, 1]) via Monte Carlo CA.

        Definition:
            S_spread(cell) = P(cell becomes BURNING at any point within next N steps)

        NON-MUTATING GUARANTEE:
        This prediction executes forward simulations on an isolated state copy.
        The live grid, cell fire_states, burn counters, main RNG, and simulation clock
        are left completely untouched.

        Parameters
        ----------
        steps : int, default=5
            Forecast horizon in CA time steps (N). If steps <= 0, returns 0.0 for all cells.
        runs : int, default=5
            Number of stochastic Monte Carlo trajectories (M). Must be > 0.
        seed : int, optional
            Deterministic seed for ensemble RNG. If None, derived from engine seed.

        Returns
        -------
        Dict[int, float]
            Mapping from cell_id to empirical burn probability in [0.0, 1.0].
        """
        if runs <= 0:
            raise ValueError(f"Ensemble runs must be positive integer, got {runs}")
        if steps <= 0:
            return {c.cell_id: 0.0 for c in self._grid.cells}

        # Base seed for ensemble execution
        base_seed = seed if seed is not None else (self._seed if self._seed is not None else 42)

        # Pre-retrieve forecast weather for each future horizon step without mutating live state
        forecast_weathers = [
            self._weather_provider.get_weather(
                self._simulation_time + k * self._timestep, self._timestep
            )
            for k in range(steps)
        ]

        # Track how many runs each unburned cell caught fire
        burn_occurrences: Dict[int, int] = {}

        # Cache initial state
        init_burning = {c.cell_id for c in self._grid.cells if c.fire_state == FireState.BURNING}
        init_burned = {c.cell_id for c in self._grid.cells if c.fire_state == FireState.BURNED}
        init_counters = dict(self._burn_counters)

        for run_idx in range(runs):
            run_seed = (base_seed * 10007 + run_idx + 1) & 0xFFFFFFFF
            run_rng = np.random.default_rng(run_seed)

            sim_burning = set(init_burning)
            sim_burned = set(init_burned)
            sim_counters = dict(init_counters)
            ever_ignited: Set[int] = set()

            for step_idx in range(steps):
                if not sim_burning:
                    break  # No active flames left in this run

                step_weather = forecast_weathers[step_idx]

                # 1. Age burning cells
                for cid in list(sim_burning):
                    sim_counters[cid] = sim_counters.get(cid, 0) + 1
                    if sim_counters[cid] >= self._burn_duration:
                        sim_burning.remove(cid)
                        sim_burned.add(cid)

                # 2. Gather candidate unburned neighbors
                candidate_sources: Dict[int, List[Cell]] = {}
                for bc_id in sim_burning:
                    bc = self._grid.get_cell_by_id(bc_id)
                    neighbors = self._grid.get_neighbors(bc.x, bc.y, mode=self._neighbor_mode)  # type: ignore
                    for neighbor in neighbors:
                        nid = neighbor.cell_id
                        if (
                            nid not in sim_burning
                            and nid not in sim_burned
                            and neighbor.fuel_amount > 0.0
                            and not (self._roads_block_spread and neighbor.is_road)
                        ):
                            candidate_sources.setdefault(nid, []).append(bc)

                # 3. Evaluate combined probability and roll RNG
                for nid in sorted(candidate_sources.keys()):
                    target = self._grid.get_cell_by_id(nid)
                    prob_not = 1.0
                    for src in candidate_sources[nid]:
                        p_spread = self._calculate_spread_probability(
                            src, target, weather=step_weather
                        )
                        prob_not *= (1.0 - p_spread)

                    p_comb = 1.0 - prob_not
                    if float(run_rng.random()) < p_comb:
                        sim_burning.add(nid)
                        sim_counters[nid] = 0
                        ever_ignited.add(nid)

            for cid in ever_ignited:
                burn_occurrences[cid] = burn_occurrences.get(cid, 0) + 1

        spread_map: Dict[int, float] = {}
        for cell in self._grid.cells:
            if cell.fire_state == FireState.UNBURNED:
                p_burn = burn_occurrences.get(cell.cell_id, 0) / float(runs)
                spread_map[cell.cell_id] = max(0.0, min(1.0, float(p_burn)))
            else:
                spread_map[cell.cell_id] = 0.0

        return spread_map

    def predict_spread(self, mode: Optional[str] = None, **kwargs) -> Dict[int, float]:
        """Dispatch spread prediction according to configured or requested mode.

        Parameters
        ----------
        mode : "geometric" | "ensemble", optional
            Spread prediction method. Defaults to ``risk_scoring.spread_mode`` in config.
        **kwargs :
            Additional arguments forwarded to `predict_spread_ensemble` (steps, runs, seed).
        """
        if mode is None:
            mode = str(self._config.get("risk_scoring", {}).get("spread_mode", "geometric"))

        if mode == "ensemble":
            ensemble_cfg = self._config.get("risk_scoring", {}).get("ensemble", {})
            steps = kwargs.get("steps", int(ensemble_cfg.get("steps", 5)))
            runs = kwargs.get("runs", int(ensemble_cfg.get("runs", 5)))
            seed = kwargs.get("seed", None)
            return self.predict_spread_ensemble(steps=steps, runs=runs, seed=seed)
        else:
            return self.predict_spread_geometric()

    def ignite_at(self, x: int, y: int) -> Cell:
        """Ignite cell at grid position (col *x*, row *y*).

        Transitions cell state from `UNBURNED` to `BURNING`.
        Raises `IndexError` if coordinates are out of bounds.
        """
        cell = self._grid.get_cell(x, y)
        cell.fire_state = FireState.BURNING
        self._burn_counters[cell.cell_id] = 0
        spread_scores = self.predict_spread()
        update_grid_risk_scores(self._grid, self._config, spread_scores=spread_scores)
        return cell

    def ignite_cell(self, cell_id: int) -> Cell:
        """Ignite cell with unique *cell_id*.

        Transitions cell state from `UNBURNED` to `BURNING`.
        Raises `KeyError` if cell_id is not found.
        """
        cell = self._grid.get_cell_by_id(cell_id)
        cell.fire_state = FireState.BURNING
        self._burn_counters[cell.cell_id] = 0
        spread_scores = self.predict_spread()
        update_grid_risk_scores(self._grid, self._config, spread_scores=spread_scores)
        return cell

    def ignite_at_latlon(
        self,
        lat: float,
        lon: float,
        snap_to_fuel: bool = False,
        snap_max_radius: int = 12,
    ) -> Cell:
        """Ignite the grid cell containing geographic point (*lat*, *lon*) in WGS84.

        Converts the lat/lon ignition point to grid row/col via the grid CRS
        (:meth:`EnvironmentGrid.latlon_to_rowcol`) and ignites that cell.

        Parameters
        ----------
        lat, lon : float
            Ignition point in WGS84 decimal degrees.
        snap_to_fuel : bool, default False
            If True and the target cell has no fuel (``fuel_amount == 0``, e.g. a
            nodata / non-fuel pixel), snap the ignition to the nearest
            fuel-bearing cell within *snap_max_radius*. A fire cannot start or
            spread on a non-fuel cell, so this makes ignition robust to points
            that land on water, roads, barren ground, or unclassified nodata.
        snap_max_radius : int, default 12
            Maximum search radius (in cells) used when *snap_to_fuel* is True.

        Raises
        ------
        IndexError
            If the point lies outside the grid bounds.
        """
        row, col = self._grid.latlon_to_rowcol(lat, lon)
        if snap_to_fuel and self._grid.get_cell(col, row).fuel_amount <= 0.0:
            snapped = self._grid.nearest_fuel_rowcol(row, col, max_radius=snap_max_radius)
            if snapped is not None:
                row, col = snapped
        # ignite_at takes (x=col, y=row)
        return self.ignite_at(col, row)

    def ignite_from_config(self) -> Cell:
        """Ignite the cell at the ignition point defined in config (``ignition.lat/lon``).

        Reads ``ignition.lat`` and ``ignition.lon`` from the configuration and
        ignites the corresponding grid cell. Honours the optional ignition keys:

        * ``ignition.snap_to_fuel`` (bool, default True) — snap to the nearest
          fuel-bearing cell when the ignition point lands on a non-fuel/nodata cell.
        * ``ignition.snap_max_radius`` (int, default 12) — snap search radius.

        Raises
        ------
        KeyError
            If ``ignition.lat`` or ``ignition.lon`` is missing from the config.
        IndexError
            If the ignition point lies outside the grid bounds.
        """
        ign_cfg = self._config.get("ignition", {})
        if "lat" not in ign_cfg or "lon" not in ign_cfg:
            raise KeyError(
                "Config is missing 'ignition.lat' / 'ignition.lon' for ignition point."
            )
        return self.ignite_at_latlon(
            float(ign_cfg["lat"]),
            float(ign_cfg["lon"]),
            snap_to_fuel=bool(ign_cfg.get("snap_to_fuel", True)),
            snap_max_radius=int(ign_cfg.get("snap_max_radius", 12)),
        )

    def _calculate_spread_probability(
        self, source: Cell, target: Cell, weather: Optional[WeatherState] = None
    ) -> float:
        """Calculate fire spread probability from *source* (BURNING) to *target* (UNBURNED).

        Modifiers:
          - Fuel amount: target.fuel_amount (if 0.0, zero spread)
          - Moisture: max(0, 1 - moisture_weight * target.moisture)
          - Slope: exp(slope_weight * gradient), where gradient = dz / dist
          - Wind: exp(wind_weight * speed * alignment), where alignment = dot product of
            direction vector (source -> target) with wind vector
        """
        if target.fuel_amount <= 0.0 or target.fire_state != FireState.UNBURNED:
            return 0.0

        # Roads are firebreaks: fire cannot spread into a road cell.
        if self._roads_block_spread and target.is_road:
            return 0.0

        fuel_factor = target.fuel_amount
        moisture_factor = max(0.0, 1.0 - self._moisture_weight * target.moisture)
        if moisture_factor <= 0.0:
            return 0.0

        dz = target.elevation - source.elevation
        dx_cell = target.x - source.x
        dy_cell = target.y - source.y
        cell_dist_m = math.hypot(dx_cell, dy_cell) * self._grid.cell_resolution
        gradient = (dz / cell_dist_m) if cell_dist_m > 0 else 0.0
        slope_factor = math.exp(self._slope_weight * gradient)

        if weather is not None:
            wind_speed = float(weather.wind_speed_ms)
            towards_deg = float(weather.wind_direction_deg)
        else:
            wind_cfg = self._config.get("wind", {})
            wind_speed = float(wind_cfg.get("speed_ms", self._current_weather.wind_speed_ms))
            towards_deg = float(self._current_weather.wind_direction_deg)

        towards_rad = math.radians(towards_deg)
        wind_dx = math.sin(towards_rad)
        wind_dy = -math.cos(towards_rad)

        dist_cell = math.hypot(dx_cell, dy_cell)
        if dist_cell > 0:
            ux = dx_cell / dist_cell
            uy = dy_cell / dist_cell
            alignment = ux * wind_dx + uy * wind_dy
        else:
            alignment = 0.0

        wind_factor = math.exp(self._wind_weight * wind_speed * alignment)
        p = self._base_spread_prob * fuel_factor * moisture_factor * slope_factor * wind_factor
        return max(0.0, min(1.0, p))

    def step(self) -> List[Cell]:
        """Execute one discrete simulation step.

        Process:
          1. Check if simulation is finished (end_time hard upper bound reached).
          2. Obtain active weather for the current CA interval.
          3. Age currently BURNING cells; transition cells exceeding `burn_duration_steps` from `BURNING -> BURNED`.
          4. For each active `BURNING` cell, identify `UNBURNED` neighbors.
          5. Compute combined spread probability for candidate `UNBURNED` cells using active weather.
          6. Evaluate stochastic ignition using seeded RNG (`self._rng`).
          7. Update states of newly ignited cells to `BURNING`.
          8. Recalculate risk scores across the grid using predicted spread scores.
          9. Advance physical simulation time by `timestep_minutes`.

        Returns
        -------
        List[Cell]
            List of cells that newly ignited during this step.
        """
        if self.simulation_finished:
            logger.warning(
                "Simulation finished: current simulation time %s exceeds end time %s. Step not executed.",
                self._simulation_time,
                self._end_time,
            )
            return []

        # Check if external caller explicitly updated self._config["wind"] or applied observation
        wind_cfg = self._config.get("wind", {})
        is_observation = (
            self._current_weather is not None
            and self._current_weather.source in ("observation_update", "external_override")
        )
        ext_speed = float(wind_cfg.get("speed_ms", self._current_weather.wind_speed_ms))
        prev_raw = (
            self._current_weather.raw_wind_direction_deg
            if self._current_weather.raw_wind_direction_deg is not None
            else self._current_weather.wind_direction_deg
        )
        ext_dir = float(wind_cfg.get("direction_deg", prev_raw))

        externally_overridden = is_observation or not (
            math.isclose(ext_speed, self._current_weather.wind_speed_ms, abs_tol=1e-6)
            and math.isclose(ext_dir, prev_raw, abs_tol=1e-6)
        )

        if externally_overridden:
            conv = self._config.get("weather", {}).get("wind_direction_convention", "from")
            towards_dir = to_toward_direction(ext_dir, conv)
            source = (
                self._current_weather.source
                if is_observation
                else "external_override"
            )
            self._current_weather = WeatherState(
                wind_speed_ms=ext_speed,
                wind_direction_deg=towards_dir,
                humidity_percent=self._current_weather.humidity_percent,
                source=source,
                raw_wind_direction_deg=ext_dir,
                wind_direction_convention=conv,
            )
        else:
            self._current_weather = self._weather_provider.get_weather(
                self._simulation_time, self._timestep
            )
            self._sync_config_weather()

        # Diagnostics logging (Section 24 & prompt2 Section 8)
        raw_str = (
            f"{self._current_weather.raw_wind_direction_deg:.2f}"
            if self._current_weather.raw_wind_direction_deg is not None
            else "None"
        )
        logger.debug(
            "step=%d simulation_time=%s weather_mode=%s wind_direction_convention=%s "
            "raw_wind_direction_deg=%s effective_toward_direction_deg=%.2f "
            "wind_speed_ms=%.2f humidity_percent=%.2f weather_source=%s",
            self._step_count,
            self._simulation_time.isoformat(),
            self._config.get("weather", {}).get("mode", "fixed"),
            self._current_weather.wind_direction_convention,
            raw_str,
            self._current_weather.wind_direction_deg,
            self._current_weather.wind_speed_ms,
            self._current_weather.humidity_percent,
            self._current_weather.source,
        )

        currently_burning = [c for c in self._grid.cells if c.fire_state == FireState.BURNING]
        for cell in currently_burning:
            cell_id = cell.cell_id
            self._burn_counters[cell_id] = self._burn_counters.get(cell_id, 0) + 1
            if self._burn_counters[cell_id] >= self._burn_duration:
                cell.fire_state = FireState.BURNED

        active_burning = [c for c in self._grid.cells if c.fire_state == FireState.BURNING]

        candidate_sources: Dict[int, List[Tuple[Cell, Cell]]] = {}
        for burning_cell in active_burning:
            neighbors = self._grid.get_neighbors(
                burning_cell.x, burning_cell.y, mode=self._neighbor_mode  # type: ignore
            )
            for neighbor in neighbors:
                if (
                    neighbor.fire_state == FireState.UNBURNED
                    and neighbor.fuel_amount > 0.0
                    and not (self._roads_block_spread and neighbor.is_road)
                ):
                    cid = neighbor.cell_id
                    if cid not in candidate_sources:
                        candidate_sources[cid] = []
                    candidate_sources[cid].append((burning_cell, neighbor))

        newly_ignited: List[Cell] = []
        sorted_candidate_ids = sorted(candidate_sources.keys())

        for cid in sorted_candidate_ids:
            pairs = candidate_sources[cid]
            target_cell = pairs[0][1]

            prob_not_igniting = 1.0
            for source_cell, _ in pairs:
                p_spread = self._calculate_spread_probability(source_cell, target_cell)
                prob_not_igniting *= (1.0 - p_spread)

            p_combined = 1.0 - prob_not_igniting

            roll = float(self._rng.random())
            if roll < p_combined:
                newly_ignited.append(target_cell)

        for cell in newly_ignited:
            cell.fire_state = FireState.BURNING
            self._burn_counters[cell.cell_id] = 0

        # Recalculate risk scores across the grid
        spread_scores = self.predict_spread()
        update_grid_risk_scores(self._grid, self._config, spread_scores=spread_scores)

        # Advance simulation time and step count
        self._simulation_time += self._timestep
        self._step_count += 1

        # Synchronize active weather state for the new simulation time if not overridden
        if not self.simulation_finished and not externally_overridden:
            self._current_weather = self._weather_provider.get_weather(
                self._simulation_time, self._timestep
            )
            self._sync_config_weather()

        return newly_ignited
