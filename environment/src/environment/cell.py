"""Cell data structures for the wildfire simulation environment grid.

Defines the FireState enum and Cell dataclass as specified in
mission/environmental-modeling.txt (Section 3.3, lines 60-69).

Categorical GIS codes (fuel_type) are explicitly distinguished from
physical fire-behavior model parameters (fuel_amount, moisture).
Neither fuel_amount nor moisture default values are hardcoded here;
they must be provided at construction time from configuration.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class FireState(Enum):
    """Three-state fire lifecycle for each grid cell.
      fire_state: UNBURNED, BURNING, BURNED
    """

    UNBURNED = "UNBURNED"
    BURNING = "BURNING"
    BURNED = "BURNED"


@dataclass
class Cell:
    """Single grid cell in the simulation environment.

    Attributes correspond exactly to the cell schema defined in
    mission/environmental-modeling.txt lines 62-69.

    Categorical vs. physical parameter distinction:
      - fuel_type (int): Categorical GIS land-cover / vegetation code
        read directly from the fuel_type raster. This is raw input data.
      - fuel_amount (float): Physical fuel load quantity derived from
        fuel_type via a configurable mapping defined in
        config/environment_config.yaml. This is a modeling assumption.
      - moisture (float): Fuel / environmental moisture content loaded
        from config/environment_config.yaml as a scenario parameter.
    """

    # --- identity & position ---
    cell_id: int
    x: int  # column index in the grid
    y: int  # row index in the grid

    # --- terrain attributes (from GIS rasters) ---
    elevation: float  # metres, from dem_clipped.tif
    slope: float      # degrees, from slope_5186.tif

    # --- fuel attributes ---
    fuel_type: int     # categorical GIS code (0, 1, 2, 3 …)
    fuel_amount: float  # physical parameter — from config mapping, NOT hardcoded
    moisture: float     # physical parameter — from config default, NOT hardcoded

    # --- simulation state ---
    fire_state: FireState = field(default=FireState.UNBURNED)
    risk_score: float = field(default=0.0)
