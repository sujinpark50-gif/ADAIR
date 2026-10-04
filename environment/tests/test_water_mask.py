"""Tests for water mask raster ingestion and physical fire-spread barrier enforcement.

Verifies specification compliance per prompt8 requirements:
1. Water raster is loaded and spatial alignment is validated.
2. water_mask == 1 produces is_water == True.
3. water_mask == 0 produces is_water == False.
4. water_mask == 255 produces is_water == False (unknown/NoData, not water).
5. Regression test: bool(255) cannot cause NoData cell to be classified as water.
6. Confirmed water cells cannot ignite directly (ignite_at, ignite_cell raise ValueError).
7. Invalid initial ignition on water cell via ignite_at_latlon is rejected and NOT snapped.
8. Cell with water_mask == 255 is not rejected as water during ignition.
9. Spread probability into confirmed water cell is strictly 0.0 across all calculations.
10. CA step never ignites or spreads into confirmed water cells.
11. water_mask == 255 does not block fire spread.
12. Both geometric and ensemble/Monte-Carlo spread prediction assign 0.0 to water cells.
13. Fuel handling remains separate from water barrier classification.
"""

from __future__ import annotations

from pathlib import Path
import sys
import pytest
import rasterio
import rasterio.warp

TEST_DIR = Path(__file__).resolve().parent
ENV_DIR = TEST_DIR.parent
if str(ENV_DIR) not in sys.path:
    sys.path.insert(0, str(ENV_DIR))

from src.cell import Cell, FireState
from src.grid import EnvironmentGrid
from src.fire_model import WildfireCAEngine


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "environment_config.yaml"


@pytest.fixture(scope="module")
def real_grid() -> EnvironmentGrid:
    """Load EnvironmentGrid backed by actual project GIS rasters."""
    return EnvironmentGrid(CONFIG_PATH, seed=42)


@pytest.fixture
def real_engine(real_grid: EnvironmentGrid) -> WildfireCAEngine:
    """Create fresh WildfireCAEngine instance with real grid."""
    # Reset fire states
    for c in real_grid.cells:
        c.fire_state = FireState.UNBURNED
        c.risk_score = 0.0
    return WildfireCAEngine(real_grid, config_path=CONFIG_PATH, seed=42)


def test_water_raster_loaded_and_aligned(real_grid: EnvironmentGrid):
    """Requirement 1: water raster is loaded and spatially aligned with dem_clipped.tif."""
    assert real_grid.water_mask is not None
    assert real_grid.water_mask.shape == (real_grid.rows, real_grid.cols)
    assert real_grid.rows == 236
    assert real_grid.cols == 308

    # Verify counts of 0, 1, 255
    water_cells = [c for c in real_grid.cells if c.is_water]
    assert len(water_cells) == 398  # Exact known count of water pixels (1) in inje_2019_water_mask_5186.tif


def test_water_mask_classification_values(real_grid: EnvironmentGrid):
    """Requirements 2, 3, 4:
    water_mask == 1 -> is_water is True
    water_mask == 0 -> is_water is False
    water_mask == 255 -> is_water is False
    """
    raw_mask = real_grid.water_mask
    assert raw_mask is not None

    # Sample cell with raw value 1 (e.g. row 68, col 47)
    cell_water = real_grid.get_cell(47, 68)
    assert raw_mask[68, 47] == 1
    assert cell_water.is_water is True

    # Sample cell with raw value 0 (e.g. row 68, col 3)
    cell_non_water = real_grid.get_cell(3, 68)
    assert raw_mask[68, 3] == 0
    assert cell_non_water.is_water is False

    # Sample cell with raw value 255 (e.g. row 0, col 0)
    cell_nodata = real_grid.get_cell(0, 0)
    assert raw_mask[0, 0] == 255
    assert cell_nodata.is_water is False

    # Full grid invariants
    for cell in real_grid.cells:
        raw_val = raw_mask[cell.y, cell.x]
        if raw_val == 1:
            assert cell.is_water is True
        elif raw_val == 0:
            assert cell.is_water is False
        elif raw_val == 255:
            assert cell.is_water is False
        else:
            assert cell.is_water is False


def test_regression_bool_255_not_classified_as_water(real_grid: EnvironmentGrid):
    """Regression test: Ensure bool(255) truthiness does not cause NoData to become water."""
    raw_mask = real_grid.water_mask
    assert raw_mask is not None

    nodata_indices = [
        (c.x, c.y) for c in real_grid.cells if raw_mask[c.y, c.x] == 255
    ]
    assert len(nodata_indices) > 0

    # In Python, bool(255) is True!
    assert bool(255) is True

    # But every cell with raw_val == 255 MUST have is_water is False
    for x, y in nodata_indices[:100]:
        cell = real_grid.get_cell(x, y)
        assert cell.is_water is False


def test_confirmed_water_cannot_ignite_direct(real_engine: WildfireCAEngine, real_grid: EnvironmentGrid):
    """Requirement 5: Confirmed water cells cannot ignite directly."""
    # Find a confirmed water cell
    water_cell = next(c for c in real_grid.cells if c.is_water)
    assert water_cell.is_water is True

    # ignite_at must raise ValueError
    with pytest.raises(ValueError, match="classified as water"):
        real_engine.ignite_at(water_cell.x, water_cell.y)

    assert water_cell.fire_state == FireState.UNBURNED

    # ignite_cell must raise ValueError
    with pytest.raises(ValueError, match="classified as water"):
        real_engine.ignite_cell(water_cell.cell_id)

    assert water_cell.fire_state == FireState.UNBURNED


def test_initial_ignition_on_water_rejected_and_not_snapped(real_engine: WildfireCAEngine, real_grid: EnvironmentGrid):
    """Requirement 6 & 10: Initial ignition on water coordinate fails and is NOT moved/snapped."""
    # Convert water cell (row 68, col 47) center to lat/lon in WGS84
    x_world, y_world = real_grid.rowcol_to_xy(68, 47)
    lons, lats = rasterio.warp.transform(real_grid.crs, "EPSG:4326", [x_world], [y_world])
    lat, lon = lats[0], lons[0]

    # ignite_at_latlon with snap_to_fuel=True must still raise ValueError, NOT snap
    with pytest.raises(ValueError, match="classified as water"):
        real_engine.ignite_at_latlon(lat, lon, snap_to_fuel=True, snap_max_radius=12)

    cell = real_grid.get_cell(47, 68)
    assert cell.fire_state == FireState.UNBURNED


def test_ignition_on_nodata_255_cell_not_rejected_as_water(real_engine: WildfireCAEngine, real_grid: EnvironmentGrid):
    """Requirement 7 & 10: Cell with water_mask == 255 should not be rejected solely as water."""
    raw_mask = real_grid.water_mask
    assert raw_mask is not None

    # Find a cell with water_mask == 255 that has fuel
    nodata_cell = next(
        c for c in real_grid.cells if raw_mask[c.y, c.x] == 255 and c.fuel_amount > 0.0
    )
    assert nodata_cell.is_water is False

    # Igniting it should succeed (not raise water ValueError)
    ignited = real_engine.ignite_at(nodata_cell.x, nodata_cell.y)
    assert ignited.fire_state == FireState.BURNING


def test_fire_cannot_spread_to_water_cell(real_engine: WildfireCAEngine, real_grid: EnvironmentGrid):
    """Requirements 6 & 9: Spread probability into water cell is 0.0 and fire never spreads to it."""
    water_cell = next(c for c in real_grid.cells if c.is_water)

    # Synthetic burning cell right next to water_cell
    burning_source = Cell(
        cell_id=999999,
        x=water_cell.x - 1,
        y=water_cell.y,
        elevation=water_cell.elevation - 10.0,  # uphill to water
        slope=10.0,
        fuel_type=3,
        fuel_amount=1.0,
        moisture=0.1,
        fire_state=FireState.BURNING,
    )

    # Even if water cell has artificially high fuel, is_water must block spread
    water_cell.fuel_amount = 1.0
    prob = real_engine._calculate_spread_probability(burning_source, water_cell)
    assert prob == 0.0


def test_step_ca_skips_water_cells(real_engine: WildfireCAEngine, real_grid: EnvironmentGrid):
    """Fire spread in step() never ignites a water cell."""
    water_cell = real_grid.get_cell(47, 68)
    assert water_cell.is_water is True
    water_cell.fuel_amount = 1.0  # test worst case: has fuel assigned

    # Find a valid neighbor to ignite
    neighbors = real_grid.get_neighbors(47, 68, mode="8-neighbor")
    valid_neighbor = next(n for n in neighbors if not n.is_water)
    valid_neighbor.fuel_amount = 1.0
    valid_neighbor.fire_state = FireState.BURNING
    real_engine._burn_counters[valid_neighbor.cell_id] = 0

    # Step simulation multiple times
    for _ in range(5):
        real_engine.step()

    # Water cell must remain UNBURNED
    assert water_cell.fire_state == FireState.UNBURNED


def test_ensemble_and_geometric_prediction_respect_water_barrier(real_engine: WildfireCAEngine, real_grid: EnvironmentGrid):
    """Requirement 9: Ensemble and geometric spread predictions give 0.0 to water cells."""
    # Ignite default origin cell
    origin = real_engine.ignite_from_config()
    assert origin.fire_state == FireState.BURNING

    # 1. Geometric spread map
    geom_map = real_engine.predict_spread_geometric()
    for cell in real_grid.cells:
        if cell.is_water:
            assert geom_map[cell.cell_id] == 0.0

    # 2. Ensemble spread map
    ens_map = real_engine.predict_spread_ensemble(steps=3, runs=3, seed=123)
    for cell in real_grid.cells:
        if cell.is_water:
            assert ens_map[cell.cell_id] == 0.0


def test_fuel_nodata_and_water_mask_remain_separate(real_grid: EnvironmentGrid):
    """Requirement 4: Fuel handling and water barrier are separate concepts."""
    # Water mask does not overwrite fuel_type or fuel_amount during grid load
    water_cells = [c for c in real_grid.cells if c.is_water]
    assert len(water_cells) > 0
    for wc in water_cells:
        assert wc.is_water is True
        # fuel_type is categorical GIS land-cover code (integer)
        assert isinstance(wc.fuel_type, int)


def test_water_mask_255_does_not_block_spread(real_engine: WildfireCAEngine, real_grid: EnvironmentGrid):
    """Requirements 7 & 8: Cells with water_mask == 255 do NOT block fire spread."""
    raw_mask = real_grid.water_mask
    assert raw_mask is not None

    # Find a cell with water_mask == 255 and fuel
    target = next(c for c in real_grid.cells if raw_mask[c.y, c.x] == 255 and c.fuel_amount > 0.0)

    # Synthetic burning neighbor
    source = Cell(
        cell_id=999998,
        x=target.x - 1,
        y=target.y,
        elevation=target.elevation,
        slope=0.0,
        fuel_type=3,
        fuel_amount=1.0,
        moisture=0.1,
        fire_state=FireState.BURNING,
    )

    # Spread probability must be strictly positive (not blocked by water)
    prob = real_engine._calculate_spread_probability(source, target)
    assert prob > 0.0

