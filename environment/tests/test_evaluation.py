"""Unit tests for the wildfire simulation evaluation module (evaluation.py).

Verifies metric mathematical accuracy on synthetic matrices, NoData handling,
FireState mask generation, raster alignment validation, and non-mutation guarantees.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import Tuple

import numpy as np
import pytest
import rasterio
import rasterio.crs
import rasterio.transform

TEST_DIR = Path(__file__).resolve().parent
ENV_DIR = TEST_DIR.parent
if str(ENV_DIR) not in sys.path:
    sys.path.insert(0, str(ENV_DIR))

from src.cell import Cell, FireState
from src.evaluation import (
    FireEvaluationResult,
    create_simulated_burn_mask,
    evaluate_fire_footprint,
    generate_overlap_classification_mask,
    load_reference_burn_mask,
    resolve_reference_raster_path,
    save_geotiff,
)
from src.fire_model import WildfireCAEngine
from src.grid import EnvironmentGrid

TEST_DIR = Path(__file__).resolve().parent
ENV_DIR = TEST_DIR.parent
CONFIG_PATH = ENV_DIR / "config" / "environment_config.yaml"
REF_RASTER_PATH = ENV_DIR.parent / "data" / "inje2019" / "burned_area" / "inje_2019_burn_reference_binary.tif"


# ---------------------------------------------------------------------------
# 1. Perfect overlap
# ---------------------------------------------------------------------------
def test_perfect_overlap():
    sim = np.array([[1, 0], [0, 1]], dtype=int)
    ref = np.array([[1, 0], [0, 1]], dtype=int)

    res = evaluate_fire_footprint(sim, ref, cell_resolution_m=90.0)

    assert res.true_positive_cells == 2
    assert res.false_positive_cells == 0
    assert res.false_negative_cells == 0
    assert res.true_negative_cells == 2
    assert math.isclose(res.iou, 1.0)
    assert math.isclose(res.dice, 1.0)
    assert math.isclose(res.precision, 1.0)
    assert math.isclose(res.recall, 1.0)
    assert math.isclose(res.area_error_ha, 0.0)
    assert math.isclose(res.area_error_percent, 0.0)


# ---------------------------------------------------------------------------
# 2. No overlap
# ---------------------------------------------------------------------------
def test_no_overlap():
    sim = np.array([[1, 0], [0, 0]], dtype=int)
    ref = np.array([[0, 0], [0, 1]], dtype=int)

    res = evaluate_fire_footprint(sim, ref, cell_resolution_m=90.0)

    assert res.true_positive_cells == 0
    assert res.false_positive_cells == 1
    assert res.false_negative_cells == 1
    assert res.true_negative_cells == 2
    assert math.isclose(res.iou, 0.0)
    assert math.isclose(res.dice, 0.0)
    assert math.isclose(res.precision, 0.0)
    assert math.isclose(res.recall, 0.0)
    assert math.isclose(res.area_error_ha, 0.0)  # 1 cell - 1 cell = 0 ha difference
    assert math.isclose(res.area_error_percent, 0.0)


# ---------------------------------------------------------------------------
# 3. Partial overlap
# ---------------------------------------------------------------------------
def test_partial_overlap():
    # 3x3 grid
    # sim burned: (0,0), (0,1), (1,0) -> 3 cells
    # ref burned: (0,1), (0,2), (1,1) -> 3 cells
    # TP: (0,1) -> 1 cell
    # FP: (0,0), (1,0) -> 2 cells
    # FN: (0,2), (1,1) -> 2 cells
    # TN: (1,2), (2,0), (2,1), (2,2) -> 4 cells
    sim = np.array([
        [1, 1, 0],
        [1, 0, 0],
        [0, 0, 0],
    ], dtype=int)

    ref = np.array([
        [0, 1, 1],
        [0, 1, 0],
        [0, 0, 0],
    ], dtype=int)

    res = evaluate_fire_footprint(sim, ref, cell_resolution_m=90.0)

    assert res.true_positive_cells == 1
    assert res.false_positive_cells == 2
    assert res.false_negative_cells == 2
    assert res.true_negative_cells == 4

    # IoU = 1 / (1 + 2 + 2) = 1/5 = 0.2
    assert math.isclose(res.iou, 0.2)
    # Dice = 2*1 / (2*1 + 2 + 2) = 2/6 = 1/3
    assert math.isclose(res.dice, 1.0 / 3.0)
    # Precision = 1 / (1 + 2) = 1/3
    assert math.isclose(res.precision, 1.0 / 3.0)
    # Recall = 1 / (1 + 2) = 1/3
    assert math.isclose(res.recall, 1.0 / 3.0)


# ---------------------------------------------------------------------------
# 4. All unburned edge case
# ---------------------------------------------------------------------------
def test_all_unburned_edge_case():
    sim = np.zeros((4, 4), dtype=int)
    ref = np.zeros((4, 4), dtype=int)

    res = evaluate_fire_footprint(sim, ref, cell_resolution_m=90.0)

    assert res.true_positive_cells == 0
    assert res.false_positive_cells == 0
    assert res.false_negative_cells == 0
    assert res.true_negative_cells == 16
    assert math.isclose(res.iou, 1.0)
    assert math.isclose(res.dice, 1.0)
    assert math.isclose(res.precision, 1.0)
    assert math.isclose(res.recall, 1.0)
    assert math.isclose(res.simulated_area_ha, 0.0)
    assert math.isclose(res.reference_area_ha, 0.0)
    assert math.isclose(res.area_error_ha, 0.0)
    assert math.isclose(res.area_error_percent, 0.0)


# ---------------------------------------------------------------------------
# 5. All burned case
# ---------------------------------------------------------------------------
def test_all_burned_case():
    sim = np.ones((3, 3), dtype=int)
    ref = np.ones((3, 3), dtype=int)

    res = evaluate_fire_footprint(sim, ref, cell_resolution_m=90.0)

    assert res.true_positive_cells == 9
    assert res.false_positive_cells == 0
    assert res.false_negative_cells == 0
    assert res.true_negative_cells == 0
    assert math.isclose(res.iou, 1.0)
    assert math.isclose(res.dice, 1.0)


# ---------------------------------------------------------------------------
# 6. NoData / valid-mask exclusion
# ---------------------------------------------------------------------------
def test_valid_mask_exclusion():
    sim = np.array([
        [1, 1],
        [0, 1],
    ], dtype=int)

    ref = np.array([
        [1, 0],
        [0, 1],
    ], dtype=int)

    # Invalidate cell (0, 1) where FP would have occurred
    valid_mask = np.array([
        [True, False],
        [True, True],
    ], dtype=bool)

    res = evaluate_fire_footprint(sim, ref, valid_mask=valid_mask, cell_resolution_m=90.0)

    # Valid cells are (0,0) [TP], (1,0) [TN], (1,1) [TP]
    assert res.true_positive_cells == 2
    assert res.false_positive_cells == 0
    assert res.false_negative_cells == 0
    assert res.true_negative_cells == 1
    assert math.isclose(res.iou, 1.0)


# ---------------------------------------------------------------------------
# 7, 8 & 9. BURNING and BURNED count as affected; UNBURNED does not
# ---------------------------------------------------------------------------
def test_fire_state_mask_generation():
    cells = [
        Cell(cell_id=0, x=0, y=0, elevation=0.0, slope=0.0, fuel_type=1, fuel_amount=1.0, moisture=0.25, fire_state=FireState.UNBURNED),
        Cell(cell_id=1, x=1, y=0, elevation=0.0, slope=0.0, fuel_type=1, fuel_amount=1.0, moisture=0.25, fire_state=FireState.BURNING),
        Cell(cell_id=2, x=0, y=1, elevation=0.0, slope=0.0, fuel_type=1, fuel_amount=1.0, moisture=0.25, fire_state=FireState.BURNED),
        Cell(cell_id=3, x=1, y=1, elevation=0.0, slope=0.0, fuel_type=1, fuel_amount=1.0, moisture=0.25, fire_state=FireState.UNBURNED),
    ]

    mask = create_simulated_burn_mask(cells, shape=(2, 2))

    assert mask.shape == (2, 2)
    assert not mask[0, 0]  # UNBURNED -> 0
    assert mask[0, 1]      # BURNING -> 1
    assert mask[1, 0]      # BURNED -> 1
    assert not mask[1, 1]  # UNBURNED -> 0

    # Ensure cells themselves were not mutated
    assert cells[0].fire_state == FireState.UNBURNED
    assert cells[1].fire_state == FireState.BURNING
    assert cells[2].fire_state == FireState.BURNED
    assert cells[3].fire_state == FireState.UNBURNED


# ---------------------------------------------------------------------------
# 10. Area calculation using 90 m x 90 m cells (0.81 ha/cell)
# ---------------------------------------------------------------------------
def test_area_calculation_90m():
    sim = np.zeros((10, 10), dtype=int)
    sim[0:2, 0:5] = 1  # 10 cells

    ref = np.zeros((10, 10), dtype=int)
    ref[0:4, 0:5] = 1  # 20 cells

    # 1 cell = 90m * 90m = 8100 m2 = 0.81 ha
    res = evaluate_fire_footprint(sim, ref, cell_resolution_m=90.0)

    assert res.simulated_burned_cells == 10
    assert res.reference_burned_cells == 20
    assert math.isclose(res.simulated_area_ha, 8.10)
    assert math.isclose(res.reference_area_ha, 16.20)


# ---------------------------------------------------------------------------
# 11. Signed area error
# ---------------------------------------------------------------------------
def test_signed_area_error():
    # Case A: Overprediction (sim=15, ref=10)
    sim_a = np.zeros((5, 5), dtype=int)
    sim_a[:3, :5] = 1  # 15 cells = 12.15 ha
    ref_a = np.zeros((5, 5), dtype=int)
    ref_a[:2, :5] = 1  # 10 cells = 8.10 ha

    res_a = evaluate_fire_footprint(sim_a, ref_a, cell_area_ha=0.81)
    assert math.isclose(res_a.area_error_ha, +4.05)
    assert math.isclose(res_a.area_error_percent, +50.0)

    # Case B: Underprediction (sim=5, ref=10)
    sim_b = np.zeros((5, 5), dtype=int)
    sim_b[:1, :5] = 1  # 5 cells = 4.05 ha
    ref_b = np.zeros((5, 5), dtype=int)
    ref_b[:2, :5] = 1  # 10 cells = 8.10 ha

    res_b = evaluate_fire_footprint(sim_b, ref_b, cell_area_ha=0.81)
    assert math.isclose(res_b.area_error_ha, -4.05)
    assert math.isclose(res_b.area_error_percent, -50.0)


# ---------------------------------------------------------------------------
# 12. Raster / grid dimension mismatch raises ValueError
# ---------------------------------------------------------------------------
def test_dimension_mismatch_raises_error():
    sim = np.zeros((10, 10), dtype=int)
    ref = np.zeros((10, 12), dtype=int)

    with pytest.raises(ValueError, match="Shape mismatch"):
        evaluate_fire_footprint(sim, ref)


# ---------------------------------------------------------------------------
# 13 & 14. Reference loader checks CRS, transform, and dimension against grid
# ---------------------------------------------------------------------------
def test_reference_loader_validation_against_grid(tmp_path: Path):
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)

    # 1. Load actual reference raster against grid -> must pass
    if REF_RASTER_PATH.is_file():
        ref_mask, valid_mask, meta = load_reference_burn_mask(REF_RASTER_PATH, expected_grid=grid)
        assert ref_mask.shape == (grid.rows, grid.cols)
        assert np.sum(ref_mask) == 425  # Satellite reference known burned cell count
        assert meta["shape"] == (grid.rows, grid.cols)

    # 2. Synthetic raster with wrong shape -> ValueError
    bad_shape_tif = tmp_path / "bad_shape.tif"
    wrong_data = np.zeros((100, 100), dtype=np.uint8)
    save_geotiff(bad_shape_tif, wrong_data, grid.transform, grid.crs)
    with pytest.raises(ValueError, match="Reference raster dimension mismatch"):
        load_reference_burn_mask(bad_shape_tif, expected_grid=grid)

    # 3. Synthetic raster with wrong CRS -> ValueError
    bad_crs_tif = tmp_path / "bad_crs.tif"
    right_shape_data = np.zeros((grid.rows, grid.cols), dtype=np.uint8)
    save_geotiff(bad_crs_tif, right_shape_data, grid.transform, "EPSG:4326")
    with pytest.raises(ValueError, match="Reference raster CRS mismatch"):
        load_reference_burn_mask(bad_crs_tif, expected_grid=grid)

    # 4. Synthetic raster with wrong transform -> ValueError
    bad_transform_tif = tmp_path / "bad_transform.tif"
    wrong_transform = rasterio.transform.Affine(90.0, 0.0, 100000.0, 0.0, -90.0, 200000.0)
    save_geotiff(bad_transform_tif, right_shape_data, wrong_transform, grid.crs)
    with pytest.raises(ValueError, match="Reference raster transform mismatch"):
        load_reference_burn_mask(bad_transform_tif, expected_grid=grid)


# ---------------------------------------------------------------------------
# 15. Evaluation does not mutate simulation / grid state
# ---------------------------------------------------------------------------
def test_evaluation_non_mutation():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)
    engine = WildfireCAEngine(grid, config_path=CONFIG_PATH, seed=42)
    engine.ignite_from_config()
    engine.step()

    states_before = [c.fire_state for c in grid.cells]
    time_before = engine.current_simulation_time

    # Generate mask & evaluate
    mask = create_simulated_burn_mask(grid)
    dummy_ref = np.zeros((grid.rows, grid.cols), dtype=int)
    res = evaluate_fire_footprint(mask, dummy_ref)

    states_after = [c.fire_state for c in grid.cells]
    assert states_before == states_after
    assert engine.current_simulation_time == time_before


# ---------------------------------------------------------------------------
# Overlap classification raster helper
# ---------------------------------------------------------------------------
def test_overlap_classification_mask():
    sim = np.array([
        [1, 1],
        [0, 0],
    ], dtype=int)

    ref = np.array([
        [1, 0],
        [1, 0],
    ], dtype=int)

    overlap = generate_overlap_classification_mask(sim, ref)

    # (0,0): TP -> 1
    # (0,1): FP -> 2
    # (1,0): FN -> 3
    # (1,1): TN -> 0
    assert overlap[0, 0] == 1
    assert overlap[0, 1] == 2
    assert overlap[1, 0] == 3
    assert overlap[1, 1] == 0
