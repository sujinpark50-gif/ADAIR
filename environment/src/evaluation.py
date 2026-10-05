"""Wildfire simulation evaluation module for footprint comparison against reference rasters.

Provides reusable spatial overlap metrics (IoU/Jaccard, Dice/F1, Precision, Recall),
confusion matrix cell counts (TP, FP, FN, TN), signed area error calculations,
and reference raster validation without modifying simulation or grid state.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union, TYPE_CHECKING

import numpy as np
import rasterio
import rasterio.crs

from src.cell import Cell, FireState
from src.grid import _are_crss_equivalent

if TYPE_CHECKING:
    from src.grid import EnvironmentGrid

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class FireEvaluationResult:
    """Structured container for spatial and magnitude fire footprint evaluation metrics.

    Attributes:
        iou: Intersection over Union (Jaccard Index) in [0.0, 1.0].
        dice: Sørensen-Dice spatial coefficient (F1 score) in [0.0, 1.0].
        precision: Spatial precision TP / (TP + FP) in [0.0, 1.0].
        recall: Spatial recall TP / (TP + FN) in [0.0, 1.0].
        true_positive_cells: Number of cells burned in both simulation and reference.
        false_positive_cells: Number of cells burned in simulation but unburned in reference.
        false_negative_cells: Number of cells unburned in simulation but burned in reference.
        true_negative_cells: Number of cells unburned in both simulation and reference.
        simulated_burned_cells: Total simulated affected cells (BURNING + BURNED).
        reference_burned_cells: Total reference burned cells in valid mask.
        simulated_area_ha: Simulated burned area in hectares.
        reference_area_ha: Reference burned area in hectares.
        area_error_ha: Signed difference (simulated_area_ha - reference_area_ha).
            Positive indicates overprediction; negative indicates underprediction.
        area_error_percent: Signed percentage error relative to reference area.
        absolute_area_error_ha: Magnitude of area error in hectares.
        absolute_area_error_percent: Magnitude of area error in percent.
    """

    iou: float
    dice: float
    precision: float
    recall: float

    true_positive_cells: int
    false_positive_cells: int
    false_negative_cells: int
    true_negative_cells: int

    simulated_burned_cells: int
    reference_burned_cells: int

    simulated_area_ha: float
    reference_area_ha: float

    area_error_ha: float
    area_error_percent: float

    absolute_area_error_ha: float = 0.0
    absolute_area_error_percent: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        """Convert result fields to a standard dictionary."""
        return asdict(self)


def create_simulated_burn_mask(
    grid_or_cells: Union[EnvironmentGrid, List[List[Cell]], List[Cell]],
    shape: Optional[Tuple[int, int]] = None,
) -> np.ndarray:
    """Convert grid cell fire states into a 2D binary burn/affected mask.

    Both FireState.BURNING and FireState.BURNED count as affected/burned footprint (1).
    FireState.UNBURNED counts as unaffected (0).

    Does NOT mutate Cell.fire_state.

    Parameters:
        grid_or_cells: EnvironmentGrid instance, 2D list of Cells, or flat list of Cells.
        shape: Optional (rows, cols) tuple required if grid_or_cells is a flat list of Cells.

    Returns:
        np.ndarray: 2D boolean array where True indicates affected (BURNING or BURNED).
    """
    # 1. EnvironmentGrid instance
    if hasattr(grid_or_cells, "rows") and hasattr(grid_or_cells, "cols") and hasattr(grid_or_cells, "get_cell"):
        rows, cols = grid_or_cells.rows, grid_or_cells.cols
        mask = np.zeros((rows, cols), dtype=bool)
        for r in range(rows):
            for c in range(cols):
                cell = grid_or_cells.get_cell(c, r)
                if cell.fire_state in (FireState.BURNING, FireState.BURNED):
                    mask[r, c] = True
        return mask

    # 2. 2D List of Cells
    if isinstance(grid_or_cells, list) and grid_or_cells and isinstance(grid_or_cells[0], list):
        rows = len(grid_or_cells)
        cols = len(grid_or_cells[0])
        mask = np.zeros((rows, cols), dtype=bool)
        for r in range(rows):
            for c in range(cols):
                cell = grid_or_cells[r][c]
                if cell.fire_state in (FireState.BURNING, FireState.BURNED):
                    mask[r, c] = True
        return mask

    # 3. Flat List of Cells
    if isinstance(grid_or_cells, list) and grid_or_cells and isinstance(grid_or_cells[0], Cell):
        if shape is None:
            max_y = max(c.y for c in grid_or_cells)
            max_x = max(c.x for c in grid_or_cells)
            shape = (max_y + 1, max_x + 1)
        rows, cols = shape
        mask = np.zeros((rows, cols), dtype=bool)
        for cell in grid_or_cells:
            if cell.fire_state in (FireState.BURNING, FireState.BURNED):
                mask[cell.y, cell.x] = True
        return mask

    raise TypeError(
        f"Unsupported input type for create_simulated_burn_mask: {type(grid_or_cells)}"
    )


def resolve_reference_raster_path(
    raster_path: Optional[Union[str, Path]] = None,
    base_dir: Optional[Union[str, Path]] = None,
) -> Path:
    """Resolve the location of the 2019 Inje burn reference raster.

    Parameters:
        raster_path: Optional explicit path.
        base_dir: Optional base directory to anchor relative search.

    Returns:
        Path: Absolute resolved path to the reference raster GeoTIFF.

    Raises:
        FileNotFoundError: If the reference raster cannot be located.
    """
    candidate_paths: List[Path] = []

    if raster_path is not None:
        p = Path(raster_path)
        if p.is_file():
            return p.resolve()
        candidate_paths.append(p)
        if base_dir is not None:
            candidate_paths.append(Path(base_dir) / p)

    # Standard known repository locations
    known_rel_paths = [
        Path("data/inje2019/burned_area/inje_2019_burn_reference_binary.tif"),
        Path("../data/inje2019/burned_area/inje_2019_burn_reference_binary.tif"),
        Path("data/processed/burn_reference.tif"),
    ]

    current_file = Path(__file__).resolve()
    env_root = current_file.parent.parent  # environment/
    project_root = env_root.parent         # repo root

    search_roots = [project_root, env_root]
    if base_dir is not None:
        search_roots.insert(0, Path(base_dir).resolve())

    for root in search_roots:
        for rel in known_rel_paths:
            cand = (root / rel).resolve()
            if cand.is_file():
                return cand

    raise FileNotFoundError(
        f"2019 burn reference raster not found. Searched candidate paths: {candidate_paths} "
        f"under roots: {search_roots}"
    )


def load_reference_burn_mask(
    raster_path: Union[str, Path],
    expected_grid: Optional[EnvironmentGrid] = None,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """Load reference burn GeoTIFF and validate spatial alignment with the simulation grid.

    Parameters:
        raster_path: Path to the reference GeoTIFF file.
        expected_grid: Optional EnvironmentGrid instance to validate against (shape, CRS, transform).

    Returns:
        Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
            - reference_mask: 2D boolean array (True = burned, False = unburned).
            - valid_mask: 2D boolean array (True = valid data, False = nodata/NaN).
            - meta: Dictionary of raster metadata (shape, crs, transform, bounds, nodata).

    Raises:
        ValueError: If dimensions, CRS, or transform do not match expected_grid.
    """
    resolved_path = Path(raster_path).resolve()
    if not resolved_path.is_file():
        resolved_path = resolve_reference_raster_path(raster_path)

    with rasterio.open(resolved_path) as ds:
        raw_data = ds.read(1)
        nodata_val = ds.nodata
        meta = {
            "path": str(resolved_path),
            "shape": raw_data.shape,
            "crs": ds.crs,
            "transform": ds.transform,
            "bounds": ds.bounds,
            "resolution": (abs(ds.transform.a), abs(ds.transform.e)),
            "nodata": nodata_val,
        }

    # Determine valid mask
    valid_mask = ~np.isnan(raw_data)
    if nodata_val is not None:
        valid_mask &= (raw_data != nodata_val)

    # Reference mask: 1 = burned, 0 = unburned
    reference_mask = (raw_data == 1) & valid_mask

    # Spatial alignment validation against expected_grid
    if expected_grid is not None:
        # 1. Dimension validation
        expected_shape = (expected_grid.rows, expected_grid.cols)
        if raw_data.shape != expected_shape:
            raise ValueError(
                f"Reference raster dimension mismatch: {raw_data.shape} vs expected grid {expected_shape}"
            )

        # 2. CRS validation
        if not _are_crss_equivalent(meta["crs"], expected_grid.crs):
            raise ValueError(
                f"Reference raster CRS mismatch: {meta['crs']} vs expected grid {expected_grid.crs}"
            )

        # 3. Transform validation
        t_ref = meta["transform"]
        t_grid = expected_grid.transform
        ref_vec = [t_ref.a, t_ref.b, t_ref.c, t_ref.d, t_ref.e, t_ref.f]
        grid_vec = [t_grid.a, t_grid.b, t_grid.c, t_grid.d, t_grid.e, t_grid.f]
        if not np.allclose(ref_vec, grid_vec, atol=1e-5):
            raise ValueError(
                f"Reference raster transform mismatch: {t_ref} vs expected grid {t_grid}"
            )

        # 4. Resolution validation
        res_x, res_y = meta["resolution"]
        if not math_isclose(res_x, expected_grid.cell_resolution, abs_tol=1e-3):
            raise ValueError(
                f"Reference raster resolution mismatch: {res_x}m vs expected grid {expected_grid.cell_resolution}m"
            )

    return reference_mask, valid_mask, meta


def math_isclose(a: float, b: float, abs_tol: float = 1e-6) -> bool:
    """Helper for absolute float closeness comparison."""
    return abs(a - b) <= abs_tol


def evaluate_fire_footprint(
    simulated_mask: np.ndarray,
    reference_mask: np.ndarray,
    valid_mask: Optional[np.ndarray] = None,
    cell_area_ha: Optional[float] = None,
    cell_resolution_m: Optional[float] = 90.0,
) -> FireEvaluationResult:
    """Calculate quantitative spatial overlap and area error metrics between simulated and reference masks.

    Parameters:
        simulated_mask: 2D array or boolean mask of simulated affected cells (1/True = affected).
        reference_mask: 2D array or boolean mask of reference burned cells (1/True = burned).
        valid_mask: Optional 2D boolean mask of valid cells (excluding NoData).
        cell_area_ha: Area per cell in hectares. If None, computed from cell_resolution_m.
        cell_resolution_m: Cell edge length in meters (default 90.0 m -> 0.81 ha).

    Returns:
        FireEvaluationResult: Comprehensive evaluation metrics dataclass.

    Raises:
        ValueError: If mask shapes do not match.
    """
    sim_arr = np.asarray(simulated_mask)
    ref_arr = np.asarray(reference_mask)

    if sim_arr.shape != ref_arr.shape:
        raise ValueError(
            f"Shape mismatch: simulated_mask {sim_arr.shape} vs reference_mask {ref_arr.shape}"
        )

    if valid_mask is not None:
        val_arr = np.asarray(valid_mask, dtype=bool)
        if val_arr.shape != sim_arr.shape:
            raise ValueError(
                f"Shape mismatch: valid_mask {val_arr.shape} vs simulated_mask {sim_arr.shape}"
            )
    else:
        val_arr = np.ones_like(sim_arr, dtype=bool)

    # Convert to boolean affected indicators constrained to valid cells
    sim_b = (sim_arr > 0) & val_arr
    ref_b = (ref_arr > 0) & val_arr

    # Confusion counts
    tp = int(np.sum(sim_b & ref_b))
    fp = int(np.sum(sim_b & ~ref_b))
    fn = int(np.sum(~sim_b & ref_b))
    tn = int(np.sum(~sim_b & ~ref_b & val_arr))

    # Spatial overlap metrics
    # 1. IoU (Jaccard Index)
    union = tp + fp + fn
    if union == 0:
        # Edge case: both simulated and reference have 0 burned cells (perfect agreement on unburned)
        iou = 1.0
    else:
        iou = float(tp / union)

    # 2. Sørensen-Dice / F1
    dice_denom = 2 * tp + fp + fn
    if dice_denom == 0:
        dice = 1.0
    else:
        dice = float((2.0 * tp) / dice_denom)

    # 3. Precision: TP / (TP + FP)
    sim_total = tp + fp
    if sim_total == 0:
        precision = 1.0 if (tp + fn == 0) else 0.0
    else:
        precision = float(tp / sim_total)

    # 4. Recall: TP / (TP + FN)
    ref_total = tp + fn
    if ref_total == 0:
        recall = 1.0 if (tp + fp == 0) else 0.0
    else:
        recall = float(tp / ref_total)

    # Area calculations
    if cell_area_ha is None:
        res = float(cell_resolution_m if cell_resolution_m is not None else 90.0)
        cell_area_ha = (res * res) / 10000.0  # 1 ha = 10,000 m2

    sim_burned_cells = sim_total
    ref_burned_cells = ref_total

    sim_area_ha = sim_burned_cells * cell_area_ha
    ref_area_ha = ref_burned_cells * cell_area_ha

    area_err_ha = sim_area_ha - ref_area_ha
    if ref_area_ha > 0.0:
        area_err_pct = (area_err_ha / ref_area_ha) * 100.0
    else:
        area_err_pct = 0.0 if sim_area_ha == 0.0 else float("inf")

    abs_area_err_ha = abs(area_err_ha)
    abs_area_err_pct = abs(area_err_pct)

    return FireEvaluationResult(
        iou=iou,
        dice=dice,
        precision=precision,
        recall=recall,
        true_positive_cells=tp,
        false_positive_cells=fp,
        false_negative_cells=fn,
        true_negative_cells=tn,
        simulated_burned_cells=sim_burned_cells,
        reference_burned_cells=ref_burned_cells,
        simulated_area_ha=sim_area_ha,
        reference_area_ha=ref_area_ha,
        area_error_ha=area_err_ha,
        area_error_percent=area_err_pct,
        absolute_area_error_ha=abs_area_err_ha,
        absolute_area_error_percent=abs_area_err_pct,
    )


def generate_overlap_classification_mask(
    simulated_mask: np.ndarray,
    reference_mask: np.ndarray,
    valid_mask: Optional[np.ndarray] = None,
    nodata_val: int = 255,
) -> np.ndarray:
    """Generate a diagnostic raster mask categorizing spatial confusion classes.

    Values:
        0 = True Negative (neither burned)
        1 = True Positive (both burned / correctly simulated footprint)
        2 = False Positive (simulation only / overprediction)
        3 = False Negative (reference only / missed footprint)
        nodata_val (255) = Invalid / NoData pixel

    Parameters:
        simulated_mask: 2D binary simulated mask.
        reference_mask: 2D binary reference mask.
        valid_mask: Optional 2D boolean valid mask.
        nodata_val: Value assigned to invalid / nodata cells (default 255).

    Returns:
        np.ndarray: 2D uint8 classified confusion array.
    """
    sim_arr = np.asarray(simulated_mask)
    ref_arr = np.asarray(reference_mask)

    if sim_arr.shape != ref_arr.shape:
        raise ValueError(
            f"Shape mismatch: simulated {sim_arr.shape} vs reference {ref_arr.shape}"
        )

    if valid_mask is not None:
        val_arr = np.asarray(valid_mask, dtype=bool)
    else:
        val_arr = np.ones_like(sim_arr, dtype=bool)

    sim_b = (sim_arr > 0)
    ref_b = (ref_arr > 0)

    overlap = np.full(sim_arr.shape, nodata_val, dtype=np.uint8)

    # 0 = TN
    overlap[val_arr & ~sim_b & ~ref_b] = 0
    # 1 = TP
    overlap[val_arr & sim_b & ref_b] = 1
    # 2 = FP
    overlap[val_arr & sim_b & ~ref_b] = 2
    # 3 = FN
    overlap[val_arr & ~sim_b & ref_b] = 3

    return overlap


def save_geotiff(
    output_path: Union[str, Path],
    data: np.ndarray,
    transform: rasterio.Affine,
    crs: Any,
    nodata: Optional[Union[int, float]] = None,
) -> Path:
    """Save a 2D numpy array as a single-band GeoTIFF with specified spatial metadata.

    Parameters:
        output_path: Destination file path.
        data: 2D numpy array.
        transform: Affine transform.
        crs: CRS identifier or object.
        nodata: Optional nodata value.

    Returns:
        Path: Resolved absolute path to saved GeoTIFF.
    """
    out_p = Path(output_path).resolve()
    out_p.parent.mkdir(parents=True, exist_ok=True)

    dtype = data.dtype
    if dtype == bool:
        data = data.astype(np.uint8)
        dtype = np.uint8

    crs_obj = rasterio.crs.CRS.from_user_input(crs) if not isinstance(crs, rasterio.crs.CRS) else crs

    with rasterio.open(
        out_p,
        "w",
        driver="GTiff",
        height=data.shape[0],
        width=data.shape[1],
        count=1,
        dtype=dtype,
        crs=crs_obj,
        transform=transform,
        nodata=nodata,
    ) as ds:
        ds.write(data, 1)

    return out_p
