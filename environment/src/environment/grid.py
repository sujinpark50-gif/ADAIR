"""Environment grid: raster ingestion and 2D cell-matrix management.

Reads preprocessed GIS raster layers from data/processed/ and constructs
a 2D grid of Cell objects. All spatial parameters (bounds, resolution,
CRS, fuel-type mapping, default moisture) are loaded from
config/environment_config.yaml — nothing is hardcoded.

Phase 1 scope only: no fire-spread logic, no observation updates, no
external-subsystem API wrappers.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Tuple

import numpy as np
import rasterio
import yaml

from src.environment.cell import Cell, FireState


def _load_config(config_path: str | Path) -> dict:
    """Load and return the YAML configuration dictionary."""
    config_path = Path(config_path)
    with open(config_path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _are_crss_equivalent(crs_a: Any, crs_b: Any) -> bool:
    """Check if two CRS specifications are equivalent."""
    if crs_a is None or crs_b is None:  # 둘 중 하나가 None이면 일단 통과; 다소 느슨한 정책
        return True
    if crs_a == crs_b:
        return True
    try:
        obj_a = rasterio.crs.CRS.from_user_input(crs_a)
        obj_b = rasterio.crs.CRS.from_user_input(crs_b)
        if obj_a == obj_b:
            return True
        epsg_a = obj_a.to_epsg()    # EPSG 코드(5186)로 바꿀 수 있는지 확인
        epsg_b = obj_b.to_epsg()
        if epsg_a is not None and epsg_b is not None:
            return epsg_a == epsg_b
        dict_a = obj_a.to_dict()    # EPSG 코드로 바로 비교가 안 되면 딕셔너리 형태로 비교
        dict_b = obj_b.to_dict()
        proj_keys = ["proj", "lat_0", "lon_0", "x_0", "y_0"]    # 프로젝션 비교 시 핵심 키 목록
        if any(k in dict_a and k in dict_b for k in proj_keys):
            return all(dict_a.get(k) == dict_b.get(k) for k in proj_keys if k in dict_a and k in dict_b)
    except Exception:
        pass
    return str(crs_a) == str(crs_b)     # 마지막 fallback; 문자열로 바꿔 비교


def _read_raster(path: str | Path, nodata_fill: float = 0.0) -> Tuple[np.ndarray, dict]:
    """Read a single-band GeoTIFF, replace nodata/NaN with *nodata_fill*, and return array and metadata."""
    with rasterio.open(path) as ds:
        data = ds.read(1).astype(np.float64)    # dataset 첫 번째 밴드만 읽기; DEM/연료/경사 같은 단일 값 지형 데이터는 1밴드 파일이라, Band 1만 읽으면 됨
        if ds.nodata is not None:
            data[data == ds.nodata] = nodata_fill   # nodata 값이 지정되어 있으면 그 값을 `nodata_fill`로 바꿈 (기본은 0.0).
        data[np.isnan(data)] = nodata_fill
        meta = {
            "shape": data.shape,
            "crs": ds.crs,
            "transform": ds.transform,
            "bounds": ds.bounds,
            "resolution": (abs(ds.transform.a), abs(ds.transform.e)),
        }
    return data, meta


class EnvironmentGrid:
    """2D spatial grid of :class:`Cell` objects backed by GIS rasters.

    Construction
    ------------
    >>> grid = EnvironmentGrid("config/environment_config.yaml", seed=42)

    The constructor reads all raster layers referenced in the config,
    validates raster alignment (shape, CRS, transform), extracts spatial
    metadata directly from the rasters, builds the fuel-type → fuel_amount
    lookup from config, and populates every cell.

    Public interface (Phase 1)
    --------------------------
    * ``get_cell(x, y)``            – O(1) cell lookup by column/row index.
    * ``get_cell_by_id(cell_id)``   – O(1) lookup by unique cell id.
    * ``get_neighbors(x, y, mode)`` – return list of adjacent valid cells.
    * ``rowcol_to_xy(row, col)``    – convert row/col indices to world (x, y) coordinates.
    * ``xy_to_rowcol(x, y)``        – convert world (x, y) coordinates to row/col indices.
    * ``rows``, ``cols``            – grid dimensions.
    * ``cells``                     – flat list of all cells.
    * ``seed``                      – stored seed parameter for reproducibility.
    """

    # ------------------------------------------------------------------
    # construction
    # ------------------------------------------------------------------
    def __init__(
        self,
        config_path: str | Path = "config/environment_config.yaml",
        seed: Optional[int] = None,
    ) -> None:
        self._config: dict = _load_config(config_path)
        self._project_root = Path(config_path).resolve().parent.parent
        self._seed: Optional[int] = seed

        # --- load raster arrays and metadata ---------------------------------------
        raster_cfg = self._config["raster_paths"]
        self._dem, dem_meta = _read_raster(self._project_root / raster_cfg["dem_path"], nodata_fill=0.0)
        self._slope, slope_meta = _read_raster(self._project_root / raster_cfg["slope_path"], nodata_fill=0.0)
        self._aspect, aspect_meta = _read_raster(self._project_root / raster_cfg["aspect_path"], nodata_fill=0.0)
        self._fuel_type, fuel_meta = _read_raster(self._project_root / raster_cfg["fuel_path"], nodata_fill=0.0)
        self._roads, roads_meta = _read_raster(self._project_root / raster_cfg["roads_path"], nodata_fill=0.0)

        # --- validate raster alignment (shape, CRS, transform) -------------------
        metas = {
            "dem": dem_meta,
            "slope": slope_meta,
            "aspect": aspect_meta,
            "fuel_type": fuel_meta,
            "roads": roads_meta,
        }

        # 1. Shape validation
        shapes = {name: meta["shape"] for name, meta in metas.items()}
        if len(set(shapes.values())) != 1:
            raise ValueError(
                f"Raster shape mismatch — all layers must be identical: {shapes}"
            )

        # 2. CRS validation (ensuring all non-None CRS objects are equivalent)
        crs_list = [meta["crs"] for meta in metas.values() if meta["crs"] is not None]
        if crs_list:
            first_crs = crs_list[0]
            for name, meta in metas.items():
                if meta["crs"] is not None and not _are_crss_equivalent(meta["crs"], first_crs):
                    raise ValueError(
                        f"Raster CRS mismatch in '{name}': {meta['crs']} vs {first_crs}"
                    )

        # 3. Transform validation
        transforms = [meta["transform"] for meta in metas.values() if meta["transform"] is not None]
        if not transforms:
            raise ValueError("No valid raster transforms found.")
        first_transform = transforms[0]
        for name, meta in metas.items():
            t = meta["transform"]
            if t is None:
                raise ValueError(f"Raster transform is missing in '{name}'")
            current = [t.a, t.b, t.c, t.d, t.e, t.f]
            reference = [
                first_transform.a, first_transform.b, first_transform.c,
                first_transform.d, first_transform.e, first_transform.f
            ]
            if not np.allclose(current, reference):
                raise ValueError(
                    f"Raster transform mismatch in '{name}': {t} vs {first_transform}"
                )

        self._rows, self._cols = self._dem.shape

        # --- spatial metadata extracted directly from primary raster --------------
        grid_cfg = self._config.get("grid", {})
        primary_crs = dem_meta["crs"]
        if primary_crs is not None:
            epsg = primary_crs.to_epsg() if hasattr(primary_crs, "to_epsg") else None
            if epsg is not None:
                self._crs: str = f"EPSG:{epsg}"
            elif _are_crss_equivalent(primary_crs, grid_cfg.get("crs", "EPSG:5186")):
                self._crs: str = grid_cfg.get("crs", "EPSG:5186")
            else:
                self._crs: str = str(primary_crs)
        else:
            self._crs: str = grid_cfg.get("crs", "EPSG:5186")

        self._transform: rasterio.Affine = dem_meta["transform"]
        self._bounds: Tuple[float, float, float, float] = dem_meta["bounds"]
        self._cell_resolution: float = dem_meta["resolution"][0] if dem_meta["resolution"][0] > 0 else float(grid_cfg.get("cell_resolution_m", 90.0))
        self._origin_x: float = dem_meta["transform"].c
        self._origin_y: float = dem_meta["transform"].f

        # --- fuel mapping & moisture from config (no hardcoded scientific values) ---
        fuel_cfg = self._config["fuel_mapping"]
        self._fuel_type_to_amount: Dict[int, float] = {
            int(k): float(v) for k, v in fuel_cfg["fuel_type_to_amount"].items()
        }
        self._default_fuel_amount: float = float(fuel_cfg["default_fuel_amount"])
        self._default_moisture: float = float(self._config["defaults"]["moisture"])

        # --- build cell grid -------------------------------------------------------
        self._grid: List[List[Cell]] = []
        self._id_map: Dict[int, Cell] = {}
        self._build_grid()

    # ------------------------------------------------------------------
    # grid construction (private)
    # ------------------------------------------------------------------
    def _build_grid(self) -> None:
        """Populate ``self._grid`` with Cell objects from raster data."""
        cell_id = 0
        for row in range(self._rows):
            row_cells: List[Cell] = []
            for col in range(self._cols):
                ft = int(self._fuel_type[row, col])     # 현재 픽셀의 연료 유형 코드를 가져옴
                fa = self._fuel_type_to_amount.get(ft, self._default_fuel_amount)

                cell = Cell(
                    cell_id=cell_id,
                    x=col,
                    y=row,
                    elevation=float(self._dem[row, col]),
                    slope=float(self._slope[row, col]),
                    fuel_type=ft,
                    fuel_amount=fa,
                    moisture=self._default_moisture,
                    fire_state=FireState.UNBURNED,
                    risk_score=0.0,
                )
                row_cells.append(cell)
                self._id_map[cell_id] = cell
                cell_id += 1
            self._grid.append(row_cells)

    # ------------------------------------------------------------------
    # public accessors
    # ------------------------------------------------------------------
    @property
    def rows(self) -> int:
        """Number of rows in the grid."""
        return self._rows

    @property
    def cols(self) -> int:
        """Number of columns in the grid."""
        return self._cols

    @property
    def cells(self) -> List[Cell]:
        """Flat list of every cell in row-major order."""
        return [cell for row in self._grid for cell in row]

    @property
    def crs(self) -> str:
        """Coordinate Reference System identifier."""
        return self._crs

    @property
    def transform(self) -> rasterio.Affine:
        """Affine spatial transform matrix."""
        return self._transform

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """Bounding box (left, bottom, right, top)."""
        return self._bounds

    @property
    def cell_resolution(self) -> float:
        """Cell edge length in metres."""
        return self._cell_resolution

    @property
    def seed(self) -> Optional[int]:
        """Random seed parameter stored for simulation reproducibility."""
        return self._seed

    def get_cell(self, x: int, y: int) -> Cell:
        """Return the cell at column *x*, row *y*.

        Raises ``IndexError`` if out of bounds.
        """
        if not (0 <= y < self._rows and 0 <= x < self._cols):
            raise IndexError(
                f"Cell ({x}, {y}) is out of bounds "
                f"(cols=0..{self._cols - 1}, rows=0..{self._rows - 1})"
            )
        return self._grid[y][x]

    def get_cell_by_id(self, cell_id: int) -> Cell:
        """Return the cell with the given unique *cell_id*.

        Raises ``KeyError`` if no such cell exists.
        """
        try:
            return self._id_map[cell_id]
        except KeyError:
            raise KeyError(f"No cell with cell_id={cell_id}")

    def rowcol_to_xy(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid row and column index to spatial (x, y) world coordinates.

        Returns coordinates of cell center.
        Raises ``IndexError`` if row or col is out of bounds.
        """
        if not (0 <= row < self._rows and 0 <= col < self._cols):
            raise IndexError(
                f"Row {row}, Col {col} out of bounds (rows=0..{self._rows - 1}, cols=0..{self._cols - 1})"
            )
        x, y = rasterio.transform.xy(self._transform, row, col, offset="center")
        return (float(x), float(y))

    def xy_to_rowcol(self, x: float, y: float) -> Tuple[int, int]:
        """Convert spatial (x, y) world coordinates to grid (row, col) indices.

        Raises ``IndexError`` if coordinates fall outside grid bounds.
        """
        row, col = rasterio.transform.rowcol(self._transform, x, y)
        if not (0 <= row < self._rows and 0 <= col < self._cols):
            raise IndexError(
                f"Coordinates ({x}, {y}) out of grid bounds."
            )
        return (int(row), int(col))

    def get_neighbors(
        self,
        x: int,
        y: int,
        mode: Literal["4-neighbor", "8-neighbor"] = "8-neighbor",
    ) -> List[Cell]:
        """Return adjacent cells for the cell at (*x*, *y*).

        Parameters
        ----------
        mode : ``"4-neighbor"`` | ``"8-neighbor"``
            ``"4-neighbor"`` returns only cardinal neighbours (N/S/E/W).
            ``"8-neighbor"`` (default) also includes diagonals.

        Raises
        ------
        IndexError
            If (*x*, *y*) is out of bounds.
        ValueError
            If *mode* is unknown.
        """
        if not (0 <= y < self._rows and 0 <= x < self._cols):
            raise IndexError(
                f"Cell ({x}, {y}) is out of bounds "
                f"(cols=0..{self._cols - 1}, rows=0..{self._rows - 1})"
            )

        if mode == "4-neighbor":
            offsets = [(0, -1), (0, 1), (-1, 0), (1, 0)]
        elif mode == "8-neighbor":
            offsets = [
                (-1, -1), (0, -1), (1, -1),
                (-1,  0),          (1,  0),
                (-1,  1), (0,  1), (1,  1),
            ]
        else:
            raise ValueError(f"Unknown neighbor mode: {mode!r}")

        result: List[Cell] = []
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy     # nx: neighbor x
            if 0 <= nx < self._cols and 0 <= ny < self._rows:
                result.append(self._grid[ny][nx])
        return result


# Spec compliance alias
TerrainGrid = EnvironmentGrid

