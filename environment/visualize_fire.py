"""Dynamic 2D Wildfire Simulation Integrated GIS Visualization Tool.

Standalone local tool for animating wildfire spread, terrain relief, vegetation,
road networks, 119 safety bases, and risk evolution on a 2D spatial map.
Exclusively for local simulation visualization, analysis, and testing.

AUTHORITATIVE REFERENCES & EXISTING ARCHITECTURE:
- `mission/environmental-modeling.txt`
- `specs/environment_acceptance_criteria.md`
- `AGENTS.md` & `CLAUDE.md`

Reuses existing environment modules without modification:
- EnvironmentGrid from src.environment.grid
- WildfireCAEngine from src.environment.fire_model
- EnvironmentModelAPI from src.environment.api
- Cell and FireState from src.environment.cell
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Optional

import matplotlib.animation as animation
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import rasterio
import rasterio.warp

# Enable font fallback for Windows
plt.rcParams["font.sans-serif"] = ["Malgun Gothic", "NanumGothic", "DejaVu Sans", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

from src.environment.api import EnvironmentModelAPI
from src.environment.cell import FireState
from src.environment.fire_model import WildfireCAEngine
from src.environment.grid import EnvironmentGrid


def create_visualization(
    config_path: str = "config/environment_config.yaml",
    seed: int = 42,
    ignition_x: Optional[int] = None,
    ignition_y: Optional[int] = None,
    steps: int = 30,
    interval: int = 200,
    save_path: Optional[str] = None,
    fps: int = 5,
    show_perimeter: bool = True,
) -> None:
    """Run and animate wildfire cellular automaton simulation with integrated GIS layers.

    Parameters
    ----------
    config_path : str
        Path to YAML configuration file.
    seed : int
        Simulation seed for bitwise reproducible execution.
    ignition_x : int, optional
        Initial ignition column index (default: grid center).
    ignition_y : int, optional
        Initial ignition row index (default: grid center).
    steps : int
        Total simulation steps to advance.
    interval : int
        Animation frame delay in milliseconds.
    save_path : str, optional
        File path to save output animation (.gif or .mp4).
    fps : int
        Frame rate for saved animation output.
    show_perimeter : bool
        Whether to display active fire perimeter boundary overlay markers.
    """
    # 1. Instantiate grid, cellular automaton engine, and API wrapper
    grid = EnvironmentGrid(config_path=config_path, seed=seed)
    ca_engine = WildfireCAEngine(grid=grid, config_path=config_path, seed=seed)
    api = EnvironmentModelAPI(grid=grid, ca_engine=ca_engine)

    rows, cols = grid.rows, grid.cols

    # Default ignition point at grid center if omitted
    if ignition_x is None:
        ignition_x = cols // 2
    if ignition_y is None:
        ignition_y = rows // 2

    # Ignite specified cell
    ca_engine.ignite_at(ignition_x, ignition_y)

    # Initialize figure and axes
    fig, ax = plt.subplots(figsize=(11, 8.5), dpi=100)

    # -------------------------------------------------------------------------
    # LAYER 1 (BOTTOM): DEM / Terrain Elevation Background (dem_clipped.tif)
    # -------------------------------------------------------------------------
    elevation_matrix = np.zeros((rows, cols), dtype=np.float64)
    for cell in grid.cells:
        elevation_matrix[cell.y, cell.x] = cell.elevation

    im_elevation = ax.imshow(
        elevation_matrix,
        cmap="viridis_r",
        origin="upper",
        aspect="equal",
        zorder=1,
    )
    cbar = fig.colorbar(im_elevation, ax=ax, shrink=0.78, pad=0.03)
    cbar.set_label("Elevation (m)", fontsize=10, fontweight="bold")

    # -------------------------------------------------------------------------
    # LAYER 2: Hillshade Relief Overlay (hillshade.tif)
    # -------------------------------------------------------------------------
    hillshade_path = Path("data/processed/hillshade.tif")
    if hillshade_path.exists():
        with rasterio.open(hillshade_path) as ds:
            hillshade_data = ds.read(1).astype(np.float32)
            h_min, h_max = np.nanmin(hillshade_data), np.nanmax(hillshade_data)
            if h_max > h_min:
                hillshade_norm = (hillshade_data - h_min) / (h_max - h_min)
            else:
                hillshade_norm = hillshade_data
            ax.imshow(
                hillshade_norm,
                cmap="gray",
                alpha=0.35,
                origin="upper",
                aspect="equal",
                zorder=2,
            )

    # -------------------------------------------------------------------------
    # LAYER 3: Vegetation / Fuel Type Overlay (fuel_type.tif)
    # -------------------------------------------------------------------------
    veg_rgba = np.zeros((rows, cols, 4), dtype=np.float32)
    for cell in grid.cells:
        ft = cell.fuel_type
        if ft == 1:
            # Low-density grassland (Light lime green)
            veg_rgba[cell.y, cell.x] = [0.45, 0.75, 0.25, 0.22]
        elif ft == 2:
            # Moderate-density shrubland (Medium forest green)
            veg_rgba[cell.y, cell.x] = [0.20, 0.55, 0.15, 0.32]
        elif ft == 3:
            # High-density forest (Deep pine green)
            veg_rgba[cell.y, cell.x] = [0.05, 0.35, 0.05, 0.42]

    ax.imshow(
        veg_rgba,
        origin="upper",
        aspect="equal",
        interpolation="nearest",
        zorder=3,
    )

    # -------------------------------------------------------------------------
    # LAYER 4: Roads Overlay (roads_raster.tif)
    # -------------------------------------------------------------------------
    roads_path = Path("data/processed/roads_raster.tif")
    if roads_path.exists():
        with rasterio.open(roads_path) as ds:
            roads_data = ds.read(1)
        roads_rgba = np.zeros((rows, cols, 4), dtype=np.float32)
        roads_mask = (roads_data == 1)
        # Dark asphalt gray line
        roads_rgba[roads_mask] = [0.15, 0.15, 0.18, 0.85]
        ax.imshow(
            roads_rgba,
            origin="upper",
            aspect="equal",
            interpolation="nearest",
            zorder=4,
        )

    # -------------------------------------------------------------------------
    # LAYER 5: 119 Safety Centers / Fire Bases (Inje 119 & Girin 119)
    # -------------------------------------------------------------------------
    stations = [
        {"name": "Inje 119", "lat": 38.0614106, "lon": 128.1685080},
        {"name": "Girin 119", "lat": 37.9648437, "lon": 128.3184546},
    ]

    station_cols, station_rows, station_names = [], [], []
    for st in stations:
        try:
            xs, ys = rasterio.warp.transform(
                "EPSG:4326", grid.crs, [st["lon"]], [st["lat"]]
            )
            r, c = grid.xy_to_rowcol(xs[0], ys[0])
            station_cols.append(c)
            station_rows.append(r)
            station_names.append(st["name"])
        except Exception:
            pass

    if station_cols:
        ax.scatter(
            station_cols,
            station_rows,
            marker="^",
            s=95,
            facecolor="#00E5FF",
            edgecolor="white",
            linewidths=1.5,
            zorder=12,
            label="119 Safety Center",
        )
        for c, r, name in zip(station_cols, station_rows, station_names):
            ax.text(
                c + 3,
                r - 2,
                name,
                color="white",
                fontsize=8,
                fontweight="bold",
                zorder=13,
                bbox=dict(
                    boxstyle="round,pad=0.2",
                    facecolor="#003366",
                    alpha=0.75,
                    edgecolor="#00E5FF",
                ),
            )

    # -------------------------------------------------------------------------
    # LAYER 6 (TOP): Fire Simulation Results (BURNING & BURNED)
    # -------------------------------------------------------------------------
    fire_rgba = np.zeros((rows, cols, 4), dtype=np.float32)

    def update_rgba_matrix() -> None:
        fire_rgba.fill(0.0)
        for cell in grid.cells:
            if cell.fire_state == FireState.BURNING:
                # Vibrant glowing red-orange (#FF2600)
                fire_rgba[cell.y, cell.x] = [1.0, 0.15, 0.0, 0.95]  # red, green, blue, alpha
            elif cell.fire_state == FireState.BURNED:
                # Deep charcoal black (#141414)
                fire_rgba[cell.y, cell.x] = [0.08, 0.08, 0.08, 0.85]

    update_rgba_matrix()
    im_fire = ax.imshow(
        fire_rgba,
        origin="upper",
        aspect="equal",
        interpolation="nearest",
        zorder=5,
    )

    # Active Fire Perimeter Overlay
    perimeter_cells = api.get_fire_perimeter() if show_perimeter else []
    px = [p["x"] for p in perimeter_cells]
    py = [p["y"] for p in perimeter_cells]
    scat_perim = ax.scatter(
        px,
        py,
        s=10,
        facecolors="none",
        edgecolors="#FFE066",
        linewidths=0.6,
        alpha=0.7,
        zorder=6,
        label="Fire Perimeter",
    )

    # Wind Vector Indicator Arrow
    spread_info = api.get_spread_direction()
    wind_dx = spread_info["dx"]
    wind_dy = spread_info["dy"]

    wind_x = cols * 0.92
    wind_y = rows * 0.08
    ax.quiver(
        [wind_x],
        [wind_y],
        [wind_dx],
        [wind_dy],
        color="#00FFFF",
        scale=12,
        width=0.008,
        headwidth=4,
        headlength=5,
        zorder=14,
    )

    # Multi-Layer Legend
    patch_burning = mpatches.Patch(color="#FF2600", label="BURNING (#FF2600)")
    patch_burned = mpatches.Patch(color="#141414", label="BURNED (#141414)")
    marker_station = plt.Line2D(
        [0],
        [0],
        marker="^",
        color="w",
        markerfacecolor="#00E5FF",
        markeredgecolor="w",
        markersize=8,
        label="119 Safety Center",
    )
    patch_road = mpatches.Patch(color="#26262E", label="Road Network")
    patch_forest = mpatches.Patch(color="#0D590D", label="Dense Forest (Fuel 3)")
    patch_hillshade = mpatches.Patch(color="#888888", label="Hillshade Relief")

    legend_handles = [
        patch_burning,
        patch_burned,
        marker_station,
        patch_road,
        patch_forest,
        patch_hillshade,
    ]

    if show_perimeter:
        line_perim = plt.Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markeredgecolor="#FFE066",
            markersize=5,
            label="Fire Perimeter",
        )
        legend_handles.append(line_perim)

    ax.legend(
        handles=legend_handles,
        loc="lower right",
        framealpha=0.85,
        fontsize=8.5,
    )

    ax.set_xlabel("Grid Column (X)", fontsize=10)
    ax.set_ylabel("Grid Row (Y)", fontsize=10)

    # Real-Time HUD Update
    def update_hud(step_num: int) -> None:
        env_state = api.get_environment_state()
        b_cnt = env_state["burning_cell_count"]
        bd_cnt = env_state["burned_cell_count"]
        ver = env_state["state_version"]
        w_spd = spread_info["wind_speed_ms"]
        w_deg = spread_info["wind_direction_deg"]
        cardinal = spread_info["primary_cardinal"]

        title_str = (
            f"Integrated GIS Wildfire Simulation | Step {step_num}/{steps} (State v{ver})\n"
            f"Burning: {b_cnt} cells | Burned: {bd_cnt} cells | Wind: {w_spd:.1f} m/s ({w_deg:.0f}°, {cardinal})"
        )
        ax.set_title(title_str, fontsize=11, fontweight="bold", pad=10)

    update_hud(0)

    # Animation Frame Update
    def animate(frame: int) -> None:
        if frame > 0:
            api.step()
            update_rgba_matrix()

        im_fire.set_data(fire_rgba)

        if show_perimeter:
            perim = api.get_fire_perimeter()
            if perim:
                pts = np.column_stack([[p["x"] for p in perim], [p["y"] for p in perim]])
                scat_perim.set_offsets(pts)
            else:
                scat_perim.set_offsets(np.empty((0, 2)))

        update_hud(frame)

    anim = animation.FuncAnimation(
        fig,
        animate,
        frames=steps + 1,
        interval=interval,
        repeat=False,
    )

    if save_path:
        out_path = Path(save_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        print(f"Saving animation to {out_path} (fps={fps})...")
        if out_path.suffix.lower() == ".gif":
            anim.save(str(out_path), writer="pillow", fps=fps)
        else:
            anim.save(str(out_path), fps=fps)
        print(f"Saved integrated GIS visualization successfully to {out_path}")
        plt.close(fig)
    else:
        plt.tight_layout()
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dynamic 2D Wildfire Simulation Integrated GIS Visualization Tool"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/environment_config.yaml",
        help="Path to YAML configuration file (default: config/environment_config.yaml)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Simulation seed for reproducibility (default: 42)",
    )
    parser.add_argument(
        "--x",
        type=int,
        default=None,
        help="Initial ignition column index (default: grid center)",
    )
    parser.add_argument(
        "--y",
        type=int,
        default=None,
        help="Initial ignition row index (default: grid center)",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=30,
        help="Total simulation steps to run (default: 30)",
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=200,
        help="Animation frame delay in milliseconds (default: 200)",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="Output file path to save animation (e.g. fire_expansion.gif)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=5,
        help="Frame rate for saved animation (default: 5)",
    )
    parser.add_argument(
        "--hide-perimeter",
        action="store_true",
        help="Hide active fire perimeter boundary overlay markers for unobstructed cell color visibility",
    )

    args = parser.parse_args()

    create_visualization(
        config_path=args.config,
        seed=args.seed,
        ignition_x=args.x,
        ignition_y=args.y,
        steps=args.steps,
        interval=args.interval,
        save_path=args.save,
        fps=args.fps,
        show_perimeter=not args.hide_perimeter,
    )


if __name__ == "__main__":
    main()
