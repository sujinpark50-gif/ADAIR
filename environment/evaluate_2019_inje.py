"""Wildfire simulation evaluation runner for the 2019 Inje wildfire reconstruction.

Runs the Cellular Automaton simulation against the 2019 historical scenario,
extracts the final simulated affected burn footprint, compares it quantitatively
against the calibrated 2019 reference burn raster, logs evaluation metrics,
and records the run to CSV.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import logging
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional
import uuid

# Ensure local package import works when invoked directly
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import yaml

from src.cell import FireState
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

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("evaluate_2019_inje")

CSV_HEADER = [
    "run_id",
    "timestamp",
    "wind_direction_convention",
    "weather_mode",
    "random_seed",
    "simulation_start_time",
    "simulation_end_time",
    "timestep_minutes",
    "total_steps_run",
    "iou",
    "dice",
    "precision",
    "recall",
    "tp_cells",
    "fp_cells",
    "fn_cells",
    "tn_cells",
    "simulated_burned_cells",
    "reference_burned_cells",
    "simulated_area_ha",
    "reference_area_ha",
    "area_error_ha",
    "area_error_percent",
    "absolute_area_error_ha",
    "absolute_area_error_percent",
    "base_spread_probability",
    "burn_duration_steps",
    "wind_factor_weight",
    "slope_factor_weight",
    "moisture_dampening_weight",
    "roads_block_spread",
]


def load_config(config_path: Path) -> dict:
    """Load YAML config file."""
    with open(config_path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def append_result_to_csv(csv_path: Path, row_data: Dict[str, Any]) -> None:
    """Append a single evaluation run row to CSV, creating file and header if missing."""
    csv_path = csv_path.resolve()
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    file_exists = csv_path.is_file()
    with open(csv_path, mode="a", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_HEADER)
        if not file_exists:
            writer.writeheader()
        writer.writerow({k: row_data.get(k, "") for k in CSV_HEADER})


def run_single_evaluation(
    config_dict: dict,
    config_path: Path,
    convention: str,
    reference_path: Path,
    seed: Optional[int] = None,
    max_steps: Optional[int] = None,
    run_id: Optional[str] = None,
    save_geotiff_dir: Optional[Path] = None,
) -> Tuple[FireEvaluationResult, Dict[str, Any]]:
    """Run simulation and evaluate against reference raster for a given convention."""
    if run_id is None:
        run_id = f"eval_{convention}_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{str(uuid.uuid4())[:6]}"

    # Set convention in cloned config
    cfg = yaml.safe_load(yaml.safe_dump(config_dict))
    if "weather" not in cfg:
        cfg["weather"] = {}
    cfg["weather"]["wind_direction_convention"] = convention

    actual_seed = (
        seed
        if seed is not None
        else (
            cfg.get("simulation", {}).get("random_seed")
            if isinstance(cfg.get("simulation"), dict)
            else cfg.get("random_seed", cfg.get("seed", 42))
        )
    )

    logger.info("Initializing EnvironmentGrid with seed=%s ...", actual_seed)
    grid = EnvironmentGrid(config_path, seed=actual_seed)

    logger.info(
        "Initializing WildfireCAEngine (mode=%s, convention=%s) ...",
        cfg.get("weather", {}).get("mode", "fixed"),
        convention,
    )
    engine = WildfireCAEngine(grid, config_path=cfg, seed=actual_seed)
    engine.ignite_from_config()

    logger.info("Starting simulation loop from %s ...", engine.current_simulation_time.isoformat())
    step_count = 0
    while not engine.simulation_finished:
        if max_steps is not None and step_count >= max_steps:
            logger.info("Stopping at max_steps=%d limit", max_steps)
            break
        engine.step()
        step_count += 1
        if step_count % 30 == 0:
            num_burning = sum(1 for c in grid.cells if c.fire_state == FireState.BURNING)
            num_burned = sum(1 for c in grid.cells if c.fire_state == FireState.BURNED)
            logger.info(
                "Step %d: sim_time=%s burning=%d burned=%d",
                step_count,
                engine.current_simulation_time.isoformat(),
                num_burning,
                num_burned,
            )

    num_burning = sum(1 for c in grid.cells if c.fire_state == FireState.BURNING)
    num_burned = sum(1 for c in grid.cells if c.fire_state == FireState.BURNED)
    logger.info(
        "Simulation completed at step %d (%s). Active burning: %d, Burned: %d",
        engine.step_count,
        engine.current_simulation_time.isoformat(),
        num_burning,
        num_burned,
    )

    # 1. Create simulated burn mask
    simulated_mask = create_simulated_burn_mask(grid)

    # 2. Load reference mask & validate
    ref_mask, valid_mask, ref_meta = load_reference_burn_mask(reference_path, expected_grid=grid)

    # 3. Evaluate metrics
    result = evaluate_fire_footprint(
        simulated_mask=simulated_mask,
        reference_mask=ref_mask,
        valid_mask=valid_mask,
        cell_area_ha=None,
        cell_resolution_m=grid.cell_resolution,
    )

    # 4. Optional GeoTIFF export
    if save_geotiff_dir is not None:
        save_geotiff_dir.mkdir(parents=True, exist_ok=True)
        sim_tif = save_geotiff_dir / f"simulated_burn_{convention}.tif"
        save_geotiff(sim_tif, simulated_mask, grid.transform, grid.crs, nodata=None)
        logger.info("Saved simulated burn mask GeoTIFF to %s", sim_tif)

        overlap_mask = generate_overlap_classification_mask(
            simulated_mask, ref_mask, valid_mask=valid_mask, nodata_val=255
        )
        overlap_tif = save_geotiff_dir / f"overlap_classification_{convention}.tif"
        save_geotiff(overlap_tif, overlap_mask, grid.transform, grid.crs, nodata=255)
        logger.info("Saved overlap classification GeoTIFF to %s", overlap_tif)

    ca_cfg = cfg.get("fire_ca", {})
    sim_cfg = cfg.get("simulation", {})
    metadata = {
        "run_id": run_id,
        "timestamp": datetime.now().isoformat(),
        "wind_direction_convention": convention,
        "weather_mode": cfg.get("weather", {}).get("mode", "fixed"),
        "random_seed": actual_seed,
        "simulation_start_time": sim_cfg.get("start_time", "2019-04-04T14:45:00"),
        "simulation_end_time": sim_cfg.get("end_time", "2019-04-06T19:00:00"),
        "timestep_minutes": sim_cfg.get("timestep_minutes", 10),
        "total_steps_run": engine.step_count,
        "iou": round(result.iou, 6),
        "dice": round(result.dice, 6),
        "precision": round(result.precision, 6),
        "recall": round(result.recall, 6),
        "tp_cells": result.true_positive_cells,
        "fp_cells": result.false_positive_cells,
        "fn_cells": result.false_negative_cells,
        "tn_cells": result.true_negative_cells,
        "simulated_burned_cells": result.simulated_burned_cells,
        "reference_burned_cells": result.reference_burned_cells,
        "simulated_area_ha": round(result.simulated_area_ha, 2),
        "reference_area_ha": round(result.reference_area_ha, 2),
        "area_error_ha": round(result.area_error_ha, 2),
        "area_error_percent": round(result.area_error_percent, 2),
        "absolute_area_error_ha": round(result.absolute_area_error_ha, 2),
        "absolute_area_error_percent": round(result.absolute_area_error_percent, 2),
        "base_spread_probability": ca_cfg.get("base_spread_probability", 0.7),
        "burn_duration_steps": ca_cfg.get("burn_duration_steps", 3),
        "wind_factor_weight": ca_cfg.get("wind_factor_weight", 0.2),
        "slope_factor_weight": ca_cfg.get("slope_factor_weight", 0.05),
        "moisture_dampening_weight": ca_cfg.get("moisture_dampening_weight", 1.0),
        "roads_block_spread": ca_cfg.get("roads_block_spread", True),
    }

    return result, metadata


def format_evaluation_report(result: FireEvaluationResult, meta: Dict[str, Any]) -> str:
    """Format evaluation result as human-readable report."""
    lines = [
        "=" * 70,
        f"2019 Inje Wildfire Simulation Evaluation Report ({meta['wind_direction_convention'].upper()})",
        "=" * 70,
        f"Run ID:                    {meta['run_id']}",
        f"Weather Mode:              {meta['weather_mode']}",
        f"Wind Direction Convention: {meta['wind_direction_convention']}",
        f"Random Seed:               {meta['random_seed']}",
        f"Simulation Steps Run:      {meta['total_steps_run']} ({meta['timestep_minutes']} min/step)",
        "-" * 70,
        "Spatial Agreement Metrics:",
        f"  IoU (Jaccard Index):     {result.iou:.4f}  ({result.iou * 100.0:.2f}%)",
        f"  Dice Coefficient (F1):   {result.dice:.4f}  ({result.dice * 100.0:.2f}%)",
        f"  Spatial Precision:       {result.precision:.4f}  ({result.precision * 100.0:.2f}%)",
        f"  Spatial Recall:          {result.recall:.4f}  ({result.recall * 100.0:.2f}%)",
        "-" * 70,
        "Confusion Matrix (Cell Counts @ 90m resolution):",
        f"  True Positive (TP):      {result.true_positive_cells:6d} cells",
        f"  False Positive (FP):     {result.false_positive_cells:6d} cells",
        f"  False Negative (FN):     {result.false_negative_cells:6d} cells",
        f"  True Negative (TN):      {result.true_negative_cells:6d} cells",
        "-" * 70,
        "Burned Area Comparison:",
        f"  Simulated Burned Area:   {result.simulated_area_ha:8.2f} ha ({result.simulated_burned_cells} cells)",
        f"  Reference Burned Area:   {result.reference_area_ha:8.2f} ha ({result.reference_burned_cells} cells)",
        f"  Signed Area Error:       {result.area_error_ha:+8.2f} ha ({result.area_error_percent:+.2f}%)",
        "=" * 70,
    ]
    return "\n".join(lines)


def main() -> None:
    """CLI entrypoint for 2019 Inje evaluation."""
    parser = argparse.ArgumentParser(
        description="Run 2019 Inje wildfire simulation and quantitatively evaluate against reference footprint."
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/environment_config.yaml",
        help="Path to YAML configuration file.",
    )
    parser.add_argument(
        "--convention",
        type=str,
        choices=["from", "toward", "both"],
        default=None,
        help="Wind direction convention to evaluate ('from', 'toward', or 'both'). Defaults to config value.",
    )
    parser.add_argument(
        "--reference",
        type=str,
        default=None,
        help="Path to 2019 burn reference binary GeoTIFF raster.",
    )
    parser.add_argument(
        "--output-csv",
        type=str,
        default="results/inje_2019_evaluation.csv",
        help="Destination CSV path to append evaluation records.",
    )
    parser.add_argument(
        "--save-geotiff",
        action="store_true",
        help="Export simulated burn mask and confusion classification GeoTIFFs to results directory.",
    )
    parser.add_argument(
        "--geotiff-dir",
        type=str,
        default="results",
        help="Directory to save output GeoTIFFs when --save-geotiff is set.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=None,
        help="Maximum simulation steps to run (defaults to simulation.end_time).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Override random seed for simulation.",
    )

    args = parser.parse_args()

    # Resolve config path
    cfg_p = Path(args.config)
    if not cfg_p.is_file():
        # Try relative to environment directory or project root
        for cand in (SCRIPT_DIR / args.config, PROJECT_ROOT / args.config):
            if cand.is_file():
                cfg_p = cand.resolve()
                break
    if not cfg_p.is_file():
        logger.error("Configuration file not found: %s", args.config)
        sys.exit(1)

    config_dict = load_config(cfg_p)

    # Resolve reference raster path
    try:
        ref_path = resolve_reference_raster_path(args.reference, base_dir=cfg_p.parent)
    except FileNotFoundError as exc:
        logger.error(str(exc))
        sys.exit(1)

    logger.info("Using 2019 burn reference raster: %s", ref_path)

    # Determine conventions to run
    if args.convention == "both":
        conventions = ["from", "toward"]
    elif args.convention in ("from", "toward"):
        conventions = [args.convention]
    else:
        conventions = [config_dict.get("weather", {}).get("wind_direction_convention", "from")]

    # Output CSV path resolution
    out_csv = Path(args.output_csv)
    if not out_csv.is_absolute():
        out_csv = (SCRIPT_DIR / out_csv).resolve()

    geotiff_dir = (SCRIPT_DIR / args.geotiff_dir).resolve() if args.save_geotiff else None

    for conv in conventions:
        logger.info(">>> Running evaluation for wind_direction_convention='%s' <<<", conv)
        result, meta = run_single_evaluation(
            config_dict=config_dict,
            config_path=cfg_p,
            convention=conv,
            reference_path=ref_path,
            seed=args.seed,
            max_steps=args.steps,
            save_geotiff_dir=geotiff_dir,
        )

        report = format_evaluation_report(result, meta)
        print("\n" + report + "\n")

        append_result_to_csv(out_csv, meta)
        logger.info("Appended evaluation run record to: %s", out_csv)


if __name__ == "__main__":
    main()
