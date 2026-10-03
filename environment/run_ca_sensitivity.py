"""CA Parameter Sensitivity & Grid Search Experiment Runner for 2019 Inje Wildfire.

Orchestrates parameter sensitivity sweeps (Stage A: OAT) and multi-parameter
calibration grid search (Stage B) for the 2019 Inje wildfire simulation case.

Key Guarantees:
- Loads baseline configuration from YAML and applies in-memory overrides per run.
- Instantiates a completely fresh EnvironmentGrid and WildfireCAEngine for each simulation run.
- Ensures identical common seed sets are evaluated across all tested parameter values.
- Reuses src.evaluation for all spatial overlap and error metric calculations.
- Appends completed run rows immediately to CSV with full reproducibility metadata.
- Implements resume capabilities that skip already completed parameter-seed tuples.
- Provides dry-run analysis and summary aggregation across stochastic seed sets.
"""

from __future__ import annotations

import argparse
import copy
import csv
from datetime import datetime
import json
import logging
from pathlib import Path
import subprocess
import sys
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple
import uuid

# Ensure local environment package imports work correctly
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name == "environment" else SCRIPT_DIR
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import yaml

from src.cell import FireState
from src.evaluation import (
    FireEvaluationResult,
    create_simulated_burn_mask,
    evaluate_fire_footprint,
    load_reference_burn_mask,
    resolve_reference_raster_path,
)
from src.fire_model import WildfireCAEngine
from src.grid import EnvironmentGrid

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("run_ca_sensitivity")

# --- Full CSV Column Schema for Raw Simulation Runs ---
RAW_CSV_HEADER = [
    "run_id",
    "timestamp",
    "git_commit",
    "experiment_type",
    "parameter_name",
    "parameter_value",
    "seed",
    "wind_direction_convention",
    "weather_mode",
    "base_spread_probability",
    "wind_factor_weight",
    "slope_factor_weight",
    "moisture_dampening_weight",
    "burn_duration_steps",
    "roads_block_spread",
    "total_steps_run",
    "iou",
    "dice",
    "precision",
    "recall",
    "max_iou",
    "step_at_max_iou",
    "timestamp_at_max_iou",
    "elapsed_minutes_at_max_iou",
    "simulated_area_at_max_iou_ha",
    "area_error_at_max_iou_ha",
    "dice_at_max_iou",
    "precision_at_max_iou",
    "recall_at_max_iou",
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
    "simulation_start_time",
    "simulation_end_time",
    "timestep_minutes",
    "reference_raster_path",
    "status",
    "error_message",
]

# --- CSV Column Schema for Step-by-Step Temporal Simulation Trajectories ---
TEMPORAL_RAW_CSV_HEADER = [
    "run_id",
    "experiment_type",
    "parameter_name",
    "parameter_value",
    "seed",
    "simulation_step",
    "simulation_timestamp",
    "elapsed_simulation_minutes",
    "iou",
    "dice",
    "precision",
    "recall",
    "simulated_burned_cells",
    "simulated_burned_area_ha",
    "reference_burned_area_ha",
    "signed_area_error_ha",
]

# --- CSV Column Schema for Seed-Aggregated Summary ---
SUMMARY_CSV_HEADER = [
    "experiment_type",
    "parameter_name",
    "parameter_value",
    "runs_count",
    "seeds",
    "base_spread_probability",
    "wind_factor_weight",
    "slope_factor_weight",
    "burn_duration_steps",
    "moisture_dampening_weight",
    "mean_iou",
    "std_iou",
    "mean_dice",
    "std_dice",
    "mean_precision",
    "std_precision",
    "mean_recall",
    "std_recall",
    "mean_simulated_area_ha",
    "std_simulated_area_ha",
    "reference_area_ha",
    "mean_area_error_ha",
    "std_area_error_ha",
    "mean_absolute_area_error_ha",
    "std_absolute_area_error_ha",
]

# --- CSV Column Schema for Temporal Peak Diagnostic Summary ---
TEMPORAL_SUMMARY_CSV_HEADER = [
    "experiment_type",
    "parameter_name",
    "parameter_value",
    "runs_count",
    "seeds",
    "base_spread_probability",
    "wind_factor_weight",
    "slope_factor_weight",
    "burn_duration_steps",
    "mean_max_iou",
    "std_max_iou",
    "mean_step_at_max_iou",
    "std_step_at_max_iou",
    "mean_elapsed_min_at_max_iou",
    "mean_simulated_area_at_max_iou_ha",
    "std_simulated_area_at_max_iou_ha",
    "mean_area_error_at_max_iou_ha",
    "mean_dice_at_max_iou",
    "mean_precision_at_max_iou",
    "mean_recall_at_max_iou",
    "mean_final_iou",
    "std_final_iou",
    "mean_final_simulated_area_ha",
    "std_final_simulated_area_ha",
]


def get_git_commit_hash() -> str:
    """Safely obtain current git commit hash if git repository is available."""
    try:
        res = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=False,
            cwd=str(PROJECT_ROOT),
        )
        if res.returncode == 0:
            return res.stdout.strip()
    except Exception:
        pass
    return "unknown"


def load_yaml(path: Path) -> dict:
    """Load and parse YAML configuration file."""
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return data or {}


def get_completed_run_keys(csv_path: Path) -> Set[str]:
    """Read existing CSV file and return set of completed unique execution keys."""
    if not csv_path.is_file():
        return set()
    completed = set()
    with open(csv_path, "r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            if row.get("status", "completed") == "completed":
                key = f"{row.get('experiment_type')}_{row.get('parameter_name')}_{row.get('parameter_value')}_{row.get('seed')}_{row.get('base_spread_probability')}_{row.get('wind_factor_weight')}_{row.get('slope_factor_weight')}_{row.get('burn_duration_steps')}"
                completed.add(key)
    return completed


def make_run_key(
    experiment_type: str,
    param_name: str,
    param_val: Any,
    seed: int,
    base_prob: float,
    wind_weight: float,
    slope_weight: float,
    burn_duration: int,
) -> str:
    """Generate unique identifier string for a specific parameter-seed run."""
    return f"{experiment_type}_{param_name}_{param_val}_{seed}_{base_prob}_{wind_weight}_{slope_weight}_{burn_duration}"


def append_raw_run_to_csv(csv_path: Path, row_dict: Dict[str, Any]) -> None:
    """Append a single run record to the raw CSV file."""
    csv_path = csv_path.resolve()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = csv_path.is_file() and csv_path.stat().st_size > 0
    with open(csv_path, "a", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=RAW_CSV_HEADER)
        if not file_exists:
            writer.writeheader()
        writer.writerow({k: row_dict.get(k, "") for k in RAW_CSV_HEADER})


def append_temporal_rows_to_csv(csv_path: Path, rows: List[Dict[str, Any]]) -> None:
    """Append a batch of step-by-step temporal simulation records to the temporal CSV file."""
    if not rows:
        return
    csv_path = csv_path.resolve()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = csv_path.is_file() and csv_path.stat().st_size > 0
    with open(csv_path, "a", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=TEMPORAL_RAW_CSV_HEADER)
        if not file_exists:
            writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k, "") for k in TEMPORAL_RAW_CSV_HEADER})


def run_single_simulation(
    env_config_base: dict,
    env_config_path: Path,
    overrides: Dict[str, Any],
    seed: int,
    reference_path: Path,
    experiment_type: str,
    parameter_name: str,
    parameter_value: Any,
    git_hash: str,
    max_steps: Optional[int] = None,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """Execute a single CA simulation from fresh grid/engine instances and return evaluated metrics."""
    run_id = f"{experiment_type}_{parameter_name}_{parameter_value}_s{seed}_{str(uuid.uuid4())[:6]}"
    now_ts = datetime.now().isoformat()

    # 1. Create deep in-memory copy of baseline configuration
    cfg = copy.deepcopy(env_config_base)

    # Disable expensive nested Monte Carlo in step risk updates for CA sensitivity sweeps
    cfg.setdefault("risk_scoring", {})["spread_mode"] = "geometric"

    # 2. Apply CA parameter overrides
    fire_ca_cfg = cfg.setdefault("fire_ca", {})
    for k, v in overrides.items():
        fire_ca_cfg[k] = v

    # Ensure required weather mode and convention are preserved
    weather_cfg = cfg.setdefault("weather", {})
    conv = weather_cfg.get("wind_direction_convention", "toward")
    w_mode = weather_cfg.get("mode", "timeseries")

    base_prob = float(fire_ca_cfg.get("base_spread_probability", 0.70))
    wind_w = float(fire_ca_cfg.get("wind_factor_weight", 0.20))
    slope_w = float(fire_ca_cfg.get("slope_factor_weight", 0.05))
    moisture_w = float(fire_ca_cfg.get("moisture_dampening_weight", 1.0))
    burn_dur = int(fire_ca_cfg.get("burn_duration_steps", 3))
    roads_block = bool(fire_ca_cfg.get("roads_block_spread", True))

    sim_cfg = cfg.get("simulation", {})
    start_time_str = sim_cfg.get("start_time", "2019-04-04T14:45:00")
    end_time_str = sim_cfg.get("end_time", "2019-04-06T12:05:00")
    timestep_min = float(sim_cfg.get("timestep_minutes", 10))

    base_row: Dict[str, Any] = {
        "run_id": run_id,
        "timestamp": now_ts,
        "git_commit": git_hash,
        "experiment_type": experiment_type,
        "parameter_name": parameter_name,
        "parameter_value": str(parameter_value),
        "seed": seed,
        "wind_direction_convention": conv,
        "weather_mode": w_mode,
        "base_spread_probability": base_prob,
        "wind_factor_weight": wind_w,
        "slope_factor_weight": slope_w,
        "moisture_dampening_weight": moisture_w,
        "burn_duration_steps": burn_dur,
        "roads_block_spread": roads_block,
        "simulation_start_time": start_time_str,
        "simulation_end_time": end_time_str,
        "timestep_minutes": timestep_min,
        "reference_raster_path": str(reference_path),
        "status": "completed",
        "error_message": "",
    }

    temporal_rows: List[Dict[str, Any]] = []

    try:
        # 3. Instantiate fresh EnvironmentGrid
        grid = EnvironmentGrid(env_config_path, seed=seed)

        # 4. Load reference burn mask & precalculate cell sets for step-by-step evaluation
        ref_mask, val_mask, _ = load_reference_burn_mask(reference_path, expected_grid=grid)
        ref_cell_ids = {c.cell_id for c in grid.cells if ref_mask[c.y, c.x] and val_mask[c.y, c.x]}
        valid_cell_ids = {c.cell_id for c in grid.cells if val_mask[c.y, c.x]}
        ref_burned_cells = len(ref_cell_ids)
        cell_area_ha = (grid.cell_resolution ** 2) / 10000.0
        ref_area_ha = round(ref_burned_cells * cell_area_ha, 2)

        # 5. Instantiate fresh WildfireCAEngine
        engine = WildfireCAEngine(grid, config_path=cfg, seed=seed)
        # Suppress per-step grid risk score recalculation during sensitivity runs to accelerate footprint simulation
        engine.predict_spread = lambda: {}  # type: ignore

        # 6. Ignite historical fire origin
        engine.ignite_from_config()

        def _evaluate_step_metrics(step_idx: int, sim_time: datetime) -> Dict[str, Any]:
            ignited_ids = set(engine._burn_counters.keys()) & valid_cell_ids
            sim_cells = len(ignited_ids)
            tp = len(ignited_ids & ref_cell_ids)
            fp = sim_cells - tp
            fn = ref_burned_cells - tp
            union = sim_cells + ref_burned_cells - tp
            s_iou = (tp / union) if union > 0 else 0.0
            s_dice = (2.0 * tp) / (sim_cells + ref_burned_cells) if (sim_cells + ref_burned_cells) > 0 else 0.0
            s_prec = (tp / sim_cells) if sim_cells > 0 else 0.0
            s_rec = (tp / ref_burned_cells) if ref_burned_cells > 0 else 0.0
            s_sim_area = round(sim_cells * cell_area_ha, 2)
            s_area_err = round(s_sim_area - ref_area_ha, 2)
            return {
                "run_id": run_id,
                "experiment_type": experiment_type,
                "parameter_name": parameter_name,
                "parameter_value": str(parameter_value),
                "seed": seed,
                "simulation_step": step_idx,
                "simulation_timestamp": sim_time.isoformat(),
                "elapsed_simulation_minutes": round(step_idx * timestep_min, 1),
                "iou": round(s_iou, 6),
                "dice": round(s_dice, 6),
                "precision": round(s_prec, 6),
                "recall": round(s_rec, 6),
                "simulated_burned_cells": sim_cells,
                "simulated_burned_area_ha": s_sim_area,
                "reference_burned_area_ha": ref_area_ha,
                "signed_area_error_ha": s_area_err,
                "tp_cells": tp,
                "fp_cells": fp,
                "fn_cells": fn,
            }

        temporal_rows.append(_evaluate_step_metrics(0, engine._simulation_time))

        # 7. Run simulation loop until termination
        step_count = 0
        while not engine.simulation_finished:
            if max_steps is not None and step_count >= max_steps:
                break
            engine.step()
            step_count += 1
            temporal_rows.append(_evaluate_step_metrics(step_count, engine._simulation_time))

        base_row["total_steps_run"] = engine.step_count

        # Extract final step evaluation
        final_eval = temporal_rows[-1]
        base_row.update(
            {
                "iou": final_eval["iou"],
                "dice": final_eval["dice"],
                "precision": final_eval["precision"],
                "recall": final_eval["recall"],
                "tp_cells": final_eval["tp_cells"],
                "fp_cells": final_eval["fp_cells"],
                "fn_cells": final_eval["fn_cells"],
                "tn_cells": len(valid_cell_ids) - (final_eval["tp_cells"] + final_eval["fp_cells"] + final_eval["fn_cells"]),
                "simulated_burned_cells": final_eval["simulated_burned_cells"],
                "reference_burned_cells": ref_burned_cells,
                "simulated_area_ha": final_eval["simulated_burned_area_ha"],
                "reference_area_ha": ref_area_ha,
                "area_error_ha": final_eval["signed_area_error_ha"],
                "area_error_percent": round((final_eval["signed_area_error_ha"] / ref_area_ha) * 100.0, 2) if ref_area_ha > 0 else 0.0,
                "absolute_area_error_ha": round(abs(final_eval["signed_area_error_ha"]), 2),
                "absolute_area_error_percent": round(abs(final_eval["signed_area_error_ha"] / ref_area_ha) * 100.0, 2) if ref_area_ha > 0 else 0.0,
                "status": "completed",
            }
        )

        # Diagnostic metrics at maximum IoU during trajectory
        best_step = max(temporal_rows, key=lambda r: (r["iou"], -abs(r["signed_area_error_ha"])))
        base_row.update(
            {
                "max_iou": best_step["iou"],
                "step_at_max_iou": best_step["simulation_step"],
                "timestamp_at_max_iou": best_step["simulation_timestamp"],
                "elapsed_minutes_at_max_iou": best_step["elapsed_simulation_minutes"],
                "simulated_area_at_max_iou_ha": best_step["simulated_burned_area_ha"],
                "area_error_at_max_iou_ha": best_step["signed_area_error_ha"],
                "dice_at_max_iou": best_step["dice"],
                "precision_at_max_iou": best_step["precision"],
                "recall_at_max_iou": best_step["recall"],
            }
        )

    except Exception as exc:
        logger.exception("Simulation run failed for %s: %s", run_id, exc)
        base_row.update(
            {
                "status": "failed",
                "error_message": str(exc),
                "total_steps_run": 0,
                "iou": 0.0,
                "dice": 0.0,
                "precision": 0.0,
                "recall": 0.0,
                "max_iou": 0.0,
                "step_at_max_iou": 0,
                "timestamp_at_max_iou": "",
                "elapsed_minutes_at_max_iou": 0.0,
                "simulated_area_at_max_iou_ha": 0.0,
                "area_error_at_max_iou_ha": 0.0,
                "dice_at_max_iou": 0.0,
                "precision_at_max_iou": 0.0,
                "recall_at_max_iou": 0.0,
                "tp_cells": 0,
                "fp_cells": 0,
                "fn_cells": 0,
                "tn_cells": 0,
                "simulated_burned_cells": 0,
                "reference_burned_cells": 0,
                "simulated_area_ha": 0.0,
                "reference_area_ha": 0.0,
                "area_error_ha": 0.0,
                "area_error_percent": 0.0,
                "absolute_area_error_ha": 0.0,
                "absolute_area_error_percent": 0.0,
            }
        )
        temporal_rows = []

    return base_row, temporal_rows


def compute_summary_records(raw_records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Aggregate raw simulation rows across random seeds into statistical summary metrics."""
    deduped_groups: Dict[Tuple[str, str, str], Dict[int, Dict[str, Any]]] = {}

    for r in raw_records:
        if r.get("status") != "completed":
            continue
        key = (
            str(r.get("experiment_type")),
            str(r.get("parameter_name")),
            str(r.get("parameter_value")),
        )
        seed = int(r.get("seed", 0))
        deduped_groups.setdefault(key, {})[seed] = r

    groups: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = {
        k: list(v.values()) for k, v in deduped_groups.items()
    }

    summaries: List[Dict[str, Any]] = []

    for (exp_type, param_name, param_val), rows in groups.items():
        if not rows:
            continue

        seeds_list = sorted([int(x["seed"]) for x in rows])
        seeds_str = ",".join(str(s) for s in seeds_list)

        ious = np.array([float(x["iou"]) for x in rows])
        dices = np.array([float(x["dice"]) for x in rows])
        precs = np.array([float(x["precision"]) for x in rows])
        recalls = np.array([float(x["recall"]) for x in rows])
        sim_areas = np.array([float(x["simulated_area_ha"]) for x in rows])
        area_errs = np.array([float(x["area_error_ha"]) for x in rows])
        abs_area_errs = np.array([float(x["absolute_area_error_ha"]) for x in rows])
        ref_area = float(rows[0]["reference_area_ha"])

        summary_row = {
            "experiment_type": exp_type,
            "parameter_name": param_name,
            "parameter_value": param_val,
            "runs_count": len(rows),
            "seeds": seeds_str,
            "base_spread_probability": rows[0]["base_spread_probability"],
            "wind_factor_weight": rows[0]["wind_factor_weight"],
            "slope_factor_weight": rows[0]["slope_factor_weight"],
            "burn_duration_steps": rows[0]["burn_duration_steps"],
            "moisture_dampening_weight": rows[0]["moisture_dampening_weight"],
            "mean_iou": round(float(np.mean(ious)), 6),
            "std_iou": round(float(np.std(ious, ddof=0)), 6),
            "mean_dice": round(float(np.mean(dices)), 6),
            "std_dice": round(float(np.std(dices, ddof=0)), 6),
            "mean_precision": round(float(np.mean(precs)), 6),
            "std_precision": round(float(np.std(precs, ddof=0)), 6),
            "mean_recall": round(float(np.mean(recalls)), 6),
            "std_recall": round(float(np.std(recalls, ddof=0)), 6),
            "mean_simulated_area_ha": round(float(np.mean(sim_areas)), 2),
            "std_simulated_area_ha": round(float(np.std(sim_areas, ddof=0)), 2),
            "reference_area_ha": round(ref_area, 2),
            "mean_area_error_ha": round(float(np.mean(area_errs)), 2),
            "std_area_error_ha": round(float(np.std(area_errs, ddof=0)), 2),
            "mean_absolute_area_error_ha": round(float(np.mean(abs_area_errs)), 2),
            "std_absolute_area_error_ha": round(float(np.std(abs_area_errs, ddof=0)), 2),
        }
        summaries.append(summary_row)

    return summaries


def compute_temporal_summary_records(raw_records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Aggregate raw simulation rows into temporal peak vs final summary metrics across seeds."""
    deduped_groups: Dict[Tuple[str, str, str], Dict[int, Dict[str, Any]]] = {}

    for r in raw_records:
        if r.get("status") != "completed":
            continue
        key = (
            str(r.get("experiment_type")),
            str(r.get("parameter_name")),
            str(r.get("parameter_value")),
        )
        seed = int(r.get("seed", 0))
        deduped_groups.setdefault(key, {})[seed] = r

    groups: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = {
        k: list(v.values()) for k, v in deduped_groups.items()
    }

    summaries: List[Dict[str, Any]] = []

    for (exp_type, param_name, param_val), rows in groups.items():
        if not rows:
            continue

        seeds_list = sorted([int(x["seed"]) for x in rows])
        seeds_str = ",".join(str(s) for s in seeds_list)

        max_ious = np.array([float(x.get("max_iou", x.get("iou", 0.0))) for x in rows])
        steps_at_max = np.array([int(x.get("step_at_max_iou", 0)) for x in rows])
        elaps_at_max = np.array([float(x.get("elapsed_minutes_at_max_iou", 0.0)) for x in rows])
        areas_at_max = np.array([float(x.get("simulated_area_at_max_iou_ha", x.get("simulated_area_ha", 0.0))) for x in rows])
        errs_at_max = np.array([float(x.get("area_error_at_max_iou_ha", x.get("area_error_ha", 0.0))) for x in rows])
        dices_at_max = np.array([float(x.get("dice_at_max_iou", x.get("dice", 0.0))) for x in rows])
        precs_at_max = np.array([float(x.get("precision_at_max_iou", x.get("precision", 0.0))) for x in rows])
        recs_at_max = np.array([float(x.get("recall_at_max_iou", x.get("recall", 0.0))) for x in rows])

        final_ious = np.array([float(x.get("iou", 0.0)) for x in rows])
        final_areas = np.array([float(x.get("simulated_area_ha", 0.0)) for x in rows])

        summary_row = {
            "experiment_type": exp_type,
            "parameter_name": param_name,
            "parameter_value": param_val,
            "runs_count": len(rows),
            "seeds": seeds_str,
            "base_spread_probability": rows[0]["base_spread_probability"],
            "wind_factor_weight": rows[0]["wind_factor_weight"],
            "slope_factor_weight": rows[0]["slope_factor_weight"],
            "burn_duration_steps": rows[0]["burn_duration_steps"],
            "mean_max_iou": round(float(np.mean(max_ious)), 6),
            "std_max_iou": round(float(np.std(max_ious, ddof=0)), 6),
            "mean_step_at_max_iou": round(float(np.mean(steps_at_max)), 1),
            "std_step_at_max_iou": round(float(np.std(steps_at_max, ddof=0)), 1),
            "mean_elapsed_min_at_max_iou": round(float(np.mean(elaps_at_max)), 1),
            "mean_simulated_area_at_max_iou_ha": round(float(np.mean(areas_at_max)), 2),
            "std_simulated_area_at_max_iou_ha": round(float(np.std(areas_at_max, ddof=0)), 2),
            "mean_area_error_at_max_iou_ha": round(float(np.mean(errs_at_max)), 2),
            "mean_dice_at_max_iou": round(float(np.mean(dices_at_max)), 6),
            "mean_precision_at_max_iou": round(float(np.mean(precs_at_max)), 6),
            "mean_recall_at_max_iou": round(float(np.mean(recs_at_max)), 6),
            "mean_final_iou": round(float(np.mean(final_ious)), 6),
            "std_final_iou": round(float(np.std(final_ious, ddof=0)), 6),
            "mean_final_simulated_area_ha": round(float(np.mean(final_areas)), 2),
            "std_final_simulated_area_ha": round(float(np.std(final_areas, ddof=0)), 2),
        }
        summaries.append(summary_row)

    return summaries


def write_summary_csv(
    csv_path: Path,
    summaries: List[Dict[str, Any]],
    fieldnames: Optional[List[str]] = None,
) -> None:
    """Write seed-aggregated summary metrics to CSV file."""
    csv_path = csv_path.resolve()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    header = fieldnames if fieldnames is not None else SUMMARY_CSV_HEADER
    with open(csv_path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=header)
        writer.writeheader()
        for s in summaries:
            writer.writerow({k: s.get(k, "") for k in header})


def run_experiment_suite(
    mode: str,
    env_config_path: Path,
    sens_config_path: Path,
    reference_path: Path,
    seeds_override: Optional[List[int]] = None,
    dry_run: bool = False,
    force: bool = False,
    max_steps: Optional[int] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Execute complete sensitivity / grid-search experiment suite according to mode."""
    env_cfg = load_yaml(env_config_path)
    sens_cfg = load_yaml(sens_config_path)

    seeds = seeds_override if seeds_override is not None else sens_cfg.get("seeds", [42, 43, 44])
    output_cfg = sens_cfg.get("output", {})
    if mode == "grid":
        raw_csv_rel = output_cfg.get("grid_raw_csv", "results/inje_2019_sensitivity_grid.csv")
        summary_csv_rel = output_cfg.get("grid_summary_csv", "results/inje_2019_sensitivity_grid_summary.csv")
        temporal_raw_rel = output_cfg.get("grid_temporal_raw_csv", "results/inje_2019_sensitivity_grid_temporal.csv")
        temporal_summary_rel = output_cfg.get("grid_temporal_summary_csv", "results/inje_2019_sensitivity_grid_temporal_summary.csv")
    elif mode == "refinement":
        raw_csv_rel = "results/inje_2019_percolation_refinement.csv"
        summary_csv_rel = "results/inje_2019_percolation_refinement_summary.csv"
        temporal_raw_rel = "results/inje_2019_percolation_refinement_temporal.csv"
        temporal_summary_rel = "results/inje_2019_percolation_refinement_temporal_summary.csv"
    else:
        raw_csv_rel = output_cfg.get("oat_raw_csv", "results/inje_2019_sensitivity_oat.csv")
        summary_csv_rel = output_cfg.get("oat_summary_csv", "results/inje_2019_sensitivity_oat_summary.csv")
        temporal_raw_rel = output_cfg.get("temporal_raw_csv", "results/inje_2019_sensitivity_temporal.csv")
        temporal_summary_rel = output_cfg.get("temporal_summary_csv", "results/inje_2019_temporal_summary.csv")

    raw_csv_path = (SCRIPT_DIR / raw_csv_rel).resolve()
    summary_csv_path = (SCRIPT_DIR / summary_csv_rel).resolve()
    temporal_raw_path = (SCRIPT_DIR / temporal_raw_rel).resolve()
    temporal_summary_path = (SCRIPT_DIR / temporal_summary_rel).resolve()

    git_hash = get_git_commit_hash()

    # Determine execution task list: List of (experiment_type, param_name, param_val, overrides_dict)
    tasks: List[Tuple[str, str, Any, Dict[str, Any]]] = []

    # 1. Baseline Task
    if mode in ("baseline", "all", "oat"):
        tasks.append(("baseline", "baseline", "baseline", {}))

    # 2. Stage A (OAT) Tasks
    if mode in ("oat", "all"):
        oat_cfg = sens_cfg.get("oat", {})
        for param_name, val_list in oat_cfg.items():
            for val in val_list:
                tasks.append(("oat", param_name, val, {param_name: val}))

    # 3. Percolation Refinement Tasks (burn_duration_steps=2, base_prob sweep)
    if mode == "refinement":
        refinement_probs = [0.25, 0.30, 0.325, 0.35, 0.375, 0.40]
        for bp in refinement_probs:
            overrides = {
                "base_spread_probability": bp,
                "burn_duration_steps": 2,
                "wind_factor_weight": 0.20,
                "slope_factor_weight": 0.05,
                "moisture_dampening_weight": 1.0,
            }
            tasks.append(("refinement", "base_spread_probability", bp, overrides))

    # 4. Stage B (Grid Search) Tasks
    if mode in ("grid", "all"):
        grid_cfg = sens_cfg.get("grid_search", {})
        base_probs = grid_cfg.get("base_spread_probability", [0.45, 0.50, 0.55, 0.60])
        wind_weights = grid_cfg.get("wind_factor_weight", [0.10, 0.15, 0.20, 0.25])
        slope_weights = grid_cfg.get("slope_factor_weight", [0.025, 0.05, 0.075])
        burn_durations = grid_cfg.get("burn_duration_steps", [3])
        for bp in base_probs:
            for ww in wind_weights:
                for sw in slope_weights:
                    for bd in burn_durations:
                        comb_label = f"bp{bp}_ww{ww}_sw{sw}_bd{bd}"
                        overrides = {
                            "base_spread_probability": bp,
                            "wind_factor_weight": ww,
                            "slope_factor_weight": sw,
                            "burn_duration_steps": bd,
                        }
                        tasks.append(("grid", "multi_param", comb_label, overrides))

    total_runs = len(tasks) * len(seeds)

    print("=" * 78)
    print(f"EXPERIMENT SUITE: {sens_cfg.get('metadata', {}).get('case_name', 'CA Sensitivity')}")
    print("=" * 78)
    print(f"Mode:                      {mode}")
    print(f"Environment Config:        {env_config_path}")
    print(f"Sensitivity Config:        {sens_config_path}")
    print(f"Reference Raster:          {reference_path}")
    print(f"Random Seeds ({len(seeds)}):          {seeds}")
    print(f"Parameter Combinations:    {len(tasks)}")
    print(f"Total Simulation Runs:     {len(tasks)} × {len(seeds)} = {total_runs}")
    print(f"Raw Output CSV:            {raw_csv_path}")
    print(f"Summary Output CSV:        {summary_csv_path}")
    print(f"Temporal Raw CSV:          {temporal_raw_path}")
    print(f"Temporal Summary CSV:      {temporal_summary_path}")
    print(f"Dry Run Mode:              {dry_run}")
    print("-" * 78)

    if dry_run:
        print("[DRY-RUN] Planned execution items:")
        for idx, (exp_type, p_name, p_val, ovr) in enumerate(tasks, start=1):
            print(f"  [{idx:02d}/{len(tasks):02d}] {exp_type.upper():<8} | {p_name:<25} = {str(p_val):<15} | overrides: {ovr}")
        print("=" * 78)
        print("[DRY-RUN] Completed without executing simulations.")
        return [], []

    completed_keys = set() if force else get_completed_run_keys(raw_csv_path)
    all_raw_results: List[Dict[str, Any]] = []

    # If resuming, load existing raw results from CSV
    if not force and raw_csv_path.is_file():
        with open(raw_csv_path, "r", encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                all_raw_results.append(row)

    run_idx = 0
    skipped_count = 0
    executed_count = 0

    for exp_type, p_name, p_val, ovr in tasks:
        # Determine effective parameter values for key generation
        eff_ca = dict(env_cfg.get("fire_ca", {}))
        eff_ca.update(ovr)
        eff_bp = float(eff_ca.get("base_spread_probability", 0.70))
        eff_ww = float(eff_ca.get("wind_factor_weight", 0.20))
        eff_sw = float(eff_ca.get("slope_factor_weight", 0.05))
        eff_bd = int(eff_ca.get("burn_duration_steps", 3))

        for seed in seeds:
            run_idx += 1
            key = make_run_key(exp_type, p_name, p_val, seed, eff_bp, eff_ww, eff_sw, eff_bd)

            if not force and key in completed_keys:
                skipped_count += 1
                logger.info(
                    "[%d/%d] Skipping already completed run: %s (seed=%d)",
                    run_idx,
                    total_runs,
                    p_val,
                    seed,
                )
                continue

            logger.info(
                "[%d/%d] Running %s (%s=%s, seed=%d) ...",
                run_idx,
                total_runs,
                exp_type,
                p_name,
                p_val,
                seed,
            )

            result_row, temporal_rows = run_single_simulation(
                env_config_base=env_cfg,
                env_config_path=env_config_path,
                overrides=ovr,
                seed=seed,
                reference_path=reference_path,
                experiment_type=exp_type,
                parameter_name=p_name,
                parameter_value=p_val,
                git_hash=git_hash,
                max_steps=max_steps,
            )

            append_raw_run_to_csv(raw_csv_path, result_row)
            append_temporal_rows_to_csv(temporal_raw_path, temporal_rows)
            all_raw_results.append(result_row)
            completed_keys.add(key)
            executed_count += 1

            logger.info(
                "    -> Final IoU: %.4f, Max IoU: %.4f (step=%s, time=%s, area=%.2f ha), Area Err: %.2f ha",
                float(result_row.get("iou", 0.0)),
                float(result_row.get("max_iou", 0.0)),
                result_row.get("step_at_max_iou", 0),
                result_row.get("timestamp_at_max_iou", ""),
                float(result_row.get("simulated_area_at_max_iou_ha", 0.0)),
                float(result_row.get("area_error_ha", 0.0)),
            )

    print("-" * 78)
    print(f"Executed: {executed_count} runs, Skipped (resumed): {skipped_count} runs, Total: {len(all_raw_results)} records")

    # Aggregate results across seeds
    summaries = compute_summary_records(all_raw_results)
    write_summary_csv(summary_csv_path, summaries, fieldnames=SUMMARY_CSV_HEADER)
    print(f"Summary metrics saved to: {summary_csv_path}")

    # Aggregate temporal peak diagnostics across seeds
    temporal_summaries = compute_temporal_summary_records(all_raw_results)
    write_summary_csv(temporal_summary_path, temporal_summaries, fieldnames=TEMPORAL_SUMMARY_CSV_HEADER)
    print(f"Temporal peak summary metrics saved to: {temporal_summary_path}")
    print(f"Step-by-step temporal trajectory saved to: {temporal_raw_path}")
    print("=" * 78)

    return all_raw_results, summaries


def main() -> None:
    """Parse command-line arguments and run experiment suite."""
    parser = argparse.ArgumentParser(
        description="Wildfire CA Parameter Sensitivity & Grid-Search Runner for 2019 Inje Case"
    )
    parser.add_argument(
        "--mode",
        choices=["baseline", "oat", "refinement", "grid", "all"],
        default="oat",
        help="Experiment mode: baseline, oat (Stage A), refinement, grid (Stage B), or all (default: oat)",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=SCRIPT_DIR / "config" / "inje_2019_sensitivity.yaml",
        help="Path to sensitivity experiment configuration YAML",
    )
    parser.add_argument(
        "--env-config",
        type=Path,
        default=SCRIPT_DIR / "config" / "environment_config.yaml",
        help="Path to baseline environment configuration YAML",
    )
    parser.add_argument(
        "--reference",
        type=Path,
        default=None,
        help="Path to 2019 burn reference binary GeoTIFF (auto-resolved if omitted)",
    )
    parser.add_argument(
        "--seeds",
        type=str,
        default=None,
        help="Comma-separated seed list to override config (e.g. '42,43,44')",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Display experiment execution plan without running simulations",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force re-running all simulations, overwriting / ignoring resume cache",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Optional safety ceiling on simulation steps per run (for testing)",
    )

    args = parser.parse_args()

    # Resolve reference path if not explicitly provided
    ref_path = (
        args.reference.resolve()
        if args.reference is not None
        else resolve_reference_raster_path(base_dir=SCRIPT_DIR)
    )

    seeds_override = None
    if args.seeds is not None:
        seeds_override = [int(s.strip()) for s in args.seeds.split(",") if s.strip()]

    run_experiment_suite(
        mode=args.mode,
        env_config_path=args.env_config.resolve(),
        sens_config_path=args.config.resolve(),
        reference_path=ref_path,
        seeds_override=seeds_override,
        dry_run=args.dry_run,
        force=args.force,
        max_steps=args.max_steps,
    )


if __name__ == "__main__":
    main()
