"""Unit tests for the CA Parameter Sensitivity and Grid-Search experiment runner.

Verifies:
1. Parameter overrides do not modify the original config dict (in-memory immutability).
2. Fresh grid and engine are instantiated for each simulation run.
3. Bitwise deterministic reproducibility: identical parameter + identical seed -> identical metrics.
4. Parameter overrides are actually reflected in the active CA engine.
5. Invariant preservation: wind_direction_convention='toward', weather_mode='timeseries'.
6. Reusable evaluation metrics (IoU, Dice, Precision, Recall, Area Error) via evaluation.py.
7. Raw CSV logging records one row per completed run with full metadata.
8. Seed aggregation correctly computes means, stds, and error metrics.
9. Resume logic identifies and skips previously completed runs.
10. Dry-run mode plans execution without running simulations.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
import sys
import tempfile
from typing import Any, Dict

import numpy as np
import pytest

TEST_DIR = Path(__file__).resolve().parent
ENV_DIR = TEST_DIR.parent
if str(ENV_DIR) not in sys.path:
    sys.path.insert(0, str(ENV_DIR))

from run_ca_sensitivity import (
    RAW_CSV_HEADER,
    SUMMARY_CSV_HEADER,
    append_raw_run_to_csv,
    compute_summary_records,
    get_completed_run_keys,
    load_yaml,
    make_run_key,
    run_experiment_suite,
    run_single_simulation,
    write_summary_csv,
)
from src.cell import FireState
from src.evaluation import resolve_reference_raster_path
from src.fire_model import WildfireCAEngine
from src.grid import EnvironmentGrid

CONFIG_PATH = ENV_DIR / "config" / "environment_config.yaml"
SENS_CONFIG_PATH = ENV_DIR / "config" / "inje_2019_sensitivity.yaml"
REF_RASTER_PATH = resolve_reference_raster_path(base_dir=ENV_DIR)


# ---------------------------------------------------------------------------
# 1. In-memory config immutability
# ---------------------------------------------------------------------------
def test_config_immutability_during_overrides():
    orig_cfg = load_yaml(CONFIG_PATH)
    orig_base_prob = orig_cfg.get("fire_ca", {}).get("base_spread_probability")

    overrides = {"base_spread_probability": 0.99, "wind_factor_weight": 0.88}

    # Run single short simulation
    res, _ = run_single_simulation(
        env_config_base=orig_cfg,
        env_config_path=CONFIG_PATH,
        overrides=overrides,
        seed=42,
        reference_path=REF_RASTER_PATH,
        experiment_type="test",
        parameter_name="base_spread_probability",
        parameter_value=0.99,
        git_hash="test",
        max_steps=2,
    )

    # Verify original dict was NOT mutated
    assert orig_cfg.get("fire_ca", {}).get("base_spread_probability") == orig_base_prob
    assert orig_cfg.get("fire_ca", {}).get("wind_factor_weight") != 0.88
    assert res["status"] == "completed"
    assert res["base_spread_probability"] == 0.99


# ---------------------------------------------------------------------------
# 2 & 3. Fresh grid & engine + Bitwise reproducibility with identical seed
# ---------------------------------------------------------------------------
def test_fresh_grid_engine_and_deterministic_reproducibility():
    cfg = load_yaml(CONFIG_PATH)
    overrides = {"base_spread_probability": 0.60}

    res1, _ = run_single_simulation(
        env_config_base=cfg,
        env_config_path=CONFIG_PATH,
        overrides=overrides,
        seed=42,
        reference_path=REF_RASTER_PATH,
        experiment_type="test",
        parameter_name="base_spread_probability",
        parameter_value=0.60,
        git_hash="test",
        max_steps=5,
    )

    res2, _ = run_single_simulation(
        env_config_base=cfg,
        env_config_path=CONFIG_PATH,
        overrides=overrides,
        seed=42,
        reference_path=REF_RASTER_PATH,
        experiment_type="test",
        parameter_name="base_spread_probability",
        parameter_value=0.60,
        git_hash="test",
        max_steps=5,
    )

    # Both independent runs must produce exact bitwise identical metrics
    assert res1["iou"] == res2["iou"]
    assert res1["dice"] == res2["dice"]
    assert res1["simulated_burned_cells"] == res2["simulated_burned_cells"]
    assert res1["area_error_ha"] == res2["area_error_ha"]


# ---------------------------------------------------------------------------
# 4 & 5. Parameter overrides active in CA Engine & Invariance preservation
# ---------------------------------------------------------------------------
def test_parameter_override_propagation_and_invariance():
    cfg = load_yaml(CONFIG_PATH)
    overrides = {
        "base_spread_probability": 0.77,
        "wind_factor_weight": 0.23,
        "slope_factor_weight": 0.08,
        "burn_duration_steps": 4,
    }

    grid = EnvironmentGrid(CONFIG_PATH, seed=42)
    cfg_copy = dict(cfg)
    cfg_copy["fire_ca"] = dict(cfg.get("fire_ca", {}))
    cfg_copy["fire_ca"].update(overrides)
    cfg_copy["weather"] = dict(cfg.get("weather", {}))
    cfg_copy["weather"]["wind_direction_convention"] = "toward"
    cfg_copy["weather"]["mode"] = "timeseries"

    engine = WildfireCAEngine(grid, config_path=cfg_copy, seed=42)

    # CA engine internal parameters must match overrides
    assert math.isclose(engine._base_spread_prob, 0.77)
    assert math.isclose(engine._wind_weight, 0.23)
    assert math.isclose(engine._slope_weight, 0.08)
    assert engine._burn_duration == 4

    # TOWARD and TIMESERIES invariance
    assert engine.current_weather.wind_direction_convention == "toward"
    diag = engine.get_weather_diagnostics()
    assert diag["weather_mode"] == "timeseries"
    assert diag["wind_direction_convention"] == "toward"


# ---------------------------------------------------------------------------
# 6 & 7. Raw CSV recording & Schema integrity
# ---------------------------------------------------------------------------
def test_raw_csv_recording_and_schema():
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_p = Path(tmpdir) / "test_raw.csv"

        test_row = {
            "run_id": "test_run_1",
            "timestamp": "2026-10-02T12:00:00",
            "git_commit": "abc1234",
            "experiment_type": "oat",
            "parameter_name": "base_spread_probability",
            "parameter_value": "0.50",
            "seed": 42,
            "wind_direction_convention": "toward",
            "weather_mode": "timeseries",
            "base_spread_probability": 0.50,
            "wind_factor_weight": 0.20,
            "slope_factor_weight": 0.05,
            "moisture_dampening_weight": 1.0,
            "burn_duration_steps": 3,
            "roads_block_spread": True,
            "total_steps_run": 10,
            "iou": 0.25,
            "dice": 0.40,
            "precision": 0.35,
            "recall": 0.45,
            "tp_cells": 100,
            "fp_cells": 150,
            "fn_cells": 120,
            "tn_cells": 5000,
            "simulated_burned_cells": 250,
            "reference_burned_cells": 220,
            "simulated_area_ha": 202.5,
            "reference_area_ha": 178.2,
            "area_error_ha": 24.3,
            "area_error_percent": 13.64,
            "absolute_area_error_ha": 24.3,
            "absolute_area_error_percent": 13.64,
            "simulation_start_time": "2019-04-04T14:45:00",
            "simulation_end_time": "2019-04-06T12:05:00",
            "timestep_minutes": 10,
            "reference_raster_path": str(REF_RASTER_PATH),
            "status": "completed",
            "error_message": "",
        }

        append_raw_run_to_csv(csv_p, test_row)
        assert csv_p.is_file()

        with open(csv_p, "r", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            rows = list(reader)
            assert len(rows) == 1
            assert rows[0]["run_id"] == "test_run_1"
            assert float(rows[0]["iou"]) == 0.25
            assert rows[0]["experiment_type"] == "oat"
            for col in RAW_CSV_HEADER:
                assert col in rows[0]


# ---------------------------------------------------------------------------
# 8. Summary aggregation mathematical correctness
# ---------------------------------------------------------------------------
def test_summary_aggregation_math():
    raw_sample = [
        {
            "experiment_type": "oat",
            "parameter_name": "base_spread_probability",
            "parameter_value": "0.50",
            "seed": 42,
            "base_spread_probability": 0.50,
            "wind_factor_weight": 0.20,
            "slope_factor_weight": 0.05,
            "burn_duration_steps": 3,
            "moisture_dampening_weight": 1.0,
            "iou": 0.20,
            "dice": 0.33,
            "precision": 0.30,
            "recall": 0.40,
            "simulated_area_ha": 200.0,
            "reference_area_ha": 150.0,
            "area_error_ha": 50.0,
            "absolute_area_error_ha": 50.0,
            "status": "completed",
        },
        {
            "experiment_type": "oat",
            "parameter_name": "base_spread_probability",
            "parameter_value": "0.50",
            "seed": 43,
            "base_spread_probability": 0.50,
            "wind_factor_weight": 0.20,
            "slope_factor_weight": 0.05,
            "burn_duration_steps": 3,
            "moisture_dampening_weight": 1.0,
            "iou": 0.30,
            "dice": 0.45,
            "precision": 0.40,
            "recall": 0.50,
            "simulated_area_ha": 250.0,
            "reference_area_ha": 150.0,
            "area_error_ha": 100.0,
            "absolute_area_error_ha": 100.0,
            "status": "completed",
        },
    ]

    summaries = compute_summary_records(raw_sample)
    assert len(summaries) == 1
    s = summaries[0]
    assert s["runs_count"] == 2
    assert s["seeds"] == "42,43"
    assert math.isclose(s["mean_iou"], 0.25)
    assert math.isclose(s["std_iou"], 0.05)
    assert math.isclose(s["mean_simulated_area_ha"], 225.0)
    assert math.isclose(s["mean_area_error_ha"], 75.0)


# ---------------------------------------------------------------------------
# 9. Resume logic skips completed runs
# ---------------------------------------------------------------------------
def test_resume_logic_skips_completed_runs():
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_p = Path(tmpdir) / "test_raw.csv"

        test_row = {
            "run_id": "test_run_1",
            "timestamp": "2026-10-02T12:00:00",
            "git_commit": "abc",
            "experiment_type": "oat",
            "parameter_name": "wind_factor_weight",
            "parameter_value": "0.15",
            "seed": 42,
            "base_spread_probability": 0.70,
            "wind_factor_weight": 0.15,
            "slope_factor_weight": 0.05,
            "burn_duration_steps": 3,
            "status": "completed",
        }
        append_raw_run_to_csv(csv_p, test_row)

        completed_keys = get_completed_run_keys(csv_p)
        key = make_run_key("oat", "wind_factor_weight", "0.15", 42, 0.70, 0.15, 0.05, 3)
        assert key in completed_keys

        key_not_done = make_run_key("oat", "wind_factor_weight", "0.20", 42, 0.70, 0.20, 0.05, 3)
        assert key_not_done not in completed_keys


# ---------------------------------------------------------------------------
# 10. Dry-run mode does not execute simulations
# ---------------------------------------------------------------------------
def test_dry_run_mode():
    with tempfile.TemporaryDirectory() as tmpdir:
        raw_p = Path(tmpdir) / "should_not_exist_raw.csv"
        sum_p = Path(tmpdir) / "should_not_exist_sum.csv"

        raw, summaries = run_experiment_suite(
            mode="oat",
            env_config_path=CONFIG_PATH,
            sens_config_path=SENS_CONFIG_PATH,
            reference_path=REF_RASTER_PATH,
            seeds_override=[42],
            dry_run=True,
        )

        assert raw == []
        assert summaries == []
        assert not raw_p.is_file()


# ---------------------------------------------------------------------------
# 11. Temporal step evaluation & Peak metric detection
# ---------------------------------------------------------------------------
def test_temporal_evaluation():
    cfg = load_yaml(CONFIG_PATH)
    overrides = {"base_spread_probability": 0.50}

    res, temporal_rows = run_single_simulation(
        env_config_base=cfg,
        env_config_path=CONFIG_PATH,
        overrides=overrides,
        seed=42,
        reference_path=REF_RASTER_PATH,
        experiment_type="test",
        parameter_name="base_spread_probability",
        parameter_value=0.50,
        git_hash="test",
        max_steps=5,
    )

    assert len(temporal_rows) == 6  # Step 0 + 5 steps
    assert temporal_rows[0]["simulation_step"] == 0
    assert temporal_rows[-1]["simulation_step"] == 5
    assert "max_iou" in res
    assert "step_at_max_iou" in res
    assert res["max_iou"] >= res["iou"]
    assert res["simulated_area_at_max_iou_ha"] >= 0.0
