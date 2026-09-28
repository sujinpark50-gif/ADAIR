"""Benchmark & Model Selection Tooling for Wildfire Spread Prediction Modes.

Evaluates forecast horizon N and Monte Carlo ensemble size M:
- Performance (execution time ms across repetitions)
- Spatial coverage (non-zero S_spread cells and grid percentage)
- S_spread distribution (min, max, mean, median)
- Stability / Convergence (Mean Absolute Difference against high-M reference)
- Multi-mode comparison (geometric baseline vs. ensemble)
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Sequence, TYPE_CHECKING
import numpy as np

from src.environment.risk import calculate_risk_scores

if TYPE_CHECKING:
    from src.environment.fire_model import WildfireCAEngine


def compare_spread_modes(
    ca_engine: WildfireCAEngine,
    steps: int = 5,
    runs: int = 5,
) -> Dict[str, Any]:
    """Compare geometric baseline vs. ensemble spread predictions under current fire state.

    Parameters
    ----------
    ca_engine : WildfireCAEngine
        The simulation engine instance with active fire states.
    steps : int, default=5
        Forecast horizon steps for ensemble mode.
    runs : int, default=5
        Number of stochastic Monte Carlo trajectories for ensemble mode.

    Returns
    -------
    Dict[str, Any]
        Detailed benchmark and statistical comparison report.
    """
    grid = ca_engine.grid
    config = ca_engine._config

    # --- Benchmark Geometric Mode ---
    t0_geo = time.perf_counter()
    geo_spread = ca_engine.predict_spread_geometric()
    t1_geo = time.perf_counter()
    geo_time_ms = (t1_geo - t0_geo) * 1000.0

    geo_risk = calculate_risk_scores(grid, config, spread_scores=geo_spread)

    # --- Benchmark Ensemble Mode ---
    t0_ens = time.perf_counter()
    ens_spread = ca_engine.predict_spread_ensemble(steps=steps, runs=runs)
    t1_ens = time.perf_counter()
    ens_time_ms = (t1_ens - t0_ens) * 1000.0

    ens_risk = calculate_risk_scores(grid, config, spread_scores=ens_spread)

    # Statistical summaries
    geo_spread_vals = np.array(list(geo_spread.values()))
    ens_spread_vals = np.array(list(ens_spread.values()))

    geo_risk_vals = np.array(list(geo_risk.values()))
    ens_risk_vals = np.array(list(ens_risk.values()))

    report = {
        "parameters": {
            "total_grid_cells": len(grid.cells),
            "ensemble_steps": steps,
            "ensemble_runs": runs,
            "active_burning_cells": sum(1 for c in grid.cells if c.fire_state.value == "BURNING"),
        },
        "geometric": {
            "time_ms": round(geo_time_ms, 2),
            "non_zero_spread_cells": int(np.count_nonzero(geo_spread_vals > 0.0)),
            "s_spread_min": float(np.min(geo_spread_vals)),
            "s_spread_max": float(np.max(geo_spread_vals)),
            "s_spread_mean": float(np.mean(geo_spread_vals)),
            "risk_score_mean": float(np.mean(geo_risk_vals)),
            "risk_score_max": float(np.max(geo_risk_vals)),
        },
        "ensemble": {
            "time_ms": round(ens_time_ms, 2),
            "non_zero_spread_cells": int(np.count_nonzero(ens_spread_vals > 0.0)),
            "s_spread_min": float(np.min(ens_spread_vals)),
            "s_spread_max": float(np.max(ens_spread_vals)),
            "s_spread_mean": float(np.mean(ens_spread_vals)),
            "risk_score_mean": float(np.mean(ens_risk_vals)),
            "risk_score_max": float(np.max(ens_risk_vals)),
        },
    }
    return report


def benchmark_ensemble_grid(
    ca_engine: WildfireCAEngine,
    n_values: Sequence[int] = (3, 5, 10, 15),
    m_values: Sequence[int] = (5, 10, 20),
    repetitions: int = 3,
    ref_runs: int = 50,
) -> List[Dict[str, Any]]:
    """Empirically evaluate multiple (N, M) parameter combinations.

    Measures execution time, spatial envelope coverage, probability distribution,
    and Mean Absolute Difference (MAD) stability against a high-M reference.

    Parameters
    ----------
    ca_engine : WildfireCAEngine
        The active simulation engine instance.
    n_values : Sequence[int]
        Candidate forecast horizon values (steps forward).
    m_values : Sequence[int]
        Candidate Monte Carlo ensemble sizes (number of trajectories).
    repetitions : int, default=3
        Number of timing repetitions per configuration for stable latency estimation.
    ref_runs : int, default=50
        Reference ensemble size for evaluating stability/convergence.

    Returns
    -------
    List[Dict[str, Any]]
        List of benchmark records for each (N, M) configuration.
    """
    total_cells = len(ca_engine.grid.cells)
    results: List[Dict[str, Any]] = []

    for n in n_values:
        # Precompute high-M reference map for this horizon N
        ref_spread = ca_engine.predict_spread_ensemble(steps=n, runs=ref_runs, seed=99999)
        ref_arr = np.array([ref_spread[c.cell_id] for c in ca_engine.grid.cells])

        for m in m_values:
            # Measure execution timing across repetitions
            times_ms: List[float] = []
            test_spread: Optional[Dict[int, float]] = None

            for r in range(repetitions):
                t0 = time.perf_counter()
                test_spread = ca_engine.predict_spread_ensemble(steps=n, runs=m, seed=1000 + r)
                t1 = time.perf_counter()
                times_ms.append((t1 - t0) * 1000.0)

            avg_time = float(np.mean(times_ms))
            min_time = float(np.min(times_ms))

            assert test_spread is not None
            test_arr = np.array([test_spread[c.cell_id] for c in ca_engine.grid.cells])

            # Spatial coverage
            nonzero_count = int(np.count_nonzero(test_arr > 0.0))
            nonzero_pct = (nonzero_count / total_cells) * 100.0

            # Distribution statistics
            s_min = float(np.min(test_arr))
            s_max = float(np.max(test_arr))
            s_mean = float(np.mean(test_arr))
            nonzero_vals = test_arr[test_arr > 0.0]
            s_median_nonzero = float(np.median(nonzero_vals)) if len(nonzero_vals) > 0 else 0.0

            # Stability / Convergence: Mean Absolute Difference against ref_runs
            # Focus on affected zone (cells where test > 0 or ref > 0)
            affected_mask = (test_arr > 0.0) | (ref_arr > 0.0)
            if np.any(affected_mask):
                mad_affected = float(np.mean(np.abs(test_arr[affected_mask] - ref_arr[affected_mask])))
            else:
                mad_affected = 0.0

            mad_global = float(np.mean(np.abs(test_arr - ref_arr)))

            record = {
                "N": n,
                "M": m,
                "avg_time_ms": round(avg_time, 2),
                "min_time_ms": round(min_time, 2),
                "nonzero_cells": nonzero_count,
                "nonzero_percent": round(nonzero_pct, 4),
                "s_spread_max": round(s_max, 4),
                "s_spread_mean": round(s_mean, 6),
                "s_spread_median_nonzero": round(s_median_nonzero, 4),
                "mad_affected_vs_ref": round(mad_affected, 4),
                "mad_global_vs_ref": round(mad_global, 8),
            }
            results.append(record)

    return results


def format_benchmark_table(results: List[Dict[str, Any]]) -> str:
    """Format benchmark records into a clean Markdown table."""
    headers = [
        "N (Horizon)",
        "M (Runs)",
        "Avg Time (ms)",
        "Non-zero Cells",
        "Grid Cover %",
        "Max S_spread",
        "Median Non-zero",
        "MAD vs Ref (Affected)",
    ]
    rows = []
    rows.append("| " + " | ".join(headers) + " |")
    rows.append("| " + " | ".join(["---"] * len(headers)) + " |")

    for r in results:
        row = [
            str(r["N"]),
            str(r["M"]),
            f"{r['avg_time_ms']:.1f}",
            str(r["nonzero_cells"]),
            f"{r['nonzero_percent']:.3f}%",
            f"{r['s_spread_max']:.2f}",
            f"{r['s_spread_median_nonzero']:.2f}",
            f"{r['mad_affected_vs_ref']:.4f}",
        ]
        rows.append("| " + " | ".join(row) + " |")

    return "\n".join(rows)


def _cli() -> None:
    """Command-line interface entry point for spread prediction benchmarking."""
    import argparse
    from src.environment.grid import EnvironmentGrid
    from src.environment.fire_model import WildfireCAEngine

    parser = argparse.ArgumentParser(
        description="Benchmark Wildfire Spread Prediction Modes (Geometric vs Ensemble N/M Grid)"
    )
    parser.add_argument(
        "-n",
        "--n-values",
        nargs="+",
        type=int,
        default=[3, 5, 10, 15],
        help="List of forecast horizon steps N to evaluate (e.g. -n 5 10 15 or -n 10)",
    )
    parser.add_argument(
        "-m",
        "--m-values",
        nargs="+",
        type=int,
        default=[5, 10, 20],
        help="List of Monte Carlo ensemble sizes M to evaluate (e.g. -m 5 10 20 or -m 10)",
    )
    parser.add_argument(
        "--x",
        type=int,
        default=120,
        help="Ignition cell X coordinate (default: 120)",
    )
    parser.add_argument(
        "--y",
        type=int,
        default=120,
        help="Ignition cell Y coordinate (default: 120)",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=3,
        help="Number of timing repetitions per (N, M) configuration (default: 3)",
    )
    parser.add_argument(
        "--ref-runs",
        type=int,
        default=50,
        help="Reference ensemble size M for MAD convergence calculation (default: 50)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for simulation (default: 42)",
    )

    args = parser.parse_args()

    print("[INFO] Initializing EnvironmentGrid and WildfireCAEngine...")
    grid = EnvironmentGrid(seed=args.seed)
    ca_engine = WildfireCAEngine(grid=grid, seed=args.seed)

    # Ignite specified cell
    ignited_cell = ca_engine.ignite_at(args.x, args.y)
    print(f"[INFO] Ignited cell ID {ignited_cell.cell_id} at grid ({args.x}, {args.y}).")
    print(f"[INFO] Running benchmark across N={args.n_values} x M={args.m_values}...\n")

    results = benchmark_ensemble_grid(
        ca_engine=ca_engine,
        n_values=args.n_values,
        m_values=args.m_values,
        repetitions=args.repetitions,
        ref_runs=args.ref_runs,
    )

    table = format_benchmark_table(results)
    print("### Benchmark Results Table")
    print(table)


if __name__ == "__main__":
    _cli()

