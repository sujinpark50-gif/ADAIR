"""Analytic Hierarchy Process (AHP) Weight Derivation Module.

Calculates priority weights for top-level wildfire risk scoring criteria:
- spread (W_spread)
- human (W_human)
- infrastructure (W_infra)
- property (W_property)
- secondary_hazard (W_secondary)

Accepts an already-aggregated pairwise comparison matrix (5x5) and computes:
1. Principal Eigenvector & Normalized Weights
2. Maximum Eigenvalue (lambda_max)
3. Consistency Index (CI)
4. Consistency Ratio (CR) with Saaty Random Index (RI = 1.12 for n=5)
5. Consistency verification against threshold (CR < 0.10)

Authority & Specification Compliance:
------------------------------------
- Authority: `mission/environmental-modeling.txt`
- Fixed criteria order: spread, human, infrastructure, property, secondary_hazard
- Non-invasive: Computes weights without modifying environment_config.yaml automatically.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Union
import numpy as np

# Fixed criteria ordering for top-level composite risk scoring
CRITERIA: List[str] = [
    "spread",
    "human",
    "infrastructure",
    "property",
    "secondary_hazard",
]

# Standard Saaty Random Index (RI) lookup table for matrix sizes 1..10
RANDOM_INDEX_TABLE: Dict[int, float] = {
    1: 0.00,
    2: 0.00,
    3: 0.58,
    4: 0.90,
    5: 1.12,
    6: 1.24,
    7: 1.32,
    8: 1.41,
    9: 1.45,
    10: 1.49,
}


def validate_ahp_matrix(
    matrix: np.ndarray,
    expected_size: int = 5,
    tolerance: float = 1e-3,
) -> None:
    """Validate properties of an AHP pairwise comparison matrix.

    Checks:
    1. 2D square matrix of expected shape (expected_size, expected_size).
    2. All entries are finite and strictly positive (> 0).
    3. Main diagonal entries are all approximately 1.0.
    4. Reciprocal property holds: A[i, j] * A[j, i] ≈ 1.0.

    Parameters
    ----------
    matrix : np.ndarray
        Pairwise comparison matrix to validate.
    expected_size : int, default=5
        Expected number of rows and columns.
    tolerance : float, default=1e-3
        Tolerance for checking diagonal and reciprocal constraints.

    Raises
    ------
    ValueError
        If any validation check fails.
    """
    if not isinstance(matrix, np.ndarray):
        matrix = np.asarray(matrix, dtype=float)

    if matrix.ndim != 2:
        raise ValueError(
            f"AHP matrix must be 2-dimensional, but got {matrix.ndim}D array."
        )

    if matrix.shape != (expected_size, expected_size):
        raise ValueError(
            f"AHP matrix shape must be ({expected_size}, {expected_size}), "
            f"but got {matrix.shape}."
        )

    if not np.all(np.isfinite(matrix)):
        raise ValueError("AHP matrix contains non-finite values (NaN or Inf).")

    if not np.all(matrix > 0):
        raise ValueError("All entries in the AHP matrix must be strictly positive (> 0).")

    # Diagonal check: A[i, i] == 1.0
    diag = np.diag(matrix)
    if not np.allclose(diag, 1.0, atol=tolerance):
        raise ValueError(
            f"All diagonal entries of AHP matrix must be approximately 1.0, "
            f"got diagonal: {diag.tolist()}."
        )

    # Reciprocal check: A[i, j] * A[j, i] ≈ 1.0
    reciprocal_product = matrix * matrix.T
    if not np.allclose(reciprocal_product, 1.0, atol=tolerance):
        max_diff = float(np.max(np.abs(reciprocal_product - 1.0)))
        raise ValueError(
            f"AHP matrix violates reciprocal property (A[i,j] * A[j,i] ≈ 1). "
            f"Max reciprocal deviation: {max_diff:.6f} > tolerance ({tolerance})."
        )


def calculate_ahp_weights(
    group_matrix: Union[np.ndarray, Sequence[Sequence[float]]],
    criteria: Optional[Sequence[str]] = None,
    consistency_threshold: float = 0.10,
    tolerance: float = 1e-3,
) -> Dict[str, Any]:
    """Calculate normalized criteria weights and consistency metrics using the Principal Eigenvector Method.

    Parameters
    ----------
    group_matrix : np.ndarray | Sequence[Sequence[float]]
        The 5x5 aggregated pairwise comparison group matrix.
    criteria : Sequence[str], optional
        List of criteria names in matrix row/column order.
        Defaults to `CRITERIA` (spread, human, infrastructure, property, secondary_hazard).
    consistency_threshold : float, default=0.10
        Maximum acceptable Consistency Ratio (CR) for matrix consistency.
    tolerance : float, default=1e-3
        Floating-point tolerance for matrix validation.

    Returns
    -------
    Dict[str, Any]
        Dictionary containing:
        - "weights": Dict[str, float] mapping criteria to normalized weights (sum=1.0)
        - "lambda_max": float (principal eigenvalue)
        - "CI": float (Consistency Index)
        - "RI": float (Saaty Random Index)
        - "CR": float (Consistency Ratio)
        - "consistent": bool (True if CR < consistency_threshold)
        - "criteria": List[str]

    Raises
    ------
    ValueError
        If the input matrix fails AHP validation.
    """
    matrix = np.asarray(group_matrix, dtype=float)
    active_criteria = list(criteria) if criteria is not None else list(CRITERIA)
    n = len(active_criteria)

    # 1. Validate matrix
    validate_ahp_matrix(matrix, expected_size=n, tolerance=tolerance)

    # 2. Principal Eigenvector Method
    eigenvalues, eigenvectors = np.linalg.eig(matrix)

    # Find eigenvalue with the largest real part
    max_index = int(np.argmax(eigenvalues.real))
    lambda_max = float(eigenvalues[max_index].real)

    # Extract corresponding principal eigenvector
    raw_weights = eigenvectors[:, max_index].real

    # Ensure eigenvector orientation is positive
    if np.sum(raw_weights) < 0:
        raw_weights = -raw_weights

    # Normalize weights so sum(w) = 1.0
    weight_sum = np.sum(raw_weights)
    if weight_sum <= 0:
        raise ValueError("Sum of principal eigenvector elements is non-positive.")

    normalized_weights = raw_weights / weight_sum

    # Map weights to criteria dictionary in fixed order
    weights_dict: Dict[str, float] = {
        criterion: float(normalized_weights[idx])
        for idx, criterion in enumerate(active_criteria)
    }

    # 3. Calculate Consistency Index (CI) and Consistency Ratio (CR)
    if n <= 1:
        ci = 0.0
        ri = 0.0
        cr = 0.0
    elif n == 2:
        ci = (lambda_max - n) / (n - 1)
        ri = 0.0
        cr = 0.0
    else:
        ci = (lambda_max - n) / (n - 1)
        # Numerical safeguard against small negative CI due to floating-point imprecision
        if ci < 0.0 and abs(ci) < 1e-10:
            ci = 0.0

        ri = RANDOM_INDEX_TABLE.get(n, 1.12)
        cr = ci / ri if ri > 0 else 0.0

    is_consistent = bool(cr < consistency_threshold)

    return {
        "weights": weights_dict,
        "lambda_max": lambda_max,
        "CI": ci,
        "RI": ri,
        "CR": cr,
        "consistent": is_consistent,
        "consistency_threshold": consistency_threshold,
        "criteria": active_criteria,
    }


def format_ahp_report(ahp_result: Dict[str, Any]) -> str:
    """Format AHP calculation results into a human-readable text summary."""
    weights = ahp_result["weights"]
    lambda_max = ahp_result["lambda_max"]
    ci = ahp_result["CI"]
    ri = ahp_result["RI"]
    cr = ahp_result["CR"]
    consistent = ahp_result["consistent"]
    thresh = ahp_result.get("consistency_threshold", 0.10)

    label_map = {
        "spread": "Spread",
        "human": "Human",
        "infrastructure": "Infrastructure",
        "property": "Property",
        "secondary_hazard": "Secondary Hazard",
    }

    lines = [
        "AHP Results",
        "-----------",
        "",
    ]

    for crit, w in weights.items():
        label = label_map.get(crit, crit.replace("_", " ").title())
        lines.append(f"{label:<19}: {w:.4f}")

    lines.extend([
        "",
        f"Lambda max         : {lambda_max:.4f}",
        f"CI                 : {ci:.4f}",
        f"RI                 : {ri:.2f}",
        f"CR                 : {cr:.4f}",
        f"Consistent         : {consistent} (threshold < {thresh:.2f})",
    ])

    return "\n".join(lines)


def format_ahp_yaml(ahp_result: Dict[str, Any]) -> str:
    """Format AHP weights into YAML-ready composite_weights configuration snippet."""
    weights = ahp_result["weights"]
    lines = [
        "risk_scoring:",
        "  composite_weights:",
    ]
    for crit, w in weights.items():
        lines.append(f"    {crit}: {w:.4f}")

    return "\n".join(lines)


def _cli() -> None:
    """Command-line interface entry point for AHP weight calculation."""
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(
        description="Calculate AHP criteria weights and consistency metrics for 5x5 pairwise matrix."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--matrix",
        type=str,
        help='JSON string of 5x5 matrix, e.g. "[[1, 0.111, 0.2, 0.333, 0.5], [9, 1, 7, 8, 7], ...]"',
    )
    group.add_argument(
        "--file",
        type=str,
        help="Path to file containing 5x5 matrix (.json, .csv, or whitespace-delimited .txt)",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.10,
        help="Consistency Ratio threshold (default: 0.10)",
    )
    parser.add_argument(
        "--yaml-only",
        action="store_true",
        help="Print only the YAML snippet for environment_config.yaml",
    )
    parser.add_argument(
        "--json-only",
        action="store_true",
        help="Print only JSON formatted results",
    )

    args = parser.parse_args()

    # Determine input matrix
    if args.matrix:
        try:
            mat = np.array(json.loads(args.matrix), dtype=float)
        except Exception as err:
            parser.error(f"Failed to parse --matrix JSON: {err}")
    elif args.file:
        file_path = Path(args.file)
        if not file_path.exists():
            parser.error(f"Matrix file not found: {file_path}")
        if file_path.suffix.lower() == ".json":
            with open(file_path, "r", encoding="utf-8") as fh:
                mat = np.array(json.load(fh), dtype=float)
        elif file_path.suffix.lower() == ".csv":
            mat = np.loadtxt(file_path, delimiter=",", dtype=float)
        else:
            mat = np.loadtxt(file_path, dtype=float)
    else:
        # Default example matrix from spec
        print("[INFO] No matrix input provided. Running with example 5x5 matrix from specification:\n")
        mat = np.array([
            [1.0,   1.0 / 9.0, 1.0 / 5.0, 1.0 / 3.0, 1.0 / 2.0],
            [9.0,   1.0,       7.0,       8.0,       7.0      ],
            [5.0,   1.0 / 7.0, 1.0,       3.0,       4.0      ],
            [3.0,   1.0 / 8.0, 1.0 / 3.0, 1.0,       2.0      ],
            [2.0,   1.0 / 7.0, 1.0 / 4.0, 1.0 / 2.0, 1.0      ],
        ], dtype=float)

    try:
        result = calculate_ahp_weights(mat, consistency_threshold=args.threshold)
    except Exception as exc:
        print(f"\n[ERROR] AHP Calculation Failed: {exc}\n")
        raise SystemExit(1)

    if args.yaml_only:
        print(format_ahp_yaml(result))
    elif args.json_only:
        print(json.dumps(result, indent=2))
    else:
        print(format_ahp_report(result))
        print("\n" + "=" * 40 + "\n")
        print("YAML snippet (copy-pasteable into config/environment_config.yaml):")
        print(format_ahp_yaml(result))


if __name__ == "__main__":
    _cli()

