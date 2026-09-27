"""Risk scoring engine for grid cells in the wildfire simulation environment.

UNRESOLVED MODELING DECISION & SPECIFICATION COMPLIANCE NOTICE:
----------------------------------------------------------------
Per `mission/environmental-modeling.txt` §3.3 (line 72):
  "risk_score의 계산 기준과 값의 범위는 총괄·통합 담당과 합의한다."
  ("The calculation criteria and value range of risk_score are
  agreed with the Master Orchestrator and System Integration Leads.")

Because final formal agreement across subsystems has not yet been established,
the calculation in this module serves as a **provisional scenario modeling assumption**.
It is implemented in a modular, configuration-driven manner so that when the final
agreed-upon formulas, weights, and bounds are specified, they can directly replace the contents
of this function without altering the Phase 1 EnvironmentGrid or Phase 2 CA Engine.

All parameters used in this calculation are loaded from `config/environment_config.yaml`
under the `risk_scoring` key — zero scientific constants are hardcoded here.

ARCHITECTURE: PURE MULTI-CRITERIA RISK AGGREGATION (WLC)
---------------------------------------------------------
This module performs pure Multi-Criteria Weighted Linear Combination (WLC) aggregation.
It does NOT compute fire propagation physics, wind vectors, slope gradients, or CA simulations;
spread hazard scores (S_spread ∈ [0, 1]) are computed externally by the fire spread prediction layer
(e.g., WildfireCAEngine.predict_spread_geometric or predict_spread_ensemble).

Top-Level Composite Risk Score (risk_score ∈ [0, 1] for UNBURNED cells):
    risk_score = W_spread * S_spread
               + W_human * S_human
               + W_infra * S_infra
               + W_property * S_property
               + W_secondary * S_secondary

where:
    S_spread    = Dynamic fire spread hazard score from spread prediction layer ∈ [0, 1]
    S_human     = Residential building proxy (building_type=1) ∈ [0, 1]
    S_infra     = Important/Critical facility proxy (building_type=2, 3) ∈ [0, 1]
    S_property  = Commercial/Industrial property proxy (building_type=5, 6) ∈ [0, 1]
    S_secondary = Hazardous material facility proxy (building_type=4) ∈ [0, 1]
    Σ W_i = 1.0  (Normalized at runtime from relative composite_weights in config)

METHODOLOGY & FUTURE CALIBRATION HOOKS:
---------------------------------------
- Fire Hazard vs. Exposure Distinction:
  Environmental conditions determine fire behavior (S_spread). Building and facility
  layers determine exposure and consequence (S_human, S_infra, S_property, S_secondary).
  These dimensions are conceptually distinct and should NOT all be inferred from
  a single wildfire occurrence dataset.
- Provisional Status of Current Weights:
  The current weights are provisional scenario/policy parameters, NOT empirically
  calibrated constants.
- Future Calibration Pathways:
  * Exposure Weights (W_human, W_infra, W_property, W_secondary):
    Can be calibrated via historical damage/loss records, expert elicitation, Multi-Criteria
    Decision Analysis (AHP / MCDA), or policy-mandated weighting schemes.
- Operational Priority vs. Numerical Risk Score:
  Operational SOP rules (e.g. human life protection as highest operational priority)
  are governed by the Master Orchestrator during task allocation and scheduling.
  The numerical risk model provides objective spatial risk estimation.

See also: docs/common/ADAIR_공통데이터규약_위험우선순위_합의안.md §3
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple, TYPE_CHECKING

from src.environment.cell import Cell, FireState

if TYPE_CHECKING:
    from src.environment.grid import EnvironmentGrid


def _compute_building_risk_components(
    building_type: int,
    building_cfg: dict,
) -> Tuple[float, float, float, float]:
    """Return (S_human, S_infra, S_property, S_secondary) for a cell's building type.

    Parameters
    ----------
    building_type : int
        Categorical building type code from the buildings raster.
        0 = no building, 1 = RESIDENTIAL, 2 = IMPORTANT, 3 = CRITICAL,
        4 = SECONDARY_HAZARD, 5 = COMMERCIAL, 6 = INDUSTRIAL, 7 = OTHER.
    building_cfg : dict
        The ``building_risk_values`` sub-dictionary from config, containing
        per-type risk contribution values.

    Returns
    -------
    Tuple[float, float, float, float]
        (S_human, S_infra, S_property, S_secondary) — each in [0.0, 1.0].
        Only one component is non-zero per building type.

    Notes
    -----
    These values represent **human and asset exposure proxies**, not actual
    population census or empirical loss counts.
    """
    s_human = 0.0
    s_infra = 0.0
    s_property = 0.0
    s_secondary = 0.0

    if building_type == 1:  # RESIDENTIAL → S_human
        s_human = float(building_cfg.get("residential", 1.0))
    elif building_type == 2:  # IMPORTANT → S_infra
        s_infra = float(building_cfg.get("important", 0.7))
    elif building_type == 3:  # CRITICAL → S_infra
        s_infra = float(building_cfg.get("critical", 1.0))
    elif building_type == 4:  # SECONDARY_HAZARD → S_secondary
        s_secondary = float(building_cfg.get("secondary_hazard", 1.0))
    elif building_type == 5:  # COMMERCIAL → S_property
        s_property = float(building_cfg.get("commercial", 1.0))
    elif building_type == 6:  # INDUSTRIAL → S_property
        s_property = float(building_cfg.get("industrial", 1.0))
    elif building_type == 7:  # OTHER → no automatic risk elevation
        pass  # intentionally 0.0

    return s_human, s_infra, s_property, s_secondary


def calculate_risk_scores(
    grid: EnvironmentGrid,
    config: Optional[dict] = None,
    spread_scores: Optional[Dict[int, float]] = None,
) -> Dict[int, float]:
    """Calculate and return composite risk scores for all cells in *grid*.

    Parameters
    ----------
    grid : EnvironmentGrid
        The simulation environment grid.
    config : dict, optional
        Configuration dictionary containing a ``risk_scoring`` section.
        If None, attempts to use configuration attached to *grid*.
    spread_scores : Dict[int, float], optional
        Precalculated mapping of ``cell_id`` to spread hazard score (S_spread ∈ [0, 1]).
        If None, S_spread defaults to 0.0 for all cells.

    Returns
    -------
    Dict[int, float]
        Dictionary mapping ``cell_id`` to calculated risk score in [min_risk, max_risk].
    """
    if config is None:
        config = getattr(grid, "_config", {})

    cfg = config.get("risk_scoring", {})
    burning_risk = float(cfg.get("burning_cell_risk", 1.0))
    burned_risk = float(cfg.get("burned_cell_risk", 0.2))
    min_risk = float(cfg.get("min_risk_score", 0.0))
    max_risk = float(cfg.get("max_risk_score", 1.0))

    # Top-Level Composite Risk Weights (normalized so sum(W_i) = 1.0)
    comp_weights_cfg = cfg.get("composite_weights")
    if comp_weights_cfg and isinstance(comp_weights_cfg, dict):
        raw_W_spread = float(comp_weights_cfg.get("spread", 1.0))
        raw_W_human = float(comp_weights_cfg.get("human", 0.3))
        raw_W_infra = float(comp_weights_cfg.get("infrastructure", 0.4))
        raw_W_property = float(comp_weights_cfg.get("property", 0.2))
        raw_W_secondary = float(comp_weights_cfg.get("secondary_hazard", 0.5))
    else:
        # Fallback to legacy flat weight keys
        raw_W_spread = float(cfg.get("spread_weight", 1.0))
        raw_W_human = float(cfg.get("human_weight", 0.3 if "human_weight" in cfg else 0.0))
        raw_W_infra = float(cfg.get("infrastructure_weight", 0.4 if "infrastructure_weight" in cfg else 0.0))
        raw_W_property = float(cfg.get("property_weight", 0.2 if "property_weight" in cfg else 0.0))
        raw_W_secondary = float(cfg.get("secondary_hazard_weight", 0.5 if "secondary_hazard_weight" in cfg else 0.0))

    total_comp_w = (
        raw_W_spread
        + raw_W_human
        + raw_W_infra
        + raw_W_property
        + raw_W_secondary
    )
    if total_comp_w > 0.0:
        W_spread = raw_W_spread / total_comp_w
        W_human = raw_W_human / total_comp_w
        W_infra = raw_W_infra / total_comp_w
        W_property = raw_W_property / total_comp_w
        W_secondary = raw_W_secondary / total_comp_w
    else:
        W_spread = 1.0
        W_human = W_infra = W_property = W_secondary = 0.0

    # Building type → exposure value mapping
    building_cfg = cfg.get("building_risk_values", {})

    cells = grid.cells
    risk_scores: Dict[int, float] = {}

    for cell in cells:
        if cell.fire_state == FireState.BURNING:
            score = burning_risk
        elif cell.fire_state == FireState.BURNED:
            score = burned_risk
        else:
            # Spread hazard component S_spread ∈ [0, 1]
            s_spread = spread_scores.get(cell.cell_id, 0.0) if spread_scores is not None else 0.0

            # Exposure / consequence components S_i ∈ [0, 1]
            s_human, s_infra, s_property, s_secondary = _compute_building_risk_components(
                cell.building_type, building_cfg
            )

            # Composite Weighted Sum (Σ W_i = 1.0)
            score = (
                W_spread * s_spread
                + W_human * s_human
                + W_infra * s_infra
                + W_property * s_property
                + W_secondary * s_secondary
            )

        # Numerical safety clamp to enforced bounds
        clamped_score = max(min_risk, min(max_risk, score))
        risk_scores[cell.cell_id] = float(clamped_score)

    return risk_scores


def update_grid_risk_scores(
    grid: EnvironmentGrid,
    config: Optional[dict] = None,
    spread_scores: Optional[Dict[int, float]] = None,
) -> Dict[int, float]:
    """Calculate risk scores and update ``cell.risk_score`` on *grid* cells.

    Parameters
    ----------
    grid : EnvironmentGrid
        The simulation environment grid.
    config : dict, optional
        Configuration dictionary.
    spread_scores : Dict[int, float], optional
        Precalculated spread hazard score mapping.

    Returns
    -------
    Dict[int, float]
        Mapping of ``cell_id`` to updated risk score.
    """
    scores = calculate_risk_scores(grid, config, spread_scores=spread_scores)

    cells = grid.cells
    for cell in cells:
        cell.risk_score = scores[cell.cell_id]

    return scores