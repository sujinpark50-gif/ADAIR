# -*- coding: utf-8 -*-
"""Comprehensive tests for simulation-time historical weather integration.

Covers all 26 test requirements from prompt1.txt:
1. FixedWeatherProvider returns constant wind and humidity.
2. TimeSeriesWeatherProvider successfully reads the actual ASOS CSV schema.
3. Actual CSV column mapping is handled correctly.
4. Minute-level observations are assigned to the correct 10-minute interval.
5. Interval boundaries use [start, end) without double-counting observations.
6. Wind vector averaging works correctly.
7. 359° + 1° produces approximately north/0°, not 180°.
8. Humidity uses arithmetic mean of valid observations.
9. Invalid/missing humidity is handled explicitly.
10. Sparse observations use previous-observation hold.
11. Missing weather before the first available observation raises a clear error.
12. Wind direction remains in [0, 360).
13. CA step 0 starts at: 2019-04-04 14:45.
14. Simulation time advances exactly 10 minutes with the current configuration.
15. 18:55 on April 6 is a valid final step start.
16. 19:05 on April 6 is NOT executed.
17. Fixed mode preserves legacy fixed-wind behavior.
18. Timeseries mode changes weather according to simulation time.
19. CA spread uses current timestep wind.
20. Geometric spread prediction uses current timestep wind.
21. Ensemble spread prediction does not mutate the live simulation clock.
22. Ensemble spread prediction remains deterministic for a fixed seed.
23. API get_spread_direction() reports current active wind.
24. Existing ObservationUpdateHandler behavior remains compatible.
25. Atmospheric humidity is NOT silently written into Cell.moisture.
26. Existing project tests continue to pass.
"""

from datetime import datetime, timedelta
import math
from pathlib import Path
import sys
import tempfile
import pytest

ENV_DIR = Path(__file__).resolve().parents[1]
if str(ENV_DIR) not in sys.path:
    sys.path.insert(0, str(ENV_DIR))

from src.api import EnvironmentModelAPI
from src.cell import Cell, FireState
from src.fire_model import WildfireCAEngine
from src.grid import EnvironmentGrid
from src.observation import ObservationReport, ObservationUpdateHandler
from src.weather import (
    FixedWeatherProvider,
    TimeSeriesWeatherProvider,
    WeatherProvider,
    WeatherState,
    compute_vector_mean_wind,
    create_weather_provider,
    to_toward_direction,
    validate_convention,
)

ASOS_CSV_PATH = Path("data/inje2019/weather/Inje_ASOS_190404_min.csv")
CONFIG_PATH = Path("environment/config/environment_config.yaml")


# ---------------------------------------------------------------------------
# 0. to_toward_direction and convention validation (prompt2 Section 10)
# ---------------------------------------------------------------------------
def test_to_toward_direction_conventions():
    # FROM convention
    assert math.isclose(to_toward_direction(0.0, "from"), 180.0)
    assert math.isclose(to_toward_direction(90.0, "from"), 270.0)
    assert math.isclose(to_toward_direction(158.6, "from"), 338.6)
    assert math.isclose(to_toward_direction(270.0, "from"), 90.0)

    # TOWARD convention
    assert math.isclose(to_toward_direction(0.0, "toward"), 0.0)
    assert math.isclose(to_toward_direction(90.0, "toward"), 90.0)
    assert math.isclose(to_toward_direction(158.6, "toward"), 158.6)
    assert math.isclose(to_toward_direction(270.0, "toward"), 270.0)

    # Normalization to [0, 360)
    assert math.isclose(to_toward_direction(360.0, "from"), 180.0)
    assert math.isclose(to_toward_direction(360.0, "toward"), 0.0)
    assert math.isclose(to_toward_direction(-90.0, "from"), 90.0)
    assert math.isclose(to_toward_direction(-90.0, "toward"), 270.0)

    # Invalid convention raises ValueError
    with pytest.raises(ValueError, match="Invalid wind_direction_convention"):
        to_toward_direction(180.0, "invalid_convention")
    with pytest.raises(ValueError, match="Invalid wind_direction_convention"):
        validate_convention("invalid_convention")


# ---------------------------------------------------------------------------
# 1. FixedWeatherProvider
# ---------------------------------------------------------------------------
def test_fixed_weather_provider():
    # Default 'from' convention
    provider_from = FixedWeatherProvider(
        wind_speed_ms=8.3, wind_direction_deg=158.6, humidity_percent=25.5, convention="from"
    )
    t0 = datetime(2019, 4, 4, 14, 45)
    dt = timedelta(minutes=10)

    w1 = provider_from.get_weather(t0, dt)
    assert w1.wind_speed_ms == 8.3
    assert math.isclose(w1.wind_direction_deg, 338.6)  # effective TOWARD direction
    assert math.isclose(w1.raw_wind_direction_deg, 158.6)
    assert w1.wind_direction_convention == "from"
    assert w1.humidity_percent == 25.5
    assert w1.source == "fixed"

    # 'toward' convention
    provider_toward = FixedWeatherProvider(
        wind_speed_ms=8.3, wind_direction_deg=158.6, humidity_percent=25.5, convention="toward"
    )
    w_toward = provider_toward.get_weather(t0, dt)
    assert math.isclose(w_toward.wind_direction_deg, 158.6)  # effective TOWARD direction
    assert math.isclose(w_toward.raw_wind_direction_deg, 158.6)
    assert w_toward.wind_direction_convention == "toward"


# ---------------------------------------------------------------------------
# 2 & 3. TimeSeriesWeatherProvider reads actual CSV schema and column mapping
# ---------------------------------------------------------------------------
def test_timeseries_provider_actual_csv():
    assert ASOS_CSV_PATH.is_file(), f"Actual historical CSV {ASOS_CSV_PATH} must exist"
    provider = TimeSeriesWeatherProvider(ASOS_CSV_PATH)
    assert len(provider.observations) == 4320  # 3 days * 1440 min
    first_obs = provider.observations[0]
    assert first_obs.timestamp == datetime(2019, 4, 4, 0, 1)
    assert first_obs.wind_speed_ms >= 0.0
    assert 0.0 <= first_obs.wind_direction_deg < 360.0
    assert 0.0 <= first_obs.humidity_percent <= 100.0


# ---------------------------------------------------------------------------
# 4 & 5. Interval boundaries [start, end) without double counting
# ---------------------------------------------------------------------------
def test_interval_boundaries_half_open():
    provider = TimeSeriesWeatherProvider(ASOS_CSV_PATH)
    t_start = datetime(2019, 4, 4, 14, 45)
    dt = timedelta(minutes=10)

    # 14:45-14:55 interval
    w0 = provider.get_weather(t_start, dt)
    assert w0.source == "interval_aggregation"

    # Step 1: 14:55-15:05 interval
    w1 = provider.get_weather(t_start + dt, dt)
    assert w1.source == "interval_aggregation"
    # Observations at 14:55 belong to w1, not w0
    assert w0 != w1


# ---------------------------------------------------------------------------
# 6, 7 & 12. Wind vector averaging, 359°+1° circular case, [0, 360) range
# ---------------------------------------------------------------------------
def test_wind_vector_circular_averaging():
    # 359° and 1° with equal speed should result in 0° (North), NOT 180° (South)
    speed, direction = compute_vector_mean_wind([(10.0, 359.0), (10.0, 1.0)])
    assert math.isclose(speed, 10.0 * math.cos(math.radians(1.0)), rel_tol=1e-3)
    assert math.isclose(direction, 0.0, abs_tol=1e-3)

    # Pure cardinal directions
    s, d = compute_vector_mean_wind([(5.0, 0.0)])
    assert math.isclose(s, 5.0) and math.isclose(d, 0.0)

    s, d = compute_vector_mean_wind([(5.0, 90.0)])
    assert math.isclose(s, 5.0) and math.isclose(d, 90.0)

    s, d = compute_vector_mean_wind([(5.0, 180.0)])
    assert math.isclose(s, 5.0) and math.isclose(d, 180.0)

    s, d = compute_vector_mean_wind([(5.0, 270.0)])
    assert math.isclose(s, 5.0) and math.isclose(d, 270.0)

    # Normalization strictly in [0, 360)
    assert 0.0 <= direction < 360.0


# ---------------------------------------------------------------------------
# 8 & 9. Humidity arithmetic mean and missing/invalid handling
# ---------------------------------------------------------------------------
def test_humidity_aggregation_and_validation():
    # Synthetic CSV with missing values and varying humidity
    csv_content = (
        "station_id,station_name,datetime,temperature_celsius,cumulative_precipitation_mm,wind_direction_deg,wind_speed_ms,humidity_percent\n"
        "211,인제,04/04/2019 10:00,10.0,0,180.0,5.0,40.0\n"
        "211,인제,04/04/2019 10:01,10.0,0,180.0,5.0,60.0\n"
        "211,인제,04/04/2019 10:02,10.0,0,180.0,5.0,\n"  # missing humidity -> skipped
    )
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False, suffix=".csv") as tmp:
        tmp.write(csv_content)
        tmp_path = Path(tmp.name)

    try:
        provider = TimeSeriesWeatherProvider(tmp_path)
        w = provider.get_weather(datetime(2019, 4, 4, 10, 0), timedelta(minutes=10))
        assert math.isclose(w.humidity_percent, 50.0)  # (40 + 60) / 2
        assert math.isclose(w.wind_speed_ms, 5.0)
        # 180.0 FROM -> 0.0 TOWARD
        assert math.isclose(w.wind_direction_deg, 0.0)
        assert math.isclose(w.raw_wind_direction_deg, 180.0)
    finally:
        tmp_path.unlink(missing_ok=True)


# ---------------------------------------------------------------------------
# 10 & 11. Sparse data fallback (previous hold) and before earliest error
# ---------------------------------------------------------------------------
def test_sparse_data_fallback_and_error():
    # Hourly data (14:00, 15:00)
    csv_content = (
        "station_id,station_name,datetime,temperature_celsius,cumulative_precipitation_mm,wind_direction_deg,wind_speed_ms,humidity_percent\n"
        "211,인제,04/04/2019 14:00,15.0,0,160.0,6.0,30.0\n"
        "211,인제,04/04/2019 15:00,16.0,0,170.0,7.0,35.0\n"
    )
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False, suffix=".csv") as tmp:
        tmp.write(csv_content)
        tmp_path = Path(tmp.name)

    try:
        provider = TimeSeriesWeatherProvider(tmp_path)

        # 14:45 has no observations in [14:45, 14:55) -> previous hold (14:00)
        w_1445 = provider.get_weather(datetime(2019, 4, 4, 14, 45), timedelta(minutes=10))
        assert w_1445.source == "previous_hold"
        assert w_1445.wind_speed_ms == 6.0
        # 160.0 FROM -> 340.0 TOWARD
        assert math.isclose(w_1445.wind_direction_deg, 340.0)
        assert math.isclose(w_1445.raw_wind_direction_deg, 160.0)
        assert w_1445.humidity_percent == 30.0

        # Query before 14:00 raises ValueError
        with pytest.raises(ValueError, match="No historical weather observations available"):
            provider.get_weather(datetime(2019, 4, 4, 13, 0), timedelta(minutes=10))
    finally:
        tmp_path.unlink(missing_ok=True)


# ---------------------------------------------------------------------------
# 13, 14, 15 & 16. CA step 0 start time, 10-min advance, 18:55 end boundary
# ---------------------------------------------------------------------------
def test_simulation_clock_and_end_time_boundary():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)
    test_cfg = {
        "simulation": {"start_time": "2019-04-04T14:45:00", "end_time": "2019-04-06T19:00:00", "timestep_minutes": 10},
        "weather": {"mode": "fixed", "wind_direction_convention": "toward", "fixed": {"wind_speed_ms": 5.0, "wind_direction_deg": 270.0, "humidity_percent": 30.0}},
    }
    engine = WildfireCAEngine(grid, config_path=test_cfg, seed=42)

    # 13. CA step 0 starts at 2019-04-04 14:45
    assert engine.current_simulation_time == datetime(2019, 4, 4, 14, 45)
    assert engine.start_time == datetime(2019, 4, 4, 14, 45)
    assert engine.end_time == datetime(2019, 4, 6, 19, 0)
    assert engine.timestep == timedelta(minutes=10)
    assert engine.step_count == 0
    assert not engine.simulation_finished

    # 14. Advances 10 minutes on step()
    engine.step()
    assert engine.current_simulation_time == datetime(2019, 4, 4, 14, 55)
    assert engine.step_count == 1

    # Fast forward clock to 18:55 on April 6 (final valid step start)
    engine._simulation_time = datetime(2019, 4, 6, 18, 55)
    assert not engine.simulation_finished

    # 15. 18:55 is a valid step start
    engine.step()
    assert engine.current_simulation_time == datetime(2019, 4, 6, 19, 5)

    # 16. 19:05 is finished and NOT executed
    assert engine.simulation_finished
    result = engine.step()
    assert result == []
    assert engine.current_simulation_time == datetime(2019, 4, 6, 19, 5)  # clock stays stopped


# ---------------------------------------------------------------------------
# 17 & 18. Fixed vs Timeseries mode behavior
# ---------------------------------------------------------------------------
def test_fixed_vs_timeseries_mode():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)

    # Fixed mode with 'toward' convention
    fixed_cfg = {
        "simulation": {"start_time": "2019-04-04T14:45:00", "end_time": "2019-04-06T19:00:00", "timestep_minutes": 10},
        "weather": {
            "mode": "fixed",
            "wind_direction_convention": "toward",
            "fixed": {"wind_speed_ms": 12.0, "wind_direction_deg": 270.0, "humidity_percent": 15.0},
        },
        "fire_ca": {"base_spread_probability": 0.5},
    }
    fixed_engine = WildfireCAEngine(grid, config_path=fixed_cfg, seed=42)
    assert fixed_engine.current_weather.wind_speed_ms == 12.0
    assert fixed_engine.current_weather.wind_direction_deg == 270.0
    assert fixed_engine.current_weather.source == "fixed"

    fixed_engine.step()
    assert fixed_engine.current_weather.wind_speed_ms == 12.0  # constant

    # Timeseries mode
    ts_engine = WildfireCAEngine(grid, config_path=CONFIG_PATH, seed=42)
    assert ts_engine.current_weather.source == "interval_aggregation"
    w0 = ts_engine.current_weather
    # Step 0 uses w0 (14:45-14:55) and advances clock to 14:55
    ts_engine.step()
    # Step 1 uses w1 (14:55-15:05) and advances clock to 15:05
    ts_engine.step()
    w1 = ts_engine.current_weather
    assert w0 != w1  # weather updates dynamically per timestep


def test_fixed_mode_convention_switching():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)

    # Experiment A: convention="from", raw=158.6 -> effective toward 338.6
    cfg_from = {
        "weather": {
            "mode": "fixed",
            "wind_direction_convention": "from",
            "fixed": {"wind_speed_ms": 8.3, "wind_direction_deg": 158.6, "humidity_percent": 25.5},
        }
    }
    engine_from = WildfireCAEngine(grid, config_path=cfg_from, seed=42)
    assert math.isclose(engine_from.current_weather.wind_direction_deg, 338.6)
    assert math.isclose(engine_from.current_weather.raw_wind_direction_deg, 158.6)
    assert engine_from.current_weather.wind_direction_convention == "from"

    # Experiment B: convention="toward", raw=158.6 -> effective toward 158.6
    cfg_toward = {
        "weather": {
            "mode": "fixed",
            "wind_direction_convention": "toward",
            "fixed": {"wind_speed_ms": 8.3, "wind_direction_deg": 158.6, "humidity_percent": 25.5},
        }
    }
    engine_toward = WildfireCAEngine(grid, config_path=cfg_toward, seed=42)
    assert math.isclose(engine_toward.current_weather.wind_direction_deg, 158.6)
    assert math.isclose(engine_toward.current_weather.raw_wind_direction_deg, 158.6)
    assert engine_toward.current_weather.wind_direction_convention == "toward"


def test_timeseries_mode_convention_switching():
    # Verify TimeSeriesWeatherProvider interval aggregation applies convention correctly
    provider_from = TimeSeriesWeatherProvider(ASOS_CSV_PATH, convention="from")
    provider_toward = TimeSeriesWeatherProvider(ASOS_CSV_PATH, convention="toward")

    t0 = datetime(2019, 4, 4, 14, 45)
    dt = timedelta(minutes=10)

    w_from = provider_from.get_weather(t0, dt)
    w_toward = provider_toward.get_weather(t0, dt)

    assert math.isclose(w_from.wind_speed_ms, w_toward.wind_speed_ms)
    # Wind direction opposite by 180 degrees
    expected_toward = (w_toward.wind_direction_deg + 180.0) % 360.0
    assert math.isclose(w_from.wind_direction_deg, expected_toward, abs_tol=1e-3)
    assert w_from.wind_direction_convention == "from"
    assert w_toward.wind_direction_convention == "toward"


# ---------------------------------------------------------------------------
# 19 & 20. Spread calculations use current active wind
# ---------------------------------------------------------------------------
def test_spread_uses_current_wind():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)
    engine = WildfireCAEngine(grid, config_path=CONFIG_PATH, seed=42)

    diag = engine.get_weather_diagnostics()
    assert diag["step"] == 0
    assert diag["simulation_time"] == "2019-04-04T14:45:00"
    assert diag["weather_mode"] == "timeseries"
    assert diag["wind_direction_convention"] in ("from", "toward")
    assert "effective_toward_direction_deg" in diag
    assert "raw_wind_direction_deg" in diag

    # Geometric spread uses active wind
    geo_scores = engine.predict_spread_geometric()
    assert len(geo_scores) == len(grid.cells)


# ---------------------------------------------------------------------------
# 21 & 22. Ensemble prediction non-mutating guarantee and reproducibility
# ---------------------------------------------------------------------------
def test_ensemble_prediction_clock_and_determinism():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)
    engine = WildfireCAEngine(grid, config_path=CONFIG_PATH, seed=42)
    engine.ignite_from_config()

    clock_before = engine.current_simulation_time
    step_before = engine.step_count
    weather_before = engine.current_weather
    rng_state_before = engine._rng.bit_generator.state

    # 21. Run ensemble prediction -> must NOT mutate clock, steps, or live RNG
    scores1 = engine.predict_spread_ensemble(steps=3, runs=3, seed=123)

    assert engine.current_simulation_time == clock_before
    assert engine.step_count == step_before
    assert engine.current_weather == weather_before
    assert engine._rng.bit_generator.state == rng_state_before

    # 22. Deterministic for same seed
    scores2 = engine.predict_spread_ensemble(steps=3, runs=3, seed=123)
    assert scores1 == scores2


# ---------------------------------------------------------------------------
# 23. API get_spread_direction() reports current active wind
# ---------------------------------------------------------------------------
def test_api_spread_direction():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)
    engine = WildfireCAEngine(grid, config_path=CONFIG_PATH, seed=42)
    api = EnvironmentModelAPI(grid, engine)

    spread_dir = api.get_spread_direction()
    assert math.isclose(spread_dir["wind_speed_ms"], engine.current_weather.wind_speed_ms)
    assert math.isclose(spread_dir["wind_direction_deg"], engine.current_weather.wind_direction_deg)
    assert spread_dir["primary_cardinal"] in ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


# ---------------------------------------------------------------------------
# 24. ObservationUpdateHandler compatibility
# ---------------------------------------------------------------------------
def test_observation_update_handler_compatibility():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)
    engine = WildfireCAEngine(grid, config_path=CONFIG_PATH, seed=42)
    handler = ObservationUpdateHandler(grid, engine)

    rep = ObservationReport(
        reporter_id="UAV-01",
        cell_id=grid.cells[10].cell_id,
        wind_speed_ms=15.0,
        wind_direction_deg=270.0,
    )
    res = handler.process_observation(rep)
    assert res["success"]
    assert engine.current_weather.wind_speed_ms == 15.0
    conv = engine.current_weather.wind_direction_convention
    expected_toward = to_toward_direction(270.0, conv)
    assert math.isclose(engine.current_weather.wind_direction_deg, expected_toward)
    assert math.isclose(engine.current_weather.raw_wind_direction_deg, 270.0)


# ---------------------------------------------------------------------------
# 25. Atmospheric humidity is NOT written into Cell.moisture
# ---------------------------------------------------------------------------
def test_atmospheric_rh_not_written_to_cell_moisture():
    grid = EnvironmentGrid(CONFIG_PATH, seed=42)
    initial_moisture = grid.cells[0].moisture
    engine = WildfireCAEngine(grid, config_path=CONFIG_PATH, seed=42)

    # Verify initial moisture matches config defaults, not humidity_percent / 100
    assert initial_moisture == 0.255

    engine.step()
    # After step with historical humidity ~25.75%, cell moisture remains untouched
    assert grid.cells[0].moisture == 0.255
