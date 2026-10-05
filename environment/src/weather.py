"""Weather data structures, historical time-series provider, and aggregation.

Owns meteorological data parsing, coordinate/circular aggregation, and retrieval
for simulation intervals [t, t + dt). Does not contain fire spread physics.
"""

from __future__ import annotations

import csv
from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime, timedelta
import math
from pathlib import Path
from typing import List, Optional, Tuple, Union


def validate_convention(convention: str) -> str:
    """Validate and normalize wind direction convention string.

    Allowed values: 'from', 'toward'.
    """
    conv = str(convention).strip().lower()
    if conv not in ("from", "toward"):
        raise ValueError(
            f"Invalid wind_direction_convention: '{convention}'. Allowed values are 'from' or 'toward'."
        )
    return conv


def to_toward_direction(direction_deg: float, convention: str = "from") -> float:
    """Convert raw wind direction degree to effective TOWARD direction in [0, 360).

    'from':
        direction_deg represents meteorological direction FROM which wind comes.
        effective TOWARD = (direction_deg + 180.0) % 360.0
    'toward':
        direction_deg represents direction TOWARD which wind blows.
        effective TOWARD = direction_deg % 360.0

    Parameters:
        direction_deg: Raw wind direction in degrees.
        convention: 'from' or 'toward'.

    Returns:
        float: Normalized direction in [0.0, 360.0) indicating the direction wind blows TOWARD.
    """
    conv = validate_convention(convention)
    deg = float(direction_deg) % 360.0

    if conv == "from":
        toward = (deg + 180.0) % 360.0
    else:
        toward = deg

    # Ensure strictly in [0.0, 360.0)
    if toward >= 360.0 or abs(toward - 360.0) < 1e-9 or toward < 0.0:
        toward = 0.0
    return toward


@dataclass(frozen=True)
class WeatherState:
    """Immutable weather state representation for a single CA simulation interval.

    Attributes:
        wind_speed_ms: Wind speed in meters per second (>= 0.0).
        wind_direction_deg: Effective direction the wind blows TOWARD in degrees [0, 360).
            (0° = blowing North, 90° = blowing East, 180° = blowing South, 270° = blowing West).
        humidity_percent: Atmospheric relative humidity in percent (0.0 - 100.0).
            NOTE: Atmospheric RH is NOT fuel moisture. Cell.moisture is a separate parameter.
        source: Diagnostic origin identifier ('interval_aggregation', 'previous_hold', 'fixed', etc.).
        raw_wind_direction_deg: Original unconverted wind direction reading before convention conversion.
        wind_direction_convention: Convention used for conversion ('from' or 'toward').
    """

    wind_speed_ms: float
    wind_direction_deg: float
    humidity_percent: float
    source: str = "unknown"
    raw_wind_direction_deg: Optional[float] = None
    wind_direction_convention: str = "from"


def compute_vector_mean_wind(observations: List[Tuple[float, float]]) -> Tuple[float, float]:
    """Calculate vector mean wind speed and direction from circular observations.

    For speed s and direction theta (deg, representing vector heading where 0° = North, 90° = East):
        u = s * sin(theta)  (East component)
        v = s * cos(theta)  (North component)

    Vector mean:
        u_mean = mean(u)
        v_mean = mean(v)
        mean_speed = sqrt(u_mean^2 + v_mean^2)
        mean_dir = atan2(u_mean, v_mean) normalized into [0, 360)

    Returns:
        Tuple[float, float]: (mean_speed_ms, mean_direction_deg)
    """
    if not observations:
        raise ValueError("Cannot calculate vector mean wind of empty observation list.")

    u_list: List[float] = []
    v_list: List[float] = []

    for s, theta in observations:
        theta_rad = math.radians(theta)
        u_list.append(s * math.sin(theta_rad))
        v_list.append(s * math.cos(theta_rad))

    u_mean = sum(u_list) / len(u_list)
    v_mean = sum(v_list) / len(v_list)

    mean_speed = math.hypot(u_mean, v_mean)

    if mean_speed < 1e-9:
        mean_dir = 0.0
    else:
        deg = math.degrees(math.atan2(u_mean, v_mean)) % 360.0
        # Ensure strictly in [0.0, 360.0)
        if deg >= 360.0 or abs(deg - 360.0) < 1e-9 or deg < 0.0:
            deg = 0.0
        mean_dir = deg

    return mean_speed, mean_dir


class WeatherProvider(ABC):
    """Abstract base provider for retrieving weather state during simulation."""

    @abstractmethod
    def get_weather(self, simulation_time: datetime, timestep: timedelta) -> WeatherState:
        """Return WeatherState for the interval [simulation_time, simulation_time + timestep)."""
        pass


class FixedWeatherProvider(WeatherProvider):
    """Provides constant weather for all timesteps (fixed mode)."""

    def __init__(
        self,
        wind_speed_ms: float = 8.3,
        wind_direction_deg: float = 158.6,
        humidity_percent: float = 25.5,
        convention: str = "from",
    ) -> None:
        self.convention = validate_convention(convention)
        self.wind_speed_ms = float(wind_speed_ms)
        self.raw_wind_direction_deg = float(wind_direction_deg) % 360.0
        self.wind_direction_deg = to_toward_direction(self.raw_wind_direction_deg, self.convention)
        self.humidity_percent = float(humidity_percent)

    def get_weather(self, simulation_time: datetime, timestep: timedelta) -> WeatherState:
        return WeatherState(
            wind_speed_ms=self.wind_speed_ms,
            wind_direction_deg=self.wind_direction_deg,
            humidity_percent=self.humidity_percent,
            source="fixed",
            raw_wind_direction_deg=self.raw_wind_direction_deg,
            wind_direction_convention=self.convention,
        )


@dataclass(frozen=True)
class ASOSObservation:
    """Internal representation of a single parsed ASOS historical observation."""

    timestamp: datetime
    wind_speed_ms: float
    wind_direction_deg: float
    humidity_percent: float
    temperature_celsius: Optional[float] = None
    cumulative_precipitation_mm: Optional[float] = None


class TimeSeriesWeatherProvider(WeatherProvider):
    """Provides dynamic weather aggregated from minute-level historical ASOS CSV observations."""

    def __init__(
        self,
        csv_path: Union[str, Path],
        aggregation: str = "interval",
        base_dir: Optional[Union[str, Path]] = None,
        convention: str = "from",
    ) -> None:
        self.convention = validate_convention(convention)
        self.csv_path = self._resolve_path(csv_path, base_dir)
        self.aggregation = aggregation
        self.observations: List[ASOSObservation] = []
        self._load_csv()

    def _resolve_path(
        self, csv_path: Union[str, Path], base_dir: Optional[Union[str, Path]]
    ) -> Path:
        raw_path = Path(csv_path)
        if raw_path.is_file():
            return raw_path.resolve()

        if base_dir is not None:
            candidate = (Path(base_dir) / raw_path).resolve()
            if candidate.is_file():
                return candidate

        # Check standard project locations
        current_file = Path(__file__).resolve()
        env_root = current_file.parent.parent  # environment/
        project_root = env_root.parent         # repo root

        for root in (project_root, env_root):
            cand = (root / raw_path).resolve()
            if cand.is_file():
                return cand

        raise FileNotFoundError(f"Historical weather CSV not found at: {csv_path}")

    @staticmethod
    def _parse_timestamp(val: str) -> datetime:
        """Parse datetime string handling common formats including '%d/%m/%Y %H:%M' and ISO."""
        val = val.strip()
        for fmt in (
            "%d/%m/%Y %H:%M",
            "%d/%m/%Y %H:%M:%S",
            "%Y-%m-%d %H:%M:%S",
            "%Y-%m-%d %H:%M",
            "%Y/%m/%d %H:%M:%S",
            "%Y/%m/%d %H:%M",
        ):
            try:
                return datetime.strptime(val, fmt)
            except ValueError:
                pass
        try:
            return datetime.fromisoformat(val)
        except ValueError as exc:
            raise ValueError(f"Unable to parse observation timestamp: '{val}'") from exc

    def _load_csv(self) -> None:
        """Load, validate, and index observations from CSV."""
        obs_list: List[ASOSObservation] = []
        with open(self.csv_path, mode="r", encoding="utf-8", errors="replace") as fh:
            reader = csv.DictReader(fh)
            fieldnames = [f.strip() for f in (reader.fieldnames or [])]

            # Discover column mapping
            dt_col = next((c for c in fieldnames if c in ("datetime", "tm", "date_time")), None)
            ws_col = next((c for c in fieldnames if c in ("wind_speed_ms", "ws", "wind_speed")), None)
            wd_col = next((c for c in fieldnames if c in ("wind_direction_deg", "wd", "wind_direction")), None)
            hm_col = next((c for c in fieldnames if c in ("humidity_percent", "hm", "humidity", "rh")), None)
            ta_col = next((c for c in fieldnames if c in ("temperature_celsius", "ta", "temp")), None)
            pr_col = next((c for c in fieldnames if c in ("cumulative_precipitation_mm", "rn", "precip")), None)

            if not (dt_col and ws_col and wd_col and hm_col):
                raise ValueError(
                    f"CSV {self.csv_path} missing required weather columns. Found: {fieldnames}"
                )

            for line_no, row in enumerate(reader, start=2):
                dt_str = row.get(dt_col)
                if not dt_str or not dt_str.strip():
                    continue

                try:
                    ts = self._parse_timestamp(dt_str)
                    ws_str = row.get(ws_col, "").strip()
                    wd_str = row.get(wd_col, "").strip()
                    hm_str = row.get(hm_col, "").strip()

                    if ws_str == "" or wd_str == "" or hm_str == "":
                        continue  # Skip rows with missing values

                    ws = float(ws_str)
                    wd = float(wd_str) % 360.0
                    hm = float(hm_str)

                    if ws < 0.0:
                        raise ValueError(f"Negative wind speed {ws}")
                    if not (0.0 <= hm <= 100.0):
                        hm = max(0.0, min(100.0, hm))

                    ta = float(row[ta_col].strip()) if ta_col and row.get(ta_col, "").strip() != "" else None
                    pr = float(row[pr_col].strip()) if pr_col and row.get(pr_col, "").strip() != "" else None

                    obs_list.append(
                        ASOSObservation(
                            timestamp=ts,
                            wind_speed_ms=ws,
                            wind_direction_deg=wd,
                            humidity_percent=hm,
                            temperature_celsius=ta,
                            cumulative_precipitation_mm=pr,
                        )
                    )
                except Exception as exc:
                    raise ValueError(f"Error parsing row {line_no} in {self.csv_path}: {exc}") from exc

        if not obs_list:
            raise ValueError(f"No valid weather observations found in {self.csv_path}")

        # Sort chronologically and deduplicate identical timestamps
        obs_list.sort(key=lambda o: o.timestamp)
        deduped: List[ASOSObservation] = []
        seen_times = set()
        for o in obs_list:
            if o.timestamp not in seen_times:
                seen_times.add(o.timestamp)
                deduped.append(o)

        self.observations = deduped

    def get_weather(self, simulation_time: datetime, timestep: timedelta) -> WeatherState:
        """Derive representative weather for interval [simulation_time, simulation_time + timestep).

        Interval observations: start <= obs.timestamp < start + timestep (half-open).
        If observations exist in interval:
            - Convert each observation's raw direction to effective TOWARD direction using convention
            - Vector mean of wind speed and TOWARD wind direction
            - Arithmetic mean of relative humidity
            - Source: "interval_aggregation"
        If no observations in interval:
            - Sparse fallback: previous-observation hold (latest obs <= simulation_time)
            - Source: "previous_hold"
        If no previous observation exists at all:
            - Raise ValueError.
        """
        sim_t = simulation_time
        if sim_t.tzinfo is not None and self.observations[0].timestamp.tzinfo is None:
            sim_t = sim_t.replace(tzinfo=None)

        interval_end = sim_t + timestep

        # 1. Gather observations within [sim_t, interval_end)
        in_interval = [
            o for o in self.observations if sim_t <= o.timestamp < interval_end
        ]

        if in_interval:
            # Convert each observation's direction to effective TOWARD direction
            wind_obs = [
                (o.wind_speed_ms, to_toward_direction(o.wind_direction_deg, self.convention))
                for o in in_interval
            ]
            mean_speed, mean_toward_dir = compute_vector_mean_wind(wind_obs)
            mean_hum = sum(o.humidity_percent for o in in_interval) / len(in_interval)

            raw_wind_obs = [(o.wind_speed_ms, o.wind_direction_deg) for o in in_interval]
            _, raw_mean_dir = compute_vector_mean_wind(raw_wind_obs)

            return WeatherState(
                wind_speed_ms=mean_speed,
                wind_direction_deg=mean_toward_dir,
                humidity_percent=mean_hum,
                source="interval_aggregation",
                raw_wind_direction_deg=raw_mean_dir,
                wind_direction_convention=self.convention,
            )

        # 2. Sparse fallback: previous-observation hold
        prev_obs: Optional[ASOSObservation] = None
        for o in reversed(self.observations):
            if o.timestamp <= sim_t:
                prev_obs = o
                break

        if prev_obs is not None:
            toward_dir = to_toward_direction(prev_obs.wind_direction_deg, self.convention)
            return WeatherState(
                wind_speed_ms=prev_obs.wind_speed_ms,
                wind_direction_deg=toward_dir,
                humidity_percent=prev_obs.humidity_percent,
                source="previous_hold",
                raw_wind_direction_deg=prev_obs.wind_direction_deg,
                wind_direction_convention=self.convention,
            )

        # 3. Before earliest observation
        earliest_time = self.observations[0].timestamp
        raise ValueError(
            f"No historical weather observations available on or before simulation time {simulation_time} "
            f"(earliest available: {earliest_time})"
        )


def create_weather_provider(
    config: dict, base_dir: Optional[Union[str, Path]] = None
) -> WeatherProvider:
    """Factory creating WeatherProvider from environment configuration dictionary."""
    weather_cfg = config.get("weather", {})
    mode = str(weather_cfg.get("mode", "fixed")).lower()
    convention = str(weather_cfg.get("wind_direction_convention", "from")).lower()

    if mode == "timeseries":
        ts_cfg = weather_cfg.get("timeseries", {})
        csv_path = ts_cfg.get("path", "data/inje2019/weather/Inje_ASOS_190404_min.csv")
        aggregation = ts_cfg.get("aggregation", "interval")
        return TimeSeriesWeatherProvider(
            csv_path=csv_path,
            aggregation=aggregation,
            base_dir=base_dir,
            convention=convention,
        )
    else:
        fixed_cfg = weather_cfg.get("fixed", {})
        wind_cfg = config.get("wind", {})
        speed = float(fixed_cfg.get("wind_speed_ms", wind_cfg.get("speed_ms", 8.3)))
        direction = float(fixed_cfg.get("wind_direction_deg", wind_cfg.get("direction_deg", 158.6)))
        default_moisture = float(config.get("defaults", {}).get("moisture", 0.255))
        humidity = float(fixed_cfg.get("humidity_percent", default_moisture * 100.0))
        return FixedWeatherProvider(
            wind_speed_ms=speed,
            wind_direction_deg=direction,
            humidity_percent=humidity,
            convention=convention,
        )
