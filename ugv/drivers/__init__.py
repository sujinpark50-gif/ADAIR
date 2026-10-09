# -*- coding: utf-8 -*-
from .base import Driver, DriverError, Progress, Telemetry


def build_driver(vehicle_cfg: dict, start_xy, clock, heading_deg: float = 0.0):
    kind = vehicle_cfg.get("driver", "sim")
    if kind == "px4":
        from .px4 import Px4Driver
        return Px4Driver(vehicle_cfg["px4"]["address"], clock)
    from .sim import SimDriver
    return SimDriver(start_xy, clock, heading_deg)


__all__ = ["Driver", "DriverError", "Progress", "Telemetry", "build_driver"]
