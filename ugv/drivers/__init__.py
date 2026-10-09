# -*- coding: utf-8 -*-
from .base import Driver, DriverError, Progress, Telemetry


def build_driver(vehicle_cfg: dict, start_xy, clock, heading_deg: float = 0.0, profile=None):
    kind = vehicle_cfg.get("driver", "sim")
    if kind == "px4":
        from .px4 import Px4Driver
        return Px4Driver(vehicle_cfg["px4"]["address"], clock, decel_mps2=profile.decel_mps2 if profile else None)
    from .sim import SimDriver, ACCEL, DECEL
    if profile is None or profile.vehicle_kind == "UGV":
        return SimDriver(start_xy, clock, heading_deg)
    # 다른 차량은 계획 가감속보다 약간 큰 한계 (UGV 실측 비율 2.5/2.0, 2.1/1.5 를 따름)
    return SimDriver(start_xy, clock, heading_deg, accel=profile.accel_mps2 * ACCEL / 2.0,
                     decel=profile.decel_mps2 * DECEL / 1.5)


__all__ = ["Driver", "DriverError", "Progress", "Telemetry", "build_driver"]
