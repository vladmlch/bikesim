"""Acceptance and bounds checking for distributed contact manifold simulations."""

from math import isfinite
from typing import Any
from bike_sim.validation.contact_manifold_rigs import (
    distributed_flat_rig,
    distributed_step_rig,
    distributed_incline_rig,
)


def check_bounds(metrics: dict[str, Any], bounds: dict[str, tuple[float, float]]) -> dict[str, bool]:
    """Check that each bounded metric exists, is finite, and lies within [lo, hi].

    Fails closed: missing metrics, non-finite values, non-numeric values, or values outside
    bounds evaluate to False.
    """
    result: dict[str, bool] = {}
    for name, (lo, hi) in bounds.items():
        if name not in metrics:
            result[name] = False
            continue
        try:
            val = float(metrics[name])
        except (TypeError, ValueError):
            result[name] = False
            continue
        result[name] = bool(isfinite(val) and lo <= val <= hi)
    return result


def contact_matrix(
    dts: tuple[float, ...] = (.00125, .000625, .0003125),
    station_counts: tuple[int, ...] = (128, 256, 512),
    flat_loads: tuple[float, ...] = (300., 600., 900.),
    step_dxs: tuple[float, ...] = (.02, .01, .005),
    incline_angle_deg: float = 15.,
) -> list[dict[str, Any]]:
    """Evaluate contact manifold across timesteps, station counts, and road cases.

    Returns the exact inputs, output metrics, bounds checks, and acceptance status
    for each matrix combination.
    """
    rows: list[dict[str, Any]] = []
    for dt in dts:
        for count in station_counts:
            for load in flat_loads:
                metrics, bounds = distributed_flat_rig(dt, count, load)
                checks = check_bounds(metrics, bounds)
                rows.append({
                    'case': 'flat',
                    'dt_s': dt,
                    'station_count': count,
                    'load_n': load,
                    'metrics': metrics,
                    'checks': checks,
                    'accepted': all(checks.values()),
                })
            for dx in step_dxs:
                metrics, bounds = distributed_step_rig(dt, count, dx)
                checks = check_bounds(metrics, bounds)
                rows.append({
                    'case': 'step',
                    'dt_s': dt,
                    'station_count': count,
                    'terrain_dx_m': dx,
                    'metrics': metrics,
                    'checks': checks,
                    'accepted': all(checks.values()),
                })
            metrics, bounds = distributed_incline_rig(dt, count, incline_angle_deg)
            checks = check_bounds(metrics, bounds)
            rows.append({
                'case': 'incline',
                'dt_s': dt,
                'station_count': count,
                'angle_deg': incline_angle_deg,
                'metrics': metrics,
                'checks': checks,
                'accepted': all(checks.values()),
            })
    return rows
