"""Unilateral suspension end stops in compression-positive coordinates."""

from math import isfinite


def end_stop(
    q: float,
    v: float,
    lo: float,
    hi: float,
    k: float,
    c: float,
    *,
    upper_boundary_force_n: float = 0.0,
    upper_boundary_energy_j: float = 0.0,
) -> tuple[float, float]:
    """Return generalized stop force and stored energy for a bounded slide joint.

    Optional upper boundary terms let an emergency stop continue a bumper's force
    and potential at the working-stroke boundary without stacking both springs.
    """
    if (
        not all(
            isfinite(x)
            for x in (q, v, lo, hi, k, c, upper_boundary_force_n, upper_boundary_energy_j)
        )
        or hi <= lo
        or k <= 0
        or c < 0
        or upper_boundary_force_n < 0
        or upper_boundary_energy_j < 0
    ):
        raise ValueError("invalid end-stop parameters")

    low_depth = max(lo - q, 0.0)
    if low_depth > 0.0:
        return max(0.0, k * low_depth - c * v), 0.5 * k * low_depth**2

    high_depth = max(q - hi, 0.0)
    if high_depth > 0.0 or (q == hi and upper_boundary_force_n > 0.0):
        elastic_force = upper_boundary_force_n + k * high_depth
        force = -max(0.0, elastic_force + c * v)
        energy = upper_boundary_energy_j + upper_boundary_force_n * high_depth + 0.5 * k * high_depth**2
        return force, energy

    return 0.0, 0.0


__all__ = ["end_stop"]
