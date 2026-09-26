"""Longitudinal tyre slip: exact relaxation and two brush discretisations.

The lumped brush is the steady parabolic-pressure solution in RIDE.md section 4.1.  The
detailed brush carries one bristle per loaded ray, advects bristle memory with the wheel,
and clips each bristle at its local friction limit.  All quantities at this boundary use
metres, seconds, newtons and a `SurfaceSpec`; there is no MuJoCo dependency.
"""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from bike_sim.terrain.surface import SurfaceSpec


@dataclass(frozen=True)
class BrushResult:
    """Tangential force and sliding state for one contact patch."""

    force_n: float
    fully_sliding: bool
    friction_coefficient: float


def relax(kappa_prime: float, v_x_mps: float, v_s_mps: float,
          sigma_m: float, dt_s: float) -> float:
    """Exact update of transient slip for constant velocities over one step.

    Integrates ``σ·dκ'/dt + |V_x|·κ' = −V_s``.  At nonzero speed it exponentially
    approaches ``−V_s/|V_x|`` with a time constant ``σ/|V_x|``.  At zero speed the exact
    limit is linear in time, so a parked tyre with ``V_s = 0`` holds its tread state without
    division by zero.

    Args:
        kappa_prime: Previous transient longitudinal slip ratio.
        v_x_mps: Hub speed along the contact tangent, in m/s.
        v_s_mps: Tread sliding speed over the road, in m/s.
        sigma_m: Relaxation length, in metres; zero means no distance lag.
        dt_s: Time step, in seconds.

    Returns:
        Updated transient slip ratio.

    Raises:
        ValueError: If a length, time step or velocity is invalid.
    """
    values = (kappa_prime, v_x_mps, v_s_mps, sigma_m, dt_s)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("slip relaxation inputs must be finite")
    if sigma_m < 0.0 or dt_s < 0.0:
        raise ValueError("relaxation length and time step must not be negative")

    speed_mps = abs(float(v_x_mps))
    if sigma_m == 0.0:
        if speed_mps > 0.0:
            return -float(v_s_mps) / speed_mps
        # With no rolling distance, the ratio has no finite algebraic target.  The detailed
        # brush integrates V_s directly into its bristles while this state remains finite.
        return float(kappa_prime)
    if speed_mps == 0.0:
        return float(kappa_prime) - float(v_s_mps) * float(dt_s) / float(sigma_m)

    steady_kappa = -float(v_s_mps) / speed_mps
    exponent = -speed_mps * float(dt_s) / float(sigma_m)
    return steady_kappa + (float(kappa_prime) - steady_kappa) * math.exp(exponent)


def _tread_stiffness_n_m2(
    normal_load_n: float,
    half_length_m: float,
    surface: SurfaceSpec,
    tread_stiffness_n_m2: Optional[float],
) -> float:
    """Resolves the pressure-normalised brush stiffness `c_px`."""
    if tread_stiffness_n_m2 is not None:
        value = float(tread_stiffness_n_m2)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"tread stiffness must be finite and nonnegative, got {value}")
        return value
    return surface.slip_stiffness_per_load * normal_load_n / (2.0 * half_length_m ** 2)


def _sigma_x(kappa_prime: float) -> float:
    """Brush slip input; a locked wheel is the negative fully-sliding limit."""
    if kappa_prime <= -1.0:
        return -math.inf
    if math.isinf(kappa_prime):
        return math.copysign(1.0, kappa_prime)
    return kappa_prime / (1.0 + kappa_prime)


def lumped_brush(
    kappa_prime: float,
    normal_load_n: float,
    half_length_m: float,
    surface: SurfaceSpec,
    tread_stiffness_n_m2: Optional[float] = None,
    sliding_speed_mps: float = 0.0,
) -> BrushResult:
    """Steady parabolic-pressure brush force for one patch.

    Args:
        kappa_prime: Transient slip from :func:`relax`.
        normal_load_n: Patch normal load, in newtons.
        half_length_m: Half of the fitted contact length, in metres.
        surface: Friction and normalised slip-stiffness parameters.
        tread_stiffness_n_m2: Optional explicit `c_px`; by default it follows the surface's
            ``C_κ/F_z`` and this patch's load and contact length.
        sliding_speed_mps: Absolute tread sliding speed used by the Stribeck curve.

    Returns:
        Tangential force (positive for drive), full-sliding flag, and friction coefficient.

    Raises:
        ValueError: If a positive load is paired with invalid dimensions or inputs.
    """
    values = (kappa_prime, normal_load_n, half_length_m, sliding_speed_mps)
    if not all(math.isfinite(float(value)) for value in values):
        if not math.isinf(float(kappa_prime)):
            raise ValueError("lumped brush inputs must be finite")
        if not all(math.isfinite(float(value)) for value in values[1:]):
            raise ValueError("load, patch length and sliding speed must be finite")
    if normal_load_n < 0.0:
        raise ValueError("normal load must not be negative")
    if normal_load_n == 0.0:
        return BrushResult(0.0, False, float(surface.mu(sliding_speed_mps)))
    if half_length_m <= 0.0:
        raise ValueError("a loaded patch must have positive half-length")

    mu = float(surface.mu(sliding_speed_mps))
    c_px = _tread_stiffness_n_m2(
        float(normal_load_n), float(half_length_m), surface, tread_stiffness_n_m2
    )
    if c_px == 0.0:
        return BrushResult(0.0, False, mu)

    theta = 2.0 * c_px * half_length_m ** 2 / (3.0 * mu * normal_load_n)
    z = theta * _sigma_x(float(kappa_prime))
    if abs(z) >= 1.0:
        return BrushResult(mu * normal_load_n * math.copysign(1.0, z), True, mu)
    force = 3.0 * mu * normal_load_n * z * (1.0 - abs(z) + z * z / 3.0)
    return BrushResult(float(force), False, mu)


class DiscretisedBrush:
    """Per-bristle brush state for one contiguous contact patch.

    Rays are ordered rear to front.  Positive wheel rotation transports bristles from larger
    indices to smaller ones; its surface speed is ``V_x - V_s``.  The patch's parabolic pressure distribution is represented by
    ``normal_loads_n``; on an irregular edge those loads may be lopsided.  The separate
    ``kappa_prime`` input filter uses ``max(0, σ − a)`` because transport through the patch
    already contributes about one half-length of relaxation.
    """

    def __init__(self, n_elements: int) -> None:
        if n_elements < 2:
            raise ValueError(f"a discretised patch needs at least 2 elements, got {n_elements}")
        self.n_elements = int(n_elements)
        self.bristle_deflection_m = np.zeros(self.n_elements, dtype=float)
        self.kappa_prime = 0.0

    def reset(self) -> None:
        """Clears the tread bristles and transient slip state."""
        self.bristle_deflection_m.fill(0.0)
        self.kappa_prime = 0.0

    def step(
        self,
        normal_loads_n: np.ndarray,
        *,
        half_length_m: float,
        v_x_mps: float,
        v_s_mps: float,
        sigma_m: float,
        surface: SurfaceSpec,
        dt_s: float,
    ) -> BrushResult:
        """Advects and updates every bristle, then sums the limited shear forces.

        Args:
            normal_loads_n: Each bristle's share of the patch normal load, in newtons.
            half_length_m: Patch half-length `a`, in metres.
            v_x_mps: Hub speed along the patch tangent; its sign sets transport direction.
            v_s_mps: Tread sliding speed over the road, in m/s.
            sigma_m: Total desired relaxation length, in metres.
            surface: Friction and normalised slip-stiffness parameters.
            dt_s: Positive simulation time step, in seconds.

        Returns:
            Sum of per-bristle forces, full-sliding flag, and the Stribeck coefficient.

        Raises:
            ValueError: If loads, kinematics, length or time step are invalid.
        """
        loads = np.asarray(normal_loads_n, dtype=float)
        if loads.shape != (self.n_elements,):
            raise ValueError(f"expected {self.n_elements} normal loads, got {loads.shape}")
        if not np.all(np.isfinite(loads)) or np.any(loads < 0.0):
            raise ValueError("bristle loads must be finite and nonnegative")
        if not all(math.isfinite(float(value)) for value in
                   (half_length_m, v_x_mps, v_s_mps, sigma_m, dt_s)):
            raise ValueError("brush kinematics and dimensions must be finite")
        if half_length_m < 0.0 or sigma_m < 0.0 or dt_s <= 0.0:
            raise ValueError("patch length and relaxation length must be nonnegative; dt must be positive")

        total_load_n = float(np.sum(loads))
        mu = float(surface.mu(abs(v_s_mps)))
        if total_load_n <= 0.0 or half_length_m <= 0.0:
            self.reset()
            return BrushResult(0.0, False, mu)

        input_sigma_m = max(0.0, float(sigma_m) - float(half_length_m))
        self.kappa_prime = relax(
            self.kappa_prime, v_x_mps, v_s_mps, input_sigma_m, dt_s
        )

        dx_m = 2.0 * float(half_length_m) / self.n_elements
        # The filter supplies the slip ratio during rolling.  At rest it has no finite
        # ratio representation, so integrate the measured V_s directly into the bristles.
        filtered_v_s_mps = (
            -self.kappa_prime * abs(float(v_x_mps))
            if v_x_mps != 0.0
            else float(v_s_mps)
        )
        wheel_surface_speed_mps = float(v_x_mps) - filtered_v_s_mps
        if wheel_surface_speed_mps != 0.0:
            source_index = (
                np.arange(self.n_elements, dtype=float)
                + wheel_surface_speed_mps * dt_s / dx_m
            )
            advected = np.interp(
                source_index,
                np.arange(self.n_elements, dtype=float),
                self.bristle_deflection_m,
                left=0.0,
                right=0.0,
            )
        else:
            advected = self.bristle_deflection_m.copy()

        updated = advected - filtered_v_s_mps * dt_s
        active = loads > 0.0
        updated = np.where(active, updated, 0.0)

        c_px = surface.slip_stiffness_per_load * total_load_n / (2.0 * half_length_m ** 2)
        element_stiffness_n_m = c_px * dx_m
        trial_force_n = element_stiffness_n_m * updated
        friction_limit_n = mu * loads
        force_n = np.clip(trial_force_n, -friction_limit_n, friction_limit_n)
        sliding = active & (np.abs(trial_force_n) >= friction_limit_n)
        fully_sliding = bool(active.any() and np.all(sliding[active]))

        if element_stiffness_n_m > 0.0 and sliding.any():
            updated[sliding] = force_n[sliding] / element_stiffness_n_m
        self.bristle_deflection_m[:] = updated

        return BrushResult(float(np.sum(force_n)), fully_sliding, mu)


__all__ = ["BrushResult", "relax", "lumped_brush", "DiscretisedBrush"]
