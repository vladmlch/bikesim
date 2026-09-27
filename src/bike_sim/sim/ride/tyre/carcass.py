"""Pneumatic tyre carcass forces, material memory and rim-strike events.

The radial rays are fixed in world space.  Carcass material moves through them as the
wheel spins, so the Maxwell state is semi-Lagrangian advected and the hysteresis rate
includes the upwind material derivative from docs/RIDE.md section 3.1.  This module is a
pure NumPy kernel: it has no MuJoCo dependency and uses metres at its boundary.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from bike_sim.physics.tyre import TyreSpecs
from bike_sim.sim.ride.tyre.geometry import RayHits, RayRing, patches

_BAR_TO_PA = 100_000.0
_MM_TO_M = 0.001
_MM2_TO_M2 = 1_000_000.0


@dataclass(frozen=True)
class RimStrikeEvent:
    """Completed rim strike; energy is the peak elastic energy stored in the rim term."""

    x_m: float
    speed_mps: float
    peak_load_n: float
    peak_rim_force_n: float
    absorbed_energy_j: float


@dataclass
class CarcassState:
    """Per-ray memory and the event under way for one wheel.

    Args:
        n_rays: Number of rays in the wheel's ``RayRing``.
    """

    n_rays: int
    delta_prev_m: np.ndarray = field(init=False)
    elastic_prev_n: np.ndarray = field(init=False)
    maxwell_force_n: np.ndarray = field(init=False)
    initialized: bool = field(default=False, init=False)
    rim_strike_active: bool = field(default=False, init=False)
    rim_event_x_m: float = field(default=0.0, init=False)
    rim_event_speed_mps: float = field(default=0.0, init=False)
    rim_event_peak_load_n: float = field(default=0.0, init=False)
    rim_event_peak_force_n: float = field(default=0.0, init=False)
    rim_event_energy_j: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        if self.n_rays < 3:
            raise ValueError(f"carcass needs at least 3 rays, got {self.n_rays}")
        self.delta_prev_m = np.zeros(self.n_rays, dtype=float)
        self.elastic_prev_n = np.zeros(self.n_rays, dtype=float)
        self.maxwell_force_n = np.zeros(self.n_rays, dtype=float)

    def reset(self) -> None:
        """Clears all tyre material history and any event in progress."""
        self.delta_prev_m.fill(0.0)
        self.elastic_prev_n.fill(0.0)
        self.maxwell_force_n.fill(0.0)
        self.initialized = False
        self._clear_rim_event()

    def advance_unloaded(
        self,
        *,
        omega_radps: float,
        dt_s: float,
        dtheta_rad: float,
        relaxation_s: float,
    ) -> None:
        """Advects and relaxes Maxwell memory while already fully unloaded.

        The caller has established that every ray was unloaded on the previous step and
        there is no rim event in progress. The elastic and deflection histories are then
        zero, so the full carcass kernel can be skipped without retaining stale material
        stress when the wheel later lands.
        """
        if not self.initialized:
            return
        if dt_s <= 0.0 or dtheta_rad <= 0.0 or relaxation_s <= 0.0:
            raise ValueError("time, angular spacing and relaxation time must be positive")
        if np.any(self.maxwell_force_n):
            self.maxwell_force_n[:] = float(np.exp(-dt_s / relaxation_s)) * _advect_material(
                self.maxwell_force_n, omega_radps, dt_s, dtheta_rad
            )

    def _clear_rim_event(self) -> None:
        self.rim_strike_active = False
        self.rim_event_x_m = 0.0
        self.rim_event_speed_mps = 0.0
        self.rim_event_peak_load_n = 0.0
        self.rim_event_peak_force_n = 0.0
        self.rim_event_energy_j = 0.0


@dataclass(frozen=True)
class CarcassResult:
    """Element forces and wheel-level summaries for one tyre step.

    Force arrays follow the ray order in ``RayRing``.  ``total_element_force_n`` acts
    radially from the road point towards the hub.  ``force_x_n`` / ``force_z_n`` are its
    resultant in world axes.  ``rim_event`` is populated on the step an event closes.
    """

    pressure_force_n: np.ndarray
    elastic_force_n: np.ndarray
    maxwell_force_n: np.ndarray
    hysteresis_force_n: np.ndarray
    rim_force_n: np.ndarray
    total_element_force_n: np.ndarray
    material_rate_mps: np.ndarray
    contact_patches: Tuple[Tuple[int, int], ...]
    contact_lengths_m: Tuple[float, ...]
    force_x_n: float
    force_z_n: float
    normal_load_n: float
    rim_energy_j: float
    rim_strike_active: bool
    rim_event: Optional[RimStrikeEvent]


def material_deflection_rate_mps(
    delta_m: np.ndarray,
    delta_prev_m: np.ndarray,
    omega_radps: float,
    dt_s: float,
    dtheta_rad: float,
) -> np.ndarray:
    """Returns the upwind material rate ``Dδ/Dt`` on the fixed world-ray grid.

    Positive ``omega_radps`` means forward wheel rotation: tyre material moves towards
    decreasing θ, so the upwind gradient uses the next ray.  The patch is zero outside the
    sampled range.  This sign makes the leading half compress and the trailing half recover
    during steady forward rolling on flat ground.

    Args:
        delta_m: Current ray deflections in metres.
        delta_prev_m: Previous step's deflections at the same fixed rays.
        omega_radps: Absolute wheel spin, signed positive for forward travel.
        dt_s: Time step in seconds.
        dtheta_rad: Angular spacing between rays.

    Raises:
        ValueError: If the arrays differ in shape or a time / angle step is not positive.
    """
    delta = np.asarray(delta_m, dtype=float)
    previous = np.asarray(delta_prev_m, dtype=float)
    if delta.ndim != 1 or delta.shape != previous.shape:
        raise ValueError("current and previous deflections must be matching 1-D arrays")
    if dt_s <= 0.0 or dtheta_rad <= 0.0:
        raise ValueError("time step and angular spacing must be positive")
    if omega_radps > 0.0:
        next_ray = np.concatenate((delta[1:], [0.0]))
        gradient = (next_ray - delta) / dtheta_rad
    elif omega_radps < 0.0:
        previous_ray = np.concatenate(([0.0], delta[:-1]))
        gradient = (delta - previous_ray) / dtheta_rad
    else:
        gradient = np.zeros_like(delta)
    return (delta - previous) / dt_s - float(omega_radps) * gradient


def _advect_material(values: np.ndarray, omega_radps: float, dt_s: float,
                     dtheta_rad: float) -> np.ndarray:
    """Back-traces a material field by ``−omega*dt`` with zero outside the ray window."""
    if omega_radps == 0.0:
        return values.copy()
    source_index = np.arange(values.size, dtype=float) + omega_radps * dt_s / dtheta_rad
    return np.interp(source_index, np.arange(values.size), values, left=0.0, right=0.0)


def _contact_lengths_m(hits: RayHits, contact_patches: Tuple[Tuple[int, int], ...],
                       contact_length_factor: float, one_ray_span_m: float) -> Tuple[float, ...]:
    """Scaled Euclidean span between the first and last road point of each patch."""
    lengths = []
    for start, stop in contact_patches:
        if stop - start == 1:
            # A single loaded ray represents one finite angular cell even though its first
            # and last sampled point are the same.  Keep a nonzero brush length at sharp
            # corners so the longitudinal stiffness remains finite.
            length_m = one_ray_span_m
        else:
            dx = float(hits.road_x_m[stop - 1] - hits.road_x_m[start])
            dz = float(hits.road_z_m[stop - 1] - hits.road_z_m[start])
            length_m = float(np.hypot(dx, dz))
        lengths.append(contact_length_factor * length_m)
    return tuple(lengths)


def evaluate_carcass(
    ring: RayRing,
    hits: RayHits,
    tyre: TyreSpecs,
    state: CarcassState,
    *,
    dt_s: float,
    omega_radps: float = 0.0,
    speed_mps: float = 0.0,
) -> CarcassResult:
    """Evaluates radial carcass forces, hysteresis, rate stiffening and rim contact.

    On the first call after ``reset`` the current deflection is taken as the reference
    state.  This gives the initial pose its static force without inventing an impact from
    a zero-filled history buffer.

    Args:
        ring: The tyre's fixed world-frame radial rays.
        hits: Road intersections and per-ray deflections from ``geometry.intersect``.
        tyre: Tyre dimensions and parameters (mm and bar at this boundary).
        state: Mutable per-wheel material state.
        dt_s: Positive integration step in seconds.
        omega_radps: Absolute wheel spin, signed positive for forward rolling.
        speed_mps: Longitudinal wheel-centre speed for a rim event record.

    Returns:
        Per-ray force components, patch spans, the resultant and a completed rim event.

    Raises:
        ValueError: If the state does not match the ray ring or the inputs are invalid.
    """
    if dt_s <= 0.0 or not np.isfinite(dt_s):
        raise ValueError(f"dt_s must be positive and finite, got {dt_s}")
    if not np.isfinite(omega_radps) or not np.isfinite(speed_mps):
        raise ValueError("wheel spin and speed must be finite")
    if state.n_rays != ring.n:
        raise ValueError(f"state has {state.n_rays} rays but ring has {ring.n}")
    delta = np.asarray(hits.delta_m, dtype=float)
    if delta.shape != (ring.n,) or not np.all(np.isfinite(delta)) or np.any(delta < 0.0):
        raise ValueError("ray deflections must be finite, nonnegative and match the ring")

    radius_m = tyre.outer_radius_mm * _MM_TO_M
    casing_width_m = tyre.casing_width_mm * _MM_TO_M
    casing_radius_m = 0.5 * casing_width_m
    ds_m = radius_m * ring.dtheta_rad
    pressure_pa = tyre.pressure_bar * _BAR_TO_PA

    # The pressure term is the casing chord; once the casing is fully flattened it cannot
    # grow beyond its physical width.  The carcass term remains progressive with δ.
    chord_m = 2.0 * np.sqrt(np.maximum(delta * (2.0 * casing_radius_m - delta), 0.0))
    chord_m = np.where(delta >= casing_radius_m, casing_width_m, chord_m)
    pressure_line_n_per_m = tyre.area_factor * pressure_pa * chord_m
    carcass_line_n_per_m = tyre.carcass_stiffness_n_mm2 * _MM2_TO_M2 * delta
    pressure_force_n = pressure_line_n_per_m * ds_m
    elastic_force_n = (pressure_line_n_per_m + carcass_line_n_per_m) * ds_m

    if state.initialized:
        material_rate_mps = material_deflection_rate_mps(
            delta, state.delta_prev_m, omega_radps, dt_s, ring.dtheta_rad
        )
        elastic_prev_n = _advect_material(
            state.elastic_prev_n, omega_radps, dt_s, ring.dtheta_rad
        )
        maxwell_prev_n = _advect_material(
            state.maxwell_force_n, omega_radps, dt_s, ring.dtheta_rad
        )
        decay = float(np.exp(-dt_s / tyre.rate_relaxation_s))
        maxwell_force_n = decay * maxwell_prev_n + tyre.rate_stiffening * (
            elastic_force_n - elastic_prev_n
        )
    else:
        material_rate_mps = np.zeros(ring.n, dtype=float)
        maxwell_force_n = np.zeros(ring.n, dtype=float)

    rim_deflection_m = tyre.rim_strike_deflection_mm * _MM_TO_M
    rim_overlap_m = np.maximum(delta - rim_deflection_m, 0.0)
    rim_elastic_force_n = (
        tyre.rim_stiffness_n_mm2 * _MM2_TO_M2 * rim_overlap_m * ds_m
    )
    hysteresis_sign = np.tanh(material_rate_mps / tyre.hysteresis_rate_eps_mps)
    carcass_hysteresis_force_n = tyre.loss_factor * elastic_force_n * hysteresis_sign
    rim_hysteresis_force_n = tyre.loss_factor * rim_elastic_force_n * hysteresis_sign
    hysteresis_force_n = carcass_hysteresis_force_n + rim_hysteresis_force_n
    rim_force_n = rim_elastic_force_n + rim_hysteresis_force_n

    loaded = delta > 0.0
    total_element_force_n = np.maximum(
        elastic_force_n + maxwell_force_n + hysteresis_force_n + rim_force_n, 0.0
    )
    total_element_force_n = np.where(loaded, np.maximum(total_element_force_n, 0.0), 0.0)

    force_x_n = -float(np.dot(total_element_force_n, ring.ux))
    force_z_n = -float(np.dot(total_element_force_n, ring.uz))
    normal_load_n = float(np.sum(total_element_force_n))
    rim_energy_j = float(0.5 * np.sum(
        tyre.rim_stiffness_n_mm2 * _MM2_TO_M2 * rim_overlap_m ** 2 * ds_m
    ))

    contact_patches = tuple(patches(delta))
    contact_lengths_m = _contact_lengths_m(
        hits, contact_patches, tyre.contact_length_factor, radius_m * ring.dtheta_rad
    )
    rim_active = bool(np.any(rim_overlap_m > 0.0))
    rim_event = _update_rim_event(
        state, rim_active, hits, rim_overlap_m, rim_force_n, force_z_n,
        rim_energy_j, speed_mps,
    )

    state.delta_prev_m[:] = delta
    state.elastic_prev_n[:] = elastic_force_n
    state.maxwell_force_n[:] = maxwell_force_n
    state.initialized = True

    return CarcassResult(
        pressure_force_n=pressure_force_n,
        elastic_force_n=elastic_force_n,
        maxwell_force_n=maxwell_force_n,
        hysteresis_force_n=hysteresis_force_n,
        rim_force_n=rim_force_n,
        total_element_force_n=total_element_force_n,
        material_rate_mps=material_rate_mps,
        contact_patches=contact_patches,
        contact_lengths_m=contact_lengths_m,
        force_x_n=force_x_n,
        force_z_n=force_z_n,
        normal_load_n=normal_load_n,
        rim_energy_j=rim_energy_j,
        rim_strike_active=rim_active,
        rim_event=rim_event,
    )


def _update_rim_event(
    state: CarcassState,
    active: bool,
    hits: RayHits,
    overlap_m: np.ndarray,
    rim_force_n: np.ndarray,
    vertical_load_n: float,
    energy_j: float,
    speed_mps: float,
) -> Optional[RimStrikeEvent]:
    """Tracks an event from first engaged ray until the entire rim patch releases."""
    if active:
        if not state.rim_strike_active:
            ray = int(np.argmax(overlap_m))
            state.rim_event_x_m = float(hits.road_x_m[ray])
            state.rim_event_speed_mps = abs(float(speed_mps))
            state.rim_event_peak_load_n = 0.0
            state.rim_event_peak_force_n = 0.0
            state.rim_event_energy_j = 0.0
        state.rim_strike_active = True
        state.rim_event_peak_load_n = max(state.rim_event_peak_load_n, vertical_load_n)
        state.rim_event_peak_force_n = max(
            state.rim_event_peak_force_n, float(np.sum(rim_force_n))
        )
        state.rim_event_energy_j = max(state.rim_event_energy_j, energy_j)
        return None

    if not state.rim_strike_active:
        return None
    completed = RimStrikeEvent(
        x_m=state.rim_event_x_m,
        speed_mps=state.rim_event_speed_mps,
        peak_load_n=state.rim_event_peak_load_n,
        peak_rim_force_n=state.rim_event_peak_force_n,
        absorbed_energy_j=state.rim_event_energy_j,
    )
    state._clear_rim_event()
    return completed


__all__ = [
    "CarcassResult",
    "CarcassState",
    "RimStrikeEvent",
    "evaluate_carcass",
    "material_deflection_rate_mps",
]
