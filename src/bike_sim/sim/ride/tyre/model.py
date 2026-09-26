"""One pneumatic tyre: radial carcass, brush slip and per-patch force outputs.

This module composes the pure geometry, carcass and brush kernels.  It receives world-frame
wheel kinematics and a road profile, but does not read or write MuJoCo state; only
``TyreForceApplier`` translates its patch forces into ``xfrc_applied``.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from bike_sim.physics.tyre import (
    CRR_PRESSURE_EXPONENT,
    CRR_REFERENCE_PRESSURE_BAR,
    TierSpec,
    TyreSpecs,
    clamp_pressure_bar,
)
from bike_sim.sim.ride.tyre.brush import BrushResult, DiscretisedBrush, lumped_brush, relax
from bike_sim.sim.ride.tyre.carcass import CarcassState, RimStrikeEvent, evaluate_carcass
from bike_sim.sim.ride.tyre.geometry import RayRing, RoadProfile, intersect
from bike_sim.terrain.surface import SurfaceMap, SurfaceSpec

_RAD = np.pi / 180.0
_ROLLING_LOSS_TAPER_RADPS = 1.0


@dataclass(frozen=True)
class TyrePatchOutput:
    """Normal and tangential force data for one contiguous contact patch."""

    ray_start: int
    ray_stop: int
    surface_name: str
    normal_load_n: float
    support_n: float
    tangential_force_n: float
    normal_force_world_n: np.ndarray
    tangent_world: np.ndarray
    force_world_n: np.ndarray
    centroid_world_m: np.ndarray
    mean_deflection_m: float
    contact_length_m: float
    slip_ratio: float
    transient_slip_ratio: float
    sliding_speed_mps: float
    friction_coefficient: float
    fully_sliding: bool
    dissipated_power_w: float


@dataclass(frozen=True)
class WheelOutputs:
    """Last computed per-wheel tyre result for force writers and telemetry."""

    tyre_name: str
    pressure_bar: float
    hub_position_world_m: np.ndarray
    hub_velocity_world_mps: np.ndarray
    omega_forward_radps: float
    force_world_n: np.ndarray
    normal_load_n: float
    support_n: float
    mean_deflection_m: float
    contact_length_m: float
    slip_ratio: float
    transient_slip_ratio: float
    fully_sliding: bool
    rim_strike_active: bool
    rim_event: Optional[RimStrikeEvent]
    dissipated_power_w: float
    coverage_event: bool
    airborne: bool
    patches: Tuple[TyrePatchOutput, ...]

    @classmethod
    def empty(cls, tyre: TyreSpecs) -> "WheelOutputs":
        """Builds a zero-load snapshot for a reset or an airborne wheel."""
        return cls(
            tyre_name=tyre.name,
            pressure_bar=tyre.pressure_bar,
            hub_position_world_m=np.zeros(3, dtype=float),
            hub_velocity_world_mps=np.zeros(3, dtype=float),
            omega_forward_radps=0.0,
            force_world_n=np.zeros(3, dtype=float),
            normal_load_n=0.0,
            support_n=0.0,
            mean_deflection_m=0.0,
            contact_length_m=0.0,
            slip_ratio=0.0,
            transient_slip_ratio=0.0,
            fully_sliding=False,
            rim_strike_active=False,
            rim_event=None,
            dissipated_power_w=0.0,
            coverage_event=False,
            airborne=True,
            patches=(),
        )


@dataclass
class _PatchBrushState:
    """Slip memory tied to a contiguous range of fixed rays."""

    start: int
    stop: int
    kappa_prime: float = 0.0
    detailed: Optional[DiscretisedBrush] = None


class PneumaticTyre:
    """Pure per-wheel pneumatic tyre model composed from the tested sub-kernels."""

    def __init__(self, tyre: TyreSpecs, tier: TierSpec) -> None:
        self.tyre = tyre
        self.tier = tier
        self.ring = RayRing(tier.n_rays, tier.half_angle_deg * _RAD)
        self.carcass_state = CarcassState(self.ring.n)
        self._patch_states: Dict[Tuple[int, int], _PatchBrushState] = {}
        self.last_outputs = WheelOutputs.empty(tyre)

    def reset(self) -> None:
        """Clears carcass, bristle and per-patch slip history."""
        self.carcass_state.reset()
        self._patch_states.clear()
        self.last_outputs = WheelOutputs.empty(self.tyre)

    def set_pressure(self, pressure_bar: float) -> None:
        """Updates the live pressure within the viewer's allowed range."""
        self.tyre = self.tyre.with_pressure(clamp_pressure_bar(pressure_bar))

    def evaluate(
        self,
        hub_position_world_m: np.ndarray,
        hub_velocity_world_mps: np.ndarray,
        omega_forward_radps: float,
        road: RoadProfile,
        surface_map: SurfaceMap,
        dt_s: float,
    ) -> WheelOutputs:
        """Evaluates the wheel's radial and longitudinal contact for one timestep.

        Args:
            hub_position_world_m: Wheel-axis centre in world coordinates.
            hub_velocity_world_mps: Absolute hub linear velocity in world coordinates.
            omega_forward_radps: Absolute wheel spin, positive for forward rolling.
            road: The same sampled profile used to build the rendered heightfield.
            surface_map: Surface lookup by track X.
            dt_s: Positive simulation timestep.

        Returns:
            Immutable wheel outputs with one record per loaded patch.
        """
        hub = np.asarray(hub_position_world_m, dtype=float)
        velocity = np.asarray(hub_velocity_world_mps, dtype=float)
        if hub.shape != (3,) or velocity.shape != (3,):
            raise ValueError("hub position and velocity must be 3-vectors in world axes")
        if not np.all(np.isfinite(hub)) or not np.all(np.isfinite(velocity)):
            raise ValueError("hub position and velocity must be finite")
        if not np.isfinite(omega_forward_radps):
            raise ValueError("wheel spin must be finite")

        hits = intersect(
            self.ring,
            float(hub[0]),
            float(hub[2]),
            self.tyre.outer_radius_mm / 1000.0,
            road,
        )
        speed_mps = float(np.hypot(velocity[0], velocity[2]))
        carcass = evaluate_carcass(
            self.ring,
            hits,
            self.tyre,
            self.carcass_state,
            dt_s=dt_s,
            omega_radps=float(omega_forward_radps),
            speed_mps=speed_mps,
        )

        if not carcass.contact_patches:
            self._patch_states.clear()
            outputs = WheelOutputs(
                tyre_name=self.tyre.name,
                pressure_bar=self.tyre.pressure_bar,
                hub_position_world_m=hub.copy(),
                hub_velocity_world_mps=velocity.copy(),
                omega_forward_radps=float(omega_forward_radps),
                force_world_n=np.zeros(3, dtype=float),
                normal_load_n=0.0,
                support_n=0.0,
                mean_deflection_m=0.0,
                contact_length_m=0.0,
                slip_ratio=0.0,
                transient_slip_ratio=0.0,
                fully_sliding=False,
                rim_strike_active=carcass.rim_strike_active,
                rim_event=carcass.rim_event,
                dissipated_power_w=0.0,
                coverage_event=hits.coverage_event,
                airborne=hits.airborne,
                patches=(),
            )
            self.last_outputs = outputs
            return outputs

        patch_states = self._states_for_patches(carcass.contact_patches)
        patch_outputs = []
        total_force_world_n = np.zeros(3, dtype=float)
        total_load_n = 0.0
        total_support_n = 0.0
        weighted_deflection_m = 0.0
        weighted_kappa = 0.0
        weighted_kappa_prime = 0.0
        total_contact_length_m = 0.0
        brush_loss_w = 0.0

        # Carcass hysteresis dissipates energy on compression and rebound.  The sign product
        # is nonnegative by construction; clip roundoff at zero before telemetry sees it.
        carcass_loss_w = float(np.sum(np.maximum(
            carcass.hysteresis_force_n * carcass.material_rate_mps, 0.0
        )))

        for (start, stop), contact_length_m in zip(
            carcass.contact_patches, carcass.contact_lengths_m
        ):
            sl = slice(start, stop)
            element_force_n = carcass.total_element_force_n[sl]
            if not np.any(element_force_n > 0.0):
                continue

            # Each element force points from the road point towards the wheel hub.
            normal_force_world_n = np.asarray([
                -float(np.dot(element_force_n, self.ring.ux[sl])),
                0.0,
                -float(np.dot(element_force_n, self.ring.uz[sl])),
            ])
            normal_load_n = float(np.linalg.norm(normal_force_world_n))
            if normal_load_n <= 0.0:
                continue
            normal_world = normal_force_world_n / normal_load_n
            tangent_world = np.asarray([normal_world[2], 0.0, -normal_world[0]])
            weights = element_force_n
            weight_sum = float(np.sum(weights))
            centroid_world_m = np.asarray([
                float(np.dot(weights, hits.road_x_m[sl]) / weight_sum),
                float(hub[1]),
                float(np.dot(weights, hits.road_z_m[sl]) / weight_sum),
            ])
            mean_deflection_m = float(np.dot(weights, hits.delta_m[sl]) / weight_sum)
            half_length_m = 0.5 * float(contact_length_m)

            v_x_mps = float(np.dot(velocity, tangent_world))
            effective_radius_m = max(
                self.tyre.outer_radius_mm / 1000.0 - mean_deflection_m / 3.0,
                1e-6,
            )
            v_s_mps = v_x_mps - float(omega_forward_radps) * effective_radius_m
            slip_ratio = -v_s_mps / abs(v_x_mps) if abs(v_x_mps) > 1e-9 else 0.0
            surface = surface_map.at(float(centroid_world_m[0]))
            state = patch_states[(start, stop)]
            if self.tier.discretised_brush:
                if state.detailed is None:
                    state.detailed = DiscretisedBrush(stop - start)
                    state.detailed.kappa_prime = state.kappa_prime
                brush_result = state.detailed.step(
                    element_force_n,
                    half_length_m=half_length_m,
                    v_x_mps=v_x_mps,
                    v_s_mps=v_s_mps,
                    sigma_m=self.tyre.relaxation_length_mm / 1000.0,
                    surface=surface,
                    dt_s=dt_s,
                )
                state.kappa_prime = state.detailed.kappa_prime
            else:
                state.kappa_prime = relax(
                    state.kappa_prime,
                    v_x_mps,
                    v_s_mps,
                    self.tyre.relaxation_length_mm / 1000.0,
                    dt_s,
                )
                brush_result = lumped_brush(
                    state.kappa_prime,
                    normal_load_n,
                    half_length_m,
                    surface,
                    sliding_speed_mps=abs(v_s_mps),
                )

            tread_loss_force_n = _tread_loss_force(
                _tread_loss_crr_at_pressure(self.tyre),
                normal_load_n,
                omega_forward_radps,
                v_x_mps,
            )
            tangential_force_n = brush_result.force_n + tread_loss_force_n
            patch_force_world_n = normal_force_world_n + tangential_force_n * tangent_world
            support_n = float(normal_force_world_n[2])
            rolling_loss_w = max(0.0, -tread_loss_force_n * v_x_mps)
            slip_loss_w = max(0.0, -brush_result.force_n * v_s_mps)
            patch_loss_w = rolling_loss_w + slip_loss_w
            brush_loss_w += patch_loss_w

            patch_output = TyrePatchOutput(
                ray_start=start,
                ray_stop=stop,
                surface_name=surface.name,
                normal_load_n=normal_load_n,
                support_n=support_n,
                tangential_force_n=tangential_force_n,
                normal_force_world_n=normal_force_world_n,
                tangent_world=tangent_world,
                force_world_n=patch_force_world_n,
                centroid_world_m=centroid_world_m,
                mean_deflection_m=mean_deflection_m,
                contact_length_m=float(contact_length_m),
                slip_ratio=float(slip_ratio),
                transient_slip_ratio=float(state.kappa_prime),
                sliding_speed_mps=abs(float(v_s_mps)),
                friction_coefficient=brush_result.friction_coefficient,
                fully_sliding=brush_result.fully_sliding,
                dissipated_power_w=patch_loss_w,
            )
            patch_outputs.append(patch_output)
            total_force_world_n += patch_force_world_n
            total_load_n += normal_load_n
            total_support_n += support_n
            weighted_deflection_m += mean_deflection_m * normal_load_n
            weighted_kappa += slip_ratio * normal_load_n
            weighted_kappa_prime += state.kappa_prime * normal_load_n
            total_contact_length_m += float(contact_length_m)

        if total_load_n > 0.0:
            mean_deflection_m = weighted_deflection_m / total_load_n
            slip_ratio = weighted_kappa / total_load_n
            transient_slip_ratio = weighted_kappa_prime / total_load_n
        else:
            mean_deflection_m = slip_ratio = transient_slip_ratio = 0.0

        outputs = WheelOutputs(
            tyre_name=self.tyre.name,
            pressure_bar=self.tyre.pressure_bar,
            hub_position_world_m=hub.copy(),
            hub_velocity_world_mps=velocity.copy(),
            omega_forward_radps=float(omega_forward_radps),
            force_world_n=total_force_world_n,
            normal_load_n=total_load_n,
            support_n=total_support_n,
            mean_deflection_m=mean_deflection_m,
            contact_length_m=total_contact_length_m,
            slip_ratio=slip_ratio,
            transient_slip_ratio=transient_slip_ratio,
            fully_sliding=any(patch.fully_sliding for patch in patch_outputs),
            rim_strike_active=carcass.rim_strike_active,
            rim_event=carcass.rim_event,
            dissipated_power_w=carcass_loss_w + brush_loss_w,
            coverage_event=hits.coverage_event,
            airborne=False,
            patches=tuple(patch_outputs),
        )
        self.last_outputs = outputs
        return outputs

    def _states_for_patches(
        self,
        patches: Tuple[Tuple[int, int], ...],
    ) -> Dict[Tuple[int, int], _PatchBrushState]:
        """Reuses slip state across small patch-boundary shifts and copies ray overlap."""
        old_states = self._patch_states
        used: set[Tuple[int, int]] = set()
        new_states: Dict[Tuple[int, int], _PatchBrushState] = {}
        for start, stop in patches:
            key = (start, stop)
            old_key = key if key in old_states and key not in used else None
            if old_key is None:
                overlaps = [
                    (min(stop, old_stop) - max(start, old_start), old_key_candidate)
                    for old_key_candidate, state in old_states.items()
                    for old_start, old_stop in [old_key_candidate]
                    if old_key_candidate not in used
                ]
                overlaps = [(count, candidate) for count, candidate in overlaps if count > 0]
                if overlaps:
                    old_key = max(overlaps)[1]

            if old_key is None:
                state = _PatchBrushState(start, stop)
                if self.tier.discretised_brush:
                    state.detailed = DiscretisedBrush(stop - start)
            else:
                used.add(old_key)
                previous = old_states[old_key]
                state = _PatchBrushState(start, stop, kappa_prime=previous.kappa_prime)
                if self.tier.discretised_brush:
                    state.detailed = DiscretisedBrush(stop - start)
                    if previous.detailed is not None:
                        state.detailed.kappa_prime = previous.detailed.kappa_prime
                        overlap_start = max(start, previous.start)
                        overlap_stop = min(stop, previous.stop)
                        state.detailed.bristle_deflection_m[
                            overlap_start - start:overlap_stop - start
                        ] = previous.detailed.bristle_deflection_m[
                            overlap_start - previous.start:overlap_stop - previous.start
                        ]
            new_states[key] = state
        self._patch_states = new_states
        return new_states


def _tread_loss_force(crr: float, normal_load_n: float,
                      omega_forward_radps: float, v_x_mps: float) -> float:
    """Optional Crr shortfall as a tangential force, tapered to zero at wheel rest."""
    if crr <= 0.0 or normal_load_n <= 0.0:
        return 0.0
    direction = float(np.sign(v_x_mps))
    if direction == 0.0:
        direction = float(np.sign(omega_forward_radps))
    if direction == 0.0:
        return 0.0
    taper = min(abs(float(omega_forward_radps)) / _ROLLING_LOSS_TAPER_RADPS, 1.0)
    return -float(crr) * float(normal_load_n) * taper * direction


def _tread_loss_crr_at_pressure(tyre: TyreSpecs) -> float:
    """Scales the reference-pressure Crr shortfall with the authored pressure law."""
    return tyre.tread_loss_crr * (
        tyre.pressure_bar / CRR_REFERENCE_PRESSURE_BAR
    ) ** CRR_PRESSURE_EXPONENT


__all__ = ["TyrePatchOutput", "WheelOutputs", "PneumaticTyre"]
