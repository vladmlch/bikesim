"""
Ride-Mode Cruise Control.

A PI regulator on **chassis longitudinal velocity** (`qvel[root_x]`), producing the rear
wheel torque the `rear_drive` actuator is driven with, clamped to +/-150 N.m
(docs/RIDE.md section 6).

**Not on wheel angular velocity.** Regulating wheel speed would command torque into a wheel
that is airborne or slipping. With nothing to react against, the loop would see no speed
response, keep integrating, spin the wheel up without bound, and produce a violent
re-engagement the moment the tyre touched down. Closing on chassis speed makes the loop
blind to wheel spin-up by construction; the airborne case is then handled by gating the
torque off entirely while the rear wheel carries no load.

Torque is allowed to go negative, which is the motor holding the target speed on a descent
rather than a brake: the `rear_drive` actuator's `ctrlrange` is two-sided for exactly that.
The wheel brakes are a separate, sign-aware path (`braking.py`).
"""

import mujoco

from bike_sim.sim.ride.contacts import TerrainContacts

# docs/RIDE.md section 6: the default target and the adjustable band around it.
DEFAULT_TARGET_SPEED_KMH = 25.0
MIN_TARGET_SPEED_KMH = 15.0
MAX_TARGET_SPEED_KMH = 45.0

KMH_PER_MPS = 3.6

# Rear-wheel drive torque ceiling, matching the `rear_drive` actuator's ctrlrange.
DRIVE_TORQUE_CEILING_NM = 150.0

# Gains, in N.m per (m/s) and N.m per (m/s . s). The plant is the whole 104.35 kg system
# pushed through the 0.352 m rear wheel: a torque T accelerates it at T / M with
# M = 36.7 kg.m. That makes the closed loop second order, with omega_n = sqrt(ki / M) and
# zeta = kp / (2 sqrt(ki M)), so the pair below puts it at 2.0 rad/s and zeta = 1.21 --
# overdamped, settling inside 3 s.
#
# The gains are deliberately stiff enough that the proportional term alone nearly holds
# speed: the ~6 N.m needed against rolling resistance and joint damping costs only 0.03 m/s
# of droop, which leaves the integrator almost nothing to accumulate. A softer pair (kp = 60,
# ki = 20) is equally stable on paper but has omega_n = 0.73 rad/s, and a 40 m track is then
# over before the integrator has finished unwinding the run-up -- measured as a 3 % speed
# overshoot still present at the finish line.
DEFAULT_KP_NM_PER_MPS = 180.0
DEFAULT_KI_NM_PER_MPS_S = 150.0


class CruiseController:
    """
    PI speed regulator producing a rear-wheel drive torque.

    Integrator wind-up is handled in two places: the accumulator is clamped so that its own
    contribution can never exceed the torque ceiling, and integration is suspended whenever
    the output is saturated in the direction the error would push it further, or whenever
    the torque is gated off because the rear wheel is airborne.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        target_speed_kmh: float = DEFAULT_TARGET_SPEED_KMH,
        kp_nm_per_mps: float = DEFAULT_KP_NM_PER_MPS,
        ki_nm_per_mps_s: float = DEFAULT_KI_NM_PER_MPS_S,
        torque_ceiling_nm: float = DRIVE_TORQUE_CEILING_NM,
    ) -> None:
        """
        Args:
            model: Compiled ride-mode model, used to resolve the `root_x` coordinate.
            target_speed_kmh: Initial cruise target, inside the 15-45 km/h band.
            kp_nm_per_mps: Proportional gain.
            ki_nm_per_mps_s: Integral gain.
            torque_ceiling_nm: Symmetric output clamp, in N.m at the rear wheel.

        Raises:
            ValueError: If a gain or the ceiling is not positive, if `root_x` is missing, or
                if the target speed is outside the adjustable band.
        """
        if kp_nm_per_mps <= 0.0 or ki_nm_per_mps_s <= 0.0:
            raise ValueError(
                f"cruise gains must be positive, got kp={kp_nm_per_mps}, ki={ki_nm_per_mps_s}"
            )
        if torque_ceiling_nm <= 0.0:
            raise ValueError(f"torque_ceiling_nm must be positive, got {torque_ceiling_nm}")

        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
        if jid < 0:
            raise ValueError("model has no joint 'root_x'; cruise control regulates chassis speed")
        self.root_x_dofadr = int(model.jnt_dofadr[jid])

        self.kp_nm_per_mps = float(kp_nm_per_mps)
        self.ki_nm_per_mps_s = float(ki_nm_per_mps_s)
        self.torque_ceiling_nm = float(torque_ceiling_nm)
        self.integral_mps_s = 0.0
        self.torque_nm = 0.0
        self.engaged = False
        # Assist compensation: a mid-drive multiplies whatever the rider's legs do by
        # (1 + support), which multiplies the loop gain by the same factor and would move
        # Turbo from zeta = 1.21 to 0.57. Scaling the gains by 1 / (1 + support) keeps the
        # closed loop identical across assist modes, so a mode comparison measures the
        # drivetrain and not the regulator. Ride mode sets this every step, because the
        # cutoff taper changes the support that is actually in force.
        self.gain_scale = 1.0

        self._target_speed_mps = 0.0
        self.target_speed_kmh = target_speed_kmh

    @property
    def target_speed_kmh(self) -> float:
        """Cruise target in km/h."""
        return self._target_speed_mps * KMH_PER_MPS

    @target_speed_kmh.setter
    def target_speed_kmh(self, value: float) -> None:
        """
        Sets the cruise target.

        Args:
            value: New target in km/h.

        Raises:
            ValueError: If the target is outside the 15-45 km/h band of section 6. The band
                is the range over which the shipped track's kicker works; a target outside
                it is a mis-specified experiment, not a clamp to apply silently.
        """
        if not MIN_TARGET_SPEED_KMH <= value <= MAX_TARGET_SPEED_KMH:
            raise ValueError(
                f"target speed {value:.1f} km/h is outside the adjustable band "
                f"[{MIN_TARGET_SPEED_KMH:.0f}, {MAX_TARGET_SPEED_KMH:.0f}] km/h"
            )
        self._target_speed_mps = float(value) / KMH_PER_MPS

    @property
    def target_speed_mps(self) -> float:
        """Cruise target in m/s."""
        return self._target_speed_mps

    def compute(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        contacts: TerrainContacts,
        traction_limited: bool = False,
    ) -> float:
        """
        Computes the rear-wheel drive torque for the current state.

        Args:
            model: Compiled ride-mode model, read for the integration timestep.
            data: Simulation state, read for the chassis longitudinal velocity.
            contacts: This step's contact snapshot; the torque is gated off while the rear
                wheel carries no load.
            traction_limited: The pneumatic rear patch is fully sliding in the direction of
                the speed error; suspend integration even when the PI torque is not saturated.

        Returns:
            Drive torque in N.m for the `rear_drive` actuator, within +/- the ceiling.
        """
        self.engaged = contacts.rear_in_contact
        error_mps = self._target_speed_mps - float(data.qvel[self.root_x_dofadr])

        if not self.engaged:
            # A gated step contributes nothing to the integral: accumulating error while the
            # torque cannot be delivered is precisely the wind-up that lands violently.
            self.torque_nm = 0.0
            return self.torque_nm

        kp = self.kp_nm_per_mps * self.gain_scale
        ki = self.ki_nm_per_mps_s * self.gain_scale
        proportional_nm = kp * error_mps
        demand_nm = proportional_nm + ki * self.integral_mps_s
        pushing_further_into_saturation = (
            abs(demand_nm) >= self.torque_ceiling_nm and (demand_nm > 0.0) == (error_mps > 0.0)
        )
        if not pushing_further_into_saturation and not traction_limited:
            self.integral_mps_s = _clamp(
                self.integral_mps_s + error_mps * float(model.opt.timestep),
                self.torque_ceiling_nm / ki,
            )
            demand_nm = proportional_nm + ki * self.integral_mps_s

        self.torque_nm = _clamp(demand_nm, self.torque_ceiling_nm)
        return self.torque_nm

    def set_assist_compensation(self, support_factor: float) -> float:
        """
        Scales the gains for the assist currently in force.

        Args:
            support_factor: Motor torque as a multiple of rider torque, after the cutoff
                taper. Zero leaves the gains as authored.

        Returns:
            The scale now in force.
        """
        self.gain_scale = 1.0 / (1.0 + max(0.0, float(support_factor)))
        return self.gain_scale

    def reset(self) -> None:
        """Clears the integrator and the last reported torque for a fresh run."""
        self.integral_mps_s = 0.0
        self.torque_nm = 0.0
        self.engaged = False


def _clamp(value: float, limit: float) -> float:
    """Clamps a value symmetrically into [-limit, +limit]."""
    return max(-limit, min(limit, value))


__all__ = [
    "CruiseController",
    "DEFAULT_TARGET_SPEED_KMH",
    "MIN_TARGET_SPEED_KMH",
    "MAX_TARGET_SPEED_KMH",
    "DRIVE_TORQUE_CEILING_NM",
]
