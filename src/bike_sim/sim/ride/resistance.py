"""
Ride-Mode Rolling Resistance.

Applies a resistive torque to each wheel-spin coordinate proportional to that wheel's
*instantaneous* normal contact load, with `Crr = 0.015` (docs/RIDE.md section 4).

The load-dependence is the whole point. The wheel-spin joints already carry constant MJCF
damping, which is a fixed loss and therefore cannot represent a resistance that doubles in
a G-out and disappears over a crest, so the load-proportional part is applied explicitly
here from `mj_contactForce`.

**The load is `TerrainContacts.*_support_n`: raw, and vertical.** Not the bridged gating
channel, which averages 111.7 % of system weight against the raw 100.2 % and would make this
sink -- which section 8 reports on its own line -- systematically 11 % high on every run, and
would keep applying `Crr . N_last . r` through the first 5 ms of every genuine take-off. See
`contacts.py` for why there are two channels.

**And it is capped**, at `LOAD_CEILING_WEIGHTS` times the system's static weight, because
`Crr . N` is a quasi-static loss model and a constraint solver hands it single-timestep
impact spikes that are not loads a tyre carcass ever sees. See `LOAD_CEILING_WEIGHTS`.

**No aerodynamic term.** Section 4 omits drag deliberately: at 25 km/h that is roughly 13 N
and 90 W of missing loss, which makes absolute wheel power about a quarter low and coast-down
optimistic, and which is identical across runs so that comparisons between suspension
settings are unaffected.

`qfrc_applied` is never cleared by MuJoCo, so both wheel torques are assigned -- not
accumulated -- on every step, including the steps where they are zero. The two wheel-spin
DOFs have no other `qfrc_applied` writer in ride mode, which is what makes assignment
correct here.
"""

import mujoco

from bike_sim.sim.ride.contacts import TerrainContacts
from bike_sim.sim.ride.wheels import (
    WheelSpin,
    opposing_torque,
    resolve_wheel_spin,
)

# Coefficient of rolling resistance, docs/RIDE.md section 4.
CRR = 0.015

# Wheel speed over which the resistance ramps in from zero. Narrower than the brake taper
# because rolling resistance is a small torque -- 2.8 N.m at the solved static rear contact
# load of 525 N -- that only needs to stop flipping sign about zero, not to stage a stop.
ROLLING_TAPER_RADPS = 0.2

# Cap on the load fed to `Crr . N . r`, as a multiple of the system's static weight (1023.7 N
# at the shipped masses). Measured over the `single_edge` traverse, the vertical support load
# per wheel has a 99.9th percentile of 1.73 x weight at the front and 1.75 x at the rear, but
# a maximum of 11.6 x and 16.0 x -- single-timestep constraint-solver impact spikes at the
# square edge. Fed straight through, those gave the channel peaks of 66 and 87 N.m: a third of
# the *brake* ceiling, under the label "rolling resistance".
#
# Vector-summing the rows instead of adding their magnitudes does not fix that. On this
# traverse the two agree to the digit, so the peak is a solver transient and not a
# differing-normals artifact; the projection is still the right summation, but the cap is what
# refuses the spike. Three weights is chosen because it clears the 99.9th percentile by 1.7x,
# so ordinary riding never reaches it; it is well above the doubling section 4 names for a
# G-out, so every load-dependence the spec cares about survives; and it holds the channel at
# 17.1 N.m front and 16.2 N.m rear. It clips 13 of 12 749 steps, 0.10 % of the traverse.
LOAD_CEILING_WEIGHTS = 3.0


class RollingResistance:
    """
    Writes the load-proportional rolling-resistance torque to both wheel-spin DOFs.

    Keeps the last torque and load per wheel so telemetry can report the loss without
    recomputing it.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        crr: float = CRR,
        taper_radps: float = ROLLING_TAPER_RADPS,
        load_ceiling_weights: float = LOAD_CEILING_WEIGHTS,
    ) -> None:
        """
        Args:
            model: Compiled ride-mode model.
            crr: Coefficient of rolling resistance.
            taper_radps: Wheel speed over which the torque ramps in from zero.
            load_ceiling_weights: Cap on the per-wheel load, as a multiple of the system's
                static weight, which is read from the model's own masses and gravity.

        Raises:
            ValueError: If `crr` is negative, the taper band is not positive, or the load
                ceiling is not positive.
        """
        if crr < 0.0:
            raise ValueError(f"crr must be non-negative, got {crr}")
        if taper_radps <= 0.0:
            raise ValueError(f"taper_radps must be positive, got {taper_radps}")
        if load_ceiling_weights <= 0.0:
            raise ValueError(f"load_ceiling_weights must be positive, got {load_ceiling_weights}")

        self.crr = float(crr)
        self.taper_radps = float(taper_radps)
        self.system_weight_n = float(model.body_mass.sum()) * abs(float(model.opt.gravity[2]))
        self.load_ceiling_n = float(load_ceiling_weights) * self.system_weight_n
        self.front_wheel: WheelSpin = resolve_wheel_spin(model, "front_wheel_spin", "geom_front_contact")
        self.rear_wheel: WheelSpin = resolve_wheel_spin(model, "rear_wheel_spin", "geom_rear_contact")

        self.front_torque_nm = 0.0
        self.rear_torque_nm = 0.0

    def apply(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        contacts: TerrainContacts,
    ) -> None:
        """
        Computes both rolling-resistance torques and writes them into `qfrc_applied`.

        Args:
            model: Compiled ride-mode model. Unused -- the addresses were cached at
                construction -- but kept so every ride-mode force writer shares one
                signature.
            data: Simulation state, written at the two wheel-spin DOFs.
            contacts: This step's contact snapshot. The *support* channel is read, not the
                bridged gating channel; see the module docstring.
        """
        self.front_torque_nm = self._wheel_torque(data, self.front_wheel, contacts.front_support_n)
        self.rear_torque_nm = self._wheel_torque(data, self.rear_wheel, contacts.rear_support_n)

        data.qfrc_applied[self.front_wheel.dofadr] = self.front_torque_nm
        data.qfrc_applied[self.rear_wheel.dofadr] = self.rear_torque_nm

    def _wheel_torque(self, data: mujoco.MjData, wheel: WheelSpin, support_n: float) -> float:
        """
        Computes one wheel's rolling-resistance torque.

        Args:
            data: Simulation state.
            wheel: Wheel-spin handle.
            support_n: Vertical normal load pressing that wheel into the road, in newtons.

        Returns:
            Generalized torque in N.m, opposing the wheel's rotation. An airborne wheel
            carries no load and therefore no resistance; a wheel taking a solver impact spike
            is charged at the load ceiling rather than at the spike.
        """
        load_n = min(max(support_n, 0.0), self.load_ceiling_n)
        return opposing_torque(
            magnitude_nm=self.crr * load_n * wheel.radius_m,
            omega_radps=wheel.omega_radps(data),
            taper_radps=self.taper_radps,
        )

    def reset(self) -> None:
        """Clears the last reported torques, so a fresh run does not read a stale pair."""
        self.front_torque_nm = 0.0
        self.rear_torque_nm = 0.0


__all__ = ["RollingResistance", "CRR", "ROLLING_TAPER_RADPS", "LOAD_CEILING_WEIGHTS"]
