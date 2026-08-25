"""
Ride-Mode Rolling Resistance.

Applies a resistive torque to each wheel-spin coordinate proportional to that wheel's
*instantaneous* normal contact load, with `Crr = 0.015` (docs/RIDE.md section 4).

The load-dependence is the whole point. The wheel-spin joints already carry constant MJCF
damping, which is a fixed loss and therefore cannot represent a resistance that doubles in
a G-out and disappears over a crest, so the load-proportional part is applied explicitly
here from `mj_contactForce`.

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
    ) -> None:
        """
        Args:
            model: Compiled ride-mode model.
            crr: Coefficient of rolling resistance.
            taper_radps: Wheel speed over which the torque ramps in from zero.

        Raises:
            ValueError: If `crr` is negative or the taper band is not positive.
        """
        if crr < 0.0:
            raise ValueError(f"crr must be non-negative, got {crr}")
        if taper_radps <= 0.0:
            raise ValueError(f"taper_radps must be positive, got {taper_radps}")

        self.crr = float(crr)
        self.taper_radps = float(taper_radps)
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
            contacts: This step's contact snapshot, supplying each wheel's normal load.
        """
        self.front_torque_nm = self._wheel_torque(data, self.front_wheel, contacts.front_load_n)
        self.rear_torque_nm = self._wheel_torque(data, self.rear_wheel, contacts.rear_load_n)

        data.qfrc_applied[self.front_wheel.dofadr] = self.front_torque_nm
        data.qfrc_applied[self.rear_wheel.dofadr] = self.rear_torque_nm

    def _wheel_torque(self, data: mujoco.MjData, wheel: WheelSpin, load_n: float) -> float:
        """
        Computes one wheel's rolling-resistance torque.

        Args:
            data: Simulation state.
            wheel: Wheel-spin handle.
            load_n: Normal contact load on that wheel, in newtons.

        Returns:
            Generalized torque in N.m, opposing the wheel's rotation. An airborne wheel
            carries no load and therefore no resistance.
        """
        return opposing_torque(
            magnitude_nm=self.crr * max(load_n, 0.0) * wheel.radius_m,
            omega_radps=wheel.omega_radps(data),
            taper_radps=self.taper_radps,
        )

    def reset(self) -> None:
        """Clears the last reported torques, so a fresh run does not read a stale pair."""
        self.front_torque_nm = 0.0
        self.rear_torque_nm = 0.0


__all__ = ["RollingResistance", "CRR", "ROLLING_TAPER_RADPS"]
