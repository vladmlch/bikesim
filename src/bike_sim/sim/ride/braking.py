"""
Ride-Mode Brake Torque.

Turns a per-wheel demand in [0, 1] into the motor torque the `front_brake` and `rear_brake`
actuators are driven with, at a ceiling of 200 N.m per wheel (docs/RIDE.md section 6).

The two actuators carry a two-sided `ctrlrange` precisely because the sign is computed
here: the torque opposes the wheel's current rotation and tapers to zero as the wheel stops,
so a held brake at a standstill does nothing instead of reversing the bike.
"""

from typing import Tuple

import mujoco
from bike_sim.physics.checks import derived, scalar
from bike_sim.sim.ride.wheels import validate_actuator_target

from bike_sim.sim.ride.wheels import (
    WheelSpin,
    opposing_torque,
    resolve_wheel_spin,
)

# Per-wheel ceiling from docs/RIDE.md section 6, and the two-sided actuator ctrlrange.
BRAKE_TORQUE_CEILING_NM = 200.0

# Wheel speed below which the brake fades out. 1 rad/s is 0.35 m/s at either wheel: slow
# enough that the fade is invisible in a braking event, wide enough that the last part of a
# stop settles instead of chattering about zero.
BRAKE_TAPER_RADPS = 1.0


class BrakeController:
    """
    Computes sign-aware front and rear brake torques from a demand per wheel.

    The controller is stateless between steps apart from the last torques it reported, which
    it keeps so telemetry and the HUD can read them without recomputing.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        torque_ceiling_nm: float = BRAKE_TORQUE_CEILING_NM,
        taper_radps: float = BRAKE_TAPER_RADPS,
    ) -> None:
        """
        Args:
            model: Compiled ride-mode model.
            torque_ceiling_nm: Torque at full demand, in N.m per wheel.
            taper_radps: Wheel speed over which the torque ramps in from zero.

        Raises:
            ValueError: If the ceiling is negative or the taper band is not positive.
        """
        scalar(torque_ceiling_nm, "BrakeController.torque_ceiling_nm", minimum=0.)
        scalar(taper_radps, "BrakeController.taper_radps", positive=True)
        if torque_ceiling_nm < 0.0:
            raise ValueError(f"torque_ceiling_nm must be non-negative, got {torque_ceiling_nm}")
        if taper_radps <= 0.0:
            raise ValueError(f"taper_radps must be positive, got {taper_radps}")

        self.torque_ceiling_nm = float(torque_ceiling_nm)
        self.taper_radps = float(taper_radps)
        self.front_wheel: WheelSpin = resolve_wheel_spin(model, "front_wheel_spin", "geom_front_contact")
        self.rear_wheel: WheelSpin = resolve_wheel_spin(model, "rear_wheel_spin", "geom_rear_contact")

        validate_actuator_target(model, "front_brake", "front_wheel_spin")
        validate_actuator_target(model, "rear_brake", "rear_wheel_spin")
        self.front_torque_nm = 0.0
        self.rear_torque_nm = 0.0

    def compute(
        self,
        data: mujoco.MjData,
        front_demand: float,
        rear_demand: float,
    ) -> Tuple[float, float]:
        """
        Computes the brake torque for each wheel at the current state.

        Args:
            data: Simulation state, read for both wheel angular velocities.
            front_demand: Front brake lever position, clamped into [0, 1].
            rear_demand: Rear brake lever position, clamped into [0, 1].

        Returns:
            Tuple of (front torque, rear torque) in N.m, ready to be written to the
            `front_brake` and `rear_brake` actuators.
        """
        front = self._wheel_torque(data, self.front_wheel, front_demand)
        rear = self._wheel_torque(data, self.rear_wheel, rear_demand)
        self.front_torque_nm, self.rear_torque_nm = front, rear
        return self.front_torque_nm, self.rear_torque_nm

    def _wheel_torque(self, data: mujoco.MjData, wheel: WheelSpin, demand: float) -> float:
        """
        Computes one wheel's brake torque.

        Args:
            data: Simulation state.
            wheel: Wheel-spin handle to brake.
            demand: Lever position, clamped into [0, 1] rather than rejected: the
                interactive brake-strength control walks up to the ends of the range.

        Returns:
            Generalized torque in N.m, opposing the wheel's rotation.
        """
        scalar(self.torque_ceiling_nm, "BrakeController.torque_ceiling_nm", minimum=0.)
        scalar(self.taper_radps, "BrakeController.taper_radps", positive=True)
        clamped = min(max(scalar(demand, "BrakeController.demand"), 0.0), 1.0)
        return opposing_torque(
            magnitude_nm=clamped * self.torque_ceiling_nm,
            omega_radps=wheel.omega_radps(data),
            taper_radps=self.taper_radps,
        )

    def reset(self) -> None:
        """Clears the last reported torques, so a fresh run does not read a stale pair."""
        self.front_torque_nm = 0.0
        self.rear_torque_nm = 0.0


__all__ = ["BrakeController", "BRAKE_TORQUE_CEILING_NM", "BRAKE_TAPER_RADPS"]
