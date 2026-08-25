"""
Ride-Mode Suspension Force Path.

Writes the fork and shock forces into `qfrc_applied` on their own slide coordinates, as
docs/RIDE.md section 5 prescribes. This is the only module in the ride package that
touches `data` for the suspension: the spring and damper models themselves are pure
calculators owned by `physics/` and `sim/controllers.py`.

**No leverage ratio appears here.** By virtual work the axial shock force *is* the
generalized force on `shock_stroke`; MuJoCo propagates it through the linkage's constraint
Jacobian, which applies the instantaneous leverage ratio exactly, at the current
configuration, every step. Multiplying by an analytically computed ratio would apply it
twice.
"""

from typing import Tuple

import mujoco

from bike_sim.physics.coil_shock import CoilShock
from bike_sim.sim.controllers import SuspensionController


class SuspensionForceApplier:
    """
    Applies fork and rear shock forces to their slide coordinates in `qfrc_applied`.

    The fork force is the existing `SuspensionController` air spring plus Charger 3
    damping. The shock force is a `CoilShock` coil plus bumper -- which replaces the linear
    spring term of `SuspensionController.compute_shock_force`, and must not be combined
    with it -- plus Super Deluxe RC2T damping with HBO.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        controller: SuspensionController,
        coil_shock: CoilShock,
    ) -> None:
        self.controller = controller
        self.coil_shock = coil_shock

        self.fork_qposadr, self.fork_dofadr = self._resolve_compression_joint(model, "fork_travel")
        self.shock_qposadr, self.shock_dofadr = self._resolve_compression_joint(model, "shock_stroke")

        # Last computed force components, so telemetry can report them without recomputing.
        self.fork_spring_n = 0.0
        self.fork_damper_n = 0.0
        self.fork_total_n = 0.0
        self.shock_spring_n = 0.0
        self.shock_bumper_n = 0.0
        self.shock_damper_n = 0.0
        self.shock_total_n = 0.0

    @staticmethod
    def _resolve_compression_joint(model: mujoco.MjModel, joint_name: str) -> Tuple[int, int]:
        """
        Resolves a suspension joint's addresses and verifies its sign convention.

        Args:
            model: Compiled MuJoCo model.
            joint_name: Name of the suspension slide joint.

        Returns:
            Tuple of (qpos address, dof address).

        Raises:
            ValueError: If the joint is missing, unlimited, or its range does not start at
                zero -- the force path writes a negative generalized force to resist
                compression, which is only correct if the coordinate increases with
                compression.
        """
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            raise ValueError(f"model has no joint '{joint_name}'; suspension forces cannot be applied")

        lo, hi = float(model.jnt_range[jid][0]), float(model.jnt_range[jid][1])
        if not bool(model.jnt_limited[jid]) or abs(lo) > 1e-9 or hi <= 0.0:
            raise ValueError(
                f"joint '{joint_name}' has range [{lo:.4f}, {hi:.4f}]; the suspension force path "
                f"requires a limited range starting at 0, so that increasing value means compression"
            )

        return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """
        Computes both suspension forces from the current state and applies them.

        Args:
            model: Compiled MuJoCo model. Unused -- the joint addresses were cached at
                construction -- but kept so every ride-mode force writer shares one
                `(model, data)` signature.
            data: Simulation state, written at the fork and shock DOFs of `qfrc_applied`.
        """
        travel_mm = float(data.qpos[self.fork_qposadr]) * 1000.0
        fork_velocity_mps = float(data.qvel[self.fork_dofadr])
        fork_total, fork_spring, fork_damper = self.controller.compute_fork_force(
            travel_mm, fork_velocity_mps
        )

        stroke_mm = float(data.qpos[self.shock_qposadr]) * 1000.0
        shock_velocity_mps = float(data.qvel[self.shock_dofadr])
        # Coil and bumper are taken separately rather than through
        # CoilShock.compute_axial_force, whose sum they are, because telemetry needs them
        # apart: a bottom-out is a bumper event, not a stiffer spring.
        shock_spring = self.coil_shock.compute_spring_force(stroke_mm)
        shock_bumper = self.coil_shock.compute_bumper_force(stroke_mm)
        shock_damper = self.controller.suspension_system.shock_damper.compute_damping_force(
            shock_velocity_mps, stroke_mm
        )
        shock_total = shock_spring + shock_bumper + shock_damper

        # Both coordinates increase with compression, so a resisting force is a negative
        # generalized force. The construction-time range check guarantees that convention.
        data.qfrc_applied[self.fork_dofadr] = -fork_total
        data.qfrc_applied[self.shock_dofadr] = -shock_total

        self.fork_spring_n = fork_spring
        self.fork_damper_n = fork_damper
        self.fork_total_n = fork_total
        self.shock_spring_n = shock_spring
        self.shock_bumper_n = shock_bumper
        self.shock_damper_n = shock_damper
        self.shock_total_n = shock_total


__all__ = ["SuspensionForceApplier"]
