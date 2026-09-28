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
import numpy as np

from bike_sim.physics.coil_shock import CoilShock
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.stops import end_stop
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
        *,
        physics_config: SimulationPhysicsConfig | None = None,
    ) -> None:
        self.controller = controller
        self.coil_shock = coil_shock
        self.physics_config = physics_config or SimulationPhysicsConfig()

        self.fork_qposadr, self.fork_dofadr = self._resolve_compression_joint(model, "fork_travel")
        self.shock_qposadr, self.shock_dofadr = self._resolve_compression_joint(
            model, "shock_stroke", allow_negative_lower=self.physics_config.physics_mode == "physical"
        )

        # Last computed force components, so telemetry can report them without recomputing.
        self.fork_spring_n = 0.0
        self.fork_damper_n = 0.0
        self.fork_total_n = 0.0
        self.shock_spring_n = 0.0
        self.shock_bumper_n = 0.0
        self.shock_damper_n = 0.0
        self.shock_top_out_n = 0.0
        self.shock_upper_stop_n = 0.0
        self.shock_total_n = 0.0
        self.potential_energy_j: dict[str, float] = {}

    @staticmethod
    def _resolve_compression_joint(
        model: mujoco.MjModel, joint_name: str, *, allow_negative_lower: bool = False
    ) -> Tuple[int, int]:
        """
        Resolves a suspension joint's addresses and verifies its sign convention.

        Args:
            model: Compiled MuJoCo model.
            joint_name: Name of the suspension slide joint.

        Returns:
            Tuple of (qpos address, dof address).

        Raises:
            ValueError: If the joint is missing, unlimited, or has a range inconsistent
                with compression increasing in the positive direction. Only the physical
                shock may have a negative lower range for top-out travel.
        """
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            raise ValueError(f"model has no joint '{joint_name}'; suspension forces cannot be applied")

        lo, hi = float(model.jnt_range[jid][0]), float(model.jnt_range[jid][1])
        lower_valid = abs(lo) <= 1e-9 or (
            allow_negative_lower and joint_name == "shock_stroke" and lo < 0.0
        )
        if not bool(model.jnt_limited[jid]) or not lower_valid or hi <= 0.0:
            raise ValueError(
                f"joint '{joint_name}' has range [{lo:.4f}, {hi:.4f}]; the suspension force path "
                f"requires a compression-positive limited range"
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
        qfrc = self.compute_qfrc(model, data)
        data.qfrc_applied[self.fork_dofadr] = qfrc[self.fork_dofadr]
        data.qfrc_applied[self.shock_dofadr] = qfrc[self.shock_dofadr]

    def compute_qfrc(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
        """Compute suspension's generalized forces without writing the MuJoCo input."""
        components = self.compute_qfrc_components(model, data)
        qfrc = np.zeros(model.nv)
        for component in components.values():
            qfrc += component
        return qfrc

    def compute_qfrc_components(
        self, model: mujoco.MjModel, data: mujoco.MjData
    ) -> dict[str, np.ndarray]:
        """Compute each named spring, damper and stop contribution once."""
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
        physical = self.physics_config.physics_mode == "physical"
        stroke_limit_mm = self.coil_shock.specs.stroke_mm
        shock_bumper = (
            0.0 if physical and stroke_mm > stroke_limit_mm
            else self.coil_shock.compute_bumper_force(stroke_mm)
        )
        shock_damper = self.controller.suspension_system.shock_damper.compute_damping_force(
            shock_velocity_mps, stroke_mm
        )
        top_out_force = 0.0
        top_out_energy = 0.0
        upper_force = 0.0
        upper_energy = 0.0
        bumper_energy = 0.0
        if physical:
            stop = self.physics_config.end_stops
            stroke_m = stroke_mm / 1000.0
            limit_m = stroke_limit_mm / 1000.0
            if stroke_m < 0.0:
                top_out_force, top_out_energy = end_stop(
                    stroke_m, shock_velocity_mps, 0.0, limit_m,
                    stop.stiffness_n_m, stop.damping_n_s_m,
                )
            bumper_length_m = self.coil_shock.specs.bumper_length_mm / 1000.0
            bumper_peak_n = self.coil_shock.specs.bumper_peak_n
            bumper_full_energy = bumper_peak_n * bumper_length_m / 3.0
            if stroke_m <= limit_m:
                bumper_depth_m = max(0.0, stroke_m - (limit_m - bumper_length_m))
                bumper_energy = bumper_peak_n * bumper_depth_m**3 / (3.0 * bumper_length_m**2)
            else:
                upper_force, upper_energy = end_stop(
                    stroke_m, shock_velocity_mps, 0.0, limit_m,
                    stop.stiffness_n_m, stop.damping_n_s_m,
                    upper_boundary_force_n=bumper_peak_n,
                    upper_boundary_energy_j=bumper_full_energy,
                )
            coil_compression_m = max(0.0, (stroke_mm + self.coil_shock.specs.preload_mm) / 1000.0)
            coil_energy = 0.5 * self.coil_shock.specs.rate_n_m * coil_compression_m**2
        else:
            coil_energy = 0.0

        shock_top_out = -top_out_force
        shock_upper_stop = -upper_force
        shock_total = (
            shock_spring + shock_bumper + shock_damper + shock_top_out + shock_upper_stop
        )

        # Both coordinates increase with compression, so a resisting force is a negative
        # generalized force. The construction-time range check guarantees that convention.
        def vector(dofadr: int, force_n: float) -> np.ndarray:
            qfrc = np.zeros(model.nv)
            qfrc[dofadr] = -force_n
            return qfrc

        components = {
            "fork_spring": vector(self.fork_dofadr, fork_spring),
            "fork_damper": vector(self.fork_dofadr, fork_damper),
            "shock_coil": vector(self.shock_dofadr, shock_spring),
            "shock_bumper": vector(self.shock_dofadr, shock_bumper),
            "shock_damper": vector(self.shock_dofadr, shock_damper),
            "shock_top_out": vector(self.shock_dofadr, shock_top_out),
            "shock_upper_stop": vector(self.shock_dofadr, shock_upper_stop),
        }

        self.fork_spring_n = fork_spring
        self.fork_damper_n = fork_damper
        self.fork_total_n = fork_total
        self.shock_spring_n = shock_spring
        self.shock_bumper_n = shock_bumper
        self.shock_damper_n = shock_damper
        self.shock_top_out_n = shock_top_out
        self.shock_upper_stop_n = shock_upper_stop
        self.shock_total_n = shock_total
        self.potential_energy_j = {
            "shock_coil": coil_energy,
            "shock_bumper": bumper_energy,
            "shock_top_out": top_out_energy,
            "shock_upper_stop": upper_energy,
        }
        return components


__all__ = ["SuspensionForceApplier"]
