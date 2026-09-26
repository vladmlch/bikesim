"""
Ride-Mode Static Equilibrium Solver.

A run starts from a numerically solved static equilibrium written into `qpos` rather than
from a settling drop: that removes the first half-second of transient from the telemetry
and makes headless runs reproducible from the first frame (docs/RIDE.md section 9).

The method is damped relaxation. The bike is placed just above the road and stepped with
the suspension forces applied; every few milliseconds the velocities are discarded, so the
state creeps down the potential-energy gradient instead of oscillating about the bottom of
it. Convergence is measured on the accelerations of the zero-velocity state, where the
dampers contribute nothing and the residual is a pure static force imbalance.
"""

from typing import Any, Dict, Optional

import mujoco
import numpy as np

from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.sim.ride.forces import SuspensionForceApplier
from bike_sim.sim.ride.rider_forces import RiderForceApplier
from bike_sim.sim.ride.tyre.applier import TyreForceApplier

# Steps between velocity resets. At the ride timestep this is a 20 ms window: short against
# the ~3 Hz suspension modes, so each cycle is a small downhill move rather than half an
# oscillation, and long enough that the state advances quadratically rather than one
# timestep at a time.
RELAX_STEPS_PER_CYCLE = 40
RELAX_CYCLE_S = RELAX_STEPS_PER_CYCLE * 0.0005

# Initial gap between the wheels and the road, so the solve starts with no penetration.
START_CLEARANCE_M = 0.005


def solve_static_equilibrium(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    applier: SuspensionForceApplier,
    solver: HorstLinkageSolver,
    start_x_m: float = 2.0,
    max_steps: int = 40000,
    tol: float = 0.05,
    rider_applier: Optional[RiderForceApplier] = None,
    tyre_applier: Optional[TyreForceApplier] = None,
    relax_steps_per_cycle: Optional[int] = None,
) -> Dict[str, Any]:
    """
    Solves the static equilibrium the bike settles into on the road under gravity.

    The model's state is reset and overwritten, so the solve is reproducible and can be
    repeated on the same `data`.

    Args:
        model: Compiled ride-mode model, with `hfield_data` already filled for the track.
        data: Simulation state to solve in place.
        applier: Suspension force path, applied at every step of the relaxation.
        solver: Analytical linkage solver, used to report the rear wheel travel that
            corresponds to the converged shaft stroke.
        start_x_m: World X for the chassis root at the start of the solve. The default
            keeps both contact patches on the heightfield, whose near edge is at x = 0.
        max_steps: Cap on simulation steps before the solve is declared failed.
        tol: Convergence threshold on the largest absolute generalized acceleration of the
            zero-velocity state, in m/s^2 and rad/s^2.
        rider_applier: The seated rider's force path, applied alongside the suspension so
            the rider's slide coordinates settle with everything else. None or an inert
            applier for the bike alone or the lumped rider.
        tyre_applier: Pneumatic tyre force path, applied before suspension and rider forces.
            Its material and bristle state is reset whenever the velocity relaxation is reset.
        relax_steps_per_cycle: Steps between velocity resets. ``None`` keeps the historical
            40-step sphere cadence; pneumatic callers pass the count that preserves the
            20 ms cycle at their selected timestep.

    Returns:
        Dict with the converged `fork_travel_mm`, `shock_stroke_mm`, `rear_travel_mm`,
        `root_z_m`, `pitch_rad`, the `steps` taken and the final `residual_qacc`. With a
        seated rider also `rider_saddle_load_n`, `rider_pedal_load_n`, `rider_bar_load_n`
        and the corresponding `rider_*_share` fractions of the rider's weight.

    Raises:
        RuntimeError: If the relaxation has not converged within `max_steps`. The
            unconverged state stays in `data` for inspection, but is never returned as an
            equilibrium.
    """
    x_adr = _qposadr(model, "root_x")
    z_adr = _qposadr(model, "root_z")
    pitch_adr = _qposadr(model, "root_pitch")

    cycle_steps = RELAX_STEPS_PER_CYCLE if relax_steps_per_cycle is None else int(relax_steps_per_cycle)
    if cycle_steps <= 0:
        raise ValueError(f"relax_steps_per_cycle must be positive, got {cycle_steps}")

    def apply_forces() -> None:
        if tyre_applier is not None:
            tyre_applier.apply(model, data)
        applier.apply(model, data)
        if rider_applier is not None:
            rider_applier.apply(model, data)

    mujoco.mj_resetData(model, data)
    if tyre_applier is not None:
        tyre_applier.reset(data)
    data.qpos[x_adr] = float(start_x_m)
    data.qpos[z_adr] = START_CLEARANCE_M
    mujoco.mj_forward(model, data)

    steps = 0
    residual = float("inf")
    while steps < max_steps:
        for _ in range(min(cycle_steps, max_steps - steps)):
            apply_forces()
            mujoco.mj_step(model, data)
            steps += 1

        data.qvel[:] = 0.0
        if tyre_applier is not None:
            tyre_applier.reset(data)
            # Object velocities are cached by MuJoCo. Refresh them after zeroing qvel before
            # the tyre reads `mj_objectVelocity`; the sphere path retains its old call order.
            mujoco.mj_forward(model, data)
        apply_forces()
        mujoco.mj_forward(model, data)
        residual = float(np.max(np.abs(data.qacc)))

        if residual <= tol:
            shock_stroke_mm = float(data.qpos[applier.shock_qposadr]) * 1000.0
            rear_state = solver.solve_state_from_shock_stroke(shock_stroke_mm)
            result = {
                "fork_travel_mm": float(data.qpos[applier.fork_qposadr]) * 1000.0,
                "shock_stroke_mm": shock_stroke_mm,
                "rear_travel_mm": float(rear_state["wheel_travel"]),
                "root_z_m": float(data.qpos[z_adr]),
                "pitch_rad": float(data.qpos[pitch_adr]),
                "steps": steps,
                "residual_qacc": residual,
            }
            if rider_applier is not None and rider_applier.active:
                loads = rider_applier.interface_loads_n()
                weight = rider_applier.pose.total_mass_kg * 9.81
                for name in ("saddle", "pedals", "bar"):
                    result[f"rider_{name}_load_n"] = float(loads[name])
                    result[f"rider_{name}_share"] = float(loads[name] / weight)
            return result

    raise RuntimeError(
        f"static equilibrium did not converge in {steps} steps: residual "
        f"{residual:.4f} > {tol:.4f}, root_z {data.qpos[z_adr] * 1000.0:.1f} mm, "
        f"fork {data.qpos[applier.fork_qposadr] * 1000.0:.1f} mm, "
        f"shock {data.qpos[applier.shock_qposadr] * 1000.0:.1f} mm"
    )


def _qposadr(model: mujoco.MjModel, joint_name: str) -> int:
    """
    Resolves the qpos address of a named joint.

    Args:
        model: Compiled MuJoCo model.
        joint_name: Name of the joint to resolve.

    Returns:
        Index into `qpos` of the joint's first coordinate.

    Raises:
        ValueError: If the model has no joint of that name.
    """
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise ValueError(f"model has no joint '{joint_name}'; the equilibrium solve needs a planar root")
    return int(model.jnt_qposadr[jid])


__all__ = [
    "solve_static_equilibrium",
    "RELAX_STEPS_PER_CYCLE",
    "RELAX_CYCLE_S",
    "START_CLEARANCE_M",
]
