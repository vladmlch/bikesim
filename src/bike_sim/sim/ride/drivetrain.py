"""
Ride-Mode Pedalled Drivetrain.

Turns the cruise controller's torque demand into the torque the `crank_drive` actuator is
driven with: the rider's pulsing crank torque, the mid-drive's assist on top of it, and the
freewheel that disconnects both on the overrun. The arithmetic lives in
`physics/drivetrain.py`; this module resolves handles, reads state and writes `ctrl`, the
same split every other writer in `sim/ride/` uses.

**Where the torque goes in.** At the crank, not the wheel. The 2.9 kg motor of this model
sits around the bottom bracket (`physics/mass.py`), so its torque passes through the gearing
and the freewheel exactly as the rider's does, and its reaction lands on the frame through
the bottom bracket instead of on the swingarm. The chain equality then carries that torque
to the rear wheel *divided* by `gear_ratio`: 32 teeth driving 14 spins the wheel
`gear_ratio` times faster than the cranks and delivers `gear_ratio` times less torque --
a small cog is the hard end of the cassette.

**What the demand means.** `CruiseController` regulates chassis speed and its gains are
scaled by `1 / (1 + support)` while assist is available, so its output is the *rider's*
share of the wheel torque: the mean crank torque is `demand * gear_ratio` (the chain divides
it back on the way out), the motor multiplies the rider's torque by the support factor, and
the closed loop behaves identically in every assist mode while the rider's ceilings do not
bind. When they do -- 60 N.m at the cranks is only ~26 N.m at the wheel through 32x14 --
the human is the limiter and the run simply takes longer to reach the target. Without the
gain scaling Turbo would move the loop from zeta = 1.21 to 0.57 and the difference between
modes would be half regulator, half drivetrain.

**The freewheel, and what replaces engine braking.** A negative demand -- the controller
holding speed on a descent -- cannot be delivered through a freewheel, so the chain equality
is cleared and the demand is handed to the brakes as a lever position instead, flagged in
telemetry as cruise-sourced so the brake channels stay readable. While disengaged the cranks
are held level by a PD stand-in for the rider's legs, because a disengaged crank with nothing
holding it is the unactuated pendulum the rigid crankset used to avoid.
"""

from math import radians
from typing import NamedTuple, Optional

import mujoco

from bike_sim.physics.drivetrain import (
    ASSIST_MODES,
    ASSIST_ORDER,
    CRANK_TORQUE_CEILING_NM,
    RPM_PER_RADPS,
    DrivetrainSpecs,
    assist_target_torque,
    cutoff_factor,
    first_order_step,
    limited_rider_torque,
    ripple_shape,
)
from bike_sim.sim.ride.braking import BRAKE_TORQUE_CEILING_NM


class CrankCommand(NamedTuple):
    """One step's drivetrain output, ready for `ctrl` and for telemetry."""

    crank_torque_nm: float
    rider_torque_nm: float
    assist_torque_nm: float
    phase_rad: float
    cadence_rpm: float
    rider_power_w: float
    motor_power_w: float
    support_factor: float
    freewheel: bool
    cutoff_active: bool
    rider_limit: str
    brake_demand: float


class PedalDrivetrain:
    """
    The crank, the chain and the mid-drive, as one per-step writer.

    Attributes:
        specs: Gearing, rider ceilings and assist ceilings.
        drive_mode: ``pedal`` (rider alone) or ``pedelec`` (rider plus mid-drive).
        assist_mode: Selected support level; always ``off`` in ``pedal`` mode.
        command: The last computed command, for the HUD and the recorder.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        specs: Optional[DrivetrainSpecs] = None,
        drive_mode: str = "pedelec",
        assist_mode: str = "tour",
    ) -> None:
        """
        Args:
            model: Compiled ride-mode model carrying `crank_spin`, `crank_drive` and the
                `chain_drive` equality.
            specs: Drivetrain limits. Defaults to the shipped 32x14 full-power eMTB.
            drive_mode: ``pedal`` or ``pedelec``.
            assist_mode: One of `ASSIST_MODES`; ignored in ``pedal`` mode.

        Raises:
            ValueError: If the mode is unknown, or the model lacks the crank joint, the crank
                actuator or the chain equality.
        """
        if drive_mode not in ("pedal", "pedelec"):
            raise ValueError(f"drive_mode must be 'pedal' or 'pedelec', got {drive_mode!r}")
        if assist_mode not in ASSIST_MODES:
            raise ValueError(
                f"unknown assist mode {assist_mode!r}; available: {', '.join(ASSIST_ORDER)}"
            )

        self.specs = specs if specs is not None else DrivetrainSpecs()
        self.drive_mode = drive_mode
        self.assist_mode = "off" if drive_mode == "pedal" else assist_mode

        self.crank_qposadr, self.crank_dofadr = _joint_addresses(model, "crank_spin")
        self.wheel_qposadr, self.wheel_dofadr = _joint_addresses(model, "rear_wheel_spin")
        self.ctrl_adr = _actuator_id(model, "crank_drive")
        self.eq_id = _equality_id(model, "chain_drive")
        # `chain_drive` is a *position* constraint, not a velocity one: MuJoCo enforces
        # `crank - crank0 = polycoef[0] + (wheel - wheel0) / ratio` against the model's
        # `qpos0`. While the freewheel is open the wheel keeps turning and the crank is held
        # still, so that residual grows without bound -- a quarter second of coasting at
        # 20 km/h is 1.8 rad of crank. Re-closing the constraint over it asks the solver to
        # spin the crank most of a turn inside one 0.5 ms step, which is not a jolt but an
        # impulse large enough to throw the bike over the bars. `polycoef[0]` is therefore
        # rewritten at every engagement to the angle the freehub actually re-engaged at, so
        # the residual is zero when the chain closes. A real freehub has no fixed angular
        # datum across a coast either: the pawls catch wherever they catch.
        self.crank_qpos0 = float(model.qpos0[self.crank_qposadr])
        self.wheel_qpos0 = float(model.qpos0[self.wheel_qposadr])

        self.start_phase_rad = radians(self.specs.crank_phase_deg)
        self.engaged = True
        self.hold_phase_rad = self.start_phase_rad
        self.assist_torque_nm = 0.0
        self.traction_limited = False
        self.command = _idle_command(self.start_phase_rad, self.nominal_support_factor)

    # --- assist selection ------------------------------------------------------------

    @property
    def nominal_support_factor(self) -> float:
        """Support factor of the selected mode, before the cutoff taper."""
        return self.specs.support_factor(self.assist_mode)

    @property
    def effective_support_factor(self) -> float:
        """Support factor actually in force, after the last step's cutoff taper."""
        return self.command.support_factor

    def set_assist_mode(self, mode: str) -> str:
        """
        Selects an assist mode.

        Args:
            mode: One of `ASSIST_MODES`.

        Returns:
            The mode now in force.

        Raises:
            ValueError: If the mode is unknown, or assist is requested in ``pedal`` mode.
        """
        if mode not in ASSIST_MODES:
            raise ValueError(
                f"unknown assist mode {mode!r}; available: {', '.join(ASSIST_ORDER)}"
            )
        if self.drive_mode == "pedal" and mode != "off":
            raise ValueError("assist modes need --drive-mode pedelec; 'pedal' is the rider alone")
        self.assist_mode = mode
        return self.assist_mode

    def cycle_assist_mode(self) -> str:
        """
        Steps to the next assist mode in `ASSIST_ORDER`.

        Returns:
            The mode now in force; unchanged in ``pedal`` mode, which has no motor.
        """
        if self.drive_mode == "pedal":
            return self.assist_mode
        nxt = ASSIST_ORDER[(ASSIST_ORDER.index(self.assist_mode) + 1) % len(ASSIST_ORDER)]
        return self.set_assist_mode(nxt)

    # --- per-step --------------------------------------------------------------------

    def compute(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        wheel_demand_nm: float,
        speed_mps: float,
        rear_in_contact: bool,
        traction_limited: bool = False,
    ) -> CrankCommand:
        """
        Computes this step's crank torque and, on the overrun, the brake demand replacing it.

        Args:
            model: Compiled model, read for the timestep.
            data: Simulation state; `ctrl` is written by the caller, not here.
            wheel_demand_nm: The speed controller's output, read as the rider's share of the
                wheel torque.
            speed_mps: Chassis speed, for the assist cutoff taper.
            rear_in_contact: Whether the driven wheel carries load. Pedalling into an
                airborne wheel spins it up for nothing, exactly as cruise control refuses to.
            traction_limited: The rear patch is fully sliding; the pulse is delivering into a
                slipping tyre. Recorded, not acted on: the brush model already owns the grip.

        Returns:
            The command, also stored on `self.command`.
        """
        dt = float(model.opt.timestep)
        phase = float(data.qpos[self.crank_qposadr])
        crank_radps = float(data.qvel[self.crank_dofadr])
        cadence_rpm = crank_radps * RPM_PER_RADPS
        pedalling = wheel_demand_nm > 0.0 and rear_in_contact

        if pedalling:
            self._engage(model, data)
            mean_nm, limit = limited_rider_torque(
                wheel_demand_nm * self.specs.gear_ratio, crank_radps, self.specs
            )
            rider_nm = mean_nm * ripple_shape(phase, self.specs.ripple_depth)
            taper = cutoff_factor(speed_mps, self.specs)
            support = self.nominal_support_factor * taper
            target = assist_target_torque(rider_nm, support, crank_radps, self.specs)
            self.assist_torque_nm = first_order_step(
                self.assist_torque_nm, target, dt, self.specs.assist_response_s
            )
            total = rider_nm + self.assist_torque_nm
            brake_demand = 0.0
        else:
            self._disengage(data, phase)
            rider_nm = 0.0
            limit = "none"
            support = 0.0
            taper = cutoff_factor(speed_mps, self.specs)
            # The motor's own torque decays through the same lag rather than vanishing.
            self.assist_torque_nm = first_order_step(
                self.assist_torque_nm, 0.0, dt, self.specs.assist_response_s
            )
            total = self.assist_torque_nm + self._hold_torque(phase, crank_radps)
            brake_demand = _cruise_brake_demand(wheel_demand_nm)

        total = max(-CRANK_TORQUE_CEILING_NM, min(CRANK_TORQUE_CEILING_NM, total))
        self.traction_limited = bool(traction_limited)
        self.command = CrankCommand(
            crank_torque_nm=total,
            rider_torque_nm=rider_nm,
            assist_torque_nm=self.assist_torque_nm,
            phase_rad=phase,
            cadence_rpm=cadence_rpm,
            rider_power_w=rider_nm * crank_radps,
            # Reported at the crank, and only while the crank is connected to the road. A
            # freewheeling crank spun by its own hold torque is not the motor doing work,
            # and reporting it as such is how a solver blow-up reads as "motor -28 kW".
            motor_power_w=self.assist_torque_nm * crank_radps if self.engaged else 0.0,
            support_factor=support,
            freewheel=not self.engaged,
            cutoff_active=self.nominal_support_factor > 0.0 and taper < 1.0,
            rider_limit=limit,
            brake_demand=brake_demand,
        )
        return self.command

    def write(self, data: mujoco.MjData) -> None:
        """
        Writes the last command into the crank actuator.

        Args:
            data: Simulation state.
        """
        data.ctrl[self.ctrl_adr] = self.command.crank_torque_nm

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """
        Returns the crank to its starting phase, re-engages the chain and clears the motor.

        The datum is rewritten here too, and it has to be: `--crank-phase` moves the crank
        away from the pose the model was compiled in while the wheel stays at its own
        reference, which is the same position residual the freewheel produces. Without this
        every run with a non-zero start phase began with a constraint violation of that many
        radians and blew up on the first step.

        Args:
            model: Compiled model; `eq_data` carries the chain datum.
            data: Simulation state, written at the crank coordinate.
        """
        data.qpos[self.crank_qposadr] = self.start_phase_rad
        data.qvel[self.crank_dofadr] = 0.0
        data.ctrl[self.ctrl_adr] = 0.0
        self.assist_torque_nm = 0.0
        self.hold_phase_rad = self.start_phase_rad
        self.engaged = True
        self.traction_limited = False
        self._redatum(model, data)
        _set_equality(data, self.eq_id, True)
        self.command = _idle_command(self.start_phase_rad, self.nominal_support_factor)

    # --- internals -------------------------------------------------------------------

    def _engage(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """
        Re-engages the freewheel, matching both the crank's datum and its speed to the wheel.

        Both halves are needed and they fix different things. Matching the velocity stops the
        engagement reading as a jolt in the suspension channels. Matching the *datum* stops it
        being a blow-up: the equality is a position constraint measured from `qpos0`, and the
        angle the wheel turned through while the chain was open is a residual the solver would
        otherwise close inside one step.

        Args:
            model: Compiled model; `eq_data` carries the chain datum.
            data: Simulation state.
        """
        if self.engaged:
            return
        data.qvel[self.crank_dofadr] = float(data.qvel[self.wheel_dofadr]) / self.specs.gear_ratio
        self._redatum(model, data)
        _set_equality(data, self.eq_id, True)
        self.engaged = True

    def _redatum(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """
        Moves the chain equality's constant term to the angles the chain is closing at.

        Args:
            model: Compiled model; `eq_data[eq_id][0]` is the polynomial's constant term.
            data: Simulation state, read for the current crank and wheel angles.
        """
        crank = float(data.qpos[self.crank_qposadr]) - self.crank_qpos0
        wheel = float(data.qpos[self.wheel_qposadr]) - self.wheel_qpos0
        model.eq_data[self.eq_id, 0] = crank - wheel / self.specs.gear_ratio

    def chain_residual(self, model: mujoco.MjModel, data: mujoco.MjData) -> float:
        """
        The chain equality's position residual, in radians of crank.

        Exposed because this is the quantity that silently grew while the freewheel was open
        and then discharged into the rear wheel. A test that watches it fails loudly; a test
        that watches only mean torques does not.

        Args:
            model: Compiled model.
            data: Simulation state.

        Returns:
            Crank angle error, rad. Zero whenever the chain is engaged and correct.
        """
        crank = float(data.qpos[self.crank_qposadr]) - self.crank_qpos0
        wheel = float(data.qpos[self.wheel_qposadr]) - self.wheel_qpos0
        return crank - wheel / self.specs.gear_ratio - float(model.eq_data[self.eq_id, 0])

    def _disengage(self, data: mujoco.MjData, phase_rad: float) -> None:
        """
        Opens the freewheel and latches the phase the cranks are to be held at.

        Args:
            data: Simulation state.
            phase_rad: Crank angle at the moment of disengagement.
        """
        if self.engaged:
            _set_equality(data, self.eq_id, False)
            self.hold_phase_rad = phase_rad
            self.engaged = False

    def _hold_torque(self, phase_rad: float, crank_radps: float) -> float:
        """
        The rider's legs holding the cranks still on the overrun.

        Args:
            phase_rad: Current crank angle.
            crank_radps: Current crank angular velocity.

        Returns:
            A PD torque about the latched phase, N.m.
        """
        return (
            self.specs.crank_hold_stiffness_nm_rad * (self.hold_phase_rad - phase_rad)
            - self.specs.crank_hold_damping_nms_rad * crank_radps
        )


def _cruise_brake_demand(wheel_demand_nm: float) -> float:
    """
    Turns a negative speed-holding demand into a brake lever position.

    Args:
        wheel_demand_nm: The controller's output; only negative values brake.

    Returns:
        Lever position in [0, 1], applied to both wheels, so the pair delivers the demanded
        torque at the 200 N.m per-wheel ceiling.
    """
    if wheel_demand_nm >= 0.0:
        return 0.0
    return min(1.0, -wheel_demand_nm / (2.0 * BRAKE_TORQUE_CEILING_NM))


def _idle_command(phase_rad: float, support_factor: float) -> CrankCommand:
    """A zero command at a given phase, for construction and reset."""
    return CrankCommand(
        crank_torque_nm=0.0,
        rider_torque_nm=0.0,
        assist_torque_nm=0.0,
        phase_rad=phase_rad,
        cadence_rpm=0.0,
        rider_power_w=0.0,
        motor_power_w=0.0,
        support_factor=support_factor,
        freewheel=False,
        cutoff_active=False,
        rider_limit="none",
        brake_demand=0.0,
    )


def _joint_addresses(model: mujoco.MjModel, name: str) -> tuple:
    """
    Resolves a hinge joint's qpos and dof addresses.

    Args:
        model: Compiled model.
        name: Joint name.

    Returns:
        Tuple of (qpos address, dof address).

    Raises:
        ValueError: If the model has no such joint.
    """
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise ValueError(f"model has no joint '{name}'; the pedalled drivetrain needs it")
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


def _actuator_id(model: mujoco.MjModel, name: str) -> int:
    """
    Resolves an actuator's index into `ctrl`.

    Args:
        model: Compiled model.
        name: Actuator name.

    Returns:
        The index.

    Raises:
        ValueError: If the model has no such actuator.
    """
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"model has no actuator '{name}'; the pedalled drivetrain needs it")
    return int(aid)


def _equality_id(model: mujoco.MjModel, name: str) -> int:
    """
    Resolves the chain equality's index.

    Args:
        model: Compiled model.
        name: Equality constraint name.

    Returns:
        The index.

    Raises:
        ValueError: If the model has no such constraint.
    """
    eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, name)
    if eid < 0:
        raise ValueError(f"model has no equality '{name}'; the chain cannot be opened or closed")
    return int(eid)


def _set_equality(data: mujoco.MjData, eq_id: int, active: bool) -> None:
    """
    Opens or closes an equality constraint at runtime.

    Args:
        data: Simulation state. `eq_active` moved from the model to the data in MuJoCo 3;
            older builds keep it on the model, so both are handled.
        eq_id: Constraint index.
        active: Whether the constraint is to be enforced.
    """
    value = 1 if active else 0
    try:
        data.eq_active[eq_id] = value
    except AttributeError:  # pragma: no cover - MuJoCo 2.x
        data.model.eq_active[eq_id] = value


__all__ = ["CrankCommand", "PedalDrivetrain"]
