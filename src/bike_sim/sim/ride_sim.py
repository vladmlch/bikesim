"""
Ride-Mode Simulation Orchestrator.

Owns the compiled ride model, the track raster, and the five per-step writers, and drives
them in a fixed order. Everything that computes a force lives in `sim/ride/`; this module
resolves handles, sequences the writers, and exposes the run's state. It coordinates, it
does not compute.

**Track data is written before anything reads the model's geometry.** `model.hfield_data` is
filled from the track raster immediately after compilation, ahead of the first
`mj_forward`, the equilibrium solve and any viewer: a heightfield read before it is filled
is a flat road, and every contact computed against it is wrong.
"""

from math import sin
from typing import Any, Callable, Dict, Optional, Tuple, Union

import mujoco

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.hardpoints import compute_ground_z
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.coil_shock import CoilShock
from bike_sim.physics.damper import BikeSuspensionSystem
from bike_sim.physics.drivetrain import DRIVE_MODES, DrivetrainSpecs, cutoff_factor
from bike_sim.physics.rider import DEFAULT_RIDER_VARIANT, RiderSpecs, SeatedPose, resolve_rider
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.controllers import SuspensionController
from bike_sim.sim.equilibrium import RELAX_CYCLE_S, solve_static_equilibrium
from bike_sim.sim.ride.braking import BrakeController
from bike_sim.sim.ride.contacts import TerrainContactQuery, TerrainContacts
from bike_sim.sim.ride.cruise import DEFAULT_TARGET_SPEED_KMH, CruiseController
from bike_sim.sim.ride.drivetrain import CrankCommand, PedalDrivetrain
from bike_sim.sim.ride.forces import SuspensionForceApplier
from bike_sim.sim.ride.resistance import RollingResistance
from bike_sim.sim.ride.rider_forces import RiderForceApplier
from bike_sim.sim.ride.termination import (
    DEFAULT_MAX_WALL_CLOCK_S,
    RunLimits,
    RunOutcome,
    RunTerminator,
)
from bike_sim.sim.ride.tyre.applier import TyreForceApplier
from bike_sim.sim.ride.tyre.geometry import RoadProfile
from bike_sim.sim.ride.virtual_rider import CrashDetector, CrashEvent, PitchStabilizer
from bike_sim.terrain import (
    DEFAULT_PRESET,
    HeightFieldSpec,
    TrackSpec,
    assert_track_fits,
    build_profile,
    build_field_data,
    get_preset,
    SurfaceMap,
)

# World X for the chassis root at the start of a run. The heightfield's near edge is at
# world x = 0, and starting at x = 0 hangs the rear contact patch off it.
DEFAULT_START_X_M = 2.0


class RideSimulation:
    """
    A ride-mode run: compiled model, track, controllers, and a single `step`.

    The per-step writers are applied in a fixed order. `sphere` uses suspension, seated
    rider, rolling resistance, cruise, brakes and the virtual rider. `pneumatic` evaluates
    the tyre first, uses its loads for the shared contact snapshot, skips the explicit Crr
    writer, then applies the other writers.
    """

    def __init__(
        self,
        track: Optional[TrackSpec] = None,
        specs: Optional[BikeSpecs] = None,
        target_speed_kmh: float = DEFAULT_TARGET_SPEED_KMH,
        include_rider: bool = True,
        start_x_m: float = DEFAULT_START_X_M,
        controller: Optional[SuspensionController] = None,
        coil_shock: Optional[CoilShock] = None,
        debug_markers: bool = False,
        field: Optional[HeightFieldSpec] = None,
        rider: Optional[Union[RiderSpecs, str]] = None,
        tyre: Optional[TyreConfig] = None,
        drive_mode: str = "motor",
        assist: str = "tour",
        drivetrain: Optional[DrivetrainSpecs] = None,
    ) -> None:
        """
        Compiles the model, rasterizes the track, and solves the starting equilibrium.

        Args:
            track: Track to ride. Defaults to the shipped default preset.
            specs: Bicycle geometry. Defaults to the shipped `BikeSpecs`.
            target_speed_kmh: Initial cruise target, inside the 15-45 km/h band.
            include_rider: Legacy switch: False rides the bike alone; True rides the default
                rider variant. Ignored when ``rider`` is given.
            rider: The rider -- a `RiderSpecs` or a variant name (``none``, ``lumped``,
                ``seated``). Defaults to the seated rider (docs/RIDE.md section 7).
            start_x_m: World X for the chassis root at the start of the run.
            controller: Fork and shock force calculator. Defaults to the shipped
                suspension; supplied explicitly when a run overrides the fork pressure.
            coil_shock: Rear coil and bumper model. Defaults to the shipped coil; supplied
                explicitly when a run overrides the spring rate.
            debug_markers: Whether to compile the yellow pivot-marker geoms, so the
                interactive viewer's marker toggle has something to show. They carry
                `mass="0"` and no collision, and a 4000-step traverse was verified to be
                bit-identical with and without them, so the flag is visual only.
            field: Heightfield geometry to compile. Defaults to
                `HeightFieldSpec.for_track(track)`: the shipped field for any track that
                fits it, a longer field otherwise.
            tyre: Wheel-contact model and fidelity tier. Defaults to the existing `sphere`
                model; `pneumatic` uses the configured pressure and the track surface.
            drive_mode: Where propulsion comes from. ``motor`` is the original ideal torque
                source at the rear wheel and stays the default, so every existing run is
                unchanged. ``pedal`` builds the turning crankset and pedals it with the rider
                alone; ``pedelec`` adds the mid-drive's assist on top.
            assist: Assist level for ``pedelec``: ``off``, ``eco``, ``tour``, ``sport`` or
                ``turbo``.
            drivetrain: Gearing and the rider and motor ceilings. Defaults to the shipped
                32x14 full-power eMTB.

        Raises:
            ValueError: If the track does not fit the heightfield envelope.
            RuntimeError: If the starting equilibrium does not converge.
        """
        if drive_mode not in DRIVE_MODES:
            raise ValueError(
                f"unknown drive mode {drive_mode!r}; available: {', '.join(DRIVE_MODES)}"
            )
        self.drive_mode = drive_mode
        self.drivetrain_specs = drivetrain if drivetrain is not None else DrivetrainSpecs()
        pedalled = drive_mode != "motor"

        self.track = track if track is not None else get_preset(DEFAULT_PRESET)
        self.field = field if field is not None else HeightFieldSpec.for_track(self.track)
        assert_track_fits(self.track, self.field)

        self.specs = specs if specs is not None else BikeSpecs()
        self.tyre_config = tyre if tyre is not None else TyreConfig()
        self.solver = HorstLinkageSolver(self.specs)
        self.start_x_m = float(start_x_m)
        self.rider: RiderSpecs = resolve_rider(
            rider, include_rider=include_rider, default_variant=DEFAULT_RIDER_VARIANT
        )
        # The seated pose is solved once here; the builder solves the same pose from the
        # same specs, so the force path and the compiled joints agree by construction.
        self.pose: Optional[SeatedPose] = (
            self.rider.seated_pose(self.specs) if self.rider.variant == "seated" else None
        )

        self.model = mujoco.MjModel.from_xml_string(
            generate_mujoco_xml(
                specs=self.specs,
                solver=self.solver,
                mode="ride",
                rider=self.rider,
                debug_markers=debug_markers,
                field=self.field,
                tyre_model=self.tyre_config.model,
                crank_joint=pedalled,
                gear_ratio=self.drivetrain_specs.gear_ratio,
            )
        )
        # Before the first forward pass, before the equilibrium solve, before any viewer.
        self.model.hfield_data[:] = build_field_data(self.track, self.field).reshape(-1)
        self.tyre_applier: Optional[TyreForceApplier] = None
        if self.tyre_config.pneumatic:
            track_x = self.field.track_x()
            road_z = (
                compute_ground_z(self.specs) / 1000.0
                + build_profile(self.track, track_x)
            )
            profile = RoadProfile.from_samples(track_x, road_z)
            self.tyre_applier = TyreForceApplier(
                self.model,
                self.tyre_config,
                profile,
                SurfaceMap.uniform(self.track.surface),
            )
        self.data = mujoco.MjData(self.model)

        self.controller = controller if controller is not None else _default_controller(self.specs)
        self.applier = SuspensionForceApplier(
            self.model,
            self.controller,
            coil_shock if coil_shock is not None else CoilShock(),
        )
        self.rider_forces = RiderForceApplier(self.model, self.pose)
        self.contact_query = TerrainContactQuery(self.model)
        self.resistance = RollingResistance(self.model)
        self.cruise = CruiseController(self.model, target_speed_kmh=target_speed_kmh)
        self.drivetrain: Optional[PedalDrivetrain] = (
            PedalDrivetrain(
                self.model,
                specs=self.drivetrain_specs,
                drive_mode=drive_mode,
                assist_mode=assist,
            )
            if pedalled
            else None
        )
        # Crank arm length in metres, for the pedal positions the rider's legs follow.
        self.crank_length_m = float(self.specs.crank_length) / 1000.0
        self.brake_source_cruise = False
        self.brakes = BrakeController(self.model)
        self.stabilizer = PitchStabilizer(self.model)
        self.crash_detector = CrashDetector(self.model)

        self.drive_ctrl_adr = _actuator_id(self.model, "rear_drive")
        self.front_brake_ctrl_adr = _actuator_id(self.model, "front_brake")
        self.rear_brake_ctrl_adr = _actuator_id(self.model, "rear_brake")
        self.root_x_qposadr, self.root_x_dofadr = _root_addresses(self.model, "root_x")
        self.root_pitch_qposadr, _ = _root_addresses(self.model, "root_pitch")

        self.contacts = TerrainContacts(
            front_load_n=0.0,
            rear_load_n=0.0,
            front_support_n=0.0,
            rear_support_n=0.0,
            handlebar_load_n=0.0,
        )
        self.steps = 0
        self.equilibrium: Dict[str, Any] = {}
        self.reset()

    def reset(self) -> None:
        """
        Returns the run to its starting equilibrium at the track start.

        Raises:
            RuntimeError: If the equilibrium solve does not converge.
        """
        if self.tyre_applier is None:
            self.equilibrium = solve_static_equilibrium(
                self.model,
                self.data,
                self.applier,
                self.solver,
                start_x_m=self.start_x_m,
                rider_applier=self.rider_forces,
            )
        else:
            relax_steps = max(1, round(RELAX_CYCLE_S / float(self.model.opt.timestep)))
            self.equilibrium = solve_static_equilibrium(
                self.model,
                self.data,
                self.applier,
                self.solver,
                start_x_m=self.start_x_m,
                rider_applier=self.rider_forces,
                tyre_applier=self.tyre_applier,
                relax_steps_per_cycle=relax_steps,
            )
        self.contact_query.reset()
        self.cruise.reset()
        if self.drivetrain is not None:
            self.drivetrain.reset(self.model, self.data)
            self.rider_forces.set_pedal_offsets(0.0, 0.0)
            mujoco.mj_forward(self.model, self.data)
        self.brake_source_cruise = False
        self.brakes.reset()
        self.resistance.reset()
        self.stabilizer.reset()
        self.crash_detector.reset()
        self.data.time = 0.0
        self.steps = 0
        if self.tyre_applier is None:
            self.contacts = self.contact_query.query(self.model, self.data)
        else:
            self.contacts = self.contact_query.query(
                self.model, self.data, wheel_load_provider=self.tyre_applier
            )

    def step(self, front_brake_demand: float = 0.0, rear_brake_demand: float = 0.0) -> None:
        """
        Advances the simulation by one timestep.

        The contact snapshot the five writers share is the one taken at the end of the
        previous step, which holds the constraint forces of the last forward pass -- the same
        numbers a fresh query at the top of this step would return, since nothing has touched
        `data.contact` in between. Querying once per state rather than once per use keeps the
        dropout counter advancing exactly one step per step.

        Args:
            front_brake_demand: Front brake lever position in [0, 1], not a torque.
            rear_brake_demand: Rear brake lever position in [0, 1], not a torque.
        """
        if self.tyre_applier is not None:
            self.tyre_applier.apply(self.model, self.data)
            self.contacts = self.contact_query.query(
                self.model, self.data, wheel_load_provider=self.tyre_applier
            )

        self.applier.apply(self.model, self.data)
        self._follow_cranks()
        self.rider_forces.apply(self.model, self.data)
        self._compensate_cruise_gains()
        if self.tyre_applier is None:
            self.resistance.apply(self.model, self.data, self.contacts)
            traction_limited = False
            drive_torque = self.cruise.compute(self.model, self.data, self.contacts)
        else:
            error_mps = self.cruise.target_speed_mps - float(self.data.qvel[self.root_x_dofadr])
            rear_outputs = self.tyre_applier.rear_outputs
            traction_limited = any(
                patch.fully_sliding and patch.slip_ratio * error_mps > 0.0
                for patch in rear_outputs.patches
            )
            drive_torque = self.cruise.compute(
                self.model, self.data, self.contacts, traction_limited=traction_limited
            )
        if self.drivetrain is None:
            self.data.ctrl[self.drive_ctrl_adr] = drive_torque
        else:
            # The wheel actuator is left idle: the crank drives, and the chain equality is
            # what puts the torque on the wheel.
            self.data.ctrl[self.drive_ctrl_adr] = 0.0
            command = self.drivetrain.compute(
                self.model,
                self.data,
                drive_torque,
                speed_mps=self.speed_mps,
                rear_in_contact=self.contacts.rear_in_contact,
                traction_limited=traction_limited,
            )
            self.drivetrain.write(self.data)
            # A freewheel cannot hold speed on a descent, so the controller's negative demand
            # is braking, and telemetry is told who asked for it.
            self.brake_source_cruise = command.brake_demand > max(
                front_brake_demand, rear_brake_demand
            )
            front_brake_demand = max(front_brake_demand, command.brake_demand)
            rear_brake_demand = max(rear_brake_demand, command.brake_demand)
        front_torque, rear_torque = self.brakes.compute(
            self.data, front_brake_demand, rear_brake_demand
        )
        self.data.ctrl[self.front_brake_ctrl_adr] = front_torque
        self.data.ctrl[self.rear_brake_ctrl_adr] = rear_torque
        self.stabilizer.apply(self.model, self.data, self.contacts)

        # Checked against the same snapshot the writers saw, so a reported crash position is
        # the state that produced it rather than the state one timestep later.
        self.crash_detector.check(self.data, self.contacts)

        mujoco.mj_step(self.model, self.data)
        self.steps += 1
        if self.tyre_applier is None:
            self.contacts = self.contact_query.query(self.model, self.data)

    def _follow_cranks(self) -> None:
        """
        Moves the seated rider's pedal interfaces to where the cranks now are.

        The crank turns about +y, so the arm built at 3 o'clock sweeps to
        `(L cos phi, 0, -L sin phi)`: the front pedal drops as the phase advances and the
        rear pedal, 180 degrees away, rises by the same amount. Only the interface moves; the
        leg masses stay on their slide coordinates, which is what makes this a bob source
        rather than a kinematic chain.
        """
        if self.drivetrain is None or not self.rider_forces.active:
            return
        phase = float(self.data.qpos[self.drivetrain.crank_qposadr])
        drop = self.crank_length_m * sin(phase)
        self.rider_forces.set_pedal_offsets(-drop, drop)

    def _compensate_cruise_gains(self) -> None:
        """
        Rescales the speed controller for the assist actually available this step.

        The taper is evaluated at the current speed rather than reused from the last command,
        so the gains and the support the drivetrain is about to apply belong to the same step.
        """
        if self.drivetrain is None:
            self.cruise.gain_scale = 1.0
            return
        taper = cutoff_factor(self.speed_mps, self.drivetrain.specs)
        self.cruise.set_assist_compensation(self.drivetrain.nominal_support_factor * taper)

    @property
    def pedal_command(self) -> Optional[CrankCommand]:
        """The last drivetrain command, or None when the ideal motor is driving."""
        return None if self.drivetrain is None else self.drivetrain.command

    @property
    def wheel_drive_torque_nm(self) -> float:
        """
        Drive torque arriving at the rear wheel, N.m.

        In `motor` mode that is the controller's output. In the pedalled modes it is the
        rider's torque plus the motor's, divided by the gearing -- the chain trades crank
        torque for wheel speed, the way 32 teeth driving 14 must.
        """
        if self.drivetrain is None:
            return float(self.cruise.torque_nm)
        command = self.drivetrain.command
        if command.freewheel:
            return 0.0
        return (
            (command.rider_torque_nm + command.assist_torque_nm)
            / self.drivetrain.specs.gear_ratio
        )

    def default_limits(self, max_wall_clock_s: float = DEFAULT_MAX_WALL_CLOCK_S) -> RunLimits:
        """
        Builds the run limits for a traverse of this run's track.

        Args:
            max_wall_clock_s: Real seconds the run may take.

        Returns:
            Limits derived from the track length, the compiled timestep and the start position.
        """
        return RunLimits.for_track(
            self.track,
            timestep_s=float(self.model.opt.timestep),
            start_x_m=self.start_x_m,
            max_wall_clock_s=max_wall_clock_s,
        )

    def run(
        self,
        limits: Optional[RunLimits] = None,
        on_step: Optional[Callable[["RideSimulation"], None]] = None,
        front_brake_demand: float = 0.0,
        rear_brake_demand: float = 0.0,
    ) -> RunOutcome:
        """
        Rides until the run terminates, and reports why it did.

        The run advances from the *current* state, so a caller that wants to start at the
        track's beginning calls `reset` first. Termination is checked before the first step, so
        a run whose limits are already met returns without stepping instead of overshooting by
        one.

        Args:
            limits: Bounds the run is judged against. Defaults to `default_limits()`.
            on_step: Called with this simulation after every step, for recording. Nothing here
                stores a trace: a 115 m traverse is 34 710 steps and the caller decides which
                channels are worth the memory.
            front_brake_demand: Front brake lever position in [0, 1], held for the whole run.
            rear_brake_demand: Rear brake lever position in [0, 1], held for the whole run.

        Returns:
            The reason the run ended, together with the state it ended in.
        """
        terminator = RunTerminator(limits if limits is not None else self.default_limits())
        terminator.start()

        while True:
            reason = terminator.reason(self.position_m, self.steps, self.crash)
            if reason is not None:
                return terminator.outcome(
                    reason, self.steps, self.time_s, self.position_m, self.crash
                )
            self.step(front_brake_demand, rear_brake_demand)
            if on_step is not None:
                on_step(self)

    @property
    def include_rider(self) -> bool:
        """Whether a rider with mass is present (legacy name)."""
        return self.rider.present

    @property
    def rider_variant(self) -> str:
        """The rider variant this run was compiled with: ``none``, ``lumped`` or ``seated``."""
        return self.rider.variant

    @property
    def time_s(self) -> float:
        """Simulation time since the start of the run, in seconds."""
        return float(self.data.time)

    @property
    def position_m(self) -> float:
        """Chassis position along the track, in metres."""
        return float(self.data.qpos[self.root_x_qposadr])

    @property
    def speed_mps(self) -> float:
        """Chassis longitudinal velocity, in m/s."""
        return float(self.data.qvel[self.root_x_dofadr])

    @property
    def pitch_rad(self) -> float:
        """Chassis pitch, in radians. Positive is nose-down about the bottom bracket."""
        return float(self.data.qpos[self.root_pitch_qposadr])

    @property
    def fork_travel_mm(self) -> float:
        """Fork compression, in millimetres of shaft travel."""
        return float(self.data.qpos[self.applier.fork_qposadr]) * 1000.0

    @property
    def shock_stroke_mm(self) -> float:
        """Rear shock compression, in millimetres of shaft stroke."""
        return float(self.data.qpos[self.applier.shock_qposadr]) * 1000.0

    @property
    def rear_travel_mm(self) -> float:
        """Rear wheel travel for the current shaft stroke, from the analytical solver."""
        return float(self.solver.solve_state_from_shock_stroke(self.shock_stroke_mm)["wheel_travel"])

    @property
    def crash(self) -> Optional[CrashEvent]:
        """The latched crash event, or None while the run is still upright."""
        return self.crash_detector.event


def _default_controller(specs: BikeSpecs) -> SuspensionController:
    """
    Builds the suspension force calculator from the shipped defaults.

    Args:
        specs: Bicycle geometry, supplying the fork travel, token count and pressure.

    Returns:
        A `SuspensionController` configured as the shipped bike.
    """
    return SuspensionController(
        specs=specs,
        air_spring=ForkAirSpring(
            specs=AirSpringSpecs(total_travel_mm=specs.fork_travel),
            num_tokens=specs.fork_air_tokens,
            gauge_pressure_psi=specs.fork_initial_psi,
        ),
        suspension_system=BikeSuspensionSystem(),
    )


def _actuator_id(model: mujoco.MjModel, name: str) -> int:
    """
    Resolves a named actuator's index into `data.ctrl`.

    Args:
        model: Compiled ride-mode model.
        name: Actuator name.

    Returns:
        The actuator's index.

    Raises:
        ValueError: If the model has no actuator of that name.
    """
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"model has no actuator '{name}'; the ride torque path needs it")
    return int(aid)


def _root_addresses(model: mujoco.MjModel, joint_name: str) -> Tuple[int, int]:
    """
    Resolves a chassis root joint's qpos and dof addresses.

    Args:
        model: Compiled ride-mode model.
        joint_name: Name of the root joint.

    Returns:
        Tuple of (qpos address, dof address).

    Raises:
        ValueError: If the model has no joint of that name.
    """
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise ValueError(f"model has no joint '{joint_name}'; ride mode needs a planar root")
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


__all__ = ["RideSimulation", "DEFAULT_START_X_M"]
