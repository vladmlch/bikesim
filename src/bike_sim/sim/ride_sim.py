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

import dataclasses
from math import isfinite, sin
from typing import Any, Callable, Dict, Mapping, Optional, Tuple, Union

import mujoco
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.hardpoints import compute_ground_z
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs
from bike_sim.physics.damper import BikeSuspensionSystem
from bike_sim.physics.drivetrain import DRIVE_MODES, DrivetrainSpecs, cutoff_factor
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.physics.rider import DEFAULT_RIDER_VARIANT, RiderSpecs, SeatedPose, resolve_rider
from bike_sim.physics.suspension_config import build_suspension_components
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.controllers import SuspensionController
from bike_sim.sim.equilibrium import RELAX_CYCLE_S, solve_static_equilibrium
from bike_sim.sim.ride.braking import BrakeController
from bike_sim.sim.ride.contacts import TerrainContactQuery, TerrainContacts
from bike_sim.sim.ride.cruise import DEFAULT_TARGET_SPEED_KMH, CruiseController
from bike_sim.sim.ride.constraint_forces import (
    ConstraintForceSnapshot,
    shock_joint_limit_qfrc,
)
from bike_sim.sim.ride.drivetrain import CrankCommand, PedalDrivetrain
from bike_sim.sim.ride.force_accumulator import ForceAccumulator
from bike_sim.sim.ride.forces import SuspensionForceApplier
from bike_sim.sim.ride.leg_drive import LegDrive
from bike_sim.sim.ride.mass_properties import compiled_center_of_mass, static_contact_loads
from bike_sim.sim.ride.telemetry_v2 import ForceSample
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
        legs: Optional[str] = None,
        visual_pedalling: bool = False,
        *,
        physics_config: Optional[SimulationPhysicsConfig] = None,
        mass_specs: Optional[BikeMassSpecs] = None,
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
            legs: Which legs the seated rider gets -- ``rigid`` keeps the lumped leg
                masses on their slide joints, ``articulated`` hangs hip/knee/ankle chains
                off the pelvis and welds the feet to the pedals. ``None`` picks
                ``articulated`` whenever the crankset turns (``pedal``/``pedelec``, or
                ``visual_pedalling``) and ``rigid`` otherwise. Supersedes ``rider.legs``:
                a ``legs`` set inside the ``rider=`` spec is overwritten by this kwarg's
                resolution.
            visual_pedalling: Build the turning crankset and articulated legs in
                ``motor`` mode too, so the pedals and feet visibly spin while the ideal
                wheel actuator drives; the legs are dragged, they deliver no torque.
                Redundant in the pedalled modes, where the crankset turns anyway.
            physics_config: Physical model selection and timestep. An omitted config uses
                the existing legacy simulation and its historical drive mode.
            mass_specs: Component mass budget passed to the MJCF builder.

        Raises:
            ValueError: If the track does not fit the heightfield envelope.
            RuntimeError: If the starting equilibrium does not converge.
        """
        if drive_mode not in DRIVE_MODES:
            raise ValueError(
                f"unknown drive mode {drive_mode!r}; available: {', '.join(DRIVE_MODES)}"
            )
        self.physics_config = physics_config if physics_config is not None else SimulationPhysicsConfig()
        self.mass_specs = mass_specs if mass_specs is not None else BikeMassSpecs()
        if self.physics_config.drive_mode in {"crank_effort", "articulated_effort"}:
            raise NotImplementedError(
                f"{self.physics_config.drive_mode} is not implemented in this physics revision"
            )
        if self.physics_config.physics_mode == "physical" and drive_mode != "motor":
            raise ValueError("physical mode uses physics_config.drive_mode, not legacy drive_mode")
        self.physics_revision = (
            "legacy-v1" if self.physics_config.physics_mode == "legacy" else "physical-v1"
        )
        self.drive_mode = drive_mode
        self.drivetrain_specs = drivetrain if drivetrain is not None else DrivetrainSpecs()
        pedalled = drive_mode != "motor"
        self.visual_pedalling = bool(visual_pedalling)
        crank_joint = pedalled or self.visual_pedalling

        self.track = track if track is not None else get_preset(DEFAULT_PRESET)
        self.field = field if field is not None else HeightFieldSpec.for_track(self.track)
        assert_track_fits(self.track, self.field)

        self.specs = specs if specs is not None else BikeSpecs()
        if not isfinite(self.specs.shock_stroke):
            raise ValueError(f"BikeSpecs shock_stroke {self.specs.shock_stroke} must be finite")
        if controller is not None:
            _validate_controller_geometry(self.specs, controller)
        if coil_shock is not None and not isfinite(coil_shock.specs.stroke_mm):
            raise ValueError(f"CoilShock stroke_mm {coil_shock.specs.stroke_mm} must be finite")
        if (
            coil_shock is not None
            and abs(coil_shock.specs.stroke_mm - self.specs.shock_stroke) > 1e-6
        ):
            raise ValueError(
                f"CoilShock stroke_mm {coil_shock.specs.stroke_mm} differs from "
                f"BikeSpecs shock_stroke {self.specs.shock_stroke}"
            )
        if self.physics_config.physics_mode == "physical":
            default_controller, default_coil = build_suspension_components(self.specs)
        else:
            default_controller = _default_controller(self.specs, legacy_behavior=True)
            default_coil = CoilShock(
                CoilShockSpecs(
                    rate_n_m=self.specs.shock_stiffness,
                    stroke_mm=self.specs.shock_stroke,
                ),
                legacy_behavior=True,
            )
        resolved_controller = controller if controller is not None else default_controller
        resolved_coil = coil_shock if coil_shock is not None else default_coil
        self.tyre_config = tyre if tyre is not None else TyreConfig()
        if (
            self.physics_config.physics_mode == "physical"
            and self.tyre_config.pneumatic
            and self.tyre_config.tier == "detailed"
            and self.physics_config.timestep_s != self.tyre_config.tier_spec.timestep_s
        ):
            raise ValueError(
                "detailed pneumatic tyre requires timestep_s="
                f"{self.tyre_config.tier_spec.timestep_s}; "
                f"got {self.physics_config.timestep_s}"
            )
        self.solver = HorstLinkageSolver(self.specs)
        self.start_x_m = float(start_x_m)
        # The legs choice lands on the rider *before* the pose is solved: `seated_pose`
        # reads `rider.legs` to decide between slide-mounted leg masses and the hip-knee-
        # ankle chains, and the pose the builder compiles must be the same one this run's
        # force path is configured from.
        self.rider: RiderSpecs = dataclasses.replace(
            resolve_rider(
                rider, include_rider=include_rider, default_variant=DEFAULT_RIDER_VARIANT
            ),
            legs=legs or ("articulated" if crank_joint else "rigid"),
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
                crank_joint=crank_joint,
                gear_ratio=self.drivetrain_specs.gear_ratio,
                physics_config=self.physics_config,
                mass_specs=self.mass_specs,
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
        self.force_accumulator = ForceAccumulator(self.model.nv)
        self.last_force_snapshot: Optional[
            Tuple[float, np.ndarray, np.ndarray, Mapping[str, np.ndarray]]
        ] = None
        self.last_force_sample: Optional[ForceSample] = None
        self.last_constraint_snapshot: Optional[ConstraintForceSnapshot] = None

        self.controller = resolved_controller
        self.applier = SuspensionForceApplier(
            self.model,
            self.controller,
            resolved_coil,
            physics_config=self.physics_config,
        )
        self.rider_forces = RiderForceApplier(self.model, self.pose)
        self.contact_query = TerrainContactQuery(self.model)
        self.resistance = RollingResistance(self.model)
        self.cruise = CruiseController(self.model, target_speed_kmh=target_speed_kmh)
        # Crank arm length in metres, for the pedal positions the rider's legs follow.
        self.crank_length_m = float(self.specs.crank_length) / 1000.0
        # Inert without articulated legs, so every caller can treat it as always present.
        self.leg_drive = LegDrive(
            self.model,
            self.pose,
            self.crank_length_m,
            ripple_depth=self.drivetrain_specs.ripple_depth,
        )
        self.drivetrain: Optional[PedalDrivetrain] = (
            PedalDrivetrain(
                self.model,
                specs=self.drivetrain_specs,
                drive_mode=drive_mode,
                assist_mode=assist,
                legs_drive=self.leg_drive.active,
            )
            if pedalled
            else None
        )
        self.brake_source_cruise = False
        self.brakes = BrakeController(self.model)
        self.stabilizer = PitchStabilizer(self.model)
        self.crash_detector = CrashDetector(self.model)

        self.drive_ctrl_adr = _actuator_id(self.model, "rear_drive")
        self.crank_drive_ctrl_adr = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "crank_drive"
        )
        self.front_brake_ctrl_adr = _actuator_id(self.model, "front_brake")
        self.rear_brake_ctrl_adr = _actuator_id(self.model, "rear_brake")
        self.root_x_qposadr, self.root_x_dofadr = _root_addresses(self.model, "root_x")
        self.root_pitch_qposadr, _ = _root_addresses(self.model, "root_pitch")
        self._cg_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "site_CG")
        self._frame_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "frame")

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
                leg_drive=self.leg_drive,
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
                leg_drive=self.leg_drive,
            )
        self.contact_query.reset()
        self.cruise.reset()
        if self.drivetrain is not None:
            self.drivetrain.reset(self.model, self.data)
            self.rider_forces.set_pedal_offsets(0.0, 0.0)
            if self.leg_drive.active:
                # The drivetrain has just written the crank's start phase; pose the legs
                # and pedal platforms to match it before the forward pass, so the foot
                # welds begin residual-free at any `--crank-phase`.
                self.leg_drive.initialize(
                    self.model,
                    self.data,
                    float(self.data.qpos[self.drivetrain.crank_qposadr]),
                )
            mujoco.mj_forward(self.model, self.data)
        elif self.leg_drive.active:
            # Motor mode with visual pedalling: no drivetrain touches the crank, which
            # stays wherever the equilibrium left it; pose the legs to match.
            self.leg_drive.initialize(
                self.model, self.data, float(self.data.qpos[self.leg_drive.crank_qposadr])
            )
            mujoco.mj_forward(self.model, self.data)
        self.brake_source_cruise = False
        self.brakes.reset()
        self.resistance.reset()
        self.stabilizer.reset()
        self.crash_detector.reset()
        self.data.time = 0.0
        self.steps = 0
        self.force_accumulator.clear()
        self.last_force_snapshot = None
        self.last_force_sample = None
        self.last_constraint_snapshot = None
        if self.tyre_applier is None:
            self.contacts = self.contact_query.query(self.model, self.data)
        else:
            self.contacts = self.contact_query.query(
                self.model, self.data, wheel_load_provider=self.tyre_applier
            )
        self._update_compiled_com_marker()
        if self.physics_config.physics_mode == "physical" and self.tyre_applier is None:
            front = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_front_contact")
            rear = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_rear_contact")
            front_load, rear_load = static_contact_loads(self.model, self.data, front, rear)
            self.equilibrium["static_front_load_n"] = front_load
            self.equilibrium["static_rear_load_n"] = rear_load

    def _update_compiled_com_marker(self) -> None:
        """Place the physical-mode site at the current compiled system CoM."""
        if self.physics_config.physics_mode != "physical" or self._cg_site_id < 0:
            return
        mujoco.mj_kinematics(self.model, self.data)
        com = compiled_center_of_mass(self.model, self.data)
        frame = self._frame_body_id
        self.model.site_pos[self._cg_site_id] = self.data.xmat[frame].reshape(3, 3).T @ (com - self.data.xpos[frame])
        self.data.site_xpos[self._cg_site_id] = com

    def step(
        self,
        front_brake_demand: float = 0.0,
        rear_brake_demand: float = 0.0,
        *,
        external_qfrc: Optional[np.ndarray] = None,
    ) -> None:
        """Advance one timestep using the selected force and control path."""
        if self.physics_config.physics_mode == "legacy":
            if external_qfrc is not None:
                raise ValueError("external_qfrc is only supported in physical mode")
            self._step_legacy(front_brake_demand, rear_brake_demand)
        else:
            self._step_physical(front_brake_demand, rear_brake_demand, external_qfrc)

    def _step_legacy(self, front_brake_demand: float, rear_brake_demand: float) -> None:
        """
        Advance the historical ride model by one timestep.

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
        if self.tyre_applier is None:
            self.resistance.apply(self.model, self.data, self.contacts)
        traction_limited = False
        drive_torque = 0.0
        if self.physics_config.drive_mode == "ideal_speed_control":
            self._compensate_cruise_gains()
            if self.tyre_applier is not None:
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
            # Motor mode with visual pedalling: the chain equality spins the crank off
            # the driven wheel and the impedance drags the legs along; zero rider torque.
            # With no leg chains at all this no-ops.
            self.leg_drive.apply(self.model, self.data, 0.0)
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
            # With articulated legs this is where the rider's torque actually enters --
            # through the legs and the foot-pedal welds, not the crank actuator. With
            # rigid legs it no-ops and the actuator carries the rider as before.
            self.leg_drive.apply(self.model, self.data, command.rider_torque_nm)
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
        if self.physics_config.pitch_assist:
            self.stabilizer.apply(self.model, self.data, self.contacts)
        else:
            self.stabilizer.disable(self.data)

        # Checked against the same snapshot the writers saw, so a reported crash position is
        # the state that produced it rather than the state one timestep later.
        self.crash_detector.check(self.data, self.contacts)

        mujoco.mj_step(self.model, self.data)
        self._update_compiled_com_marker()
        self.steps += 1
        if self.tyre_applier is None:
            self.contacts = self.contact_query.query(self.model, self.data)

    def _collect_legacy_qfrc(self, name: str, writer: Callable[[], None]) -> None:
        """Adapt an assigning legacy writer to one named, additive contribution."""
        self.data.qfrc_applied.fill(0.0)
        try:
            writer()
            self.force_accumulator.add(name, self.data.qfrc_applied)
        finally:
            self.data.qfrc_applied.fill(0.0)

    def _step_physical(
        self,
        front_brake_demand: float,
        rear_brake_demand: float,
        external_qfrc: Optional[np.ndarray],
    ) -> None:
        """Advance the physical model with forces read from current kinematics.

        ``contacts`` is the snapshot used by this step's force writers. It remains the
        pre-step snapshot until the next call refreshes MuJoCo's kinematics.
        """
        external = None
        if external_qfrc is not None:
            # Copy before clearing: callers may explicitly pass a view of MuJoCo's array.
            external = np.array(external_qfrc, dtype=float, copy=True)
            if external.shape != (self.model.nv,) or not np.isfinite(external).all():
                raise ValueError("invalid generalized force")
            external.setflags(write=False)
        # MuJoCo retains both arrays after a step. The current-state forward pass must
        # not see force input from the previous interval; explicit input is kept apart.
        self.stabilizer.disable(self.data)
        self.data.qfrc_applied.fill(0.0)
        self.data.xfrc_applied.fill(0.0)
        mujoco.mj_forward(self.model, self.data)
        self.force_accumulator.clear()
        if external is not None:
            self.force_accumulator.add("external", external)
        if self.tyre_applier is not None:
            self.tyre_applier.apply(self.model, self.data)
            self.contacts = self.contact_query.query(
                self.model, self.data, wheel_load_provider=self.tyre_applier
            )
        else:
            self.contacts = self.contact_query.query(self.model, self.data)

        for name, qfrc in self.applier.compute_qfrc_components(self.model, self.data).items():
            self.force_accumulator.add(name, qfrc)
        if self.rider_forces.active:
            self._collect_legacy_qfrc(
                "rider", lambda: self.rider_forces.apply(self.model, self.data)
            )
        if self.tyre_applier is None:
            self._collect_legacy_qfrc(
                "rolling_resistance",
                lambda: self.resistance.apply(self.model, self.data, self.contacts),
            )

        self.data.ctrl[self.drive_ctrl_adr] = 0.0
        if self.crank_drive_ctrl_adr >= 0:
            self.data.ctrl[self.crank_drive_ctrl_adr] = 0.0
        if self.leg_drive.active:
            self._collect_legacy_qfrc(
                "leg_drive", lambda: self.leg_drive.apply(self.model, self.data, 0.0)
            )
        if self.physics_config.drive_mode == "ideal_speed_control":
            traction_limited = False
            if self.tyre_applier is not None:
                error_mps = self.cruise.target_speed_mps - float(
                    self.data.qvel[self.root_x_dofadr]
                )
                traction_limited = any(
                    patch.fully_sliding and patch.slip_ratio * error_mps > 0.0
                    for patch in self.tyre_applier.rear_outputs.patches
                )
            drive_torque = self.cruise.compute(
                self.model, self.data, self.contacts, traction_limited=traction_limited,
                controller_grounded=self.contacts.rear_controller_grounded,
            )
            if (front_brake_demand > 0.0 or rear_brake_demand > 0.0) and drive_torque > 0.0:
                drive_torque = 0.0
            self.data.ctrl[self.drive_ctrl_adr] = drive_torque

        front_torque, rear_torque = self.brakes.compute(
            self.data, front_brake_demand, rear_brake_demand
        )
        self.data.ctrl[self.front_brake_ctrl_adr] = front_torque
        self.data.ctrl[self.rear_brake_ctrl_adr] = rear_torque
        self.crash_detector.check(self.data, self.contacts)

        self.data.qfrc_applied[:] = self.force_accumulator.total()
        self.last_force_sample = ForceSample(
            float(self.data.time), self.data.qpos, self.data.qvel,
            self.force_accumulator.components,
        )
        self.last_force_snapshot = (
            self.last_force_sample.time_s,
            self.last_force_sample.qpos,
            self.last_force_sample.qvel,
            self.last_force_sample.components,
        )
        mujoco.mj_step(self.model, self.data)
        self.last_constraint_snapshot = ConstraintForceSnapshot(
            self.last_force_sample.time_s,
            float(self.data.time),
            self.last_force_sample.qvel,
            {"shock_solver_limit": shock_joint_limit_qfrc(self.model, self.data)},
        )
        self.steps += 1

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


def _validate_controller_geometry(specs: BikeSpecs, controller: SuspensionController) -> None:
    """Reject a force calculator built for geometry unlike the compiled bike."""
    for field in (
        "fork_travel", "shock_stroke", "shock_eye_to_eye", "rear_wheel_travel",
        "front_wheel_radius", "rear_wheel_radius", "wheel_radius",
    ):
        expected = getattr(specs, field)
        actual = getattr(controller.specs, field)
        if not isfinite(expected) or not isfinite(actual) or abs(expected - actual) > 1e-6:
            raise ValueError(f"BikeSpecs {field} {expected} differs from controller {field} {actual}")
    for field, actual in (
        ("fork_travel", controller.air_spring.specs.total_travel_mm),
        ("fork_travel", controller.suspension_system.fork_damper.total_travel_mm),
        ("shock_stroke", controller.suspension_system.shock_damper.total_stroke_mm),
    ):
        expected = getattr(specs, field)
        if not isfinite(expected) or not isfinite(actual) or abs(expected - actual) > 1e-6:
            raise ValueError(f"BikeSpecs {field} {expected} differs from component {field} {actual}")


def _default_controller(specs: BikeSpecs, *, legacy_behavior: bool) -> SuspensionController:
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
        suspension_system=BikeSuspensionSystem(legacy_behavior=legacy_behavior),
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
