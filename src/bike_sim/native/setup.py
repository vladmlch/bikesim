"""Setup boundary for the owned native ride runtime (design section 3.1).

``validate_supported`` gates the resolved configuration against the design
section-1 capability set — structural predicates, never filenames.
``capture_bootstrap`` freezes the complete t=0 state inventory (compiled
model, integration vector, mutable model coefficients, mechanical and
controller state) into an owning ``RuntimeBootstrap`` the native
``NativeRideRuntime`` restores verbatim.
"""
from dataclasses import asdict, dataclass
import hashlib
from pathlib import Path

import mujoco
import numpy as np

from bike_sim.native.config import project as _project_writers
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.tire import TireSpec
from bike_sim.sim.ride.physical_samples import plain

RUNTIME_SCHEMA = 1


def _reject(path: str, actual, supported) -> None:
    raise ValueError(
        f'unsupported native configuration: {path}={actual!r} '
        f'(supported: {supported!r})')


def validate_supported(cfg: SimulationPhysicsConfig,
                       rider: RiderSpecs) -> None:
    """Require the design section-1 capability set on resolved fields.

    Every rejection names the full configuration field path. Supported
    values include seated-climb on/off, assist/battery/shifting settings,
    numerical tuning and valid rider geometry.
    """
    if not isinstance(cfg, SimulationPhysicsConfig):
        raise ValueError(
            f'expected a resolved SimulationPhysicsConfig, got {type(cfg).__name__}')
    if not isinstance(rider, RiderSpecs):
        raise ValueError(f'expected RiderSpecs, got {type(rider).__name__}')

    checks = {
        'physics_mode': (cfg.physics_mode, ('physical',)),
        'drive_mode': (cfg.drive_mode, ('articulated_effort',)),
        'pitch_assist': (cfg.pitch_assist, (False,)),
        'rider.variant': (rider.variant, ('articulated_planar',)),
        'articulated.pedal_attachment': (cfg.articulated.pedal_attachment, ('spindle',)),
        'articulated.saddle_attachment': (cfg.articulated.saddle_attachment, ('pin',)),
        'articulated.grip_attachment': (cfg.articulated.grip_attachment, ('connect',)),
        'tires.backend': (cfg.tires.backend, ('compliant_2d',)),
        'tires.surface_mode': (cfg.tires.surface_mode, ('track',)),
        'drive.transmission_model': (cfg.drive.transmission_model, ('ideal_mid_drive',)),
        'drive.motor_clutch': (cfg.drive.motor_clutch, (False,)),
        'drive.rotor_inertia_kgm2': (cfg.drive.rotor_inertia_kgm2, (0.,)),
    }
    for path, (actual, supported) in checks.items():
        if actual not in supported:
            _reject(path, actual, supported[0] if len(supported) == 1
                    else supported)
    for side in ('front', 'rear'):
        material = getattr(cfg.tires, side).material
        if not isinstance(material, TireSpec):
            _reject(f'tires.{side}.material', type(material).__name__,
                    'analytic TireSpec')


def _posture_dict(posture) -> dict | None:
    return None if posture is None else {
        'torso_lean_rad': posture.torso_lean_rad,
        'pelvis_pitch_rad': posture.pelvis_pitch_rad,
        'pelvis_offset_m': (None if posture.pelvis_offset_m is None
                            else list(posture.pelvis_offset_m)),
        'use_saddle': posture.use_saddle}


def _control_dict(control) -> dict:
    """RideControl -> owned wire dict, preserving None vs zero."""
    return {'motor_torque_nm': control.motor_torque_nm,
            'motor_limit_nm': control.motor_limit_nm,
            'human_torque_nm': control.human_torque_nm,
            'crank_target_rate_rad_s': control.crank_target_rate_rad_s,
            'posture': _posture_dict(control.posture),
            'rider_enabled': bool(control.rider_enabled)}


def _signals_dict(signals) -> dict:
    """SeatedClimbSignals -> owned wire dict."""
    return {'pitch_rate_up_rad_s': signals.pitch_rate_up_rad_s,
            'specific_force_body_mps2': list(signals.specific_force_body_mps2),
            'crank_rate_rad_s': signals.crank_rate_rad_s,
            'human_crank_torque_nm': signals.human_crank_torque_nm,
            'front_load_share': signals.front_load_share}


def _contacts_state(contacts) -> dict:
    """sim.contacts — the durable TerrainContacts the next step consumes."""
    def slip(snapshot):
        return None if snapshot is None else snapshot.slip_mps
    return {'front_load_n': contacts.front_load_n,
            'rear_load_n': contacts.rear_load_n,
            'front_support_n': contacts.front_support_n,
            'rear_support_n': contacts.rear_support_n,
            'handlebar_load_n': contacts.handlebar_load_n,
            'front_controller_grounded': bool(contacts.front_controller_grounded),
            'rear_controller_grounded': bool(contacts.rear_controller_grounded),
            'front_slip_mps': slip(contacts.front_snapshot),
            'rear_slip_mps': slip(contacts.rear_snapshot)}


def _crash_state(sim) -> dict:
    """sim.crash_detector — the latched CrashEvent, or none."""
    event = sim.crash_detector.event
    return {'event': None if event is None else {
        'cause': event.cause, 'time_s': event.time_s,
        'position_m': event.position_m, 'pitch_rad': event.pitch_rad}}


def _runtime_state(sim, physical) -> dict:
    """Runtime scalars/clocks/history — owner: PhysicalRuntime et al.

    Field-to-owner map: ``step``/``generation`` are sim.steps and
    physical.generation; ``held_control`` is ControlClock's held command;
    ``filters``/``balance``/``crash`` are the GroundedFilter, BalanceMonitor
    and CrashDetector state_dicts; ``contact_query``/``probe_query`` carry
    the TerrainContactQuery held-load bridges and grounded debounce;
    ``contacts`` is the last published TerrainContacts; ``energy``/
    ``history``/``model_status``/``monitor`` mirror the runtime's work
    counters, WorkHistory, ModelStatus and ReferenceMonitor.
    """
    history = physical.history
    status = physical.model_status
    monitor = getattr(physical, 'reference_monitor', None)
    energy = physical.energy
    return {
        'step': int(sim.steps),
        'generation': int(physical.generation),
        'record_decimation': int(physical.record_decimation),
        'applied_control': _control_dict(physical.applied_control),
        'held_control': physical.control_clock.held_state(),
        'held_rider_terms': plain(physical.held_rider_terms),
        'rollback_hold': bool(physical.rollback_hold),
        'research_accounting_valid': bool(physical.research_accounting_valid),
        'initializing': bool(getattr(physical, 'initializing', False)),
        'filters': {side: physical.filters[side].state_dict()
                    for side in ('front', 'rear')},
        'balance': physical.balance_monitor.state_dict(),
        'crash': _crash_state(sim),
        'contacts': _contacts_state(sim.contacts),
        'contact_query': sim.contact_query.state_dict(),
        'probe_query': physical.probe_query.state_dict(),
        'energy': {
            'initial_energy_j': physical.initial_energy_j,
            'energy_scale_j': physical.energy_scale_j,
            'loss_j': physical.loss_j,
            'active_work_j': physical.active_work_j,
            'external_work_j': physical.external_work_j,
            'solver_work_j': physical.solver_work_j,
            'muscle_signed_j': physical.muscle_signed_j,
            'muscle_positive_j': physical.muscle_positive_j,
            'motor_signed_j': physical.motor_signed_j,
            'motor_positive_j': physical.motor_positive_j,
            'constraint_absolute_j': physical.constraint_absolute_j,
            'initial_battery_j': physical.initial_battery_j,
            'electrical_work_j': physical.electrical_work_j,
            'mechanical_energy_j': energy['mechanical_energy_j'],
            'elastic_energy_j': dict(energy['elastic_energy_j']),
            'residual_j': energy['residual_j'],
        },
        'history': {
            'work_j': dict(history.work_j),
            'last_id': history.last_id,
            'last_end': history.last_end,
            'airtime_s': {side: dict(buckets)
                          for side, buckets in history.airtime_s.items()},
            'duration_s': history.duration_s,
        },
        'model_status': {
            'counts': dict(status.counts),
            'first': plain(status.first),
            'last_interval': status.last_interval,
            'maximum_compression_fraction': status.maximum_compression_fraction,
            'maximum_linkage_error_m': status.maximum_linkage_error_m,
            'numerically_valid': status.numerically_valid,
            'calibration_status': status.calibration_status,
        },
        'monitor': {
            'strict': monitor.strict,
            'first_failure': plain(monitor.first_failure),
        } if monitor is not None else {'strict': bool(physical.strict),
                                       'first_failure': None},
    }


def _joint_ref(model, dofadr: int, what: str) -> dict:
    joint = int(model.dof_jntid[dofadr])
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    if name is None:
        raise ValueError(f'{what} dof {dofadr} resolves to an unnamed joint')
    return {'joint': name, 'dof_index': int(dofadr - model.jnt_dofadr[joint])}


def _model_mutable(sim, physical, model) -> dict:
    """Running coefficient state Python physics writes into mjModel.

    Brake bounds: StaticBrakeApplier writes dof_frictionloss at the wheel
    dof addresses — emitted name-keyed so the native side re-resolves them.
    The compiled-COM visualization marker is a site_pos row. Tendon
    ranges/coefficients ride inside the ``drive`` snapshot instead.
    """
    brake = physical.brake
    frictionloss = {
        label: {**_joint_ref(model, dof, 'brake'),
                'value': float(model.dof_frictionloss[dof])}
        for label, dof in (('front', brake.front), ('rear', brake.rear))}
    site_pos = {}
    site_id = sim.cg_site_id
    if site_id >= 0:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, int(site_id))
        if name is None:
            raise ValueError('compiled com marker site is unnamed')
        site_pos[name] = model.site_pos[int(site_id)].copy()
    return {'dof_frictionloss': frictionloss, 'site_pos': site_pos}


def _geometry(sim, physical, model) -> dict:
    """Resolved named IDs and geometry the running loop needs (design 3.1)."""
    out = {
        'crank_length_m': float(sim.crank_length_m),
        'brake_dofs': {
            'front': _joint_ref(model, physical.brake.front, 'front brake'),
            'rear': _joint_ref(model, physical.brake.rear, 'rear brake'),
        },
        'wheel_bodies': {
            side: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            for side, body_id in zip(('front', 'rear'), physical.wheel_bodies)},
        'com_marker_site': (None if sim.cg_site_id < 0 else mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_SITE, sim.cg_site_id)),
    }
    control = physical.rider_control
    out['pose'] = (None if control is None
                   else plain(asdict(control.pose)))
    return out


@dataclass(frozen=True)
class RuntimeBootstrap:
    """An owning, reusable t=0 capture for the native runtime.

    ``config`` is the runtime envelope (runtime_schema=1, the nested
    schema-2 writer config, controller/intent/monitor settings, resolved
    geometry). ``state`` is the complete decoded state inventory —
    integration vector, model identity, mutable model coefficients,
    mechanical snapshots and runtime scalars. All arrays own their storage.
    """
    model_path: Path
    config: dict
    state: dict


def capture_bootstrap(sim, directory: Path) -> RuntimeBootstrap:
    """Freeze a freshly initialized physical ride into a RuntimeBootstrap.

    Requires t=0 on a completed initialization: ``sim.steps == 0``,
    ``data.time == 0`` and a finished physical reset (the equilibrium,
    settle and accounting baselines all exist). Bootstrap capture follows
    the frontend's initialization exactly once (design section 3.1); it is
    not a mid-run checkpoint format.
    """
    physical = getattr(sim, 'physical', None)
    if physical is None:
        raise ValueError('capture_bootstrap requires a physical ride runtime')
    validate_supported(sim.physics_config, sim.rider)
    if not physical.research_accounting_valid:
        raise ValueError('bootstrap capture requires a completed physical reset')
    if getattr(physical, 'initializing', False):
        raise ValueError('bootstrap capture requires a finished initialization')
    if sim.steps != 0 or float(sim.data.time) != 0.:
        raise ValueError(
            f'bootstrap capture requires t=0 (steps={sim.steps}, '
            f'time={float(sim.data.time)})')
    model = sim.model

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    model_path = directory / 'model.mjb'
    mujoco.mj_saveModel(model, str(model_path))
    digest = hashlib.sha256(model_path.read_bytes()).hexdigest()

    width = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION)
    values = np.empty(width, dtype=np.float64)
    mujoco.mj_getState(model, sim.data, values, mujoco.mjtState.mjSTATE_INTEGRATION)
    if not np.all(np.isfinite(values)):
        raise ValueError('bootstrap integration state contains non-finite values')

    config = {
        'runtime_schema': RUNTIME_SCHEMA,
        'timestep_s': float(model.opt.timestep),
        'control_period_s': float(sim.physics_config.control_period_s),
        'control_period_steps': int(physical.control_clock.steps_per_period),
        'physics_mode': sim.physics_config.physics_mode,
        'drive_mode': sim.physics_config.drive_mode,
        'strict': bool(physical.strict),
        'record_decimation': int(physical.record_decimation),
        'writer_config': _project_writers(sim),
        'rider_controller': plain(asdict(sim.physics_config.articulated)),
        'rider_intent': plain(asdict(sim.physics_config.seated_climb)),
        'monitors': {
            'balance_floor_mps': float(physical.balance_monitor.floor_mps),
            'balance_dwell_s': float(physical.balance_monitor.dwell_s),
            'balance_grace_s': float(physical.balance_monitor.grace_s),
            'grounded_hold_s': float(physical.filters['front'].hold_s),
        },
        'geometry': _geometry(sim, physical, model),
    }

    state = {
        'schema': RUNTIME_SCHEMA,
        'model_digest': digest,
        'model_dims': {name: int(getattr(model, name)) for name in
                       ('nq', 'nv', 'nu', 'na', 'nbody', 'njnt', 'ntendon',
                        'nwrap', 'nsite', 'ngeom', 'neq', 'nsensor',
                        'nsensordata')},
        'integration_state': values,
        'model_mutable': _model_mutable(sim, physical, model),
        'tire': physical.tire.state_dict(),
        'drive': physical.drive.state_dict(model),
        'rider_contacts': (None if physical.rider_contacts is None
                           else physical.rider_contacts.state_dict()),
        'rider_controller': (None if physical.rider_control is None
                             else physical.rider_control.state_dict()),
        'rider_intent': physical.rider_intent.state_dict(),
        'signals': _signals_dict(physical.rider_intent_signals),
        'runtime': _runtime_state(sim, physical),
    }
    return RuntimeBootstrap(model_path=model_path, config=config, state=state)
