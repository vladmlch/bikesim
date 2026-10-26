"""Persistent cache for validated physical initial poses.

Two reuse mechanisms share this module's wire format and strict ``restore_state``
contract: the derived-name auto-cache (``equilibrium_cache_enabled``) and explicit
user-managed pose files (``equilibrium_state_file``, written by ``bike-settle``).
The auto-cache names entries by :func:`fingerprint`; the pose file embeds it so a
load can prove the pose belongs to the resolved configuration.

Two namespaces coexist under the cache directory. The *canonical* one —
:func:`fingerprint`, a pure function of the plant — is the only key ``load``
ever reads, and only the construction-time settle (``generation == 0``, the
run that provably started from pristine material state) may write it. A
``reset()`` re-settle starts from post-episode state, so its pose is a
function of the episode that just ran: valid, but not canonical. Those
results are published under :func:`episode_fingerprint`, which additionally
binds the experiment context (``sim.equilibrium_context``) the owning
environment attaches — quarantined evidence of which experiment produced
which pose, never a cache hit.
"""
import hashlib
import json
import os
from dataclasses import asdict, is_dataclass
from pathlib import Path
import tempfile

import mujoco
import numpy as np

from bike_sim.sim.ride.physical_samples import plain
from bike_sim.terrain.trackfile import track_to_dict
from bike_sim.validation.environment import source_fingerprint


# 6: the canonical key now binds every plant input the pose derives from
# (settle options, speed target, active suspension overrides, drivetrain/tyre
# compile inputs, heightfield spec, explicit initial state). A re-settle's
# output moved to the quarantined episode namespace at the same bump.
CACHE_VERSION = 6


def _serializable(value):
    if is_dataclass(value):
        return _serializable(asdict(value))
    if isinstance(value, dict):
        return {str(key): _serializable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_serializable(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _update_array(digest, name, value):
    array = np.ascontiguousarray(np.asarray(value))
    digest.update(name.encode('utf-8'))
    digest.update(str(array.dtype).encode('ascii'))
    digest.update(repr(array.shape).encode('ascii'))
    digest.update(array.tobytes())


def _active_suspension(sim):
    """Effective suspension identity, mirroring ``configuration_metadata``.

    The suspension owners default to ``build_suspension_components(specs)``
    but callers may substitute their own controller/coil; those overrides are
    not reachable through ``sim.specs``. The recorded fields are the same ones
    the run metadata certifies.
    """
    controller = sim.controller
    return {
        'fork_pressure_psi': controller.air_spring.gauge_pressure_psi,
        'fork_tokens': controller.air_spring.num_tokens,
        'coil': _serializable(sim.applier.coil_shock.specs),
        'fork_damper': _serializable(
            controller.suspension_system.fork_damper.get_effective_coefficients()),
        'shock_damper': _serializable(
            controller.suspension_system.shock_damper.get_effective_coefficients()),
    }


def fingerprint(runtime) -> str:
    """The configuration digest a settled pose is pinned to.

    Covers everything that can change the equilibrium itself: source, engine,
    physics config, geometry, track, compiled model, the settle contract
    (step budget and residual tolerance), and the compile/run inputs that
    live outside ``physics_config`` — the speed-controller target, legacy
    drivetrain/tyre/compile knobs, the heightfield spec, effective suspension
    coefficients and an explicit t=0 transfer's identity. The reuse
    *mechanism* knobs — the auto-cache toggle and the pose-file pointer — are
    deliberately excluded (``initial_state._contract`` pops them likewise): a
    file path or a cache flag is not physics, and pinning them would make a
    written pose reject a consumer that only differs in how it reuses poses.

    The experiment context (``sim.equilibrium_context``) is just as
    deliberately excluded: the canonical entry must stay loadable by every
    experiment on the same plant, so only ``episode_fingerprint`` — the
    quarantine namespace — binds it.
    """
    sim = runtime.sim
    source_root = Path(__file__).resolve().parents[2]
    physics = _serializable(sim.physics_config)
    for mechanism in ('equilibrium_cache_enabled', 'equilibrium_state_file'):
        physics.pop(mechanism, None)
    payload = {
        'version': CACHE_VERSION,
        'source': source_fingerprint(source_root),
        'mujoco': mujoco.mj_versionString(),
        'physics': physics,
        'specs': _serializable(sim.specs),
        'mass_specs': _serializable(sim.mass_specs),
        'rider': _serializable(sim.rider),
        'track': plain(track_to_dict(sim.track)),
        'start_x_m': sim.start_x_m,
        'field': _serializable(sim.field),
        'drive_mode': sim.drive_mode,
        'visual_pedalling': sim.visual_pedalling,
        'drivetrain_specs': _serializable(sim.drivetrain_specs),
        'tyre_config': _serializable(sim.tyre_config),
        'target_speed_kmh': (sim.cruise.target_speed_kmh
                             if sim.physics_config.drive_mode == 'ideal_speed_control'
                             else None),
        'active_suspension': _active_suspension(sim),
        'equilibrium_max_steps': sim.equilibrium_max_steps,
        'equilibrium_tolerance': sim.equilibrium_tolerance,
        'initial_state': (None if sim.physical_initial_state is None
                          else sim.physical_initial_state.sha256),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    digest = hashlib.sha256(encoded)
    for name in ('qpos0', 'body_mass', 'body_ipos', 'body_inertia', 'geom_pos', 'geom_size',
                 'jnt_range', 'hfield_data'):
        _update_array(digest, name, getattr(sim.model, name))
    _update_array(digest, 'timestep', np.array([sim.model.opt.timestep], dtype='<f8'))
    return digest.hexdigest()


def episode_fingerprint(runtime, canonical=None) -> str:
    """The quarantine digest for a settle that did not start canonically.

    A ``reset()`` re-settle starts from post-episode material state, so its
    pose is a function of the episode that just ran — not of the plant alone.
    This key therefore binds the canonical fingerprint, the fact of the
    re-settle, and ``sim.equilibrium_context``: the experiment identity
    (config, demand, rider program/behavior, sensors, seed) the owning
    environment attaches to mark whose episode seeded the state. Two
    different experiment inputs yield different keys under one cache
    directory; a ``None`` context (a bare ``sim.reset()``) still lands
    outside the canonical namespace. Entries named by this digest are
    quarantined evidence — nothing loads them — so a history-dependent pose
    can never masquerade as the canonical one again.
    """
    canonical = fingerprint(runtime) if canonical is None else canonical
    payload = {
        'equilibrium': canonical,
        'resettle': True,
        'experiment': _serializable(getattr(runtime.sim, 'equilibrium_context', None)),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _cache_dir():
    root = os.environ.get('BIKE_SIM_EQUILIBRIUM_CACHE_DIR')
    return Path(root) if root else Path(tempfile.gettempdir()) / 'bike_sim_equilibrium'


def _cache_path(runtime):
    return _cache_dir() / f'{fingerprint(runtime)}.npz'


def _state_arrays(runtime):
    state = {}
    for name in ('ideal_hub','clutch'):
        constraint=getattr(runtime.drive,name)
        if constraint is not None:
            state['state_drive_'+name+'_boundary']=np.array([constraint.boundary],dtype=float)
    rider = runtime.rider_contacts
    if rider is not None:
        keys = tuple(rider.states)
        state['state_rider_keys'] = np.asarray(keys)
        state['state_rider_xi'] = np.asarray([rider.states[key].xi for key in keys], dtype=float)
        state['state_rider_tangent'] = np.asarray([
            value.tangent if value.tangent is not None else np.zeros(3)
            for value in (rider.states[key] for key in keys)
        ], dtype=float)
        state['state_rider_tangent_valid'] = np.asarray([
            value.tangent is not None for value in (rider.states[key] for key in keys)
        ], dtype=bool)
        # One shear vector per hand, stacked left-then-right.
        state['state_rider_grip_xi'] = np.asarray(
            [rider.grip_xi_local[side] for side in ('left', 'right')], dtype=float)
        state['state_rider_enabled'] = np.asarray([
            rider.enabled[name] for name in rider.CONTACTS
        ], dtype=bool)
        state['state_rider_pending_release_loss'] = np.asarray(
            [rider.pending_release_loss_j], dtype=float
        )
    control = getattr(runtime, 'rider_control', None)
    if control is not None and getattr(control, 'spindle', False):
        # The weld-press servo state is part of the equilibrium the pose was
        # certified against: restoring the pose without it would release the
        # pedal press the servo had bled off.
        state['state_control_coast_hold_nm'] = np.asarray(
            [control._coast_hold_nm], dtype=float)
    tire = runtime.tire
    if tire is not None and tire.config.backend == 'pneumatic':
        # The pneumatic carcass/brush state is not expressible in the xi/tangent
        # schema; its canonical wire is the applier's own state_dict.
        wire = tire.state_dict()
        state['state_tire_wire_names'] = np.asarray(wire['names'])
        state['state_tire_wire_row'] = np.asarray(wire['row'],dtype=float)
    elif tire is not None:
        distributed = tire.config.backend == 'distributed_2d_reference'
        sides = tuple(sorted(tire.states)) if distributed else tuple(tire.states)
        state['state_tire_sides'] = np.asarray([json.dumps(key) for key in sides])
        state['state_tire_xi'] = np.asarray([tire.states[side].xi for side in sides], dtype=float)
        state['state_tire_tangent'] = np.asarray([
            value.tangent if value.tangent is not None else np.zeros(3)
            for value in (tire.states[side] for side in sides)
        ], dtype=float).reshape(-1,3)
        state['state_tire_tangent_valid'] = np.asarray([
            value.tangent is not None for value in (tire.states[side] for side in sides)
        ], dtype=bool)
        if not distributed:
            state['state_tire_point'] = np.asarray([
                value.point if value.point is not None else np.zeros(3)
                for value in (tire.states[side] for side in sides)
            ], dtype=float).reshape(-1,3)
            state['state_tire_point_valid'] = np.asarray([
                value.point is not None for value in (tire.states[side] for side in sides)
            ], dtype=bool)
            state['state_tire_segment'] = np.asarray([
                -1 if value.segment is None else value.segment
                for value in (tire.states[side] for side in sides)
            ], dtype=int)
            state['state_tire_center'] = np.asarray([
                value.center if value.center is not None else np.zeros(3)
                for value in (tire.states[side] for side in sides)
            ], dtype=float).reshape(-1,3)
            state['state_tire_center_valid'] = np.asarray([
                value.center is not None for value in (tire.states[side] for side in sides)
            ], dtype=bool)
    return state


def restore_state(runtime, state):
    drive_boundaries=[]
    for name in ('ideal_hub','clutch'):
        constraint=getattr(runtime.drive,name)
        if constraint is not None:
            try:
                value=np.asarray(state.get('state_drive_'+name+'_boundary',()),dtype=float)
            except (ValueError,TypeError):
                return False
            if value.shape != (1,) or not np.isfinite(value).all():
                return False
            drive_boundaries.append((constraint,float(value[0])))
    rider = runtime.rider_contacts
    rider_update=None
    if rider is not None:
        required = {
            'state_rider_keys', 'state_rider_xi', 'state_rider_tangent',
            'state_rider_tangent_valid', 'state_rider_grip_xi',
            'state_rider_enabled', 'state_rider_pending_release_loss',
        }
        if not required.issubset(state):
            return False
        try:
            keys = tuple(str(value) for value in state['state_rider_keys'])
            xi = np.asarray(state['state_rider_xi'],dtype=float)
            tangent = np.asarray(state['state_rider_tangent'],dtype=float)
            valid = np.asarray(state['state_rider_tangent_valid'],dtype=bool)
            enabled = np.asarray(state['state_rider_enabled'],dtype=bool)
            grip_xi = np.asarray(state['state_rider_grip_xi'],dtype=float)
            pending = np.asarray(state['state_rider_pending_release_loss'],dtype=float)
        except (ValueError,TypeError):
            return False
        if keys != tuple(rider.states):
            return False
        if (xi.shape != (len(keys),) or tangent.shape != (len(keys), 3)
                or valid.shape != (len(keys),)
                or enabled.shape != (len(rider.CONTACTS),)
                or grip_xi.shape != (2, 3) or pending.shape != (1,)):
            return False
        if (not np.isfinite(xi).all() or not np.isfinite(tangent[valid]).all()
                or not np.isfinite(grip_xi).all() or not np.isfinite(pending).all()):
            return False
        rider_update=(
            {key:type(rider.states[key])(float(xi[index]),
                tangent[index].copy() if valid[index] else None)
             for index,key in enumerate(keys)},
            dict(zip(rider.CONTACTS,map(bool,enabled))),
            {side:grip_xi[index].copy() for index,side in enumerate(('left','right'))},
            float(pending[0]))
    elif any(key.startswith('state_rider_') for key in state):
        return False
    control = getattr(runtime, 'rider_control', None)
    coast_hold = None
    if control is not None and getattr(control, 'spindle', False):
        try:
            value = np.asarray(state.get('state_control_coast_hold_nm',()),dtype=float)
        except (ValueError,TypeError):
            return False
        if value.shape != (1,) or not np.isfinite(value).all():
            return False
        coast_hold = float(value[0])
    elif 'state_control_coast_hold_nm' in state:
        return False
    tire = runtime.tire
    tire_update=None
    if tire is not None and tire.config.backend == 'pneumatic':
        required = {'state_tire_wire_names','state_tire_wire_row'}
        if not required.issubset(state):
            return False
        try:
            names = [str(value) for value in state['state_tire_wire_names']]
            row = np.asarray(state['state_tire_wire_row'],dtype=float)
        except (ValueError,TypeError):
            return False
        if row.shape != (len(names),):
            return False
        # set_state is the only decoder and is itself all-or-nothing: every
        # canonical column and cell contract is checked before it commits.
        # The rider and drive updates queued above cannot fail, so committing
        # the tire here keeps the restore atomic.
        try:
            tire.set_state(names,row)
        except (ValueError,TypeError):
            return False
    elif tire is not None:
        distributed = tire.config.backend == 'distributed_2d_reference'
        required = {'state_tire_sides', 'state_tire_xi', 'state_tire_tangent', 'state_tire_tangent_valid'}
        if not distributed:
            required |= {'state_tire_point', 'state_tire_point_valid', 'state_tire_segment',
                         'state_tire_center', 'state_tire_center_valid'}
        if not required.issubset(state):
            return False
        try:
            decoded = [json.loads(str(value)) for value in state['state_tire_sides']]
            sides = tuple(tuple(key) if isinstance(key,list) else key for key in decoded)
        except (ValueError,TypeError):
            return False
        if not distributed and sides != tuple(tire.states):
            return False
        if distributed and (any(not isinstance(key,tuple) or len(key)!=2 or key[0] not in ('front','rear') or type(key[1]) is not int or not 0<=key[1]<tire.config.distributed.station_count for key in sides) or len(set(sides)) != len(sides)):
            return False
        try:
            xi = np.asarray(state['state_tire_xi'],dtype=float)
            tangent = np.asarray(state['state_tire_tangent'],dtype=float)
            tangent_valid = np.asarray(state['state_tire_tangent_valid'],dtype=bool)
        except (ValueError,TypeError):
            return False
        count = len(sides)
        if xi.shape != (count,) or tangent.shape != (count, 3) or tangent_valid.shape != (count,):
            return False
        if not np.isfinite(xi).all() or not np.isfinite(tangent[tangent_valid]).all():
            return False
        if distributed:
            from bike_sim.sim.ride.distributed_tire_forces import _StationState
            updates = {side: _StationState(float(xi[index]), tangent[index].copy() if tangent_valid[index] else None)
                       for index, side in enumerate(sides)}
        else:
            try:
                point = np.asarray(state['state_tire_point'],dtype=float)
                point_valid = np.asarray(state['state_tire_point_valid'],dtype=bool)
                segment = np.asarray(state['state_tire_segment'],dtype=float)
                center = np.asarray(state['state_tire_center'],dtype=float)
                center_valid = np.asarray(state['state_tire_center_valid'],dtype=bool)
            except (ValueError,TypeError):
                return False
            if (point.shape != (count, 3) or point_valid.shape != (count,)
                    or segment.shape != (count,) or center.shape != (count, 3)
                    or center_valid.shape != (count,)):
                return False
            if (not np.isfinite(segment).all() or (segment != np.trunc(segment)).any()
                    or not np.isfinite(point[point_valid]).all()
                    or not np.isfinite(center[center_valid]).all()
                    or (tangent_valid & (~point_valid | (segment < 0))).any()):
                return False
            from bike_sim.sim.ride.tire_forces import _BrushState
            updates={}
            for index, side in enumerate(sides):
                value = tire.states.get(side,_BrushState())
                updates[side] = type(value)(
                    float(xi[index]),
                    tangent[index].copy() if tangent_valid[index] else None,
                    point[index].copy() if point_valid[index] else None,
                    None if segment[index] < 0 else int(segment[index]),
                    center[index].copy() if center_valid[index] else None,
                )
        tire_update=(distributed,updates)
    elif any(key.startswith('state_tire_') for key in state):
        return False
    if rider_update is not None:
        states,enabled,grip_xi,pending = rider_update
        rider.states.update(states)
        rider.enabled = enabled
        rider.grip_xi_local = grip_xi
        rider.pending_release_loss_j = pending
    if tire_update is not None:
        distributed,updates = tire_update
        if distributed:
            tire.states.clear()
        tire.states.update(updates)
    for constraint,boundary in drive_boundaries:
        constraint.boundary=boundary
        runtime.sim.model.tendon_range[constraint.tendon_id,1]=boundary
    if coast_hold is not None:
        control._coast_hold_nm = coast_hold
    return True


def load(runtime):
    path = _cache_path(runtime)
    try:
        with np.load(path, allow_pickle=False) as cached:
            qpos = np.array(cached['qpos'], dtype=float, copy=True)
            steps = int(cached['steps'])
            residual = float(cached['residual_qacc'])
            state = {key: np.array(cached[key], copy=True)
                     for key in cached.files if key.startswith('state_')}
    except (OSError, KeyError, ValueError, TypeError, EOFError):
        return None
    if qpos.shape != (runtime.sim.model.nq,) or not np.isfinite(qpos).all():
        return None
    if steps <= 0 or not np.isfinite(residual) or residual < 0.:
        return None
    return path, qpos, steps, residual, state


def load_state_file(runtime, path):
    """Read an explicit equilibrium pose file pinned to this configuration.

    Unlike the derived-name auto-cache, an explicit file is user-managed
    state: every failure is a ``ValueError`` naming the file rather than a
    silent miss. The embedded fingerprint must equal this runtime's
    configuration digest — ``save_state_file`` writes it — and the strict
    state payload checks are identical to ``load``'s.
    """
    path = Path(path)
    try:
        with np.load(path, allow_pickle=False) as cached:
            embedded = str(np.asarray(cached['fingerprint_sha256']))
            qpos = np.array(cached['qpos'], dtype=float, copy=True)
            steps = int(cached['steps'])
            residual = float(cached['residual_qacc'])
            state = {key: np.array(cached[key], copy=True)
                     for key in cached.files if key.startswith('state_')}
    except (OSError, KeyError, ValueError, TypeError, EOFError) as exc:
        raise ValueError(f'equilibrium state file {path}: unreadable or missing fingerprint ({exc})') from exc
    if embedded != fingerprint(runtime):
        raise ValueError(
            f'equilibrium state file {path}: fingerprint mismatch — the pose was '
            'settled for a different configuration')
    if qpos.shape != (runtime.sim.model.nq,) or not np.isfinite(qpos).all():
        raise ValueError(f'equilibrium state file {path}: invalid settled pose')
    if steps <= 0 or not np.isfinite(residual) or residual < 0.:
        raise ValueError(f'equilibrium state file {path}: invalid settle metadata')
    return path, qpos, steps, residual, state


def _write(path, runtime, qpos, steps, residual, *, key=None):
    """Single npz writer behind ``save``/``save_state_file``; the payload is the
    pose, the settle provenance, the material state and the configuration
    fingerprint the file is pinned to (``key`` — the canonical fingerprint
    unless a quarantined episode write supplies its own digest)."""
    temporary = None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.equilibrium-', suffix='.npz', delete=False) as stream:
            temporary = Path(stream.name)
            np.savez_compressed(stream, qpos=np.asarray(qpos, dtype=float),
                                steps=int(steps), residual_qacc=float(residual),
                                fingerprint_sha256=(
                                    fingerprint(runtime) if key is None else key),
                                **_state_arrays(runtime))
        temporary.replace(path)
    except (OSError, ValueError):
        if temporary is not None:
            temporary.unlink(missing_ok=True)
        raise


def save(runtime, qpos, steps, residual):
    """Persist a freshly settled pose; a missed write must not fail the run.

    Only the first solve — the construction-time settle ``generation == 0``,
    the one that provably started from pristine material state — may write
    the canonical key ``load`` reads. A ``reset()`` re-settle starts from
    post-episode state; its pose is a valid equilibrium for this plant but
    not the canonical one, and publishing it under the load key is the
    poisoning vector: the next consumer of that name adopts a pose no
    canonical settle produced. Re-settles are therefore written under
    :func:`episode_fingerprint`, a quarantined namespace nothing loads.

    ``TypeError`` joins the swallowed errors: an ``equilibrium_context``
    carrying a value JSON cannot encode must never fail the run that just
    settled — the write is evidence, not physics.
    """
    try:
        canonical = fingerprint(runtime)
        if getattr(runtime, 'generation', 0) == 0:
            _write(_cache_dir()/f'{canonical}.npz', runtime, qpos, steps, residual,
                   key=canonical)
        else:
            episode = episode_fingerprint(runtime, canonical)
            _write(_cache_dir()/f'{episode}.npz', runtime, qpos, steps, residual,
                   key=episode)
    except (OSError, ValueError, TypeError):
        pass


def save_state_file(path, runtime, qpos, steps, residual):
    """Write an explicit pose file; unlike the auto-cache, a failed write is
    reported to the caller."""
    _write(Path(path), runtime, qpos, steps, residual)
