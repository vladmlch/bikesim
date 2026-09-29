"""Persistent cache for validated physical initial poses."""
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


CACHE_VERSION = 2


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


def _cache_path(runtime):
    sim = runtime.sim
    source_root = Path(__file__).resolve().parents[2]
    payload = {
        'version': CACHE_VERSION,
        'source': source_fingerprint(source_root),
        'mujoco': mujoco.mj_versionString(),
        'physics': _serializable(sim.physics_config),
        'specs': _serializable(sim.specs),
        'mass_specs': _serializable(sim.mass_specs),
        'rider': _serializable(sim.rider),
        'track': plain(track_to_dict(sim.track)),
        'start_x_m': sim.start_x_m,
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    digest = hashlib.sha256(encoded)
    for name in ('qpos0', 'body_mass', 'body_ipos', 'body_inertia', 'geom_pos', 'geom_size',
                 'jnt_range', 'hfield_data'):
        _update_array(digest, name, getattr(sim.model, name))
    _update_array(digest, 'timestep', np.array([sim.model.opt.timestep], dtype='<f8'))
    root = os.environ.get('BIKE_SIM_EQUILIBRIUM_CACHE_DIR')
    directory = Path(root) if root else Path(tempfile.gettempdir()) / 'bike_sim_equilibrium'
    return directory / f'{digest.hexdigest()}.npz'


def _state_arrays(runtime):
    state = {}
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
        state['state_rider_grip_xi'] = np.asarray(rider.grip_xi_local, dtype=float)
        state['state_rider_enabled'] = np.asarray([
            rider.enabled[name] for name in rider.CONTACTS
        ], dtype=bool)
        state['state_rider_pending_release_loss'] = np.asarray(
            [rider.pending_release_loss_j], dtype=float
        )
    tire = runtime.tire
    if tire is not None:
        sides = tuple(tire.states)
        state['state_tire_sides'] = np.asarray(sides)
        state['state_tire_xi'] = np.asarray([tire.states[side].xi for side in sides], dtype=float)
        state['state_tire_tangent'] = np.asarray([
            value.tangent if value.tangent is not None else np.zeros(3)
            for value in (tire.states[side] for side in sides)
        ], dtype=float)
        state['state_tire_tangent_valid'] = np.asarray([
            value.tangent is not None for value in (tire.states[side] for side in sides)
        ], dtype=bool)
        state['state_tire_point'] = np.asarray([
            value.point if value.point is not None else np.zeros(3)
            for value in (tire.states[side] for side in sides)
        ], dtype=float)
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
        ], dtype=float)
        state['state_tire_center_valid'] = np.asarray([
            value.center is not None for value in (tire.states[side] for side in sides)
        ], dtype=bool)
    return state


def restore_state(runtime, state):
    rider = runtime.rider_contacts
    if rider is not None:
        required = {
            'state_rider_keys', 'state_rider_xi', 'state_rider_tangent',
            'state_rider_tangent_valid', 'state_rider_grip_xi',
            'state_rider_enabled', 'state_rider_pending_release_loss',
        }
        if not required.issubset(state):
            return False
        keys = tuple(str(value) for value in state['state_rider_keys'])
        if keys != tuple(rider.states):
            return False
        xi = state['state_rider_xi']
        tangent = state['state_rider_tangent']
        valid = state['state_rider_tangent_valid']
        if xi.shape != (len(keys),) or tangent.shape != (len(keys), 3) or valid.shape != (len(keys),):
            return False
        for index, key in enumerate(keys):
            value = tangent[index].copy() if valid[index] else None
            rider.states[key] = type(rider.states[key])(float(xi[index]), value)
        enabled = state['state_rider_enabled']
        if enabled.shape != (len(rider.CONTACTS),):
            return False
        rider.enabled = dict(zip(rider.CONTACTS, map(bool, enabled)))
        rider.grip_xi_local = np.array(state['state_rider_grip_xi'], dtype=float, copy=True)
        if rider.grip_xi_local.shape != (3,):
            return False
        pending = state['state_rider_pending_release_loss']
        if pending.shape != (1,):
            return False
        rider.pending_release_loss_j = float(pending[0])
    elif any(key.startswith('state_rider_') for key in state):
        return False
    tire = runtime.tire
    if tire is not None:
        required = {
            'state_tire_sides', 'state_tire_xi', 'state_tire_tangent',
            'state_tire_tangent_valid', 'state_tire_point',
            'state_tire_point_valid', 'state_tire_segment', 'state_tire_center',
            'state_tire_center_valid',
        }
        if not required.issubset(state):
            return False
        sides = tuple(str(value) for value in state['state_tire_sides'])
        if sides != tuple(tire.states):
            return False
        xi = state['state_tire_xi']
        tangent = state['state_tire_tangent']
        tangent_valid = state['state_tire_tangent_valid']
        point = state['state_tire_point']
        point_valid = state['state_tire_point_valid']
        segment = state['state_tire_segment']
        center = state['state_tire_center']
        center_valid = state['state_tire_center_valid']
        count = len(sides)
        if (xi.shape != (count,) or tangent.shape != (count, 3)
                or tangent_valid.shape != (count,) or point.shape != (count, 3)
                or point_valid.shape != (count,) or segment.shape != (count,)
                or center.shape != (count, 3) or center_valid.shape != (count,)):
            return False
        for index, side in enumerate(sides):
            value = tire.states[side]
            tire.states[side] = type(value)(
                float(xi[index]),
                tangent[index].copy() if tangent_valid[index] else None,
                point[index].copy() if point_valid[index] else None,
                None if segment[index] < 0 else int(segment[index]),
                center[index].copy() if center_valid[index] else None,
            )
    elif any(key.startswith('state_tire_') for key in state):
        return False
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


def save(runtime, qpos, steps, residual):
    path = _cache_path(runtime)
    temporary = None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.equilibrium-', suffix='.npz', delete=False) as stream:
            temporary = Path(stream.name)
            np.savez_compressed(stream, qpos=np.asarray(qpos, dtype=float),
                                steps=int(steps), residual_qacc=float(residual),
                                **_state_arrays(runtime))
        temporary.replace(path)
    except (OSError, ValueError):
        if temporary is not None:
            temporary.unlink(missing_ok=True)
