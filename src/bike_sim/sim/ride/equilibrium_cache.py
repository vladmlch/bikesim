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


CACHE_VERSION = 4


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
    tire = runtime.tire
    if tire is not None:
        sides = tuple(tire.states)
        state['state_tire_sides'] = np.asarray([json.dumps(key) for key in sides])
        state['state_tire_xi'] = np.asarray([tire.states[side].xi for side in sides], dtype=float)
        state['state_tire_tangent'] = np.asarray([
            value.tangent if value.tangent is not None else np.zeros(3)
            for value in (tire.states[side] for side in sides)
        ], dtype=float).reshape(-1,3)
        state['state_tire_tangent_valid'] = np.asarray([
            value.tangent is not None for value in (tire.states[side] for side in sides)
        ], dtype=bool)
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
    tire = runtime.tire
    tire_update=None
    if tire is not None:
        required = {
            'state_tire_sides', 'state_tire_xi', 'state_tire_tangent',
            'state_tire_tangent_valid', 'state_tire_point',
            'state_tire_point_valid', 'state_tire_segment', 'state_tire_center',
            'state_tire_center_valid',
        }
        if not required.issubset(state):
            return False
        try:
            decoded = [json.loads(str(value)) for value in state['state_tire_sides']]
            sides = tuple(tuple(key) if isinstance(key,list) else key for key in decoded)
        except (ValueError,TypeError):
            return False
        distributed = tire.config.backend == 'distributed_2d_reference'
        if not distributed and sides != tuple(tire.states):
            return False
        if distributed and (any(not isinstance(key,tuple) or len(key)!=2 or key[0] not in ('front','rear') or type(key[1]) is not int or not 0<=key[1]<tire.config.distributed.station_count for key in sides) or len(set(sides)) != len(sides)):
            return False
        try:
            xi = np.asarray(state['state_tire_xi'],dtype=float)
            tangent = np.asarray(state['state_tire_tangent'],dtype=float)
            tangent_valid = np.asarray(state['state_tire_tangent_valid'],dtype=bool)
            point = np.asarray(state['state_tire_point'],dtype=float)
            point_valid = np.asarray(state['state_tire_point_valid'],dtype=bool)
            segment = np.asarray(state['state_tire_segment'],dtype=float)
            center = np.asarray(state['state_tire_center'],dtype=float)
            center_valid = np.asarray(state['state_tire_center_valid'],dtype=bool)
        except (ValueError,TypeError):
            return False
        count = len(sides)
        if (xi.shape != (count,) or tangent.shape != (count, 3)
                or tangent_valid.shape != (count,) or point.shape != (count, 3)
                or point_valid.shape != (count,) or segment.shape != (count,)
                or center.shape != (count, 3) or center_valid.shape != (count,)):
            return False
        if (not np.isfinite(xi).all() or not np.isfinite(segment).all()
                or (segment != np.trunc(segment)).any()
                or not np.isfinite(tangent[tangent_valid]).all()
                or not np.isfinite(point[point_valid]).all()
                or not np.isfinite(center[center_valid]).all()
                or (tangent_valid & (~point_valid | (segment < 0))).any()):
            return False
        from bike_sim.sim.ride.tire_forces import _BrushState
        updates={}
        for index, side in enumerate(sides):
            value = _BrushState() if distributed else tire.states.get(side,_BrushState())
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
