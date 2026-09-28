"""Schema-2 force intervals and accounting, independent of recording decimation."""
from dataclasses import dataclass
from types import MappingProxyType
from collections.abc import Mapping
import numpy as np
from bike_sim.physics.checks import array, scalar


def freeze(value):
    if isinstance(value, np.ndarray):
        return array(value, 'sample array', readonly=True)
    if isinstance(value, Mapping):
        return MappingProxyType({str(k): freeze(v) for k, v in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(freeze(v) for v in value)
    if isinstance(value, (bool, str)) or value is None:
        return value
    if isinstance(value, (int, float, np.number)):
        return scalar(value, 'sample value')
    raise ValueError('unsupported sample value')


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Mapping):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


@dataclass(frozen=True)
class PhysicalSample:
    interval_id: int
    time_s: float
    end_time_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    forces: Mapping
    channels: Mapping

    def __post_init__(self):
        if isinstance(self.interval_id, bool) or not isinstance(self.interval_id, int) or self.interval_id < 0:
            raise ValueError('invalid interval ID')
        t = scalar(self.time_s, 'state time', minimum=0)
        end = scalar(self.end_time_s, 'interval end', minimum=0)
        if end <= t:
            raise ValueError('force interval must have positive duration')
        q, v = array(self.qpos, 'sample qpos'), array(self.qvel, 'sample qvel')
        if q.ndim != 1 or v.ndim != 1:
            raise ValueError('sample coordinates must be vectors')
        forces = {name: array(f, name, v.shape, readonly=True) for name, f in self.forces.items()}
        object.__setattr__(self, 'qpos', freeze(q))
        object.__setattr__(self, 'qvel', freeze(v))
        object.__setattr__(self, 'forces', MappingProxyType(forces))
        object.__setattr__(self, 'channels', freeze(self.channels))

    @property
    def dt_s(self):
        return self.end_time_s-self.time_s

    @property
    def powers_w(self):
        return {name: float(force @ self.qvel) for name, force in self.forces.items()}

    def as_dict(self):
        return {'schema_version': 2, 'interval_id': self.interval_id,
                'time_s': self.time_s, 'interval_end_s': self.end_time_s, 'dt_s': self.dt_s,
                'qpos': self.qpos.tolist(), 'qvel': self.qvel.tolist(),
                'powers_w': self.powers_w, **plain(self.channels)}


class WorkHistory:
    """One left-rectangle integration per physical step, never per CSV row."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.work_j = {}
        self.last_id = None
        self.last_end = None
        self.airtime_s = {side: {str(t): 0. for t in (0., 1., 5.)} for side in ('front', 'rear')}
        self.duration_s = 0.

    def add(self, sample):
        if self.last_id is not None:
            if sample.interval_id != self.last_id+1 or abs(sample.time_s-self.last_end) > 1e-10:
                raise ValueError('missing, overlapping or repeated force interval')
        # Missing physical contact evidence is not an airborne interval. Validate
        # both wheels before updating any work or elapsed-time accumulator.
        tires = sample.channels.get('tires')
        if not isinstance(tires, Mapping) or not {'front', 'rear'} <= set(tires):
            raise ValueError('raw contact snapshots for both wheels are required')
        loads = {}
        for side in self.airtime_s:
            if 'patches' not in tires[side]:
                raise ValueError('raw contact patches are required for airtime')
            load = 0.
            for patch in tires[side]['patches']:
                normal = scalar(patch['normal_load_n'], 'raw normal load', minimum=0)
                source = patch['source_geom']
                if source not in ('terrain', 'catch_plane'):
                    raise ValueError('unrecognized physical contact source')
                if source == 'terrain':
                    load += normal
            loads[side] = scalar(load, 'total working-road load', minimum=0)
        increments = {name: scalar(power*sample.dt_s, 'interval work')
                      for name, power in sample.powers_w.items()}
        updated = {name: scalar(self.work_j.get(name, 0.)+work, 'accumulated work')
                   for name, work in increments.items()}
        self.work_j.update(updated)
        for side, raw_load in loads.items():
            for threshold in self.airtime_s[side]:
                if raw_load <= float(threshold):
                    self.airtime_s[side][threshold] += sample.dt_s
        self.duration_s += sample.dt_s
        self.last_id, self.last_end = sample.interval_id, sample.end_time_s
