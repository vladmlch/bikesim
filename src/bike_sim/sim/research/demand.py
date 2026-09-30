"""Scripted 'the bike wants to go' torque request.

The demand is an advisory command echo for the policy (env.demand_nm), not a
sensor and not an enforced limit: the policy decides how much of it to pass on,
and metrics compare delivered torque with what was actually requested.
Interpolation mirrors RiderProgram.at (quintic smoothstep, zero slope at the
knots) so scripted inputs do not inject step disturbances of their own.
"""
from bisect import bisect_right
from dataclasses import dataclass
from pathlib import Path
import tomllib
from bike_sim.physics.checks import scalar


@dataclass(frozen=True)
class DemandProgram:
    keyframes: tuple[tuple[float, float], ...]

    def __post_init__(self):
        try:
            frames = tuple(tuple(k) for k in self.keyframes)
        except TypeError as exc:
            raise ValueError('demand keyframes must be a sequence of (time_s, torque_nm)') from exc
        if not frames or any(len(k) != 2 for k in frames):
            raise ValueError('demand needs at least one (time_s, torque_nm) keyframe')
        frames = tuple((scalar(t, 'demand keyframe time', minimum=0.),
                        scalar(nm, 'demand torque', minimum=0.)) for t, nm in frames)
        if frames[0][0] != 0. or any(b[0] <= a[0] for a, b in zip(frames, frames[1:])):
            raise ValueError('demand keyframes must start at zero and strictly increase')
        object.__setattr__(self, 'keyframes', frames)
        object.__setattr__(self, '_times', tuple(t for t, _ in frames))

    @classmethod
    def constant(cls, torque_nm):
        return cls(((0., torque_nm),))

    def at(self, time_s):
        t = max(0., scalar(time_s, 'demand time'))
        i = bisect_right(self._times, t)-1
        if i >= len(self.keyframes)-1:
            return self.keyframes[-1][1]
        (t0, a), (t1, b) = self.keyframes[i:i+2]
        u = (t-t0)/(t1-t0)
        return a+(b-a)*u*u*u*(10.+u*(-15.+6.*u))

    def to_dict(self):
        return {'keyframes': [{'time_s': t, 'torque_nm': nm} for t, nm in self.keyframes]}

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict) or set(data)-{'keyframes'}:
            raise ValueError('demand program must be a table with only a keyframes array')
        frames = data.get('keyframes')
        if not isinstance(frames, (tuple, list)):
            raise ValueError('demand keyframes must be an array of tables')
        try:
            return cls(tuple((entry['time_s'], entry['torque_nm']) for entry in frames))
        except (KeyError, TypeError) as exc:
            raise ValueError(f'invalid demand keyframe: {exc}') from exc

    @classmethod
    def load(cls, path):
        with Path(path).open('rb') as stream:
            return cls.from_dict(tomllib.load(stream))
