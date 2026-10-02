"""Independent, deterministic rider intent; never prescribed body motion.

Numeric targets use a quintic smoothstep (zero velocity and acceleration at
knots). Saddle-support requests switch at their knot; actual saddle contact is
never disabled. The existing bounded internal joint controller realizes intent.
"""
from bisect import bisect_right
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
import tomllib
from bike_sim.physics.checks import scalar
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl


@dataclass(frozen=True)
class RiderKeyframe:
    time_s: float
    posture: RiderPosture = field(default_factory=RiderPosture)
    human_torque_nm: float | None = None
    crank_reposition: bool | None = None

    def __post_init__(self):
        scalar(self.time_s, 'rider keyframe time', minimum=0.)
        if not isinstance(self.posture, RiderPosture):
            raise ValueError('rider keyframe requires a RiderPosture')
        if self.human_torque_nm is not None:
            scalar(self.human_torque_nm, 'rider effort', minimum=0.)
        if self.crank_reposition is not None and not isinstance(self.crank_reposition, bool):
            raise ValueError('crank_reposition must be a bool or None')


@dataclass(frozen=True)
class RiderProgram:
    keyframes: tuple[RiderKeyframe, ...]
    reaction_delay_s: float = 0.

    def __post_init__(self):
        try:
            frames = tuple(self.keyframes)
        except TypeError as exc:
            raise ValueError('rider keyframes must be a sequence') from exc
        if not frames or any(not isinstance(f, RiderKeyframe) for f in frames):
            raise ValueError('rider program requires RiderKeyframes')
        if frames[0].time_s != 0. or any(b.time_s <= a.time_s for a, b in zip(frames, frames[1:])):
            raise ValueError('rider keyframes must start at zero and strictly increase')
        if len({f.human_torque_nm is None for f in frames}) != 1:
            raise ValueError('specify human effort in all keyframes or in none')
        if len({f.crank_reposition is None for f in frames}) != 1:
            raise ValueError('specify crank_reposition in all keyframes or in none')
        if len({f.posture.pelvis_offset_m is None for f in frames}) != 1:
            raise ValueError('hip offsets must be explicit in all keyframes or in none')
        scalar(self.reaction_delay_s, 'rider reaction delay', minimum=0.)
        object.__setattr__(self, 'keyframes', frames)
        object.__setattr__(self, '_times', tuple(f.time_s for f in frames))

    @property
    def owns_human_effort(self):
        return self.keyframes[0].human_torque_nm is not None

    @property
    def owns_crank_reposition(self):
        return self.keyframes[0].crank_reposition is not None

    def at(self, time_s: float) -> RiderKeyframe:
        t = max(0., scalar(time_s, 'rider program time')-self.reaction_delay_s)
        i = bisect_right(self._times, t)-1
        if i >= len(self.keyframes)-1:
            return self.keyframes[-1]
        a, b = self.keyframes[i:i+2]
        u = (t-a.time_s)/(b.time_s-a.time_s)
        blend = u*u*u*(10.+u*(-15.+6.*u))
        def mix(x, y):
            return x+(y-x)*blend
        pa, pb = a.posture, b.posture
        offset = (None if pa.pelvis_offset_m is None else
                  tuple(mix(x, y) for x, y in zip(pa.pelvis_offset_m, pb.pelvis_offset_m)))
        pose = RiderPosture(mix(pa.torso_lean_rad, pb.torso_lean_rad),
            mix(pa.pelvis_pitch_rad, pb.pelvis_pitch_rad), offset, pa.use_saddle)
        effort = mix(a.human_torque_nm, b.human_torque_nm) if self.owns_human_effort else None
        # A reposition request is an event flag, not a blendable quantity: it
        # holds for the keyframe segment and each False->True transition fires
        # one maneuver in the plant.
        reposition = a.crank_reposition if self.owns_crank_reposition else None
        return RiderKeyframe(t, pose, effort, reposition)

    def validate_control(self, control: RideControl):
        if not isinstance(control, RideControl):
            raise ValueError('rider program expects RideControl')
        if control.posture is not None:
            raise ValueError('rider program owns posture; do not also command it from the motor policy')
        if self.owns_human_effort and control.human_torque_nm is not None:
            raise ValueError('rider program owns human effort; leave policy human_torque_nm=None')
        if self.owns_crank_reposition and control.crank_reposition:
            raise ValueError('rider program owns crank reposition; leave policy crank_reposition=False')

    def apply(self, control: RideControl, time_s: float) -> RideControl:
        self.validate_control(control)
        intent = self.at(time_s)
        return replace(control, posture=intent.posture,
            human_torque_nm=intent.human_torque_nm if self.owns_human_effort else control.human_torque_nm,
            crank_reposition=intent.crank_reposition if self.owns_crank_reposition
                             else control.crank_reposition)

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError('rider program must be a table')
        if set(data)-{'keyframes', 'reaction_delay_s'}:
            raise ValueError('unknown rider program field')
        frames = data.get('keyframes')
        if not isinstance(frames, (tuple, list)):
            raise ValueError('rider keyframes must be a sequence')
        result = []
        for entry in frames:
            if not isinstance(entry, dict) or set(entry)-{'time_s', 'posture', 'human_torque_nm', 'crank_reposition'}:
                raise ValueError('unknown or malformed rider keyframe')
            try:
                result.append(RiderKeyframe(time_s=entry['time_s'],
                    posture=RiderPosture(**entry.get('posture', {})),
                    human_torque_nm=entry.get('human_torque_nm'),
                    crank_reposition=entry.get('crank_reposition')))
            except (KeyError, TypeError) as exc:
                raise ValueError(f'invalid rider keyframe: {exc}') from exc
        return cls(tuple(result), reaction_delay_s=data.get('reaction_delay_s', 0.))

    @classmethod
    def load(cls, path):
        with Path(path).open('rb') as stream:
            return cls.from_dict(tomllib.load(stream))
