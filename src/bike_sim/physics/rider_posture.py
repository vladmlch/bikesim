"""Bounded internal rider pose goals, never prescribed body coordinates."""
from dataclasses import dataclass
from bike_sim.physics.checks import scalar, array


@dataclass(frozen=True)
class RiderPosture:
    """Offsets from the articulated neutral posture.

    Positive torso/pelvis pitch is forward lean (+Y in the engine).
    An optional (forward, upward) hip offset is measured in the bicycle frame.
    None retains the original support-following IK. Translational balance is
    a separate nominal support-wrench request; set its gains to zero for a
    passive experiment. A supplied (0, 0) instead
    requests the nominal hip location. Targets can be unreachable: the bounded
    joint actuators and unilateral contacts, not this object, decide motion.
    use_saddle controls requested support only, not the physical saddle contact.
    """
    torso_lean_rad: float = 0.
    pelvis_pitch_rad: float = 0.
    pelvis_offset_m: tuple[float, float] | None = None
    use_saddle: bool = True

    def __post_init__(self):
        for name, limit in (('torso_lean_rad', .8), ('pelvis_pitch_rad', .5)):
            value = scalar(getattr(self, name), name)
            if abs(value) > limit:
                raise ValueError(f'{name} must lie in [-{limit}, {limit}]')
        if self.pelvis_offset_m is not None:
            value = array(self.pelvis_offset_m, 'hip offset', (2,))
            if abs(value[0]) > .25 or not -.15 <= value[1] <= .25:
                raise ValueError('hip offset exceeds the posture command envelope')
            object.__setattr__(self, 'pelvis_offset_m', tuple(map(float, value)))
        if not isinstance(self.use_saddle, bool):
            raise ValueError('use_saddle must be a bool')

    @classmethod
    def standing(cls):
        return cls(torso_lean_rad=.08, pelvis_offset_m=(0., .16), use_saddle=False)
