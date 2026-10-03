"""Per-cycle rider intent for the seated pedaling program.

A `PedalIntent` is a bounded wish, never a trajectory command: cadence is a
request the R4 allocator may relax, the ankle offset is a foot-pitch
modulation inside the existing joint limits, and the scrape fraction only
chooses how much of the friction cone the return foot asks for. Nothing here
assigns qpos/qvel or brakes the crank from outside the legs.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PedalIntent:
    cadence_rad_s: float
    ankle_offset_rad: float
    scrape_fraction: float


def pedal_intent(phase_rad, cadence_rad_s, *, ankle_amplitude_rad,
                 scrape_fraction) -> PedalIntent:
    values = (phase_rad, cadence_rad_s, ankle_amplitude_rad, scrape_fraction)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('nonfinite pedaling intent')
    if cadence_rad_s < 0 or ankle_amplitude_rad < 0 or not 0 <= scrape_fraction <= 1:
        raise ValueError('invalid pedaling intent')
    return PedalIntent(cadence_rad_s,
        ankle_amplitude_rad*math.sin(phase_rad), scrape_fraction)


def body_pulse(time_s: float, start_s: float, duration_s: float,
               amplitude_rad: float) -> float:
    """One smooth finite torso-thrust wish, zero outside (start, start+dur).

    The 64 p^3 (1-p)^3 window is C1 at both endpoints, so scheduling a pulse
    can never produce an impulsive posture request; its peak equals the
    amplitude. This is a wish on top of the ordinary lean intent -- the R4
    allocator, not this function, decides how much actually becomes joint
    torque and contact reaction.
    """
    if not all(math.isfinite(v) for v in (time_s, start_s, duration_s,
                                          amplitude_rad)):
        raise ValueError('nonfinite posture program')
    if duration_s <= 0:
        raise ValueError('pulse duration must be positive')
    phase = (time_s-start_s)/duration_s
    if not 0 < phase < 1:
        return 0.
    return amplitude_rad * 64. * phase**3 * (1.-phase)**3


class SeatedPostureProgram:
    """Strategy-level torso-lean intent, kept separate from state estimation.

    Consumes only the allowed rider information: road grade under the wheels
    and in the preview window, the strategy's own parameters, the
    simulation clock, and scheduled body pulses. It never inspects wheelie
    outcomes or solved contact reactions, and it produces only a
    rate-limited torso-lean wish inside the declared lean envelope.
    """

    def __init__(self, config):
        from bike_sim.physics.seated_climb import SeatedClimbConfig
        if not isinstance(config, SeatedClimbConfig):
            raise ValueError('expected a seated climb configuration')
        self.config = config
        self.lean_rad = 0.
        self._pulses = []

    def reset(self):
        self.lean_rad = 0.
        self._pulses.clear()

    def schedule_pulse(self, start_s, duration_s, amplitude_rad):
        for name, value in (('pulse start', start_s),
                            ('pulse duration', duration_s),
                            ('pulse amplitude', amplitude_rad)):
            if not math.isfinite(value):
                raise ValueError(f'nonfinite {name}')
        if duration_s <= 0:
            raise ValueError('pulse duration must be positive')
        self._pulses.append((float(start_s), float(duration_s),
                             float(amplitude_rad)))

    def update(self, time_s, road_grade, dt_s):
        config = self.config
        if not (math.isfinite(time_s) and math.isfinite(road_grade)):
            raise ValueError('nonfinite posture program input')
        if not (math.isfinite(dt_s) and dt_s > 0.):
            raise ValueError('posture program interval must be positive')
        dt = dt_s
        extra = sum(body_pulse(time_s, start, duration, amplitude)
                    for start, duration, amplitude in self._pulses)
        target = max(-config.max_backward_lean_rad,
            min(config.max_forward_lean_rad,
                config.lean_gain*math.atan(road_grade) + extra))
        self.lean_rad += max(-config.lean_rate_rad_s*dt,
            min(config.lean_rate_rad_s*dt, target-self.lean_rad))
        return self.lean_rad
