"""Synthetic crank effort profile; it is not a speed controller."""
from dataclasses import dataclass
from math import copysign, cos, pi
from bike_sim.physics.checks import scalar


def human_crank_torque(mean_nm,phase_rad,ripple=.35):
    mean_nm = scalar(mean_nm,'mean human torque')
    phase_rad = scalar(phase_rad,'crank phase')
    ripple = scalar(ripple,'pedal ripple',minimum=0)
    if ripple >= 1:
        raise ValueError('pedal ripple must be less than one')
    return scalar(mean_nm*(1-ripple*cos(2*phase_rad)),'human torque')


@dataclass(frozen=True)
class PedalingState:
    mode: str
    reason: str
    effort_nm: float
    required_cadence_rpm: float
    target_phase_rad: float | None = None
    target_rate_rad_s: float = 0.


class PedalingPolicy:
    """Rider intention and a bounded stopping goal, independent of torque sensing."""
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.coasting = False
        self.target_phase_rad = None
        self.target_rate_rad_s = 0.
        self.deceleration_rad_s2 = 0.

    def update(self, phase_rad, rate_rad_s, required_cadence_rpm, effort_nm, dt,
               *, enabled=True, braking=False):
        phase_rad = scalar(phase_rad, 'crank phase')
        rate_rad_s = scalar(rate_rad_s, 'crank rate')
        required_cadence_rpm = max(scalar(required_cadence_rpm, 'required cadence'), 0.)
        effort_nm = scalar(effort_nm, 'rider effort', minimum=0.)
        dt = scalar(dt, 'pedaling interval', positive=True)
        if not isinstance(enabled, bool) or not isinstance(braking, bool):
            raise ValueError('pedaling enable and braking must be booleans')
        if not enabled:
            self.reset()
            return PedalingState('disabled', 'disabled', 0., required_cadence_rpm)
        cadence = max(abs(rate_rad_s) * 60. / (2. * pi), required_cadence_rpm)
        threshold = (self.config.resume_below_rpm if self.coasting
                     else self.config.coast_above_rpm)
        excessive = self.config.enabled and cadence >= threshold
        reason = ('braking' if braking else 'no_effort' if effort_nm == 0.
                  else 'cadence' if excessive else '')
        if not reason:
            self.reset()
            return PedalingState('pedaling', '', effort_nm, required_cadence_rpm)
        if not self.coasting:
            self.target_phase_rad = phase_rad
            self.target_rate_rad_s = rate_rad_s
            self.deceleration_rad_s2 = abs(rate_rad_s) / self.config.stop_time_s
        self.coasting = True
        previous_rate = self.target_rate_rad_s
        next_speed = max(0., abs(previous_rate) - self.deceleration_rad_s2 * dt)
        self.target_rate_rad_s = copysign(next_speed, previous_rate)
        self.target_phase_rad += .5 * (previous_rate + self.target_rate_rad_s) * dt
        return PedalingState('coasting', reason, 0., required_cadence_rpm,
                             self.target_phase_rad, self.target_rate_rad_s)
