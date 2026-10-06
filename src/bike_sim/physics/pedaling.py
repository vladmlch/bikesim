"""Synthetic crank effort profile; it is not a speed controller."""
from dataclasses import dataclass
from math import copysign, cos, pi
from bike_sim.physics.checks import boolean, derived, scalar
from bike_sim.physics.domain_validation import pedaling_config


def human_crank_torque(mean_nm,phase_rad,ripple=.35):
    mean_nm = scalar(mean_nm,'mean human torque')
    phase_rad = scalar(phase_rad,'crank phase')
    ripple = scalar(ripple,'pedal ripple',minimum=0)
    if ripple >= 1:
        raise ValueError('pedal ripple must be less than one')
    return derived(mean_nm*(1-ripple*cos(derived(2*phase_rad, 'human_crank_torque.phase'))), 'human_crank_torque.torque')


@dataclass(frozen=True)
class PedalingState:
    mode: str
    reason: str
    effort_nm: float
    required_cadence_rpm: float
    target_phase_rad: float | None = None
    target_rate_rad_s: float = 0.


class PedalingPolicy:
    """Rider effort and bounded coast intent; a stall is an outcome."""

    def __init__(self, config):
        pedaling_config(config)
        self.config = config
        self.reset()

    def reset(self):
        self.coasting = False
        self.target_phase_rad = None
        self.target_rate_rad_s = 0.
        self.deceleration_rad_s2 = 0.
        self._effort = 0.
        self._cadence_ema = None

    def update(self, phase_rad, rate_rad_s, required_cadence_rpm, effort_nm, dt,
               *, enabled=True, braking=False):
        pedaling_config(self.config)
        phase_rad = scalar(phase_rad, 'crank phase')
        rate_rad_s = scalar(rate_rad_s, 'crank rate')
        required_cadence_rpm = scalar(required_cadence_rpm, 'required cadence')
        # Keep the signed wheel rate for forward cadence intent.
        wheel_rad_s = derived(required_cadence_rpm * 2. * pi / 60., "PedalingPolicy.wheel_rate")
        required_cadence_rpm = max(required_cadence_rpm, 0.)
        effort_nm = scalar(effort_nm, 'rider effort', minimum=0.)
        dt = scalar(dt, 'pedaling interval', positive=True)
        if not isinstance(enabled, bool) or not isinstance(braking, bool):
            raise ValueError('pedaling enable and braking must be booleans')
        if not enabled:
            self.reset()
            return PedalingState('disabled', 'disabled', 0., required_cadence_rpm)
        actual_cadence = derived(abs(rate_rad_s) * 60. / (2. * pi), "PedalingPolicy.cadence")
        cadence = max(actual_cadence, required_cadence_rpm)
        tau = self.config.coast_cadence_tau_s
        if tau > 0. and self._cadence_ema is not None:
            change = actual_cadence-self._cadence_ema
            derived(change, 'PedalingPolicy.cadence_delta')
            ema = self._cadence_ema + min(1., dt/tau)*change
            derived(ema, 'PedalingPolicy.cadence_ema')
            self._cadence_ema = ema
        else:
            self._cadence_ema = actual_cadence
        threshold = (self.config.resume_below_rpm if self.coasting
                     else self.config.coast_above_rpm)
        excessive = self.config.enabled and self._cadence_ema >= threshold
        reason = ('braking' if braking else 'no_effort' if effort_nm == 0.
                  else 'cadence' if excessive else '')
        if not reason:
            previous, ema = self._effort, self._cadence_ema
            self.reset()
            self._effort, self._cadence_ema = previous, ema
            # Muscle force-velocity is inverted relative to the naive constant
            # effort: as cadence collapses a real rider converts to standing on
            # the pedal, and the available torque rises toward the isometric
            # ceiling instead of fading. The ramp is linear in cadence and
            # never reduces the commanded effort. The slew bound models force
            # development from the actually issued previous effort, so a mash
            # builds over a fraction of a second rather than appearing as an
            # impulse that rips the feet off the pedals.
            target = effort_nm
            if self.config.mash_torque_nm > target and cadence < self.config.mash_cadence_rpm:
                target = effort_nm + (self.config.mash_torque_nm-effort_nm)*(1.-cadence/self.config.mash_cadence_rpm)
            slew = self.config.effort_slew_nm_s
            if slew <= 0.:
                effort = target
            else:
                base = self._effort
                delta = slew*dt
                derived(delta, 'PedalingPolicy.slew_delta')
                lower, upper = base-delta, base+delta
                derived(lower, 'PedalingPolicy.slew_lower')
                derived(upper, 'PedalingPolicy.slew_upper')
                effort = max(lower, min(upper, target))
            self._effort = effort
            return PedalingState('pedaling', '', effort, required_cadence_rpm,
                                 None, max(wheel_rad_s, 0.))
        if not self.coasting:
            deceleration = abs(rate_rad_s) / self.config.stop_time_s
            derived(deceleration, 'PedalingPolicy.deceleration')
            self.target_phase_rad = phase_rad
            self.target_rate_rad_s = rate_rad_s
            self.deceleration_rad_s2 = deceleration
        self.coasting = True
        previous_rate = self.target_rate_rad_s
        change = self.deceleration_rad_s2 * dt
        derived(change, 'PedalingPolicy.rate_delta')
        speed = abs(previous_rate) - change
        derived(speed, 'PedalingPolicy.next_speed')
        next_speed = max(0., speed)
        self.target_rate_rad_s = copysign(next_speed, previous_rate)
        rate_sum = previous_rate + self.target_rate_rad_s
        derived(rate_sum, 'PedalingPolicy.rate_sum')
        phase = self.target_phase_rad + .5 * rate_sum * dt
        derived(phase, 'PedalingPolicy.target_phase_rad')
        self.target_phase_rad = phase
        self._effort = 0.
        return PedalingState('coasting', reason, 0., required_cadence_rpm,
                             self.target_phase_rad, self.target_rate_rad_s)
