"""Cadence-based rider gear selection without prescribing crank or wheel motion."""
from bike_sim.physics.checks import scalar


class CadenceShifter:
    def __init__(self, gearing, config):
        self.gearing = gearing
        self.config = config
        self.reset()

    def reset(self):
        self.rear_teeth = self.gearing.rear_teeth
        self.cooldown_s = 0.
        self.cut_remaining_s = 0.
        self.shift_count = 0
        self.direction = 'none'
        self.from_teeth = self.rear_teeth
        self.cadence_ema = None
        self.required_ema = None

    @property
    def gear_ratio(self):
        return self.gearing.front_teeth / self.rear_teeth

    @property
    def torque_factor(self):
        return self.config.torque_factor if self.cut_remaining_s > 0. else 1.

    def update(self, cadence_rpm, required_cadence_rpm, dt, *, pedaling=True,
               braking=False, rear_in_contact=True, rear_slip_mps=None):
        cadence_rpm = scalar(cadence_rpm, 'shift cadence')
        required_cadence_rpm = scalar(required_cadence_rpm, 'wheel-required shift cadence')
        dt = scalar(dt, 'shift interval', positive=True)
        if rear_slip_mps is not None:
            rear_slip_mps = scalar(rear_slip_mps, 'rear tire slip')
        # Both inputs are smoothed: a rider reacts to sustained cadence, not to a
        # transient. The crank rate carries pedal-stroke ripple; the wheel-implied
        # required cadence carries slip and impact spikes (an obstacle crossing can
        # spin the wheel 84->110+ rpm in ~40 ms -- faster than anyone could notice,
        # let alone decide to shift). Neither may read as a sustained spin-up.
        tau = self.config.cadence_smoothing_tau_s
        alpha = 1. if tau <= 0. else min(1., dt/tau)
        if self.cadence_ema is None:
            self.cadence_ema = cadence_rpm
            self.required_ema = required_cadence_rpm
        else:
            self.cadence_ema += alpha*(cadence_rpm-self.cadence_ema)
            self.required_ema += alpha*(required_cadence_rpm-self.required_ema)
        self.cooldown_s = max(0., self.cooldown_s - dt)
        self.cut_remaining_s = max(0., self.cut_remaining_s - dt)
        if (not self.config.enabled or not pedaling or braking or not rear_in_contact
                or self.cooldown_s > 1e-12 or min(cadence_rpm, required_cadence_rpm) < 0.):
            return False
        cadence = max(self.cadence_ema, self.required_ema)
        if cadence > self.config.target_cadence_max_rpm:
            candidates = [teeth for teeth in self.config.cassette if teeth < self.rear_teeth]
            selected = max(candidates) if candidates else self.rear_teeth
            direction = 'up'
        elif cadence < self.config.target_cadence_min_rpm:
            candidates = [teeth for teeth in self.config.cassette if teeth > self.rear_teeth]
            selected = min(candidates) if candidates else self.rear_teeth
            direction = 'down'
        else:
            return False
        if selected == self.rear_teeth:
            return False
        # A slipping wheel spins up without the bike accelerating, so the implied
        # cadence over-reads. Upshifting on wheelspin is never a rider's intent.
        slip_for_gate = rear_slip_mps
        if slip_for_gate is not None and self.config.upshift_slip_mode == 'magnitude':
            slip_for_gate = abs(slip_for_gate)
        if direction == 'up' and slip_for_gate is not None \
                and slip_for_gate > self.config.upshift_slip_limit_mps:
            return False
        # A shift that lands the wheel-implied cadence outside the target band is
        # hunting: an upshift while grinding throws the next required cadence
        # below the minimum and forces an immediate shift back.
        landing = self.required_ema*selected/self.rear_teeth
        if (direction == 'up' and landing < self.config.target_cadence_min_rpm) \
                or (direction == 'down' and landing > self.config.target_cadence_max_rpm):
            return False
        self.from_teeth, self.rear_teeth = self.rear_teeth, selected
        # Re-anchor the filter: required cadence is gear-relative, so the smoothed
        # value must be rescaled or the next steps compare stale magnitudes.
        self.required_ema = landing
        self.direction = direction
        self.shift_count += 1
        self.cooldown_s = self.config.shift_cooldown_s
        self.cut_remaining_s = self.config.shift_cut_duration_s
        return True
