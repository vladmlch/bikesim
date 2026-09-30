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

    @property
    def gear_ratio(self):
        return self.gearing.front_teeth / self.rear_teeth

    @property
    def torque_factor(self):
        return self.config.torque_factor if self.cut_remaining_s > 0. else 1.

    def update(self, cadence_rpm, required_cadence_rpm, dt, *, pedaling=True,
               braking=False, rear_in_contact=True):
        cadence_rpm = scalar(cadence_rpm, 'shift cadence')
        required_cadence_rpm = scalar(required_cadence_rpm, 'wheel-required shift cadence')
        dt = scalar(dt, 'shift interval', positive=True)
        self.cooldown_s = max(0., self.cooldown_s - dt)
        self.cut_remaining_s = max(0., self.cut_remaining_s - dt)
        if (not self.config.enabled or not pedaling or braking or not rear_in_contact
                or self.cooldown_s > 1e-12 or min(cadence_rpm, required_cadence_rpm) < 0.):
            return False
        cadence = max(cadence_rpm, required_cadence_rpm)
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
        self.from_teeth, self.rear_teeth = self.rear_teeth, selected
        self.direction = direction
        self.shift_count += 1
        self.cooldown_s = self.config.shift_cooldown_s
        self.cut_remaining_s = self.config.shift_cut_duration_s
        return True
