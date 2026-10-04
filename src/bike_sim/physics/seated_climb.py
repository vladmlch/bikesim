"""Synthetic bounded seated intentions from delayed proprioceptive signals."""
from collections import deque
from dataclasses import dataclass
from math import atan2, exp, hypot, pi, sin, cos

from bike_sim.physics.checks import array, scalar
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.physics.rider_program import SeatedPostureProgram


@dataclass(frozen=True)
class SeatedClimbConfig:
    enabled: bool = False
    period_s: float = .01
    reaction_delay_s: float = .15
    target_crank_power_w: float = 225.
    max_crank_torque_nm: float = 60.
    torque_slew_nm_s: float = 300.
    lean_gain: float = 1.
    max_forward_lean_rad: float = .35
    max_backward_lean_rad: float = .10
    lean_rate_rad_s: float = .5
    orientation_tau_s: float = .5
    surge_power_w: float = 400.
    surge_grade: float = .20
    surge_budget_s: float = 15.
    surge_recovery_rate: float = 1/3
    front_load_share_target: float = .30
    lean_trim_gain_rad_s: float = .3
    lean_trim_limit_rad: float = .15

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('seated climb enable must be boolean')
        for name in ('period_s', 'target_crank_power_w', 'max_crank_torque_nm',
                     'torque_slew_nm_s', 'lean_rate_rad_s', 'orientation_tau_s',
                     'surge_power_w', 'surge_budget_s'):
            scalar(getattr(self, name), name, positive=True)
        for name in ('reaction_delay_s', 'lean_gain', 'max_forward_lean_rad', 'max_backward_lean_rad',
                     'surge_grade', 'surge_recovery_rate', 'front_load_share_target',
                     'lean_trim_gain_rad_s', 'lean_trim_limit_rad'):
            scalar(getattr(self, name), name, minimum=0.)
        if self.front_load_share_target > 1.:
            raise ValueError('front load share target must not exceed one')
        if max(self.max_forward_lean_rad, self.max_backward_lean_rad) > .8:
            raise ValueError('seated lean exceeds the posture envelope')


@dataclass(frozen=True)
class SeatedClimbSignals:
    pitch_rate_up_rad_s: float = 0.
    specific_force_body_mps2: tuple[float, float, float] = (0., 0., 0.)
    crank_rate_rad_s: float = 0.
    human_crank_torque_nm: float = 0.

    def __post_init__(self):
        for name in ('pitch_rate_up_rad_s', 'crank_rate_rad_s', 'human_crank_torque_nm'):
            scalar(getattr(self, name), name)
        acceleration = array(self.specific_force_body_mps2, 'rider specific force', (3,))
        object.__setattr__(self, 'specific_force_body_mps2', tuple(map(float, acceleration)))


@dataclass(frozen=True)
class SeatedClimbIntent:
    posture: RiderPosture
    effort_ceiling_nm: float


def crank_effort_ceiling(power_w, torque_limit_nm, crank_rate_rad_s):
    power = scalar(power_w, 'crank power request', minimum=0.)
    limit = scalar(torque_limit_nm, 'crank torque ceiling', minimum=0.)
    rate = scalar(crank_rate_rad_s, 'crank rate')
    speed_floor = 20. * 2. * pi / 60.
    return min(limit, power / max(max(0., rate), speed_floor))


class SeatedClimbPolicy:
    def __init__(self, config: SeatedClimbConfig):
        if not isinstance(config, SeatedClimbConfig):
            raise ValueError('expected a seated climb configuration')
        self.config = config
        # The lean intent lives in the posture program; this class only
        # estimates inclination from delayed proprioceptive signals and
        # shapes the effort ceiling.
        self.program = SeatedPostureProgram(config)
        self.reset()

    def reset(self):
        self.time_s = 0.
        self.inclination_rad = 0.
        self.lean_rad = 0.
        self.effort_nm = 0.
        self._samples = deque()
        self._delayed = None
        self.program.reset()

    def update(self, signals: SeatedClimbSignals, dt_s: float, *, road_grade: float) -> SeatedClimbIntent:
        if not isinstance(signals, SeatedClimbSignals):
            raise ValueError('expected finite seated rider signals')
        dt = scalar(dt_s, 'rider intention interval', positive=True)
        road_grade = scalar(road_grade, 'posture road grade')
        config = self.config
        if not config.enabled:
            return SeatedClimbIntent(RiderPosture(), 0.)
        self._samples.append((self.time_s, signals))
        ready_time = self.time_s - config.reaction_delay_s
        while self._samples and self._samples[0][0] <= ready_time + 1e-12:
            self._delayed = self._samples.popleft()[1]
        if self._delayed is not None:
            delayed = self._delayed
            self.inclination_rad += delayed.pitch_rate_up_rad_s * dt
            forward, lateral, upward = delayed.specific_force_body_mps2
            norm = hypot(forward, lateral, upward)
            if .8 * 9.81 <= norm <= 1.2 * 9.81:
                estimate = atan2(forward, upward)
                difference = atan2(sin(estimate-self.inclination_rad), cos(estimate-self.inclination_rad))
                self.inclination_rad += (1.-exp(-dt/config.orientation_tau_s))*difference
            self.inclination_rad = atan2(sin(self.inclination_rad), cos(self.inclination_rad))
            target_effort = crank_effort_ceiling(config.target_crank_power_w,
                config.max_crank_torque_nm, delayed.crank_rate_rad_s)
            self.effort_nm += max(-config.torque_slew_nm_s*dt,
                min(config.torque_slew_nm_s*dt, target_effort-self.effort_nm))
        self.lean_rad = self.program.update(self.time_s, road_grade, dt)
        self.time_s += dt
        return SeatedClimbIntent(RiderPosture(torso_lean_rad=self.lean_rad, use_saddle=True), self.effort_nm)
