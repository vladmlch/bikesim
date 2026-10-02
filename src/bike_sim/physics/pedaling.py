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
    """Rider intention and a bounded stopping goal, independent of torque sensing.

    ``reposition`` is the crank-reposition request: a rising edge starts one
    backpedal-to-power-phase maneuver through the ordinary coasting servo
    (crank phase target + rate damping). While it runs, effort is zero and the
    mash ramp cannot fight the reverse rotation; on completion, abort or
    timeout the normal path resumes and effort slews back from zero.
    """
    _APPROACH_TAU_S = .05
    _STALL_RATE_RAD_S = .3

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.coasting = False
        self.target_phase_rad = None
        self.target_rate_rad_s = 0.
        self.deceleration_rad_s2 = 0.
        self._effort = None
        self._reposition_armed = True
        self._reposition_goal = None
        self._reposition_reason = ''
        self._reposition_elapsed = 0.
        self._stall_dwell = 0.
        self._reposition_cooldown = 0.

    def update(self, phase_rad, rate_rad_s, required_cadence_rpm, effort_nm, dt,
               *, enabled=True, braking=False, reposition=False):
        phase_rad = scalar(phase_rad, 'crank phase')
        rate_rad_s = scalar(rate_rad_s, 'crank rate')
        required_cadence_rpm = max(scalar(required_cadence_rpm, 'required cadence'), 0.)
        effort_nm = scalar(effort_nm, 'rider effort', minimum=0.)
        dt = scalar(dt, 'pedaling interval', positive=True)
        if not isinstance(enabled, bool) or not isinstance(braking, bool) \
                or not isinstance(reposition, bool):
            raise ValueError('pedaling enable, braking and reposition must be booleans')
        if not enabled:
            self.reset()
            # A flag held across a disable/enable toggle stays consumed: only a
            # fresh falling/rising pair earns a maneuver.
            self._reposition_armed = not reposition
            return PedalingState('disabled', 'disabled', 0., required_cadence_rpm)
        self._reposition_cooldown = max(0., self._reposition_cooldown-dt)
        if braking:
            # Abort on braking: the maneuver never outranks a stop request.
            self._reposition_goal = None
            self._reposition_cooldown = self.config.reposition_cooldown_s
        # A held request cannot retrigger: it must fall before the next edge.
        if reposition:
            request_edge = self._reposition_armed
            self._reposition_armed = False
        else:
            self._reposition_armed = True
            request_edge = False
        # Opt-in stall reflex: commanded effort with neither the crank nor the
        # wheel moving means the rider is stalled against a dead spot.
        stalled = (self.config.reposition_on_stall and not braking
                   and self._reposition_goal is None
                   and self._reposition_cooldown <= 0.
                   and effort_nm >= self.config.reposition_min_effort_nm
                   and abs(rate_rad_s) < self._STALL_RATE_RAD_S
                   and required_cadence_rpm < self.config.reposition_stall_cadence_rpm)
        self._stall_dwell = self._stall_dwell+dt if stalled else 0.
        reflex_edge = self._stall_dwell >= self.config.reposition_stall_dwell_s
        if reflex_edge:
            self._stall_dwell = 0.
        for edge, source in ((request_edge, 'requested'), (reflex_edge, 'stall_reflex')):
            if (braking or not edge or self._reposition_goal is not None
                    or self._reposition_cooldown > 0.):
                continue
            # Power positions are the two crank-horizontal poses: front arm
            # leads at even half-turns, the rear arm at odd ones. Backpedaling
            # reaches the nearest lower boundary in at most pi rad.
            turn = phase_rad % pi
            if min(turn, pi-turn) <= self.config.reposition_noop_rad:
                continue
            self._reposition_goal = phase_rad-turn
            self._reposition_reason = source
            self._reposition_elapsed = 0.
            self._effort = 0.
            self.coasting = False
            break
        if self._reposition_goal is not None:
            remaining = phase_rad-self._reposition_goal
            self._reposition_elapsed += dt
            done = remaining <= self.config.reposition_phase_tolerance_rad
            timed_out = self._reposition_elapsed > self.config.reposition_timeout_s
            if done or timed_out:
                self._reposition_goal = None
                self._reposition_cooldown = self.config.reposition_cooldown_s
            else:
                rate = -min(self.config.reposition_back_rate_rad_s,
                            remaining/self._APPROACH_TAU_S)
                return PedalingState('reposition', self._reposition_reason, 0.,
                    required_cadence_rpm, phase_rad+rate*dt, rate)
        cadence = max(abs(rate_rad_s) * 60. / (2. * pi), required_cadence_rpm)
        threshold = (self.config.resume_below_rpm if self.coasting
                     else self.config.coast_above_rpm)
        excessive = self.config.enabled and cadence >= threshold
        reason = ('braking' if braking else 'no_effort' if effort_nm == 0.
                  else 'cadence' if excessive else '')
        if not reason:
            previous = self._effort
            latch = (self._reposition_armed, self._reposition_cooldown, self._stall_dwell)
            self.reset()
            self._effort = previous
            self._reposition_armed, self._reposition_cooldown, self._stall_dwell = latch
            # Muscle force-velocity is inverted relative to the naive constant
            # effort: as cadence collapses a real rider converts to standing on
            # the pedal, and the available torque rises toward the isometric
            # ceiling instead of fading. The ramp is linear in cadence and
            # never reduces the commanded effort. The slew bound models force
            # development and initializes from the commanded effort, so a mash
            # builds over a fraction of a second rather than appearing as an
            # impulse that rips the feet off the pedals.
            target = effort_nm
            if self.config.mash_torque_nm > target and cadence < self.config.mash_cadence_rpm:
                target = effort_nm + (self.config.mash_torque_nm-effort_nm)*(1.-cadence/self.config.mash_cadence_rpm)
            slew = self.config.effort_slew_nm_s
            if slew <= 0.:
                effort = target
            else:
                base = effort_nm if self._effort is None else self._effort
                effort = max(base-slew*dt, min(base+slew*dt, target))
            self._effort = effort
            return PedalingState('pedaling', '', effort, required_cadence_rpm)
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
