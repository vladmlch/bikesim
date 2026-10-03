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


class ProgressStallDetector:
    """Loaded drive that makes no forward progress within a window.

    A low-rate test misses the dead-spot stall that actually happens: the
    crank rocks back and forth around top/bottom dead centre at several rad/s
    while going nowhere, so it is never slow for long. Net forward advance of
    the crank, or of the wheel, since the last progress anchor separates a
    slow grind from a stall; rocking and rollback both count as no progress.
    """

    def __init__(self, window_s, crank_progress_rad, wheel_progress_rad):
        self.window_s = scalar(window_s, 'stall window', positive=True)
        self.crank_progress_rad = scalar(crank_progress_rad, 'crank stall progress', positive=True)
        self.wheel_progress_rad = scalar(wheel_progress_rad, 'wheel stall progress', positive=True)
        self.reset()

    def reset(self):
        self._crank_rad = 0.
        self._wheel_rad = 0.
        self._elapsed_s = None

    def update(self, crank_rad_s, wheel_rad_s, dt, *, loaded):
        """True once per stall; the next stall needs a fresh full window."""
        if not loaded:
            self.reset()
            return False
        if self._elapsed_s is None:
            self._elapsed_s = 0.
            return False
        self._crank_rad += crank_rad_s * dt
        self._wheel_rad += wheel_rad_s * dt
        if self._crank_rad > self.crank_progress_rad or self._wheel_rad > self.wheel_progress_rad:
            self.reset()
            self._elapsed_s = 0.
            return False
        self._elapsed_s += dt
        if self._elapsed_s >= self.window_s:
            self.reset()
            return True
        return False


class PedalingPolicy:
    """Rider intention and a bounded stopping goal, independent of torque sensing.

    ``reposition`` is the crank-reposition request: a rising edge starts one
    backpedal-to-power-phase maneuver through the ordinary coasting servo
    (crank phase target + rate damping). While it runs, effort is zero and the
    mash ramp cannot fight the reverse rotation; on completion, abort or
    timeout the normal path resumes and effort slews back from zero.
    """
    _APPROACH_TAU_S = .05

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.coasting = False
        self.target_phase_rad = None
        self.target_rate_rad_s = 0.
        self.deceleration_rad_s2 = 0.
        self._effort = 0.
        self._reposition_armed = True
        self._reposition_goal = None
        self._reposition_reason = ''
        self._reposition_elapsed = 0.
        self._stall = ProgressStallDetector(self.config.reposition_stall_window_s,
            self.config.reposition_stall_progress_rad, self.config.reposition_stall_progress_rad)
        self._reposition_cooldown = 0.

    def update(self, phase_rad, rate_rad_s, required_cadence_rpm, effort_nm, dt,
               *, enabled=True, braking=False, reposition=False):
        phase_rad = scalar(phase_rad, 'crank phase')
        rate_rad_s = scalar(rate_rad_s, 'crank rate')
        required_cadence_rpm = scalar(required_cadence_rpm, 'required cadence')
        # Signed wheel rate in crank-equivalent units: rollback is not progress.
        wheel_rad_s = required_cadence_rpm * 2. * pi / 60.
        required_cadence_rpm = max(required_cadence_rpm, 0.)
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
        # Opt-in stall reflex: commanded effort while neither the crank nor the
        # wheel advances means the rider is stalled against a dead spot.
        loaded = (self.config.reposition_on_stall and not braking
                  and self._reposition_goal is None
                  and self._reposition_cooldown <= 0.
                  and effort_nm >= self.config.reposition_min_effort_nm)
        reflex_edge = self._stall.update(rate_rad_s, wheel_rad_s, dt, loaded=loaded)
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
            latch = (self._reposition_armed, self._reposition_cooldown, self._stall)
            self.reset()
            self._effort = previous
            self._reposition_armed, self._reposition_cooldown, self._stall = latch
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
        self._effort = 0.
        return PedalingState('coasting', reason, 0., required_cadence_rpm,
                             self.target_phase_rad, self.target_rate_rad_s)
