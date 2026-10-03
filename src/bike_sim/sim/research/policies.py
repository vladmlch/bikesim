"""Demo plumbing policies for the batch runner: not anti-wheelie solutions.

They only show the `policy(observation, demand_nm) -> RideControl` wiring and
give the batch report something to compare against. None of them estimates or
prevents a wheelie; the real policy is the user's to write.
"""
from dataclasses import replace

from bike_sim.physics.checks import scalar
from bike_sim.physics.pedaling import ProgressStallDetector
from bike_sim.sim.ride.control import RideControl


def _request(demand_nm):
    # RideControl(motor_torque_nm=None) would silently select pedelec assistance.
    return 0. if demand_nm is None else float(demand_nm)


def passthrough(observation, demand_nm):
    return RideControl(motor_torque_nm=_request(demand_nm))


def zero(observation, demand_nm):
    return RideControl(motor_torque_nm=0.)


def fixed_limit_40(observation, demand_nm):
    return RideControl(motor_torque_nm=_request(demand_nm), motor_limit_nm=40.)


POLICIES = {'passthrough': passthrough, 'zero': zero, 'fixed_limit_40': fixed_limit_40}


class PassthroughPolicy:
    def reset(self, seed):
        self.seed = seed

    def act(self, observation, demand_nm):
        return RideControl(motor_torque_nm=demand_nm)


class ZeroPolicy(PassthroughPolicy):
    def act(self, observation, demand_nm):
        return RideControl(motor_torque_nm=0.)


class FixedLimitPolicy(PassthroughPolicy):
    def act(self, observation, demand_nm):
        return RideControl(motor_torque_nm=demand_nm, motor_limit_nm=40.)


def passthrough_factory():
    return PassthroughPolicy()


def zero_factory():
    return ZeroPolicy()


def fixed_limit_40_factory():
    return FixedLimitPolicy()


class RepositionOnStallPolicy(PassthroughPolicy):
    """Passthrough plus one crank-reposition pulse per detected stall.

    Stalled means a loaded drive where neither the crank nor the rear wheel
    advanced past its progress threshold within the window -- the
    encoder-only signature of being caught on a dead spot, including the
    rocking variant that never looks slow (see ProgressStallDetector). The
    pulse is a single rising edge; the plant owns the maneuver itself.
    Requires the drive.motor_clutch topology or the request is rejected at
    validation.
    """
    def __init__(self, window_s=1., crank_progress_rad=.5, wheel_progress_rad=.3, cooldown_s=2.):
        if cooldown_s < 0:
            raise ValueError('cooldown_s must be nonnegative')
        self.detector = ProgressStallDetector(window_s, crank_progress_rad, wheel_progress_rad)
        self.cooldown_s = float(cooldown_s)

    def reset(self, seed):
        super().reset(seed)
        self.detector.reset()
        self._last_t = None
        self._cooldown_until = -float('inf')

    def act(self, observation, demand_nm):
        control = super().act(observation, demand_nm)
        elapsed = 0. if self._last_t is None else max(0., observation.time_s-self._last_t)
        self._last_t = observation.time_s
        loaded = (observation.valid
                  and (demand_nm is None or demand_nm > 0.)
                  and observation.time_s >= self._cooldown_until)
        if self.detector.update(observation.crank_rad_s, observation.rear_wheel_rad_s,
                                elapsed, loaded=loaded):
            self._cooldown_until = observation.time_s+self.cooldown_s
            return replace(control, crank_reposition=True)
        return control


def reposition_on_stall_factory():
    return RepositionOnStallPolicy()


class TractionControlPolicy(PassthroughPolicy):
    """Leaky-integrator wheel-slip cap on top of a base motor policy.

    Slip is estimated from the wheel encoders only --
    ``rear_radius_m*rear_wheel_rad_s - front_radius_m*front_wheel_rad_s``;
    positive means rear wheelspin. While slip exceeds the target the motor
    ceiling decays proportionally to the excess; once the tire re-grips the
    ceiling recovers at a fixed rate, so the cap remembers across brief
    re-grips how much torque the contact could not transmit (mu_slide*N vs
    mu_peak*N). It only ever removes torque the tire cannot use; it never
    adds any and cannot preempt a slip excursion inside the loop delay.

    The cap's reference is the tightest level where limiting stops mattering:
    the base's own ``motor_limit_nm``, else its ``motor_torque_nm`` request,
    else (a pure pedelec base) the measured applied torque. Reaching that
    level disengages the cap back to a plain passthrough.

    A stopped rear wheel under a numeric base demand releases
    ``motor_torque_nm`` to None after ``standstill_release_s``: a numeric
    demand, even 0, keeps the plant's `pressing` alive and could wedge its
    stall latch. The release holds while the wheel stays stopped and the
    base demand is restored the tick the wheel moves again.

    Rider fields are never owned: only ``motor_torque_nm`` and
    ``motor_limit_nm`` are written; ``crank_reposition`` and everything else
    pass through from the base unchanged.
    """
    def __init__(self, base=None, *, front_radius_m=.372, rear_radius_m=.352,
                 slip_target_mps=.2, cut_gain_nm_s_per_mps=200.,
                 recover_nm_s=25., min_limit_nm=8.,
                 standstill_wheel_rad_s=.3, standstill_release_s=.8):
        if (base is not None and not callable(getattr(base, 'act', None))
                and not callable(base)):
            raise ValueError('base must be a policy object or a callable')
        self._base = base
        self.front_radius_m = scalar(front_radius_m, 'front wheel radius', positive=True)
        self.rear_radius_m = scalar(rear_radius_m, 'rear wheel radius', positive=True)
        self.slip_target_mps = scalar(slip_target_mps, 'slip target', minimum=0.)
        self.cut_gain_nm_s_per_mps = scalar(cut_gain_nm_s_per_mps, 'slip cut gain', positive=True)
        self.recover_nm_s = scalar(recover_nm_s, 'cap recovery rate', positive=True)
        self.min_limit_nm = scalar(min_limit_nm, 'minimum torque limit', minimum=0.)
        self.standstill_wheel_rad_s = scalar(standstill_wheel_rad_s,
                                             'standstill wheel speed', minimum=0.)
        self.standstill_release_s = scalar(standstill_release_s,
                                           'standstill release delay', minimum=0.)
        self._cap_nm = None
        self._last_t = None
        self._standstill_s = 0.

    @property
    def limit_nm(self):
        """The active motor ceiling in N*m; None while the cap is disengaged."""
        return self._cap_nm

    def reset(self, seed):
        super().reset(seed)
        if self._base is not None and callable(getattr(self._base, 'reset', None)):
            self._base.reset(seed)
        self._cap_nm = None
        self._last_t = None
        self._standstill_s = 0.

    def act(self, observation, demand_nm):
        base = self._base
        if base is None:
            control = super().act(observation, demand_nm)
        elif callable(getattr(base, 'act', None)):
            control = base.act(observation, demand_nm)
        else:
            control = base(observation, demand_nm)
        elapsed = 0. if self._last_t is None else max(0., observation.time_s-self._last_t)
        self._last_t = observation.time_s
        if not observation.valid:
            return control
        slip = (self.rear_radius_m*observation.rear_wheel_rad_s
                - self.front_radius_m*observation.front_wheel_rad_s)
        ceiling = control.motor_limit_nm
        if ceiling is None:
            ceiling = control.motor_torque_nm
        if ceiling is None:
            ceiling = observation.motor_torque_nm
        ceiling = max(0., ceiling)
        cap = self._cap_nm
        if cap is None:
            if slip > self.slip_target_mps:
                cap = ceiling
        else:
            cap = min(cap, ceiling)
        if cap is not None:
            if slip > self.slip_target_mps:
                cap -= self.cut_gain_nm_s_per_mps*(slip-self.slip_target_mps)*elapsed
            else:
                cap += self.recover_nm_s*elapsed
            cap = min(max(cap, min(self.min_limit_nm, ceiling)), ceiling)
            if cap >= ceiling:
                cap = None
            self._cap_nm = cap
            if cap is not None:
                control = replace(control, motor_limit_nm=cap)
        stopped = abs(observation.rear_wheel_rad_s) < self.standstill_wheel_rad_s
        if stopped and control.motor_torque_nm is not None:
            self._standstill_s += elapsed
        elif not stopped:
            self._standstill_s = 0.
        if stopped and self._standstill_s >= self.standstill_release_s:
            control = replace(control, motor_torque_nm=None)
        return control


def traction_control_factory():
    return TractionControlPolicy()
