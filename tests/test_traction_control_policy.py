"""Encoder-only wheel-slip traction cap: leaky integrator + standstill release.

Unit-level tests over TractionControlPolicy; no plant instances. The mullet
radii (front .372 m, rear .352 m) make slip = .352*rear - .372*front, so equal
wheel speeds read slightly negative -- engagement needs real rear wheelspin.
Observation times must stay contiguous: the policy integrates on time deltas.
"""
import pytest

from bike_sim.sim.research.policies import (
    PassthroughPolicy, TractionControlPolicy, traction_control_factory,
)
from bike_sim.sim.research.policy_session import load_policy
from bike_sim.sim.research.sensors import SensorObservation
from bike_sim.sim.ride.control import RideControl


DT = .01


def obs(t, *, front=0., rear=0., crank=0., motor=0., valid=True):
    return SensorObservation(time_s=t, source_time_s=t, valid=valid,
        specific_force_body_mps2=(0., 0., 9.81), pitch_rate_up_rad_s=0.,
        front_wheel_rad_s=front, rear_wheel_rad_s=rear, crank_rad_s=crank,
        motor_torque_nm=motor, human_torque_nm=0.)


def drive(policy, clock, steps, demand, **state):
    """Feed `steps` observations at the mutable clock, advancing DT per tick."""
    outs = []
    for _ in range(steps):
        outs.append(policy.act(obs(clock[0], **state), demand))
        clock[0] += DT
    return outs


def spin_out(policy, clock, steps=200, demand=80.):
    """rear=3 rad/s, front stopped -> slip = 1.056 m/s >> .2 target."""
    return drive(policy, clock, steps, demand, front=0., rear=3.)


class FixedBase:
    """Stand-in base policy: returns one fixed control, records resets."""
    def __init__(self, control):
        self.control = control
        self.resets = []

    def reset(self, seed):
        self.resets.append(seed)

    def act(self, observation, demand_nm):
        return self.control


# --- constructor surface ------------------------------------------------------

def test_constructor_validation():
    with pytest.raises(ValueError, match='radius'):
        TractionControlPolicy(front_radius_m=0.)
    with pytest.raises(ValueError):
        TractionControlPolicy(rear_radius_m=-.352)
    with pytest.raises(ValueError):
        TractionControlPolicy(slip_target_mps=-1.)
    with pytest.raises(ValueError):
        TractionControlPolicy(cut_gain_nm_s_per_mps=0.)
    with pytest.raises(ValueError):
        TractionControlPolicy(recover_nm_s=0.)
    with pytest.raises(ValueError):
        TractionControlPolicy(min_limit_nm=-1.)
    with pytest.raises(ValueError):
        TractionControlPolicy(standstill_wheel_rad_s=-.1)
    with pytest.raises(ValueError):
        TractionControlPolicy(standstill_release_s=-.1)
    with pytest.raises(ValueError, match='base'):
        TractionControlPolicy(base=42)


def test_factory_and_loader_contract():
    first, second = traction_control_factory(), traction_control_factory()
    assert isinstance(first, TractionControlPolicy) and first is not second
    loaded = load_policy('bike_sim.sim.research.policies:traction_control_factory')
    assert isinstance(loaded, TractionControlPolicy)
    loaded.reset(0)
    assert isinstance(loaded.act(obs(0.), 30.), RideControl)


# --- slip cap -----------------------------------------------------------------

def test_no_slip_is_plain_passthrough():
    clock = [0.]
    p = TractionControlPolicy()
    p.reset(0)
    base = PassthroughPolicy()
    # Equal wheel speeds on the mullet radii read negative slip: never engages.
    for out in drive(p, clock, 50, 60., front=10., rear=10.):
        assert out == base.act(None, 60.) == RideControl(motor_torque_nm=60.)
    assert p.limit_nm is None


def test_sustained_slip_decays_cap_monotonically_to_floor():
    clock = [0.]
    p = TractionControlPolicy()
    p.reset(0)
    outs = spin_out(p, clock, steps=200)
    limits = [o.motor_limit_nm for o in outs]
    numeric = [v for v in limits if v is not None]
    assert len(numeric) > 150, 'cap did not stay engaged under sustained slip'
    # Monotone decay into the floor, then held: never above demand, never < 0.
    assert all(b <= a + 1e-12 for a, b in zip(numeric, numeric[1:]))
    assert numeric[0] < 80.
    assert min(numeric) == pytest.approx(8.)
    assert all(v == pytest.approx(8.) for v in numeric[-40:])
    assert p.limit_nm == pytest.approx(8.)
    # The cap cuts the ceiling only; the setpoint is untouched while turning.
    assert all(o.motor_torque_nm == 80. for o in outs)


def test_min_limit_floor_reaches_zero_when_allowed():
    clock = [0.]
    p = TractionControlPolicy(min_limit_nm=0.)
    p.reset(0)
    outs = spin_out(p, clock, steps=300)
    assert outs[-1].motor_limit_nm == pytest.approx(0.)
    assert p.limit_nm == pytest.approx(0.)


def test_cap_recovers_at_recover_rate_then_disengages():
    clock = [0.]
    p = TractionControlPolicy()
    p.reset(0)
    spin_out(p, clock, steps=100)
    assert p.limit_nm == pytest.approx(8.)
    # Re-grip while still rolling: equal speeds, no standstill interference.
    outs = drive(p, clock, 120, 80., front=10., rear=10.)
    # +25 Nm/s recovery -> +0.25 per 10 ms tick, never above the demand level.
    caps = [o.motor_limit_nm for o in outs]
    assert all(v is not None and v < 80. for v in caps)
    assert all(b - a == pytest.approx(.25) for a, b in zip(caps, caps[1:]))
    assert p.limit_nm == pytest.approx(8.+.25*120, abs=.51)
    # Long clean running disengages back to a plain passthrough (base had no
    # limit, so the output returns to motor_limit_nm=None).
    outs = drive(p, clock, 400, 80., front=10., rear=10.)
    assert p.limit_nm is None
    assert all(o.motor_limit_nm is None for o in outs[-50:])


def test_cap_never_exceeds_base_limit():
    clock = [0.]
    base = FixedBase(RideControl(motor_torque_nm=80., motor_limit_nm=40.))
    p = TractionControlPolicy(base)
    p.reset(0)
    outs = drive(p, clock, 200, 80., front=0., rear=3.)
    engaged = [o.motor_limit_nm for o in outs
               if o.motor_limit_nm is not None and o.motor_limit_nm < 40.]
    assert engaged and min(engaged) == pytest.approx(8.)
    assert all(o.motor_limit_nm <= 40. for o in outs)
    # Long re-grip: the cap must never exceed the base's own 40 Nm ceiling, and
    # once recovered the base limit passes through untouched.
    outs = drive(p, clock, 400, 80., front=10., rear=10.)
    assert all(o.motor_limit_nm <= 40. for o in outs)
    assert p.limit_nm is None
    assert all(o.motor_limit_nm == 40. for o in outs[-50:])


def test_brief_regrip_keeps_cap_memory():
    clock = [0.]
    p = TractionControlPolicy()
    p.reset(0)
    spin_out(p, clock, steps=60)                      # cap at the 8 Nm floor
    mid = p.limit_nm
    assert mid == pytest.approx(8.)
    drive(p, clock, 10, 80., front=10., rear=10.)     # 0.1 s of grip lifts it
    lifted = p.limit_nm
    assert lifted == pytest.approx(mid+2.5, abs=.05)
    outs = spin_out(p, clock, steps=5)
    # Cutting resumes from the remembered cap, not from the full ceiling.
    assert outs[0].motor_limit_nm == pytest.approx(lifted-200.*.856*DT, abs=.05)
    assert outs[0].motor_limit_nm < 20.


# --- standstill press-release --------------------------------------------------

def test_standstill_releases_pressing_and_restores_on_motion():
    clock = [0.]
    p = TractionControlPolicy()
    p.reset(0)
    outs = drive(p, clock, 200, 80., front=0., rear=0.)
    first_none = next(i for i, o in enumerate(outs) if o.motor_torque_nm is None)
    assert 70 <= first_none <= 90            # ~standstill_release_s / DT
    assert all(o.motor_torque_nm is None for o in outs[first_none:])
    # The release holds while the wheel stays stopped...
    outs = drive(p, clock, 50, 80., front=0., rear=0.)
    assert all(o.motor_torque_nm is None for o in outs)
    # ...and the base demand returns the tick the wheel moves again.
    out = p.act(obs(clock[0], front=5., rear=5.), 80.)
    assert out.motor_torque_nm == 80.


def test_zero_motor_torque_still_counts_as_demand():
    # A numeric 0 keeps the plant's `pressing` alive just like any number.
    clock = [0.]
    base = FixedBase(RideControl(motor_torque_nm=0.))
    p = TractionControlPolicy(base)
    p.reset(0)
    outs = drive(p, clock, 200, 0., front=0., rear=0.)
    assert outs[10].motor_torque_nm == 0.    # still held early on
    assert all(o.motor_torque_nm is None for o in outs[95:])


def test_standstill_timer_ignores_pedelec_base():
    clock = [0.]
    base = FixedBase(RideControl(motor_torque_nm=None))
    p = TractionControlPolicy(base)
    p.reset(0)
    # Pedelec base: nothing is pressing; the timer must not accumulate and the
    # passthrough must not be rewritten.
    outs = drive(p, clock, 200, None, front=0., rear=0.)
    assert all(o.motor_torque_nm is None and o.motor_limit_nm is None for o in outs)
    assert p._standstill_s == 0.


# --- field ownership -----------------------------------------------------------

def test_rider_fields_and_reposition_survive_the_wrap():
    clock = [0.]
    base = FixedBase(RideControl(motor_torque_nm=60., human_torque_nm=20.,
                                 crank_reposition=True))
    p = TractionControlPolicy(base)
    p.reset(0)
    outs = spin_out(p, clock, steps=150, demand=60.)
    out = outs[-1]
    assert out.motor_limit_nm == pytest.approx(8.)   # cap path ran
    assert out.motor_torque_nm == 60.
    assert out.crank_reposition is True
    assert out.human_torque_nm == 20.
    assert out.posture is None
    assert out.rider_enabled is True
    # The standstill-release path preserves them too.
    clock2 = [0.]
    p2 = TractionControlPolicy(base)
    p2.reset(0)
    outs = drive(p2, clock2, 120, 60., front=0., rear=0.)
    out = outs[-1]
    assert out.motor_torque_nm is None               # released
    assert out.crank_reposition is True and out.human_torque_nm == 20.


def test_callable_base_and_base_reset_forwarding():
    clock = [0.]
    base = lambda o, d: RideControl(motor_torque_nm=float(d or 0.),
                                    motor_limit_nm=40.)
    p = TractionControlPolicy(base)
    p.reset(0)                                       # no base.reset -> tolerated
    outs = drive(p, clock, 50, 80., front=0., rear=3.)
    assert all(o.motor_limit_nm <= 40. for o in outs)
    engaged = [o.motor_limit_nm for o in outs if o.motor_limit_nm < 40.]
    assert engaged and all(8. <= v for v in engaged)
    base_obj = FixedBase(RideControl(motor_torque_nm=10.))
    TractionControlPolicy(base_obj).reset(7)
    assert base_obj.resets == [7]


# --- lifecycle -----------------------------------------------------------------

def test_reset_clears_cap_and_timers():
    clock = [0.]
    p = TractionControlPolicy()
    p.reset(0)
    spin_out(p, clock, steps=60)
    assert p.limit_nm is not None
    drive(p, clock, 60, 80., front=0., rear=0.)       # ~0.6 s of standstill
    p.reset(1)
    assert p.limit_nm is None and p._standstill_s == 0. and p._last_t is None
    out = p.act(obs(2., front=0., rear=0.), 80.)
    assert out.motor_torque_nm == 80.                 # timer restarted from 0
    assert p.seed == 1


def test_invalid_observation_is_untouched_passthrough():
    clock = [0.]
    p = TractionControlPolicy()
    p.reset(0)
    spin_out(p, clock, steps=60)
    assert p.limit_nm is not None
    out = p.act(obs(clock[0], front=0., rear=3., valid=False), 80.)
    # Unchanged base control: the engaged cap is not even emitted.
    assert out == RideControl(motor_torque_nm=80.)
    cap = p.limit_nm
    clock[0] += DT
    out = p.act(obs(clock[0], front=0., rear=3., valid=False), 80.)
    assert out.motor_limit_nm is None and p.limit_nm == cap
