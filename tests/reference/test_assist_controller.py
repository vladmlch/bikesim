"""AssistController: Bosch-like support factor through a 40 ms lag, gated as a permission."""
import math
import copy

import pytest

from bike_sim.physics.motor import AssistController

RPM = 2*math.pi/60


def _turbo(**kw):
    return AssistController(profile='bosch_cx_gen4', mode='turbo', **kw)


def _run(ctrl, human, cadence_rpm, speed, seconds, dt=.001, request=None):
    torque = 0.
    for _ in range(round(seconds/dt)):
        torque = ctrl.step(human, cadence_rpm, speed, False, dt, torque_request_nm=request)
    return torque


def test_profile_owns_limits_and_lag():
    c = _turbo()
    assert (c.max_torque, c.max_power, c.tau) == (85., 600., .04)
    assert c.cutoff == pytest.approx(25/3.6) and c.width == pytest.approx(2/3.6)
    assert c.gate_min_crank_rad_s == pytest.approx(math.radians(5.))
    assert c.profile.name.startswith('bosch') and c.mode == 'turbo'


def test_turbo_multiplies_rider_torque_through_a_40ms_first_order_lag():
    c = _turbo()
    after_40ms = _run(c, 20., 60., 3., .04)
    assert after_40ms == pytest.approx(68.*(1-math.exp(-1.)), rel=.03)
    settled = _run(c, 20., 60., 3., .5)
    assert settled == pytest.approx(68., rel=1e-3)
    assert c.last_gain == pytest.approx(3.4)


def test_emtb_gain_rises_with_rider_torque():
    c = AssistController(profile='bosch_cx_gen4', mode='emtb')
    assert _run(c, 10., 60., 3., .5) == pytest.approx((1.4+.5)*10., rel=1e-3)


def test_stationary_or_backward_crank_gets_nothing_even_with_pedal_pressure():
    c = _turbo()
    for cadence in (0., .5, -30.):
        assert _run(c, 50., cadence, 0., .2) == 0.
    # 5 deg/s is the gate itself; just above it assist is allowed
    assert _run(_turbo(), 50., 5./6.+1e-3, 0., .2) > 0.
    assert _run(_turbo(), 50., 5./6.-1e-3, 0., .2) == 0.


def test_torque_below_engage_threshold_is_not_pedalling():
    c = _turbo(engage_torque_nm=4.)
    assert _run(c, 4., 60., 3., .2) == 0.
    assert _run(c, 4.01, 60., 3., .2) > 0.


def test_release_of_rider_torque_hard_zeroes_and_forgets():
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    assert c.step(0., 60., 3., False, .001) == 0.
    assert c.torque == 0. and not c.pedaling
    # Permission restored: the lag restarts from zero, not from the old 68 N.m
    assert c.step(20., 60., 3., False, .001) < 5.


def test_positive_unloading_keeps_the_lag():
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    first = c.step(10., 60., 3., False, .001)
    assert 34. < first < 68.


def test_speed_taper_and_cutoff():
    kmh = lambda v: v/3.6
    assert _run(_turbo(), 20., 60., kmh(22.), .5) == pytest.approx(68., rel=1e-3)
    assert _run(_turbo(), 20., 60., kmh(24.), .5) == pytest.approx(34., rel=1e-2)
    assert _run(_turbo(), 20., 60., kmh(25.), .5) == 0.
    assert _run(_turbo(), 20., 60., kmh(30.), .5) == 0.


def test_torque_curve_and_power_ceiling_bind_on_shaft_rpm():
    curve = [[0., 85.], [120., 85.], [120.1, 47.7], [180., 0.]]
    c = _turbo(torque_curve=curve)
    # 60 rpm: 600 W / 6.283 rad/s = 95.5 > 85 -> peak torque binds
    assert _run(c, 100., 60., 3., .5) == pytest.approx(85., rel=1e-3)
    # 150 rpm: curve 23.85 N.m vs power 38.2 N.m -> curve binds
    assert _run(_turbo(torque_curve=curve), 100., 150., 3., .5) == pytest.approx(23.85, rel=1e-2)
    # shaft_rpm overrides cadence for the ceiling (legacy clutch topology)
    c = _turbo(torque_curve=curve)
    for _ in range(500):
        t = c.step(100., 60., 3., False, .001, shaft_rpm=150.)
    assert t == pytest.approx(23.85, rel=1e-2)


def test_external_request_is_a_ceiling_not_a_throttle():
    assert _run(_turbo(), 0., 60., 3., .5, request=100.) == 0.
    assert _run(_turbo(), 20., 60., 3., .5, request=100.) == pytest.approx(68., rel=1e-3)
    assert _run(_turbo(), 20., 60., 3., .5, request=30.) == pytest.approx(30., rel=1e-3)
    assert _run(_turbo(), 20., 60., 3., .5, request=0.) == 0.


def test_braking_resets_everything():
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    assert c.step(20., 60., 3., True, .001) == 0.
    assert c.torque == 0.


def test_without_profile_the_plain_gain_is_used():
    c = AssistController(gain=4., tau=.3, max_torque=85., max_power=600.)
    assert c.profile is None
    assert _run(c, 10., 60., 3., 3.) == pytest.approx(40., rel=1e-3)


def test_rejects_unknown_profile_mode_and_removed_knobs():
    with pytest.raises(ValueError):
        AssistController(profile='shimano_ep8')
    with pytest.raises(ValueError):
        AssistController(profile='bosch_cx_gen4', mode='sport')
    for knob in ('stall_timeout_s', 'spin_rpm', 'boost_s', 'stop_delay'):
        with pytest.raises(TypeError):
            AssistController(**{knob: 1.})


def test_assist_config_accepts_profile_and_mode():
    from bike_sim.physics.physical_config import AssistConfig
    cfg = AssistConfig(profile='bosch_cx_gen4', mode='emtb')
    assert cfg.mode == 'emtb'
    with pytest.raises(ValueError):
        AssistConfig(profile='bosch_cx_gen4', mode='off')
    with pytest.raises(TypeError):
        AssistConfig(stall_timeout_s=1.)


def test_slow_forward_pedalling_is_assisted_without_a_stall_timer():
    c = _turbo()
    assert _run(c, 20., 1., 0., 2.) == pytest.approx(68., rel=1e-3)


@pytest.mark.parametrize('cadence', [0., -30., 5./6.])
def test_crank_gate_resets_the_filter_even_with_an_external_cap(cadence):
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    assert c.step(20., cadence, 3., False, .001, torque_request_nm=100.) == 0.
    assert c.torque == 0.
    assert c.step(20., 60., 3., False, .001) < 5.


def test_cutoff_is_instantaneous_after_assist_has_settled():
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    assert c.step(20., 60., 25./3.6, False, .001) == 0.
    assert c.torque == 0.


def test_profile_overrides_explicit_torque_power_tau_speed_and_gate():
    c = _turbo(max_torque=1., max_power=1., tau=1., cutoff_mps=1.,
               taper_width_mps=.1, gate_min_crank_rad_s=10.)
    assert _run(c, 20., 60., 3., .5) == pytest.approx(68., rel=1e-3)


@pytest.mark.parametrize('changes', [{'tau': math.nan}, {'slew': math.inf},
                                     {'taper_width_mps': math.nan}, {'mode': []}])
def test_controller_rejects_invalid_parameters(changes):
    with pytest.raises(ValueError):
        AssistController(**changes)


def test_assist_config_exposes_profile_torque_limit():
    from bike_sim.physics.physical_config import AssistConfig
    assert AssistConfig(profile='bosch_cx_gen4', max_torque=1.).effective_max_torque == 85.
    assert AssistConfig(max_torque=27.).effective_max_torque == 27.


def test_profile_controller_can_be_copied_for_read_only_force_probes():
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    probe = copy.deepcopy(c)
    probe.step(0., 60., 3., False, .001)
    assert probe.torque == 0.
    assert c.torque == pytest.approx(68., rel=1e-3)
    assert probe.profile is c.profile


def test_physical_cli_assist_selects_mode_without_replacing_the_toml_profile(tmp_path):
    from bike_sim.cli.ride import parse_args
    config = tmp_path/'physics.toml'
    config.write_text('physics_mode = "physical"\ndrive_mode = "crank_effort"\n'
                      '[drive.assist]\nprofile = "bosch_cx_gen4"\nmode = "turbo"\n')
    args = parse_args(['--physics-config', str(config), '--assist', 'emtb'])
    assert args.resolved_physics.drive.assist.profile == 'bosch_cx_gen4'
    assert args.resolved_physics.drive.assist.mode == 'emtb'
    defaults = parse_args(['--physics-config', str(config)])
    assert defaults.resolved_physics.drive.assist.mode == 'turbo'
