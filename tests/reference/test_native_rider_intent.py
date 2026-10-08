"""Parity between the native seated-climb intent stack and the Python oracle.

RiderIntent (rider_intent.py RiderIntentResolver), SeatedClimbPolicy and
SeatedPostureProgram are exercised through NativeTestAdapter's
``rider_intent`` section and compared against the live Python objects on
identical inputs — the integer-step clock, probe-mode non-mutation, surge
budget, inclination filter, effort slew, trim delay queue, posture-program
rate limits, scheduled pulses and the state_dict wire are all part of the
contract.
"""
import math
from dataclasses import asdict

import mujoco
import numpy as np
import pytest

from native_loader import load_native

from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.physics.seated_climb import (
    SeatedClimbConfig, SeatedClimbSignals)
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.rider_intent import RiderIntentResolver

_DT = 0.001

_XML = """<mujoco>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size=".1 .1 .1" mass="20"/>
    </body>
  </worldbody>
</mujoco>"""


def _config(**overrides):
    values = {'enabled': True, 'period_s': 0.01, 'reaction_delay_s': 0.15,
              'target_crank_power_w': 225., 'max_crank_torque_nm': 60.,
              'torque_slew_nm_s': 300., 'lean_gain': 1.,
              'max_forward_lean_rad': .35, 'max_backward_lean_rad': .10,
              'lean_rate_rad_s': .5, 'orientation_tau_s': .5,
              'surge_power_w': 400., 'surge_grade': .20,
              'surge_budget_s': 15., 'surge_recovery_rate': 1. / 3.,
              'front_load_share_target': .30, 'lean_trim_gain_rad_s': .3,
              'lean_trim_limit_rad': .15, 'trim_dead_time_s': .2}
    values.update(overrides)
    return values


def _signals(**overrides):
    values = {'pitch_rate_up_rad_s': 0.,
              'specific_force_body_mps2': [0., 0., 9.81],
              'crank_rate_rad_s': 8., 'human_crank_torque_nm': 40.,
              'front_load_share': .3}
    values.update(overrides)
    return values


def _control(**overrides):
    values = {'motor_torque_nm': None, 'motor_limit_nm': None,
              'human_torque_nm': None, 'crank_target_rate_rad_s': None,
              'posture': None, 'rider_enabled': True}
    values.update(overrides)
    return values


def _py_signals(d):
    return SeatedClimbSignals(
        pitch_rate_up_rad_s=d['pitch_rate_up_rad_s'],
        specific_force_body_mps2=tuple(d['specific_force_body_mps2']),
        crank_rate_rad_s=d['crank_rate_rad_s'],
        human_crank_torque_nm=d['human_crank_torque_nm'],
        front_load_share=d['front_load_share'])


def _py_posture(d):
    if d is None:
        return None
    offset = d['pelvis_offset_m']
    return RiderPosture(torso_lean_rad=d['torso_lean_rad'],
                        pelvis_pitch_rad=d['pelvis_pitch_rad'],
                        pelvis_offset_m=None if offset is None else tuple(offset),
                        use_saddle=d['use_saddle'])


def _py_control(d):
    return RideControl(motor_torque_nm=d['motor_torque_nm'],
                       motor_limit_nm=d['motor_limit_nm'],
                       human_torque_nm=d['human_torque_nm'],
                       crank_target_rate_rad_s=d['crank_target_rate_rad_s'],
                       posture=_py_posture(d['posture']),
                       rider_enabled=d['rider_enabled'])


def _normalize(value):
    if isinstance(value, dict):
        return {key: _normalize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_normalize(item) for item in value]
    return value


def _compare(actual, expected, path=''):
    actual = _normalize(actual)
    expected = _normalize(expected)
    if isinstance(expected, dict):
        assert isinstance(actual, dict), path
        assert list(actual.keys()) == list(expected.keys()), path
        for key in expected:
            _compare(actual[key], expected[key], path + '.' + key)
        return
    if isinstance(expected, list):
        assert isinstance(actual, list), path
        assert len(actual) == len(expected), path
        for index, item in enumerate(expected):
            _compare(actual[index], item, f'{path}[{index}]')
        return
    if expected is None or isinstance(expected, bool):
        assert actual is expected or actual == expected, path
        return
    if isinstance(expected, int):
        assert actual == expected, path
        return
    assert actual == pytest.approx(expected, rel=1e-12, abs=1e-12), path


@pytest.fixture
def plant(tmp_path):
    """Native adapter + Python resolver on identical config/dt."""
    model = mujoco.MjModel.from_xml_string(_XML)
    path = tmp_path / 'intent.mjb'
    mujoco.mj_saveModel(model, str(path))
    config = _config()
    probe = load_native().NativeTestAdapter(
        str(path),
        {'rider_intent': {'config': dict(config), 'dt_s': _DT}})
    resolver = RiderIntentResolver(SeatedClimbConfig(**config), _DT)
    return probe, resolver


def _resolve(probe, resolver, step, control=None, signals=None,
             road_grade=0.02, preview_grade=None, lean_limit_rad=None,
             active=True, advance=True):
    control = _control() if control is None else control
    signals = _signals() if signals is None else signals
    native = probe.intent_resolve(
        dict(control), dict(signals), step, road_grade, preview_grade,
        lean_limit_rad, active, advance)
    expected = resolver.resolve(
        _py_control(control), _py_signals(signals), step=step,
        road_grade=road_grade, preview_grade=preview_grade,
        lean_limit_rad=lean_limit_rad, active=active, advance=advance)
    return native, asdict(expected)


def test_resolve_matches_oracle_over_mixed_steps(plant):
    probe, resolver = plant
    for step in range(35):
        signals = _signals(pitch_rate_up_rad_s=0.01 * step,
                           crank_rate_rad_s=8. + 0.1 * step,
                           front_load_share=.28 + .001 * step)
        native, expected = _resolve(probe, resolver, step, signals=signals,
                                    road_grade=0.02 + 0.001 * step,
                                    preview_grade=0.05,
                                    lean_limit_rad=.6)
        _compare(native, expected)
        _compare(probe.intent_state(), resolver.state_dict())


def test_resolve_disabled_passthrough(tmp_path):
    model = mujoco.MjModel.from_xml_string(_XML)
    path = tmp_path / 'intent.mjb'
    mujoco.mj_saveModel(model, str(path))
    config = _config(enabled=False)
    probe = load_native().NativeTestAdapter(
        str(path),
        {'rider_intent': {'config': dict(config), 'dt_s': _DT}})
    resolver = RiderIntentResolver(SeatedClimbConfig(**config), _DT)
    for step in (0, 3, 7):
        native, expected = _resolve(probe, resolver, step,
                                    signals=_signals(front_load_share=None),
                                    road_grade=float('nan'))
        _compare(native, expected)
    _compare(probe.intent_state(), resolver.state_dict())


def test_resolve_merges_only_unset_fields(plant):
    probe, resolver = plant
    held = {'torso_lean_rad': .12, 'pelvis_pitch_rad': -.03,
            'pelvis_offset_m': [.01, .1], 'use_saddle': False}
    native, expected = _resolve(
        probe, resolver, 0,
        control=_control(posture=held, human_torque_nm=55.))
    _compare(native, expected)
    assert native['posture']['torso_lean_rad'] == pytest.approx(.12)
    assert native['human_torque_nm'] == pytest.approx(55.)


def test_resolve_partial_merge(plant):
    probe, resolver = plant
    native, expected = _resolve(probe, resolver, 0,
                                control=_control(human_torque_nm=25.))
    _compare(native, expected)
    assert native['posture'] is not None
    assert native['human_torque_nm'] == pytest.approx(25.)


def test_resolve_rider_disabled_keeps_control_but_ticks(plant):
    probe, resolver = plant
    control = _control(rider_enabled=False)
    native, expected = _resolve(probe, resolver, 0, control=control)
    _compare(native, expected)
    assert native['posture'] is None
    assert native['human_torque_nm'] is None
    # The acquisition still ran — only the merge is gated.
    _compare(probe.intent_state(), resolver.state_dict())
    assert probe.intent_state()['last_tick_step'] == 0


def test_resolve_inactive_returns_control(plant):
    probe, resolver = plant
    native, expected = _resolve(probe, resolver, 0, active=False)
    _compare(native, expected)
    _compare(probe.intent_state(), resolver.state_dict())
    assert probe.intent_state()['last_tick_step'] == -1


def test_probe_advance_preserves_state(plant):
    probe, resolver = plant
    _resolve(probe, resolver, 0)
    before = probe.intent_state()
    expected_before = _normalize(resolver.state_dict())
    native, expected = _resolve(probe, resolver, 10, advance=False)
    _compare(native, expected)
    assert probe.intent_state() == before
    assert _normalize(resolver.state_dict()) == expected_before
    # The probing call did not consume the tick: advancing step 10 works.
    _resolve(probe, resolver, 10)


def test_resolve_step_validation(plant):
    probe, resolver = plant
    signals = _signals()
    for bad_step in (True, np.int64(3), 3.0, '2'):
        with pytest.raises(Exception):
            probe.intent_resolve(_control(), dict(signals), bad_step, 0.02,
                                 None, None, True, True)
        with pytest.raises(ValueError):
            resolver.resolve(_py_control(_control()), _py_signals(signals),
                             step=bad_step, road_grade=0.02)
    with pytest.raises(Exception):
        probe.intent_resolve(_control(), dict(signals), -1, 0.02, None, None,
                             True, True)
    with pytest.raises(ValueError):
        resolver.resolve(_py_control(_control()), _py_signals(signals),
                         step=-1, road_grade=0.02)


def test_resolve_skipped_acquisition(plant):
    probe, resolver = plant
    _resolve(probe, resolver, 0)
    signals = _signals()
    with pytest.raises(Exception):
        probe.intent_resolve(_control(), dict(signals), 20, 0.02, None, None,
                             True, True)
    with pytest.raises(ValueError):
        resolver.resolve(_py_control(_control()), _py_signals(signals),
                         step=20, road_grade=0.02)


def test_resolve_rewind_requires_reset(plant):
    probe, resolver = plant
    _resolve(probe, resolver, 0)
    _resolve(probe, resolver, 10)
    signals = _signals()
    with pytest.raises(Exception):
        probe.intent_resolve(_control(), dict(signals), 5, 0.02, None, None,
                             True, True)
    with pytest.raises(ValueError):
        resolver.resolve(_py_control(_control()), _py_signals(signals),
                         step=5, road_grade=0.02)
    probe.intent_reset()
    resolver.reset()
    _resolve(probe, resolver, 0)


def test_period_must_be_integer_multiple(tmp_path):
    model = mujoco.MjModel.from_xml_string(_XML)
    path = tmp_path / 'intent.mjb'
    mujoco.mj_saveModel(model, str(path))
    config = _config(period_s=0.0155)
    with pytest.raises(Exception):
        load_native().NativeTestAdapter(
            str(path),
            {'rider_intent': {'config': dict(config), 'dt_s': _DT}})
    with pytest.raises(ValueError):
        RiderIntentResolver(SeatedClimbConfig(**config), _DT)
    # A disabled resolver tolerates the misaligned period.
    config = _config(enabled=False, period_s=0.0155)
    load_native().NativeTestAdapter(
        str(path),
        {'rider_intent': {'config': dict(config), 'dt_s': _DT}})
    RiderIntentResolver(SeatedClimbConfig(**config), _DT)


def test_config_validation(tmp_path):
    model = mujoco.MjModel.from_xml_string(_XML)
    path = tmp_path / 'intent.mjb'
    mujoco.mj_saveModel(model, str(path))
    adapter = load_native().NativeTestAdapter
    for override in ({'enabled': 1}, {'period_s': 0.},
                     {'surge_budget_s': -1.}, {'lean_gain': -0.5},
                     {'front_load_share_target': 1.2},
                     {'max_forward_lean_rad': .9}):
        config = _config(**override)
        with pytest.raises(Exception):
            adapter(str(path),
                    {'rider_intent': {'config': dict(config), 'dt_s': _DT}})
        with pytest.raises(ValueError):
            SeatedClimbConfig(**config)
    for override in ({'enabled': 'yes'}, {'bogus_key': 1.},
                     {'period_s': float('nan')}):
        config = _config(**override)
        with pytest.raises(Exception):
            adapter(str(path),
                    {'rider_intent': {'config': dict(config), 'dt_s': _DT}})


def test_signals_validation(plant):
    probe, resolver = plant
    control = _control()
    bad = [_signals(pitch_rate_up_rad_s=float('nan')),
           _signals(specific_force_body_mps2=[0., 0.]),
           _signals(specific_force_body_mps2=[0., 0., float('inf')]),
           _signals(front_load_share=1.5),
           _signals(front_load_share=float('nan')),
           _signals(crank_rate_rad_s=float('inf'))]
    for signals in bad:
        with pytest.raises(Exception):
            probe.intent_resolve(dict(control), dict(signals), 0, 0.02,
                                 None, None, True, True)
        with pytest.raises(ValueError):
            resolver.resolve(_py_control(control), _py_signals(signals),
                             step=0, road_grade=0.02)


def test_control_validation(plant):
    probe, resolver = plant
    signals = _signals()
    for control in (_control(human_torque_nm=-1.),
                    _control(motor_torque_nm=-.5),
                    _control(rider_enabled=1),
                    _control(posture={'torso_lean_rad': .9,
                                      'pelvis_pitch_rad': 0.,
                                      'pelvis_offset_m': None,
                                      'use_saddle': True})):
        with pytest.raises(Exception):
            probe.intent_resolve(dict(control), dict(signals), 0, 0.02,
                                 None, None, True, True)
        with pytest.raises(ValueError):
            resolver.resolve(_py_control(control), _py_signals(signals),
                             step=0, road_grade=0.02)
    with pytest.raises(Exception):
        probe.intent_resolve({'not': 'a control'}, dict(signals), 0, 0.02,
                             None, None, True, True)
    with pytest.raises(ValueError):
        resolver.resolve({'not': 'a control'}, _py_signals(signals), step=0,
                         road_grade=0.02)


def test_road_grade_validation_is_lazy(plant):
    probe, resolver = plant
    _resolve(probe, resolver, 0)
    # A non-tick step never validates the road window in the oracle.
    native, expected = _resolve(probe, resolver, 1,
                                road_grade=float('nan'),
                                preview_grade=float('inf'),
                                lean_limit_rad=float('nan'))
    _compare(native, expected)
    # On a tick step the same arguments reach policy.update's scalars.
    signals = _signals()
    with pytest.raises(Exception):
        probe.intent_resolve(_control(), dict(signals), 10, float('nan'),
                             None, None, True, True)
    with pytest.raises(ValueError):
        resolver.resolve(_py_control(_control()), _py_signals(signals),
                         step=10, road_grade=float('nan'))


def test_update_validates_before_enabled_check(tmp_path):
    model = mujoco.MjModel.from_xml_string(_XML)
    path = tmp_path / 'intent.mjb'
    mujoco.mj_saveModel(model, str(path))
    config = _config(enabled=False)
    probe = load_native().NativeTestAdapter(
        str(path),
        {'rider_intent': {'config': dict(config), 'dt_s': _DT}})
    resolver = RiderIntentResolver(SeatedClimbConfig(**config), _DT)
    with pytest.raises(Exception):
        probe.intent_update(dict(_signals()), float('nan'), 0.02, None, None)
    with pytest.raises(ValueError):
        resolver.policy.update(_py_signals(_signals()), float('nan'),
                               road_grade=0.02)
    with pytest.raises(Exception):
        probe.intent_update(dict(_signals()), 0.01, float('nan'), None, None)
    with pytest.raises(ValueError):
        resolver.policy.update(_py_signals(_signals()), 0.01,
                               road_grade=float('nan'))


def test_policy_update_parity_with_delays(plant):
    probe, resolver = plant
    policy = resolver.policy
    timeline = [(_signals(pitch_rate_up_rad_s=.4,
                          specific_force_body_mps2=[2., 0., 9.6],
                          crank_rate_rad_s=rate,
                          front_load_share=.3 + .01 * index),
                 0.01, 0.02 + .002 * index, 0.06)
                for index, rate in enumerate((8., 7., 9., 4., 12., 0., 6.))]
    # Long enough for the reaction-delay queue to start delivering.
    for _ in range(18):
        timeline.extend(timeline[:4])
    for signals, dt, grade, preview in timeline:
        native = probe.intent_update(dict(signals), dt, grade, preview, None)
        expected = policy.update(_py_signals(signals), dt,
                                 road_grade=grade, preview_grade=preview)
        _compare(native['posture'], _normalize(asdict(expected.posture)))
        assert native['effort_ceiling_nm'] == pytest.approx(
            expected.effort_ceiling_nm, rel=1e-12, abs=1e-12)
    _compare(probe.intent_state(), resolver.state_dict())


def test_policy_inclination_norm_gate(plant):
    probe, resolver = plant
    policy = resolver.policy
    # Specific force far outside the .8..1.2 g window gates the estimate but
    # not the pitch-rate integration or the wrap.
    tilted = _signals(pitch_rate_up_rad_s=.2,
                      specific_force_body_mps2=[30., 0., 30.])
    for _ in range(40):
        native = probe.intent_update(dict(tilted), 0.01, 0.1, 0.1, None)
        expected = policy.update(_py_signals(tilted), 0.01, road_grade=0.1,
                                 preview_grade=0.1)
        _compare(native['posture'], _normalize(asdict(expected.posture)))
    _compare(probe.intent_state(), resolver.state_dict())
    assert resolver.policy.inclination_rad != 0.


def test_surge_budget_spend_and_recovery(tmp_path):
    model = mujoco.MjModel.from_xml_string(_XML)
    path = tmp_path / 'intent.mjb'
    mujoco.mj_saveModel(model, str(path))
    config = _config(surge_budget_s=.05, surge_recovery_rate=.5)
    probe = load_native().NativeTestAdapter(
        str(path),
        {'rider_intent': {'config': dict(config), 'dt_s': _DT}})
    resolver = RiderIntentResolver(SeatedClimbConfig(**config), _DT)
    policy = resolver.policy
    # Spend: .05s budget at .02s acquisitions drains to .01; the third steep
    # preview finds budget+1e-12 < dt and falls back to target without
    # draining further.
    for preview in (.3, .3, .3, .3):
        native = probe.intent_power_target(preview, .02)
        expected = policy.power_target_w(preview, .02)
        assert native == pytest.approx(expected, rel=1e-12, abs=1e-12)
    # Recovery on gentle ground.
    for _ in range(4):
        native = probe.intent_power_target(0.02, .02)
        expected = policy.power_target_w(0.02, .02)
        assert native == pytest.approx(expected, rel=1e-12, abs=1e-12)
    _compare(probe.intent_state(), resolver.state_dict())


def test_program_update_parity(plant):
    probe, resolver = plant
    program = resolver.policy.program
    cases = [(0., .1, .01, .3, None, None),
             (.005, .25, .005, .35, .3, None),
             (.2, .4, .01, None, None, .05),
             (.21, -.1, .01, .5, .4, None),
             (.5, 0., .02, .31, None, None)]
    for time_s, grade, dt, share, lean, dead in cases:
        native = probe.intent_program_update(time_s, grade, dt, share, lean,
                                             dead)
        expected = program.update(time_s, grade, dt, front_load_share=share,
                                  lean_limit_rad=lean, dead_time_s=dead)
        assert native == pytest.approx(expected, rel=1e-12, abs=1e-12)
    _compare(probe.intent_state(), resolver.state_dict())


def test_program_validation(plant):
    probe, resolver = plant
    program = resolver.policy.program
    for args in ((float('nan'), .1, .01), (0., float('nan'), .01),
                 (0., .1, 0.), (0., .1, -.01)):
        with pytest.raises(Exception):
            probe.intent_program_update(*args, None, None, None)
        with pytest.raises(ValueError):
            program.update(*args)
    with pytest.raises(Exception):
        probe.intent_program_update(0., .1, .01, 1.5, None, None)
    with pytest.raises(ValueError):
        program.update(0., .1, .01, front_load_share=1.5)
    with pytest.raises(Exception):
        probe.intent_program_update(0., .1, .01, None, -0.1, None)
    with pytest.raises(ValueError):
        program.update(0., .1, .01, lean_limit_rad=-0.1)
    with pytest.raises(Exception):
        probe.intent_program_update(0., .1, .01, None, None, -0.2)
    with pytest.raises(ValueError):
        program.update(0., .1, .01, dead_time_s=-0.2)


def test_schedule_pulse_and_reset(plant):
    probe, resolver = plant
    _resolve(probe, resolver, 0)
    probe.intent_schedule_pulse(.015, .02, .25)
    resolver.schedule_pulse(.015, .02, .25)
    for step in range(10, 50, 10):
        native, expected = _resolve(probe, resolver, step)
        _compare(native, expected)
    _compare(probe.intent_state(), resolver.state_dict())
    # The pulse is program state: reset clears it like the oracle.
    probe.intent_reset()
    resolver.reset()
    _compare(probe.intent_state(), resolver.state_dict())
    with pytest.raises(Exception):
        probe.intent_schedule_pulse(float('nan'), .5, .1)
    with pytest.raises(ValueError):
        resolver.schedule_pulse(float('nan'), .5, .1)
    with pytest.raises(Exception):
        probe.intent_schedule_pulse(0., 0., .1)
    with pytest.raises(ValueError):
        resolver.schedule_pulse(0., 0., .1)


def test_state_roundtrip(plant):
    probe, resolver = plant
    resolver.schedule_pulse(.03, .02, .2)
    for step in range(0, 40, 10):
        _resolve(probe, resolver, step,
                 signals=_signals(pitch_rate_up_rad_s=.1 * step,
                                  front_load_share=.25 + .001 * step))
    # Python state -> native restore -> identical emitted state.
    snapshot = _normalize(resolver.state_dict())
    probe.intent_restore(snapshot)
    _compare(probe.intent_state(), resolver.state_dict())
    # Native state -> restore again is a fixed point.
    again = probe.intent_state()
    probe.intent_restore(again)
    assert _normalize(probe.intent_state()) == _normalize(again)
    # Restored resolver keeps its clock: the next tick is step 40.
    native, expected = _resolve(probe, resolver, 40)
    _compare(native, expected)


def test_state_rejects_malformed(plant):
    probe, _ = plant
    state = probe.intent_state()
    for corrupt in ('last_tick_step', 'intent', 'policy', 'program'):
        bad = dict(state)
        del bad[corrupt]
        with pytest.raises(Exception):
            probe.intent_restore(bad)
    bad = dict(state)
    bad['extra'] = 1
    with pytest.raises(Exception):
        probe.intent_restore(bad)
    bad = dict(state)
    bad['last_tick_step'] = 1.5
    with pytest.raises(Exception):
        probe.intent_restore(bad)
    bad = dict(state)
    bad['policy'] = dict(bad['policy'], samples=[[0.]])
    with pytest.raises(Exception):
        probe.intent_restore(bad)


def test_state_unknown_signals_keys_rejected(tmp_path):
    model = mujoco.MjModel.from_xml_string(_XML)
    path = tmp_path / 'intent.mjb'
    mujoco.mj_saveModel(model, str(path))
    probe = load_native().NativeTestAdapter(
        str(path),
        {'rider_intent': {'config': _config(), 'dt_s': _DT}})
    signals = dict(_signals(), bogus=1.)
    with pytest.raises(Exception):
        probe.intent_resolve(_control(), signals, 0, 0.02, None, None, True,
                             True)
