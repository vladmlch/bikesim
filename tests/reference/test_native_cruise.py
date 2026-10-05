"""Sequential scalar PI oracle, state replay, and atomic validation."""
import sys
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from bike_sim.sim.ride.cruise import CruiseController

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'native' / 'build'))
from _bits import assert_bitwise_equal

CONFIG = dict(target_speed_kmh=25., kp_nm_per_mps=180.,
              ki_nm_per_mps_s=150., torque_ceiling_nm=150.)
NUMERIC = ('target_speed_mps', 'integral_mps_s', 'torque_nm', 'gain_scale')


def _model(tmp_path, name='root_x', kind='slide'):
    model = mujoco.MjModel.from_xml_string(f'''
        <mujoco><option timestep="0.0005"/><worldbody><body>
        <joint name="{name}" type="{kind}"/>
        <geom type="sphere" size=".1" mass="1"/>
        </body></worldbody></mujoco>''')
    path = tmp_path / 'cruise.mjb'
    mujoco.mj_saveModel(model, str(path))
    return model, path


@pytest.fixture
def pair(tmp_path):
    native = pytest.importorskip('bike_native')
    model, path = _model(tmp_path)
    return (model, mujoco.MjData(model), CruiseController(model, **CONFIG),
            native.Stepper(str(path), {'schema': 1, 'cruise': CONFIG}))


def _state(oracle):
    return dict(target_speed_mps=oracle.target_speed_mps,
                integral_mps_s=oracle.integral_mps_s, torque_nm=oracle.torque_nm,
                engaged=oracle.engaged, gain_scale=oracle.gain_scale)


def _assert_state(actual, expected):
    assert set(actual) == set(expected) == {*NUMERIC, 'engaged'}
    assert type(actual['engaged']) is bool
    assert actual['engaged'] == expected['engaged']
    assert_bitwise_equal(np.array([actual[k] for k in NUMERIC]),
                         np.array([expected[k] for k in NUMERIC]))


def _run(pair, cases):
    model, data, oracle, native = pair
    outputs = []
    for speed, grounded, limited, override in cases:
        data.qvel[oracle.root_x_dofadr] = speed
        native.set_state(data.qpos, data.qvel, data.act,
                         data.qacc_warmstart, data.time)
        expected = oracle.compute(model, data, SimpleNamespace(rear_in_contact=grounded),
                                  limited, controller_grounded=override)
        expected_state = _state(oracle)
        actual = native.cruise_compute(grounded, limited, override)
        assert_bitwise_equal(actual, expected)
        _assert_state(native.cruise_state(), expected_state)
        outputs.append(actual)
    return np.array(outputs)


def test_sequential_bitwise_oracle(pair):
    t = pair[2].target_speed_mps
    cases = [(s, True, False, None) for s in
             (-0., 0., 1., t - .4, t - .1, t, t + .1, t + .4, 12., 20.)]
    cases += [(t - .2, False, False, None), (t + .2, False, True, True),
              (t - .2, True, False, False), (t + .2, True, True, None),
              (t - .2, True, False, None)]
    rng = np.random.default_rng(728)
    cases += [(float(rng.uniform(3., 12.)), bool(rng.integers(2)),
               bool(rng.integers(2)), (None, False, True)[int(rng.integers(3))])
              for _ in range(500)]
    _run(pair, cases)


@pytest.mark.parametrize('integral', [-100., -1., -0., 0., 1., 100.])
def test_restored_integral_unwinds_and_exact_saturation(pair, integral):
    _, _, oracle, native = pair
    oracle.integral_mps_s = integral
    native.set_cruise_state(_state(oracle))
    t = oracle.target_speed_mps
    _run(pair, [(t, True, False, None), (t - .1, True, False, None),
                (t + .1, True, False, None), (t, False, False, None)])


def test_signed_zero_snapshot_and_compute(pair):
    _, _, oracle, native = pair
    oracle.integral_mps_s = -0.
    oracle.torque_nm = -0.
    native.set_cruise_state(_state(oracle))
    _assert_state(native.cruise_state(), _state(oracle))
    _run(pair, [(oracle.target_speed_mps, True, True, None),
                (-0., False, False, None)])


def test_target_compensation_reset_and_defaults(pair):
    _, _, oracle, native = pair
    for target in (15., 25., 45.):
        oracle.target_speed_kmh = target
        native.cruise_set_target_speed(target)
        for support in (-12., -0., 0., .3, 2., 5., 1e300, np.finfo(float).max):
            expected = oracle.set_assist_compensation(support)
            assert_bitwise_equal(native.cruise_set_assist_compensation(support), expected)
            _run(pair, [(oracle.target_speed_mps - .2, True, False, None)])
    oracle.reset()
    native.cruise_reset()
    _assert_state(native.cruise_state(), _state(oracle))
    model, data, oracle, native = pair
    expected = oracle.compute(model, data, SimpleNamespace(rear_in_contact=True))
    assert_bitwise_equal(native.cruise_compute(rear_in_contact=True), expected)


def test_snapshot_replay_and_mjdata_restore_is_independent(pair):
    _, data, oracle, native = pair
    t = oracle.target_speed_mps
    _run(pair, [(t - .2, True, False, None)] * 10)
    snapshot = native.cruise_state()
    expected_snapshot = _state(oracle)
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 12.)
    _assert_state(native.cruise_state(), expected_snapshot)
    cases = [(t + .3, True, False, None), (t, False, False, True)] * 20
    first = _run(pair, cases)
    oracle._target_speed_mps = expected_snapshot['target_speed_mps']
    for key in ('integral_mps_s', 'torque_nm', 'engaged', 'gain_scale'):
        setattr(oracle, key, expected_snapshot[key])
    native.set_cruise_state(snapshot)
    assert_bitwise_equal(_run(pair, cases), first)
    snapshot['integral_mps_s'] = 999.
    assert native.cruise_state()['integral_mps_s'] != 999.


@pytest.mark.parametrize('config', [None, {}, {'schema': 1}])
def test_every_api_requires_config(tmp_path, config):
    bike_native = pytest.importorskip('bike_native')
    _, path = _model(tmp_path)
    native = (bike_native.Stepper(str(path)) if config is None else
              bike_native.Stepper(str(path), config))
    for name, args in [('cruise_compute', (True,)), ('cruise_reset', ()),
                       ('cruise_set_target_speed', (25.,)),
                       ('cruise_set_assist_compensation', (0.,)),
                       ('cruise_state', ()), ('set_cruise_state', ({},))]:
        with pytest.raises(RuntimeError, match='cruise'):
            getattr(native, name)(*args)


@pytest.mark.parametrize('key', CONFIG)
def test_missing_config_keys(tmp_path, key):
    bike_native = pytest.importorskip('bike_native')
    _, path = _model(tmp_path)
    config = {k: v for k, v in CONFIG.items() if k != key}
    with pytest.raises(ValueError, match=key):
        bike_native.Stepper(str(path), {'schema': 1, 'cruise': config})


@pytest.mark.parametrize('config', [{'cruise': CONFIG}, {'schema': 2, 'cruise': CONFIG},
                                   {'schema': 1, 'cruise': []}])
def test_schema_and_section_validation(tmp_path, config):
    bike_native = pytest.importorskip('bike_native')
    _, path = _model(tmp_path)
    with pytest.raises(ValueError):
        bike_native.Stepper(str(path), config)


@pytest.mark.parametrize('key', CONFIG)
@pytest.mark.parametrize('value', [0., -1., np.nan, np.inf, -np.inf])
def test_invalid_config(tmp_path, key, value):
    bike_native = pytest.importorskip('bike_native')
    _, path = _model(tmp_path)
    with pytest.raises(ValueError, match=key):
        bike_native.Stepper(str(path), {'schema': 1, 'cruise': {**CONFIG, key: value}})


@pytest.mark.parametrize('target', [14.999, 45.001])
def test_config_target_band(tmp_path, target):
    bike_native = pytest.importorskip('bike_native')
    _, path = _model(tmp_path)
    with pytest.raises(ValueError):
        bike_native.Stepper(str(path), {'schema': 1, 'cruise':
                                      {**CONFIG, 'target_speed_kmh': target}})


@pytest.mark.parametrize('name,kind', [('other', 'slide'), ('root_x', 'ball'),
                                      ('root_x', 'free')])
def test_root_joint_validation(tmp_path, name, kind):
    bike_native = pytest.importorskip('bike_native')
    _, path = _model(tmp_path, name, kind)
    with pytest.raises(ValueError, match='root_x'):
        bike_native.Stepper(str(path), {'schema': 1, 'cruise': CONFIG})


def test_hinge_root_is_supported(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    model, path = _model(tmp_path, kind='hinge')
    pair = (model, mujoco.MjData(model), CruiseController(model, **CONFIG),
            bike_native.Stepper(str(path), {'schema': 1, 'cruise': CONFIG}))
    _run(pair, [(6.8, True, False, None)])


@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize('method', ['cruise_set_target_speed',
                                   'cruise_set_assist_compensation'])
def test_nonfinite_setters_preserve_state(pair, method, value):
    native = pair[3]
    old = native.cruise_state()
    with pytest.raises(ValueError):
        getattr(native, method)(value)
    _assert_state(native.cruise_state(), old)


@pytest.mark.parametrize('value', [14.999, 45.001])
def test_target_setter_band_preserves_state(pair, value):
    native = pair[3]
    old = native.cruise_state()
    with pytest.raises(ValueError):
        native.cruise_set_target_speed(value)
    _assert_state(native.cruise_state(), old)


@pytest.mark.parametrize('key', NUMERIC)
@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf])
def test_nonfinite_restore_preserves_state(pair, key, value):
    native = pair[3]
    old = native.cruise_state()
    with pytest.raises(ValueError, match=key):
        native.set_cruise_state({**old, key: value})
    _assert_state(native.cruise_state(), old)


@pytest.mark.parametrize('key,value', [('target_speed_mps', 15. / 3.6 - .001),
                                      ('target_speed_mps', 45. / 3.6 + .001),
                                      ('gain_scale', 0.), ('gain_scale', -1.),
                                      ('gain_scale', np.finfo(float).max),
                                      ('engaged', 1), ('engaged', 'yes')])
def test_invalid_restore_preserves_state(pair, key, value):
    native = pair[3]
    old = native.cruise_state()
    with pytest.raises(ValueError):
        native.set_cruise_state({**old, key: value})
    _assert_state(native.cruise_state(), old)


@pytest.mark.parametrize('key', [*NUMERIC, 'engaged'])
def test_missing_state_field_preserves_state(pair, key):
    native = pair[3]
    old = native.cruise_state()
    with pytest.raises(ValueError, match=key):
        native.set_cruise_state({k: v for k, v in old.items() if k != key})
    _assert_state(native.cruise_state(), old)


def test_extra_state_field_rejected(pair):
    native = pair[3]
    old = native.cruise_state()
    with pytest.raises(ValueError):
        native.set_cruise_state({**old, 'extra': 1.})
    _assert_state(native.cruise_state(), old)


@pytest.mark.parametrize('sign', [-1., 1.])
def test_exact_saturation_freezes_nonzero_error(pair, sign):
    _, _, oracle, native = pair
    oracle.integral_mps_s = sign * .4
    native.set_cruise_state(_state(oracle))
    speed = oracle.target_speed_mps - sign * .5
    _run(pair, [(speed, True, False, None)])
    assert oracle.integral_mps_s == sign * .4
    assert oracle.torque_nm == sign * 150.


def test_negative_zero_output_bitwise(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    model, path = _model(tmp_path)
    config = {**CONFIG, 'kp_nm_per_mps': 1e-300, 'ki_nm_per_mps_s': 1e-300}
    oracle = CruiseController(model, **config)
    native = bike_native.Stepper(str(path), {'schema': 1, 'cruise': config})
    oracle.gain_scale = 1e-20
    oracle.integral_mps_s = -0.
    native.set_cruise_state(_state(oracle))
    pair = model, mujoco.MjData(model), oracle, native
    output = _run(pair, [(oracle.target_speed_mps + 1e-6, True, True, None)])
    assert_bitwise_equal(output, np.array([-0.]))


@pytest.mark.parametrize('target', [15. / 3.6, 45. / 3.6,
                                   np.nextafter(6., 7.)])
def test_restored_target_keeps_mps_bits(pair, target):
    _, _, oracle, native = pair
    oracle._target_speed_mps = target
    native.set_cruise_state(_state(oracle))
    _assert_state(native.cruise_state(), _state(oracle))
    _run(pair, [(target - .2, True, False, None)])


@pytest.mark.parametrize('key', NUMERIC)
@pytest.mark.parametrize('value', ['wrong', None, []])
def test_nonnumeric_restore_preserves_state(pair, key, value):
    native = pair[3]
    old = native.cruise_state()
    with pytest.raises(ValueError, match=key):
        native.set_cruise_state({**old, key: value})
    _assert_state(native.cruise_state(), old)


def test_support_underflow_preserves_state(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    _, path = _model(tmp_path)
    native = bike_native.Stepper(str(path), {'schema': 1, 'cruise':
                                {**CONFIG, 'ki_nm_per_mps_s': 1e-300}})
    old = native.cruise_state()
    with pytest.raises(ValueError):
        native.cruise_set_assist_compensation(np.finfo(float).max)
    _assert_state(native.cruise_state(), old)
    with pytest.raises(ValueError):
        native.set_cruise_state({**old, 'gain_scale': 1e-100})
    _assert_state(native.cruise_state(), old)


def test_projection_optional_cruise(tmp_path):
    from bike_sim.sim.ride_sim import RideSimulation
    from tools.native_config import project
    bike_native = pytest.importorskip('bike_native')
    sim = RideSimulation()
    sim.cruise.target_speed_kmh = 45.
    sim.cruise.integral_mps_s = 123.
    config = project(SimpleNamespace(sim=sim))
    assert config['cruise'] == {**CONFIG, 'target_speed_kmh': 45.}
    path = tmp_path / 'project.mjb'
    mujoco.mj_saveModel(sim.model, str(path))
    native = bike_native.Stepper(str(path), config)
    assert native.cruise_state()['integral_mps_s'] == 0.
    del sim.cruise
    assert 'cruise' not in project(sim)
    bike_native.Stepper(str(path), project(sim))
    sim.cruise = None
    assert 'cruise' not in project(sim)
