"""Native distributed tyre writer: config, bitwise per-state parity, state wire, probe, stands."""
import re
from pathlib import Path

import mujoco
import numpy as np
import pytest

from native_loader import load_native
bike_native = load_native()

from _bits import assert_bitwise_equal
from test_native_brake_resistance import _restore

ROOT = Path(__file__).resolve().parents[2]
DISTRIBUTED = ROOT / 'examples/research/viewer_physics_distributed.toml'
ROCKY = 'examples/research/alpine_rocky.toml'


def _distributed_config(tmp_path, station_count=None):
    text = DISTRIBUTED.read_text()
    for key in ('joint_envelope_path', 'joint_strength_path'):
        text = re.sub(rf'{key} = "([^"]+)"', lambda m: f'{key} = "{(DISTRIBUTED.parent / m.group(1)).resolve()}"', text)
    if station_count is not None:
        text = re.sub(r'station_count = \d+', f'station_count = {station_count}', text)
    path = tmp_path / 'physics.toml'
    path.write_text(text)
    return path


def _env(tmp_path, duration='1', station_count=None):
    from bike_sim.cli import research as research_cli
    args = research_cli.parser().parse_args([
        '--physics-config', str(_distributed_config(tmp_path, station_count)),
        '--track-file', ROCKY, '--duration', duration, '--dt', '.00125',
        '--diagnostic-model-limits', '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


def _golden(tmp_path, steps=40):
    from tools.golden_episode import capture_episode, save
    from bike_sim.sim.ride.control import RideControl
    env = _env(tmp_path)
    ep = capture_episode(env, steps, RideControl(human_torque_nm=35.))
    save(ep, tmp_path / 'g', env.sim.model)
    return env, tmp_path / 'g'


def test_signed_distance_matches_python_bitwise():
    from bike_sim.terrain.profile_distance import SignedProfile
    rng = np.random.default_rng(3)
    x = np.linspace(0., 12., 2401)
    z = .05 * np.sin(x) + np.where((x > 5.) & (x < 5.3), .04, 0.) + rng.normal(0., .002, len(x))
    vertices = np.column_stack([x, z])
    pts = np.column_stack([rng.uniform(.1, 11.9, 400), rng.uniform(-.1, .5, 400)])
    want = SignedProfile(vertices).query(pts)
    got = bike_native._debug_signed_distance(vertices, pts)
    assert_bitwise_equal(np.asarray(got[0]), want.distance, 'distance')
    assert_bitwise_equal(np.asarray(got[1]), want.normal, 'normal')
    assert_bitwise_equal(np.asarray(got[2]), want.point, 'point')
    assert np.array_equal(np.asarray(got[3]), want.segment)
    assert np.array_equal(np.asarray(got[4]), want.nonsmooth)


def test_config_projection_carries_the_distributed_section(tmp_path):
    from tools.native_config import project, validate_config
    env = _env(tmp_path, duration='.01')
    cfg = project(env)
    tire = cfg['tire']
    assert tire['backend'] == 'distributed_2d_reference'
    assert 'material' not in tire['front'] and set(tire['front']) == {'tangent_k_n_m', 'mu', 'relaxation_length_m'}
    d = tire['distributed']
    assert d['station_count'] == env.sim.physics_config.tires.distributed.station_count
    assert list(d['density']['knots_m']) == [0., .004, .012] and d['front']['calibration_status'] == 'experimental_unvalidated'
    validate_config(cfg)
    pytest.xfail('writer lands in Task 5')
    bike_native.Stepper(str(_model_path(tmp_path, env)), cfg)   # construction accepts the backend


def _model_path(tmp_path, env):
    path = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(env.sim.model, str(path), None)
    return path


def test_config_rejects_bad_distributed_sections(tmp_path):
    from tools.native_config import project
    import copy
    env = _env(tmp_path, duration='.01')
    good = project(env)
    model = str(_model_path(tmp_path, env))
    for mutate in (lambda c: c['tire'].pop('distributed'),
                   lambda c: c['tire']['distributed'].__setitem__('station_count', 8),
                   lambda c: c['tire']['distributed'].__setitem__('station_count', 513),
                   lambda c: c['tire']['distributed']['density'].__setitem__('knots_m', [0., .012, .004]),
                   lambda c: c['tire']['distributed']['density'].__setitem__('stiffness_n_m', [-1., 0., 0.])):
        bad = copy.deepcopy(good)
        mutate(bad)
        with pytest.raises((ValueError, RuntimeError)):
            bike_native.Stepper(model, bad)
