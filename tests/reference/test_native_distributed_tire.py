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


def test_distributed_state_wire_round_trip(tmp_path):
    from tools.native_config import project
    env = _env(tmp_path, duration='.01')
    st = bike_native.Stepper(str(_model_path(tmp_path, env)), project(env))
    names = list(st.tire_state_names)
    N = env.sim.physics_config.tires.distributed.station_count
    assert names == env.sim.physical.tire.state_names() and len(names) == 2 * N * 4
    row = np.full(len(names), np.nan)
    row[names.index('front.s003.xi')] = .002
    row[names.index('front.s003.tangent.0'):names.index('front.s003.tangent.0') + 3] = [1., 0., 0.]
    # rear.s007: finite xi with an all-NaN tangent — the active/no-tangent
    # wire form; inactive stations stay four NaNs.
    row[names.index('rear.s007.xi')] = -.001
    st.set_tire_state(names, row)
    assert_bitwise_equal(np.asarray(st.tire_state()), row, 'state round trip')
    st.set_tire_state(names, st.tire_state())
    assert_bitwise_equal(np.asarray(st.tire_state()), row, 'state re-round trip')
    with pytest.raises(RuntimeError, match='rear.s'):
        st.set_tire_state(names[:-1], row[:-1])
    with pytest.raises(RuntimeError, match='unknown column'):
        st.set_tire_state(names + ['bogus'], np.append(row, 0.))
    # The writer's own energy channels surface through tire_diagnostics —
    # the stored_energy accessor stays exercised on the runtime's
    # stored_terms 'tires' channel (Task 6 observation tests).


def _assert_snapshots_equal(native, row, k):
    """Native tire_snapshots() dict vs the manifest digest — the
    distributed twin of test_native_tire's helper: backend
    'distributed_2d_reference' and one patch per active station."""
    assert set(native) == set(row) == {'front', 'rear'}
    for side in ('front', 'rear'):
        snap, want = native[side], row[side]
        assert snap['interval_id'] == want['interval_id']
        assert snap['backend'] == want['backend'] == 'distributed_2d_reference'
        assert snap['geometric_contact'] == bool(want['geometric_contact'])
        assert_bitwise_equal(snap['time_s'], float(want['time_s']),
                             f'{side}.time_s step {k}')
        assert_bitwise_equal(snap['effective_radius_m'],
                             float(want['effective_radius_m']),
                             f'{side}.effective_radius_m step {k}')
        assert len(snap['patches']) == len(want['patches'])
        for i, (p, w) in enumerate(zip(snap['patches'], want['patches'])):
            assert bool(p['working_surface']) == bool(w['working_surface'])
            for f in ('normal_load_n', 'tangent_force_n', 'slip_mps'):
                assert_bitwise_equal(p[f], float(w[f]),
                                     f'{side}.patches[{i}].{f} step {k}')


def _assert_diagnostics_equal(native, oracle, k):
    """Native tire_diagnostics() dict vs the live applier's own dict —
    the distributed schema (distributed_tire_forces.py:204-213): string
    and flag cells equal, numeric cells bitwise."""
    assert set(native) == set(oracle) == {'front', 'rear'}
    for side in ('front', 'rear'):
        got, want = native[side], oracle[side]
        assert set(got) == set(want)
        for key, w in want.items():
            if isinstance(w, float):
                assert_bitwise_equal(got[key], w,
                                     f'{side}.{key} step {k}')
            else:
                assert got[key] == w, f'{side}.{key} step {k}'


@pytest.mark.slow
def test_distributed_qfrc_bitwise(tmp_path):
    """Golden per-state parity on alpine_rocky: the consumed-state wire
    rows, qfrc vectors, evolved wire, contact snapshots and diagnostics
    — bitwise against the manifest and the live Python applier."""
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g / 'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g / 'model.mjb'), project(env))
    applier = env.sim.physical.tire          # live DistributedTireForceApplier
    tire_i = ep.force_names.index('tires')
    names = list(ep.tire_state_names)
    assert tuple(st.tire_state_names) == tuple(applier.state_names()) == tuple(names)
    dt = float(ref.opt.timestep)
    for k in range(len(ep.state_qpos)):
        _restore(ref, dref, ep, k)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        st.set_tire_state(names, ep.tire_state[k])
        got = st.tire_qfrc(dt)
        assert_bitwise_equal(got, ep.forces[k][tire_i],
                             f'tires golden step {k}')
        # Live Python oracle on the same restored state. set_state resets
        # last_time_s, so a fresh compute at this timestamp is legal.
        applier.set_state(names, ep.tire_state[k])
        want = applier.compute_qfrc(ref, dref, dt)
        assert_bitwise_equal(got, want, f'tires live-oracle step {k}')
        # The evolved per-station wire is the step's second output.
        assert_bitwise_equal(np.asarray(st.tire_state()),
                             applier.state_dict()['row'],
                             f'tire_state evolve step {k}')
        _assert_snapshots_equal(st.tire_snapshots(),
                                ep.manifest['tire_snapshots'][k], k)
        _assert_diagnostics_equal(st.tire_diagnostics(),
                                  applier.diagnostics, k)


def _integration_state(sim):
    expected = np.empty(
        mujoco.mj_stateSize(sim.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(sim.model, sim.data, expected,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    return expected


@pytest.mark.slow
def test_distributed_probe_and_clock(tmp_path, native_module):
    """advance=False probe isolation and the once-per-timestamp clock.

    The Stepper binding exposes no tire-probe entry point (binding.cpp
    publishes tire_qfrc only): the production probe surface is
    NativeRideDriver.probe_step_inputs -> apply_forces(active,
    advance=False) -> TireWriterView::probe_compute_into — the same
    detached-copy kernel the C++ distributed_probe_isolation contract
    drives on the writer directly. The first half gates the clock and
    restore atomicity on the Stepper; the second gates the probe's
    Python-visible consequences: the staged fold equals the oracle's
    apply_forces(advance=False) fold bitwise, no committed boundary or
    published sample moves, and the interval after the probe still lands
    on the oracle's integration vector — only possible while the
    committed station state and clock stay untouched by the detached
    copy.
    """
    from tools.golden_episode import load_episode
    from tools.native_config import project
    from bike_sim.native.runtime import create_native_ride
    from bike_sim.sim.ride.control import RideControl
    env, g = _golden(tmp_path, steps=5)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g / 'model.mjb'))
    st = bike_native.Stepper(str(g / 'model.mjb'), project(env))
    names = list(ep.tire_state_names)
    dt = float(ref.opt.timestep)
    st.set_state(ep.state_qpos[2], ep.state_qvel[2], ep.state_act[2],
                 ep.state_warmstart[2], float(ep.state_time[2]))
    st.forward()
    st.set_tire_state(names, ep.tire_state[2])
    # One advancing compute per timestamp (the std::invalid_argument the
    # compliant clock test pins as ValueError).
    st.tire_qfrc(dt)
    with pytest.raises(ValueError, match='increasing timestamp'):
        st.tire_qfrc(dt)
    committed = np.asarray(st.tire_state())
    # A rejected restore is atomic: staged decode never commits, so the
    # state wire AND the committed clock are preserved.
    bad = committed.copy()
    bad[names.index('front.s000.xi')] = np.inf
    with pytest.raises(RuntimeError, match='finite or NaN'):
        st.set_tire_state(names, bad)
    assert_bitwise_equal(np.asarray(st.tire_state()), committed,
                         'rejected set_state preserves the state wire')
    with pytest.raises(ValueError, match='increasing timestamp'):
        st.tire_qfrc(dt)
    st.set_tire_state(names, committed)      # a valid restore re-arms it
    st.tire_qfrc(dt)
    with pytest.raises(ValueError, match='increasing timestamp'):
        st.tire_qfrc(dt)

    # The runtime probe entry point: probe_step_inputs -> apply_forces
    # (active, advance=False) -> the writer's detached probe_compute_into.
    probe_dir = tmp_path / 'probe'
    probe_dir.mkdir()
    probe_env = _env(probe_dir)
    native = create_native_ride(probe_env.sim, strict=False,
                                record_decimation=1)
    command = RideControl()
    try:
        period = probe_env.sim.physical.control_clock.steps_per_period
        for target in range(1, period + 1):
            probe_env.sim.step(control=command)
            native.advance(target, command)
            assert_bitwise_equal(native.snapshot().integration_state,
                                 _integration_state(probe_env.sim),
                                 f'interval {target}')
        # The probe replays the staged pre-solve fold on the committed
        # state; the oracle's apply_forces(advance=False) is the same fold.
        probe_env.sim.physical.apply_forces(
            active=True, advance=False, front=0., rear=0., control=command)
        expected = probe_env.sim.force_accumulator.components
        before = native.snapshot()
        probe = native.probe_step_inputs(command)
        assert list(probe['components']) == list(expected)
        for name, force in expected.items():
            assert_bitwise_equal(np.asarray(probe['components'][name]),
                                 np.asarray(force), f'probe {name}')
        # Probe output is published through probe_snapshots only — the
        # resistance rows in the fold consume it — while the committed
        # boundary and the published sample do not move.
        after = native.snapshot()
        assert after.step == before.step == period
        assert before.latest_sample is not None
        assert (after.latest_sample.interval_id
                == before.latest_sample.interval_id)
        probe_env.sim.step(control=command)
        native.advance(period + 1, command)
        assert_bitwise_equal(native.snapshot().integration_state,
                             _integration_state(probe_env.sim),
                             'post-probe interval')
    finally:
        native.close()
        probe_env.close()


def test_distributed_observations_declare_multi_contact(tmp_path, native_module):
    """The committed tire channel declares the multi-contact capability:
    the wire channel_violations (status.cpp) reads before trusting a
    multi_support flag. The observation accessor resolved from
    test_native_research_environment.py is snapshot().latest_sample —
    the latest published interval's channels."""
    from bike_sim.native.research import create_native_research
    from bike_sim.sim.ride.control import RideControl
    env = _env(tmp_path, duration='.05')
    native = create_native_research(env)
    try:
        native.step(RideControl())
        tires = native.snapshot().latest_sample.channels['tires']
        for side in ('front', 'rear'):
            assert tires[side]['supports_multiple_contacts'] is True
            assert tires[side]['backend'] == 'distributed_2d_reference'
    finally:
        native.close()
        env.close()
