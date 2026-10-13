"""Native pneumatic tyre writer: config, bitwise per-state parity, state wire, probe, energy."""
import copy
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
# The distributed research profile is the nearest resolved native-eligible
# physics config; swapping its backend to 'pneumatic' gives the pneumatic
# writer the same production-shaped env the distributed tests exercise.
PNEUMATIC_BASE = ROOT / 'examples/research/viewer_physics_distributed.toml'
ROCKY = 'examples/research/alpine_rocky.toml'


def _pneumatic_config(tmp_path, tier='fast'):
    """Derive a pneumatic physics TOML: backend swap, drop the distributed
    section, declare the tier under [tires.pneumatic] (resolution.py maps it
    onto PneumaticTireConfig; front/rear specs extend the defaults)."""
    text = PNEUMATIC_BASE.read_text()
    for key in ('joint_envelope_path', 'joint_strength_path'):
        text = re.sub(
            rf'{key} = "([^"]+)"',
            lambda m: f'{key} = "{(PNEUMATIC_BASE.parent / m.group(1)).resolve()}"',
            text)
    text = text.replace('backend = "distributed_2d_reference"',
                        'backend = "pneumatic"')
    text = re.sub(r'\[tires\.distributed\][^\[]*', '', text)
    text += f'\n[tires.pneumatic]\ntier = "{tier}"\n'
    path = tmp_path / 'physics.toml'
    path.write_text(text)
    return path


def _env(tmp_path, duration='1', tier='fast'):
    from bike_sim.cli import research as research_cli
    # The detailed tier's discretised brush is written for its declared
    # step — SimulationPhysicsConfig gates timestep_s to the tier's own
    # 0.00025, so the physics dt follows the tier (the control period
    # stays an integer multiple either way).
    dt = '.00025' if tier == 'detailed' else '.00125'
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pneumatic_config(tmp_path, tier)),
        '--track-file', ROCKY, '--duration', duration, '--dt', dt,
        '--diagnostic-model-limits', '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


def _model_path(tmp_path, env):
    path = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(env.sim.model, str(path), None)
    return path


def _golden(tmp_path, steps=40, tier='fast'):
    from tools.golden_episode import capture_episode, save
    from bike_sim.sim.ride.control import RideControl
    env = _env(tmp_path, tier=tier)
    ep = capture_episode(env, steps, RideControl(human_torque_nm=35.))
    save(ep, tmp_path / 'g', env.sim.model)
    return env, tmp_path / 'g'


def _integration_state(sim):
    expected = np.empty(
        mujoco.mj_stateSize(sim.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(sim.model, sim.data, expected,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    return expected


def test_config_projection_carries_the_pneumatic_section(tmp_path):
    from tools.native_config import project
    env = _env(tmp_path, duration='.01')
    cfg = project(env)
    tire = cfg['tire']
    assert tire['backend'] == 'pneumatic'
    assert 'material' not in tire['front'] and set(tire['front']) == {
        'tangent_k_n_m', 'mu', 'relaxation_length_m'}
    p = tire['pneumatic']
    from bike_sim.physics.tyre import TIERS
    tier = TIERS['fast']
    assert p['tier'] == 'fast'
    assert p['n_rays'] == tier.n_rays
    assert p['half_angle_deg'] == tier.half_angle_deg
    assert p['discretised_brush'] == tier.discretised_brush
    assert p['timestep_s'] == tier.timestep_s
    assert 'surface' not in p                    # track resolution, not a name
    assert p['profile']['dx_m'] > 0. and len(p['profile']['z_m']) >= 2
    assert 'surface_map' in tire                 # the track map rode the wire
    bike_native.Stepper(str(_model_path(tmp_path, env)), cfg)


def test_config_rejects_bad_pneumatic_sections(tmp_path):
    from tools.native_config import project
    env = _env(tmp_path, duration='.01')
    good = project(env)
    model = str(_model_path(tmp_path, env))
    mutants = (
        lambda c: c['tire'].pop('pneumatic'),
        lambda c: c['tire']['pneumatic'].__setitem__('tier', 'detailed'),
        lambda c: c['tire']['pneumatic'].__setitem__('n_rays', 33),
        lambda c: c['tire']['pneumatic'].__setitem__('half_angle_deg', 74.),
        lambda c: c['tire']['pneumatic'].__setitem__('discretised_brush', True),
        lambda c: c['tire']['pneumatic'].__setitem__('timestep_s', .001),
        lambda c: c['tire']['pneumatic'].pop('profile'),
        lambda c: c['tire']['pneumatic']['profile'].__setitem__('dx_m', 0.),
        lambda c: c['tire']['pneumatic']['profile'].__setitem__('z_m', [.0]),
        lambda c: c['tire'].pop('surface_map'),
    )
    for mutate in mutants:
        bad = copy.deepcopy(good)
        mutate(bad)
        with pytest.raises((ValueError, RuntimeError)):
            bike_native.Stepper(model, bad)


def test_pneumatic_state_wire_schema_and_round_trip(tmp_path):
    from tools.native_config import project
    from bike_sim.physics.tyre import TIERS
    env = _env(tmp_path, duration='.5')
    for _ in range(40):
        env.sim.step()
    st = bike_native.Stepper(str(_model_path(tmp_path, env)), project(env))
    applier = env.sim.physical.tire
    names = list(st.tire_state_names)
    n = TIERS['fast'].n_rays
    width = (n + 1) // 2
    assert names == applier.state_names()
    assert len(names) == 2 * (4 * n + 8 + 3 * width)
    assert names[:4] == ['front.ray.000.delta_prev_m',
                         'front.ray.000.elastic_prev_n',
                         'front.ray.000.maxwell_n',
                         'front.ray.000.bristle_m']
    assert names[4 * n] == 'front.carcass.initialized'
    assert names[4 * n + 1] == 'front.rim.active'
    assert names[4 * n + 7] == 'front.patch.count'
    assert names[4 * n + 8] == 'front.patch.00.start'
    assert names[-1] == f'rear.patch.{width - 1:02d}.kappa_prime'
    # The committed 40-step state round-trips bitwise through the writer.
    row = applier.state_dict()['row']
    st.set_tire_state(names, row)
    assert_bitwise_equal(np.asarray(st.tire_state()), row, 'state round trip')
    st.set_tire_state(names, st.tire_state())
    assert_bitwise_equal(np.asarray(st.tire_state()), row,
                         'state re-round trip')
    # The lumped fast tier leaves every bristle cell NaN.
    wire = np.asarray(st.tire_state())
    bristle = np.array([wire[names.index(f'front.ray.{i:03d}.bristle_m')]
                        for i in range(n)])
    assert np.isnan(bristle).all()


def test_pneumatic_state_rejections_are_atomic(tmp_path):
    from tools.native_config import project
    env = _env(tmp_path, duration='.5')
    for _ in range(20):
        env.sim.step()
    st = bike_native.Stepper(str(_model_path(tmp_path, env)), project(env))
    names = list(st.tire_state_names)
    committed = np.asarray(st.tire_state())
    with pytest.raises(RuntimeError, match='rear.patch'):
        st.set_tire_state(names[:-1], committed[:-1])
    with pytest.raises(RuntimeError, match='unknown column'):
        st.set_tire_state(names + ['bogus'],
                          np.append(committed, 0.))
    bad = committed.copy()
    bad[names.index('front.ray.000.delta_prev_m')] = np.inf
    with pytest.raises(RuntimeError):
        st.set_tire_state(names, bad)
    # Disjointness is wire contract: an overlapping patch pair rejects
    # before any commit — and nothing visible moves.
    idx = {leaf: names.index(f'front.patch.00.{leaf}')
           for leaf in ('start', 'stop', 'kappa_prime')}
    count_i = names.index('front.patch.count')
    overlap = committed.copy()
    overlap[count_i] = 2.
    overlap[idx['start']], overlap[idx['stop']] = 0., 5.
    overlap[names.index('front.patch.01.start')] = 4.
    overlap[names.index('front.patch.01.stop')] = 8.
    overlap[idx['kappa_prime']] = .1
    overlap[names.index('front.patch.01.kappa_prime')] = .2
    with pytest.raises(RuntimeError):
        st.set_tire_state(names, overlap)
    assert_bitwise_equal(np.asarray(st.tire_state()), committed,
                         'rejected restores preserve the state wire')


def _assert_snapshots_equal(native, row, k):
    """Native tire_snapshots() dict vs the manifest digest — the pneumatic
    twin of test_native_distributed_tire's helper."""
    assert set(native) == set(row) == {'front', 'rear'}
    for side in ('front', 'rear'):
        snap, want = native[side], row[side]
        assert snap['interval_id'] == want['interval_id']
        assert snap['backend'] == want['backend'] == 'pneumatic'
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
    the pneumatic schema (pneumatic_tire_forces.py:426-455): string and
    flag cells equal, numeric cells bitwise."""
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


def _run_golden_parity(env, g):
    """The shared per-state parity loop over a captured golden episode:
    qfrc, evolved wire, snapshots and diagnostics — bitwise vs the
    manifest and the live Python applier."""
    from tools.golden_episode import load_episode
    from tools.native_config import project
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g / 'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g / 'model.mjb'), project(env))
    applier = env.sim.physical.tire     # live PneumaticTireForceApplier
    tire_i = ep.force_names.index('tires')
    names = list(ep.tire_state_names)
    assert tuple(st.tire_state_names) == tuple(applier.state_names()) \
        == tuple(names)
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
        # The evolved per-ray/patch wire is the step's second output.
        assert_bitwise_equal(np.asarray(st.tire_state()),
                             applier.state_dict()['row'],
                             f'tire_state evolve step {k}')
        _assert_snapshots_equal(st.tire_snapshots(),
                                ep.manifest['tire_snapshots'][k], k)
        _assert_diagnostics_equal(st.tire_diagnostics(),
                                  applier.diagnostics, k)


@pytest.mark.slow
def test_pneumatic_qfrc_bitwise(tmp_path):
    """Golden per-state parity on alpine_rocky: the consumed-state wire
    rows, qfrc vectors, evolved wire, contact snapshots and diagnostics —
    bitwise against the manifest and the live Python applier."""
    env, g = _golden(tmp_path, steps=40, tier='fast')
    _run_golden_parity(env, g)


def _wheel_rig_xml(tier, front_drop_m=.004, rear_drop_m=.003):
    """Two named wheels on a flat road — the oracle fixture's shape
    (test_pneumatic_tire_forces._wheel_rig): every geom collision-free,
    sphere radii matching the tyre specs. Returns (xml, radii)."""
    from bike_sim.physics.tyre import FRONT_TYRE, REAR_TYRE, TIERS
    front_r = FRONT_TYRE.outer_radius_mm / 1000.0
    rear_r = REAR_TYRE.outer_radius_mm / 1000.0
    return (f'''<mujoco><compiler angle="radian"/>
      <option timestep="{TIERS[tier].timestep_s}"
              gravity="0 0 -9.81" integrator="Euler"/>
      <worldbody>
        <body name="front_wheel" pos="0 0 {front_r - front_drop_m}">
          <joint name="front_root_x" type="slide" axis="1 0 0"/>
          <joint name="front_root_z" type="slide" axis="0 0 1"/>
          <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="6" pos="0 0 0" diaginertia=".3 .5 .3"/>
          <geom name="geom_front_contact" type="sphere" size="{front_r}"
                contype="0" conaffinity="0" mass="0"/>
        </body>
        <body name="rear_wheel" pos="0.8 0 {rear_r - rear_drop_m}">
          <joint name="rear_root_x" type="slide" axis="1 0 0"/>
          <joint name="rear_root_z" type="slide" axis="0 0 1"/>
          <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="6" pos="0 0 0" diaginertia=".3 .5 .3"/>
          <geom name="geom_rear_contact" type="sphere" size="{rear_r}"
                contype="0" conaffinity="0" mass="0"/>
        </body>
      </worldbody></mujoco>''', front_r, rear_r)


def _pneumatic_wire(cfg_tires, profile):
    """The tire section through the production emitter (the same dict
    Stepper receives from project()) under a tire-only top-level dict —
    every other writer section is optional in config.hpp."""
    from bike_sim.native.config import _tire_section
    return {'schema': 2,
            'tire': _tire_section(cfg_tires, None, profile)}


def _assert_live_snapshots(native, oracle, k):
    """Native tire_snapshots() vs the live applier's WheelContactSnapshot
    objects — the small-rig path has no manifest digest."""
    assert set(native) == set(oracle)
    for side, snap in native.items():
        want = oracle[side]
        assert snap['interval_id'] == want.interval_id
        assert snap['backend'] == want.backend == 'pneumatic'
        assert snap['geometric_contact'] == want.geometric_contact
        assert_bitwise_equal(snap['time_s'], want.time_s,
                             f'{side}.time_s tick {k}')
        assert_bitwise_equal(snap['effective_radius_m'],
                             want.effective_radius_m,
                             f'{side}.effective_radius_m tick {k}')
        assert len(snap['patches']) == len(want.patches)
        for i, (p, w) in enumerate(zip(snap['patches'], want.patches)):
            assert bool(p['working_surface']) == bool(w.working_surface)
            for f in ('normal_load_n', 'tangent_force_n', 'slip_mps'):
                assert_bitwise_equal(p[f], getattr(w, f),
                                     f'{side}.patches[{i}].{f} tick {k}')


def _joint_adr(model, name, field):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return getattr(model, f'jnt_{field}adr')[jid]


@pytest.mark.slow
def test_pneumatic_qfrc_bitwise_detailed_tier(tmp_path):
    """The detailed tier's discretised brush on the small two-wheel rig:
    bristle cells ride the wire, the shear fold engages, and qfrc, the
    evolved wire, snapshots, diagnostics and the energy surface stay
    bitwise-equal to the live applier across a sinking/spinning sweep.

    (The research-env gate requires timestep_s = tier's 0.00025, where
    the static equilibrium solver does not converge within its 80000-step
    budget — so the detailed tier is covered on this rig instead, which
    exercises the identical writer path.)"""
    from bike_sim.physics.physical_config import (
        PneumaticTireConfig, TireBackendConfig)
    from bike_sim.sim.ride.pneumatic_tire_forces import (
        PneumaticTireForceApplier)
    from bike_sim.sim.ride.tyre.geometry import RoadProfile
    tier = 'detailed'
    xml, front_r, rear_r = _wheel_rig_xml(tier)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    x = np.linspace(-2.0, 3.0, 1001)
    profile = RoadProfile.from_samples(x, np.zeros_like(x))
    cfg_tires = TireBackendConfig(
        backend='pneumatic', surface_mode='configured',
        pneumatic=PneumaticTireConfig(tier=tier, surface='asphalt'))
    applier = PneumaticTireForceApplier(model, profile, cfg_tires)
    mjb = tmp_path / 'rig.mjb'
    mujoco.mj_saveModel(model, str(mjb), None)
    st = bike_native.Stepper(str(mjb), _pneumatic_wire(cfg_tires, profile))
    names = list(applier.state_names())
    n = 256
    assert tuple(st.tire_state_names) == tuple(names)
    assert len(names) == 2 * (4 * n + 8 + 3 * ((n + 1) // 2))
    front_z = _joint_adr(model, 'front_root_z', 'qpos')
    rear_z = _joint_adr(model, 'rear_root_z', 'qpos')
    rear_spin = _joint_adr(model, 'rear_wheel_spin', 'dof')
    front_spin = _joint_adr(model, 'front_wheel_spin', 'dof')
    dt = float(model.opt.timestep)
    for k in range(12):
        # Slide-joint qpos is a DELTA on the XML pos: the rig already
        # sits .004/.003 m into the road, so these deepen from there.
        # k=0 keeps the rear wheel airborne: the zero-load diagnostics
        # path (fallback mu, empty patches, airborne flag) — then it
        # descends into and through contact.
        data.qpos[rear_z] = 5.0 if k == 0 else -.004 - (k - 1) * .001
        data.qpos[front_z] = -k * .001
        data.qvel[rear_spin] = 2. + k
        data.qvel[front_spin] = 1. + .5 * k
        data.time = k * dt
        mujoco.mj_forward(model, data)
        row = applier.state_dict()['row']             # the consumed seed
        st.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                     np.asarray(data.act),
                     np.asarray(data.qacc_warmstart), float(data.time))
        st.forward()
        st.set_tire_state(names, row)
        got = np.asarray(st.tire_qfrc(dt))
        want = applier.compute_qfrc(model, data, dt)
        assert_bitwise_equal(got, want, f'detailed qfrc tick {k}')
        assert_bitwise_equal(np.asarray(st.tire_state()),
                             applier.state_dict()['row'],
                             f'detailed evolved wire {k}')
        _assert_live_snapshots(st.tire_snapshots(),
                               applier.snapshots, k)
        _assert_diagnostics_equal(st.tire_diagnostics(),
                                  applier.diagnostics, k)
        assert_bitwise_equal(st.tire_stored_energy(),
                             applier.stored_energy(model, data),
                             f'detailed stored_energy tick {k}')
        assert_bitwise_equal(st.tire_brush_loss_step_j(),
                             applier.brush_loss_step_j,
                             f'detailed brush_loss tick {k}')
        assert_bitwise_equal(st.tire_radial_dissipation_power_w(),
                             applier.radial_dissipation_power_w,
                             f'detailed radial_power tick {k}')
    # The bristle wire engaged on this tier: finite inside committed
    # patches, NaN outside — exactly what the applier's state_dict wrote.
    wire = applier.state_dict()['row']
    bristle = np.array([wire[names.index(f'front.ray.{i:03d}.bristle_m')]
                        for i in range(n)])
    assert np.isfinite(bristle).any() and not np.isfinite(bristle).all()


@pytest.mark.slow
def test_pneumatic_probe_and_clock(tmp_path, native_module):
    """advance=False probe isolation and the once-per-timestamp clock —
    the pneumatic twin of the distributed test: the Stepper half gates
    the clock and restore atomicity; the create_native_ride half gates
    the production probe path's Python-visible consequences bitwise."""
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
    st.tire_qfrc(dt)
    with pytest.raises(ValueError, match='increasing timestamp'):
        st.tire_qfrc(dt)
    committed = np.asarray(st.tire_state())
    bad = committed.copy()
    bad[names.index('front.ray.000.delta_prev_m')] = np.inf
    with pytest.raises(RuntimeError):
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
        # committed boundary and the published sample do not move.
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


def test_pneumatic_observations_declare_multi_contact(tmp_path, native_module):
    """The committed tire channel publishes the pneumatic dict: backend,
    the multi-contact capability flag and the per-side load counters —
    the wire tire_channels reads before trusting multi_support."""
    from bike_sim.native.research import create_native_research
    from bike_sim.sim.ride.control import RideControl
    env = _env(tmp_path, duration='.05')
    native = create_native_research(env)
    try:
        native.step(RideControl())
        tires = native.snapshot().latest_sample.channels['tires']
        for side in ('front', 'rear'):
            assert tires[side]['backend'] == 'pneumatic'
            assert tires[side]['supports_multiple_contacts'] is True
            assert tires[side]['normal_load_n'] > 0.
            assert tires[side]['patch_count'] >= 1
            assert tires[side]['surface']
    finally:
        native.close()
        env.close()


@pytest.mark.slow
def test_pneumatic_stored_energy_and_counters(tmp_path):
    """The energy surface: stored_energy at the committed pose plus the
    post-commit counters — bitwise against the live oracle's attributes."""
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path, steps=20)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g / 'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g / 'model.mjb'), project(env))
    applier = env.sim.physical.tire
    names = list(ep.tire_state_names)
    dt = float(ref.opt.timestep)
    for k in range(0, len(ep.state_qpos), 5):
        _restore(ref, dref, ep, k)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        st.set_tire_state(names, ep.tire_state[k])
        st.tire_qfrc(dt)
        applier.set_state(names, ep.tire_state[k])
        applier.compute_qfrc(ref, dref, dt)
        assert_bitwise_equal(st.tire_stored_energy(),
                             applier.stored_energy(ref, dref),
                             f'stored_energy step {k}')
        assert_bitwise_equal(st.tire_brush_loss_step_j(),
                             applier.brush_loss_step_j,
                             f'brush_loss_step_j step {k}')
        assert_bitwise_equal(st.tire_radial_dissipation_power_w(),
                             applier.radial_dissipation_power_w,
                             f'radial_dissipation_power_w step {k}')
