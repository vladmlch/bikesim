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
    qfrc, evolved wire, snapshots, diagnostics and the energy surface —
    bitwise vs the recorded manifest rows and the live Python applier."""
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
        # The evolved per-ray/patch wire is the step's second output —
        # bitwise vs the live applier AND vs the row the next recorded
        # physics step consumed (the artifact's own ordering evidence).
        wire = np.asarray(st.tire_state())
        assert_bitwise_equal(wire, applier.state_dict()['row'],
                             f'tire_state evolve step {k}')
        if k + 1 < len(ep.state_qpos):
            assert_bitwise_equal(wire, ep.tire_state[k + 1],
                                 f'tire_state golden evolve step {k}')
        _assert_snapshots_equal(st.tire_snapshots(),
                                ep.manifest['tire_snapshots'][k], k)
        # Diagnostics: the recorded manifest row and the live applier's
        # committed dict are the same publication, so compare both.
        _assert_diagnostics_equal(st.tire_diagnostics(),
                                  ep.manifest['tire_diagnostics'][k], k)
        _assert_diagnostics_equal(st.tire_diagnostics(),
                                  applier.diagnostics, k)
        assert_bitwise_equal(st.tire_stored_energy(),
                             applier.stored_energy(ref, dref),
                             f'stored_energy step {k}')
        assert_bitwise_equal(st.tire_brush_loss_step_j(),
                             applier.brush_loss_step_j,
                             f'brush_loss_step_j step {k}')
        assert_bitwise_equal(st.tire_radial_dissipation_power_w(),
                             applier.radial_dissipation_power_w,
                             f'radial_dissipation_power_w step {k}')


@pytest.mark.slow
def test_pneumatic_qfrc_bitwise(tmp_path):
    """Golden per-state parity on alpine_rocky: the consumed-state wire
    rows, qfrc vectors, evolved wire, contact snapshots and diagnostics —
    bitwise against the manifest and the live Python applier."""
    env, g = _golden(tmp_path, steps=40, tier='fast')
    _run_golden_parity(env, g)


EVENT_TRACK = '''\
name = "pneumatic_event_probe"
length_m = 40.0
surface = "hardpack"
grade_profile = { knots = [[0.0, 0.0], [40.0, 0.0]] }
surface_sections = [
  { start_m = 1.3, end_m = 3.0, surface = "scree" },
  { start_m = 4.6, end_m = 8.0, surface = "wet" },
]
[[obstacles]]
type = "bump"
shape = "cosine"
start_m = 3.5
height_m = 0.14
bump_length_m = 0.5
[[obstacles]]
type = "square_edge"
start_m = 4.9
height_m = 0.42
ledge_length_m = 0.6
'''


def _event_config(tmp_path):
    """Event-probe physics TOML: the pneumatic backend on the lumped-rider
    drive ('crank_effort'). The articulated rider's attachment model raises
    once the bike pitches onto the ledge — a rider-model limit outside the
    tyre path under test — so the event episode rides the lumped rider."""
    text = PNEUMATIC_BASE.read_text()
    for key in ('joint_envelope_path', 'joint_strength_path'):
        text = re.sub(
            rf'{key} = "([^"]+)"',
            lambda m: f'{key} = "{(PNEUMATIC_BASE.parent / m.group(1)).resolve()}"',
            text)
    text = text.replace('backend = "distributed_2d_reference"',
                        'backend = "pneumatic"')
    text = re.sub(r'\[tires\.distributed\][^\[]*', '', text)
    text = text.replace('drive_mode = "articulated_effort"',
                        'drive_mode = "crank_effort"')
    text = re.sub(r'\[articulated\][^\[]*', '', text)
    text = re.sub(r'\[seated_climb\][^\[]*',
                  '[seated_climb]\nenabled = false\n', text)
    text += '\n[tires.pneumatic]\ntier = "fast"\n'
    path = tmp_path / 'physics_events.toml'
    path.write_text(text)
    return path


def _event_env(tmp_path):
    from bike_sim.cli import research as research_cli
    track = tmp_path / 'event_track.toml'
    track.write_text(EVENT_TRACK)
    args = research_cli.parser().parse_args([
        '--physics-config', str(_event_config(tmp_path)),
        '--track-file', str(track), '--duration', '2', '--dt', '.00125',
        '--rider', 'lumped', '--initial-speed', '8',
        '--diagnostic-model-limits', '--out', str(tmp_path / 'event_out')])
    return research_cli.make_environment(args)


def _patch_bounds(row, names, side):
    """The consumed patch (start, stop) bound tuples of one wire row."""
    count = int(row[names.index(f'{side}.patch.count')])
    return tuple(
        (int(row[names.index(f'{side}.patch.{j:02d}.start')]),
         int(row[names.index(f'{side}.patch.{j:02d}.stop')]))
        for j in range(count))


@pytest.mark.slow
def test_pneumatic_golden_episode_event_rich(tmp_path):
    """A physical episode built to stress the pneumatic branches: the bump
    launches the front, the 0.42 m square edge drives the coverage gate and
    rim strikes while the tyre bridges the face, the pitch-up hangs both
    wheels airborne, and the scree/wet sections give the surface-map
    transitions. Every step still passes the shared golden parity —
    qfrc, wire, snapshots, diagnostics and the energy counters."""
    from tools.golden_episode import capture_episode, load_episode, save
    from bike_sim.sim.ride.control import RideControl
    env = _event_env(tmp_path)
    ep = capture_episode(env, 280, RideControl(human_torque_nm=35.))
    g = tmp_path / 'ge'
    save(ep, g, env.sim.model)
    _run_golden_parity(env, g)
    ep = load_episode(g)
    diag = ep.manifest['tire_diagnostics']
    assert len(diag) == len(ep.state_qpos) == 280
    sides = {side: [row[side] for row in diag] for side in ('front', 'rear')}

    def air_runs(flags):
        """(run count, longest run) — airborne sequences, not single blips."""
        runs = longest = current = 0
        for flag in flags:
            current = current + 1 if flag else 0
            if flag and current == 1:
                runs += 1
            longest = max(longest, current)
        return runs, longest

    front_runs, front_longest = air_runs(
        d['airborne'] for d in sides['front'])
    # The bump launch plus the nose-stand hang on the ledge.
    assert front_runs >= 2 and front_longest >= 30
    # The rear wheel leaves the ground when the bike pitches onto the ledge.
    assert sum(d['airborne'] for d in sides['rear']) >= 10
    # Coverage gate and rim strikes on the square edge's face.
    assert sum(d['coverage_event'] for d in sides['front']) >= 5
    assert sum(d['rim_strike_active'] for d in sides['front']) >= 15
    # The bump lip splits the front contact into two patches.
    assert sum(d['patch_count'] > 1 for d in sides['front']) >= 3
    # scree at spawn -> hardpack -> wet at the ledge; the rear leaves scree.
    for side, minimum in (('front', 2), ('rear', 1)):
        surfaces = [d['surface'] for d in sides[side]]
        transitions = sum(a != b for a, b in zip(surfaces, surfaces[1:]))
        assert transitions >= minimum
    # The consumed patch (start, stop) bounds migrate as the contact
    # evolves — the state-reuse path sees every drift through the wire.
    names = list(ep.tire_state_names)
    drift = sum(
        _patch_bounds(ep.tire_state[k], names, side)
        != _patch_bounds(ep.tire_state[k - 1], names, side)
        for side in ('front', 'rear')
        for k in range(1, len(ep.tire_state)))
    assert drift >= 60
    # The rim-event record rides the wire: finite cells while active.
    assert np.isfinite(
        ep.tire_state[:, names.index('front.rim.energy_j')]).sum() >= 10


@pytest.mark.slow
def test_pneumatic_golden_reset_restore_mid_run(tmp_path):
    """Stepper.reset() mid-replay drops the committed tyre state and the
    timestamp clock; re-seeding the saved wire on the restored pose resumes
    the golden stream bitwise — mirrored by applier.reset() + set_state."""
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path, steps=30)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g / 'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g / 'model.mjb'), project(env))
    applier = env.sim.physical.tire
    tire_i = ep.force_names.index('tires')
    names = list(ep.tire_state_names)
    dt = float(ref.opt.timestep)
    mid = 15
    for k in range(len(ep.state_qpos)):
        if k == mid:
            # reset() also restores mjData to model defaults, so the pose
            # is re-seeded from the artifact before the wire restore.
            st.reset()
            applier.reset()
        _restore(ref, dref, ep, k)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        st.set_tire_state(names, ep.tire_state[k])
        got = np.asarray(st.tire_qfrc(dt))
        applier.set_state(names, ep.tire_state[k])
        want = applier.compute_qfrc(ref, dref, dt)
        assert_bitwise_equal(got, ep.forces[k][tire_i],
                             f'tires golden step {k}')
        assert_bitwise_equal(got, want, f'tires live-oracle step {k}')
        assert_bitwise_equal(np.asarray(st.tire_state()),
                             applier.state_dict()['row'],
                             f'tire_state evolve step {k}')
        _assert_snapshots_equal(st.tire_snapshots(),
                                ep.manifest['tire_snapshots'][k], k)
        _assert_diagnostics_equal(st.tire_diagnostics(),
                                  ep.manifest['tire_diagnostics'][k], k)


@pytest.mark.slow
def test_pneumatic_golden_failure_atomicity(tmp_path):
    """Poisoned inputs mid-replay leave every committed artifact untouched:
    a non-finite wire cell rejects set_tire_state, a non-positive dt and a
    repeated timestamp reject tire_qfrc — state wire, snapshots,
    diagnostics and the energy counters hold bitwise, and the stream
    resumes bitwise afterwards."""
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path, steps=24)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g / 'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g / 'model.mjb'), project(env))
    tire_i = ep.force_names.index('tires')
    names = list(ep.tire_state_names)
    dt = float(ref.opt.timestep)

    def replay(k):
        _restore(ref, dref, ep, k)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        st.set_tire_state(names, ep.tire_state[k])
        got = np.asarray(st.tire_qfrc(dt))
        assert_bitwise_equal(got, ep.forces[k][tire_i],
                             f'tires golden step {k}')

    mid = 12
    for k in range(mid):
        replay(k)
    committed = np.asarray(st.tire_state())
    snaps = st.tire_snapshots()
    diags = st.tire_diagnostics()
    energy = (st.tire_stored_energy(), st.tire_brush_loss_step_j(),
              st.tire_radial_dissipation_power_w())

    def assert_untouched(label):
        assert_bitwise_equal(np.asarray(st.tire_state()), committed,
                             f'{label}: state wire')
        _assert_snapshots_equal(st.tire_snapshots(), snaps, label)
        _assert_diagnostics_equal(st.tire_diagnostics(), diags, label)
        assert_bitwise_equal(st.tire_stored_energy(), energy[0],
                             f'{label}: stored_energy')
        assert_bitwise_equal(st.tire_brush_loss_step_j(), energy[1],
                             f'{label}: brush_loss_step_j')
        assert_bitwise_equal(st.tire_radial_dissipation_power_w(), energy[2],
                             f'{label}: radial_dissipation_power_w')

    # A non-finite state cell: the rejection precedes any commit.
    bad = committed.copy()
    bad[names.index('front.ray.000.delta_prev_m')] = np.inf
    with pytest.raises(RuntimeError):
        st.set_tire_state(names, bad)
    assert_untouched('poisoned wire')
    # The gated compute rejects before the wheel loop: nothing half-advances.
    with pytest.raises(ValueError):
        st.tire_qfrc(-dt)
    assert_untouched('negative dt')
    with pytest.raises(ValueError, match='increasing timestamp'):
        st.tire_qfrc(dt)
    assert_untouched('repeated timestamp')
    # The stream resumes exactly — no residue from the rejected calls.
    for k in range(mid, len(ep.state_qpos)):
        replay(k)
    _assert_snapshots_equal(st.tire_snapshots(),
                            ep.manifest['tire_snapshots'][-1],
                            'final step')
    _assert_diagnostics_equal(st.tire_diagnostics(),
                              ep.manifest['tire_diagnostics'][-1],
                              'final step')


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


def _pneumatic_wire(cfg_tires, profile, surface_map=None):
    """The tire section through the production emitter (the same dict
    Stepper receives from project()) under a tire-only top-level dict —
    every other writer section is optional in config.hpp. A SurfaceMap is
    required only when the config's surface_mode is 'track'."""
    from bike_sim.native.config import _tire_section
    return {'schema': 2,
            'tire': _tire_section(cfg_tires, surface_map, profile)}


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
def test_pneumatic_rig_event_sweep_detailed_tier(tmp_path):
    """The detailed tier through the event branches the research environment
    cannot reach (its 0.00025 s timestep gate leaves the static equilibrium
    solver unconverged within budget, so the physical episode above is the
    fast tier): scripted poses on the collision-free two-wheel rig drive the
    rear wheel through the bridging notch for a second patch, the front
    across the surface-map sections and into the step face — coverage,
    rim strike and multi-patch together — plus airborne and flat rim
    ticks. Every tick is bitwise-equal to the live applier: qfrc, the
    evolved bristle/patch wire, snapshots, diagnostics, energy."""
    from bike_sim.physics.physical_config import (
        PneumaticTireConfig, TireBackendConfig)
    from bike_sim.sim.ride.pneumatic_tire_forces import (
        PneumaticTireForceApplier)
    from bike_sim.sim.ride.tyre.geometry import RoadProfile
    from bike_sim.terrain.surface import (
        SurfaceMap, SurfaceSection, get_surface)
    tier = 'detailed'
    xml, front_r, rear_r = _wheel_rig_xml(tier)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # A 60 mm bridging notch ahead of the spawn, then a 0.45 m step face.
    x = np.arange(0.0, 4.0, 0.002)
    z = np.where((x >= 1.47) & (x <= 1.53), -0.09, 0.0)
    z = np.where(x >= 2.2, 0.45, z)
    profile = RoadProfile.from_samples(x, z)
    # scree and wet bands the patch centroids cross as the hub advances.
    surface_map = SurfaceMap(
        get_surface('hardpack'),
        [SurfaceSection(1.5, 1.8, 'scree'), SurfaceSection(1.8, 2.2, 'wet')])
    cfg_tires = TireBackendConfig(
        backend='pneumatic', surface_mode='track',
        pneumatic=PneumaticTireConfig(tier=tier))
    applier = PneumaticTireForceApplier(model, profile, cfg_tires,
                                      surface_map)
    mjb = tmp_path / 'rig_events.mjb'
    mujoco.mj_saveModel(model, str(mjb), None)
    st = bike_native.Stepper(
        str(mjb), _pneumatic_wire(cfg_tires, profile, surface_map))
    names = list(applier.state_names())
    assert tuple(st.tire_state_names) == tuple(names)
    front_x = _joint_adr(model, 'front_root_x', 'qpos')
    front_z = _joint_adr(model, 'front_root_z', 'qpos')
    rear_x = _joint_adr(model, 'rear_root_x', 'qpos')
    rear_z = _joint_adr(model, 'rear_root_z', 'qpos')
    front_spin = _joint_adr(model, 'front_wheel_spin', 'dof')
    rear_spin = _joint_adr(model, 'rear_wheel_spin', 'dof')
    dt = float(model.opt.timestep)
    # The slide-joint qpos is a delta on the XML pos: front hub x IS its
    # qpos (pos0 x=0), rear hub x = 0.8 + qpos; hub z offsets ride on the
    # rig's authored .004/.003 m drops. hub_z = radius + dz_offset.
    ticks = [
        # label, front_x, front_dz, rear_x, rear_dz, front_spin, rear_spin
        ('flat',        1.30, -.012, 1.30, -.012, 4.0, 4.0),
        ('notch-appr',  1.44, -.012, 1.30, -.012, 4.0, 4.0),
        ('notch-F',     1.50, -.012, 1.40, -.012, 4.0, 4.0),
        ('notch-R',     1.56, -.012, 1.50, -.012, 4.0, 4.0),
        ('hold',        1.56, -.012, 1.50, -.012, 4.0, 4.0),
        ('scree-both',  1.70, -.012, 1.62, -.012, 4.0, 4.0),
        ('wet-F',       1.85, -.012, 1.70, -.012, 4.0, 4.0),
        ('cliff-1',     1.90, -.012, 1.30, -.012, 4.0, 4.0),
        ('cliff-2',     1.95, -.012, 1.30, -.012, 4.0, 4.0),
        ('cliff-3',     2.00, -.012, 1.30, -.012, 4.0, 4.0),
        ('cliff-4',     2.05, -.012, 1.30, -.012, 4.0, 4.0),
        ('cliff-face',  2.10, -.012, 1.30, -.012, 4.0, 4.0),
        ('airborne',    0.90, +.100, 1.30, -.012, 4.0, 4.0),
        ('flat-rim',    1.35, -.055, 1.30, -.012, 4.0, 4.0),
        ('rear-cliff',  1.30, -.012, 1.95, -.012, 4.0, 4.0),
    ]
    seen = {side: {'multi': [], 'coverage': [], 'rim': [], 'airborne': [],
                   'surfaces': []} for side in ('front', 'rear')}
    wire_bounds = []
    for k, (label, fx, fz, rx, rz, fom, rom) in enumerate(ticks):
        data.qpos[front_x] = fx
        data.qpos[front_z] = fz + .004
        data.qpos[rear_x] = rx - 0.8
        data.qpos[rear_z] = rz + .003
        data.qvel[front_spin] = fom
        data.qvel[rear_spin] = rom
        data.time = (k + 1) * dt
        mujoco.mj_forward(model, data)
        row = applier.state_dict()['row']             # the consumed seed
        st.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                     np.asarray(data.act),
                     np.asarray(data.qacc_warmstart), float(data.time))
        st.forward()
        st.set_tire_state(names, row)
        got = np.asarray(st.tire_qfrc(dt))
        want = applier.compute_qfrc(model, data, dt)
        assert_bitwise_equal(got, want, f'{label}: qfrc')
        wire = np.asarray(st.tire_state())
        assert_bitwise_equal(wire, applier.state_dict()['row'],
                             f'{label}: evolved wire')
        wire_bounds.append(tuple(
            _patch_bounds(wire, names, side) for side in ('front', 'rear')))
        _assert_live_snapshots(st.tire_snapshots(), applier.snapshots, label)
        _assert_diagnostics_equal(st.tire_diagnostics(),
                                  applier.diagnostics, label)
        assert_bitwise_equal(st.tire_stored_energy(),
                             applier.stored_energy(model, data),
                             f'{label}: stored_energy')
        assert_bitwise_equal(st.tire_brush_loss_step_j(),
                             applier.brush_loss_step_j,
                             f'{label}: brush_loss')
        assert_bitwise_equal(st.tire_radial_dissipation_power_w(),
                             applier.radial_dissipation_power_w,
                             f'{label}: radial_power')
        for side in ('front', 'rear'):
            diag = applier.diagnostics[side]
            seen[side]['multi'].append(diag['patch_count'] > 1)
            seen[side]['coverage'].append(diag['coverage_event'])
            seen[side]['rim'].append(diag['rim_strike_active'])
            seen[side]['airborne'].append(diag['airborne'])
            seen[side]['surfaces'].append(diag['surface'])
    # The scripted poses hit every branch the sweep was built for —
    # asserted on the oracle's committed diagnostics, which the native
    # dict matched bitwise at each tick.
    assert any(seen['front']['multi']) and any(seen['rear']['multi'])
    assert sum(seen['front']['coverage']) >= 4
    assert any(seen['rear']['coverage']) and any(seen['rear']['rim'])
    assert sum(seen['front']['rim']) >= 5
    assert any(seen['front']['airborne'])
    for side in ('front', 'rear'):
        surfaces = seen[side]['surfaces']
        assert sum(a != b for a, b in zip(surfaces, surfaces[1:])) >= 1
    # The patch bounds migrate under the moving contact (state reuse on
    # the detailed tier copies bristle overlap across the shift) — and the
    # held pose repeats identical bounds for the exact-key path.
    assert sum(a != b for a, b in zip(wire_bounds, wire_bounds[1:])) >= 10


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
