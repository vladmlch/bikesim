"""Async period-close: spawned worker must publish identical samples."""
import pickle
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'


def _env(tmp_path, *, async_telemetry, rider='articulated_planar', duration=1.2, seed=3):
    from bike_sim.cli import research as research_cli
    track = tmp_path / f'track-{seed}.toml'
    track.write_text('name = "probe"\nlength_m = 40.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.02], [40.0, 0.02]] }\n')
    argv = ['--track-file', str(track),
            '--duration', str(duration), '--rider', rider, '--seed', str(seed),
            '--energy-tolerance', '1e9', '--diagnostic-model-limits',
            '--initial-speed', '2', '--record-decimation', '400',
            '--out', str(tmp_path / f'out-{seed}-{async_telemetry}')]
    if rider == 'articulated_planar':
        argv += ['--physics-config', str(WELDED)]
    if async_telemetry:
        argv.append('--fast-telemetry')
    args = research_cli.parser().parse_args(argv)
    return research_cli.make_environment(args)


def _collect(env):
    """Every published sample across the episode, in publication order."""
    from bike_sim.sim.ride.control import RideControl
    seen = []
    while not env.done:
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
        seen.extend(env.sim.physical.completed_samples)
    env.sim.physical.flush()
    seen.extend(env.sim.physical.completed_samples)
    return seen


def _same_sample(a, b):
    if (a.interval_id, a.time_s, a.end_time_s) != (b.interval_id, b.time_s, b.end_time_s):
        return False
    if not (np.array_equal(a.qpos, b.qpos) and np.array_equal(a.qvel, b.qvel)):
        return False
    if set(a.powers_w) != set(b.powers_w):
        return False
    return all(np.array_equal(a.powers_w[k], b.powers_w[k]) for k in a.powers_w)


@pytest.mark.slow
def test_async_close_publishes_identical_samples(tmp_path):
    """Lumped rider has no effort channel: async must be bitwise identical."""
    sync_samples = _collect(_env(tmp_path, async_telemetry=False, rider='lumped'))
    async_samples = _collect(_env(tmp_path, async_telemetry=True, rider='lumped'))
    assert len(sync_samples) == len(async_samples) > 0
    for a, b in zip(sync_samples, async_samples):
        assert _same_sample(a, b), f'sample {a.interval_id} diverged'
        assert a.channels['model_status'] == b.channels['model_status']


@pytest.mark.slow
def test_async_close_articulated_episode_completes(tmp_path):
    """Articulated rider: samples arrive one step late but the episode closes
    with the same count and identical physics payload."""
    sync_samples = _collect(_env(tmp_path, async_telemetry=False, rider='articulated_planar'))
    async_samples = _collect(_env(tmp_path, async_telemetry=True, rider='articulated_planar'))
    assert len(sync_samples) == len(async_samples) > 0
    for a, b in zip(sync_samples, async_samples):
        assert _same_sample(a, b), f'sample {a.interval_id} diverged'
        assert a.channels['model_status']['model_valid'] == b.channels['model_status']['model_valid']


def _synthetic_ctx(tmp_path):
    """Minimal context for a rider-free worker on the shipped physics config."""
    from bike_sim.physics.resolution import load_physics_config
    cfg = load_physics_config()
    return {'cfg': cfg, 'timestep_s': .0005, 'rider_control': None,
            'has_rider_contacts': False,
            'initial_energy_j': 100., 'energy_scale_j': 50.,
            'initial_battery_j': 0., 'electrical_work_j': 0.,
            'loss_j': 0., 'active_work_j': 0., 'external_work_j': 0.,
            'solver_work_j': 0., 'muscle_signed_j': 0., 'muscle_positive_j': 0.,
            'motor_signed_j': 0., 'motor_positive_j': 0., 'constraint_absolute_j': 0.,
            'history': None, 'model_status': None}


def _raw(i):
    from bike_sim.sim.ride.period_buffer import RawStep
    return RawStep(interval_id=i, time_s=i*.0005, end_time_s=(i+1)*.0005,
                   qpos=np.zeros(4), qvel=np.zeros(4), components={'chain': np.ones(4)},
                   attachment_raw={}, actuator_force=np.zeros(0), qfrc_passive=np.zeros(4),
                   contact_truth={}, metadata={
                       'loss_step_j': .1, 'electrical_power_w': 0.,
                       'mechanical_energy_j': 99.5, 'elastic_energy_j': {},
                       'battery_energy_j': 0., 'attachment_errors': (),
                       'rider_control_terms': {}, 'effort_base': {},
                       'numerical_constraint_power_w': {'equality:0': .01},
                       'constraint_snapshot': None, 'full': i % 10 == 9,
                       'invalid_controller': False,
                       'channels': {'tires': {'front': {'patches': [{'normal_load_n': 50.,
                                              'source_geom': 'terrain'}]},
                                              'rear': {'patches': [{'normal_load_n': 60.,
                                             'source_geom': 'terrain'}]}},
                                    'sensors': {}},
                       'diagnostics': {'rider': {}, 'rider_welds': {}, 'endpoint_mass': {},
                                       'rider_intent': None, 'inclination_rad': 0.,
                                       'rider_allocation': {}, 'rider_ik_saturation': {},
                                       'rider_support_targets': {}}})


def test_worker_roundtrip_publishes_period(tmp_path):
    """A spawned worker applies publish_close to synthetic raws verbatim."""
    from bike_sim.sim.ride.async_close import AsyncCloseRunner
    runner = AsyncCloseRunner()
    try:
        runner.send_context(_synthetic_ctx(tmp_path))
        raws = [_raw(i) for i in range(10)]
        runner.submit(raws)
        results = runner.collect(block=True)
        assert len(results) == 1
        fields = results[0]['fields']
        assert len(results[0]['samples']) == 10
        assert results[0]['samples'][-1][0] == 9
        assert abs(fields['loss_j'] - 1.0) < 1e-12
        from bike_sim.sim.ride.async_close import _sample_restore
        sample = _sample_restore(results[0]['samples'][0])
        assert sample.interval_id == 0 and not sample.qpos.flags.writeable
    finally:
        runner.close()


def test_worker_reports_publication_error(tmp_path):
    """A malformed raw fails in the worker exactly once, as a RuntimeError."""
    from bike_sim.sim.ride.async_close import AsyncCloseRunner
    runner = AsyncCloseRunner()
    try:
        runner.send_context(_synthetic_ctx(tmp_path))
        bad = _raw(0)
        bad = pickle.loads(pickle.dumps(bad))  # prove serializability first
        bad.metadata.pop('loss_step_j')
        runner.submit([bad])
        with pytest.raises(RuntimeError, match='async close worker failed'):
            runner.collect(block=True)
    finally:
        runner.close()
