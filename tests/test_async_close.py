"""Async period-close: spawned worker must publish identical samples."""
import pickle
import queue
from collections import deque
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
    """Every published sample across the episode, in publication order.

    Samples publish per physics step inside an env.step window, so reading
    `completed_samples` once per env.step misses windows that closed earlier.
    The real consumer observes each physics step — spy at that boundary.
    """
    from bike_sim.sim.ride.control import RideControl
    seen, seen_ids = [], set()
    consume = env._consume_completed_samples

    def spy(**kwargs):
        for sample in env.sim.physical.completed_samples:
            if sample.interval_id not in seen_ids:
                seen_ids.add(sample.interval_id)
                seen.append(sample)
        consume(**kwargs)

    env._consume_completed_samples = spy
    try:
        while not env.done:
            env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
        env.sim.physical.flush()
        for sample in env.sim.physical.completed_samples:
            if sample.interval_id not in seen_ids:
                seen.append(sample)
        return seen
    finally:
        env.close(discard_pending=True)


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


@pytest.mark.parametrize('start_method', [None, 'spawn'])
def test_worker_roundtrip_publishes_period(tmp_path, start_method):
    """A spawned worker applies publish_close to synthetic raws verbatim."""
    from bike_sim.sim.ride.async_close import AsyncCloseRunner
    runner = AsyncCloseRunner(start_method=start_method)
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


@pytest.mark.parametrize('start_method', [None, 'spawn'])
def test_worker_reports_publication_error(tmp_path, start_method):
    """A malformed raw fails in the worker exactly once, as a RuntimeError."""
    from bike_sim.sim.ride.async_close import AsyncCloseRunner
    runner = AsyncCloseRunner(start_method=start_method)
    try:
        runner.send_context(_synthetic_ctx(tmp_path))
        bad = _raw(0)
        bad = pickle.loads(pickle.dumps(bad))  # prove serializability first
        bad.metadata.pop('loss_step_j')
        runner.submit([bad])
        with pytest.raises(RuntimeError, match='async close worker failed'):
            runner.collect(block=True)
        assert runner.collect(block=True) == []  # the error is not counted twice
    finally:
        runner.close()


def _reply_runner(messages, *, alive=None):
    """Control the reply boundary without depending on OS scheduling races."""
    from bike_sim.sim.ride.async_close import AsyncCloseRunner
    runner = AsyncCloseRunner.__new__(AsyncCloseRunner)
    runner._pending = deque([(0, ['first']), (1, ['second'])])
    runner._out_q = queue.Queue()
    for message in messages:
        runner._out_q.put(message)
    runner._proc = SimpleNamespace(is_alive=(lambda: True) if alive is None else alive,
                                   exitcode=17)
    return runner


def test_worker_loss_preserves_consumed_but_unapplied_prefix():
    from bike_sim.sim.ride.async_close import AsyncWorkerLost
    states = iter([True, False])
    runner = _reply_runner([('ok', 0, {'first': True})], alive=lambda: next(states))
    with pytest.raises(AsyncWorkerLost, match='unacknowledged'):
        runner.collect(block=True)
    assert runner.detach_pending() == [['first'], ['second']]


@pytest.mark.parametrize('reply,match', [(('ok', 7, {}), 'out of order'),
                                       (('invalid', 1, {}), 'invalid result status')])
def test_bad_reply_preserves_all_unapplied_periods(reply, match):
    from bike_sim.sim.ride.async_close import AsyncWorkerLost
    runner = _reply_runner([('ok', 0, {}), reply])
    with pytest.raises(AsyncWorkerLost, match=match):
        runner.collect(block=True)
    assert runner.detach_pending() == [['first'], ['second']]


def test_publication_error_applies_successful_prefix_exactly_once():
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    from bike_sim.sim.ride.async_close import AsyncPublicationError
    runner = _reply_runner([('ok', 0, {'first': True}),
                            ('err', 1, ('ValueError', 'invalid raw'))])
    runtime = PhysicalRuntime.__new__(PhysicalRuntime)
    runtime._async_close = runner
    applied = []
    runtime._apply_close_result = applied.append
    with pytest.raises(AsyncPublicationError, match='invalid raw'):
        runtime._drain_async(block=True)
    assert applied == [{'first': True}]
    assert runner.detach_pending() == []
    assert runtime._drain_async(block=True) is False
    assert applied == [{'first': True}]


def test_explicit_drain_applies_prefix_before_publication_error():
    runner = _reply_runner([('ok', 0, {'first': True}),
                            ('err', 1, ('ValueError', 'invalid raw'))])
    applied = []
    with pytest.raises(RuntimeError, match='invalid raw'):
        runner.drain(applied.append)
    assert applied == [{'first': True}]
    assert runner.collect(block=True) == []


def test_resource_close_never_publishes_or_drains():
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    closed = []

    def forbidden(*args, **kwargs):
        raise AssertionError('publication during resource-only close')

    runtime = PhysicalRuntime.__new__(PhysicalRuntime)
    runtime._async_close = SimpleNamespace(close=lambda: closed.append(True), drain=forbidden)
    runtime._async_ctx_dirty = True
    runtime.flush = runtime._publish_and_accept = forbidden
    runtime._close_async_worker()
    runtime._close_async_worker()
    assert closed == [True]
    assert runtime._async_close is None and not runtime._async_ctx_dirty


def test_disabling_worker_preserves_publication_error_and_closes():
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    closed = []

    def publication_failure(apply):
        raise RuntimeError('publication failed')

    def cleanup_failure():
        closed.append(True)
        raise OSError('cleanup failed')

    runtime = PhysicalRuntime.__new__(PhysicalRuntime)
    runtime._async_close = SimpleNamespace(drain=publication_failure, close=cleanup_failure)
    runtime._async_ctx_dirty = True
    with pytest.raises(RuntimeError, match='publication failed') as failure:
        runtime.set_async_close(False)
    assert closed == [True]
    assert any('cleanup failed' in note for note in failure.value.__notes__)
    assert runtime._async_close is None


def test_research_close_keeps_active_window_guard_and_discards_without_steps():
    from bike_sim.sim.research.environment import ResearchEnvironment
    closed = []
    environment = ResearchEnvironment.__new__(ResearchEnvironment)
    environment.sim = SimpleNamespace(physical=SimpleNamespace(
        _close_async_worker=lambda: closed.append(True)))
    window = environment._window = object()
    environment._closed = False
    with pytest.raises(RuntimeError, match='active interval'):
        environment.close()
    assert environment._window is window and not environment._closed and closed == []
    environment.close(discard_pending=True)
    assert environment._window is None and environment._closed and closed == [True]


def test_research_close_attempts_both_owners_preserving_first_failure():
    from bike_sim.sim.research.environment import ResearchEnvironment
    calls = []

    def worker_close():
        calls.append('worker')
        raise RuntimeError('worker cleanup')

    def asset_close():
        calls.append('assets')
        raise OSError('asset cleanup')

    environment = ResearchEnvironment.__new__(ResearchEnvironment)
    environment._window = None
    environment.sim = SimpleNamespace(physical=SimpleNamespace(_close_async_worker=worker_close))
    environment._replay_assets_owner = SimpleNamespace(cleanup=asset_close)
    with pytest.raises(RuntimeError, match='worker cleanup') as failure:
        environment.close()
    assert calls == ['worker', 'assets']
    assert any('asset cleanup' in note for note in failure.value.__notes__)
    # A failed asset cleanup remains retryable even though the environment is closed.
    environment.sim.physical._close_async_worker = lambda: None
    environment._replay_assets_owner.cleanup = lambda: calls.append('retry')
    environment.close()
    assert calls[-1] == 'retry' and environment._replay_assets_owner is None


@pytest.mark.parametrize('failure', [RuntimeError('setup failed'), KeyboardInterrupt()])
def test_research_constructor_failure_releases_worker(monkeypatch, failure):
    from bike_sim.sim.research.environment import ResearchEnvironment
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    physical = PhysicalRuntime.__new__(PhysicalRuntime)
    closed = []
    sim = SimpleNamespace(physical=physical, steps=0, time_s=0.,
        model=SimpleNamespace(opt=SimpleNamespace(timestep=.0005)),
        physics_config=SimpleNamespace(physics_mode='physical', drive_mode='crank_effort',
            pitch_assist=False, tires=SimpleNamespace(backend='compliant_2d', surface_mode='track')))

    def failed_setup(self):
        self.sim.physical._async_close = SimpleNamespace(close=lambda: closed.append(True))
        raise failure

    monkeypatch.setattr(ResearchEnvironment, '_begin_episode', failed_setup)
    with pytest.raises(type(failure)) as caught:
        ResearchEnvironment(sim)
    assert caught.value is failure
    assert closed == [True] and physical._async_close is None


@pytest.mark.parametrize('backend,failing', [('python', True), ('native', False)])
def test_physical_driver_construction_owns_only_needed_worker(monkeypatch, backend, failing):
    from bike_sim.sim.ride import physical_driver, physical_session
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    physical = PhysicalRuntime.__new__(PhysicalRuntime)
    calls = []

    def start(enabled):
        assert backend == 'python' and enabled
        calls.append('start')
        physical._async_close = SimpleNamespace(close=lambda: calls.append('close'))

    physical.set_async_close = start
    sim = SimpleNamespace(physical=physical)
    expected = object()

    def driver(*args, **kwargs):
        if failing:
            raise ValueError('driver setup failed')
        return expected

    monkeypatch.setattr(physical_driver, 'require_backend', lambda *args: None)
    monkeypatch.setattr(physical_session, 'build_physical_simulation', lambda *args: sim)
    monkeypatch.setattr(physical_driver, 'PhysicalRideDriver', driver)
    args = SimpleNamespace(backend=backend, resolved_physics=None, headless=True,
                           decimate=1, duration=.01, record=False)
    if failing:
        with pytest.raises(ValueError, match='driver setup failed'):
            physical_driver.build_physical_driver(None, args, None, async_telemetry=True)
        assert calls == ['start', 'close']
    else:
        assert physical_driver.build_physical_driver(None, args, None,
            async_telemetry=True) is expected
        assert calls == []


def test_python_driver_close_releases_worker_once():
    from bike_sim.sim.ride.physical_driver import PhysicalRideDriver
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    physical = PhysicalRuntime.__new__(PhysicalRuntime)
    calls = []
    physical._async_close = SimpleNamespace(close=lambda: calls.append('close'))
    driver = PhysicalRideDriver.__new__(PhysicalRideDriver)
    driver._closed, driver._native = False, None
    driver._sim = SimpleNamespace(physical=physical)
    driver.close()
    driver.close()
    assert calls == ['close']


def test_close_reaps_forced_worker_and_preserves_interrupt(monkeypatch):
    from bike_sim.sim.ride import async_close
    events = []
    interrupted = KeyboardInterrupt()

    class Process:
        alive = True
        joins = 0

        def join(self, timeout):
            events.append(('join', timeout))
            self.joins += 1
            if self.joins == 1:
                raise interrupted

        def is_alive(self):
            return self.alive

        def terminate(self):
            events.append('terminate')

        def kill(self):
            events.append('kill')
            self.alive = False

    runner = async_close.AsyncCloseRunner.__new__(async_close.AsyncCloseRunner)
    runner._proc, runner._closed = Process(), False
    runner._in_q = SimpleNamespace(put=lambda message: events.append(message))
    runner._out_q = object()

    def queue_failure(queues):
        assert tuple(queues) == (runner._in_q, runner._out_q)
        events.append('queues')
        raise OSError('queue close failed')

    monkeypatch.setattr(async_close, '_close_queues', queue_failure)
    with pytest.raises(KeyboardInterrupt) as caught:
        runner.close()
    assert caught.value is interrupted
    assert any('queue close failed' in note for note in interrupted.__notes__)
    assert events == [('stop', None, None), ('join', 2.), 'terminate',
                      ('join', 2.), 'kill', ('join', 2.), 'queues']
    assert not runner._proc.is_alive()
    runner.close()
    assert len(events) == 7
