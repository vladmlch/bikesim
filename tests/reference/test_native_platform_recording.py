"""T21 cross-platform run-recording qualification (Linux/macOS).

The lifecycle suite pins the journal/seal contracts through fake window
pumps and publication spies; this file proves the same contracts land on
a real filesystem on either supported OS: the exclusive final directory
appears only at ``finalize``, a published run is always the
``run.duckdb``/``manifest.json``/``summary.md`` triple readable through
the manifest-first ``Recording`` reader, sealed sources stay owned and
decodable across resets, interruption and publication failure never lose
evidence, and a crashed publication leaves nothing that looks like a
run. Storage seams are the production ``write_recording``/``Recording``
pair — never a hand-built fake database.
"""
from datetime import timezone
import os
from pathlib import Path
import platform
import subprocess
import sys
from types import SimpleNamespace

import pytest

from bike_sim.recording.contracts import InputEvent
from bike_sim.recording.json_codec import loads
from bike_sim.recording.reader import Recording, RecordingError
from bike_sim.recording.storage import write_recording
from bike_sim.sim.ride.physical_frontend import run_headless, run_viewer

import _recording_support as support
from _recording_support import sample_row
from native_loader import load_native
from test_frontend_windows import Clock, patch_window, row
from test_native_recording_lifecycle import (
    RecordingDriver, StoringDriver, _event_kinds,
    _headless_args, _interrupting_publisher, _recording, _second_sigint)

# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def assert_published_run(run_directory):
    """One exclusive final directory holds exactly the sealed payload."""
    root = Path(run_directory)
    assert (root / 'run.duckdb').is_file()
    assert (root / 'manifest.json').is_file()
    assert (root / 'summary.md').is_file()


def _unpublished(out_root):
    """No public payload exists under ``out_root`` yet (final-only R14)."""
    for name in ('manifest.json', 'run.duckdb', 'summary.md'):
        assert not list(Path(out_root).rglob(name)), name


def _published_run(out_root):
    """Exactly one run directory was published under ``out_root``."""
    manifests = list(Path(out_root).rglob('manifest.json'))
    assert len(manifests) == 1
    return manifests[0].parent


def _publishing_spy(published):
    """Observe the capture at publication, then run the real storage seam."""
    def publish(capture, out_root):
        published.append(capture)
        return write_recording(capture, out_root)
    return publish


def _physical_rows(saved):
    return saved.rows(
        'SELECT episode_id, physical_rows FROM episodes ORDER BY episode_id')


def _ride_args(out_root, *, record=True, policy='diagnostic', duration='0.01'):
    """Parsed CLI args for the documented welded/rough-track fixture."""
    from bike_sim.cli import ride
    from _native_runtime_support import PHYSICS, TRACK
    argv = ['--physics-config', str(PHYSICS), '--track', str(TRACK),
            '--backend', 'native', '--headless', '--duration', duration,
            '--no-plots', '--out', str(out_root),
            '--check-policy', policy]
    if record:
        argv.append('--record')
    args = ride.parse_args(argv)
    track = ride.resolve_track(args.track, seed=args.seed,
                               length_m=args.length)
    return args, track, ride.resolve_rider(args)


def _event_values(saved, kind, episode_id=0):
    return [loads(entry['values_json']) for entry in saved.rows(
        'SELECT values_json FROM events WHERE episode_id=? AND kind=? '
        'ORDER BY event_id', [episode_id, kind])]


# --- Final-only publication through the fake window pump + real storage ---


def test_viewer_reset_twice_publishes_all_episode_ids(tmp_path, monkeypatch):
    """Two resets seal three owned episodes; the exclusive run directory
    and its public files appear only at finalize, never mid-pump."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock,
                 {5: [ord('R')], 10: [ord('R')], 15: [ord('Q')]})

    def still_unpublished():
        _unpublished(tmp_path)

    driver = StoringDriver(
        clock,
        episode_rows=[[row(0, contact=True, brake=.4)],
                      [row(0, contact=True, brake=.4),
                       row(1, contact=False, brake=.4)],
                      [row(0, contact=True, brake=.4),
                       row(1, contact=False, brake=.4),
                       row(2, contact=False, brake=0.)]],
        seal_hooks=[still_unpublished, still_unpublished,
                    still_unpublished])
    published = []
    assert run_viewer(driver, out_root=tmp_path,
                      capture=_recording(
                          publisher=_publishing_spy(published))) == 0
    assert driver.seal_calls == [(1, 0, 'operator_reset'),
                                 (2, 1, 'operator_reset'),
                                 (3, 2, 'viewer_closed')]
    assert len(published) == 1
    # Sealed sources stay owned and decodable after the resets that
    # produced them — the typed tables re-read through the bound catalog.
    episodes = {e.episode_id: e for e in published[0].iter_episodes()}
    assert sorted(episodes) == [0, 1, 2]
    counts = {}
    for episode_id, evidence in episodes.items():
        tables = {}
        for batch in evidence.physical.tables(batch_rows=2):
            tables[batch.table] = tables.get(batch.table, 0) + batch.row_count
        counts[episode_id] = tables['physics']
    assert counts == {0: 1, 1: 2, 2: 3}
    run_directory = _published_run(tmp_path)
    assert_published_run(run_directory)
    with Recording(run_directory) as saved:
        assert saved.episode_ids == (0, 1, 2)
        assert _physical_rows(saved) == [
            {'episode_id': 0, 'physical_rows': 1},
            {'episode_id': 1, 'physical_rows': 2},
            {'episode_id': 2, 'physical_rows': 3}]
        # The reset that closed an episode is journaled on that episode;
        # quit lands on the final one.
        assert 'reset' in _event_kinds(run_directory, 0)
        assert 'reset' in _event_kinds(run_directory, 1)
        assert 'quit' in _event_kinds(run_directory, 2)
        assert 'derived_transition' in _event_kinds(run_directory, 2)


def test_viewer_recording_disabled_with_out_writes_nothing(tmp_path, monkeypatch):
    """--out is a location, never an opt-in: a launch without --record
    leaves no run files behind, not even empty ones."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {3: [ord('Q')]})
    driver = RecordingDriver(clock)
    assert run_viewer(driver, out_root=tmp_path) == 0
    assert driver.closed
    assert list(tmp_path.rglob('*')) == []


def test_viewer_ctrl_c_seals_operator_stop_and_publishes(tmp_path, monkeypatch):
    """A single Ctrl-C stops the pump, seals the committed prefix as
    operator_stop and still publishes the owned run."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {40: [ord('Q')]})
    driver = StoringDriver(
        clock, advance_failures={3: KeyboardInterrupt()},
        episode_rows=[[row(0, contact=True, brake=.4),
                       row(1, contact=False, brake=.4)]])
    with pytest.raises(KeyboardInterrupt):
        run_viewer(driver, out_root=tmp_path, capture=_recording())
    assert driver.seal_calls == [(1, 0, 'operator_stop')]
    assert driver.metadata.get('failure') is None
    run_directory = _published_run(tmp_path)
    assert_published_run(run_directory)
    with Recording(run_directory) as saved:
        assert saved.episode_ids == (0,)
        summary = loads(saved.episode()['summary_json'])
        assert summary['outcome']['reason'] == 'operator_stop'
        assert _event_kinds(run_directory) == [
            'episode_start', 'derived_transition']
    assert driver.closed


def test_viewer_second_ctrl_c_during_seal_journals_and_publishes(
        tmp_path, monkeypatch):
    """Double Ctrl-C: the first interrupts the pump, the second lands
    inside the shielded seal — deferred, journaled, and the owned prefix
    still publishes as a complete run."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {40: [ord('Q')]})
    driver = StoringDriver(
        clock, advance_failures={3: KeyboardInterrupt()},
        episode_rows=[[row(0, contact=True, brake=.4),
                       row(1, contact=False, brake=.4)]],
        seal_hooks=[_second_sigint])
    with pytest.raises(KeyboardInterrupt):
        run_viewer(driver, out_root=tmp_path, capture=_recording())
    assert driver.seal_calls == [(1, 0, 'operator_stop')]
    run_directory = _published_run(tmp_path)
    assert_published_run(run_directory)
    with Recording(run_directory) as saved:
        assert saved.episode_ids == (0,)
        assert saved.episode()['physical_rows'] == 2
        assert _event_kinds(run_directory) == [
            'episode_start', 'derived_transition', 'operator_stop_deferred']
    assert driver.closed


def test_viewer_ctrl_c_inside_finalize_is_noted_and_published(
        tmp_path, monkeypatch):
    """A Ctrl-C deferred inside ``finalize`` lands too late to journal;
    the run is already committed and the propagating interrupt carries
    the deferral note."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {40: [ord('Q')]})
    driver = StoringDriver(clock, advance_failures={3: KeyboardInterrupt()},
                           episode_rows=[[row(0, contact=True, brake=.4)]])
    with pytest.raises(KeyboardInterrupt) as excinfo:
        run_viewer(driver, out_root=tmp_path,
                   capture=_recording(publisher=_interrupting_publisher()))
    assert driver.seal_calls == [(1, 0, 'operator_stop')]
    assert any('operator interrupt deferred' in note
               for note in getattr(excinfo.value, '__notes__', ()))
    run_directory = _published_run(tmp_path)
    assert_published_run(run_directory)
    assert 'operator_stop_deferred' not in _event_kinds(run_directory)
    assert driver.closed


def test_viewer_simulation_error_publishes_owned_prefix(tmp_path, monkeypatch):
    """A simulation error seals the committed prefix under
    'simulation_error' and publishes it — evidence survives the
    propagating primary failure."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {40: [ord('Q')]})
    driver = StoringDriver(
        clock, advance_failures={2: RuntimeError('engine broke')},
        episode_rows=[[row(0, contact=True, brake=.4)]])
    with pytest.raises(RuntimeError, match='engine broke'):
        run_viewer(driver, out_root=tmp_path, capture=_recording())
    assert driver.metadata['failure'] == 'RuntimeError: engine broke'
    run_directory = _published_run(tmp_path)
    assert_published_run(run_directory)
    with Recording(run_directory) as saved:
        assert saved.episode_ids == (0,)
        summary = loads(saved.episode()['summary_json'])
        assert summary['outcome']['reason'] == 'simulation_error'
    assert driver.closed


# --- Real storage seam: retries, exclusivity and crash survivors ---


def test_publication_permission_denied_then_retry(tmp_path):
    """A filesystem-level EACCES inside publication fails closed: the
    capture keeps its owned evidence in ``publication_failed`` and an
    explicit retry publishes it once — no duplicated episodes or events."""
    if hasattr(os, 'geteuid') and os.geteuid() == 0:
        # Root bypasses the mode bits this regression must exercise. A fresh
        # child drops all groups and its UID before importing the probe and
        # creates its own temporary tree; root-owned pytest parents are never
        # made writable or chowned. Missing privilege support is a failure,
        # not a successful skip of the EACCES contract (completion 3.11).
        import pwd
        account = pwd.getpwnam('nobody')
        script = r'''
import os
import sys
os.setgroups([])
os.setgid(int(sys.argv[1]))
os.setuid(int(sys.argv[2]))
assert os.geteuid() != 0, 'permission probe still has root privileges'
from pathlib import Path
from tempfile import TemporaryDirectory
sys.path.insert(0, sys.argv[3])
from test_native_platform_recording import _publication_permission_denied_probe
with TemporaryDirectory(prefix='bike-eacces-') as directory:
    _publication_permission_denied_probe(Path(directory))
print('unprivileged EACCES and explicit publication retry passed')
'''
        child = subprocess.run(
            [sys.executable, '-c', script, str(account.pw_gid),
             str(account.pw_uid), str(Path(__file__).resolve().parent)],
            text=True, capture_output=True, check=False, timeout=60)
        assert child.returncode == 0, child.stdout + '\n' + child.stderr
        assert 'unprivileged EACCES and explicit publication retry passed' in child.stdout
        return
    _publication_permission_denied_probe(tmp_path)


def _publication_permission_denied_probe(tmp_path):
    denied = tmp_path/'denied'
    denied.mkdir()
    capture = support.synthetic_capture(
        rows=[sample_row(0, start=0., dt=.001)],
        events=[InputEvent(0, 0, 'episode_start', 0., 0, None, {})])
    denied.chmod(0o500)
    try:
        with pytest.raises(PermissionError):
            capture.finalize(denied)
        assert capture.state == 'publication_failed'
        # The denied attempt created nothing readable — not even staging.
        assert not list(denied.iterdir())
    finally:
        denied.chmod(0o700)
    run_directory = capture.finalize(denied)
    assert_published_run(run_directory)
    with Recording(run_directory) as saved:
        assert saved.episode_ids == (0,)
        assert saved.episode()['physical_rows'] == 1
        assert _event_kinds(run_directory) == ['episode_start']


def test_exclusive_names_collide_without_overwrite(tmp_path):
    """Two publications under one run identity never share a name: the
    exclusive mkdir skips to a suffixed directory instead of adopting or
    overwriting — even a pre-existing empty collision stays untouched."""
    first_capture = support.synthetic_capture(
        rows=[sample_row(0, start=0., dt=.001)])
    metadata = first_capture.metadata
    stamp = metadata.started_at.astimezone(
        timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    base = stamp + '-' + str(metadata.run_id)
    # An empty squatting directory is somebody else's: it is skipped,
    # never adopted as this run's destination.
    squatter = tmp_path/base
    squatter.mkdir()
    first = first_capture.finalize(tmp_path)
    assert first.name == base + '-1'
    assert list(squatter.iterdir()) == []
    second = support.synthetic_capture(
        rows=[sample_row(0, start=0., dt=.001)]).finalize(tmp_path)
    assert second.name == base + '-2'
    for directory in (first, second):
        assert_published_run(directory)
        with Recording(directory) as saved:
            assert saved.episode_ids == (0,)


def test_finalization_crash_leaves_no_readable_run(tmp_path):
    """A process dying mid-publication cannot leave an apparently
    complete run: payload files may sit in the exclusive directory but
    without manifest.json the reader rejects it, and surviving staging
    directories keep their UNSEALED marker."""
    child = tmp_path/'child'
    script = f'''
import os, sys
sys.path.insert(0, {str(Path(__file__).resolve().parent)!r})
import time
import mujoco.viewer
import bike_sim.sim.ride.physical_frontend as frontend
from test_frontend_windows import Clock, Window, Presenter, row
from test_native_recording_lifecycle import StoringDriver, _recording

clock = Clock()
frontend.FramePresenter = lambda model: Presenter(model, clock)
time.monotonic, time.sleep = clock.monotonic, clock.sleep
mujoco.viewer.launch_passive = lambda model, data, *, key_callback, **kw: \\
    Window(key_callback, {{3: [ord('Q')]}})

real_link = os.link
def crash_before_manifest(src, dst):
    if str(dst).endswith('manifest.json'):
        os._exit(111)
    return real_link(src, dst)
os.link = crash_before_manifest

driver = StoringDriver(
    clock, episode_rows=[[row(0, contact=True, brake=.4)]])
frontend.run_viewer(driver, out_root={str(child)!r}, capture=_recording())
'''
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(
        [str(Path(__file__).resolve().parent)]
        + ([env['PYTHONPATH']] if env.get('PYTHONPATH') else []))
    result = subprocess.run(
        [sys.executable, '-c', script], env=env,
        capture_output=True, text=True, timeout=180, check=False)
    assert result.returncode == 111, result.stderr
    # Nothing readable was published: no manifest exists anywhere.
    assert not list(tmp_path.rglob('manifest.json'))
    # A destination holding payloads but no manifest is not a run.
    unsealed = [p.parent for p in tmp_path.rglob('run.duckdb')]
    assert unsealed
    for directory in unsealed:
        if directory.name.startswith('.run-recording-unsealed-'):
            assert (directory/'UNSEALED').is_file()
        else:
            with pytest.raises(RecordingError):
                Recording(directory)


# --- Real native runs through the production CLI path ---


def test_headless_native_record_publishes_real_run(native_module, tmp_path):
    """The documented short fixture: a real native --record headless run
    publishes every retained scientific state through the real storage
    seam — intervals, validity flags, first-failure evidence, source
    assets, selected backend and v2 execution identity."""
    args, track, rider = _ride_args(tmp_path)
    _unpublished(tmp_path)
    code = run_headless(track, args, args.seed, rider)
    # The rough recipe trips a sticky model violation; diagnostic mode
    # still reaches the duration and reports the failure honestly.
    assert code == 2
    run_directory = _published_run(tmp_path)
    assert_published_run(run_directory)
    with Recording(run_directory) as saved:
        assert saved.episode_ids == (0,)
        assert run_directory.name.endswith(saved.manifest['run_id'])
        metadata = saved.metadata
        assert metadata['backend'] == 'native'
        assert metadata['mode'] == 'headless'
        assert metadata['values']['check_policy'] == 'diagnostic'
        episode = saved.episode()
        assert episode['generation'] == 1
        assert episode['replay_contract_version'] is None
        assert episode['replay_eligible'] is False
        assert episode['physical_rows'] > 0
        physics = saved.rows('SELECT min(interval_id) lo, max(interval_id) hi,'
                             ' count(*) n FROM physics')[0]
        assert physics == {'lo': 0, 'hi': episode['physical_rows'] - 1,
                           'n': episode['physical_rows']}
        summary = loads(episode['summary_json'])
        assert summary['outcome']['reason'] == 'duration_reached'
        assert summary['model_status']['model_valid'] is False
        assert summary['recording_retention'] == 'full'
        failure_time, violations = summary['first_failure']
        assert 'energy.constraint_work' in violations
        execution = summary['execution']
        assert execution['identity_schema'] == 2
        assert execution['backend'] == 'native'
        assert execution['build_context']['platform']['system'] == \
            platform.system()
        for digest in ('extension_sha256', 'mujoco_library_sha256',
                       'native_source_sha256'):
            assert isinstance(execution[digest], str) \
                and len(execution[digest]) == 64
        # Event journal: source fact precedes channel-derived facts.
        event_kinds = _event_kinds(run_directory)
        assert event_kinds[0] == 'episode_start'
        measured = _event_values(saved, 'model_first_failure')
        assert len(measured) == 1
        assert measured[0]['classification'] == 'measured'
        assert measured[0]['time_s'] == pytest.approx(failure_time)
        assert measured[0]['violations'] == violations
        assert 'derived_transition' in event_kinds
        # Typed source assets bundled with the sealed episode.
        assets = {entry['name'] for entry in saved.rows(
            'SELECT name FROM assets')}
        assert assets == {'resolved_config.json', 'rider_joint_envelope.json',
                          'rider_joint_strength.json', 'terrain_vertices.npy',
                          'track.toml'}
        for table in ('channel_tires', 'channel_energy', 'channel_control',
                      'forces'):
            assert saved.rows(
                f'SELECT count(*) n FROM {table}')[0]['n'] > 0


def test_headless_native_record_strict_stop_keeps_invalid_prefix(
        native_module, tmp_path):
    """The documented strict scenario stops at the first
    energy.constraint_work violation well inside the requested duration;
    the published run keeps its actual invalid outcome and committed
    prefix — never relabeled success."""
    args, track, rider = _ride_args(tmp_path, policy='strict')
    code = run_headless(track, args, args.seed, rider)
    assert code == 2
    run_directory = _published_run(tmp_path)
    assert_published_run(run_directory)
    with Recording(run_directory) as saved:
        assert saved.episode_ids == (0,)
        episode = saved.episode()
        assert episode['physical_rows'] > 0
        summary = loads(episode['summary_json'])
        assert summary['outcome']['reason'] == 'invalid_controller'
        assert summary['model_status']['model_valid'] is False
        failure_time, violations = summary['first_failure']
        assert 'energy.constraint_work' in violations
        # The stop happens at the violation, well before the requested
        # duration — the prefix is honest, not lengthened.
        assert summary['outcome']['time_s'] == pytest.approx(failure_time)
        assert summary['outcome']['time_s'] < args.duration
        physics = saved.rows('SELECT min(interval_id) lo, max(interval_id) hi,'
                             ' count(*) n FROM physics')[0]
        assert physics == {'lo': 0, 'hi': episode['physical_rows'] - 1,
                           'n': episode['physical_rows']}
        measured = _event_values(saved, 'model_first_failure')
        assert len(measured) == 1
        assert measured[0]['classification'] == 'measured'
        assert measured[0]['time_s'] == pytest.approx(failure_time)


def test_headless_native_record_off_with_out_writes_nothing(
        native_module, tmp_path):
    """Recording off + --out supplied: no run files at all — --record is
    the sole opt-in."""
    args, track, rider = _ride_args(tmp_path, record=False)
    code = run_headless(track, args, args.seed, rider)
    assert code == 2
    assert list(tmp_path.rglob('*')) == []


def test_headless_native_record_under_spaced_unicode_path(
        native_module, tmp_path):
    """Publication and read-back survive spaces and non-ASCII characters
    in the --out path on Linux/macOS filesystems."""
    out_root = tmp_path/'runs dir'/'ünïcodé ✓'
    args, track, rider = _ride_args(out_root)
    code = run_headless(track, args, args.seed, rider)
    assert code == 2
    run_directory = _published_run(out_root)
    assert_published_run(run_directory)
    with Recording(run_directory) as saved:
        assert saved.episode_ids == (0,)
        assert saved.episode()['physical_rows'] > 0


def test_headless_python_record_rejected_before_setup(tmp_path, monkeypatch):
    """Python-backend --record is refused before any environment or
    window setup — never a silent native/Python fallback."""
    import bike_sim.sim.ride.physical_frontend as frontend
    def forbidden(*args, **kwargs):
        raise AssertionError('driver construction ran for python --record')
    monkeypatch.setattr(frontend, 'build_physical_driver', forbidden)
    args = SimpleNamespace(record=True, backend='python')
    with pytest.raises(ValueError, match='native backend'):
        frontend.run_headless(None, args, 0, None)
    assert list(tmp_path.rglob('*')) == []


def test_model_construction_failure_leaves_no_files(tmp_path, monkeypatch):
    """A failure before capture construction publishes nothing: the
    launch-level capture does not exist yet, so --out stays empty."""
    import bike_sim.sim.ride.physical_frontend as frontend
    def boom(*args, **kwargs):
        raise RuntimeError('model construction failed')
    monkeypatch.setattr(frontend, 'build_physical_driver', boom)
    args = _headless_args(tmp_path, record=True)
    with pytest.raises(RuntimeError, match='model construction failed'):
        frontend.run_headless(SimpleNamespace(name='fake'), args, 0, None)
    assert list(tmp_path.rglob('*')) == []


def test_viewer_native_reset_twice_publishes_owned_episodes(
        environment_factory, native_module, tmp_path, monkeypatch):
    """A real native driver under the fake window seals each generation
    into owned decodable evidence; after two resets the single finalized
    run holds all three episode IDs."""
    import bike_sim.sim.ride.physical_frontend as frontend
    from bike_sim.sim.ride.physical_driver import PhysicalRideDriver
    driver = PhysicalRideDriver(environment_factory().sim, backend='native',
                                duration_s=.02, record=True)
    clock = Clock()
    patch_window(monkeypatch, frontend, clock,
                 {4: [ord('R')], 8: [ord('R')], 12: [ord('Q')]})
    published = []
    try:
        code = run_viewer(driver, out_root=tmp_path,
                          capture=_recording(
                              publisher=_publishing_spy(published)))
        # The environment recipe reports a sticky model violation; the
        # check-policy exit code is honest about it.
        assert code == 2
        episodes = {e.episode_id: e for e in published[0].iter_episodes()}
        assert sorted(episodes) == [0, 1, 2]
        assert [e.generation for e in episodes.values()] == [1, 2, 3]
        # Each sealed typed source still decodes through its catalog
        # after the resets that outlived it.
        for evidence in episodes.values():
            tables = {batch.table
                      for batch in evidence.physical.tables(batch_rows=64)}
            assert 'physics' in tables and 'channel_tires' in tables
        run_directory = _published_run(tmp_path)
        assert_published_run(run_directory)
        with Recording(run_directory) as saved:
            assert saved.episode_ids == (0, 1, 2)
            assert [entry['generation']
                    for entry in saved.rows('SELECT generation FROM episodes'
                                            ' ORDER BY episode_id')] == [1, 2, 3]
            assert 'reset' in _event_kinds(run_directory, 0)
            assert 'reset' in _event_kinds(run_directory, 1)
            assert 'quit' in _event_kinds(run_directory, 2)
    finally:
        driver.close()


# --- Research entrypoint: same final-only lifecycle, research payload ---


def test_research_native_run_publishes_sealed_recording(
        environment_pair, tmp_path):
    """A real native research launch through ``ResearchRun`` publishes
    the sealed run once at finish: research tables, v2 execution
    identity and episode boundaries all land in the final directory."""
    from bike_sim.sim.research.recording_lifecycle import ResearchRun
    from bike_sim.sim.ride.control import RideControl
    _, env = environment_pair(capture_mode='full')
    launch = ResearchRun(env, tmp_path, enabled=True,
                         mode='research_headless')
    _unpublished(tmp_path)
    while not env.done:
        env.step(RideControl(0.))
    path = launch.finish()
    assert_published_run(path)
    with Recording(path) as saved:
        assert saved.episode_ids == (0,)
        episode = saved.episode()
        # A clean completed research episode is checked-replay eligible.
        assert episode['replay_contract_version'] == 3
        assert episode['replay_eligible'] is True
        assert episode['physical_rows'] > 0
        assert episode['complete_control_steps'] > 0
        metadata = saved.metadata
        assert metadata['backend'] == 'native'
        assert metadata['mode'] == 'research_headless'
        execution = metadata['values']['execution']
        assert execution['identity_schema'] == 2
        assert execution['backend'] == 'native'
        assert execution['build_context']['platform']['system'] == \
            platform.system()
        # Episode boundary: the run opens with episode_started and
        # closes with episode_ended.
        event_kinds = _event_kinds(path)
        assert event_kinds[0] == 'episode_started'
        assert event_kinds[-1] == 'episode_ended'
        # Research evidence extents: control history, observations and
        # the initial/final integration-state pair are all sealed.
        for table in ('commands_requested', 'commands_applied',
                      'observations', 'transitions'):
            assert saved.rows(
                f'SELECT count(*) n FROM {table}')[0]['n'] > 0
        states = saved.rows('SELECT phase FROM integration_states'
                            ' ORDER BY phase')
        assert [entry['phase'] for entry in states] == ['final', 'initial']


def test_research_policy_failure_after_capture_publishes_evidence(
        environment_pair, tmp_path):
    """A policy failure after capture still seals the owned episode and
    publishes it under its real outcome — with launch_failure recorded,
    not relabeled."""
    from bike_sim.sim.research.recording_lifecycle import ResearchRun
    from bike_sim.sim.ride.control import RideControl
    _, env = environment_pair(capture_mode='full')
    launch = ResearchRun(env, tmp_path, enabled=True,
                         mode='research_headless')
    _unpublished(tmp_path)
    env.step(RideControl(0.))
    env.fail_policy(RuntimeError('policy blew up'))
    path = launch.finish(primary=RuntimeError('policy blew up'))
    assert_published_run(path)
    with Recording(path) as saved:
        assert saved.episode_ids == (0,)
        episode = saved.episode()
        # A policy-failed episode is published, never replay eligible.
        assert episode['replay_eligible'] is False
        summary = loads(episode['summary_json'])
        assert summary['outcome']['reason'] == 'policy_error'
        failure = summary['research']['run_metadata']['launch_failure']
        assert failure['error_type'] == 'RuntimeError'
        assert 'policy blew up' in failure['error']
        event_kinds = _event_kinds(path)
        assert 'launch_failed' in event_kinds
        assert event_kinds[-1] == 'episode_ended'
