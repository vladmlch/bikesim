"""T22: portable inspection/playback stay separate from checked replay.

Two contracts, one corpus:

* ``bike-recording`` inspection commands and ``bike-replay --view`` recorded-
  frame playback are portable: no extension load, no policy import, no
  execution-identity comparison and no physical stepping. A recording
  authored on another host — or one whose run ended in a policy/launch
  failure — is still fully inspectable and playable.
* ``bike-replay`` (and ``--viewer``) remain checked physical replay: the
  complete versioned execution identity is compared before any plant
  construction or step, the strictest incompatibility explanation names the
  exact field path, and zero plant steps may occur on a mismatch.

The literal brief:

    "Do not call either [execution compatibility] function in the
    independent data-inspection/non-simulating playback branch."

is enforced by the subprocess test below, which forbids
``require_execution_compatible``, ``execution_provenance``,
``load_native_extension``, ``create_native_research``, ``require_backend``
and ``mujoco.mj_step`` and then runs the inspection commands plus a headless
``--view --dry-run`` to completion.
"""
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest

# tests/ is on sys.path for this file's imports; the reviewed recording
# helpers live in tests/reference/.
_REFERENCE = Path(__file__).resolve().parent / 'reference'
sys.path.insert(0, str(_REFERENCE))
try:
    from _recording_support import synthetic_capture
finally:
    sys.path.remove(str(_REFERENCE))

from bike_sim.recording.json_codec import dumps, loads
from bike_sim.recording.reader import open_recording
from bike_sim.recording.replay_input import load_replay_input
from bike_sim.recording.reader import RecordingError


def _record_run(tmp_path, *, steps=2, fail=False):
    """A real sealed run container recorded by the Python research frontend.

    ``capture_mode='full'`` keeps the authored replay contract — recipe.json,
    resolved_config.json, terrain vertices — inside the container so both the
    checked adapter and the portable playback loader can consume it.
    """
    from bike_sim.cli.research import make_environment, parser
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.research.recording_lifecycle import ResearchRun
    root = Path(__file__).resolve().parents[1]
    arguments = ['--physics-config',
                 str(root / 'examples/research/viewer_physics_welded.toml'),
                 '--scenario', 'flat', '--duration', '0.04', '--seed', '19',
                 '--record-decimation', '1', '--actuator-delay', '0.005',
                 '--diagnostic-model-limits']
    env = make_environment(parser().parse_args(arguments), capture_mode='full')
    launch = ResearchRun(env, tmp_path, enabled=True, mode='test')
    for _ in range(steps):
        env.step(RideControl(0.))
    if fail:
        env.fail_policy(RuntimeError('policy exploded'))
    else:
        env.stop()
    path = launch.finish()
    env.close()
    return path


def _file_hashes(path):
    """sha256 of every regular file in the recording — mutation evidence."""
    return {entry.name: hashlib.sha256(entry.read_bytes()).hexdigest()
            for entry in sorted(Path(path).iterdir()) if entry.is_file()}


def _patch_execution(path, mutate):
    """Rewrite the recorded execution identity inside a sealed container.

    ``episodes.summary_json`` and the bundled ``recipe.json`` must agree, and
    the manifest's file digest must be rewritten for the changed database —
    exactly the fields a transported cross-host recording would carry.
    """
    import duckdb
    path = Path(path)
    database = path / 'run.duckdb'
    connection = duckdb.connect(str(database))
    try:
        row = connection.execute(
            'SELECT summary_json FROM episodes WHERE episode_id=0').fetchone()
        summary = loads(row[0])
        summary['research']['execution'] = mutate(summary['research']['execution'])
        connection.execute('UPDATE episodes SET summary_json=? WHERE episode_id=0',
                           [dumps(summary)])
        row = connection.execute(
            "SELECT content FROM assets WHERE episode_id=0 AND name='recipe.json'"
        ).fetchone()
        recipe = loads(bytes(row[0]).decode('utf-8'))
        recipe['execution'] = mutate(recipe['execution'])
        content = dumps(recipe).encode('utf-8')
        connection.execute(
            "UPDATE assets SET content=?, sha256=? WHERE episode_id=0 AND name='recipe.json'",
            [content, hashlib.sha256(content).hexdigest()])
    finally:
        connection.close()
    manifest = json.loads((path / 'manifest.json').read_text())
    manifest['file_sha256']['run.duckdb'] = hashlib.sha256(
        database.read_bytes()).hexdigest()
    (path / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')


def _other_system():
    return 'Linux' if platform.system() == 'Darwin' else 'Darwin'


def _foreign_system(record):
    """The identity record the same software would carry on the other OS.

    A recorded record must stay self-consistent — validation rejects an
    impossible platform/numerics pairing outright — so transporting an
    identity across hosts rewrites the qualified pair together: platform
    identity *and* the numerics provider that platform hosts.
    """
    from copy import deepcopy
    record = deepcopy(record)
    context = record['build_context']
    platform_record, numerics = context['platform'], context['numerics']
    if platform_record['system'] == 'Darwin':
        context['platform'] = dict(
            libc={'name': 'glibc', 'version': '2.39'},
            machine=platform_record['machine'], os_build='#1 SMP',
            python_soabi='cpython-314-aarch64-linux-gnu',
            python_version=platform_record['python_version'], system='Linux')
        context['numerics'] = dict(
            backend='NUMPY_OPENBLAS', blas_integer_bits=64, lapack_integer_bits=64,
            cpu_profile=numerics['cpu_profile'], numpy_version=numerics['numpy_version'],
            threading={'implementation': 'openblas',
                       'num_threads': numerics['threading']['num_threads']},
            libraries=[dict(role='blas_lapack', identity_kind='file',
                            sha256='0' * 64, build_id=None)])
    else:
        context['platform'] = dict(
            libc={'name': 'libSystem', 'version': '1'},
            machine=platform_record['machine'], os_build='1',
            python_soabi='cpython-314-darwin',
            python_version=platform_record['python_version'], system='Darwin')
        context['numerics'] = dict(
            backend='ACCELERATE', blas_integer_bits=32, lapack_integer_bits=64,
            cpu_profile=numerics['cpu_profile'], numpy_version=numerics['numpy_version'],
            threading={'implementation': 'accelerate',
                       'num_threads': numerics['threading']['num_threads']},
            libraries=[dict(role='blas_lapack', identity_kind='apple-system',
                            sha256=None, build_id='foreign-fixture')])
    return record


def _foreignize(path):
    """Make the container's recorded identity claim the other operating system."""
    _patch_execution(path, _foreign_system)


def _recipe_execution(path):
    """The recorded execution identity inside a sealed container's recipe."""
    import duckdb
    connection = duckdb.connect(str(Path(path) / 'run.duckdb'), read_only=True)
    try:
        content = connection.execute(
            "SELECT content FROM assets WHERE episode_id=0 "
            "AND name='recipe.json'").fetchone()[0]
    finally:
        connection.close()
    return loads(bytes(content).decode('utf-8'))['execution']


# --------------------------------------------------------------------------
# Inspection commands: identical contract on any host, never a verify claim
# --------------------------------------------------------------------------


def test_inspection_commands_open_recordings_without_any_native_artifact(tmp_path, capsys):
    path = _record_run(tmp_path / 'run')
    from bike_sim.cli.recording import main
    for argv in (['overview', str(path)],
                 ['fields', str(path)],
                 ['events', str(path), '--episode', '0'],
                 ['window', str(path), '--episode', '0',
                  '--start', '0', '--end', '1', '--fields', 'physics.time_s'],
                 ['sql', str(path), '--query',
                  'SELECT count(*) AS n FROM physics', '--json'],
                 ['compare', str(path), str(path),
                  '--left-episode', '0', '--right-episode', '0'],
                 ['compare', str(path), str(path), '--mode', 'summary',
                  '--left-episode', '0', '--right-episode', '0']):
        assert main(argv) == 0, argv
        capsys.readouterr()
    out = tmp_path / 'exported.jsonl'
    assert main(['export', str(path), '--episode', '0', '--format', 'jsonl',
                 '--output', str(out), '--table', 'metrics']) == 0
    # The export contract states what it is — analytics, not a replay bundle.
    payload = json.loads(capsys.readouterr().out)
    assert 'not a checked-replay bundle' in payload['semantics']
    assert out.read_text().strip()


def test_inspection_reads_a_recording_authored_on_another_host(tmp_path, capsys):
    path = _record_run(tmp_path / 'run')
    _foreignize(path)
    from bike_sim.cli.recording import main
    assert main(['overview', str(path), '--json']) == 0
    overview = json.loads(capsys.readouterr().out)
    assert overview['episodes']['rows'][0]['episode_id'] == 0
    assert main(['sql', str(path), '--query',
                 'SELECT count(*) AS n FROM physics', '--json']) == 0
    # Inspection is read-only: the database still opens against its manifest.
    with open_recording(path) as recording:
        assert recording.schema_version == 1


def test_checked_replay_still_demands_an_explicit_episode(tmp_path):
    from bike_sim.cli.research import make_environment, parser
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.research.recording_lifecycle import ResearchRun
    root = Path(__file__).resolve().parents[1]
    arguments = ['--physics-config',
                 str(root / 'examples/research/viewer_physics_welded.toml'),
                 '--scenario', 'flat', '--duration', '0.04', '--seed', '19',
                 '--record-decimation', '1', '--actuator-delay', '0.005',
                 '--diagnostic-model-limits']
    env = make_environment(parser().parse_args(arguments), capture_mode='full')
    launch = ResearchRun(env, tmp_path / 'multi', enabled=True, mode='test')
    env.step(RideControl(0.))
    launch.reset()
    env.step(RideControl(0.))
    env.stop()
    path = launch.finish()
    env.close()
    with pytest.raises(RecordingError, match='--episode'):
        load_replay_input(path)
    from bike_sim.sim.research.playback import load_playback
    with pytest.raises(RecordingError, match='--episode'):
        load_playback(path)
    # Each episode remains individually selectable.
    assert load_playback(path, episode_id=1).episode_id == 1


# --------------------------------------------------------------------------
# Recorded-frame playback: portable across hosts, explicitly unverified
# --------------------------------------------------------------------------


def test_recorded_playback_supports_a_foreign_host_recording(tmp_path):
    from bike_sim.sim.research.playback import load_playback
    path = _record_run(tmp_path / 'run')
    _foreignize(path)
    playback = load_playback(path)
    assert playback.verified is False
    assert playback.qpos.shape == playback.qvel.shape
    assert playback.times_s.shape[0] == playback.qpos.shape[0] >= 2
    assert np.isfinite(playback.duration_s) and playback.duration_s > 0.
    pose = playback.pose_at(playback.duration_s * 0.5)
    assert pose.shape == playback.qpos.shape[1:]


def test_recorded_playback_never_claims_physical_verification(tmp_path, capsys):
    from bike_sim.cli.replay import main
    from bike_sim.sim.research.playback import load_playback
    path = _record_run(tmp_path / 'run')
    playback = load_playback(path)
    assert playback.verified is False
    assert not hasattr(playback, 'report')
    assert main([str(path), '--view', '--dry-run']) == 0
    captured = capsys.readouterr()
    assert 'unverified' in captured.err
    assert 'verified' not in captured.out.replace('unverified', '')
    assert 'passed' not in captured.out


def test_failed_runs_stay_playable_but_never_replayable(tmp_path, capsys):
    """A policy-failure run is failure evidence: inspection and recorded-frame
    playback keep working; checked replay refuses it as a completed prefix."""
    path = _record_run(tmp_path / 'run', fail=True)
    with pytest.raises(RecordingError, match='not completed replay'):
        load_replay_input(path)
    from bike_sim.sim.research.playback import load_playback
    playback = load_playback(path)
    assert playback.verified is False and playback.times_s.size >= 2
    from bike_sim.cli.replay import main as replay_main
    from bike_sim.cli.recording import main as recording_main
    assert replay_main([str(path), '--view', '--dry-run']) == 0
    captured = capsys.readouterr()
    assert 'unverified' in captured.err
    assert recording_main(['overview', str(path)]) == 0


def test_playback_rejects_recordings_without_the_authored_contract(tmp_path):
    """Ordinary analysis captures own no replay recipe; playback says so
    instead of manufacturing one."""
    capture = synthetic_capture()
    path = capture.finalize(tmp_path / 'capture')
    from bike_sim.sim.research.playback import load_playback
    with pytest.raises(RecordingError, match='recipe'):
        load_playback(path)
    # Inspection of the same capture is unaffected.
    from bike_sim.recording.analysis import overview
    with open_recording(path) as recording:
        assert overview(recording)['episodes']['rows'][0]['episode_id'] == 0


def test_inspection_and_playback_never_mutate_the_source(tmp_path):
    from bike_sim.cli.replay import main as replay_main
    from bike_sim.cli.recording import main as recording_main
    path = _record_run(tmp_path / 'run')
    before = _file_hashes(path)
    assert recording_main(['overview', str(path)]) == 0
    assert recording_main(['sql', str(path), '--query',
                           'SELECT count(*) AS n FROM physics']) == 0
    from bike_sim.sim.research.playback import load_playback
    load_playback(path)
    assert replay_main([str(path), '--view', '--dry-run']) == 0
    assert _file_hashes(path) == before


def test_recording_survives_a_directory_move(tmp_path, capsys):
    """Containers are self-describing: relocating the run returns identical
    semantic inspection counts and identical playback — the contract never
    embeds the recording creator's checkout path."""
    source = _record_run(tmp_path / 'src')
    from bike_sim.cli.recording import main as recording_main
    assert recording_main(['overview', str(source), '--json']) == 0
    before_overview = json.loads(capsys.readouterr().out)
    assert recording_main(['sql', str(source), '--query',
                           'SELECT count(*) AS n FROM physics', '--json']) == 0
    before_sql = json.loads(capsys.readouterr().out)

    destination = tmp_path / 'moved' / 'run'
    destination.parent.mkdir(parents=True)
    shutil.move(str(source), destination)
    assert recording_main(['overview', str(destination), '--json']) == 0
    after_overview = json.loads(capsys.readouterr().out)
    assert recording_main(['sql', str(destination), '--query',
                           'SELECT count(*) AS n FROM physics', '--json']) == 0
    after_sql = json.loads(capsys.readouterr().out)
    # Semantic equality, not byte equality: same counts, same episode rows.
    assert after_overview['episodes'] == before_overview['episodes']
    assert (after_overview['manifest']['row_counts']
            == before_overview['manifest']['row_counts'])
    assert after_sql == before_sql
    from bike_sim.sim.research.playback import load_playback
    playback = load_playback(destination)
    assert playback.times_s.size >= 2 and playback.verified is False


def test_corrupted_assets_fail_closed(tmp_path):
    """A byte-level edit inside the container is a checksum failure — for
    inspection, playback and checked replay alike."""
    path = _record_run(tmp_path / 'run')
    database = path / 'run.duckdb'
    blob = bytearray(database.read_bytes())
    blob[-1] ^= 0xFF
    database.write_bytes(bytes(blob))
    from bike_sim.sim.research.playback import load_playback
    for probe in (lambda: open_recording(path),
                  lambda: load_playback(path),
                  lambda: load_replay_input(path)):
        with pytest.raises((RecordingError, ValueError), match='[Cc]hecksum|sha256'):
            probe()


def test_missing_episode_is_a_named_error(tmp_path):
    path = _record_run(tmp_path / 'run')
    from bike_sim.sim.research.playback import load_playback
    for probe in (lambda: load_replay_input(path, episode_id=7),
                  lambda: load_playback(path, episode_id=7)):
        with pytest.raises(RecordingError, match='no episode 7'):
            probe()


# --------------------------------------------------------------------------
# Checked replay: the full versioned identity gates before any physics
# --------------------------------------------------------------------------


@pytest.mark.parametrize('recorded_foreign', [True, False])
def test_foreign_identity_cannot_be_checked_replayed(tmp_path, recorded_foreign):
    """The brief's literal field-level gate: either direction names the path."""
    from bike_sim.native.execution_identity import require_execution_compatible
    recorded_identity = _recipe_execution(_record_run(tmp_path / 'run'))
    current_identity = _foreign_system(recorded_identity)
    if recorded_foreign:
        recorded_identity, current_identity = current_identity, recorded_identity
    with pytest.raises(ValueError,
                       match=r'execution\.build_context\.platform\.system'):
        require_execution_compatible(recorded_identity, current_identity)


def test_legacy_identity_report_names_the_missing_fields(tmp_path):
    """A pre-T12 record (no identity_schema) against a v2 runtime reports the
    missing fields by name — nothing is silently projected away."""
    from copy import deepcopy
    from bike_sim.native.execution_identity import require_execution_compatible
    recorded_v2 = _recipe_execution(_record_run(tmp_path / 'run'))
    legacy = {key: value for key, value in deepcopy(recorded_v2).items()
              if key != 'identity_schema'}
    # The legacy contract: a python-backend record carries build_context=None.
    if legacy['backend'] == 'python':
        legacy['build_context'] = None
    else:
        legacy['build_context'] = {
            key: recorded_v2['build_context'][key]
            for key in ('build_type', 'compiler_id', 'compiler_version', 'numerical_flags')}
    with pytest.raises(ValueError, match=r'execution\.identity_schema'):
        require_execution_compatible(legacy, recorded_v2)


def test_foreign_identity_rejects_checked_replay_with_the_named_field(tmp_path, monkeypatch):
    """The literal brief: a recorded identity claiming the other OS is refused
    by ``require_execution_compatible`` before a plant exists — zero steps."""
    path = _record_run(tmp_path / 'run')
    _foreignize(path)
    forbidden = 'a cross-host recording must not reach plant construction'
    monkeypatch.setattr('bike_sim.sim.ride_sim.RideSimulation.__init__',
                        lambda *a, **k: pytest.fail(forbidden))
    from bike_sim.sim.research.replay_session import ReplaySession
    with pytest.raises(ValueError,
                       match=r'execution\.build_context\.platform\.system'):
        ReplaySession(path)
    # The data are still readable: inspection does not apply the gate.
    with open_recording(path) as recording:
        assert recording.schema_version == 1


def test_unknown_identity_schema_fails_the_recipe_before_stepping(tmp_path):
    path = _record_run(tmp_path / 'run')

    def schema99(record):
        record = dict(record)
        record['identity_schema'] = 99
        return record
    _patch_execution(path, schema99)
    with pytest.raises((RecordingError, ValueError), match='identity_schema'):
        load_replay_input(path)


# --------------------------------------------------------------------------
# The subprocess contract: portable paths carry no verify machinery at all
# --------------------------------------------------------------------------


def test_inspection_and_dry_run_run_without_artifact_or_stepping(tmp_path):
    """In a clean process, all verify-machinery symbols are poisoned; the
    inspection commands and headless playback still complete."""
    path = _record_run(tmp_path / 'run')
    script = textwrap.dedent('''
        import sys
        import mujoco
        import bike_sim.native.artifact as artifact
        import bike_sim.native.research as native_research
        import bike_sim.native.execution_identity as identity
        import bike_sim.sim.backend as backend_mod

        def forbidden(*args, **kwargs):
            raise AssertionError('verify machinery reached in a portable path')

        artifact.load_native_extension = forbidden
        artifact.execution_provenance = forbidden
        native_research.create_native_research = forbidden
        identity.require_execution_compatible = forbidden
        backend_mod.require_backend = forbidden
        mujoco.mj_step = forbidden

        from bike_sim.cli.recording import main as recording_main
        from bike_sim.cli.replay import main as replay_main
        run = sys.argv[1]
        assert recording_main(['overview', run, '--json']) == 0
        assert recording_main(['fields', run]) == 0
        assert recording_main(['sql', run, '--query',
                               'SELECT count(*) AS n FROM physics', '--json']) == 0
        assert replay_main([run, '--view', '--dry-run']) == 0
    ''')
    result = subprocess.run([sys.executable, '-c', script, str(path)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


# --------------------------------------------------------------------------
# Fixture manifests: the recorded provenance documents must stay well-formed
# --------------------------------------------------------------------------


def test_platform_recording_fixture_manifests_are_wellformed():
    fixtures = sorted((Path(__file__).resolve().parent /
                       'fixtures/platform-recordings').glob('*.json'))
    assert fixtures, 'tests/fixtures/platform-recordings must carry manifests'
    for file in fixtures:
        manifest = json.loads(file.read_text())
        assert manifest['fixture'] == 'platform-recordings', file
        assert manifest['recording']['container_schema_version'] == 1, file
        assert manifest['recording']['replay_contract_version'] == 3, file
        host = manifest['generator']['host']
        assert host['system'] in ('Darwin', 'Linux'), file
        files = manifest['recording']['files']
        assert set(files) == {'manifest.json', 'run.duckdb', 'summary.md'}, file
        for name, entry in files.items():
            assert len(entry['sha256']) == 64 and entry['bytes'] > 0, (file, name)
        identity = manifest['execution_identity']
        assert identity['identity_schema'] >= 2, file
        assert identity['build_context']['platform']['system'] == host['system'], file
