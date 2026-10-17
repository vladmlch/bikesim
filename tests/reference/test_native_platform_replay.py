"""T22 native: checked replay gates on the full versioned execution identity.

``tests/test_recording_platform_compatibility.py`` owns the portable
inspection/playback contract; this file pins the native side of the same
boundary:

* a same-host native run still completes checked physical replay;
* a recording whose recorded identity belongs to the other operating system
  is rejected with the exact field path — before plant construction, so zero
  physics may occur — and the failure explanation is preserved end to end;
* recorded-frame playback of that same foreign run still works and is pinned
  unverified, the strictest reading of the portability split;
* neither ``require_execution_compatible`` nor ``execution_provenance`` is
  invoked by the inspection/playback branch — the verbatim brief.
"""
from copy import deepcopy
from pathlib import Path
import sys

import pytest
from native_loader import load_native
from test_replay_recording_v3 import save_run

_TESTS_DIR = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, _TESTS_DIR)
from test_recording_platform_compatibility import (  # noqa: E402
    _foreign_system, _patch_execution, _recipe_execution)
sys.path.remove(_TESTS_DIR)

from bike_sim.sim.research.replay import replay_episode  # noqa: E402
from bike_sim.sim.research.replay_session import ReplaySession  # noqa: E402

# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def test_native_checked_replay_passes_for_a_same_host_recording(environment_pair, tmp_path):
    """Control case: the gate must admit the identity this host recorded."""
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    report = replay_episode(path)
    assert report['passed'] and report['backend'] == 'native'


def test_native_replay_rejects_a_recording_authored_on_another_host(
        environment_pair, tmp_path, monkeypatch):
    """A transported recording carries its recorded identity; the gate names
    the field before a plant can be constructed — zero steps occur."""
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    _patch_execution(path, _foreign_system)
    def forbidden(*args, **kwargs):
        raise AssertionError('cross-host recording reached plant construction')
    monkeypatch.setattr('bike_sim.sim.ride_sim.RideSimulation.__init__', forbidden)
    with pytest.raises(ValueError,
                       match=r'execution\.build_context\.platform\.system') as failure:
        ReplaySession(path)
    assert 'execution identity mismatch' in str(failure.value)
    # The same file still opens read-only: rejection is a replay verdict,
    # never a reason to delete or hide provenance.
    from bike_sim.recording.reader import open_recording
    with open_recording(path) as recording:
        assert recording.schema_version == 1


def test_native_replay_rejects_a_foreign_runtime_with_the_named_field(
        environment_pair, tmp_path, monkeypatch):
    """The symmetric direction: an unchanged recording on a foreign runtime
    is the same field-level mismatch — nothing is projected away."""
    import bike_sim.native.artifact as artifact
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    recorded = _recipe_execution(path)
    foreign = _foreign_system(recorded)
    monkeypatch.setattr(artifact, 'execution_provenance',
                        lambda backend, *, artifact=None: deepcopy(foreign))
    def forbidden(*args, **kwargs):
        raise AssertionError('foreign runtime reached plant construction')
    monkeypatch.setattr('bike_sim.sim.ride_sim.RideSimulation.__init__', forbidden)
    with pytest.raises(ValueError,
                       match=r'execution\.build_context\.platform\.system'):
        ReplaySession(path)


def test_native_replay_recheck_still_fires_on_drift_during_replay(
        environment_pair, tmp_path, monkeypatch):
    """The later recheck cannot be weakened by the construction-time gate:
    drift mid-replay freezes the session at the named field."""
    import bike_sim.native.artifact as artifact
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    drifted = _foreign_system(_recipe_execution(path))
    session = ReplaySession(path)
    try:
        monkeypatch.setattr(artifact, 'execution_provenance',
                            lambda backend, *, artifact=None: deepcopy(drifted))
        with pytest.raises(ValueError,
                           match=r'execution\.build_context\.platform\.system'):
            while not session.advance(target_step=session.step + 1):
                pass
        steps_at_failure = session._env.sim.steps
        assert not session.done and session.error is not None
        with pytest.raises(RuntimeError, match='replay stopped after mismatch'):
            session.advance()
        assert session._env.sim.steps == steps_at_failure
    finally:
        session.close()


def test_native_recorded_playback_supports_the_foreign_recording(
        environment_pair, tmp_path, capsys):
    """The strictest split: the file checked replay refuses is still fully
    playable — and forever labeled unverified."""
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    _patch_execution(path, _foreign_system)
    from bike_sim.cli.replay import main
    from bike_sim.sim.research.playback import load_playback
    playback = load_playback(path)
    assert playback.verified is False and playback.times_s.size >= 2
    assert not hasattr(playback, 'report')
    assert main([str(path), '--view', '--dry-run']) == 0
    captured = capsys.readouterr()
    assert 'unverified' in captured.err
    assert 'verified' not in captured.out.replace('unverified', '')
    # Checked replay of the same foreign file still refuses.
    with pytest.raises(ValueError,
                       match=r'execution\.build_context\.platform\.system'):
        replay_episode(path)


def test_native_verify_machinery_is_never_called_for_inspection_or_playback(
        environment_pair, tmp_path, capsys, monkeypatch):
    """The verbatim brief: neither compatibility function may be invoked by
    the independent inspection/non-simulating playback branch — while the
    checked path invokes the construction gate and the final recheck."""
    import bike_sim.native.artifact as artifact
    import bike_sim.native.execution_identity as identity
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    compatible_calls, provenance_calls = [], []
    original_compatible = identity.require_execution_compatible
    original_provenance = artifact.execution_provenance

    def spy_compatible(recorded, current):
        compatible_calls.append(recorded)
        return original_compatible(recorded, current)

    def spy_provenance(backend, *, artifact=None):
        provenance_calls.append(backend)
        return original_provenance(backend, artifact=artifact)

    monkeypatch.setattr(identity, 'require_execution_compatible', spy_compatible)
    monkeypatch.setattr(artifact, 'execution_provenance', spy_provenance)

    from bike_sim.cli.recording import main as recording_main
    from bike_sim.cli.replay import main as replay_main
    from bike_sim.sim.research.playback import load_playback
    assert recording_main(['overview', str(path), '--json']) == 0
    assert recording_main(['sql', str(path), '--query',
                           'SELECT count(*) AS n FROM physics', '--json']) == 0
    playback = load_playback(path)
    assert playback.verified is False and playback.times_s.size >= 2
    assert replay_main([str(path), '--view', '--dry-run']) == 0
    capsys.readouterr()
    assert compatible_calls == [] and provenance_calls == []

    # Checked replay invokes the gate at plant construction and again at the
    # final boundary — both times against the recorded identity.
    recorded = _recipe_execution(path)
    session = ReplaySession(path)
    try:
        while not session.advance():
            pass
        assert session.report()['passed']
    finally:
        session.close()
    assert compatible_calls == [recorded, recorded]
    # The recorded backend's provenance is computed for the gate; the rebuilt
    # python-side environment may add its own — never zero.
    assert len(provenance_calls) >= 2 and set(provenance_calls) <= {'python', 'native'}
