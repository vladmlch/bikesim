import json
import numpy as np
import pytest
from bike_sim.native.artifact import EXECUTION_FIELDS
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.replay import (
    SCHEMA2_FILES, ENVELOPE_FILE, STRENGTH_FILE, validate_recording, rebuild_environment,
)


def test_shared_export_native_rows_and_complete_provenance(environment_pair, tmp_path, assert_tree):
    reference, native = environment_pair(decimation=3)
    for _ in range(2):
        assert_tree(native.step(RideControl(0.)), reference.step(RideControl(0.)))
    reference.stop()
    native.stop()
    python_path, native_path = tmp_path/'python', tmp_path/'native'
    reference.save(python_path)
    native.save(native_path)
    summary, manifest = validate_recording(native_path)
    python_summary, python_manifest = validate_recording(python_path)
    assert set(manifest['execution']) == EXECUTION_FIELDS
    assert manifest['execution']['backend'] == 'native'
    assert python_manifest['execution']['backend'] == 'python'
    assert set(manifest['file_sha256']) == set(SCHEMA2_FILES) | {ENVELOPE_FILE, STRENGTH_FILE}
    assert not summary['research']['execution_changed_during_run']
    assert not python_summary['research']['execution_changed_during_run']
    # This is the sole intentional summary difference for a paired owner.
    from copy import deepcopy
    actual_summary, expected_summary = deepcopy(summary), deepcopy(python_summary)
    del actual_summary['research']['execution']
    del expected_summary['research']['execution']
    assert_tree(actual_summary, expected_summary)
    for name in ('observations.jsonl', 'transitions.jsonl', 'commands_requested.jsonl', 'commands_applied.jsonl',
                 'intervals.jsonl'):
        actual = [json.loads(line) for line in (native_path/name).read_text().splitlines()]
        expected = [json.loads(line) for line in (python_path/name).read_text().splitlines()]
        assert_tree(actual, expected)
    assert_tree(json.loads((native_path/'episode_metrics.json').read_text()),
                json.loads((python_path/'episode_metrics.json').read_text()))
    with np.load(native_path/'states.npz', allow_pickle=False) as actual:
        with np.load(python_path/'states.npz', allow_pickle=False) as expected:
            for name in ('initial', 'final'):
                np.testing.assert_allclose(actual[name], expected[name], atol=1e-9, rtol=1e-9)
    with pytest.raises(ValueError, match='Track C3'):
        rebuild_environment(native_path)


def test_empty_recording_and_no_overwrite(environment_pair, tmp_path):
    _, native = environment_pair()
    destination = tmp_path/'empty'
    native.save(destination)
    validate_recording(destination)
    assert (destination/'intervals.jsonl').read_text() == ''
    before = (destination/'replay.json').read_bytes()
    with pytest.raises(FileExistsError):
        native.save(destination)
    assert (destination/'replay.json').read_bytes() == before


def test_failed_staging_does_not_destroy_existing_recording(environment_pair, tmp_path, monkeypatch):
    import bike_sim.native.recording as recording
    _, native = environment_pair()
    destination = tmp_path/'preserved'
    native.save(destination)
    before = {path.name: path.read_bytes() for path in destination.iterdir()}
    def broken(*args, **kwargs):
        raise OSError('injected export failure')
    monkeypatch.setattr(recording, '_write_episode', broken)
    with pytest.raises(OSError, match='injected export failure'):
        native.save(destination, overwrite=True)
    assert {path.name: path.read_bytes() for path in destination.iterdir()} == before
    validate_recording(destination)


def test_save_refuses_pending_control_window(environment_pair, tmp_path):
    _, native = environment_pair()
    native.begin_control(RideControl(0.))
    with pytest.raises(RuntimeError, match='boundary'):
        native.save(tmp_path/'pending')
    assert not (tmp_path/'pending').exists()
    native.advance_control()


@pytest.mark.parametrize('changed_field', ['native_source_sha256', 'extension_sha256', 'mujoco_library_sha256'])
def test_changed_execution_is_not_relabelled_as_startup_identity(environment_pair, tmp_path, monkeypatch,
                                                               changed_field):
    from copy import deepcopy
    import bike_sim.native.artifact as artifact
    _, native = environment_pair()
    startup = deepcopy(native.execution)
    changed = deepcopy(startup)
    changed[changed_field] = '0'*64 if startup[changed_field] != '0'*64 else '1'*64
    monkeypatch.setattr(artifact, 'execution_provenance', lambda backend: deepcopy(changed))
    destination = tmp_path/'changed'
    native.save(destination)
    summary, manifest = validate_recording(destination)
    assert summary['research']['execution_changed_during_run']
    assert summary['research']['execution'] == manifest['execution'] == startup
