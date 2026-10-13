"""Native capture ownership and suffix regression sources; no implicit build."""
from dataclasses import asdict

import numpy as np
import pytest

from bike_sim.recording.json_codec import thaw
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.replay import integration_state
from native_loader import load_native
from test_research_capture import _noise

# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def test_native_off_full_are_passive_and_full_records_every_interval(environment_pair, monkeypatch, assert_tree):
    _noise(monkeypatch)
    _, off = environment_pair(capture_mode='off', decimation=3)
    reference, full = environment_pair(capture_mode='full', decimation=3)
    for torque in (0., 2., 1.):
        expected = reference.step(RideControl(torque))
        assert_tree(off.step(RideControl(torque)), expected)
        assert_tree(full.step(RideControl(torque)), expected)
        np.testing.assert_array_equal(integration_state(off.sim), integration_state(full.sim))
    assert not off.commands_requested and not off.commands_applied and not off.trace
    assert not off.observations and off.recorder.rows == 0
    assert full.recorder.rows == full.sim.steps
    evidence = full.seal_episode(0, reason='reset')
    assert evidence.research.complete_control_steps == 3
    assert evidence.physical.row_count == full.sim.steps
    owner = evidence.physical.episode
    assert owner.sample_batch(0, 2).columns == {}
    held = owner.sample_batch(0, 1).samples[0].qpos.copy()
    full.reset()
    full.close()
    np.testing.assert_array_equal(owner.sample_batch(0, 1).samples[0].qpos, held)
    assert sum(batch.row_count for batch in evidence.research.history.tables(batch_rows=2)
               if batch.table == 'observations') == 4


def test_native_verify_boxing_failure_does_not_acknowledge_suffix(environment_pair, monkeypatch):
    import bike_sim.recording.contracts as contracts
    _, env = environment_pair(capture_mode='verify')
    original = contracts.freeze
    def fail(value):
        raise MemoryError('box/freeze injection')
    monkeypatch.setattr(contracts, 'freeze', fail)
    with pytest.raises(MemoryError):
        env.applied_since(0)
    monkeypatch.setattr(contracts, 'freeze', original)
    rows, cursor = env.applied_since(0)
    assert len(rows) == cursor == 1
    assert not env.commands_applied
    assert env.applied_since(cursor) == ((), cursor)
    with pytest.raises(ValueError):
        env.applied_since(0)
    env.step(RideControl(2.))
    suffix, end = env.applied_since(cursor)
    assert end == cursor + len(suffix)
    assert not env.commands_requested and not env.observations and not env.trace
    assert env.last_requested['control']['motor_torque_nm'] == 2.
