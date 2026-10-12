"""Physical driver parity uses full accounting, not a preview force shortcut."""
import numpy as np
import pytest
from native_loader import load_native
from _bits import assert_bitwise_equal
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_driver import PhysicalRideDriver


# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def test_native_physical_driver_finishes_exports_and_retains_owned_snapshot(
        environment_factory, native_module, tmp_path, assert_tree):
    reference = PhysicalRideDriver(environment_factory().sim, backend='python',
                                   duration_s=.005, decimation=2)
    native = PhysicalRideDriver(environment_factory().sim, backend='native',
                                duration_s=.005, decimation=2)
    try:
        command = RideControl(0.)
        a = reference.advance(100, command)
        b = native.advance(100, command)
        assert a == b
        assert b.step == 4 and b.outcome == 'duration_reached'
        reference.flush()
        native.flush()
        assert_tree(native.recorder.columns(), reference.recorder.columns())
        assert_tree(native.recorder.samples, reference.recorder.samples)
        old = native.snapshot()
        expected = old.integration_state.copy()
        assert_bitwise_equal(expected, reference.snapshot().integration_state)
        summary = native.export(tmp_path/'physical')
        assert summary['execution']['backend'] == 'native'
        assert summary['outcome']['steps'] == 4
        for name in ('summary.json', 'telemetry.csv', 'intervals.jsonl', 'terrain_vertices.npy'):
            assert (tmp_path/'physical'/name).is_file()
        native.reset()
        assert native.snapshot().generation == old.generation+1
        assert native.recorder.rows == 0
        np.testing.assert_array_equal(old.integration_state, expected)
    finally:
        reference.close()
        native.close()


def test_native_driver_seals_episodes_that_survive_reset_and_close(
        environment_factory, native_module):
    """driver.seal_episode owns evidence per generation (R03).

    Sealing is idempotent per generation, rejects a conflicting episode id,
    and the owned typed source still decodes after reset and close.
    """
    driver = PhysicalRideDriver(environment_factory().sim, backend='native',
                                duration_s=.02, record=True)
    try:
        driver.advance(16)
        first = driver.seal_episode(0, reason='operator_reset')
        assert first.episode_id == 0 and first.generation == 1
        assert first.physical.row_count > 0
        assert first.summary['outcome']['reason'] == 'operator_reset'
        assert {a.name for a in first.assets} >= {
            'track.toml', 'resolved_config.json'}
        assert driver.seal_episode(0, reason='operator_reset') is first
        driver.reset()
        second = driver.seal_episode(1, reason='viewer_closed')
        assert second.episode_id == 1 and second.generation == 2
        assert driver.seal_episode(1, reason='viewer_closed') is second
        with pytest.raises(ValueError, match='already sealed'):
            driver.seal_episode(4, reason='conflict')
        rows = first.physical.row_count
        driver.close()
        # Owned evidence outlives the runtime it was sealed from.
        assert first.physical.row_count == rows
        tables = {batch.table for batch in first.physical.tables(batch_rows=64)}
        assert 'channel_tires' in tables and 'physics' in tables
    finally:
        driver.close()
