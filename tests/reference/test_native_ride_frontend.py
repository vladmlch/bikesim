"""Physical driver parity uses full accounting, not a preview force shortcut."""
import numpy as np
from native_loader import load_native
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
        np.testing.assert_allclose(expected, reference.snapshot().integration_state, atol=1e-9, rtol=1e-9)
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
